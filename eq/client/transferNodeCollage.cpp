#include "transferNodeCollage.h"
#include "frame.h"
#include "frameData.h"
#include "image.h"
#include "pixelData.h"
#include "log.h"

#include <eq/fabric/commands.h>

#include <co/connectionDescription.h>
#include <co/objectOCommand.h>
#include <co/objectICommand.h>
#include <co/sendToken.h>

namespace eq
{

TransferNodeCollage::TransferNodeCollage()
    : TransferNode()
{
    _type = COLLAGE;
}

bool TransferNodeCollage::setupTransmitter()
{
    co::ConnectionPtr connection = _targetNode->getConnection();
    return ( connection && connection->isConnected() );
}

bool TransferNodeCollage::setupReceiver( co::ObjectICommand& /*cmd*/ )
{
    return true;
}


void TransferNodeCollage::sendImage( Image* image, 
                                    uint32_t frameNumber, 
                                    uint32_t taskID, 
                                    const uint128_t& frameID )
{
    co::ConnectionPtr connection = _targetNode->getConnection();
    if ( !connection || !connection->isConnected() )
        return;

    PixelDatas pixelDatas;
    Qualities qualities;

    uint64_t imageDataSize = _getPixelDatas( pixelDatas, qualities, 
                                             frameNumber, taskID, image );

    if( pixelDatas.empty( ))
        return;

    co::LocalNode::SendToken token = _aquireSendToken( frameNumber, taskID );

    // send image pixel data command
    LBASSERT( image->getPixelViewport().isValid( ));

    _send( image, imageDataSize, frameNumber, frameID, pixelDatas, qualities );
}

uint64_t TransferNodeCollage::_getPixelDatas( PixelDatas& pixelDatas,
                                              Qualities& qualities,
                                              uint32_t frameNumber, 
                                              uint32_t taskID, 
                                              Image* image )
{
    uint64_t imageDataSize = 0;

    // use compression on links up to 2 GBit/s
    co::ConnectionPtr conn = _targetNode->getConnection();
    co::ConstConnectionDescriptionPtr description = conn->getDescription();
    const bool useCompression = ( description->bandwidth <= 262144 );

    uint64_t rawSize( 0 );
    ProxyStatistics compressEvent( Statistic::CHANNEL_FRAME_COMPRESS, 
        _stats, frameNumber/*, useCompression ? AUTO : OFF*/ );
    compressEvent.event.data.statistic.task = taskID;
    compressEvent.event.data.statistic.ratio = 1.0f;
    compressEvent.event.data.statistic.plugins[0] = EQ_COMPRESSOR_NONE;
    compressEvent.event.data.statistic.plugins[1] = EQ_COMPRESSOR_NONE;

    // Prepare image pixel data
    Frame::Buffer buffers[] = {Frame::BUFFER_COLOR,Frame::BUFFER_DEPTH};

    // for each image attachment
    for( unsigned j = 0; j < 2; ++j )
    {
        Frame::Buffer buffer = buffers[j];
        if( image->hasPixelData( buffer ))
        {
            // format, type, nChunks, compressor name
            imageDataSize += sizeof( FrameData::ImageHeader );

            const PixelData& data = useCompression ?
                image->compressPixelData( buffer ) :
                image->getPixelData( buffer );
            pixelDatas.push_back( &data );
            qualities.push_back( image->getQuality( buffer ));

            if( data.isCompressed )
            {
                const uint32_t nElements =
                    uint32_t( data.compressedSize.size( ));
                for( uint32_t k = 0 ; k < nElements; ++k )
                {
                    imageDataSize += sizeof( uint64_t );
                    imageDataSize += data.compressedSize[ k ];
                }
                compressEvent.event.data.statistic.plugins[j] =
                    data.compressorName;
            }
            else
            {
                imageDataSize += sizeof( uint64_t );
                imageDataSize += image->getPixelDataSize( buffer );
            }

            rawSize += image->getPixelDataSize( buffer );
        }
    }

    if( rawSize > 0 )
        compressEvent.event.data.statistic.ratio =
        static_cast< float >( imageDataSize ) /
        static_cast< float >( rawSize );

    return imageDataSize;
}

co::LocalNode::SendToken TransferNodeCollage::_aquireSendToken( 
    uint32_t frameNumber, uint32_t taskID )
{
    if( _requestToken )
    {
        StatisticSampler<StatisticsProxy> waitEvent( 
            Statistic::CHANNEL_FRAME_WAIT_SENDTOKEN, _stats, frameNumber );
        waitEvent.event.data.statistic.task = taskID;
        return getLocalNode()->acquireSendToken( _targetNode );
    }
    return co::LocalNode::SendToken();
}

void TransferNodeCollage::_send( Image* image, 
                                 uint64_t imageDataSize, 
                                 uint32_t frameNumber, 
                                 const uint128_t& frameID, 
                                 const PixelDatas& pixelDatas,
                                 const Qualities& qualities )
{
    co::ConnectionPtr connection = _targetNode->getConnection();
    uint32_t commandBuffers = image->hasPixelData( Frame::BUFFER_COLOR ) ? 
        Frame::BUFFER_COLOR : Frame::BUFFER_NONE;
    if ( image->hasPixelData( Frame::BUFFER_DEPTH ))
        commandBuffers |= Frame::BUFFER_DEPTH;

    co::ObjectOCommand command( initiateSend( frameNumber ));
    
    command << image->getPixelViewport() << image->getZoom() << commandBuffers 
        << image->getAlphaUsage() << frameID;    
    
    command.sendHeader( imageDataSize );
    _sendPixelData( pixelDatas, qualities );
}

void TransferNodeCollage::_sendPixelData( const PixelDatas& pixelDatas,
                                          const Qualities& qualities )
{
    co::ConnectionPtr connection = _targetNode->getConnection();
#ifndef NDEBUG
    size_t sentBytes = 0;
#endif

    for( uint32_t j=0; j < pixelDatas.size(); ++j )
    {
#ifndef NDEBUG
        sentBytes += sizeof( FrameData::ImageHeader );
#endif
        const PixelData* data = pixelDatas[j];
        const FrameData::ImageHeader header =
        { data->internalFormat, data->externalFormat,
        data->pixelSize, data->pvp,
        data->isCompressed ? data->compressorName : EQ_COMPRESSOR_NONE,
        data->compressorFlags,
        data->isCompressed ? uint32_t( data->compressedSize.size()) : 1,
        qualities[ j ] };

        connection->send( &header, sizeof( header ), true );

        if( data->isCompressed )
        {
            for( uint32_t k = 0 ; k < data->compressedSize.size(); ++k )
            {
                const uint64_t dataSize = data->compressedSize[k];
                connection->send( &dataSize, sizeof( dataSize ), true );
                if( dataSize > 0 )
                    connection->send( data->compressedData[k],
                    dataSize, true );
#ifndef NDEBUG
                sentBytes += sizeof( dataSize ) + dataSize;
#endif
            }
        }
        else
        {
            const uint64_t dataSize = data->pvp.getArea() *
                data->pixelSize;
            connection->send( &dataSize, sizeof( dataSize ), true );
            connection->send( data->pixels, dataSize, true );
#ifndef NDEBUG
            sentBytes += sizeof( dataSize ) + dataSize;
#endif
        }
    }
}

bool TransferNodeCollage::receiveImage( Image* image, co::ObjectICommand& cmd )
{
    const PixelViewport pvp = cmd.get< PixelViewport >();
    const Zoom zoom = cmd.get< Zoom >();
    const uint32_t buffers_ = cmd.get< uint32_t >();
    const bool useAlpha = cmd.get< bool >();
    /*const uint128_t frameID = */ cmd.get< uint128_t >();
    // Note on the const_cast: since the PixelData structure stores non-const
    // pointers, we have to go non-const at some point, even though we do not
    // modify the data.
    uint8_t* data = const_cast< uint8_t* >(reinterpret_cast< const uint8_t* >(
        cmd.getRemainingBuffer( cmd.getRemainingBufferSize( ))));

    LBLOG( LOG_ASSEMBLY )
        << "received image data for " << co::ObjectVersion( this ) 
        << ", buffers " << buffers_ << " pvp " << pvp << std::endl;

    LBASSERT( pvp.isValid( ));

    image->setPixelViewport( pvp );
    image->setAlphaUsage( useAlpha );
    image->setZoom( zoom );

    Frame::Buffer buffers[] = { Frame::BUFFER_COLOR, Frame::BUFFER_DEPTH };
    for( unsigned i = 0; i < 2; ++i )
    {
        const Frame::Buffer buffer = buffers[i];

        if( buffers_ & buffer )
        {
            PixelData pixelData;
            const FrameData::ImageHeader* header = 
                        reinterpret_cast<FrameData::ImageHeader*>( data );
            pixelData.internalFormat  = header->internalFormat;
            pixelData.externalFormat  = header->externalFormat;
            pixelData.pixelSize       = header->pixelSize;
            pixelData.pvp             = header->pvp;
            pixelData.compressorName  = header->compressorName;
            pixelData.compressorFlags = header->compressorFlags;
            pixelData.isCompressed =
                pixelData.compressorName > EQ_COMPRESSOR_NONE;

            const uint32_t nChunks    = header->nChunks;
            data += sizeof( FrameData::ImageHeader );

            if( pixelData.isCompressed )
            {
                pixelData.compressedSize.resize( nChunks );
                pixelData.compressedData.resize( nChunks );

                for( uint32_t j = 0; j < nChunks; ++j )
                {
                    const uint64_t size = *reinterpret_cast< uint64_t*>( data );
                    data += sizeof( uint64_t );

                    pixelData.compressedSize[j] = size;
                    pixelData.compressedData[j] = data;
                    data += size;
                }
            }
            else
            {
                const uint64_t size = *reinterpret_cast< uint64_t*>( data );
                data += sizeof( uint64_t );
                pixelData.pixels = data;
                data += size;
                LBASSERT( size == pixelData.pvp.getArea()*pixelData.pixelSize );
            }

            image->setQuality( buffer, header->quality );
            image->setPixelData( buffer, pixelData );
        }
    }

    return true;
}

}
