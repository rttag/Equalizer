
/* Copyright (c) 2006-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *                    2011, Daniel Nachbaur <danielnachbaur@gmail.com>
 *                    2010, Cedric Stalder <cedric.stalder@gmail.com>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 2.1 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "frameData.h"

#include "nodeStatistics.h"
#include "channelStatistics.h"
#include "exception.h"
#include "image.h"
#include "log.h"
#include "pixelData.h"
#include "roiFinder.h"

#include <eq/fabric/drawableConfig.h>
#include <eq/fabric/commands.h>
#include <eq/util/objectManager.h>
#include <co/commandFunc.h>
#include <co/connectionDescription.h>
#include <co/dataIStream.h>
#include <co/dataOStream.h>
#include <co/objectOCommand.h>
#include <co/objectICommand.h>
#include <lunchbox/monitor.h>
#include <lunchbox/scopedMutex.h>
#include <lunchbox/plugins/compressor.h>

#include <algorithm>

namespace eq
{

typedef co::CommandFunc<FrameData> CmdFunc;

FrameData::FrameData()
        : _version( co::VERSION_NONE.low( ))
        , _useAlpha( true )
        , _colorQuality( 1.f )
        , _depthQuality( 1.f )
        , _colorCompressor( EQ_COMPRESSOR_AUTO )
        , _depthCompressor( EQ_COMPRESSOR_AUTO )
        , _asyncUploadMap()
{
    _roiFinder = new ROIFinder();
}

void FrameData::attach( const UUID& id, const uint32_t instanceID )
{
    co::Object::attach( id, instanceID );

    co::CommandQueue* commandQ = getLocalNode()->getCommandThreadQueue();

    registerCommand( fabric::CMD_FRAMEDATA_TRANSMIT,
        CmdFunc( this, &FrameData::_cmdFrameDataTransmit ), commandQ );
    registerCommand( fabric::CMD_FRAMEDATA_READY,
        CmdFunc( this, &FrameData::_cmdFrameDataReady ), commandQ );
}

FrameData::~FrameData()
{
    clear();

    for( Images::const_iterator i = _imageCache.begin();
         i != _imageCache.end(); ++i )
    {
       Image* image = *i;
        LBWARN << "Unflushed image in FrameData destructor" << std::endl;
        delete image;
    }
    _imageCache.clear();

    delete _roiFinder;
    _roiFinder = 0;
}

void FrameData::setQuality( Frame::Buffer buffer, float quality )
{
    if( buffer != Frame::BUFFER_COLOR )
    {
        LBASSERT( buffer == Frame::BUFFER_DEPTH );
        _depthQuality = quality;
        return;
    }

    _colorQuality = quality;
}

void FrameData::useCompressor( const Frame::Buffer buffer, const uint32_t name )
{
    if( buffer != Frame::BUFFER_COLOR )
    {
        LBASSERT( buffer == Frame::BUFFER_DEPTH );
        _depthCompressor = name;
        return;
    }

    _colorCompressor = name;
}

void FrameData::getInstanceData( co::DataOStream& os )
{
    LBUNREACHABLE;
    _data.serialize( os );
}

void FrameData::applyInstanceData( co::DataIStream& is )
{
    clear();
    _data.deserialize( is );
    LBLOG( LOG_ASSEMBLY ) << "applied " << this << std::endl;
}

FrameData::Data& FrameData::Data::operator = ( const Data& rhs )
{
    if( this != &rhs )
    {
        pvp = rhs.pvp;
        frameType = rhs.frameType;
        buffers = rhs.buffers;
        period = rhs.period;
        phase = rhs.phase;
        range = rhs.range;
        pixel = rhs.pixel;
        subpixel = rhs.subpixel;
        zoom = rhs.zoom;
        // don't assign input nodes & -netNodes here
    }
    return *this;
}

void FrameData::Data::serialize( co::DataOStream& os ) const
{
    os << pvp << frameType << buffers << period << phase << range
       << pixel << subpixel << zoom;
}

void FrameData::Data::deserialize( co::DataIStream& is )
{
    is >> pvp >> frameType >> buffers >> period >> phase >> range
       >> pixel >> subpixel >> zoom;
}

void FrameData::clear()
{
    _imageCacheLock.set();
    _imageCache.insert( _imageCache.end(), _images.begin(), _images.end( ));
    _imageCacheLock.unset();
    _images.clear();
}

void FrameData::flush()
{
    clear();

    for( ImagesCIter i = _imageCache.begin(); i != _imageCache.end(); ++i )
    {
        Image* image = *i;
        image->flush();
        delete image;
    }

    _imageCache.clear();
}

void FrameData::deleteGLObjects( ObjectManager* om )
{
    for( ImagesCIter i = _images.begin(); i != _images.end(); ++i )
        (*i)->deleteGLObjects( om );
    for( ImagesCIter i = _imageCache.begin(); i != _imageCache.end(); ++i )
        (*i)->deleteGLObjects( om );
}

void FrameData::resetPlugins()
{
    for( ImagesCIter i = _images.begin(); i != _images.end(); ++i )
        (*i)->resetPlugins();
    for( ImagesCIter i = _imageCache.begin(); i != _imageCache.end(); ++i )
        (*i)->resetPlugins();
}

Image* FrameData::newImage( const eq::Frame::Type type,
                            const DrawableConfig& config )
{
    Image* image = _allocImage( type, config, true /* set quality */ );
    _images.push_back( image );
    return image;
}

Image* FrameData::_allocImage( const eq::Frame::Type type,
                               const DrawableConfig& config,
                               const bool setQuality_ )
{
    Image* image;
    _imageCacheLock.set();

    if( _imageCache.empty( ))
    {
        _imageCacheLock.unset();
        image = new Image;
    }
    else
    {
        image = _imageCache.back();
        _imageCache.pop_back();
        _imageCacheLock.unset();

        image->reset();
    }

    image->setAlphaUsage( _useAlpha );
    image->setStorageType( type );
    if( setQuality_ )
    {
        image->setQuality( Frame::BUFFER_COLOR, _colorQuality );
        image->setQuality( Frame::BUFFER_DEPTH, _depthQuality );
    }

    image->useCompressor( Frame::BUFFER_COLOR, _colorCompressor );
    image->useCompressor( Frame::BUFFER_DEPTH, _depthCompressor );

    image->setInternalFormat( Frame::BUFFER_DEPTH,
                              EQ_COMPRESSOR_DATATYPE_DEPTH );
    switch( config.colorBits )
    {
        case 16:
            image->setInternalFormat( Frame::BUFFER_COLOR,
                                      EQ_COMPRESSOR_DATATYPE_RGBA16F );
            break;
        case 32:
            image->setInternalFormat( Frame::BUFFER_COLOR,
                                      EQ_COMPRESSOR_DATATYPE_RGBA32F );
            break;
        case 10:
            image->setInternalFormat( Frame::BUFFER_COLOR,
                                      EQ_COMPRESSOR_DATATYPE_RGB10_A2 );
            break;
        default:
            image->setInternalFormat( Frame::BUFFER_COLOR,
                                      EQ_COMPRESSOR_DATATYPE_RGBA );
    }

    return image;
}

#ifndef EQ_2_0_API
void FrameData::readback( const Frame& frame,
                          ObjectManager* glObjects,
                          const DrawableConfig& config )
{
    const Images& images = startReadback( frame, glObjects, config,
                                        PixelViewports( 1, getPixelViewport( )));

    for( ImagesCIter i = images.begin(); i != images.end(); ++i )
        (*i)->finishReadback( frame.getZoom(), glObjects->glewGetContext( ));
}
#endif

Images FrameData::startReadback( const Frame& frame,
                                 ObjectManager* glObjects,
                                 const DrawableConfig& config,
                                 const PixelViewports& regions )
{
    if( _data.buffers == Frame::BUFFER_NONE )
        return Images();

    const Zoom& zoom = frame.getZoom();
    if( !zoom.isValid( ))
    {
        LBWARN << "Invalid zoom factor, skipping frame" << std::endl;
        return Images();
    }

    const eq::PixelViewport& framePVP = getPixelViewport();
    const PixelViewport      absPVP   = framePVP + frame.getOffset();
    if( !absPVP.isValid( ))
        return Images();

    Images images;

    // readback the whole screen when using textures
    if( getType() == eq::Frame::TYPE_TEXTURE )
    {
        Image* image = newImage( getType(), config );
        if( image->startReadback( getBuffers(), absPVP, zoom, glObjects ))
            images.push_back( image );
        image->setOffset( 0, 0 );
        return images;
    }
    //else read only required regions

// TODO: issue #85: move automatic ROI detection to eq::Channel
#if 0
    PixelViewports regions;
    if( _data.buffers & Frame::BUFFER_DEPTH && zoom == Zoom::NONE )
        regions = _roiFinder->findRegions( _data.buffers, absPVP, zoom,
//                    frame.getAssemblyStage(), frame.getFrameID(), glObjects );
                                        0, 0, glObjects );
    else
        regions.push_back( absPVP );
#endif

    LBASSERT( getType() == eq::Frame::TYPE_MEMORY );
    const eq::Pixel& pixel = getPixel();

    for( uint32_t i = 0; i < regions.size(); ++i )
    {
        PixelViewport pvp = regions[ i ] + frame.getOffset();
        pvp.intersect( absPVP );
        if( !pvp.hasArea( ))
            continue;

        Image* image = newImage( getType(), config );
        if( image->startReadback( getBuffers(), pvp, zoom, glObjects ))
            images.push_back( image );

        pvp -= frame.getOffset();
        image->setOffset( (pvp.x - framePVP.x) * pixel.w,
                          (pvp.y - framePVP.y) * pixel.h );
    }
    return images;
}

void FrameData::setVersion( const uint64_t version )
{
    LBASSERTINFO( _version <= version, _version << " > " << version );
    _version = version;
    LBLOG( LOG_ASSEMBLY ) << "New v" << version << std::endl;
}

void FrameData::waitReady( const uint32_t timeout ) const
{
    if( !_readyVersion.timedWaitGE( getVersion().low() , timeout ))
        throw Exception( Exception::TIMEOUT_INPUTFRAME );
}

void FrameData::setReady()
{
    _setReady( _version );
}

void FrameData::setReady( const co::ObjectVersion& frameData,
                          const FrameData::Data& data )
{
    clear();
    LBASSERT(  frameData.version.high() == 0 );
    LBASSERT( _readyVersion < frameData.version.low( ));
    LBASSERT( _readyVersion == 0 ||
              _readyVersion + 1 == frameData.version.low( ));
    LBASSERT( _version == frameData.version.low( ));

    _images.swap( _pendingImages );
    _data = data;
    _setReady( frameData.version.low());

    LBLOG( LOG_ASSEMBLY ) << this << " applied v"
                          << frameData.version.low() << std::endl;
}

void FrameData::_setReady( const uint64_t version )
{

    LBASSERTINFO( _readyVersion <= version,
                  "v" << _version << " ready " << _readyVersion << " new "
                      << version );

    lunchbox::ScopedMutex< lunchbox::SpinLock > mutex( _listeners );
    if( _readyVersion >= version )
        return;

    _readyVersion = version;
    LBLOG( LOG_ASSEMBLY ) << "set ready " << this << ", " << _listeners->size()
                          << " monitoring" << std::endl;

    for( Listeners::iterator i= _listeners->begin();
         i != _listeners->end(); ++i )
    {
        Listener* listener = *i;
        ++(*listener);
    }
}

void FrameData::addListener( lunchbox::Monitor<uint32_t>& listener )
{
    lunchbox::ScopedMutex< lunchbox::SpinLock > mutex( _listeners );

    _listeners->push_back( &listener );
    if( _readyVersion >= _version )
        ++listener;
}

void FrameData::removeListener( lunchbox::Monitor<uint32_t>& listener )
{
    lunchbox::ScopedMutex< lunchbox::SpinLock > mutex( _listeners );

    Listeners::iterator i = std::find( _listeners->begin(), _listeners->end(),
                                      &listener );
    LBASSERT( i != _listeners->end( ));
    _listeners->erase( i );
}

bool FrameData::addImage( const co::ObjectVersion& frameDataVersion,
                          const PixelViewport& pvp, const Zoom& zoom,
                          const uint32_t buffers_, const bool useAlpha,
                          uint8_t* data )
{
    Image* image = _allocImage( Frame::TYPE_MEMORY, DrawableConfig(),
                                false /* set quality */ );

    image->setPixelViewport( pvp );
    image->setAlphaUsage( useAlpha );

    Frame::Buffer buffers[] = { Frame::BUFFER_COLOR, Frame::BUFFER_DEPTH };
    for( unsigned i = 0; i < 2; ++i )
    {
        const Frame::Buffer buffer = buffers[i];

        if( buffers_ & buffer )
        {
            PixelData pixelData;
            const ImageHeader* header = reinterpret_cast<ImageHeader*>( data );
            pixelData.internalFormat  = header->internalFormat;
            pixelData.externalFormat  = header->externalFormat;
            pixelData.pixelSize       = header->pixelSize;
            pixelData.pvp             = header->pvp;
            pixelData.compressorName  = header->compressorName;
            pixelData.compressorFlags = header->compressorFlags;
            pixelData.isCompressed =
                pixelData.compressorName > EQ_COMPRESSOR_NONE;

            const uint32_t nChunks    = header->nChunks;
            data += sizeof( ImageHeader );

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

            image->setZoom( zoom );
            image->setQuality( buffer, header->quality );
            image->setPixelData( buffer, pixelData );
        }
    }

    LBASSERT( _readyVersion < frameDataVersion.version.low( ));
    _pendingImages.push_back( image );
    return true;
}

void FrameData::uploadImages( const Vector2i& offset , ObjectManager* glObjects )
{
    Images::iterator it = _pendingImages.begin();
    for ( ; it != _pendingImages.end(); ++it )
    {
        LBERROR << "upload of one image" << std::endl;
        Image* image = *it;
        image->setStorageType( Frame::TYPE_TEXTURE );
        image->upload( Frame::BUFFER_COLOR, 0, offset, glObjects );
    }
}

void FrameData::triggerAsyncUpload( const uint128_t& frameID, const UUID& id, 
                                    const Vector2i& offset )
{
    _asyncUploadMap[frameID] = std::make_pair(id, offset);
    triggerUpload( frameID );
}

void FrameData::triggerReady( const uint128_t& frameID,
                              const co::ObjectVersion& framedataVersion,
                              const FrameData::Data& data )
{
    _readyMap[frameID] = std::make_pair( framedataVersion, data );
    triggerUpload( frameID );
}

bool FrameData::triggerUpload( const uint128_t& frameID )
{
    if ( _readyMap.find( frameID ) == _readyMap.end() )
        return false;

    if ( _asyncUploadMap.find( frameID ) == _asyncUploadMap.end() )
        return false;

    LBERROR << "upload " << frameID.low() << " " << this << std::endl;
    const UUID& chanID = _asyncUploadMap[ frameID ].first;
    const Vector2i& offset = _asyncUploadMap[ frameID ].second;
    const co::ObjectVersion& ov = _readyMap[ frameID ].first;
    const FrameData::Data& data = _readyMap[ frameID ].second;
    
    co::ObjectOCommand( getLocalNode().get(), getLocalNode(), 
        fabric::CMD_CHANNEL_FRAME_UPLOAD_IMAGES, co::COMMANDTYPE_OBJECT, 
        chanID, EQ_INSTANCE_ALL ) << ov << data << offset;

    _readyMap.erase( frameID );
    _asyncUploadMap.erase( frameID );
    return true;
}


bool FrameData::_cmdFrameDataTransmit( co::ICommand& cmd )
{
    co::ObjectICommand command( cmd );

    const PixelViewport pvp = command.get< PixelViewport >();
    const Zoom zoom = command.get< Zoom >();
    const uint32_t buffers = command.get< uint32_t >();
    const uint32_t frameNumber = command.get< uint32_t >();
    const bool useAlpha = command.get< bool >();
    const uint128_t frameID = command.get< uint128_t >();
    const uint8_t* data = reinterpret_cast< const uint8_t* >(
        command.getRemainingBuffer( command.getRemainingBufferSize( )));

    LBERROR << "frame " << frameID.low() << " " << this << std::endl;

    LBLOG( LOG_ASSEMBLY )
        << "received image data for " << co::ObjectVersion( this ) 
        << ", buffers " << buffers << " pvp " << pvp << std::endl;

    LBASSERT( pvp.isValid( ));
    LBASSERT( isReady() );

    //--------------TODO DECOMPRESS STATS
    //
    //
    //NodeStatistics event( Statistic::NODE_FRAME_DECOMPRESS, this,
    //    frameNumber );
    //
    //
    //
    //--------------TODO DECOMPRESS STATS

    // Note on the const_cast: since the PixelData structure stores non-const
    // pointers, we have to go non-const at some point, even though we do not
    // modify the data.
    LBCHECK( addImage( co::ObjectVersion( this ), pvp, zoom, buffers,
        useAlpha, const_cast< uint8_t* >( data )));
    return true;
}

bool FrameData::_cmdFrameDataReady( co::ICommand& cmd )
{
    co::ObjectICommand command( cmd );

    const co::ObjectVersion frameDataVersion =
        command.get< co::ObjectVersion >();
    const FrameData::Data data = command.get< FrameData::Data >();
    const uint128_t& frameID = command.get< uint128_t >();

    LBLOG( LOG_ASSEMBLY ) << "received ready for " << co::ObjectVersion( this )
        << std::endl;

    LBASSERT( !isReady() );

    LBERROR << "ready " << frameID.low() << " " << this << std::endl;

    triggerReady( frameID, frameDataVersion, data );
    return true;

    setReady( frameDataVersion, data );
    LBASSERT( isReady() );
    return true;
}


std::ostream& operator << ( std::ostream& os, const FrameData& data )
{
    return os << "frame data id " << data.getID() << "." << data.getInstanceID()
              << " v" << data.getVersion() << ' ' << data.getImages().size()
              << " images, ready " << ( data.isReady() ? 'y' :'n' ) << " "
              << data.getZoom();
}

}
