#include "transferNodeSharedMemory.h"
#include "frame.h"
#include "image.h"
#include "pixelData.h"

#include <eq/fabric/commands.h>

#include <co/objectOCommand.h>
#include <co/objectICommand.h>

namespace eq
{

const char* METADATA_OBJECT = "imageMeta";
const char* COLORBUFFER_ARRAY = "colorBuffer";
const char* DEPTHBUFFER_ARRAY = "depthBuffer";

TransferNodeSharedMemory::TransferNodeSharedMemory()
    : TransferNode()
    , _memory( 0 )
    , _imageMeta( 0 )
{
    _type = SHAREDMEM;
    _pixels[0] = 0;
    _pixels[1] = 0;
}

TransferNodeSharedMemory::~TransferNodeSharedMemory()
{
    clear();
}

typedef co::CommandFunc<TransferNodeSharedMemory> CmdFunc;

void TransferNodeSharedMemory::attach( const UUID& id, const uint32_t instanceID )
{
    TransferNode::attach( id, instanceID );

    co::CommandQueue* commandQ = getLocalNode()->getCommandThreadQueue();

    registerCommand( fabric::CMD_TRANSFERNODE_SHAREDMEM_CLEAR,
        CmdFunc( this, &TransferNodeSharedMemory::_cmdClear ), commandQ );
    registerCommand( fabric::CMD_TRANSFERNODE_SHAREDMEM_CLEAR_REPLY,
        CmdFunc( this, &TransferNodeSharedMemory::_cmdClearReply ), 0 );
    registerCommand( fabric::CMD_TRANSFERNODE_SHAREDMEM_MAPMEM,
        CmdFunc( this, &TransferNodeSharedMemory::_cmdMapMemory ), commandQ );
    registerCommand( fabric::CMD_TRANSFERNODE_SHAREDMEM_MAPMEM_REPLY,
        CmdFunc( this, &TransferNodeSharedMemory::_cmdMapMemoryReply ), 0 );
}

void TransferNodeSharedMemory::clear()
{
    if ( _imageMeta )
    {
        _memory->destroy_ptr( _imageMeta );
        _imageMeta = 0;
    }

    for( unsigned j = 0; j < 2; ++j )
    {
        if ( _pixels[ j ] )
        {
            _memory->destroy_ptr( _pixels[ j ] );
            _pixels[ j ] = 0;
        }
    }

    if ( _memory )
    {
        delete _memory;
        _memory = 0;
    }
}

bool TransferNodeSharedMemory::_cmdClear( co::ICommand& cmd )
{
    co::ObjectICommand command( cmd );
    uint32_t reqID = command.get<uint32_t>();
    clear();

    send( cmd.getNode(), fabric::CMD_TRANSFERNODE_SHAREDMEM_CLEAR_REPLY ) 
        << reqID;

    return true;
}

bool TransferNodeSharedMemory::setupTransmitter()
{
    // default size 4 channel 1080p half float, no depth buffer
    const size_t size = 1920 * 1080 * 2 * sizeof( float ) 
        + sizeof( ImageMetaData );

    std::stringstream name;
    name << "eqSharedMem" << getID() << "_" 
        << getLocalNode()->getNodeID() << "_" << _targetNode->getNodeID();
    _name = name.str();

    _memory = new bip::managed_windows_shared_memory( bip::create_only,
        _name.c_str(), size );

    return true;
}

bool TransferNodeSharedMemory::finishSetupTransmitter( bool ok )
{
    if ( !ok )
    {
        delete _memory;
        _memory = 0;
        getLocalNode()->deregisterObject( this );
    }

    return ok;
}

bool TransferNodeSharedMemory::setupReceiver( co::ObjectICommand& cmd )
{
    _memory = new bip::managed_windows_shared_memory( bip::open_only,
        _name.c_str() );

    return _memory != 0;
}


void TransferNodeSharedMemory::sendImage( Image* image, uint32_t frameNumber, 
                                    uint32_t taskID, const uint128_t& frameID )
{
    resizeMemory( image );
    
    if ( !_memory )
        return;

    if ( !_imageMeta )
    {
        _imageMeta = _memory->construct< ImageMetaData >( METADATA_OBJECT )();
    }

    _imageMeta->commandBuffers = Frame::BUFFER_NONE;
    _imageMeta->alphaUsage = image->getAlphaUsage();
    _imageMeta->pvp = image->getPixelViewport();
    _imageMeta->zoom = image->getZoom();
    _imageMeta->frameNumber = frameNumber;
    _imageMeta->frameID = frameID;

    Frame::Buffer buffers[] = {Frame::BUFFER_COLOR,Frame::BUFFER_DEPTH};

    // for each image attachment
    for( unsigned j = 0; j < 2; ++j )
    {
        Frame::Buffer buffer = buffers[j];
        if ( image->hasPixelData( buffer ))
        {
            uint32_t size = image->getPixelDataSize( buffer );
            if ( _pixels[j] && ( _imageMeta->dataSize[ j ] < size ))
            {
                _memory->destroy_ptr( _pixels[ j ] );
                _pixels[j] = 0;
            }

            if ( !_pixels[ j ] )
            {
                _pixels[ j ] = _memory->construct< uint8_t >(
                    j ? COLORBUFFER_ARRAY : DEPTHBUFFER_ARRAY )[ size ]();
            }

            const PixelData& data = image->getPixelData( buffer );
            _imageMeta->commandBuffers |= buffer;
            _imageMeta->dataSize[j] = size;
            _imageMeta->internalFormat[j] = data.internalFormat;
            _imageMeta->externalFormat[j] = data.externalFormat;
            _imageMeta->pixelSize[j] = data.pixelSize;
            _imageMeta->quality[j] = image->getQuality( buffer );

            memcpy( _pixels[ j ], data.pixels, size );
        }
    }

    // actually this send was supposed be in the beginning of the function and 
    // synchronization was supposed to happen with a interprocess::mutex, but
    // unlocking bip::mutex seems to be really slow for some reason ( > 1500µs )
    initiateSend( frameNumber );
}

bool TransferNodeSharedMemory::receiveImage( Image* image, co::ObjectICommand& )
{
    if ( !_memory )
        return false;

    _imageMeta = _memory->find< ImageMetaData >( METADATA_OBJECT ).first;

    if ( !_imageMeta )
        return false;

    image->setPixelViewport( _imageMeta->pvp );
    image->setAlphaUsage( _imageMeta->alphaUsage );
    image->setZoom( _imageMeta->zoom );

    Frame::Buffer buffers[] = { Frame::BUFFER_COLOR, Frame::BUFFER_DEPTH };
    for( unsigned j = 0; j < 2; ++j )
    {
        const Frame::Buffer buffer = buffers[j];

        if( _imageMeta->commandBuffers & buffer )
        {
            _pixels[j] = _memory->find< uint8_t >( 
                j ? COLORBUFFER_ARRAY : DEPTHBUFFER_ARRAY ).first;
            if ( !_pixels[j] )
                return false;

            PixelData pixelData;
            pixelData.internalFormat  = _imageMeta->internalFormat[j];
            pixelData.externalFormat  = _imageMeta->externalFormat[j];
            pixelData.pixelSize       = _imageMeta->pixelSize[j];
            pixelData.pvp             = _imageMeta->pvp;
            pixelData.compressorName  = EQ_COMPRESSOR_NONE;
            pixelData.compressorFlags = 0;
            pixelData.isCompressed    = false;
            pixelData.pixels = _pixels[j];

            image->setQuality( buffer, _imageMeta->quality[j] );
            image->setPixelData( buffer, pixelData, true );
        }
    }
    return true;
}

void TransferNodeSharedMemory::resizeMemory( const Image* const image )
{
    uint64_t size = 0;
    if ( image->hasPixelData( Frame::BUFFER_COLOR ))
        size += image->getPixelDataSize( Frame::BUFFER_COLOR );
    if ( image->hasPixelData( Frame::BUFFER_DEPTH ))
        size += image->getPixelDataSize( Frame::BUFFER_DEPTH );
    size += sizeof( ImageMetaData );
    
    if ( _memory->get_size() < size )
    {
        uint64_t newsize = _memory->get_size();
        while ( newsize < size )
            newsize <<= 1;

        LBINFO << "resizing shared memory to " << newsize << " bytes" 
               << std::endl;
        
        uint32_t reqID = getLocalNode()->registerRequest();
        send( _targetNode, fabric::CMD_TRANSFERNODE_SHAREDMEM_CLEAR ) << reqID;
        clear();

        getLocalNode()->waitRequest( reqID );

        _memory = new bip::managed_windows_shared_memory( bip::create_only,
            _name.c_str(), newsize );

        if ( !_memory )
        {
            LBERROR << "couldn't create shared memory" << std::endl;
            return;
        }
        
        reqID = getLocalNode()->registerRequest();
        send( _targetNode, fabric::CMD_TRANSFERNODE_SHAREDMEM_MAPMEM ) << reqID;

        bool ok = 0;
        getLocalNode()->waitRequest( reqID, ok );

        if ( !ok )
        {
            clear();
            return;
        }
    }
}

bool TransferNodeSharedMemory::_cmdMapMemory( co::ICommand& cmd )
{
    co::ObjectICommand command( cmd );
    uint32_t reqID = command.get< uint32_t >();

    _memory = new bip::managed_windows_shared_memory( bip::open_only,
        _name.c_str() );

    bool ok = _memory != 0;
    send( cmd.getNode(), fabric::CMD_TRANSFERNODE_SHAREDMEM_MAPMEM_REPLY ) 
        << reqID << ok;

    if ( !ok )
        LBERROR << "couldn't remap shared memory on receiver side" << std::endl;

    return true;
}

bool TransferNodeSharedMemory::_cmdClearReply( co::ICommand& cmd )
{
    co::ObjectICommand command( cmd );
    uint32_t reqID = command.get< uint32_t >();

    getLocalNode()->serveRequest( reqID );
    return true;
}

bool TransferNodeSharedMemory::_cmdMapMemoryReply( co::ICommand& cmd )
{
    co::ObjectICommand command( cmd );
    uint32_t reqID = command.get< uint32_t >();
    bool ok = command.get< bool >();

    getLocalNode()->serveRequest( reqID, ok );
    return true;
}

void TransferNodeSharedMemory::getInstanceData( co::DataOStream& os )
{
    os << _name;
    TransferNode::getInstanceData( os );
}

void TransferNodeSharedMemory::applyInstanceData( co::DataIStream& is )
{
    is >> _name;
    TransferNode::applyInstanceData( is );
}

}
