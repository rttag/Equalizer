#pragma once

#include "transferNode.h"

#include <eq/fabric/pixelViewport.h>
#include <eq/fabric/zoom.h>

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/managed_windows_shared_memory.hpp>

namespace bip = boost::interprocess;

namespace eq
{

class TransferNodeSharedMemory : public TransferNode
{

    ~TransferNodeSharedMemory();

    virtual bool setupTransmitter();
    virtual bool setupReceiver( co::ObjectICommand& cmd );
    virtual bool finishSetupTransmitter( bool ok );
    virtual void sendImage( Image* image, 
                            uint32_t frameNumber, 
                            uint32_t taskID, 
                            const co::uint128_t& frameID );
    virtual bool receiveImage( Image* image, co::ObjectICommand& cmd );

    virtual void getInstanceData( co::DataOStream& os );
    virtual void applyInstanceData( co::DataIStream& is );

protected:
    virtual void attach( const UUID& id, const uint32_t instanceID );

    struct ImageMetaData
    {
        fabric::PixelViewport pvp;
        fabric::Zoom zoom;
        uint32_t commandBuffers;
        bool alphaUsage;
        uint32_t frameNumber;
        co::uint128_t frameID;
        float quality[ 2 ];
        uint32_t internalFormat[ 2 ];
        uint32_t externalFormat[ 2 ];
        uint32_t pixelSize[ 2 ];
        uint64_t dataSize[ 2 ];
    };

    friend class TransferNode;

    TransferNodeSharedMemory();

private:

    void resizeMemory( const Image* const image );
    void clear();

    bool _cmdClear( co::ICommand& cmd );
    bool _cmdMapMemory( co::ICommand& cmd );
    bool _cmdMapMemoryReply( co::ICommand& cmd );

    bip::managed_windows_shared_memory* _memory;
    ImageMetaData*                      _imageMeta;
    uint8_t*                            _pixels[ 2 ];
    std::string                         _name;
};

}