#pragma once

#include "transferNode.h"

namespace eq
{

class TransferNodeCollage : public TransferNode
{
    virtual bool setupTransmitter();
    virtual bool setupReceiver( co::ObjectICommand& cmd );
    virtual bool finishSetupTransmitter( bool ok ) { return ok; }
    virtual void sendImage( Image* image, 
                            uint32_t frameNumber, 
                            uint32_t taskID, 
                            const co::uint128_t& frameID );
    virtual bool receiveImage( Image* image, co::ObjectICommand& cmd );

    virtual void getInstanceData( co::DataOStream& os );
    virtual void applyInstanceData( co::DataIStream& is );

protected:
    friend class TransferNode;

    typedef std::vector< const PixelData* > PixelDatas;
    typedef std::vector< const float > Qualities;

    TransferNodeCollage();

    co::LocalNode::SendToken _aquireSendToken( uint32_t frameNumber, 
                                               uint32_t taskID );

    uint64_t _getPixelDatas( PixelDatas& pixelDatas,
                             Qualities& qualities,
                             uint32_t frameNumber, 
                             uint32_t taskID, 
                             Image* image );

    void _send( Image* image, 
                      uint64_t imageDataSize, 
                      uint32_t frameNumber, 
                      const co::uint128_t& frameID,
                      const PixelDatas& pixelDatas,
                      const Qualities& qualities);

    void _sendPixelData( const PixelDatas& pixelDatas, 
                         const Qualities& qualities );
};

}
