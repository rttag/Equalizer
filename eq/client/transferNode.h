#pragma once

#include "types.h"

#include <co/object.h>

#include <boost/function.hpp>

namespace eq
{

class Image;
class StatisticsProxy;

class TransferNode : public co::Object
{
public:

    enum Type
    {
        INVALID = -1,
        COLLAGE = 0,
        SHAREDMEM
    };

    virtual bool setupTransmitter() = 0;
    virtual bool setupReceiver( co::ObjectICommand& cmd ) = 0;
    virtual bool finishSetupTransmitter( bool ok ) { return ok; }
    virtual void sendImage( Image* image, uint32_t frameNumber, 
        uint32_t taskID, const co::uint128_t& frameID ) = 0;
    virtual bool receiveImage( Image* image, co::ObjectICommand& cmd ) = 0;

    static TransferNode* createTransmitter( co::NodePtr toNode, 
                                            co::LocalNodePtr localNode );
    static TransferNode* createReceiver( co::LocalNodePtr localNode, 
                                         Type type, const UUID& id );

    void setStatsProxy( StatisticsProxy* stats );
    void setTokenHint( bool flag );
    void setTargetNode( co::NodePtr targetNode );

    typedef boost::function< Image* ( void ) > CreateImageFunc;
    typedef boost::function< void ( Image* ) > AddImageFunc;
    void setCreateImageFunc( CreateImageFunc func );
    void setAddImageFunc( AddImageFunc func );
    Type getType();

protected:
    virtual void attach( const UUID& id, const uint32_t instanceID );
    co::ObjectOCommand initiateSend( uint32_t frameNumber );

    TransferNode();

    co::NodePtr _targetNode;
    StatisticsProxy* _stats;
    bool _requestToken;
    Type _type;

private:
    bool _cmdReceiveImage( co::ICommand& cmd );

    CreateImageFunc _createImageFunc;
    AddImageFunc    _addImageFunc;
};
}

namespace lunchbox
{
    template<> inline void byteswap( eq::TransferNode::Type& value ) 
    { byteswap( reinterpret_cast< uint32_t& >( value )); }
}
