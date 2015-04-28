#include "transferNode.h"
#include "transferNodeSharedMemory.h"
#include "transferNodeCollage.h"
#include "proxyStatistics.h"

#include <eq/fabric/commands.h>

#include <co/connectionDescription.h>
#include <co/objectOCommand.h>
#include <co/objectICommand.h>

#include <lunchbox/buffer.h>

#include "image.h"

namespace eq
{

typedef co::CommandFunc<TransferNode> CmdFunc;

TransferNode::TransferNode()
    : _targetNode( 0 )
    , _stats( 0 )
    , _requestToken( false )
    , _type( INVALID )
    , _addImageFunc()
    , _createImageFunc()
{}

void TransferNode::attach( const UUID& id, const uint32_t instanceID )
{
    co::Object::attach( id, instanceID );

    co::CommandQueue* commandQ = getLocalNode()->getCommandThreadQueue();

    registerCommand( fabric::CMD_TRANSFERNODE_TRANSFER_IMAGE,
        CmdFunc( this, &TransferNode::_cmdReceiveImage ), commandQ );
}

TransferNode* TransferNode::createTransmitter( co::NodePtr toNode, 
                                               co::LocalNodePtr localNode )
{
    TransferNode* transmitter = 0;
    co::ConnectionPtr connection = toNode->getConnection();
    co::ConstConnectionDescriptionPtr description =connection->getDescription();

#ifdef _WIN32
    if ( description->type == co::ConnectionType::CONNECTIONTYPE_NAMEDPIPE )
    {
        LBINFO << "Creating shared memory transfer node for " 
            << toNode->getNodeID() << std::endl;
        transmitter = new TransferNodeSharedMemory;
    }
#endif
    if ( !transmitter )
    {
        LBINFO << "Creating collage based transfer node for "
               << toNode->getNodeID() << std::endl;
        transmitter = new TransferNodeCollage;
    }

    transmitter->setTargetNode( toNode );

    bool ret = localNode->registerObject( transmitter );
    if ( !ret )
    {
        LBERROR << "registering of transfer node failed" << std::endl;
        delete transmitter;
        return 0;
    }

    if ( !transmitter->setupTransmitter() )
    {
        LBERROR << "setup of transfer node unsuccessful" << std::endl;
        delete transmitter;
        transmitter = 0;
    }

    return transmitter;
}

TransferNode* TransferNode::createReceiver( co::LocalNodePtr localNode, 
                                            Type type, const UUID& id )
{
    TransferNode* transferNode;

    switch ( type )
    {
#ifdef _WIN32
    case TransferNode::Type::SHAREDMEM:
        LBINFO << "creating shared memory receiver node" << std::endl;
        transferNode = new TransferNodeSharedMemory;
        break;
#endif
    case TransferNode::Type::COLLAGE:
        LBINFO << "creating collage based receiver node" << std::endl;
        transferNode = new TransferNodeCollage;
        break;
    case TransferNode::Type::INVALID:
    default:
        LBERROR << "unknown transfer node type" << std::endl;
        return 0;
    }

    if ( !localNode->mapObject( transferNode, id ))
    {
        LBERROR << "Can't map transfer node" << std::endl;
        delete transferNode;
    }

    return transferNode;
}


void TransferNode::setStatsProxy( StatisticsProxy* stats ) 
{ 
    _stats = stats; 
}

void TransferNode::setTokenHint( bool flag ) 
{ 
    _requestToken = flag; 
}

void TransferNode::setCreateImageFunc( CreateImageFunc func )
{
    _createImageFunc = func;
}

void TransferNode::setAddImageFunc( AddImageFunc func )
{
    _addImageFunc = func;
}

void TransferNode::setTargetNode( co::NodePtr targetNode )
{
    _targetNode = targetNode;
}

bool TransferNode::_cmdReceiveImage( co::ICommand& cmd )
{
    if ( !_addImageFunc || !_createImageFunc )
        return false;

    co::ObjectICommand command( cmd );
    const uint32_t frameNumber = command.get< uint32_t >();
    ProxyStatistics event( Statistic::NODE_FRAME_DECOMPRESS, _stats, 
        frameNumber );

    Image* image = _createImageFunc();
    if ( image && receiveImage( image, command ))
    {
        _addImageFunc( image );
    }
    else
    {
        LBERROR << "Error creating or receiving image" << std::endl;
    }

    return true;
}

TransferNode::Type TransferNode::getType()
{
    return _type;
}

co::ObjectOCommand TransferNode::initiateSend( uint32_t frameNumber )
{
    co::ObjectOCommand cmd( send( _targetNode, 
        fabric::CMD_TRANSFERNODE_TRANSFER_IMAGE ));
    cmd << frameNumber;
    return cmd;
}

}

