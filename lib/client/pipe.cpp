
/* Copyright (c) 2005, Stefan Eilemann <eile@equalizergraphics.com> 
   All rights reserved. */

#include "pipe.h"

#include "commands.h"
#include "global.h"
#include "nodeFactory.h"
#include "packets.h"
#include "object.h"
#include "window.h"

#include <sstream>

using namespace eq;
using namespace std;

Pipe::Pipe()
        : eqNet::Object( eq::Object::TYPE_PIPE, CMD_PIPE_ALL ),
          _node(NULL),
          _windowSystem( WINDOW_SYSTEM_NONE ),
#ifdef GLX
          _xDisplay(NULL),
#endif
#ifdef CGL
          _cglDisplayID(NULL),
#endif
          _display(EQ_UNDEFINED_UINT32),
          _screen(EQ_UNDEFINED_UINT32)
{
    registerCommand( CMD_PIPE_CREATE_WINDOW, this, reinterpret_cast<CommandFcn>(
                         &eq::Pipe::_cmdCreateWindow ));
    registerCommand( CMD_PIPE_DESTROY_WINDOW, this,reinterpret_cast<CommandFcn>(
                         &eq::Pipe::_cmdDestroyWindow ));
    registerCommand( CMD_PIPE_INIT, this, reinterpret_cast<CommandFcn>(
                         &eq::Pipe::_cmdInit ));
    registerCommand( REQ_PIPE_INIT, this, reinterpret_cast<CommandFcn>(
                         &eq::Pipe::_reqInit ));
    registerCommand( CMD_PIPE_EXIT, this, reinterpret_cast<CommandFcn>( 
                         &eq::Pipe::pushRequest ));
    registerCommand( REQ_PIPE_EXIT, this, reinterpret_cast<CommandFcn>( 
                         &eq::Pipe::_reqExit ));
    registerCommand( CMD_PIPE_FRAME_SYNC, this, reinterpret_cast<CommandFcn>( 
                         &eq::Pipe::pushRequest ));
    registerCommand( REQ_PIPE_FRAME_SYNC, this, reinterpret_cast<CommandFcn>( 
                         &eq::Pipe::_reqFrameSync ));

    _thread = new PipeThread( this );
}

Pipe::~Pipe()
{
    delete _thread; 
}

void Pipe::_addWindow( Window* window )
{
    _windows.push_back( window );
    window->_pipe = this;
}

void Pipe::_removeWindow( Window* window )
{
    vector<Window*>::iterator iter = find( _windows.begin(), _windows.end(),
                                           window );
    if( iter == _windows.end( ))
        return;
    
    _windows.erase( iter );
    window->_pipe = NULL;
}

bool Pipe::supportsWindowSystem( const WindowSystem windowSystem ) const
{
#ifdef GLX
    if( windowSystem == WINDOW_SYSTEM_GLX )
        return true;
#endif
#ifdef CGL
    if( windowSystem == WINDOW_SYSTEM_CGL )
        return true;
#endif
    return false;
}

WindowSystem Pipe::selectWindowSystem() const
{
    for( WindowSystem i=WINDOW_SYSTEM_NONE; i<WINDOW_SYSTEM_ALL; 
         i = (WindowSystem)((int)i+1) )
    {
        if( supportsWindowSystem( i ))
            return i;
    }
    EQASSERTINFO( 0, "No supported window system found" );
    return WINDOW_SYSTEM_NONE;
}

#ifdef GLX
void Pipe::setXDisplay( Display* display )
{
    _xDisplay = display; 
    _pvp.x    = 0;
    _pvp.y    = 0;
    if( display )
    {
        _pvp.w = DisplayWidth(  display, DefaultScreen( display ));
        _pvp.h = DisplayHeight( display, DefaultScreen( display ));
    }
    else
    {
        _pvp.w = 0;
        _pvp.h = 0;
    }
}
#endif
#ifdef CGL
void Pipe::setCGLDisplayID( CGDirectDisplayID id )
{
    _cglDisplayID = id; 
    if( id )
    {
        const CGRect displayRect = CGDisplayBounds( id );
        _pvp.x = displayRect.origin.x;
        _pvp.y = displayRect.origin.y;
        _pvp.w = displayRect.size.width;
        _pvp.h = displayRect.size.height;
    }
    else
    {
        _pvp.x = 0;
        _pvp.y = 0;
        _pvp.w = 0;
        _pvp.h = 0;
    }
}
#endif

ssize_t Pipe::_runThread()
{
    Config* config = getConfig();
    EQASSERT( config );

    Node::setLocalNode( config->getNode( ));

    eqNet::Node*   node;
    eqNet::Packet* packet;

    while( _thread->isRunning( ))
    {
        _requestQueue.pop( &node, &packet );
        switch( config->dispatchPacket( node, packet ))
        {
            case eqNet::COMMAND_HANDLED:
                break;

            case eqNet::COMMAND_ERROR:
                EQERROR << "Error handling command packet" << endl;
                abort();

            case eqNet::COMMAND_RESCHEDULE:
                EQUNIMPLEMENTED;
        }
    }

    _thread->join();
    return EXIT_SUCCESS;
}

//---------------------------------------------------------------------------
// command handlers
//---------------------------------------------------------------------------
eqNet::CommandResult Pipe::_cmdCreateWindow( eqNet::Node* node, const eqNet::Packet* pkg )
{
    PipeCreateWindowPacket* packet = (PipeCreateWindowPacket*)pkg;
    EQINFO << "Handle create window " << packet << endl;

    Window* window = Global::getNodeFactory()->createWindow();
    
    getConfig()->_addRegisteredObject( packet->windowID, window );
    _addWindow( window );
    return eqNet::COMMAND_HANDLED;
}

eqNet::CommandResult Pipe::_cmdDestroyWindow( eqNet::Node* node, const eqNet::Packet* pkg )
{
    PipeDestroyWindowPacket* packet = (PipeDestroyWindowPacket*)pkg;
    EQINFO << "Handle destroy window " << packet << endl;

    Config* config = getConfig();
    Window* window = (Window*)config->getObject( packet->windowID );
    if( !window )
        return eqNet::COMMAND_HANDLED;

    _removeWindow( window );
    config->deregisterObject( window );
    delete window;
    return eqNet::COMMAND_HANDLED;
}

eqNet::CommandResult Pipe::_cmdInit( eqNet::Node* node, const eqNet::Packet* pkg )
{
    PipeInitPacket* packet = (PipeInitPacket*)pkg;
    EQINFO << "handle pipe init (recv) " << packet << endl;

    EQASSERT( _thread->isStopped( ));
    _thread->start();
    pushRequest( node, pkg );
    return eqNet::COMMAND_HANDLED;
}

eqNet::CommandResult Pipe::_reqInit( eqNet::Node* node, const eqNet::Packet* pkg )
{
    PipeInitPacket* packet = (PipeInitPacket*)pkg;
    EQINFO << "handle pipe init (pipe) " << packet << endl;
    PipeInitReplyPacket reply( packet );
    
    _display = packet->display;
    _screen  = packet->screen;

    _windowSystem = selectWindowSystem();
    reply.result = init( packet->initID );

    if( !reply.result )
    {
        node->send( reply );
        return eqNet::COMMAND_HANDLED;
    }

    switch( _windowSystem )
    {
#ifdef GLX
        case WINDOW_SYSTEM_GLX:
            if( !_xDisplay )
            {
                EQERROR << "init() did not set a valid display connection" 
                        << endl;
                reply.result = false;
                node->send( reply );
                return eqNet::COMMAND_HANDLED;
            }
            // TODO: gather and send back display information
            EQINFO << "Using display " << DisplayString( _xDisplay ) << endl;
            break;
#endif
#ifdef CGL
        case WINDOW_SYSTEM_CGL:
            if( !_cglDisplayID )
            {
                EQERROR << "init() did not set a valid display id" << endl;
                reply.result = false;
                node->send( reply );
                    return eqNet::COMMAND_HANDLED;
            }
                
            // TODO: gather and send back display information
            EQINFO << "Using display " << _display << endl;
            break;
#endif

        default: EQUNIMPLEMENTED;
    }

    node->send( reply );
    return eqNet::COMMAND_HANDLED;
}

eqNet::CommandResult Pipe::_reqExit( eqNet::Node* node, 
                                     const eqNet::Packet* pkg )
{
    PipeExitPacket* packet = (PipeExitPacket*)pkg;
    EQINFO << "handle pipe exit " << packet << endl;

    exit();
    
    PipeExitReplyPacket reply( packet );
    node->send( reply );

    _thread->exit( EXIT_SUCCESS );
    return eqNet::COMMAND_HANDLED;
}

eqNet::CommandResult Pipe::_reqFrameSync(eqNet::Node* node, 
                                         const eqNet::Packet* packet )
{
    PipeFrameSyncPacket reply( getSession()->getID(), getID( ));
    node->send( reply );
    return eqNet::COMMAND_HANDLED;
}

//---------------------------------------------------------------------------
// pipe-thread methods
//---------------------------------------------------------------------------
bool Pipe::init( const uint32_t initID )
{
    switch( _windowSystem )
    {
        case WINDOW_SYSTEM_GLX:
            return initGLX();

        case WINDOW_SYSTEM_CGL:
            return initCGL();

        default:
            EQERROR << "Unknown windowing system: " << _windowSystem << endl;
            return false;
    }
}

bool Pipe::initGLX()
{
#ifdef GLX
    ostringstream  stringStream;
    const uint32_t display = getDisplay();
    const uint32_t screen  = getScreen();
    
    if( display != EQ_UNDEFINED_UINT32 )
    { 
        if( screen == EQ_UNDEFINED_UINT32 )
            stringStream << ":" << display;
        else
            stringStream << ":" << display << "." << screen;
    }
    else if( screen != EQ_UNDEFINED_UINT32 )
        stringStream << ":0." << screen;
    
    const string displayName  = stringStream.str();
    const char*  cDisplayName = ( displayName.length() == 0 ? 
                                  NULL : displayName.c_str( ));
    Display*     xDisplay     = XOpenDisplay( cDisplayName );
            
    if( !xDisplay )
    {
        EQERROR << "Can't open display: " << displayName << endl;
        return false;
    }
    
    setXDisplay( xDisplay );
    _screen = DefaultScreen( xDisplay );
    EQINFO << "Opened X display " << xDisplay << ", screen " << _screen << endl;
    return true;
#else
    return false;
#endif
}

bool Pipe::initCGL()
{
#ifdef CGL
    const uint32_t    display   = getDisplay();
    CGDirectDisplayID displayID = CGMainDisplayID();

    if( display != EQ_UNDEFINED_UINT32 )
    {
        CGDirectDisplayID displayIDs[display+1];
        CGDisplayCount    nDisplays;

        if( CGGetOnlineDisplayList( display+1, displayIDs, &nDisplays ) !=
            kCGErrorSuccess )
        {
            EQERROR << "Can't get display identifier for display " << display 
                  << endl;
            return false;
        }

        if( nDisplays <= display )
        {
            EQERROR << "Can't get display identifier for display " << display 
                  << ", not enough displays for this system" << endl;
            return false;
        }

        displayID = displayIDs[display];
    }

    setCGLDisplayID( displayID );
    EQINFO << "Using CGL displayID " << displayID << endl;
    return true;
#else
    return false;
#endif
}

bool Pipe::exit()
{
    switch( _windowSystem )
    {
        case WINDOW_SYSTEM_GLX:
            exitGLX();
            return true;

        case WINDOW_SYSTEM_CGL:
            exitCGL();
            return true;

        default:
            EQWARN << "Unknown windowing system: " << _windowSystem << endl;
            return false;
    }
}

void Pipe::exitGLX()
{
#ifdef GLX
    Display* xDisplay = getXDisplay();
    if( !xDisplay )
        return;

    setXDisplay( NULL );
    XCloseDisplay( xDisplay );
    EQINFO << "Closed X display " << xDisplay << endl;
#endif
}

void Pipe::exitCGL()
{
#ifdef CGL
    setCGLDisplayID( NULL );
    EQINFO << "Reset X CGL displayID " << endl;
#endif
}
