
/* Copyright (c) 2006-2012, Stefan Eilemann <eile@equalizergraphics.com>
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

#include "frame.h"

#include "compound.h"
#include "frameData.h"
#include "node.h"

#include <co/dataIStream.h>
#include <co/dataOStream.h>

namespace eq
{
namespace server
{

Frame::Frame()
        : _compound( 0 )
        , _buffers( BUFFER_UNDEFINED )
        , _type( TYPE_MEMORY )
        , _masterFrameData( 0 )
{
    setNativeZoom( Zoom( 0.f, 0.f )); //set invalid zoom to detect 'set' state
    for( unsigned i = 0; i < NUM_EYES; ++i )
        _frameData[i] = 0;
}

Frame::Frame( const Frame& from )
        : fabric::Frame( from )
        , _compound( 0 )
        , _vp( from._vp )
        , _buffers( from._buffers )
        , _type( from._type )
        , _masterFrameData( 0 )
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
        _frameData[i] = 0;
}

Frame::~Frame()
{
    LBASSERT( _datas.empty());
    _compound = 0;
    _masterFrameData = 0;
}

void Frame::flush()
{
    unsetData();

    while( !_datas.empty( ))
    {
        FrameData* data = _datas.front();
        _datas.pop_front();
        getLocalNode()->deregisterObject( data );
        delete data;
    }
}

void Frame::unsetData()
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        _frameData[i] = 0;
        _inputFrames[i].clear();
        _getInputFrameDatas( i ).clear();
        _getInputNetNodes( i ).clear();
        _getOutputNetNodes( i ).clear();
    }
}

void Frame::commitData()
{
    if( !_masterFrameData ) // not used
        return;

    for( unsigned i = 0; i< NUM_EYES; ++i )
    {
        if( _frameData[i] )
        {
            if( _frameData[i] != _masterFrameData )
                _frameData[i]->_data = _masterFrameData->_data;

            _frameData[i]->commit();
        }
    }
}

uint128_t Frame::commit( const uint32_t incarnation )
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
        _setDataVersion( i, _frameData[i] );
    return co::Object::commit( incarnation );
}

void Frame::cycleData( const uint32_t frameNumber, const Compound* compound )
{
    _masterFrameData = 0;
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        for( FramesCIter j = _inputFrames[i].begin(); j != _inputFrames[i].end(); ++j )
        {
            //deleted inputframes from Nodes
            (*j)->_getOutputNetNodes( i ).clear();
        }

        _inputFrames[i].clear();

        const Eye eye = Eye(1<<i);
        if( !compound->isInheritActive( eye )) // eye pass not used
        {
            _frameData[i] = 0;
            continue;
        }

        // reuse unused frame data
        FrameData*     data    = _datas.empty() ? 0 : _datas.back();
        const uint32_t latency = getAutoObsolete();
        const uint32_t dataAge = data ? data->getFrameNumber() : 0;

        if( data && dataAge < frameNumber-latency-1 && frameNumber > latency )
            // not used anymore
            _datas.pop_back();
        else // still used - allocate new data
        {
            data = new FrameData;

            getLocalNode()->registerObject( data );
            data->setAutoObsolete( 1 ); // current + in use by render nodes
        }

        data->setFrameNumber( frameNumber );
    
        _datas.push_front( data );
        _frameData[i] = data;
        _getInputFrameDatas( i ).clear();
        _getInputNetNodes( i ).clear();
        _getOutputNetNodes( i ).clear();
        if( !_masterFrameData )
            _masterFrameData = data;
    }
}

void Frame::addInputFrame( Frame* frame, const Compound* compound )
{
    for( unsigned i = 0; i < NUM_EYES; ++i )
    {
        // eye pass not used && no output frame for eye pass
        const Eye eye = Eye( 1<<i );
        if( compound->isInheritActive( eye ) && _frameData[i] )     
        {
            frame->_frameData[i] = _frameData[i];
            _inputFrames[i].push_back( frame );

            const Node* inputNode = frame->getNode();
            if( inputNode != getNode( ))
            {
                co::NodePtr inputNetNode = inputNode->getNode();
                FrameData* data = frame->getData( eye );
                co::ObjectVersion v( data );

                if( inputNetNode ) 
                {
                    if ( v.identifier == 0 )
                        LBERROR << "identifier zero ACHTUNG" << std::endl;
                    _getInputFrameDatas( i ).push_back( v.identifier );
                    _getInputNetNodes( i ).push_back( inputNetNode->getNodeID( ));
                }
                else
                {
                    if ( v.identifier != 0 )
                        LBERROR << "identifier nonzero ACHTUNG" << std::endl;
                }
            }
            //add output frames too
            const Node* outputNode = getNode();
            if( outputNode != frame->getNode( ))
            {
                co::NodePtr outputNetNode = outputNode->getNode();
                if( outputNetNode )
                {
                    frame->_getOutputNetNodes( i ).push_back( outputNetNode->getNodeID());
                }
            }
        }
        else
        {
            frame->_frameData[i] = 0;
        }
    }
}

std::ostream& operator << ( std::ostream& os, const Frame& frame )
{
    os << lunchbox::disableFlush << "frame" << std::endl
       << "{" << std::endl << lunchbox::indent
       << "name     \"" << frame.getName() << "\"" << std::endl;

    const uint32_t buffers = frame.getBuffers();
    if( buffers != Frame::BUFFER_UNDEFINED )
    {
        os << "buffers  [";
        if( buffers & Frame::BUFFER_COLOR )  os << " COLOR";
        if( buffers & Frame::BUFFER_DEPTH )  os << " DEPTH";
        os << " ]" << std::endl;
    }

    const Frame::Type frameType = frame.getType();
    if( frameType != Frame::TYPE_MEMORY )
        os  << frameType;
    
    const Viewport& vp = frame.getViewport();
    if( vp != Viewport::FULL )
        os << "viewport " << vp << std::endl;

    const Zoom& zoom = frame.getZoom();
    if( zoom.isValid() && zoom != Zoom::NONE )
        os << zoom << std::endl;

    return os << lunchbox::exdent << "}" << lunchbox::enableFlush;
}

}
}
