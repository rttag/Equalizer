
/* Copyright (c) 2007-2013, Stefan Eilemann <eile@equalizergraphics.com>
 *               2011-2012, Daniel Nachbaur <danielnachbaur@gmail.com>
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

#include "channelUploadVisitor.h"
#include "compound.h"
#include "frame.h"

#include <eq/fabric/commands.h>
#include <eq/fabric/eye.h>

namespace eq
{
namespace server
{

ChannelUploadVisitor::ChannelUploadVisitor( Channel* channel,
                                            const uint128_t frameID )
        : _channel( channel )
        , _frameID( frameID )
{}

VisitorResult ChannelUploadVisitor::visitPre( const Compound* compound )
{
    if( compound->getChannel() != _channel ||
        !compound->testInheritTask( fabric::TASK_ASSEMBLE ) )
        return TRAVERSE_CONTINUE;

    const Frames& inputFrames = compound->getInputFrames();
    LBASSERT( !inputFrames.empty( ));

    std::vector< const co::ObjectVersion > frames;
    std::vector< const Vector2i > offsets;
    std::vector< const Eye > eyes;
    for( Frames::const_iterator iter = inputFrames.begin();
        iter != inputFrames.end(); ++iter )
    {
        Frame* frame = *iter;
        for( uint32_t i = fabric::EYE_CYCLOP; i <= fabric::EYE_LAST; i <<= 1 )
        {
            if( !frame->hasData( Eye( i ) )) // TODO: filter: buffers, vp, eye
                continue;

            frames.push_back( co::ObjectVersion( frame ) );
            offsets.push_back( frame->getOffset() );
            eyes.push_back( Eye( i ) );
        }
    }

    if( frames.empty( ))
        return TRAVERSE_CONTINUE;

    // prepare upload task
    _channel->send( fabric::CMD_CHANNEL_PREPARE_UPLOAD ) 
        << frames << offsets << eyes << _frameID 
        << compound->testInheritTask( fabric::TASK_ASYNCUPLOAD );

    return TRAVERSE_CONTINUE;
}

}
}

