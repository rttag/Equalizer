
/* Copyright (c) 2010, Stefan Eilemann <eile@eyescale.ch> 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer. Redistributions in binary
 * form must reproduce the above copyright notice, this list of conditions and
 * the following disclaimer in the documentation and/or other materials provided
 * with the distribution. Neither the name of Eyescale Software GmbH nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EQ_ADMIN_ADD_WINDOW_H
#define EQ_ADMIN_ADD_WINDOW_H

#include "findPipe.h"

#include <eq/admin.h>

namespace eqAdmin
{

inline bool addWindow( eq::admin::ServerPtr server )
{
    // Find first pipe...
    eq::admin::Pipe* pipe = findPipe( server );
    if( !pipe )
       return false;

    // Add window (->channel->segment->canvas->layout->view)
    eq::admin::Config* config = pipe->getConfig();
    eq::admin::Window* window = new eq::admin::Window( pipe );
    eq::admin::Channel* channel = new eq::admin::Channel( window );
    eq::admin::Canvas* canvas = new eq::admin::Canvas( config );
    eq::admin::Segment* segment = new eq::admin::Segment( canvas );
    eq::admin::Layout* layout = new eq::admin::Layout( config );
    eq::admin::View* view = new eq::admin::View( layout );
    eq::admin::Observer* observer = new eq::admin::Observer( config );

    window->setPixelViewport( eq::fabric::PixelViewport( 100, 100, 400, 300 ));
    window->setName( "Runtime-created window" );
    canvas->setName( "Runtime-created canvas" );
    layout->setName( "Runtime-created layout" );
    observer->setName( "Runtime-created observer" );

    segment->setEyes( eq::fabric::EYE_CYCLOP ); // Mono

    // Passive stereo
    eq::admin::Channel* channelLeft = new eq::admin::Channel( window );
    eq::admin::Channel* channelRight = new eq::admin::Channel( window );

    eq::admin::Segment* segmentLeft = new eq::admin::Segment( canvas );
    eq::admin::Segment* segmentRight = new eq::admin::Segment( canvas );

    channelLeft->setViewport( eq::fabric::Viewport( .0f, .0f, 0.5f, 1.0f ) );
    segmentLeft->setEyes( eq::fabric::EYE_LEFT );
    segmentLeft->setChannel( channelLeft );

    channelRight->setViewport( eq::fabric::Viewport( .5f, .0f, 0.5f, 1.0f ) );
    segmentRight->setEyes( eq::fabric::EYE_RIGHT );
    segmentRight->setChannel( channelRight );

    view->setObserver( observer );
    segment->setChannel( channel );
    canvas->addLayout( layout );
    canvas->setWall( eq::fabric::Wall( eq::fabric::Vector3f(-.4f,-.3f,-1.f ),
                                       eq::fabric::Vector3f( .4f,-.3f,-1.f ),
                                       eq::fabric::Vector3f(-.4f, .3f,-1.f )));
    config->commit();
    return true;
}

}

#endif // EQ_ADMIN_ADD_WINDOW_H
