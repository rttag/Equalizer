
/* Copyright (c) 2007-2011, Stefan Eilemann <eile@equalizergraphics.com> 
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

#pragma once

#include "compoundVisitor.h" // base class
#include "types.h"

namespace eq
{

namespace server
{
    class Channel;
    class FrustumData;
    
    /** The compound visitor generating the draw tasks for a channel. */
    class ChannelUploadVisitor : public CompoundVisitor
    {
    public:
        ChannelUploadVisitor( Channel* channel, const uint128_t frameID );
        virtual ~ChannelUploadVisitor() {}

        /** Visit a non-leaf compound on the down traversal. */
        virtual VisitorResult visitPre( const Compound* compound );

    private:
        Channel*        _channel;
        const uint128_t _frameID;
    };
}
}
