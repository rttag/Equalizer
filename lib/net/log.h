
/* Copyright (c) 2006, Stefan Eilemann <eile@equalizergraphics.com> 
   All rights reserved. */

#ifndef EQNET_LOG_H
#define EQNET_LOG_H

#include <eq/base/log.h>

namespace eq
{
namespace net
{
    enum LogTopics
    {
        LOG_OBJECTS = base::LOG_CUSTOM, // 16
        LOG_BARRIER = 0x20,               // 32
        LOG_WIRE    = 0x40,               // 64
        LOG_NETPERF = 0x80,               // 128
        LOG_CUSTOM  = 0x100               // 256
    };
}
}
#endif // EQNET_LOG_H
