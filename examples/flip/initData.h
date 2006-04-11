
/* Copyright (c) 2006, Stefan Eilemann <eile@equalizergraphics.com> 
   All rights reserved. */

#ifndef EQ_FLIP_INITDATA_H
#define EQ_FLIP_INITDATA_H

#include "flip.h"
#include "frameData.h"

#include <eq/eq.h>

class InitData : public eqNet::Object
{
public:
    InitData();
    InitData( const void* data, const uint64_t size );

    void       setFrameData( FrameData* frameData );
    FrameData* getFrameData();

    void               setFilename( const std::string& filename );
    const std::string& getFilename() const { return _filename; }

protected:
    const void* getInstanceData( uint64_t* size );

private:
    uint32_t                  _frameDataID;
    eqBase::RefPtr<FrameData> _frameData;

    std::string _filename; 

    char* _instanceData;
    void  _clearInstanceData();
};



#endif // EQ_FLIP_INITDATA_H

