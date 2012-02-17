##
# Path : libs/collage/files.cmake
# Copyright (c) 2010 Cedric Stalder <cedric.stalder@gmail.ch>
#               2011-2012 Stefan Eilemann <eile@eyescale.ch>
##

set(COBASE_PUBLIC_HEADERS 
    api.h
    atomic.h
    base.h
    bitOperation.h
    buffer.h
    clock.h
    compiler.h
    condition.h
    debug.h
    defines.h
    dso.h
    error.h
    errorRegistry.h
    file.h
    global.h
    hash.h
    init.h
    launcher.h
    lfQueue.h
    lock.h
    lockable.h
    log.h
    memoryMap.h
    monitor.h
    mtQueue.h
    nonCopyable.h
    omp.h
    os.h
    perThread.h
    perThreadRef.h
    pluginRegistry.h
    pool.h
    refPtr.h
    referenced.h
    requestHandler.h
    rng.h
    scopedMutex.h
    sleep.h
    spinLock.h
    stdExt.h
    thread.h
    threadID.h
    timedLock.h
    types.h
    uint128_t.h
    uuid.h
  )

set(COBASE_HEADERS 
    compressor.h
    compressorInfo.h
    cpuCompressor.h
    memcpy.h
    plugin.h
  )

 set(COBASE_SOURCES
    atomic.cpp
    clock.cpp
    compressor.cpp
    condition.cpp
    condition_w32.ipp
    cpuCompressor.cpp
    debug.cpp
    dso.cpp
    error.cpp
    errorRegistry.cpp
    file.cpp
    global.cpp
    init.cpp
    launcher.cpp
    lock.cpp
    log.cpp
    memoryMap.cpp
    omp.cpp
    plugin.cpp
    pluginRegistry.cpp
    referenced.cpp
    requestHandler.cpp
    rng.cpp
    spinLock.cpp
    sleep.cpp
    thread.cpp
    threadID.cpp
    timedLock.cpp
    uint128_t.cpp
    uuid.cpp
  )
