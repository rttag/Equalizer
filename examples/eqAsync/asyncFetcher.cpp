/* Copyright (c) 2009-2011, Maxim Makhinya <maxmah@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of Eyescale Software GmbH nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
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
 *
 */

#include "asyncFetcher.h"

#include "eqAsync.h"

#include <eq/util/objectManager.ipp>
#include <eq/util/bitmapFont.ipp>

#include <eq/util/texture.h>

#include <eq/window.h>
#include <eq/systemWindow.h>

#ifdef GLX
    #include <eq/glXWindow.h>
#endif
#ifdef AGL
    #include "aglWindowShared.h"
#endif
#ifdef WGL
    #include <eq/wglWindow.h>
#endif

#include <ctime>


namespace eqAsync
{

static eq::SystemWindow* initSharedContextWindow( eq::Window* wnd )
{
    EQASSERT( wnd );

    // store old drawable of window and set window's drawable to FBO,
    // create another osWindow and restore original drowable

    const int32_t drawable  = wnd->getIAttribute( eq::Window::IATTR_HINT_DRAWABLE );
    if( drawable != eq::FBO ) wnd->setIAttribute( eq::Window::IATTR_HINT_DRAWABLE, eq::FBO );

    const int32_t stencil = wnd->getIAttribute( eq::Window::IATTR_PLANES_STENCIL );
    if( stencil != 0 )      wnd->setIAttribute( eq::Window::IATTR_PLANES_STENCIL, 0 );

    const eq::Pipe* pipe = wnd->getPipe();
    EQASSERT( pipe );

    eq::SystemWindow* sharedContextWindow = 0;
    switch( pipe->getWindowSystem( ))
    {
    #ifdef GLX
        case eq::WINDOW_SYSTEM_GLX:
            EQINFO << "Using GLXWindow" << std::endl;
            sharedContextWindow = new eq::GLXWindow( wnd );
            break;
    #endif

    #ifdef AGL
        case eq::WINDOW_SYSTEM_AGL:
            EQINFO << "Using AGLWindow" << std::endl;
            sharedContextWindow = new AGLWindowShared( wnd );
            break;
    #endif

    #ifdef WGL
        case eq::WINDOW_SYSTEM_WGL:
            EQINFO << "Using WGLWindow" << std::endl;
            sharedContextWindow = new eq::WGLWindow( wnd );
            break;
    #endif

        default:
            EQERROR << "Window system " << pipe->getWindowSystem() 
                    << " not implemented or supported" << std::endl;
            return 0;
    }
    EQASSERT( sharedContextWindow );

    if( !sharedContextWindow->configInit( ))
    {
        EQWARN << "OS Window initialization failed: " << std::endl;
        delete sharedContextWindow;
        return 0;
    }

    if( drawable != eq::FBO )
        wnd->setIAttribute( eq::Window::IATTR_HINT_DRAWABLE, drawable );

    if( stencil != 0 )
        wnd->setIAttribute( eq::Window::IATTR_PLANES_STENCIL, stencil );

    sharedContextWindow->makeCurrent();

    EQWARN << "Async fetcher initialization finished" << std::endl;
    return sharedContextWindow;
}


static void deleteScharedContextWindow( eq::Window*                   wnd,
                                        eq::SystemWindow**            sharedContextWindow,
                                        AsyncFetcher::ObjectManager** objectManager )
{
    EQWARN << "Deleting shared context" << std::endl;
    if( !sharedContextWindow || !*sharedContextWindow )
        return;

    if( *objectManager )
    {
        (*objectManager)->deleteAll();
        delete *objectManager;
        *objectManager = 0;
    }

    const int32_t drawable = wnd->getIAttribute( eq::Window::IATTR_HINT_DRAWABLE );
    if( drawable != eq::FBO )
        wnd->setIAttribute( eq::Window::IATTR_HINT_DRAWABLE, eq::FBO );

    (*sharedContextWindow)->configExit( ); // mb set window to 0 before that?

    delete *sharedContextWindow;
    *sharedContextWindow = 0;

    if( drawable != eq::FBO )
        wnd->setIAttribute( eq::Window::IATTR_HINT_DRAWABLE, drawable );
}


AsyncFetcher::AsyncFetcher()
    : co::base::Thread()
    , _wnd( 0 )
    , _objectManager( 0 )
    , _sharedContextWindow( 0 )
{
    _tmpTexture = new GLbyte[ 64*64*4 ];
}


AsyncFetcher::~AsyncFetcher()
{
    if( _wnd && _sharedContextWindow )
        deleteScharedContextWindow( _wnd, &_sharedContextWindow, &_objectManager );

    delete []_tmpTexture;
}


const GLEWContext* AsyncFetcher::glewGetContext() const
{
    return _sharedContextWindow->glewGetContext();
}


/**
 *  Funciton for creating and holding of shared context.
 *  Generation and uploading of new textures over some 
 *  period of sleep time.
 */
void AsyncFetcher::run()
{
    EQASSERT( !_sharedContextWindow );
    _sharedContextWindow = initSharedContextWindow( _wnd );
    _outQueue.push( TextureId()); // unlock pipe thread
    if( !_sharedContextWindow )
        return;

    _objectManager = new ObjectManager( glewGetContext( ));
    EQWARN << "async fetcher is initialized: " << _wnd << std::endl;

    int i = 0;
    bool exitLoop = false;
    sleep( 1 ); // imitate loading of the first texture
    while( !exitLoop )
    {
        // generate new texture
        eq::util::Texture* tx = _objectManager->newEqTexture( ++i, GL_TEXTURE_2D );
        tx->init( GL_RGBA8, 64, 64 );

        int j = 0;
        for( int y = 0; y < 64; y++ )
        {
            for( int x = 0; x < 64; x++ )
            {
                const GLbyte rnd = 127 + rand() % 127;
                const GLbyte val = (x / 8) % 2 == (y / 8) % 2 ? rnd : 0;
                _tmpTexture[ j++ ] = val;
                _tmpTexture[ j++ ] = val;
                _tmpTexture[ j++ ] = val;
                _tmpTexture[ j++ ] = val;
            }
        }
        tx->upload( 64, 64, _tmpTexture );
        EQ_GL_CALL( glFinish( ));

        _outQueue.push( TextureId( tx->getName( ), i )); // add new texture to the pool

        sleep( rand() % 5 ); // imitate hard work of loading something else

        // clean unused textures
        int keyToDelete = 0;
        while( _inQueue.tryPop( keyToDelete ))
        {
            if( keyToDelete )
            {
                EQWARN << "Deleting eq texture " << keyToDelete << std::endl;
                _objectManager->deleteEqTexture( keyToDelete );
            }else
                exitLoop = true;
        }
    }
    deleteScharedContextWindow( _wnd, &_sharedContextWindow, &_objectManager );
}

} //namespace eqAsync
