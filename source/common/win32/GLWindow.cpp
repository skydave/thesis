/*---------------------------------------------------------------------

A very simple class which encapuslates some Win32API calls for window
creation and management.

----------------------------------------------------------------------*/
#include "GLWindow.h"

// add the opengl library
#pragma message("     _Adding library: opengl32.lib" ) 
#pragma comment( lib, "opengl32.lib" )


namespace dk
{

	GLWindow::GLWindow()
	{
		// intitialisierung
		mhwnd       =  NULL;
		mhDC        =  NULL;
		mhRC        =  NULL;
		bpp         =    32;
		fsbpp       =    32;
		fullscreen  = false;
		zdepth      =    32;
		pixelformat =     0;

		WindowRect.left       =   0;
		WindowRect.top        =   0;
		WindowRect.right      = 640;
		WindowRect.bottom     = 480;
		
		FullscreenRect.left   =   0;
		FullscreenRect.top    =   0;
		FullscreenRect.right  = 640;
		FullscreenRect.bottom = 480;

		// Save The Current Display State
		EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &this->DMsaved); 

	}
	GLWindow::~GLWindow()
	{
		if( this->mhwnd )
		{
			if( this->fullscreen == true )
				this->restoreScreen();

			if( mhRC )
			{
				wglMakeCurrent( NULL , NULL );
				wglDeleteContext( mhRC );
				mhRC  =  NULL;
			}
			
			if( mhDC )
			{
				ReleaseDC( mhwnd , mhDC );
				mhDC  =  NULL;
			}

			DestroyWindow( this->mhwnd );

			UnregisterClass( (LPCWSTR) "mf window class" , mhinstance );
		}
	}






	bool GLWindow::createGLWindow( std::string  _caption, int _width, int _height,
								  int _startx, int _starty, int _bpp, bool _fullscreen,
									WNDPROC WindowProcedure, HINSTANCE _hInstance )
	{
		WindowRect.left		= _startx;
		WindowRect.top		= _starty;
		WindowRect.right	= _width  + _startx;
		WindowRect.bottom	= _height + _starty;

		bpp                 = _bpp;

		// convert std::wstring to std::string
		caption             = std::wstring( _caption.begin(), _caption.end() );
	;


		mhinstance          = _hInstance;

		wc.style			= CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
		wc.lpfnWndProc		= (WNDPROC) WindowProcedure;
		wc.cbClsExtra		= 0;
		wc.cbWndExtra		= 0;
		wc.hInstance		= mhinstance;
		wc.hIcon			= NULL;
		wc.hCursor			= LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground	= NULL;
		wc.lpszMenuName		= NULL;
		wc.lpszClassName	= (LPCWSTR)"mf window class";


		if( !RegisterClass( &wc ) )
			return false;

		dwExstyle	        = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;
		dwstyle		        = WS_OVERLAPPEDWINDOW;

		if ( _fullscreen == true )
		{
			dwstyle = WS_POPUP;
		}
		else
		{
			dwstyle = WS_OVERLAPPEDWINDOW;
		}

		AdjustWindowRectEx( &WindowRect , dwstyle , FALSE , dwExstyle );


		mhwnd = CreateWindowEx(	dwExstyle, (LPCWSTR)"mf window class" , caption.c_str() ,
								dwstyle | WS_CLIPSIBLINGS | WS_CLIPCHILDREN,
								WindowRect.left, WindowRect.top, 			 
								WindowRect.right  - WindowRect.left,	
								WindowRect.bottom - WindowRect.top, 
								NULL, NULL, mhinstance, NULL);
		
		if( mhwnd == NULL) return false;

		mhDC = GetDC( mhwnd );

		if ( _SetPixelFormat( mhwnd ) )
		{
			mhRC = wglCreateContext( mhDC );
			wglMakeCurrent( mhDC , mhRC );
		} else
		{
			//DestroyWindow( mhwnd );
		}

		if( _fullscreen )
		{
			setFullScreen( _width , _height , bpp );
		}

		return true;
	}



	bool GLWindow::_SetPixelFormat( HWND hWnd )
	{
		PIXELFORMATDESCRIPTOR pfd=				// pfd Tells Windows How We Want Things To Be
		{
			sizeof(PIXELFORMATDESCRIPTOR),		// Size Of This Pixel Format Descriptor
			1,									// Version Number
			PFD_DRAW_TO_WINDOW |				// Format Must Support Window
			PFD_SUPPORT_OPENGL |				// Format Must Support OpenGL
			PFD_DOUBLEBUFFER,					// Must Support Double Buffering
			PFD_TYPE_RGBA,						// Request An RGBA Format
			bpp,								// Select Our Color Depth
			0, 0, 0, 0, 0, 0,					// Color Bits Ignored
			0,									// No Alpha Buffer
			0,									// Shift Bit Ignored
			0,									// No Accumulation Buffer
			0, 0, 0, 0,							// Accumulation Bits Ignored
			zdepth,								// 16Bit Z-Buffer (Depth Buffer)  
			0,									// No Stencil Buffer
			0,									// No Auxiliary Buffer
			PFD_MAIN_PLANE,						// Main Drawing Layer
			0,									// Reserved
			0, 0, 0								// Layer Masks Ignored
		};


		mhDC=GetDC(hWnd);

		pixelformat = ChoosePixelFormat( mhDC , &pfd );

		SetPixelFormat( mhDC , pixelformat , &pfd );

		return true;
	}



	bool GLWindow::setFullScreen( int width, int height, int _bpp )
	{
		// already fullscreen? -> go back
		if( fullscreen )
			return true;

		dwExstyle=WS_EX_APPWINDOW;
		dwstyle = WS_POPUP | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;

		// Save The Current Display State
		EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &this->DMsaved);

		DEVMODE dmScreenSettings;

		memset(&dmScreenSettings,0,sizeof(dmScreenSettings));	// Makes Sure Memory's Cleared 

		dmScreenSettings.dmSize             = sizeof(dmScreenSettings);
		dmScreenSettings.dmPelsWidth        = width; 
		dmScreenSettings.dmPelsHeight       = height;
		dmScreenSettings.dmBitsPerPel       = bpp;
		dmScreenSettings.dmFields           = DM_BITSPERPEL|DM_PELSWIDTH|DM_PELSHEIGHT|DM_DISPLAYFREQUENCY;
		dmScreenSettings.dmDisplayFrequency = 85;  // we assume that there are at least 85 hz

		if ( ChangeDisplaySettings( &dmScreenSettings , CDS_FULLSCREEN ) != DISP_CHANGE_SUCCESSFUL )
		{
			restoreScreen();
			// somethings gone wrong
			return false;
		}

		fullscreen = true;

		// adjust the window
		FullscreenRect.left   =      0;
		FullscreenRect.top    =      0;
		FullscreenRect.right  =  width;
		FullscreenRect.bottom = height;

		SetWindowLong( mhwnd , GWL_STYLE   , dwstyle   );
		SetWindowLong( mhwnd , GWL_EXSTYLE , dwExstyle );

		SetWindowPos(  mhwnd , HWND_TOP , 0 , 0 , width , height , SWP_SHOWWINDOW );

		AdjustWindowRectEx( &FullscreenRect , dwstyle , FALSE , dwExstyle );

		SetForegroundWindow( mhwnd );

		fsbpp = _bpp;

		// hide the cursor
		//ShowCursor( FALSE );

		return true;
	}

	void GLWindow::restoreScreen( bool show_cursor )
	{	
		dwExstyle	= WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;
		dwstyle		= WS_OVERLAPPEDWINDOW | WS_CLIPSIBLINGS | WS_CLIPCHILDREN;

		AdjustWindowRectEx(&WindowRect, dwstyle, FALSE, dwExstyle);

		SetWindowLong( mhwnd , GWL_STYLE   , dwstyle   );
		SetWindowLong( mhwnd , GWL_EXSTYLE , dwExstyle );

		SetWindowPos( mhwnd, HWND_TOP, WindowRect.left, WindowRect.top, WindowRect.right - WindowRect.left, WindowRect.bottom - WindowRect.top, SWP_SHOWWINDOW );

		if( show_cursor )
			ShowCursor( TRUE );

		if( fullscreen )
		{
			// If The Shortcut Doesn't Work
			if (!ChangeDisplaySettings(NULL,CDS_TEST))
			{
				// Do It Anyway (To Get The Values Out Of The Registry)
				ChangeDisplaySettings(NULL,CDS_RESET);
				// Change Resolution To The Saved Settings
				ChangeDisplaySettings(&this->DMsaved,CDS_RESET);
			}else
			{
				ChangeDisplaySettings(NULL,CDS_RESET);
			}

			fullscreen = false;
		}
	}

	bool GLWindow::toggleFullscreen( void )
	{
		if( this->fullscreen == true )
			this->restoreScreen();
		else
			this->setFullScreen( this->FullscreenRect.right , this->FullscreenRect.bottom , this->bpp );
		return true;
	}

	void GLWindow::setWindowTitle( const std::string &title )
	{
		SetWindowTextA( mhwnd, title.c_str() );
	}


	void GLWindow::show( void )
	{
		ShowWindow( mhwnd , SW_SHOW );
		UpdateWindow( mhwnd );
		SetForegroundWindow( mhwnd );
		SetFocus( mhwnd );
	}



	int GLWindow::getWidth( void )
	{
		RECT windowRect;
		GetClientRect( mhwnd, &windowRect );

		//To access members
		return windowRect.right - windowRect.left;
	}

	int GLWindow::getHeight( void )
	{
		RECT windowRect;
		GetClientRect( mhwnd, &windowRect );

		//To access members
		return windowRect.bottom - windowRect.top;
	}


} // namespace dk
