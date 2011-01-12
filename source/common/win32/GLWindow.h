/*---------------------------------------------------------------------

A very simple class which encapuslates some Win32API calls for window
creation and management.

----------------------------------------------------------------------*/
#pragma once
#include <windows.h>
#include <string>
#include <stdio.h>
#include <GL/gl.h>
#include <GL/glu.h>

namespace dk
{
	///
	/// \brief very simple interface to make win32-window handling more easy
	///
	class GLWindow
	{
		HWND                                            mhwnd;
		HINSTANCE                                  mhinstance;
		HGLRC                                            mhRC;
		WNDCLASS                                           wc;

		unsigned int                              pixelformat;
		unsigned long                               dwExstyle;	
		unsigned long                                 dwstyle;

		int                                        bpp, fsbpp;
		int                                            zdepth;

		bool                                       fullscreen;
		std::wstring                                  caption;           ///< caption text of the window
		RECT                                       WindowRect,
											   FullscreenRect;

		DEVMODE	                                      DMsaved;           ///< Saves The Previous Screen Settings (NEW)

	public:
		HDC                                              mhDC;

		GLWindow();
		~GLWindow();


		bool            createGLWindow( std::string  _caption,           // title of the window
												   int _width,           // width of the window
												  int _height,           // height of the window
												  int _startx,           // x offset of the window
												  int _starty,           // y offset of the window
													 int _bpp,           // bit depth of the window
											 bool _fullscreen,           // fullscreen ?
									  WNDPROC WindowProcedure,           // window handling procedure
									   HINSTANCE _hInstance );

		void                                     show( void );           ///< shows the window

		int                                  getWidth( void );
		int                                 getHeight( void );

		bool                         toggleFullscreen( void );
		bool setFullScreen( int width, int height, int _bpp );
		void         restoreScreen( bool show_cursor = true ); 

		void       setWindowTitle( const std::string &title );


	private:


		bool                       _SetPixelFormat(HWND hWnd);           ///< set the pixelformat of this window
	};
}