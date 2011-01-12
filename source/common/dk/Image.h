/*---------------------------------------------------------------------

A simple image class which is used as pixel container.

----------------------------------------------------------------------*/
#pragma once

namespace dk
{
	///
	/// \brief holds a block of pixels in a buffer
	///
	class Image
	{
	public:
		Image();
		~Image();

		void                          setResolution( unsigned int width, unsigned int height );

		unsigned int                                                          getWidth( void );
		unsigned int                                                         getHeight( void );

		void                   setPixel( const unsigned int index, const unsigned long value );
		void setPixel( const unsigned int x, const unsigned int y, const unsigned long value );
		unsigned long                   getPixel( const unsigned int x, const unsigned int y );

		unsigned long                                                  *getImageBuffer( void );

		// some manipulation routines ------------------------
		void                                            flip( bool horizontal, bool vertical );

	private:

		unsigned int                                                                   m_width;
		unsigned int                                                                  m_height;

		unsigned long                                                                *m_buffer;
	};
}