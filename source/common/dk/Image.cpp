/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "Image.h"

#include <stdio.h>
#include <stdlib.h>
#include <vector>


namespace dk
{
	//
	//
	//
	Image::Image()
	{
		m_buffer = 0;

		setResolution( 640, 480 );
	}

	//
	//
	//
	Image::~Image()
	{
		if( m_buffer )
			free( m_buffer );
	}

	//
	//
	//
	void Image::setResolution( unsigned int width, unsigned int height )
	{
		m_width = width;
		m_height = height;

		m_buffer = (unsigned long*) realloc( m_buffer, sizeof(unsigned long) * m_width * m_height );

	}

	//
	//
	//
	unsigned int Image::getWidth( void )
	{
		return m_width;
	}

	//
	//
	//
	unsigned int Image::getHeight( void )
	{
		return m_height;
	}

	//
	//
	//
	void Image::setPixel( const unsigned int index, const unsigned long value )
	{
		m_buffer[ index ] = value;
	}

	void Image::setPixel( const unsigned int x, const unsigned int y, const unsigned long value )
	{
		m_buffer[ y*m_width + x ] = value;
	}

	unsigned long Image::getPixel( const unsigned int x, const unsigned int y )
	{
		return m_buffer[ y*m_width + x ];
	}

	//
	//
	//
	unsigned long *Image::getImageBuffer( void )
	{
		return m_buffer;
	}

	//
	//
	//
	void Image::flip( bool horizontal, bool vertical )
	{
		// neither horizontal nor vertical ?
		if( !horizontal && !vertical )
			// nothing to do
			return;

		// temporary buffer
		std::vector<unsigned long> buffer;

		buffer.resize( m_width*m_height );

		for( unsigned int j = 0; j<m_height; ++j )
		{
			unsigned int y_line = j;

			if( vertical )
				y_line = m_height - j - 1;

			if( horizontal )
			{
				for( unsigned int i = 0; i<m_width; ++i )
					buffer[ y_line*m_width + i ] = m_buffer[ j*m_width + (m_width - i - 1) ];
			}else
				memcpy( &buffer[ y_line*m_width ], &m_buffer[ j*m_width ], sizeof( unsigned long ) * m_width );
		}
		memcpy( &m_buffer[ 0 ], &buffer[ 0 ], sizeof( unsigned long ) * m_width * m_height );


	}
}