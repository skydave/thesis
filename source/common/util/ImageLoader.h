/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once
#include <string>
#include "dk/Image.h"

namespace dk
{

	namespace util
	{
		///
		/// \brief offers a bunch of static methods for loading image files from disk
		///
		class ImageLoader
		{
		public:
			static Image *loadImage( const std::string &filename ); ///< load name-specified imagefile and autodetec the type
			static Image   *loadBMP( const std::string &filename ); ///< load name-specified imagefile and assume its a *.bmp
			static Image        *loadBMP( char *mem, size_t size ); ///< load name-specified imagefile from memory and assume its a *.bmp
		};

	}
}