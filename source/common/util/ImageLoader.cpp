/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "ImageLoader.h"
#include <windows.h>
#include <fstream>
#include "math/Math.h"

using namespace std;

namespace dk
{

	namespace util
	{
		//
		//
		//
		Image *ImageLoader::loadImage( const std::string &filename )
		{
			return 0;
		}

		//
		//
		//
		Image *ImageLoader::loadBMP( const std::string &filename )
		{
			BITMAPFILEHEADER bmfh;
			BITMAPINFOHEADER bmih;

			// Open file.
			ifstream bmpfile( filename.c_str() , ios::in | ios::binary);
			if (! bmpfile.is_open())
				return 0;		// Error opening file

			// Load bitmap fileheader & infoheader
			bmpfile.read ((char*)&bmfh,sizeof (BITMAPFILEHEADER));
			bmpfile.read ((char*)&bmih,sizeof (BITMAPINFOHEADER));

			// Check filetype signature
			if (bmfh.bfType!='MB')
				return 0;		// File is not BMP

			// Assign some short variables:
			int BPP=bmih.biBitCount;
			int Width=bmih.biWidth;
			int Height= (bmih.biHeight>0) ? bmih.biHeight : -bmih.biHeight; // absoulte value
			int BytesPerRow = Width * BPP / 8;
			BytesPerRow += (4-BytesPerRow%4) % 4;	// int alignment

			BITMAPINFO *pbmi;
			RGBQUAD *Palette;

			// If BPP aren't 24, load Palette:
			if (BPP==24)
				pbmi=(BITMAPINFO*)new char [sizeof(BITMAPINFO)];
			else
			{
				pbmi=(BITMAPINFO*) new char[sizeof(BITMAPINFOHEADER)+(1<<BPP)*sizeof(RGBQUAD)];
				Palette=(RGBQUAD*)((char*)pbmi+sizeof(BITMAPINFOHEADER));
				bmpfile.read ((char*)Palette,sizeof (RGBQUAD) * (1<<BPP));
			}
			pbmi->bmiHeader=bmih;

			// Load Raster
			bmpfile.seekg (bmfh.bfOffBits,ios::beg);

			Image *image = new Image();
			image->setResolution( Width, Height );

			if( BPP==24 )
			{
				unsigned char *scanline = (unsigned char *)malloc( BytesPerRow*sizeof(unsigned char) );
				//for (int n=Height-1;n>=0;n--)
				for (int n=0;n<Height;++n)
				{
					bmpfile.read( (char*)scanline, BytesPerRow );

					for( int i=0; i<Width; ++i )
					{
						int pixel = i*3;

						image->setPixel( i, n, math::setColor( scanline[pixel+0], scanline[pixel+1], scanline[pixel+2], 255 ) );
					}
				}
			}
			/*
			// (if height is positive the bmp is bottom-up, read it reversed)
			if (bmih.biHeight>0)
				for (int n=Height-1;n>=0;n--)
					bmpfile.read (Raster+BytesPerRow*n,BytesPerRow);
			else
				bmpfile.read (Raster,BytesPerRow*Height);
			*/

			// so, we always have a up-bottom raster (that is negative height for windows):
			pbmi->bmiHeader.biHeight=-Height;

			bmpfile.close();

			return image;
		}
		


		//
		//
		//
		Image *ImageLoader::loadBMP( char *mem, size_t size )
		{
			return 0;
		}
	}
}