/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "VoxelMapReader.h"

#include <string>
#include <fstream>
#include <iostream>
#include "util/PathInfo.h"
#include "util/ImageLoader.h"
#include "dk/ObjIO.h"
#include "dk/BinObjIO.h"





//
//
//
VoxelMap2d *readVoxelMap2d( std::string filename )
{
	dk::Image *img = dk::util::ImageLoader::loadBMP( filename );

	if( !img )
		return 0;

	VoxelMap2d *vm = new VoxelMap2d( img->getWidth(), img->getHeight() );

	for( size_t j=0; j<img->getHeight(); ++j )
		for( size_t i=0; i<img->getWidth(); ++i )
		{
			unsigned long pix = img->getPixel( (unsigned int)i, (unsigned int)j);
			if( (math::getRed(pix) + math::getGreen(pix) + math::getBlue(pix)) / 3 )
				vm->setVoxelState( i, j, true );
			else
				vm->setVoxelState( i, j, false );

		}



	return vm;
}


//
// reads a *.binvox file
//
VoxelMap3d *readVoxelMap3d( std::string filename )
{
	std::ifstream *input = new std::ifstream( filename.c_str(), std::ios::in | std::ios::binary);

	if( !(*input) )
		std::cout << "error reading binvox file: file "<< filename <<" not found";

	// read header --------------------------------------------------------------------------------------
	std::string line;
	*input >> line;

	// check the first line to be #binvox
	if (line.compare("#binvox") != 0)
	{
		// no? probably not a binvox file...
		std::cout << "error reading binvox file: first line reads [" << line << "] instead of [#binvox]" << std::endl;
		delete input;
		return 0;
	}

	// read version info
	int version;
	*input >> version;
	std::cout << "reading binvox version " << version << std::endl;

	// read other header content

	int depth, height, width;
	depth = -1;
	float scale = 1.0f;
	math::Vec3f translate;
	int headerDone = 0;

	while(input->good() && !headerDone)
	{
		*input >> line;
		// keyword data -> if the keyword data is read, then the reading of the header is finished
		if (line.compare("data") == 0)
			headerDone = 1;
		else
		// keyword dim -> read the resolution of the voxel grid
		if (line.compare("dim") == 0)
		{
			*input >> depth >> height >> width;
		}else
		// keyword scale
		if (line.compare("scale") == 0)
		{
			*input >> scale;
		}else
		// keyword translate
		if (line.compare("translate") == 0)
		{
			*input >> translate.x >> translate.y >> translate.z;
		}else
		// unknown keyword
		{
			std::cout << " unrecognized keyword [" << line << "], skipping" << std::endl;
			char c;
			do
			{// skip until end of line
				c = input->get();
			}while(input->good() && (c != '\n'));
		}
	}

	if (!headerDone)
	{
		std::cout << " error reading header" << std::endl;
		return 0;
	}

	if (depth == -1)
	{
		std::cout << " missing dimensions in header" << std::endl;
		return 0;
	}


	VoxelMap3d *vm = new VoxelMap3d( width, height, depth );

	vm->vox_scale = scale;
	vm->vox_translate = translate;

	int size = width * height * depth;

	// read voxel data -----------------------------------------------
	unsigned char value;
	bool     voxelState;
	unsigned char count;
	int index = 0;
	int end_index = 0;
	int nr_voxels = 0;

	input->unsetf(std::ios::skipws); // need to read every byte now (!)

	*input >> value; // read the linefeed char 

	while((end_index < size) && input->good())
	{
		*input >> value >> count;

		if( value )
			voxelState = true;
		else
			voxelState = false;

		if (input->good())
		{
			end_index = index + count;
			if (end_index > size)
				return 0;
			for(int s=index; s < end_index; ++s)
			{
				// compute i,j,k coordinates from running index
				div_t result = div( s, width * height );
				size_t k = result.quot;
				result = div( result.rem, width );
				size_t j = result.quot;
				size_t i = result.rem;

				//vm->setVoxelState( s, voxelState );
				vm->setVoxelState( k, i, j, voxelState );
			}
			if (value)
				nr_voxels += count;
			index = end_index;
		}// if file still ok
	} // while
	input->close();
	std::cout << " read " << nr_voxels << " voxels" << std::endl;


	// voxeldata could be read successfully - now we try to look for a triangle representation
	// of the same filename...
	dk::Mesh *associatedMesh = 0;
	if( dk::util::fileExists( dk::util::PathInfo::getPathAs( filename, ".obj" ) ) )
	{
		// read the object
		associatedMesh = dk::io::importFromObjFile( dk::util::PathInfo::getPathAs( filename, ".obj" ) );
	}else
	if( dk::util::fileExists( dk::util::PathInfo::getPathAs( filename, ".binobj" ) ) )
	{
		// read the binary object
		associatedMesh = dk::io::importFromBinObjFile( dk::util::PathInfo::getPathAs( filename, ".binobj" ) );
	}

	if( associatedMesh )
	{
		// temp mountain 1 -> zvalues are wrong
		for( std::vector<dk::Mesh::Vertex *>::iterator it = associatedMesh->vertices.begin(); it != associatedMesh->vertices.end(); ++it )
			(*it)->position.z += 1.0f;


		associatedMesh->computeVertexIndicees();
		associatedMesh->computeNormals();
		vm->setAssociatedMesh( associatedMesh );
	}



	return vm;
}
