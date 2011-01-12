/*---------------------------------------------------------------------

BinObjIO offers import and export support for obj files in binary format.

*.binobj are binary files which store vertices and triangle lists.
You should prefer the *.obj file which are ASCII based. But for fast
loading and saving the binary format will be better.

----------------------------------------------------------------------*/
#pragma once

#include <iostream>
#include <fstream>
#include "math/Math.h"
#include "Mesh.h"

namespace dk
{
	///
	/// \brief offers functions for reading and writing binary object files from and to disk
	///
	namespace io
	{
		Mesh          *importFromBinObjFile( std::string fileName );  ///< reads and creates a Mesh from the given *.binobj file. returns 0 if it fails
		void exportToBinObjFile( Mesh *mesh, std::string fileName );  ///< creates and writes a *.binobj file from the given mesh
	}
}