/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once
#include "StaticSolids2d.h"
#include "StaticSolids3d.h"






/// reads a 2-dimensional voxelmap which is a simple bitmap or image file
VoxelMap2d *readVoxelMap2d( std::string filename );


/// reads a *.binvox file
VoxelMap3d *readVoxelMap3d( std::string filename );