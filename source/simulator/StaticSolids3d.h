/*---------------------------------------------------------------------

This header declares some basic 2d static solids which can be put into
a 2dfluidsimulation.

----------------------------------------------------------------------*/
#pragma once
#include "FluidSimulator3d.h"
#include "dk/Mesh.h"








///
/// \brief Sphere defines through center coordinates and a radius
///
class Sphere3d : public FluidSimulator3d::StaticSolid
{
public:
	///
	/// constructor
	///
	Sphere3d( float centerX, float centerY, float centerZ, float radius )
	{
		center = math::Vec3f( centerX, centerY, centerZ );
		radiusSquared = radius*radius;
	}

	///
	/// derived from the StaticSolid interface
	///
	/// this function is used to indirectly voxelize the obstacle
	///
	virtual bool occupies( const math::Vec3f &cellCenter )
	{
		// the sphere occupies the cell when the cellCenter lies within the radius of the sphere
		if( (cellCenter - center).getSquaredLength() < radiusSquared )
			return true;
		return false;
	}

private:
	math::Vec3f                           center; // center of the sphere in world coordinates
	float                          radiusSquared; // radius
};


///
/// 
/// \brief Axis aligned cube defined through a center, a width and a height
///
class Cube3d : public FluidSimulator3d::StaticSolid
{
public:
	///
	/// constructor
	///
	Cube3d( float centerX, float centerY, float centerZ, float width, float height, float depth )
	{
		boundinbBox = math::BoundingBox( math::Vec3f( centerX - width/2.0f, centerY - height/2.0f, centerZ - depth/2.0f ), math::Vec3f( centerX + width/2.0f, centerY + height/2.0f, centerZ + depth/2.0f ) );
	}

	///
	/// derived from the StaticSolid interface
	///
	/// this function is used to indirectly voxelize the obstacle
	///
	virtual bool occupies( const math::Vec3f &cellCenter )
	{
		// the cube occupies the cell when the cellCenter lies within the boundingbox of the cube
		if( boundinbBox.encloses( cellCenter ) )
			return true;
		return false;
	}

	math::Vec3f getMinPoint()
	{
		return boundinbBox.minPoint;
	}

	math::Vec3f getMaxPoint()
	{
		return boundinbBox.maxPoint;
	}

	math::Vec3f getDimensions()
	{
		return boundinbBox.size();
	}

private:
	math::BoundingBox                         boundinbBox;
};



/// 
/// \brief a map of voxels which specify a solid domain within |R³
///
class VoxelMap3d : public FluidSimulator3d::StaticSolid
{
public:
	///
	/// constructor
	///
	VoxelMap3d( size_t resX, size_t resY, size_t resZ, float cellSize = 0.1f ) : FluidSimulator3d::StaticSolid()
	{
		setup( resX, resY, resZ, cellSize );
	}

	///
	/// (re)initializes the voxelmap
	///
	void setup( size_t resX, size_t resY, size_t resZ, float _cellSize )
	{
		mesh = 0;
		resolutionX      = resX;
		resolutionY      = resY;
		resolutionZ      = resZ;
		cellSize    = _cellSize;

		size.x = resolutionX*cellSize;
		size.y = resolutionY*cellSize;
		size.z = resolutionZ*cellSize;

		voxelData.resize( resolutionX*resolutionY*resolutionZ, 0 );
		voxelData2.resize( resolutionX*resolutionY*resolutionZ, 0 );
	}

	///
	///
	///
	void setCellSize( float _cellSize )
	{
		cellSize    = _cellSize;

		size.x = resolutionX*cellSize;
		size.y = resolutionY*cellSize;
		size.z = resolutionZ*cellSize;
	}

	//
	//
	//
	float getCellSize()
	{
		return cellSize;
	}

	//
	//
	//
	size_t getResolutionX()
	{
		return resolutionX;
	}

	//
	//
	//
	size_t getResolutionY()
	{
		return resolutionY;
	}

	//
	//
	//
	size_t getResolutionZ()
	{
		return resolutionZ;
	}



	struct Check
	{
		size_t           i,j,k;
		math::Vec3f cellCenter;
		math::Vec3f      color;
	};
	std::vector<Check> checks;

	void check( size_t i, size_t j, size_t k, const math::Vec3f &cellCenter )
	{
		checks.push_back( Check() );
		checks.back().i = i;
		checks.back().j = j;
		checks.back().k = k;
		checks.back().cellCenter = cellCenter;


		math::Vec3f localCoord = cellCenter; // position to query in local space
		//localCoord = math::Vec3f( (i + 0.5f)/resolutionX, (j + 0.5f)/resolutionY, (k + 0.5f)/resolutionZ );

		// now convert local space into normalized unit space
		localCoord -= vox_translate;
		localCoord /= vox_scale;

		size_t i2 = int(localCoord.x*resolutionX - 0.5f);
		size_t j2 = int(localCoord.y*resolutionY - 0.5f);
		size_t k2 = int(localCoord.z*resolutionZ - 0.5f);

		//checks.back().cellCenter = localCoord + math::Vec3f( 0.001f, 0.001f, 0.001f );

		//checks.back().cellCenter = math::Vec3f( (i2+0.5f)*cellSize, (j2+0.5f)*cellSize, (k2+0.5f)*cellSize );

		checks.back().color = math::Vec3f(  0.2f, 1.0f, 0.43f );

		if( ((i < 0)||(i>=resolutionX))||((j < 0)||(j>=resolutionY))||((k < 0)||(k>=resolutionZ)) )
		{
			checks.back().color = math::Vec3f(  0.0f, 0.0f, 0.0f );
		}else
		{
			if( voxelData[ k2*resolutionX*resolutionY + j2*resolutionX + i2 ] != 0 )
				checks.back().color = math::Vec3f( 0.0f, 0.0f, 1.0f );

		}

	}


	///
	/// derived from the StaticSolid interface
	///
	/// this function is used to indirectly voxelize the obstacle
	///
	virtual bool occupies( const math::Vec3f &cellCenter )
	{
		/*
		math::Vec3f localCoord = cellCenter; // position to query in local space

		// now convert local space into normalized unit space
		localCoord -= vox_translate;
		localCoord /= vox_scale;

		size_t i = int(localCoord.x*resolutionX - 0.5f);
		size_t j = int(localCoord.y*resolutionY - 0.5f);
		size_t k = int(localCoord.z*resolutionZ - 0.5f);



		// map the cellCenter into discrete voxelindex-space
		//size_t i = (size_t)(cellCenter.x / cellSize);
		//size_t j = (size_t)(cellCenter.y / cellSize);
		//size_t k = (size_t)(cellCenter.z / cellSize);

		if( ((i < 0)||(i>=resolutionX))||((j < 0)||(j>=resolutionY))||((k < 0)||(k>=resolutionZ)) )
			// cell coordinate out of range
			return false;

		if( !voxelData2[ k*resolutionX*resolutionY + j*resolutionX + i ] )
		{
			voxelData2[ k*resolutionX*resolutionY + j*resolutionX + i ] = 1;
			printf( "check\n" );
		}

		// now check whether the specified voxel is solid or empty
		return (voxelData[ k*resolutionX*resolutionY + j*resolutionX + i ] != 0);
		*/

		//checks.push_back( Check() );
		//checks.back().cellCenter = cellCenter;

		math::Vec3f localCoord = cellCenter; // position to query in local space

		// now convert local space into normalized unit space
		localCoord -= vox_translate;
		localCoord /= vox_scale;

		size_t i = int(localCoord.x*resolutionX - 0.5f);
		size_t j = int(localCoord.y*resolutionY - 0.5f);
		size_t k = int(localCoord.z*resolutionZ - 0.5f);


		//checks.back().color = math::Vec3f(  0.2f, 1.0f, 0.43f );

		if( ((i < 0)||(i>=resolutionX))||((j < 0)||(j>=resolutionY))||((k < 0)||(k>=resolutionZ)) )
			// index out of range
			return false;

		return ( voxelData[ k*resolutionX*resolutionY + j*resolutionX + i ] != 0 );
	}

	///
	/// returns the center of the voxel in local space
	///
	math::Vec3f getVoxelCenterInLocalSpace( size_t i, size_t j, size_t k )
	{
		// convert i,j,k into normalized/unit space
		math::Vec3f cellCenter_n( (i + 0.5f)/resolutionX, (j + 0.5f)/resolutionY, (k + 0.5f)/resolutionZ );

		//...
		cellCenter_n *= vox_scale;
		cellCenter_n += vox_translate;

		return cellCenter_n;
	}

	///
	/// returns the center of the voxel in local space from onedimensional voxelindex
	///
	math::Vec3f getVoxelCenterInLocalSpace( size_t index )
	{
		div_t result = div( (int)index, int(resolutionX * resolutionY) );
		size_t k = result.quot;
		result = div( result.rem, resolutionX );
		size_t j = result.quot;
		size_t i = result.rem;

		return getVoxelCenterInLocalSpace( i, j, k );
	}

	///
	///
	///
	void setVoxelState( size_t i, size_t j, size_t k, bool state )
	{
		if( state )
			voxelData[ k*resolutionX*resolutionY + j*resolutionX + i ] = 1;
		else
			voxelData[ k*resolutionX*resolutionY + j*resolutionX + i ] = 0;
	}

	///
	/// set voxelmap from one-dimensional array index
	///
	void setVoxelState( size_t i, bool state )
	{
		if( state )
			voxelData[ i ] = 1;
		else
			voxelData[ i ] = 0;
	}

	///
	///
	///
	bool getVoxelState( size_t i, size_t j, size_t k )
	{
		return (voxelData[ k*resolutionX*resolutionY + j*resolutionX + i ] != 0);
	}

	///
	/// returns voxelstate from a one-dimensional index (no range check is done)
	///
	bool getVoxelState( size_t i )
	{
		return (voxelData[i] != 0);
	}

	///
	/// this can be used to associate a triangle representation with this voxelmap which can be used to draw
	/// a visual more appealing representation of the obstacle
	///
	void setAssociatedMesh( dk::Mesh *mesh )
	{
		this->mesh = mesh;
	}

	///
	/// returns a triangle representation of the voxelmap or null if none exists
	///
	dk::Mesh *getAssociatedMesh()
	{
		return mesh;
	}

	// these members have been introduced with the support of the binvox fileformat
	math::Vec3f                               vox_translate;
	float                                         vox_scale;
	std::vector<char>                            voxelData2;

private:
	float                                          cellSize; // uniform cellsize of the voxels
	math::Vec3f                                        size; // size of the voxelmap in 3dspace (depends on resolution and cellsize)
	size_t                                      resolutionX; // number of cells in x
	size_t                                      resolutionY; // number of cells in y
	size_t                                      resolutionZ; // number of cells in z
	std::vector<char>                             voxelData;
	dk::Mesh                                          *mesh;

};