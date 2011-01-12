/*---------------------------------------------------------------------

This header declares some basic 2d static solids which can be put into
a 2dfluidsimulation.

----------------------------------------------------------------------*/
#pragma once
#include "FluidSimulator2d.h"









///
/// \brief Sphere defines through center coordinates and a radius
///
class Sphere2d : public FluidSimulator2d::StaticSolid
{
public:
	///
	/// constructor
	///
	Sphere2d( float centerX, float centerY, float radius )
	{
		zRotation = 0.0f;
		center = math::Vec2f( centerX, centerY );
		radiusSquared = radius*radius;
	}

	///
	/// derived from the StaticSolid interface
	///
	/// this function is used to indirectly voxelize the obstacle
	///
	virtual bool occupies( const math::Vec2f &cellCenter )
	{
		// the sphere occupies the cell when the cellCenter lies within the radius of the sphere
		if( (cellCenter - center).getSquaredLength() < radiusSquared )
			return true;
		return false;
	}

private:
	math::Vec2f                           center; // center of the sphere in world coordinates
	float                          radiusSquared; // radius
};


///
/// 
/// \brief Axis aligned cube defined through a center, a width and a height
///
class Cube2d : public FluidSimulator2d::StaticSolid
{
public:
	///
	/// constructor
	///
	Cube2d( float centerX, float centerY, float width, float height )
	{
		zRotation = 0.0f;
		boundinbBox = math::BoundingBox2d( math::Vec2f( centerX - width/2.0f, centerY - height/2.0f ), math::Vec2f( centerX + width/2.0f, centerY + height/2.0f ) );
	}

	///
	/// derived from the StaticSolid interface
	///
	/// this function is used to indirectly voxelize the obstacle
	///
	virtual bool occupies( const math::Vec2f &cellCenter )
	{
		// the cube occupies the cell when the cellCenter lies within the boundingbox of the cube
		if( boundinbBox.encloses( cellCenter ) )
			return true;
		return false;
	}

	math::Vec2f getMinPoint()
	{
		return boundinbBox.minPoint;
	}

	math::Vec2f getMaxPoint()
	{
		return boundinbBox.maxPoint;
	}

	math::Vec2f getDimensions()
	{
		return boundinbBox.size();
	}
private:
	math::BoundingBox2d                         boundinbBox;
};

/// 
/// \brief a map of voxels which specify a solid domain
///
class VoxelMap2d : public FluidSimulator2d::StaticSolid
{
public:
	///
	/// constructor
	///
	VoxelMap2d( size_t resX, size_t resY, float cellSize = 0.1f )
	{
		setup( resX, resY, cellSize );
	}

	///
	/// (re)initializes the voxelmap
	///
	void setup( size_t resX, size_t resY, float _cellSize )
	{
		resolutionX      = resX;
		resolutionY      = resY;
		cellSize    = _cellSize;

		size.x = resolutionX*cellSize;
		size.y = resolutionY*cellSize;

		voxelData.resize( resolutionX*resolutionY, 0 );
	}

	///
	///
	///
	void setCellSize( float _cellSize )
	{
		cellSize    = _cellSize;

		size.x = resolutionX*cellSize;
		size.y = resolutionY*cellSize;
	}

	///
	///
	///
	float getCellSize()
	{
		return cellSize;
	}

	///
	///
	///
	size_t getResolutionX()
	{
		return resolutionX;
	}

	///
	///
	///
	size_t getResolutionY()
	{
		return resolutionY;
	}


	///
	/// derived from the StaticSolid interface
	///
	/// this function is used to indirectly voxelize the obstacle
	///
	virtual bool occupies( const math::Vec2f &cellCenter )
	{
		// map the cellCenter into discrete voxelindex-space
		size_t i = (size_t)(cellCenter.x / cellSize);
		size_t j = (size_t)(cellCenter.y / cellSize);

		// now check whether the specified voxel is solid or empty
		return (voxelData[ j*resolutionX + i ] != 0);
	}

	///
	///
	///
	void setVoxelState( size_t i, size_t j, bool state )
	{
		if( state )
			voxelData[ j*resolutionX + i ] = 1;
		else
			voxelData[ j*resolutionX + i ] = 0;
	}

	///
	///
	///
	bool getVoxelState( size_t i, size_t j )
	{
		return (voxelData[ j*resolutionX + i ] != 0);
	}

private:
	float                                          cellSize; // uniform cellsize of the voxels
	math::Vec2f                                        size; // size of the voxelmap in 3dspace (depends on resolution and cellsize)
	size_t                                      resolutionX; // number of cells in x
	size_t                                      resolutionY; // number of cells in y
	std::vector<char>                             voxelData;
};