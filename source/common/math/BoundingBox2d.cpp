/*---------------------------------------------------------------------

The BoundingBox2d class is a very simple utility class for working with
axis aligned bounding boxes

----------------------------------------------------------------------*/
#include "BoundingBox2d.h"




namespace math
{
	//
	// constructor
	//
	BoundingBox2d::BoundingBox2d()
	{
		minPoint = math::Vec2f( 99999999999.0f, 99999999999.0f );
		maxPoint = math::Vec2f( -99999999999.0f, -99999999999.0f );
	}

	//
	// constructor
	//
	BoundingBox2d::BoundingBox2d( math::Vec2f _minPoint, math::Vec2f _maxPoint )
	{
		minPoint = _minPoint;
		maxPoint = _maxPoint;
	}

	//
	// returns a vector which represents the dimension in each axis
	//
	math::Vec2f BoundingBox2d::size( void ) const
	{
		return math::Vec2f( maxPoint - minPoint );
	}

	//
	// adobts the bounding borders if neccessary so that the given point lies within
	// the bounding box
	//
	void BoundingBox2d::extend( const math::Vec2f &nextPoint )
	{
		if( nextPoint.x > maxPoint.x )
			maxPoint.x = nextPoint.x;
		if( nextPoint.y > maxPoint.y )
			maxPoint.y = nextPoint.y;

		if( nextPoint.x < minPoint.x )
			minPoint.x = nextPoint.x;
		if( nextPoint.y < minPoint.y )
			minPoint.y = nextPoint.y;
	}


	//
	// returns the geometrical center of the box
	//
	math::Vec2f BoundingBox2d::getCenter( void ) const
	{
		return (minPoint + maxPoint)*0.5f;
	}

	//
	// this utility function checks wether the given point
	// is within the volume descripted by the bounding box
	//
	bool BoundingBox2d::encloses( const math::Vec2f &point ) const
	{
		// check each dimension
		if( (point.x > minPoint.x)&&(point.x < maxPoint.x)&&
			(point.y > minPoint.y)&&(point.y < maxPoint.y))
			return true;
		else
			return false;
	}

	//
	// this utility function checks wether the given box
	// is within the volume descripted by the bounding box
	//
	bool BoundingBox2d::encloses( const math::Vec2f &min, const math::Vec2f &max ) const
	{
		// check each dimension for each point
		if( (min.x >= minPoint.x)&&(min.y >= minPoint.y)&&
			(max.x <= maxPoint.x)&&(max.y <= maxPoint.y))
			return true;
		else
			return false;
	}
}