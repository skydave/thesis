/*---------------------------------------------------------------------

The BoundingBox2d class is a very simple utility class for working with
axis aligned bounding boxes in 2d

----------------------------------------------------------------------*/
#pragma once
#include "Math.h"


namespace math
{
	///
	/// \brief The BoundingBox2d class is a very simple utility class for working
	/// with axis aligned bounding boxes
	///
	struct BoundingBox2d
	{
		BoundingBox2d();                                                        ///< constructor
		BoundingBox2d( math::Vec2f _minPoint, math::Vec2f _maxPoint );          ///< constructor

		math::Vec2f                                        size( void ) const;  ///< returns a vector which represents the dimension in each axis
		void                           extend( const math::Vec2f &nextPoint );  ///< adobts the bounding borders if neccessary so that the given point lies within the bounding box
		math::Vec2f                                   getCenter( void ) const;  ///< returns the geometrical center of the box
		bool                       encloses( const math::Vec2f &point ) const;  ///< this utility function checks wether the given point is within the volume descripted by the bounding box
		bool encloses( const math::Vec2f &min, const math::Vec2f &max ) const;  ///< this utility function checks wether the given box is within the volume descripted by the bounding box

		math::Vec2f                                                  minPoint;  ///< the position of all lowends for each axis
		math::Vec2f                                                  maxPoint;  ///< the position of all highends for each axis
	};
}