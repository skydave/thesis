/*---------------------------------------------------------------------

The BoundingBox class is a very simple utility class for working with
axis aligned bounding boxes

----------------------------------------------------------------------*/
#pragma once


#include "Math.h"




namespace math
{
	///
	/// \brief The BoundingBox class is a very simple utility class for working
	/// with axis aligned bounding boxes
	///
	struct BoundingBox
	{
		BoundingBox();                                                          ///< constructor
		BoundingBox( math::Vec3f _minPoint, math::Vec3f _maxPoint );            ///< constructor

		math::Vec3f                                        size( void ) const;  ///< returns a vector which represents the dimension in each axis
		void                           extend( const math::Vec3f &nextPoint );  ///< adobts the bounding borders if neccessary so that the given point lies within the bounding box
		math::Vec3f                                   getCenter( void ) const;  ///< returns the geometrical center of the box
		bool                       encloses( const math::Vec3f &point ) const;  ///< this utility function checks wether the given point is within the volume descripted by the bounding box
		bool encloses( const math::Vec3f &min, const math::Vec3f &max ) const;  ///< this utility function checks wether the given box is within the volume descripted by the bounding box

		math::Vec3f                                                  minPoint;  ///< the position of all lowends for each axis
		math::Vec3f                                                  maxPoint;  ///< the position of all highends for each axis
	};
}