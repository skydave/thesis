/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once
#include "Vec3f.h"




namespace math
{
	///
	/// \brief a class which holds an origin and a target and is used by intersection routines
	///
	class Ray3d
	{
	public:
		Ray3d();                                                                               // constructor
		Ray3d( const math::Vec3f &origin, const math::Vec3f &direction, const float &length ); // constructor
		Ray3d( const math::Vec3f &origin, const math::Vec3f &target );                         // constructor

		math::Vec3f                                                               getTarget(); // returns origin+direction*length

		math::Vec3f                                                                    origin; // point in space where the ray originates from
		math::Vec3f                                                                 direction; // normalized direction of the ray
		float                                                                          length; // length of the ray -> some sort of maximum travel distance
	};

}