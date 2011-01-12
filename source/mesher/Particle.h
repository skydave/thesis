/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once


#include <math/Math.h>




///
/// \brief particles which are used by the mesher
///
struct Particle
{
	Particle( math::Vec3f _position ) : position(_position), tag(false)
	{
	}
	Particle( )
	{
	}
	math::Vec3f position;

	bool tag;
};


