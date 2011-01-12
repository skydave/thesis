/*---------------------------------------------------------------------

The SurfaceConstructor class is a utility class which creates a triangle
based surface from a set of particles. The popular marching cube algorithm
is used.

----------------------------------------------------------------------*/
#pragma once

#include <vector>
#include <math/Math.h>
#include <dk/Mesh.h>

struct Particle;


///
///
/// \brief use it to generate a triangle surface from a set of particles after zhou2005
///
///
class SurfaceConstructor
{
//private:
public:
	struct GridPoint
	{
		math::Vec3f       position;
		float             isoValue;
		float                 temp;
		bool                 fixed;  // used for the fast sweeping algorithm
	};
	struct Sample
	{
		GridPoint*     vertices[8];    // order: x-y-z+ x+y-z+ x+y-z- x-y-z- x-y+z+ x+y+z+ x+y+z- x-y+z-
	};

	float            m_sampleWidth;  // width of the samples(marching cube cells) in each dimension
public:

	dk::Mesh *SurfaceConstructor::constructSurface( std::vector<Particle> &particles, float sampleWidth, float averageParticleSpacing );
private:
	void generateTrianglesFromSample( const Sample &sample, float isoLevel, std::vector<math::Vec3f> &vertices, std::vector<int> &indicees  );

	math::Vec3f interpolateGridPoints( const GridPoint *p1, const GridPoint *p2, float isoLevel );

	void sweepUpdateDistances( float &d, float a, float b, float c ); // used for the fast-sweeping-distance computation

};