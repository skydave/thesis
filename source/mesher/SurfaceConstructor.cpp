/*---------------------------------------------------------------------

The SurfaceConstructor class is a utility class which creates a triangle
based surface from a set of particles. The popular marching cube algorithm
is used.

----------------------------------------------------------------------*/
#include "SurfaceConstructor.h"

#include "Particle.h"

#include "kdTree.h"

kdTree particleKDTree;


//
// This is the kernelfunction k which is proposed in the paper
//
float kernelFunction( float s )
{
	float temp = 1.0f - s*s;
	float v = temp*temp*temp;
	return (v < 0.0f) ? 0.0f : v;
}



//
// This is the implementation of the sigma function given in the paper
//
unsigned int computeAverages( const math::Vec3f &x, std::vector<Particle> *particles, const float &averageParticleSpacing, math::Vec3f &averagePosition, float &averageRadius )
{
	float R = averageParticleSpacing*2.0f;
	unsigned int neighbourCount;
	static std::vector<Particle *> neighbourhood;

	static std::vector<float>       kernelValues;
	float                                 weight;
	float                            denominator;

	neighbourhood.clear();
	particleKDTree.findNeighboursWithinRange( x, R*R, neighbourhood );
	neighbourCount = (unsigned int)neighbourhood.size();

	// if there is no neighbourhood we return 0
	if( !neighbourCount )
		return 0;

	// compute the kernelvalues
	kernelValues.resize( neighbourCount );
	for( unsigned int i=0; i<neighbourCount; ++i )
		kernelValues[i] = kernelFunction( (x - neighbourhood[i]->position).getLength() / R );


	averagePosition = math::Vec3f( 0.0f, 0.0f, 0.0f );
	averageRadius = 0.0f;
	for( unsigned int i=0; i<neighbourCount; ++i )
	{
		// compute the weighting function wi
		denominator = 0.0f;
		for( unsigned int j=0; j<neighbourCount; ++j )
			denominator += kernelValues[j];
		weight = kernelValues[i] / denominator;

		averagePosition += weight * neighbourhood[i]->position;
		averageRadius += weight * averageParticleSpacing;
	}

	return neighbourCount;
}


void SurfaceConstructor::sweepUpdateDistances( float &d, float a, float b, float c )
{
	float phi = d;
	float h = m_sampleWidth;
	float aDistance = a;
	float bDistance = b;
	float cDistance = c;

	// sort distances in increasing order
	if( fabs( aDistance ) > fabs( bDistance ) ){ float t = aDistance; aDistance = bDistance; bDistance = t; }
	if( fabs( bDistance ) > fabs( cDistance ) ){ float t = bDistance; bDistance = cDistance; cDistance = t; }
	if( fabs( aDistance ) > fabs( bDistance ) ){ float t = aDistance; aDistance = bDistance; bDistance = t; }

	if( fabs(phi) <= fabs( aDistance ) )return;

	phi = aDistance + h;

	if( fabs(phi) >= fabs( bDistance ) )
	{
		phi = 0.5f*( aDistance + bDistance + sqrt(2*h*h - (bDistance-aDistance)*(bDistance-aDistance)));

		if( fabs(phi) >= fabs( cDistance ) )
		{
			phi = (aDistance + bDistance + cDistance)/3.0f + 0.5f*sqrt( 4.0f/9.0f*(aDistance + bDistance + cDistance)*(aDistance + bDistance + cDistance) - 4.0f/3.0f*(aDistance*aDistance+bDistance*bDistance+cDistance*cDistance - h*h) );
		}
	}

	if( phi < d )
		d = phi;
}

//
// TODO: - balance kdtree
//       - cleanup/simplify
//       - check gridOffset computation
//
dk::Mesh *SurfaceConstructor::constructSurface( std::vector<Particle> &particles, float sampleWidth, float averageParticleSpacing )
{
	// Parameters for the marching cube algorithm
	float         isoLevel;

	math::Vec3f         bbMin;
	math::Vec3f    gridOffset;


	// compute average particle distance
	unsigned int particleCount = (unsigned int)particles.size();

	m_sampleWidth = sampleWidth;

	//averageParticleSpacing *= 0.1f;
	//averageParticleSpacing = 0.006f;
	//averageParticleSpacing = 0.0045f;
	//averageParticleSpacing = 0.0043f; // water3
	//averageParticleSpacing = 0.0215443f; // test cube
	//averageParticleSpacing = pow( 1.0f, 1.0f / 3.0f ) / pow( particleCount, 1.0f / 3.0f ); // test cube2
	//averageParticleSpacing = 0.1f; // 2 particles
	//averageParticleSpacing = 0.04f; // 2 particles




	//we use a kdtree for faster nearest neighbour search
	particleKDTree.clear();
	printf( "SurfaceConstructor::building kdtree...\n" );
	for( unsigned int i=0; i<particleCount; ++i )
		particleKDTree.insert( &particles[i] );
		


	isoLevel    = 0.0f;   // isoLevel is zero since we want to get the zerolevel of the signed distance function

	// we have to find the boundaries of the particle cloud
	math::BoundingBox bbox;

	for( std::vector<Particle>::const_iterator it = particles.begin(); it != particles.end(); ++it )
		bbox.extend( (*it).position );

	bbox.maxPoint += averageParticleSpacing + m_sampleWidth*2.0f;
	bbox.minPoint -= averageParticleSpacing + m_sampleWidth*2.0f;



	// ---------------------------------------------------------------------------------------------
	// compute samples with the method proposed in [Zhu2005]
	// ---------------------------------------------------------------------------------------------

	// determine the position of the first subsample in x,y and z
	gridOffset = math::Vec3f( bbox.minPoint.x - fmod( bbox.minPoint.x, m_sampleWidth ), bbox.minPoint.y - fmod( bbox.minPoint.y, m_sampleWidth ), bbox.minPoint.z - fmod( bbox.minPoint.z, m_sampleWidth ) );

	// determine the number of subSamples in each dimension
	long gridResolutionX = long(bbox.size().x / m_sampleWidth + 1);
	long gridResolutionY = long(bbox.size().y / m_sampleWidth + 1);
	long gridResolutionZ = long(bbox.size().z / m_sampleWidth + 1); 

	// create and setup the voxelgrid for the subsamples
	std::vector<GridPoint> voxelGrid( (gridResolutionX+1)*(gridResolutionY+1)*(gridResolutionZ+1) );

	printf( "SurfaceConstructor::setting up gridpoints...\n" );

	//size_t maxCount = (gridResolutionZ+1)*(gridResolutionY+1)*(gridResolutionX+1);
	//size_t current = 0;

	// setup each gridpoint
	for( long k=0; k<gridResolutionZ+1; ++k )
		for( long j=0; j<gridResolutionY+1; ++j )
			for( long i=0; i<gridResolutionX+1; ++i )
			{
				// compute index into the one-dimensional gridpointarray
				long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
				// store the position of the current gridpoint
				voxelGrid[ index ].position = gridOffset + math::Vec3f( i*m_sampleWidth, j*m_sampleWidth, k*m_sampleWidth );

				// compute the isovalue of the current gridpoint
				// there are different approaches to this
				// We take the approach which is proposed in the paper

				math::Vec3f           averagePosition;
				float                   averageRadius;

				// first we need to compute the average position and the average radius
				// of the particles which are "near"
				// "near" means in this case: closer than twice the average particle spacing
				unsigned int neighbours = computeAverages( voxelGrid[ index ].position, &particles, averageParticleSpacing, averagePosition, averageRadius );


				// no neighbours? then we are outside the fluidsurface
				if( neighbours == 0 )
				{
					voxelGrid[ index ].fixed = false;
					voxelGrid[ index ].isoValue = 999999999999.0f;
				}else
				{
					voxelGrid[ index ].fixed = true;
					voxelGrid[ index ].isoValue = (voxelGrid[ index ].position - averagePosition).getLength() - averageRadius;
				}
				//printf( "%i / %i\n", current++, maxCount );
			}

	// compute remaining(unkown distances) with the fast sweeping algorithm
	// fast sweeping algorithm: iterations
	for(int k=0; k<2; ++k)
	{
		// now we do 8 sweeps (one for each quadrant) and update the signed distance values
		//

		// Quadrant I   : i 0->1 | j 0->1 | k 0->1
		for( long k=1; k<gridResolutionZ+1; ++k )
			for( long j=1; j<gridResolutionY+1; ++j )
				for( long i=1; i<gridResolutionX+1; ++i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i-1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j-1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k-1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}

		// Quadrant II   : i 1->0 | j 0->1 | k 0->1
		for( long k=1; k<gridResolutionZ+1; ++k )
			for( long j=1; j<gridResolutionY+1; ++j )
				for( long i=gridResolutionX+1-2; i>=0; --i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i+1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j-1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k-1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}

		// Quadrant III   : i 1->0 | j 1->0 | k 0->1
		for( long k=1; k<gridResolutionZ+1; ++k )
			for( long j=gridResolutionY+1-2; j>=0; --j )
				for( long i=gridResolutionX+1-2; i>=0; --i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i+1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k-1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}

		// Quadrant IV   : i 0->1 | j 1->0 | k 0->1
		for( long k=1; k<gridResolutionZ+1; ++k )
			for( long j=gridResolutionY+1-2; j>=0; --j )
				for( long i=1; i<gridResolutionX+1; ++i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i-1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k-1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}

		// Quadrant I   : i 0->1 | j 0->1 | k 1->0
		for( long k=gridResolutionZ+1-2; k>=0; --k )
			for( long j=1; j<gridResolutionY+1; ++j )
				for( long i=1; i<gridResolutionX+1; ++i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i-1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j-1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k+1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}

		// Quadrant II   : i 1->0 | j 0->1 | k 1->0
		for( long k=gridResolutionZ+1-2; k>=0; --k )
			for( long j=1; j<gridResolutionY+1; ++j )
				for( long i=gridResolutionX+1-2; i>=0; --i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i+1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j-1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k+1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}

		// Quadrant III   : i 1->0 | j 1->0 | k 1->0
		for( long k=gridResolutionZ+1-2; k>=0; --k )
			for( long j=gridResolutionY+1-2; j>=0; --j )
				for( long i=gridResolutionX+1-2; i>=0; --i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i+1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k+1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}

		// Quadrant IV   : i 0->1 | j 1->0 | k 1->0
		for( long k=gridResolutionZ+1-2; k>=0; --k )
			for( long j=gridResolutionY+1-2; j>=0; --j )
				for( long i=1; i<gridResolutionX+1; ++i )
				{
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;
					long hNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i-1;
					long vNeighbourIndex = k*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i;
					long dNeighbourIndex = (k+1)*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					if( !voxelGrid[ index ].fixed )
						sweepUpdateDistances( voxelGrid[ index ].isoValue, voxelGrid[ hNeighbourIndex ].isoValue, voxelGrid[ vNeighbourIndex ].isoValue, voxelGrid[ dNeighbourIndex ].isoValue );
				}
	}

	// create and setup the samples
	std::vector<Sample> samples( gridResolutionX*gridResolutionY*gridResolutionZ );

	printf( "SurfaceConstructor::setting up samples...\n" );

	// setup each sample
	for( long k=0; k<gridResolutionZ; ++k )
		for( long j=0; j<gridResolutionY; ++j )
			for( long i=0; i<gridResolutionX; ++i )
			{
				// compute index into the one-dimensional samplearray
				long index = k*gridResolutionX*gridResolutionY + j*gridResolutionX + i;

				// each sample has 8 vertices which are gridpoints
				// now locate all neighbouring gridpoints of the current sample and store it
				// info: the indicees 1<->3 and 5<->7 are exchanged because the tables assume a right handed coordinate system
				samples[ index ].vertices[0] = &voxelGrid[ (k+1)*(gridResolutionX+1)*(gridResolutionY+1) +     j*(gridResolutionX+1) + i ];
				samples[ index ].vertices[1] = &voxelGrid[ (k+1)*(gridResolutionX+1)*(gridResolutionY+1) +     j*(gridResolutionX+1) + i+1 ];
				samples[ index ].vertices[2] = &voxelGrid[     k*(gridResolutionX+1)*(gridResolutionY+1) +     j*(gridResolutionX+1) + i+1 ];
				samples[ index ].vertices[3] = &voxelGrid[     k*(gridResolutionX+1)*(gridResolutionY+1) +     j*(gridResolutionX+1) + i ];
				samples[ index ].vertices[4] = &voxelGrid[ (k+1)*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i ];
				samples[ index ].vertices[5] = &voxelGrid[ (k+1)*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i+1 ];
				samples[ index ].vertices[6] = &voxelGrid[     k*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i+1 ];
				samples[ index ].vertices[7] = &voxelGrid[     k*(gridResolutionX+1)*(gridResolutionY+1) + (j+1)*(gridResolutionX+1) + i ];
			}

	// ---------------------------------------------------------------------------------------------
	// smooth the distance values
	// ---------------------------------------------------------------------------------------------

	float sigmaX = 0.144f;
	float sigmaY = 0.144f;

	float tt1 = pow( 2.0f*MATH_PIf, 3.0f / 2.0f );
	float tt2 = pow( 3.0f, 0.5f );

	float gauss_temp = 1.0f / (tt1*tt2);

	int kernelSize = 3;
	float filterKernel[27];

	for( int z = -1; z<2; ++z )
		for( int y = -1; y<2; ++y )
			for( int x = -1; x<2; ++x )
			{
				math::Vec3f dist( x*m_sampleWidth, y*m_sampleWidth, z*m_sampleWidth );

				filterKernel[ (z+1)*kernelSize*kernelSize + (y+1)*kernelSize + x+1 ] = gauss_temp * exp(-0.5f*math::dotProduct(dist, dist));
			}

	printf( "SurfaceConstructor::smoothing gridpoints...\n" );
	for( unsigned int smoothingPass = 0; smoothingPass < 1; ++smoothingPass )
	{
		for( size_t i=0; i<voxelGrid.size(); ++i )
			voxelGrid[i].temp = 0.0f;

		for( long k=1; k<gridResolutionZ+1-1; ++k )
			for( long j=1; j<gridResolutionY+1-1; ++j )
				for( long i=1; i<gridResolutionX+1-1; ++i )
				{
					// compute index into the one-dimensional gridpointarray
					size_t index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					for( int z = -1; z<2; ++z )
						for( int y = -1; y<2; ++y )
							for( int x = -1; x<2; ++x )
							{
								voxelGrid[ index ].temp += filterKernel[ (z+1)*kernelSize*kernelSize + (y+1)*kernelSize + x+1 ]*voxelGrid[ (k+z)*(gridResolutionX+1)*(gridResolutionY+1) + (j+y)*(gridResolutionX+1) + (i+x) ].isoValue;

							}
				}

		for( long k=1; k<gridResolutionZ+1-1; ++k )
			for( long j=1; j<gridResolutionY+1-1; ++j )
				for( long i=1; i<gridResolutionX+1-1; ++i )
				{
					// compute index into the one-dimensional gridpointarray
					long index = k*(gridResolutionX+1)*(gridResolutionY+1) + j*(gridResolutionX+1) + i;

					//if( voxelGrid[index].temp < voxelGrid[index].isoValue )
						voxelGrid[index].isoValue = voxelGrid[index].temp;
				}		
	}

	std::vector<math::Vec3f> vertices;    // the list of all created vertices
	std::vector<int>         indicees;    // the list of all indicees which connect the vertices to triangles


	printf( "SurfaceConstructor::sampling...\n" );
	// create the actual geometry for each sample by marching through all samples
	// and adding content to the vertex and index lists
	for( std::vector<Sample>::iterator it=samples.begin(); it != samples.end(); ++it )
		generateTrianglesFromSample( *it, isoLevel, vertices, indicees );

	// create and return the final output mesh
	return new dk::Mesh( vertices, indicees );
}

//
// generates triangles from the given sample which in fact is a gridcell
// the triangles are generated depending on the combination of in/out states of all 8 sample-vertices
// see marching cube algorithm for detail
//
void SurfaceConstructor::generateTrianglesFromSample( const Sample &sample, float isoLevel, std::vector<math::Vec3f> &vertices, std::vector<int> &indicees )
{
	// there are 256 combinations of in/out states of the vertices
	// the sampling of each combination results in the generation of
	// a unique set of triangles and vertices within the volume of the sample

	// the edgeTable tells for each combination, on which of the 8 edges lie vertices
	// it is used to avoid the computation of interpolationvalues on edges which are not needed
	// the values are bitcoded flags
	int edgeTable[256]={
		0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
		0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
		0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
		0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
		0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
		0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
		0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
		0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
		0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
		0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
		0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
		0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
		0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
		0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
		0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
		0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
		0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
		0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
		0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
		0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
		0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
		0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
		0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
		0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
		0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
		0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
		0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
		0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
		0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
		0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
		0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
		0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0 };

	// from the edgeTable we know which edges have to be interpolated and so we
	// know where and how many vertices are created

	// next we have to create the triangles which connect the created vertices
	// for this we use the triTable
	// it tells us for every of the 256 combinations how (in which order) we have
	// to connect the created vertices. So in fact this is a list of 256 index tables
	int triTable[256][16] = {
		{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
		{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
		{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
		{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
		{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
		{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
		{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
		{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
		{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
		{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
		{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
		{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
		{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
		{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
		{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
		{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
		{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
		{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
		{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
		{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
		{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
		{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
		{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
		{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
		{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
		{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
		{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
		{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
		{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
		{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
		{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
		{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
		{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
		{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
		{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
		{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
		{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
		{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
		{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
		{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
		{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
		{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
		{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
		{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
		{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
		{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
		{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
		{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
		{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
		{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
		{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
		{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
		{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
		{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
		{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
		{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
		{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
		{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
		{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
		{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
		{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
		{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
		{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
		{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
		{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
		{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
		{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
		{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
		{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
		{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
		{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
		{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
		{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
		{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
		{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
		{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
		{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
		{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
		{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
		{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
		{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
		{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
		{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
		{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
		{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
		{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
		{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
		{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
		{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
		{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
		{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
		{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
		{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
		{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
		{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
		{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
		{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
		{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
		{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
		{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
		{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
		{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
		{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
		{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
		{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
		{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
		{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
		{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
		{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
		{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
		{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
		{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
		{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
		{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
		{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
		{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
		{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
		{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
		{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
		{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
		{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
		{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
		{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
		{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
		{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
		{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
		{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
		{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
		{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
		{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
		{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
		{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
		{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
		{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
		{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
		{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
		{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
		{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
		{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
		{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
		{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
		{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
		{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
		{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
		{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
		{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
		{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
		{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
		{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
		{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
		{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
		{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
		{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
		{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
		{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
		{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
		{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
		{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
		{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
		{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
		{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
		{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
		{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
		{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
		{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
		{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
		{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
		{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
		{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
		{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
		{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
		{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
		{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
		{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
		{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}};


		// first we have to find out which of the 256 combinations we have for the current sample
		unsigned int cubeIndex = 0;  // cubeindex is the combination index and the index into the LUTs

		// if the isoValue is lower then the isoLevel, then the point is considered to be within the isoSurface
		if (sample.vertices[0]->isoValue < isoLevel) cubeIndex |= 1;
		if (sample.vertices[1]->isoValue < isoLevel) cubeIndex |= 2;
		if (sample.vertices[2]->isoValue < isoLevel) cubeIndex |= 4;
		if (sample.vertices[3]->isoValue < isoLevel) cubeIndex |= 8;
		if (sample.vertices[4]->isoValue < isoLevel) cubeIndex |= 16;
		if (sample.vertices[5]->isoValue < isoLevel) cubeIndex |= 32;
		if (sample.vertices[6]->isoValue < isoLevel) cubeIndex |= 64;
		if (sample.vertices[7]->isoValue < isoLevel) cubeIndex |= 128; 

		// any edge at all?
		if( edgeTable[ cubeIndex ] == 0 )
			// the current sample is either complete within our out
			return;

		math::Vec3f vertexList[12];  // this list will hold the interpolated vertices
		
		// compute the vertices which we will need
		if (edgeTable[cubeIndex] & 1)
		{
			vertexList[0] = interpolateGridPoints( sample.vertices[0], sample.vertices[1], isoLevel );
		}
		if (edgeTable[cubeIndex] & 2)
		{
			vertexList[1] = interpolateGridPoints( sample.vertices[1], sample.vertices[2], isoLevel );
		}
		if (edgeTable[cubeIndex] & 4)
		{
			vertexList[2] = interpolateGridPoints( sample.vertices[2], sample.vertices[3], isoLevel );
		}
		if (edgeTable[cubeIndex] & 8)
		{
			vertexList[3] = interpolateGridPoints( sample.vertices[3], sample.vertices[0], isoLevel );
		}
		if (edgeTable[cubeIndex] & 16)
		{
			vertexList[4] = interpolateGridPoints( sample.vertices[4], sample.vertices[5], isoLevel );
		}
		if (edgeTable[cubeIndex] & 32)
		{
			vertexList[5] = interpolateGridPoints( sample.vertices[5], sample.vertices[6], isoLevel );
		}
		if (edgeTable[cubeIndex] & 64)
		{
			vertexList[6] = interpolateGridPoints( sample.vertices[6], sample.vertices[7], isoLevel );
		}
		if (edgeTable[cubeIndex] & 128)
		{
			vertexList[7] = interpolateGridPoints( sample.vertices[7], sample.vertices[4], isoLevel );
		}
		if (edgeTable[cubeIndex] & 256)
		{
			vertexList[8] = interpolateGridPoints( sample.vertices[0], sample.vertices[4], isoLevel );
		}
		if (edgeTable[cubeIndex] & 512)
		{
			vertexList[9] = interpolateGridPoints( sample.vertices[1], sample.vertices[5], isoLevel );
		}
		if (edgeTable[cubeIndex] & 1024)
		{
			vertexList[10] = interpolateGridPoints( sample.vertices[2], sample.vertices[6], isoLevel );
		}
		if (edgeTable[cubeIndex] & 2048)
		{
			vertexList[11] = interpolateGridPoints( sample.vertices[3], sample.vertices[7], isoLevel );
		}

		unsigned int verticesOffset = (unsigned int)vertices.size();  // when adding triangles, we need to know where the new vertices are


		// now create the triangles by adding vertices and indicees to their lists respectively
		for( unsigned int i=0; triTable[cubeIndex][i] != -1; i+=3 )
		{
			// make sure we dont catch any degenerate triangle (where any 2 vertices fall together )
			if( (vertexList[triTable[cubeIndex][i+0]] != vertexList[triTable[cubeIndex][i+1]])&&
				(vertexList[triTable[cubeIndex][i+1]] != vertexList[triTable[cubeIndex][i+2]])&&
				(vertexList[triTable[cubeIndex][i+2]] != vertexList[triTable[cubeIndex][i+0]]))
			{
				vertices.push_back( vertexList[triTable[cubeIndex][i+0]] );
				vertices.push_back( vertexList[triTable[cubeIndex][i+1]] );
				vertices.push_back( vertexList[triTable[cubeIndex][i+2]] );

				// reverse order
				//indicees.push_back( (int)vertices.size() - 3 );
				//indicees.push_back( (int)vertices.size() - 2 );
				//indicees.push_back( (int)vertices.size() - 1 );
				indicees.push_back( (int)vertices.size() - 1 );
				indicees.push_back( (int)vertices.size() - 2 );
				indicees.push_back( (int)vertices.size() - 3 );

			}
		}
}


//
// Will interpolate the given GridPoints from the given isoLevel.
//
// Each gridpoint has a isoValue. The interpolateGridPoints Method will find the point which lies
// on the line between these two GridPoints and has a isoValue of isoLevel.
//
math::Vec3f SurfaceConstructor::interpolateGridPoints( const GridPoint *p1, const GridPoint *p2, float isoLevel )
{
	/*
	// is the isoLevel "very" close to the first GridPoint?
	if( fabs( isoLevel - p1->isoValue ) < 0.00001f )
	{
		// then we dont need to interpolate just return this position 
		return p1->position;
	}
	// is the isoLevel "very" close to the second GridPoint?
	if( fabs( isoLevel - p2->isoValue ) < 0.00001 )
	{
		// then we dont need to interpolate just return this position 
		return p2->position;
	}
	// are the isoLevels of both GridPoints "very" close ?
	if( fabs( p1->isoValue - p2->isoValue ) < 0.00001 )
	{
		// then return the position of the first GridPoint (which is not the best one can do)
		return p1->position;
	}

	if( fabs(p2->isoValue - p1->isoValue) < 0.001f )
	{
		int debug = 0;
		debug = 10;
	}
	*/

	// compute normalized interpolation parameter t
	float t = (isoLevel - p1->isoValue) / (p2->isoValue - p1->isoValue);
	//float t = 0.5f;

	// now compute and return the linear interpolated result
	return p1->position + t*(p2->position - p1->position);
}