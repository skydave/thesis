/*---------------------------------------------------------------------

The smoothMesh function performs a laplace smooth operation on a
mesh.
The algorithm is designed in way that it matches exactly the way
softimage|xsi does it.

----------------------------------------------------------------------*/
#pragma once
#include <stdio.h>
#include <vector>
#include <algorithm>

#include <math/Math.h>
#include "dk/Mesh.h"

namespace dk
{

	///
	/// \brief Laplace smooth on a mesh
	/// Parameter strength is the number of iterations.
	/// 0.5 would mean that a halve laplace smooth iteration is done
	///
	void smoothMesh( Mesh *m, float strength )
	{
		std::vector<float>                iterationWeights;  // influence value for every smooth iteration
		int numberOfFullIterations = int( floor(strength) );

		// every iteration but the last has full influence
		for( int i=0; i<numberOfFullIterations; ++i )
			iterationWeights.push_back( 1.0f );

		// the last iteration has a weight of the fractional part of the strength parameter
		iterationWeights.push_back( strength - numberOfFullIterations );


		// prepare the vertices
		int currentIndex = 0;
		for( std::vector<Mesh::Vertex *>::iterator vit = m->vertices.begin(); vit != m->vertices.end(); ++vit, ++currentIndex )
			(*vit)->index = currentIndex;

		// this vector holds a list of triangles for each vertex
		// the triangles in each list are the triangles which reference the current vertex
		// this is used to speed up 1-neighbourhood access
		std::vector< std::vector<Mesh::Triangle *> > triangles;

		triangles.resize( m->vertices.size() );

		for( std::vector<Mesh::Triangle *>::iterator tit = m->triangles.begin(); tit != m->triangles.end(); ++tit )
		{
			triangles[(*tit)->v0->index].push_back( (*tit) );
			triangles[(*tit)->v1->index].push_back( (*tit) );
			triangles[(*tit)->v2->index].push_back( (*tit) );
		}

		std::vector<math::Vec3f> newVertexPositions;
		newVertexPositions.resize( m->vertices.size() );

		// for every iteration:
		for( unsigned int i=0; i<iterationWeights.size(); ++i )
		{
			// get the weight of the current iteration
			float weight = iterationWeights[i];

			// iterate over all vertices
			for( std::vector<Mesh::Vertex *>::iterator vit = m->vertices.begin(); vit != m->vertices.end(); ++vit )
			{
				Mesh::Vertex *v              = *vit;
				std::set<Mesh::Vertex *> neighbourhood;

				// find 1-neighbourhood
				for( std::vector<Mesh::Triangle *>::iterator tit = triangles[(*vit)->index].begin(); tit != triangles[(*vit)->index].end(); ++tit )
				{
					if( (*tit)->v0 == v )
					{
						neighbourhood.insert( (*tit)->v1 );
						neighbourhood.insert( (*tit)->v2 );
					}else
					if( (*tit)->v1 == v )
					{
						neighbourhood.insert( (*tit)->v0 );
						neighbourhood.insert( (*tit)->v2 );
					}else
					if( (*tit)->v2 == v )
					{
						neighbourhood.insert( (*tit)->v0 );
						neighbourhood.insert( (*tit)->v1 );
					}
				}


				// average position
				float lambda = .5f;
				math::Vec3f pos;
				for( std::set<Mesh::Vertex *>::iterator nit = neighbourhood.begin(); nit != neighbourhood.end(); ++nit )
					pos += (*nit)->position - v->position;

				newVertexPositions[v->index] = v->position + weight*lambda*(pos/float(neighbourhood.size()));
			}

			// copy new vertex position to the final vertex positions
			for( std::vector<Mesh::Vertex *>::iterator vit = m->vertices.begin(); vit != m->vertices.end(); ++vit )
				(*vit)->position = newVertexPositions[(*vit)->index];
		}
	}

}