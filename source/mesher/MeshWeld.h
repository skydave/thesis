/*---------------------------------------------------------------------

The MeshWeld class is a class which offers a public function doit.
This function will weld all vertices of the given mesh together which
are in the range which is specified through the weldEpsilon parameter.

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
	/// \brief A utility class which can be used to weld together any vertices of a given mesh
	/// which are closer to each other than a user-specified distance.
	///
	class MeshWeld
	{
		float                             cellSize; // grid cell size; must be at least 2*weld_epsilon
		float                          weldEpsilon; // radius around vertex defining weld-neighbourhood
		float                   weldEpsilonSquared; // squared radius around vertex defining weld-neighbourhood (for speedup)

		std::vector<int>                     first; // start of linked list for each bucket (size = numBuckets)
		std::vector<int>                      next; // links each vertex to next in linked list (size = maxVertices)

		int                             numBuckets; // number of hash buckets to map grid cells into
		int                            maxVertices; // max number of vertices that can be welded at once
		std::vector<Mesh::Vertex *> weldedVertices;
		std::vector<Mesh::Vertex *>      xrefTable; // tells for every inputVertex to which vertex it was welded to
		int                            numVertices; // number of unique vertices currently stored



	public:

		//
		// constructor
		//
		MeshWeld()
		{
			// init helper structures
			numBuckets  = 768;
			maxVertices = 900000;
			weldEpsilon = 0.0001f;
			weldEpsilonSquared = weldEpsilon*weldEpsilon;
			cellSize    = 0.01f;

			first.resize( numBuckets, -1 );
			next.resize( maxVertices, -1 );

			weldedVertices.resize( maxVertices );
		}



		//
		//
		//
		void doIt( Mesh *m, float _weldEpsilon )
		{
			// get the weldEpsilon parameter
			weldEpsilon = _weldEpsilon;
			weldEpsilonSquared = weldEpsilon*weldEpsilon;

			// compute vertex indicees (we will need them) to resolve the vertex index into the xreftable
			int currentIndex = 0;
			for( std::vector<Mesh::Vertex *>::iterator it = m->vertices.begin(); it != m->vertices.end(); ++it, ++currentIndex )
			{
				(*it)->index = currentIndex; // set index
				(*it)->tag = false;         // mark vertex as not used
			}


			weldVertices( m->vertices );

			// post process:
			// the xref table now holds for every vertex the vertex it was welded to


			// iterate through all triangles and update the reference from the xref table
			std::vector<Mesh::Triangle *>::iterator tit = m->triangles.begin();
			while( tit != m->triangles.end() )
			{
				// go through all 3 vertices of the triangle
				for( int i=0; i<3; ++i )
				{
					(*tit)->v[i] = xrefTable[(*tit)->v[i]->index];
				}

				// remove triangle if it is degenerated
				if( ((*tit)->v0 == (*tit)->v1)||((*tit)->v1 == (*tit)->v2)||((*tit)->v2 == (*tit)->v0) )
				{
					tit = m->triangles.erase( tit );
				}else
				{
					// the triangle is valid and wont be removed - so we mark all vertices as used
					(*tit)->v[0]->tag = true;
					(*tit)->v[1]->tag = true;
					(*tit)->v[2]->tag = true;

					// proceed the iteration
					++tit;
				}
			}

			// remove all unused vertices
			std::vector<Mesh::Vertex *>::iterator vit=m->vertices.begin();
			while( vit != m->vertices.end() )
			{
				if( (*vit)->tag == false )
					vit = m->vertices.erase( vit );
				else
					++vit;
			}
		}






	private:
		// maps unbounded gridcell coodinates (x,y) into an index
		// into a fixed-sized array of hash buckets
		unsigned int getGridCellBucket( int x, int y, int z )
		{
			const unsigned int magic1 = 0x8da6b343; // large multiplicative constants;
			const unsigned int magic2 = 0xd8163841; // here: arbitrarly chosen prims
			const unsigned int magic3 = 0xf5cdc4cf;
			unsigned int index = magic1*x + magic2*y + magic3*z;

			// map index into [0, NUM_BUCKETS) range
			return index % numBuckets;
		}



		void locateWeldCandidatesInBucket( Mesh::Vertex *v, unsigned int bucket, std::vector<int> &candidates )
		{
			//scan through linked list of vertices at the index-specified bucket
			for( int index = first[bucket]; index >= 0; index = next[index] )
			{
				// current vertex within range?
				if( (v->position - weldedVertices[index]->position).getSquaredLength() < weldEpsilonSquared )
					// yes, at the index to the list of candidates
					candidates.push_back( index );
			}
		}


		void addVertexToBucket( Mesh::Vertex *v, unsigned int bucket )
		{
			// fill next available vertex buffer entry and link it into vertex list
			weldedVertices[ numVertices ] = v;
			next[ numVertices ] = first[ bucket ];
			first[ bucket ] = numVertices++;
		}


		Mesh::Vertex *weldVertex( Mesh::Vertex *v )
		{
			// ...


			// list of indicees of vertices into the weldedVertices list which
			// are within range
			std::vector<int> weldCandidates;


			// Compute cell coordinates of bounding box of vertex epsilon neighbourhood
			int top = int( (v->position.y - weldEpsilon) / cellSize );
			int bottom = int( (v->position.y + weldEpsilon) / cellSize );
			int left = int( (v->position.x - weldEpsilon) / cellSize );
			int right = int( (v->position.x + weldEpsilon) / cellSize );
			int front = int( (v->position.z - weldEpsilon) / cellSize );
			int back = int( (v->position.z + weldEpsilon) / cellSize );

			// to lessen effect of worst-case behaviour, we track previously tested buckets
			unsigned int prevBucket[8]; // 4 in 2d and 8 in 3d
			int numPrevBuckets = 0;


			// loop over all overlapped cells and test against their buckets
			for( int i=left; i <= right; ++i )
			{
				for( int j=top; j <= bottom; ++j )
				{
					for( int k=front; k <= back; ++k )
					{
						unsigned int bucket = getGridCellBucket( i, j, k );

						// if this bucket already has been tested, dont test it again
						for( int b=0; b< numPrevBuckets; b++ )
							if( bucket == prevBucket[b] )
								goto skipcell;

						// add this bucket to visited list, then test against its contents
						prevBucket[numPrevBuckets++] = bucket;

						// call the function which steps through the linked list of the current bucket, testing
						// if v is within the epsilon of one of the vertices in the bucket
						// the indicees of all vertices which are in range are added to the list of candidates
						locateWeldCandidatesInBucket( v, bucket, weldCandidates );
		skipcell:;
					}
				}
			}

			// no candidates found?
			if( weldCandidates.empty() )
			{
				// couldnt locate any vertex, so add it to the grid, then return vertex itsself
				int x = int (v->position.x / cellSize);
				int y = int (v->position.y / cellSize);
				int z = int (v->position.z / cellSize);

				addVertexToBucket( v, getGridCellBucket(x, y, z) );

				return v;
			}else
			{
				// there are one ore more vertices within range
				// now we will weld the vertex v to the candidate which was added first
				// to know which vertex was added first, we sort the candidates after their index
				// because the lower the index, the earlier the candidate was added 
				std::sort( weldCandidates.begin(), weldCandidates.end() );

				// now we simply take the first candidate - thats the oldest one after sorting
				return weldedVertices[weldCandidates[0]];
			}
		}


		void weldVertices( std::vector<Mesh::Vertex *> &inputVertices )
		{
			// temp
			//math::BoundingBox debugArea( math::Vec3f(0.4447f, 0.0397f, 0.7297f), math::Vec3f(0.4455f, 0.0403f, 0.7303f) );
			math::BoundingBox debugArea( math::Vec3f(0.4495f, 0.0899f, 0.1447f), math::Vec3f(0.4503f, 0.0905f, 0.1453f) );

			// initialize the hash table of linked vertex list
			for( int k=0; k < numBuckets; ++k )
				first[k] = -1;

			// at the beginning we have no weldedVertices
			numVertices = 0;

			// setup the xref table which tells for every inputvertex which vertex it was welded to
			// if the inputVertex was not welded, then the xref vertex reference identical to the reference to the inputVertex
			xrefTable.resize( inputVertices.size() );


			// loop over all vertices, doing something with the welded vertices
			size_t i = 0;
			for( std::vector<Mesh::Vertex *>::iterator it = inputVertices.begin(); it != inputVertices.end(); ++it, ++i )
			{
				// we assume the vertex will not be welded
				xrefTable[i] = *it;

				// try to weld it
				Mesh::Vertex *pVert = weldVertex( xrefTable[i] );

				// reference different? -> then the vertex had been welded -> change xref entry
				if( pVert != xrefTable[i] )
					xrefTable[i] = pVert;
			}
		}
	};






} // namespace dk