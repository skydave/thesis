/*---------------------------------------------------------------------

BinObjIO offers import and export support for obj files in binary format.

*.binobj are binary files which store vertices and triangle lists.
You should prefer the *.obj file which are ASCII based. But for fast
loading and saving the binary format will be better.

----------------------------------------------------------------------*/
#include "BinObjIO.h"

namespace dk
{
	namespace io
	{
		//
		// reads and creates a Mesh from the given *.binobj file. returns 0 if it fails
		//
		Mesh *importFromBinObjFile( std::string fileName )
		{
			// temporary vertex and index buffer
			std::vector<math::Vec3f> vertices;
			std::vector<int>         indicees;


			// try to open a text file stream
			std::ifstream file;
			file.open( fileName.c_str(), std::ios::in | std::ios::binary );

			// file open?
			if( !file )
				// error file could not be opened
				return 0;

			// read vertex number
			unsigned int vertexNum = 0;
			file.read((char *) &vertexNum, sizeof(unsigned int) );

			// prepare vertex buffer
			vertices.resize( vertexNum );

			// read vertices
			for( unsigned int i=0; i<vertexNum; ++i )
			{
				// read x y and z
				file.read((char *) &vertices[i].x, sizeof(float) );
				file.read((char *) &vertices[i].y, sizeof(float) );
				file.read((char *) &vertices[i].z, sizeof(float) );
			}

			// read triangle number
			unsigned int triangleNum = 0;
			file.read((char *) &triangleNum, sizeof(unsigned int) );

			// prepare index buffer
			indicees.resize( triangleNum*3 );

			// read triangles
			for( unsigned int i=0; i<triangleNum; ++i )
			{
				int i1, i2, i3;
				i1 = i2 = i3 = 0;

				file.read((char *) &indicees[i*3+0], sizeof(int) );
				file.read((char *) &indicees[i*3+1], sizeof(int) );
				file.read((char *) &indicees[i*3+2], sizeof(int) );
			}

			file.close();

			return new Mesh( vertices, indicees );
		}

		//
		// creates and writes a *.binobj file from the given mesh
		//
		void exportToBinObjFile( Mesh *mesh, std::string fileName )
		{
			mesh->computeVertexIndicees();

			// create temporary index buffer (resolve the vertex references of each triangle into list-indices)
			std::vector<int> indexBuffer;
			indexBuffer.resize( mesh->triangles.size() * 3 );

			int currentTriangleIndex = 0;
			for( std::vector<Mesh::Triangle *>::iterator it=mesh->triangles.begin(); it != mesh->triangles.end(); ++it, ++currentTriangleIndex )
			{
				indexBuffer[currentTriangleIndex*3 + 0] = (*it)->v0->index;
				indexBuffer[currentTriangleIndex*3 + 1] = (*it)->v1->index;
				indexBuffer[currentTriangleIndex*3 + 2] = (*it)->v2->index;
			}



			// create the file
			std::ofstream file;
			file.open( fileName.c_str(), std::ios::out | std::ios::binary );

			// write number of vertices
			unsigned int vertexNum = (unsigned int)mesh->vertices.size();
			file.write((char *) &vertexNum, sizeof(unsigned int) );

			// vertices
			for( std::vector<Mesh::Vertex *>::iterator it=mesh->vertices.begin(); it != mesh->vertices.end(); ++it )
			{
				file.write( (char *) &(*it)->position.x, sizeof(float) );
				file.write( (char *) &(*it)->position.y, sizeof(float) );
				file.write( (char *) &(*it)->position.z, sizeof(float) );
			}


			// write number of triangles
			unsigned int triangleNum = (unsigned int)mesh->triangles.size();
			file.write((char *) &triangleNum, sizeof(unsigned int) );


			// write out the indices for each triangle
			for( unsigned int i=0; i<mesh->triangles.size(); ++i )
			{
				int i1,i2,i3;

				// be aware that the indexing starts from 0!
				i1 = indexBuffer[i*3+0];
				i2 = indexBuffer[i*3+1];
				i3 = indexBuffer[i*3+2];

				file.write((char *) &i1, sizeof(int) );
				file.write((char *) &i2, sizeof(int) );
				file.write((char *) &i3, sizeof(int) );
			}

			// finish by closing the file
			file.close();
		}
	}
}