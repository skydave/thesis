/*---------------------------------------------------------------------

This class holds information about cracks on a surface mesh.

----------------------------------------------------------------------*/
#pragma once
#include <vector>
#include <math/Math.h>







///
/// \brief the crackinfo class is used to interchange information about generated cracks between multiple apps by reading/writing its data to disk
///
class CrackInfo
{
public:
	///
	/// \brief is a node which has failed the crack-criterion and has been split
	///
	struct CrackPoint
	{
		CrackPoint( const math::Vec3f &pos, const math::Vec3f &norm ) : position(pos), normal(norm){}
		math::Vec3f position;
		math::Vec3f   normal;
	};


	typedef std::vector<CrackPoint> CrackPoints;
	//CrackInfo();               // constructor


	

	void loadFromDisk( std::string filename )
	{
		// open the file
		std::ifstream file;
		file.open( filename.c_str(), std::ios::in | std::ios::binary );

		// file creation not successfull?
		if( !file )
		{
			// quit
			return;
		}

		// reset the current data
		clear();

		// read number of points
		int pointNum = 0;
		file.read( (char*)&pointNum, sizeof( int ) );

		// now read the position of each point
		for( unsigned int i=0; i<(unsigned int)pointNum; ++i )
		{
			math::Vec3f pos, norm;

			file.read( (char*)&pos.x, sizeof( float ) );
			file.read( (char*)&pos.y, sizeof( float ) );
			file.read( (char*)&pos.z, sizeof( float ) );

			file.read( (char*)&norm.x, sizeof( float ) );
			file.read( (char*)&norm.y, sizeof( float ) );
			file.read( (char*)&norm.z, sizeof( float ) );

			points.push_back( CrackPoint( pos, norm ) );
		}
		
		file.close();
	}

	void saveToDisk( std::string filename )
	{
		// createopen the file
		std::ofstream file;
		file.open( filename.c_str(), std::ios::out | std::ios::binary );


		// file creation not successfull?
		if( !file )
			// quit
			return;


		// number of points
		int pointNum = (int)points.size();
		file.write( (char *) &pointNum, sizeof(int) );

		// now write the position of each point
		for( unsigned int i=0; i<(unsigned int)pointNum; ++i )
		{
			file.write( (char *) &points[i].position.x, sizeof(float) );
			file.write( (char *) &points[i].position.y, sizeof(float) );
			file.write( (char *) &points[i].position.z, sizeof(float) );

			file.write( (char *) &points[i].normal.x, sizeof(float) );
			file.write( (char *) &points[i].normal.y, sizeof(float) );
			file.write( (char *) &points[i].normal.z, sizeof(float) );
		}

		// done
		file.close();

		// write edges
		//...

	}



	void addCrackPoint( const math::Vec3f &pos, const math::Vec3f &normal )
	{
		points.push_back( CrackPoint( pos, normal ) );
	}


	void clear()
	{
		points.clear();
		edges.clear();
	}




	std::vector<CrackPoint>                       points;
	std::vector<std::pair<size_t, size_t> >        edges;
};