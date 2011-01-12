/*---------------------------------------------------------------------

Meshing is the process of creating a triangle representation of a
fluid which is represented by particles. It consist of:
- surface construction (computing in/out -> marching cubes)
- welding (mc produces a raw triangle soup)
- smoothing (makes the result look better)

This program will load and mesh .pcl files which are files generated
from the simulator. They hold a certain number of particles which
are nothing more than mere 3d points.
The pcl file has the following layout:

int number of particles
number of particles times:
	float x-coordinate
	float y-coordinate
	float z-coordinate

The meshing results (which are meshes) are stored in *.obj format so
that they can be used down the pipeline.

----------------------------------------------------------------------*/
#include <windows.h>
#include <string>
#include <iostream>
#include <fstream>
#include <tchar.h>
#include <boost/program_options.hpp>

#include <math/Math.h>
#include <util/FileSequence.h>
#include <util/StringManip.h>
#include <util/PathInfo.h>
#include <util/unzip.h>
#include <util/MemoryFile.h>
#include <dk/ObjIO.h>
#include <dk/BinObjIO.h>
#include <dk/Mesh.h>
#include "MeshSmooth.h"
#include "MeshWeld.h"
#include "SurfaceConstructor.h"
#include "Particle.h"

using namespace dk;

namespace po = boost::program_options;



float                                                             sampleSize; // size of a marching cubes cell
float                                                averageParticleDistance; // average spacing between the particles -> tells how dense the particle cloud is...
void loadParticles( std::string fileName, std::vector<Particle> &particles ); // loads *.pcl files and pcl files from zip packaged fluid simulationstates


void processTest(  std::string objFileName )
{
	std::vector<Particle>         particles;  // list of particles of the current frame

	Particle p1,p2;
	p1.position = math::Vec3f( 0.0f, 0.0f, 0.0f );
	p2.position = math::Vec3f( 0.1f, 0.0f, 0.0f );
	particles.push_back( p1 );
	particles.push_back( p2 );

	// are there any particles?
	if( particles.empty() )
		return;

	// do the meshing
	printf("constructing surface...\n");
	SurfaceConstructor surfaceConstructor;
	Mesh *mesh = surfaceConstructor.constructSurface( particles, sampleSize, averageParticleDistance );

	printf("welding...\n");
	MeshWeld meshWeld;
	meshWeld.doIt( mesh, 0.0001f );

	printf("smoothing...\n");
	smoothMesh( mesh, 5.0f );

	printf("exporting mesh...\n");
	io::exportToObjFile( mesh, objFileName );

	delete mesh;
}


void processPclFile( std::string pclFileName, std::string objFileName )
{
	std::vector<Particle>         particles;  // list of particles of the current frame

	printf("loading particles from file %s\n", pclFileName.c_str());
	loadParticles( pclFileName, particles );

	// are there any particles?
	if( particles.empty() )
		return;

	// do the meshing
	printf("constructing surface...\n");
	SurfaceConstructor surfaceConstructor;
	Mesh *mesh = surfaceConstructor.constructSurface( particles, sampleSize, averageParticleDistance );

	printf("postprocessing result...\n");

	printf("welding...\n");
	MeshWeld meshWeld;
	meshWeld.doIt( mesh, 0.0001f );

	printf("smoothing...\n");
	smoothMesh( mesh, 1.0f );

	printf("exporting mesh...\n");

	if( util::getExtension( objFileName ) == ".obj" )
		io::exportToObjFile( mesh, objFileName );
	if( util::getExtension( objFileName ) == ".binobj" )
		io::exportToBinObjFile( mesh, objFileName );

	delete mesh;
}

//
// entry point
//
int main( int argc, char **argv )
{
	//if( argc < 2 )
	//{
	//	printf_s( "Missing argument: First file in sequence\n" );
	//	return 0;
	//}


	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "print help message")
		("input", po::value<std::string>(), "input file" )
		("size", po::value<float>(&sampleSize)->default_value(0.0042f), "with of one marching cubes cell")
		("dist", po::value<float>(&averageParticleDistance)->default_value(0.0043f), "average distance between the particles (this value tells how dense the particle cloud is)")
	;

	po::positional_options_description p;
	p.add("input", -1 );


	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	po::notify(vm);

	if( !vm.count("input") )
	{
		printf_s( "Missing argument: First file in sequence\n" );
		return 0;
	}

	util::FileSequence fs( vm["input"].as<std::string>() );

	// for each file in the sequence
	for( util::FileSequence::iterator it = fs.begin(); it != fs.end(); ++it )
		processPclFile( *it, util::setExtension( *it, ".binobj" ) );


	return 0;
}


//
// tries to open a file from the given framenumber and reads the content into the vector of particles
//
void loadParticles( std::string fileName, std::vector<Particle> &particles )
{

	// if the file is a zip file, then we assume that the zip contains the pcl file among other things
	if( util::getExtension( fileName ) == ".zip" )
	{
		// unzip the pcl file into the memory

		std::string title = dk::util::PathInfo::getTitle( fileName );

		// open the zipfile
		HZIP zip = OpenZip( util::toWString( fileName ).c_str(), 0 );

		// set base directory for unzipping operations
		SetUnzipBaseDir( zip, _T("\\") );

		// zip entry for the pcl file
		ZIPENTRY ze;
		int entryIndex = -1;

		// look for the pcl file
		FindZipItem( zip, util::toWString( std::string( title ) + ".pcl" ).c_str(), false, &entryIndex, &ze );

		if( entryIndex == -1 )
		{
			printf( "error : zip file doesnt contain %s\n", util::setExtension( fileName, ".pcl" ).c_str() );
			// stop everything
			return;
		}

		// prepare memory
		MemoryFile file;

		file.resize( ze.unc_size );

		// we have found the particles -> unpack them into memory
		UnzipItem( zip, entryIndex, file.getMemory(), file.size() );

		CloseZip( zip );

		// now read the particles
		int particleCount = 0;

		file.read((char *) &particleCount, sizeof(int) );
		printf( "number of particles: %i\n", particleCount );

		// prepare vector
		particles.resize( particleCount );

		// now read in (kinda rough)
		for( int i=0; i<particleCount; ++i )
		{
			float pos[3];
			file.read((char *) pos, sizeof(float) * 3 );

			particles[i].position.x = pos[0];
			particles[i].position.y = pos[1];
			particles[i].position.z = pos[2];
		}
	}else
	{
		// open file from disk
		std::ifstream in( fileName.c_str(), std::ios::in | std::ios::binary );

		if( !in )
			return;

		int particleCount = 0;

		in.read((char *) &particleCount, sizeof(int) );
		printf( "number of particles: %i\n", particleCount );


		// prepare vector
		particles.resize( particleCount );

		// now read in (kinda rough)
		for( int i=0; i<particleCount; ++i )
		{
			float pos[3];
			in.read((char *) pos, sizeof(float) * 3 );

			particles[i].position.x = pos[0];
			particles[i].position.y = pos[1];
			particles[i].position.z = pos[2];
		}

		in.close();
	}
}