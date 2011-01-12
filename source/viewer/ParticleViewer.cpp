/*---------------------------------------------------------------------

The particle viewer displays sequences of particles from a simulation

----------------------------------------------------------------------*/
#include "ParticleViewer.h"
#include "util/PathInfo.h"
#include "win32/OpenGLTools.h"


//
// renders the particles out with opengl
//
void ParticleViewer::render()
{
	// render the particles with opengl
	glPointSize( 1.2f );
	glBegin( GL_POINTS );
	glColor3f( 0.0f, 0.0f, 1.0f );

	for( std::vector<math::Vec3f>::iterator it=stepData.particles.begin(); it != stepData.particles.end(); ++it )
	{
		glColor3f( 0.0f, 0.0f, 1.0f );

		glVertex3fv( (*it).v );
	}
	glEnd();
}

//
// assigns the file to get the data from
//
void ParticleViewer::assignFile( const std::string &filename )
{
	// examine extension
	if( dk::util::PathInfo::getExtension( filename ) == "pcl" )
	{
		// read particle file (SimulationStepData will automaticly detect the pcl file)
		stepData.load( filename, false, true );
	}else
	if( dk::util::PathInfo::getExtension( filename ) == "zip" )
	{
		// load only the particle data
		stepData.load( filename, false, true );
	}
}
