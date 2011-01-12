/*---------------------------------------------------------------------

The cracksurface application loads in a mesh and looks for an associated
.zip file with the same filename (but instead of *.obj it expects
to be *.zip). The grid values from the zip are loaded and the stresses
which are stored in each grid cell will be used to compute the surface
stresses on the mesh. Afterwards the surface will be cracked with the
method proposed by [iben04].

----------------------------------------------------------------------*/
#include <vector>
#include <stdarg.h>
#include <iostream>
#include <fstream>

#include "win32/GLWindow.h"
#include "win32/OpenGLTools.h"
#include "math/Math.h"
#include "util/FileSequence.h"
#include "util/StringManip.h"
#include "dk/Camera.h"
#include "dk/DebugNavigator.h"
#include "dk/ColorGradient.h"
#include "dk/Mesh.h"
#include "dk/BinObjIO.h"
#include "dk/ObjIO.h"
#include "SimulationStepData.h"

#include "SurfaceCrackGenerator.h"

using namespace dk;


LRESULT CALLBACK                    glWinProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );  // event handling routine
void                                  renderSurfaceCrackGenerator( SurfaceCrackGenerator *cracker );
void renderCracks( SurfaceCrackGenerator *cracker, float r = 0.0f, float g = 0.0f, float b = 0.0f );  // renders only the boundary edges of the surfacecracker


bool                                                                                 g_done = false;  // indicates finish of the message loop
bool                                                                       renderMeshSwitch = false;
bool                                                                             displayMesh = true;  // display cracker/mesh or not
int                                                                            displayWireframe = 0;
bool                                                                           displayEigen = false;
int                                                                               lastMousePosX = 0;  // position of the mouse within the last frame is needed for interaction
int                                                                               lastMousePosY = 0;  // position of the mouse within the last frame is needed for interaction

GLWindow                                                                                     window;

Camera                                                                                       camera;  // camera which represents a view on the scene
DebugNavigator                                                                      viewManipulator;  // the manipulator is used to manipulate the camera from some interaction inputs

util::FileSequence                                                                     fileSequence;  // Sequence of mesh files
util::FileSequence::iterator                                                   fileSequenceIterator;  // points to the current filename within the sequence


Mesh                                                                                     *plane = 0;  // mesh for the current frame
Mesh                                                                                  *tempMesh = 0;  // mesh for the current frame



///
/// \brief This structure holds a mesh and its associated SimulationStepData
///
struct Data
{
	///
	/// constructor
	///
	Data()
	{
		mesh = 0;
		cracker = 0;
		maxEquivalentStress = 1.0f;
		maxPressure = 1.0f;
	}

	///
	/// loads the filename-specified mesh and the specified simulationstepdata (specified through an *.zip file)
	///
	void load( std::string meshFilename, std::string simdataFilename )
	{
		// load mesh -------------------------------------------------------
		if( mesh )
		{
			delete mesh;
			mesh = 0;
		}

		if( util::getExtension(meshFilename) == ".obj" )
			mesh = io::importFromObjFile( meshFilename );
		if( util::getExtension(meshFilename) == ".binobj" )
			mesh = io::importFromBinObjFile( meshFilename );

		if( mesh )
			mesh->computeNormals();

		// load simulationstepdata -------------------------------------------------------
		// load the simulationstepdata and only read the grid data (if the simdataFilename specifies a *.grid file, then it is automaticly loaded )
		simData.load( simdataFilename, true, false );

		// TEMP: negate stress tensor components
		for( size_t i=0; i<simData.grid.getCellCount(); ++i )
		{
			Grid::Cell *cell = simData.grid.getCell(i);
			//cell->value("stress[0]") = -cell->value("stress[0]");
			//cell->value("stress[1]") = -cell->value("stress[1]");
			//cell->value("stress[2]") = -cell->value("stress[2]");
			//cell->value("stress[3]") = -cell->value("stress[3]");
			//cell->value("stress[4]") = -cell->value("stress[4]");
			//cell->value("stress[5]") = -cell->value("stress[5]");
			//cell->value("stress[6]") = -cell->value("stress[6]");
			//cell->value("stress[7]") = -cell->value("stress[7]");
			//cell->value("stress[8]") = -cell->value("stress[8]");
		}


		// compute equivalent stress for each cell and find the maximum equivalent stress
		// this is done so that the values can be used to represent the stress as color
		maxEquivalentStress = 1.0f;
		maxPressure = 1.0f;
		for( size_t i=0; i<simData.grid.getCellCount(); ++i )
		{
			// add von mises equivalent stress value
			Grid::Cell *cell = simData.grid.getCell(i);

			// compute equivalent stress
			float eqvStress = sqrt( 3.0f/2.0f ) * math::frobeniusNorm( math::Matrix33f( cell->value("stress[0]"), cell->value("stress[1]"), cell->value("stress[2]"),
																						cell->value("stress[3]"), cell->value("stress[4]"), cell->value("stress[5]"),
																						cell->value("stress[6]"), cell->value("stress[7]"), cell->value("stress[8]") ) );

			// add value with the comuted stress as initial value to the cell
			cell->addValue( "equivalentStress", eqvStress );

			// find maximum equivalent stress
			if( eqvStress > maxEquivalentStress )
				maxEquivalentStress = eqvStress;

			// find maximum pressure
			if( cell->value("pressure") > maxPressure )
				maxPressure = cell->value("pressure");

		}


		// normalize equivalent stress values and pressure of all cells
		for( size_t i=0; i<simData.grid.getCellCount(); ++i )
		{
			simData.grid.getCell(i)->value( "equivalentStress" ) = math::mapValueToRange( 0.0f, maxEquivalentStress, 0.0f, 1.0f, simData.grid.getCell(i)->value( "equivalentStress" ) );
			simData.grid.getCell(i)->value( "pressure" ) = 1.0f - math::mapValueToRange( 0.0f, maxPressure, 0.0f, 1.0f, simData.grid.getCell(i)->value( "pressure" ) );
		}

		printf( "maximum vonMises Stress : %f\n", maxEquivalentStress );
		printf( "maximum pressure : %f\n", maxPressure );

		// set the vertex colors of the mesh according to the stress
		applyColorFromStress();
	}

	/// \brief compute meshvertexcolors from stress
	/// This method will compute the color of each vertex of the mesh from the equivalent stress at the vertex position.
	/// Result is a mesh which colors represent the stress present at the surface.
	///
	void applyColorFromStress()
	{
		if( !mesh )
			return;

		// this color gradient is the one typical for engineering visualizations
		ColorGradient stressGradient = ColorGradient::Stress();

		printf( "transfering gridvalues to vertex-colors...\n" );
		// go over all mesh vertices and compute vertex colors
		for( std::vector<Mesh::Vertex *>::iterator it = mesh->vertices.begin(); it != mesh->vertices.end(); ++it )
		{
			Mesh::Vertex *vert = *it;

			// get interpolated stress tensor at the vertex position ----------------------------------
			math::Matrix33f stressTensor = math::Matrix33f::Zero();

			for( size_t i=0; i<9; ++i )
			{
				char identifier[10];
				sprintf_s( identifier, 10, "stress[%i]", i );
				stressTensor.ma[i] = simData.grid.getValueAt( vert->position.x, vert->position.y, vert->position.z, identifier );
			}

			// compute von Mises Stress from Stress Tensor -------------------------------------------
			//float equivalentStress = sqrt( 3.0f/2.0f ) * math::frobeniusNorm( stressTensor );
			float equivalentStress = sqrt( 3.0f/2.0f ) * math::frobeniusNorm( stressTensor*simData.grid.getValueAt( vert->position.x, vert->position.y, vert->position.z, "pressure" ) );

			// use the equivalent stress (which is within [0,1] because it has been normalized) as an index into a color gradient
			vert->color = stressGradient.getOutput( math::mapValueToRange( 0.0f, maxEquivalentStress, 0.0f, 1.0f, equivalentStress ) );
			//vert->color = stressGradient.getOutput( math::mapValueToRange( 0.0f, maxPressure, 0.0f, 1.0f, simData.grid.getValueAt( vert->position.x, vert->position.y, vert->position.z, "pressure" ) ) );
		}
	}

	///
	/// This method computes the surface stress of the cracker mesh from the grid
	///
	void applySurfaceStressesFromGrid()
	{
		printf( "applying stress from grid-data...\n" );

		SurfaceCrackGenerator *cg = cracker;

		// iterate over all cracker nodes
		for( std::vector<SurfaceCrackGenerator::Node *>::iterator it = cg->nodes.begin(); it != cg->nodes.end(); ++it )
		{
			SurfaceCrackGenerator::Node *n = *it;

			// get interpolated stress tensor at the node position
			math::Matrix33f stressTensor = math::Matrix33f::Zero();

			for( size_t i=0; i<9; ++i )
			{
				char identifier[10];
				sprintf_s( identifier, 10, "stress[%i]", i );
				stressTensor.ma[i] = simData.grid.getValueAt( n->position.x, n->position.y, n->position.z, identifier );
			}

			n->globalStressTensor = stressTensor*simData.grid.getValueAt( n->position.x, n->position.y, n->position.z, "pressure" );

			// max eigenvalue
			std::vector< float > eigenValues;
			std::vector< std::vector<float> > f_eigenVectors;
			std::vector<math::Vec3f> eigenVectors;

			doEigenDecomposition( 3, 3, stressTensor.ma, eigenValues, f_eigenVectors );
			
			for( size_t i=0; i<f_eigenVectors.size(); ++i )
				eigenVectors.push_back( math::Vec3f( f_eigenVectors[i][0], f_eigenVectors[i][1], f_eigenVectors[i][2] ) );

			n->maxGlobalEigenvalue = eigenValues[0];

			for( size_t i=1; i<eigenValues.size(); ++i )
			{
				if( fabs(eigenValues[i]) > fabs(n->maxGlobalEigenvalue) )
				{
					n->maxGlobalEigenvalue = eigenValues[i];
				}
			}
		}

		// now iterate over all elements and transfer the tensor into the local coordinate system
		// of each element
		for( std::vector<SurfaceCrackGenerator::Element *>::iterator it = cg->elements.begin(); it != cg->elements.end(); ++it )
		{
			SurfaceCrackGenerator::Element *e = *it;

			// get interpolated stress tensor
			math::Matrix33f stressTensor = math::Matrix33f::Zero();

			for( size_t i=0; i<9; ++i )
				stressTensor.ma[i] = simData.grid.getValueAt( e->center.x, e->center.y, e->center.z, i+1 );

			// rotate tensor into the local coordinate system of the element using the direction cosine
			// the global coordinate system is defined by its euklidian basevectors e1,e2,e3
			math::Vec3f parent_u = math::Vec3f( 1.0f, 0.0f, 0.0f );
			math::Vec3f parent_v = math::Vec3f( 0.0f, 1.0f, 0.0f );
			math::Vec3f parent_w = math::Vec3f( 0.0f, 0.0f, 1.0f );

			math::Matrix33f rot = math::Matrix33f( math::dotProduct( parent_u, e->u ), math::dotProduct( parent_u, e->v ), math::dotProduct( parent_u, e->normal ),
												   math::dotProduct( parent_v, e->u ), math::dotProduct( parent_v, e->v ), math::dotProduct( parent_v, e->normal ),
												   math::dotProduct( parent_w, e->u ), math::dotProduct( parent_w, e->v ), math::dotProduct( parent_w, e->normal ));
			math::Matrix33f rott = math::transpose( rot );

			math::Matrix33f localStressTensor = rott * stressTensor * rot;

			// just ignore the stresses which act normal to the element normal (local z direction of the element)
			e->sigma._11 = localStressTensor._11;
			e->sigma._12 = localStressTensor._12;
			e->sigma._21 = localStressTensor._21;
			e->sigma._22 = localStressTensor._22;
		}
	}

	///
	/// This method creates a surface cracker class from the mesh and sets it up for usage
	///
	void prepareSurfaceCracker()
	{
		printf( "creating SurfaceCrackGenerator from mesh...\n" );

		// create a cracker class from the mesh -------------------------------------------------
		cracker = new SurfaceCrackGenerator( mesh );

		// --------------------------------------------------------------------------------------
		// compute the 2-dimensional stresstensors of the cracker-elements from the 3-dimensional
		// stress-tensors from the grid
		applySurfaceStressesFromGrid();

		//cracker->applyDirectionStress( 10.0f, 0.0f, 0.0f );
		//cracker->applyUniformShrinkage( 5.0f );

		// inform that all stresses have been initialized --------------------------------------
		// this is to compute svThreshold and eigenvalues/vectors of sigma
		cracker->stressesInitialized();

		// setup cracker parameters ------------------------------------------------------------
		//cracker->setMaterialToughness( 0.0005f );

		// visc1.283 reduced2
		//cracker->setMaterialToughness( 0.05f );
		//cracker->setCrackPropagation( 0.75f );
		//cracker->setCracksPerIteration( 3 );
		//cracker->setRelaxationRate( 0.005f );
		//cracker->setRelaxIterations( 7 );


		// visc1.283 reduced
		cracker->setMaterialToughness( 0.02f );
		cracker->setCrackPropagation( 0.95f );
		cracker->setCracksPerIteration( 6 );
		cracker->setRelaxationRate( 0.005f );
		cracker->setRelaxIterations( 3 );

		// visc1.283 non-reduced
		//cracker->setMaterialToughness( 0.0001f );
		////cracker->setMaterialToughness( 0.0005f ); // mountain2
		//cracker->setCrackPropagation( 0.999f );
		//cracker->setCracksPerIteration( 24 );
		//cracker->setRelaxationRate( 0.002f );
		//cracker->setRelaxIterations( 1 );

		// mountain2.029 - reduced
		//cracker->setMaterialToughness( 0.004f );
		//cracker->setCrackPropagation( 0.55f );
		//cracker->setCracksPerIteration( 3 );
		//cracker->setRelaxationRate( 0.002f );
		//cracker->setRelaxIterations( 1 );


		// one pass with many cracks
		//cracker->setMaterialToughness( 1.0f );
		//cracker->setCrackPropagation( 1.0f );
		//cracker->setCracksPerIteration( 760 );
	}


	Mesh                                                                        *mesh; // mesh for the current frame
	SurfaceCrackGenerator                                                    *cracker; // this class will be used to generate the surface cracks...
	SimulationStepData                                                        simData; // this class holds the grid and particle data of the associated simulation step
	float                                                         maxEquivalentStress;
	float                                                                 maxPressure;
};


Data            *current = 0; // current data which is processed




//
// entry point
//
int main( int argc, char **argv )
{
	/*
	if( argc < 2 )
	{
		printf_s( "Missing argument: First file in sequence\n" );
		return 0;
	}
	*/

	// load the filesequence of meshes
	//fileSequence = util::FileSequence( argv[1] );
	//fileSequenceIterator = fileSequence.begin();


	/*
		std::string meshFilename = "mountain2\\mountain2.obj";
		if( util::getExtension(meshFilename) == ".obj" )
			tempMesh = io::importFromObjFile( meshFilename );
		if( util::getExtension(meshFilename) == ".binobj" )
			tempMesh = io::importFromBinObjFile( meshFilename );

		if( tempMesh )
		{
			for( std::vector<Mesh::Vertex *>::iterator it = tempMesh->vertices.begin(); it != tempMesh->vertices.end(); ++it )
				(*it)->position.z +=1.0f;
			tempMesh->computeNormals();
		}*/


	
	Data data;
	data.load( "visc1\\visc1.283.binobj", "visc1\\visc1.283.grid" );

	//data.load( "visc1\\visc1.283_reduced.obj", "visc1\\visc1.283.grid" );

	//data.load( "mountain2\\mountain2.029.binobj", "mountain2\\mountain2.029.zip" );
	//data.load( "mountain2\\mountain2.029_reduced.obj", "mountain2\\mountain2.029.zip" );
	//data.load( "plane.obj", "mountain2\\mountain2.028.zip" );
	//data.load( "plane.obj", "visc1.283.grid" );

	//data.prepareSurfaceCracker();

	//current = &data;


	// load the mesh
	////loadMesh( *fileSequenceIterator, mesh );
	////loadMesh( "visc1.000_reduced.obj", mesh );
	//loadMesh( "test_reduced.binobj", mesh );
	//loadMesh( "visc1.283.obj", mesh );
	//loadMesh( "visc1.283_reduced.obj", mesh );
	//loadMesh( "visc1.283_reduced.obj", mesh );
	//loadMesh( "visc1.283_reduced.obj", mesh );
	//loadMesh( "visc1.283_reduced.binobj", mesh ); // !!
	//loadMesh( "crack_sphere_retiled.obj", mesh );
	//loadMesh( "plane.obj", plane );






	//viewManipulator.lookAt = math::Vec3f( 0.5f, 0.5f, 0.5f );
	//viewManipulator.azimuth = viewManipulator.elevation = 0.0f;

	viewManipulator.setLookAt( math::Vec3f( 0.4668f, 0.31053f, 0.52456f ) );
	viewManipulator.azimuth = 53.0f;
	viewManipulator.elevation = 19.0f;
	viewManipulator.distance = .6558f;

	//viewManipulator.setLookAt( math::Vec3f( 0.450299f, 0.243314f, 0.464231f ) );
	//viewManipulator.azimuth = 32.0f;
	//viewManipulator.elevation = 9.0f;
	//viewManipulator.distance = .405866f;

	viewManipulator.setCamera( &camera );

	window.createGLWindow( "test", 800, 600, 100, 100, 32, 0, glWinProc, NULL);

	window.show();



	// Main message loop:
	while ( !g_done ) 
	{
		MSG msg;
		if( PeekMessage( &msg,NULL,0,0,PM_REMOVE) )
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}else
		{
			// draw the scene -----------------------
			//glClearColor( .5f, .5f, .5f, 1.0f );
			glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );
			glClear( GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

			glViewport( (GLsizei)0.0f, (GLsizei)0.0f, (GLsizei)window.getWidth(), (GLsizei)window.getHeight() );

			glMatrixMode( GL_PROJECTION );
			glLoadMatrixf( camera.getProjectionMatrix().ma );

			glMatrixMode( GL_MODELVIEW );
			// the worldToEyeMatrix is computed by the OrbitManipulator which is coupled with the mouse input
			glLoadMatrixf( (GLfloat *)camera.getViewMatrix().ma );

			//drawGrid();

			// draw mesh
			//if( displayMesh && data.mesh )
			{
				glEnable( GL_DEPTH_TEST );
				glEnable( GL_CULL_FACE );
				glEnable( GL_NORMALIZE );

				glEnable( GL_LIGHTING );

				headLight();

				math::Matrix44f transform = math::Matrix44f::Identity();
				math::Vec3f translation( .8f, 0.3f, .5f );
				math::Vec3f rotation( 0.0f, 0.0f, math::degToRad(-15.0f) );
				math::Vec3f scale( 0.23f, 0.11f, 0.35f );



				transform.translate( translation.x, translation.y, translation.z );
				transform.rotateZ( rotation.z );
				transform.rotateY( rotation.y );
				transform.rotateX( rotation.x );
				transform.scale( scale.x, scale.y, scale.z );

				glPushMatrix();
				//glLoadIdentity();
				//glMultMatrixf( (GLfloat *)transform.ma );

				math::Vec3f verts[8];

				verts[0] = math::Vec3f( -1.0f, -1.0f, -1.0f );
				verts[1] = math::Vec3f( 1.0f, -1.0f, -1.0f );
				verts[2] = math::Vec3f( 1.0f, 1.0f, -1.0f );
				verts[3] = math::Vec3f( -1.0f, 1.0f, -1.0f );

				verts[4] = math::Vec3f( -1.0f, -1.0f, 1.0f );
				verts[5] = math::Vec3f( 1.0f, -1.0f, 1.0f );
				verts[6] = math::Vec3f( 1.0f, 1.0f, 1.0f );
				verts[7] = math::Vec3f( -1.0f, 1.0f, 1.0f );

				for( unsigned int i=0; i<8; ++i )
					verts[i] = math::transform( verts[i], transform );

				//renderMesh( tempMesh );
				glBegin( GL_QUADS );
				//front
				glNormal3f( 0.0f, 0.0f, -1.0f );
				glVertex3fv( verts[3].v );
				glVertex3fv( verts[2].v );
				glVertex3fv( verts[1].v );
				glVertex3fv( verts[0].v );

				// top
				glNormal3f( 0.0f, 1.0f, 0.0f );
				glVertex3fv( verts[2].v );
				glVertex3fv( verts[3].v );
				glVertex3fv( verts[7].v );
				glVertex3fv( verts[6].v );

				// bottom
				glNormal3f( 0.0f, -1.0f, 0.0f );
				glVertex3fv( verts[1].v );
				glVertex3fv( verts[5].v );
				glVertex3fv( verts[4].v );
				glVertex3fv( verts[0].v );

				// back
				glNormal3f( 0.0f, 0.0f, 1.0f );
				glVertex3fv( verts[4].v );
				glVertex3fv( verts[5].v );
				glVertex3fv( verts[6].v );
				glVertex3fv( verts[7].v );

				// left
				glNormal3f( -1.0f, 0.0f, 0.0f );
				glVertex3fv( verts[0].v );
				glVertex3fv( verts[4].v );
				glVertex3fv( verts[7].v );
				glVertex3fv( verts[3].v );

				// right
				glNormal3f( 1.0f, 0.0f, 0.0f );
				glVertex3fv( verts[1].v );
				glVertex3fv( verts[2].v );
				glVertex3fv( verts[6].v );
				glVertex3fv( verts[5].v );

				glEnd();

				glPopMatrix();


				if( displayMesh && data.mesh )
				{
					glDisable( GL_LIGHTING ); // disable it to render the vertex colors

					glEnable( GL_POLYGON_OFFSET_FILL );
					glPolygonOffset( 1.001f , .01f );

					renderMesh( data.mesh );
					if( data.cracker )
						renderCracks( data.cracker );

					glDisable( GL_POLYGON_OFFSET_FILL );
				}



				glDisable( GL_LIGHTING );
			}


			// blit screen
			SwapBuffers( window.mhDC );

        }
	}
}

void renderCracks( SurfaceCrackGenerator *cracker, float r, float g, float b )
{
	glLineWidth( 2.0f );
	glBegin( GL_LINES );
	glColor3f( r, g, b );
	// iterate over all edges
	for( std::vector<SurfaceCrackGenerator::Edge*>::iterator eit = cracker->edges.begin(); eit != cracker->edges.end(); ++eit )
	{
		SurfaceCrackGenerator::Edge *e = *eit;
		SurfaceCrackGenerator::Node *n1 = (*eit)->n1;
		SurfaceCrackGenerator::Node *n2 = (*eit)->n2;

		// draw boundary edges only
		if( !e->isBoundaryEdge() )
			continue;

		//glColor3f( n1->color.r, n1->color.g, n1->color.b );
		glNormal3f( n1->normal.x, n1->normal.y, n1->normal.z );
		glVertex3f( n1->position.x, n1->position.y, n1->position.z );
		//glColor3f( n2->color.r, n2->color.g, n2->color.b );
		glNormal3f( n2->normal.x, n2->normal.y, n2->normal.z );
		glVertex3f( n2->position.x, n2->position.y, n2->position.z );
	}
	glEnd();
}


//
// WinAPI message handler
//
LRESULT CALLBACK  glWinProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam )
{
	int    wmId, wmEvent;


	switch( uMsg )
	{
	case WM_CLOSE:								
		{
			PostQuitMessage(0);
			g_done = true;
			return 0;
		}break;
	case WM_KEYDOWN: // a key has been pressed
		//printf( "%i\n", wParam );
		switch( wParam )
		{
		case VK_ESCAPE:
			PostQuitMessage(0);
			g_done = true;
			break;
		case 0 :
			break;
		case VK_RETURN:
			{
				if( current->cracker )
				{
					// many passes with only 3 cracks per iteration
					printf( "generating crack pattern from stresses...\n" );
					for( size_t i=0; i<150; ++i )
					{
						printf( "iteration: %i\n", current->cracker->iterationCount );
						current->cracker->performIteration();
					}

					Mesh *m = current->cracker->getMesh();
					dk::io::exportToObjFile( m, "mountain2.029_cracked.obj" );
					delete m;
				}else
					printf( "unable to do crack simulation since cracker is not initialized\n" );
			}break;
		case VK_SPACE:
			{
				//printf( "iteration: %i\n", cracker->iterationCount );
				//cracker->performIteration();
			}break;
		case VK_UP :
			break;
		case VK_DOWN :
			break;
		case VK_LEFT :
			{
				if( fileSequenceIterator != fileSequence.begin() )
					--fileSequenceIterator;

				//loadMesh( *fileSequenceIterator, mesh );
			}break;
		case VK_RIGHT :
			{
				if( fileSequenceIterator != fileSequence.end() )
					++fileSequenceIterator;

				//loadMesh( *fileSequenceIterator, mesh );
			}break;
			/*
		case VK_NUMPAD4 :
			break;
		case VK_NUMPAD6 :
			break;
		case VK_NUMPAD5 :
			break;
		case VK_NUMPAD8 :
			break;
		case VK_NUMPAD9 :
			break;
		case VK_NUMPAD3 :
			break;
		case VK_NUMPAD1 :
			break;
		case VK_ADD :
			break;
		case VK_SUBTRACT :
			break;
		*/
		case 69 :// KEY : 'e'
			displayEigen = !displayEigen;
			break;
		case 70 :// KEY : 'f'
			break;
		case 82 :// KEY : 'r'
			break;
		case 68 :// KEY : 'd'
			break;
		case 86 :// KEY : 'v'
			break;
		case 87 :// KEY : 'w'
			displayWireframe = (displayWireframe+1)%3;
			break;
		case 71 :// KEY : 'g'
			break;
		case 80 :// KEY : 'p'
			{
				//Mesh *temp = mesh;
				//mesh = plane;
				//plane = temp;
			}break;
		case 77 :// KEY: 'm'
			{
				displayMesh = !displayMesh;
			}break;
		case 83 :// KEY: 's'
			renderMeshSwitch = !renderMeshSwitch;
			break;
		case 49 : // 1 key
			break;
		case 50 : // 2 key
			break;
		case 51 : // 3 key
			break;
		case 52 : // 4 key
			break;
		case 107 : // numpad + key
			// go over all mesh vertices and move them
			for( std::vector<Mesh::Vertex *>::iterator it = current->mesh->vertices.begin(); it != current->mesh->vertices.end(); ++it )
			{
				Mesh::Vertex *vert = *it;
				vert->position.z += 0.01f;
			}
			// recompute the vertex color from stress
			current->applyColorFromStress();
			break;
		case 109 : // numpad - key
			// go over all mesh vertices and move them
			for( std::vector<Mesh::Vertex *>::iterator it = current->mesh->vertices.begin(); it != current->mesh->vertices.end(); ++it )
			{
				Mesh::Vertex *vert = *it;
				vert->position.z -= 0.01f;
			}
			// recompute the vertex color from stress
			current->applyColorFromStress();
			break;
		default:
			return DefWindowProc( hWnd, uMsg, wParam, lParam );
			break;
		}
		break;

	case WM_MOUSEMOVE :
		{
			int xPos = ((int)(short)LOWORD(lParam)); 
			int yPos = ((int)(short)HIWORD(lParam)); 

			// if a mousebutton had been pressed
			if(((wParam == MK_LBUTTON)||(wParam == MK_MBUTTON)||(wParam == MK_RBUTTON))&&(GetKeyState( VK_LMENU )))
			{
				if( wParam == MK_LBUTTON )
				{
					// Alt + LMB => rotation

					//if( Deg2Rad(elevation) > RT_PI )
					//	twist   -=(xPos - lastMousePosX);
					//else
					//	twist   +=(xPos - lastMousePosX);

					viewManipulator.orbitView( xPos - lastMousePosX, yPos - lastMousePosY );
				}else
					if( wParam == MK_RBUTTON )
					{
						// Alt + RMB => move camera along lookat vector
						float x = xPos - lastMousePosX;
						viewManipulator.zoomView( -x*viewManipulator.getDistance()*0.005 );
					}else
					{// MMBUTTON
						float x = xPos - lastMousePosX;
						float y = yPos - lastMousePosY;
						viewManipulator.panView( x, -y );
					}
				//printf( "lookAt: %f   %f   %f\n", viewManipulator.lookAt.x, viewManipulator.lookAt.y, viewManipulator.lookAt.z );
				//printf( "azimuth/elevation/distance: %f   %f   %f\n", viewManipulator.azimuth, viewManipulator.elevation, viewManipulator.distance );

			}


			lastMousePosX = xPos;
			lastMousePosY = yPos;
		}break;

	default:
        return DefWindowProc( hWnd, uMsg, wParam, lParam );   
	}


	return DefWindowProc( hWnd, uMsg, wParam, lParam );
}

