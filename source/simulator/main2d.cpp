/*---------------------------------------------------------------------

Runs a fluid simulation and stores the particles for each frame

TODO: reflect simulationDomain parameter in the simualtion process

----------------------------------------------------------------------*/
#include "main.h"

#include <stdarg.h>

#include <iostream>
#include <fstream>

#ifdef WRITE_AVI
#include "win32/AVIWriter.h"
#endif


#include <math/Math.h>
#include "FluidSimulator2d.h"
#include "SandSimulator2d.h"
#include "ViscoElasticFluid2d.h"

#include "StaticSolids2d.h"
#include "VoxelMapReader.h"

// temp ----------------------------
#include "dk/Camera.h"
#include "dk/DebugNavigator.h"
#include "win32/GlWindow.h"
#include "dk/ColorGradient.h"

#include "nr3/nr3_util.h"



#ifdef FLUID_2D



using namespace dk;

bool                                   g_done = false;  // indicates finish of the message loop
bool                                      run = false;
bool                         displayRigidCells = true;
bool                         displayVelocities = true;
bool                          displayDebugCell = true;
bool                         displayCellStress = true;
int                                    debugCellI = 0;
int                                    debugCellJ = 0;

int                                 lastMousePosX = 0;  // position of the mouse within the last frame is needed for interaction
int                                 lastMousePosY = 0;  // position of the mouse within the last frame is needed for interaction
Camera                                         camera;  // camera which represents a view on the scene
DebugNavigator                        viewManipulator;  // the manipulator is used to manipulate the camera from some interaction inputs
ColorGradient                          stressGradient;


// the fluidsimulator which is used to run the simulation
//FluidSimulator2d simulator;
SandSimulator2d simulator;
//ViscoElasticFluid2d simulator;

LRESULT CALLBACK  glWinProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );  // event handling routine

// temp ----------------------------


#define NUMBER_OF_FRAMES  300
#define FRAMES_PER_SECOND  30

// helper functions for 2d fluid simulations
void                              setupSimulation( FluidSimulator2d &fluidSimulator, int sceneIndex );  // sets the initial state of the marker particles of the given fluidsimulator
void writeMarkerParticles( const std::vector<Particle2d> &particles, const char *filename_format, ... );  // writes the marker Particles out to a binary file

//
// entry point
//
int main(void)
{



	// setup the initial state which is only defined through the marker particles
	setupSimulation( simulator, 1 );
	//setupSimulation( simulator, 0 );
	//setupSimulation( simulator, 4 );// 4 -> masse auf solidcube leicht schräg

	
	//simulator.setElasticModulus( 10.0f );// good elastic behaviour
	//simulator.setPlasticYieldRate( 50.0f );
	//simulator.setElasticModulus( 2.75f ); // nice creeping behaviour!
	//simulator.setPlasticYieldRate( 50.0f );


/*
	int frameCount = 0;
	// now run the simulation
	while( frameCount < NUMBER_OF_FRAMES )
	{
		printf("computing frame %03i\n", frameCount);

		// perform one simulationstep
		simulator.advanceFrame( 1.0f / FRAMES_PER_SECOND );

		// write the positions of all marker particles into a file
		writeMarkerParticles( simulator.markerParticles, "water5.%03i.pcl", frameCount );

		++frameCount;
	}
return 0;
*/

// TEMP ---------------------------------
 	MSG msg;
	GLWindow window;
	viewManipulator.setCamera( &camera );
	window.createGLWindow( "test", 800, 600, 100, 100, 32, 0, glWinProc, NULL);
	window.show();

	// set the stress gradient to the typical engeneeringcolors
	//stressGradient.addSample( 0.0f, math::Color::Black() );
	//stressGradient.addSample( 1.0f, math::Color::White() );
	stressGradient.addSample( 0.0f, math::Color::From255( 0, 0, 143) );
	stressGradient.addSample( 0.11f, math::Color::From255( 0, 0, 255) );
	stressGradient.addSample( 0.36f, math::Color::From255( 0, 255, 255) );
	stressGradient.addSample( 0.62f, math::Color::From255( 255, 255, 0) );
	stressGradient.addSample( 0.86f, math::Color::From255( 255, 0, 0) );
	stressGradient.addSample( 1.0f, math::Color::From255( 119, 0, 0) );


#ifdef WRITE_AVI
	// setup aviwriter
	int frameSkip = 0;
	AVIWriter aviWriter;	// generator
	BYTE* bmBits;	// image buffer
	HRESULT hr;
	BITMAPINFOHEADER bih;
	int frameCount = 0;
	memset( &bih, 0, sizeof(BITMAPINFOHEADER) );
	bih.biSize = sizeof(BITMAPINFOHEADER);
	bih.biWidth = 800;
	bih.biHeight = 600;
	bih.biPlanes = 1;
	bih.biBitCount = 24;
	bih.biCompression = BI_RGB;
	//bih.biSizeImage;
	//bih.biXPelsPerMeter;
	//bih.biYPelsPerMeter;
	//bih.biClrUsed;
	//bih.biClrImportant;

	aviWriter.SetRate(30);
	aviWriter.SetBitmapHeader( &bih );	// get bitmap info out of the view
	hr=aviWriter.InitEngine();	// start engine
	if (FAILED(hr))
	{
		printf( "AVIWriter init error!\n" );
	}
	// reget
	LPBITMAPINFOHEADER lpbih=aviWriter.GetBitmapHeader(); // getting bitmap info
	// allocating memory for bmBits
	bmBits=new BYTE[3* lpbih->biWidth* lpbih->biHeight];	
#endif

	// Main message loop:
	while ( !g_done ) 
	{
		if( PeekMessage( &msg,NULL,0,0,PM_REMOVE) )
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}else
		{
			if( run )
			{
				//simulator.advanceFrame( 1.0f / FRAMES_PER_SECOND );
				//sim->advanceStep( 0.005f );
				//sim->advanceStep( 0.005f );
				//simulator.advanceStep( 0.001f );
				//simulator.advanceStep( 0.0001f );
				simulator.advanceStep( 0.001f );
			}
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

			// draw grid
			glPointSize( 1.2f );
			glBegin( GL_LINES );
			glColor3f( 0.9f, 0.9f, 0.9f );
			int numb = 50;
			for( int i=0; i<numb; ++i )
			{
				glVertex3f(0.0f, i*(1.0f/numb), 0.0f);
				glVertex3f(1.0f, i*(1.0f/numb), 0.0f);

				glVertex3f(i*(1.0f/numb), 0.0f, 0.0f);
				glVertex3f(i*(1.0f/numb), 1.0f, 0.0f);
			}
			glEnd();

			// draw boundaries
			glColor3f( 0.2f, 0.2f, 0.2f );
			glLineWidth( 1.2f );
			glBegin( GL_LINES );
			glVertex3f( 0.0f, 0.0f, 0.0f );
			glVertex3f( 1.0f, 0.0f, 0.0f );

			glVertex3f( 1.0f, 0.0f, 0.0f );
			glVertex3f( 1.0f, 1.0f, 0.0f );

			glVertex3f( 1.0f, 1.0f, 0.0f );
			glVertex3f( 0.0f, 1.0f, 0.0f );

			glVertex3f( 0.0f, 1.0f, 0.0f );
			glVertex3f( 0.0f, 0.0f, 0.0f );
			glEnd();

			// draw static solids
			for( size_t i = 0; i<simulator.getStaticSolidCount(); ++i )
			{
				// get the current one
				FluidSimulator2d::StaticSolid *solid = simulator.getStaticSolid( i );
				// check type
				if( dynamic_cast<Cube2d *>(solid) )
				{
					Cube2d *cube = dynamic_cast<Cube2d *>(solid);
					math::Vec2f minPoint = cube->getMinPoint();
					math::Vec2f maxPoint = cube->getMaxPoint();
					math::Vec2f dim = cube->getDimensions();
					
					// draw the cube
					glPushMatrix();
					math::Matrix44f cubeTM = cube->getTransformationMatrix();
					glMultMatrixf( cubeTM.ma );

					glLineWidth( 2.0f );
					glBegin( GL_LINES );
					glColor3f( 1.0f, 0.0f, 0.0f );
					glVertex3f( minPoint.x, minPoint.y, 0.0f );
					glVertex3f( minPoint.x + dim.x, minPoint.y, 0.0f );

					glVertex3f( minPoint.x + dim.x, minPoint.y, 0.0f );
					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, 0.0f );

					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, 0.0f );
					glVertex3f( minPoint.x, minPoint.y + dim.y, 0.0f );

					glVertex3f( minPoint.x, minPoint.y + dim.y, 0.0f );
					glVertex3f( minPoint.x, minPoint.y, 0.0f );
					glEnd();

					glPopMatrix();

					// temp
					math::Vec3f minPoint2( minPoint.x, minPoint.y, 0.0f );
					math::Vec3f maxPoint2( maxPoint.x, maxPoint.y, 0.0f );

					math::Vec3f pos1 = math::transform( minPoint2, cube->getTransformationMatrix() );
					math::Vec3f pos2 = math::transform( maxPoint2, cube->getTransformationMatrix() );
					math::Vec3f pos3 = math::transform( minPoint2+math::Vec3f(0.0f, dim.y, 0.0f), cube->getTransformationMatrix() );
					math::Vec3f pos4 = math::transform( minPoint2+math::Vec3f(dim.x, 0.0f, 0.0f), cube->getTransformationMatrix() );

					/*
					glPointSize( 10.0f );
					glBegin( GL_POINTS );
					glColor3f( 0.0f, 1.0f, 0.0f );
					glVertex3f( pos1.x, pos1.y, 0.0f );
					glVertex3f( pos2.x, pos2.y, 0.0f );
					glVertex3f( pos3.x, pos3.y, 0.0f );
					glVertex3f( pos4.x, pos4.y, 0.0f );
					glEnd();
					*/
				}else
				if( dynamic_cast<VoxelMap2d *>(solid) )
				{
					VoxelMap2d *vm = dynamic_cast<VoxelMap2d *>(solid);

					glBegin( GL_QUADS );
					glColor3f( 1.0f, 0.0f, 0.0f );
					for( size_t j = 0; j<vm->getResolutionY(); ++j )
						for( size_t i = 0; i<vm->getResolutionX(); ++i )
						{
							if( vm->getVoxelState(i, j) )
							{
								math::Vec2f offset = math::Vec2f( i*vm->getCellSize(), j*vm->getCellSize() );

								glVertex3f( offset.x, offset.y, 0.0f );
								glVertex3f( offset.x+vm->getCellSize(), offset.y, 0.0f );
								glVertex3f( offset.x+vm->getCellSize(), offset.y+vm->getCellSize(), 0.0f );
								glVertex3f( offset.x, offset.y+vm->getCellSize(), 0.0f );
							}
						}
					glEnd();
				}
			}
	
			// draw particles
			glPointSize( 1.2f );
			glBegin( GL_POINTS );
			glColor3f( 0.0f, 0.0f, 1.0f );
			for( std::vector<Particle2d>::iterator it=simulator.markerParticles.begin(); it != simulator.markerParticles.end(); ++it )
			{
				glColor3f( 0.0f, 0.0f, 1.0f );

				glVertex3f( (*it).position.x, (*it).position.y, 0.0f );
			}
			glEnd();

			//draw rigid grid cells
			if( displayRigidCells )
			{
				glEnable( GL_BLEND );
				glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
				glBegin( GL_QUADS );
				for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
				{
					glColor4f( 1.0f, 0.0f, 0.0f, 0.4f );

					if( (*it).second.rigid )
					{
						// compute lower left corner
						math::Vec2f offset = math::Vec2f( (*it).first.i*simulator.cellSize, (*it).first.j*simulator.cellSize );

						glVertex3f( offset.x, offset.y, 0.0f );
						glVertex3f( offset.x+simulator.cellSize, offset.y, 0.0f );
						glVertex3f( offset.x+simulator.cellSize, offset.y+simulator.cellSize, 0.0f );
						glVertex3f( offset.x, offset.y+simulator.cellSize, 0.0f );
					}
				}
				glEnd();
				glDisable( GL_BLEND );
			}

			// visualize signed distance
			if( 0 )
			{
				dk::ColorGradient gradient = dk::ColorGradient::BlackWhite();
				float maxDistance = 0.0f;

				for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
					if( fabs(it->second.signedDistance) > maxDistance )
						maxDistance = fabs(it->second.signedDistance);



				glEnable( GL_BLEND );
				glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
				glBegin( GL_QUADS );
				for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
				{
					float val = math::mapValueToRange( 0.0f, maxDistance, 0.0f, 1.0f, fabs(it->second.signedDistance) );

					glColor4fv( math::invert( gradient.getOutput( val ) ).v );

					math::Vec2f offset = math::Vec2f( (*it).first.i*simulator.cellSize, (*it).first.j*simulator.cellSize );

					glVertex3f( offset.x, offset.y, 0.0f );
					glVertex3f( offset.x+simulator.cellSize, offset.y, 0.0f );
					glVertex3f( offset.x+simulator.cellSize, offset.y+simulator.cellSize, 0.0f );
					glVertex3f( offset.x, offset.y+simulator.cellSize, 0.0f );
				}
				glEnd();
				glDisable( GL_BLEND );
			}


			float minimumEquivalentStress = 0.0f;
			float maximumEquivalentStress = 1.0f;

			float maximumPressure = 1.0f;

			// collect maximum stress
			for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
			{
				if( it->second.vonMisesEquivalentStress > maximumEquivalentStress )
				{
					maximumEquivalentStress = it->second.vonMisesEquivalentStress;
				}

				if( it->second.pressure > maximumPressure )
					maximumPressure = it->second.pressure;
			}

			//printf( "maximum equivalent stress: %f\n", maximumEquivalentStress );
			//draw stresses
			//if( displayCellStress )
			if( 0 )
			{
				//glEnable( GL_BLEND );
				//glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
				glBegin( GL_QUADS );
				for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
				{
					//glColor4f( (*it).second.meanStress * 1.0f, 0.0f, 0.0f, 0.4f );
					//glColor4f( 0.0f, (*it).second.shearStress * 1.0f, 0.0f, 0.4f );
					//glColor4fv( stressGradient( (*it).second.vonMisesEquivalentStress ).v );
					glColor4fv( stressGradient( math::mapValueToRange( minimumEquivalentStress, maximumEquivalentStress, 0.0f, 1.0f, (*it).second.vonMisesEquivalentStress ) ).v );
					//glColor4fv( stressGradient( math::mapValueToRange( 0.0f, maximumPressure, 0.0f, 1.0f, it->second.pressure ) ).v );

					//if( (*it).second.rigid )
					if((*it).second.type == FluidSimulator2d::Cell::Fluid)
					{
						// compute lower left corner
						math::Vec2f offset = math::Vec2f( (*it).first.i*simulator.cellSize, (*it).first.j*simulator.cellSize );

						glVertex3f( offset.x, offset.y, 0.0f );
						glVertex3f( offset.x+simulator.cellSize, offset.y, 0.0f );
						glVertex3f( offset.x+simulator.cellSize, offset.y+simulator.cellSize, 0.0f );
						glVertex3f( offset.x, offset.y+simulator.cellSize, 0.0f );
					}
					//glColor4f( (*it).second.meanStress * 1.0f, 0.0f, 0.0f, 0.4f );
				}
				glEnd();
				//glDisable( GL_BLEND );
			}


			// draw eigenvectors and eigenvalues
			if( 0 )
			{
				glColor3f( 1.0f, 1.0f, 0.0f );
				glBegin( GL_LINES );
				for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
				{
					FluidSimulator2d::Cell *cell = &it->second;

					// handle fluidcells only
					if( !cell->isFluidCell() )
						continue;

					// do eigenvector decomposition of strain tensor
					std::vector<float>                   eigenValues;
					std::vector< math::Vec2f >          eigenVectors;
					std::vector< std::vector<float> > f_eigenVectors;
					
					doEigenDecomposition( 2, 2, cell->stressTensor.ma, eigenValues, f_eigenVectors );

					for( size_t ei=0; ei<eigenValues.size(); ++ei )
						eigenValues[ei] = -eigenValues[ei];


					for( size_t ei=0; ei<f_eigenVectors.size(); ++ei )
						eigenVectors.push_back( math::Vec2f( f_eigenVectors[ei][0], f_eigenVectors[ei][1] ) );

					// find the greatest eigenvalue -> easy, since they come out sorted of the nr3 algorithm
					// in descending order
					float maxEigenvalue = eigenValues[0];
					math::Vec2f maxEigenvector = eigenVectors[0]*maxEigenvalue;

					float scndEigenvalue = eigenValues[1];
					math::Vec2f scndEigenvector = eigenVectors[1]*scndEigenvalue;

					/*
					for( size_t ei=0; ei<cell->eigenValues.size(); ++ei )
					{
						if( fabs(cell->eigenValues[ei]) > cell->maxEigenvalue )
						{
							cell->maxEigenvalue = fabs(cell->eigenValues[ei]);
							cell->maxEigenvaluesEigenvector = cell->eigenVectors[ei];
						}
					}

					if( fabs(cell->maxEigenvalue) > grid->maxEigenvalueOfAllCells )
						grid->maxEigenvalueOfAllCells = fabs(cell->maxEigenvalue);
					*/

					if( maxEigenvalue > 0.0f )
						continue;

					if( cell->signedDistance > 0.01f )
						continue;

					if( fabs(maxEigenvalue) < 1.5f )
						continue;

					// color the eigenvector depending on the sign of the eigenvalue


					// compute center
					math::Vec2f offset = math::Vec2f( (it->first.i+0.5f)*simulator.cellSize, (it->first.j+0.5f)*simulator.cellSize );



					// draw eigen vector from cellcenter					
					glVertex3f( offset.x, offset.y, 0.0f );
					glVertex3f( offset.x+maxEigenvector.x*0.1f, offset.y+maxEigenvector.y*0.1f, 0.0f );

					glVertex3f( offset.x, offset.y, 0.0f );
					glVertex3f( offset.x+scndEigenvector.x*0.1f, offset.y+scndEigenvector.y*0.1f, 0.0f );


					// draw velcity vector from cellcenter					
					//glVertex3f( offset.x, offset.y, 0.0f );
					//glVertex3f( offset.x+cell->velocity.x*0.1f, offset.y+cell->velocity.y*0.1f, 0.0f );
				}
				glEnd();

			}



			// draw velocities
			//if( displayVelocities )
			if( 1 )
			{
				glColor3f( .3f, .3f, 0.0f );
				glBegin( GL_LINES );
				for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
				{
					//if( (*it).second.type != FluidSimulator2d::Cell::Solid )
					//	continue;

					// compute lower left corner
					math::Vec2f offset = math::Vec2f( (*it).first.i*simulator.cellSize, (*it).first.j*simulator.cellSize );

					// compute x-velocity
					glVertex3f( offset.x, offset.y+simulator.cellSize/2.0f, 0.0f );
					glVertex3f( offset.x+(*it).second.velocity.x*0.1f, offset.y+simulator.cellSize/2.0f, 0.0f );

					glVertex3f( offset.x+simulator.cellSize/2.0f, offset.y, 0.0f );
					glVertex3f( offset.x+simulator.cellSize/2.0f, offset.y+(*it).second.velocity.y*0.1f, 0.0f );
				}
				glEnd();
			}

			//if( displayDebugCell )
			if( 1 )
			{
				glEnable( GL_BLEND );
				glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
				glBegin( GL_QUADS );
				// compute lower left corner
				math::Vec2f offset = math::Vec2f( debugCellI*simulator.cellSize, debugCellJ*simulator.cellSize );

				glVertex3f( offset.x, offset.y, 0.0f );
				glVertex3f( offset.x+simulator.cellSize, offset.y, 0.0f );
				glVertex3f( offset.x+simulator.cellSize, offset.y+simulator.cellSize, 0.0f );
				glVertex3f( offset.x, offset.y+simulator.cellSize, 0.0f );
				glEnd();
				glDisable( GL_BLEND );

				FluidSimulator2d::Cell *cell = simulator.getCell( debugCellI, debugCellJ );
				if( cell )
				{
					printf( "signed distance : %f\n", cell->signedDistance );
					printf( "velocity : %f   %f\n", cell->velocity.x, cell->velocity.y );
				}
			}


			// blit screen
			SwapBuffers( window.mhDC );

			#ifdef WRITE_AVI
			if( run )
			{
				//if( frameCount >= frameSkip )
				//{
				//	frameCount = 0;

					// Read the pixels from opengl and use the aviwriter to add an image to the final video
					glReadPixels(0,0,lpbih->biWidth,lpbih->biHeight, GL_BGR_EXT,GL_UNSIGNED_BYTE,bmBits);
					hr=aviWriter.AddFrame(bmBits);
					if (FAILED(hr))
					{
						printf("error writing image to avi!\n");
					}

				//}else
				//	frameCount++;
			}
			#endif
        }
	}

	#ifdef WRITE_AVI
	aviWriter.ReleaseEngine(); // releasing ressources
	delete[] bmBits;	// release ressources
	#endif


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
				run = !run;
			}break;
		case VK_SPACE:
			{
				//sim->advanceFrame( 1.0f / FRAMES_PER_SECOND );
				simulator.advanceStep( 0.001f );
				/*
				stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = sim->gridCells.find( FluidSimulator2d::Cell::Coordinate( debugCellI, debugCellJ ) );

				if( it != sim->gridCells.end() )
				{
					FluidSimulator2d::Cell *debugCell = &((*it).second);

					printf( "Cell( %i, %i )\n", debugCellI, debugCellJ );
					printf( "velocity  %f       %f\n", debugCell->velocity.x, debugCell->velocity.y );
				}
				*/
			}break;
		case VK_UP :
			++debugCellJ;
			break;
		case VK_DOWN :
			--debugCellJ;
			break;
		case VK_LEFT :
			--debugCellI;
			break;
		case VK_RIGHT :
			++debugCellI;
			break;
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
		case 70 :// KEY : 'f'
			break;
		case 82 :// KEY : 'r'
			displayRigidCells=!displayRigidCells;
			break;
		case 68 :// KEY : 'd'
			break;
		case 86 :// KEY : 'v'
			displayVelocities = !displayVelocities;
			break;
		case 71 :// KEY : 'g'
			break;
		case 83 :// KEY: 's'
			displayCellStress = !displayCellStress;
			break;
		case 49 : // 1 key
			break;
		case 50 : // 2 key
			break;
		case 51 : // 3 key
			break;
		case 52 : // 4 key
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
			}

			lastMousePosX = xPos;
			lastMousePosY = yPos;
		}break;

	default:
        return DefWindowProc( hWnd, uMsg, wParam, lParam );   
	}


	return DefWindowProc( hWnd, uMsg, wParam, lParam );
}






//
// sets the initial state of the marker particles
//
void setupSimulation( FluidSimulator2d &fluidSimulator, int sceneIndex )
{
	// clear gridcells and marker particles
	fluidSimulator.reset();

	// the marker particles are defined depending on the given scene
	switch( sceneIndex )
	{
	case 0:  // simple column with a sphere cut
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			//fluidSimulator.setCellSize( 0.05f );
			//fluidSimulator.setCellSize( 0.001f );
			fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setCellSize( 0.001f );
			//fluidSimulator.setDeltaTimeRange( 0.001f, 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec2f( 0.0f, 0.0f ), math::Vec2f( 1.0f, 1.0f ) );

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle2d p;
				//float height = 0.9f;
				float height = 0.57f;
				//float width = 0.27f;
				float width = 0.20f;


				p.position = math::Vec2f( math::g_randomNumber(), math::g_randomNumber() );
				if( (p.position.y < height)&&(p.position.x < width) )
				{
					// sphere
					//math::Vec2f sphereCenter( 0.35f, 0.27f );
					math::Vec2f sphereCenter( 0.25f, 0.27f );
					float distance = (sphereCenter - p.position).getSquaredLength();
					if( !(distance < 0.0225f) ) //radius: 0.15
						fluidSimulator.markerParticles.push_back( p );
				}

				// TODO: align particles on the border of the column so that we have a flat surface
			}
		}break;
	case 1: // sphere
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			//fluidSimulator.setCellSize( 0.05f );
			fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setCellSize( 0.001f );
			//fluidSimulator.setDeltaTimeRange( 0.001f, 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec2f( 0.0f, 0.0f ), math::Vec2f( 1.0f, 1.0f ) );

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle2d p;
				float height = 0.9f;
				float width = 0.2f;


				p.position = math::Vec2f( math::g_randomNumber(), math::g_randomNumber() );
				// sphere
				//math::Vec2f sphereCenter( 0.35f, 0.27f );
				math::Vec2f sphereCenter( 0.5f, 0.5f );
				float distance = (sphereCenter - p.position).getSquaredLength();
				if( distance < 0.0225f ) //radius: 0.15
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
			/*
			size_t c = fluidSimulator.markerParticles.size();
			while( fluidSimulator.markerParticles.size() < c + 50000 )
			{
				Particle2d p;
				float height = 0.2f;
				float width = 1.0f;


				p.position = math::Vec2f( math::g_randomNumber(), math::g_randomNumber() );
				if( (p.position.y < height)&&(p.position.x < width) )
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
			*/
		}break;
	case 2: // cube
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			//fluidSimulator.setCellSize( 0.05f );
			//fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setCellSize( 0.001f );
			fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setDeltaTimeRange( 0.001f, 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec2f( 0.0f, 0.0f ), math::Vec2f( 1.0f, 1.0f ) );

			// add obstacle
			/*
			FluidSimulator2d::StaticSolid *sCube = new Cube2d( 0.0f, 0.0f, 0.5f, 0.2f );
			sCube->setTranslation( .8f, 0.3f );
			//sCube->setTranslation( .5f, 0.0f );
			sCube->setRotation( -15.0f );
			fluidSimulator.addStaticSolid( sCube );
			*/

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle2d p;
				float height = 0.15f;
				float width = 0.5f;


				p.position = math::Vec2f( math::g_randomNumber(), math::g_randomNumber() );
				// cube
				math::Vec2f cubeCenter( 0.5f, 0.5f );
				if( (fabs( p.position.x - cubeCenter.x ) < width / 2.0f)&&(fabs( p.position.y - cubeCenter.y ) < height / 2.0f) )
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
		}break;
	case 3: // stretch
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			//fluidSimulator.setCellSize( 0.05f );
			//fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setCellSize( 0.001f );
			fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setDeltaTimeRange( 0.001f, 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec2f( 0.0f, 0.0f ), math::Vec2f( 1.0f, 1.0f ) );

			// add obstacle
			/*
			FluidSimulator2d::StaticSolid *sCube = new Cube2d( 0.0f, 0.0f, 0.5f, 0.2f );
			sCube->setTranslation( .8f, 0.3f );
			//sCube->setTranslation( .5f, 0.0f );
			sCube->setRotation( -15.0f );
			fluidSimulator.addStaticSolid( sCube );
			*/

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle2d p;
				float height = 0.15f;
				float width = 0.5f;


				p.position = math::Vec2f( math::g_randomNumber(), math::g_randomNumber() );
				// cube
				math::Vec2f cubeCenter( 0.5f, 0.5f );
				if( (fabs( p.position.x - cubeCenter.x ) < width / 2.0f)&&(fabs( p.position.y - cubeCenter.y ) < height / 2.0f) )
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
		}break;
	case 4:
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			//fluidSimulator.setCellSize( 0.05f );
			//fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setCellSize( 0.001f );
			fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setDeltaTimeRange( 0.001f, 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec2f( 0.0f, 0.0f ), math::Vec2f( 1.0f, 1.0f ) );


			// add obstacle
			FluidSimulator2d::StaticSolid *sCube = new Cube2d( 0.0f, 0.0f, 0.5f, 0.2f );
			sCube->setTranslation( .8f, 0.3f );
			sCube->setRotation( -15.0f );
			//fluidSimulator.addStaticSolid( sCube );


			VoxelMap2d *vm = readVoxelMap2d( "2d_1.bmp" );
			if( vm )
			{
				vm->setCellSize( fluidSimulator.getCellSize() );
				fluidSimulator.addStaticSolid( vm );
			}

			math::Matrix44f test = sCube->getTransformationMatrix();
			test.translate( 0.0f, 0.18f, 0.0f );

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle2d p;
				float height = 0.15f;
				float width = 0.5f;


				p.position = math::Vec2f( (math::g_randomNumber() - 0.5f)*width, (math::g_randomNumber() - 0.5f)*height );

				p.velocity = math::transform( math::Vec2f( 0.0f, 0.0f), math::Matrix22f::RotationMatrix(math::degToRad(-15.0f)) );

				// cube
				if( (fabs( p.position.x ) < width / 2.0f)&&(fabs( p.position.y ) < height / 2.0f) )
				{
					// transform particle
					p.position = math::transform( p.position, test );
					fluidSimulator.markerParticles.push_back( p );
				}

				// TODO: align particles on the border of the column so that we have a flat surface
			}

			fluidSimulator.updateGrid();
			fluidSimulator.transferParticleVelocitiesToGrid();
		}break;
	};
}


//
// writes the marker Particles out to a binary file
//
void writeMarkerParticles( const std::vector<Particle2d> &particles, const char *filenameFormat, ... )
{
	char filename[2048];

	// assemble filename from given format and arguments
	va_list argumentList;
	va_start( argumentList, filenameFormat );
	vsprintf_s( filename, 2048, filenameFormat, argumentList);
	va_end(argumentList);

	// createopen the file
	std::ofstream file;
	file.open( filename, std::ios::out | std::ios::binary );


	// file creation not successfull?
	if( !file )
		// quit
		return;

	// write into the file

	// number of particles
	int particleNum = (int)particles.size();
	file.write( (char *) &particleNum, sizeof(int) );

	// now write the position of each particle
	for( unsigned int i=0; i<(unsigned int)particleNum; ++i )
	{
		float x = particles[i].position.x;
		float y = particles[i].position.y;
		float z = 0.0f;

		file.write( (char *) &x, sizeof(float) );
		file.write( (char *) &y, sizeof(float) );
		file.write( (char *) &z, sizeof(float) );
	}

	// done
	file.close();
}


#endif // FLUID_2D