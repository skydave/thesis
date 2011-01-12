/*---------------------------------------------------------------------

Runs a fluid simulation and stores the particles for each frame

TODO: reflect simulationDomain parameter in the simualtion process

----------------------------------------------------------------------*/
#include <windows.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>

#include <tchar.h>
#include "util/zip.h"
#include "util/MemoryFile.h"
#include "util/StringManip.h"
#include "util/PathInfo.h"


#include <math/Math.h>
#include "FluidSimulator3d.h"
#include "SandSimulator3d.h"
#include "ViscoElasticFluid3d.h"

#include "StaticSolids3d.h"
#include "VoxelMapReader.h"

// temp ----------------------------
#include "dk/Camera.h"
#include "dk/DebugNavigator.h"
#include "win32/GlWindow.h"
#include "win32/OpenGLTools.h"
#include "dk/ColorGradient.h"

#include "main.h"

#ifdef FLUID_3D

#ifdef WRITE_AVI
#include "win32/AVIWriter.h"
#endif


using namespace dk;

bool                                        g_done = false;  // indicates finish of the message loop
bool                                           run = false;
int                                         frameCount = 0;
int                                          stepCount = 0;  // counts the number of simulationsteps done
bool                             displayRigidCells = true;
bool                              displayVelocities = true;
bool                               displayDebugCell = true;
int                                         debugCellI = 0;
int                                         debugCellJ = 0;
int                                         debugCellK = 0;

int                                      lastMousePosX = 0;  // position of the mouse within the last frame is needed for interaction
int                                      lastMousePosY = 0;  // position of the mouse within the last frame is needed for interaction
Camera                                              camera;  // camera which represents a view on the scene
DebugNavigator                             viewManipulator;  // the manipulator is used to manipulate the camera from some interaction inputs
ColorGradient     stressGradient = ColorGradient::Stress();


// the fluidsimulator which is used to run the simulation
//FluidSimulator3d simulator;
//SandSimulator3d simulator;
ViscoElasticFluid3d simulator;

LRESULT CALLBACK  glWinProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );  // event handling routine
void drawCell( math::Vec3f offset, math::Vec3f offset2 );
// temp ----------------------------



#define NUMBER_OF_FRAMES  1000
#define FRAMES_PER_SECOND  30

// helper functions for 3d fluid simulation
void                                setupSimulation( FluidSimulator3d &fluidSimulator, int sceneIndex );  // sets the initial state of the marker particles of the given fluidsimulator
void writeMarkerParticles( const std::vector<Particle3d> &particles, const char *filename_format, ... );  // writes the marker Particles out to a binary file
void                                                 writeGridValues( const char *filenameFormat, ... );  // writes the gridvalues
void                                             writeStateDataToZip( const char *filenameFormat, ... );  // this function will write the grid data and the particle positions into a zip file

//
// entry point
//
int main(void)
{


	// setup the initial state which is only defined through the marker particles

	//setupSimulation( simulator, 0 ); // rectangular column with a sphere cut
	//setupSimulation( simulator, 1 );
	//setupSimulation( simulator, 2 );
	//setupSimulation( simulator, 3 ); // mountain1
	setupSimulation( simulator, 4 ); // mountain2


	// now run the simulation
	while( frameCount < NUMBER_OF_FRAMES )
	{
		printf("computing frame %03i\n", frameCount);

		// perform one simulationstep
		simulator.advanceFrame( 1.0f / FRAMES_PER_SECOND );

		// write the positions of all marker particles into a file
		//writeStateDataToZip( "sand3d_1\\sand3d_1.%03i", frameCount );
		//writeStateDataToZip( "mountain1\\mountain1.%03i", frameCount);
		writeStateDataToZip( "mountain2\\mountain2.%03i", frameCount);

		++frameCount;
	}
	return 0;



	// TEMP ---------------------------------
	MSG msg;
	GLWindow window;
	viewManipulator.setCamera( &camera );
	window.createGLWindow( "tset", 800, 600, 100, 100, 32, 0, glWinProc, NULL);
	window.show();


#ifdef WRITE_AVI
	// setup aviwriter
	int frameSkip = 0;
	AVIWriter aviWriter;	// generator
	BYTE* bmBits;	// image buffer
	HRESULT hr;
	BITMAPINFOHEADER bih;
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
				// write the values of the gridcells into a file
				//writeGridValues( "visc1.%03i.grid", stepCount );
				// write the positions of all marker particles into a file
				//writeMarkerParticles( simulator.markerParticles, "visc1.%03i.pcl", stepCount );
				//writeStateDataToZip( "visc1.%03i", stepCount);

				if( stepCount < 1000 )
					//writeStateDataToZip( "sand3d_1\\sand3d_1.%03i", stepCount);
					//writeStateDataToZip( "mountain1\\mountain1.%03i", stepCount);
					writeStateDataToZip( "mountain2\\mountain2.%03i", stepCount);

				//sim->advanceFrame( 1.0f / FRAMES_PER_SECOND );
				//sim->advanceStep( 0.005f );
				//sim->advanceStep( 0.005f );
				simulator.advanceStep( 0.001f );
				//sim->advanceStep( 0.0001f );
				++stepCount;
			}
			// draw the scene -----------------------
			glClearColor( .5f, .5f, .5f, 1.0f );
			glClear( GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

			glViewport( (GLsizei)0.0f, (GLsizei)0.0f, (GLsizei)window.getWidth(), (GLsizei)window.getHeight() );

			glMatrixMode( GL_PROJECTION );
			glLoadMatrixf( camera.getProjectionMatrix().ma );

			glMatrixMode( GL_MODELVIEW );
			// the worldToEyeMatrix is computed by the OrbitManipulator which is coupled with the mouse input
			glLoadMatrixf( (GLfloat *)camera.getViewMatrix().ma );

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
				FluidSimulator3d::StaticSolid *solid = simulator.getStaticSolid( i );
				// check type
				if( dynamic_cast<Cube3d *>(solid) )
				{
					Cube3d *cube = dynamic_cast<Cube3d *>(solid);
					math::Vec3f minPoint = cube->getMinPoint();
					math::Vec3f maxPoint = cube->getMaxPoint();
					math::Vec3f dim = cube->getDimensions();
					
					// draw the cube
					glPushMatrix();
					math::Matrix44f cubeTM = cube->getTransformationMatrix();
					glMultMatrixf( cubeTM.ma );

					glLineWidth( 2.0f );
					glBegin( GL_LINES );
					glColor3f( 1.0f, 0.0f, 0.0f );
					glVertex3f( minPoint.x, minPoint.y, minPoint.z );
					glVertex3f( minPoint.x + dim.x, minPoint.y, minPoint.z );

					glVertex3f( minPoint.x + dim.x, minPoint.y, minPoint.z );
					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, minPoint.z );

					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, minPoint.z );
					glVertex3f( minPoint.x, minPoint.y + dim.y, minPoint.z );

					glVertex3f( minPoint.x, minPoint.y + dim.y, minPoint.z );
					glVertex3f( minPoint.x, minPoint.y, minPoint.z );

					glVertex3f( minPoint.x, minPoint.y, maxPoint.z );
					glVertex3f( minPoint.x + dim.x, minPoint.y, maxPoint.z );

					glVertex3f( minPoint.x + dim.x, minPoint.y, maxPoint.z );
					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, maxPoint.z );

					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, maxPoint.z );
					glVertex3f( minPoint.x, minPoint.y + dim.y, maxPoint.z );

					glVertex3f( minPoint.x, minPoint.y + dim.y, maxPoint.z );
					glVertex3f( minPoint.x, minPoint.y, maxPoint.z );

					glVertex3f( minPoint.x, minPoint.y, minPoint.z );
					glVertex3f( minPoint.x, minPoint.y, minPoint.z + dim.z );

					glVertex3f( minPoint.x, minPoint.y + dim.y, minPoint.z );
					glVertex3f( minPoint.x, minPoint.y + dim.y, minPoint.z + dim.z );

					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, minPoint.z );
					glVertex3f( minPoint.x + dim.x, minPoint.y + dim.y, minPoint.z + dim.z );

					glVertex3f( minPoint.x + dim.x, minPoint.y, minPoint.z );
					glVertex3f( minPoint.x + dim.x, minPoint.y, minPoint.z + dim.z );
					glEnd();

					glPopMatrix();

					// temp
					math::Vec3f pos1 = math::transform( minPoint, cube->getTransformationMatrix() );
					math::Vec3f pos2 = math::transform( maxPoint, cube->getTransformationMatrix() );
					math::Vec3f pos3 = math::transform( minPoint+math::Vec3f(0.0f, dim.y, 0.0f), cube->getTransformationMatrix() );
					math::Vec3f pos4 = math::transform( minPoint+math::Vec3f(dim.x, 0.0f, 0.0f), cube->getTransformationMatrix() );
					math::Vec3f pos5 = math::transform( minPoint+math::Vec3f(dim.x, dim.y, 0.0f), cube->getTransformationMatrix() );

					/*
					glPointSize( 10.0f );
					glBegin( GL_POINTS );
					glColor3f( 0.0f, 1.0f, 0.0f );
					glVertex3f( pos1.x, pos1.y, pos1.z );
					glVertex3f( pos2.x, pos2.y, pos2.z );
					glVertex3f( pos3.x, pos3.y, pos3.z );
					glVertex3f( pos4.x, pos4.y, pos4.z );
					glVertex3f( pos5.x, pos5.y, pos5.z );
					glEnd();
					*/
				}else
				// check type
				if( dynamic_cast<VoxelMap3d *>(solid) )
				{
					// draw voxelmap
					VoxelMap3d *vm = dynamic_cast<VoxelMap3d *>(solid);


					/*
					for( std::vector<VoxelMap3d::Check>::iterator it = vm->checks.begin(); it != vm->checks.end(); ++it )
					{
						if( (*it).color.z != 1.0f )
							continue;

						glColor3fv( (*it).color.v );
						glVertex3fv( (*it).cellCenter.v );
					}
					*/



					
					glEnable( GL_DEPTH_TEST );

					dk::Mesh *associatedMesh = vm->getAssociatedMesh();
					// if there is an associated mesh attached to this voxelmap
					if( associatedMesh )
					{
						// render this mesh
						glEnable( GL_LIGHTING );
						headLight();
						// draw the associated mesh
						renderMesh( associatedMesh );
						glDisable( GL_LIGHTING );
					}else
					{
						// render the voxels by splatting points into the center of each voxel
						glPointSize( 5.2f );
						glBegin( GL_POINTS );
						glColor3f( 0.8f, 0.0f, 0.0f );
						for( size_t k=0; k<vm->getResolutionZ(); ++k )
							for( size_t j=0; j<vm->getResolutionY(); ++j )
								for( size_t i=0; i<vm->getResolutionX(); ++i )
								{
									math::Vec3f cellCenter = vm->getVoxelCenterInLocalSpace( i, j, k );

									if( vm->getVoxelState( i, j, k ) )
										glVertex3fv( cellCenter.v );
								}
						glEnd();
					}

					glDisable( GL_DEPTH_TEST );

				}

			}

			// draw particles
			glEnable( GL_DEPTH_TEST );
			glPointSize( 2.2f );
			glBegin( GL_POINTS );
			glColor3f( 0.0f, 0.0f, 1.0f );
			for( std::vector<Particle3d>::iterator it=simulator.markerParticles.begin(); it != simulator.markerParticles.end(); ++it )
			{
				glColor3f( 0.0f, 0.0f, 1.0f );

				glVertex3f( (*it).position.x, (*it).position.y, (*it).position.z );
			}
			glEnd();
			glDisable( GL_DEPTH_TEST );


			//draw rigid grid cells
			if( displayRigidCells )
			{
				glEnable( GL_DEPTH_TEST );
				glPointSize( 5.2f );
				glBegin( GL_POINTS );
				glColor3f( 1.0f, 0.0f, 0.0f );
				for( size_t k=0; k<100; ++k )
					for( size_t j=0; j<100; ++j )
						for( size_t i=0; i<100; ++i )
						{
							FluidSimulator3d::Cell *cell = simulator.getCell( i, j, k );

							if( !cell )
								continue;

							math::Vec3f cellCenter = cell->center;

							if( cell->rigid )
								glVertex3fv( cellCenter.v );
						}
				glEnd();
				glDisable( GL_DEPTH_TEST );
			/*
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
			*/
			}


			/*
			float minimumEquivalentStress = 0.0f;
			float maximumEquivalentStress = 1.0f;

			// collect maximum stress
			for( stdext::hash_map<FluidSimulator3d::Cell::Coordinate, FluidSimulator3d::Cell, FluidSimulator3d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
			{
				if( (*it).second.vonMisesEquivalentStress > maximumEquivalentStress )
				{
					maximumEquivalentStress = (*it).second.vonMisesEquivalentStress;
				}
			}
			*/

			//printf( "maximum equivalent stress: %f\n", maximumEquivalentStress );


			/*
			//draw stresses
			//if( displayRigidCells )
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
			*/

			/*
			// draw velocities
			if( displayVelocities )
			{
				glColor3f( 1.0f, 1.0f, 0.0f );
				glBegin( GL_LINES );
				for( stdext::hash_map<FluidSimulator2d::Cell::Coordinate, FluidSimulator2d::Cell, FluidSimulator2d::hash_compare>::iterator it = simulator.gridCells.begin(); it != simulator.gridCells.end(); ++it )
				{
					if( (*it).second.type != FluidSimulator2d::Cell::Solid )
						continue;

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
			*/


			if( displayDebugCell )
			{
				glColor3f( 0.0f, 0.0f, 1.0f );
				glEnable( GL_BLEND );
				glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
				glBegin( GL_QUADS );
				// compute lower left corner
				math::Vec3f offset = math::Vec3f( debugCellI*simulator.cellSize, debugCellJ*simulator.cellSize, debugCellK*simulator.cellSize );
				math::Vec3f offset2 = math::Vec3f( (debugCellI+1)*simulator.cellSize, (debugCellJ+1)*simulator.cellSize, (debugCellK+1)*simulator.cellSize );

				drawCell( offset, offset2 );

				glDisable( GL_BLEND );

			}



			// blit screen
			SwapBuffers( window.mhDC );

#ifdef WRITE_AVI
			if( run )
			{
				if( frameCount >= frameSkip )
				{
					frameCount = 0;

					// Read the pixels from opengl and use the aviwriter to add an image to the final video
					glReadPixels(0,0,lpbih->biWidth,lpbih->biHeight, GL_BGR_EXT,GL_UNSIGNED_BYTE,bmBits);
					hr=aviWriter.AddFrame(bmBits);
					if (FAILED(hr))
					{
						printf("error writing image to avi!\n");
					}
				}else
					frameCount++;
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
//
//
void printCellDebugInfo( FluidSimulator3d::Cell *cell )
{
	//printf( "strain:\n" );
	//printf( "%04f  %04f  %04f\n", cell->strainTensor.m[0][0], cell->strainTensor.m[0][1], cell->strainTensor.m[0][2] );
	//printf( "%04f  %04f  %04f\n", cell->strainTensor.m[1][0], cell->strainTensor.m[1][1], cell->strainTensor.m[1][2] );
	//printf( "%04f  %04f  %04f\n", cell->strainTensor.m[2][0], cell->strainTensor.m[2][1], cell->strainTensor.m[2][2] );
	//printf( "strain divergence:\n" );
	//printf( "%04f  %04f   %04f\n", cell->tempVector.x, cell->tempVector.y, cell->tempVector.z );

	printf( "shear stress %04f    meanstress %04f\n", cell->shearStress, cell->meanStress );

	if( cell->rigid )
		printf( "cell is rigid\n" );
}

//
//
//
void drawCell( math::Vec3f offset, math::Vec3f offset2 )
{

	glBegin( GL_LINES );

	glVertex3f( offset.x, offset.y, offset.z );
	glVertex3f( offset2.x, offset.y, offset.z );

	glVertex3f( offset2.x, offset.y, offset.z );
	glVertex3f( offset2.x, offset2.y, offset.z );

	glVertex3f( offset2.x, offset2.y, offset.z );
	glVertex3f( offset.x, offset2.y, offset.z );

	glVertex3f( offset.x, offset2.y, offset.z );
	glVertex3f( offset.x, offset.y, offset.z );


	glVertex3f( offset.x, offset.y, offset2.z );
	glVertex3f( offset2.x, offset.y, offset2.z );

	glVertex3f( offset2.x, offset.y, offset2.z );
	glVertex3f( offset2.x, offset2.y, offset2.z );

	glVertex3f( offset2.x, offset2.y, offset2.z );
	glVertex3f( offset.x, offset2.y, offset2.z );

	glVertex3f( offset.x, offset2.y, offset2.z );
	glVertex3f( offset.x, offset.y, offset2.z );

	//glVertex3f( offset.x, offset.y, offset2.z );

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
				run = !run;
			}break;
		case VK_SPACE:
			{
				//sim->advanceFrame( 1.0f / FRAMES_PER_SECOND );
				simulator.advanceStep( 0.001f );
				++stepCount;

				FluidSimulator3d::Cell *cell = simulator.getCell( debugCellI, debugCellJ, debugCellK );
				if( cell )
					printCellDebugInfo( cell );
			}break;
		case VK_UP :
			break;
		case VK_DOWN :
			break;
		case VK_LEFT :
			break;
		case VK_RIGHT :
			break;
		case VK_NUMPAD4 :
			{
				--debugCellI;
				FluidSimulator3d::Cell *cell = simulator.getCell( debugCellI, debugCellJ, debugCellK );
				if( cell )
					printCellDebugInfo( cell );
			}break;
		case VK_NUMPAD6 :
			{
				++debugCellI;
				FluidSimulator3d::Cell *cell = simulator.getCell( debugCellI, debugCellJ, debugCellK );
				if( cell )
					printCellDebugInfo( cell );
			}break;
		case VK_NUMPAD5 :
			{
				--debugCellJ;
				FluidSimulator3d::Cell *cell = simulator.getCell( debugCellI, debugCellJ, debugCellK );
				if( cell )
					printCellDebugInfo( cell );
			}break;
		case VK_NUMPAD8 :
			{
				++debugCellJ;
				FluidSimulator3d::Cell *cell = simulator.getCell( debugCellI, debugCellJ, debugCellK );
				if( cell )
					printCellDebugInfo( cell );
			}break;
		case VK_NUMPAD9 :
			{
				++debugCellK;
				FluidSimulator3d::Cell *cell = simulator.getCell( debugCellI, debugCellJ, debugCellK );
				if( cell )
					printCellDebugInfo( cell );
			}break;
		case VK_NUMPAD3 :
			{
				--debugCellK;
				FluidSimulator3d::Cell *cell = simulator.getCell( debugCellI, debugCellJ, debugCellK );
				if( cell )
					printCellDebugInfo( cell );
			}break;
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
		case 87 :// KEY : 'w'
			// write the values of the gridcells into a file
			//writeGridValues( "visc1.%03i.grid", stepCount );
			// write the positions of all marker particles into a file
			//writeMarkerParticles( simulator.markerParticles, "visc1.%03i.pcl", stepCount );

			for( size_t i = 0; i<simulator.getStaticSolidCount(); ++i )
			{
				// get the current one
				FluidSimulator3d::StaticSolid *solid = simulator.getStaticSolid( i );
				// check type
				if( dynamic_cast<VoxelMap3d *>(solid) )
				{
					size_t count = 0;
					// draw voxelmap
					VoxelMap3d *vm = dynamic_cast<VoxelMap3d *>(solid);
					for( size_t k=0; k<vm->getResolutionZ(); ++k )
						for( size_t j=0; j<vm->getResolutionY(); ++j )
							for( size_t i=0; i<vm->getResolutionX(); ++i )
							{
								math::Vec3f cellCenter( i*simulator.getCellSize()+simulator.getCellSize()*0.5f, j*simulator.getCellSize()+simulator.getCellSize()*0.5f, k*simulator.getCellSize()+simulator.getCellSize()*0.5f );

								//if( vm->occupies( cellCenter ) )
								//	glVertex3fv( cellCenter.v );

								if( vm->voxelData2[ k*vm->getResolutionX()*vm->getResolutionY() + j*vm->getResolutionX() + i ] )
								{
									count++;
								}
							}
					printf( "%i\n", count );
				}
			}
			break;
		case 71 :// KEY : 'g'
		case 80:// KEY : 'p'
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
void setupSimulation( FluidSimulator3d &fluidSimulator, int sceneIndex )
{
	// clear gridcells and marker particles
	fluidSimulator.reset();

	// the marker particles are defined depending on the given scene
	switch( sceneIndex )
	{
	case 0:  // simple column with a sphere cut
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			//fluidSimulator.setCellSize( 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec3f( 0.0f, 0.0f, 0.0f ), math::Vec3f( 1.0f, 1.0f, 1.0f ) );

			fluidSimulator.setCellSize( 0.025f );

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			while( fluidSimulator.markerParticles.size() < 433479 )
			{
				Particle3d p;
				// water column
				float width = 0.35f;
				float height = 0.57f;
				float depth = 0.27f;
				// sand column
				//float width = 0.3f;
				//float height = 0.57f;
				//float depth = 0.3f;

				p.position = math::Vec3f( math::g_randomNumber(), math::g_randomNumber(), math::g_randomNumber() );
				if( (p.position.y < height)&&(p.position.x < width)&&(p.position.z < depth) )
				{
					// sphere
					math::Vec3f sphereCenter( 0.35f, 0.27f, 0.27f );
					float distance = (sphereCenter - p.position).getSquaredLength();
					if( !(distance < 0.0225f) ) //radius: 0.15
						fluidSimulator.markerParticles.push_back( p );
				}

				// TODO: align particles on the border of the column so that we have a flat surface
			}
		}break;
	case 1:  // simple sphere
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			fluidSimulator.setCellSize( 0.025f );
			//fluidSimulator.setCellSize( 0.01f );
			//fluidSimulator.setCellSize( 0.001f );
			//fluidSimulator.setDeltaTimeRange( 0.001f, 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec3f( 0.0f, 0.0f, 0.0f ), math::Vec3f( 1.0f, 1.0f, 1.0f ) );

			// add obstacle
			Cube3d *sCube = new Cube3d( 0.0f, 0.0f, 0.0f, 1.0f, 0.2f, .5f );
			sCube->setTranslation( .5f, 0.1f, .25f );
			fluidSimulator.addStaticSolid( sCube );

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle3d p;
				p.position = math::Vec3f( math::g_randomNumber(), math::g_randomNumber(), math::g_randomNumber() );
				// sphere
				math::Vec3f sphereCenter( 0.5f, 0.5f, 0.5f );
				float sphereRadius = 0.2f;

				if( (sphereCenter - p.position).getSquaredLength() < sphereRadius*sphereRadius )
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
		}break;
	case 2:  // simple rotated cube as obstacle and a cubeshaped fluid mass going down on it
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			//fluidSimulator.setCellSize( 0.05f );
			//fluidSimulator.setCellSize( 0.01f );
			fluidSimulator.setCellSize( 0.025f );
			//fluidSimulator.setCellSize( 0.001f );
			//fluidSimulator.setDeltaTimeRange( 0.001f, 0.01f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec3f( 0.0f, 0.0f, 0.0f ), math::Vec3f( 1.0f, 1.0f, 1.0f ) );

			// add obstacle
			Cube3d *sCube = new Cube3d( 0.0f, 0.0f, 0.0f, .5f, 0.2f, .5f );
			sCube->setTranslation( .8f, 0.3f, .5f );
			sCube->setRotation( 0.0f, 0.0f, -15.0f );
			fluidSimulator.addStaticSolid( sCube );

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			while( fluidSimulator.markerParticles.size() < 433479 )
			//while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle3d p;
				float height = 0.2f;
				float width = 0.4f;
				float depth = 0.4f;


				p.position = math::Vec3f( math::g_randomNumber(), math::g_randomNumber(), math::g_randomNumber() );
				// cube
				math::Vec3f cubeCenter( 0.5f, 0.6f, 0.5f );
				if( (fabs( p.position.x - cubeCenter.x ) < width / 2.0f)&&(fabs( p.position.y - cubeCenter.y ) < height / 2.0f)&&(fabs( p.position.z - cubeCenter.z ) < depth / 2.0f) )
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
		}break;
	case 3:  // mountainlike voxelmap
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			fluidSimulator.setCellSize( 0.025f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec3f( 0.0f, 0.0f, 0.0f ), math::Vec3f( 1.0f, 1.0f, 1.0f ) );

			// add mountain voxelmap
			VoxelMap3d *vm = readVoxelMap3d( "mountain1\\mountain1.binvox" );
			if( vm )
			{
				//vm->setCellSize( fluidSimulator.getCellSize() );
				vm->setCellSize( 0.01f );
				fluidSimulator.addStaticSolid( vm );
			}

			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle3d p;
				float height = 0.2f;
				float width = 0.4f;
				float depth = 0.4f;


				p.position = math::Vec3f( math::g_randomNumber(), math::g_randomNumber(), math::g_randomNumber() );
				// cube
				math::Vec3f cubeCenter( 0.5f, 0.25f, 0.5f );
				if( (fabs( p.position.x - cubeCenter.x ) < width / 2.0f)&&(fabs( p.position.y - cubeCenter.y ) < height / 2.0f)&&(fabs( p.position.z - cubeCenter.z ) < depth / 2.0f) )
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
		}break;
	case 4:  // mountainlike voxelmap
		{
			// setup basic parameters (gridsize, the min max range of delta time) and simulation boundaries
			fluidSimulator.setCellSize( 0.025f );
			fluidSimulator.setDeltaTimeRange( 0.0001f, 0.005f );
			fluidSimulator.setSimulationDomain( math::Vec3f( 0.0f, 0.0f, 0.0f ), math::Vec3f( 1.0f, 1.0f, 1.0f ) );

			// add mountain voxelmap
			VoxelMap3d *vm = readVoxelMap3d( "mountain2\\mountain2.binvox" );
			if( vm )
			{
				//vm->setCellSize( fluidSimulator.getCellSize() );
				vm->setCellSize( 0.01f );
				fluidSimulator.addStaticSolid( vm );
			}


			// use a voxelmap to spawn particles in a non arbitrary shape manner
			vm = readVoxelMap3d( "mountain2\\mountain2_glacier_smooth.binvox" );

			if( vm )
			{

				//vm->setCellSize( fluidSimulator.getCellSize() );
				vm->setCellSize( 0.01f );

				// as long as we have not enough particles...
				//while( fluidSimulator.markerParticles.size() < 25000 )
				while( fluidSimulator.markerParticles.size() < 300000 )
				{
					// randomly select a voxel
					size_t voxelIndex = math::g_randomNumber()*vm->getResolutionX()*vm->getResolutionY()*vm->getResolutionZ();

					// if the voxel is set
					if( vm->getVoxelState( voxelIndex ) )
					{
						Particle3d p;

						// randomly position a particle within this voxel
						p.position = vm->getVoxelCenterInLocalSpace( voxelIndex ) - vm->getCellSize()*0.5f + vm->getCellSize()*math::Vec3f( math::g_randomNumber(), math::g_randomNumber(), math::g_randomNumber() );

						// and at it to the simulator
						fluidSimulator.markerParticles.push_back( p );
					}
				}

				// remove voxmap
				delete vm;
			}


			/*
			// randomly add marker particles until we have enough of them
			//while( fluidSimulator.markerParticles.size() < 40000 )
			//while( fluidSimulator.markerParticles.size() < 433479 )
			while( fluidSimulator.markerParticles.size() < 25000 )
			{
				Particle3d p;
				float height = 0.2f;
				float width = 0.4f;
				float depth = 0.4f;


				p.position = math::Vec3f( math::g_randomNumber(), math::g_randomNumber(), math::g_randomNumber() );
				// cube
				math::Vec3f cubeCenter( 0.5f, 0.25f, 0.5f );
				if( (fabs( p.position.x - cubeCenter.x ) < width / 2.0f)&&(fabs( p.position.y - cubeCenter.y ) < height / 2.0f)&&(fabs( p.position.z - cubeCenter.z ) < depth / 2.0f) )
					fluidSimulator.markerParticles.push_back( p );

				// TODO: align particles on the border of the column so that we have a flat surface
			}
			*/
		}break;

	};
}



//
// writes the marker Particles out to a binary file
//
void writeMarkerParticles( const std::vector<Particle3d> &particles, const char *filenameFormat, ... )
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
		float z = particles[i].position.z;

		file.write( (char *) &x, sizeof(float) );
		file.write( (char *) &y, sizeof(float) );
		file.write( (char *) &z, sizeof(float) );
	}

	// done
	file.close();
}

//
// float cellSize, size_t resx, size_t resy, size_t resz, [float value, 9floats stresstensor] resx*resy*resz times
//
void writeGridValues( const char *filenameFormat, ... )
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

	// cellSize
	float cellSize = simulator.getCellSize();
	file.write( (char *) &cellSize, sizeof(float) );

	// we dump an array of 100x100x100 gridcells
	size_t resx = 100;
	size_t resy = 100;
	size_t resz = 100;

	file.write( (char *) &resx, sizeof(size_t) );
	file.write( (char *) &resy, sizeof(size_t) );
	file.write( (char *) &resz, sizeof(size_t) );
	
	for( size_t k=0; k<resz; ++k )
		for( size_t j=0; j<resy; ++j )
			for( size_t i=0; i<resx; ++i )
			{
				float stressTensorValues[9];
				float vonMisesEquivalentStressValue = 0.0f;


				memset( stressTensorValues, 0, sizeof(float)*9 );

				// try to receive the cell
				FluidSimulator3d::Cell *cell = simulator.getCell( i,j,k );

				if( cell )
				{
					vonMisesEquivalentStressValue = cell->vonMisesEquivalentStress;

					math::Matrix44f stressTensor;
					//stressTensor = -simulator.getElasticModulus()*cell->strainTensor;
					stressTensorValues[0] = stressTensor.ma[0];
					stressTensorValues[1] = stressTensor.ma[1];
					stressTensorValues[2] = stressTensor.ma[2];

					stressTensorValues[3] = stressTensor.ma[3];
					stressTensorValues[4] = stressTensor.ma[4];
					stressTensorValues[5] = stressTensor.ma[5];

					stressTensorValues[6] = stressTensor.ma[6];
					stressTensorValues[7] = stressTensor.ma[7];
					stressTensorValues[8] = stressTensor.ma[8];

				}
				
				file.write( (char *) &vonMisesEquivalentStressValue, sizeof(float) );
				file.write( (char *) &stressTensorValues, sizeof(float)*9 );
			}

	// done
	file.close();
}


//
// this function will fill the given memory pointer with the data of the grid
//
void dumpMarkerParticlesToMemory( const std::vector<Particle3d> &particles, MemoryFile &file )
{
	// write into the file
	file.reserve( particles.size() * 3 * sizeof(float) );

	// number of particles
	int particleNum = (int)particles.size();
	file.write( (char *) &particleNum, sizeof(int) );

	// now write the position of each particle
	for( unsigned int i=0; i<(unsigned int)particleNum; ++i )
	{
		float x = particles[i].position.x;
		float y = particles[i].position.y;
		float z = particles[i].position.z;

		file.write( (char *) &x, sizeof(float) );
		file.write( (char *) &y, sizeof(float) );
		file.write( (char *) &z, sizeof(float) );
	}
}

//
// this function will fill the given memory pointer with the data of the grid
//
void dumpGridValuesToMemory( MemoryFile &file )
{
	// we dump an array of 100x100x100 gridcells
	size_t resx = 100;
	size_t resy = 100;
	size_t resz = 100;

	file.reserve( resx*resy*resz*500 );

	// cellSize
	float cellSize = simulator.getCellSize();
	file.write( (char *) &cellSize, sizeof(float) );


	file.write( (char *) &resx, sizeof(size_t) );
	file.write( (char *) &resy, sizeof(size_t) );
	file.write( (char *) &resz, sizeof(size_t) );
	
	for( size_t k=0; k<resz; ++k )
		for( size_t j=0; j<resy; ++j )
			for( size_t i=0; i<resx; ++i )
			{
				math::Vec3f velocity( 0.0f, 0.0f, 0.0f );
				float                    pressure = 0.0f;
				float              stressTensorValues[9];
				memset( stressTensorValues, 0, sizeof(float)*9 );

				// try to receive the cell
				FluidSimulator3d::Cell *cell = simulator.getCell( i,j,k );

				if( cell )
				{
					velocity = cell->velocity;

					pressure = cell->pressure;

					math::Matrix44f stressTensor;
					//stressTensor = -simulator.getElasticModulus()*cell->strainTensor; // prev version
					stressTensor = simulator.getElasticModulus()*cell->strainTensor; // new try without negative sign
					stressTensorValues[0] = stressTensor.ma[0];
					stressTensorValues[1] = stressTensor.ma[1];
					stressTensorValues[2] = stressTensor.ma[2];

					stressTensorValues[3] = stressTensor.ma[3];
					stressTensorValues[4] = stressTensor.ma[4];
					stressTensorValues[5] = stressTensor.ma[5];

					stressTensorValues[6] = stressTensor.ma[6];
					stressTensorValues[7] = stressTensor.ma[7];
					stressTensorValues[8] = stressTensor.ma[8];
				}
				
				file.write( (char *) velocity.v, sizeof(float)*3 );
				file.write( (char *) &pressure, sizeof(float) );
				file.write( (char *) &stressTensorValues, sizeof(float)*9 );
			}
}

//
// this function will write the grid data and the particle positions into a zip file
//
void writeStateDataToZip( const char *filenameFormat, ... )
{
	char filename[2048];

	// assemble filename from given format and arguments
	va_list argumentList;
	va_start( argumentList, filenameFormat );
	vsprintf_s( filename, 2048, filenameFormat, argumentList);
	va_end(argumentList);

	std::string title = dk::util::PathInfo::getName( filename );

	// create zip file
	HZIP zip = CreateZip( util::toWString( std::string( filename ) + ".zip" ).c_str(), 0 );

	// get memory version of the grid data and write grid data to zip -----------------------
	MemoryFile gridFile;
	dumpGridValuesToMemory( gridFile );
	ZipAdd( zip, util::toWString( std::string( title ) + ".grid" ).c_str(), gridFile.getMemory(), gridFile.size() );

	// get memory version of the particle data and write grid data to zip -----------------------
	MemoryFile pclfile;
	dumpMarkerParticlesToMemory( simulator.markerParticles, pclfile );
	ZipAdd( zip, util::toWString( std::string( title ) + ".pcl" ).c_str(), pclfile.getMemory(), pclfile.size() );


	// done
	CloseZip(zip);
}

#endif FLUID_3D