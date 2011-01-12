/*---------------------------------------------------------------------

The viewer is a multi-purpose application for viewing sequences of
particlesets, meshes etc.

The viewer is organized into displaymodes. There is a class for each
displaymode which may read the data for its displaytype (i.e. mesh or
particle data) and offers a render method to display it in opengl.

----------------------------------------------------------------------*/
#include <vector>
#include <algorithm>
#include <stdarg.h>
#include <iostream>
#include <fstream>

#include <win32/GLWindow.h>
#include <win32/OpenGLTools.h>
#include <math/Math.h>
#include <util/FileSequence.h>
#include <util/StringManip.h>
#include <util/PathInfo.h>
#include <dk/Camera.h>
#include <dk/DebugNavigator.h>
#include <dk/Mesh.h>
#include <dk/BinObjIO.h>
#include <dk/ObjIO.h>

#include <CrackInfo.h>

#include "ParticleViewer.h"

using namespace dk;


LRESULT CALLBACK  glWinProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );  // event handling routine
void                                      loadMesh( const std::string &fileName );  // loads the mesh from given filename into the current mesh
void                                                   renderShaded( Mesh *mesh );  // renders the given mesh with opengl
void                                                renderWireframe( Mesh *mesh );  // renders the given mesh with opengl
void                                                                   drawGrid();


bool                                                               g_done = false;  // indicates finish of the message loop
bool                                                     displayWireframe = false;
int                                                             lastMousePosX = 0;  // position of the mouse within the last frame is needed for interaction
int                                                             lastMousePosY = 0;  // position of the mouse within the last frame is needed for interaction


GLWindow                                                                   window;
Camera                                                                     camera;  // camera which represents a view on the scene
DebugNavigator                                                    viewManipulator;  // the manipulator is used to manipulate the camera from some interaction inputs

util::FileSequence                                                   fileSequence;  // Sequence of mesh files
util::FileSequence::iterator                                 fileSequenceIterator;  // points to the current filename within the sequence

std::vector<std::pair<math::Vec3f, math::Color> >                          points;
std::vector< std::pair<math::Vec3f, math::Vec3f> >                        vectors;

Mesh                                                                    *mesh = 0;  // mesh for the current frame

CrackInfo                                                               crackInfo;

enum
{
	NONE,
	PARTICLES,
	MESH
}displayMode = NONE;

ParticleViewer                                                     particleViewer;



//
// entry point
//
int main( int argc, char **argv )
{
	if( argc < 2 )
	{
		printf_s( "Missing argument: First file in sequence\n" );
		return 0;
	}

	// load the filesequence of meshes
	fileSequence = util::FileSequence( argv[1] );
	fileSequenceIterator = fileSequence.begin();

	// examine the extension of the argument
	if( (dk::util::PathInfo::getExtension( argv[1] ) == "zip")||(dk::util::PathInfo::getExtension( argv[1] ) == "pcl") )
	{
		// we are going to display a particleset
		displayMode = PARTICLES;

		particleViewer.assignFile( *fileSequenceIterator );
	}else
	if( (dk::util::PathInfo::getExtension( argv[1] ) == "obj")||(dk::util::PathInfo::getExtension( argv[1] ) == "binobj") )
	{
		// we are going to display meshes
		displayMode = MESH;

		// load first mesh
		loadMesh( *fileSequenceIterator );
	}



	window.createGLWindow( "tset", 800, 600, 100, 100, 32, 0, glWinProc, NULL);

	window.show();

	//viewManipulator.setLookAt( 0.0f, 0.0f, 0.0f );
	//viewManipulator.azimuth = viewManipulator.elevation = 0.0f;
	//viewManipulator.distance = .5f;


	
	viewManipulator.setLookAt( math::Vec3f( 0.05678f, 0.213533f, -0.102318f ) );
	viewManipulator.azimuth = -34.0f;
	viewManipulator.elevation = 21.0f;
	viewManipulator.distance = 1.305f;

	viewManipulator.setCamera( &camera );


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

			switch( displayMode )
			{
			case PARTICLES:
				{
					particleViewer.render();
				}break;
			case MESH:
				{
					// draw mesh
					if( mesh )
					{
						glEnable( GL_DEPTH_TEST );
						//glEnable( GL_CULL_FACE );
						glEnable( GL_LIGHTING );

						/*
						glEnable(GL_LIGHT0);
						glEnable(GL_LIGHT1);

						// Create light components
						float ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
						float diffuseLight[] = { 0.8f, 0.8f, 0.8f, 1.0f };
						float specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
						float position[] = { -1.5f, 1.0f, -4.0f, 1.0f };
						float position2[] = { 1.5f, 1.0f, 4.0f, 1.0f };

						// Assign created components to GL_LIGHT0
						glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
						glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
						glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
						glLightfv(GL_LIGHT0, GL_POSITION, position);

						glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
						glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
						glLightfv(GL_LIGHT1, GL_POSITION, position2);
						*/
						headLight();

						renderShaded( mesh );

						if( displayWireframe )
							renderWireframe( mesh );

						


						glDisable( GL_LIGHTING );

						// render Vectorys
						glLineWidth( 2.0f );
						glBegin( GL_LINES );
						for( size_t i=0; i<vectors.size(); ++i )
						{
							glColor3f( 0.0f, 1.0f, 0.0f );
							glVertex3fv( vectors[i].first.v );
							glVertex3fv( vectors[i].second.v );

							// render arrowhead
							if(0)
							{
								float scl = 0.2f;
								float angle = math::degToRad(25.0f);

								math::Vec3f vec = vectors[i].second - vectors[i].first;
								float vec_length = vec.getLength();
								math::Vec3f temp = math::crossProduct( math::Vec3f( 0.0f, 1.0f, 0.0f ), math::normalize(vec) );

								glVertex3fv( vectors[i].second.v );
								glVertex3fv( (vectors[i].second - scl*(vec*acosf(angle) + tanf(angle)*vec_length*temp)).v );

								glVertex3fv( vectors[i].second.v );
								glVertex3fv( (vectors[i].second - scl*(vec*acosf(angle) - tanf(angle)*vec_length*temp)).v );
							}
						}
						glEnd();

						glDisable( GL_LIGHTING );
						glPointSize( 5.0f );
						// render points
						glBegin( GL_POINTS );
						for( size_t i=0; i<points.size(); ++i )
						{
							glColor3fv( points[i].second.v );
							glVertex3fv( points[i].first.v );
						}
						glEnd();
						glDisable( GL_LIGHTING );
					}
				}break;
			case NONE:
			default:
				break;
			};

			// blit screen
			SwapBuffers( window.mhDC );
        }
	}
}


void renderShaded( Mesh *mesh )
{
	glEnable( GL_LIGHTING );
	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset( 1.001f , .01f );

	int count = 0;
	// color/shaded display
	glBegin( GL_TRIANGLES );
	// iterate over all triangles
	for( std::vector<Mesh::Triangle *>::iterator tit = mesh->triangles.begin(); tit != mesh->triangles.end(); ++tit )
	{
		for( unsigned int i=0; i<3; ++i )
		{
			Mesh::Vertex *v = (*tit)->v[i];

			glNormal3f( v->normal.x, v->normal.y, v->normal.z );
			glVertex3f( v->position.x, v->position.y, v->position.z );
		}
	}
	glEnd();

	glDisable( GL_POLYGON_OFFSET_FILL );
}

//
//
//
void renderWireframe( Mesh *mesh )
{
	std::vector<std::pair<Mesh::Vertex *, Mesh::Vertex *> > edges;

	// iterate over all triangles
	for( std::vector<Mesh::Triangle *>::iterator tit = mesh->triangles.begin(); tit != mesh->triangles.end(); ++tit )
	{
		for( unsigned int i=0; i<3; ++i )
		{
			Mesh::Vertex *v0 = (*tit)->v[i];
			Mesh::Vertex *v1 = (*tit)->v[(i+1)%3];

			/*
			glNormal3f( v0->normal.x, v0->normal.y, v0->normal.z );
			glVertex3f( v0->position.x, v0->position.y, v0->position.z );
			glNormal3f( v1->normal.x, v1->normal.y, v1->normal.z );
			glVertex3f( v1->position.x, v1->position.y, v1->position.z );
			*/
			//if( v1 < v0 )
			//	std::swap( v0, v1 );

			edges.push_back( std::make_pair( v0, v1 ) );
		}
	}

	std::sort( edges.begin(), edges.end() );
	edges.erase( std::unique( edges.begin(), edges.end() ), edges.end() );



	glDisable( GL_LIGHTING );
	glLineWidth( 2.0f );
	glBegin( GL_LINES );
	glColor3f( 0.0f, 0.0f, 0.0f );
	for( std::vector<std::pair<Mesh::Vertex *, Mesh::Vertex *> >::iterator it = edges.begin(); it != edges.end(); ++it )
	{
		glVertex3fv( it->first->position.v );
		glVertex3fv( it->second->position.v );
	}
	glEnd();
}

//
// loads the mesh from given filename into the current mesh
//
void loadMesh( const std::string &fileName )
{
	if( mesh )
	{
		delete mesh;
		mesh = 0;
	}

	if( util::getExtension(fileName) == ".obj" )
		mesh = io::importFromObjFile( fileName );
	if( util::getExtension(fileName) == ".binobj" )
		mesh = io::importFromBinObjFile( fileName );

	if( mesh )
		mesh->computeNormals();
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
			}break;
		case VK_SPACE:
			{
			}break;
		case VK_UP :
			break;
		case VK_DOWN :
			break;
		case VK_LEFT :
			{
				if( fileSequenceIterator != fileSequence.begin() )
					--fileSequenceIterator;

				switch( displayMode )
				{
				case PARTICLES:
					{
						particleViewer.assignFile( *fileSequenceIterator );
					}break;
				case MESH:
					{
						loadMesh( *fileSequenceIterator );
						std::string crackFile = util::PathInfo::getTitle( *fileSequenceIterator ) + ".cracks";

						// remove _reduced
						std::string::size_type pos = crackFile.find("_reduced");
						if( pos != std::string::npos )
							crackFile.erase( pos, 8 );



						// try to load additional crackdata
						CrackInfo ci;
						ci.loadFromDisk( crackFile );

						points.clear();
						// add the crackpoints
						for( CrackInfo::CrackPoints::iterator it = ci.points.begin(); it != ci.points.end(); ++it )
						{
							CrackInfo::CrackPoint *cp = &(*it);
							points.push_back( std::make_pair( cp->position, math::Color::Yellow() ) );
						}
					}break;
				case NONE:
				default:
					break;
				};
			}break;
		case VK_RIGHT :
			{
				if( fileSequenceIterator != fileSequence.end() )
					++fileSequenceIterator;


				switch( displayMode )
				{
				case PARTICLES:
					{
						particleViewer.assignFile( *fileSequenceIterator );
					}break;
				case MESH:
					{
						loadMesh( *fileSequenceIterator );

						std::string crackFile = util::PathInfo::getTitle( *fileSequenceIterator ) + ".cracks";

						// remove _reduced
						std::string::size_type pos = crackFile.find("_reduced");
						if( pos != std::string::npos )
							crackFile.erase( pos, 8 );

						printf( "%s\n", crackFile.c_str() );

						// try to load additional crackdata
						CrackInfo ci;
						ci.loadFromDisk( crackFile );

						points.clear();
						// add the crackpoints
						for( CrackInfo::CrackPoints::iterator it = ci.points.begin(); it != ci.points.end(); ++it )
						{
							CrackInfo::CrackPoint *cp = &(*it);
							points.push_back( std::make_pair( cp->position, math::Color::Yellow() ) );
						}
					}break;
				case NONE:
				default:
					break;
				};
			}break;
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
			break;
		case 68 :// KEY : 'd'
			break;
		case 86 :// KEY : 'v'
			break;
		case 87 :// KEY : 'w'
			displayWireframe = !displayWireframe;
			break;
		case 71 :// KEY : 'g'
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

	case WM_LBUTTONDOWN:
			{
				if( !GetKeyState( VK_LMENU ) && (displayMode == MESH) )
				{
					// get normalized screen coordinates
					int xpos = ((int)(short)LOWORD(lParam)); 
					int ypos = ((int)(short)HIWORD(lParam)); 
					float _x = (float)xpos / (float)window.getWidth();
					float _y = (float)ypos / (float)window.getHeight();
					// select a point
					math::Ray3d ray = camera.getRay( _x, _y );

					math::Vec3f origin = ray.origin;
					math::Vec3f target = ray.getTarget();

					// vertex and distance to the line
					//std::vector<std::pair<Mesh::Vertex *, float> >
					//vectors.push_back( std::make_pair(origin, target) );

					Mesh::Vertex *minVert = 0;
					float       minDist = 99999999.0f;

					// find clostest vertex with the normal pointing into the direction of the camera
					for( std::vector<Mesh::Vertex *>::iterator it = mesh->vertices.begin(); it != mesh->vertices.end(); ++it )
					{
						Mesh::Vertex *v = *it;

						if( math::dotProduct( v->normal, ray.direction ) > 0.0f )
							continue;

						// compute distance to the ray
						float d = math::distancePointLine( v->position, origin, target );

						if( d < minDist )
						{
							minVert = v;
							minDist = d;
						}
					}

					if( minVert )
					{
						points.push_back( std::make_pair( minVert->position, math::Color::Red() ) );

						crackInfo.addCrackPoint( minVert->position, minVert->normal );

						// resave
						crackInfo.saveToDisk( "cracktest.cracks" );
					}
				}
			}break;
	case WM_MOUSEMOVE :
		{
			int xPos = ((int)(short)LOWORD(lParam)); 
			int yPos = ((int)(short)HIWORD(lParam)); 

			// left alt?
			if( GetKeyState( VK_LMENU ) )
			{
				// if a mousebutton had been pressed
				if(((wParam == MK_LBUTTON)||(wParam == MK_MBUTTON)||(wParam == MK_RBUTTON)))
				{
					if( wParam == MK_LBUTTON )
					{
						// Alt + LMB => rotation
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
			}

			lastMousePosX = xPos;
			lastMousePosY = yPos;
		}break;

	default:
        return DefWindowProc( hWnd, uMsg, wParam, lParam );   
	}


	return DefWindowProc( hWnd, uMsg, wParam, lParam );
}