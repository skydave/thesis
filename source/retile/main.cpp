/*---------------------------------------------------------------------

The retile application takes an inputmesh (*.obj or *.binobj) and
performs retiling of polygonal surfaces, a method form greg turk(1992).
The result is saved as *_tiled.obj.

----------------------------------------------------------------------*/
#include <map>
#include <vector>
#include <stdarg.h>
#include <iostream>
#include <fstream>

#include "win32/GLWindow.h"
#include "win32/OpenGLTools.h"
#include "math/Math.h"
#include "util/FileSequence.h"
#include "util/StringManip.h"
#include "util/PathInfo.h"
#include "dk/Camera.h"
#include "dk/DebugNavigator.h"
#include "dk/ColorGradient.h"
#include "dk/Mesh.h"
#include "dk/BinObjIO.h"
#include "dk/ObjIO.h"

using namespace dk;

#include "CrackInfo.h"
#include "SimulationStepData.h"
#include "MeshEx.h"
#include "RetileSurface.h"




LRESULT CALLBACK  glWinProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );  // event handling routine
void                        loadMesh( const std::string &fileName, Mesh* &_mesh );  // loads the mesh from given filename into the current mesh
void                                                 renderShaded( MeshEx *mesh );  // renders the given mesh with opengl
void                                              renderWireframe( MeshEx *mesh );

bool                                                               g_done = false;  // indicates finish of the message loop
bool                                                     renderMeshSwitch = false;
bool                                                           displayMesh = true;  // display cracker/mesh or not
int                                                          displayWireframe = 0;
bool                                                         displayEigen = false;
int                                                             lastMousePosX = 0;  // position of the mouse within the last frame is needed for interaction
int                                                             lastMousePosY = 0;  // position of the mouse within the last frame is needed for interaction

GLWindow                                                                   window;

Camera                                                                     camera;  // camera which represents a view on the scene
DebugNavigator                                                    viewManipulator;  // the manipulator is used to manipulate the camera from some interaction inputs


MeshEx                                                                  *mesh = 0;  // mesh for the current frame


// temp
size_t debugVertex = 257;
std::vector<math::Vec3f>                         lineStrip;
std::vector< std::pair<math::Vec3f, math::Vec3f> > vectors;
std::vector< std::pair<math::Vec2f, math::Vec2f> > vectors2d;
std::vector<std::pair<math::Vec3f, math::Color> >   points;
std::vector<std::pair<math::Vec2f, math::Color> >   points2d;

















//
// entry point
//
int main( int argc, char **argv )
{
	//math::g_randomNumber.setSeed( 6393852ULL );

	if( argc < 2 )
	{
		printf_s( "Missing argument: First file in sequence\n" );
		return 0;
	}
	// load the filesequence of meshes
	util::FileSequence fileSequence = util::FileSequence( argv[1] );


	// load the mesh
	//Mesh *tempMesh=0;
	//printf( "loading mesh...\n" );
	////loadMesh( *fileSequenceIterator, tempMesh );
	////loadMesh( "visc1.000_reduced.obj", tempMesh );
	//loadMesh( "visc1.290.binobj", tempMesh );
	//loadMesh( "visc1.283.obj", tempMesh );
	//loadMesh( "visc1.283_reduced2.obj", tempMesh );
	//loadMesh( "test3.obj", tempMesh );
	//loadMesh( "triangle.obj", tempMesh );
	//loadMesh( "crack_sphere.obj", tempMesh );
	//loadMesh( "plane.obj", tempMesh );
	//loadMesh( "vertex_remove_test.obj", tempMesh );

	//// single shot
	//{
	//	std::string filename = "visc1.000.binobj";
	//	tempMesh=0;
	//	printf( "loading mesh...\n" );
	//	loadMesh( filename, tempMesh );
	//	tempMesh->assertManifold();
	//	printf( "converting mesh to MeshEx...\n" );
	//	mesh = new MeshEx( tempMesh );
	//	printf( "MeshEx: %i vertices, %i edges and %i triangles\n", mesh->m_vertices.size(), mesh->m_edges.size(), mesh->m_triangles.size() );

	//	printf( "Retiling mesh...\n" );
	//	RetileSurface retile;

	//	// perform retiling of the mesh
	//	retile.retile( mesh, 2000 );

	//	{
	//		Mesh *tempMesh = mesh->getMesh();
	//		dk::io::exportToBinObjFile( tempMesh, util::PathInfo::getTitle( filename ) + "_reduced.binobj" );
	//		delete tempMesh;
	//	}
	//}

	std::string prevFile = "";
	for( util::FileSequence::iterator it = fileSequence.begin(); it != fileSequence.end(); ++it )
	//for( util::FileSequence::iterator it = fileSequence.begin(); it == fileSequence.begin(); ++it )
	//util::FileSequence::iterator it = fileSequence.begin();
	//for( size_t count = 0; count <2; ++count, ++it )
	{
		Mesh *inMesh = 0;

		printf( "loading mesh %s...\n", (*it).c_str() );
		loadMesh( *it, inMesh );
		inMesh->assertManifold();

		printf( "converting mesh to MeshEx...\n" );
		mesh = new MeshEx( inMesh );
		delete inMesh;
		printf( "MeshEx: %i vertices, %i edges and %i triangles\n", mesh->m_vertices.size(), mesh->m_edges.size(), mesh->m_triangles.size() );

		// if there are crackpoints we create a set of constrain vertices from them
		RetileSurface::ConstrainedVertexSet cvs;

		// if is a previous file -> we try to track its crack information
		if( !prevFile.empty() )
		//if(0)
		{
			// cracktracking ------------------------------
			// load crackinfo from last step
			std::string crackFile( util::PathInfo::getTitle( prevFile ) + ".cracks" );
			CrackInfo ci;
			printf( "loading crackinformation %s...\n", crackFile.c_str() );
			ci.loadFromDisk( crackFile );

			// read the velocity for the current step
			printf( "loading velocity field...\n" );
			SimulationStepData simdata( util::PathInfo::getTitle( prevFile ) + ".zip" );

			printf( "updating crackinformation...\n" );
			for( CrackInfo::CrackPoints::iterator cpit = ci.points.begin(); cpit != ci.points.end(); ++cpit )
			{
				CrackInfo::CrackPoint *cp = &(*cpit);
				math::Vec3f             intersection; // position on the mesh surface
				math::Vec3f                   normal; // normal on the mesh surface
				MeshEx::Triangle              *t = 0; // the triangle on which the crackpoint lies

				// move the crackpoints through the velocity field of the current step to update them ---
				cp->position += simdata.getDeltaTime()*simdata.getVelocityAt(cp->position);

				// project the point onto the mesh - we use the normal of the crackpoint for this -------

				// construct a line from far away coming in direction of the normal through the point
				math::Vec3f p1, p2;

				p1 = cp->position + cp->normal*4.0f;
				p2 = cp->position - cp->normal*4.0f;

				// find all intersections of the line with the mesh and return the closest to the point
				bool result = mesh->findClosestIntersection( cp->position, p1, p2, intersection, normal, t );

				// if an intersection was found
				if( result )
				{
					// update crackpoint
					cp->position = intersection;
					cp->normal = normal;

					// add a constrained vertex for the current crackpoint
					cvs.push_back( std::make_pair( cp->position, t )  );
					
					// temp
					//points.push_back( std::make_pair( (*cpit).position, math::Color::Yellow() ) );
					points.push_back( std::make_pair( cp->position, math::Color::Yellow() ) );
				}else
					printf( "error : could not find closest intersection to the surface\n" );
			}

			// save the updated crack information for the current step
			ci.saveToDisk( util::PathInfo::getTitle( *it ) + ".cracks" );
		}


		// --------------------------------------------

		try
		{
			printf( "Retiling mesh...\n" );
			RetileSurface retile;


			// perform retiling of the mesh
			retile.retile( mesh, 2000, cvs );

			{
				Mesh *outMesh = mesh->getMesh();
				//dk::io::exportToBinObjFile( outMesh, util::PathInfo::getTitle( *it ) + "_reduced.binobj" );
				dk::io::exportToObjFile( outMesh, util::PathInfo::getTitle( *it ) + "_reduced.obj" );
				delete outMesh;
			}
		}
		catch(...)
		{
		}

		delete mesh;
		mesh = 0;

		prevFile = *it;
	}

	//return 0;





/*
	{
		Mesh *tempMesh = mesh->getMesh();
		dk::io::exportToBinObjFile( tempMesh, "test_reduced.binobj" );
		delete tempMesh;
	}
*/




	viewManipulator.lookAt = math::Vec3f( 0.5f, 0.5f, 0.5f );
	//viewManipulator.lookAt = mesh->m_vertices[debugVertex]->position;
	//viewManipulator.lookAt = oldVertices[debugVertex]->position;
	//viewManipulator.setLookAt( oldVertices2[debugVertex]->position );
	//viewManipulator.setLookAt( vertices[debugVertex]->position );
	//viewManipulator.setLookAt( points.back().first );
	viewManipulator.azimuth = viewManipulator.elevation = 0.0f;
	viewManipulator.distance = .7f;
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
			glClearColor( .5f, .5f, .5f, 1.0f );
			glClear( GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);

			glViewport( (GLsizei)0.0f, (GLsizei)0.0f, (GLsizei)window.getWidth(), (GLsizei)window.getHeight() );

			glMatrixMode( GL_PROJECTION );
			glLoadMatrixf( camera.getProjectionMatrix().ma );

			glMatrixMode( GL_MODELVIEW );
			// the worldToEyeMatrix is computed by the OrbitManipulator which is coupled with the mouse input
			glLoadMatrixf( (GLfloat *)camera.getViewMatrix().ma );

			// draw mesh
			if( displayMesh && mesh )
			{
				glEnable( GL_DEPTH_TEST );
				glEnable( GL_CULL_FACE );
				//glEnable( GL_LIGHTING );

				// Create light components
				float ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
				float diffuseLight[] = { 0.9f, 0.9f, 0.9f, 1.0f };
				float specularLight[] = { 0.7f, 0.7f, 0.7f, 1.0f };
				//float position[] = { -.5f, 1.5f, -1.0f, 1.0f };
				float position[] = { 0.0f, 0.0f, 1.0f, 0.0f }; // headlight (directional)
				float position2[] = { .0f,  .0f, .0f, 1.0f };

				// Assign created components to GL_LIGHT0
				/*
				glEnable(GL_LIGHT0);
				glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
				glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
				glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
				glLightfv(GL_LIGHT0, GL_POSITION, position);

				glEnable(GL_LIGHT1);
				glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight);
				glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight);
				glLightfv(GL_LIGHT1, GL_POSITION, position2);
				*/
				// head-light
				glPushMatrix();
				glLoadIdentity();
				glEnable(GL_LIGHT0);
				glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
				glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
				glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
				glLightfv(GL_LIGHT0, GL_POSITION, position);
				glPopMatrix();

				glDisable( GL_LIGHTING );
				//drawGrid();

				renderShaded( mesh );
				renderWireframe( mesh );

				glDisable( GL_LIGHTING );







				
				glDisable( GL_LIGHTING );
				glPointSize( 5.0f );
				// render vertices
				glBegin( GL_POINTS );
				/*
				for( size_t i=0; i<vertices.size(); ++i )
				{
					if( i == debugVertex )
						glColor3f( 1.0f, 6.0f, 1.0f );
					else
						glColor3f( 1.0f, 0.0f, 0.0f );

					//if( vertices[i]->tag )
					//	glColor3f( 0.0f, 0.7f, 1.0f );
					//else
					//	continue;

					glVertex3fv( vertices[i]->position.v );
				}
				*/
				glColor3f( 1.0f, 1.0f, 1.0f );
				//glVertex3fv( mesh->m_vertices[debugVertex]->position.v );
				glEnd();






				// render points
				glBegin( GL_POINTS );
				for( size_t i=0; i<points.size(); ++i )
				{
					glColor3fv( points[i].second.v );
					glVertex3fv( points[i].first.v );
				}
				for( size_t i=0; i<points2d.size(); ++i )
				{
					glColor3fv( points2d[i].second.v );
					glVertex3f( points2d[i].first.x, points2d[i].first.y, 0.0f );
				}
				glEnd();


				/*
				glPointSize( 8.0f );
				glBegin( GL_POINTS );
				glColor3f( 0.0f, 1.0f, 0.0f );
				glVertex3fv( g_currentPoint.v );
				glColor3f( 0.0f, 1.0f, 1.0f );
				glVertex3fv( g_targetPoint.v );
				glEnd();

				glLineWidth( 5.0f );
				glBegin( GL_LINES );
				glColor3f( 1.0f, 0.0f, 0.0f );
				//glVertex3fv( (mesh->edges[0]->v1->position).v );
				//glVertex3fv( (mesh->edges[0]->v1->position + g_axis).v );
				glEnd();

				glLineWidth( 5.0f );
				glBegin( GL_LINE_STRIP );
				for( size_t i=0; i<lineStrip.size(); ++i )
				glVertex3fv( lineStrip[i].v );
				glEnd();
				*/

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
				// 2d vectors
				for( size_t i=0; i<vectors2d.size(); ++i )
				{
					//if( (currentVector >= 0) && (i != currentVector) )
					//	continue;

					glColor3f( 0.0f, 1.0f, 0.0f );
					glVertex3f( vectors2d[i].first.x, vectors2d[i].first.y, 0.0f );
					glVertex3f( vectors2d[i].second.x, vectors2d[i].second.y, 0.0f );
				}
				glEnd();

				// render debug vertex stuff
				{

					glLineWidth( 1.0f );
					glColor3f( 1.0f, 0.0f, 0.0f );
					glBegin( GL_LINE_STRIP );
					//for( size_t i=0; i<vertices[debugVertex]->path.size(); ++i )
					//	glVertex3fv( vertices[debugVertex]->path[i].v );
					glEnd();


					glBegin( GL_LINES );
					glColor3f( 0.2f, 0.6f, 0.8f );
					//glVertex3fv( vertices[debugVertex]->position.v );
					//glVertex3fv( (vertices[debugVertex]->position + vertices[debugVertex]->force*g_damp).v );
					glEnd();

				}

				//glDisable( GL_LIGHTING );
			}




			// blit screen
			SwapBuffers( window.mhDC );

        }
	}
}


void renderShaded( MeshEx *mesh )
{
	//glEnable( GL_LIGHTING );
	glDisable( GL_LIGHTING );
	glEnable( GL_POLYGON_OFFSET_FILL );
	glPolygonOffset( 1.001f , .01f );

	int count = 0;
	// color/shaded display
	glBegin( GL_TRIANGLES );
	// iterate over all triangles
	for( std::vector<MeshEx::Triangle *>::iterator tit = mesh->m_triangles.begin(); tit != mesh->m_triangles.end(); ++tit )
	{
		//if( count++ == 1 )
		glColor3f( 1.0f, 0.0f, 0.0f );

		for( unsigned int i=0; i<3; ++i )
		{
			MeshEx::Vertex *v = (*tit)->v[i];

			if( !(*tit)->tag )
				glColor3f( 0.2f, 0.2f, 0.2f );
			//glColor3f( v->color.r, v->color.g, v->color.b );
			glNormal3f( v->normal.x, v->normal.y, v->normal.z );
			glVertex3f( v->position.x, v->position.y, v->position.z );
		}
	}
	glEnd();

	glDisable( GL_POLYGON_OFFSET_FILL );
}

void renderWireframe( MeshEx *mesh )
{
	glDisable( GL_LIGHTING );
	// color/shaded display
	glLineWidth( 2.0f );
	glBegin( GL_LINES );
	glColor3f( 0.3f, 0.3f, 0.3f );

	int count = 0;
	// iterate over all edges
	for( std::vector<MeshEx::Edge *>::iterator eit = mesh->m_edges.begin(); eit != mesh->m_edges.end(); ++eit )
	{
		MeshEx::Edge *e = *eit;

		//if( count++ == 0 )
		//if( e->isBoundaryEdge() )
		if( e->tag )
			glColor3f( 0.5f, 1.0f, 0.76f );
		else
			glColor3f( 0.0f, 0.0f, 0.0f );

		//if( !e->isBoundaryEdge() )
		//	continue;

		//	glColor3f( 0.5f, 1.0f, 0.76f );
		//if(e->tag)
		{
			MeshEx::Vertex *v1 = e->v1;
			MeshEx::Vertex *v2 = e->v2;

			//glColor3f( v->color.r, v->color.g, v->color.b );
			//glNormal3f( v->normal.x, v->normal.y, v->normal.z );
			glVertex3fv( v1->position.v );
			glVertex3fv( v2->position.v );
		}
	}

	glEnd();
}



//
// loads the mesh from given filename into the current mesh
//
void loadMesh( const std::string &fileName, Mesh* &_mesh )
{
	if( _mesh )
	{
		delete _mesh;
		_mesh = 0;
	}

	if( util::getExtension(fileName) == ".obj" )
		_mesh = io::importFromObjFile( fileName );
	if( util::getExtension(fileName) == ".binobj" )
		_mesh = io::importFromBinObjFile( fileName );

	if( _mesh )
		_mesh->computeNormals();
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
				//if( debugVertex > 0 )
				//	--debugVertex;
				//printf( "%i            - %i\n", debugVertex, mesh->m_vertices[debugVertex]->triangleRing.size() );
				//printf( "%i\n", debugVertex);
				//viewManipulator.setLookAt( mesh->m_vertices[debugVertex]->position );
				//viewManipulator.setLookAt( vertices[debugVertex]->position );
			}break;
		case VK_RIGHT :
			{
				//if( debugVertex < mesh->m_vertices.size()-1 )
				//if( debugVertex < vertices.size()-1 )
				//	++debugVertex;
				//printf( "%i            - %i\n", debugVertex, mesh->m_vertices[debugVertex]->triangleRing.size() );
				//printf( "%i\n", debugVertex);
				//viewManipulator.setLookAt( mesh->m_vertices[debugVertex]->position );
				//viewManipulator.setLookAt( vertices[debugVertex]->position );
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
		case 73 :// KEY: 'i'
			//for( size_t i = 0; i<84; ++i )
			{
				//printf( "performing iteration...%i\n", currentIteration++ );
				//performIteration( g_radius, g_damp );
			}
			break;
		case 80 :// KEY : 'p'
			{
			}break;
		case 77 :// KEY: 'm'
			{
				displayMesh = !displayMesh;
			}break;
		case 83 :// KEY: 's'
			renderMeshSwitch = !renderMeshSwitch;
			break;
		case 84 :// KEY: 't'
			//doMutualTesselation();
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
			break;
		case 109 : // numpad - key
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

