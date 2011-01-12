/*---------------------------------------------------------------------

The OpenglTools are a bunch of methods which are commonly used in any
simple opengl application like drawing a grid, drawing a cell/cube or
drawing a dk::Mesh etc.


----------------------------------------------------------------------*/
#include "OpenGLTools.h"



namespace dk
{
	//
	// renders a dk::Mesh using opengl
	// no state changes are made
	//
	void renderMesh( dk::Mesh *mesh )
	{
		glBegin( GL_TRIANGLES );
		// iterate over all triangles
		for( std::vector<dk::Mesh::Triangle *>::iterator tit = mesh->triangles.begin(); tit != mesh->triangles.end(); ++tit )
		{
			for( unsigned int i=0; i<3; ++i )
			{
				dk::Mesh::Vertex *v = (*tit)->v[i];

				glColor3f( v->color.r, v->color.g, v->color.b );
				glNormal3f( v->normal.x, v->normal.y, v->normal.z );
				glVertex3f( v->position.x, v->position.y, v->position.z );
			}
		}
		glEnd();
	}


	//
	// sets light0 parameters accordingly
	//
	void headLight()
	{
		float ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
		float diffuseLight[] = { 0.9f, 0.9f, 0.9f, 1.0f };
		float specularLight[] = { 0.7f, 0.7f, 0.7f, 1.0f };
		float position[] = { 0.0f, 0.0f, 1.0f, 0.0f }; // headlight (directional)

		// head-light
		glPushMatrix();
		glLoadIdentity();
		glEnable(GL_LIGHT0);
		glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
		glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
		glLightfv(GL_LIGHT0, GL_POSITION, position);
		glPopMatrix();
	}




	//
	// draw a orientation grid identical to the ones used in popular dcc apps
	//
	void drawGrid()
	{

		float gridSizeHalved = 5.0f / 2.0f;
		float gridSpacing = 0.5f;
		float y = 0.0f;

		glLineWidth( 1.0f );
		glBegin( GL_LINES );
		// main axes
		glColor3f( 0.3f, 0.3f, 0.3f );
		glVertex3f( -gridSizeHalved, y, 0.0f );
		glVertex3f(  gridSizeHalved, y, 0.0f );
		glVertex3f( 0.0f, y, -gridSizeHalved );
		glVertex3f( 0.0f, y,  gridSizeHalved );

		// gridlines
		glColor3f( 0.7f, 0.7f, 0.7f );
		int i1 = 1;
		while( i1*gridSpacing < gridSizeHalved )
		{
			// grid in x
			glVertex3f( i1*gridSpacing, y, -gridSizeHalved );
			glVertex3f( i1*gridSpacing , y , gridSizeHalved );

			glVertex3f( -i1*gridSpacing , y , -gridSizeHalved );
			glVertex3f( -i1*gridSpacing , y , gridSizeHalved );
			
			// grid in z
			glVertex3f( -gridSizeHalved , y , i1*gridSpacing );
			glVertex3f( gridSizeHalved , y , i1*gridSpacing );

			glVertex3f( -gridSizeHalved ,  y , -i1*gridSpacing );
			glVertex3f( gridSizeHalved , y , -i1*gridSpacing );

			i1++;
		}

		// wrap lines all around to make the grid closed
		glVertex3f( -gridSizeHalved , y , -gridSizeHalved );
		glVertex3f( -gridSizeHalved, y , gridSizeHalved );

		glVertex3f( -gridSizeHalved, y , gridSizeHalved );
		glVertex3f( gridSizeHalved , y , gridSizeHalved );

		glVertex3f( gridSizeHalved , y , gridSizeHalved );
		glVertex3f( gridSizeHalved, y , -gridSizeHalved );

		glVertex3f( gridSizeHalved, y , -gridSizeHalved );
		glVertex3f( -gridSizeHalved , y , -gridSizeHalved );

		glEnd();

		// draw koordinate axes
		glLineWidth( 5.0f );
		glBegin( GL_LINES );
		// right axes
		glColor3f( 0.8f, 0.0f, 0.0f );
		glVertex3f( 0.0f, 0.0f, 0.0f );
		glVertex3f( 1.0f, 0.0f, 0.0f );
		// forward axes
		glColor3f( 0.0f, 0.0f, 0.8f );
		glVertex3f( 0.0f, 0.0f, 0.0f );
		glVertex3f( 0.0f, 0.0f, 1.0f );
		// up axes
		glColor3f( 0.0f, 0.8f, 0.0f );
		glVertex3f( 0.0f, 0.0f, 0.0f );
		glVertex3f( 0.0f, 1.0f, 0.0f );

		glEnd();
	}
}