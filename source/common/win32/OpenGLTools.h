/*---------------------------------------------------------------------

The OpenglTools are a bunch of methods which are commonly used in any
simple opengl application like drawing a grid, drawing a cell/cube or
drawing a dk::Mesh etc.


----------------------------------------------------------------------*/
#pragma once
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "dk/Mesh.h"




namespace dk
{
	void                                      renderMesh( dk::Mesh *mesh );  ///< renders the given mesh out to opengl without any state changes
	void                                                        drawGrid();  ///< draw a orientation grid identical to the ones used in popular dcc apps
	void                                                       headLight();  ///< sets light0 parameters accordingly
}





