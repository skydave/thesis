/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once






/*

concept:





// add a per vertex property
PropertyHandle ph = mesh.addVertexProperty();

// stl style iteration
for_each( ... )
{
}


// flexible extension of mesh and information
request_triangle_areas( mesh );
request_triangle_centers();
// adds area property to triangles
// and area property to the mesh holding the sum of all areas

// internal: callbacks to help properties to hold the information together
// example: area property -> when triangles get removed or created, the
// sum of the area must be updated



*/