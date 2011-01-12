/*---------------------------------------------------------------------

This simulator derives from FluidSimulator3d and overloads the
updateVelocity method. There are some small additions which will
make the fluid behave like sand.
The idea and theory behind that method was introduced in the paper
"Animating Sand as a Fluid" at Siggraph in 2005 by Zhou/Bridson.

----------------------------------------------------------------------*/
#include "SandSimulator3d.h"






//
// constructor
//
SandSimulator3d::SandSimulator3d()
{
	//setViscosity( 1500.0f );
	particleBasedAdvection = false;
}





//
// takes the coloumb friction into account and evaluates
// the mohr-coloumb yield-condition to change the fluid
// velocity so that it behaves like sand
//
void SandSimulator3d::applySandModel( float dt )
{
	// prepare cells -------------------------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.rigid        = false;
		(*it).second.layer        = -1;    // used to find groups of connected rigid cells
	}


	// check yield-condition for every fluid cell --------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// get reference to current cell
		Cell *cell = &((*it).second);

		// if the current cell is not a fluid cell
		if( cell->type != Cell::Fluid )
			// skip
			continue;
		//if( !(cell->xNeighboursFluid || cell->yNeighboursFluid || cell->zNeighboursFluid) )
		//	continue;

		// compute strain rate D from strain rate tensor D and the frobenius norm of D
		math::Matrix44f           D;  // strain rate tensor
		math::Matrix44f     sigma_f;  // friction stress tensor in yielding area (dynamic friction)
		math::Matrix44f sigma_rigid;  // friction stress tensor in rigid areas (static friction)

		D = math::Matrix44f::Zero();
		sigma_f = math::Matrix44f::Zero();
		sigma_rigid = math::Matrix44f::Zero();

		// compute the rate of strain tensor D
		D = computeStrainRateTensor( cell );

		// compute frobenius norm of sigma_rigid
		float frobD = math::frobeniusNorm( D );  // frobenius Norm of the rate of strain tensor D

		float frictionCoefficient = 0.8f;

		// compute sigma_f from D
		if( frobD > 0.0f )
			//sigma_f = -frictionCoefficient*cell->pressure*(1.0f/(sqrt(1.0f/3.0f)*frobD))*D;
			sigma_f = frictionCoefficient*cell->pressure*(1.0f/(sqrt(1.0f/3.0f)*frobD))*D;
		else
			sigma_f = math::Matrix44f::Zero();


		// compute sigma_rigid from D
		sigma_rigid = -(D*cellSize*cellSize*cellSize)/dt;

		// compute frobenius norm of sigma_rigid
		float frobSigma_rigid = math::frobeniusNorm( sigma_rigid );

		// compute frobenius norm of sigma_f
		float frobSigma_f =  math::frobeniusNorm( sigma_f );


		float cohesion = 0.01f;
		//cohesion = 0.01f;
		//cohesion = 0.005f;
		//cohesion = 0.005f;

		cell->meanStress = cell->pressure;
		cell->shearStress = sqrt(3.0f/2.0f)*math::frobeniusNorm( sigma_rigid );  // zhou?
		// use sigma_rigid to check for yield-condition
		//if( frobD < frictionCoefficient*cell->pressure )
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < frictionCoefficient*cell->pressure )
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < sqrt(3.0f)*frobSigma_f/sqrt(2.0f) )
		//if( frobD < frictionCoefficient*cell->pressure*cell->pressure*3.0f )
		//if( frobSigma_rigid < frictionCoefficient*cell->pressure*cell->pressure*3.0f )
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < frictionCoefficient*cell->pressure*cell->pressure*2.0f + cohesion )
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < frictionCoefficient*cell->pressure + cohesion )
		//if( frobSigma_rigid < frictionCoefficient*cell->pressure )
		//if( frobSigma_rigid < frobSigma_f )
		if( cell->shearStress < frictionCoefficient*cell->meanStress + cohesion ) // sand
		{
			// cell is rigid
			cell->rigid = true;
			// store stress tensor
			cell->strainTensor = sigma_rigid;
		}else
			// cell is not rigid -> there is flow
			cell->strainTensor = sigma_f;

		cell->meanStress = frictionCoefficient*cell->meanStress + cohesion;
	}


	// rigidify all chunks/groups of connected rigid fluid cells -----------
	std::vector< std::vector<Cell *> > groups;  // list of cellgroups - the cellgroups are chuncks of connected rigid cells

	// find all groups
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// get reference to current cell
		Cell *cell = &((*it).second);

		if( cell->rigid )
		{
			// we have found a rigid cell
			// is the cell is not already within a group
			if( cell->layer == -1 )
			{
				// we will add a new group...
				groups.push_back( std::vector<Cell *>() );

				// ...and find all cells which belong to this group through region growing
				findGroup( cell, groups[groups.size()-1] );
			}
		}
	}

	// rigidify all groups
	for( size_t i=0; i<groups.size(); ++i )
		rigidify( groups[i] );



	// apply friction force (sigma_f) to all non-rigid fluid cells -------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// get reference to current cell
		Cell *cell = &((*it).second);

		// if cell neighbours not any fluid cell, or if the cell is a solid cell
		if( !(cell->xNeighboursFluid || cell->yNeighboursFluid || cell->zNeighboursFluid) || (cell->type == Cell::Solid) )
			// skip
			continue;

		if( !cell->rigid )
		{
			// non-rigid fluid cell -> apply internal coloumb friction

			// compute divergence of the stress tensor which is stored at the cell
			// tensor E ist a matrix which consists of 3 column vectors c1, c2 ,c3
			// the divergence of tensor E is the vector of the gradients of these
			// column vectors divE = (divc1, divc2, divc3)
			// we approximate the gradient using standard central differences of neighbouring cells
			// note: thats why we have stored sigma_rigid with each cell

			//math::Vec3f divStressTensor( divc1, divc2, divc3 );
			math::Vec3f divE = computeDivE( cell );

			// apply euler step
			if( cell->xNeighboursFluid )
				cell->velocity.x += dt*divE.x;
			if( cell->yNeighboursFluid )
				cell->velocity.y += dt*divE.y;
			if( cell->zNeighboursFluid )
				cell->velocity.z += dt*divE.z;

		}
	}
}

//
// will add all cells which can reach the given cell through neighbourhood
// accesses to the groupvector
//
void SandSimulator3d::findGroup( Cell *cell, std::vector<Cell *> &group )
{
	int groupId = (int)group.size();

	// add current cell to the group and tag it with the group Id
	cell->layer = groupId;
	group.push_back( cell );

	// perform until no more cells can be found
	bool done = false;
	while( !done )
	{
		done = true;

		// go through all cells...
		for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		{
			// get reference to current cell
			Cell *c = &((*it).second);

			//...and look for all rigid cells which dont belong to a group
			if( c->rigid && (c->layer == -1) )
			{
				// now check if the current cell neighbours a rigid cell which
				// belongs to the current group
				for( int i=0; i<6; ++i )
					// is the current neighbour rigid and does it belong to the current group?
					if( c->neighbours[i] && c->neighbours[i]->rigid && (c->neighbours[i]->layer == groupId) )
					{
						// yes, then add the current cell to the group and continue with the next cell
						c->layer = groupId;
						group.push_back( c );

						// we are done when we dont add a cell in a loop
						done = false;

						break;
					}
			}
		}
	};
}

//
// will project the velocities of all cells within the
//  group into the space of the groupd rigid body motion
//
void SandSimulator3d::rigidify( std::vector<Cell *> &group )
{
	// these 2 booleans indicate wether one of the cells of the current group
	// borders the boundary domain in x or y.
	// because if they do, the velocity in the respective direction has to
	// be zero (rigidbody collision)
	bool bordersLeft = false;
	bool bordersRight = false;
	bool bordersTop = false;
	bool bordersBottom = false;
	bool bordersFront = false;
	bool bordersBack = false;

	math::BoundingBox bounds;

	bounds.minPoint = simulationDomain.minPoint + cellSize/2.0f;
	bounds.maxPoint = simulationDomain.maxPoint + cellSize/2.0f;

	// compute the center of mass
	math::Vec3f center;

	for( size_t i=0; i<group.size(); ++i )
	{
		center += group[i]->center;

		// notify if the current cell in the group is on the boundary of the simulation
		if( group[i]->center.x <= bounds.minPoint.x )
			bordersLeft = true;
		if( group[i]->center.x >= bounds.maxPoint.x )
			bordersRight = true;
		if( group[i]->center.y <= bounds.minPoint.y )
			bordersBottom = true;
		if( group[i]->center.y >= bounds.maxPoint.y )
			bordersTop = true;
		if( group[i]->center.z <= bounds.minPoint.z )
			bordersFront = true;
		if( group[i]->center.z >= bounds.maxPoint.z )
			bordersBack = true;

	}

	center /= (float)group.size();

	// compute total linear momentum and total angular momentum
	math::Vec3f v;  // total linear momentum
	math::Vec3f w;  // total angular momentum

	for( size_t i=0; i<group.size(); ++i )
	{
		v += group[i]->velocity;

		math::Vec3f r = group[i]->center - center;
		w += math::crossProduct( r, group[i]->velocity );
	}

	v /= (float)group.size();
	w /= (float)group.size();
	

	// set velocity
	for( size_t i=0; i<group.size(); ++i )
	{
		math::Vec3f r = group[i]->center - center;
		math::Vec3f r_cross_w = math::crossProduct( r, w );

		group[i]->velocity = v + r_cross_w;
		//group[i]->velocity = v;

		if( (group[i]->velocity.x < 0.0f)&&(bordersLeft) )
			group[i]->velocity.x = 0.0f;
		if( (group[i]->velocity.x > 0.0f)&&(bordersRight) )
			group[i]->velocity.x = 0.0f;
		if( (group[i]->velocity.y < 0.0f)&&(bordersBottom) )
			group[i]->velocity.y = 0.0f;
		if( (group[i]->velocity.y > 0.0f)&&(bordersTop) )
			group[i]->velocity.y = 0.0f;
		if( (group[i]->velocity.z < 0.0f)&&(bordersFront) )
			group[i]->velocity.z = 0.0f;
		if( (group[i]->velocity.z > 0.0f)&&(bordersBack) )
			group[i]->velocity.z = 0.0f;


		if( bordersLeft || bordersRight )
			group[i]->velocity.x = 0.0f;
		if( bordersTop || bordersBottom )
			group[i]->velocity.y = 0.0f;
		if( bordersFront || bordersBack )
			group[i]->velocity.z = 0.0f;



		group[i]->velocity = v + r_cross_w;
		//group[i]->velocity = v;
		//group[i]->velocity = math::Vec2f();
	}
}

//
// This method is a modified version of the FluidSimulator2d::updateVelocity
// method to reflect sand behavior
//
void SandSimulator3d::updateVelocity( float dt )
{
	//FluidSimulator3d::updateVelocity( dt );
	//return;

	// store the velocity in the oldVelocity member of each cell so that we later can compute the change in velocity --------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.oldVelocity = (*it).second.velocity;

	// if advection is not done through particles (PIC/FLIP)
	if( !particleBasedAdvection )
		advectGridVelocities( dt );


	printf( "            applying external forces\n" );
	// apply external forces ----------------------------------
	applyGravity( dt, 0.0f, -9.1f, 0.0f );


	// apply viscosity ----------------------------------------
	solveViscosity( dt );



	printf( "            extrapolating velocities\n" );
	// extrapolate velocities into surrounding buffer cells ------
	extrapolateVelocities();
	// set boundary conditions...
	setBoundaryConditions();


	// compute pressure ------------------------------------------
	solvePressure( dt );


	printf( "            extrapolating velocities\n" );
	// extrapolate velocity into buffer cells second time -----------------
	extrapolateVelocities();
	// set boundary conditions...
	setBoundaryConditions();


	// sand
	applySandModel( dt );

	// extrapolate velocity into buffer cells second time -----------------
	//extrapolateVelocities();
	// set solid cell velocities ------------------------------
	//setBoundaryConditions();



	// compute the change in velocity using the oldVelocity member which we have stored at the beginning of this procedure ----------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.velocityChange = (*it).second.velocity - (*it).second.oldVelocity;

	// if advection is not done through particles (PIC/FLIP)
	if( particleBasedAdvection )
	{
		// update the velocities of the particles from the grid --------------------------------------
		for( std::vector<Particle3d>::iterator it=markerParticles.begin(); it != markerParticles.end(); ++it )
		{
			// the solution of PIC and FLIP are blended together according to a specified blendweight
			// FLIP
			math::Vec3f flipVelocity = (*it).velocity + getVelocityChange( (*it).position.x, (*it).position.y, (*it).position.z );
			// PIC
			math::Vec3f picVelocity = getVelocity( (*it).position.x, (*it).position.y, (*it).position.z );
			// FINAL
			(*it).velocity = PICFLIPWeight*flipVelocity + (1.0f - PICFLIPWeight)*picVelocity;
		}
	}
}



void SandSimulator3d::setBoundaryConditions( void )
{
	// set solid cell velocities ------------------------------

	// to prevent fluid from entering solid objects we have to set the velocity-components
	// which point into solid-cells from liquid or air-cells to zero
	// velocity-components that point out of solid-cells or lie between the border of
	// 2 solid-cells are left unchanged

	// first we set the velocity components which are between fluid and solid cells to 0 --------

	// iterate over all cells
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// the idea is, to check all velocity-components of fluid and air cells for pointing out of the cell and check whether these
		// components point into solid cells AND to check all solid-cell velocity-components for the components which point
		// into fluid cells -> and set these to zero
		if( ((*it).second.type == Cell::Fluid) || ((*it).second.type == Cell::Air) )
		{
			// outgoing velocity-components must not go into solid cells
			// does the left neighbour exist?
			if( (*it).second.neighbours[0] )
				// yes! - is it a solid cell?
				if( (*it).second.neighbours[0]->type == Cell::Solid )
					// then set the velocity-component of the fluid cell to zero
					(*it).second.velocity.x = 0.0f;
			// check whether the bottom neighbour exists
			if( (*it).second.neighbours[2] )
				// yes! - is it a solid cell?
				if( (*it).second.neighbours[2]->type == Cell::Solid )
					// then set the velocity-component of the fluid cell to zero
					(*it).second.velocity.y = 0.0f;
			// check whether the bottom neighbour exists
			if( (*it).second.neighbours[4] )
				// yes! - is it a solid cell?
				if( (*it).second.neighbours[4]->type == Cell::Solid )
					// then set the velocity-component of the fluid cell to zero
					(*it).second.velocity.z = 0.0f;
		}else
		{
			(*it).second.velocity.x = 0.0f;
			(*it).second.velocity.y = 0.0f;
			(*it).second.velocity.z = 0.0f;
		}
	}


	// next we set the velocity components which are between solid and solid cells to 0 if they dont neighbour
	// fluid cells or we set accordingly so that no-slip or free-slip boundary conditions are met
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &((*it).second);


		if( cell->type != Cell::Solid )
			continue;


		// cell is solid

		// check if it neighbours a solid cell in x
		if( cell->neighbours[0] && (cell->neighbours[0]->type == Cell::Solid) )
		{
			// then we have to set tangential velocity so that no-slip or free-slip boundary conditions are met

			// there are 4 potential neighbours which may be a fluid cell and therefore should be
			// considered - but only one of them can be taken into account
			// y-
			if( cell->neighbours[2] && (cell->neighbours[2]->type != Cell::Solid) )
			{
				cell->velocity.x = -cell->neighbours[2]->velocity.x;
			}else
			// y+
			if( cell->neighbours[3] && (cell->neighbours[3]->type != Cell::Solid) )
			{
				cell->velocity.x = -cell->neighbours[3]->velocity.x;
			}else
			// z-
			if( cell->neighbours[4] && (cell->neighbours[4]->type != Cell::Solid) )
			{
				cell->velocity.x = -cell->neighbours[4]->velocity.x;
			}else
			// z+
			if( cell->neighbours[5] && (cell->neighbours[5]->type != Cell::Solid) )
			{
				cell->velocity.x = -cell->neighbours[5]->velocity.x;
			}
		}
		// check if it neighbours a solid cell in y
		if( cell->neighbours[2] && (cell->neighbours[2]->type == Cell::Solid) )
		{
			// then we have to set tangential velocity so that no-slip or free-slip boundary conditions are met

			// there are 4 potential neighbours which may be a fluid cell and therefore should be
			// considered - but only one of them can be taken into account
			// x-
			if( cell->neighbours[0] && (cell->neighbours[0]->type != Cell::Solid) )
			{
				cell->velocity.y = cell->neighbours[0]->velocity.y;
			}else
			// x+
			if( cell->neighbours[1] && (cell->neighbours[1]->type != Cell::Solid) )
			{
				cell->velocity.y = cell->neighbours[1]->velocity.y;
			}else
			// z-
			if( cell->neighbours[4] && (cell->neighbours[4]->type != Cell::Solid) )
			{
				cell->velocity.y = cell->neighbours[4]->velocity.y;
			}else
			// z+
			if( cell->neighbours[5] && (cell->neighbours[5]->type != Cell::Solid) )
			{
				cell->velocity.y = cell->neighbours[5]->velocity.y;
			}
		}
		// check if it neighbours a solid cell in z
		if( cell->neighbours[4] && (cell->neighbours[4]->type == Cell::Solid) )
		{
			// then we have to set tangential velocity so that no-slip or free-slip boundary conditions are met

			// there are 4 potential neighbours which may be a fluid cell and therefore should be
			// considered - but only one of them can be taken into account
			// x-
			if( cell->neighbours[0] && (cell->neighbours[0]->type != Cell::Solid) )
			{
				cell->velocity.z = -cell->neighbours[0]->velocity.z;
			}else
			// x+
			if( cell->neighbours[1] && (cell->neighbours[1]->type != Cell::Solid) )
			{
				cell->velocity.z = -cell->neighbours[1]->velocity.z;
			}else
			// y-
			if( cell->neighbours[2] && (cell->neighbours[2]->type != Cell::Solid) )
			{
				cell->velocity.z = -cell->neighbours[2]->velocity.z;
			}else
			// y+
			if( cell->neighbours[3] && (cell->neighbours[3]->type != Cell::Solid) )
			{
				cell->velocity.z = -cell->neighbours[3]->velocity.z;
			}
		}
	}
}



//
// this will compute the rate of strain tensor D for the given cell
//
math::Matrix44f SandSimulator3d::computeStrainRateTensor( Cell *cell )
{
	math::Matrix44f D = math::Matrix44f::Zero();

	D.m[0][0] = cell->neighbours[1]->velocity.x - cell->velocity.x;
	D.m[0][1] = cell->neighbours[1]->velocity.y - cell->velocity.y;
	D.m[0][2] = cell->neighbours[1]->velocity.z - cell->velocity.z;
	D.m[1][0] = cell->neighbours[3]->velocity.x - cell->velocity.x;
	D.m[1][1] = cell->neighbours[3]->velocity.y - cell->velocity.y;
	D.m[1][2] = cell->neighbours[3]->velocity.z - cell->velocity.z;
	D.m[2][0] = cell->neighbours[5]->velocity.x - cell->velocity.x;
	D.m[2][1] = cell->neighbours[5]->velocity.y - cell->velocity.y;
	D.m[2][2] = cell->neighbours[5]->velocity.z - cell->velocity.z;

	math::Matrix44f Dtransposed = D;
	Dtransposed.transpose();

	D = (D + Dtransposed)*0.5f;

	return D*(1.0f/cellSize);
}



math::Vec3f SandSimulator3d::computeDivE( Cell *cell )
{
	// compute divergence of the strain tensor which is stored at the cell
	// Tensor E ist a matrix which consists of 3 column vectors c1, c2 ,c3
	// the divergence of tensor E is the vector of the gradients of these
	// column vectors divE = (divc1, divc2, divc3)
	//float divC1 = (cell->strainTensor.m[0][0] - cell->neighbours[0]->strainTensor.m[0][0])+
	//			  (cell->neighbours[3]->strainTensor.m[1][0] - cell->strainTensor.m[1][0]);
	//float divC2 = (cell->neighbours[1]->strainTensor.m[0][1] - cell->strainTensor.m[0][1])+
	//			  (cell->strainTensor.m[1][1] - cell->neighbours[2]->strainTensor.m[1][1]);
	//return math::Vec2f( divC1/cellSize, divC2/cellSize );

	float divC1 = (cell->neighbours[1]->strainTensor.m[0][0] - cell->neighbours[0]->strainTensor.m[0][0])/2.0f+
				  (cell->neighbours[3]->strainTensor.m[1][0] - cell->neighbours[2]->strainTensor.m[1][0])/2.0f+
				  (cell->neighbours[5]->strainTensor.m[2][0] - cell->neighbours[4]->strainTensor.m[2][0])/2.0f;
	float divC2 = (cell->neighbours[1]->strainTensor.m[0][1] - cell->neighbours[0]->strainTensor.m[0][1])/2.0f+
				  (cell->neighbours[3]->strainTensor.m[1][1] - cell->neighbours[2]->strainTensor.m[1][1])/2.0f+
				  (cell->neighbours[5]->strainTensor.m[2][1] - cell->neighbours[4]->strainTensor.m[2][1])/2.0f;
	float divC3 = (cell->neighbours[1]->strainTensor.m[0][2] - cell->neighbours[0]->strainTensor.m[0][2])/2.0f+
				  (cell->neighbours[3]->strainTensor.m[1][2] - cell->neighbours[2]->strainTensor.m[1][2])/2.0f+
				  (cell->neighbours[5]->strainTensor.m[2][2] - cell->neighbours[4]->strainTensor.m[2][2])/2.0f;

	return math::Vec3f( divC1/cellSize, divC2/cellSize, divC3/cellSize );
}