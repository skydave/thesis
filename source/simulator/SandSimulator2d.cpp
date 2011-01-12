/*---------------------------------------------------------------------

This simulator derives from FluidSimulator2d and overloads the
updateVelocity method. There are some small additions which will
make the fluid behave like sand.
The idea and theory behind that method was introduced in the paper
"Animating Sand as a Fluid" at Siggraph in 2005 by Zhou/Bridson.

----------------------------------------------------------------------*/
#include "SandSimulator2d.h"


//
// constructor
//
SandSimulator2d::SandSimulator2d()
{
	//setViscosity( 1500.0f );
	particleBasedAdvection = false;
}

//
// takes the coloumb friction into account and evaluates
// the mohr-coloumb yield-condition to change the fluid
// velocity so that it behaves like sand
//
void SandSimulator2d::applySandModel( float dt )
{
	// prepare cells -------------------------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.rigid        = false;
		(*it).second.layer        = -1;    // used to find groups of connected rigid cells
		(*it).second.strainTensor =  math::Matrix22f::Zero();
		(*it).second.vonMisesEquivalentStress = 0.0f;

		(*it).second.center =  math::Vec2f( (*it).first.i*cellSize + cellSize*0.5f, (*it).first.j*cellSize + cellSize*0.5f );
	}


	// check yield-condition for every fluid cell --------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// get reference to current cell
		Cell *cell = &((*it).second);

		if( cell->type != Cell::Fluid )
			continue;
		//if( !(cell->xNeighboursFluid || cell->yNeighboursFluid) )
		//	continue;

		// compute strain rate D from strain rate tensor D and the frobenius norm of D
		math::Matrix22f           D;  // strain rate tensor
		math::Matrix22f     sigma_f;  // friction stress tensor in yielding area (dynamic friction)
		math::Matrix22f sigma_rigid;  // friction stress tensor in rigid areas (static friction)

		D = math::Matrix22f::Zero();
		sigma_f = math::Matrix22f::Zero();
		sigma_rigid = math::Matrix22f::Zero();

		// compute the rate of strain tensor D
		D = computeStrainRateTensor( cell );

		// compute frobenius norm of sigma_rigid
		float frobD = math::frobeniusNorm( D );  // frobenius Norm of the rate of strain tensor D

		//float frictionCoefficient = 100.6f;
		//float frictionCoefficient = 60.6f;
		float frictionCoefficient = .8f;

		// compute sigma_f from D
		if( frobD > 0.0f )
			sigma_f = frictionCoefficient*cell->pressure*(1.0f/(sqrt(1.0f/2.0f)*frobD))*D;
			//sigma_f = -frictionCoefficient*cell->pressure*(D*cellSize*cellSize)/frobD;
			//sigma_f = -frictionCoefficient*cell->pressure*D*cellSize;
			//sigma_f = frictionCoefficient*cell->pressure*D*cellSize;
			//sigma_f = -frictionCoefficient*dt*D;
		else
			sigma_f = math::Matrix22f::Zero();

		// compute sigma_rigid from D
		sigma_rigid = -(D*cellSize*cellSize)/dt;

		float cohesion = 0.0001f;
		//cohesion = 0.01f;
		//cohesion = 0.005f;
		//cohesion = 0.0045f;
		//cohesion = 0.00001f;

		cell->meanStress = cell->pressure;
		//cell->shearStress = sqrt(3.0f/2.0f)*frobD;  // zhou
		cell->shearStress = sqrt(3.0f/2.0f)*math::frobeniusNorm( sigma_rigid );  // zhou?
		//cell->shearStress = frobD;  // zhou
		//cell->shearStress = frobD;  // zhou
		//cell->shearStress = sqrt( 3.0f*(D.m[0][0]*D.m[1][1] - D.m[0][1]*D.m[0][1]) ); // wiki -> problem: root from negative value
		//cell->shearStress = sqrt( D.m[0][0]*D.m[0][0] + D.m[1][1]*D.m[1][1] - D.m[0][0]*D.m[1][1] + 1*D.m[0][1] ); // ...
		//cell->shearStress = sqrt( D.m[0][0]*D.m[0][0] + D.m[1][1]*D.m[1][1] - D.m[0][0]*D.m[1][1] + 3*D.m[0][1] ); // ...
		//float t = sigma2.m[0][0]*sigma2.m[0][0] + sigma2.m[1][1]*sigma2.m[1][1] - sigma2.m[0][0]*sigma2.m[1][1] + 3*sigma2.m[0][1];
		//if( t < 0.0f )
		//	printf( "---->  " );
		//cell->shearStress = sqrt( sigma2.m[0][0]*sigma2.m[0][0] + sigma2.m[1][1]*sigma2.m[1][1] - sigma2.m[0][0]*sigma2.m[1][1] + 3*sigma2.m[0][1] ); // ...

		//printf( "%f           %f\n", cell->shearStress, cell->meanStress );

		// use sigma_rigid to check for yield-condition
		//if( math::frobeniusNorm( D ) < frictionCoefficient*sqrt(cell->pressure*cell->pressure*2.0f) )
		//if( math::frobeniusNorm( D ) < frictionCoefficient*sqrt(cell->pressure*cell->pressure*2.0f) )
		//if( sqrt(3.0f)*math::frobeniusNorm( D )/sqrt(2.0f) < frictionCoefficient*cell->pressure )
		//if( math::frobeniusNorm( D )/sqrt(2.0f) < 10.0*cell->pressure )
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < frictionCoefficient*cell->pressure )
		//printf( "%f   <   %f \n",  frobSigma_rigid/sqrt(2.0f), frictionCoefficient*cell->pressure + cohesion );
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < frictionCoefficient*cell->pressure + cohesion )
		//if( sqrt(3.0f)*frobD/sqrt(2.0f) < frictionCoefficient*cell->pressure*cell->pressure*2.0f )
		//if( frobSigma_rigid < frictionCoefficient*cell->pressure*cell->pressure*3.0f )
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < frictionCoefficient*cell->pressure*cell->pressure*2.0f + cohesion )
		//if( sqrt(3.0f)*frobSigma_rigid/sqrt(2.0f) < frictionCoefficient*cell->pressure + cohesion ) //*
		//if( frobSigma_rigid < frictionCoefficient*cell->pressure )
		//if( sqrt(3.0f)*math::frobeniusNorm( sigma2 )/sqrt(2.0f) < frictionCoefficient*cell->pressure ) // **
		//if( sqrt(3.0f)*math::frobeniusNorm( sigma2 )/sqrt(2.0f) < frictionCoefficient*cell->pressure + cohesion ) // **
		//if( frobSigma_rigid < frictionCoefficient*sqrt(cell->pressure*cell->pressure*2.0f) )
		//if( frictionCoefficient*sqrt(3.0f)*math::frobeniusNorm( testm )/sqrt(2.0f) < frictionCoefficient*cell->pressure + cohesion )
		//if( sigma_v < frictionCoefficient*cell->pressure + cohesion )
		//if( cell->shearStress < 35.0f*cell->meanStress )
		if( cell->shearStress < frictionCoefficient*cell->meanStress + cohesion ) // sand
		//if( (cell->pressure < 3.0f) && ( cell->shearStress < frictionCoefficient*cell->meanStress + cohesion ) ) // galcier-test
		{
			// cell is rigid
			cell->rigid = true;
			// store stress tensor
			cell->strainTensor = sigma_rigid;
		}else
			// cell is not rigid -> there is flow
			cell->strainTensor = sigma_f;

		// store equivalent stress
		cell->vonMisesEquivalentStress = cell->shearStress;
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

		if( !(cell->xNeighboursFluid || cell->yNeighboursFluid) )
			continue;

		//if( (!cell->rigid)||(cell->rigid && cell->neighbours[0]->type == Cell::Air)||(cell->rigid && cell->neighbours[2]->type == Cell::Air) )
		if( !cell->rigid )
		{
			// non-rigid fluid cell -> apply internal coloumb friction

			// compute divergence of the stress tensor which is stored at the cell
			// tensor E ist a matrix which consists of 3 column vectors c1, c2 ,c3
			// the divergence of tensor E is the vector of the gradients of these
			// column vectors divE = (divc1, divc2, divc3)
			// we approximate the gradient using standard central differences of neighbouring cells
			// note: thats why we have stored sigma_rigid with each cell
			math::Vec2f divE = computeDivE( cell );

			// apply euler step
			if( cell->xNeighboursFluid )
				cell->velocity.x += dt*divE.x;
			if( cell->yNeighboursFluid )
				cell->velocity.y += dt*divE.y;
		}
	}
}

//
// will add all cells which can reach the given cell through neighbourhood
// accesses to the groupvector
//
void SandSimulator2d::findGroup( Cell *cell, std::vector<Cell *> &group )
{
	// first we add the given cell
	cell->layer = 0;
	group.push_back( cell );

	// now add all direct neighbours if they are not within a group
	for( int i=0; i<4; ++i )
		if(cell->neighbours[i])  
		if( (cell->neighbours[i]->layer == -1)&&(cell->rigid) )
		{
			findGroup( cell->neighbours[i], group );
		}

}

//
// will project the velocities of all cells within the
//  group into the space of the groupd rigid body motion
//
void SandSimulator2d::rigidify( std::vector<Cell *> &group )
{
	// these 2 booleans indicate wether one of the cells of the current group
	// borders the boundary domain in x or y.
	// because if they do, the velocity in the respective direction has to
	// be zero (rigidbody collision)
	bool bordersLeft = false;
	bool bordersRight = false;
	bool bordersTop = false;
	bool bordersBottom = false;

	math::BoundingBox bounds;

	bounds.minPoint = simulationDomain.minPoint + cellSize/2.0f;
	bounds.maxPoint = simulationDomain.maxPoint + cellSize/2.0f;

	// compute the center of mass
	math::Vec2f center;

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

	}

	center /= (float)group.size();

	// compute total linear momentum and total angular momentum
	math::Vec2f v;  // total linear momentum
	float w = 0.0f;  // total angular momentum

	for( size_t i=0; i<group.size(); ++i )
	{
		v += group[i]->velocity;

		math::Vec2f r = group[i]->center - center;
		w += math::crossProduct( r, group[i]->velocity );
	}

	v /= (float)group.size();
	w /= (float)group.size();
	

	// set velocity
	for( size_t i=0; i<group.size(); ++i )
	{
		Cell *cell  =group[i];

		if( (cell->neighbours[2]->type == Cell::Air)||(cell->neighbours[2]->type == Cell::Air) )
			continue;

		math::Vec2f r = group[i]->center - center;
		math::Vec3f r3(r.x, r.y, 0.0f);
		math::Vec3f w3(0.0f, 0.0f, w);
		math::Vec3f rot = math::crossProduct( r3, w3 );
		math::Vec2f r_cross_w(rot.x, rot.y);

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

		if( bordersLeft || bordersRight )
			group[i]->velocity.x = 0.0f;
		if( bordersTop || bordersBottom )
			group[i]->velocity.y = 0.0f;

		if( cell->neighbours[2]->type != Cell::Air )
			group[i]->velocity.y = v.y + r_cross_w.y;

		if( cell->neighbours[0]->type != Cell::Air )
			group[i]->velocity.x = v.x + r_cross_w.x;

		//group[i]->velocity = v + r_cross_w;
		//group[i]->velocity = v;
		//group[i]->velocity = math::Vec2f();
	}

}

//
// This method is a modified version of the FluidSimulator2d::updateVelocity
// method to reflect sand behavior
//
void SandSimulator2d::updateVelocity( float dt )
{
	//FluidSimulator2d::updateVelocity(dt);
	//return;

	// store the velocity in the oldVelocity member of each cell so that we later can compute the change in velocity --------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.oldVelocity = (*it).second.velocity;

	// if advection is not done through particles (PIC/FLIP)
	if( !particleBasedAdvection )
		// then we use a sem-lagrange method to advect velocities...
		advectGridVelocities( dt );

	// apply external forces ----------------------------------
	applyGravity( dt, 0.0f, -9.1f );


	// apply viscosity ----------------------------------------
	solveViscosity( dt );




	// extrapolate velocities into surrounding buffer cells ------
	extrapolateVelocities();
	// set Boundary conditions -------
	setBoundaryConditions();


	// solve for pressure and make the velocity grid divergence free ----
	solvePressure( dt );

	// extrapolate velocity into buffer cells second time -----------------
	extrapolateVelocities();
	// set solid cell velocities ------------------------------
	setBoundaryConditions();

	// sand
	applySandModel( dt );

	// extrapolate velocity into buffer cells second time -----------------
	//extrapolateVelocities();
	// set solid cell velocities ------------------------------
	//setBoundaryConditions();



	// compute the change in velocity using the oldVelocity member which we have stored at the beginning of this procedure ----------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.velocityChange.x = (*it).second.velocity.x - (*it).second.oldVelocity.x;
		(*it).second.velocityChange.y = (*it).second.velocity.y - (*it).second.oldVelocity.y;
	}


	// if advection is not done through particles (PIC/FLIP)
	if( particleBasedAdvection )
	{
		// update the velocities of the particles from the grid --------------------------------------
		for( std::vector<Particle2d>::iterator it=markerParticles.begin(); it != markerParticles.end(); ++it )
		{
			// the solution of PIC and FLIP are blended together according to a specified blendweight
			// FLIP
			math::Vec2f flipVelocity = (*it).velocity + getVelocityChange( (*it).position.x, (*it).position.y );
			// PIC
			math::Vec2f picVelocity = getVelocity( (*it).position.x, (*it).position.y );
			// FINAL
			(*it).velocity = PICFLIPWeight*flipVelocity + (1.0f - PICFLIPWeight)*picVelocity;
		}
	}
}


//
// this method will set the velocities on solid cells which border
// non-solid cells so that coloumb friction is reflected in the model
//
void SandSimulator2d::setBoundaryConditions( void )
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
				{
					// then set the velocity-component of the fluid cell to zero
					//(*it).second.velocity.x = 0.0f;
					(*it).second.velocity.y = 0.0f;
				}
		}else
		{
			// set normal components to 0 - tangential components will be set in a second pass
			(*it).second.velocity.x = 0.0f;
			(*it).second.velocity.y = 0.0f;
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

			// there are 2 potential neighbours which may be a fluid cell and therefore should be
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
			}
		}
	}
}
















//
// this will compute the rate of strain tensor D for the given cell
//
math::Matrix22f SandSimulator2d::computeStrainRateTensor( Cell *cell )
{
	math::Matrix22f D = math::Matrix22f::Zero();

	D.m[0][0] = (cell->neighbours[1]->velocity.x - cell->neighbours[0]->velocity.x)/2.0f;
	D.m[0][1] = (cell->neighbours[1]->velocity.y - cell->neighbours[0]->velocity.y)/2.0f;
	D.m[1][0] = (cell->neighbours[3]->velocity.x - cell->neighbours[2]->velocity.x)/2.0f;
	D.m[1][1] = (cell->neighbours[3]->velocity.y - cell->neighbours[2]->velocity.y)/2.0f;

	math::Matrix22f Dtransposed = D;
	Dtransposed.transpose();

	D = (D + Dtransposed)*0.5f;

	return D*(1.0f/cellSize);
}

//
//
//
math::Matrix22f SandSimulator2d::test( Cell *cell )
{
	math::Matrix22f D = math::Matrix22f::Zero();

	D.m[0][0] = 2.0f*(cell->neighbours[1]->velocity.x - cell->neighbours[0]->velocity.x)/2.0f;
	D.m[1][1] = 2.0f*(cell->neighbours[3]->velocity.y - cell->neighbours[2]->velocity.y)/2.0f;
	D.m[0][1] = (cell->neighbours[3]->velocity.x - cell->neighbours[2]->velocity.x)/2.0f + (cell->neighbours[1]->velocity.y - cell->neighbours[0]->velocity.y)/2.0f;
	D.m[1][0] = D.m[0][1];

	return D*(1.0f/cellSize);
}

math::Vec2f SandSimulator2d::computeDivE(FluidSimulator2d::Cell *cell)
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
				  (cell->neighbours[3]->strainTensor.m[1][0] - cell->neighbours[2]->strainTensor.m[1][0])/2.0f;
	float divC2 = (cell->neighbours[1]->strainTensor.m[0][1] - cell->neighbours[0]->strainTensor.m[0][1])/2.0f+
				  (cell->neighbours[3]->strainTensor.m[1][1] - cell->neighbours[2]->strainTensor.m[1][1])/2.0f;
	return math::Vec2f( divC1/cellSize, divC2/cellSize );
/*
	float divC1 = (cell->strainTensor.m[0][0] - cell->neighbours[0]->strainTensor.m[0][0])+ // backward difference (diagonal element)
				  (cell->neighbours[3]->strainTensor.m[1][0] - cell->strainTensor.m[1][0]); // forward difference (off-diagonal element)
	float divC2 = (cell->neighbours[1]->strainTensor.m[0][1] - cell->strainTensor.m[0][1])+
				  (cell->strainTensor.m[1][1] - cell->neighbours[2]->strainTensor.m[1][1]);

	return math::Vec2f( divC1/cellSize, divC2/cellSize );
*/
}