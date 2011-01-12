/*---------------------------------------------------------------------

MAC bases fluid simulator which uses a staggered grid and particles
or tracking the fluid. In addition to basic MAC simulation the PIC
and FLIP methods can be used to achieve better animations.

----------------------------------------------------------------------*/
#include "FluidSimulator2d.h"






// -------------------- FluidSimulator2d ------------------------------------------

//
// constructor
//
FluidSimulator2d::FluidSimulator2d()
{
	//setCellSize( 0.025f );
	setCellSize( 0.01f );
	//setCellSize( 0.1f );

	setSimulationDomain( math::Vec2f( 0.0f, 0.0f ), math::Vec2f( 1.0f, 1.0f ) );

	setViscosity( 0.0f );

	setDeltaTimeRange( 0.001f, 0.01f );
	//setDeltaTimeRange( 0.00001f, 0.001f );

	particleBasedAdvection = true;
	PICFLIPWeight = 0.95f;

	atmosphericPressure = 0.0f;
}

//
// destructor
//
FluidSimulator2d::~FluidSimulator2d()
{
}

//
// advances a specified timestep with a dynamic amount of substeps depending
// on the CFL-condition and dt min/max values.
// (a conviencen function)
//
void FluidSimulator2d::advanceFrame( float timePerFrame )
{
	// we wont do anything if there are no particles
	if( markerParticles.size() == 0 )
		return;

	printf( "advanceFrame( %f )\n", timePerFrame );

	float t=0;         // the time which already had been computed in this step
	float  deltaTime;  // deltaTime for the next substep
	bool finished=false; // indicates when we are done

	// as long as we dont have reached the specified timePerFrame
	while(!finished)
	{
		// get the deltaTime which makes sure that the CFL condition will be satisfied
		deltaTime = CFLDeltaTime();

		// do we exceed the timePerFrame with the next simulation step?
		if( t+deltaTime>=timePerFrame )
		{
			// then step only the timeamount forward which remains
			deltaTime=timePerFrame-t;
			//were done after the last
			finished=true;
		}else
			// 
			if(t+1.5*deltaTime>=timePerFrame)
				deltaTime=0.5f*(timePerFrame-t);

		printf( "    advanceStep( %f )    t= %f/%f\n", deltaTime, t, timePerFrame );
		advanceStep( deltaTime );

		// keep track of the time we already have advanced
		t+=deltaTime;
	}
}

//
// advances the fluid the given timedelta
//
void FluidSimulator2d::advanceStep( float deltaTime )
{
	// we wont do anything if there are no particles
	if( markerParticles.size() == 0 )
		return;

	// first move the particles through the grid using 5 sub RK2 steps 
	for( unsigned int i=0; i<5; ++i )
	{
		printf( "         moveParticles( %f )\n", 0.2f*deltaTime );
		moveParticles( 0.2f*deltaTime );
	}

	// update the grid (mark fluid cells, create new air cells, tag solid cells,
	// compute distances to fluid and extrapolate velocities, through away unused cells)
	printf( "         updateGrid()\n" );
	updateGrid();

	// solve the navier-stokes equation for all fluid cells
	printf( "         updateVelocity( %f )\n", deltaTime );
	updateVelocity( deltaTime );
}

//
// clears all gridcells and all marker particles
//
void FluidSimulator2d::reset( void )
{
	// remove all marker particles
	markerParticles.clear();
	// remove all gridcells
	gridCells.clear();
}

//
// setup the size of one grid cell in each dimension
//
void FluidSimulator2d::setCellSize( float _cellSize )
{
	if( _cellSize > 0.0f )
		cellSize = _cellSize;
}

//
// returns cellsize
//
float FluidSimulator2d::getCellSize()
{
	return cellSize;
}

//
// returns a pointer to the cell which lies at the specified coordinate
// if the cell does not exist, null is returned
//
FluidSimulator2d::Cell *FluidSimulator2d::getCell( size_t i, size_t j )
{
	// find the cell in the hashmap
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate(i,j) );

	if( it != gridCells.end() )
		// entry found
		return &(*it).second;

	return 0;
}


//
// set the range in which the delta time must be for one simulation step
//
void FluidSimulator2d::setDeltaTimeRange( float _dtMin, float _dtMax )
{
	if( _dtMin <= _dtMax )
	{
		dtMin = _dtMin;
		dtMax = _dtMax;
	}
}

//
// sets the spatial boundaries of the simulation
//
void FluidSimulator2d::setSimulationDomain( const math::Vec2f &min, const math::Vec2f &max )
{
	math::Vec3f _min( min.x, min.y, 0.0f );
	math::Vec3f _max( max.x, max.y, 0.0f );
	simulationDomain = math::BoundingBox( _min, _max );	
}


//
// sets the simple-fluid visosity
//
void FluidSimulator2d::setViscosity( float _viscosity )
{
	viscosity = _viscosity;
}

//
// This method evaluates the current maximum velocity within the grid and computes
// a timeDelta so that the condition velocity*timeDelta < cellSize is true for every
// velocity in the grid. This condition is called the CFL condition.
//
float FluidSimulator2d::CFLDeltaTime( void )
{
	// compute deltaTime so that the CFL-condition is satisfied (maxVelocity*deltaTime < cellSize)

	// first, we look for the maximum Velocity in each component
	float maxVelx = 0.0f;
	float maxVely = 0.0f;


	// iterate over all gridcells...
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// ...and look if there are velocity components which are larger then our current maximums
		if( fabs((*it).second.velocity.x) > maxVelx )
			maxVelx = fabs((*it).second.velocity.x);
		if( fabs((*it).second.velocity.y) > maxVely )
			maxVely = fabs((*it).second.velocity.y);
	}

	// compute the squared length of our maximum-velocity-vector
	float length2 = maxVelx*maxVelx + maxVely*maxVely;

	// if the length is very small
	if( length2 < 0.0000000000000001f )
		// then we return the minimum since CFL would cause a very very small value
		return dtMax;

	// compute the dt from CFL...
	float dt = cellSize / sqrt( length2 );

	// and clamp it to our dt min/max range
	if( dt > dtMax )
		return dtMax;
	if( dt < dtMin )
		return dtMin;

	// done
	return dt;
}



//
// This method will update the gridcells dynamicly depending on the marker particles.
// New cells are added if needed and no longer used cells are removed.
// New cells are cells where some marker particles have moved to.
// Unused cells are cells where all marker particles have left.
//
void FluidSimulator2d::updateGrid( void )
{
	// compute the transform matrices of all static solids
	for( std::vector<StaticSolid *>::iterator soIt = staticSolids.begin(); soIt != staticSolids.end(); ++soIt )
		(*soIt)->computeTransformationMatrix();


	// iterate all gridcells and set the layer to -1 (potentially unused) -------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.layer = -1;
		(*it).second.xNeighboursFluid = false;
		(*it).second.yNeighboursFluid = false;
	}

	// update and mark all cells which have currently fluid in it -----------------------------------------------
	// we do this by iterating over all markerParticles...
	for( std::vector<Particle2d>::iterator it=markerParticles.begin(); it != markerParticles.end(); ++it )
	{
		// is the particle within simulation bounds?
		if( !(((*it).position.x > simulationDomain.minPoint.x)&&((*it).position.x < simulationDomain.maxPoint.x)&&
			((*it).position.y > simulationDomain.minPoint.y)&&((*it).position.y < simulationDomain.maxPoint.y)))
			continue;

		// find the coordinates of the cell in which the current particle is located
		Cell::Coordinate cellCoord;

		cellCoord.i = (int)((*it).position.x / cellSize);
		cellCoord.j = (int)((*it).position.y / cellSize);

		// try to locate the cell in the hashtable
		stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellEntry = gridCells.find( cellCoord );
		
		// if the cell doesnt exist in the hash_table then end() is returned
		if( cellEntry == gridCells.end() )
		{
			// cell doesnt exist - create it
			cellEntry = gridCells.insert( std::pair<Cell::Coordinate, Cell>( cellCoord, Cell() ) ).first;
			// and update its neighbours
			updateNeighbours( &((*cellEntry).second), (*cellEntry).first );
			// cell creation callback
			onCellCreated( &cellEntry->first, &cellEntry->second );
		}

		// cell was created or did already exist - however, set it up
			
		// setup the type of the cell
		// is the cell not part of a solid object?
		//if( !(cell is part of solid object) )
			// mark it as fluid
			(*cellEntry).second.type = Cell::Fluid;
		//else
			// mark it as solid
			//(*cellEntry).second.type = Cell::Solid;

		// set layer to 0 marking the cell as "most important"
		// if we did not aready set the layer (touched the cell)
		if( (*cellEntry).second.layer == -1 )
			// set the layer
			(*cellEntry).second.layer = 0;

		// set the velocity-component-neighbours-fluid-indicator
		(*cellEntry).second.xNeighboursFluid = true;
		(*cellEntry).second.yNeighboursFluid = true;

	}

	// create a buffer zone of gridCells which wraps around all fluid cells ----------------------------
	for( unsigned int i=1; i<5; ++i )
	{
		bool tag = false;

		do
		{
			tag = false;
			// for each cell with layer == i-1
			for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
			{
				if( (*it).second.layer == i-1 )
				{
					stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator neighbour;
					// iterate over all potential neighbours
					for( unsigned int n = 0; n < 4; ++n )
					{
						// if the current neighbour already exists...
						neighbour = gridCells.find( (*it).first.getNeighbourCoordinate(n) );

						// cell not found?
						if( neighbour == gridCells.end() )
						{
							// create it
							neighbour = gridCells.insert( std::pair<Cell::Coordinate, Cell>( (*it).first.getNeighbourCoordinate(n), Cell() ) ).first;
							// and update its interpolation neighbour
							updateNeighbours( &((*neighbour).second), (*neighbour).first );
							// cell creation callback
							onCellCreated( &neighbour->first, &neighbour->second );

							tag = true;
							
							// within simulationbounds?
							math::Vec2f cellMin( (*neighbour).first.i * cellSize, (*neighbour).first.j * cellSize );
							math::Vec2f cellMax( ((*neighbour).first.i+1) * cellSize, ((*neighbour).first.j+1) * cellSize);
							if( (cellMin.x >= simulationDomain.minPoint.x)&&(cellMin.y >= simulationDomain.minPoint.y)&&
								(cellMax.x <= simulationDomain.maxPoint.x)&&(cellMax.y <= simulationDomain.maxPoint.y))
							{
								// check wether the cell is occupied by one of the static solids within the domain
								math::Vec2f cellCenter = (cellMin + cellMax)*0.5f;

								bool isOccupied = false;
								for( std::vector<StaticSolid *>::iterator soIt = staticSolids.begin(); soIt != staticSolids.end(); ++soIt )
								{
									// transform the position of the current cell center into the local coordinate system of the currentshape
									math::Matrix44f itm = (*soIt)->getInverseTransformationMatrix();

									math::Vec2f cellCenterInStaticSolidSpace = transform( cellCenter, itm );

									if( (*soIt)->occupies( cellCenterInStaticSolidSpace ) )
									{
										// the cell is a solid cell
										isOccupied = true;
										break;
									}
								}

								if( isOccupied )
									(*neighbour).second.type  = Cell::Solid;
								else
									// set the celltype to air
									(*neighbour).second.type  = Cell::Air;
							}else
								// else set it to solid
								(*neighbour).second.type  = Cell::Solid;

							// set the neighbour cell up
							(*neighbour).second.layer = i;
						}else
						{
							if( (*neighbour).second.layer == -1 )
							{
								if( (*neighbour).second.type != Cell::Solid )
									(*neighbour).second.type  = Cell::Air;

								// set the neighbour cell up
								(*neighbour).second.layer = i;
							}
						}

						// we have just created, or at least set up the neighbour of a cell
						// if we are currently in layer 1 we know that we have created a neighbour of a fluidcell (which is layer0)
						// every cell of layertype 1 should have 3 booleans which indicate for each velocity-component whether this
						// component neighbours a fluid cell or not
						// later these indicators will be very usefull for the update algorithm
						// we just have to look at the 3 cell neighbours in the positive axis directions (because velocities lie in the lower faces)

						// are we currently on the first air layer?
						if( i == 1 )
						{
							// on which neighbour index are we?
							switch( n )
							{
							case 1: // right neigbour (i+1)
								// we know the fluid cell lies on the left side (i-1)
								(*neighbour).second.xNeighboursFluid = true;
								break;
							case 3: // top neighbour (j+1)
								// we know the fluid cell lies on the bottom side (j-1)
								(*neighbour).second.yNeighboursFluid = true;
								break;
							};
						}

					}
				}
			}
		}while(tag);
	}

	// remove all unsused/unimportant cells from the hash_table ---------------------------
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin();
	while( it != gridCells.end() )
	{
		// current cell unused?
		if( (*it).second.layer == -1 )
		{
			// update (remove) the neighbour-references from neighbours
			updateNeighbours( (*it).first );

			// remove it
			it = gridCells.erase( it );

			// the iterator now points to the very next element beyound the one which was removed
			// thats why we dont need to increment
		}else
			// step further
			++it;
	};



	// if advection is based on the velocities then we have to transfere the velocities to the grid before
	// we start solving
	if( particleBasedAdvection )
	{
		// PIC/FLIP method (cell velocities of fluidcells are retreived from surrounding particles) -----------------
		// transfere particle velocites to cell velocities which border fluid cells
		transferParticleVelocitiesToGrid();
	}

	
	// compute distances to fluid --------------------------------
	computeDistanceToFluid();

	// extrapolate velocity into buffer cells -----------------
	extrapolateVelocities();


	// done - the grid is up and ready
}







//
// this method is used by updategrid when pic/flip us used and
// may be used to define the grid velocities in an intuitive manner
//
void FluidSimulator2d::transferParticleVelocitiesToGrid()
{
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.sum = 0.0f;
		(*it).second.velocity.x = 0.0f;
	}

	for( std::vector<Particle2d>::iterator pIt=markerParticles.begin(); pIt != markerParticles.end(); ++pIt )
	{
		float x = (*pIt).position.x/cellSize;
		float y = (*pIt).position.y/cellSize - 0.5f;

		int i = (int)floor( x );
		int j = (int)floor( y );

		// interpolation weights for each dimension
		float one_minus_u = i + 1.0f - x;
		float u = x - i;
		float one_minus_v = j + 1.0f - y;
		float v = y - j;

		// try to get the cell
		stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j ) );

		if( cellIterator != gridCells.end() )
		{
			float weight;

			weight = one_minus_u*one_minus_v;
			(*cellIterator).second.velocity.x += weight*(*pIt).velocity.x;
			(*cellIterator).second.sum += weight;

			// right neighbour
			if( (*cellIterator).second.neighbours[1] )
			{
				weight = u*one_minus_v;
				(*cellIterator).second.neighbours[1]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[1]->sum += weight;
			}
			// top neighbour
			if( (*cellIterator).second.neighbours[3] )
			{
				weight = one_minus_u*v;
				(*cellIterator).second.neighbours[3]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[3]->sum += weight;
			}
			// top right neighbour
			if( (*cellIterator).second.neighbours[4] )
			{
				weight = u*v;
				(*cellIterator).second.neighbours[4]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[4]->sum += weight;
			}
		}
	}

	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		if( (*it).second.sum != 0.0f )
			(*it).second.velocity.x /= (*it).second.sum;


	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.sum = 0.0f;
		(*it).second.velocity.y = 0.0f;
	}

	for( std::vector<Particle2d>::iterator pIt=markerParticles.begin(); pIt != markerParticles.end(); ++pIt )
	{
		float x = (*pIt).position.x/cellSize - 0.5f;
		float y = (*pIt).position.y/cellSize;

		int i = (int)floor( x );
		int j = (int)floor( y );

		// interpolation weights for each dimension
		float one_minus_u = i + 1.0f - x;
		float u = x - i;
		float one_minus_v = j + 1.0f - y;
		float v = y - j;

		// try to get the cell
		stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j ) );

		if( cellIterator != gridCells.end() )
		{
			float weight;

			weight = one_minus_u*one_minus_v;
			(*cellIterator).second.velocity.y += weight*(*pIt).velocity.y;
			(*cellIterator).second.sum += weight;

			// right neighbour
			if( (*cellIterator).second.neighbours[1] )
			{
				weight = u*one_minus_v;
				(*cellIterator).second.neighbours[1]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[1]->sum += weight;
			}
			// top neighbour
			if( (*cellIterator).second.neighbours[3] )
			{
				weight = one_minus_u*v;
				(*cellIterator).second.neighbours[3]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[3]->sum += weight;
			}
			// top right neighbour
			if( (*cellIterator).second.neighbours[4] )
			{
				weight = u*v;
				(*cellIterator).second.neighbours[4]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[4]->sum += weight;
			}
		}
	}

	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		if( (*it).second.sum != 0.0f )
			(*it).second.velocity.y /= (*it).second.sum;
}










//
// This method performs the actual solving of the navier-stokes equation
//
//
void FluidSimulator2d::updateVelocity( float dt )
{
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


	// compute the change in velocity using the oldVelocity member which we have stored at the beginning of this procedure ----------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.velocityChange = (*it).second.velocity - (*it).second.oldVelocity;


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
// This method will move the marker particles depending on the velocity in the grid through the grid
// Velocity is retreived from interpolation and a simple ODE is used for moving.
//
void FluidSimulator2d::moveParticles( float dt )
{
	float xmin=0.001f*cellSize, xmax=1.0f - 0.001f*cellSize;
	float ymin=0.001f*cellSize, ymax=1.0f - 0.001f*cellSize;

	// iterate over all particles - rk2
	for( std::vector<Particle2d>::iterator it=markerParticles.begin(); it != markerParticles.end(); ++it )
	{
		// perform one rk2 step

		// RK2 first stage (mid position)
		math::Vec2f velocity = getVelocity( (*it).position.x, (*it).position.y );

		float midPosx = (*it).position.x + velocity.x*0.5f*dt;
		float midPosy = (*it).position.y + velocity.y*0.5f*dt;

		// clamp
		if( midPosx < xmin )
			midPosx = xmin;
		if( midPosx > xmax )
			midPosx = xmax;
		if( midPosy < ymin )
			midPosy = ymin;
		if( midPosy > ymax )
			midPosy = ymax;


		// RK2 second stage (final position)
		velocity = getVelocity( midPosx, midPosy );
		(*it).position.x += velocity.x*dt;
		(*it).position.y += velocity.y*dt;

		// clamp
		if( (*it).position.x < xmin )
			(*it).position.x = xmin;
		if( (*it).position.x > xmax )
			(*it).position.x = xmax;
		if( (*it).position.y < ymin )
			(*it).position.y = ymin;
		if( (*it).position.y > ymax )
			(*it).position.y = ymax;
	}
}





//
// this method will set the velocities on solid cells which border
// non-solid cells so that free-slip, no-slip, or frictional boundary
// conditions are met
//
void FluidSimulator2d::setBoundaryConditions( void )
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
					// then set the velocity-component of the fluid cell to zero
					(*it).second.velocity.y = 0.0f;
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
				cell->velocity.x = cell->neighbours[2]->velocity.x;
			}else
			// y+
			if( cell->neighbours[3] && (cell->neighbours[3]->type != Cell::Solid) )
			{
				cell->velocity.x = cell->neighbours[3]->velocity.x;
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
// this will solve the pressure-term and will make the grid velocities divergence free
//
void FluidSimulator2d::solvePressure( float dt )
{
	// compute pressure ------------------------------------------

	// currently a gauss-seidel relaxation is used - conjugate gradient methods are proposed and promise to be better
	// (converge faster)

	// the vector tempVelocity of each cell is used to store some values for each line of the system
	// tempVelocity.x = b
	// tempVelocity.y = number of non-solid neighbouring cells (number of air/fluid cells) which enter the pressure-calculation
	// we prepare our linear solver by...
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		if( (*it).second.type == Cell::Fluid )
		{
			//...computing b...
			// ->compute the velocity-divergence using forward differencing
			(*it).second.tempVelocity.x = 0.0f;

			float u1,u2;         // these are the velocity-components of the current (u1) and the next cell (u2)

			// if the lower cell is a solid cell, then u1 becomes zero (since u1 lies on the border between a fluid and a solid cell)
			// x-component
			if( (*it).second.neighbours[0]->type == Cell::Solid )
				u1 = 0.0f;
			else
				u1 = (*it).second.velocity.x;
			if( (*it).second.neighbours[1]->type == Cell::Solid )
				u2 = 0.0f;
			else
				u2 = (*it).second.neighbours[1]->velocity.x;
			(*it).second.tempVelocity.x += u2 - u1;


			// y-component
			if( (*it).second.neighbours[2]->type == Cell::Solid )
				u1 = 0.0f;
			else
				u1 = (*it).second.velocity.y;
			if( (*it).second.neighbours[3]->type == Cell::Solid )
				u2 = 0.0f;
			else
				u2 = (*it).second.neighbours[3]->velocity.y;
			(*it).second.tempVelocity.x += u2 - u1;

			(*it).second.tempVelocity.x *= cellSize/dt;

			(*it).second.tempVelocity.y = 4.0f;                // by default we assume that all neighbour cells are non-solid

			// determining the number of non-solid neighbours which will enter the actual solving
			// and initiating the pressure of all nonfluid neighbours
			for( unsigned int n = 0; n<4; ++n )
				if( (*it).second.neighbours[n]->type == Cell::Solid )
				{
					(*it).second.tempVelocity.y -= 1.0f;
				}

			//...and initializing pressure p
			(*it).second.pressure = 0.0f;

		}else
			// air cell ?
			if( (*it).second.type == Cell::Air )
				(*it).second.pressure = atmosphericPressure;
			else
				// solid cell!
				(*it).second.pressure = 0.0f;

	// solve the system with the gauss-seidel-relaxation method
	for( unsigned int k=0; k<200; ++k )
		for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
			if( (*it).second.type == Cell::Fluid )
			{
				// do one iteration step
				(*it).second.pressure = (-(*it).second.tempVelocity.x+(*it).second.neighbours[0]->pressure+(*it).second.neighbours[1]->pressure
					                                                 +(*it).second.neighbours[2]->pressure+(*it).second.neighbours[3]->pressure )/(*it).second.tempVelocity.y;
			}



	// apply pressure -----------------------------------------

	// pressureapplication is done by substracting the pressure-gradient from velocities of each fluid cell
	// this will make the velocity field divergence free
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// the new computed velocity values will be stored in the tempVelocity vector
		(*it).second.tempVelocity = (*it).second.velocity;

		// if any of the lower neighbours (which we need for computation) is missing, then we can skip this cell
		// since we can be sure that it wont border a fluid cell (it will be a layer==2 cell)
		if( !(*it).second.neighbours[0] || !(*it).second.neighbours[2] )
			continue;

		// compute the pressure gradient

		// we will use backward differences here since the pressure gradient of a air cell which is on the right side
		// of a fluid cell could not be computed with forward differences (but the pressure gradient of a air cell which
		// is on the left side of a fluid cell can be computed with backward differences )
		math::Vec2f pressureGradient( (*it).second.pressure - (*it).second.neighbours[0]->pressure, (*it).second.pressure - (*it).second.neighbours[2]->pressure );

		pressureGradient.x *= dt / cellSize;
		pressureGradient.y *= dt / cellSize;



		// update only velocity-components which border fluid cells but not solid cells !
		if( (*it).second.type == Cell::Fluid )
		{
			if( (*it).second.neighbours[0]->type != Cell::Solid )
				(*it).second.tempVelocity.x = (*it).second.velocity.x - pressureGradient.x;


			if( (*it).second.neighbours[2]->type != Cell::Solid )
				(*it).second.tempVelocity.y = (*it).second.velocity.y - pressureGradient.y;

		}else
			if( (*it).second.type == Cell::Air )
			{
				if( (*it).second.neighbours[0]->type == Cell::Fluid )
					(*it).second.tempVelocity.x = (*it).second.velocity.x - pressureGradient.x;

				if( (*it).second.neighbours[2]->type == Cell::Fluid )
					(*it).second.tempVelocity.y = (*it).second.velocity.y - pressureGradient.y;
			}
	}

	// we have stored the new velocities in the tempVelocity vectors, now since all cells are updated we can copy
	// them to the real velocity vectors
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.velocity = (*it).second.tempVelocity;
}


//
// if simple MAC simulation is used, the velocities of the grid have to be advected...
//
void FluidSimulator2d::advectGridVelocities( float dt )
{
	// apply convection (ONLY WHEN NO PIC/FLIP) ---------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// we have to advance velocity-components which border fluid cells
		if( (*it).second.xNeighboursFluid )
		{
			math::Vec2f velXPos = traceParticle( math::Vec2f( (*it).first.i * cellSize, (*it).first.j * cellSize+(cellSize/2.0f) ), dt );
			(*it).second.tempVelocity.x = getVelocity( velXPos.x, velXPos.y ).x;
		}else
			(*it).second.tempVelocity.x = (*it).second.velocity.x;
		if( (*it).second.yNeighboursFluid )
		{
			math::Vec2f velYPos = traceParticle( math::Vec2f( (*it).first.i * cellSize+(cellSize/2.0f), (*it).first.j * cellSize ), dt );
			(*it).second.tempVelocity.y = getVelocity( velYPos.x, velYPos.y ).y;
		}else
			(*it).second.tempVelocity.y= (*it).second.velocity.y;
	}


	// we have stored the new velocities in the tempVelocity vectors, now since all cells are updated we can copy
	// them to the real velocity vectors
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		// we have to take into account these velocitie-components which border fluid cells
		(*it).second.velocity = (*it).second.tempVelocity;
}


//
// this method will solve the viscosity term...
//
void FluidSimulator2d::solveViscosity( float dt )
{
	// implicit integration....
	if( viscosity == 0.0f )
		return;

	float a = dt*viscosity;


	// prepare solving...
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.tempVelocity.x = (*it).second.velocity.x; // component of b for the current cell
		(*it).second.tempVelocity.y = (*it).second.velocity.y; //

		//...and initializing velocity.x and y
		(*it).second.velocity.x = 0.0f;
		(*it).second.velocity.y = 0.0f;
	}


	// solve the system with the gauss-seidel-relaxation method
	for( unsigned int k=0; k<200; ++k )
		for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
			if( (*it).second.xNeighboursFluid || (*it).second.yNeighboursFluid )
			{
				float tx = (*it).second.tempVelocity.x;
				float xNeighbourCount = 0.0f;      // how many neighbours go into the equation for the x-component
				float ty = (*it).second.tempVelocity.y;
				float yNeighbourCount = 0.0f;      // how many neighbours go into the equation for the y-component

				for( unsigned int n = 0; n<4; ++n )
				{
					if( (*it).second.neighbours[n]->xNeighboursFluid )
					{
						tx += a*(*it).second.neighbours[n]->velocity.x;
						xNeighbourCount += 1.0f;
					}
					if( (*it).second.neighbours[n]->yNeighboursFluid )
					{
						ty += a*(*it).second.neighbours[n]->velocity.y;
						yNeighbourCount += 1.0f;
					}
				}

				if( (*it).second.xNeighboursFluid )
					(*it).second.velocity.x = tx / (1+a*xNeighbourCount);
				if( (*it).second.yNeighboursFluid  )
					(*it).second.velocity.y = ty / (1+a*yNeighbourCount);
			}
}


//
// will add dt*math::Vec3f(x,y) to the respective velocity components, which neighbour fluid cells
//
void FluidSimulator2d::applyGravity( float dt, float x, float y )
{
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// we have to advance velocity-components of all cells which border fluid cells
		if( (*it).second.xNeighboursFluid )
			(*it).second.velocity.x += x*dt;
		if( (*it).second.yNeighboursFluid )
			(*it).second.velocity.y += y*dt;
	}
}

//
// This is the actual distance computation proposed in [Zhao2005]
//
void FluidSimulator2d::sweepUpdateDistances( Cell *cell, int horizontalNeighbourIndex, int verticalNeighbourIndex )
{
	float aDistance = 100.0f;
	float bDistance = 100.0f;
	float h = cellSize;


	// try to access horizontal neighbour
	if( cell->neighbours[horizontalNeighbourIndex] )
		// and take its distance
		aDistance = cell->neighbours[horizontalNeighbourIndex]->signedDistance;


	// try to access vertical neighbour
	if( cell->neighbours[verticalNeighbourIndex] )
		// and take its distance
		bDistance = cell->neighbours[verticalNeighbourIndex]->signedDistance;

#define min(a,b) ((a < b) ? a : b)
#define max(a,b) ((a > b) ? a : b)

	float d;
	
	d = min( aDistance, bDistance ) + h;
	
	//if( d > fmax(p,q) )
	//	d=(p+q+sqrt(2-sqr(p-q)))/2;
	if( d > max(aDistance, bDistance) )
		d=(aDistance+bDistance+sqrt(2*h*h-(aDistance-bDistance)*(aDistance-bDistance)))/2.f;


	if(d<cell->signedDistance)
		cell->signedDistance = d;
}




//
// Computes the signed distance to the fluid surface for each cell in the grid using fast sweeping [Zhao 2005].
// We need the signed distances for each cell within the grid for velocity extrapolation.
//
void FluidSimulator2d::computeDistanceToFluid( void )
{
	// the fast sweeping algorithm [Zhao2005] is used to compute the signed
	// distance function \sigma for each velocity component which doesnt border a fluid cell

	// preparation of the fast sweeping algorithm -------------------------------------------
	float h = cellSize;

	int       Istart;    // index of the first gridpoint/cell in x direction (needed since we have a dynamic grid)
	int       Jstart;    // index of the first gridpoint/cell in y direction (needed since we have a dynamic grid)
	int            I;    // index of the last gridpoint/cell in x direction (needed since we have a dynamic grid)
	int            J;    // index of the last gridpoint/cell in y direction (needed since we have a dynamic grid)

	// intiate all values from the first cell in town
	Istart = I = (*gridCells.begin()).first.i;
	Jstart = J = (*gridCells.begin()).first.j;

	// evaluate the parameters Istart, Jstart, I and J
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		if( (*it).first.i < Istart )
			Istart = (*it).first.i;
		if( (*it).first.j < Jstart )
			Jstart = (*it).first.j;
		if( (*it).first.i > I )
			I = (*it).first.i;
		if( (*it).first.j > J )
			J = (*it).first.j;
	}

	// fast sweeping algorithm: initialization -------------------------------------------

	// set the distance function for all velocity components which border fluidcells to 0 and to a very large positive
	// value otherwise
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(it->second);
		/*
		if( (*it).second.type == Cell::Fluid )
		{
			(*it).second.signedDistance = -0.5f*cellSize;
		}else
			(*it).second.signedDistance = 999999999.0f;
		*/

		cell->signedDistance = 999999999.0f;

		// we look for fluid cells which lie on the fluid surface
		if( cell->xNeighboursFluid || cell->yNeighboursFluid )
		{
			// when the cell is a fluid cell...
			if( cell->isFluidCell() )
			{
				// ...check if cell lies on the fluid surface by checking all neighbours
				// if one neighbour is not a fluid cell then the cell lies on the surface of the fluid
				if( !cell->neighbours[0]->isFluidCell() || !cell->neighbours[1]->isFluidCell() || !cell->neighbours[2]->isFluidCell() || !cell->neighbours[3]->isFluidCell() )
					// cell is a surface cell and a fluid cell
					cell->signedDistance = 0.5f*cellSize;
				else
					cell->signedDistance = 999999999.0f;
			}else
			{
				cell->signedDistance = 0.5f*cellSize;
			}
		}else
			cell->signedDistance = 999999999.0f;

	}



	// fast sweeping algorithm: iterations -----------------------------------------------
	for(int k=0; k<2; ++k)
	{
		// now we do 4 sweeps (one for each quadrant) and update the signed distance values
		//

		// Quadrant I   : i 0->1 | j 0->1
		for( int j=Jstart; j<=J; ++j  )
			for( int i=Istart; i<=I; ++i  )		
			{
				// first try to receive cell
				stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j ) );

				// cell not found?
				if( it == gridCells.end() )
					// skip current
					continue;

				// compute the distance-value and update the cell if it is smaller than the current one
				//if( (*it).second.type != Cell::Fluid )
					sweepUpdateDistances( &((*it).second), 0, 2 );
			}


		// Quadrant II  : i 1->0 | j 0->1
		for( int j=Jstart; j<=J; ++j  )
			for( int i=I; i>=Istart; --i  )
			{
				// first try to receive cell
				stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j ) );

				// cell not found?
				if( it == gridCells.end() )
					// skip current
					continue;

				// compute the distance-value and update the cell if it is smaller than the current one
				//if( (*it).second.type != Cell::Fluid )
					sweepUpdateDistances( &((*it).second), 1, 2 );
			}

		// Quadrant III : i 1->0 | j 1->0
		for( int j=J; j>=Jstart; --j  )
			for( int i=I; i>=Istart; --i  )
			{
				// first try to receive cell
				stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j ) );

				// cell not found?
				if( it == gridCells.end() )
					// skip current
					continue;

				// compute the distance-value and update the cell if it is smaller than the current one
				//if( (*it).second.type != Cell::Fluid )
					sweepUpdateDistances( &((*it).second), 1, 3 );
			}

		// Quadrant IV  : i 0->1 | j 1->0
		for( int j=J; j>=Jstart; --j  )
			for( int i=Istart; i<=I; ++i  )		
			{
				// first try to receive cell
				stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j ) );

				// cell not found?
				if( it == gridCells.end() )
					// skip current
					continue;

				// compute the distance-value and update the cell if it is smaller than the current one
				//if( (*it).second.type != Cell::Fluid )
					sweepUpdateDistances( &((*it).second), 0, 3 );
			}
	}
}




//
// General implementation of the sweeping for velocity extrapolation for the v-component of the velocity
//
void FluidSimulator2d::sweepVelocityUV( int iStart, int iEnd, int jStart, int jEnd )
{
	// compute increments which are needed to run from the start to end values
	// so the increments are either 1 (running from min to max) or -1 (running from max to min)
	int di = (iStart<iEnd) ? 1 : -1;
	int dj = (jStart<jEnd) ? 1 : -1;

	int horizontalNeighbourIndex = (di > 0) ? 0 : 1;
	int verticalNeighbourIndex = (dj > 0) ? 2 : 3;

	int oppositeHorizontalNeighbourIndex = (di > 0) ? 1 : 0;
	int oppositeVerticalNeighbourIndex = (dj > 0) ? 3 : 2;

	// for each vertical cell
	for( int j=jStart; j!=jEnd; j+=dj )
		// for each horizontal cell
		for( int i=iStart; i!=iEnd; i+=di )
		{
			// try to retrieve the current indexed cell
			stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j ) );

			// cell not found?
			if( it == gridCells.end() )
				// skip current loopcycle
				continue;

			// get the reference to the current cell
			Cell *cell = &((*it).second);


			if( cell->type == Cell::Fluid )
				continue;


			Cell *temp = 0;


			// we can skip the computation if the horizontal and vertical neighbours dont exist since this would
			// bring in large distance value which would prevent the velocity component from being updated
			float p_c, p_b, p_px, p_px_b, p_opx_b, p_opx;
			float      p_l, p_py, p_opy, p_py_l, p_opy_l;
			float v1, v2;
			float u1, u2;
			
			p_c = p_b = p_px = p_opx = p_px_b = p_opx_b = p_l = p_py = p_opy = p_py_l = p_opy_l = 100.0f;
			u1 = u2 = cell->velocity.x;
			v1 = v2 = cell->velocity.y;

			p_c = cell->signedDistance;                       // distance of current cell

			if( cell->neighbours[2] )                          // distance of bottom neighbour cell (j-1)
			{
				p_b = cell->neighbours[2]->signedDistance;
			}

			if( cell->neighbours[0] )                          // distance of left neighbour cell (i-1)
			{
				p_l = cell->neighbours[0]->signedDistance;
			}

			if( cell->neighbours[horizontalNeighbourIndex] )   // distance of previous horizontal neighbour cell (i-di)
			{
				p_px = cell->neighbours[horizontalNeighbourIndex]->signedDistance;
				v1 = cell->neighbours[horizontalNeighbourIndex]->velocity.y;
				u1 = (*it).second.neighbours[horizontalNeighbourIndex]->velocity.x;
			}

			if( cell->neighbours[oppositeHorizontalNeighbourIndex] )   // distance of opposite previous horizontal neighbour cell (i+di)
			{
				p_opx = cell->neighbours[oppositeHorizontalNeighbourIndex]->signedDistance;
			}


			if( cell->neighbours[verticalNeighbourIndex] )     // distance of previous vertical neighbour cell (j-dj)
			{
				v2 = cell->neighbours[verticalNeighbourIndex]->velocity.y;
				p_py = cell->neighbours[verticalNeighbourIndex]->signedDistance;
				u2 = cell->neighbours[verticalNeighbourIndex]->velocity.x;
			}


			if( (*it).second.neighbours[oppositeVerticalNeighbourIndex] )     // distance of opposite previous vertical neighbour cell (j+dj)
			{
				p_opy = cell->neighbours[oppositeVerticalNeighbourIndex]->signedDistance;
			}


			temp = cell->getNeighbour( -di, -1 );
			if( temp )                              // distance of the left cell in the previous cell row (i-di, j-1)
				p_px_b = temp->signedDistance;

			temp = cell->getNeighbour( di, -1 );
			if( temp )                              // distance of the opposite left cell in the previous cell row (i+di, j-1)
				p_opx_b = temp->signedDistance;

			temp = cell->getNeighbour( -1, -dj );
			if( temp )                              // distance of the left cell in the previous cell row (i-1, j-dj)
				p_py_l = temp->signedDistance;

			temp = cell->getNeighbour( -1, dj );
			if( temp )                              // distance of the left cell in the opposite previous cell row (i-1, j+dj)
				p_opy_l = temp->signedDistance;

			// compute U---------------
			if( !cell->xNeighboursFluid  )
			{
				// compute the horizontal change in distance (using backward differences)
				float deltaPhiX = di * (p_c - p_l);

				// skip current cycle if the horizontal distance to the fluid gets smaller
				if( deltaPhiX >= 0 )
				{
					// compute the vertical change of distance
					// since we want the vertical change of distance at the position of the u-component in the cell we have to
					// compute the vertical change of distance for the current cell and for the cell i-1 and then have to
					// compute the average of both to get the vertical change of distance between the cell i and i-1
					float deltaPhiY = 0.5f*( p_opy_l - p_py_l + p_opy - p_py );

					// skip current cycle if the vertical distance to the fluid gets smaller
					if( deltaPhiY >= 0 )
					{
						float alpha;

						if( deltaPhiX + deltaPhiY == 0.0f )
							alpha = 0.5f;
						else
							alpha = deltaPhiX/(deltaPhiX + deltaPhiY);
						
						(*it).second.velocity.x = alpha*u1 + (1.0f-alpha)*u2;
					}
				}
			}
			// compute V---------------
			if( !cell->yNeighboursFluid )
			{
				// compute the vertical change in distance (using backward differences)
				float deltaPhiY = dj * (p_c - p_b);

				// skip current cycle if the vertical distance to the fluid gets smaller
				if( deltaPhiY >= 0 )
				{
					// compute the horizontal change of distance
					// since we want the horizontal change of distance at the position of the v-component in the cell we have to
					// compute the horizontal change of distance for the current cell and for the cell j-1 and then have to
					// compute the average of both to get the vertical change of distance between the cell j and j-1
					float deltaPhiX = 0.5f*( (p_opx_b - p_px_b)/2.f + (p_opx - p_px)/2.f );

					// skip current cycle if the vertical distance to the fluid gets smaller
					if( deltaPhiX >= 0 )
					{
						float alpha;

						if( deltaPhiX + deltaPhiY == 0.0f )
							alpha = 0.5f;
						else
							alpha = deltaPhiX/(deltaPhiX + deltaPhiY);
						
						(*it).second.velocity.y = alpha*v1 + (1.0f-alpha)*v2;
					}
				}
			}
		}
}

//
// this method will compute all the velocity components of grid-cell-components which dont border fluid cells
//
void FluidSimulator2d::extrapolateVelocities( void )
{
	// quit if there are no cells
	if( !gridCells.size() )
		return;

	float h = cellSize;

	int       iMin;    // index of the first gridpoint/cell in x direction (needed since we have a dynamic grid)
	int       jMin;    // index of the first gridpoint/cell in y direction (needed since we have a dynamic grid)
	int       iMax;    // index of the last gridpoint/cell in x direction (needed since we have a dynamic grid)
	int       jMax;    // index of the last gridpoint/cell in y direction (needed since we have a dynamic grid)

	// intiate all values from the first cell in town
	iMin = iMax = (*gridCells.begin()).first.i;
	jMin = jMax = (*gridCells.begin()).first.j;

	// evaluate the parameters Istart, Jstart, I and J
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		if( (*it).first.i < iMin )
			iMin = (*it).first.i;
		if( (*it).first.j < jMin )
			jMin = (*it).first.j;
		if( (*it).first.i > iMax )
			iMax = (*it).first.i;
		if( (*it).first.j > jMax )
			jMax = (*it).first.j;
	}



	// fast sweeping algorithm: iterations for solving pde \nabla u \dot \nabla \sigma = 0--------------

	for(int k=0; k<4; ++k)
	{
		// now we do 4 sweeps (one for each quadrant) for the u-component of the velocity
		sweepVelocityUV( iMin, iMax+1, jMax, jMin-1 );  // Quadrant IV
		sweepVelocityUV( iMax, iMin-1, jMin, jMax+1 );  // Quadrant II
		sweepVelocityUV( iMax, iMin-1, jMax, jMin-1 );  // Quadrant III
		sweepVelocityUV( iMin, iMax+1, jMin, jMax+1 );  // Quadrant I
	}
}






//
// returns the index-specified trilinear interpolated velocity-component
// from the grid.
//
float FluidSimulator2d::getInterpolatedVelocityValue( float x, float y, unsigned int componentIndex  )
{
	// coordinate of the lower left cell
	int i = (int)floor( x );
	int j = (int)floor( y );


	// interpolation weights for each dimension
	float u = i + 1.0f - x;
	float one_minus_u = x - i;
	float v = j + 1.0f - y;
	float one_minus_v = y - j;

	// retreive the indexed cell
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j ) );
	Cell *cell;

	// if the cell could not be found
	if( cellIterator == gridCells.end() )
		// quit the function
		return 0.0f;
	else
		// set the reference to the cell
		cell = &((*cellIterator).second);

	// does every neighbour of the current cell exist?
	if( cell->allInterpolationNeighboursExist )
	{
		// then calculate the full trilinear interpolation
		return u*v*cell->velocity[componentIndex] + one_minus_u*v*cell->neighbours[1]->velocity[componentIndex] +
			u*one_minus_v*cell->neighbours[3]->velocity[componentIndex] + one_minus_u*one_minus_v*cell->neighbours[4]->velocity[componentIndex];
	}else
	{
		// we have to check each neighbour cell individiually and leave it out in our calculation
		float result = u*v*cell->velocity[componentIndex];  // this float will hold the sum of all values which entered the calculation
		float resultWeighting = u*v;                        // this variable will hold the sum of all weights which entered the calculation

		if( cell->neighbours[1] )
		{
			result += one_minus_u*v*cell->neighbours[1]->velocity[componentIndex];
			resultWeighting += one_minus_u*v;
		}
		if( cell->neighbours[3] )
		{
			result += u*one_minus_v*cell->neighbours[3]->velocity[componentIndex];
			resultWeighting += u*one_minus_v;
		}
		if( cell->neighbours[4] )
		{
			result += one_minus_u*one_minus_v*cell->neighbours[4]->velocity[componentIndex];
			resultWeighting += one_minus_u*one_minus_v;
		}

		// now adapt the result so that the overall weighting remains 1.0f even if we skipped some terms
		return result / resultWeighting;
	}

	return 0.0f;
}

//
// returns the index-specified trilinear interpolated velocity-component
// from the grid.
//
float FluidSimulator2d::getInterpolatedChangeValue( float x, float y, unsigned int componentIndex  )
{
	// coordinate of the lower left cell
	int i = (int)floor( x );
	int j = (int)floor( y );


	// interpolation weights for each dimension
	float u = i + 1.0f - x;
	float one_minus_u = x - i;
	float v = j + 1.0f - y;
	float one_minus_v = y - j;

	// retreive the indexed cell
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j ) );
	Cell *cell;

	// if the cell could not be found
	if( cellIterator == gridCells.end() )
		// quit the function
		return 0.0f;
	else
		// set the reference to the cell
		cell = &((*cellIterator).second);

	// does every neighbour of the current cell exist?
	if( cell->allInterpolationNeighboursExist )
	{
		// then calculate the full trilinear interpolation
		return u*v*cell->velocityChange[componentIndex] + one_minus_u*v*cell->neighbours[1]->velocityChange[componentIndex] +
			u*one_minus_v*cell->neighbours[3]->velocityChange[componentIndex] + one_minus_u*one_minus_v*cell->neighbours[4]->velocityChange[componentIndex];
	}else
	{
		// we have to check each neighbour cell individiually and leave it out in our calculation
		float result = u*v*cell->velocityChange[componentIndex];  // this float will hold the sum of all values which entered the calculation
		float resultWeighting = u*v;                        // this variable will hold the sum of all weights which entered the calculation

		if( cell->neighbours[1] )
		{
			result += one_minus_u*v*cell->neighbours[1]->velocityChange[componentIndex];
			resultWeighting += one_minus_u*v;
		}
		if( cell->neighbours[3] )
		{
			result += u*one_minus_v*cell->neighbours[3]->velocityChange[componentIndex];
			resultWeighting += u*one_minus_v;
		}
		if( cell->neighbours[4] )
		{
			result += one_minus_u*one_minus_v*cell->neighbours[4]->velocityChange[componentIndex];
			resultWeighting += one_minus_u*one_minus_v;
		}

		// now adapt the result so that the overall weighting remains 1.0f even if we skipped some terms
		return result / resultWeighting;
	}

	return 0.0f;
}

//
// This function does look up the velocity within the simulationDomain
//
math::Vec2f FluidSimulator2d::getVelocity( float x, float y )
{
	math::Vec2f velocity;

	velocity.x = getInterpolatedVelocityValue( x/cellSize, y/cellSize-0.5f, 0 );
	velocity.y = getInterpolatedVelocityValue( x/cellSize-0.5f, y/cellSize, 1 );

	return velocity;
}
//
// This function does look up the velocityChange within the simulationDomain (used for FLIP)
//
math::Vec2f FluidSimulator2d::getVelocityChange( float x, float y )
{
	math::Vec2f velocityChange;

	velocityChange.x = getInterpolatedChangeValue( x/cellSize, y/cellSize-0.5f, 0 );
	velocityChange.y = getInterpolatedChangeValue( x/cellSize-0.5f, y/cellSize, 1 );

	return velocityChange;
}

//
// This method returns the position of the given point at t - deltaTime
// (used for semi-lagrange style advection)
//
math::Vec2f FluidSimulator2d::traceParticle( const math::Vec2f &position, float deltaTime )
{
	// solve ODE with RK2
	math::Vec2f vel = getVelocity( position.x, position.y );
	vel = getVelocity( position.x-0.5f*deltaTime*vel.x, position.y-0.5f*deltaTime*vel.y );

	return math::Vec2f( position.x - deltaTime*vel.x, position.y - deltaTime*vel.y );
}

//
// This is a little helper function which will look for all
// cells which might adress the given cell at the given coordinates.
// Then the interpolation-neighbour reference is stored in each depending
// cell and the flag of each is recomputed.
// In addition all interpolationneighbours of cell are checked whether they
// exist and are stored as references.
//
void FluidSimulator2d::updateNeighbours( Cell *cell, const Cell::Coordinate &c )
{
	// by adding a new cell to the grid, we have to look for all cells which
	// would reference us as a neighbour and add a reference to those which could
	// be found (did exist)
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator neighbourIterator;

	for( unsigned int neighbourIndex=0; neighbourIndex<8; ++neighbourIndex )
	{
		// try to find the neighbour
		neighbourIterator = gridCells.find( c.getNeighbourCoordinate(neighbourIndex) );

		// neighbour found?
		if( neighbourIterator != gridCells.end() )
			// store the reference to the neighbour in the new cell
			cell->setNeighbour( neighbourIndex, &((*neighbourIterator).second) );

		// try to find the cell which would reference the new cell under the given neighbourIndex
		neighbourIterator = gridCells.find( c.getInverseNeighbourCoordinate(neighbourIndex) );

		// "inverse" neighbour found?
		if( neighbourIterator != gridCells.end() )
			// store the reference to the new cell in that referencing cell
			(*neighbourIterator).second.setNeighbour( neighbourIndex, cell );
	}
}

//
// This method does the same as the other updateNeighbours method,
// but it is used when a cell was removed and all depending cells which reference
// it as neighbour have to be updated (their reference to the removed cell have to be cleared).
//
void FluidSimulator2d::updateNeighbours( const Cell::Coordinate &c )
{
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator;

	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator neighbourIterator;

	for( unsigned int neighbourIndex=0; neighbourIndex<8; ++neighbourIndex )
	{
		// try to find the cell which would reference the new cell under the given neighbourIndex
		neighbourIterator = gridCells.find( c.getInverseNeighbourCoordinate(neighbourIndex) );

		// "inverse" neighbour found?
		if( neighbourIterator != gridCells.end() )
			// remove the reference to the removed-cell
			(*neighbourIterator).second.removeNeighbour( neighbourIndex );
	}
}

//
// adds a non moving obstacle to the fluid simulation domain
//

void FluidSimulator2d::addStaticSolid( StaticSolid *staticSolid )
{
	staticSolids.push_back( staticSolid );
}



//
// returns the number of static solids
//
size_t FluidSimulator2d::getStaticSolidCount()
{
	return staticSolids.size();
}

//
// returns the index specified solid
//
FluidSimulator2d::StaticSolid *FluidSimulator2d::getStaticSolid( size_t index )
{
	return staticSolids[index];
}


//
// This method will be called when a new cell just has been created.
// Override it to get a hand on newly created cells.
//
void FluidSimulator2d::onCellCreated( const Cell::Coordinate *coord, Cell *newCell )
{
	// compute the cell center - we will need it for collision detection etc.
	newCell->center =  math::Vec2f( coord->i*cellSize + cellSize*0.5f, coord->j*cellSize + cellSize*0.5f );
}


// -------------------- FluidSimulator2d::Cell ------------------------------------------



//
// constructor
//
FluidSimulator2d::Cell::Cell()
{
	// setup neighbourreferences
	for( unsigned int i=0; i<10; ++i )neighbours[i] = 0;
	allInterpolationNeighboursExist = false;

	// no fluid borders the cell by default
	xNeighboursFluid = yNeighboursFluid = false;

	// intialize other members
	layer = -1;
	velocity = math::Vec2f( 0.0f, 0.0f );

	strainTensor = math::Matrix22f::Zero();
	rigid        = false;
	vonMisesEquivalentStress = 0.0f;
}

//
// Registers reference to the index-specified neighbour
//
void FluidSimulator2d::Cell::setNeighbour( unsigned int index, Cell *neighbour )
{
	neighbours[ index ] = neighbour;

	allInterpolationNeighboursExist = neighbours[1]&&neighbours[3]&&neighbours[4];
}

//
// Unregisters reference to the index-specified neighbour
//
void FluidSimulator2d::Cell::removeNeighbour( unsigned int index )
{
	neighbours[index] = 0;

	allInterpolationNeighboursExist = neighbours[1]&&neighbours[3]&&neighbours[4];
}

//
// retrieve the neighbour from given relative coordinates.
// ri, rj each are -1, 0 or +1
//
FluidSimulator2d::Cell *FluidSimulator2d::Cell::getNeighbour( int ri, int rj )
{
	if( ri > 0 )
	{
		if( rj > 0 )
		{
			// ri>0 && rj>0
			return neighbours[4];
		}else
			if( rj < 0 )
			{
				// ri>0 && rj<0
				return neighbours[5];
			}else
			{
				// ri>0 && rj==0
				return neighbours[1];
			}
	}else
		if( ri < 0 )
		{
			if( rj > 0 )
			{
				// ri<0 && rj>0
				return neighbours[7];
			}else
				if( rj < 0 )
				{
					// ri<0 && rj<0
					return neighbours[6];
				}else
				{
					// ri<0 && rj==0
					return neighbours[0];
				}
		}else
		{
			if( rj > 0 )
			{
				// ri==0 && rj>0
				return neighbours[3];
			}else
				if( rj < 0 )
				{
					// ri==0 && rj<0
					return neighbours[2];
				}else
				{
					// ri==0 && rj==0
					return this;
				}
		}
}

//
// returns true if the cell is of type fluid
//
bool FluidSimulator2d::Cell::isFluidCell()
{
	return (type == Fluid);
}



// -------------------- FluidSimulator2d::hash_compare ------------------------------------------

//
// hash function which returns an hashvalue for the given key
//
size_t FluidSimulator2d::hash_compare::operator() (const FluidSimulator2d::Cell::Coordinate& key) const
{
	// hash function proposed in [Worley 1996]
	return 541*key.i + 79*key.j;
}

//
// comperator for internal sorting (dont ask me why hash_table is sorting at all)
//
bool FluidSimulator2d::hash_compare::operator() (const FluidSimulator2d::Cell::Coordinate& key1, const FluidSimulator2d::Cell::Coordinate& key2) const
{
	// lexicographical ordering
	if( key1.i==key2.i )
	{
		return key1.j<key2.j;
	}else
		return key1.i<key2.i;
}



// -------------------- FluidSimulator2d::Cell::Coordinate ------------------------------------------


//
// standard constructor
//
FluidSimulator2d::Cell::Coordinate::Coordinate()
{
}

//
// constructs the cell from given coordinates
//
FluidSimulator2d::Cell::Coordinate::Coordinate( int _i, int _j ) : i(_i), j(_j)
{
}


//
// returns the Coordinate within the grid of the index-specified neighbour
//
FluidSimulator2d::Cell::Coordinate FluidSimulator2d::Cell::Coordinate::getNeighbourCoordinate( int neighbourIndex ) const
{
	switch( neighbourIndex )
	{
	case 0:
		return Coordinate( i-1, j );
	case 1:
		return Coordinate( i+1, j );
	case 2:
		return Coordinate( i, j-1 );
	case 3:
		return Coordinate( i, j+1 );
	case 4:
		return Coordinate( i+1, j+1 );
	case 5:
		return Coordinate( i+1, j-1 );
	case 6:
		return Coordinate( i-1, j-1 );
	case 7:
		return Coordinate( i-1, j+1 );
	default:
		return Coordinate( i, j );
	};
}

//
// this method returns the coordinate of the cell which references this (current)cell-coordinate through the given neighbour-index
//
FluidSimulator2d::Cell::Coordinate FluidSimulator2d::Cell::Coordinate::getInverseNeighbourCoordinate( int neighbourIndex ) const
{
	// other cells will reference the cell at this coordinate. So for convinient updating of the gridcell inter-references
	// this method is used to find all coordinates of cells which could reference the current cell(coordinate)
	// So: Coordinate c; c.getNeighbourCoordinate( int ).getInverseNeighbourCoordinate( int ) returns a coordinate which is identical to c
	switch( neighbourIndex )
	{
	case 0:
		return Coordinate( i+1, j );
	case 1:
		return Coordinate( i-1, j );
	case 2:
		return Coordinate( i, j+1 );
	case 3:
		return Coordinate( i, j-1 );
	case 4:
		return Coordinate( i-1, j-1 );
	case 5:
		return Coordinate( i-1, j+1 );
	case 6:
		return Coordinate( i+1, j+1 );
	case 7:
		return Coordinate( i+1, j-1 );
	default:
		return Coordinate( i, j );
	};
}




// -------------------- FluidSimulator2d::StaticSolid ------------------------------------------


void FluidSimulator2d::StaticSolid::computeTransformationMatrix()
{
	float z = 0.0f;
	transform = math::Matrix44f::Identity();



	transform.translate( translation.x, translation.y, z );
	transform.rotateZ( zRotation );

	inverseTransform = transform;
	inverseTransform.invert();
}

math::Matrix44f FluidSimulator2d::StaticSolid::getTransformationMatrix()
{
	return transform;
}
math::Matrix44f FluidSimulator2d::StaticSolid::getInverseTransformationMatrix()
{
	return inverseTransform;

}
void FluidSimulator2d::StaticSolid::setTranslation( float x, float y )
{
	translation = math::Vec2f( x, y );
	computeTransformationMatrix();
}
void FluidSimulator2d::StaticSolid::setRotation( float zAngle )
{
	zRotation = math::degToRad(zAngle);
	computeTransformationMatrix();
}
