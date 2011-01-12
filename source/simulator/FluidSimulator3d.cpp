/*--------------------------------------------------------------------

MAC bases fluid simulator which uses a staggered grid and particles
or tracking the fluid. In addition to basic MAC simulation the PIC
and FLIP methods can be used to achieve better animations.

----------------------------------------------------------------------*/
#include "FluidSimulator3d.h"




// -------------------- FluidSimulator3d ------------------------------------------

//
// constructor
//
FluidSimulator3d::FluidSimulator3d()
{
	//setCellSize( 0.025f );
	setCellSize( 0.01f );
	//setCellSize( 0.1f );

	setSimulationDomain( math::Vec3f( 0.0f, 0.0f, 0.0f ), math::Vec3f( 1.0f, 1.0f, 1.0f ) );

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
FluidSimulator3d::~FluidSimulator3d()
{
}

//
// advances a specified timestep with a dynamic amount of substeps depending
// on the CFL-condition and dt min/max values.
// (a conviencen function)
//
void FluidSimulator3d::advanceFrame( float timePerFrame )
{
	// we wont do anything if there are no particles
	if( markerParticles.size() == 0 )
		return;

	printf( "advanceFrame( %f )\n", timePerFrame );

	float t=0;         // the time which already had been computed in this step
	float  deltaTime = 0.01f;  // deltaTime for the next substep
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
void FluidSimulator3d::advanceStep( float deltaTime )
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
void FluidSimulator3d::reset( void )
{
	// remove all marker particles
	markerParticles.clear();
	// remove all gridcells
	gridCells.clear();
}

//
// setup the size of one grid cell in each dimension
//
void FluidSimulator3d::setCellSize( float _cellSize )
{
	if( _cellSize > 0.0f )
		cellSize = _cellSize;
}
//
// returns the cellsize
//
float FluidSimulator3d::getCellSize()
{
	return cellSize;
}

//
// returns a pointer to the cell which lies at the specified coordinate
// if the cell does not exist, null is returned
//
FluidSimulator3d::Cell *FluidSimulator3d::getCell( size_t i, size_t j, size_t k )
{
	// find the cell in the hashmap
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate(i,j,k) );

	if( it != gridCells.end() )
		// entry found
		return &(*it).second;

	return 0;
}

//
// set the range in which the delta time must be for one simulation step
//
void FluidSimulator3d::setDeltaTimeRange( float _dtMin, float _dtMax )
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
void FluidSimulator3d::setSimulationDomain( const math::Vec3f &min, const math::Vec3f &max )
{
	simulationDomain = math::BoundingBox( min, max );	
}



//
// sets the simple-fluid visosity
//
void FluidSimulator3d::setViscosity( float _viscosity )
{
	viscosity = _viscosity;
}

//
// adds a non moving obstacle to the fluid simulation domain
//
void FluidSimulator3d::addStaticSolid( FluidSimulator3d::StaticSolid *staticSolid )
{
	staticSolids.push_back( staticSolid );
}


//
// returns the number of static solids
//
size_t FluidSimulator3d::getStaticSolidCount()
{
	return staticSolids.size();
}


//
// returns the index specified solid
//
FluidSimulator3d::StaticSolid *FluidSimulator3d::getStaticSolid( size_t index )
{
	return staticSolids[index];
}

//
// This method evaluates the current maximum velocity within the grid and computes
// a timeDelta so that the condition velocity*timeDelta < cellSize is true for every
// velocity in the grid. This condition is called the CFL condition.
//
float FluidSimulator3d::CFLDeltaTime( void )
{
	// compute deltaTime so that the CFL-condition is satisfied (maxVelocity*deltaTime < cellSize)

	// first, we look for the maximum Velocity in each component
	float maxVelx = 0.0f;
	float maxVely = 0.0f;
	float maxVelz = 0.0f;

	// iterate over all gridcells...
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// ...and look if there are velocity components which are larger then our current maximums
		if( fabs((*it).second.velocity.x) > maxVelx )
			maxVelx = fabs((*it).second.velocity.x);
		if( fabs((*it).second.velocity.y) > maxVely )
			maxVely = fabs((*it).second.velocity.y);
		if( fabs((*it).second.velocity.z) > maxVelz )
			maxVelz = fabs((*it).second.velocity.z);
	}

	// compute the squared length of our maximum-velocity-vector
	float length2 = maxVelx*maxVelx + maxVely*maxVely + maxVelz*maxVelz;

	// if the length is very small
	if( length2 < 0.0000000000000001f )
		// then we return the maximum since CFL would cause a very very large value
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
void FluidSimulator3d::updateGrid( void )
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
		(*it).second.zNeighboursFluid = false;
	}

	// update and mark all cells which have currently fluid in it -----------------------------------------------
	// we do this by iterating over all markerParticles...
	for( std::vector<Particle3d>::iterator it=markerParticles.begin(); it != markerParticles.end(); ++it )
	{
		// is the particle not within simulation bounds?
		if( !simulationDomain.encloses( (*it).position ) )
			continue;

		// find the coordinates of the cell in which the current particle is located
		Cell::Coordinate cellCoord;

		cellCoord.i = (int)((*it).position.x / cellSize);
		cellCoord.j = (int)((*it).position.y / cellSize);
		cellCoord.k = (int)((*it).position.z / cellSize);

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
		(*cellEntry).second.zNeighboursFluid = true;

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
					for( unsigned int n = 0; n < 6; ++n )
					{
						// if the current neighbour already exists...
						neighbour = gridCells.find( (*it).first.getNeighbourCoordinate(n) );

						// cell not found?
						if( neighbour == gridCells.end() )
						{
							// create it
							neighbour = gridCells.insert( std::pair<Cell::Coordinate, Cell>( (*it).first.getNeighbourCoordinate(n), Cell() ) ).first;
							tag = true;

							// and update its interpolation neighbour
							updateNeighbours( &((*neighbour).second), (*neighbour).first );
							// cell creation callback
							onCellCreated( &neighbour->first, &neighbour->second );

							
							// within simulationbounds?
							math::Vec3f cellMin( (*neighbour).first.i * cellSize, (*neighbour).first.j * cellSize, (*neighbour).first.k * cellSize );
							math::Vec3f cellMax( ((*neighbour).first.i+1) * cellSize, ((*neighbour).first.j+1) * cellSize, ((*neighbour).first.k+1) * cellSize );

							if( (cellMin.x >= simulationDomain.minPoint.x)&&(cellMin.y >= simulationDomain.minPoint.y)&&(cellMin.z >= simulationDomain.minPoint.z)&&
								(cellMax.x <= simulationDomain.maxPoint.x)&&(cellMax.y <= simulationDomain.maxPoint.y)&&(cellMax.z <= simulationDomain.maxPoint.z))
							{
								// check wether the cell is occupied by one of the static solids within the domain
								math::Vec3f cellCenter = (cellMin + cellMax)*0.5f;

								bool isOccupied = false;
								for( std::vector<StaticSolid *>::iterator soIt = staticSolids.begin(); soIt != staticSolids.end(); ++soIt )
								{
									// transform the position of the current cell center into the local coordinate system of the currentshape
									math::Matrix44f itm = (*soIt)->getInverseTransformationMatrix();

									math::Vec3f cellCenterInStaticSolidSpace = transform( cellCenter, itm );

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
								// else set it to solid since its out of the simulation domain
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
							case 5: // back neighbour (k+1)
								// we know the fluid cell lies on the front side (k-1)
								(*neighbour).second.zNeighboursFluid = true;
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

	printf( "            computing distance\n" );
	// compute distances to fluid --------------------------------
	computeDistanceToFluid();
	printf( "            extrapolating velocities\n" );
	// extrapolate velocity into buffer cells second time -----------------
	extrapolateVelocities();


	// done - the grid is up and ready
}

//
// this method is used by updategrid when pic/flip us used and
// may be used to define the grid velocities in an intuitive manner
//
void FluidSimulator3d::transferParticleVelocitiesToGrid()
{
	printf( "            computing grid velocities from particles\n" );
	// PIC/FLIP method (cell velocities of fluidcells are retreived from surrounding particles) -----------------
	// transfere particle velocites to cell velocities which border fluid cells
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.sum = 0.0f;
		(*it).second.velocity.x = 0.0f;
	}

	for( std::vector<Particle3d>::iterator pIt=markerParticles.begin(); pIt != markerParticles.end(); ++pIt )
	{
		float x = (*pIt).position.x/cellSize;
		float y = (*pIt).position.y/cellSize - 0.5f;
		float z = (*pIt).position.z/cellSize - 0.5f;

		int i = (int)floor( x );
		int j = (int)floor( y );
		int k = (int)floor( z );

		// interpolation weights for each dimension
		float one_minus_u = i + 1.0f - x;
		float u = x - i;
		float one_minus_v = j + 1.0f - y;
		float v = y - j;
		float one_minus_w = k + 1.0f - z;
		float w = z - k;

		// try to get the cell
		stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j, k ) );

		if( cellIterator != gridCells.end() )
		{
			float weight;

			weight = one_minus_u*one_minus_v*one_minus_w;
			(*cellIterator).second.velocity.x += weight*(*pIt).velocity.x;
			(*cellIterator).second.sum += weight;

			// right front neighbour
			if( (*cellIterator).second.neighbours[1] )
			{
				weight = u*one_minus_v*one_minus_w;
				(*cellIterator).second.neighbours[1]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[1]->sum += weight;
			}
			// top front neighbour
			if( (*cellIterator).second.neighbours[3] )
			{
				weight = one_minus_u*v*one_minus_w;
				(*cellIterator).second.neighbours[3]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[3]->sum += weight;
			}
			// top right front neighbour
			if( (*cellIterator).second.neighbours[9] )
			{
				weight = u*v*one_minus_w;
				(*cellIterator).second.neighbours[9]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[9]->sum += weight;
			}

			// center back neighbour
			if( (*cellIterator).second.neighbours[5] )
			{
				weight = one_minus_u*one_minus_v*w;
				(*cellIterator).second.neighbours[5]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[5]->sum += weight;
			}

			// right back neighbour
			if( (*cellIterator).second.neighbours[6] )
			{
				weight = u*one_minus_v*w;
				(*cellIterator).second.neighbours[6]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[6]->sum += weight;
			}
			// top back neighbour
			if( (*cellIterator).second.neighbours[7] )
			{
				weight = one_minus_u*v*w;
				(*cellIterator).second.neighbours[7]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[7]->sum += weight;
			}
			// top right back neighbour
			if( (*cellIterator).second.neighbours[8] )
			{
				weight = u*v*w;
				(*cellIterator).second.neighbours[8]->velocity.x += weight*(*pIt).velocity.x;
				(*cellIterator).second.neighbours[8]->sum += weight;
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

	for( std::vector<Particle3d>::iterator pIt=markerParticles.begin(); pIt != markerParticles.end(); ++pIt )
	{
		float x = (*pIt).position.x/cellSize - 0.5f;
		float y = (*pIt).position.y/cellSize;
		float z = (*pIt).position.z/cellSize - 0.5f;

		int i = (int)floor( x );
		int j = (int)floor( y );
		int k = (int)floor( z );

		// interpolation weights for each dimension
		float one_minus_u = i + 1.0f - x;
		float u = x - i;
		float one_minus_v = j + 1.0f - y;
		float v = y - j;
		float one_minus_w = k + 1.0f - z;
		float w = z - k;

		// try to get the cell
		stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j, k ) );

		if( cellIterator != gridCells.end() )
		{
			float weight;

			weight = one_minus_u*one_minus_v*one_minus_w;
			(*cellIterator).second.velocity.y += weight*(*pIt).velocity.y;
			(*cellIterator).second.sum += weight;

			// right neighbour
			if( (*cellIterator).second.neighbours[1] )
			{
				weight = u*one_minus_v*one_minus_w;
				(*cellIterator).second.neighbours[1]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[1]->sum += weight;
			}
			// top neighbour
			if( (*cellIterator).second.neighbours[3] )
			{
				weight = one_minus_u*v*one_minus_w;
				(*cellIterator).second.neighbours[3]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[3]->sum += weight;
			}
			// top right neighbour
			if( (*cellIterator).second.neighbours[9] )
			{
				weight = u*v*one_minus_w;
				(*cellIterator).second.neighbours[9]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[9]->sum += weight;
			}


			// center back neighbour
			if( (*cellIterator).second.neighbours[5] )
			{
				weight = one_minus_u*one_minus_v*w;
				(*cellIterator).second.neighbours[5]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[5]->sum += weight;
			}

			// right back neighbour
			if( (*cellIterator).second.neighbours[6] )
			{
				weight = u*one_minus_v*w;
				(*cellIterator).second.neighbours[6]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[6]->sum += weight;
			}
			// top back neighbour
			if( (*cellIterator).second.neighbours[7] )
			{
				weight = one_minus_u*v*w;
				(*cellIterator).second.neighbours[7]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[7]->sum += weight;
			}
			// top right back neighbour
			if( (*cellIterator).second.neighbours[8] )
			{
				weight = u*v*w;
				(*cellIterator).second.neighbours[8]->velocity.y += weight*(*pIt).velocity.y;
				(*cellIterator).second.neighbours[8]->sum += weight;
			}
		}
	}

	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		if( (*it).second.sum != 0.0f )
			(*it).second.velocity.y /= (*it).second.sum;


	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		(*it).second.sum = 0.0f;
		(*it).second.velocity.z = 0.0f;
	}

	for( std::vector<Particle3d>::iterator pIt=markerParticles.begin(); pIt != markerParticles.end(); ++pIt )
	{
		float x = (*pIt).position.x/cellSize - 0.5f;
		float y = (*pIt).position.y/cellSize - 0.5f;
		float z = (*pIt).position.z/cellSize;

		int i = (int)floor( x );
		int j = (int)floor( y );
		int k = (int)floor( z );

		// interpolation weights for each dimension
		float one_minus_u = i + 1.0f - x;
		float u = x - i;
		float one_minus_v = j + 1.0f - y;
		float v = y - j;
		float one_minus_w = k + 1.0f - z;
		float w = z - k;

		// try to get the cell
		stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j, k ) );

		if( cellIterator != gridCells.end() )
		{
			float weight;

			weight = one_minus_u*one_minus_v*one_minus_w;
			(*cellIterator).second.velocity.z += weight*(*pIt).velocity.z;
			(*cellIterator).second.sum += weight;

			// right neighbour
			if( (*cellIterator).second.neighbours[1] )
			{
				weight = u*one_minus_v*one_minus_w;
				(*cellIterator).second.neighbours[1]->velocity.z += weight*(*pIt).velocity.z;
				(*cellIterator).second.neighbours[1]->sum += weight;
			}
			// top neighbour
			if( (*cellIterator).second.neighbours[3] )
			{
				weight = one_minus_u*v*one_minus_w;
				(*cellIterator).second.neighbours[3]->velocity.z += weight*(*pIt).velocity.z;
				(*cellIterator).second.neighbours[3]->sum += weight;
			}
			// top right neighbour
			if( (*cellIterator).second.neighbours[9] )
			{
				weight = u*v*one_minus_w;
				(*cellIterator).second.neighbours[9]->velocity.z += weight*(*pIt).velocity.z;
				(*cellIterator).second.neighbours[9]->sum += weight;
			}


			// center back neighbour
			if( (*cellIterator).second.neighbours[5] )
			{
				weight = one_minus_u*one_minus_v*w;
				(*cellIterator).second.neighbours[5]->velocity.z += weight*(*pIt).velocity.z;
				(*cellIterator).second.neighbours[5]->sum += weight;
			}

			// right back neighbour
			if( (*cellIterator).second.neighbours[6] )
			{
				weight = u*one_minus_v*w;
				(*cellIterator).second.neighbours[6]->velocity.z += weight*(*pIt).velocity.z;
				(*cellIterator).second.neighbours[6]->sum += weight;
			}
			// top back neighbour
			if( (*cellIterator).second.neighbours[7] )
			{
				weight = one_minus_u*v*w;
				(*cellIterator).second.neighbours[7]->velocity.z += weight*(*pIt).velocity.z;
				(*cellIterator).second.neighbours[7]->sum += weight;
			}
			// top right back neighbour
			if( (*cellIterator).second.neighbours[8] )
			{
				weight = u*v*w;
				(*cellIterator).second.neighbours[8]->velocity.z += weight*(*pIt).velocity.z;
				(*cellIterator).second.neighbours[8]->sum += weight;
			}
		}
	}

	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		if( (*it).second.sum != 0.0f )
			(*it).second.velocity.z /= (*it).second.sum;
}


//
// This method performs the actual solving of the navier-stokes equation
//
//
void FluidSimulator3d::updateVelocity( float dt )
{
	// store the velocity in the oldVelocity member of each cell so that we later can compute the change in velocity --------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.oldVelocity = (*it).second.velocity;

	// if advection is not done through particles (PIC/FLIP)
	if( !particleBasedAdvection )
		// then we use a sem-lagrange method to advect velocities...
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

























//
// This method will move the marker particles depending on the velocity in the grid through the grid
// Velocity is retreived from interpolation and a simple ODE is used for moving.
//
void FluidSimulator3d::moveParticles( float dt )
{
	float xmin=simulationDomain.minPoint.x + 0.001f*cellSize, xmax=simulationDomain.maxPoint.x - .001f*cellSize;
	float ymin=simulationDomain.minPoint.y + 0.001f*cellSize, ymax=simulationDomain.maxPoint.y - .001f*cellSize;
	float zmin=simulationDomain.minPoint.z + 0.001f*cellSize, zmax=simulationDomain.maxPoint.z - .001f*cellSize;


	// iterate over all particles - rk2
	for( std::vector<Particle3d>::iterator it=markerParticles.begin(); it != markerParticles.end(); ++it )
	{
		// perform one rk2 step

		// RK2 first stage (mid position)
		math::Vec3f velocity = getVelocity( (*it).position.x, (*it).position.y, (*it).position.z );

		float midPosx = (*it).position.x + velocity.x*0.5f*dt;
		float midPosy = (*it).position.y + velocity.y*0.5f*dt;
		float midPosz = (*it).position.z + velocity.z*0.5f*dt;

		// clamp
		if( midPosx < xmin )
			midPosx = xmin;
		if( midPosx > xmax )
			midPosx = xmax;
		if( midPosy < ymin )
			midPosy = ymin;
		if( midPosy > ymax )
			midPosy = ymax;
		if( midPosz < zmin )
			midPosz = zmin;
		if( midPosz > zmax )
			midPosz = zmax;
		
		// RK2 second stage (final position)
		velocity = getVelocity( midPosx, midPosy, midPosz );
		(*it).position.x += velocity.x*dt;
		(*it).position.y += velocity.y*dt;
		(*it).position.z += velocity.z*dt;

		// clamp
		if( (*it).position.x < xmin )
			(*it).position.x = xmin;
		if( (*it).position.x > xmax )
			(*it).position.x = xmax;
		if( (*it).position.y < ymin )
			(*it).position.y = ymin;
		if( (*it).position.y > ymax )
			(*it).position.y = ymax;
		if( (*it).position.z < zmin )
			(*it).position.z = zmin;
		if( (*it).position.z > zmax )
			(*it).position.z = zmax;
	}
}






//
// this method will set the velocities on solid cells
// which border non-solid cells so that free-slip, no-slip,
// or frictional boundary conditions are met
//
void FluidSimulator3d::setBoundaryConditions( void )
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
				cell->velocity.x = cell->neighbours[2]->velocity.x;
			}else
			// y+
			if( cell->neighbours[3] && (cell->neighbours[3]->type != Cell::Solid) )
			{
				cell->velocity.x = cell->neighbours[3]->velocity.x;
			}else
			// z-
			if( cell->neighbours[4] && (cell->neighbours[4]->type != Cell::Solid) )
			{
				cell->velocity.x = cell->neighbours[4]->velocity.x;
			}else
			// z+
			if( cell->neighbours[5] && (cell->neighbours[5]->type != Cell::Solid) )
			{
				cell->velocity.x = cell->neighbours[5]->velocity.x;
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
				cell->velocity.z = cell->neighbours[0]->velocity.z;
			}else
			// x+
			if( cell->neighbours[1] && (cell->neighbours[1]->type != Cell::Solid) )
			{
				cell->velocity.z = cell->neighbours[1]->velocity.z;
			}else
			// y-
			if( cell->neighbours[2] && (cell->neighbours[2]->type != Cell::Solid) )
			{
				cell->velocity.z = cell->neighbours[2]->velocity.z;
			}else
			// y+
			if( cell->neighbours[3] && (cell->neighbours[3]->type != Cell::Solid) )
			{
				cell->velocity.z = cell->neighbours[3]->velocity.z;
			}
		}
	}
}

//
// this method will solve the viscosity term...
//
void FluidSimulator3d::solveViscosity( float dt )
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
		(*it).second.tempVelocity.z = (*it).second.velocity.z; //

		//...and initializing velocity.x and y and z
		(*it).second.velocity.x = 0.0f;
		(*it).second.velocity.y = 0.0f;
		(*it).second.velocity.z = 0.0f;
/*
		if( !(*it).second.xNeighboursFluid )
			(*it).second.velocity.x = 0.0f;
		if( !(*it).second.yNeighboursFluid  )
			(*it).second.velocity.y = 0.0f;
		if( !(*it).second.zNeighboursFluid  )
			(*it).second.velocity.z = 0.0f;
*/
	}


	// solve the system with the gauss-seidel-relaxation method
	for( unsigned int k=0; k<200; ++k )
		for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
			if( (*it).second.xNeighboursFluid || (*it).second.yNeighboursFluid || (*it).second.zNeighboursFluid )
			{
				float tx = (*it).second.tempVelocity.x;
				float xNeighbourCount = 0.0f;      // how many neighbours go into the equation for the x-component
				float ty = (*it).second.tempVelocity.y;
				float yNeighbourCount = 0.0f;      // how many neighbours go into the equation for the y-component
				float tz = (*it).second.tempVelocity.z;
				float zNeighbourCount = 0.0f;      // how many neighbours go into the equation for the z-component

				for( unsigned int n = 0; n<6; ++n )
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
					if( (*it).second.neighbours[n]->zNeighboursFluid )
					{
						tz += a*(*it).second.neighbours[n]->velocity.z;
						zNeighbourCount += 1.0f;
					}
				}

				if( (*it).second.xNeighboursFluid )
					(*it).second.velocity.x = tx / (1+a*xNeighbourCount);
				if( (*it).second.yNeighboursFluid  )
					(*it).second.velocity.y = ty / (1+a*yNeighbourCount);
				if( (*it).second.zNeighboursFluid  )
					(*it).second.velocity.z = tz / (1+a*zNeighbourCount);
			}
}

//
// will add dt*math::Vec3f(x,y,z) to the respective velocity components, which neighbour fluid cells
//
void FluidSimulator3d::applyGravity( float dt, float x, float y, float z )
{
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// we have to advance velocity-components of all cells which border fluid cells
		if( (*it).second.xNeighboursFluid )
			(*it).second.velocity.x += x*dt;
		if( (*it).second.yNeighboursFluid )
			(*it).second.velocity.y += y*dt;
		if( (*it).second.zNeighboursFluid )
			(*it).second.velocity.z += z*dt;
	}	
}

//
// this will solve the pressure-term and will make the
// grid velocities divergence free
//
void FluidSimulator3d::solvePressure( float dt )
{
	printf( "            solving for pressure\n" );

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

			// z-component
			if( (*it).second.neighbours[4]->type == Cell::Solid )
				u1 = 0.0f;
			else
				u1 = (*it).second.velocity.z;
			if( (*it).second.neighbours[5]->type == Cell::Solid )
				u2 = 0.0f;
			else
				u2 = (*it).second.neighbours[5]->velocity.z;
			(*it).second.tempVelocity.x += u2 - u1;




			(*it).second.tempVelocity.x *= cellSize/dt;

			(*it).second.tempVelocity.y = 6.0f;                // by default we assume that all neighbour cells are non-solid

			// determining the number of non-solid neighbours which will enter the actual solving
			// and initiating the pressure of all nonfluid neighbours
			for( unsigned int n = 0; n<6; ++n )
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
					                                                 +(*it).second.neighbours[2]->pressure+(*it).second.neighbours[3]->pressure
																	 +(*it).second.neighbours[4]->pressure+(*it).second.neighbours[5]->pressure)/(*it).second.tempVelocity.y;
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
		if( !(*it).second.neighbours[0] || !(*it).second.neighbours[2] || !(*it).second.neighbours[4] )
			continue;

		// compute the pressure gradient

		// we will use backward differences here since the pressure gradient of a air cell which is on the right side
		// of a fluid cell could not be computed with forward differences (but the pressure gradient of a air cell which
		// is on the left side of a fluid cell can be computed with backward differences )
		math::Vec3f pressureGradient( (*it).second.pressure - (*it).second.neighbours[0]->pressure, (*it).second.pressure - (*it).second.neighbours[2]->pressure, (*it).second.pressure - (*it).second.neighbours[4]->pressure );

		pressureGradient.x *= dt / cellSize;
		pressureGradient.y *= dt / cellSize;
		pressureGradient.z *= dt / cellSize;



		// update only velocity-components which border fluid cells but not solid cells !
		if( (*it).second.type == Cell::Fluid )
		{
			if( (*it).second.neighbours[0]->type != Cell::Solid )
				(*it).second.tempVelocity.x = (*it).second.velocity.x - pressureGradient.x;

			if( (*it).second.neighbours[2]->type != Cell::Solid )
				(*it).second.tempVelocity.y = (*it).second.velocity.y - pressureGradient.y;

			if( (*it).second.neighbours[4]->type != Cell::Solid )
				(*it).second.tempVelocity.z = (*it).second.velocity.z - pressureGradient.z;

		}else
			if( (*it).second.type == Cell::Air )
			{
				if( (*it).second.neighbours[0]->type == Cell::Fluid )
					(*it).second.tempVelocity.x = (*it).second.velocity.x - pressureGradient.x;

				if( (*it).second.neighbours[2]->type == Cell::Fluid )
					(*it).second.tempVelocity.y = (*it).second.velocity.y - pressureGradient.y;

				if( (*it).second.neighbours[4]->type == Cell::Fluid )
					(*it).second.tempVelocity.z = (*it).second.velocity.z - pressureGradient.z;
			}
	}

	// we have stored the new velocities in the tempVelocity vectors, now since all cells are updated we can copy
	// them to the real velocity vectors
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.velocity = (*it).second.tempVelocity;
}


//
// if simple MAC simulation is used (no PIC/FLIP), the velocities
// of the grid have to be advected...
//
void FluidSimulator3d::advectGridVelocities( float dt )
{
	printf( "            computing advection\n" );

	// apply convection (ONLY WHEN NO PIC/FLIP) ---------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		// we have to advance velocity-components which border fluid cells
		if( (*it).second.xNeighboursFluid )
		{
			math::Vec3f velXPos = traceParticle( math::Vec3f( (*it).first.i * cellSize, (*it).first.j * cellSize+(cellSize/2.0f), (*it).first.k * cellSize+(cellSize/2.0f) ), dt );
			(*it).second.tempVelocity.x = getVelocity( velXPos.x, velXPos.y, velXPos.z ).x;
		}else
			(*it).second.tempVelocity.x = (*it).second.velocity.x;
		if( (*it).second.yNeighboursFluid )
		{
			math::Vec3f velYPos = traceParticle( math::Vec3f( (*it).first.i * cellSize+(cellSize/2.0f), (*it).first.j * cellSize, (*it).first.k * cellSize+(cellSize/2.0f) ), dt );
			(*it).second.tempVelocity.y = getVelocity( velYPos.x, velYPos.y, velYPos.z ).y;
		}else
			(*it).second.tempVelocity.y = (*it).second.velocity.y;
		if( (*it).second.zNeighboursFluid )
		{
			math::Vec3f velZPos = traceParticle( math::Vec3f( (*it).first.i * cellSize+(cellSize/2.0f), (*it).first.j * cellSize+(cellSize/2.0f), (*it).first.k * cellSize ), dt );
			(*it).second.tempVelocity.z = getVelocity( velZPos.x, velZPos.y, velZPos.z ).z;
		}else
			(*it).second.tempVelocity.z = (*it).second.velocity.z;

	}
	// we have stored the new velocities in the tempVelocity vectors, now since all cells are updated we can copy
	// them to the real velocity vectors
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		// we have to take into account these velocitie-components which border fluid cells
		(*it).second.velocity = (*it).second.tempVelocity;
}







//
// This is the actual distance computation proposed in [Zhao2005]
//
void FluidSimulator3d::sweepUpdateDistances( Cell *cell, int horizontalNeighbourIndex, int verticalNeighbourIndex, int depthNeighbourIndex )
{
	float aDistance = 100.0f;
	float bDistance = 100.0f;
	float cDistance = 100.0f;
	float h = cellSize;

	// try to access horizontal neighbour
	if( cell->neighbours[horizontalNeighbourIndex] )
		// and take its distance
		aDistance = cell->neighbours[horizontalNeighbourIndex]->signedDistance;

	// try to access vertical neighbour
	if( cell->neighbours[verticalNeighbourIndex] )
		// and take its distance
		bDistance = cell->neighbours[verticalNeighbourIndex]->signedDistance;

	// try to access depth neighbour
	if( cell->neighbours[depthNeighbourIndex] )
		// and take its distance
		cDistance = cell->neighbours[depthNeighbourIndex]->signedDistance;

	float phi = cell->signedDistance;

	// sort distances in increasing order
	if( fabs( aDistance ) > fabs( bDistance ) ){ float t = aDistance; aDistance = bDistance; bDistance = t; }
	if( fabs( bDistance ) > fabs( cDistance ) ){ float t = bDistance; bDistance = cDistance; cDistance = t; }
	if( fabs( aDistance ) > fabs( bDistance ) ){ float t = aDistance; aDistance = bDistance; bDistance = t; }

	if( fabs(phi) <= fabs( aDistance ) )return;

	phi = aDistance + h;

	if( fabs(phi) >= fabs( bDistance ) )
	{
		phi = 0.5f*( aDistance + bDistance + sqrt(2*h*h - (bDistance-aDistance)*(bDistance-aDistance)));

		if( fabs(phi) >= fabs( cDistance ) )
		{
			phi = (aDistance + bDistance + cDistance)/3.0f + 0.5f*sqrt( 4.0f/9.0f*(aDistance + bDistance + cDistance)*(aDistance + bDistance + cDistance) - 4.0f/3.0f*(aDistance*aDistance+bDistance*bDistance+cDistance*cDistance - h*h) );
		}
	}

	if( phi < cell->signedDistance )
		cell->signedDistance = phi;
}




//
// Computes the signed distance to the fluid surface for each cell in the grid using fast sweeping [Zhao 2005].
// We need the signed distances for each cell within the grid for velocity extrapolation.
//
void FluidSimulator3d::computeDistanceToFluid( void )
{
	// the fast sweeping algorithm [Zhao2005] is used to compute the signed
	// distance function \sigma for each velocity component which doesnt border a fluid cell

	// preparation of the fast sweeping algorithm -------------------------------------------
	float h = cellSize;

	int       Istart;    // index of the first gridpoint/cell in x direction (needed since we have a dynamic grid)
	int       Jstart;    // index of the first gridpoint/cell in y direction (needed since we have a dynamic grid)
	int       Kstart;    // index of the first gridpoint/cell in z direction (needed since we have a dynamic grid)
	int            I;    // index of the last gridpoint/cell in x direction (needed since we have a dynamic grid)
	int            J;    // index of the last gridpoint/cell in y direction (needed since we have a dynamic grid)
	int            K;    // index of the last gridpoint/cell in z direction (needed since we have a dynamic grid)

	// intiate all values from the first cell in town
	Istart = I = (*gridCells.begin()).first.i;
	Jstart = J = (*gridCells.begin()).first.j;
	Kstart = K = (*gridCells.begin()).first.k;

	// evaluate the parameters Istart, Jstart, I and J
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		if( (*it).first.i < Istart )
			Istart = (*it).first.i;
		if( (*it).first.j < Jstart )
			Jstart = (*it).first.j;
		if( (*it).first.k < Kstart )
			Kstart = (*it).first.k;

		if( (*it).first.i > I )
			I = (*it).first.i;
		if( (*it).first.j > J )
			J = (*it).first.j;
		if( (*it).first.k > K )
			K = (*it).first.k;
	}

	// fast sweeping algorithm: initialization -------------------------------------------

	// set the distance function for all velocity components which border fluidcells to 0 and to a very large positive
	// value otherwise
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		if( (*it).second.type == Cell::Fluid )
		{
			(*it).second.signedDistance = -0.5f*cellSize;
		}else
			(*it).second.signedDistance = 99999999.0f;
	}


	Cell *cell = 0;


	// fast sweeping algorithm: iterations -----------------------------------------------
	for(int k=0; k<2; ++k)
	{
		// now we do 8 sweeps (one for each quadrant) and update the signed distance values
		//

		// Quadrant I   : i 0->1 | j 0->1
		for( int k=Kstart; k<=K; ++k )
		{
			cell = 0;
			for( int j=Jstart; j<=J; ++j  )
			{
				cell = 0;
				for( int i=Istart; i<=I; ++i, cell= cell ? cell->neighbours[1] : 0  )		
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}


					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 0, 2, 4 );
				}
			}
		}

		// Quadrant II  : i 1->0 | j 0->1
		for( int k=Kstart; k<=K; ++k )
		{
			cell = 0;
			for( int j=Jstart; j<=J; ++j  )
			{
				cell = 0;
				for( int i=I; i>=Istart; --i, cell= cell ? cell->neighbours[0] : 0   )
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}

					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 1, 2, 4 );
				}
			}
		}

		// Quadrant III : i 1->0 | j 1->0
		for( int k=Kstart; k<=K; ++k )
		{
			cell = 0;
			for( int j=J; j>=Jstart; --j  )
			{
				cell = 0;
				for( int i=I; i>=Istart; --i, cell= cell ? cell->neighbours[0] : 0   )
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}


					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 1, 3, 4 );
				}
			}
		}

		// Quadrant IV  : i 0->1 | j 1->0
		for( int k=Kstart; k<=K; ++k )
		{
			cell = 0;
			for( int j=J; j>=Jstart; --j  )
			{
				cell = 0;
				for( int i=Istart; i<=I; ++i, cell= cell ? cell->neighbours[1] : 0   )		
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}


					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 0, 3, 4 );
				}
			}
		}


		// Quadrant I   : i 0->1 | j 0->1
		for( int k=K; k>=Kstart; --k )
		{
			cell = 0;
			for( int j=Jstart; j<=J; ++j  )
			{
				cell = 0;
				for( int i=Istart; i<=I; ++i, cell= cell ? cell->neighbours[1] : 0   )		
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}


					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 0, 2, 5 );
				}
			}
		}

		// Quadrant II  : i 1->0 | j 0->1
		for( int k=K; k>=Kstart; --k )
		{
			cell = 0;
			for( int j=Jstart; j<=J; ++j  )
			{
				cell = 0;
				for( int i=I; i>=Istart; --i, cell= cell ? cell->neighbours[0] : 0   )
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}


					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 1, 2, 5 );
				}
			}
		}

		// Quadrant III : i 1->0 | j 1->0
		for( int k=K; k>=Kstart; --k )
		{
			cell = 0;
			for( int j=J; j>=Jstart; --j  )
			{
				cell = 0;
				for( int i=I; i>=Istart; --i, cell= cell ? cell->neighbours[0] : 0   )
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}


					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 1, 3, 5 );
				}
			}
		}

		// Quadrant IV  : i 0->1 | j 1->0
		for( int k=K; k>=Kstart; --k )
		{
			cell = 0;
			for( int j=J; j>=Jstart; --j  )
			{
				cell = 0;
				for( int i=Istart; i<=I; ++i, cell= cell ? cell->neighbours[1] : 0   )		
				{
					if( cell == 0 )
					{
						stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

						if( it == gridCells.end() )
						{
							cell = 0;
							continue;
						}else
							cell = &((*it).second);
					}


					// compute the distance-value and update the cell if it is smaller than the current one
					if( cell->type != Cell::Fluid )
						sweepUpdateDistances( cell, 0, 3, 5 );
				}
			}
		}
	}
}



//
// General implementation of the sweeping for velocity extrapolation for the v-component of the velocity
//
void FluidSimulator3d::sweepVelocityUVW( int iStart, int iEnd, int jStart, int jEnd, int kStart, int kEnd )
{
	// compute increments which are needed to run from the start to end values
	// so the increments are either 1 (running from min to max) or -1 (running from max to min)
	int di = (iStart<iEnd) ? 1 : -1;
	int dj = (jStart<jEnd) ? 1 : -1;
	int dk = (kStart<kEnd) ? 1 : -1;

	int horizontalNeighbourIndex = (di > 0) ? 0 : 1;
	int verticalNeighbourIndex = (dj > 0) ? 2 : 3;
	int depthNeighbourIndex = (dk > 0) ? 4 : 5;

	int oppositeHorizontalNeighbourIndex = (di > 0) ? 1 : 0;
	int oppositeVerticalNeighbourIndex = (dj > 0) ? 3 : 2;
	int oppositeDepthNeighbourIndex = (dk > 0) ? 5 : 4;

	int nextNeighbourIndex = (di > 0) ? 1 : 0;

	Cell *cell = 0;

	// for each depth cell
	for( int k=kStart; k!=kEnd; k+=dk )
	{
		cell = 0;

		// for each vertical cell
		for( int j=jStart; j!=jEnd; j+=dj )
		{
			cell = 0;

			// for each horizontal cell
			for( int i=iStart; i!=iEnd; i+=di, cell= cell ? cell->neighbours[nextNeighbourIndex] : 0 )
			{
				if( cell == 0 )
				{
					stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.find( Cell::Coordinate( i, j, k ) );

					if( it == gridCells.end() )
						cell = 0;
					else
						cell = &((*it).second);
				}

				// try to retrieve the current indexed cell
				FluidSimulator3d::Cell *temp;

				// cell not found?
				if( cell == 0 )
					// skip current loopcycle
					continue;

				if( cell->type == Cell::Fluid )
					continue;


				// distance value of center cell               (i,j,k)
				float p_c = cell->signedDistance;

				// distance value of previous cell in x        (i-di,j,k)
				float p_px = 100.0f;
				float ux = cell->velocity.x;
				float vx = cell->velocity.y;
				float wx = cell->velocity.z;
				if( cell->neighbours[horizontalNeighbourIndex] )
				{
					p_px = cell->neighbours[horizontalNeighbourIndex]->signedDistance;
					ux = cell->neighbours[horizontalNeighbourIndex]->velocity.x;
					vx = cell->neighbours[horizontalNeighbourIndex]->velocity.y;
					wx = cell->neighbours[horizontalNeighbourIndex]->velocity.z;
				}

				// distance value of opposite previous cell in x        (i+di,j,k)
				float p_opx = 100.0f;
				if( cell->neighbours[oppositeHorizontalNeighbourIndex] )
					p_opx = cell->neighbours[oppositeHorizontalNeighbourIndex]->signedDistance;

				// distance value of previous cell in y        (i,j-dj,k)
				float p_py = 100.0f;
				float uy = cell->velocity.x;
				float vy = cell->velocity.y;
				float wy = cell->velocity.z;
				if( cell->neighbours[verticalNeighbourIndex] )
				{
					p_py = cell->neighbours[verticalNeighbourIndex]->signedDistance;
					uy = cell->neighbours[verticalNeighbourIndex]->velocity.x;
					vy = cell->neighbours[verticalNeighbourIndex]->velocity.y;
					wy = cell->neighbours[verticalNeighbourIndex]->velocity.z;
				}

				// distance value of opposite previous cell in y        (i,j+dj,k)
				float p_opy = 100.0f;
				if( cell->neighbours[oppositeVerticalNeighbourIndex] )
					p_opy = cell->neighbours[oppositeVerticalNeighbourIndex]->signedDistance;


				// distance value of previous cell in z        (i,j,k-dk)
				float p_pz = 100.0f;
				float uz = cell->velocity.x;
				float vz = cell->velocity.y;
				float wz = cell->velocity.z;
				if( cell->neighbours[depthNeighbourIndex] )
				{
					p_pz = cell->neighbours[depthNeighbourIndex]->signedDistance;
					uz = cell->neighbours[depthNeighbourIndex]->velocity.x;
					vz = cell->neighbours[depthNeighbourIndex]->velocity.y;
					wz = cell->neighbours[depthNeighbourIndex]->velocity.z;
				}

				// distance value of opposite previous cell in z        (i,j,k+dk)
				float p_opz = 100.0f;
				if( cell->neighbours[oppositeDepthNeighbourIndex] )
					p_opz = cell->neighbours[oppositeDepthNeighbourIndex]->signedDistance;


				// additional cells for sweepingW ---------------
				// distance value of front neighbour cell       (i,j,k-1)
				float p_f = 100.0f;
				if( cell->neighbours[4] )
					p_f = cell->neighbours[4]->signedDistance;

				// distance value of previous cell in x and front        (i-di,j,k-1)
				temp = cell->getNeighbour( -di, 0, -1 );
				float p_px_f = 100.0f;
				if( temp )
					p_px_f = temp->signedDistance;

				// distance value of opposite previous cell in x and front        (i+di,j,k-1)
				temp = cell->getNeighbour( di, 0, -1 );
				float p_opx_f = 100.0f;
				if( temp )
					p_opx_f = temp->signedDistance;

				// distance value of previous cell in y and front        (i,j-dj,k-1)
				temp = cell->getNeighbour( 0, -dj, -1 );
				float p_py_f = 100.0f;
				if( temp )
					p_py_f = temp->signedDistance;

				// distance value of opposite previous cell in y and front        (i,j+dj,k-1)
				temp = cell->getNeighbour( 0, dj, -1 );
				float p_opy_f = 100.0f;
				if( temp )
					p_opy_f = temp->signedDistance;


				// additional cells for sweepingV ---------------
				// distance value of bottom neighbour cell       (i,j-1,k)
				float p_b = 100.0f;
				if( cell->neighbours[2] )
					p_b = cell->neighbours[2]->signedDistance;

				// distance value of previous cell in x and bottom        (i-di,j-1,k)
				temp = cell->getNeighbour( -di, -1, 0 );
				float p_px_b = 100.0f;
				if( temp )
					p_px_b = temp->signedDistance;

				// distance value of opposite previous cell in x and bottom        (i+di,j-1,k)
				temp = cell->getNeighbour( di, -1, 0 );
				float p_opx_b = 100.0f;
				if( temp )
					p_opx_b = temp->signedDistance;

				// distance value of previous cell in z and bottom        (i,j-1,k-dk)
				temp = cell->getNeighbour( 0, -1, -dk );
				float p_pz_b = 100.0f;
				if( temp )
					p_pz_b = temp->signedDistance;

				// distance value of previous cell in z and bottom        (i,j-1,k+dk)
				temp = cell->getNeighbour( 0, -1, dk );
				float p_opz_b = 100.0f;
				if( temp )
					p_opz_b = temp->signedDistance;

				// additional cells for sweepingU ---------------
				// distance value of left neighbour cell       (i-1,j,k)
				float p_l = 100.0f;
				if( cell->neighbours[0] )
					p_l = cell->neighbours[0]->signedDistance;

				// distance value of previous cell in y and left        (i-1,j-dj,k)
				temp = cell->getNeighbour( -1, -dj, 0 );
				float p_py_l = 100.0f;
				if( temp )
					p_py_l = temp->signedDistance;

				// distance value of opposite previous cell in y and left        (i-1,j+dj,k)
				temp = cell->getNeighbour( -1, dj, 0 );
				float p_opy_l = 100.0f;
				if( temp )
					p_opy_l = temp->signedDistance;

				// distance value of previous cell in z and left        (i-1,j,k-dk)
				temp = cell->getNeighbour( -1, 0, -dk );
				float p_pz_l = 100.0f;
				if( temp )
					p_pz_l = temp->signedDistance;

				// distance value of opposite previous cell in z and left        (i-1,j,k+dk)
				temp = cell->getNeighbour( -1, 0, dk );
				float p_opz_l = 100.0f;
				if( temp )
					p_opz_l = temp->signedDistance;

				// compute U ---------------
				if( !cell->xNeighboursFluid )
				{
					// change of distance in x direction
					float deltaPhiX = di*(p_c - p_l);

					// continue only if distance gets larger
					if( deltaPhiX >= 0.0f )
					{
						// change of distance in y direction
						float deltaPhiY = 0.5f*( (p_opy_l - p_py_l)/2.f + (p_opy - p_py)/2.f );

						// continue only if distance gets larger
						if( deltaPhiY >= 0.0f )
						{
							// change of distance in z direction
							float deltaPhiZ = 0.5f*( (p_opz_l - p_pz_l)/2.f + (p_opz - p_pz)/2.f );

							// continue only if distance gets larger
							if( deltaPhiZ >= 0.0f )
							{
								// now compute the final velocity u component from the previous neighbour cells
								// weighted by the change of distance in their axis
								float wx,wy,wz;
								if( deltaPhiX+deltaPhiY+deltaPhiZ == 0.0f )
									// every neighbour has the same share on the final value
									wx = wy = wz = 0.33333333333f;
								else
								{
									wx = deltaPhiX / (deltaPhiX+deltaPhiY+deltaPhiZ);
									wy = deltaPhiY / (deltaPhiX+deltaPhiY+deltaPhiZ);
									wz = deltaPhiZ / (deltaPhiX+deltaPhiY+deltaPhiZ);
								}

								cell->velocity.x = wx*ux + wy*uy + wz*uz;
							}
						}
					}
				}

				// compute V ---------------
				if( !cell->yNeighboursFluid )
				{
					// change of distance in y direction
					float deltaPhiY = dj*(p_c - p_b);

					// continue only if distance gets larger
					if( deltaPhiY >= 0.0f )
					{
						// change of distance in x direction
						float deltaPhiX = 0.5f*( (p_opx_b - p_px_b)/2.f + (p_opx - p_px)/2.f );

						// continue only if distance gets larger
						if( deltaPhiX >= 0.0f )
						{
							// change of distance in z direction
							float deltaPhiZ = 0.5f*( (p_opz_b - p_pz_b)/2.f + (p_opz - p_pz)/2.f );

							// continue only if distance gets larger
							if( deltaPhiZ >= 0.0f )
							{
								// now compute the final velocity u component from the previous neighbour cells
								// weighted by the change of distance in their axis
								float wx,wy,wz;
								if( deltaPhiX+deltaPhiY+deltaPhiZ == 0.0f )
									// every neighbour has the same share on the final value
									wx = wy = wz = 0.33333333333f;
								else
								{
									wx = deltaPhiX / (deltaPhiX+deltaPhiY+deltaPhiZ);
									wy = deltaPhiY / (deltaPhiX+deltaPhiY+deltaPhiZ);
									wz = deltaPhiZ / (deltaPhiX+deltaPhiY+deltaPhiZ);
								}

								cell->velocity.y = wx*vx + wy*vy + wz*vz;
							}
						}
					}
				}


				// compute W ---------------
				if( !cell->zNeighboursFluid )
				{
					// change of distance in z direction
					float deltaPhiZ = dk*(p_c - p_f);

					// continue only if distance gets larger
					if( deltaPhiZ >= 0.0f )
					{

						// change of distance in y direction
						float deltaPhiX = 0.5f*( (p_opx_f - p_px_f)/2.f + (p_opx - p_px)/2.f );

						// continue only if distance gets larger
						if( deltaPhiX >= 0.0f )
						{
							// change of distance in y direction
							float deltaPhiY = 0.5f*( (p_opy_f - p_py_f)/2.f + (p_opy - p_py)/2.f );

							// continue only if distance gets larger
							if( deltaPhiY >= 0.0f )
							{
								// now compute the final velocity u component from the previous neighbour cells
								// weighted by the change of distance in their axis
								float weightx,weighty,weightz;
								if( deltaPhiX+deltaPhiY+deltaPhiZ == 0.0f )
									// every neighbour has the same share on the final value
									weightx = weighty = weightz = 0.33333333333f;
								else
								{
									weightx = deltaPhiX / (deltaPhiX+deltaPhiY+deltaPhiZ);
									weighty = deltaPhiY / (deltaPhiX+deltaPhiY+deltaPhiZ);
									weightz = deltaPhiZ / (deltaPhiX+deltaPhiY+deltaPhiZ);
								}

								cell->velocity.z = weightx*wx + weighty*wy + weightz*wz;
							}
						}
					}
				}

			}
		}
	}
}

//
// this method will compute all the velocity components of grid-cell-components which dont border fluid cells
//
void FluidSimulator3d::extrapolateVelocities( void )
{
	// quit if there are no cells
	if( !gridCells.size() )
		return;

	float h = cellSize;

	int       iMin;    // index of the first gridpoint/cell in x direction (needed since we have a dynamic grid)
	int       jMin;    // index of the first gridpoint/cell in y direction (needed since we have a dynamic grid)
	int       kMin;    // index of the first gridpoint/cell in z direction (needed since we have a dynamic grid)
	int       iMax;    // index of the last gridpoint/cell in x direction (needed since we have a dynamic grid)
	int       jMax;    // index of the last gridpoint/cell in y direction (needed since we have a dynamic grid)
	int       kMax;    // index of the last gridpoint/cell in z direction (needed since we have a dynamic grid)

	// intiate all values from the first cell in town
	iMin = iMax = (*gridCells.begin()).first.i;
	jMin = jMax = (*gridCells.begin()).first.j;
	kMin = kMax = (*gridCells.begin()).first.k;

	// evaluate the parameters Istart, Jstart, I and J and K
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		if( (*it).first.i < iMin )
			iMin = (*it).first.i;
		if( (*it).first.j < jMin )
			jMin = (*it).first.j;
		if( (*it).first.k < kMin )
			kMin = (*it).first.k;
		if( (*it).first.i > iMax )
			iMax = (*it).first.i;
		if( (*it).first.j > jMax )
			jMax = (*it).first.j;
		if( (*it).first.k > kMax )
			kMax = (*it).first.k;
	}



	// fast sweeping algorithm: iterations for solving pde \nabla u \dot \nabla \sigma = 0--------------

	// 4 iterations in 2d
	// maybe we have to do 8 iterations
	for(int k=0; k<8; ++k)
	{
		sweepVelocityUVW( iMin, iMax+1, jMax, jMin-1, kMin, kMax+1 );  // Quadrant IV
		sweepVelocityUVW( iMax, iMin-1, jMin, jMax+1, kMin, kMax+1 );  // Quadrant II
		sweepVelocityUVW( iMax, iMin-1, jMax, jMin-1, kMin, kMax+1 );  // Quadrant III
		sweepVelocityUVW( iMin, iMax+1, jMin, jMax+1, kMin, kMax+1 );  // Quadrant I
		sweepVelocityUVW( iMin, iMax+1, jMax, jMin-1, kMax, kMin-1 );  // Quadrant IV
		sweepVelocityUVW( iMax, iMin-1, jMin, jMax+1, kMax, kMin-1 );  // Quadrant II
		sweepVelocityUVW( iMax, iMin-1, jMax, jMin-1, kMax, kMin-1 );  // Quadrant III
		sweepVelocityUVW( iMin, iMax+1, jMin, jMax+1, kMax, kMin-1 );  // Quadrant I
	}
}










//
// returns the index-specified trilinear interpolated velocity-component
// from the grid.
//
float FluidSimulator3d::getInterpolatedVelocityValue( float x, float y, float z, unsigned int componentIndex  )
{
	// coordinate of the lower left cell
	int i = (int)floor( x );
	int j = (int)floor( y );
	int k = (int)floor( z );


	// interpolation weights for each dimension
	float u = i + 1.0f - x;
	float one_minus_u = x - i;
	float v = j + 1.0f - y;
	float one_minus_v = y - j;
	float w = k + 1.0f - z;
	float one_minus_w = z - k;

	// retreive the indexed cell
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j, k ) );
	Cell *cell;

	// if the cell could not be found
	if( cellIterator == gridCells.end() )
		// quit the function
		return 0.0f;
	else
		// set the reference to the cell
		cell = &((*cellIterator).second);

	// does every neighbour of the current cell exist?
	// does every neighbour of the current cell exist?
	if( cell->allInterpolationNeighboursExist )
	{
		// then calculate the full trilinear interpolation
		return u*v*w*cell->velocity[componentIndex] + one_minus_u*v*w*cell->neighbours[1]->velocity[componentIndex] +
			u*one_minus_v*w*cell->neighbours[3]->velocity[componentIndex] + one_minus_u*one_minus_v*w*cell->neighbours[9]->velocity[componentIndex] +
			u*v*one_minus_w*cell->neighbours[5]->velocity[componentIndex] + one_minus_u*v*one_minus_w*cell->neighbours[6]->velocity[componentIndex] +
			u*one_minus_v*one_minus_w*cell->neighbours[7]->velocity[componentIndex] + one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->velocity[componentIndex];
	}else
	{
		// we have to check each neighbour cell individiually and leave it out in our calculation
		float result = u*v*w*cell->velocity[componentIndex];  // this float will hold the sum of all values which entered the calculation
		float resultWeighting = u*v*w;                        // this variable will hold the sum of all weights which entered the calculation

		if( cell->neighbours[1] )
		{
			result += one_minus_u*v*w*cell->neighbours[1]->velocity[componentIndex];
			resultWeighting += one_minus_u*v*w;
		}
		if( cell->neighbours[3] )
		{
			result += u*one_minus_v*w*cell->neighbours[3]->velocity[componentIndex];
			resultWeighting += u*one_minus_v*w;
		}
		if( cell->neighbours[9] )
		{
			result += one_minus_u*one_minus_v*w*cell->neighbours[9]->velocity[componentIndex];
			resultWeighting += one_minus_u*one_minus_v*w;
		}
		if( cell->neighbours[5] )
		{
			result += u*v*one_minus_w*cell->neighbours[5]->velocity[componentIndex];
			resultWeighting += u*v*one_minus_w;
		}
		if( cell->neighbours[6] )
		{
			result += one_minus_u*v*one_minus_w*cell->neighbours[6]->velocity[componentIndex];
			resultWeighting += one_minus_u*v*one_minus_w;
		}
		if( cell->neighbours[7] )
		{
			result += u*one_minus_v*one_minus_w*cell->neighbours[7]->velocity[componentIndex];
			resultWeighting += u*one_minus_v*one_minus_w;
		}
		if( cell->neighbours[8] )
		{
			result += one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->velocity[componentIndex];
			resultWeighting += one_minus_u*one_minus_v*one_minus_w;
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
float FluidSimulator3d::getInterpolatedChangeValue( float x, float y, float z, unsigned int componentIndex  )
{
	// coordinate of the lower left cell
	int i = (int)floor( x );
	int j = (int)floor( y );
	int k = (int)floor( z );


	// interpolation weights for each dimension
	float u = i + 1.0f - x;
	float one_minus_u = x - i;
	float v = j + 1.0f - y;
	float one_minus_v = y - j;
	float w = k + 1.0f - z;
	float one_minus_w = z - k;

	// retreive the indexed cell
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator = gridCells.find( Cell::Coordinate( i, j, k ) );
	Cell *cell;

	// if the cell could not be found
	if( cellIterator == gridCells.end() )
		// quit the function
		return 0.0f;
	else
		// set the reference to the cell
		cell = &((*cellIterator).second);

	// does every neighbour of the current cell exist?
	// does every neighbour of the current cell exist?
	if( cell->allInterpolationNeighboursExist )
	{
		// then calculate the full trilinear interpolation
		return u*v*w*cell->velocityChange[componentIndex] + one_minus_u*v*w*cell->neighbours[1]->velocityChange[componentIndex] +
			u*one_minus_v*w*cell->neighbours[3]->velocityChange[componentIndex] + one_minus_u*one_minus_v*w*cell->neighbours[9]->velocityChange[componentIndex] +
			u*v*one_minus_w*cell->neighbours[5]->velocityChange[componentIndex] + one_minus_u*v*one_minus_w*cell->neighbours[6]->velocityChange[componentIndex] +
			u*one_minus_v*one_minus_w*cell->neighbours[7]->velocityChange[componentIndex] + one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->velocityChange[componentIndex];
	}else
	{
		// we have to check each neighbour cell individiually and leave it out in our calculation
		float result = u*v*w*cell->velocityChange[componentIndex];  // this float will hold the sum of all values which entered the calculation
		float resultWeighting = u*v*w;                        // this variable will hold the sum of all weights which entered the calculation

		if( cell->neighbours[1] )
		{
			result += one_minus_u*v*w*cell->neighbours[1]->velocityChange[componentIndex];
			resultWeighting += one_minus_u*v*w;
		}
		if( cell->neighbours[3] )
		{
			result += u*one_minus_v*w*cell->neighbours[3]->velocityChange[componentIndex];
			resultWeighting += u*one_minus_v*w;
		}
		if( cell->neighbours[9] )
		{
			result += one_minus_u*one_minus_v*w*cell->neighbours[9]->velocityChange[componentIndex];
			resultWeighting += one_minus_u*one_minus_v*w;
		}
		if( cell->neighbours[5] )
		{
			result += u*v*one_minus_w*cell->neighbours[5]->velocityChange[componentIndex];
			resultWeighting += u*v*one_minus_w;
		}
		if( cell->neighbours[6] )
		{
			result += one_minus_u*v*one_minus_w*cell->neighbours[6]->velocityChange[componentIndex];
			resultWeighting += one_minus_u*v*one_minus_w;
		}
		if( cell->neighbours[7] )
		{
			result += u*one_minus_v*one_minus_w*cell->neighbours[7]->velocityChange[componentIndex];
			resultWeighting += u*one_minus_v*one_minus_w;
		}
		if( cell->neighbours[8] )
		{
			result += one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->velocityChange[componentIndex];
			resultWeighting += one_minus_u*one_minus_v*one_minus_w;
		}

		// now adapt the result so that the overall weighting remains 1.0f even if we skipped some terms
		return result / resultWeighting;
	}

	return 0.0f;
}

//
// This function does look up the velocity within the simulationDomain
//
math::Vec3f FluidSimulator3d::getVelocity( float x, float y, float z )
{
	math::Vec3f velocity;

	velocity.x = getInterpolatedVelocityValue( x/cellSize, y/cellSize-0.5f, z/cellSize-0.5f, 0 );
	velocity.y = getInterpolatedVelocityValue( x/cellSize-0.5f, y/cellSize, z/cellSize-0.5f, 1 );
	velocity.z = getInterpolatedVelocityValue( x/cellSize-0.5f, y/cellSize-0.5f, z/cellSize, 2 );

	return velocity;
}
//
// This function does look up the velocityChange within the simulationDomain (used for FLIP)
//
math::Vec3f FluidSimulator3d::getVelocityChange( float x, float y, float z )
{
	math::Vec3f velocityChange;

	velocityChange.x = getInterpolatedChangeValue( x/cellSize, y/cellSize-0.5f, z/cellSize-0.5f, 0 );
	velocityChange.y = getInterpolatedChangeValue( x/cellSize-0.5f, y/cellSize, z/cellSize-0.5f, 1 );
	velocityChange.z = getInterpolatedChangeValue( x/cellSize-0.5f, y/cellSize-0.5f, z/cellSize, 2 );

	return velocityChange;
}

//
// This method returns the position of the given point at t - deltaTime
// (used for semi-lagrange style advection)
//
math::Vec3f FluidSimulator3d::traceParticle( const math::Vec3f &position, float deltaTime )
{
	// solve ODE with RK2
	math::Vec3f vel = getVelocity( position.x, position.y, position.z );
	vel = getVelocity( position.x-0.5f*deltaTime*vel.x, position.y-0.5f*deltaTime*vel.y, position.z-0.5f*deltaTime*vel.z );

	return math::Vec3f( position.x - deltaTime*vel.x, position.y - deltaTime*vel.y, position.z - deltaTime*vel.z );
}













//
// This is a little helper function which will look for all
// cells which might adress the given cell at the given coordinates.
// Then the interpolation-neighbour reference is stored in each depending
// cell and the flag of each is recomputed.
// In addition all interpolationneighbours of cell are checked whether they
// exist and are stored as references.
//
void FluidSimulator3d::updateNeighbours( Cell *cell, const Cell::Coordinate &c )
{
	// by adding a new cell to the grid, we have to look for all cells which
	// would reference us as a neighbour and add a reference to those which could
	// be found (did exist)
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator neighbourIterator;

	for( unsigned int neighbourIndex=0; neighbourIndex<26; ++neighbourIndex )
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
void FluidSimulator3d::updateNeighbours( const Cell::Coordinate &c )
{
	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator cellIterator;

	stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator neighbourIterator;

	for( unsigned int neighbourIndex=0; neighbourIndex<26; ++neighbourIndex )
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
// This method will be called when a new cell just has been created.
// Override it to get a hand on newly created cells.
//
void FluidSimulator3d::onCellCreated( const Cell::Coordinate *coord, Cell *newCell )
{
	// compute the cell center - we will need it for collision detection etc.
	newCell->center = math::Vec3f( coord->i*cellSize + cellSize*0.5f, coord->j*cellSize + cellSize*0.5f, coord->k*cellSize + cellSize*0.5f );
}




// -------------------- FluidSimulator3d::Cell ------------------------------------------



//
// constructor
//
FluidSimulator3d::Cell::Cell()
{
	// setup neighbourreferences
	for( unsigned int i=0; i<26; ++i )neighbours[i] = 0;
	allInterpolationNeighboursExist = false;

	// no fluid borders the cell by default
	xNeighboursFluid = yNeighboursFluid = zNeighboursFluid = false;

	// intialize other members
	layer = -1;
	velocity = math::Vec3f( 0.0f, 0.0f, 0.0f );
	
	// sand extension
	rigid        = false;

	// viscoelastic extension
	strainTensor =  math::Matrix44f::Zero();
	vonMisesEquivalentStress = 0.0f;


}

//
// Registers reference to the index-specified neighbour
//
void FluidSimulator3d::Cell::setNeighbour( unsigned int index, Cell *neighbour )
{
	neighbours[ index ] = neighbour;

	allInterpolationNeighboursExist = neighbours[1]&&neighbours[3]&&
										neighbours[9]&&neighbours[5]&&
										neighbours[6]&&neighbours[7]&&
										neighbours[8];
}

//
// Unregisters reference to the index-specified neighbour
//
void FluidSimulator3d::Cell::removeNeighbour( unsigned int index )
{
	neighbours[index] = 0;

	allInterpolationNeighboursExist = neighbours[1]&&neighbours[3]&&
										neighbours[9]&&neighbours[5]&&
										neighbours[6]&&neighbours[7]&&
										neighbours[8];
}

//
// retrieve the neighbour from given relative coordinates. ri, rj, rk each are -1, 0 or +1
//
FluidSimulator3d::Cell *FluidSimulator3d::Cell::getNeighbour( int ri, int rj, int rk )
{
	if( ri < 0 )
	{
		if( rj > 0 )
		{
			if( rk > 0 )
			{
				// ri < 0 && rj > 0 && rk > 0
				return neighbours[25];
			}else
				if( rk < 0 )
				{
					// ri < 0 && rj > 0 && rk < 0
					return neighbours[20];
				}else
				{
					// ri < 0 && rj > 0 && rk == 0
					return neighbours[21];
				}
		}else
			if( rj < 0 )
			{
				if( rk > 0 )
				{
					// ri < 0 && rj < 0 && rk > 0
					return neighbours[23];
				}else
					if( rk < 0 )
					{
						// ri < 0 && rj < 0 && rk < 0
						return neighbours[18];
					}else
					{
						// ri < 0 && rj < 0 && rk == 0
						return neighbours[22];
					}
			}else
			{
				if( rk > 0 )
				{
					// ri < 0 && rj == 0 && rk > 0
					return neighbours[24];
				}else
					if( rk < 0 )
					{
						// ri < 0 && rj == 0 && rk < 0
						return neighbours[19];
					}else
					{
						// ri < 0 && rj == 0 && rk == 0
						return neighbours[0];
					}
			}
	}else
		if( ri > 0 )
		{
			if( rj > 0 )
			{
				if( rk > 0 )
				{
					// ri > 0 && rj > 0 && rk > 0
					return neighbours[8];
				}else
					if( rk < 0 )
					{
						// ri > 0 && rj > 0 && rk < 0
						return neighbours[15];
					}else
					{
						// ri > 0 && rj > 0 && rk == 0
						return neighbours[9];
					}
			}else
				if( rj < 0 )
				{
					if( rk > 0 )
					{
						// ri > 0 && rj < 0 && rk > 0
						return neighbours[12];
					}else
						if( rk < 0 )
						{
							// ri > 0 && rj < 0 && rk < 0
							return neighbours[13];
						}else
						{
							// ri > 0 && rj < 0 && rk == 0
							return neighbours[10];
						}
				}else
				{
					if( rk > 0 )
					{
						// ri > 0 && rj == 0 && rk > 0
						return neighbours[6];
					}else
						if( rk < 0 )
						{
							// ri > 0 && rj == 0 && rk < 0
							return neighbours[14];
						}else
						{
							// ri > 0 && rj == 0 && rk == 0
							return neighbours[1];
						}
				}
		}else
			{
				if( rj > 0 )
				{
					if( rk > 0 )
					{
						// ri == 0 && rj > 0 && rk > 0
						return neighbours[7];
					}else
						if( rk < 0 )
						{
							// ri == 0 && rj > 0 && rk < 0
							return neighbours[16];
						}else
						{
							// ri == 0 && rj > 0 && rk == 0
							return neighbours[3];
						}
				}else
					if( rj < 0 )
					{
						if( rk > 0 )
						{
							// ri == 0 && rj < 0 && rk > 0
							return neighbours[11];
						}else
							if( rk < 0 )
							{
								// ri == 0 && rj < 0 && rk < 0
								return neighbours[17];
							}else
							{
								// ri == 0 && rj < 0 && rk == 0
								return neighbours[2];
							}
					}else
					{
						if( rk > 0 )
						{
							// ri == 0 && rj == 0 && rk > 0
							return neighbours[5];
						}else
							if( rk < 0 )
							{
								// ri == 0 && rj == 0 && rk < 0
								return neighbours[4];
							}else
							{
								// ri == 0 && rj == 0 && rk == 0
								return this;
							}
					}
			}
}


//
// returns true if the cell is of type fluid
//
bool FluidSimulator3d::Cell::isFluidCell()
{
	return (type == Fluid);
}
// -------------------- FluidSimulator3d::hash_compare ------------------------------------------

//
// hash function which returns an hashvalue for the given key
//
size_t FluidSimulator3d::hash_compare::operator() (const FluidSimulator3d::Cell::Coordinate& key) const
{
	// hash function proposed in [Worley 1996]
	return 541*key.i + 79*key.j + 31*key.k;
}

//
// comperator for internal sorting (dont ask me why hash_table is sorting at all)
//
bool FluidSimulator3d::hash_compare::operator() (const FluidSimulator3d::Cell::Coordinate& key1, const FluidSimulator3d::Cell::Coordinate& key2) const
{
	// lexicographical ordering
	if( key1.i==key2.i )
	{
		if( key1.j==key2.j )
		{
			return key1.k<key2.k;
		}else
			return key1.j<key2.j;
	}else
		return key1.i<key2.i;
}



// -------------------- FluidSimulator3d::Cell::Coordinate ------------------------------------------


//
// standard constructor
//
FluidSimulator3d::Cell::Coordinate::Coordinate()
{
}

//
// constructs the cell from given coordinates
//
FluidSimulator3d::Cell::Coordinate::Coordinate( int _i, int _j, int _k ) : i(_i), j(_j), k(_k)
{
}


//
// returns the Coordinate within the grid of the index-specified neighbour
//
FluidSimulator3d::Cell::Coordinate FluidSimulator3d::Cell::Coordinate::getNeighbourCoordinate( int neighbourIndex ) const
{
	switch( neighbourIndex )
	{
	case 0:
		return Coordinate( i-1, j, k );
	case 1:
		return Coordinate( i+1, j, k );
	case 2:
		return Coordinate( i, j-1, k );
	case 3:
		return Coordinate( i, j+1, k );
	case 4:
		return Coordinate( i, j, k-1 );
	case 5:
		return Coordinate( i, j, k+1 );
	case 6:
		return Coordinate( i+1, j, k+1 );
	case 7:
		return Coordinate( i, j+1, k+1 );
	case 8:
		return Coordinate( i+1, j+1, k+1 );
	case 9:
		return Coordinate( i+1, j+1, k );
	case 10:
		return Coordinate( i+1, j-1, k );
	case 11:
		return Coordinate( i, j-1, k+1 );
	case 12:
		return Coordinate( i+1, j-1, k+1 );
	case 13:
		return Coordinate( i+1, j-1, k-1 );
	case 14:
		return Coordinate( i+1, j, k-1 );
	case 15:
		return Coordinate( i+1, j+1, k-1 );
	case 16:
		return Coordinate( i, j+1, k-1 );
	case 17:
		return Coordinate( i, j-1, k-1 );
	case 18:
		return Coordinate( i-1, j-1, k-1 );
	case 19:
		return Coordinate( i-1, j, k-1 );
	case 20:
		return Coordinate( i-1, j+1, k-1 );
	case 21:
		return Coordinate( i-1, j+1, k );
	case 22:
		return Coordinate( i-1, j-1, k );
	case 23:
		return Coordinate( i-1, j-1, k+1);
	case 24:
		return Coordinate( i-1, j, k+1 );
	case 25:
		return Coordinate( i-1, j+1, k+1 );
	default:
		return Coordinate( i, j, k );
	};
}

//
// this method returns the coordinate of the cell which references this (current)cell-coordinate through the given neighbour-index
//
FluidSimulator3d::Cell::Coordinate FluidSimulator3d::Cell::Coordinate::getInverseNeighbourCoordinate( int neighbourIndex ) const
{
	// other cells will reference the cell at this coordinate. So for convinient updating of the gridcell inter-references
	// this method is used to find all coordinates of cells which could reference the current cell(coordinate)
	// So: Coordinate c; c.getNeighbourCoordinate( int ).getInverseNeighbourCoordinate( int ) returns a coordinate which is identical to c
	switch( neighbourIndex )
	{
	case 0:
		return Coordinate( i+1, j, k );
	case 1:
		return Coordinate( i-1, j, k );
	case 2:
		return Coordinate( i, j+1, k );
	case 3:
		return Coordinate( i, j-1, k );
	case 4:
		return Coordinate( i, j, k+1 );
	case 5:
		return Coordinate( i, j, k-1 );
	case 6:
		return Coordinate( i-1, j, k-1 );
	case 7:
		return Coordinate( i, j-1, k-1 );
	case 8:
		return Coordinate( i-1, j-1, k-1 );
	case 9:
		return Coordinate( i-1, j-1, k );
	case 10:
		return Coordinate( i-1, j+1, k );
	case 11:
		return Coordinate( i, j+1, k-1 );
	case 12:
		return Coordinate( i-1, j+1, k-1 );
	case 13:
		return Coordinate( i-1, j+1, k+1 );
	case 14:
		return Coordinate( i-1, j, k+1 );
	case 15:
		return Coordinate( i-1, j-1, k+1 );
	case 16:
		return Coordinate( i, j-1, k+1 );
	case 17:
		return Coordinate( i, j+1, k+1 );
	case 18:
		return Coordinate( i+1, j+1, k+1 );
	case 19:
		return Coordinate( i+1, j, k+1 );
	case 20:
		return Coordinate( i+1, j-1, k+1 );
	case 21:
		return Coordinate( i+1, j-1, k );
	case 22:
		return Coordinate( i+1, j+1, k );
	case 23:
		return Coordinate( i+1, j+1, k-1);
	case 24:
		return Coordinate( i+1, j, k-1 );
	case 25:
		return Coordinate( i+1, j-1, k-1 );
	default:
		return Coordinate( i, j, k );
	};
}



// -------------------- FluidSimulator2d::StaticSolid ------------------------------------------


void FluidSimulator3d::StaticSolid::computeTransformationMatrix()
{
	transform = math::Matrix44f::Identity();



	transform.translate( translation.x, translation.y, translation.z );
	transform.rotateZ( rotation.z );
	transform.rotateY( rotation.y );
	transform.rotateX( rotation.x );

	inverseTransform = transform;
	inverseTransform.invert();
}

math::Matrix44f FluidSimulator3d::StaticSolid::getTransformationMatrix()
{
	return transform;
}
math::Matrix44f FluidSimulator3d::StaticSolid::getInverseTransformationMatrix()
{
	return inverseTransform;

}
void FluidSimulator3d::StaticSolid::setTranslation( float x, float y, float z )
{
	translation = math::Vec3f( x, y, z );
	computeTransformationMatrix();
}
void FluidSimulator3d::StaticSolid::setRotation( float xAngle, float yAngle, float zAngle )
{
	rotation = math::Vec3f( math::degToRad(xAngle), math::degToRad(yAngle), math::degToRad(zAngle) );
	computeTransformationMatrix();
}
