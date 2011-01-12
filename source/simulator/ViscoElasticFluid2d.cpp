/*---------------------------------------------------------------------

This simulator derives from FluidSimulator2d and overloads the
updateVelocity method. There are some small additions which will
make the fluid behave like a viscoelastic fluid.
The idea and theory behind that method was introduced in the paper
"A Method for Animating Viscoelastic Fluids "
at Siggraph in 2004 by Goktekin

----------------------------------------------------------------------*/
#include "ViscoElasticFluid2d.h"
#include <algorithm>












//
// constructor
//
ViscoElasticFluid2d::ViscoElasticFluid2d()
{
	//elasticModulus    = 10.0f;
	//elasticYieldPoint = .5f;  // gamma
	//plasticYieldRate  = 25.0f;  // alpha -> elastic decay rate - determines rate of plastic flow
	//elasticModulus    = 10.0f;

	// good elastic behavior
	elasticModulus    = 20.0f;
	elasticYieldPoint = 0.1f;  // gamma
	plasticYieldRate  = 25.0f;  // alpha -> elastic decay rate - determines rate of plastic flow

	// 5 200 25

	particleBasedAdvection = false;
}



//
//
//
void ViscoElasticFluid2d::setElasticModulus( float elasticModulus )
{
	this->elasticModulus = elasticModulus;
}

//
// alpha -> elastic decay rate - determines rate of plastic flow
//
void ViscoElasticFluid2d::setPlasticYieldRate( float yieldRate )
{
	this->plasticYieldRate  = yieldRate;
}

//
// This method is a modified version of the FluidSimulator2d::updateVelocity
// method to reflect viscoelastic behavior
//
void ViscoElasticFluid2d::updateVelocity( float dt )
{
	//FluidSimulator2d::updateVelocity(dt);
	//return;

	// store the velocity in the oldVelocity member of each cell so that we later can compute the change in velocity --------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		(*it).second.oldVelocity = (*it).second.velocity;

	// update total strain for each fluid cell ------------------------------------------------------------------------------
	// calculate T and accumulate new strain in E
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(*it).second;

		if( cell->type != Cell::Fluid )
			continue;

		// first term T
		math::Matrix22f D = computeStrainRateTensor( cell );

		cell->strainTensor += dt*D;

		// second term -kyr*...
		float norm = math::frobeniusNorm( cell->strainTensor );
		math::Matrix22f plasticYielding = math::Matrix22f::Zero();
		if( norm )
			plasticYielding = plasticYieldRate*std::max<float>( 0, norm - elasticYieldPoint )*cell->strainTensor*(1.0f / norm);

		cell->strainTensor -= dt*plasticYielding;
	}

	// advect elastic strain tensor E -------------------------------------------------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		// clear temp variable for the intermediate result
		(*it).second.temp = math::Matrix22f::Zero();

	// now advect the tensor components for each fluid-cell
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(*it).second;

		float e11, e22, e12;


		e11 = cell->strainTensor.m[0][0];
		e22 = cell->strainTensor.m[1][1];
		e12 = cell->strainTensor.m[0][1];

		if(cell->type == Cell::Fluid)
		{
			// track strain tensor components which lie at the cell center -> E[1][1] && E[2][2]
			math::Vec2f centerPos = traceParticle( math::Vec2f( (*it).first.i * cellSize+(cellSize/2.0f), (*it).first.j * cellSize+(cellSize/2.0f) ), dt );
			e11 = getInterpolatedStrainTensorComponent( centerPos.x/cellSize - 0.5f, centerPos.y/cellSize - 0.5f, 0, 0 );
			e22 = getInterpolatedStrainTensorComponent( centerPos.x/cellSize - 0.5f, centerPos.y/cellSize - 0.5f, 1, 1 );
		}

		if( cell->xNeighboursFluid || cell->yNeighboursFluid )
		{
			// track strain tensor components which lie at the cell edges -> E[1][2]
			math::Vec2f offPos = traceParticle( math::Vec2f( (*it).first.i * cellSize, (*it).first.j * cellSize ), dt );
			e12 = getInterpolatedStrainTensorComponent( offPos.x/cellSize, offPos.y/cellSize, 0, 1 );
		}

		cell->temp.m[0][0] = e11;
		cell->temp.m[1][1] = e22;
		cell->temp.m[0][1] = e12;
		cell->temp.m[1][0] = e12;

	}

	// copy the intermediate results to the final values
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		if( (*it).second.type == Cell::Fluid )
			(*it).second.strainTensor = (*it).second.temp;
		else
			(*it).second.strainTensor = (*it).second.temp;
	}

	// extrapolate elastic strain into air
	extrapolateStrain();

	// advect velocities (if advection is not done through particles (PIC/FLIP) ) --------------------------------
	if( !particleBasedAdvection )
		// then we use a sem-lagrange method to advect velocities...
		advectGridVelocities( dt );

	// apply external forces ------------------------------------------------------------------------------------
	applyGravity( dt, 0.0f, -9.1f );


	/*
	// stretch test
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &it->second;

		// we have to advance velocity-components of all cells which border fluid cells
		if( cell->xNeighboursFluid )
		{
		if( cell->center.x > 0.6f )
		{
			cell->velocity.x += 10.0f*dt;
			//cell->velocity.y += 0.0f;
		}else
		if( cell->center.x < 0.4f )
		{
			cell->velocity.x += -10.0f*dt;
			//cell->velocity.y += 0.0f;
		}
		}
	}
	*/


	// apply elastic strain force to u -----------------------------------------------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(*it).second;

		cell->vonMisesEquivalentStress = 0.0f;

		if( cell->xNeighboursFluid || cell->yNeighboursFluid )
		{
			math::Vec2f f = elasticModulus*computeDivE( cell );

			if( cell->xNeighboursFluid )
				cell->velocity.x += dt*f.x;
			if( cell->yNeighboursFluid )
				cell->velocity.y += dt*f.y;

			// compute equivalent stress
			cell->stressTensor = -elasticModulus*cell->strainTensor;
			cell->vonMisesEquivalentStress = sqrt( 3.0f/2.0f ) * math::frobeniusNorm( cell->stressTensor );
		}
	}



	// apply viscosity ----------------------------------------------------
	solveViscosity( dt );




	// extrapolate velocities into surrounding buffer cells ---------------
	extrapolateVelocities();
	// set Boundary conditions --------------------------------------------
	setBoundaryConditions();


	// solve for pressure and make the velocity grid divergence free ------
	solvePressure( dt );

	// extrapolate velocity into buffer cells second time -----------------
	extrapolateVelocities();
	// set solid cell velocities ------------------------------------------
	setBoundaryConditions();

	

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
//
//
float ViscoElasticFluid2d::getInterpolatedStrainTensorComponent( float x, float y, int row, int column )
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
		return u*v*cell->strainTensor.m[row][column] + one_minus_u*v*cell->neighbours[1]->strainTensor.m[row][column] +
			u*one_minus_v*cell->neighbours[3]->strainTensor.m[row][column] + one_minus_u*one_minus_v*cell->neighbours[4]->strainTensor.m[row][column];
	}else
	{
		// we have to check each neighbour cell individiually and leave it out in our calculation
		float result = u*v*cell->strainTensor.m[row][column];  // this float will hold the sum of all values which entered the calculation
		float resultWeighting = u*v;                        // this variable will hold the sum of all weights which entered the calculation

		if( cell->neighbours[1] )
		{
			result += one_minus_u*v*cell->neighbours[1]->strainTensor.m[row][column];
			resultWeighting += one_minus_u*v;
		}
		if( cell->neighbours[3] )
		{
			result += u*one_minus_v*cell->neighbours[3]->strainTensor.m[row][column];
			resultWeighting += u*one_minus_v;
		}
		if( cell->neighbours[4] )
		{
			result += one_minus_u*one_minus_v*cell->neighbours[4]->strainTensor.m[row][column];
			resultWeighting += one_minus_u*one_minus_v;
		}

		// now adapt the result so that the overall weighting remains 1.0f even if we skipped some terms
		return result / resultWeighting;
	}

	return 0.0f;
}




//
// this will compute the rate of strain tensor D for the given cell
//
math::Matrix22f ViscoElasticFluid2d::computeStrainRateTensor( Cell *cell )
{
	math::Matrix22f D = math::Matrix22f::Zero();


	D.m[0][0] = cell->neighbours[1]->velocity.x - cell->velocity.x;
	D.m[0][1] = cell->velocity.y - cell->neighbours[0]->velocity.y;
	D.m[1][0] = cell->velocity.x - cell->neighbours[2]->velocity.x;
	D.m[1][1] = cell->neighbours[3]->velocity.y - cell->velocity.y;

	math::Matrix22f Dtransposed = D;
	Dtransposed.transpose();

	D = (D + Dtransposed)*0.5f;

	return D*(1.0f/cellSize);
}



math::Vec2f ViscoElasticFluid2d::computeDivE(FluidSimulator2d::Cell *cell)
{
	// compute divergence of the strain tensor which is stored at the cell
	// Tensor E ist a matrix which consists of 3 column vectors c1, c2 ,c3
	// the divergence of tensor E is the vector of the gradients of these
	// column vectors divE = (divc1, divc2, divc3)
	float divC1 = (cell->strainTensor.m[0][0] - cell->neighbours[0]->strainTensor.m[0][0])+ // backward difference (diagonal element)
				  (cell->neighbours[3]->strainTensor.m[1][0] - cell->strainTensor.m[1][0]); // forward difference (off-diagonal element)
	float divC2 = (cell->neighbours[1]->strainTensor.m[0][1] - cell->strainTensor.m[0][1])+
				  (cell->strainTensor.m[1][1] - cell->neighbours[2]->strainTensor.m[1][1]);

	return math::Vec2f( divC1/cellSize, divC2/cellSize );
}









void ViscoElasticFluid2d::extrapolateStrain( void )
{
	// iterate all gridcells and set the layer to 0 for every fluid cell and -1 to any other -------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(*it).second;
		if( cell->type == Cell::Fluid )
			(*it).second.layer2 = 0;
		else
			(*it).second.layer2 = -1;
	}

	// now get the layer for each cell
	for( unsigned int i=1; i<5; ++i )
	{
		// iterate all gridcells and set the layer to 0 for every fluid cell and -1 to any other -------------------------------
		for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		{
			Cell *cell = &(*it).second;
			
			// if the current cell does not have a layer assigned yet
			if( cell->layer2 == -1 )
			{
				// look into each neighbour and check if one has layer i-1
				for( size_t n = 0; n<4; ++n )
					if( cell->neighbours[n] && (cell->neighbours[n]->layer2 == i-1) )
					{
						cell->layer2 = i;
						break;
					}
			}
		}
	}

	// now do some simple extrapolation of the strain tensors
	for( unsigned int i=1; i<5; ++i )
	{
		// iterate all gridcells and set the layer to 0 for every fluid cell and -1 to any other -------------------------------
		for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		{
			Cell *cell = &(*it).second;

			// if the current cell does not have a layer assigned yet
			if( cell->layer2 == i )
			{
				math::Matrix22f temp = math::Matrix22f::Zero();

				cell->strainTensor = temp;

				/*
				cell->strainTensor.m[0][0] = 0;
				cell->strainTensor.m[1][1] = 0;

				if( !(cell->xNeighboursFluid || cell->yNeighboursFluid) )
					cell->strainTensor.m[0][1] = cell->strainTensor.m[1][0] = 0;
					*/

				/*
				// compute the strain tensor as an average of all strain tensors of all neighbours
				// with layer2 == i-1
				int count = 0;

				// look into each neighbour and check if one has layer i-1
				for( size_t n = 0; n<4; ++n )
					if( cell->neighbours[n] && (cell->neighbours[n]->layer2 == i-1) )
					{
						temp += cell->neighbours[n]->strainTensor;
						++count;
					}

				// weight
				if( count )
					temp /= float (count);

				cell->strainTensor.m[0][0] = temp.m[0][0];
				cell->strainTensor.m[1][1] = temp.m[1][1];

				// tensor elements that border fluid cells are left unchanged
				if( !(cell->xNeighboursFluid || cell->yNeighboursFluid) )
					cell->strainTensor.m[0][1] = cell->strainTensor.m[1][0] = temp.m[0][1];
				*/
					
			}
		}
	}



}















/*
//
//
//
void ViscoElasticFluid2d::sweepStrain( int iStart, int iEnd, int jStart, int jEnd )
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


			// points to be aware of:
			// we use central differences which is problematic in some cases
			// we may have to treat the off-diagonal elements of the strain tensor
			// differently since they might border fluid cells



			Cell *temp = 0;

			// distances (center, left, right, top, bottom, bottomleft)
			float p_c, p_r,p_l, p_t, p_b, p_bl;

			float e00_1, e00_2;
			float e11_1, e11_2;
			float e01_1, e01_2;

			e00_1 = e00_2 = e11_1 = e11_2 = e01_1 = e01_2 = 0.0f;


			p_c = p_r = p_l = p_t = p_b = p_bl = 100.0f;

			p_c = cell->signedDistance;

			if( cell->neighbours[1] )
				p_r = cell->neighbours[1]->signedDistance;
			if( cell->neighbours[0] )
				p_l = cell->neighbours[0]->signedDistance;
			if( cell->neighbours[3] )
				p_t = cell->neighbours[3]->signedDistance;
			if( cell->neighbours[2] )
				p_b = cell->neighbours[2]->signedDistance;
			if( cell->neighbours[6] )
				p_bl = cell->neighbours[6]->signedDistance;

			if( cell->neighbours[horizontalNeighbourIndex] )
			{
				e00_1 = cell->neighbours[horizontalNeighbourIndex]->strainTensor.m[0][0];
				e11_1 = cell->neighbours[horizontalNeighbourIndex]->strainTensor.m[1][1];
				e01_1 = cell->neighbours[horizontalNeighbourIndex]->strainTensor.m[0][1];
			}
			if( cell->neighbours[verticalNeighbourIndex] )
			{
				e00_2 = cell->neighbours[verticalNeighbourIndex]->strainTensor.m[0][0];
				e11_2 = cell->neighbours[verticalNeighbourIndex]->strainTensor.m[1][1];
				e01_2 = cell->neighbours[verticalNeighbourIndex]->strainTensor.m[0][1];
			}





			// compute tensor values which lie at the cell center (0,0 + 1,1)---------------
			if( cell->type != Cell::Fluid )
			{
				// compute the horizontal change in distance (using central differences)
				float deltaPhiX = di * (p_r - p_l)/2.0f;

				// skip current cycle if the horizontal distance to the fluid gets smaller
				if( deltaPhiX >= 0 )
				{
					// compute the vertical change of distance
					float deltaPhiY = dj*(p_t - p_b)/2.0f;

					// skip current cycle if the vertical distance to the fluid gets smaller
					if( deltaPhiY >= 0 )
					{
						float alpha;

						if( deltaPhiX + deltaPhiY == 0.0f )
							alpha = 0.5f;
						else
							alpha = deltaPhiX/(deltaPhiX + deltaPhiY);
						
						cell->strainTensor.m[0][0] = alpha*e00_1 + (1.0f-alpha)*e00_2;
						cell->strainTensor.m[1][1] = alpha*e11_1 + (1.0f-alpha)*e11_2;
					}
				}
			}

			// compute tensor values which lie at the cell edges (0,1 + 1,0)---------------
			if( cell->type != Cell::Fluid )
			{
				// compute the horizontal change in distance - keep in  mind: the off-diagonal tensor values lie on the corners of the cell
				float deltaPhiX = di * 0.5f*(p_c - p_l + p_b - p_bl);

				// skip current cycle if the horizonal distance to the fluid gets smaller
				if( deltaPhiX >= 0 )
				{
					// compute the vertical change of distance
					float deltaPhiY = dj * 0.5f*( p_c - p_b + p_l - p_bl );

					// skip current cycle if the vertical distance to the fluid gets smaller
					if( deltaPhiY >= 0 )
					{
						float alpha;

						if( deltaPhiX + deltaPhiY == 0.0f )
							alpha = 0.5f;
						else
							alpha = deltaPhiX/(deltaPhiX + deltaPhiY);
						
						cell->strainTensor.m[0][1] = alpha*e01_1 + (1.0f-alpha)*e01_2;
						cell->strainTensor.m[1][0] = cell->strainTensor.m[0][1];
					}
				}
			}
		}
}


//
// extrapolate strain from fluid into air cells
//
void ViscoElasticFluid2d::extrapolateStrain( void )
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
		sweepStrain( iMin, iMax+1, jMax, jMin-1 );  // Quadrant IV
		sweepStrain( iMax, iMin-1, jMin, jMax+1 );  // Quadrant II
		sweepStrain( iMax, iMin-1, jMax, jMin-1 );  // Quadrant III
		sweepStrain( iMin, iMax+1, jMin, jMax+1 );  // Quadrant I
	}
}
*/