/*---------------------------------------------------------------------

This simulator derives from FluidSimulator2d and overloads the
updateVelocity method. There are some small additions which will
make the fluid behave like a viscoelastic fluid.
The idea and theory behind that method was introduced in the paper
"A Method for Animating Viscoelastic Fluids "
at Siggraph in 2004 by Goktekin

----------------------------------------------------------------------*/
#include "ViscoElasticFluid3d.h"
#include <algorithm>












//
// constructor
//
ViscoElasticFluid3d::ViscoElasticFluid3d()
{
	//elasticModulus    = 10.0f;
	//elasticYieldPoint = .5f;  // gamma
	//plasticYieldRate  = 25.0f;  // alpha -> elastic decay rate - determines rate of plastic flow
	elasticModulus    = 12.0f;
	elasticYieldPoint = 0.25f;  // gamma
	plasticYieldRate  = 25.0f;  // alpha -> elastic decay rate - determines rate of plastic flow

	// 5 200 25

	particleBasedAdvection = false;
}


//
//
//
void ViscoElasticFluid3d::setElasticModulus( float elasticModulus )
{
	this->elasticModulus = elasticModulus;
}

//
//
//
float ViscoElasticFluid3d::getElasticModulus()
{
	return elasticModulus;
}

//
// alpha -> elastic decay rate - determines rate of plastic flow
//
void ViscoElasticFluid3d::setPlasticYieldRate( float yieldRate )
{
	this->plasticYieldRate  = yieldRate;
}



//
// This method is a modified version of the FluidSimulator2d::updateVelocity
// method to reflect viscoelastic behavior
//
void ViscoElasticFluid3d::updateVelocity( float dt )
{
	//FluidSimulator3d::updateVelocity(dt);
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
		math::Matrix44f D = computeStrainRateTensor( cell );

		cell->strainTensor += dt*D;

		// second term -kyr*... (plastic yielding)
		float norm = math::frobeniusNorm( cell->strainTensor );
		math::Matrix44f plasticYielding = math::Matrix44f::Zero();
		if( norm )
			plasticYielding = plasticYieldRate*std::max<float>( 0, norm - elasticYieldPoint )*cell->strainTensor*(1.0f / norm);

		cell->strainTensor -= dt*plasticYielding;
	}

	// advect elastic strain tensor E -------------------------------------------------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
		// clear temp variable for the intermediate result
		(*it).second.temp = math::Matrix44f::Zero();

	// now advect the tensor components for each fluid-cell
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(*it).second;

		float e11, e22, e33, e12, e13, e23; // advected strain tensor components

		e11 = cell->strainTensor.m[0][0];
		e22 = cell->strainTensor.m[1][1];
		e33 = cell->strainTensor.m[2][2];

		if( cell->type == Cell::Fluid )
		{
			// track strain tensor components which lie at the cell center -> E[1][1] && E[2][2]
			math::Vec3f centerPos = traceParticle( math::Vec3f( (*it).first.i * cellSize+(cellSize/2.0f), (*it).first.j * cellSize+(cellSize/2.0f), (*it).first.k * cellSize+(cellSize/2.0f) ), dt );
			e11 = getInterpolatedStrainTensorComponent( centerPos.x/cellSize - 0.5f, centerPos.y/cellSize - 0.5f, centerPos.z/cellSize - 0.5f, 0, 0 );
			e22 = getInterpolatedStrainTensorComponent( centerPos.x/cellSize - 0.5f, centerPos.y/cellSize - 0.5f, centerPos.z/cellSize - 0.5f, 1, 1 );
			e33 = getInterpolatedStrainTensorComponent( centerPos.x/cellSize - 0.5f, centerPos.y/cellSize - 0.5f, centerPos.z/cellSize - 0.5f, 2, 2 );

			cell->temp.m[0][0] = e11;
			cell->temp.m[1][1] = e22;
			cell->temp.m[2][2] = e33;
		}

		// track strain tensor components which lie at the cell edges
		math::Vec3f offPos;

		e12 = cell->strainTensor.m[0][1];
		e13 = cell->strainTensor.m[0][2];
		e23 = cell->strainTensor.m[1][2];

		if( cell->xNeighboursFluid || cell->yNeighboursFluid )
		{
			//-> E[1][2]
			offPos = traceParticle( math::Vec3f( (*it).first.i * cellSize, (*it).first.j * cellSize, (*it).first.k * cellSize + (cellSize/2.0f) ), dt );
			e12 = getInterpolatedStrainTensorComponent( offPos.x/cellSize, offPos.y/cellSize, offPos.z/cellSize - 0.5f, 0, 1 );
		}
		if( cell->xNeighboursFluid || cell->zNeighboursFluid )
		{
			//-> E[1][3]
			offPos = traceParticle( math::Vec3f( (*it).first.i * cellSize, (*it).first.j * cellSize + (cellSize/2.0f), (*it).first.k * cellSize ), dt );
			e13 = getInterpolatedStrainTensorComponent( offPos.x/cellSize, offPos.y/cellSize - 0.5f, offPos.z/cellSize, 0, 2 );
		}

		if( cell->zNeighboursFluid || cell->yNeighboursFluid )
		{
			//-> E[2][3]
			offPos = traceParticle( math::Vec3f( (*it).first.i * cellSize + (cellSize/2.0f), (*it).first.j * cellSize, (*it).first.k * cellSize ), dt );
			e23 = getInterpolatedStrainTensorComponent( offPos.x/cellSize - 0.5f, offPos.y/cellSize, offPos.z/cellSize, 1, 2 );
		}

		cell->temp.m[0][1] = e12;
		cell->temp.m[0][2] = e13;
		cell->temp.m[1][2] = e23;

		// the tensor is symmetric...
		cell->temp.m[1][0] = e12;
		cell->temp.m[2][0] = e13;
		cell->temp.m[2][1] = e23;
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
	applyGravity( dt, 0.0f, -9.1f, 0.0f );


	// apply elastic strain force to u -----------------------------------------------------------------------------
	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(*it).second;

		cell->vonMisesEquivalentStress = 0.0f;

		if( cell->xNeighboursFluid || cell->yNeighboursFluid || cell->zNeighboursFluid )
		{
			math::Vec3f f = elasticModulus*computeDivE( cell );

			if( cell->xNeighboursFluid )
				cell->velocity.x += dt*f.x;
			if( cell->yNeighboursFluid )
				cell->velocity.y += dt*f.y;
			if( cell->zNeighboursFluid )
				cell->velocity.z += dt*f.z;

			// compute equivalent stress
			math::Matrix44f temp = -elasticModulus*cell->strainTensor;
			cell->vonMisesEquivalentStress = sqrt( 3.0f/2.0f ) * math::frobeniusNorm( temp );
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
// TODO: current implementation might be a problem since all components are not stored at the same position
//
float ViscoElasticFluid3d::getInterpolatedStrainTensorComponent( float x, float y, float z, int row, int column )
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

	// TODO: this may be wrong since the tensor components are stored at different locations
	// does every neighbour of the current cell exist?
	if( cell->allInterpolationNeighboursExist )
	{
		// then calculate the full trilinear interpolation
		return u*v*w*cell->strainTensor.m[row][column] + one_minus_u*v*w*cell->neighbours[1]->strainTensor.m[row][column] +
			u*one_minus_v*w*cell->neighbours[3]->strainTensor.m[row][column] + one_minus_u*one_minus_v*w*cell->neighbours[9]->strainTensor.m[row][column] +
			u*v*one_minus_w*cell->neighbours[5]->strainTensor.m[row][column] + one_minus_u*v*one_minus_w*cell->neighbours[6]->strainTensor.m[row][column] +
			u*one_minus_v*one_minus_w*cell->neighbours[7]->strainTensor.m[row][column] + one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->strainTensor.m[row][column];
	}else
	{
		// we have to check each neighbour cell individiually and leave it out in our calculation
		float result = u*v*w*cell->strainTensor.m[row][column];  // this float will hold the sum of all values which entered the calculation
		float resultWeighting = u*v*w;                        // this variable will hold the sum of all weights which entered the calculation

		if( cell->neighbours[1] )
		{
			result += one_minus_u*v*w*cell->neighbours[1]->strainTensor.m[row][column];
			resultWeighting += one_minus_u*v*w;
		}
		if( cell->neighbours[3] )
		{
			result += u*one_minus_v*w*cell->neighbours[3]->strainTensor.m[row][column];
			resultWeighting += u*one_minus_v*w;
		}
		if( cell->neighbours[9] )
		{
			result += one_minus_u*one_minus_v*w*cell->neighbours[9]->strainTensor.m[row][column];
			resultWeighting += one_minus_u*one_minus_v*w;
		}
		if( cell->neighbours[5] )
		{
			result += u*v*one_minus_w*cell->neighbours[5]->strainTensor.m[row][column];
			resultWeighting += u*v*one_minus_w;
		}
		if( cell->neighbours[6] )
		{
			result += one_minus_u*v*one_minus_w*cell->neighbours[6]->strainTensor.m[row][column];
			resultWeighting += one_minus_u*v*one_minus_w;
		}
		if( cell->neighbours[7] )
		{
			result += u*one_minus_v*one_minus_w*cell->neighbours[7]->strainTensor.m[row][column];
			resultWeighting += u*one_minus_v*one_minus_w;
		}
		if( cell->neighbours[8] )
		{
			result += one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->strainTensor.m[row][column];
			resultWeighting += one_minus_u*one_minus_v*one_minus_w;
		}

		// now adapt the result so that the overall weighting remains 1.0f even if we skipped some terms
		return result / resultWeighting;
	}

	return 0.0f;
}




//
// this will compute the rate of strain tensor D for the given cell
//
math::Matrix44f ViscoElasticFluid3d::computeStrainRateTensor( Cell *cell )
{
	math::Matrix44f D = math::Matrix44f::Zero();


	D.m[0][0] = cell->neighbours[1]->velocity.x - cell->velocity.x;
	D.m[0][1] = cell->velocity.y - cell->neighbours[0]->velocity.y;
	D.m[0][2] = cell->velocity.z - cell->neighbours[0]->velocity.z;
	D.m[1][0] = cell->velocity.x - cell->neighbours[2]->velocity.x;
	D.m[1][1] = cell->neighbours[3]->velocity.y - cell->velocity.y;
	D.m[1][2] = cell->velocity.z - cell->neighbours[2]->velocity.z;
	D.m[2][0] = cell->velocity.x - cell->neighbours[4]->velocity.x;
	D.m[2][1] = cell->velocity.y - cell->neighbours[4]->velocity.y;
	D.m[2][2] = cell->neighbours[5]->velocity.z - cell->velocity.z;

	math::Matrix44f Dtransposed = D;
	Dtransposed.transpose();

	D = (D + Dtransposed)*0.5f;

	return D*(1.0f/cellSize);
}



math::Vec3f ViscoElasticFluid3d::computeDivE(FluidSimulator3d::Cell *cell)
{
	// compute divergence of the strain tensor which is stored at the cell
	// Tensor E ist a matrix which consists of 3 column vectors c1, c2 ,c3
	// the divergence of tensor E is the vector of the gradients of these
	// column vectors divE = (divc1, divc2, divc3)
	float divC1 = (cell->strainTensor.m[0][0] - cell->neighbours[0]->strainTensor.m[0][0])+ // backward difference (diagonal element)
				  (cell->neighbours[3]->strainTensor.m[1][0] - cell->strainTensor.m[1][0])+ // forward difference (off-diagonal element)
				  (cell->neighbours[5]->strainTensor.m[2][0] - cell->strainTensor.m[2][0]); // forward difference (off-diagonal element)
	float divC2 = (cell->neighbours[1]->strainTensor.m[0][1] - cell->strainTensor.m[0][1])+
				  (cell->strainTensor.m[1][1] - cell->neighbours[2]->strainTensor.m[1][1])+
				  (cell->neighbours[5]->strainTensor.m[2][1] - cell->strainTensor.m[2][1]);
	float divC3 = (cell->neighbours[1]->strainTensor.m[0][2] - cell->strainTensor.m[0][2])+
				  (cell->neighbours[3]->strainTensor.m[1][2] - cell->strainTensor.m[1][2])+
				  (cell->strainTensor.m[2][2] - cell->neighbours[4]->strainTensor.m[2][2]);

	return math::Vec3f( divC1/cellSize, divC2/cellSize, divC3/cellSize );
}





//
// extrapolate strain from fluid into air cells
//
void ViscoElasticFluid3d::extrapolateStrain( void )
{

	for( stdext::hash_map<Cell::Coordinate, Cell, hash_compare>::iterator it = gridCells.begin(); it != gridCells.end(); ++it )
	{
		Cell *cell = &(*it).second;

		if( cell->type != Cell::Fluid )
		{
			cell->strainTensor.m[0][0] = 0;
			cell->strainTensor.m[1][1] = 0;
			cell->strainTensor.m[2][2] = 0;

			// z axis
			if( !(cell->xNeighboursFluid || cell->yNeighboursFluid) )
				cell->strainTensor.m[0][1] = cell->strainTensor.m[1][0] = 0;

			// y axis
			if( !(cell->xNeighboursFluid || cell->zNeighboursFluid) )
				cell->strainTensor.m[0][2] = cell->strainTensor.m[2][0] = 0;

			// x axis
			if( !(cell->yNeighboursFluid || cell->zNeighboursFluid) )
				cell->strainTensor.m[1][2] = cell->strainTensor.m[2][1] = 0;

		}

		//if(cell->type != Cell::Fluid )
		//	cell->strainTensor = math::Matrix44f::Zero();
	}

}