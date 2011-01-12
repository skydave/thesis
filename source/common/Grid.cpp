/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "Grid.h"


//
// constructor
//
Grid::Grid()
{
	cellSize = 1.0f;
	resolutionX = 0;
	resolutionY = 0;
	resolutionZ = 0;
}


//
// (re)initializes the grid
//
void Grid::setup( float _cellSize, size_t x, size_t y, size_t z )
{
	cellSize = _cellSize;
	resolutionX = x;
	resolutionY = y;
	resolutionZ = z;
	size_t cellCount = resolutionX*resolutionY*resolutionZ;

	// setup grid memory
	grid.resize( cellCount );

	// setup each individual cell
	for( size_t k=0; k<resolutionZ; ++k )
		for( size_t j=0; j<resolutionY; ++j )
			for( size_t i=0; i<resolutionX; ++i )
			{
				// compute the index to the cell
				size_t index = GRIDINDEX(i,j,k);

				// initialize values
				grid[index].center = math::Vec3f( i*cellSize + cellSize*0.5f, j*cellSize + cellSize*0.5f, k*cellSize + cellSize*0.5f );

				// setup references to neighbours
				grid[index].neighbours[0] = getCell( i-1, j, k );
				grid[index].neighbours[1] = getCell( i+1, j, k );
				grid[index].neighbours[2] = getCell( i, j-1, k );
				grid[index].neighbours[3] = getCell( i, j+1, k );
				grid[index].neighbours[4] = getCell( i, j, k-1 );
				grid[index].neighbours[5] = getCell( i, j, k+1 );
				grid[index].neighbours[6] = getCell( i+1, j, k+1 );
				grid[index].neighbours[7] = getCell( i, j+1, k+1 );
				grid[index].neighbours[8] = getCell( i+1, j+1, k+1 );
				grid[index].neighbours[9] = getCell( i+1, j+1, k );

				grid[index].allInterpolationNeighboursExist = (grid[index].neighbours[0] &&
																grid[index].neighbours[1] &&
																grid[index].neighbours[2] &&
																grid[index].neighbours[3] &&
																grid[index].neighbours[4] &&
																grid[index].neighbours[5] &&
																grid[index].neighbours[6] &&
																grid[index].neighbours[7] &&
																grid[index].neighbours[8] &&
																grid[index].neighbours[9] );
			}
}


//
// reldirects to a call to setup( 1.0f, 0, 0, 0 );
//
void Grid::clear()
{
	setup( 1.0f, 0, 0, 0 );
}

//
// returns reference to a kartesian coordinate specified cell
//
Grid::Cell *Grid::getCell( size_t i, size_t j, size_t k )
{
	// check wether the i,j,k coordinate points into the grid
	if( (i < 0)||(i >= resolutionX)||(j < 0)||(j >= resolutionY)||(k < 0)||(k >= resolutionZ) )
		return 0;

	// return cell
	return &grid[ GRIDINDEX(i,j,k) ];
}

//
// returns reference to a index specified cell
//
Grid::Cell *Grid::getCell( size_t index )
{
	// return cell
	return &grid[ index ];
}

//
// Returns the trilinear interpolated value from grid at the specified location
//
float Grid::getValueAt( float x, float y, float z, size_t valueIndex )
{
	// normalize the coordinate -> convert it into local gridspace where the cellsize is 1.0f
	return getValueAtNormalized( x / cellSize, y / cellSize, z / cellSize, valueIndex );
}

//
// Returns the trilinear interpolated identifier-specified value
//
float Grid::getValueAt( float x, float y, float z, std::string valueIdentifier )
{
	// find the valueIndex from identifier -----------------------

	// we assume that all cells have the same value layout so we can take the valueindex from any
	// cells and it is valid for all other cells too

	// if there is no cell
	if( grid.empty() )
		// then we can skip everything
		return 0.0f;

	// we have at least one cell - this cell will be used to find the valueIndex from the valueidentifier
	size_t valueIndex = grid.front().getValueIndex( valueIdentifier );

	// redirect to getGridValueAt with valueindex
	return getValueAt( x, y, z, valueIndex );
}


//
// Returns the trilinear interpolated value from grid at the specified location
//
// IMPORTANT: the values x, y and z are given in normalized space (they are mapped into a range where
// the cellSize is 1.0f by normalized_x = x/cellSize)
//
float Grid::getValueAtNormalized( float x, float y, float z, size_t valueIndex )
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
	Cell *cell = getCell( i, j, k );

	// if the cell could not be found
	if( !cell )
		// quit the function
		return 0.0f;


	// does every neighbour of the current cell exist?
	if( cell->allInterpolationNeighboursExist )
	{
		// then calculate the full trilinear interpolation
		return u*v*w*cell->values[valueIndex] + one_minus_u*v*w*cell->neighbours[1]->values[valueIndex] +
			u*one_minus_v*w*cell->neighbours[3]->values[valueIndex] + one_minus_u*one_minus_v*w*cell->neighbours[9]->values[valueIndex] +
			u*v*one_minus_w*cell->neighbours[5]->values[valueIndex] + one_minus_u*v*one_minus_w*cell->neighbours[6]->values[valueIndex] +
			u*one_minus_v*one_minus_w*cell->neighbours[7]->values[valueIndex] + one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->values[valueIndex];
	}else
	{
		// we have to check each neighbour cell individiually and leave it out in our calculation
		float result = u*v*w*cell->values[valueIndex];  // this float will hold the sum of all values which entered the calculation
		float resultWeighting = u*v*w;                        // this variable will hold the sum of all weights which entered the calculation

		if( cell->neighbours[1] )
		{
			result += one_minus_u*v*w*cell->neighbours[1]->values[valueIndex];
			resultWeighting += one_minus_u*v*w;
		}
		if( cell->neighbours[3] )
		{
			result += u*one_minus_v*w*cell->neighbours[3]->values[valueIndex];
			resultWeighting += u*one_minus_v*w;
		}
		if( cell->neighbours[9] )
		{
			result += one_minus_u*one_minus_v*w*cell->neighbours[9]->values[valueIndex];
			resultWeighting += one_minus_u*one_minus_v*w;
		}
		if( cell->neighbours[5] )
		{
			result += u*v*one_minus_w*cell->neighbours[5]->values[valueIndex];
			resultWeighting += u*v*one_minus_w;
		}
		if( cell->neighbours[6] )
		{
			result += one_minus_u*v*one_minus_w*cell->neighbours[6]->values[valueIndex];
			resultWeighting += one_minus_u*v*one_minus_w;
		}
		if( cell->neighbours[7] )
		{
			result += u*one_minus_v*one_minus_w*cell->neighbours[7]->values[valueIndex];
			resultWeighting += u*one_minus_v*one_minus_w;
		}
		if( cell->neighbours[8] )
		{
			result += one_minus_u*one_minus_v*one_minus_w*cell->neighbours[8]->values[valueIndex];
			resultWeighting += one_minus_u*one_minus_v*one_minus_w;
		}

		// now adapt the result so that the overall weighting remains 1.0f even if we skipped some terms
		return result / resultWeighting;
	}

	return 0.0f;
}




//
// returns the number of cells in x direction
//
size_t Grid::getResolutionX()
{
	return resolutionX;
}

//
// returns the number of cells in y direction
//
size_t Grid::getResolutionY()
{
	return resolutionY;
}

//
// returns the number of cells in z direction
//
size_t Grid::getResolutionZ()
{
	return resolutionZ;
}

//
// returns unform size of one cel
//
float Grid::getCellSize()
{
	return cellSize;
}

//
// returns the total number of cells
//
size_t Grid::getCellCount()
{
	return grid.size();
}

//
// Grid::Cell -------------------------------------------------------------------------
//

//
// constructor
//
Grid::Cell::Cell()
{
	// initialize neighbour references with 0
	memset( neighbours, 0, sizeof( Cell* ) * 10 );
}

//
// adds a float value to the value list of the cell
//
void Grid::Cell::addValue( float initValue )
{
	values.push_back( initValue );
}

//
// adds a float value to the value list of the cell and associated it with a string identifier
//
void Grid::Cell::addValue( std::string identifier, float initValue )
{
	valueIdentifiers[identifier] = values.size();
	addValue( initValue );
}

//
// returns a reference to the index specified value within the value list
//
float &Grid::Cell::value( size_t valueIndex )
{
	return values[valueIndex];
}

//
// returns a reference to the identifier specified value within the value list
//
float &Grid::Cell::value( std::string valueIdentifier )
{
	return values[ valueIdentifiers[valueIdentifier] ];
}

//
// returns the index of the identifier-specified value into the value-vector
//
size_t Grid::Cell::getValueIndex( std::string valueIdentifier )
{
	return valueIdentifiers[valueIdentifier];
}
