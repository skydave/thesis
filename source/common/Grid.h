/*---------------------------------------------------------------------

The grid class encapsulates a simple 3 dimensional grid and utility
methods like trilinear interpolation etc.

----------------------------------------------------------------------*/
#pragma once
#include <string>
#include <vector>
#include <map>
#include "math/Math.h"


#define GRIDINDEX( i, j, k )(k*resolutionX*resolutionY + j*resolutionX + i)




///
/// \brief axis aligned 3dimensional grid in space
///
class Grid
{
public:
	///
	/// \brief The grid consists of gridcells
	///
	struct Cell
	{
		Cell();                                           // constructor

		void                                        addValue( float initValue ); // adds a float value to the value list of the cell
		void                addValue( std::string identifier, float initValue ); // adds a float value to the value list of the cell and associated it with a string identifier
		float                                       &value( size_t valueIndex ); // returns a reference to the index specified value within the value list
		float                             &value( std::string valueIdentifier ); // returns a reference to the identifier specified value within the value list
		size_t                     getValueIndex( std::string valueIdentifier ); // returns the index of the identifier-specified value into the value-vector


		// TEMP:
		math::Matrix33f getStress()
		{
			return math::Matrix33f( values[1], values[2], values[3],
									values[4], values[5], values[6],
									values[7], values[8], values[9]);
		}


		Cell                                                    *neighbours[10]; // references to the 10 neighbours of the cell

		std::vector<float>                                               values; // dynamic array of values defined per cell. The meaning of the values is left to the user
		std::map<std::string, int>                             valueIdentifiers; // this maps name identifiers to indices into the value vector
		bool                                    allInterpolationNeighboursExist; // is true if the cell is not a boundary cell and therefore all neighbours exist
		math::Vec3f                                                      center; // spational center of the cell


		// TEMP:used for testing
		float                   maxEigenvalue;
		math::Vec3f maxEigenvaluesEigenvector;

		std::vector<math::Vec3f>            eigenVectors;
		std::vector<float>                   eigenValues;
	};




	Grid();                                                                            // constructor
	void                       setup( float _cellSize, size_t x, size_t y, size_t z ); // (re)initializes the grid
	void                                                                      clear(); // reldirects to a call to setup( 1.0f, 0, 0, 0 );
	Cell                                     *getCell( size_t i, size_t j, size_t k ); // returns reference to a kartesian coordinate specified cell
	Cell                                                    *getCell(  size_t index ); // returns reference to a index specified cell
	float                  getValueAt( float x, float y, float z, size_t valueIndex ); // Returns the trilinear interpolated index-specified value
	float        getValueAt( float x, float y, float z, std::string valueIdentifier ); // Returns the trilinear interpolated identifier-specified value
	float        getValueAtNormalized( float x, float y, float z, size_t valueIndex ); // Returns the trilinear interpolated index-specified value
	size_t                                                           getResolutionX(); // returns the number of cells in x direction
	size_t                                                           getResolutionY(); // returns the number of cells in y direction
	size_t                                                           getResolutionZ(); // returns the number of cells in z direction
	float                                                               getCellSize(); // returns unform size of one cell
	size_t                                                             getCellCount(); // returns the total number of cells


//private:
	std::vector<Cell>                                                            grid; // grid data
	size_t                                      resolutionX, resolutionY, resolutionZ; // number of cells within each direction
	float                                                                    cellSize; // width height and depth of one cell 


	// used for testing
	float                                                     maxEigenvalueOfAllCells;
};