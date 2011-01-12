/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "nr3_util.h"






//
// performs eigenvalue decomposition on a nxn symmetric matrix
//
// note: values are assumed to be stored row major (like normal c++ arrays)
//
void doEigenDecomposition( int rows, int columns, float *values, std::vector<float> &eigenValues, std::vector< std::vector<float> > &eigenVectors )
{
	MatDoub matrix;

	matrix.resize( rows, columns );
	int count = 0;
	for( int j=0; j<rows; ++j )
		for( int i=0; i<columns; ++i )
			matrix[j][i] = values[count++];

	Symmeig test( matrix, true );

	// extract results --------------------------------------------------

	// eigenvalues and columns of solutionmatrix (the columns of z are the normalized eigenvector
	// corresponding to d)
	for( int i=0; i<test.d.size(); ++i )
	{
		eigenValues.push_back( (float)test.d[i] );

		eigenVectors.push_back( std::vector<float>() );
		for( int j=0; j<test.z.nrows(); ++j )
			eigenVectors.back().push_back( (float)test.z[j][i] );
	}
}