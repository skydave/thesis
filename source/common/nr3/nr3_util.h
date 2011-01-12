/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once

#include "nr3/nr3.h"
// used for singular value decomposition
#include "nr3/svd.h"
// used for eigenvalue value decomposition
#include "nr3/eigen_sym.h"

//
// performs eigenvalue decomposition on a nxn symmetric matrix
//
// note: values are assumed to be stored row major (like normal c++ arrays)
//
void doEigenDecomposition( int rows, int columns, float *values, std::vector<float> &eigenValues, std::vector< std::vector<float> > &eigenVectors );