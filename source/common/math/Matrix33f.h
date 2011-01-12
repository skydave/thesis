/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once

#include "Vec3f.h"

namespace math
{
	//
	//
	// memory storage is row major (which is native to the c++ arrays)
    //     | 0  1  2 |
    // M = | 3  4  5 |
	//     | 6  7  8 |
	//
	// index the matrix with matrix.m[row][column] -> _m12 means: first row, second column
	//
	//           m11 m12 m13
	//           m21 m22 m23
	//           m31 m32 m33
	//
	class Matrix33f
	{
	public:
        Matrix33f();
		~Matrix33f();
		Matrix33f( const float &_11, const float &_12, const float &_13,
			       const float &_21, const float &_22, const float &_23,
				   const float &_31, const float &_32, const float &_33);

		// convience matrix creation functions
		static Matrix33f                                                                 Zero( void ); // returns the zeromatrix
		static Matrix33f                                                             Identity( void ); // returns the identitymatrix
		static Matrix33f                      RotationMatrix( const Vec3f &axis, const float &angle );  // returns a matrix with a transformation that rotates around a certain axis which starts at the origin

		// public methods
		void                                                                        transpose( void );
		void                                                                           invert( void );

		float                                                                        getDeterminant(); // computes and returns the determinant

		// operators
		bool                                                       operator==( const Matrix33f &rhs );
		bool                                                       operator!=( const Matrix33f &rhs );
		
		bool                                                       operator+=( const Matrix33f &rhs );
		bool                                                       operator-=( const Matrix33f &rhs );

		bool                                                           operator+=( const float &rhs );
		bool                                                           operator-=( const float &rhs );
		bool                                                           operator*=( const float &rhs );
		bool                                                           operator/=( const float &rhs );

		union
		{
			struct
			{
				float _11, _12, _13;
				float _21, _22, _23;
				float _31, _32, _33;
			};
			float m[3][3];
			float ma[9];
		};
	};
}