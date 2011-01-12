/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once



namespace math
{
	//
	//
	// memory storage is row major (which is native to the c++ arrays)
    //     | 0  1  |
    // M = | 2  3  |
	//
	// index the matrix with matrix.m[row][column] -> _m12 means: first row, second column
	//
	//           m11 m12
	//           m21 m22
	//
	//
	class Matrix22f
	{
	public:
        Matrix22f();
		~Matrix22f();
		Matrix22f( const float &_11, const float &_12,
			       const float &_21, const float &_22 );

		// convience matrix creation functions
		static Matrix22f                                                                 Zero( void );  // returns the zeromatrix
		static Matrix22f                                                             Identity( void );  // returns the identitymatrix
		static Matrix22f                                         RotationMatrix( const float &angle );  // returns a matrix which defines a rotation with the float-specified amount (in radians)

		// public methods
		void                                                                        transpose( void );
		void                                                                           invert( void );

		// operators
		bool                                                      operator==( const Matrix22f &rhs );
		bool                                                      operator!=( const Matrix22f &rhs );
		
		bool                                                      operator+=( const Matrix22f &rhs );
		bool                                                      operator-=( const Matrix22f &rhs );

		bool                                                          operator+=( const float &rhs );
		bool                                                          operator-=( const float &rhs );
		bool                                                          operator*=( const float &rhs );
		bool                                                          operator/=( const float &rhs );

		union
		{
			struct
			{
				float _11, _12;	
				float _21, _22;
			};
			float m[2][2];
			float ma[4];
		};
	};
}