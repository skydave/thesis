/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once

#include "Vec3f.h"



namespace math
{
	/// \brief matrix class which is specialiced in the usage as transform matrix
	///
	/// Layout of base vectors within matrices
    ///     | Ix  Iy  Iz  0 |
    /// M = | Jx  Jy  Jz  0 |
    ///     | Kx  Ky  Kz  0 |
    ///     | Tx  Ty  Tz  1  |
	/// where I,J and K are the base vectors of |R³
	///
	/// memory storage is row major (which is native to the c++ arrays)
    ///     | 0  1  2  3 |
    /// M = | 4  5  6  7 |
    ///     | 8  9  10 11 |
    ///     | 12 13 14 15 |
	///
	/// index the matrix with matrix.m[row][column] -> _m12 means: first row, second column
	///
	///              m11 m12 m13 m14
	/// (x y z 1)    m21 m22 m23 m24
	///              m31 m32 m33 m34
	///              m41 m42 m43 m44
	///
	/// matrix multiplication with a vector assumes row vectors which are multiplied on the left side like vector*matrix
	/// this is from how we have layed out our base vectors
	///
	/// In terms of computergraphics the matrix is layed out in the spirit of direct x in terms of vector-multiplication
	/// and basevectors and in terms of memory layout.
	/// Since opengl has layed out its base vectors in a transposed fashion but in addition assumes column-major memory
	/// layout, this matrix can be used for opengl as well.
	///
	///
	class Matrix44f
	{
	public:
        Matrix44f();
		~Matrix44f();
		Matrix44f( const float &_11, const float &_12, const float &_13, const float &_14,
			       const float &_21, const float &_22, const float &_23, const float &_24,
				   const float &_31, const float &_32, const float &_33, const float &_34,
                   const float &_41, const float &_42, const float &_43, const float &_44 );
		Matrix44f( const Vec3f &right, const Vec3f &up, const Vec3f &forward );

		// convience matrix creation functions
		static Matrix44f                                                                 Zero( void );  ///< returns the zeromatrix
		static Matrix44f                                                             Identity( void );  ///< returns the identitymatrix
		static Matrix44f                                        RotationMatrixX( const float &angle );  ///< returns a matrix which defines a rotation around the x axis with the float-specified amount
		static Matrix44f                                        RotationMatrixY( const float &angle );  ///< returns a matrix which defines a rotation around the y axis with the float-specified amount
		static Matrix44f                                        RotationMatrixZ( const float &angle );  ///< returns a matrix which defines a rotation around the z axis with the float-specified amount
		static Matrix44f                      RotationMatrix( const Vec3f &axis, const float &angle );  ///< returns a matrix with a transformation that rotates around a certain axis which starts at the origin
		static Matrix44f                                TranslationMatrix( const Vec3f &translation );  ///< returns a matrix which defines a translation of the specified translation vector
		static Matrix44f          TranslationMatrix( const float &x, const float &y, const float &z );  ///< returns a matrix which defines a translation of the specified translation vector
		static Matrix44f                                     ScaleMatrix( const float &uniformScale );  ///< returns a matrix which defines a uniform scale
		static Matrix44f                ScaleMatrix( const float &x, const float &y, const float &z );  ///< returns a matrix which defines a non-uniform scale

		// public methods
		void                                                                        transpose( void );
		void                                                                           invert( void );

		Vec3f                                                 getRight( const bool &normalized=true );
		Vec3f                                                    getUp( const bool &normalized=true );
		Vec3f                                                   getDir( const bool &normalized=true );
		Vec3f                                                                  getTranslation( void );

		Matrix44f                                                              getOrientation( void );
		Matrix44f	                                                 getNormalizedOrientation( void );
		Matrix44f                                                               getTransposed( void );

		// convinience functions for low level matrix manipulation
		void                                                           setRight( const Vec3f &right );
		void                                                                 setUp( const Vec3f &up );
		void                                                               setDir( const Vec3f &dir );
		void                                               setTranslation( const Vec3f &translation );

		// convience functions for higher level matrix manipulations
		void                                                            rotateX( const float &angle ); ///< rotates the current transform around the x-axis (angle in radians)
		void                                                            rotateY( const float &angle ); ///< rotates the current transform around the y-axis (angle in radians)
		void                                                            rotateZ( const float &angle ); ///< rotates the current transform around the z-axis (angle in radians)
		void                                                    translate( const Vec3f &translation ); ///< translates the current transform
		void                              translate( const float &x, const float &y, const float &z ); ///< translates the current transform
		void                                                       scale( const float &uniformScale ); ///< scales the current transform uniformly
		void                                  scale( const float &x, const float &y, const float &z ); ///< scales the current transform non-uniformly
		//void rotate( const Vec3f &origin, const Vec3f &axis, const float &angle ); // will multiply the matrix with a transformation that rotates around a certain axis anywhere in space
		//void                      rotate( const Vec3f &axis, const float &angle ); // will multiply the matrix with a transformation that rotates around a certain axis which starts at the origin

		// operators
		bool                                                      operator==( const Matrix44f &rhs );
		bool                                                      operator!=( const Matrix44f &rhs );
		
		bool                                                      operator+=( const Matrix44f &rhs );
		bool                                                      operator-=( const Matrix44f &rhs );

		bool                                                          operator+=( const float &rhs );
		bool                                                          operator-=( const float &rhs );
		bool                                                          operator*=( const float &rhs );
		bool                                                          operator/=( const float &rhs );

		union
		{
			struct
			{
				float _11, _12, _13, _14;	
				float _21, _22, _23, _24;	
				float _31, _32, _33, _34;	
				float _41, _42, _43, _44;	
			};
			float m[4][4];
			float ma[16];
		};
	};
}