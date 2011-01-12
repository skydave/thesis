/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once

#include <math.h>

#define MATH_PIf 3.14159265f
#define MATH_PI 3.14159265

#include "Vec2f.h"
#include "Vec3f.h"
#include "Vec3d.h"
#include "Ray3d.h"
#include "Matrix22f.h"
#include "Matrix33f.h"
#include "Matrix44f.h"
#include "Color.h"
#include "BoundingBox.h"
#include "BoundingBox2d.h"
#include "RNG.h"

///
/// \brief vectors matrices and mathematical operations
///
namespace math
{
	float radToDeg( float rad );
	float degToRad( float degree );

	double radToDeg( double rad );
	double degToRad( double degree );
	//
	// global Vec2f related ops
	//
	inline Vec2f operator-( const Vec2f &lhs, const Vec2f &rhs )
	{
		return Vec2f( lhs.x-rhs.x, lhs.y-rhs.y );
	}
	inline Vec2f operator+( const Vec2f &lhs, const Vec2f &rhs )
	{
		return Vec2f( lhs.x+rhs.x, lhs.y+rhs.y );
	}


	inline Vec2f operator*( const Vec2f &lhs, const float &rhs )
	{
		return Vec2f( lhs.x*rhs, lhs.y*rhs );
	}
	inline Vec2f operator*( const float &lhs, const Vec2f &rhs )
	{
		return (rhs*lhs);
	}

	//
	// global Vec2f related functions
	//
/*
	inline Vec3f normalize( const Vec3f &vector )
	{
		Vec3f result = vector;
		result.normalize();
		return result;
	}

	inline float dotProduct( const Vec3f &lhs, const Vec3f &rhs )
	{
		return (lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z);
	}

	inline void dotProduct( float &result, const Vec3f &lhs, const Vec3f &rhs )
	{
		result = (lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z);
	}
*/
	inline float crossProduct( const Vec2f &lhs, const Vec2f &rhs )
	{
		return lhs.x*rhs.y - lhs.y*rhs.x;
	}

	inline void crossProduct( float &result, const Vec2f &lhs, const Vec2f &rhs )
	{
		result = lhs.x*rhs.y - lhs.y*rhs.x;
	}

	//
	// global Vec3f related ops
	//

	inline Vec3f operator-( const Vec3f &rhs )
	{
		return Vec3f( -rhs.x, -rhs.y, -rhs.z );
	}

	inline Vec3f operator+( const Vec3f &lhs, const Vec3f &rhs )
	{
		return Vec3f( lhs.x+rhs.x, lhs.y+rhs.y, lhs.z+rhs.z );
	}
	inline Vec3f operator-( const Vec3f &lhs, const Vec3f &rhs )
	{
		return Vec3f( lhs.x-rhs.x, lhs.y-rhs.y, lhs.z-rhs.z );
	}
	inline Vec3f operator/( const Vec3f &lhs, const Vec3f &rhs )
	{
		return Vec3f( lhs.x/rhs.x, lhs.y/rhs.y, lhs.z/rhs.z );
	}		
	inline Vec3f operator+( const Vec3f &lhs, const float &rhs )
	{
		return Vec3f( lhs.x+rhs, lhs.y+rhs, lhs.z+rhs );
	}
	inline Vec3f operator-( const Vec3f &lhs, const float &rhs )
	{
		return Vec3f( lhs.x-rhs, lhs.y-rhs, lhs.z-rhs );
	}
	inline Vec3f operator*( const Vec3f &lhs, const float &rhs )
	{
		return Vec3f( lhs.x*rhs, lhs.y*rhs, lhs.z*rhs );
	}

	inline Vec3f operator/( const Vec3f &lhs, const float &rhs )
	{
		return Vec3f( lhs.x/rhs, lhs.y/rhs, lhs.z/rhs );
	}

	inline Vec3f operator+( const float &lhs, const Vec3f &rhs )
	{
		return (rhs+lhs);
	}
	inline Vec3f operator-( const float &lhs, const Vec3f &rhs )
	{
		return (rhs-lhs);
	}
	inline Vec3f operator*( const float &lhs, const Vec3f &rhs )
	{
		return (rhs*lhs);
	}
	inline Vec3f operator/( const float &lhs, const Vec3f &rhs )
	{
		return (rhs/lhs);
	}

	//
	// global Vec3f related functions
	//

	inline Vec3f normalize( const Vec3f &vector )
	{
		Vec3f result = vector;
		result.normalize();
		return result;
	}

	inline float dotProduct( const Vec3f &lhs, const Vec3f &rhs )
	{
		return (lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z);
	}

	inline void dotProduct( float &result, const Vec3f &lhs, const Vec3f &rhs )
	{
		result = (lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z);
	}

	inline Vec3f crossProduct( const Vec3f &lhs, const Vec3f &rhs )
	{
		return Vec3f( lhs.y*rhs.z - lhs.z*rhs.y,
					  lhs.z*rhs.x - lhs.x*rhs.z,
					  lhs.x*rhs.y - lhs.y*rhs.x );
	}

	inline void crossProduct( Vec3f &result, const Vec3f &lhs, const Vec3f &rhs )
	{
		result.x = lhs.y*rhs.z - lhs.z*rhs.y;
		result.y = lhs.z*rhs.x - lhs.x*rhs.z;
		result.z = lhs.x*rhs.y - lhs.y*rhs.x;
	}

	inline double dotProduct( const math::Vec3d &vec1, const math::Vec3d &vec2 )
	{
		return vec1.x*vec2.x+vec1.y*vec2.y+vec1.z*vec2.z;
	}

	inline math::Vec3d crossProduct( const math::Vec3d &lhs, const math::Vec3d &rhs )
	{
		return Vec3d( lhs.y*rhs.z - lhs.z*rhs.y,
				  lhs.z*rhs.x - lhs.x*rhs.z,
				  lhs.x*rhs.y - lhs.y*rhs.x );
	}


	//
	// global Matrix44f related ops
	//

	inline Matrix44f operator+( const Matrix44f &lhs, const Matrix44f &rhs )
	{
		return Matrix44f(   lhs._11+rhs._11, lhs._12+rhs._12, lhs._13+rhs._13, lhs._14+rhs._14,
							lhs._21+rhs._21, lhs._22+rhs._22, lhs._23+rhs._23, lhs._24+rhs._24,
							lhs._31+rhs._31, lhs._32+rhs._32, lhs._33+rhs._33, lhs._34+rhs._34,
							lhs._41+rhs._41, lhs._42+rhs._42, lhs._43+rhs._43, lhs._44+rhs._44 );
	}

	inline Matrix44f operator-( const Matrix44f &lhs, const Matrix44f &rhs )
	{
		return Matrix44f(   lhs._11-rhs._11, lhs._12-rhs._12, lhs._13-rhs._13, lhs._14-rhs._14,
							lhs._21-rhs._21, lhs._22-rhs._22, lhs._23-rhs._23, lhs._24-rhs._24,
							lhs._31-rhs._31, lhs._32-rhs._32, lhs._33-rhs._33, lhs._34-rhs._34,
							lhs._41-rhs._41, lhs._42-rhs._42, lhs._43-rhs._43, lhs._44-rhs._44 );
	}

	static unsigned short g_r, g_c, g_k;
		
	inline Matrix44f operator+( const Matrix44f &lhs, const float &rhs )
	{
		Matrix44f result;
		result = lhs;

		result._11 += rhs;
		result._12 += rhs;
		result._13 += rhs;
		result._14 += rhs;
		result._21 += rhs;
		result._22 += rhs;
		result._23 += rhs;
		result._24 += rhs;
		result._31 += rhs;
		result._32 += rhs;
		result._33 += rhs;
		result._34 += rhs;
		result._41 += rhs;
		result._42 += rhs;
		result._43 += rhs;
		result._44 += rhs;

		return result;
	}
	inline Matrix44f operator-( const Matrix44f &lhs, const float &rhs )
	{
		Matrix44f result;
		result = lhs;

		result._11 -= rhs;
		result._12 -= rhs;
		result._13 -= rhs;
		result._14 -= rhs;
		result._21 -= rhs;
		result._22 -= rhs;
		result._23 -= rhs;
		result._24 -= rhs;
		result._31 -= rhs;
		result._32 -= rhs;
		result._33 -= rhs;
		result._34 -= rhs;
		result._41 -= rhs;
		result._42 -= rhs;
		result._43 -= rhs;
		result._44 -= rhs;

		return result;
	}
	inline Matrix44f operator*( const Matrix44f &lhs, const float &rhs )
	{
		Matrix44f result;
		result = lhs;

		result._11 *= rhs;
		result._12 *= rhs;
		result._13 *= rhs;
		result._14 *= rhs;
		result._21 *= rhs;
		result._22 *= rhs;
		result._23 *= rhs;
		result._24 *= rhs;
		result._31 *= rhs;
		result._32 *= rhs;
		result._33 *= rhs;
		result._34 *= rhs;
		result._41 *= rhs;
		result._42 *= rhs;
		result._43 *= rhs;
		result._44 *= rhs;

		return result;
	}
	inline Matrix44f operator/( const Matrix44f &lhs, const float &rhs )
	{
		Matrix44f result;
		result = lhs;

		result._11 /= rhs;
		result._12 /= rhs;
		result._13 /= rhs;
		result._14 /= rhs;
		result._21 /= rhs;
		result._22 /= rhs;
		result._23 /= rhs;
		result._24 /= rhs;
		result._31 /= rhs;
		result._32 /= rhs;
		result._33 /= rhs;
		result._34 /= rhs;
		result._41 /= rhs;
		result._42 /= rhs;
		result._43 /= rhs;
		result._44 /= rhs;

		return result;
	}

	inline Matrix44f operator+( const float &lhs, const Matrix44f &rhs )
	{
		return (rhs+lhs);
	}
	inline Matrix44f operator-( const float &lhs, const Matrix44f &rhs )
	{
		return (rhs-lhs);
	}
	inline Matrix44f operator*( const float &lhs, const Matrix44f &rhs )
	{
		return (rhs*lhs);
	}
	inline Matrix44f operator/( const float &lhs, const Matrix44f &rhs )
	{
		return (rhs/lhs);
	}

	inline Matrix44f operator-( const Matrix44f &rhs )
	{
		return Matrix44f( -rhs._11, -rhs._12, -rhs._13, -rhs._13,
						  -rhs._21, -rhs._22, -rhs._23, -rhs._23,
						  -rhs._31, -rhs._32, -rhs._33, -rhs._33,
						  -rhs._41, -rhs._42, -rhs._43, -rhs._43);
	}

	//
	// global Matrix44f related functions
	//

	inline void matrixMultiply( Matrix44f &result, const Matrix44f &lhs, const Matrix44f &rhs )
	{
		result._11 = lhs._11*rhs._11 + lhs._12*rhs._21 + lhs._13*rhs._31 + lhs._14*rhs._41;
		result._12 = lhs._11*rhs._12 + lhs._12*rhs._22 + lhs._13*rhs._32 + lhs._14*rhs._42;
		result._13 = lhs._11*rhs._13 + lhs._12*rhs._23 + lhs._13*rhs._33 + lhs._14*rhs._43;
		result._14 = lhs._11*rhs._14 + lhs._12*rhs._24 + lhs._13*rhs._34 + lhs._14*rhs._44;

		result._21 = lhs._21*rhs._11 + lhs._22*rhs._21 + lhs._23*rhs._31 + lhs._24*rhs._41;
		result._22 = lhs._21*rhs._12 + lhs._22*rhs._22 + lhs._23*rhs._32 + lhs._24*rhs._42;
		result._23 = lhs._21*rhs._13 + lhs._22*rhs._23 + lhs._23*rhs._33 + lhs._24*rhs._43;
		result._24 = lhs._21*rhs._14 + lhs._22*rhs._24 + lhs._23*rhs._34 + lhs._24*rhs._44;

		result._31 = lhs._31*rhs._11 + lhs._32*rhs._21 + lhs._33*rhs._31 + lhs._34*rhs._41;
		result._32 = lhs._31*rhs._12 + lhs._32*rhs._22 + lhs._33*rhs._32 + lhs._34*rhs._42;
		result._33 = lhs._31*rhs._13 + lhs._32*rhs._23 + lhs._33*rhs._33 + lhs._34*rhs._43;
		result._34 = lhs._31*rhs._14 + lhs._32*rhs._24 + lhs._33*rhs._34 + lhs._34*rhs._44;

		result._41 = lhs._41*rhs._11 + lhs._42*rhs._21 + lhs._43*rhs._31 + lhs._44*rhs._41;
		result._42 = lhs._41*rhs._12 + lhs._42*rhs._22 + lhs._43*rhs._32 + lhs._44*rhs._42;
		result._43 = lhs._41*rhs._13 + lhs._42*rhs._23 + lhs._43*rhs._33 + lhs._44*rhs._43;
		result._44 = lhs._41*rhs._14 + lhs._42*rhs._24 + lhs._43*rhs._34 + lhs._44*rhs._44;
	}

	inline Vec3f transform(  const Vec3f &lhs, const Matrix44f &rhs )
	{
		return Vec3f(lhs.x*rhs._11 + lhs.y*rhs._21 + lhs.z*rhs._31 + rhs._41,
					 lhs.x*rhs._12 + lhs.y*rhs._22 + lhs.z*rhs._32 + rhs._42,
					 lhs.x*rhs._13 + lhs.y*rhs._23 + lhs.z*rhs._33 + rhs._43 );
	}

	inline void transform( Vec3f &result, const Vec3f &lhs, const Matrix44f &rhs )
	{
		result.x = lhs.x*rhs._11 + lhs.y*rhs._21 + lhs.z*rhs._31 + rhs._41;
		result.y = lhs.x*rhs._12 + lhs.y*rhs._22 + lhs.z*rhs._32 + rhs._42;
		result.z = lhs.x*rhs._13 + lhs.y*rhs._23 + lhs.z*rhs._33 + rhs._43;
	}

	inline Matrix44f operator*( const Matrix44f &lhs, const Matrix44f &rhs )
	{
		Matrix44f result;
		matrixMultiply( result, lhs, rhs );
		return result;
	}

	inline float frobeniusNorm( const Matrix44f &lhs )
	{
		float result = 0.0f;
		result += lhs._11*lhs._11;
		result += lhs._12*lhs._12;
		result += lhs._13*lhs._13;
		result += lhs._14*lhs._14;
		result += lhs._21*lhs._21;
		result += lhs._22*lhs._22;
		result += lhs._23*lhs._23;
		result += lhs._24*lhs._24;
		result += lhs._31*lhs._31;
		result += lhs._32*lhs._32;
		result += lhs._33*lhs._33;
		result += lhs._34*lhs._34;
		result += lhs._41*lhs._41;
		result += lhs._42*lhs._42;
		result += lhs._43*lhs._43;
		result += lhs._44*lhs._44;
		return sqrt(result);
	}


	Matrix44f createLookAtMatrix( const Vec3f &position, const Vec3f &target, const Vec3f &_up );
	Matrix44f createMatrixFromPolarCoordinates( const float &azimuth, const float &elevation, const float &distance );







	// Vec2 stuff
	inline Vec2f operator-( const Vec2f &rhs )
	{
		return Vec2f( -rhs.x, -rhs.y );
	}

	inline Vec2f transform( const Vec2f &lhs, const Matrix44f &rhs )
	{
		return Vec2f(lhs.x*rhs._11 + lhs.y*rhs._21 + rhs._41,
			lhs.x*rhs._12 + lhs.y*rhs._22 + rhs._42 );
	}

	inline Vec2f transform( const Vec2f &lhs, const Matrix22f &rhs )
	{
		return Vec2f(lhs.x*rhs._11 + lhs.y*rhs._21,
					 lhs.x*rhs._12 + lhs.y*rhs._22 );
	}

	inline Matrix22f outerProduct( const Vec2f &lhs, const Vec2f &rhs )
	{
		Matrix22f result;

		result._11 = lhs.x * rhs.x;
		result._12 = lhs.y * rhs.x;

		result._21 = lhs.x * rhs.y;
		result._22 = lhs.y * rhs.y;

		return result;
	}


	//
	// global Matrix22f related ops
	//




	inline Matrix22f operator+( const Matrix22f &lhs, const Matrix22f &rhs )
	{
		return Matrix22f(   lhs._11+rhs._11, lhs._12+rhs._12,
							lhs._21+rhs._21, lhs._22+rhs._22 );
	}

	inline Matrix22f operator-( const Matrix22f &lhs, const Matrix22f &rhs )
	{
		return Matrix22f(   lhs._11-rhs._11, lhs._12-rhs._12,
							lhs._21-rhs._21, lhs._22-rhs._22 );
	}




	inline void matrixMultiply( Matrix22f &result, const Matrix22f &lhs, const Matrix22f &rhs )
	{
		result._11 = lhs._11*rhs._11 + lhs._12*rhs._21;
		result._12 = lhs._11*rhs._12 + lhs._12*rhs._22;

		result._21 = lhs._21*rhs._11 + lhs._22*rhs._21;
		result._22 = lhs._21*rhs._12 + lhs._22*rhs._22;
	}

	inline Matrix22f operator/( const Matrix22f &lhs, const float &rhs )
	{
		Matrix22f result;
		result = lhs;

		result._11 /= rhs;
		result._12 /= rhs;

		result._21 /= rhs;
		result._22 /= rhs;

		return result;
	}

	inline Matrix22f operator*( const Matrix22f &lhs, const float &rhs )
	{
		Matrix22f result;
		result = lhs;

		result._11 *= rhs;
		result._12 *= rhs;

		result._21 *= rhs;
		result._22 *= rhs;

		return result;
	}

	inline Matrix22f operator-( const Matrix22f &rhs )
	{
		return Matrix22f( -rhs._11, -rhs._12,
						  -rhs._21, -rhs._22);
	}

	inline Matrix22f operator*( const float &lhs, const Matrix22f &rhs )
	{
		return rhs * lhs;
	}
	inline Matrix22f operator/( const float &lhs, const Matrix22f &rhs )
	{
		return rhs / lhs;
	}
	inline Matrix22f operator*( const Matrix22f &lhs, const Matrix22f &rhs )
	{
		Matrix22f result;
		matrixMultiply( result, lhs, rhs );
		return result;
	}

	inline Matrix22f transpose( const Matrix22f &lhs )
	{
		Matrix22f result = lhs;
		result.transpose();
		return result;
	}

	inline float frobeniusNorm( const Matrix22f &lhs )
	{
		float result = 0.0f;

		result += lhs._11*lhs._11;
		result += lhs._12*lhs._12;

		result += lhs._21*lhs._21;
		result += lhs._22*lhs._22;

		return sqrt(result);
	}

	//
	// global Matrix33f related ops
	//
	inline void matrixMultiply( Matrix33f &result, const Matrix33f &lhs, const Matrix33f &rhs )
	{
		result._11 = lhs._11*rhs._11 + lhs._12*rhs._21 + lhs._13*rhs._31;
		result._12 = lhs._11*rhs._12 + lhs._12*rhs._22 + lhs._13*rhs._32;
		result._13 = lhs._11*rhs._13 + lhs._12*rhs._23 + lhs._13*rhs._33;

		result._21 = lhs._21*rhs._11 + lhs._22*rhs._21 + lhs._23*rhs._31;
		result._22 = lhs._21*rhs._12 + lhs._22*rhs._22 + lhs._23*rhs._32;
		result._23 = lhs._21*rhs._13 + lhs._22*rhs._23 + lhs._23*rhs._33;

		result._31 = lhs._31*rhs._11 + lhs._32*rhs._21 + lhs._33*rhs._31;
		result._32 = lhs._31*rhs._12 + lhs._32*rhs._22 + lhs._33*rhs._32;
		result._33 = lhs._31*rhs._13 + lhs._32*rhs._23 + lhs._33*rhs._33;
	}

	inline Matrix33f operator/( const Matrix33f &lhs, const float &rhs )
	{
		Matrix33f result;
		result = lhs;

		result._11 /= rhs;
		result._12 /= rhs;
		result._13 /= rhs;

		result._21 /= rhs;
		result._22 /= rhs;
		result._23 /= rhs;

		result._31 /= rhs;
		result._32 /= rhs;
		result._33 /= rhs;

		return result;
	}

	inline Matrix33f operator*( const Matrix33f &lhs, const float &rhs )
	{
		Matrix33f result;
		result = lhs;

		result._11 *= rhs;
		result._12 *= rhs;
		result._13 *= rhs;

		result._21 *= rhs;
		result._22 *= rhs;
		result._23 *= rhs;

		result._31 *= rhs;
		result._32 *= rhs;
		result._33 *= rhs;

		return result;
	}

	inline Matrix33f operator-( const Matrix33f &lhs, const float &rhs )
	{
		Matrix33f result;
		result = lhs;

		result._11 -= rhs;
		result._12 -= rhs;
		result._13 -= rhs;

		result._21 -= rhs;
		result._22 -= rhs;
		result._23 -= rhs;

		result._31 -= rhs;
		result._32 -= rhs;
		result._33 -= rhs;

		return result;
	}

	inline Matrix33f operator+( const Matrix33f &lhs, const float &rhs )
	{
		Matrix33f result;
		result = lhs;

		result._11 += rhs;
		result._12 += rhs;
		result._13 += rhs;

		result._21 += rhs;
		result._22 += rhs;
		result._23 += rhs;

		result._31 += rhs;
		result._32 += rhs;
		result._33 += rhs;

		return result;
	}


	inline Matrix33f operator-( const Matrix33f &rhs )
	{
		return Matrix33f( -rhs._11, -rhs._12, -rhs._13,
						  -rhs._21, -rhs._22, -rhs._23,
						  -rhs._31, -rhs._32, -rhs._33);
	}

	inline Matrix33f operator-( const Matrix33f &lhs, const Matrix33f &rhs )
	{
		return Matrix33f( lhs._11-rhs._11, lhs._12-rhs._12, lhs._13-rhs._13,
						  lhs._21-rhs._21, lhs._22-rhs._22, lhs._23-rhs._23,
						  lhs._31-rhs._31, lhs._32-rhs._32, lhs._33-rhs._33);
	}
	inline Matrix33f operator+( const Matrix33f &lhs, const Matrix33f &rhs )
	{
		return Matrix33f( lhs._11+rhs._11, lhs._12+rhs._12, lhs._13+rhs._13,
						  lhs._21+rhs._21, lhs._22+rhs._22, lhs._23+rhs._23,
						  lhs._31+rhs._31, lhs._32+rhs._32, lhs._33+rhs._33);
	}

	inline Matrix33f operator*( const float &lhs, const Matrix33f &rhs )
	{
		return rhs * lhs;
	}

	inline Matrix33f operator/( const float &lhs, const Matrix33f &rhs )
	{
		return rhs / lhs;
	}

	inline Matrix33f operator-( const float &lhs, const Matrix33f &rhs )
	{
		return rhs - lhs;
	}

	inline Matrix33f operator+( const float &lhs, const Matrix33f &rhs )
	{
		return rhs + lhs;
	}

	inline Matrix33f operator*( const Matrix33f &lhs, const Matrix33f &rhs )
	{
		Matrix33f result;
		matrixMultiply( result, lhs, rhs );
		return result;
	}

	inline Vec3f transform( const Vec3f &lhs, const Matrix33f &rhs )
	{
		return Vec3f(lhs.x*rhs._11 + lhs.y*rhs._21 + lhs.z*rhs._31,
					 lhs.x*rhs._12 + lhs.y*rhs._22 + lhs.z*rhs._32,
					 lhs.x*rhs._13 + lhs.y*rhs._23 + lhs.z*rhs._33);
	}

	inline Matrix33f transpose( const Matrix33f &lhs )
	{
		Matrix33f result = lhs;
		result.transpose();
		return result;
	}

	inline float frobeniusNorm( const Matrix33f &lhs )
	{
		float result = 0.0f;

		result += lhs._11*lhs._11;
		result += lhs._12*lhs._12;
		result += lhs._13*lhs._13;

		result += lhs._21*lhs._21;
		result += lhs._22*lhs._22;
		result += lhs._23*lhs._23;

		result += lhs._31*lhs._31;
		result += lhs._32*lhs._32;
		result += lhs._33*lhs._33;

		return sqrt(result);
	}

	//
	// color conversion functions
	//

	/*
	#define ALPHAMASK	0xff000000
	#define REDMASK		0x00ff0000
	#define GREENMASK	0x0000ff00
	#define BLUEMASK	0x000000ff

	#define ALPHASHIFT 24
	#define REDSHIFT   16
	#define GREENSHIFT 8
	#define BLUESHIFT  0
	*/
	// RGBA
	#define REDMASK		0x000000ff
	#define GREENMASK	0x0000ff00
	#define BLUEMASK	0x00ff0000
	#define ALPHAMASK	0xff000000

	#define REDSHIFT   0
	#define GREENSHIFT 8
	#define BLUESHIFT  16
	#define ALPHASHIFT 24

	inline unsigned int getAlpha( const unsigned long &color )
	{
		return ((color&ALPHAMASK) >> ALPHASHIFT);
	}
	inline unsigned int	getRed( const unsigned long &color )
	{
		return ((color&REDMASK) >> REDSHIFT);
	}
	inline unsigned int	getGreen( const unsigned long &color )
	{
		return ((color&GREENMASK) >> GREENSHIFT);
	}
	inline unsigned int	getBlue( const unsigned long &color )
	{
		return ((color&BLUEMASK) >> BLUESHIFT);
	}
	inline unsigned long setColor( const unsigned int &r, const unsigned int &g, const unsigned int &b, const unsigned int &a )
	{
		return ((a << ALPHASHIFT) + (r << REDSHIFT) + (g << GREENSHIFT) + (b << BLUESHIFT));
	}

	inline unsigned int	getAlpha( const Color &color )
	{
		return (unsigned int)(color.a*255.0f);
	}

	inline unsigned int	getRed(		const Color &color )
	{
		return (unsigned int)(color.r*255.0f);
	}

	inline unsigned int	getGreen(	const Color &color )
	{
		return (unsigned int)(color.g*255.0f);
	}

	inline unsigned int	getBlue(	const Color &color )
	{
		return (unsigned int)(color.b*255.0f);
	}

	inline Color setRGBColor(const unsigned int &r, const unsigned int &g, const unsigned int &b, const unsigned int &a )
	{
		return Color( ((float)r)/255.0f, ((float)g)/255.0f, ((float)b)/255.0f, ((float)a)/255.0f );
	}

//
// Color related ops
//

inline Color operator+( const Color &lhs, const Color &rhs )
{
return Color( lhs.r+rhs.r, lhs.g+rhs.g, lhs.b+rhs.b, lhs.a+rhs.a );
}
inline Color operator-( const Color &lhs, const Color &rhs )
{
return Color( lhs.r-rhs.r, lhs.g-rhs.g, lhs.b-rhs.b, lhs.a-rhs.a );
}
inline Color operator*( const Color &lhs, const Color &rhs )
{
return Color( lhs.r*rhs.r, lhs.g*rhs.g, lhs.b*rhs.b, lhs.a*rhs.a );
}
inline Color operator/( const Color &lhs, const Color &rhs )
{
return Color( lhs.r/rhs.r, lhs.g/rhs.g, lhs.b/rhs.b, lhs.a/rhs.a );
}

inline Color operator+( const Color &lhs, const float &rhs )
{
return Color( lhs.r+rhs, lhs.g+rhs, lhs.b+rhs, lhs.a+rhs );
}
inline Color operator-( const Color &lhs, const float &rhs )
{
return Color( lhs.r-rhs, lhs.g-rhs, lhs.b-rhs, lhs.a-rhs );
}
inline Color operator*( const Color &lhs, const float &rhs )
{
return Color( lhs.r*rhs, lhs.g*rhs, lhs.b*rhs, lhs.a*rhs );
}
inline Color operator/( const Color &lhs, const float &rhs )
{
return Color( lhs.r/rhs, lhs.g/rhs, lhs.b/rhs, lhs.a/rhs );
}

inline Color operator+( const float &lhs, const Color &rhs )
{
return Color( (lhs + rhs.r), (lhs + rhs.g), (lhs + rhs.b), (lhs + rhs.a) );
}
inline Color operator-( const float &lhs, const Color &rhs )
{
return (rhs-lhs);
}
inline Color operator*( const float &lhs, const Color &rhs )
{
return (rhs*lhs);
}
inline Color operator/( const float &lhs, const Color &rhs )
{
return (rhs/lhs);
}

//
// Inverts only the color channels of the given color, not the alpha channel
//
inline Color invert( const Color &col )
{
	return Color( 1.0f - col.r, 1.0f - col.g, 1.0f - col.b, col.a );
}


//
// ray related functions
//

//bool RayHitSphere( const Vector &vSpherePosition, const float &fSphereRadius, CRay &oRay );
//bool RayHitSphereValues( const Vector &vSpherePosition, const float &fSphereRadius, CRay &oRay, float &fHitNear, float &fHitFar, Vector *pHitPointNear, Vector *pHitPointFar );

//bool RayHitPlane( const Vector &vPlaneNormal, const float &fD, CRay &oRay );
//bool rayHitPlaneValues( const Vec3f &planeNormal, const float &planeDistance, Ray &ray, float &hitDistance, Vec3f *hitPoint );
bool        intersectionRayPlane( const Ray3d &ray, const Vec3f &normal, const float &distance, Vec3f &hitPoint );
bool                                  intersectionRayRay( const Ray3d &ray1, const Ray3d &ray2, Vec3f &hitPoint );

//bool RayHitTriangle( const Vector &vPoint1, const Vector &vPoint2, const Vector &vPoint3, CRay &oRay );
//bool RayHitTriangleValues( const Vector &vPoint1, const Vector &vPoint2, const Vector &vPoint3, CRay &oRay, float &fHit, Vector *pHitPoint, float *pU, float *pV );


float                                                   area( const Vec3f &p0, const Vec3f &p1, const Vec3f &p2 ); // computes area of an triangle

float                                                                distance( const Vec3f &p0, const Vec3f &p1 ); // computes the euclidian distance between 2 points in space
float                                                         squaredDistance( const Vec3f &p0, const Vec3f &p1 ); // computes the squared euclidian distance between 2 points in space
float                  distancePointPlane( const math::Vec3f &point, const Vec3f &normal, const float &distance ); // computes the distance of a point to a given plane
float                             distancePointLine( const math::Vec3f &point, const Vec3f &p1, const Vec3f &p2 ); // returns the distance of the given point to the line specified by two points


math::Vec3f     projectPointOnPlane( const math::Vec3f &normal, const float &distance, const math::Vec3f &point ); // returns the projection of the given point on the normal and distance specified plane
math::Vec3f          projectPointOnLine( const math::Vec3f &point, const math::Vec3f &p1, const math::Vec3f &p2 );

//
// Misc mathematical utilities
//

//
//
//
float mapValueToRange( const float &sourceRangeMin, const float &sourceRangeMax, const float &targetRangeMin, const float &targetRangeMax, const float &value );
float mapValueTo0_1( const float &sourceRangeMin, const float &sourceRangeMax, const float &value );










}