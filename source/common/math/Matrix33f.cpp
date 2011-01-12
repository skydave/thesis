/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "Matrix33f.h"

#include "Math.h"


namespace math
{
	//
	//
	//
	Matrix33f::Matrix33f()
	{
		_11=_12=_13=
		_21=_22=_23=
		_31=_32=_33=0.0f;
	}

	Matrix33f::Matrix33f( const float &_11, const float &_12, const float &_13,
                          const float &_21, const float &_22, const float &_23,
						  const float &_31, const float &_32, const float &_33)
	{
        this->_11=_11; this->_12=_12; this->_13=_13;
        this->_21=_21; this->_22=_22; this->_23=_23;
		this->_31=_31; this->_32=_32; this->_33=_33;
	}

	//
	//
	//
	Matrix33f::~Matrix33f()
	{
	}

	//
	// returns the zeromatrix
	//
	Matrix33f Matrix33f::Zero( void )
	{
		return Matrix33f( 0.0f, 0.0f, 0.0f,
			              0.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 0.0f);
	}

	//
	// returns the identitymatrix
	//
	Matrix33f Matrix33f::Identity( void )
	{
		return Matrix33f( 1.0f, 0.0f, 0.0f,
			              0.0f, 1.0f, 0.0f,
						  0.0f, 0.0f, 1.0f);
	}


	//
	// returns a matrix with a transformation that rotates around a certain axis which starts at the origin
	//
	// code from Graphics Gems (Glassner, Academic Press, 1990)
	//
	Matrix33f Matrix33f::RotationMatrix( const Vec3f &axis, const float &angle )
	{
		float c = cos( angle );
		float t = 1.0f - c;
		float s = sin( angle );
		float txy = t*axis.x*axis.y;
		float txz = t*axis.x*axis.z;
		float tyz = t*axis.y*axis.z;
		float sx = s*axis.x;
		float sy = s*axis.y;
		float sz = s*axis.z;
		return Matrix33f( t*axis.x*axis.x+c, txy+sz, txz-sy,
			              txy-sz, t*axis.y*axis.y+c, tyz+sx,
						  txz+sy, tyz-sx, t*axis.z*axis.z+c);
	}


	void Matrix33f::transpose( void )
	{
		Matrix33f temp = *this;

		_11 = temp._11;
		_12 = temp._21;
		_13 = temp._31;

		_21 = temp._12;
		_22 = temp._22;
		_23 = temp._32;

		_31 = temp._13;
		_32 = temp._23;
		_33 = temp._33;
	}


	//
	// computes the determinant of a 2x2 matrix
	//
	inline float Det( float &_11, float &_12,
					  float &_21, float &_22 )
	{
		return _11*_22 - _12*_21;
	}

	//
	// computes and returns the determinant
	//
	float Matrix33f::getDeterminant()
	{
		// rule of sarrus
		return _11*_22*_33 + _21*_32*_13 + _31*_12*_23 - _13*_22*_31 - _23*_32*_11 - _33*_12*_21;
	}



	//
	//
	//
	void Matrix33f::invert( void )
	{
		float det =	_11*Det(_22, _23,
							_32, _33 ) -
					_12*Det(_21, _23,
							_31, _33 ) +
					_13*Det(_21, _22,
							_31, _32 );
		// determinant must be not zero
		if( fabsf( det ) < 0.00001f )
			// error determinant is zero
			return;

		det = 1.0f / det;

		Matrix33f mMatrix;

		// Row1
		mMatrix._11 = Det( _22, _23,
						   _32, _33 );
		mMatrix._12 = -Det( _12, _13,
						   _32, _33 );
		mMatrix._13 = Det( _12, _13,
						   _22, _23 );
		// Row2
		mMatrix._21 = -Det( _21, _23,
						   _31, _33 );
		mMatrix._22 = Det( _11, _13,
						   _31, _33 );
		mMatrix._23 = -Det( _11, _13,
						   _21, _23 );
		// Row2
		mMatrix._31 = Det( _21, _22,
						   _31, _32 );
		mMatrix._32 = -Det( _11, _12,
						   _31, _32 );
		mMatrix._33 = Det( _11, _12,
						   _21, _22 );

		_11 = det*mMatrix._11;
		_12 = det*mMatrix._12;
		_13 = det*mMatrix._13;

		_21 = det*mMatrix._21;
		_22 = det*mMatrix._22;
		_23 = det*mMatrix._23;

		_31 = det*mMatrix._31;
		_32 = det*mMatrix._32;
		_33 = det*mMatrix._33;
	}

	//
	//
	//
	bool Matrix33f::operator==( const Matrix33f &rhs )
	{
		if( _11==rhs._11 && _12==rhs._12 && _13==rhs._13 &&
			_21==rhs._21 && _22==rhs._22 && _23==rhs._23 &&
			_31==rhs._31 && _32==rhs._32 && _33==rhs._33 )
			return true;
		else
			return false; 
	}

	//
	//
	//
	bool Matrix33f::operator!=( const Matrix33f &rhs )
	{
		return !((*this)==rhs);
	}
	
	//
	//
	//
	bool Matrix33f::operator+=( const Matrix33f &rhs )
	{
		_11+=rhs._11;
		_12+=rhs._12;
		_13+=rhs._13;

		_21+=rhs._21;
		_22+=rhs._22;
		_23+=rhs._23;

		_31+=rhs._31;
		_32+=rhs._32;
		_33+=rhs._33;

		return true;
	}

	//
	//
	//
	bool Matrix33f::operator-=( const Matrix33f &rhs )
	{
		_11-=rhs._11;
		_12-=rhs._12;
		_13-=rhs._13;

		_21-=rhs._21;
		_22-=rhs._22;
		_23-=rhs._23;

		_31-=rhs._31;
		_32-=rhs._32;
		_33-=rhs._33;

		return true;
	}

	//
	//
	//
	bool Matrix33f::operator+=( const float &rhs )
	{
		_11+=rhs;
		_12+=rhs;
		_13+=rhs;

		_21+=rhs;
		_22+=rhs;
		_23+=rhs;

		_31+=rhs;
		_32+=rhs;
		_33+=rhs;
		return true;
	}

	//
	//
	//
	bool Matrix33f::operator-=( const float &rhs )
	{
		_11-=rhs;
		_12-=rhs;
		_13-=rhs;

		_21-=rhs;
		_22-=rhs;
		_23-=rhs;

		_31-=rhs;
		_32-=rhs;
		_33-=rhs;
		return true;
	}

	//
	//
	//
	bool Matrix33f::operator*=( const float &rhs )
	{
		_11*=rhs;
		_12*=rhs;
		_13*=rhs;

		_21*=rhs;
		_22*=rhs;
		_23*=rhs;

		_31*=rhs;
		_32*=rhs;
		_33*=rhs;
		return true;
	}


	//
	//
	//
	bool Matrix33f::operator/=( const float &rhs )
	{
		_11/=rhs;
		_12/=rhs;
		_13/=rhs;

		_21/=rhs;
		_22/=rhs;
		_23/=rhs;

		_31/=rhs;
		_32/=rhs;
		_33/=rhs;
		return true;
	}
}