/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "Matrix22f.h"
#include "Math.h"


namespace math
{
	//
	//
	//
	Matrix22f::Matrix22f()
	{
		_11=_12=
		_21=_22=0.0f;
	}

	Matrix22f::Matrix22f( const float &_11, const float &_12,
                          const float &_21, const float &_22 )
	{
        this->_11=_11; this->_12=_12;
        this->_21=_21; this->_22=_22;
	}

	//
	//
	//
	Matrix22f::~Matrix22f()
	{
	}

	//
	// returns the zeromatrix
	//
	Matrix22f Matrix22f::Zero( void )
	{
		return Matrix22f( 0.0f, 0.0f,
			              0.0f, 0.0f);
	}

	//
	// returns the identitymatrix
	//
	Matrix22f Matrix22f::Identity( void )
	{
		return Matrix22f( 1.0f, 0.0f,
			              0.0f, 1.0f);
	}

	//
	// returns a matrix which defines a rotation with the float-specified amount (in radians)
	//
	Matrix22f Matrix22f::RotationMatrix( const float &angle )
	{
		float fSin = sinf( angle );
		float fCos = cosf( angle );
		return Matrix22f( fCos, -fSin,
			              fSin, fCos );
	}

	//
	//
	//
	void Matrix22f::transpose( void )
	{
		Matrix22f temp = *this;

		_11 = temp._11;
		_12 = temp._21;

		_21 = temp._12;
		_22 = temp._22;
	}


	void Matrix22f::invert( void )
	{
		// a b
		// c d
		//Then the inverse is 1/(ad-cb) * d -b-c a
		//float c = 1.0f/( _11*_22 - _21*_12 );
		//*this = Matrix22f( c*_22, -c*_12, -c*_21, c*_11 );
	}

	//
	//
	//
	bool Matrix22f::operator==( const Matrix22f &rhs )
	{
		if( _11==rhs._11 && _12==rhs._12 &&
			_21==rhs._21 && _22==rhs._22 )
			return true;
		else
			return false; 
	}

	//
	//
	//
	bool Matrix22f::operator!=( const Matrix22f &rhs )
	{
		return !((*this)==rhs);
	}
	
	//
	//
	//
	bool Matrix22f::operator+=( const Matrix22f &rhs )
	{
		_11+=rhs._11;
		_12+=rhs._12;


		_21+=rhs._21;
		_22+=rhs._22;

		return true;
	}

	//
	//
	//
	bool Matrix22f::operator-=( const Matrix22f &rhs )
	{
		_11-=rhs._11;
		_12-=rhs._12;


		_21-=rhs._21;
		_22-=rhs._22;

		return true;
	}

	//
	//
	//
	bool Matrix22f::operator+=( const float &rhs )
	{
		_11+=rhs;
		_12+=rhs;


		_21+=rhs;
		_22+=rhs;

		return true;
	}

	//
	//
	//
	bool Matrix22f::operator-=( const float &rhs )
	{
		_11-=rhs;
		_12-=rhs;

		_21-=rhs;
		_22-=rhs;

		return true;
	}

	//
	//
	//
	bool Matrix22f::operator*=( const float &rhs )
	{
		_11*=rhs;
		_12*=rhs;

		_21*=rhs;
		_22*=rhs;

		return true;
	}


	//
	//
	//
	bool Matrix22f::operator/=( const float &rhs )
	{
		_11/=rhs;
		_12/=rhs;

		_21/=rhs;
		_22/=rhs;

		return true;
	}
}