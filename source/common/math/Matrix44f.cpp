/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "Matrix44f.h"

#include "Math.h"


namespace math
{
	//
	//
	//
	Matrix44f::Matrix44f()
	{
		_11=_12=_13=_14=
		_21=_22=_23=_24=
		_31=_32=_33=_34=
		_41=_42=_43=_44=0.0f;
	}

	Matrix44f::Matrix44f( const float &_11, const float &_12, const float &_13, const float &_14,
                          const float &_21, const float &_22, const float &_23, const float &_24,
                          const float &_31, const float &_32, const float &_33, const float &_34,
                          const float &_41, const float &_42, const float &_43, const float &_44 )
	{
        this->_11=_11; this->_12=_12; this->_13=_13; this->_14=_14;
        this->_21=_21; this->_22=_22; this->_23=_23; this->_24=_24;
        this->_31=_31; this->_32=_32; this->_33=_33; this->_34=_34;
        this->_41=_41; this->_42=_42; this->_43=_43; this->_44=_44;
	}

	//
	// creates the Matrix from 3 rows
	//
	Matrix44f::Matrix44f( const Vec3f &right, const Vec3f &up, const Vec3f &forward )
	{
		this->_11=right.x; this->_12=up.x; this->_13=forward.x; this->_14=0.0f;
        this->_21=right.y; this->_22=up.y; this->_23=forward.y; this->_24=0.0f;
        this->_31=right.z; this->_32=up.z; this->_33=forward.z; this->_34=0.0f;
        this->_41=0.0f; this->_42=0.0f; this->_43=0.0f; this->_44=1.0f;
	}


	//
	//
	//
	Matrix44f::~Matrix44f()
	{
	}

	//
	// returns the zeromatrix
	//
	Matrix44f Matrix44f::Zero( void )
	{
		return Matrix44f( 0.0f, 0.0f, 0.0f, 0.0f,
			              0.0f, 0.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 0.0f, 0.0f);
	}

	//
	// returns the identitymatrix
	//
	Matrix44f Matrix44f::Identity( void )
	{
		return Matrix44f( 1.0f, 0.0f, 0.0f, 0.0f,
			              0.0f, 1.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 1.0f, 0.0f,
						  0.0f, 0.0f, 0.0f, 1.0f);
	}

	//
	// returns a matrix which defines a rotation about the x axis with the float-specified amount
	//
	Matrix44f Matrix44f::RotationMatrixX( const float &angle )
	{
		float fSin = sinf( angle );
		float fCos = cosf( angle );

		return Matrix44f( 1.0f, 0.0f, 0.0f, 0.0f,
						  0.0f, fCos, -fSin, 0.0f,
						  0.0f, fSin, fCos, 0.0f,
						  0.0f, 0.0f, 0.0f, 1.0f);
	}

	//
	// returns a matrix which defines a rotation about the y axis with the float-specified amount
	//
	Matrix44f Matrix44f::RotationMatrixY( const float &angle )
	{
		float fSin = sinf( angle );
		float fCos = cosf( angle );

		return Matrix44f( fCos, 0.0f, fSin, 0.0f,
			              0.0f, 1.0f, 0.0f, 0.0f,
						  -fSin, 0.0f, fCos, 0.0f,
						  0.0f, 0.0f, 0.0f, 1.0f);
	}

	//
	// returns a matrix which defines a rotation about the z axis with the float-specified amount
	//
	Matrix44f Matrix44f::RotationMatrixZ( const float &angle )
	{
		float fSin = sinf( angle );
		float fCos = cosf( angle );
		return Matrix44f( fCos, -fSin, 0.0f, 0.0f,
			              fSin, fCos, 0.0f, 0.0f,
						  0.0f, 0.0f, 1.0f, 0.0f,
						  0.0f, 0.0f, 0.0f, 1.0f);
	}

	//
	// returns a matrix with a transformation that rotates around
	// a certain axis which starts at the origin
	//
	Matrix44f Matrix44f::RotationMatrix( const Vec3f &axis, const float &angle )
	{
		// code from Graphics Gems (Glassner, Academic Press, 1990)
		float c = cos( angle );
		float t = 1.0f - c;
		float s = sin( angle );
		float txy = t*axis.x*axis.y;
		float txz = t*axis.x*axis.z;
		float tyz = t*axis.y*axis.z;
		float sx = s*axis.x;
		float sy = s*axis.y;
		float sz = s*axis.z;
		return Matrix44f( t*axis.x*axis.x+c, txy+sz, txz-sy, 0.0f,
			              txy-sz, t*axis.y*axis.y+c, tyz+sx, 0.0f,
						  txz+sy, tyz-sx, t*axis.z*axis.z+c, 0.0f,
						  0.0f, 0.0f, 0.0f, 1.0f);
	}

	//
	// returns a matrix which defines a translation of the specified translation vector
	//
	Matrix44f Matrix44f::TranslationMatrix( const Vec3f &translation )
	{
		return Matrix44f( 1.0f, 0.0f, 0.0f, 0.0f,
			              0.0f, 1.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 1.0f, 0.0f,
						  translation.x, translation.y, translation.z, 1.0f);		
	}

	//
	// returns a matrix which defines a translation of the specified translation vector
	//
	Matrix44f Matrix44f::TranslationMatrix( const float &x, const float &y, const float &z )
	{
		return Matrix44f( 1.0f, 0.0f, 0.0f, 0.0f,
			              0.0f, 1.0f, 0.0f, 0.0f,
						  0.0f, 0.0f, 1.0f, 0.0f,
						  x, y, z, 1.0f);		
	}

	//
	// returns a matrix which defines a uniform scale
	//
	Matrix44f Matrix44f::ScaleMatrix( const float &uniformScale )
	{
		return Matrix44f( uniformScale, 0.0f, 0.0f, 0.0f,
			              0.0f, uniformScale, 0.0f, 0.0f,
						  0.0f, 0.0f, uniformScale, 0.0f,
						  0.0f, 0.0f, 0.0f, 1.0f);
	}

	//
	// returns a matrix which defines a non-uniform scale
	//
	Matrix44f Matrix44f::ScaleMatrix( const float &x, const float &y, const float &z )
	{
		return Matrix44f( x, 0.0f, 0.0f, 0.0f,
			              0.0f, y, 0.0f, 0.0f,
						  0.0f, 0.0f, z, 0.0f,
						  0.0f, 0.0f, 0.0f, 1.0f);
	}

	//
	//
	//
	void Matrix44f::rotateX( const float &angle )
	{
		//*this = *this * RotationMatrixX( angle );
		*this = RotationMatrixX( angle ) * *this;
	}

	//
	//
	//
	void Matrix44f::rotateY( const float &angle )
	{
		//*this = *this * RotationMatrixY( angle );
		*this = RotationMatrixY( angle ) * *this;
	}

	//
	//
	//
	void Matrix44f::rotateZ( const float &angle )
	{
		//*this = *this * RotationMatrixZ( angle );
		*this = RotationMatrixZ( angle ) * *this;
	}

	//
	//
	//
	void Matrix44f::translate( const Vec3f &translation )
	{
		//*this = *this * TranslationMatrix( translation );
		*this = TranslationMatrix( translation ) * *this;
	}

	//
	//
	//
	void Matrix44f::translate( const float &x, const float &y, const float &z )
	{
		//*this = *this * TranslationMatrix( x, y, z ) ;
		*this = TranslationMatrix( x, y, z ) * *this;
	}

	//
	// scales the current transform
	//
	void Matrix44f::scale( const float &uniformScale )
	{
		*this = ScaleMatrix( uniformScale ) * *this;
	}

	//
	// scales the current transform non-uniformly
	//
	void Matrix44f::scale( const float &x, const float &y, const float &z )
	{
		*this = ScaleMatrix( x, y, z ) * *this;
	}

	//
	//
	//
	void Matrix44f::transpose( void )
	{
		Matrix44f temp = *this;

		_12 = temp._21;
		_13 = temp._31;
		_14 = temp._41;

		_21 = temp._12;
		_23 = temp._32;
		_24 = temp._42;

		_31 = temp._13;
		_32 = temp._23;
		_34 = temp._43;

		_41 = temp._14;
		_42 = temp._24;
		_43 = temp._34;
	}

	//
	// computes the determinant of a 3x3 matrix after the rule of Sarrus
	//
	inline float Det(	float &f_11, float &f_12, float &f_13,
						float &f_21, float &f_22, float &f_23,
						float &f_31, float &f_32, float &f_33 )
	{
		return f_11*f_22*f_33 + f_21*f_32*f_13 + f_31*f_12*f_23 - f_13*f_22*f_31 - f_23*f_32*f_11 - f_33*f_12*f_21;
	}


	//
	//
	//
	void Matrix44f::invert( void )
	{
		float fDet =	_11*Det(	_22, _23, _24,
									_32, _33, _34,
									_42, _43, _44 ) -

						_12*Det(	_21, _23, _24,
									_31, _33, _34,
									_41, _43, _44 ) +

						_13*Det(	_21, _22, _24,
									_31, _32, _34,
									_41, _42, _44 ) -

						_14*Det(	_21, _22, _23,
									_31, _32, _33,
									_41, _42, _43 );

		// determinant must be not zero
		if( fabsf( fDet ) < 0.00001f )
			// error determinant is zero
			return;

		fDet = 1.0f / fDet;

		Matrix44f mMatrix;

		// Row1
		mMatrix._11 =		   Det(	_22, _23, _24,
									_32, _33, _34,
									_42, _43, _44 );

		mMatrix._12 = -1.0f * Det(	_21, _23, _24,
									_31, _33, _34,
									_41, _43, _44 );

		mMatrix._13 =		   Det(	_21, _22, _24,
									_31, _32, _34,
									_41, _42, _44 );

		mMatrix._14 = -1.0f * Det(	_21, _22, _23,
									_31, _32, _33,
									_41, _42, _43 );


		// Row2
		mMatrix._21 = -1.0f * Det(	_12, _13, _14,
									_32, _33, _34,
									_42, _43, _44 );

		mMatrix._22 =		   Det(	_11, _13, _14,
									_31, _33, _34,
									_41, _43, _44 );

		mMatrix._23 = -1.0f * Det(	_11, _12, _14,
									_31, _32, _34,
									_41, _42, _44 );

		mMatrix._24 =		   Det(	_11, _12, _13,
									_31, _32, _33,
									_41, _42, _43 );


		// Row3
		mMatrix._31 =		   Det(	_12, _13, _14,
									_22, _23, _24,
									_42, _43, _44 );

		mMatrix._32 = -1.0f * Det(	_11, _13, _14,
									_21, _23, _24,
									_41, _43, _44 );

		mMatrix._33 =		   Det(	_11, _12, _14,
									_21, _22, _24,
									_41, _42, _44 );

		mMatrix._34 = -1.0f * Det(	_11, _12, _13,
									_21, _22, _23,
									_41, _42, _43 );


		// Row4
		mMatrix._41 = -1.0f * Det(	_12, _13, _14,
									_22, _23, _24,
									_32, _33, _34 );

		mMatrix._42 =		   Det(	_11, _13, _14,
									_21, _23, _24,
									_31, _33, _34 );

		mMatrix._43 = -1.0f * Det(	_11, _12, _14,
									_21, _22, _24,
									_31, _32, _34 );

		mMatrix._44 =		   Det(	_11, _12, _13,
									_21, _22, _23,
									_31, _32, _33 );

		_11 = fDet*mMatrix._11;
		_12 = fDet*mMatrix._21;
		_13 = fDet*mMatrix._31;
		_14 = fDet*mMatrix._41;

		_21 = fDet*mMatrix._12;
		_22 = fDet*mMatrix._22;
		_23 = fDet*mMatrix._32;
		_24 = fDet*mMatrix._42;

		_31 = fDet*mMatrix._13;
		_32 = fDet*mMatrix._23;
		_33 = fDet*mMatrix._33;
		_34 = fDet*mMatrix._43;

		_41 = fDet*mMatrix._14;
		_42 = fDet*mMatrix._24;
		_43 = fDet*mMatrix._34;
		_44 = fDet*mMatrix._44;
	}

	//
	//
	//
	Vec3f Matrix44f::getRight( const bool &normalized )
	{
		Vec3f right( _11, _12, _13 );

		if( normalized )
			right.normalize();

		return right;
	}

	//
	//
	//
	Vec3f Matrix44f::getUp( const bool &normalized )
	{
		Vec3f up( _21, _22, _23 );

		if( normalized )
			up.normalize();

		return up;
	}

	//
	//
	//
	Vec3f Matrix44f::getDir( const bool &normalized )
	{
		Vec3f dir( _31, _32, _33 );

		if( normalized )
			dir.normalize();

		return dir; 
	}

	//
	//
	//
	Vec3f Matrix44f::getTranslation( void )
	{
		return Vec3f( _41, _42, _43 );
	}

	//
	//
	//
	Matrix44f Matrix44f::getOrientation( void )
	{
		return Matrix44f( _11, _12, _13, 0.0f,
                          _21, _22, _23, 0.0f,
                          _31, _32, _33, 0.0f,
                           0.0f, 0.0f, 0.0f, 1.0f );
	}

	Matrix44f Matrix44f::getTransposed( void )
	{
		return Matrix44f( _11, _21, _31, _41,
                          _12, _22, _32, _42,
                          _13, _23, _33, _43,
                          _14, _24, _34, _44 );
	}

	//
	//
	//
	Matrix44f Matrix44f::getNormalizedOrientation( void )
	{
		static Vec3f g_vRight;
		static Vec3f g_vUp;
		static Vec3f g_vDir;

		g_vRight	= getRight();
		g_vUp		= getUp();
		g_vDir		= getDir();

		return Matrix44f(	g_vRight.x, g_vRight.y, g_vRight.z, 0.0f,
					    	g_vUp.x,	g_vUp.y, g_vUp.z, 0.0f,
					    	g_vDir.x,	g_vDir.y, g_vDir.z, 0.0f,
						    0.0f, 0.0f, 0.0f, 1.0f );
	}

	//
	//
	//
	void Matrix44f::setRight( const Vec3f &right )
	{
		_11 = right.x;
		_12 = right.y;
		_13 = right.z;
	}

	//
	//
	//
	void Matrix44f::setUp( const Vec3f &up )
	{
		_21 = up.x;
		_22 = up.y;
		_23 = up.z;
	}

	//
	//
	//
	void Matrix44f::setDir( const Vec3f &dir )
	{
		_31 = dir.x;
		_32 = dir.y;
		_33 = dir.z;
	}

	//
	//
	//
	void Matrix44f::setTranslation( const Vec3f &translation )
	{
		_41 = translation.x;
		_42 = translation.y;
		_43 = translation.z;
	}

	//
	//
	//
	bool Matrix44f::operator==( const Matrix44f &rhs )
	{
		if( _11==rhs._11 && _12==rhs._12 && _13==rhs._13 && _14==rhs._14 &&
			_21==rhs._21 && _22==rhs._22 && _23==rhs._23 && _24==rhs._24 &&
			_31==rhs._31 && _32==rhs._32 && _33==rhs._33 && _34==rhs._34 &&
			_41==rhs._41 && _42==rhs._42 && _43==rhs._43 && _44==rhs._44 )
			return true;
		else
			return false; 
	}

	//
	//
	//
	bool Matrix44f::operator!=( const Matrix44f &rhs )
	{
		return !((*this)==rhs);
	}
	
	//
	//
	//
	bool Matrix44f::operator+=( const Matrix44f &rhs )
	{
		_11+=rhs._11;
		_12+=rhs._12;
		_13+=rhs._13;
		_14+=rhs._14;

		_21+=rhs._21;
		_22+=rhs._22;
		_23+=rhs._23;
		_24+=rhs._24;

		_31+=rhs._31;
		_32+=rhs._32;
		_33+=rhs._33;
		_34+=rhs._34;

		_41+=rhs._41;
		_42+=rhs._42;
		_43+=rhs._43;
		_44+=rhs._44;

		return true;
	}

	//
	//
	//
	bool Matrix44f::operator-=( const Matrix44f &rhs )
	{
		_11-=rhs._11;
		_12-=rhs._12;
		_13-=rhs._13;
		_14-=rhs._14;

		_21-=rhs._21;
		_22-=rhs._22;
		_23-=rhs._23;
		_24-=rhs._24;

		_31-=rhs._31;
		_32-=rhs._32;
		_33-=rhs._33;
		_34-=rhs._34;

		_41-=rhs._41;
		_42-=rhs._42;
		_43-=rhs._43;
		_44-=rhs._44;

		return true;
	}

	//
	//
	//
	bool Matrix44f::operator+=( const float &rhs )
	{
		_11+=rhs;
		_12+=rhs;
		_13+=rhs;
		_14+=rhs;

		_21+=rhs;
		_22+=rhs;
		_23+=rhs;
		_24+=rhs;

		_31+=rhs;
		_32+=rhs;
		_33+=rhs;
		_34+=rhs;

		_41+=rhs;
		_42+=rhs;
		_43+=rhs;
		_44+=rhs;

		return true;
	}

	//
	//
	//
	bool Matrix44f::operator-=( const float &rhs )
	{
		_11-=rhs;
		_12-=rhs;
		_13-=rhs;
		_14-=rhs;

		_21-=rhs;
		_22-=rhs;
		_23-=rhs;
		_24-=rhs;

		_31-=rhs;
		_32-=rhs;
		_33-=rhs;
		_34-=rhs;

		_41-=rhs;
		_42-=rhs;
		_43-=rhs;
		_44-=rhs;

		return true;
	}

	//
	//
	//
	bool Matrix44f::operator*=( const float &rhs )
	{
		_11*=rhs;
		_12*=rhs;
		_13*=rhs;
		_14*=rhs;

		_21*=rhs;
		_22*=rhs;
		_23*=rhs;
		_24*=rhs;

		_31*=rhs;
		_32*=rhs;
		_33*=rhs;
		_34*=rhs;

		_41*=rhs;
		_42*=rhs;
		_43*=rhs;
		_44*=rhs;

		return true;
	}


	//
	//
	//
	bool Matrix44f::operator/=( const float &rhs )
	{
		_11/=rhs;
		_12/=rhs;
		_13/=rhs;
		_14/=rhs;

		_21/=rhs;
		_22/=rhs;
		_23/=rhs;
		_24/=rhs;

		_31/=rhs;
		_32/=rhs;
		_33/=rhs;
		_34/=rhs;

		_41/=rhs;
		_42/=rhs;
		_43/=rhs;
		_44/=rhs;

		return true;
	}
}