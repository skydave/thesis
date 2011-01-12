/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "Vec3d.h"


#include "Math.h"



namespace math
{
	//
	//
	//
	Vec3d::Vec3d()
	{
        x=0.0f; y=0.0f; z=0.0f;
    }

    //
	//
	//
	Vec3d::Vec3d( const double &x, const double &y, const double &z )
    {
        this->x=x; this->y=y; this->z=z;
    }

	//
	//
	//
	Vec3d::~Vec3d( )
	{
	}

	//
	//
	//
	void Vec3d::set( const double &x, const double &y, const double &z )
	{
		this->x=x; this->y=y; this->z=z;
	}

	//
	//
	//
	double Vec3d::getLength( void ) const
	{
        return sqrt( x*x + y*y + z*z );
	}

	double Vec3d::getSquaredLength( void ) const
	{
		return x*x + y*y + z*z;
	}

	//
	//
	//
	void Vec3d::setLength( const double &length )
	{
		normalize();

		x*=length;
		y*=length;
		z*=length;
	}

	//
	//
	//
	void Vec3d::normalize( void )
	{
		double length = getLength();

		if( length != 0.0f )
		{
			x /= length;
			y /= length;
			z /= length;	
		}
	}

	//
	//
	//
	void Vec3d::negate( void )
	{
		x=-x; y=-y; z=-z;
	}

	//
	// reflects the vector at the given normal
	//
	void Vec3d::reflect( const Vec3d &normal )
	{
		normalize();

		Vec3d	temp( x, y, z );
		double	value;
		
		value = dotProduct( normal, *this );
		value *= 2.0f;

		x = normal.x * value;
		y = normal.y * value;
		z = normal.z * value;

		x -= temp.x;
		y -= temp.y;
		z -= temp.z;
	}

	//
	//
	//
	bool Vec3d::operator==( const Vec3d &rhs )
	{
		if( (fabs(x - rhs.x) < 0.00001f) && (fabs(y - rhs.y) < 0.00001f) && (fabs(z - rhs.z) < 0.00001f) )
			return true;
		else
			return false;
	}

	//
	//
	//
	bool Vec3d::operator!=( const Vec3d &rhs )
	{
		return !((*this)==rhs);
	}
	
	//
	//
	//
	bool Vec3d::operator+=( const Vec3d &rhs )
	{
		x+=rhs.x;
		y+=rhs.y;
		z+=rhs.z;

		return true;
	}

	//
	//
	//
	bool Vec3d::operator-=( const Vec3d &rhs )
	{
		x-=rhs.x;
		y-=rhs.y;
		z-=rhs.z;

		return true;
	}

	//
	//
	//
	bool Vec3d::operator+=( const double &rhs )
	{
		x+=rhs;
		y+=rhs;
		z+=rhs;

		return true;
	}

	//
	//
	//
	bool Vec3d::operator-=( const double &rhs )
	{
		x-=rhs;
		y-=rhs;
		z-=rhs;

		return true; 
	}

	//
	//
	//
	bool Vec3d::operator*=( const double &rhs )
	{
		x*=rhs;
		y*=rhs;
		z*=rhs;

		return true;
	}

	//
	//
	//
	bool Vec3d::operator/=( const double &rhs )
	{
		x/=rhs;
		y/=rhs;
		z/=rhs;

		return true; 
	}

	//
	//
	//
	const double& Vec3d::operator[]( int i ) const
	{
		return v[i];
	}

	//
	//
	//
	double& Vec3d::operator[]( int i )
	{
		return v[i];
	}
}