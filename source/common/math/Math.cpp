/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "Math.h"




namespace math
{
	//
	//
	//
	float radToDeg( float rad )
	{
		return (float) ( (rad * 180.0f) / MATH_PIf );
	}

	//
	//
	//
	float degToRad( float degree )
	{
		return (float) (( degree *  MATH_PIf)/180.0f );
	}

	//
	//
	//
	double radToDeg( double rad )
	{
		return (double) ( (rad * 180.0) / MATH_PI );
	}

	//
	//
	//
	double degToRad( double degree )
	{
		return (double) (( degree *  MATH_PI)/180.0 );
	}


	/*
	//
	//
	//
	bool rayHitPlaneValues( const Vec3f &planeNormal, const float &planeDistance, Ray &ray, float &hitDistance, Vec3f *hitPoint )
	{
        Vec3f rayOrigin    = ray.getOrigin();
		Vec3f rayDirection = ray.getDirection();//ray.getTarget() - rayOrigin;
		//Vec3f rayOrigin    = Vec3f( 0.0f, 10.0f, 0.0f );
		//Vec3f rayDirection = Vec3f( 0.0f, -1.0f, 0.0f );


		float temp = dotProduct( rayDirection, planeNormal );

		// 
		//if( temp >= 0.0f )
		//	return false;

		hitDistance = -(dotProduct( planeNormal, rayOrigin ) + planeDistance) / temp;

		// the point must lie on the raysegment between origin and target to pass the test
		if( (hitDistance > ray.getLength()) || (hitDistance < 0.0f) )
			return false;

		if( hitPoint )
			*hitPoint = rayOrigin + hitDistance*rayDirection;

		return true;
	}
	*/

	//
	// computes a intersection between a ray and a plane
	//
	// returns true if an intersection occured, false otherwise
	//
	bool intersectionRayPlane( const Ray3d &ray, const Vec3f &normal, const float &distance, Vec3f &hitPoint )
	{
		// project the ray direction onto the plane normal
		float temp = dotProduct( ray.direction, normal );

		// if result is zero, then the direction is parallel to the plane -> no intersection
		if( !temp )
			return false;

		float hitDistance = -(dotProduct( normal, ray.origin ) + distance) / temp;

		// the point must lie on the raysegment between origin and target to pass the test
		if( (hitDistance >= ray.length) || (hitDistance <= 0.0f) )
			return false;

		hitPoint = ray.origin + hitDistance*ray.direction;

		return true;
	}

	//
	// computes a intersection between a ray and another ray
	// algorithm based on Graphics Gems I page 304
	//
	// note: the 2 rays must be coplanar
	//
	// returns true if an intersection occured, false otherwise
	//
	bool intersectionRayRay( const Ray3d &ray1, const Ray3d &ray2, Vec3f &hitPoint )
	{
		math::Vec3f cp = math::crossProduct( ray1.direction, ray2.direction );
		float denom = cp.getSquaredLength();

		if( denom == 0.0f )
			// lines are parallel
			return false;

		// we need to compute s and t to test the line segments

		math::Vec3f c1 = ray2.origin - ray1.origin;

		float t = math::Matrix33f( c1.x, ray2.direction.x, cp.x, c1.y, ray2.direction.y, cp.y, c1.z, ray2.direction.z, cp.z ).getDeterminant() / denom;
		float s = math::Matrix33f( c1.x, ray1.direction.x, cp.x, c1.y, ray1.direction.y, cp.y, c1.z, ray1.direction.z, cp.z ).getDeterminant() / denom;


		// check line segments
		if( (t < 0.0f) || (s < 0.0f) || (t > ray1.length) || (s > ray2.length) )
			return false;

		// compute intersection point
		hitPoint = ray2.origin + ray2.direction*s;

		// check for coplanarity
		//if( hitPoint != (ray1.origin + ray1.direction*t) )
		//	return false;

		// done
		return true;
	}






	//
	// computes area of an triangle
	//
	float area( const Vec3f &p0, const Vec3f &p1, const Vec3f &p2 )
	{
		float la = (p1 - p0).getLength(); // compute lengths of the triangle sides
		float lb = (p2 - p1).getLength();
		float lc = (p2 - p0).getLength();
		float s = 0.5f*( la+lb+lc ); // compute the semiperimeter
		return sqrt( s*(s-la)*(s-lb)*(s-lc) ); // compute the area
	}



	//
	// returns the distance of the given point to the line specified by two points
	//
	float distancePointLine( const math::Vec3f &point, const Vec3f &p1, const Vec3f &p2 )
	{
		math::Vec3f vec = point - p1;
		math::Vec3f direction = math::normalize( p2 - p1 );

		return (vec - dotProduct( vec, direction  ) * direction).getLength();
	}


	//
	// computes the distance of a point to a plane
	//
	inline float distancePointPlane( const math::Vec3f &point, const Vec3f &normal, const float &distance )
	{
		return dotProduct( normal, point ) + distance;
	}

	//
	// computes the euclidian distance between 2 points in space
	//
	float distance( const Vec3f &p0, const Vec3f &p1 )
	{
		return (p1-p0).getLength();
	}

	//
	// computes the squared euclidian distance between 2 points in space
	//
	inline float squaredDistance( const Vec3f &p0, const Vec3f &p1 )
	{
		return (p1-p0).getSquaredLength();
	}





	//
	// returns the projection of the given point on the normal and distance specified plane
	//
	math::Vec3f projectPointOnPlane( const math::Vec3f &normal, const float &distance, const math::Vec3f &point )
	{
		return point - distancePointPlane( point, normal, distance )*normal;
	}

	math::Vec3f projectPointOnLine( const math::Vec3f &point, const math::Vec3f &p1, const math::Vec3f &p2 )
	{
		math::Vec3f direction = math::normalize( p2 - p1 );

		return p1 + dotProduct( point - p1, direction  ) * direction;
	}

	//
	// creates the matrix which descripes _only_ the orientation which comes from a
	// lookat constrain
	//
	// upVector must be normalized
	//
	Matrix44f createLookAtMatrix( const Vec3f &position, const Vec3f &target, const Vec3f &_up )
	{
		math::Vec3f forward = normalize( target - position );

		math::Vec3f right = crossProduct( _up, forward );

		math::Vec3f up = crossProduct( forward, right );

		return math::Matrix44f( right, up, forward );
	}

	//
	// creates a ViewMatrix from the given polar coordinates
	//
	Matrix44f createMatrixFromPolarCoordinates( const float &azimuth, const float &elevation, const float &distance )
	{
		math::Matrix44f m = math::Matrix44f::Identity();

		m.translate( math::Vec3f( 0.0f, 0.0f, -distance ) );

		m.rotateX( elevation );
		m.rotateY( azimuth );
		//m.rotateZ( twist );

		return m;


		/*
		m.tr
		vm.Translate( 0.0f , 0.0f , -distance );
		//vm.RotateZ(XMath.Deg2Rad(azimuth)); //TODO
		//vm.RotateX( -XMath.Deg2Rad(elevation) );
		//vm.RotateY( XMath.Deg2Rad(twist) );
		vm.RotateX( XMath.Deg2Rad(elevation) );
		vm.RotateY( XMath.Deg2Rad(twist) );
		vm.RotateZ( XMath.Deg2Rad(azimuth)); //TODO
		vm.Translate( - lx ,- ly , - lz ); 
		*/
		return m;
	}









	//
	//
	//
	float mapValueToRange( const float &sourceRangeMin, const float &sourceRangeMax, const float &targetRangeMin, const float &targetRangeMax, const float &value )
	{
		return (value-sourceRangeMin) / (sourceRangeMax - sourceRangeMin) * (targetRangeMax - targetRangeMin) + targetRangeMin;
	}

	//
	//
	//
	float mapValueTo0_1( const float &sourceRangeMin, const float &sourceRangeMax, const float &value )
	{
		return (value-sourceRangeMin) / (sourceRangeMax - sourceRangeMin);
	}
}