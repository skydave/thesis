/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#include "DebugNavigator.h"
#include "Camera.h"






namespace dk
{

	//
	// constructor
	//
	DebugNavigator::DebugNavigator()
	{

		// add properties
		//...

		// init local stuff
		//lookAt = math::Vec3f( -0.5f, -0.5f, -0.5f );
		//lookAt = math::Vec3f( 0.5f, 0.5f, 0.5f );
		lookAt = math::Vec3f( 0.0f, 0.0f, 0.0f );
		azimuth = elevation = 0.0f;
		distance = .5f;

		camera = 0;
	}

	//
	// specifiy the camer which has to be modified with this manipulator
	//
	void DebugNavigator::setCamera( Camera *cam )
	{
		camera = cam;
		

		update();
	}

	//
	// returns the distance to the lookat point
	//
	float DebugNavigator::getDistance( void ) const
	{
		return distance;
	}

	//
	//
	//
	void DebugNavigator::update( void )
	{
		// compute the final transformation from lookat and camera polar coordinates
		// and write the result to the writeOnlyReference
		math::Matrix44f m = math::Matrix44f::Identity();


		m.translate( lookAt );

		m.rotateY( math::degToRad(azimuth) );
		m.rotateX( math::degToRad(elevation) );
		//m.rotateZ( twist ); // not used

		m.translate( math::Vec3f( 0.0f, 0.0f, distance ) );



		if( camera )
		{
			camera->transform = m;

			camera->update();
		}
	}

	//
	// moves the camera on the view plane
	//
	void DebugNavigator::panView( float x, float y )
	{
		lookAt += (-x*distance*0.01f)*camera->transform.getRight();
		lookAt += (-y*distance*0.01f)*camera->transform.getUp();

		update();
	}

	//
	// rotates the camera around the origin
	//
	void DebugNavigator::orbitView( float azimuthDelta, float elevationDelta )
	{
		azimuth   += azimuthDelta;
		elevation += elevationDelta;

		update();
	}

	//
	// zoomes the camera in or out (depending on the sign of the amount parameter)
	//
	void DebugNavigator::zoomView( float distanceDelta )
	{
		distance += distanceDelta;

		update();
	}

	//
	//
	//
	void DebugNavigator::setLookAt( float x, float y, float z )
	{
		lookAt = math::Vec3f( x, y, z );
		update();
	}

	//
	//
	//
	void DebugNavigator::setLookAt( math::Vec3f newLookAt )
	{
		lookAt = newLookAt;
		update();
	}
}
