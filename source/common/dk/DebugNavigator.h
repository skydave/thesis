/*---------------------------------------------------------------------



----------------------------------------------------------------------*/
#pragma once
#include "math/Math.h"

namespace dk
{
	class Camera;

	///
	/// \brief is used to manipulate the camera from mouse movements and enables the user to control the camera like in a dcc-app
	///
	class DebugNavigator
	{
	public:

		//
		// constructor
		//
		DebugNavigator();

		//
		//
		//
		virtual void                                update( void );  ///< recomputes the transform from current polar coordinates and distance value

		void                           panView( float x, float y );  ///< moves the camera on the view plane
		void orbitView( float azimuthDelta, float elevationDelta );  ///< rotates the camera around the origin (angles in degree)
		void                       zoomView( float distanceDelta );  ///< zoomes the camera in or out (depending on the sign of the amount parameter)
		void                setLookAt( float x, float y, float z );  ///< sets the point to look at - change this to move the camera around
		void                    setLookAt( math::Vec3f newLookAt );  ///< returns the distance of the camera to the lookat-point

		float                            getDistance( void ) const;

		void                              setCamera( Camera *cam );  ///< specifiy the camer which has to be modified with this manipulator



		math::Matrix44f                                   transform;

		math::Vec3f                                         lookAt;
		float                                   azimuth, elevation;  // polar coordinates
		float                                             distance;

		Camera                                             *camera;
	};
}
