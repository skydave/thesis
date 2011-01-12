/*---------------------------------------------------------------------

simple  camera class

----------------------------------------------------------------------*/
#pragma once

#include "math/Math.h"
#include <vector>
#include <algorithm>


///
/// \brief contains a collection of classes which are quite important for most of the projects
///
namespace dk
{
	///
	/// \brief holds a view and a projection matrix which can be computed from a given transform matrix
	///
	class Camera
	{
	public:
		Camera();

		virtual                             void update( void ); ///< will compute the cameras projection matrix

		math::Matrix44f                               transform; ///< the matrix which transforms the camera from local into world space - the inverse of this matrix is the view transform

		math::Matrix44f                        &getViewMatrix(); ///< this method will return the inverse of the camera transform
		math::Matrix44f                  &getProjectionMatrix(); ///< this method will return the projection matrix

		math::Ray3d                  getRay( float x, float y ); ///< returns ray from given normalized screen coordinates
	private:





		float                                           m_znear;
		float                                            m_zfar;
		float                                             m_fov;
		float                                     m_aspectRatio;





		math::Matrix44f                        projectionMatrix;
		math::Matrix44f                              viewMatrix;

	};
}