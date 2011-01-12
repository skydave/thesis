/*---------------------------------------------------------------------

This simulator derives from FluidSimulator2d and overloads the
updateVelocity method. There are some small additions which will
make the fluid behave like sand.
The idea and theory behind that method was introduced in the paper
"Animating Sand as a Fluid" at Siggraph in 2005 by Zhou/Bridson.

----------------------------------------------------------------------*/
#pragma once

#include "FluidSimulator2d.h"






///
/// \brief Simulator which simulates granular media as a fluid after zhou2005
///
class SandSimulator2d : public FluidSimulator2d
{
public:
	SandSimulator2d();                                        ///< constructor
private:
	virtual void                  updateVelocity( float dt ); ///< This method is a modified version of the FluidSimulator2d::updateVelocity method to reflect sand behavior
	void                          applySandModel( float dt ); ///< performs some changes to the velocity field so that the fluid behaves like sand

	virtual void               setBoundaryConditions( void ); ///< this method will set the velocities on solid cells which border non-solid cells so that coloumb friction is reflected in the model

	void findGroup( Cell *cell, std::vector<Cell *> &group ); ///< will add all cells which can reach the given cell through neighbourhood accesses to the groupvector
	void              rigidify( std::vector<Cell *> &group ); ///< will project the velocities of all cells within the group into the space of the groupd rigid body motion

	math::Matrix22f    computeStrainRateTensor( Cell *cell ); ///< this will compute the rate of strain tensor D for the given cell
	math::Matrix22f                       test( Cell *cell ); ///< this will compute the rate of strain tensor D for the given cell
	math::Vec2f                    computeDivE( Cell *cell );
};