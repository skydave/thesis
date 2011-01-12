/*---------------------------------------------------------------------

This simulator derives from FluidSimulator2d and overloads the
updateVelocity method. There are some small additions which will
make the fluid behave like a viscoelastic fluid.
The idea and theory behind that method was introduced in the paper
"A Method for Animating Viscoelastic Fluids "
at Siggraph in 2004 by Goktekin

----------------------------------------------------------------------*/
#pragma once

#include "FluidSimulator3d.h"





///
/// \brief Simulator which simulates viscoelastic fluids after goktekin2004
///
class ViscoElasticFluid3d : public FluidSimulator3d
{
public:
	ViscoElasticFluid3d();                                                                        ///< constructor

	void                                               setElasticModulus( float elasticModulus ); ///< sets the elastic modulus which drives the material behaviour
	float                                                                    getElasticModulus(); ///< returns the elastic modulus which drives the material behaviour
	void                                                  setPlasticYieldRate( float yieldRate ); ///< sets the plastic yield rate which determines how fast the material yields plastic

private:
	virtual void                                                      updateVelocity( float dt ); ///< This method is a modified version of the FluidSimulator2d::updateVelocity method to reflect sand behavior

	math::Matrix44f                                        computeStrainRateTensor( Cell *cell ); ///< this will compute the rate of strain tensor D for the given cell
	math::Vec3f                                                        computeDivE( Cell *cell );

	float getInterpolatedStrainTensorComponent( float x, float y, float z, int row, int column );

	//void                      sweepStrain( int iStart, int iEnd, int jStart, int jEnd );
	void                                                               extrapolateStrain( void ); ///< extrapolate strain from fluid into air cells

	// additional parameters for viscoelastic fluids
	float                                                                         elasticModulus;
	float                                                                      elasticYieldPoint;
	float                                                                       plasticYieldRate;

};