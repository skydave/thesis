/*---------------------------------------------------------------------

The particle viewer displays sequences of particles from a simulation

----------------------------------------------------------------------*/
#pragma once
#include <windows.h>
#include <string>

#include "SimulationStepData.h"



///
/// \brief the par5ticle viewer is used by the viewer project to load and display particle sets
///
class ParticleViewer
{
public:
	void                                                                render(); // renders the particles out with opengl

	void                               assignFile( const std::string &filename ); // assigns the file to get the data from


private:
	SimulationStepData                                                  stepData; // holds information of current simulationstep
};
