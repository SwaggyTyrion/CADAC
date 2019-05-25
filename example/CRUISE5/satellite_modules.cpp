///////////////////////////////////////////////////////////////////////////////
//FILE: 'satellite_modules.cpp'
//Contains all Modules of class 'Satellite'
//						seeker()
//
//010811 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////
//Definition of force module-variables
//Member function of class 'Satellite'
//
//Note that FSPV is entered into the round3[10] array because it is needed
//for the 'newton' module, which is a member of the 'Round3' class
//		
//010812 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Satellite::def_forces()
{
	//Definition of module-variables
	round3[10].init("FSPV",0,0,0,"Specific force in V-coord - m/s^2","forces","out","");
	satellite[4].init("sat_thrust",0,"Satellite thrust - N","forces","data","");
	satellite[5].init("sat_mass",100,"Satellite mass - kg","forces","data","");
}

///////////////////////////////////////////////////////////////////////////////
//Force Module 
//Member function of class 'Satellite' 
//Includes thrust force as specific force
//
//010812 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Satellite::forces()
{
	//local module variable
	Matrix FSPV(3,1);

	//localizing module-variables
	//input data
	double sat_thrust=satellite[4].real();
	double sat_mass=satellite[5].real();
	//-------------------------------------------------------------------------
	//Specific force, thrust pointing in direction of 1V-axis
	FSPV.assign_loc(0,0,sat_thrust/sat_mass);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	round3[10].gets_vec(FSPV);
}
