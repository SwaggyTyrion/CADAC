///////////////////////////////////////////////////////////////////////////////
//FILE: 'satellite_modules.cpp'
//Contains all modules of class 'Satellite'
//	only module 'forces()' needed
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

//////////////////////////////////////////////////////////////////////////////
//Definition of force module-variables
//Member function of class 'Satellite'
//Module-variable locations are assigned to satellite[10-19]
//
//Note (1) FSPV is entered into the round3[21] array because it is needed
//          for the 'newton' module, which is a member of the 'Round3' class
//	   (2) 'sat_mass' is initialized to 100kg to prevent divide by zero, otherwise
//			it has no effect on the satellite trajectory. Only if thrust is applied
//			must 'sat_mass' be set to the actual value 
//		
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Satellite::def_forces()
{
	//Definition of module-variables
	round3[21].init("FSPV",0,0,0,"Specific force in V-coord - m/s^2","forces","out","");
	satellite[14].init("sat_thrust",0,"Satellite thrust - N","forces","data","");
	satellite[15].init("sat_mass",100,"Satellite mass - kg","forces","data","");
}

///////////////////////////////////////////////////////////////////////////////
//Force Module 
//Member function of class 'Satellite' 
//Includes thrust force as specific force
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Satellite::forces()
{
	//localizing module-variables
	Matrix FSPV=round3[10].vec();
	double sat_thrust=satellite[14].real();
	double sat_mass=satellite[15].real();
	//-------------------------------------------------------------------------
	//Specific force, thrust pointing in direction of 1V-axis
	FSPV.assign_loc(0,0,sat_thrust/sat_mass);
	//-------------------------------------------------------------------------
	//loading module-variables
	round3[21].gets_vec(FSPV);
}
