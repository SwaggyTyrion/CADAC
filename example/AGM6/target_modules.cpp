///////////////////////////////////////////////////////////////////////////////
//FILE: 'target_modules.cpp'
//
//Contains all modules of class 'Target'
//						forces()		target[10-19]
//
// generally used variables are assigned to target[0-9] 
//
//100413 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'force' module-variables
//Member function of class 'Target'
//Module-variable locations are assigned to target[10-19]
//
//Note that FSPA is entered into the 'flat3[20]' array because it is needed
// for the 'newton' module, which is a member of the 'Flat3' class
//
//100413 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Target::def_forces()
{
	//Definition of module-variables
	flat3[20].init("FSPA",0,0,0,"Specific force in vehicle coor - m/s^2","forces","out","");
	target[10].init("aax",0,"Axial acceleration of target - g's","forces","diag","com");
	target[11].init("alx",0,"Yaw acceleration of target - g's","forces","diag","com");
	target[12].init("anx",0,"Pitch acceleration of target - g's","forces","diag","com");
	target[13].init("acc_longx",0,"Longitudinal acceleration of target - g's","forces","data","");
	target[14].init("acc_latx",0,"Lateral acceleration of target - g's","forces","data","");
}
///////////////////////////////////////////////////////////////////////////////
//'force' module 
//Member function of class 'Target' 
//Calculates forces acting on the target
//
//100413 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Target::forces()
{
	//local module-variables
	Matrix FSPA(3,1);
	double aax(0);
	double alx(0);
	double anx(0);

	//localizing module-variables
	//input data
	double acc_longx=target[13].real();
	double acc_latx=target[14].real();
	//input from other modules
	double grav=flat3[11].real();
	//-------------------------------------------------------------------------
	FSPA[0]=acc_longx*grav;
	FSPA[1]=acc_latx*grav;
	FSPA[2]=-grav;

	//diagnostics: accelerations in target body coord
	aax=FSPA[0]/grav;
	alx=FSPA[1]/grav;
	anx=-FSPA[2]/grav;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	flat3[20].gets_vec(FSPA);
	//diagnostics
	target[10].gets(aax);
	target[11].gets(alx);
	target[12].gets(anx);
}
