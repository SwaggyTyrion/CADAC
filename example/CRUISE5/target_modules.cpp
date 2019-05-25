///////////////////////////////////////////////////////////////////////////////
//FILE: 'target_modules.cpp'
//Contains all Modules of class 'Target'
//						forces()
//						intercept()
//
//010205 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of force module-variables
//Member function of class 'Target'
//Module-variable locations are assigned to target[10-14]
//
//Note that FSPV is entered into the round3[10] array because it is needed
//for the newton module, which is a member of the 'Round3' class
//		
//010205 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::def_forces()
{
	//Definition of module-variables
	round3[10].init("FSPV",0,0,0,"Specific force in V-coord - m/s^2","forces","out","");
	target[11].init("fwd_accel",0,"Forward acceleration - m/s^2","forces","data","");
	target[12].init("side_accel",0,"Sideward acceleration - m/s^2","forces","data","");
	target[13].init("CORIO_V",0,"Coriolis acceleration - m/s^2","forces","diag","");
	target[14].init("CENTR_V",0,"Centrifugal acceleration - m/s^2","forces","diag","");
}

///////////////////////////////////////////////////////////////////////////////
//Force Module 
//Member function of class 'Target' 
//Provides the centrifugal and Coriolis accelerations of the target in order for it
// to move on the (rotating) Earth
//
//010205 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::forces()
{
	//local variables
	double fspv1(0);
	double fspv2(0);
	double fspv3(0);
	Matrix GRAV_G(3,1);
	Matrix GRAV_V(3,1);
	Matrix ACC_V(3,1);
	Matrix TVG(3,3);
	Matrix TGI(3,3);
	Matrix TEG(3,3);
	Matrix WEIG(3,3);

	//local module-variables
	Matrix CORIO_V(3,1);
	Matrix CENTR_V(3,1);
	Matrix FSPV(3,1);

	//localizing module-variables
	//input data
	double fwd_accel=target[11].real();
	double side_accel=target[12].real();
	//input from other modules
	double grav=round3[11].real();
	double thtvgx=round3[29].real();
	Matrix TGV=round3[22].mat();
	Matrix TIG=round3[23].mat();
	Matrix TGE=round3[33].mat();
	Matrix WEII=round3[27].mat();
	Matrix VBEG=round3[32].vec();
	Matrix SBII=round3[35].vec();
	//-------------------------------------------------------------------------
	//transposing
	TVG=TGV.trans();
	TGI=TIG.trans();
	TEG=TGE.trans();
	
	//Coriolis acceleration in V-coordinates (note; WEIE=WEII)
	WEIG=TGE*WEII*TEG;
	CORIO_V=TVG*WEIG*VBEG*(2);

	//centrifugal acceleration in V-coordinates
	CENTR_V=TVG*WEIG*WEIG*TGI*SBII;

	//gravitational acceleration in V-coordiantes
	GRAV_G.assign_loc(2,0,grav);
	GRAV_V=TVG*GRAV_G;

	//adding apparant and gravitational accelerations
	ACC_V=CORIO_V+CENTR_V-GRAV_V;

	//combining with commanded accelerations
	fspv1=ACC_V.get_loc(0,0)+fwd_accel;
	fspv2=ACC_V.get_loc(1,0)+side_accel;
	fspv3=ACC_V.get_loc(2,0);
	
	FSPV.assign_loc(0,0,fspv1);
	FSPV.assign_loc(1,0,fspv2);
	FSPV.assign_loc(2,0,fspv3);
	//-------------------------------------------------------------------------
	//loading module-variables
	//ouput to other modules
	round3[10].gets_vec(FSPV);
	//diagnostics
	target[13].gets_vec(CORIO_V);
	target[14].gets_vec(CENTR_V);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of intercept module-variables
//Member function of class 'Target'
//Module-variable locations are assigned to cruise[15]
//		
//010420 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::def_intercept()
{
	//definition of module-variables
	target[15].init("targ_health","int",0,"Target health status - ND","intercept","diag","");
}
///////////////////////////////////////////////////////////////////////////////
//Intercept module
//Member function of class 'Target'
//Copies health of target from 'combus' to module-variable 'targ_health' so it can be used 
//in 'event' blocks of 'input.asc'
//
//status =	1: alive
//			0: dead
//		   -1: hit (but not dead)
//
//010420 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::intercept(Packet *combus,int vehicle_slot,double int_step,char *title)
{
	//local module-variables
	int targ_health(0);
	//-------------------------------------------------------------------------
	//get target health from 'combus'
	targ_health=combus[vehicle_slot].get_status();
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	target[15].gets(targ_health);
}

