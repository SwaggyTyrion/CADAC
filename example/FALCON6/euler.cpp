///////////////////////////////////////////////////////////////////////////////
//FILE: 'euler.cpp'
//Contains 'euler' module of class 'Flat6'
//
//030424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'euler' module-variables 
//Member function of class 'Flat6'
//Module-variable locations are assigned to flat6[150-199]
// 
//030424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::def_euler()
{
	//Definition and initialization of module-variables
    flat6[160].init("ppx",0,"Body roll angular vel wrt Earth in body axes - deg/s","euler","init/out","plot");
    flat6[161].init("qqx",0,"Body pitch angular vel wrt Earth in body axes - deg/s","euler","init/out","plot");
    flat6[162].init("rrx",0,"Body yaw angular vel wrt Earth in body axes - deg/s","euler","init/out","plot");
    flat6[164].init("WBEB",0,0,0,"Ang vel of veh wrt inertial frame, body axes - rad/s","euler","state","");
    flat6[165].init("WBEBD",0,0,0,"Ang vel of veh wrt inertl frame, deriv - rad/s^2","euler","state","");
}

///////////////////////////////////////////////////////////////////////////////
//Euler initialization module
//Member function of class 'Flat6'
//
//030424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::init_euler()
{
	//local module-variables
	Matrix WBEB(3,1);

	//localizing module-variables
	//input data
	double ppx=flat6[160].real();
	double qqx=flat6[161].real();
	double rrx=flat6[162].real();
	//-------------------------------------------------------------------------
	//body rate wrt Earth frame in body coordinates
	WBEB.build_vec3(ppx*RAD,qqx*RAD,rrx*RAD);

	//-------------------------------------------------------------------------
	//loading module-variables
	//initialization
	flat6[164].gets_vec(WBEB);
}
///////////////////////////////////////////////////////////////////////////////
//Euler module
//Member function of class 'Flat6'
//
//030424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::euler(double int_step)
{
	//local variable 
	Matrix L_ENGINE(3,1);

	//local module-variables
	double ppx(0),qqx(0),rrx(0);

	//localizing module-variables
	//input from other modules
	Matrix FMB=flat6[201].vec();
	Matrix IBBB=plane[191].mat();
	double eng_ang_mom=plane[192].real();
	//state variable
	Matrix WBEB=flat6[164].vec();
	Matrix WBEBD=flat6[165].vec();
	//-------------------------------------------------------------------------
	//forming turbojet angular momentum vector
	L_ENGINE.build_vec3(eng_ang_mom,0,0);

	//integrating the angular velocity acc wrt the inertial frame in body coord
	Matrix WACC_NEXT=IBBB.inverse()*(FMB-WBEB.skew_sym()*(IBBB*WBEB+L_ENGINE));
	WBEB=integrate(WACC_NEXT,WBEBD,WBEB,int_step);
	WBEBD=WACC_NEXT;

	//body rates in deg/s
	ppx=WBEB.get_loc(0,0)*DEG;
	qqx=WBEB.get_loc(1,0)*DEG;
	rrx=WBEB.get_loc(2,0)*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	flat6[164].gets_vec(WBEB);	
	flat6[165].gets_vec(WBEBD); 	
	//output to other modules
	flat6[160].gets(ppx);	
	flat6[161].gets(qqx);	
	flat6[162].gets(rrx);
}