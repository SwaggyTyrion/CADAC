///////////////////////////////////////////////////////////////////////////////
//FILE: 'propulsion.cpp'
//
//Contains 'propulsion' module of class 'Plane'
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of propulsion module-variables 
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[50-99]
// 
//Defining and initializing module-variables
//
//030722 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::def_propulsion()
{
	//Definition and initialization of module-variables
    plane[50].init("mprop","int",0,"=0: off,=1: manual throttle,=2: Mach hold","propulsion","data","");
    plane[51].init("vmachcom",0,"Commanded Mach # - ND","propulsion","data","");
    plane[52].init("throttle",0,"Throttle setting (0->1) ND","propulsion","data","scrn,plot");
    plane[53].init("gmach",0,"Gain conversion from Mach to throttle - ND","propulsion","data","");
    plane[54].init("thrustf",0,"Saved thrust when 'mfeeze'=1 - N","propulsion","save","");
    plane[55].init("thrust",0,"Turbojet thrust - N","propulsion","out","scrn,plot");
    plane[56].init("thrust_req",0,"Required thrust - N","propulsion","diag","");
    plane[57].init("mfreeze_prop","int",0,"Saving m'mfreeze' value - ND","propulsion","save","");
    plane[58].init("powerd",0,"Derivative of achieved power setting - %/sec","propulsion","state","");
    plane[59].init("power",0,"Achieved power setting - %","propulsion","state","");
    plane[60].init("power_com",0,"Commanded power setting - %","propulsion","diag","");
    plane[61].init("tpower",0,"Spool-up time constant - sec","propulsion","diag","");
    plane[62].init("idle",0,"Idle thrust from tables - N","propulsion","diag","plot");
    plane[63].init("mil",0,"Military thrust from tables - N","propulsion","diag","plot");
    plane[64].init("max",0,"Maximum thrust from tables - N","propulsion","diag","plot");
}	

///////////////////////////////////////////////////////////////////////////////
//Propulsion module
//Member function of class 'Plane'
//
//Single turbofan engine with afterburner
//References: (1) Stevens & Lewis, "Aircraft Control and
//Simulation" Wiley-Interscience Publication, 1992.
//Data based on: (2) Nguyen, J.T., et al., "Simulator Study of
//Stall/Post-Stall Characteristics of a Fighter Airplane with
//Relaxec Longitudinal Static Stability", NASA Tech. Paper 1538,
//NASA, Washington, D.C. Dec. 1979.
//
//This module performs the following functions:
//(1) Calculates the thrust of the F16 aircraft as a function of
//    throttle setting, altitude, and Mach. Includes
//    a lag filter. Throttle setting is limited to 0.77 (military power)
//(2) Implements a Mach hold controller
//(3) Tables are in file 'f16_prop_deck.asc'
//(4) Freezes thrust for autopilot analysis
//Note: Effect of fuel consumption on vehicle mass is neglected
//
//030722 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::propulsion(double int_step)
{
	//local module-variables
	double thrust(0);
	double thrust_req(0);

	//localizing module-variables
	//input data
	int mprop=plane[50].integer();
	double vmachcom=plane[51].real();
	double throttle=plane[52].real();
	double gmach=plane[53].real();
	//getting saved values
	double thrustf=plane[54].real();
	int mfreeze_prop=plane[57].integer();
	//from other modules
	double time=flat6[0].real();
	double vmach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double alphax=flat6[144].real();
	double refa=plane[104].real();
	double cdrag=plane[157].real();
	int mfreeze=plane[503].integer();
	//-------------------------------------------------------------------------
	if(mprop==1){
		//direct throttle input
		thrust=propulsion_thrust(throttle,int_step);
	}
	else if(mprop==2){

		//holding Mach number
		double emach=vmachcom-vmach;
		throttle=gmach*emach;
		if(throttle <0) throttle=0;
		if(throttle>0.77)throttle=0.77;
		thrust=propulsion_thrust(throttle,int_step);

		//diagnostic: thrust required in body coord
		thrust_req=cdrag*pdynmc*refa/cos(alphax*RAD);
	}
	else
		thrust=0;

	//freezing thrust for autopilot response calculations
	if(mfreeze==0)
		mfreeze_prop=0;
	else{
		if(mfreeze!=mfreeze_prop){
			mfreeze_prop=mfreeze;
			thrustf=thrust;
			}
		thrust=thrustf;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	plane[54].gets(thrustf);
	plane[57].gets(mfreeze_prop);
	//output to other modules
	plane[55].gets(thrust);
	//diagnostics
	plane[56].gets(thrust_req);
	plane[52].gets(throttle);
}
	
///////////////////////////////////////////////////////////////////////////////
//Turbojet engine thrust function
//Member function of class 'Plane'
//
// Calculates the thrust of the F16 aircraft as a function of
//  throttle setting, altitude, and Mach.
// Includes a time lag to represent spooling transients.
// Tables are in English units. If 'OPTMET=1' thrust is in Newton
//
// Return output
//          thrust= Turbojet thrust - N
// Parameter input:
//          throttle= Throttle 0 -> 1
//
//030722 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::propulsion_thrust(double throttle,double int_step)
{
	//local variables
	double thrust(0);
	
	//local module-variables
	double power_com(0);
	double tpower(0);
	double idle(0);
	double mil(0);
	double max(0);

	//localizing module-variables
	//input data
	//state variables
	double powerd=plane[58].real();
	double power=plane[59].real();
	//from other modules
	double time=flat6[0].real();
	double vmach=flat6[56].real();
	double hbe=flat6[239].real();
	//-------------------------------------------------------------------------
	//calculating commanded power from throttle setting
	if(throttle<=0.77)
		power_com=64.94*throttle;
	else
		power_com=217.38*throttle-117.38;

	//determining the spooling time constant
	if(power_com<=50)
		tpower=1;
	else
		tpower=0.2;

	//delaying achieved power by first order time constant
	double powerd_new=(power_com-power)/tpower;
	power=integrate(powerd_new,powerd,power,int_step);
	powerd=powerd_new;

	//getting thrust from look-up tables (lbf converted to N)
	//input to the tables is in Mach and altitude - ft
	double hbe_ft=hbe*FOOT;
	//idle thrust
	double idle_lb=proptable.look_up("idle_vs_mach_alt",vmach,hbe_ft);
	idle=idle_lb*NT;
	//military thrust
	double mil_lb=proptable.look_up("mil_vs_mach_alt",vmach,hbe_ft);
	mil=mil_lb*NT;
	//maximum thrust
	double max_lb=proptable.look_up("max_vs_mach_alt",vmach,hbe_ft);
	max=max_lb*NT;

	//calculating actual thrust
	if(power<50)
		thrust=idle+power*0.02*(mil-idle);
	else
		thrust=mil+(power-50)*0.02*(max-mil);

	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	plane[58].gets(powerd);
	plane[59].gets(power);
	//diagnostics
	plane[60].gets(power_com);
	plane[61].gets(tpower);
	plane[62].gets(idle);
	plane[63].gets(mil);
	plane[64].gets(max);

	return thrust;
}
