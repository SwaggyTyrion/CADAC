///////////////////////////////////////////////////////////////////////////////
//FILE: 'flat0_modules.cpp'
//
//Contains all modules of class 'Flat0'
//							kinematics()  flat0[0-9]
//							newton()	  flat0[10-19]
//
//Created 170916 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Defining of 'kinematics' module-variables
//Member function of class 'Flat0'
//Module-variable locations are assigned to flat0[0-9]
//
//Initializing the module-variables
//		
//Created 170916 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat0::def_kinematics()
{
	//defining module-variables
	flat0[0].init("time",0,"Vehicle time since launch - s","kinematics","out","com");
	flat0[1].init("launch_delay",0,"Delay of vehicle launch since sim start - s","kinematics","data","");
	flat0[2].init("launch_epoch",0,"'sim_time' at launch - s","kinematics","init","");
	flat0[3].init("launch_time",0,"Elapsed time since launch - s","kinematics","out","");
}
///////////////////////////////////////////////////////////////////////////////
//Initial calculations of 'kinematics' module 
//Member function of class 'Flat0'
// 
//Timing functions
//
//Created 170916 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat0::init_kinematics(double sim_time,double not_used,double &launch_dly)
{
	//local module-variables
	double time(0);
	double launch_epoch(0);

	//localising module-variables
	double launch_delay=flat0[1].real();
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//launch epoch
	launch_epoch=launch_delay;

	//loading launch delay value into argument list
	launch_dly=launch_delay;
	//-------------------------------------------------------------------------
	//loading module-variables
	flat0[0].gets(time);
	flat0[2].gets(launch_epoch);
}
///////////////////////////////////////////////////////////////////////////////
//'kinematic' module
//Member function of class 'Flat0'
//
//Timing functions
//
//Created 170916 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat0::kinematics(double sim_time,double not_used1,double &not_used2,double &not_used3,Packet *combus,int num_vehicles,int vehicle_slot)
{
	//local module-variables
	double time(0);
	double launch_time(0);

	//initialization
	double launch_epoch=flat0[2].real();
	//-------------------------------------------------------------------------
	//running launch time clock
	launch_time=sim_time-launch_epoch;

	//setting vehicle time to simulation time
	time=sim_time;
	//-------------------------------------------------------------------------
	//loading module-variables
	flat0[0].gets(time);
	flat0[3].gets(launch_time);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'newton' module-variables 
//Member function of class 'Flat0'
//Module-variable locations are assigned to flat0[10-19]
// 
//Initializing variables for the Newton Module.
//
//Created 170916 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat0::def_newton()
{
	//Definition of module-variables
	flat0[11].init("srel1",0,"Radar north position - m","newton","data","");
	flat0[12].init("srel2",0,"Radar east position - m","newton","data","");
	flat0[13].init("srel3",0,"Radar down position - m","newton","data","");
	flat0[14].init("SREL",0,0,0,"Radar position in local level coor - m","newton","out","");
}
///////////////////////////////////////////////////////////////////////////////
//Initial calculations of 'newton' module 
//Member function of class 'Flat0'
// 
//Initial calculations.
//
//Created 170916 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat0::init_newton()
{
	//local module-variables
	Matrix SREL(3,1);

	//localizing module-variables
	double srel1=flat0[11].real();
	double srel2=flat0[12].real();
	double srel3=flat0[13].real();
	//-------------------------------------------------------------------------
	SREL.build_vec3(srel1,srel2,srel3);
	//-------------------------------------------------------------------------
	//loading module-variables
	flat0[14].gets_vec(SREL);
}
///////////////////////////////////////////////////////////////////////////////
//'newton' module
//Member function of class 'Flat0'
//
// *** no dynamics ***
//
//Created 170916 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat0::newton(double int_step)
{
}
