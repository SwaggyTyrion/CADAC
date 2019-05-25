///////////////////////////////////////////////////////////////////////////////
//FILE: 'propulsion.cpp'
//
//Contains 'propulsion' module of class 'Missile'
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of propulsion module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[50-99]
// 
//Defining and initializing module-variables
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_propulsion()
{
	//Definition and initialization of module-variables
    missile[50].init("mprop","int",0,"=0: Motor off, =1:Motor on","propulsion","data","");
    missile[51].init("aexit",0,"Nozzle exit area - m^2","propulsion","data","");
    missile[59].init("vmass",92,"Vehicle mass - kg","propulsion","out","plot");
    missile[63].init("thrust",0,"Rocket thrust parallel to vehicle centerline - N","propulsion","out","plot");
    missile[64].init("xcgref",1.536,"Launch CG aft of vehicle nose - m","propulsion","init","");
    missile[70].init("xcg",1.536,"Vehicle CG aft of vehicle nose - m","propulsion","diag","");
    missile[71].init("ai11",0.308,"Roll moment of inertia - kg*m^2","propulsion","out","plot");
    missile[72].init("ai33",59.80,"Pitch/Yaw moment of inertia - kg*m^2","propulsion","out","plot");
    missile[73].init("mfreeze_prop","int",0,"Saving mfreeze variable","propulsion","save","");
    missile[74].init("thrustf",0,"Saved thrust when mfreez=1 - N","propulsion","save","");
    missile[75].init("vmassf",0,"Saved mass when mfreez=1 - kg","propulsion","save","");
    missile[76].init("xcgf",0,"Saved cg when mfreez=1 - m","propulsion","save","");
    missile[77].init("ai11f",0,"Saved ai11 when mfreez=1 - kgm^2","propulsion","save","");
    missile[78].init("ai33f",0,"Saved ai33 when mfreez=1 - kgm^2","propulsion","save","");
}	

///////////////////////////////////////////////////////////////////////////////
//Propulsion module
//Member function of class 'Missile'
// Calculates missile mass properties
// Calculates rocket thrust at altitude
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::propulsion()
{
	//local variables
	double psl(101325); //sea level pressure - Pa
	
	//local module-variables
	double thrust(0);
	//localizing module-variables
	//input data
	int mprop=missile[50].integer();
	double aexit=missile[51].real();
	//getting saved values
	double vmass=missile[59].real();
	double xcg=missile[70].real();
	double ai11=missile[71].real();
	double ai33=missile[72].real();
	int mfreeze_prop=missile[73].integer();
	double thrustf=missile[74].real();
	double vmassf=missile[75].real();
	double xcgf=missile[76].real();
	double ai11f=missile[77].real();
	double ai33f=missile[78].real();
	//from other modules
	int mfreeze=missile[501].integer();
	double time=flat6[0].real();
	double press=flat6[52].real();
	//-------------------------------------------------------------------------
	if(mprop==1)
	{
		//thrust compensated by back pressure
		double tsl=proptable.look_up("thrust_vs_time",time);
		thrust=tsl+(psl-press)*aexit;

		//mass of missile
		vmass=proptable.look_up("mass_vs_time",time);

		//c.g. location
		xcg=proptable.look_up("cg_vs_time",time);

		//yaw (=pitch) moment of inertia
		ai33=proptable.look_up("moipitch_vs_time",time);

		//roll moment of inertia
		ai11=proptable.look_up("moiroll_vs_time",time);

		if(time>2.69)mprop=0;
	}
	else
		thrust=0;

	//freeze variables for autopilot response calculations
	if(mfreeze==0)
		mfreeze_prop=0;
	else{
		if(mfreeze!=mfreeze_prop){
			mfreeze_prop=mfreeze;
			thrustf=thrust;
			vmassf=vmass;
			xcgf=xcg;
			ai11f=ai11;
			ai33f=ai33;
		}
		thrust=thrustf;
		vmass=vmassf;
		xcg=xcgf;
		ai11=ai11f;
		ai33=ai33f;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[50].gets(mprop);
	missile[63].gets(thrust);
	//output and save values
	missile[59].gets(vmass);
	missile[70].gets(xcg);
	missile[71].gets(ai11);
	missile[72].gets(ai33);
	missile[73].gets(mfreeze_prop);
	missile[74].gets(thrustf);
	missile[75].gets(vmassf);
	missile[76].gets(xcgf);
	missile[77].gets(ai11f);
	missile[78].gets(ai33f);
}	
