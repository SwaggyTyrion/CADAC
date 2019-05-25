///////////////////////////////////////////////////////////////////////////////
//FILE: 'propulsion.cpp'
//
//Contains 'propulsion' module of class 'Missile'
//
//170821 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'propulsion' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[10-49]
// 
//170821 Created by Peter H Zipfel///////////////////////////////////////////////////////////////////////////////
void Missile::def_propulsion()
{
	//Definition and initialization of module-variables
    missile[10].init("mprop","int",0,"=0: off;=1:on - ND","propulsion","out","");
    missile[11].init("aexit",0.0314,"Nozzle exit area - m^2","propulsion","data","");
    missile[12].init("mass",300,"Vehicle mass - kg","propulsion","out","plot,scrn");
    missile[13].init("thrust",0,"Rocket thrust - N","propulsion","out","scrn,plot");
    missile[14].init("xcgref",0,"Vehicle reference CG aft of vehicle nose - m","propulsion","data","");
    missile[15].init("xcg",2.9,"Vehicle CG aft of vehicle nose - m","propulsion","diag","scrn,plot");
    missile[16].init("ai11",2.9,"Roll moment of inertia - kg*m^2","propulsion","out","plot");
    missile[17].init("ai33",440,"Pitch/Yaw moment of inertia - kg*m^2","propulsion","out","plot");
    missile[18].init("mfreeze_prop","int",0,"Saving mfreeze variable","propulsion","save","");
    missile[19].init("thrustf",0,"Saved thrust when mfreez=1 - N","propulsion","save","");
    missile[20].init("massf",0,"Saved mass when mfreez=1 - kg","propulsion","save","");
    missile[21].init("xcgf",0,"Saved cg when mfreez=1 - m","propulsion","save","");
    missile[22].init("ai11f",0,"Saved ai11 when mfreez=1 - kgm^2","propulsion","save","");
    missile[23].init("ai33f",0,"Saved ai33 when mfreez=1 - kgm^2","propulsion","save","");
}	
///////////////////////////////////////////////////////////////////////////////
//'propulsion' module
//Member function of class 'Missile'
// Calculates missile mass properties
// Calculates rocket thrust at altitude
//
//170821 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::propulsion()
{
	//local variables
	double psl(101325); //sea level pressure - Pa
	
	//local module-variables
	double thrust(0);
	int mprop(0);

	//localizing module-variables
	//input data
	double aexit=missile[11].real();
	//getting saved values
	double mass=missile[12].real();
	double xcg=missile[15].real();
	double ai11=missile[16].real();
	double ai33=missile[17].real();
	int mfreeze_prop=missile[18].integer();
	double thrustf=missile[19].real();
	double massf=missile[20].real();
	double xcgf=missile[21].real();
	double ai11f=missile[22].real();
	double ai33f=missile[23].real();
	//input from other modules
	double msl_time=flat6[10].real();
	double press=flat6[52].real();
	int mfreeze=missile[501].integer();
	//-------------------------------------------------------------------------
	//thrust compensated by back pressure
	double tsl=proptable.look_up("thrust_vs_time",msl_time);
	thrust=tsl+(psl-press)*aexit;

	//mass of missile
	mass=proptable.look_up("mass_vs_time",msl_time);

	//c.g. location
	xcg=proptable.look_up("cg_vs_time",msl_time);

	//yaw (=pitch) moment of inertia
	ai33=proptable.look_up("moipitch_vs_time",msl_time);

	//roll moment of inertia
	ai11=proptable.look_up("moiroll_vs_time",msl_time);

	//setting the propulsion flag
	if(msl_time<=60)
		mprop=1;
	else
		mprop=0;

	//freezing variables for autopilot response calculations
	if(mfreeze==0)
		mfreeze_prop=0;
	else{
		if(mfreeze!=mfreeze_prop){
			mfreeze_prop=mfreeze;
			thrustf=thrust;
			massf=mass;
			xcgf=xcg;
			ai11f=ai11;
			ai33f=ai33;
		}
		thrust=thrustf;
		mass=massf;
		xcg=xcgf;
		ai11=ai11f;
		ai33=ai33f;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[10].gets(mprop);
	missile[13].gets(thrust);
	//output and saving values
	missile[12].gets(mass);
	missile[15].gets(xcg);
	missile[16].gets(ai11);
	missile[17].gets(ai33);
	missile[18].gets(mfreeze_prop);
	missile[19].gets(thrustf);
	missile[20].gets(massf);
	missile[21].gets(xcgf);
	missile[22].gets(ai11f);
	missile[23].gets(ai33f);
}	
