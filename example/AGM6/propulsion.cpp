///////////////////////////////////////////////////////////////////////////////
//FILE: 'propulsion.cpp'
//
//Contains 'propulsion' module of class 'Missile'
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'propulsion' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[10-49]
// 
//Defining and initializing module-variables
//
//		mrpop = 0 no thrusting
//				1 motor on
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::def_propulsion()
{
	//Definition and initialization of module-variables
    missile[10].init("mprop","int",0,"=0: Off;=1:On;=2 2nd pulse;=3 Input","propulsion","data","");
    missile[11].init("aexit",0,"Nozzle exit area - m^2","propulsion","data","");
    missile[12].init("vmass",0,"Vehicle vmass - kg","propulsion","out","scrn,plot");
    missile[13].init("thrust",0,"Rocket thrust parallel to vehicle centerline - N","propulsion","out","scrn,plot");
    missile[14].init("vmass0",0,"Initial mass - kg","propulsion","data","");
    missile[16].init("ai11",0,"Roll moment of inertia - kg*m^2","propulsion","out","");
    missile[17].init("ai33",0,"Pitch/Yaw moment of inertia - kg*m^2","propulsion","out","");
    missile[18].init("mfreeze_prop","int",0,"Saving mfreeze variable","propulsion","save","");
    missile[19].init("thrustf",0,"Saved thrust when mfreez=1 - N","propulsion","save","");
    missile[20].init("vmassf",0,"Saved mass when mfreez=1 - kg","propulsion","save","");
    missile[21].init("spi",0,"specific impulse of motor - sec","propulsion","data","");
    missile[22].init("throtl",0,"Throttle setting (0-1) - ND","propulsion","data","");
	missile[26].init("thrsl",0,"Thrust at sea level- N","propulsion","data","");
	missile[27].init("fmass0",0,"Initial fuel mass - kg","propulsion","data","");
	missile[28].init("fmasse",0,"Fuel mass expended - kg","propulsion","state","plot");
	missile[29].init("fmassed",0,"Fuel mass expended derivative - kg","propulsion","state","");
}	
///////////////////////////////////////////////////////////////////////////////
//'propulsion' initialization module
//Member function of class 'Missile'
// 
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::init_propulsion()
{
	//local module-variables
//	double fmass(0);
	double vmass(0);

	//localizing module-variables
	//input data
	double vmass0=missile[14].real();
	//-------------------------------------------------------------------------
	//initializing state variable
	vmass=vmass0;
	//-------------------------------------------------------------------------
	missile[12].gets(vmass);
}
///////////////////////////////////////////////////////////////////////////////
//'propulsion' module
//Member function of class 'Missile'
// Calculates missile mass
// Calculates rocket thrust at altitude
//
//		mrpop = 0 no thrusting
//				1 motor on
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::propulsion(double int_step)
{
	//local variables
	double psl(101325); //sea level pressure - Pa
	
	//local module-variables
	double thrust(0);

	//localizing module-variables
	//input data
	int mprop=missile[10].integer();
	double aexit=missile[11].real();
	double vmass=missile[12].real();
	double vmass0=missile[14].real();
	double spi=missile[21].real();
	double throtl=missile[22].real();
	double thrsl=missile[26].real();
	double fmass0=missile[27].real();
	//getting saved values
	int mfreeze_prop=missile[18].integer();
	double thrustf=missile[19].real();
	double vmassf=missile[20].real();
	//state variables
	double fmasse=missile[28].real();
	double fmassed=missile[29].real();
	//input from other modules
	int mfreeze = missile[501].integer();
	double press=flat6[52].real();
	//-------------------------------------------------------------------------
	//boost phase
	if(mprop==1){
		double fmassed_new=thrsl*throtl/(spi*9.81);
		fmasse=integrate(fmassed_new,fmassed,fmasse,int_step);
		fmassed=fmassed_new;
		vmass=vmass0-fmasse;
		thrust=thrsl*throtl+(psl-press)*aexit;
	}else{
		thrust=0;
	}
	//stop motor if fuel is expended
	if(fmasse>=fmass0)
		mprop=0;
		
	//freeze thrust and mass for autopilot response calculations
	if(mfreeze==0)
		mfreeze_prop=0;
	else{
		if(mfreeze!=mfreeze_prop){
			mfreeze_prop=mfreeze;
			thrustf=thrust;
			vmassf=vmass;
		}
		thrust=thrustf;
		vmass=vmassf;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[28].gets(fmasse);
	missile[29].gets(fmassed);

	//output to other modules
	missile[10].gets(mprop);
	missile[13].gets(thrust);
	//output and saving values
	missile[12].gets(vmass);
	missile[18].gets(mfreeze_prop);
	missile[19].gets(thrustf);
	missile[20].gets(vmassf);
}	
