///////////////////////////////////////////////////////////////////////////////
//FILE: 'environment.cpp'
//Contains 'environment' module of class 'Flat6'
//
//011126 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030319 Upgraded to SM Item32, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'environment' module-variables 
//Member function of class 'Flat6'
//Module-variable locations are assigned to flat6[50-99]
// 
//011126 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat6::def_environment()
{
	//Definition and initialization of module-variables
     flat6[52].init("press",0,"Atmospheric pressure - Pa","environment","out","");
     flat6[53].init("rho",0,"Atmospheric density - kg/m^3","environment","out","");
     flat6[54].init("vsound",0,"Sonic speed - m/sec","environment","diag","");
     flat6[55].init("grav",0,"Gravity acceleration - m/s^2","environment","out","");
     flat6[56].init("mach",0,"Mach number of missile","environment","out","scrn,plot,com");
     flat6[57].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot,com");
	 flat6[58].init("tempk",0,"Atmospheric temperature - K","environment","out","");
	 flat6[59].init("mfreeze_environ","int",0,"Saving 'mfreeze' value","environment","save","");
	 flat6[60].init("pdynmcf",0,"Saved dyn.press. when mfreeze=1 - Pa","environment","save","");
	 flat6[61].init("machf",0,"Saved Mach# when mfreeze=1 - Pa","environment","save","");
}	

///////////////////////////////////////////////////////////////////////////////
//'environment' module
//Member function of class 'Flat6'
//
//US 1976 Standard Atmosphere
// Calculates the atmospheric properties
// Calculates the gravitational acceleration
// Calculates the missile's Mach number and dynamic pressure
//
//011127 Created by Peter H Zipfel
//030319 Upgraded to US76 atmosphere, PZi
///////////////////////////////////////////////////////////////////////////////
void Flat6::environment()
{
	//local variables
	double tempk(0);
	
	//local module-variables
	double grav(0);
	double press(0);
	double rho(0);
	double vsound(0);
	double mach(0);
	double pdynmc(0);

	//localizing module-variables
	//getting saved values
	int mfreeze_environ=flat6[59].integer();
	double pdynmcf=flat6[60].real(); 
	double machf=flat6[61].real(); 
	//input from other modules
	double dvbe=flat6[236].real(); 
	double alt=flat6[239].real(); 
	int trcond=missile[180].integer();
	double trdynm=missile[184].real();
	int mguide=missile[400].integer();
	int mfreeze=missile[501].integer();
	//-------------------------------------------------------------------------
	//altitude above Earth's center
	double rad=REARTH+alt;

	//calculating the gravity acceleration
	grav=G*EARTH_MASS/pow(rad,2);

	//US 1976 Standard Atmosphere
	atmosphere76(rho,press,tempk, alt);

	//speed of sound
	vsound=sqrt(1.4*R*tempk);

	//missile mach number
	mach=fabs(dvbe/vsound);

	//missile dynamic pressure
	pdynmc=0.5*rho*dvbe*dvbe;

	//termination coditions
	//decoding guidance flag
    int guid_term=(mguide%10);
	if(guid_term==6){
	   if(pdynmc<=trdynm)
		   trcond=3;
	   }

	//freezing variables for autopilot response calculations
	if(mfreeze==0)
		mfreeze_environ=0;
	else{
		if(mfreeze!=mfreeze_environ){
			mfreeze_environ=mfreeze;
			machf=mach;
			pdynmcf=pdynmc;
		}
		mach=machf;
		pdynmc=pdynmcf;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	flat6[59].gets(mfreeze_environ);
	flat6[60].gets(pdynmcf);
	flat6[61].gets(machf);
	//output to other modules
	flat6[52].gets(press);
	flat6[53].gets(rho);
	flat6[55].gets(grav);
	flat6[56].gets(mach);
	flat6[57].gets(pdynmc);
	flat6[58].gets(tempk);
	//diagnostics
	flat6[54].gets(vsound);
	missile[180].gets(trcond);
}	
