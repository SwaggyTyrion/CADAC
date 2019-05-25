///////////////////////////////////////////////////////////////////////////////
//FILE: 'environment.cpp'
//Contains 'environment' module of class 'Flat6'
//
//011126 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//020513 Modified for DRMDR6, PZi
//030319 Upgraded to SM Item32, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of environment module-variables 
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
     flat6[56].init("vmach",0,"Mach number of missile","environment","out","scrn,plot,com");
     flat6[57].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","plot");
	 flat6[58].init("tempk",0,"Atmospheric temperature - K","environment","out","");
	 flat6[59].init("mfreeze_environ","int",0,"Saving 'mfreeze' value","environment","save","");
	 flat6[60].init("pdynmcf",0,"Saved dyn.press. when mfreeze=1 - Pa","environment","save","");
	 flat6[61].init("vmachf",0,"Saved Mach# when mfreeze=1 - Pa","environment","save","");
}	

///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Flat6'
//
//US 1976 Standard Atmosphere
// Calculates the atmospheric properties
// Calculates the gravitational acceleration
// Calculates the missile's Mach number
//
//011127 Created by Peter Zipfel
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
	double vmach(0);
	double pdynmc(0);

	//localizing module-variables
	//getting saved values
	int mfreeze_environ=flat6[59].integer();
	double pdynmcf=flat6[60].real(); 
	double vmachf=flat6[61].real(); 
	//input from other modules
	double dvbe=flat6[236].real(); 
	double hbe=flat6[239].real(); 
	double trcode=missile[180].real();
	double trmach=missile[183].real();
	double trdynm=missile[184].real();
	int mguid=missile[400].integer();
	int mfreeze=missile[501].integer();
	//-------------------------------------------------------------------------
	//altitude above earth center
	double rad=REARTH+hbe;

	//calculating the gravity acceleration
	grav=G*EARTH_MASS/pow(rad,2);

	//US 1976 Standard Atmosphere
	atmosphere76(rho,press,tempk, hbe);

	//speed of sound
	vsound=sqrt(1.4*R*tempk);

	//missile mach number
	vmach=fabs(dvbe/vsound);

	//missile dynamic pressure
	pdynmc=0.5*rho*dvbe*dvbe;

	//termination coditions
	if(mguid==6){
	   if(vmach<=trmach) trcode=2.;
	   if(pdynmc<=trdynm) trcode=3.;
	   }

	//freezing variables for autopilot response calculations
	if(mfreeze==0)
		mfreeze_environ=0;
	else{
		if(mfreeze!=mfreeze_environ){
			mfreeze_environ=mfreeze;
			vmachf=vmach;
			pdynmcf=pdynmc;
		}
		vmach=vmachf;
		pdynmc=pdynmcf;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	flat6[59].gets(mfreeze_environ);
	flat6[60].gets(pdynmcf);
	flat6[61].gets(vmachf);
	//output to other modules
	flat6[52].gets(press);
	flat6[53].gets(rho);
	flat6[55].gets(grav);
	flat6[56].gets(vmach);
	flat6[57].gets(pdynmc);
	flat6[58].gets(tempk);
	//diagnostics
	flat6[54].gets(vsound);
}	
