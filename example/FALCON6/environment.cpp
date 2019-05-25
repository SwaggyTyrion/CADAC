///////////////////////////////////////////////////////////////////////////////
//FILE: 'environment.cpp'
//Contains 'environment' module of class 'Flat6'
//
//030626 Adopted from AIMC and wind added, PZi
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
     flat6[50].init("mwind","int",0,"=0: no wind; =1:const; =2:shear","environment","data","");
     flat6[52].init("press",0,"Atmospheric pressure - Pa","environment","out","");
     flat6[53].init("rho",0,"Atmospheric density - kg/m^3","environment","out","");
     flat6[54].init("vsound",0,"Sonic speed - m/sec","environment","diag","");
     flat6[55].init("grav",0,"Gravity acceleration - m/s^2","environment","out","");
     flat6[56].init("vmach",0,"Mach number of plane","environment","out","scrn,plot,com");
     flat6[57].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot");
	 flat6[58].init("tempk",0,"Atmospheric temperature - K","environment","out","");
	 flat6[59].init("mfreeze_environ","int",0,"Saving 'mfreeze' value","environment","save","");
	 flat6[60].init("pdynmcf",0,"Saved dyn.press. when mfreeze=1 - Pa","environment","save","");
	 flat6[61].init("vmachf",0,"Saved Mach# when mfreeze=1 - Pa","environment","save","");
     flat6[64].init("dvae",0,"Magnitude of constant air speed - m/s","environment","data","");
     flat6[65].init("dvael",0,"Air speed at low altitude - m/s","environment","data","");
     flat6[66].init("waltl",0,"Low altitude of air speed - m","environment","data","");
     flat6[67].init("dvaeh",0,"Air speed at high altitude - m/s","environment","data","");
     flat6[68].init("walth",0,"High altitude of air speed - m","environment","data","");
     flat6[69].init("vaed3",0,"Vertical air speed (pos.down) - m/s","environment","data","");
     flat6[70].init("psiwdx",0,"Wind direction from north - m/s","environment","data","");
     flat6[71].init("twind",0.1,"Wind smoothing time constant - sec","environment","data","");
	 flat6[72].init("VAELS",0,0,0,"Smoothed wind vel in local level coord - m/s","environment","state","");
	 flat6[73].init("VAELSD",0,0,0,"Smoothed wind velocity derivative - m/s","environment","state","");
	 flat6[74].init("VAEL",0,0,0,"Wind vel in local level coordinates - m/s","environment","out","");
	 flat6[75].init("dvba",0,"Vehicle speed wrt air - m/s","environment","out","plot");
	 flat6[76].init("VBAL",0,0,0,"Vehicle vel wrt air in local level coord - m/s","environment","out","");
}	

///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Flat6'
//
//US 1976 Standard Atmosphere
// Calculates the atmospheric properties
// Calculates the gravitational acceleration
// Calculates the plane's Mach number
//
//011127 Created by Peter Zipfel
//030319 Upgraded to US76 atmosphere, PZi
///////////////////////////////////////////////////////////////////////////////

void Flat6::environment(double int_step)
{
	//local variables
	double tempk(0);
	double dvw(0);
	
	//local module-variables
	double grav(0);
	double press(0);
	double rho(0);
	double vsound(0);
	double vmach(0);
	double pdynmc(0);
	double dvba(0);
	Matrix VAEL(3,1);
	Matrix VBAL(3,1);

	//localizing module-variables
	//input data
	int mwind=flat6[50].integer();
	double dvae=flat6[64].real(); 
	double dvael=flat6[65].real(); 
	double waltl=flat6[66].real(); 
	double dvaeh=flat6[67].real(); 
	double walth=flat6[68].real(); 
	double vaed3=flat6[69].real(); 
	double psiwdx=flat6[70].real(); 
	double twind=flat6[71].real(); 
	//getting saved values
	int mfreeze_environ=flat6[59].integer();
	double pdynmcf=flat6[60].real(); 
	double vmachf=flat6[61].real(); 
	//state variables
	Matrix VAELS=flat6[72].vec();
	Matrix VAELSD=flat6[73].vec();
	//input from other modules
	Matrix VBEL=flat6[233].vec();
	double hbe=flat6[239].real(); 
	double trcode=plane[180].real();
	double trmach=plane[183].real();
	double trdynm=plane[184].real();
	int mguid=plane[400].integer();
	int mfreeze=plane[503].integer();
	//-------------------------------------------------------------------------
	//altitude above earth center
	double rad=REARTH+hbe;

	//calculating the gravity acceleration
	grav=G*EARTH_MASS/pow(rad,2);

	//US 1976 Standard Atmosphere
	atmosphere76(rho,press,tempk, hbe);

	//speed of sound
	vsound=sqrt(1.4*R*tempk);

	//wind options
	if(mwind>0){
		if(mwind==1)
			//constant wind
			dvw=dvae;

		if(mwind==2){
			//wind with constant shear
			dvw=dvael+(dvaeh-dvael)*(hbe-waltl)/(walth-waltl);
			if(hbe<waltl) dvw=0;
			if(hbe>walth) dvw=0;
		}
		//wind components in local level coordinates
		Matrix VAEL_RAW(3,1);
		VAEL_RAW[0]=-dvw*cos(psiwdx*RAD);
		VAEL_RAW[1]=-dvw*sin(psiwdx*RAD);
		VAEL_RAW[2]=vaed3;

		//smoothing wind by filtering with time constant 'twind' sec
		Matrix VAELSD_NEW=(VAEL_RAW-VAELS)*(1/twind);
		VAELS=integrate(VAELSD_NEW,VAELSD,VAELS,int_step);
		VAELSD=VAELSD_NEW;
		VAEL=VAELS;
	}
	//flight conditions
	VBAL=VBEL-VAEL;
	dvba=VBAL.absolute();

	//plane mach number
	vmach=fabs(dvba/vsound);

	//plane dynamic pressure
	pdynmc=0.5*rho*dvba*dvba;

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
	//state variables
	flat6[72].gets_vec(VAELS);
	flat6[73].gets_vec(VAELSD);		
	//output to other modules
	flat6[52].gets(press);
	flat6[53].gets(rho);
	flat6[55].gets(grav);
	flat6[56].gets(vmach);
	flat6[57].gets(pdynmc);
	flat6[58].gets(tempk);
	flat6[75].gets(dvba);
	//diagnostics
	flat6[54].gets(vsound);
	flat6[74].gets_vec(VAEL);
	flat6[76].gets_vec(VBAL);
}	
