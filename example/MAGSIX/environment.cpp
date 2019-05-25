///////////////////////////////////////////////////////////////////////////////
//FILE: 'environment.cpp'
//Contains 'environment' module of class 'Rotor'
//
//030626 Adopted from AIMC and wind added, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of environment module-variables 
//Member function of class 'Rotor'
//Module-variable locations are assigned to rotor[0-99]
// 
//130523 Modified for MAGSIX, PZi 
///////////////////////////////////////////////////////////////////////////////

void Rotor::def_environment()
{
	//Definition and initialization of module-variables
     rotor[50].init("mwind","int",0,"=0: no wind; =1:const; =2:shear","environment","data","");
     rotor[52].init("press",0,"Atmospheric pressure - Pa","environment","out","");
     rotor[53].init("rho",0,"Atmospheric density - kg/m^3","environment","out","");
     rotor[54].init("vsound",0,"Sonic speed - m/sec","environment","diag","");
     rotor[55].init("grav",0,"Gravity acceleration - m/s^2","environment","out","");
     rotor[56].init("vmach",0,"Mach number of rotor","environment","out","scrn,plot,com");
     rotor[57].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot");
	 rotor[58].init("tempk",0,"Atmospheric temperature - K","environment","out","");
     rotor[64].init("dvae",0,"Magnitude of constant air speed - m/s","environment","data","");
     rotor[65].init("dvael",0,"Air speed at low altitude - m/s","environment","data","");
     rotor[66].init("waltl",0,"Low altitude of air speed - m","environment","data","");
     rotor[67].init("dvaeh",0,"Air speed at high altitude - m/s","environment","data","");
     rotor[68].init("walth",0,"High altitude of air speed - m","environment","data","");
     rotor[69].init("vaed3",0,"Vertical air speed (pos.down) - m/s","environment","data","");
     rotor[70].init("psiwdx",0,"Wind direction from north - m/s","environment","data","");
     rotor[71].init("twind",0.1,"Wind smoothing time constant - sec","environment","data","");
	 rotor[72].init("VAELS",0,0,0,"Smoothed wind vel in Lcoord - m/s","environment","state","");
	 rotor[73].init("VAELSD",0,0,0,"Smoothed wind velocity derivative - m/s","environment","state","");
	 rotor[74].init("VAEL",0,0,0,"Wind vel in local level coordinates - m/s","environment","out","");
	 rotor[75].init("dvba",0,"Vehicle speed wrt air - m/s","environment","out","");
	 rotor[76].init("VBAL",0,0,0,"Vehicle vel wrt air in L-coord - m/s","environment","out","");
}	

///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Rotor'
//
//US 1976 Standard Atmosphere
// Calculates the atmospheric properties
// Calculates the gravitational acceleration
// Calculates the rotor's Mach number
//
//011127 Created by Peter Zipfel
//030319 Upgraded to US76 atmosphere, PZi
//130523 Modified for MAGSIX, PZi
///////////////////////////////////////////////////////////////////////////////

void Rotor::environment(double int_step)
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
	int mwind=rotor[50].integer();
	double dvae=rotor[64].real(); 
	double dvael=rotor[65].real(); 
	double waltl=rotor[66].real(); 
	double dvaeh=rotor[67].real(); 
	double walth=rotor[68].real(); 
	double vaed3=rotor[69].real(); 
	double psiwdx=rotor[70].real(); 
	double twind=rotor[71].real(); 
	//state variables
	Matrix VAELS=rotor[72].vec();
	Matrix VAELSD=rotor[73].vec();
	//input from other modules
	Matrix VBEL=rotor[143].vec();
	double hbe=rotor[128].real(); 
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

	//rotor mach number
	vmach=fabs(dvba/vsound);

	//rotor dynamic pressure
	pdynmc=0.5*rho*dvba*dvba;

	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	rotor[72].gets_vec(VAELS);
	rotor[73].gets_vec(VAELSD);		
	//output to other modules
	rotor[52].gets(press);
	rotor[53].gets(rho);
	rotor[55].gets(grav);
	rotor[56].gets(vmach);
	rotor[57].gets(pdynmc);
	rotor[58].gets(tempk);
	rotor[75].gets(dvba);
	//diagnostics
	rotor[54].gets(vsound);
	rotor[74].gets_vec(VAEL);
	rotor[76].gets_vec(VBAL);
}	
