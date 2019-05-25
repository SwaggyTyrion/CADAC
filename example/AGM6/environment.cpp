///////////////////////////////////////////////////////////////////////////////
//FILE: 'environment.cpp'
//Contains 'environment' module of class 'Flat6'
//
//100424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of environment module-variables 
//Member function of class 'Flat6'
//Module-variable locations are assigned to flat6[50-99]
// 
// The switch 'mair' controls the atmosphere, wind and air turbulence options:
//
//     mair=|matmo|mturb|mwind|
//
//		     matmo = 0 US 1976 Standard Atmosphere (public domain shareware)
//				   = 2 tabular atmosphere from WEATHER_DECK
//
//				   mturb = 0 no turbulence
//						 = 1 dryden turbulence model
//
//						 mwind = 0 no wind
//							   = 1 constant wind, input: dvaeg,psiwdx
//      	   	               = 2 tabular wind from WEATHER_DECK
//
//100424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::def_environment()
{
	//Definition and initialization of module-variables
     flat6[50].init("mair","int",0,"mair =|matmo|mturb|mwind|","environment","data","");
     flat6[51].init("warning_flag","int",0,"altitude is outside 'us76_nasa2002' atmosphere ","environment","init","");
     flat6[52].init("press",0,"Atmospheric pressure - Pa","environment","out","");
     flat6[53].init("rho",0,"Atmospheric density - kg/m^3","environment","out","");
     flat6[54].init("vsound",0,"Sonic speed - m/sec","environment","diag","");
     flat6[55].init("grav",0,"Magnitude of grav accel - m/s^2","environment","out","");
     flat6[56].init("vmach",0,"Mach number of missile - ND","environment","out","scrn,plot,com");
     flat6[57].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot");
	 flat6[58].init("tempk",0,"Atmospheric temperature - K","environment","out","");
	 flat6[59].init("mfreeze_evrn","int",0,"Saving 'mfreeze' value","environment","save","");
	 flat6[60].init("pdynmcf",0,"Saved dyn.press. when mfreeze=1 - Pa","environment","save","");
	 flat6[61].init("vmachf",0,"Saved Mach# when mfreeze=1 - ND","environment","save","");
	 flat6[62].init("GRAVL",0,0,0,"Grav accel in local level coord - m/s^2","environment","out","");
     flat6[64].init("dvae",0,"Magnitude of constant air speed - m/s","environment","data","");
     flat6[65].init("dvael",0,"Air speed at low altitude - m/s","environment","data","");
     flat6[66].init("waltl",0,"Low altitude of air speed - m","environment","data","");
     flat6[67].init("dvaeh",0,"Air speed at high altitude - m/s","environment","data","");
     flat6[68].init("walth",0,"High altitude of air speed - m","environment","data","");
     flat6[69].init("vaed3",0,"Vertical air speed (pos.down) - m/s","environment","data","");
     flat6[70].init("psiwdx",0,"Wind direction from north - m/s","environment","data","");
     flat6[71].init("twind",0.1,"Wind smoothing time constant - sec","environment","data","");
	 flat6[72].init("VAELS",0,0,0,"Smoothed wind velocity in local level coord. - m/s","environment","state","");
	 flat6[73].init("VAELSD",0,0,0,"Smoothed wind velocity derivative - m/s","environment","state","");
	 flat6[74].init("VAEL",0,0,0,"Wind velocity in local level coord. - m/s","environment","out","plot");
	 flat6[75].init("dvba",0,"Vehicle speed wrt air - m/s","environment","out","");
	 flat6[76].init("markov_value",0,"Markov variable - m/s","environment","save","");
	 flat6[77].init("turb_length",0,"Turbulence correlation length - m","environment","data","");
	 flat6[78].init("turb_sigma",0,"Turbulence magnitude (1sigma) - m/s","environment","data","");
	 flat6[79].init("taux1",0,"First turbulence state variable - ND","environment","state","");
	 flat6[80].init("taux1d",0,"First turbulence state variable - ND","environment","state","");
	 flat6[81].init("taux2",0,"First turbulence state variable derivative - 1/s","environment","state","");
	 flat6[82].init("taux2d",0,"First turbulence state variable derivative - 1/s","environment","state","");
	 flat6[83].init("tau",0,"Turblence velocity component in load factor plane - m/s","environment","diag","");
	 flat6[84].init("gauss_value",0,"White Gaussian noise - ND","environment","diag","");
	 flat6[85].init("tempc",0,"Atmospheric temperature - Centigrade","environment","diag","");
}	
///////////////////////////////////////////////////////////////////////////////
//Environment module initialiation
//Member function of class 'Flat6'
//
// (1) Initializes airspeed dvba with geographic speed dvbe
//
//100424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::init_environment()
{
	//local module-variables
	double dvba(0);

	//localizing module-variables
	//input from other modules
	double dvbe=flat6[236].real();
//-----------------------------------------------------------------------------
	dvba=dvbe;
//-----------------------------------------------------------------------------
	//loading module-variables
	//initialization
	flat6[75].gets(dvba);
}
///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Flat6'
//
// (1) Calculates the atmospheric properties
//
// The switch 'mair' controls the atmosphere, wind and air turbulence options:
//
//     mair=|matmo|mturb|mwind|
//
//		     matmo = 0 US 1976 Standard Atmosphere (public domain shareware)
//				   = 2 tabular atmosphere from WEATHER_DECK
//
//				   mturb = 0 no turbulence
//						 = 1 dryden turbulence model
//
//						 mwind = 0 no wind
//							   = 1 constant wind, input: dvaeg,psiwdx
//      	   	               = 2 tabular wind from WEATHER_DECK
//
// (2) Tabular atmosphere is from WEATHER_DECK with density 'rho' in kg/m^3
//		pressure 'press' in Pa, and temperature in 'tempc' in deg Centigrade,
//		as a function of altitude 'hbe' in m
// (3) Constant horizontal wind is input by 'dvae' and wind direction 'psiwdx'
//	   Tabular wind is from WEATHER_DECK with heading from north 'psiwdx' in deg
//		 and magnitude 'dvw' in m/s as a function of altitude 'hbe' in m
// (4) Calculates the vehicles's Mach number and dynamic pressure
// (5) Heat equilibrium calculations on nose of vehicle
//
//100424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::environment(double int_step)
{
	//local variables
	double tempc(0);
	double tempk(0);
	double dvw(0);
	
	//local module-variables
	double press(0);
	double rho(0);
	double vsound(0);
	double vmach(0);
	double pdynmc(0);
	double grav(0);
	Matrix GRAVL(3,1);
	Matrix VAEL(3,1);

	//localizing module-variables
	//input data
	int mair=flat6[50].integer();
	double dvae=flat6[64].real(); 
	double dvael=flat6[65].real(); 
	double waltl=flat6[66].real(); 
	double dvaeh=flat6[67].real(); 
	double walth=flat6[68].real(); 
	double vaed3=flat6[69].real(); 
	double psiwdx=flat6[70].real(); 
	double twind=flat6[71].real(); 
	//getting saved values
	int warning_flag=flat6[51].integer();
	int mfreeze_evrn=flat6[59].integer();
	double pdynmcf=flat6[60].real(); 
	double vmachf=flat6[61].real();
	double dvba=flat6[75].real(); 
	//state variables
	Matrix VAELS=flat6[72].vec();
	Matrix VAELSD=flat6[73].vec();
	//input from other modules
	double time=flat6[0].real(); 
	double hbe=flat6[239].real();
	Matrix VBEL=flat6[233].vec();
	int trcond=missile[180].integer();
	double trmach=missile[183].real();
	double trdynm=missile[184].real();
	int mguid=missile[400].integer();
	int mfreeze=missile[501].integer();
	//-------------------------------------------------------------------------
	//decoding the air switch
	int matmo=mair/100;
	int mturb=(mair-matmo*100)/10;
	int mwind=(mair-matmo*100)%10;

	//altitude above Earth's center
	double rad=REARTH+hbe;

	//calculating the gravity acceleration
	grav=G*EARTH_MASS/pow(rad,2);

	//US 1976 Standard Atmosphere (public domain)
	if(matmo==0){
		atmosphere76(rho,press,tempk, hbe);
		tempc=tempk-273.16;
		//speed of sound
		vsound=sqrt(1.4*RGAS*tempk);
	}
	//tabular atmosphere from WEATHER_DECK
	if(matmo==2){
		rho=weathertable.look_up("density",hbe);
		press=weathertable.look_up("pressure",hbe);
		tempc=weathertable.look_up("temperature",hbe);
		//speed of sound
		tempk=tempc+273.16;
		vsound=sqrt(1.4*RGAS*tempk);
	}
	//mach number
	vmach=fabs(dvba/vsound);

	//dynamic pressure
	pdynmc=0.5*rho*dvba*dvba;

	//wind options
	if(mwind>0){
		if(mwind==1)
			//constant wind
			dvw=dvae;

		if(mwind==2){
			//tabular wind from WEATHER_DECK
			dvw=weathertable.look_up("speed",hbe);
			psiwdx=weathertable.look_up("direction",hbe);
		}
		//wind components in local level coord.
		Matrix VAED_RAW(3,1);
		VAED_RAW[0]=-dvw*cos(psiwdx*RAD);
		VAED_RAW[1]=-dvw*sin(psiwdx*RAD);
		VAED_RAW[2]=vaed3;

		//smoothing wind by filtering with time constant 'twind' sec
		Matrix VAEDSD_NEW=(VAED_RAW-VAELS)*(1/twind);
		VAELS=integrate(VAEDSD_NEW,VAELSD,VAELS,int_step);
		VAELSD=VAEDSD_NEW;
		VAEL=VAELS;
	}
	//wind turbulence in normal-load plane
	if(mturb==1){
		Matrix VTAL=environment_dryden(dvba,int_step);
		VAEL=VTAL+VAELS;
	}
	//flight conditions
	Matrix VBAL=VBEL-VAEL;
	dvba=VBAL.absolute();

	//mach number
	vmach=fabs(dvba/vsound);
	//dynamic pressure
	pdynmc=0.5*rho*dvba*dvba;

		//termination coditions
	if(mguid){
	   if(vmach<=trmach)
		   trcond=2;
	   if(pdynmc<=trdynm)
		   trcond=3;
	   }

	//freezing variables for autopilot response calculations
	if(mfreeze==0)
	    mfreeze_evrn=0;
	else{
		if(mfreeze!=mfreeze_evrn){
		  mfreeze_evrn=mfreeze;
		  vmachf=vmach;
		  pdynmcf=pdynmc;
		}
	    vmach=vmachf;
	    pdynmc=pdynmcf;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	flat6[51].gets(warning_flag);
	flat6[59].gets(mfreeze_evrn);
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
	flat6[62].gets_vec(GRAVL);
	flat6[74].gets_vec(VAEL);
	flat6[75].gets(dvba);
	missile[180].gets(trcond);
	//diagnostics
	flat6[54].gets(vsound);
	flat6[58].gets(tempk);
	flat6[85].gets(tempc);
}	
///////////////////////////////////////////////////////////////////////////////
// Dryden turbulence model
// Ref: Etkin, Dynamics of Flight,Wiley 1958, p.318
// Modeling assumption: turbulence is of interest only in the load factor plane
// Return output:
//          VTAL(3)=Velocity of turbulence wrt steady air mass in local level coord. - m/s
// Parameter input:
//          dvba = Vehicle speed wrt air mass - m/s
//
//030528 Adapted from GHAME6 FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Flat6::environment_dryden(double dvba,double int_step)
{	
	//local variables
	Matrix VTAL(3,1);
	//local module-variables
	double tau(0);
	double gauss_value(0);
	
	//localizing module-variables
	//input data
	double turb_length=flat6[77].real();
	double turb_sigma=flat6[78].real();
	//input from other modules
	double time=flat6[0].real();
	Matrix TBD=flat6[120].mat();
	double alppx=flat6[140].real();
	double phipx=flat6[141].real();
	//getting saved value
	double markov_value=flat6[76].real();
	//state variables
	double taux1=flat6[79].real();
	double taux1d=flat6[80].real();
	double taux2=flat6[81].real();
	double taux2d=flat6[82].real();
	//-------------------------------------------------------------------------
	//white Gaussian noise with zero mean
	double value1;
	do
		value1=(double)rand()/RAND_MAX;
	while(value1==0);
	double value2=(double)rand()/RAND_MAX;
	gauss_value=(1/sqrt(int_step))*sqrt(2*log(1/value1))*cos(2*PI*value2);

	//filter, converting white gaussian noise into a time sequence of Dryden
	// turbulence velocity variable 'tau'  (one-dimensional cross-velocity Dryden spectrum)
	//integrating first state variable
	double taux1d_new=taux2;
	taux1=integrate(taux1d_new,taux1d,taux1,int_step);
	taux1d=taux1d_new;
	//integrating second state variable
	double vl=dvba/turb_length;
	double taux2d_new=-vl*vl*taux1-2*vl*taux2+vl*vl*gauss_value;
	taux2=integrate(taux2d_new,taux2d,taux2,int_step);
	taux2d=taux2d_new;
	//computing Dryden 'tau' from the two filter states ('2*PI' changed to 'PI' according to Pritchard)
	tau=turb_sigma*sqrt(1/(vl*PI))*(taux1+sqrt(3.)*taux2/vl);

	//inserting the turbulence into the load factor plane (aeroballistic 1A-3A plane)
	// and transforming into body coordinates VTAB=TBA*VTAA; VTAA=[0 0 tau]
	Matrix VTAB(3,1);
	VTAB[0]=-tau*sin(alppx*RAD);
	VTAB[1]=tau*sin(phipx*RAD)*cos(alppx*RAD);
	VTAB[2]=tau*cos(phipx*RAD)*cos(alppx*RAD);

	//turbulence in local level coord.
	VTAL=~TBD*VTAB;
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving value
	flat6[76].gets(markov_value);
	//state variables
	flat6[79].gets(taux1);
	flat6[80].gets(taux1d);
	flat6[81].gets(taux2);
	flat6[82].gets(taux2d);
	//diagnostics
	flat6[83].gets(tau);
	flat6[84].gets(gauss_value);

	return VTAL;
}
