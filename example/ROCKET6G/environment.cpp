///////////////////////////////////////////////////////////////////////////////
//FILE: 'environment.cpp'
//Contains 'environment' module of class 'Round6'
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of environment module-variables 
//Member function of class 'Round6'
//Ref: Zipfel, Section 10.3.2, p. 465
//Module-variable locations are assigned to round6[50-99]
// 
// The switch 'mair' controls the atmosphere, wind and air turbulence options:
//
//     mair=|matmo|mturb|mwind|
//
//		     matmo = 0 US 1976 Standard Atmosphere (public domain shareware)
//				   = 1 US 1976 Standard Atmosphere with extension up to 1000 km (NASA Marshall)
//				   = 2 tabular atmosphere from WEATHER_DECK
//
//				   mturb = 0 no turbulence
//						 = 1 dryden turbulence model
//
//						 mwind = 0 no wind
//							   = 1 constant wind, input: dvaeg,psiwdx
//      	   	               = 2 tabular wind from WEATHER_DECK
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round6::def_environment()
{
	//Definition and initialization of module-variables
     round6[50].init("mair","int",0,"mair =|matmo|mturb|mwind|","environment","data","");
     round6[51].init("warning_flag","int",0,"altitude is outside 'us76_nasa2002' atmosphere ","environment","init","");
     round6[52].init("press",0,"Atmospheric pressure - Pa","environment","out","");
     round6[53].init("rho",0,"Atmospheric density - kg/m^3","environment","out","");
     round6[54].init("vsound",0,"Sonic speed - m/sec","environment","diag","");
     round6[56].init("vmach",0,"Mach number of hyper - ND","environment","out","scrn,plot,com");
     round6[57].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot");
	 round6[58].init("tempk",0,"Atmospheric temperature - K","environment","out","");
	 round6[59].init("mfreeze_evrn","int",0,"Saving 'mfreeze' value","environment","save","");
	 round6[60].init("pdynmcf",0,"Saved dyn.press. when mfreeze=1 - Pa","environment","save","");
	 round6[61].init("vmachf",0,"Saved Mach# when mfreeze=1 - ND","environment","save","");
	 round6[62].init("GRAVG",0,0,0,"Grav accel in geocentric coord - m/s^2","environment","out","");
     round6[63].init("grav",0,"Magnitude of grav accel - m/s^2","environment","out","");
     round6[64].init("dvae",0,"Magnitude of constant air speed - m/s","environment","data","");
     round6[65].init("dvael",0,"Air speed at low altitude - m/s","environment","data","");
     round6[66].init("waltl",0,"Low altitude of air speed - m","environment","data","");
     round6[67].init("dvaeh",0,"Air speed at high altitude - m/s","environment","data","");
     round6[68].init("walth",0,"High altitude of air speed - m","environment","data","");
     round6[69].init("vaed3",0,"Vertical air speed (pos.down) - m/s","environment","data","");
     round6[70].init("psiwdx",0,"Wind direction from north - m/s","environment","data","");
     round6[71].init("twind",0.1,"Wind smoothing time constant - sec","environment","data","");
	 round6[72].init("VAEDS",0,0,0,"Smoothed wind velocity in geodetic coord - m/s","environment","state","");
	 round6[73].init("VAEDSD",0,0,0,"Smoothed wind velocity derivative - m/s","environment","state","");
	 round6[74].init("VAED",0,0,0,"Wind velocity in geodetic coordinates - m/s","environment","out","");
	 round6[75].init("dvba",0,"Vehicle speed wrt air - m/s","environment","out","");
	 round6[76].init("markov_value",0,"Markov variable - m/s","environment","save","");
	 round6[77].init("turb_length",0,"Turbulence correlation length - m","environment","data","");
	 round6[78].init("turb_sigma",0,"Turbulence magnitude (1sigma) - m/s","environment","data","");
	 round6[79].init("taux1",0,"First turbulence state variable - ND","environment","state","");
	 round6[80].init("taux1d",0,"First turbulence state variable - ND","environment","state","");
	 round6[81].init("taux2",0,"First turbulence state variable derivative - 1/s","environment","state","");
	 round6[82].init("taux2d",0,"First turbulence state variable derivative - 1/s","environment","state","");
	 round6[83].init("tau",0,"Turblence velocity component in load factor plane - m/s","environment","diag","");
	 round6[84].init("gauss_value",0,"White Gaussian noise - ND","environment","diag","");
	 round6[85].init("tempc",0,"Atmospheric temperature - Centigrade","environment","diag","");
}	

///////////////////////////////////////////////////////////////////////////////
//Environment module initialiation
//Member function of class 'Round6'
//
// (1) Initializes airspeed dvba with geographic speed dvbe
//
//030528 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round6::init_environment()
{
	//local module-variables
	double dvba(0);

	//localizing module-variables
	//input from other modules
	double dvbe=round6[225].real();
//-----------------------------------------------------------------------------
	dvba=dvbe;
//-----------------------------------------------------------------------------
	//loading module-variables
	//initialization
	round6[75].gets(dvba);
}

///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Round6'
//
// (1) Calculates the atmospheric properties
//
// The switch 'mair' controls the atmosphere, wind and air turbulence options:
//
//     mair=|matmo|mturb|mwind|
//
//		     matmo = 0 US 1976 Standard Atmosphere (public domain shareware)
//				   = 1 US 1976 Standard Atmosphere with extension up to 1000 km (NASA Marshall)
//				   = 2 tabular atmosphere from WEATHER_DECK
//
//				   mturb = 0 no turbulence
//						 = 1 dryden turbulence model
//
//						 mwind = 0 no wind
//							   = 1 constant wind, input: dvaeg,psiwdx
//      	   	               = 2 tabular wind from WEATHER_DECK
//
// (2) Tabular atmosphere in from WEATHER_DECK with density 'rho' in kg/m^3
//		pressure 'press' in Pa, and temperature in 'tempc' in deg Centigrade,
//		as a function of altitude 'alt' in m
// (3) Constant horizontal wind is input by 'dvae' and wind direction 'psiwdx'
//	   Tabular wind is from WEATHER_DECK with heading from north 'psiwdx' in deg
//		 and magnitude 'dvw' in m/s as a function of altitude 'alt' in m
// (4) Calculates the vehicles's Mach number and dynamic pressure
// (5) Heat equilibrium calculations on nose of vehicle
// (6) Gravitational acceleration based on WGS84 ellipsoid
//
//030507 Created by Peter H Zipfel
//040311 Added US76 Atmosphere extended to 1000km (NASA Marshall), PZi
//091216 Added tabular atmosphere and wind, PZi
///////////////////////////////////////////////////////////////////////////////

void Round6::environment(double int_step)
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
	Matrix GRAVG(3,1);
	Matrix VAED(3,1);

	//localizing module-variables
	//input data
	int mair=round6[50].integer();
	double dvae=round6[64].real(); 
	double dvael=round6[65].real(); 
	double waltl=round6[66].real(); 
	double dvaeh=round6[67].real(); 
	double walth=round6[68].real(); 
	double vaed3=round6[69].real(); 
	double psiwdx=round6[70].real(); 
	double twind=round6[71].real(); 
	//getting saved values
	int warning_flag=round6[51].integer();
	int mfreeze_evrn=round6[59].integer();
	double pdynmcf=round6[60].real(); 
	double vmachf=round6[61].real();
	double dvba=round6[75].real(); 
	//state variables
	Matrix VAEDS=round6[72].vec();
	Matrix VAEDSD=round6[73].vec();
	//input from other modules
	double time=round6[0].real(); 
	double alt=round6[221].real();
	Matrix VBED=round6[232].vec();
	Matrix SBII=round6[235].vec();	
	int trcond=hyper[180].integer();
	double trmach=hyper[183].real();
	double trdynm=hyper[184].real();
	int mguid=hyper[400].integer();
	int mfreeze=hyper[503].integer();
	//-------------------------------------------------------------------------
	//decoding the air switch
	int matmo=mair/100;
	int mturb=(mair-matmo*100)/10;
	int mwind=(mair-matmo*100)%10;

	//gravitational acceleration in geocentric coordinates
	GRAVG=cad_grav84(SBII,time);
	grav=GRAVG.absolute();

	//US 1976 Standard Atmosphere (public domain)
	if(matmo==0){
		atmosphere76(rho,press,tempk, alt);
		tempc=tempk-273.16;
		//speed of sound
		vsound=sqrt(1.4*RGAS*tempk);
	}
	//US 1976 Standard Atmosphere (NASA Marshall)
	if(matmo==1){
		double alt_km=alt/1000;
		int check=us76_nasa2002(alt_km,&rho,&press,&tempk,&vsound);
		tempc=tempk-273.16;
		if(check&&warning_flag==0){
			warning_flag=1;
			cerr<<" *** Warning: altitude is outside 'us76_nasa2002' atmosphere (0<alt<1000km) *** \n";
		}
	}
	//tabular atmosphere from WEATHER_DECK
	if(matmo==2){
		rho=weathertable.look_up("density",alt);
		press=weathertable.look_up("pressure",alt);
		tempc=weathertable.look_up("temperature",alt);
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
			dvw=weathertable.look_up("speed",alt);
			psiwdx=weathertable.look_up("direction",alt);
		}
		//wind components in geodetic coordinates
		Matrix VAED_RAW(3,1);
		VAED_RAW[0]=-dvw*cos(psiwdx*RAD);
		VAED_RAW[1]=-dvw*sin(psiwdx*RAD);
		VAED_RAW[2]=vaed3;

		//smoothing wind by filtering with time constant 'twind' sec
		Matrix VAEDSD_NEW=(VAED_RAW-VAEDS)*(1/twind);
		VAEDS=integrate(VAEDSD_NEW,VAEDSD,VAEDS,int_step);
		VAEDSD=VAEDSD_NEW;
		VAED=VAEDS;
	}
	//wind turbulence in normal-load plane
	if(mturb==1){
		Matrix VTAD=environment_dryden(dvba,int_step);
		VAED=VTAD+VAEDS;
	}
	//flight conditions
	Matrix VBAD=VBED-VAED;
	dvba=VBAD.absolute();

	//mach number
	vmach=fabs(dvba/vsound);
	//dynamic pressure
	pdynmc=0.5*rho*dvba*dvba;

		//termination coditions
	if(mguid==6){
	   if(vmach<=trmach) trcond=2;
	   if(pdynmc<=trdynm) trcond=3;
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
	round6[51].gets(warning_flag);
	round6[59].gets(mfreeze_evrn);
	round6[60].gets(pdynmcf);
	round6[61].gets(vmachf);
	//state variables
	round6[72].gets_vec(VAEDS);
	round6[73].gets_vec(VAEDSD);		
	//output to other modules
	round6[52].gets(press);
	round6[53].gets(rho);
	round6[56].gets(vmach);
	round6[57].gets(pdynmc);
	round6[62].gets_vec(GRAVG);
	round6[63].gets(grav);
	round6[74].gets_vec(VAED);
	round6[75].gets(dvba);
	hyper[180].gets(trcond);
	//diagnostics
	round6[54].gets(vsound);
	round6[58].gets(tempk);
	round6[85].gets(tempc);
}	
///////////////////////////////////////////////////////////////////////////////
// Dryden turbulence model
// Ref: Etkin, Dynamics of Flight,Wiley 1958, p.318
// Modeling assumption: turbulence is of interest only in the load factor plane
// Return output:
//          VTAD(3)=Velocity of turbulence wrt steady air mass in geodetic coord - m/s
// Parameter input:
//          dvba = Vehicle speed wrt air mass - m/s
//
//030528 Adapted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Round6::environment_dryden(double dvba,double int_step)
{	
	//local variables
	Matrix VTAD(3,1);
	//local module-variables
	double tau(0);
	double gauss_value(0);
	
	//localizing module-variables
	//input data
	double turb_length=round6[77].real();
	double turb_sigma=round6[78].real();
	//input from other modules
	double time=round6[0].real();
	Matrix TBD=round6[120].mat();
	double alppx=round6[140].real();
	double phipx=round6[141].real();
	//getting saved value
	double markov_value=round6[76].real();
	//state variables
	double taux1=round6[79].real();
	double taux1d=round6[80].real();
	double taux2=round6[81].real();
	double taux2d=round6[82].real();
	//-------------------------------------------------------------------------
	//white Gaussian noise with zero mean
	double value1;
	do
		value1=(double)rand()/RAND_MAX;
	while(value1==0);
	double value2=(double)rand()/RAND_MAX;
	gauss_value=(1/sqrt(int_step))*sqrt(2*log(1/value1))*cos(2*PI*value2);

	//filter, converting white gaussian noise into a time sequence of Dryden
	// turbulence velocity variable 'tau'  (One-dimensional cross-velocity Dryden spectrum)
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

	//turbulence in geodetic coordinates
	VTAD=~TBD*VTAB;
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving value
	round6[76].gets(markov_value);
	//state variables
	round6[79].gets(taux1);
	round6[80].gets(taux1d);
	round6[81].gets(taux2);
	round6[82].gets(taux2d);
	//diagnostics
	round6[83].gets(tau);
	round6[84].gets(gauss_value);

	return VTAD;
}
