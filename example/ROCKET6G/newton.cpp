///////////////////////////////////////////////////////////////////////////////
//FILE: 'newton.cpp'
//Contains 'newton' module of class 'Round6'
//
//030416 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of newton module-variables 
//Member function of class 'Round6'
//Module-variable locations are assigned to round6[210-299]
// 
//Initializing variables for the newton module.
//
//030410 Created by Peter H Zipfel
//050222 Variabl integration step size, PZi
///////////////////////////////////////////////////////////////////////////////

void Round6::def_newton()
{
	//Definition of module-variables
	round6[217].init("alpha0x",0,"Initial angle-of-attack - deg","newton","data","");
	round6[218].init("beta0x",0,"Initial sideslip angle - deg","newton","data","");
	round6[219].init("lonx",0,"Vehicle longitude - deg","newton","init,diag","scrn,plot,com");
	round6[220].init("latx",0,"Vehicle latitude - deg","newton","init,diag","scrn,plot,com");
	round6[221].init("alt",0,"Vehicle altitude - m","newton","init,out","scrn,plot,com");
	round6[222].init("TVD",0,0,0,0,0,0,0,0,0,"TM of geo velocity wrt geodetic coord - ND ","newton","out","");
	round6[223].init("TDI",0,0,0,0,0,0,0,0,0,"TM of geodetic wrt inertial  coordinates ","newton","init","");
	round6[225].init("dvbe",0,"Vehicle geographic speed - m/s","newton","init,out","scrn,plot,com");
	round6[226].init("dvbi",0,"Vehicle inertial speed - m/s","newton","out","scrn,plot,com");
	round6[227].init("WEII",0,0,0,0,0,0,0,0,0,"Earth's angular velocity (skew-sym) - rad/s ","newton","init","");
	round6[228].init("psivdx",0,"Vehicle heading angle - deg","newton","init,out","scrn,plot,com");
	round6[229].init("thtvdx",0,"Vehicle flight path angle - deg","newton","init,out","scrn,plot,com");
	round6[230].init("dbi",0,"Vehicle distance from center of Earth - m","newton","out","");
	round6[231].init("TGI",0,0,0,0,0,0,0,0,0,"TM of geocentric wrt inertial  coordinates ","newton","init","");
	round6[232].init("VBED",0,0,0,"Geographic velocity in geodetic coord - m/s ","newton","out","");
	round6[234].init("altx",0,"Vehicle altitude - kft","newton","diag","");
	round6[235].init("SBII",0,0,0,"Inertial position - m ","newton","state","com");
	round6[236].init("VBII",0,0,0,"Inertial velocity - m/s ","newton","state","com");
	round6[237].init("ABII",0,0,0,"Inertial acceleration - m/s^2 ","newton","save","");
	round6[238].init("grndtrck",0,"Vehicle ground track on Earth surface - m","newton","diag","plot");
	round6[239].init("FSPB",0,0,0,"Specifiv force in body coord - m/s^2 ","newton","out","scrn");
	round6[240].init("ayx",0,"Achieved side acceleration - g's","newton","diag","plot");
	round6[241].init("anx",0,"Achieved normal acceleration - g's","newton","diag","plot");
	round6[242].init("gndtrkmx",0,"Ground track - km","newton","diag","");
	round6[243].init("gndtrnmx",0,"Ground track - nm","newton","diag","plot");
    round6[247].init("mfreeze_newt","int",0,"Saving mfreze value - ND","newton","save","");
    round6[248].init("dvbef",0,"Saved speed when mfreeze=1 - m/s","newton","save","");
}

///////////////////////////////////////////////////////////////////////////////
//Initial calculations of newton module 
//Member function of class 'Round6'
// 
//Initial calculations.
// 
//030410 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round6::init_newton()
{
	//local variables
	Matrix VBEB(3,1);
	Matrix POLAR(3,1);

	//local module-variables
	Matrix WBII(3,3);
	double dbi(0);
	Matrix VBED(3,1);
	Matrix SBII(3,1);
	Matrix VBII(3,1);
	double dvbi(0);
	double psivdx(0);
	double thtvdx(0);

	//localizing module-variables
	//input data
	double dvbe=round6[225].real();  
	double lonx=round6[219].real();  
	double latx=round6[220].real();  
	double alt=round6[221].real();
	//input from other modules
	double time=round6[0].real();
	double psibdx=round6[137].real();
	double thtbdx=round6[138].real();
	double phibdx=round6[139].real();
	double alpha0x=round6[217].real();	
	double beta0x=round6[218].real();	
	//-----------------------------------------------------------------------------
	//Earth's angular velocity skew-symmetric matrix (3x3)
	Matrix WEII(3,3);
	WEII.assign_loc(0,1,-WEII3);
	WEII.assign_loc(1,0,WEII3);

	//converting geodetic lonx, latx, alt to SBII
	SBII=cad_in_geo84(lonx*RAD,latx*RAD,alt,time);
	dbi=SBII.absolute();
			
	//building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe 
    double salp=sin(alpha0x*RAD);
    double calp=cos(alpha0x*RAD);
    double sbet=sin(beta0x*RAD);
    double cbet=cos(beta0x*RAD);
    double vbeb1=calp*cbet*dvbe;
    double vbeb2=sbet*dvbe;
    double vbeb3=salp*cbet*dvbe;
	VBEB.build_vec3(vbeb1,vbeb2,vbeb3);

	//building TBD 
	Matrix TBD=mat3tr(psibdx*RAD,thtbdx*RAD,phibdx*RAD); 
	//Geodetic velocity
	VBED=~TBD*VBEB;

	//building inertial velocity
	Matrix TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,time);
	Matrix TGI=cad_tgi84(lonx*RAD,latx*RAD,alt,time);
	VBII=~TDI*VBED+WEII*SBII;
	dvbi=VBII.absolute();

	//calculating geodetic flight path angles (plotting initialization)
	POLAR=VBED.pol_from_cart();
	psivdx=DEG*POLAR.get_loc(1,0);
	thtvdx=DEG*POLAR.get_loc(2,0);
	//-----------------------------------------------------------------------------
	//loading module-variables
	round6[219].gets(lonx);
	round6[220].gets(latx);
	round6[223].gets_mat(TDI);
	round6[226].gets(dvbi);
	round6[227].gets_mat(WEII);
	round6[228].gets(psivdx);
	round6[229].gets(thtvdx);
	round6[230].gets(dbi);
	round6[231].gets_mat(TGI);
	round6[232].gets_vec(VBED);
	round6[235].gets_vec(SBII);
	round6[236].gets_vec(VBII);
	//output to other module
	round6[137].gets(psibdx);
}
///////////////////////////////////////////////////////////////////////////////
//Newton module
//Member function of class 'Round6'
//
//Solving the translational equations of motion using Newton's 2nd Law
//
//030410 Created by Peter H Zipfel
//050222 Variable integration step size, PZi
///////////////////////////////////////////////////////////////////////////////

void Round6::newton(double int_step)
{
	//local variables
	double lon(0);
	double lat(0);

	//local module-variables
	double dvbe(0);
	double dvbi(0);
	double lonx(0);
	double latx(0);
	double alt(0);
	double altx(0);
	double psivdx(0);
	double thtvdx(0);
	Matrix FSPB(3,1);
	double anx(0);
	double ayx(0);
	double dbi(0);
	double gndtrkmx(0);
	double gndtrnmx(0);
	Matrix TVD(3,3);
	Matrix VBED(3,1);
	
	//localizing module-variables
	//initializations
	Matrix TDI=round6[223].mat();
	Matrix TGI=round6[231].mat();
	Matrix WEII=round6[227].mat();
	//input data
	//getting saved values
	double grndtrck=round6[238].real();
	int mfreeze_newt=round6[247].integer();
	double dvbef=round6[248].real();
	//state variables
	Matrix SBII=round6[235].vec();
	Matrix VBII=round6[236].vec();
	Matrix ABII=round6[237].vec();
	//input from other modules
	double time=round6[0].real();
	Matrix GRAVG=round6[62].vec();
	Matrix TBI=round6[121].mat();	    
	Matrix FAPB=round6[200].vec();  	
	double vmass=hyper[15].real();
	int mfreeze=hyper[503].integer();
	//-----------------------------------------------------------------------------
	//integrating vehicle acceleration
	FSPB=FAPB*(1./vmass);
	Matrix NEXT_ACC=~TBI*FSPB+~TGI*GRAVG;
	Matrix NEXT_VEL=integrate(NEXT_ACC,ABII,VBII,int_step);
	SBII=integrate(NEXT_VEL,VBII,SBII,int_step);
	ABII=NEXT_ACC;
	VBII=NEXT_VEL;
	dvbi=VBII.absolute();
	dbi=SBII.absolute();

	//geodetic longitude, latitude and altitude
	cad_geo84_in(lon,lat,alt,SBII,time);
	TDI=cad_tdi84(lon,lat,alt,time);
	TGI=cad_tgi84(lon,lat,alt,time);
	lonx=lon*DEG;
	latx=lat*DEG;
	//altitude in kft for diagnostics
	altx=0.001*alt*FOOT;

	//geographic velocity in geodetic axes VBED(3x1) and flight path angles
	VBED=TDI*(VBII-WEII*SBII);
	Matrix POLAR=VBED.pol_from_cart();
	dvbe=POLAR[0];
	psivdx=DEG*POLAR[1];
	thtvdx=DEG*POLAR[2];

	//T.M. of geographic velocity wrt geodetic coordinates
	TVD=mat2tr(psivdx*RAD,thtvdx*RAD);

	//diagnostics: acceleration achieved
	ayx=FSPB[1]/AGRAV;
	anx=-FSPB[2]/AGRAV;

	//ground track travelled (10% accuracy, usually on the high side)
	double vbed1=VBED[0];
	double vbed2=VBED[1];
	grndtrck+=sqrt(vbed1*vbed1+vbed2*vbed2)*int_step*REARTH/dbi;
	gndtrkmx=0.001*grndtrck;
	gndtrnmx=NMILES*grndtrck;

	//freeze variables for autopilot response calculations
	if(mfreeze==0)
		mfreeze_newt=0;
	else{
		if(mfreeze!=mfreeze_newt){
			mfreeze_newt=mfreeze;
			dvbef=dvbe;
		}
		dvbe=dvbef;
	}	
	//-----------------------------------------------------------------------------
	//loading module-variables
	//state variables
	round6[235].gets_vec(SBII);
	round6[236].gets_vec(VBII);
	round6[237].gets_vec(ABII);
	//saving values
	round6[238].gets(grndtrck);
	round6[247].gets(mfreeze_newt);
	round6[248].gets(dvbef);
	//output to other modules
	round6[219].gets(lonx);
	round6[220].gets(latx);
	round6[221].gets(alt);
	round6[222].gets_mat(TVD);
	round6[223].gets_mat(TDI);
	round6[225].gets(dvbe);
	round6[226].gets(dvbi);
	round6[231].gets_mat(TGI);
	round6[232].gets_vec(VBED);
	round6[239].gets_vec(FSPB);
	//diagnostics
	round6[228].gets(psivdx);
	round6[229].gets(thtvdx);
	round6[230].gets(dbi);
	round6[234].gets(altx);
	round6[240].gets(ayx);
	round6[241].gets(anx);
	round6[242].gets(gndtrkmx);
	round6[243].gets(gndtrnmx);
}
