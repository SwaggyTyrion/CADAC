///////////////////////////////////////////////////////////////////////////////
//FILE: 'flat3_modules.cpp'
//
//Contains all modules of class 'Flat3'
//							environment()
//							newton()
//
//020114 Created by Peter H Zipfel
//030319 Upgraded to SM Item32, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Defining of environmental module-variables
//Member function of class 'Flat3'
//Module-variable locations are assigned to flat3[10-19]
//
//Initializing the module-variables
//		
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat3::def_environment()
{
	//defining module-variables
	flat3[11].init("grav",0,"Gravitational acceleration - m/s^2","environment","out","");
	flat3[12].init("rho",0,"Air density - kg/m^3","environment","out","");
	flat3[13].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot");
	flat3[14].init("mach",0,"Mach number - ND","environment","out","scrn,plot");
	flat3[15].init("vsound",0,"Speed of sound - m/s","environment","diag","");
}

///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Flat3' 
//
//US 1976 Standard Atmosphere
//
//020114 Created by Peter H Zipfel
//030319 Upgraded to US76 atmosphere, PZi
///////////////////////////////////////////////////////////////////////////////

void Flat3::environment()
{	
	//local variables
	double tempk(0);
	double press(0);
	
	//localized module-variables	
	double grav(0);
	double rho(0); 
	double pdynmc(0);
	double mach(0);
	double vsound(0);
	double alt(0);

	//localizing module-variables
	double dvae=flat3[25].real();
	Matrix SAEL=flat3[26].vec();
	//-------------------------------------------------------------------------
	//Altitude above flat Earth
	alt=-SAEL.get_loc(2,0);

	//Newtonian gravitational acceleration
	grav=G*EARTH_MASS/pow((REARTH+alt),2);

	//US 1976 Standard Atmosphere
	atmosphere76(rho,press,tempk, alt);

	vsound=sqrt(1.4*R*tempk);
	mach=fabs(dvae/vsound);
	pdynmc=0.5*rho*pow(dvae,2);
	//-------------------------------------------------------------------------
	//loading module-variables
	flat3[11].gets(grav);
	flat3[12].gets(rho);
	flat3[13].gets(pdynmc);
	flat3[14].gets(mach);
	flat3[15].gets(vsound);
}

///////////////////////////////////////////////////////////////////////////////
//Definition of newton module-variables 
//Member function of class 'Flat3'
//Module-variable locations are assigned to flat3[20-39]
// 
//Initializing variables for the Newton Module.
//
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat3::def_newton()
{
	//Definition of module-variables
	flat3[22].init("TAL",0,0,0,0,0,0,0,0,0,"TM of aircraft wrt loc lev coor","newton","out","");
	flat3[23].init("TAV",0,0,0,0,0,0,0,0,0,"TM of aircraft wrt velocity coor","newton","diag","");
	flat3[24].init("TVL",0,0,0,0,0,0,0,0,0,"TM of vel coor wrt loc lev coor ","newton","diag","");
	flat3[25].init("dvae",0,"Aircraft speed - m/s","newton","init/out","com");
	flat3[26].init("SAEL",0,0,0,"Aircraft position wrt point E - m ","newton","state","com,plot");
	flat3[27].init("VAEL",0,0,0,"Aircraft velocity wrt Earth - m/s ","newton","state","com");
	flat3[28].init("AAEL",0,0,0,"Aircraft acceleration  wrt Earth - m/s^2 ","newton","state","");
	flat3[29].init("psialx",0,"Aircraft heading angle - deg","newton","init/diag","com");
	flat3[30].init("thtalx",0,"Aircraft flight path angle - deg","newton","init/diag","com");
	flat3[31].init("sael1",0,"Aircraft initial north position - m","newton","init","");
	flat3[32].init("sael2",0,"Aircraft initial east position - m","newton","init","");
	flat3[33].init("sael3",0,"Aircraft initial down position - m","newton","init","");
	flat3[34].init("psial",0,"Aircraft heading angle - rad","newton","out","");
	flat3[35].init("thtal",0,"Aircraft flight path angle - rad","newton","out","");
}

///////////////////////////////////////////////////////////////////////////////
//Initial calculations of newton module 
//Member function of class 'Flat3'
// 
//Initial calculations.
//
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat3::init_newton()
{
	//local module-variables
	Matrix TAL(3,3);
	Matrix TAV(3,3);
	Matrix TVL(3,3);

	//localizing module-variables
	double phiavout=flat3[21].real();
	double dvae=flat3[25].real();
	Matrix SAEL=flat3[26].vec();
	Matrix VAEL=flat3[27].vec();
	double psialx=flat3[29].real();
	double thtalx=flat3[30].real();
	double sael1=flat3[31].real();
	double sael2=flat3[32].real();
	double sael3=flat3[33].real();
	//-------------------------------------------------------------------------
	//conversion
	double psial=psialx*RAD;
	double thtal=thtalx*RAD;

	//calculating initial vehicle position in local level coord
	VAEL.cart_from_pol(dvae,psial,thtal);

	//initializing the T.M. TAL
	TVL=mat2tr(psial,thtal);

	//initializing the T.M. TAV
	TAV.identity();
	double cphi=cos(phiavout);
	double sphi=sin(phiavout);
	TAV.assign_loc(1,1,cphi);
	TAV.assign_loc(2,2,cphi);
	TAV.assign_loc(1,2,sphi);
	TAV.assign_loc(2,1,-sphi);

	//calculating the T.M. TAL of aircraft body ares wrt local level axes
	TAL=TAV*TVL;

	//defining initial postion vector
	SAEL.build_vec3(sael1,sael2,sael3);
	//-------------------------------------------------------------------------
	//loading module-variables
	flat3[26].gets_vec(SAEL);
	flat3[27].gets_vec(VAEL);
	flat3[22].gets_mat(TAL);
	flat3[23].gets_mat(TAV);
	flat3[24].gets_mat(TVL);
}
///////////////////////////////////////////////////////////////////////////////
//Newton module
//Member function of class 'Flat3'
//
//Solving the translational equations of motion using Newton's 2nd Law
//
//020114 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat3::newton(double sim_time,double int_step)
{
	//local variables
	Matrix TAV(3,3);
	Matrix GRAVL(3,1);

	//local module-variables
	double dvae(0);
	double psial(0),thtal(0);
	double psialx(0),thtalx(0);
	Matrix TVL(3,3);

	//localizing module-variables
	//input from other modules
	Matrix FSPA=flat3[20].vec();
	double grav=flat3[11].real();
	double phiavout=flat3[21].real();
	Matrix TAL=flat3[22].mat();
	//state variables
	Matrix SAEL=flat3[26].vec();
	Matrix VAEL=flat3[27].vec();
	Matrix AAEL=flat3[28].vec();
	//-------------------------------------------------------------------------
	//building gravitational vector in local level coord
	GRAVL.build_vec3(0,0,grav);

	//integrating state variables
	Matrix TLA=TAL.trans();
	Matrix NEXT_ACC=TLA*FSPA+GRAVL;
	Matrix NEXT_VEL=integrate(NEXT_ACC,AAEL,VAEL,int_step);
	SAEL=integrate(NEXT_VEL,VAEL,SAEL,int_step);
	AAEL=NEXT_ACC;
	VAEL=NEXT_VEL;

	//calculating the T.M. TAL
	Matrix POLAR=VAEL.pol_from_cart();
	dvae=POLAR.get_loc(0,0);
	psial=POLAR.get_loc(1,0);
	thtal=POLAR.get_loc(2,0);
	TVL=mat2tr(psial,thtal);

	//calculating the T.M. TAV
	TAV.identity();
	double cphi=cos(phiavout);
	double sphi=sin(phiavout);
	TAV.assign_loc(1,1,cphi);
	TAV.assign_loc(2,2,cphi);
	TAV.assign_loc(1,2,sphi);
	TAV.assign_loc(2,1,-sphi);

	//calculating the T.M. TAL of aircraft body ares wrt local level axes
	TAL=TAV*TVL;

	//diagnostic output
	psialx=psial*DEG;
	thtalx=thtal*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	flat3[26].gets_vec(SAEL);
	flat3[27].gets_vec(VAEL);
	flat3[28].gets_vec(AAEL);
	//output to other modules
	flat3[22].gets_mat(TAL);
	flat3[24].gets_mat(TVL);
	flat3[25].gets(dvae);
	flat3[34].gets(psial);
	flat3[35].gets(thtal);
	//diagnostics
	flat3[29].gets(psialx);
	flat3[30].gets(thtalx);
}
