///////////////////////////////////////////////////////////////////////////////
//FILE: 'round3_modules.cpp'
//Contains all modules of class 'Round3'
//							environment()	round3[0-16]
//							newton()		round3[17-39]
//
//001211 Introduced 'Variable' class to manage module-variables, PZi
//060512 Upgraded variable initialization, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Defining of environmental module-variables
//Member function of class 'Round3'
//Module-variable locations are assigned to round3[11-16]
//For executive use: round3[0-9]
//round3[10] is used for FSPV in 'Cruise::def_forces()' 
//
//Initializing the module-variables
//		
//001223 Created by Peter Zipfel
//060519 Moved 'time' calculations to 'environment' module, PZi
//100506 Pressure output added, PZi
///////////////////////////////////////////////////////////////////////////////
void Round3::def_environment()
{
	//defining module-variables
	round3[0].init("time",0,"Vehicle time since launch - s","environment","exec","scrn,plot,com");
	round3[1].init("event_time",0,"Time elapsed during an event - s","environment","exec","scrn");
	round3[2].init("int_step_new",0,"New integration step size  - s","environment","data","");
	round3[3].init("out_step_fact",0,"Fact.to mod output,e.g.plot_step*(1+out_step_fact)","environment","data","");
	round3[11].init("grav",0,"Gravitational acceleration - m/s^2","environment","out","");
	round3[12].init("rho",0,"Air density - kg/m^3","environment","out","");
	round3[13].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","scrn,plot");
	round3[14].init("mach",0,"Mach number - ND","environment","out","scrn,plot,com");
	round3[15].init("vsound",0,"Speed of sound - m/s","environment","diag","");
	round3[16].init("press",0,"Ambient pressure - Pa","environment","diag","");
}
///////////////////////////////////////////////////////////////////////////////
//Initialization of environment module
//Member function of class 'Round3'
//Setting 'sim_time' = 'time'
//Accepting new integration step size 'int_step' from 'input.asc'
//
//060519 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
void Round3::init_environment(double sim_time,double int_step)
{	
	//local module-variables
	double time(0);
	double int_step_new(0);
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//initializing the variable integration step
	int_step_new=int_step;
	//-------------------------------------------------------------------------
	//loading module-variables
	//executive output 
	round3[0].gets(time);
	round3[2].gets(int_step_new);
}
///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Round3' 
//
//Setting 'sim_time' = 'time'
//Accepting new integration step size 'int_step' from 'input.asc'
//Implementing the ISO 62 standard atmosphere
//
//000623 Created by Michael Chiaramonte
//060512 Upgraded variable initialization, PZi
//100506 Pressure output added, PZi
///////////////////////////////////////////////////////////////////////////////
void Round3::environment(double sim_time,double event_time,double &int_step,double &out_fact)
{	
	//local variables
	double ptemp(0);
	double k(0);
	
	//local module-variables
	double time(0);
	double grav(0);
	double rho(0); 
	double pdynmc(0);
	double mach(0);
	double vsound(0);
	double press(0);

	//localizing module-variables
	//input data
	double int_step_new=round3[2].real();
	double out_step_fact=round3[3].real();
	//input from other modules
	double alt=round3[21].real();
	double dvbe=round3[25].real();
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//accepting new integration step size at 'events'
	int_step=int_step_new;
	//changing the step size of all outputs by 'out_step_fact' at 'events'
	out_fact=out_step_fact;
	//-------------------------------------------------------------------------
	//Newtonian gravitational acceleration
	grav=G*EARTH_MASS/pow((REARTH+alt),2);

	//ISO 62 standard atmosphere
	if(alt<11000.0)
	{
		k=288.15-0.0065*alt;
		ptemp=101325.0*pow((k/288.15),5.2559);
		press=ptemp;
	}
	else
	{
		k=216.0;
		ptemp=22630.0*exp(-0.00015769*(alt-11000.0));
		press=ptemp;
	}	
	rho=ptemp/(R*k);
	vsound=sqrt(1.4*R*k);

	mach=fabs(dvbe/vsound);
	pdynmc=0.5*rho*pow(dvbe,2);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	round3[0].gets(time);
	round3[1].gets(event_time);
	round3[2].gets(int_step_new);
	round3[11].gets(grav);
	round3[12].gets(rho);
	round3[13].gets(pdynmc);
	round3[14].gets(mach);
	round3[16].gets(press);
	//diagnostics
	round3[15].gets(vsound);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of newton module-variables 
//Member function of class 'Round3'
//Module-variable locations are assigned to round3[17-39]
// 
//Initializing variables for the 'newton' module.
//
//000624 Created by Michael Chiaramonte
//000721 Removed function calls, Michael Horvath
//001223 Upgraded to module-variable arrays, PZi
///////////////////////////////////////////////////////////////////////////////
void Round3::def_newton()
{
	//Definition of module-variables
	round3[17].init("psivg",0,"Vehicle heading angle - rad","newton","out","");
	round3[18].init("thtvg",0,"Vehicle flight path angle - rad","newton","out","");
	round3[19].init("lonx",0,"Vehicle longitude - deg","newton","init/diag","scrn,plot,com");
	round3[20].init("latx",0,"Vehicle latitude - deg","newton","init/diag","scrn,plot,com");
	round3[21].init("alt",0,"Vehicle altitude - m","newton","init/out","scrn,plot,com");
	round3[22].init("TGV",0,0,0,0,0,0,0,0,0,"TM of geographic wrt geo velocity coord - ND ","newton","init","");
	round3[23].init("TIG",0,0,0,0,0,0,0,0,0,"TM of inertial wrt geographic coordinates ","newton","init/out","");
	round3[25].init("dvbe",0,"Vehicle speed - m/s","newton","init/out","scrn,plot,com");
	round3[27].init("WEII",0,0,0,0,0,0,0,0,0,"Earth's angular velocity (skew-sym) - rad/s ","newton","init","");
	round3[28].init("psivgx",0,"Vehicle heading angle - deg","newton","init/out","scrn,plot,com");
	round3[29].init("thtvgx",0,"Vehicle flight path angle - deg","newton","init/out","scrn,plot,com");
	round3[30].init("SB0II",0,0,0,"Initial inertial position - m ","newton","init","");
	round3[31].init("SBEG",0,0,0,"Geographic position wrt ground point below launch - m","newton","state","scrn,plot,com");
	round3[32].init("VBEG",0,0,0,"Geographic velocity - m/s","newton","state","scrn,plot,com");
	round3[33].init("TGE",0,0,0,0,0,0,0,0,0,"Geographic wrt Earth - ND ","newton","out","");	
	round3[34].init("altx",0,"Vehicle altitude - km","newton","diag","");
	round3[35].init("SBII",0,0,0,"Inertial position - m ","newton","state","com");
	round3[36].init("VBII",0,0,0,"Inertial velocity - m/s ","newton","state","");
	round3[37].init("ABII",0,0,0,"Inertial acceleration - m/s^2 ","newton","state","");
}
///////////////////////////////////////////////////////////////////////////////
//Initial calculations of newton module 
//Member function of class 'Round3'
// 
//Initial calculations.
//
//000624 Created by Michael Chiaramonte
//000721 Removed function calls, Michael Horvath
//001223 Upgraded to module-variable arrays, PZi
///////////////////////////////////////////////////////////////////////////////
void Round3::init_newton()
{
	//local variables
	double temp(0);
	double psivg(0);
	double thtvg(0);
	Matrix TEI(3,3);
	Matrix SBIE(3,1);
	Matrix SBIG(3,1);
	Matrix VBEI(3,1);
	Matrix TGI(3,3);
	Matrix TEG(3,3);
	Matrix TVG(3,3);

	//local module-variables
	Matrix SB0II(3,1);
	Matrix TGE(3,3);
	Matrix TGV(3,3);
	Matrix TIG(3,3);
	Matrix WEII(3,3);
	Matrix VBEG(3,1);
	Matrix SBII(3,1);
	Matrix VBII(3,1);

	//localizing module-variables
	//initialization 
	double dvbe=round3[25].real();
	double psivgx=round3[28].real();
	double thtvgx=round3[29].real();
	double lonx=round3[19].real();
	double latx=round3[20].real();
	double alt=round3[21].real();
	//-------------------------------------------------------------------------
	//calculating initial vehicle position in Earth coordinates
	temp=-(alt+REARTH);
	SBIG.assign_loc(2,0,temp);
	TGE=cadtge(lonx*RAD,latx*RAD);
	TEG=TGE.trans();
	SBIE=TEG*SBIG;

	//at start of simulation inertial and earth coordinates coincide
	//(must be modified if vehicles are launched at different times!)
	SBII=SBIE;

	//saving initial position for later use
	SB0II=SBII;

	psivg=psivgx*RAD;
	thtvg=thtvgx*RAD;

	//georgaphic velocity
	VBEG.cart_from_pol(dvbe,psivg,thtvg);
	
	//Earth's angular velocity tensor in inertial coordinates
	WEII.zero();
	WEII.assign_loc(0,1,-WEII3);
	WEII.assign_loc(1,0,WEII3);

	//at start of simulation the earth and inertial axes coincide
	TIG=TEG;

	//initializing velocity state variables	
	VBII=TIG*VBEG+WEII*SBII;

	//TM of velocity wrt geographic coordinates
	TVG=mat2tr(psivg,thtvg);
	TGV=TVG.trans();
	//-------------------------------------------------------------------------
	//loading module-variables
	//initialization
	round3[22].gets_mat(TGV);
	round3[23].gets_mat(TIG);
	round3[27].gets_mat(WEII);
	round3[30].gets_vec(SB0II);
	round3[32].gets_vec(VBEG);
	round3[33].gets_mat(TGE);
	round3[35].gets_vec(SBII);
	round3[36].gets_vec(VBII);
}
///////////////////////////////////////////////////////////////////////////////
//Newton module
//Member function of class 'Round3'
//
//Solving the translational equations of motion using Newton's 2nd Law
//
//000623 Created by Michael Chiaramonte
//000721 Function calls have been removed, Michael Horvath
//001227 Upgraded to module-variable arrays, PZi
//010730 Corrected calculation of geographic position, PZi
///////////////////////////////////////////////////////////////////////////////
void Round3::newton(double int_step)
{
	//local variables
	double lon(0);
	double lat(0);
	Matrix SBIE(3,1);
	Matrix TEMP(3,1);
	Matrix VBEI(3,1);
	Matrix GRAV(3,1);
	Matrix ABII_NEW(3,1);
	Matrix VBII_NEW(3,1);
	Matrix POLAR(3,1);
	Matrix VBEG_NEW(3,1);
	Matrix TEI(3,3);
	Matrix TGI(3,3);
	Matrix TEG(3,3);

	//localized module-variables
	double dvbe(0);
	double psivg(0);
	double thtvg(0);
	double lonx(0);
	double latx(0);
	double alt(0);
	double psivgx(0);
	double thtvgx(0);
	double altx(0);
	Matrix TVG(3,3);
	Matrix TGE(3,3);
	
	//localizing module-variables
	//input from initialization
	Matrix WEII=round3[27].mat();
	//state variables
	Matrix SBEG=round3[31].vec();
	Matrix VBEG=round3[32].vec();
	Matrix SBII=round3[35].vec();
	Matrix VBII=round3[36].vec();
	Matrix ABII=round3[37].vec();
	//restore saved values
	Matrix TGV=round3[22].mat();
	Matrix TIG=round3[23].mat();
	//input from other modules
	double time=round3[0].real();
	Matrix FSPV=round3[10].vec();
	double grav=round3[11].real();
	//-------------------------------------------------------------------------
	//building gravitational vector in geographic coordinates
	GRAV.assign_loc(2,0,grav);

	//integrating inertial state variables
	ABII_NEW=TIG*((TGV*FSPV)+GRAV);
	VBII_NEW=integrate(ABII_NEW,ABII,VBII,int_step);
	SBII=integrate(VBII_NEW,VBII,SBII,int_step);
	ABII=ABII_NEW;
	VBII=VBII_NEW;

	//inertial position in earth coordinates
	TEI=cadtei(time);
	SBIE=TEI*SBII;

	//getting lon, lat and alt
	TEMP=cadsph(SBIE);
	lon=TEMP.get_loc(0,0);
	lat=TEMP.get_loc(1,0);
	alt=TEMP.get_loc(2,0);
	lonx=lon*DEG;
	latx=lat*DEG;
	altx=alt/1000;
	
	//calculating TM of geographic wrt earth coordinates
	TGE=cadtge(lon,lat);

	//calculating TM of geographic wrt inertial coordinates
	TGI=TGE*TEI;

	//calculating geographic velocity VBEG=TGI*(VBII-(WEII*SBII));
	VBEG_NEW=TGI*(VBII-(WEII*SBII));

	//and integrating to obtain geographic displacement wrt initial launch point E
	//(SBEG should only be used for diagnostics!)
	SBEG=integrate(VBEG_NEW,VBEG,SBEG,int_step);
	VBEG=VBEG_NEW;

	//getting speed, heading and flight path angle
	POLAR=VBEG.pol_from_cart();		
	dvbe=POLAR.get_loc(0,0);
	psivg=POLAR.get_loc(1,0);
	thtvg=POLAR.get_loc(2,0);
	psivgx=psivg*DEG;
	thtvgx=thtvg*DEG;

	//preparing TMs for output
	TIG=TGI.trans();
	TVG=mat2tr(psivg,thtvg);
	TGV=TVG.trans();
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	round3[31].gets_vec(SBEG);
	round3[32].gets_vec(VBEG);
	round3[35].gets_vec(SBII);
	round3[36].gets_vec(VBII);
	round3[37].gets_vec(ABII);
	//saving variables
	round3[22].gets_mat(TGV);
	round3[23].gets_mat(TIG);
	//output to other modules
	round3[25].gets(dvbe);
	round3[17].gets(psivg);
	round3[18].gets(thtvg);
	round3[21].gets(alt);
	round3[28].gets(psivgx);
	round3[29].gets(thtvgx);
	//diagnostics
	round3[19].gets(lonx);
	round3[20].gets(latx);
	round3[33].gets_mat(TGE);
	round3[34].gets(altx);
}
