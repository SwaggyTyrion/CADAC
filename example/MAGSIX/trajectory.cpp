///////////////////////////////////////////////////////////////////////////////
//FILE: 'trajectory.cpp'
//Contains 'trajectory' module of class 'Rotor'
//
//130523 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'trajectory' module-variables 
//Member function of class 'Rotor'
//Module-variable locations are assigned to rotor[100-199]
//
//Equations of motion of the trajectory (center of mass)
//Equations are in dynamic normalized units (DNU) and are dimensionless
//Ref: Zipfel, 'On Flight Dynamics of Magnus Rotors',
// DTIC AD 716345, NOv 1970,Chapter 5, Normalization, p 43
// DNT = dynamic normalized time unit is dimensionless
// DNU = dynamic normalized units are dimensionless
// 
//130523 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Rotor::def_trajectory()
{
	//Definition and initialization of module-variables
	rotor[0].init("time",0,"Real time - sec","trajectory","dia","scrn,plot");
	rotor[1].init("sim_time",0,"Simulation time - DNT","trajectory","exec","scrn,plot");
	rotor[110].init("cd",0,"Drag coefficient - ND","trajectory","data","");
    rotor[111].init("cmdw",0,"Damping spin torque - rad","trajectory","data","");
    rotor[112].init("clw",0,"Magnus lift coefficient - rad","trajectory","data","");
    rotor[113].init("cma",0,"Acceleration spin torque - ND","trajectory","data","");
    rotor[114].init("mass",0,"Rotor mass - kg","trajectory","data","");
    rotor[115].init("ref_area",0,"Reference area - m^2","trajectory","data","");
    rotor[116].init("ref_length",0,"Reference length - m","trajectory","data","");
    rotor[118].init("velocity_ss",0,"Flight speed in steady-state - m/s","trajectory","out","");  
    rotor[119].init("gamma_ss",0,0,0," Flight path angle in steady-state - rad","trajectory","out","");
	rotor[120].init("omega_ss",0,"Spin rate in steady-state - rad/s","trajectory","out","");
    rotor[123].init("dvbe",0,"Rotor speed - m/s","trajectory","in/dia","scrn,plot");
    rotor[124].init("psivlx",0,"Heading angle - deg","trajectory","in/dia","scrn,plot");
    rotor[125].init("thtvlx",0,"Vertical flight path angle - deg","trajectory","in/dia","scrn,plot");
	rotor[127].init("hbg",0,"Height of ground - m","trajectory","data",""); 
    rotor[128].init("hbe",0,"Height above sea level - m","trajectory","in/dia","scrn,plot");
    rotor[129].init("omega",0,"Spin rate - rad/s","trajectory","in/out","");
    rotor[130].init("sbel1",0,"Initial north comp of SBEL - m","trajectory","in","");
    rotor[131].init("sbel2",0,"Initial east comp of SBEL - m","trajectory","in","");
    rotor[132].init("sbel3",0,"Initial down comp of SBEL - m","trajectory","in","");
    rotor[133].init("velocityx",0,"Rotor speed - DNU","trajectory","state","plot");
    rotor[134].init("velocityxd",0,"Derivative velocityx - DNU","trajectory","state","");
	rotor[135].init("gamma",0,"Rotor glide angle - rad","trajectory","state","plot");
	rotor[136].init("gammaxd",0,"Der of gamma - DNU","trajectory","state","");
	rotor[137].init("omegax",0,"Rotor spin rate - DNU","trajectory","state","plot");
	rotor[138].init("omegaxd",0,"Derivatgive of omegaxd - DNU","trajectory","state","");
    rotor[139].init("moi_spin",0,"Spin moment of intertia -DNU","trajectory","out","");
    rotor[140].init("moi_spinx",0,"Spin moment of intertia - kg.m^2","trajectory","data","");
    rotor[141].init("SBEL",0,0,0,"Rotor pos. wrt point E in L coord - m","trajectory","state","plot");
    rotor[142].init("SBELD",0,0,0,"Derivative of SBEL - m","trajectory","state","");
	rotor[143].init("VBEL",0,0,0,"Rotor vel. wrt point E in L coord - m/DNT","trajectory","dia","plot");
	rotor[144].init("omega_rpm",0,"Rotor spin rate - RPM","trajectory","dia","scrn,plot");
	rotor[145].init("tau",0,"Time parameter - sec","trajectory","out","");
	rotor[146].init("mu",0,"Mass parameter - ND","trajectory","out","");
	rotor[147].init("tpsp_ratio",0,"Tip speed ratio - ND","trajectory","dia","plot,scrn");
}	
///////////////////////////////////////////////////////////////////////////////
//Initialization of 'trajectory' module
//Member function of class 'Rotor'
//
//Initializing state variables
//Calculating stead-state velocity, glide angle, and spin rate
// 
//130523 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Rotor::init_trajectory()
{
	//local variables ('x' indicates dynamic normalized variable)
	Matrix TVL(3,3);
	double velocityx(0);
	double gamma(0);
	double omegax(0);
	double tau(0);

	//local module-variables
	Matrix SBEL(3,1);
	double time(0);

	Matrix VBEL(3,1);

	//localizing module-variables
	//input data
	double cd=rotor[110].real();
	double cmdw=rotor[111].real();
	double clw=rotor[112].real();
	double cma=rotor[113].real();
	double mass=rotor[114].real();
	double ref_area=rotor[115].real();
	double ref_length=rotor[116].real();
	double dvbe=rotor[123].real();
	double psivlx=rotor[124].real();
	double thtvlx=rotor[125].real();
	double hbe=rotor[128].real();
	double sbel1=rotor[130].real();
	double sbel2=rotor[131].real();
	double sbel3=rotor[132].real();
	double omega_rpm=rotor[144].real();
	//-------------------------------------------------------------------------
	//steady-state glide angle
	double gamma_ss=atan(cd*cmdw/(clw*cma));

	//steady-state Velocity
	double velocity_ss=sqrt(2*AGRAV*mass*fabs(sin(gamma_ss))/(RHO_SL*ref_area*cd));

	//steady-state spin rate
	double omega_ss=-velocity_ss*cma/(ref_length*cmdw);

	//initial displacement vector in local level coord.
	SBEL.build_vec3(sbel1,sbel2,-hbe);

	//initial velocity vector in local level coord.
	TVL=mat2tr(psivlx*RAD,thtvlx*RAD);
	Matrix VBEV(3,1); VBEV.build_vec3(dvbe,0,0);
	VBEL=~TVL*VBEV;

	//*initialization of state variables
	velocityx=dvbe/velocity_ss;
	gamma=thtvlx*RAD;
	//get air density from US 1976 Standard Atmosphere at launch altitude
	double rho(0),press(0),tempk(0);
	atmosphere76(rho,press,tempk, hbe);
	//time parameter
	tau=2*mass/(rho*ref_area*velocity_ss);
	//spin state variable
	omegax=(omega_rpm/RPM)*tau;

	//writing information to console
	cout<<"\n"<<" *** Steady-State:";
	cout<<"  Speed(@SL) = "<<velocity_ss<<" m/s  Glide angle = "<<gamma_ss*DEG<<" deg  Spin(@SL) = "<<omega_ss*RPM<<" RPM tau(@SL) =" <<tau<<" sec\n\n";    
	//-------------------------------------------------------------------------
	//loading module-variables
	//output
	rotor[118].gets(velocity_ss);
	rotor[119].gets(gamma_ss);
	rotor[120].gets(omega_ss);
	rotor[145].gets(tau);
	//state variables
	rotor[133].gets(velocityx);
	rotor[135].gets(gamma);
	rotor[137].gets(omegax);
	rotor[141].gets_vec(SBEL);
	//diagnostics
	rotor[0].gets(time);
	rotor[128].gets(hbe);
	rotor[143].gets_vec(VBEL);
}
///////////////////////////////////////////////////////////////////////////////
//Planar trajectory equations module
//Member function of class 'Rotor'
//
//Trajectory equations of motion
//Ref: Zipfel, 'On Flight Dynamics of Magnus Rotors',
//  DTIC AD 716345, NOv 1970, Table 12.1, Eqs. 1, 2, 3 
//
//130526 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Rotor::trajectory(Packet *combus,int vehicle_slot,double sim_time,double int_step)
{
	//local variables
	Matrix GRAVL(3,1);
	double psivl(0);
	Matrix TVL(3,3);
	Matrix VBELX(3,1);
	
	//local module-variables
	double tau(0);
	double mu(0);
	double moi_spinx(0);
	double time(0);
	double dvbe(0);
	double thtvlx(0);
	double omega_rpm(0);
	Matrix VBEL(3,1);
	double hbe(0);
	double omega(0);
	double tpsp_ratio(0);

	//localizing module-variables
	//input data
	double cd=rotor[110].real();
	double cmdw=rotor[111].real();
	double clw=rotor[112].real();
	double cma=rotor[113].real();
	double mass=rotor[114].real();
	double ref_area=rotor[115].real();
	double ref_length=rotor[116].real();
	double psivlx=rotor[124].real();
	double hbg=rotor[127].real();
	double moi_spin=rotor[139].real();
	//initialization
	double velocity_ss=rotor[118].real();
	//input from other modules
	double rho=rotor[53].real();
	double grav=rotor[55].real();
	//state variables
	double velocityx=rotor[133].real();
	double velocityxd=rotor[134].real();
	double gamma=rotor[135].real();
	double gammaxd=rotor[136].real();
	double omegax=rotor[137].real();
	double omegaxd=rotor[138].real();
	Matrix SBEL=rotor[141].vec();
	Matrix SBELD=rotor[142].vec();
	//-----------------------------------------------------------------------------	
	//*parameters used in differential equations
	//time parameter
	tau=2*mass/(rho*ref_area*velocity_ss);
	//mass parameter
	mu=2*mass/(rho*ref_area*ref_length);
	//spin moment of inertia in DNU
	moi_spinx=moi_spin/(ref_length*ref_length*mu*mu*mass);

	//*trajectory equations of motion
	//velocity in Dynamic Normalized Units (DNU)
	double velocityxd_new=-cd*velocityx*velocityx-tau*grav*sin(gamma)/velocity_ss;
	velocityx=integrate(velocityxd_new,velocityxd,velocityx,int_step);
	velocityxd=velocityxd_new;
	//glide path angle in DNU
	double gammaxd_new=clw*omegax/mu-tau*grav*cos(gamma)/(velocity_ss*velocityx);
	gamma=integrate(gammaxd_new,gammaxd,gamma,int_step);
	gammaxd=gammaxd_new;
	//spin rate in DNU
	double omegaxd_new=cma*pow(velocityx,2)/(mu*moi_spinx)+cmdw*velocityx*omegax/(mu*mu*moi_spinx);
	omegax=integrate(omegaxd_new,omegaxd,omegax,int_step);
	omegaxd=omegaxd_new;

	//*converting to metric units
	//speed - m/s
	dvbe=velocityx*velocity_ss;
	//glide angle - deg
	thtvlx=gamma*DEG;
	//spin rate - RPM
	omega=omegax/tau;
	omega_rpm=omega*RPM;

	//*cartesian coordinates
	//velocity vector in local level coord.
	TVL=mat2tr(psivlx*RAD,thtvlx*RAD);
	Matrix VBEV(3,1); VBEV.build_vec3(dvbe,0,0);
	VBEL=~TVL*VBEV;
	VBEL=TVL.trans()*VBEV;
	//position vector in local level coord.
	//integration is in real time (sec)
	Matrix SBELD_NEW=VBEL;
	SBEL=integrate(SBELD_NEW,SBELD,SBEL,int_step*tau);
	SBELD=SBELD_NEW;

	//altitude above Earth
	hbe=-SBEL.get_loc(2,0);

	//tip speed ratio
	tpsp_ratio=omega*ref_length/dvbe;

	//output time in seconds
	time=tau*sim_time;

	//output to screen at ground impact
	if(hbe<hbg){
		//getting rotor #
		string id_rotor=combus[vehicle_slot].get_id();

		//downrange at impact
		double range=sqrt(SBEL[0]*SBEL[0]+SBEL[1]*SBEL[1]);

		//writing information to console and stop run
		cout<<"\n"<<" *** Ground impact of rotor_"<<id_rotor<<"  Time = "<<time<<" sec ***\n";
		cout<<"     Altitude = "<<hbe<<" m Speed = "<<dvbe<<" m/s  Heading = "<<psivlx<<" deg   Glide angle = "<<thtvlx<<" deg   Spin = "<<omega_rpm<<" RPM\n";     
		cout<<"     Downrange = "<<range<<" m  SBEL1 = "<<SBEL[0]<<" m  SBEL2 = "<<SBEL[1]<<" m \n\n";

		//declaring rotor 'dead'
		combus[vehicle_slot].set_status(0);
	}

	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	rotor[133].gets(velocityx);
	rotor[134].gets(velocityxd);
	rotor[135].gets(gamma);
	rotor[136].gets(gammaxd);
	rotor[137].gets(omegax);
	rotor[138].gets(omegaxd);
	rotor[141].gets_vec(SBEL);
	rotor[142].gets_vec(SBELD);
	//output to other modules
	rotor[140].gets(moi_spinx);
	rotor[145].gets(tau);
	rotor[146].gets(mu);
	//diagnostics
	rotor[0].gets(time);
	rotor[1].gets(sim_time);
	rotor[123].gets(dvbe);
	rotor[125].gets(thtvlx);
	rotor[128].gets(hbe);
	rotor[129].gets(omega);
	rotor[143].gets_vec(VBEL);
	rotor[144].gets(omega_rpm);
	rotor[147].gets(tpsp_ratio);
}
///////////////////////////////////////////////////////////////////////////////
//Final trajectory calculations
//Member function of class 'Rotor'
//This function is only called, if the run is stopped on ENDTIME
// 
//130523 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Rotor::term_trajectory()
{
	//localizing module-variables
	//output
	double time=rotor[0].real();
	double dvbe=rotor[123].real();
	double psivlx=rotor[124].real();
	double thtvlx=rotor[125].real();
	double hbe=rotor[128].real();
	Matrix SBEL=rotor[141].vec();
	double omega_rpm=rotor[144].real();

	//downrange at impact
	double range=sqrt(SBEL[0]*SBEL[0]+SBEL[1]*SBEL[1]);

	//writing information to console
	cout<<"\n"<<" *** Stopped at time = "<<time<<" sec ***\n";
	cout<<"     Altitude = "<<hbe<<" m Speed = "<<dvbe<<" m/s  Heading = "<<psivlx<<" deg   Glide angle = "<<thtvlx<<" deg   Spin = "<<omega_rpm<<" RPM\n";     
	cout<<"     Downrange = "<<range<<" m  SBEL1 = "<<SBEL[0]<<" m  SBEL2 = "<<SBEL[1]<<" m \n\n";
}