///////////////////////////////////////////////////////////////////////////////
//FILE: 'cruise_modules.cpp'
//Contains all modules of class 'Cruise'
//							aerodynamics()	cruise[30-39]
//							propulsion()	cruise[10-29]
//							forces()		round3[10]
//
//001122 Created by Peter H Zipfel
//001211 Introduced 'Variable' class to manage module-variables, PZi
//060512 Updated variable initialization, PZi
//060424 Included 'targeting' module, PZi
//100505 Modified for GHAME3, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of aerodynamic module-variables 
//Member function of class 'Cruise'
//Module-variable locations are assigned to cruise[30-39]
// 
//Defining and initializing module-variables
//
//001226 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Cruise::def_aerodynamics()
{
	//Definition of module-variables
	cruise[30].init("cl",0,"Lift coefficient - ND","aerodynamics","out","");
	cruise[31].init("cd",0,"Drag coefficient - ND","aerodynamics","out","");
	cruise[32].init("cl_ov_cd",0,"Lift-over-drag ratio - ND","aerodynamics","diag","scrn,plot");
	cruise[33].init("area",0,"Aerodynamic reference area - m^2","aerodynamics","data","");
	cruise[34].init("cla",0,"Lift coefficient slope - 1/deg","aerodynamics","out","");
	cruise[35].init("alphax",0,"Angle of attack - deg","aerodynamics","data","");
	cruise[36].init("phimvx",0,"Bank angle - deg","aerodynamics","data","");
}	
///////////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Cruise'
// Generates drag polar of X30 'Cruise3'
// Area = 557.42 m^2
//
//001023 Created by Peter Zipfel
//001227 Upgraded to module-variable arrays, PZi
///////////////////////////////////////////////////////////////////////////////
void Cruise::aerodynamics()
{
	//local variables
	double cd0(0);
	double cl0(0);
	double ckk(0);
	double cla0(0);
	
	//local module-variables
	double cl(0);
	double cd(0);
	double cl_ov_cd(0);
	double cla(0);

	//localizing module-variables
	//input data
	double alphax=cruise[35].real();
	//input from other modules
	double mach=round3[14].real();
	//-------------------------------------------------------------------------
	cd0=aerotable.look_up("cd0_vs_mach",mach);
	cl0=aerotable.look_up("cl0_vs_mach",mach);
	cla=aerotable.look_up("cla_vs_mach",mach);
	ckk=aerotable.look_up("ckk_vs_mach",mach);
	cla0=aerotable.look_up("cla0_vs_mach",mach);
	cl=cla0+cla*alphax;
	cd=cd0+ckk*pow((cl-cl0),2);
	cl_ov_cd=cl/cd;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	cruise[30].gets(cl);
	cruise[31].gets(cd);
	cruise[34].gets(cla);
	//diagnostics
	cruise[32].gets(cl_ov_cd);
}	
///////////////////////////////////////////////////////////////////////////////
//Definition of 'propulsion' module-variables 
//Member function of class 'Cruise'
//Module-variable locations are assigned to cruise[10-29]
// 
// Hypersonic propulsion
// Ref: "Handbook of Intelligent Control", Chapter 11 "Flight
//       Propulsion, and Thermal Control of Advanced Aircraft and
//       Hypersonic Vehicles", Edited by D.A.White and D.A. Sofge,
//       Van Nostrand Reinhold,New York, NY, 1992
// Type: turbojet 0-2  Mach
//       ramjet   2-6  Mach
//       scramjet 6-12 Mach
// Mass properties: take-off mass 136,077 kg
//                  total fuel mass 81,646 kg
//
// mrpop = 0 No thrusting
//       = 1 Hypersonic propulsion throttle command (input)
//       = 2 Hypersonic propulsion autothrottle (input)
//
// This module performs the following functions:
// (1) Provides propulsion deck (capture area ratio 'ca' and 'isp')
// (2) Initializes vehicle mass properties
// (3) Sets up fuel mass integration variable
//
//100505 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Cruise::def_propulsion()
{
	//Definition and initialization of module-variables
    cruise[10].init("mprop","int",0,"=0:none; =1:fixed-throttle; =2:auto-throttle","propulsion","data","");
    cruise[11].init("acowl",0,"Cowl area of engine inlet - m^2","propulsion","data","");
    cruise[12].init("throttle",0,"Throttle controlling fuel/air ratio - ND","propulsion","data/diag","scrn,plot");
    cruise[13].init("thrtl_max",0,"Max throttle - ND","propulsion","data","");
    cruise[14].init("qhold",0,"Dynamic pressure hold command - Pa","propulsion","data","");
    cruise[15].init("mass",0,"Vehicle mass - kg","propulsion","out","scrn,plot");
    cruise[16].init("mass0",0,"Initial gross mass - kg","propulsion","data","");
    cruise[18].init("tq",0,"Autothrottle time constant - sec","propulsion","data","");
    cruise[19].init("thrtl_idle",0,"Idle throttle - ND","propulsion","data","");
	cruise[21].init("fmass0",0,"Initial fuel mass in stage - kg","propulsion","data","");
    cruise[22].init("fmasse",0,"Fuel mass expended (zero initialization required) - kg","propulsion","state","");
    cruise[23].init("fmassd",0,"Fuel mass expended derivative - kg/s","propulsion","state","");
    cruise[24].init("ca",0,"Capture area factor - ND","propulsion","diag","");
    cruise[25].init("spi",0,"Specific impulse - sec","propulsion","diag","");
    cruise[26].init("thrust",0,"Thrust - N","propulsion","out","scrn,plot");
    cruise[27].init("mass_flow",0,"Mass flow through hypersonic engine - kg/s","propulsion","diag","");
    cruise[28].init("fmassr",0,"Remaining fuel mass - kg","propulsion","diag","scrn,plot");
}	
///////////////////////////////////////////////////////////////////////////////
//Propulsion initialization module
//Member function of class 'Cruise'
// Initializes mass properties
//
//100505 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Cruise::init_propulsion()
{
	//local module-variables
	double mass(0);

	//localizing module-variables
	int mprop=cruise[10].integer();
	double mass0=cruise[16].real();
	//-------------------------------------------------------------------------
	mass=mass0;
	//-------------------------------------------------------------------------
	//loading module-variables
	//initialization
	cruise[15].gets(mass);
}
///////////////////////////////////////////////////////////////////////////////
//Propulsion module
//Member function of class 'Cruise'
// Calculates engine thrust
// Provides dynamic pressure controller
//
// mrpop = 0 No thrusting
//       = 1 Hypersonic propulsion throttle command (input)
//       = 2 Hypersonic propulsion autothrottle (input)
//
//100505 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Cruise::propulsion(double int_step)
{
	//local module-variables
	double spi(0);
	double ca(0);
	double thrust(0);
	double mass_flow(0);

	//localizing module-variables
	//input data
	int mprop=cruise[10].integer();
	double acowl=cruise[11].real();
	double throttle=cruise[12].real();
	double thrtl_max=cruise[13].real();
	double qhold=cruise[14].real();
	double mass0=cruise[16].real();
	double fmass0=cruise[21].real();
	double tq=cruise[18].real();
	double thrtl_idle=cruise[19].real();
	//getting saved variable
	double mass=cruise[15].real();
	double fmassr=cruise[28].real();
	//state variable
	double fmasse=cruise[22].real();
	double fmassd=cruise[23].real();
	//input from other modules
	double rho=round3[12].real();
	double pdynmc=round3[13].real();
	double mach=round3[14].real();
	double dvbe=round3[25].real();
	double cd=cruise[31].real();
	double area=cruise[33].real();
	double alphax=cruise[35].real(); 
	//-------------------------------------------------------------------------
	//making thrust calculations only if engine is on
	if(mprop>0){

		//hypersonic propulsion table look-up
		if(mprop==1||mprop==2){
			//spi table look-up
			spi=proptable.look_up("spi_vs_throttle_mach",throttle,mach);

			//capture area coefficient look-up
			ca=proptable.look_up("ca_vs_alpha_mach",alphax,mach);
		}
		//hypersonic propulsion with fixed throttle
		if(mprop==1){
			thrust=spi*0.029*throttle*AGRAV*rho*dvbe*ca*acowl;
		}
		//hypersonic propulsion with auto throttle
		if(mprop==2){
			double denom=0.029*spi*AGRAV*rho*dvbe*ca*acowl;
			if(denom!=0){
				double thrst_req=area*cd*qhold/cos(alphax*RAD);
				double throtl_req=thrst_req/denom;
				double gainq=2*mass/(rho*dvbe*denom*tq);
				double ethrotl=gainq*(qhold-pdynmc);
				throttle=ethrotl+throtl_req;
			}
			//throttle limiters
			if(throttle<0) throttle=thrtl_idle;				
			if (throttle>thrtl_max) throttle=thrtl_max;
			
			//calculating thrust
			spi=proptable.look_up("spi_vs_throttle_mach",throttle,mach);
			thrust=spi*0.029*throttle*AGRAV*rho*dvbe*ca*acowl;
		}
		//calculating fuel consumption
		if (spi!=0){
			double fmassd_next=thrust/(spi*AGRAV);
			fmasse=integrate(fmassd_next,fmassd,fmasse,int_step);
			fmassd=fmassd_next;
		}
		//calculating vehicle mass, mass flow, and fuel mass remaining
		mass=mass0-fmasse;
		fmassr=fmass0-fmasse;

		//diagnostics: thrust in kN
		mass_flow=thrust/(AGRAV*spi);

		//shutting down engine when all fuel is expended
		if(fmassr<=0)
			mprop=0;
	}
	//no thrusting
	if(mprop==0){
		fmassd=0;
		thrust=0;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	cruise[22].gets(fmasse);
	cruise[23].gets(fmassd);
	//saving variable
	cruise[10].gets(mprop);
	//output to other modules
	cruise[15].gets(mass);
	cruise[26].gets(thrust);
	//diagnostics
	cruise[12].gets(throttle);
	cruise[24].gets(ca);
	cruise[25].gets(spi);
	cruise[27].gets(mass_flow);
	cruise[28].gets(fmassr);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of force module-variables
// Member function of class 'Cruise'
//
//Note that FSPV is entered into the round3[10] array because it is needed
// for the newton module, which is a member of the 'Round3' class
//		
//001129 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
void Cruise::def_forces()
{
	//Definition of module-variables
	round3[10].init("FSPV",0,0,0,"Specific force in V-coord - m/s^2","forces","out","plot");
}
///////////////////////////////////////////////////////////////////////////////
//Force Module 
//Member function of class 'Cruise' 
//Calulates the total force acting on the vehicle
//
//000623 Created by Michael Chiaramonte
//000724 Function calls have been removed, Michael Horvath
//001227 Upgraded to module-variable arrays, PZi
///////////////////////////////////////////////////////////////////////////////
void Cruise::forces()
{
	//localizing module-variables
	//input from other modules
	double pdynmc=round3[13].real();
	Matrix FSPV=round3[10].vec();
	double mass=cruise[15].real();
	double thrust=cruise[26].real();
	double cl=cruise[30].real();
	double cd=cruise[31].real();
	double area=cruise[33].real();
	double alphax=cruise[35].real();
	double phimvx=cruise[36].real();
	//-------------------------------------------------------------------------
	double phimv=phimvx*RAD;
	double alpha=alphax*RAD;

	double fspv1=(-pdynmc*area*cd+thrust*cos(alpha))/mass; 
	double fspv2=sin(phimv)*(pdynmc*area*cl+thrust*sin(alpha))/mass;
	double fspv3=-cos(phimv)*(pdynmc*area*cl+thrust*sin(alpha))/mass;
	
	FSPV.assign_loc(0,0,fspv1);
	FSPV.assign_loc(1,0,fspv2);
	FSPV.assign_loc(2,0,fspv3);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	round3[10].gets_vec(FSPV);
}
