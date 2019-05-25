///////////////////////////////////////////////////////////////////////////////
//FILE: 'propulsion.cpp'
//Contains 'propulsion' module of class 'Round6'
//
//030514 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'propulsion' module-variables 
//Member function of class 'Round6'
//Module-variable locations are assigned to round6[10-49]
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
//       = 3 Constant thrust rocket under LTG control (set in 'guidance' module)
//       = 4 Constant thrust rocket (input)
//
// This module performs the following functions:
// (1) Provides propulsion deck (capture area ratio CA and ISP)
// (2) Initializes vehicle mass properties
// (3) Sets up fuel mass integration variable
//
//030514 Created by Peter H Zipfel
//040302 Added rocket propulsion, PZi
//041009 Included 'mfreeze' capability, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_propulsion()
{
	//Definition and initialization of module-variables
    hyper[10].init("mprop","int",0,"=0:none; =1:hyper; =2:hyper-auto; =3(LTG)&4(input):rocket","propulsion","data","");
    hyper[11].init("acowl",0,"Cowl area of engine inlet - m^2","propulsion","data","");
    hyper[12].init("throttle",.05,"Throttle controlling fuel/air ratio - ND","propulsion","data/diag","scrn,plot");
    hyper[13].init("thrtl_max",0,"Max throttle - ND","propulsion","data","");
    hyper[14].init("qhold",0,"Dynamic pressure hold command - Pa","propulsion","data","");
    hyper[15].init("vmass",0,"Vehicle mass - kg","propulsion","out","scrn,plot");
    hyper[16].init("vmass0",0,"Initial gross mass - kg","propulsion","data","");
    hyper[18].init("IBBB",0,0,0,0,0,0,0,0,0,"Vehicle moment of inertia - kgm^2","propulsion","out","");
    hyper[19].init("IBBB0",0,0,0,0,0,0,0,0,0,"Initial hypersonic vehicle moment of inertia - kgm^2","propulsion","init","");
    hyper[20].init("IBBB1",0,0,0,0,0,0,0,0,0,"Burn-out hypersonic vehicle moment of inertia - kgm^2","propulsion","init","");
    hyper[21].init("fmass0",0,"Initial fuel mass in stage - kg","propulsion","data","");
    hyper[22].init("fmasse",0,"Fuel mass expended (zero initialization required) - kg","propulsion","state","");
    hyper[23].init("fmassd",0,"Fuel mass expended derivative - kg/s","propulsion","state","");
    hyper[24].init("ca",0,"Capture area factor - ND","propulsion","diag","");
    hyper[25].init("spi",0,"Specific impulse - sec","propulsion","diag","");
    hyper[26].init("thrust",0,"Thrust - N","propulsion","out","");
    hyper[27].init("mass_flow",0,"Mass flow through hypersonic engine - kg/s","propulsion","diag","");
    hyper[28].init("fmassr",0,"Remaining fuel mass - kg","propulsion","diag","scrn,plot");
    hyper[29].init("thrustx",0,"Thrust in kN","propulsion","diag","scrn,plot");
    hyper[30].init("tq",0,"Autothrottle time constant - sec","propulsion","data","");
    hyper[31].init("thrtl_idle",0,"Idle throttle - ND","propulsion","data","");
    hyper[33].init("fuel_flow_rate",0,"Fuel flow rate of rocket motor - kg/s","propulsion","data","");
    hyper[36].init("vmass0_st",0,"Initial mass of exo-vehicle - kg","propulsion","data","");
    hyper[37].init("fmass0_st",0,"Initial fuel mass of exo-vehicle - kg","propulsion","data","");
    hyper[38].init("moi_roll_exo_0",0,"Roll MOI of exo-vehicle, initial - kgm^2","propulsion","data","");
    hyper[39].init("moi_roll_exo_1",0,"Roll MOI of exo-vehicle, burn-out - kgm^2","propulsion","data","");
    hyper[40].init("moi_trans_exo_0",0,"Transverse MOI of exo-vehicle, initial - kgm^2","propulsion","data","");
    hyper[41].init("moi_trans_exo_1",0,"Transverse MOI of exo-vehicle, burn-out - kgm^2","propulsion","data","");
	hyper[42].init("mfreeze_prop","int",0,"Saving 'mfreeze' value","propulsion","save","");
    hyper[43].init("thrustf",0,"Saved thrust when mfreeze=1 - N ","propulsion","save","");
    hyper[44].init("vmassf",0,"Saved mass when mfreeze=1 - N ","propulsion","save","");
    hyper[45].init("IBBBF",0,0,0,0,0,0,0,0,0,"Saved MOI when mfreeze=1 - kgm^2","propulsion","save","");
}	

///////////////////////////////////////////////////////////////////////////////
//Propulsion initialization module
//Member function of class 'Round6'
// Initializes mass properties
//
//030514 Created by Peter H Zipfel
//040302 Added rocket propulsion, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::init_propulsion()
{
	//local module-variables
	double vmass(0);
	double vmass0_st(0);
	double fmass0_st(0);
	Matrix IBBB(3,3);				 
	Matrix IBBB0(3,3);				 
	Matrix IBBB1(3,3);				 

	//localizing module-variables
	int mprop=hyper[10].integer();
	double vmass0=hyper[16].real();

	//-------------------------------------------------------------------------
	//initialization of mass properties
	//for the hypersonic vehicle with exo-vehicle in cargo bay	
	vmass=vmass0;
	IBBB0.assign_loc(0,0,1.573e6);
	IBBB0.assign_loc(1,1,31.6e6);
	IBBB0.assign_loc(2,2,32.54e6);
	IBBB0.assign_loc(0,2,0.38e6);
	IBBB0.assign_loc(2,0,0.38e6);

	IBBB1.assign_loc(0,0,1.18e6);
	IBBB1.assign_loc(1,1,19.25e6);
	IBBB1.assign_loc(2,2,20.2e6);
	IBBB1.assign_loc(0,2,0.24e6);
	IBBB1.assign_loc(2,0,0.24e6);

	IBBB=IBBB0;

	//-------------------------------------------------------------------------
	//loading module-variables
	//initialization
	hyper[15].gets(vmass);
	hyper[18].gets_mat(IBBB);
	hyper[19].gets_mat(IBBB0);
	hyper[20].gets_mat(IBBB1);
	hyper[36].gets(vmass0_st);
	hyper[37].gets(fmass0_st);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Propulsion module
//Member function of class 'Round6'
// Calculates engine thrust
// Provides dynamic pressure controller
//
// mrpop = 0 No thrusting
//       = 1 Hypersonic propulsion throttle command (input)
//       = 2 Hypersonic propulsion autothrottle (input)
//       = 3 Constant thrust rocket under LTG control (set in 'guidance' module)
//       = 4 Constant thrust rocket (input)
//
//030514 Created by Peter H Zipfel
//040302 Added rocket propulsion, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::propulsion(double int_step)
{
	//local variable 
	double gainq(0);
	//local module-variables
	double spi(0);
	double ca(0);
	double thrust(0);
	double mass_flow(0);
	double thrustx(0);

	//localizing module-variables
	//input data
	int mprop=hyper[10].integer();
	double throttle=hyper[12].real();
	double qhold=hyper[14].real();
	double vmass0=hyper[16].real();
	double fmass0=hyper[21].real();
	double tq=hyper[30].real();
	double thrtl_idle=hyper[31].real();
	double fuel_flow_rate=hyper[33].real();
	double moi_roll_exo_0=hyper[38].real();
	double moi_roll_exo_1=hyper[39].real();
	double moi_trans_exo_0=hyper[40].real();
	double moi_trans_exo_1=hyper[41].real();
	//from initialization
	double acowl=hyper[11].real();
	double thrtl_max=hyper[13].real();
	Matrix IBBB=hyper[18].mat();				 
	Matrix IBBB0=hyper[19].mat();				 
	Matrix IBBB1=hyper[20].mat();
	double vmass0_st=hyper[36].real();
	double fmass0_st=hyper[37].real();
	//getting saved variable
	double vmass=hyper[15].real();
	double fmassr=hyper[28].real();
	int mfreeze_prop=hyper[42].integer();
	double thrustf=hyper[43].real();
	double vmassf=hyper[44].real();
	Matrix IBBBF=hyper[45].mat();				 
	//state variable
	double fmasse=hyper[22].real();
	double fmassd=hyper[23].real();
	//input from other modules
	double time=round6[0].real();
	double rho=round6[53].real();
	double vmach=round6[56].real();
	double pdynmc=round6[57].real();
	double dvba=round6[75].real();
	double refa=hyper[104].real();
	double cd=hyper[110].real();
	double alphax=round6[144].real();
	double isp_fuel=hyper[472].real();
	double burntime=hyper[473].real();
	int mfreeze=hyper[503].integer();
	//-------------------------------------------------------------------------

	//making thrust calculations only if engine is on
	if(mprop>0){

		//hypersonic propulsion table look-up
		if(mprop==1||mprop==2){
			//spi table look-up
			spi=proptable.look_up("spi_vs_throttle_mach",throttle,vmach);

			//capture area coefficient look-up
			ca=proptable.look_up("ca_vs_alpha_mach",alphax,vmach);
		}
		//hypersonic propulsion with fixed throttle
		if(mprop==1){
			thrust=spi*0.029*throttle*AGRAV*rho*dvba*ca*acowl;
		}
		//hypersonic propulsion with auto throttle
		if(mprop==2){
			double denom=0.029*spi*AGRAV*rho*dvba*ca*acowl;
			if(denom!=0){
				double thrst_req=refa*cd*qhold/cos(alphax*RAD);
				double throtl_req=thrst_req/denom;
				double gainq=2*vmass/(rho*dvba*denom*tq);
				double ethrotl=gainq*(qhold-pdynmc);
				throttle=ethrotl+throtl_req;
			}
			//throttle limiters
			if(throttle<0) throttle=thrtl_idle;				
			if (throttle>thrtl_max) throttle=thrtl_max;
			
			//calculating thrust
			spi=proptable.look_up("spi_vs_throttle_mach",throttle,vmach);
			thrust=spi*0.029*throttle*AGRAV*rho*dvba*ca*acowl;
		}
		//constant thrust rocket engine for exo-vehicle
		if(mprop==3||mprop==4){
			spi=isp_fuel;
			if(mprop==3){
				//motor under LTG control
				fuel_flow_rate=fmass0/burntime;
				thrust=spi*fuel_flow_rate*AGRAV;
			}
			else if(mprop==4){
				//motor not under LTG control
				thrust=spi*fuel_flow_rate*AGRAV;
			}
			//load MOI of exo-vehicles (T.V. or interceptor)
			IBBB0.zero();
			IBBB0.assign_loc(0,0,moi_roll_exo_0);
			IBBB0.assign_loc(1,1,moi_trans_exo_0);
			IBBB0.assign_loc(2,2,moi_trans_exo_0);
			IBBB1.zero();
			IBBB1.assign_loc(0,0,moi_roll_exo_1);
			IBBB1.assign_loc(1,1,moi_trans_exo_1);
			IBBB1.assign_loc(2,2,moi_trans_exo_1);
		}
		//calculating fuel consumption
		if (spi!=0){
			double fmassd_next=thrust/(spi*AGRAV);
			fmasse=integrate(fmassd_next,fmassd,fmasse,int_step);
			fmassd=fmassd_next;
		}
		//calculating vehicle mass, mass flow, and fuel mass remaining
		vmass=vmass0-fmasse;
		fmassr=fmass0-fmasse;

		//interpolating moment of inertia tensor as a fucntion of fuel expended
		double mass_ratio=fmasse/fmass0;
		IBBB=IBBB0+(IBBB1-IBBB0)*mass_ratio;

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
		thrustx=0;
	}
	//freezing variables for autopilot response calculations
	if(mfreeze==0)
	    mfreeze_prop=0;
	else{
		if(mfreeze!=mfreeze_prop){
		  mfreeze_prop=mfreeze;
		  thrustf=thrust;
		  vmassf=vmass;
		  IBBBF=IBBB;
		}
	    thrust=thrustf;
	    vmass=vmassf;
		IBBB=IBBBF;
	}
	//diagnostics: thrust in kN
	thrustx=thrust/1000;

	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[22].gets(fmasse);
	hyper[23].gets(fmassd);
	//saving variable
	hyper[10].gets(mprop);
	hyper[42].gets(mfreeze_prop);
	hyper[43].gets(thrustf);
	hyper[44].gets(vmassf);
	hyper[45].gets_mat(IBBBF);				 
	//output to other modules
	hyper[15].gets(vmass);
	hyper[18].gets_mat(IBBB);
	hyper[26].gets(thrust);
	//diagnostics
	hyper[12].gets(throttle);
	hyper[24].gets(ca);
	hyper[25].gets(spi);
	hyper[27].gets(mass_flow);
	hyper[28].gets(fmassr);
	hyper[29].gets(thrustx);

}
