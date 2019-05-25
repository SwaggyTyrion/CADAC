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
// mrpop = 0 no thrusting
//       = 3 constant thrust rocket (input)
//       = 4 constant thrust rocket under LTG control (set in 'guidance' module)
//
//030514 Created by Peter H Zipfel
//040302 Added rocket propulsion, PZi
//041009 Included 'mfreeze' capability, PZi
//050105 Modified for GSWS6, PZi
//060120 Modified for ascent booster, PZi
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_propulsion()
{
	//Definition and initialization of module-variables
    hyper[10].init("mprop","int",0,"=0:none; =3 input; =4 LTG control","propulsion","data","");
    hyper[11].init("acowl",0,"Cowl area of engine inlet - m^2","propulsion","data","");
    hyper[15].init("vmass",0,"Vehicle mass - kg","propulsion","out","scrn,plot");
    hyper[16].init("vmass0",0,"Initial gross mass - kg","propulsion","data","");
    hyper[17].init("xcg",0,"CG location from nose (pos) - m","propulsion","out","plot");
    hyper[18].init("IBBB",0,0,0,0,0,0,0,0,0,"Vehicle moment of inertia - kgm^2","propulsion","out","");
    hyper[21].init("fmass0",0,"Initial fuel mass in stage - kg","propulsion","data","");
    hyper[22].init("fmasse",0,"Fuel mass expended (zero initialization required) - kg","propulsion","state","scrn,plot");
    hyper[23].init("fmassd",0,"Fuel mass expended derivative - kg/s","propulsion","state","");
    hyper[24].init("aexit",0,"Nozzle exit area - m^2","propulsion","data","");
	hyper[25].init("spi",0,"Specific impulse - sec","propulsion","data","");
	hyper[26].init("thrust",0,"Thrust - N","propulsion","out","scrn,plot");
    hyper[27].init("fmassr",0,"Remaining fuel mass - kg","propulsion","save","scrn,plot"); 
    hyper[28].init("xcg_0",0,"Initial cg location from nose - m","propulsion","data","");
    hyper[29].init("xcg_1",0,"Final cg location from nose - m","propulsion","data","");
	hyper[33].init("fuel_flow_rate",0,"Fuel flow rate of rocket motor - kg/s","propulsion","data","");
	hyper[36].init("vmass0_st",0,"Initial mass of exo-vehicle - kg","propulsion","data","");
    hyper[37].init("fmass0_st",0,"Initial fuel mass of exo-vehicle - kg","propulsion","data","");
    hyper[38].init("moi_roll_0",0,"Roll MOI of vehicle, initial - kgm^2","propulsion","data","");
    hyper[39].init("moi_roll_1",0,"Roll MOI of vehicle, burn-out - kgm^2","propulsion","data","");
    hyper[40].init("moi_trans_0",0,"Transverse MOI of vehicle, initial - kgm^2","propulsion","data","");
    hyper[41].init("moi_trans_1",0,"Transverse MOI of vehicle, burn-out - kgm^2","propulsion","data","");
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
//placeholder
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Propulsion module
//Member function of class 'Round6'
// Calculates engine thrust
// Provides dynamic pressure controller for hypersonic propulsion
//
// mrpop = 0 no thrusting
//       = 3 constant thrust rocket (input)
//       = 4 constant thrust rocket under LTG control (set in 'guidance' module)
//
//030514 Created by Peter H Zipfel
//040302 Added rocket propulsion, PZi
//050502 Added provisions for discrete change of MOI, PZi
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////
void Hyper::propulsion(double int_step)
{
	//local variable
	double psl(101325); //sea level pressure - Pa
	double gainq(0);
	Matrix IBBB0(3,3);
	Matrix IBBB1(3,3);

	//local module-variables
	double thrust(0);

	//localizing module-variables
	//input data
	int mprop=hyper[10].integer();
	double vmass0=hyper[16].real();
	double fmass0=hyper[21].real();
	double aexit=hyper[24].real();
	double spi=hyper[25].real();
	double xcg_0=hyper[28].real();
	double xcg_1=hyper[29].real();
	double fuel_flow_rate=hyper[33].real();
	double moi_roll_0=hyper[38].real();
	double moi_roll_1=hyper[39].real();
	double moi_trans_0=hyper[40].real();
	double moi_trans_1=hyper[41].real();
	//getting saved variable
	double vmass=hyper[15].real();
	double xcg=hyper[17].real();
	Matrix IBBB=hyper[18].mat();				 
	double fmassr=hyper[27].real();
	int mfreeze_prop=hyper[42].integer();
	double thrustf=hyper[43].real();
	double vmassf=hyper[44].real();
	Matrix IBBBF=hyper[45].mat();				 
	//state variable
	double fmasse=hyper[22].real();
	double fmassd=hyper[23].real();
	//input from other modules
	double time=round6[0].real();
	double press=round6[52].real();
	int mfreeze=hyper[503].integer();
	//-------------------------------------------------------------------------
	//no thrusting
	if(mprop==0){
		fmassd=0;
		thrust=0;
		fmasse=0;
		fmassr=0;
	}
	//making thrust calculations only if engine is on
	if(mprop>0){

		//constant thrust rocket engine for booster
		if(mprop==3||mprop==4){

			thrust=spi*fuel_flow_rate*AGRAV+(psl-press)*aexit;

			//load MOI of booster
			IBBB0.zero();
			IBBB0.assign_loc(0,0,moi_roll_0);
			IBBB0.assign_loc(1,1,moi_trans_0);
			IBBB0.assign_loc(2,2,moi_trans_0);
			IBBB1.zero();
			IBBB1.assign_loc(0,0,moi_roll_1);
			IBBB1.assign_loc(1,1,moi_trans_1);
			IBBB1.assign_loc(2,2,moi_trans_1);
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

		//interpolating moment of inertia tensor as a function of fuel expended
		double mass_ratio=fmasse/fmass0;
		IBBB=IBBB0+(IBBB1-IBBB0)*mass_ratio;

		//interpolating cg as a function of fuel expended
		xcg=xcg_0+(xcg_1-xcg_0)*mass_ratio;

		//shutting down engine when all fuel is expended
		if(fmassr<=0){
			mprop=0;
			thrust=0;
		}
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
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[22].gets(fmasse);
	hyper[23].gets(fmassd);
	//saving variables
	hyper[10].gets(mprop);
	hyper[27].gets(fmassr);
	hyper[42].gets(mfreeze_prop);
	hyper[43].gets(thrustf);
	hyper[44].gets(vmassf);
	hyper[45].gets_mat(IBBBF);
	//output to other modules
	hyper[15].gets(vmass);
	hyper[17].gets(xcg);
	hyper[18].gets_mat(IBBB);
	hyper[26].gets(thrust);
}
