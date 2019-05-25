///////////////////////////////////////////////////////////////////////////////
//FILE: 'actuator.cpp'
//Contains 'actuator' module of class 'Hyper'
//
//030515 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of actuator module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[600-649]
//
// mact=|morder|mvehicle|
//
//		 morder = 0 no dynamics, fins are position limited 
//				= 2 second order dynamics, fins are position and rate limited 
//				mvehicle = 1 not used
//						 = 2 Rocket
//
//030515 Created by Peter H Zipfel
//050405 New formulation,  1-6 fin actuator model, PZi
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_actuator()
{
	//Definition and initialization of module-variables
	hyper[600].init("mact","int",0,"mact=|morder|mvehicle|, see table","actuator","data","");
	hyper[601].init("num_fins","int",0,"Number of fins - ND","actuator","data","");
	hyper[602].init("dlimx",0,"Control fin limiter - deg","actuator","data","");
	hyper[603].init("dlimx_min",0,"Minimum fin limiter (optional, usually negative) - deg","actuator","data","");
	hyper[604].init("ddlimx",0,"Control fin rate limiter - deg/s","actuator","data","");
	hyper[605].init("wnact",0,"Natural frequency of actuator - rad/s","actuator","data","plot");
	hyper[606].init("zetact",0,"Damping of actuator - ND","actuator","data","");
	hyper[607].init("mvehicle","int",0,"=1:Unused; =2:X51","actuator","out","");
	hyper[608].init("factwnact",0,"Fact to change bandwidth wnact*(1+factwnact) - ND","actuator","data","");
	hyper[609].init("wnact_limit",0,"Maximum bandwidth limit - rad/s","actuator","data","");
	hyper[619].init("delax",0,"Aileron control deflection - deg","actuator","out","scrn,plot");
	hyper[620].init("delex",0,"Elevator control deflection - deg","actuator","out","scrn,plot");
	hyper[621].init("delrx",0,"Rudder control deflection - deg","actuator","out","scrn,plot");
	hyper[622].init("elvlx",0,"Left elevon deflection - deg","actuator","diag","");
	hyper[623].init("elvrx",0,"Right elevon deflection - deg","actuator","diag","");
	hyper[624].init("elvlcx",0,"Commanded left elevon deflection - deg","actuator","diag","");
	hyper[625].init("elvrcx",0,"Commanded right elevon deflection - deg","actuator","diag","");
	hyper[630].init("DXD",0,0,0,"Fin position 1-3 derivative - deg/s","actuator","state","");
	hyper[631].init("DX",0,0,0,"Fin position 1-3 - deg","actuator","state","");
	hyper[632].init("DDXD",0,0,0,"Fin rate derivative 1-3 - deg/s^2","actuator","state","");
	hyper[633].init("DDX",0,0,0,"Fin rate 1-3 - deg/s","actuator","state","");
	hyper[634].init("DYD",0,0,0,"Fin position derivative 4-6 - deg/s","actuator","state","");
	hyper[635].init("DY",0,0,0,"Fin position 4-6 - deg","actuator","state","");
	hyper[636].init("DDYD",0,0,0,"Fin rate derivative 4-6 - deg/s^2","actuator","state","");
	hyper[637].init("DDY",0,0,0,"Fin rate 4-6 - deg/s","actuator","state","");
	hyper[638].init("delx1",0,"Fin 1 deflection - deg","actuator","out","");
	hyper[639].init("delx2",0,"Fin 2 deflection - deg","actuator","out","");
	hyper[640].init("delx3",0,"Fin 3 deflection - deg","actuator","out","");
	hyper[641].init("delx4",0,"Fin 4 deflection - deg","actuator","out","");
	hyper[642].init("delx5",0,"Fin 5 deflection - deg","actuator","out","");
	hyper[643].init("delx6",0,"Fin 6 deflection - deg","actuator","out","");
}	

///////////////////////////////////////////////////////////////////////////////
//Actuator module
//Member function of class 'Hyper'
// Converts from control deflections to fin deflections
// Calls actuator dynamic subroutine
// Limits fins excursions and converts back to control deflections, as applicable
//
// mact=|morder|mvehicle|
//
//		 morder = 0 no dynamics, fins are position limited 
//				= 2 second order dynamics, fins are position and rate limited 
//				mvehicle = 1 not used
//						 = 2 Rocket
//
//030515 Created by Peter H Zipfel
//050405 New formulation,  1-6 fin actuator model, PZi
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::actuator(double int_step)
{
	//local variable
	Matrix ACTCZ(6,1);
	Matrix ACTZ(6,1);
	double delcx1(0);
	double delcx2(0);
	double delcx3(0);
	double delcx4(0);
	double delcx5(0);
	double delcx6(0);

	//local module-variables
	int mvehicle(0);
	double delax(0),delex(0),delrx(0);
	double elvlx(0),elvrx(0);
	double elvlcx(0),elvrcx(0);
	double delx1(0);
	double delx2(0);
	double delx3(0);
	double delx4(0);
	double delx5(0);
	double delx6(0);

	//localizing module-variables
	//input data
	int mact=hyper[600].integer();
	double dlimx=hyper[602].real();
	double dlimx_min=hyper[603].real();
	int num_fins=hyper[607].integer();
	//input from other modules
	double delacx=hyper[519].real();
	double delecx=hyper[520].real();
	double delrcx=hyper[521].real();
	//-------------------------------------------------------------------------
	//decoding actuator flag
    int morder=mact/10;
    mvehicle=(mact%10);

	//Rocket
	if(mvehicle==2){
		num_fins=4;
		dlimx_min=-dlimx;
		//converting to four fin deflections
		delcx1=-delacx+delecx-delrcx;
		delcx2=-delacx+delecx+delrcx;
		delcx3=+delacx+delecx-delrcx;
		delcx4=+delacx+delecx+delrcx;
		ACTCZ.assign_loc(0,0,delcx1);
		ACTCZ.assign_loc(1,0,delcx2);
		ACTCZ.assign_loc(2,0,delcx3);
		ACTCZ.assign_loc(3,0,delcx4);

		if(morder==0)
			//no dynamics
			ACTZ=actuator_0th(ACTCZ,dlimx,dlimx_min,num_fins);
		else if(morder==2)
			//second order dynamics
			ACTZ=actuator_scnd(ACTCZ,dlimx,dlimx_min,num_fins,int_step);

		delx1=ACTZ.get_loc(0,0);
		delx2=ACTZ.get_loc(1,0);
		delx3=ACTZ.get_loc(2,0);
		delx4=ACTZ.get_loc(3,0);
		//converting to control deflections
		delax=(-delx1-delx2+delx3+delx4)/4;
		delex=(+delx1+delx2+delx3+delx4)/4;
		delrx=(-delx1+delx2-delx3+delx4)/4;
	}
//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[607].gets(mvehicle);
	hyper[619].gets(delax);
	hyper[620].gets(delex);
	hyper[621].gets(delrx);
	hyper[622].gets(elvlx);
	hyper[623].gets(elvrx);
	hyper[624].gets(elvlcx);
	hyper[625].gets(elvrcx);
	hyper[638].gets(delx1);
	hyper[639].gets(delx2);
	hyper[640].gets(delx3);
	hyper[641].gets(delx4);
	hyper[642].gets(delx5);
	hyper[643].gets(delx6);
}	
///////////////////////////////////////////////////////////////////////////////
//No actuator dynamics
//Member function of class 'Hyper'
// Models 6 control surfaces with position limiters
//
//Return output
//		ACTZ(6x1) = 6 control surface deflections (some may be zero, unused)
//Parameter input
//		ACTCZ(6x1) = 6 control surfaces commanded deflections (some may be zero, unused)		
//
//050406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::actuator_0th(Matrix ACTCZ,double dlimx,double dlimx_min,int num_fins)
{	
	Matrix ACTZ=ACTCZ;
	//limiting surface deflections
	for(int i=0;i<num_fins;i++){
	//limiting position and the fin rate derivative
		if(ACTZ[i]>dlimx){
			ACTZ[i]=dlimx;
		}
		if(ACTZ[i]<dlimx_min){
			ACTZ[i]=dlimx_min;
		}
	}
	return ACTZ;
}
///////////////////////////////////////////////////////////////////////////////
//Second order actuator
//Member function of class 'Hyper'
// Models second order lags of up to 6 control surfaces
// Limits fin positions
// Limits fin rates
//
//Return output
//		ACTZ(6x1) = 6 control surface deflections (some may be zero, unused)
//Parameter input
//		ACTCZ(6x1) = 6 control surfaces commanded deflections (some may be zero, unused)		
//
//050405 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::actuator_scnd(Matrix ACTCZ,double dlimx,double dlimx_min,int num_fins, double int_step)
{	
	//local variables
	Matrix DZD_NEW(6,1);
	Matrix DDZD_NEW(6,1);
	Matrix ACTZ(6,1);
	Matrix DZD(6,1);
	Matrix DZ(6,1);
	Matrix DDZD(6,1);
	Matrix DDZ(6,1);

	//localizing module-variables
	//input data
	double ddlimx=hyper[604].real();
	double wnact=hyper[605].real();
	double zetact=hyper[606].real();
	double factwnact=hyper[608].real();
	double wnact_limit=hyper[609].real();
	//input from other modules
//!	double pdynmc=round6[57].real();
//!	int maero=hyper[100].integer();
	//state variables
	Matrix DXD=hyper[630].vec();
	Matrix DX=hyper[631].vec();
	Matrix DDXD=hyper[632].vec();
	Matrix DDX=hyper[633].vec();
	Matrix DYD=hyper[634].vec();
	Matrix DY=hyper[635].vec();
	Matrix DDYD=hyper[636].vec();
	Matrix DDY=hyper[637].vec();
	//packing state variables
	for(int n=0;n<3;n++){
		DZD[n]=DXD[n];
		DZD[n+3]=DYD[n];
		DZ[n]=DX[n];
		DZ[n+3]=DY[n];
		DDZD[n]=DDXD[n];
		DDZD[n+3]=DDYD[n];
		DDZ[n]=DDX[n];
		DDZ[n+3]=DDY[n];
	}
	//-------------------------------------------------------------------------
	//second order integrations for each surface
	for(int i=0;i<num_fins;i++){

	//limiting fin position, rate, and rate derivative
		if(DZ[i]>dlimx){
			DZ[i]=dlimx;
			if(DZ[i]*DDZ[i]>0) DDZ[i]=0;
		}
		if(DZ[i]<dlimx_min){
			DZ[i]=dlimx_min;
			if(DZ[i]*DDZ[i]>0) DDZ[i]=0;
		}
		//limiting fin rate
		int iflag=0;
		if(fabs(DDZ[i])>ddlimx){
			iflag=1;
			DDZ[i]=ddlimx*sign(DDZ[i]);
		}
		//state integration
		DZD_NEW[i]=DDZ[i];
		DZ[i]=integrate(DZD_NEW[i],DZD[i],DZ[i],int_step);
		DZD[i]=DZD_NEW[i];
		double edx=ACTCZ[i]-DZ[i];
		DDZD_NEW[i]=wnact*wnact*edx-2*zetact*wnact*DZD[i];
		DDZ[i]=integrate(DDZD_NEW[i],DDZD[i],DDZ[i],int_step);
		DDZD[i]=DDZD_NEW[i];

	//limiting fin position, rate, and rate derivative
		if(DZ[i]>dlimx){
			DZ[i]=dlimx;
			if(DZ[i]*DDZ[i]>0) DDZ[i]=0;
		}
		if(DZ[i]<dlimx_min){
			DZ[i]=dlimx_min;
			if(DZ[i]*DDZ[i]>0) DDZ[i]=0;
		}
		//limiting fin rate
		iflag=0;
		if(fabs(DDZ[i])>ddlimx){
			iflag=1;
			DDZ[i]=ddlimx*sign(DDZ[i]);
		}
		//setting fin rate derivative to zero if rate is limited
		if(iflag&&DDZ[i]*DDZD[i]>0) DDZD[i]=0;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//unpacking of state variables
	for(int n=0;n<3;n++){
		DXD[n]=DZD[n];
		DYD[n]=DZD[n+3];
		DX[n]=DZ[n];
		DY[n]=DZ[n+3];
		DDXD[n]=DDZD[n];
		DDYD[n]=DDZD[n+3];
		DDX[n]=DDZ[n];
		DDY[n]=DDZ[n+3];
	}
	//state variables
	hyper[630].gets_vec(DXD);
	hyper[631].gets_vec(DX);
	hyper[632].gets_vec(DDXD);
	hyper[633].gets_vec(DDX);
	hyper[634].gets_vec(DYD);
	hyper[635].gets_vec(DY);
	hyper[636].gets_vec(DDYD);
	hyper[637].gets_vec(DDY);
	//diagnostics
	hyper[605].gets(wnact);

	return ACTZ=DZ;
}
