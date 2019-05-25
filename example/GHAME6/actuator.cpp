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
// mact=0 No dynamics with position limiting
//	   =1 First order dynamics (not implemented)
//     =2 Second order dynamics with position and rate limiting
//
//030515 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_actuator()
{
	//Definition and initialization of module-variables
	hyper[600].init("mact","int",0,"=0:no dynamics, =2:second order","actuator","data","");
	hyper[602].init("dlimx",0,"Control fin limiter - deg","actuator","data","");
	hyper[604].init("ddlimx",0,"Control fin rate limiter - deg/s","actuator","data","");
	hyper[605].init("wnact",0,"Natural frequency of actuator - rad/s","actuator","data","");
	hyper[606].init("zetact",0,"Damping of actuator - ND","actuator","data","");
	hyper[619].init("delax",0,"Aileron control deflection - deg","actuator","out","scrn,plot");
	hyper[620].init("delex",0,"Elevator control deflection - deg","actuator","out","scrn,plot");
	hyper[621].init("delrx",0,"Rudder control deflection - deg","actuator","out","scrn,plot");
	hyper[622].init("elvlx",0,"Left elevon deflection - deg","actuator","dia","");
	hyper[623].init("elvrx",0,"Right elevon deflection - deg","actuator","dia","");
	hyper[624].init("elvlcx",0,"Commanded left elevon deflection - deg","actuator","dia","");
	hyper[625].init("elvrcx",0,"Commanded right elevon deflection - deg","actuator","dia","");
	hyper[630].init("DXD",0,0,0,"Fin position derivative - deg/s","actuator","state","");
	hyper[631].init("DX",0,0,0,"Fin position - deg","actuator","state","");
	hyper[632].init("DDXD",0,0,0,"Fin rate derivative - deg/s^2","actuator","state","");
	hyper[633].init("DDX",0,0,0,"Fin rate - deg/s","actuator","state","");
}	

///////////////////////////////////////////////////////////////////////////////
//Actuator module
//Member function of class 'Hyper'
// Converts from control deflections to fin deflections
// Calls actuator dynamic subroutine
//    mact=0 No dynamics with position limiting
//		  =1 First order dynamics (not implemented)
//        =2 Second order dynamics with rate limiting
// Limits fins excursions and converts back to control deflections
//
//030515 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::actuator(double int_step)
{
	//local variable
	Matrix ACTCX(3,1),ACTX(3,1);

	//local module-variables
	double delax(0),delex(0),delrx(0);
	double elvlx(0),elvrx(0);
	double elvlcx(0),elvrcx(0);

	//localizing module-variables
	//input data
	int mact=hyper[600].integer();
	double dlimx=hyper[602].real();
	//input from other modules
	double delacx=hyper[519].real();
	double delecx=hyper[520].real();
	double delrcx=hyper[521].real();
//GHAME4-start
//	delecx=-4.24;
//GHAME4-end
	//-------------------------------------------------------------------------
	//Conversion of aileron end elevator command to left and right elevon commands
	elvlcx=delecx+delacx;
	elvrcx=delecx-delacx;
	ACTCX.assign_loc(0,0,elvlcx);
	ACTCX.assign_loc(1,0,elvrcx);
	ACTCX.assign_loc(2,0,delrcx);

	//no actuator dynamics
	int i(0);
	switch(mact){
	case 0:
		ACTX=ACTCX;
		//limiting deflections
		for(i=0;i<3;i++){
			if(fabs(ACTX.get_loc(i,0))>dlimx) ACTX.assign_loc(i,0,dlimx*sign(ACTX.get_loc(i,0)));
		}
		break;

	//second order dynamics
	case 2:
		ACTX=actuator_scnd(ACTCX,int_step);
		break;
	}
	elvlx=ACTX.get_loc(0,0);
	elvrx=ACTX.get_loc(1,0);
	delrx=ACTX.get_loc(2,0);

	delax=(elvlx-elvrx)/2;
	delex=(elvlx+elvrx)/2;
//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[619].gets(delax);
	hyper[620].gets(delex);
	hyper[621].gets(delrx);
	//diagnostics
	hyper[622].gets(elvlx);
	hyper[623].gets(elvrx);
	hyper[624].gets(elvlcx);
	hyper[625].gets(elvrcx);
}	
///////////////////////////////////////////////////////////////////////////////
//Second order actuator
//Member function of class 'Hyper'
// Models second order lags of elevons and rudder
// Limits fin positions
// Limits fin rates
//
//030515 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::actuator_scnd(Matrix ACTCX, double int_step)
{	
	//local variables
	Matrix DXD_NEW(3,1);
	Matrix DDXD_NEW(3,1);
	Matrix ACTX(3,1);

	//localizing module-variables
	//input data
	double dlimx=hyper[602].real();
	double ddlimx=hyper[604].real();
	double wnact=hyper[605].real();
	double zetact=hyper[606].real();
	//state variables
	Matrix DXD=hyper[630].vec();
	Matrix DX=hyper[631].vec();
	Matrix DDXD=hyper[632].vec();
	Matrix DDX=hyper[633].vec();
	//-------------------------------------------------------------------------
	//sencond order integrations for each surface
	for(int i=0;i<3;i++){
	//limiting position and the fin rate derivative
		if(fabs(DX[i])>dlimx){
			DX[i]=dlimx*sign(DX[i]);
			if(DX[i]*DDX[i]>0.) DDX[i]=0;
		}
		//limiting fin rate
		int iflag=0;
		if(fabs(DDX[i])>ddlimx){
			iflag=1;
			DDX[i]=ddlimx*sign(DDX[i]);
		}
		//state integration
		DXD_NEW[i]=DDX[i];
		DX[i]=integrate(DXD_NEW[i],DXD[i],DX[i],int_step);
		DXD[i]=DXD_NEW[i];
		double edx=ACTCX[i]-DX[i];
		DDXD_NEW[i]=wnact*wnact*edx-2.*zetact*wnact*DXD[i];
		DDX[i]=integrate(DDXD_NEW[i],DDXD[i],DDX[i],int_step);
		DDXD[i]=DDXD_NEW[i];
		//setting fin rate derivative to zero if rate is limited
		if(iflag&&DDX[i]*DDXD[i]>0.) DDXD[i]=0;
	}
	ACTX=DX;	
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[630].gets_vec(DXD);
	hyper[631].gets_vec(DX);
	hyper[632].gets_vec(DDXD);
	hyper[633].gets_vec(DDX);

	return ACTX;
}