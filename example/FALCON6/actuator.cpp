///////////////////////////////////////////////////////////////////////////////
//FILE: 'actuator.cpp'
//Contains 'actuator' module of class 'Plane'
//
//030515 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of actuator module-variables 
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[600-649]
//
// mact=0 No dynamics with position limiting
//	   =1 First order dynamics (not implemented)
//     =2 Second order dynamics with position and rate limiting
//
//030725 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::def_actuator()
{
	//Definition and initialization of module-variables
	plane[600].init("mact","int",0,"=0:no dynamics, =2:second order","actuator","data","");
	plane[602].init("dlimx",0,"Control fin limiter - deg","actuator","data","");
	plane[604].init("ddlimx",0,"Control fin rate limiter - deg/s","actuator","data","");
	plane[605].init("wnact",0,"Natural frequency of actuator - rad/s","actuator","data","");
	plane[606].init("zetact",0,"Damping of actuator - ND","actuator","data","");
	plane[619].init("delax",0,"Aileron control deflection - deg","actuator","out","scrn,plot");
	plane[620].init("delex",0,"Elevator control deflection - deg","actuator","out","scrn,plot");
	plane[621].init("delrx",0,"Rudder control deflection - deg","actuator","out","scrn,plot");
	plane[630].init("DXD",0,0,0,"Fin position derivative - deg/s","actuator","state","");
	plane[631].init("DX",0,0,0,"Fin position - deg","actuator","state","");
	plane[632].init("DDXD",0,0,0,"Fin rate derivative - deg/s^2","actuator","state","");
	plane[633].init("DDX",0,0,0,"Fin rate - deg/s","actuator","state","");
}	

///////////////////////////////////////////////////////////////////////////////
//Actuator module
//Member function of class 'Plane'
// Converts from control deflections to fin deflections
// Calls actuator dynamic subroutine
//    mact=0 No dynamics with position limiting
//        =2 Second order dynamics with rate limiting
// Limits fins excursions and converts back to control deflections
//
//030725 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::actuator(double int_step)
{
	//local variable
	Matrix ACTCX(3,1),ACTX(3,1);

	//local module-variables
	double delax(0),delex(0),delrx(0);
	double elvlx(0),elvrx(0);
	double elvlcx(0),elvrcx(0);

	//localizing module-variables
	//input data
	int mact=plane[600].integer();
	double dlimx=plane[602].real();
	//input from other modules
	double time=flat6[0].real();
	double delacx=plane[519].real();
	double delecx=plane[520].real();
	double delrcx=plane[521].real();
	//-------------------------------------------------------------------------
	//packing the actuator command vector
	ACTCX[0]=delacx;
	ACTCX[1]=delecx;
	ACTCX[2]=delrcx;

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
	//unpacking the actuator vector
	delax=ACTX[0];
	delex=ACTX[1];
	delrx=ACTX[2];
//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	plane[619].gets(delax);
	plane[620].gets(delex);
	plane[621].gets(delrx);
	//diagnostics
}	
///////////////////////////////////////////////////////////////////////////////
//Second order actuator
//Member function of class 'Plane'
// Models second order lags of motivators
// All variables are in deg (not rad)
// Limits fin positions
// Limits fin rates
//
//030725 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Plane::actuator_scnd(Matrix ACTCX, double int_step)
{	
	//local variables
	Matrix DXD_NEW(3,1);
	Matrix DDXD_NEW(3,1);
	Matrix ACTX(3,1);

	//localizing module-variables
	//input data
	double dlimx=plane[602].real();
	double ddlimx=plane[604].real();
	double wnact=plane[605].real();
	double zetact=plane[606].real();
	//state variables
	Matrix DXD=plane[630].vec();
	Matrix DX=plane[631].vec();
	Matrix DDXD=plane[632].vec();
	Matrix DDX=plane[633].vec();
	//-------------------------------------------------------------------------
	for(int i=0;i<3;i++){
	//limiting position and the fin rate derivative
		if(fabs(DX[i])>dlimx){
			DX[i]=dlimx*sign(DX[i]);
			if(DX[i]*DDX[i]>0) DDX[i]=0;
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
		if(iflag&&DDX[i]*DDXD[i]>0)
			DDXD[i]=0;
	}
	ACTX=DX;	
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	plane[630].gets_vec(DXD);
	plane[631].gets_vec(DX);
	plane[632].gets_vec(DDXD);
	plane[633].gets_vec(DDX);

	return ACTX;
}