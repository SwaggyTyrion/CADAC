///////////////////////////////////////////////////////////////////////////////
//FILE: 'actuator.cpp'
//Contains 'actuator' module of class 'Missile'
//
//030607 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of actuator module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[600-649]
// 
//170823 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_actuator()
{
	//Definition and initialization of module-variables
	missile[600].init("mact","int",0,"=0:no dynamics, =2:second order","actuator","data","");
	missile[602].init("dlimx",0,"Control fin limiter - deg","actuator","data","");
	missile[604].init("ddlimx",0,"Control fin rate limiter - deg/s","actuator","data","");
	missile[605].init("wnact",0,"Natural frequency of actuator - rad/s","actuator","data","");
	missile[606].init("zetact",0,"Damping of actuator - ND","actuator","data","");
	missile[619].init("dpx",0,"Roll control deflection - deg","actuator","out","plot");
	missile[620].init("dqx",0,"Pitch control deflection - deg","actuator","out","plot");
	missile[621].init("drx",0,"Yaw control deflection - deg","actuator","out","plot");
	missile[622].init("delx1",0,"Fin #1 output position - deg","actuator","diag","scrn,plot");
	missile[623].init("delx2",0,"Fin #2 output position - deg","actuator","diag","scrn,plot");
	missile[624].init("delx3",0,"Fin #3 output position - deg","actuator","diag","scrn,plot");
	missile[625].init("delx4",0,"Fin #4 output position - deg","actuator","diag","scrn,plot");
	missile[630].init("dxd1",0,"Fin #1 position derivative - deg/s","actuator","state","");
	missile[631].init("dxd2",0,"Fin #2 position derivative - deg/s","actuator","state","");
	missile[632].init("dxd3",0,"Fin #3 position derivative - deg/s","actuator","state","");
	missile[633].init("dxd4",0,"Fin #4 position derivative - deg/s","actuator","state","");
	missile[634].init("dx1",0,"Fin #1 position - deg","actuator","state","");
	missile[635].init("dx2",0,"Fin #2 position - deg","actuator","state","");
	missile[636].init("dx3",0,"Fin #3 position - deg","actuator","state","");
	missile[637].init("dx4",0,"Fin #4 position - deg","actuator","state","");
	missile[638].init("ddxd1",0,"Fin #1 rate derivative - deg/s^2","actuator","state","");
	missile[639].init("ddxd2",0,"Fin #2 rate derivative - deg/s^2","actuator","state","");
	missile[640].init("ddxd3",0,"Fin #3 rate derivative - deg/s^2","actuator","state","");
	missile[641].init("ddxd4",0,"Fin #4 rate derivative - deg/s^2","actuator","state","");
	missile[642].init("ddx1",0,"Fin #1 rate - deg/s","actuator","state","");
	missile[643].init("ddx2",0,"Fin #2 rate - deg/s","actuator","state","");
	missile[644].init("ddx3",0,"Fin #3 rate - deg/s","actuator","state","");
	missile[645].init("ddx4",0,"Fin #4 rate - deg/s","actuator","state","");
	missile[646].init("delcx1",0,"Fin #1 position command - deg","actuator","diag","");
	missile[647].init("delcx2",0,"Fin #2 position command - deg","actuator","diag","");
	missile[648].init("delcx3",0,"Fin #3 position command - deg","actuator","diag","");
	missile[649].init("delcx4",0,"Fin #4 position command - deg","actuator","diag","");
}	
///////////////////////////////////////////////////////////////////////////////
//Actuator module
//Member function of class 'Missile'
// (1) Converts from control deflections to fin deflections
// (2) Calls actuator dynamic subroutine
//
//    mact=0 No dynamics with position limiting
//        =2 Second order dynamics with rate and position limiting
//
// (3) Limits fins excursions and converts back to control deflections for aerodynamics
// (4) Fins are in the cross configuration; positive deflections follow the Missile DATCOM convention
//
//170823 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::actuator(double int_step)
{
	//local module-variables
	double delx1(0),delx2(0),delx3(0),delx4(0);
	double delcx1(0),delcx2(0),delcx3(0),delcx4(0);
	double dpx(0),dqx(0),drx(0);

	//localizing module-variables
	//input data
	int mact=missile[600].integer();
	double dlimx=missile[602].real();
	//input from other modules
	double time=flat6[0].real();
	double dpcx=missile[519].real();
	double dqcx=missile[520].real();
	double drcx=missile[521].real();

    //-------------------------------------------------------------------------
	//converting to four fin deflections (fins are in the cross configuration)
	delcx1=-dpcx-drcx;
	delcx2=-dpcx+dqcx;
	delcx3=-dpcx+drcx;
	delcx4=-dpcx-dqcx;

	//no actuator dynamics
	if(mact<2)
	{
		delx1=delcx1;
		if(fabs(delx1)>dlimx)delx1=dlimx*sign(delx1);
		delx2=delcx2;
		if(fabs(delx2)>dlimx)delx2=dlimx*sign(delx2);
		delx3=delcx3;
		if(fabs(delx3)>dlimx)delx3=dlimx*sign(delx3);
		delx4=delcx4;
		if(fabs(delx4)>dlimx)delx4=dlimx*sign(delx4);

		//converting back to control deflections for aerodynamics
		dpx=0.25*(-delx1-delx2-delx3-delx4);
		dqx=0.5*(delx2-delx4);
		drx=0.5*(-delx1+delx3);

	//-------------------------------------------------------------------------
		//loading module-variables
		//output to other modules
		missile[619].gets(dpx);
		missile[620].gets(dqx);
		missile[621].gets(drx);
		missile[622].gets(delx1);
		missile[623].gets(delx2);
		missile[624].gets(delx3);
		missile[625].gets(delx4);
		//diagnostics
		missile[646].gets(delcx1);
		missile[647].gets(delcx2);
		missile[648].gets(delcx3);
		missile[649].gets(delcx4);
	//-------------------------------------------------------------------------
	}
	//calling second order actuator dynamics
	if(mact==2) actuator_scnd(int_step);

}	
///////////////////////////////////////////////////////////////////////////////
//Second order actuator
//Member function of class 'Missile'
// Models second order lags of all four control fins
// Limits fin positions
// Limits fin rates
//
//030607 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::actuator_scnd(double int_step)
{	
	//local module-variables
	double delx1(0),delx2(0),delx3(0),delx4(0);
	double delcx1(0),delcx2(0),delcx3(0),delcx4(0);
	double dpx(0),dqx(0),drx(0);

	//localizing module-variables
	//input data
	double dlimx=missile[602].real();
	double ddlimx=missile[604].real();
	double wnact=missile[605].real();
	double zetact=missile[606].real();
	//input from other modules
	double time=flat6[0].real();
	double dpcx=missile[519].real();
	double dqcx=missile[520].real();
	double drcx=missile[521].real();
	//state variables
	double dxd1=missile[630].real();
	double dxd2=missile[631].real();
	double dxd3=missile[632].real();
	double dxd4=missile[633].real();
	double dx1=missile[634].real();
	double dx2=missile[635].real();
	double dx3=missile[636].real();
	double dx4=missile[637].real();
	double ddxd1=missile[638].real();
	double ddxd2=missile[639].real();
	double ddxd3=missile[640].real();
	double ddxd4=missile[641].real();
	double ddx1=missile[642].real();
	double ddx2=missile[643].real();
	double ddx3=missile[644].real();
	double ddx4=missile[645].real();
    //-------------------------------------------------------------------------
	//converting to four fin deflections (fins are in the cross configuration)
	delcx1=-dpcx-drcx;
	delcx2=-dpcx+dqcx;
	delcx3=-dpcx+drcx;
	delcx4=-dpcx-dqcx;

	//fin#1
	//limiting position and the fin rate derivative
	if(fabs(dx1)>dlimx){
		dx1=dlimx*sign(dx1);
		if(dx1*ddx1>0)ddx1=0;
	}
	//limiting fin rate
	int iflag=0;
	if(fabs(ddx1)>ddlimx){
		iflag=1;
		ddx1=ddlimx*sign(ddx1);
	}
	//state integration
	double dxd1_new=ddx1;
	dx1=integrate(dxd1_new,dxd1,dx1,int_step);

	dxd1=dxd1_new;
	double edx1=delcx1-dx1;
	double ddxd1_new=wnact*wnact*edx1-2*zetact*wnact*dxd1;
	ddx1=integrate(ddxd1_new,ddxd1,ddx1,int_step);
	ddxd1=ddxd1_new;
	//setting fin rate derivative to zero if rate is limited
	if(iflag&&ddx1*ddxd1>0) ddxd1=0;

	//fin#2
	//limiting position and the fin rate derivative
	if(fabs(dx2)>dlimx){
		dx2=dlimx*sign(dx2);
		if(dx2*ddx2>0)ddx2=0;
	}
	//limiting fin rate
	iflag=0;
	if(fabs(ddx2)>ddlimx){
		iflag=1;
		ddx2=ddlimx*sign(ddx2);
	}
	//state integration
	double dxd2_new=ddx2;
	dx2=integrate(dxd2_new,dxd2,dx2,int_step);
	dxd2=dxd2_new;
	double edx2=delcx2-dx2;
	double ddxd2_new=wnact*wnact*edx2-2*zetact*wnact*dxd2;
	ddx2=integrate(ddxd2_new,ddxd2,ddx2,int_step);
	ddxd2=ddxd2_new;
	//setting fin rate derivative to zero if rate is limited
	if(iflag&&ddx2*ddxd2>0) ddxd2=0;

	//fin#3
	//limiting position and the fin rate derivative
	if(fabs(dx3)>dlimx){
		dx3=dlimx*sign(dx3);
		if(dx3*ddx3>0)ddx3=0;
	}
	//limiting fin rate
	iflag=0;
	if(fabs(ddx3)>ddlimx){
		iflag=1;
		ddx3=ddlimx*sign(ddx3);
	}
	//state integration
	double dxd3_new=ddx3;
	dx3=integrate(dxd3_new,dxd3,dx3,int_step);
	dxd3=dxd3_new;
	double edx3=delcx3-dx3;
	double ddxd3_new=wnact*wnact*edx3-2*zetact*wnact*dxd3;
	ddx3=integrate(ddxd3_new,ddxd3,ddx3,int_step);
	ddxd3=ddxd3_new;
	//setting fin rate derivative to zero if rate is limited
	if(iflag&&ddx3*ddxd3>0) ddxd3=0;

	//fin#4
	//limiting position and the fin rate derivative
	if(fabs(dx4)>dlimx){
		dx4=dlimx*sign(dx4);
		if(dx4*ddx4>0)ddx4=0;
	}
	//limiting fin rate
	iflag=0;
	if(fabs(ddx4)>ddlimx){
		iflag=1;
		ddx4=ddlimx*sign(ddx4);
	}
	//state integration
	double dxd4_new=ddx4;
	dx4=integrate(dxd4_new,dxd4,dx4,int_step);
	dxd4=dxd4_new;
	double edx4=delcx4-dx4;
	double ddxd4_new=wnact*wnact*edx4-2*zetact*wnact*dxd4;
	ddx4=integrate(ddxd4_new,ddxd4,ddx4,int_step);
	ddxd4=ddxd4_new;
	//setting fin rate derivative to zero if rate is limited
	if(iflag&&ddx4*ddxd4>0) ddxd4=0;

	//for output
	delx1=dx1;
	delx2=dx2;
	delx3=dx3;
	delx4=dx4;

	//converting back to control deflections for aerodynamics
	dpx=0.25*(-delx1-delx2-delx3-delx4);
	dqx=0.5*(delx2-delx4);
	drx=0.5*(-delx1+delx3);

	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[630].gets(dxd1);
	missile[631].gets(dxd2);
	missile[632].gets(dxd3);
	missile[633].gets(dxd4);
	missile[634].gets(dx1);
	missile[635].gets(dx2);
	missile[636].gets(dx3);
	missile[637].gets(dx4);
	missile[638].gets(ddxd1);
	missile[639].gets(ddxd2);
	missile[640].gets(ddxd3);
	missile[641].gets(ddxd4);
	missile[642].gets(ddx1);
	missile[643].gets(ddx2);
	missile[644].gets(ddx3);
	missile[645].gets(ddx4);
	//output to other modules
	missile[619].gets(dpx);
	missile[620].gets(dqx);
	missile[621].gets(drx);
	//diagnostics
	missile[622].gets(delx1);
	missile[623].gets(delx2);
	missile[624].gets(delx3);
	missile[625].gets(delx4);
	missile[646].gets(delcx1);
	missile[647].gets(delcx2);
	missile[648].gets(delcx3);
	missile[649].gets(delcx4);

}