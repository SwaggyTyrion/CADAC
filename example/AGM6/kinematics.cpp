///////////////////////////////////////////////////////////////////////////////
//FILE: 'kinematics.cpp'
//Contains 'kinematics' module of class 'Flat6'
//
//011128 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030319 Upgraded to SM Item32, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'kinematics' module-variables 
//Member function of class 'Flat6'
//Module-variable locations for executive functions are assigned to flat6[0-9]
//Module-variable locations for 'kinematics' module are assigned to flat6[100-149]
// 
//011126 Created by Peter H Zipfel
//051018 Moved 'time' calculations from 'newton' to 'kinematics' module, PZi
//060509 Modified for SWEEP++, PZi
///////////////////////////////////////////////////////////////////////////////
void Flat6::def_kinematics()
{
	//Definition and initialization of module-variables
	flat6[0].init("time",0,"time since start of simulation - s","kinematics","exec","scrn,plot,com");
	flat6[1].init("event_time",0,"Time elapsed during an event - s","kinematics","exec","");
	flat6[2].init("int_step_new",0,"New integration step size  - s","kinematics","exec","");
	flat6[3].init("out_step_fact",0,"Factor to modify output, e.g.: plot_step*(1+out_step_fact)  - ND","kinematics","exec","");
	flat6[4].init("halt","int",0,"=1: Halting vehicle object via 'input.asc' - ND","kinematics","exec","");
	flat6[5].init("stop","int",0,"=1: Stopping vehicle object via 'intercept' module - ND","kinematics","exec","");
	flat6[6].init("lconv","int",0, "Flag of type of trajectory termination  - ND","kinematics","exec","plot");
	flat6[7].init("launch_delay",0,"Delay of vehicle launch since sim start - s","kinematics","exec","");
	flat6[8].init("launch_epoch",0,"'sim_time' at launch - s","kinematics","exec","");
	flat6[9].init("launch_time",0,"Elapsed time since launch - s","kinematics","exec","");

	flat6[101].init("ck",50.,"Quaternion orthogonalizing factor ","kinematics","data","");
	flat6[110].init("q0d",0,"Quaternion derivative, 0-th component","kinematics","state","");
	flat6[111].init("q0",0,"Quaternion, 0-th component","kinematics","state","");
	flat6[112].init("q1d",0,"Quaternion derivative, 1-th component","kinematics","state","");
	flat6[113].init("q1",0,"Quaternion, 1-th component","kinematics","state","");
	flat6[114].init("q2d",0,"Quaternion derivative, 2-th component","kinematics","state","");
	flat6[115].init("q2",0,"Quaternion, 2-th component","kinematics","state","");
	flat6[116].init("q3d",0,"Quaternion derivative, 3-th component","kinematics","state","");
	flat6[117].init("q3",0,"Quaternion, 3-th component","kinematics","state","");
	flat6[120].init("TBL",0,0,0,0,0,0,0,0,0,"T.M. of body wrt local level coord","kinematics","out","");
	flat6[134].init("psibl",0,"Yawing angle of vehicle - rad","kinematics","diag","");
	flat6[135].init("thtbl",0,"Pitching angle of vehicle - rad","kinematics","diag","");
	flat6[136].init("phibl",0,"Rolling angle of vehicle - rad","kinematics","diag","");
	flat6[137].init("psiblx",0,"Yawing angle of vehicle - deg","kinematics","in/di","scrn,plot,com");
	flat6[138].init("thtblx",0,"Pitching angle of vehicle - deg","kinematics","in/di","scrn,plot,com");
	flat6[139].init("phiblx",0,"Rolling angle of vehicle - deg","kinematics","in/di","scrn,plot,com");
	flat6[140].init("alppx",0,"Total angle of attack - deg","kinematics","out","plot");
	flat6[141].init("phipx",0,"Aerodynamic roll angle - deg","kinematics","out","plot");
	flat6[142].init("alpp",0,"Total angle of attack - rad","kinematics","out","");
	flat6[143].init("phip",0,"Aerodynamic roll angle - rad","kinematics","out","");
	flat6[144].init("alphax",0,"Angle of attack - deg","kinematics","diag","scrn,plot");
	flat6[145].init("betax",0,"Side slip angle - deg","kinematics","diag","scrn,plot");
	flat6[146].init("ortho_error",0,"Error of quaternion nonorthogonality","kinematics","diag","scrn");
	flat6[147].init("etbl",0,"Error of D.C.M. nonorthogonality","kinematics","diag","");
	flat6[148].init("TLB",0,0,0,0,0,0,0,0,0,"T.M. of local level wrt body coord","kinematics","diag","");
}	
///////////////////////////////////////////////////////////////////////////////
//Initialization of 'kinematics' module
//Member function of class 'Flat6'
//Setting 'sim_time' = 'time'
//Accepting new integration step size 'int_step' from 'input.asc'
//Initializes the quaternions from Euler angles
// 
//011126 Created by Peter H Zipfel
//051018 Moved 'time' calculations from 'newton' to 'kinematics' module, PZi
//070530 Inserted launch delay, PZi
///////////////////////////////////////////////////////////////////////////////
void Flat6::init_kinematics(double sim_time,double int_step,double &launch_dly)
{	
	//local module-variables
	double time(0);
	double int_step_new(0);
	double launch_epoch(0);

	//localizing module-variables
	//input data
	double launch_delay=flat6[7].real();
	double psiblx=flat6[137].real();
	double thtblx=flat6[138].real();
	double phiblx=flat6[139].real();
	
	//quaternion initialization
	double spsi=sin(psiblx/(2.*DEG));
	double cpsi=cos(psiblx/(2.*DEG));
	double stht=sin(thtblx/(2.*DEG));
	double ctht=cos(thtblx/(2.*DEG));
	double sphi=sin(phiblx/(2.*DEG));
	double cphi=cos(phiblx/(2.*DEG));

   	double q0=cpsi*ctht*cphi+spsi*stht*sphi;
	double q1=cpsi*ctht*sphi-spsi*stht*cphi;
	double q2=cpsi*stht*cphi+spsi*ctht*sphi;
	double q3=-cpsi*stht*sphi+spsi*ctht*cphi;
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//launch epoch
	launch_epoch=launch_delay;

	//loading launch delay value into argument list
	launch_dly=launch_delay;

	//initializing the variable integration step
	int_step_new=int_step;

	//initializing direction cosine matrix TBL
	Matrix TBL=mat3tr(psiblx/DEG,thtblx/DEG,phiblx/DEG);

	//-------------------------------------------------------------------------
	//loading module-variables
	//executive 
	flat6[0].gets(time);
	flat6[2].gets(int_step_new);
	flat6[8].gets(launch_epoch);
	//initialized output
	flat6[111].gets(q0);
	flat6[113].gets(q1);
	flat6[115].gets(q2);
	flat6[117].gets(q3);
	flat6[120].gets_mat(TBL);
}
///////////////////////////////////////////////////////////////////////////////
//'kinematics' module
//Member function of class 'Flat6'
// Solves the quaternion differential equations
// Calculates the direction cosine matrix of body wrt to earth
//  coordinate system
// Calculates incidence angles
//
//011127 Created by Peter H Zipfel
//100416 Modified for AGM6, PZi
///////////////////////////////////////////////////////////////////////////////
void Flat6::kinematics(double sim_time,double event_time,double &int_step,double &out_fact)
{
	//local variables
	double phip(0);
	double cthtbl(0);
	
	//local module-variables
	double time(0);
	double etbl(0);
	double thtbl(0);
	double psibl(0);
	double phibl(0);
	double psiblx(0);
	double thtblx(0);
	double phiblx(0);
	double alpha(0);
	double beta(0);
	double alpp(0);
	double alphax(0);
	double betax(0);
	double alppx(0);
	double phipx(0);
	double launch_time(0);

	//localizing module-variables
	//input data
	double int_step_new=flat6[2].real();
	double out_step_fact=flat6[3].real();
	double ck=flat6[101].real();
	//initialization
	double launch_epoch=flat6[8].real();
	Matrix TBL=flat6[120].mat();
	//input from other modules
	Matrix VAEL=flat6[72].vec();
	double dvba=flat6[75].real();
	double pp=flat6[155].real();
	double qq=flat6[157].real();
	double rr=flat6[159].real();
	Matrix VBEB=flat6[213].vec();
	//state variables
	double q0d=flat6[110].real();
	double q0=flat6[111].real();
	double q1d=flat6[112].real();
	double q1=flat6[113].real();
	double q2d=flat6[114].real();
	double q2=flat6[115].real();
	double q3d=flat6[116].real();
	double q3=flat6[117].real();

	int trcond=missile[180].integer();
	double trortho=missile[182].real();
	double tralp=missile[186].real();
	//----------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;

	//running launch time clock
	launch_time=sim_time-launch_epoch;

	//accepting new integration step size at 'events'
	int_step=int_step_new;

	//changing the step size of all outputs by 'out_step_fact' at 'events'
	out_fact=out_step_fact;
	//-------------------------------------------------------------------------
	//integrating quaternions
    double ortho_error=1-(q0*q0+q1*q1+q2*q2+q3*q3);

	//termination condition
	if(fabs(ortho_error)>trortho)
		trcond=1;

    double new_q0d=0.5*(-pp*q1-qq*q2-rr*q3)+ck*ortho_error*q0;
    double new_q1d=0.5*(pp*q0+rr*q2-qq*q3)+ck*ortho_error*q1;
    double new_q2d=0.5*(qq*q0-rr*q1+pp*q3)+ck*ortho_error*q2;
    double new_q3d=0.5*(rr*q0+qq*q1-pp*q2)+ck*ortho_error*q3;

	q0=integrate(new_q0d,q0d,q0,int_step);
	q1=integrate(new_q1d,q1d,q1,int_step);
	q2=integrate(new_q2d,q2d,q2,int_step);
	q3=integrate(new_q3d,q3d,q3,int_step);

	q0d=new_q0d;
	q1d=new_q1d;
	q2d=new_q2d;
	q3d=new_q3d;

	//transformation matrix of body wrt local level coordinates
	TBL.assign_loc(0,0,q0*q0+q1*q1-q2*q2-q3*q3);
	TBL.assign_loc(0,1,2.*(q1*q2+q0*q3));
	TBL.assign_loc(0,2,2.*(q1*q3-q0*q2));
	TBL.assign_loc(1,0,2.*(q1*q2-q0*q3));
	TBL.assign_loc(1,1,q0*q0-q1*q1+q2*q2-q3*q3);
	TBL.assign_loc(1,2,2.*(q2*q3+q0*q1));
	TBL.assign_loc(2,0,2.*(q1*q3+q0*q2));
	TBL.assign_loc(2,1,2.*(q2*q3-q0*q1));
	TBL.assign_loc(2,2,q0*q0-q1*q1-q2*q2+q3*q3);

	//TBL orthogonality check
	Matrix TLB=TBL.trans();
	Matrix UBL=TLB*TBL;
	double e1=UBL.get_loc(0,0)-1.;
	double e2=UBL.get_loc(1,1)-1.;
	double e3=UBL.get_loc(2,2)-1.;
	etbl=sqrt(e1*e1+e2*e2+e3*e3);

	//Euler angles
	double tbl13=TBL.get_loc(0,2);
	double tbl11=TBL.get_loc(0,0);
	double tbl33=TBL.get_loc(2,2);
	double tbl12=TBL.get_loc(0,1);
	double tbl23=TBL.get_loc(1,2);

	//note: when |tbl13| >= 1., thtbl = +- pi/2, but cos(thtbl) is
	//		forced to be a small positive number to prevent divide by zero
	if(fabs(tbl13)<1){
		thtbl=asin(-tbl13);
		cthtbl=cos(thtbl);
	}
	else{
		thtbl=PI/2*sign(-tbl13);
		cthtbl=EPS;
	}
	//yaw angle: 'psibl'
	double cpsi=tbl11/cthtbl;
	if(fabs(cpsi)>1)
		cpsi=1*sign(cpsi);
	psibl=acos(cpsi)*sign(tbl12);

	//roll angle: 'phibdc'
	double cphi=tbl33/cthtbl;
	if(fabs(cphi)>1)
		cphi=1*sign(cphi);
	phibl=acos(cphi)*sign(tbl23);

	psiblx=DEG*psibl;
	thtblx=DEG*thtbl;
	phiblx=DEG*phibl;

	//incidence angles
	Matrix VBAB=VBEB-TBL*VAEL;
	double vbab1=VBAB.get_loc(0,0);
	double vbab2=VBAB.get_loc(1,0);
	double vbab3=VBAB.get_loc(2,0);

	alpha=atan2(vbab3,vbab1);
	beta=asin(vbab2/dvba);

	double dum=vbab1/dvba;
	if(fabs(dum)>=1.)
		dum=(1.-EPS)*sign(dum);
	alpp=acos(dum);

	if(fabs(vbab2)<EPS&&fabs(vbab3)<EPS)
		phip=0.;
	//note: to prevent oscillating of phip between +- 180 deg, vbeb2
	//		 is set to +SMALL if its absolute value is < SMALL 
	else if(fabs(vbab2)<SMALL)
		phip=atan2(SMALL,vbab3);
	else
		phip=atan2(vbab2,vbab3);

	alphax=alpha*DEG;
	betax=beta*DEG;
	alppx=alpp*DEG;
	phipx=phip*DEG;

	//termination condition
	if(alpp>tralp)
		trcond=5;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	flat6[110].gets(q0d);
	flat6[111].gets(q0);
	flat6[112].gets(q1d);
	flat6[113].gets(q1);
	flat6[114].gets(q2d);
	flat6[115].gets(q2);
	flat6[116].gets(q3d);
	flat6[117].gets(q3);
	//output to other modules
	flat6[0].gets(time);
	flat6[1].gets(event_time);
	flat6[2].gets(int_step_new);
	flat6[9].gets(launch_time);
	flat6[120].gets_mat(TBL);
	flat6[140].gets(alppx);
	flat6[141].gets(phipx);
	flat6[142].gets(alpp);
	flat6[143].gets(phip);
	missile[180].gets(trcond);
	//diagnostics
	flat6[134].gets(psibl);
	flat6[135].gets(thtbl);
	flat6[136].gets(phibl);
	flat6[137].gets(psiblx);
	flat6[138].gets(thtblx);
	flat6[139].gets(phiblx);
	flat6[144].gets(alphax);
	flat6[145].gets(betax);
	flat6[146].gets(ortho_error);
	flat6[147].gets(etbl);
	flat6[148].gets_mat(TLB);
}
