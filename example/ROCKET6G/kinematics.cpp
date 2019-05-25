///////////////////////////////////////////////////////////////////////////////
//FILE: 'kinematics.cpp'
//Contains 'kinematics' module of class 'Round6'
//
//011126 Created from SRAAM6 by Peter H Zipfel
//030307 Upgraded to SM Item32, PZi
//050326 Moved 'time' calculations to kinematics module, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of kinematics module-variables 
//Member function of class 'Round6'
//Module-variable locations are assigned to round6[100-149]
// 
//011126 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round6::def_kinematics()
{
	//Definition and initialization of module-variables
	round6[0].init("time",0,"Vehicle time since launch - s","kinematics","exec","scrn,plot,com");
	round6[1].init("event_time",0,"Time elapsed during an event - s","kinematics","exec","");
	round6[2].init("int_step_new",0,"New integration step size  - s","kinematics","data","");
	round6[3].init("out_step_fact",0,"Factor to modify output, e.g.: plot_step*(1+out_step_fact)  - ND","kinematics","data","");
	round6[5].init("stop","int",0,"=1: Stopping vehicle if 'trcond' is met - ND","kinematics","exec","");
	round6[6].init("lconv","int",0, "Flag of type of trajectory termination  - ND","kinematics","exec","");
    round6[120].init("TBD",0,0,0,0,0,0,0,0,0,"T.M. of body wrt geodetic coord","kinematics","out","");
    round6[121].init("TBI",0,0,0,0,0,0,0,0,0,"T.M. of body wrt inertial coord","kinematics","state","");
    round6[122].init("TBID",0,0,0,0,0,0,0,0,0,"T.M. of body wrt inertial coord derivative - 1/s","kinematics","state","");
    round6[123].init("ortho_error",0,"Direction cosine matrix orthogonality error - ND","kinematics","diag","scrn");
    round6[134].init("psibd",0,"Yawing angle of veh wrt geod coord - rad","kinematics","diag","");
    round6[135].init("thtbd",0,"Pitching angle of veh wrt geod coord - rad","kinematics","diag","");
    round6[136].init("phibd",0,"Rolling angle of veh wrt geod coord - rad","kinematics","diag","");
    round6[137].init("psibdx",0,"Yawing angle of veh wrt geod coord - deg","kinematics","in/di","scrn,plot");
    round6[138].init("thtbdx",0,"Pitching angle of veh wrt geod coord - deg","kinematics","in/di","scrn,plot");
    round6[139].init("phibdx",0,"Rolling angle of veh wrt geod coord - deg","kinematics","in/di","scrn,plot");
    round6[140].init("alppx",0,"Total angle of attack - deg","kinematics","out","scrn,plot");
    round6[141].init("phipx",0,"Aerodynamic roll angle - deg","kinematics","out","scrn,plot");
    round6[144].init("alphax",0,"Angle of attack - deg","kinematics","init/diag","scrn,plot");
    round6[145].init("betax",0,"Side slip angle - deg","kinematics","diag","scrn,plot");
    round6[146].init("alphaix",0,"Angle of attack, inertial velocity - deg","kinematics","diag","");
    round6[147].init("betaix",0,"Side slip angle, inertial velocity - deg","kinematics","diag","");
}	

///////////////////////////////////////////////////////////////////////////////
//Initialization of kinematics module
//Member function of class 'Round6'
//Setting 'sim_time' = 'time'
//Accepting new integration step size 'int_step' from 'input.asc'
//Initializes direction cosine matrix
// 
//011126 Created by Peter H Zipfel
//050326 Moved 'time' calculations to kinematics module, PZi
///////////////////////////////////////////////////////////////////////////////

void Round6::init_kinematics(double sim_time,double int_step)
{	
	//local module-variables
	double time(0);
	double int_step_new(0);
	Matrix TBD(3,3);
	Matrix TBI(3,3);

	//localizing module-variables
	//input data
	double psibdx=round6[137].real();
	double thtbdx=round6[138].real();
	double phibdx=round6[139].real();
	//input from other modules
	double lonx=round6[219].real();  
	double latx=round6[220].real();  
	double alt=round6[221].real();   
	double dvbe=round6[225].real();   
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//initializing the variable integration step
	int_step_new=int_step;
	//-------------------------------------------------------------------------
	//initializing direction cosine matrix TBD, body wrt geodetic coord
	TBD=mat3tr(psibdx*RAD,thtbdx*RAD,phibdx*RAD);
	Matrix TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,time);
	TBI=TBD*TDI;
	//-------------------------------------------------------------------------
	//loading module-variables
	//executive 
	round6[0].gets(time);
	round6[2].gets(int_step_new);
	//initialized output
	round6[120].gets_mat(TBD);
	round6[121].gets_mat(TBI);
}
///////////////////////////////////////////////////////////////////////////////
//Kinematics module
//Member function of class 'Round6'
//Setting 'sim_time' = 'time'
//Saving 'event_time' of 'this' vehicle object
//Accepting new integration step size 'int_step' from 'input.asc'
//Changing all output step sizes by a factor  
// Solves the direction cosine differential equations
// Calulates Euler angles
// Calculates incidence angles
//
//011127 Created by Peter Zipfel
//050326 Moved 'time' calculations from newton to kinematics module, PZi
///////////////////////////////////////////////////////////////////////////////

void Round6::kinematics(double sim_time,double event_time,double &int_step,double &out_fact)
{
	//local variables
	Matrix UNIT(3,3); UNIT.identity();
	double cthtbd(0);
	double phip(0);
	
	//local module-variables
	double time(0);
	double ortho_error(0);
	double thtbd(0);
	double psibd(0);
	double phibd(0);
	double psibdx(0);
	double thtbdx(0);
	double phibdx(0);
	double alphax(0);
	double betax(0);
	Matrix TBD(3,3);
	double alppx(0);
	double phipx(0);

	//localizing module-variables
	//input data
	double int_step_new=round6[2].real();
	double out_step_fact=round6[3].real();
	double ck=round6[101].real();
	//input from other modules
	double dvba=round6[75].real();
	Matrix WBIB=round6[164].vec();
	double lonx=round6[219].real();  
	double latx=round6[220].real();  
	double alt=round6[221].real();
	Matrix VBED=round6[232].vec();
	Matrix VAED=round6[72].vec();
	int trcond=hyper[180].integer();
	Matrix VBII=round6[236].vec();
	//state variables
	Matrix TBI=round6[121].mat();
	Matrix TBID=round6[122].mat();
	//----------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//accepting new integration step size at 'events'
	int_step=int_step_new;
	//changing the step size of all outputs by 'out_step_fact' at 'events'
	out_fact=out_step_fact;
	//----------------------------------------------------------------------------
	//*integrating direction cosine matrix
	Matrix TBID_NEW=~WBIB.skew_sym()*TBI;
	TBI=integrate(TBID_NEW,TBID,TBI,int_step);
	TBID=TBID_NEW;

	//orthonormalizing TBI
	Matrix EE=UNIT-TBI*~TBI;
	TBI=TBI+EE*TBI*0.5;

	//TBI orthogonality check
	double e1=EE.get_loc(0,0);
	double e2=EE.get_loc(1,1);
	double e3=EE.get_loc(2,2);
	ortho_error=sqrt(e1*e1+e2*e2+e3*e3);

	//Euler angles
	Matrix TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,time);
	TBD=TBI*~TDI;
	double tbd13=TBD.get_loc(0,2);
	double tbd11=TBD.get_loc(0,0);
	double tbd33=TBD.get_loc(2,2);
	double tbd12=TBD.get_loc(0,1);
	double tbd23=TBD.get_loc(1,2);

	//*geodetic Euler angles
	//pitch angle: 'thtbd'
	//note: when |tbd13| >= 1, thtbd = +- pi/2, but cos(thtbd) is
	//		forced to be a small positive number to prevent division by zero
	if(fabs(tbd13)<1){
		thtbd=asin(-tbd13);
		cthtbd=cos(thtbd);
	}
	else{
		thtbd=PI/2*sign(-tbd13);
		cthtbd=EPS;
	}
	//yaw angle: 'psibd'
	double cpsi=tbd11/cthtbd;
	if(fabs(cpsi)>1)
		cpsi=1*sign(cpsi);
	psibd=acos(cpsi)*sign(tbd12);

	//roll angle: 'phibdc'
	double cphi=tbd33/cthtbd;
	if(fabs(cphi)>1)
		cphi=1*sign(cphi);
	phibd=acos(cphi)*sign(tbd23);

	psibdx=DEG*psibd;
	thtbdx=DEG*thtbd;
	phibdx=DEG*phibd;

	//*incidence angles using wind vector VAED in geodetic coord
	Matrix VBAB=TBD*(VBED-VAED);
	double vbab1=VBAB.get_loc(0,0);
	double vbab2=VBAB.get_loc(1,0);
	double vbab3=VBAB.get_loc(2,0);
	double alpha=atan2(vbab3,vbab1);
	double beta=asin(vbab2/dvba);
	alphax=alpha*DEG;
	betax=beta*DEG;
	
	//incidence angles in load factor plane (aeroballistic)
	double dum=vbab1/dvba;
	if(fabs(dum)>1)
		dum=1*sign(dum);
	double alpp=acos(dum);

	if(vbab2==0&&vbab3==0)
		phip=0.;
	//note: if vbeb2 is <EPS the value phip is forced to be 0 or PI
	//		to prevent oscillations
	else if(fabs(vbab2)<EPS)
		if(vbab3>0) phip=0;
		if(vbab3<0) phip=PI;
	else
		phip=atan2(vbab2,vbab3);
	alppx=alpp*DEG;
	phipx=phip*DEG;

	//*diagnostic: calculating the inertial incidence angles
	Matrix VBIB=TBI*VBII;
	double vbib1=VBIB.get_loc(0,0);
	double vbib2=VBIB.get_loc(1,0);
	double vbib3=VBIB.get_loc(2,0);
	double alphai=atan2(vbib3,vbib1);
	double dvbi=VBIB.absolute();
	double betai=asin(vbib2/dvbi);
	double alphaix=alphai*DEG;
	double betaix=betai*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	round6[121].gets_mat(TBI);
	round6[122].gets_mat(TBID);
	//output to other modules
	round6[0].gets(time);
	round6[1].gets(event_time);
	round6[2].gets(int_step_new);
	round6[120].gets_mat(TBD);
	round6[137].gets(psibdx);
	round6[138].gets(thtbdx);
	round6[139].gets(phibdx);
	round6[140].gets(alppx);
	round6[141].gets(phipx);
	round6[144].gets(alphax);
	round6[145].gets(betax);
	//diagnostics
	round6[123].gets(ortho_error);
	round6[134].gets(psibd);
	round6[135].gets(thtbd);
	round6[136].gets(phibd);
	round6[146].gets(alphaix);
	round6[147].gets(betaix);
}