///////////////////////////////////////////////////////////////////////////////
//FILE: 'kinematics.cpp'
//Contains 'kinematics' module of class 'Flat6'
//
//011126 Created from SRAAM6 by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of kinematics module-variables 
//Member function of class 'Flat6'
//Module-variable locations are assigned to flat6[100-149]
// 
//011126 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::def_kinematics()
{
	//Definition and initialization of module-variables
     flat6[101].init("ck",50,"Quaternion orthogonalizing factor ","kinematics","data","");
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
     flat6[137].init("psiblx",0,"G Yawing angle of vehicle - deg","kinematics","in/di","scrn,plot");
     flat6[138].init("thtblx",0,"G Pitching angle of vehicle - deg","kinematics","in/di","scrn,plot");
     flat6[139].init("phiblx",0,"G Rolling angle of vehicle - deg","kinematics","in/di","scrn,plot");
     flat6[140].init("alppx",0,"Total angle of attack - deg","kinematics","out","scrn,plot");
     flat6[141].init("phipx",0,"Aerodynamic roll angle - deg","kinematics","out","scrn,plot");
     flat6[142].init("alpp",0,"Total angle of attack - rad","kinematics","out","");
     flat6[143].init("phip",0,"Aerodynamic roll angle - rad","kinematics","out","");
     flat6[144].init("alphax",0,"Angle of attack - deg","kinematics","diag","plot");
     flat6[145].init("betax",0,"Side slip angle - deg","kinematics","diag","plot");
     flat6[146].init("erq",0,"Error of quaternion nonorthogonality","kinematics","diag","plot");
     flat6[147].init("etbl",0,"Error of D.C.M. nonorthogonality","kinematics","diag","plot");
     flat6[148].init("TLB",0,0,0,0,0,0,0,0,0,"T.M. of local level wrt body coord","kinematics","diag","");
}	

///////////////////////////////////////////////////////////////////////////////
//Initialization of kinematics module
//Member function of class 'Flat6'
//Initializes the quaternions from Euler angles
// 
//011126 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::init_kinematics()
{	
	//localizing module-variables
	//input data
	double psiblx=flat6[137].real();
	double thtblx=flat6[138].real();
	double phiblx=flat6[139].real();
	//-------------------------------------------------------------------------	
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

	//initializing direction cosine matrix TBL
	Matrix TBL=mat3tr(psiblx/DEG,thtblx/DEG,phiblx/DEG);

	//-------------------------------------------------------------------------
	//loading module-variables
	//initialized output
	flat6[111].gets(q0);
	flat6[113].gets(q1);
	flat6[115].gets(q2);
	flat6[117].gets(q3);
	flat6[120].gets_mat(TBL);
}
///////////////////////////////////////////////////////////////////////////////
//Kinematics module
//Member function of class 'Flat6'
// Solves the quaternion differential equations
// Calculates the direction cosine matrix of body wrt to local level
//  coordinate system
// Calculates incidence angles
//
//011127 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Flat6::kinematics(double int_step)
{
	//local variables
	double phip(0);
	double cthtbl(0);
	
	//local module-variables
	Matrix TBL(3,3);
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

	//localizing module-variables
	//input data
	double ck=flat6[101].real();
	//input from other modules
	double time=flat6[0].real();
	double trcode=missile[180].real();
	double tralp=missile[186].real();
	Matrix VBEB=flat6[213].vec();
	double pp=flat6[155].real();
	double qq=flat6[157].real();
	double rr=flat6[159].real();
	//state variables
	double q0d=flat6[110].real();
	double q0=flat6[111].real();
	double q1d=flat6[112].real();
	double q1=flat6[113].real();
	double q2d=flat6[114].real();
	double q2=flat6[115].real();
	double q3d=flat6[116].real();
	double q3=flat6[117].real();
	//-------------------------------------------------------------------------
	//integrating quaternions
	double quat_metric=q0*q0+q1*q1+q2*q2+q3*q3;
    double erq=1.-quat_metric;

    double new_q0d=0.5*(-pp*q1-qq*q2-rr*q3)+ck*erq*q0;
    double new_q1d=0.5*(pp*q0+rr*q2-qq*q3)+ck*erq*q1;
    double new_q2d=0.5*(qq*q0-rr*q1+pp*q3)+ck*erq*q2;
    double new_q3d=0.5*(rr*q0+qq*q1-pp*q2)+ck*erq*q3;

	q0=integrate(new_q0d,q0d,q0,int_step);
	q1=integrate(new_q1d,q1d,q1,int_step);
	q2=integrate(new_q2d,q2d,q2,int_step);
	q3=integrate(new_q3d,q3d,q3,int_step);

	q0d=new_q0d;
	q1d=new_q1d;
	q2d=new_q2d;
	q3d=new_q3d;

	//transformation matrix of body wrt local level coordinates
	TBL.dimension(3,3);
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

	//note: when |tbl13| >= 1., thtbl = +- pi/2, and cos(thtbl) is
	//		forced to be a small positive number to prevent divide by zero
	if(fabs(tbl13)<1){
		thtbl=asin(-tbl13);
		cthtbl=cos(thtbl);
	}
	else{
		thtbl=PI/2*sign(-tbl13);
		cthtbl=EPS;
	}
	//note: to avoid rare errors: #IND (quiet NaN) of angles psibl 
	//      and phibl (acos, asin problem), cpsi and sphi are forced 
	//		to be always <= (1.-EPS)
	double cpsi=tbl11/cthtbl;
	if(fabs(cpsi)>=1)
		cpsi=(1-EPS)*sign(cpsi);
	double cphi=tbl33/cthtbl;
	if(fabs(cphi)>=1)
		cphi=(1-EPS)*sign(cphi);

	psibl=acos(cpsi)*sign(tbl12);
	phibl=acos(cphi)*sign(tbl23);

	psiblx=DEG*psibl;
	thtblx=DEG*thtbl;
	phiblx=DEG*phibl;

	//incidence angles
	double vbeb1=VBEB.get_loc(0,0);
	double vbeb2=VBEB.get_loc(1,0);
	double vbeb3=VBEB.get_loc(2,0);

	alpha=atan2(vbeb3,vbeb1);

	double dvbe=VBEB.absolute();
	beta=asin(vbeb2/dvbe);

	double dum=vbeb1/dvbe;
	if(fabs(dum)>1)
		dum=1*sign(dum);
	alpp=acos(dum);
	if(vbeb2==0 && vbeb3==0)
		phip=0;
	//note: if vbeb2 is <EPS the value if phip is forced to be 0 or PI
	//		to prevent oscillations
	else if(fabs(vbeb2)<EPS){
		if(vbeb3>0) phip=0;
		if(vbeb3<0) phip=PI;
	}
	else
		phip=atan2(vbeb2,vbeb3);

	alphax=alpha*DEG;
	betax=beta*DEG;
	alppx=alpp*DEG;
	phipx=phip*DEG;

	//termination condition
	if(alpp>tralp) trcode=5.;

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
	flat6[120].gets_mat(TBL);
	flat6[140].gets(alppx);
	flat6[141].gets(phipx);
	flat6[142].gets(alpp);
	flat6[143].gets(phip);
	//diagnostics
	flat6[134].gets(psibl);
	flat6[135].gets(thtbl);
	flat6[136].gets(phibl);
	flat6[137].gets(psiblx);
	flat6[138].gets(thtblx);
	flat6[139].gets(phiblx);
	flat6[144].gets(alphax);
	flat6[145].gets(betax);
	flat6[146].gets(erq);
	flat6[147].gets(etbl);
	flat6[148].gets_mat(TLB);
}
