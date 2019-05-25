///////////////////////////////////////////////////////////////////////////////
//FILE: 'attitude.cpp'
//Contains 'attitude' module of class 'Rotor'
//
//130523 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'attitude' module-variables 
//Member function of class 'Rotor
//Module-variable locations are assigned to rotor[200-299]
//
//Attitude equations of motion
//Equations are in dynamic normalized units (DNU) and are dimensionless
//Ref: Zipfel, 'On Flight Dynamics of Magnus Rotors',
// DTIC AD 716345, NOv 1970,Chapter 5, Normalization, p 43
// 
//130523 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rotor::def_attitude()
{
	//Definition and initialization of module-variables
    rotor[200].init("nonlinear","int",0,"Add nonlinear terms; =0:No; =1:Yes","attitude","data","");
	rotor[201].init("moi_trans",0,"Transverse moment of intertia - kg.m^2","attitude","data","");
    rotor[202].init("cyb",0,"Side force derivative - 1/rad","attitude","data","");
    rotor[203].init("cyb3",0,"Cubic side force derivative - 1/rad^3","attitude","data","");
    rotor[204].init("clwb",0,"Magnus moment derivative - 1/rad","attitude","data","");
    rotor[205].init("clp",0,"roll damping derivative - 1/rad","attitude","data","");
    rotor[206].init("clwb3",0,"Cubic Magnus moment derivative - 1/rad^3","attitude","data","");
    rotor[207].init("clp3",0,"Cubic roll damping derivative - 1/rad^3","attitude","data","");
    rotor[208].init("clwb2p",0,"Sideslip squared-roll damping derivative - 1/rad^3","attitude","data","");
    rotor[209].init("clwbp2",0,"Sideslip-roll squared damping derivative - 1/rad^3","attitude","data","");
    rotor[210].init("cnb",0,"Yawing moment derivative - 1/rad","attitude","data","");
    rotor[211].init("cnr",0,"Yaw damping derivative - 1/rad","attitude","data","");
	rotor[212].init("cnb3",0,"Cubic yawing moment derivative - 1/rad^3","attitude","data","");
    rotor[213].init("cnr3",0,"Cubic yaw damping derivative - 1/rad^3","attitude","data","");
    rotor[214].init("cnb2r",0,"Sideslip squared-yaw damping derivative - 1/rad^3","attitude","data","");
    rotor[215].init("cnbr2",0,"Sideslip-yaw squared damping derivative - 1/rad^3","attitude","data","");
	rotor[220].init("beta",0,"Sideslip angle - rad","attitude","state","");	
    rotor[221].init("betad",0,"Sideslip angle derivative - rad","attitude","state","");	
    rotor[222].init("phi",0,"Roll angle - rad","attitude","state","");	
	rotor[223].init("phid",0,"Roll angle derivative - rad","attitude","state","");	
	rotor[224].init("phidd",0,"Roll angle 2nd derivative - rad","attitude","state","");	
    rotor[225].init("psi",0,"Yaw angle - rad","attitude","state","");	
	rotor[226].init("psid",0,"Yaw angle derivative - rad","attitude","state","");	
	rotor[227].init("psidd",0,"Yaw angle 2nd derivative - rad","attitude","state","");	
	rotor[230].init("betax",0,"Sideslip angle - deg","attitude","dia","scrn,plot");	
	rotor[231].init("phix",0,"Roll angle - deg","attitude","dia","scrn,plot");	
	rotor[232].init("ppx",0,"Roll rate - deg/s","attitude","dia","scrn,plot");	
	rotor[233].init("psix",0,"Yaw angle - deg","attitude","dia","scrn,plot");	
	rotor[234].init("rrx",0,"Yaw rate - deg/s","attitude","dia","scrn,plot");	
}
///////////////////////////////////////////////////////////////////////////////
//Attitude initialization module
//Member function of class 'Rotor'
//
//Initialization of state variables
//130530 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Rotor::init_attitude()
{
	//module variables
	double beta(0);
	double phi(0);
	double phid(0);
	double psi(0);
	double psid(0);
	
	//initializations
	double betax=rotor[230].real();
	double phix=rotor[231].real();
	double ppx=rotor[232].real();
	double psix=rotor[233].real();
	double rrx=rotor[234].real();
	//input from other modules
	double tau=rotor[145].real();
	//-------------------------------------------------------------------------
	//initializing state variables
	beta=betax*RAD;
	phi=phix*RAD;
	phid=ppx*RAD*tau;
	psi=psix*RAD;
	psid=rrx*RAD*tau;
	//-------------------------------------------------------------------------
	//state variables
	rotor[220].gets(beta);
	rotor[222].gets(phi);
	rotor[223].gets(phid);
	rotor[225].gets(psi);
	rotor[226].gets(psid);
}
///////////////////////////////////////////////////////////////////////////////
//Attitude perturbation module
//Member function of class 'Rotor'
//
//Attitude equations of motion
//Ref: Zipfel, 'On Flight Dynamics of Magnus Rotors',
//  DTIC AD 716345, NOv 1970, Table 12.1, Eqs. 4-8
//
//130530 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Rotor::attitude(double int_step)
{
	//local variable
	double moi_transx(0);

	//local module-variables
	double betax(0);
	double phix(0);
	double ppx(0);
	double psix(0);
	double rrx(0);

	//localizing module-variables
	//input data
	int nonlinear=rotor[200].integer();
	double moi_trans=rotor[201].real();
	double cyb=rotor[202].real();
	double cyb3=rotor[203].real();
	double clwb=rotor[204].real();
	double clp=rotor[205].real();
	double clwb3=rotor[206].real();
	double clp3=rotor[207].real();
	double clwb2p=rotor[208].real();
	double clwbp2=rotor[209].real();
	double cnb=rotor[210].real();
	double cnr=rotor[211].real();
	double cnb3=rotor[212].real();
	double cnr3=rotor[213].real();
	double cnb2r=rotor[214].real();
	double cnbr2=rotor[215].real();
	//input from other modules
	double grav=rotor[55].real();
	double mass=rotor[114].real();
	double ref_length=rotor[116].real();
	double velocity_ss=rotor[118].real();
	double velocityx=rotor[133].real();
	double velocityxd=rotor[134].real();
	double gamma=rotor[135].real();
	double omegax=rotor[137].real();
	double tau=rotor[145].real();
	double eng_ang_mom=rotor[213].real();
	double moi_spinx=rotor[140].real();
	double mu=rotor[146].real();
	//state variable
	double beta=rotor[220].real();
	double betad=rotor[221].real();
	double phi=rotor[222].real();
	double phid=rotor[223].real();
	double phidd=rotor[224].real();
	double psi=rotor[225].real();
	double psid=rotor[226].real();
	double psidd=rotor[227].real();
	//-------------------------------------------------------------------------
	//transverse moment of inertia in DNU (dynamic normalized units)
	moi_transx=moi_trans/(ref_length*ref_length*mu*mu*mass);

	//*attitude equations of motion
	//sideslip angle beta 
	double betad_new=(-velocityxd/velocityx+velocityx*cyb)*beta-psid
						+tau*grav*cos(gamma)/(velocityx*velocity_ss)*phi
						+tau*grav*sin(gamma)/(velocityx*velocity_ss)*psi
			+nonlinear*(velocityx*cyb3*pow(beta,3)/6);
	beta=integrate(betad_new,betad,beta,int_step);
	betad=betad_new;
	//roll angle derivative phid
	double phidd_new=velocityx*omegax*clwb/(mu*mu*moi_transx)*beta
						+velocityx*clp/(mu*mu*moi_transx)*phid
						+moi_spinx*omegax/moi_transx*psid
			+nonlinear*(velocityx*omegax*clwb3/(6*mu*mu*moi_transx)*pow(beta,3)
						+clp3/(6*pow(mu,4)*moi_transx*velocityx)*pow(phid,3)
						+omegax*clwb2p/(6*pow(mu,3)*moi_transx)*beta*beta*phid
						+omegax*clwbp2/(6*pow(mu,4)*moi_transx*velocityx)*beta*phid*phid);
	phid=integrate(phidd_new,phidd,phid,int_step);
	phidd=phidd_new;
	double phid_new=phid;
	phi=integrate(phid_new,phid,phi,int_step);
	phid=phid_new;
	//yaw angle derivative psid
	double psidd_new=velocityx*velocityx*cnb/(mu*moi_transx)*beta
						-moi_spinx*omegax/moi_transx*phid
						+velocityx*cnr/(mu*mu*moi_transx)*psid
				+nonlinear*(velocityx*velocityx*cnb3/(6*mu*moi_transx)*pow(beta,3)
						+cnr3/(6*pow(mu,4)*moi_transx*velocityx)*pow(psid,3)
						+velocityx*cnb2r/(6*mu*mu*moi_transx)*beta*beta*psid
						+cnbr2/(6*pow(mu,3)*moi_transx)*beta*psid*psid);
	psid=integrate(psidd_new,psidd,psid,int_step);
	psidd=psidd_new;
	double psid_new=psid;
	psi=integrate(psid_new,psid,psi,int_step);
	psid=psid_new;

	//output in degrees and sec
	betax=beta*DEG;
	phix=phi*DEG;
	ppx=phid*DEG/tau;
	psix=psi*DEG;
	rrx=psid*DEG/tau;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	rotor[220].gets(beta);
	rotor[221].gets(betad);
	rotor[222].gets(phi);
	rotor[223].gets(phid);
	rotor[224].gets(phidd);
	rotor[225].gets(psi);
	rotor[226].gets(psid);
	rotor[227].gets(psidd);
	//diagnostics
	rotor[230].gets(betax);
	rotor[231].gets(phix);
	rotor[232].gets(ppx);
	rotor[233].gets(psix);
	rotor[234].gets(rrx);
}