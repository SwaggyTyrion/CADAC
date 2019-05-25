///////////////////////////////////////////////////////////////////////////////
//FILE: 'tvc.cpp'
//Contains 'tvc' module of class 'Missile'
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of TVC (thrust vector control) module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[700-749]
//
//     mtvc=0 No TVC
//         =1 No dynamics
//         =2 TVC Second order dynamics with rate limiting
//         =3 same as 2 but with inline TVC gain
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_tvc()
{
	//Definition and initialization of module-variables
	missile[700].init("mtvc","int",0,"=0:no TVC;=1:no dyn;=2:scnd order;=3:2+gain","tvc","data","");
	missile[702].init("tvclimx",0,"Nozzle deflection limiter - deg","tvc","data","");
	missile[704].init("dtvclimx",0,"Nozzle deflection rate limiter - deg/s","tvc","data","");
	missile[705].init("wntvc",0,"Natural frequency of TVC - rad/s","tvc","data","");
	missile[706].init("zettvc",0,"Damping of TVC - ND","tvc","data","");
	missile[707].init("factgtvc",0,"Factor for TVC gain - ND","tvc","data","");
	missile[708].init("gtvc",0,"TVC nozzle deflection gain - ND","tvc","data,diag","");
	missile[709].init("parm",0,"Propulsion moment arm from vehicle nose - m","tvc","data","");
	missile[710].init("FPB",0,0,0,"Thrust force in body axes - N","tvc","out","");
	missile[711].init("FMPB",0,0,0,"Thrust moment in body axes - Nm","tvc","out","");
	missile[712].init("etax",0,"Nozzle pitch deflection - deg","tvc","diag","plot");
	missile[713].init("zetx",0,"Nozzle yaw deflection - deg","tvc","diag","plot");
	missile[714].init("etacx",0,"Commanded nozzle pitch deflection - deg","tvc","diag","plot");
	missile[715].init("zetcx",0,"Commanded nozzle yaw deflection - deg","tvc","diag","plot");
	missile[730].init("etasd",0,"Pitch nozzle derivative - rad/s","tvc","state","");
	missile[731].init("zetad",0,"Yaw nozzle derivative - rad/s","tvc","state","");
	missile[734].init("etas",0,"Pitch nozzle deflection - rad","tvc","state","");
	missile[735].init("zeta",0,"Yaw nozzle deflection - rad","tvc","state","");
	missile[738].init("detasd",0,"Pitch nozzle rate derivative - rad/s^2","tvc","state","");
	missile[739].init("dzetad",0,"Yaw nozzle rate derivative - rad/s^2","tvc","state","");
	missile[742].init("detas",0,"Pitch nozzle rate - rad/s","tvc","state","plot");
	missile[743].init("dzeta",0,"Yaw nozzle rate - rad/s","tvc","state","plot");
}	
///////////////////////////////////////////////////////////////////////////////
//Actuator module
//Member function of class 'Missile'
//     mtvc=0 No TVC
//         =1 No dynamics
//         =2 TVC Second order dynamics with rate limiting
//         =3 same as 2 but with inline TVC gain
//
// This subroutine performs the following functions:
// (1) Converts from control commands to nozzle deflections
// (2) Calls nozzle dynamic subroutine
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::tvc(double int_step)
{
	//local variables
	double eta(0),zet(0);
	
	//local module-variables
	Matrix FPB(3,1);
	Matrix FMPB(3,1);
	double etax(0),zetx(0);
	double etacx(0),zetcx(0);

	//localizing module-variables
	//input data
	int mtvc=missile[700].integer();
	double factgtvc=missile[707].real();
	double gtvc=missile[708].real();
	double parm=missile[709].real();
	//input from other modules
	double time=flat6[0].real();
	double thrust=missile[63].real();
	double xcg=missile[70].real();
	double dqcx=missile[520].real();
	double drcx=missile[521].real();
	double pdynmc=flat6[57].real();
	//-------------------------------------------------------------------------
	//return if no tvc
	if(mtvc==0) return;

	//variable nozzle reduction gain (low q, high gain)
	if(mtvc==3){
		if(pdynmc>1e5)
			gtvc=0;
		else
			gtvc=(-5.e-6*pdynmc+0.5)*(factgtvc+1);
	}
	//reduction of nozzle deflection command
	double etac=gtvc*dqcx*RAD;
	double zetc=gtvc*drcx*RAD;

	//no nozzle dynamics
	if(mtvc==1){
		eta=etac;
		zet=zetc;
	}
	//calling second order nozzle dynamics
	if(mtvc>=2)
		tvc_scnd(eta,zet,etac,zetc,int_step);

	//thrust forces in body axes
	double seta=sin(eta);
	double ceta=cos(eta);
	double czet=cos(zet);
	double szet=sin(zet);
	FPB[0]=ceta*czet*thrust;
	FPB[1]=ceta*szet*thrust;
	FPB[2]=-seta*thrust;

	//thrust moments in body axes
	double arm=parm-xcg;
	FMPB[0]=0;
	FMPB[1]=arm*FPB[2];
	FMPB[2]=-arm*FPB[1];

	//output
	etax=eta*DEG;
	zetx=zet*DEG;

	//diagnostic
	etacx=etac*DEG;
	zetcx=zetc*DEG;

	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[710].gets_vec(FPB);
	missile[711].gets_vec(FMPB);
	//diagnostics
	missile[712].gets(etax);
	missile[713].gets(zetx);
	missile[714].gets(etax);
	missile[715].gets(zetx);
}	
///////////////////////////////////////////////////////////////////////////////
//Second order TVC
//Member function of class 'Missile'
// This subroutine performs the following functions:
// (1) Models second order lags of pitch and yaw deflections
// (2) Limits nozzle deflections
// (3) Limits nozzle deflection rates
//
//Return output
//          eta=Nozzle pitch deflection - rad
//          zet=Nozzle yaw deflection - rad
// Argument Input:
//          etac=Nozzle pitch command - rad
//          zetc=Nozzle yaw command - rad
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::tvc_scnd(double &eta,double &zet,double etac,double zetc,double int_step)
{	
	//localizing module-variables
	//input data
	double tvclimx=missile[702].real();
	double dtvclimx=missile[704].real();
	double wntvc=missile[705].real();
	double zettvc=missile[706].real();
	//input from other modules
	double time=flat6[0].real();
	//state variables
	double etasd=missile[730].real();
	double zetad=missile[731].real();
	double etas=missile[734].real();
	double zeta=missile[735].real();
	double detasd=missile[738].real();
	double dzetad=missile[739].real();
	double detas=missile[742].real();
	double dzeta=missile[743].real();
	//-------------------------------------------------------------------------
	//pitch nozzle dynamics
	//limiting position and the nozzle rate derivative
	if(fabs(etas)>tvclimx*RAD){
		etas=tvclimx*RAD*sign(etas);
		if(etas*detas>0.)detas=0.;
	}
	//limiting nozzle rate
	int iflag=0;
	if(fabs(detas)>dtvclimx*RAD){
		iflag=1;
		detas=dtvclimx*RAD*sign(detas);
	}
	//state integration
	double etasd_new=detas;
	etas=integrate(etasd_new,etasd,etas,int_step);

	etasd=etasd_new;
	double eetas=etac-etas;
	double detasd_new=wntvc*wntvc*eetas-2.*zettvc*wntvc*etasd;
	detas=integrate(detasd_new,detasd,detas,int_step);
	detasd=detasd_new;
	//setting nozzle rate derivative to zero if rate is limited
	if(iflag&&detas*detasd>0.) detasd=0.;
	eta=etas;

	//yaw nozzle dynamics
	//limiting position and the nozzle rate derivative
	if(fabs(zeta)>tvclimx*RAD){
		zeta=tvclimx*RAD*sign(zeta);
		if(zeta*dzeta>0.)dzeta=0.;
	}
	//limiting nozzle rate
	iflag=0;
	if(fabs(dzeta)>dtvclimx*RAD){
		iflag=1;
		dzeta=dtvclimx*RAD*sign(dzeta);
	}
	//state integration
	double zetad_new=dzeta;
	zeta=integrate(zetad_new,zetad,zeta,int_step);
	zetad=zetad_new;
	double ezeta=zetc-zeta;
	double dzetad_new=wntvc*wntvc*ezeta-2.*zettvc*wntvc*zetad;
	dzeta=integrate(dzetad_new,dzetad,dzeta,int_step);
	dzetad=dzetad_new;
	//setting nozzle rate derivative to zero if rate is limited
	if(iflag&&dzeta*dzetad>0.) dzetad=0.;
	zet=zeta;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[730].gets(etasd);
	missile[731].gets(zetad);
	missile[734].gets(etas);
	missile[735].gets(zeta);
    missile[738].gets(detasd);
    missile[739].gets(dzetad);
	missile[742].gets(detas);
	missile[743].gets(dzeta);
}