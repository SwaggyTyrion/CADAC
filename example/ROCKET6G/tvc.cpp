///////////////////////////////////////////////////////////////////////////////
//FILE: 'tvc.cpp'
//Contains 'tvc' module of class 'Hyper'
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of TVC (thrust vector control) module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[900-949]
//
//     mtvc=0 No TVC
//         =1 No dynamics
//         =2 TVC Second order dynamics with rate limiting
//         =3 same as 2 but with on-line TVC gain
//
//030608 Created by Peter H Zipfel
//051207 Modified for GSWS6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_tvc()
{
	//Definition and initialization of module-variables
	hyper[900].init("mtvc","int",0,"=0:no TVC;=1:no dyn;=2:scnd order;=3:2+gain","tvc","data","");
	hyper[902].init("tvclimx",0,"Nozzle deflection limiter - deg","tvc","data","");
	hyper[904].init("dtvclimx",0,"Nozzle deflection rate limiter - deg/s","tvc","data","");
	hyper[905].init("wntvc",0,"Natural frequency of TVC - rad/s","tvc","data","");
	hyper[906].init("zettvc",0,"Damping of TVC - ND","tvc","data","");
	hyper[907].init("factgtvc",0,"Factor for TVC gain - ND","tvc","data","");
	hyper[908].init("gtvc",0,"TVC nozzle deflection gain - ND","tvc","data,diag","");
	hyper[909].init("parm",0,"Propulsion moment arm from vehicle nose - m","tvc","data","");
	hyper[910].init("FPB",0,0,0,"Thrust force in body axes - N","tvc","out","");
	hyper[911].init("FMPB",0,0,0,"Thrust moment in body axes - Nm","tvc","out","");
	hyper[912].init("etax",0,"Nozzle pitch deflection - deg","tvc","diag","plot");
	hyper[913].init("zetx",0,"Nozzle yaw deflection - deg","tvc","diag","plot");
	hyper[914].init("etacx",0,"Commanded nozzle pitch deflection - deg","tvc","diag","");
	hyper[915].init("zetcx",0,"Commanded nozzle yaw deflection - deg","tvc","diag","");
	hyper[916].init("etasd",0,"Pitch nozzle derivative - rad/s","tvc","state","");
	hyper[917].init("zetad",0,"Yaw nozzle derivative - rad/s","tvc","state","");
	hyper[918].init("etas",0,"Pitch nozzle deflection - rad","tvc","state","");
	hyper[919].init("zeta",0,"Yaw nozzle deflection - rad","tvc","state","");
	hyper[920].init("detasd",0,"Pitch nozzle rate derivative - rad/s^2","tvc","state","");
	hyper[921].init("dzetad",0,"Yaw nozzle rate derivative - rad/s^2","tvc","state","");
	hyper[922].init("detas",0,"Pitch nozzle rate - rad/s","tvc","state","");
	hyper[923].init("dzeta",0,"Yaw nozzle rate - rad/s","tvc","state","");
}	
///////////////////////////////////////////////////////////////////////////////
//Actuator module
//Member function of class 'Hyper'
//     mtvc=0 No TVC
//         =1 No dynamics
//         =2 TVC Second order dynamics with rate limiting
//         =3 same as 2 but with on-line TVC gain
//
// This subroutine performs the following functions:
// (1) Converts from control commands to nozzle deflections
// (2) Calls nozzle dynamic subroutine
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::tvc(double int_step)
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
	int mtvc=hyper[900].integer();
	double factgtvc=hyper[907].real();
	double gtvc=hyper[908].real();
	double parm=hyper[909].real();
	//input from other modules
	double time=round6[0].real();
	double pdynmc=round6[57].real();
	double xcg=hyper[17].real();
	double thrust=hyper[26].real();
	double delecx=hyper[520].real();
	double delrcx=hyper[521].real();
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
	double etac=gtvc*delecx*RAD;
	double zetc=gtvc*delrcx*RAD;

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
	hyper[910].gets_vec(FPB);
	hyper[911].gets_vec(FMPB);
	//diagnostics
	hyper[912].gets(etax);
	hyper[913].gets(zetx);
	hyper[914].gets(etacx);
	hyper[915].gets(zetcx);
}	
///////////////////////////////////////////////////////////////////////////////
//Second order TVC
//Member function of class 'Hyper'
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

void Hyper::tvc_scnd(double &eta,double &zet,double etac,double zetc,double int_step)
{	
	//localizing module-variables
	//input data
	double tvclimx=hyper[902].real();
	double dtvclimx=hyper[904].real();
	double wntvc=hyper[905].real();
	double zettvc=hyper[906].real();
	//input from other modules
	double time=round6[0].real();
	//state variables
	double etasd=hyper[916].real();
	double zetad=hyper[917].real();
	double etas=hyper[918].real();
	double zeta=hyper[919].real();
	double detasd=hyper[920].real();
	double dzetad=hyper[921].real();
	double detas=hyper[922].real();
	double dzeta=hyper[923].real();
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
	hyper[916].gets(etasd);
	hyper[917].gets(zetad);
	hyper[918].gets(etas);
	hyper[919].gets(zeta);
    hyper[920].gets(detasd);
    hyper[921].gets(dzetad);
	hyper[922].gets(detas);
	hyper[923].gets(dzeta);
}