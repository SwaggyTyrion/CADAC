///////////////////////////////////////////////////////////////////////////////
//FILE: 'tvc.cpp'
//Contains 'tvc' module of class 'Missile'
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'tvc' (thrust vector control) module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[700-749]
//
//     mtvc=0 No TVC
//         =1 No dynamics
//         =2 TVC Second order dynamics with rate limiting and constant gain
//         =3 same as 2 but with exponentially decaying gain
//
//030608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::def_tvc()
{
	//Definition and initialization of module-variables
	missile[700].init("mtvc","int",0,"=0:no TVC;=1:no dyn;=2:dynamics;=3:2+var.gain","tvc","data","");
	missile[702].init("tvclimx",0,"Nozzle deflection limiter - deg","tvc","data","");
	missile[704].init("dtvclimx",0,"Nozzle deflection rate limiter - deg/s","tvc","data","");
	missile[705].init("wntvc",0,"Natural frequency of TVC - rad/s","tvc","data","");
	missile[706].init("zettvc",0,"Damping of TVC - ND","tvc","data","");
	missile[707].init("pdynmc_gtvc36",0,"Dynamic pressure for 36% of gtvc - ND","tvc","data","");
	missile[708].init("gtvc0",0,"Initial or constant TVC nozzle deflection gain - ND","tvc","data","");
	missile[709].init("parm",0,"Propulsion moment arm from vehicle nose - m","tvc","data","");
	missile[710].init("FPB",0,0,0,"Thrust force in body axes - N","tvc","out","");
	missile[711].init("FMPB",0,0,0,"Thrust moment in body axes - Nm","tvc","out","");
	missile[712].init("etax",0,"Nozzle pitch deflection - deg","tvc","diag","plot");
	missile[713].init("zetx",0,"Nozzle yaw deflection - deg","tvc","diag","plot");
	missile[714].init("etacx",0,"Commanded nozzle pitch deflection - deg","tvc","diag","");
	missile[715].init("zetcx",0,"Commanded nozzle yaw deflection - deg","tvc","diag","");
	missile[716].init("gtvc",0,"Effective nozzle gain - ND","tvc","out","");
	missile[730].init("etasd",0,"Pitch nozzle derivative - rad/s","tvc","state","");
	missile[731].init("zetad",0,"Yaw nozzle derivative - rad/s","tvc","state","");
	missile[734].init("etas",0,"Pitch nozzle deflection - rad","tvc","state","");
	missile[735].init("zeta",0,"Yaw nozzle deflection - rad","tvc","state","");
	missile[738].init("detasd",0,"Pitch nozzle rate derivative - rad/s^2","tvc","state","");
	missile[739].init("dzetad",0,"Yaw nozzle rate derivative - rad/s^2","tvc","state","");
	missile[742].init("detas",0,"Pitch nozzle rate - rad/s","tvc","state","");
	missile[743].init("dzeta",0,"Yaw nozzle rate - rad/s","tvc","state","");
}	
///////////////////////////////////////////////////////////////////////////////
//'tvc' module
//Member function of class 'Missile'
//
//     mtvc=0 No TVC
//         =1 No dynamics
//         =2 TVC Second order dynamics with rate limiting and constant gain
//         =3 same as 2 but with exponentially decaying gain
//
// This subroutine performs the following functions:
// (1) Converts from control commands to nozzle deflections
// (2) Calls nozzle dynamic subroutine
//
//030608 Created by Peter H Zipfel
//071002 Changed variable gain to exponential, PZi
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
	double gtvc(0);

	//localizing module-variables
	//input data
	int mtvc=missile[700].integer();
	double pdynmc_gtvc36=missile[707].real();
	double gtvc0=missile[708].real();
	double parm=missile[709].real();
	//input from other modules
	double time=flat6[0].real();
	int mprop=missile[10].integer();
	int maut=missile[500].integer();
	double thrust=missile[13].real();
	double xcg=missile[15].real();
	double dqcx=missile[520].real();
	double drcx=missile[521].real();
	double dqcx_rcs=missile[526].real();
	double drcx_rcs=missile[527].real();
	double pdynmc=flat6[57].real();
	//-------------------------------------------------------------------------
	//enter TVC only if TVC is 'on' and motor burns
	if(mtvc>0 && mprop>0){

		//constant nozzle gain
		if(mtvc==2){
			gtvc=gtvc0;
		}		
		//variable nozzle gain 
		if(mtvc==3){
			gtvc=gtvc0*exp(-pdynmc/pdynmc_gtvc36);
		}
		//reduction of nozzle deflection command
		//but if separate rate damping (maut=4), making substitution
		if(maut==4){
			dqcx=dqcx_rcs;
			drcx=drcx_rcs;
		}
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

		//diagnostic
		etax=eta*DEG;
		zetx=zet*DEG;
		etacx=etac*DEG;
		zetcx=zetc*DEG;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[710].gets_vec(FPB);
	missile[711].gets_vec(FMPB);
	missile[716].gets(gtvc);
	//diagnostics
	missile[712].gets(etax);
	missile[713].gets(zetx);
	missile[714].gets(etacx);
	missile[715].gets(zetcx);
}	
///////////////////////////////////////////////////////////////////////////////
//Second order TVC
//Member function of class 'Missile'
// This subroutine performs the following functions:
// (1) Models second order lags of pitch and yaw deflections
// (2) Limits nozzle deflections
// (3) Limits nozzle deflection rates
//
//	Return output
//          eta=Nozzle pitch deflection - rad
//          zet=Nozzle yaw deflection - rad
// Argument input:
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