///////////////////////////////////////////////////////////////////////////////
//FILE: 'control.cpp'
//Contains 'control' module of class 'Hyper'
//
//030520 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of control module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[500-599]
// 
// maut = |mauty|mautp|
//
//         mauty = 0 no control, fixed control surfaces
//               = 5 yaw acceleration control for yaw-to-turn
//
//               mautp = 0 no control, fixed control surfaces
//                     = 3 pitch acceleration control
//
//030520 Created by Peter H Zipfel
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////
 
void Hyper::def_control()
{
 	//Definition and initialization of module-variables
 	hyper[500].init("maut","int",0,"maut=|mauty|mautp| see table","control","data","");
	hyper[503].init("mfreeze","int",0,"=0:Unfreeze; =1:Freeze; increment for more","control","data","");
	hyper[504].init("waclp",0,"Nat freq of accel close loop complex pole - rad/s","control","data","plot");
	hyper[505].init("zaclp",0,"Damping of accel close loop complex pole - ND","control","data","plot");
	hyper[506].init("paclp",0,"Close loop real pole - ND","control","data","plot");
	hyper[509].init("delimx",0,"Pitch command limiter - deg","control","data","");
	hyper[510].init("drlimx",0,"Yaw command limiter - deg","control","data","");
	hyper[514].init("yyd",0,"Yaw feed-forward derivative variable - m/s^2","control","state","");
	hyper[515].init("yy",0,"Yaw feed-forward integration variable - m/s","control","state","");
	hyper[516].init("zzd",0,"Pitch feed-forward derivative variable - m/s^2","control","state","");
	hyper[517].init("zz",0,"Pitch feed-forward integration variable - m/s","control","state","");
	hyper[520].init("delecx",0,"Pitch command deflection - deg","control","out","");
	hyper[521].init("delrcx",0,"Yaw  command deflection - deg","control","out","");
	hyper[522].init("alcomx_actual",0,"Later accel com limited by 'betalimx' - g's","control","diag","plot");
	hyper[523].init("ancomx_actual",0,"Normal accel com limited by 'alplimx' - g's","control","diag","plot");
	hyper[524].init("GAINFP",0,0,0,"Feedback gains of pitch accel controller","control","diag","plot");
	hyper[525].init("gainp",0,"Proportional gain in pitch acceleration loop - s^2/m","control","data","");
	hyper[526].init("gainl",0,"Gain in lateral acceleration loop - rad/g's","control","data","");
	hyper[531].init("gkp",0,"Gain of roll rate feedback - s","control","diag","");
	hyper[532].init("gkphi",0,"Gain of roll angle feedback -","control","diag","");
	hyper[545].init("isetc2",0,"Flag to print freeze variables","control","init","");
	hyper[568].init("wacly",0,"Nat freq of accel close loop pole, yaw - rad/s","control","data","plot");
	hyper[569].init("zacly",0,"Damping of accel close loop pole, yaw - ND","control","data","");
	hyper[570].init("pacly",0,"Close loop real pole, yaw - ND","control","data","");
	hyper[571].init("gainy",0,"Gain in lateral acceleration loop - rad/g's","control","data","");
	hyper[572].init("GAINFY",0,0,0,"Feedback gains of yaw accel controller","control","diag","");
	hyper[573].init("factwaclp",0,"Factor to mod 'waclp': waclp*(1+factwacl) - ND","control","data","");
	hyper[574].init("factwacly",0,"Factor to mod 'wacly': wacly*(1+factwacl) - ND","control","data","");
	hyper[575].init("alcomx",0,"Lateral (horizontal) acceleration command - g's","control","data","plot");
	hyper[576].init("ancomx",0,"Pitch (normal) acceleration command - g's","control","data","plot");
}
///////////////////////////////////////////////////////////////////////////////
//'control' module
//Member function of class 'Hyper'
//
// maut = |mauty|mautp|
//
//         mauty = 0 no control, fixed control surfaces
//               = 5 yaw acceleration control for yaw-to-turn
//
//               mautp = 0 no control, fixed control surfaces
//                     = 3 pitch acceleration control
//
//030520 Created by Peter H Zipfel
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::control(double int_step)
{
	//local module-variables
	double delacx(0),delecx(0),delrcx(0);
	double alcomx_actual(0),ancomx_actual(0);
	
	//localizing module-variables
	//input data
	int maut=hyper[500].integer();
	double dalimx=hyper[508].real();
	double delimx=hyper[509].real();
	double drlimx=hyper[510].real();
	double philimx=hyper[511].real();
	double phicomx=hyper[553].real();
	//input from other modules
	double time=round6[0].real();
	int mprop=hyper[10].integer();
	double gnmax=hyper[191].real();
	double gymax=hyper[192].real();
	double alcomx=hyper[575].real(); 
	double ancomx=hyper[576].real();
	//-------------------------------------------------------------------------
	//decoding control flag
    int mauty=maut/10;
    int mautp=(maut%10);

	//calling acceleration controller in yaw-plane 
	if(mauty==5){
		//limiting lateral acceleration by max g capability (estabished by 'betalimx')
		if(alcomx>gymax) alcomx=gymax;
		if(alcomx<-gymax) alcomx=-gymax;
		//for booster while thrusting - TVC control
		if(mprop) 
			delrcx=control_yaw_accel(alcomx,int_step);
	}
	//calling acceleration controller in normal-plane
	if(mautp==3){
		if(ancomx>gnmax) ancomx=gnmax;
		if(ancomx<-gnmax) ancomx=-gnmax;
		//for booster while thrusting - TVC control
		if(mprop) 
			delecx=control_normal_accel(ancomx,int_step);
	}
	//limiting control commands
	if(fabs(delecx)>delimx) delecx=delimx*sign(delecx);
	if(fabs(delrcx)>drlimx) delrcx=drlimx*sign(delrcx);

	//diagnostic output
	alcomx_actual=alcomx;
	ancomx_actual=ancomx;
	//-------------------------------------------------------------------------
	//output to other modules
	hyper[520].gets(delecx);
	hyper[521].gets(delrcx);
	//diagnostics
	hyper[522].gets(alcomx_actual);
	hyper[523].gets(ancomx_actual);
}	

///////////////////////////////////////////////////////////////////////////////
//Acceleration controller in normal (pitch) plane
//Member function of class 'Hyper'
//Employs pole placement technique (no matrix inversion required)
//Ref: Zipfel, p.416
//Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded pitch control deflection
//
//Return output
//		delecx = pitch control command - deg
//Parameter input
//		ancomx = normal loadfactor command - g
//		int_step = integration step size - s
//		
//021015 Created by Peter H Zipfel
//060120 Added variable bandwidth, PZi
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

double Hyper::control_normal_accel(double ancomx,double int_step)
{
	//local module-variables
	Matrix GAINFP(3,1);

	//localizing module-variables
	//input data
	double waclp=hyper[504].real();
	double zaclp=hyper[505].real();
	double paclp=hyper[506].real();
	double gainp=hyper[525].real();
	double factwaclp=hyper[573].real();
	//input from other modules
	double time=round6[0].real();
	double pdynmc=round6[57].real();
	int maero=hyper[100].integer();
	double dla=hyper[145].real();
	double dma=hyper[147].real();
	double dmq=hyper[148].real();
	double dmde=hyper[149].real();
	double dvbec=hyper[330].real();
	double qqcx=hyper[321].real();
	Matrix FSPCB=hyper[334].vec();
	//state variables
	double zzd=hyper[516].real();
	double zz=hyper[517].real();
	//-------------------------------------------------------------------------
	//calculating online close loop poles
	waclp=(0.1+0.5e-5*(pdynmc-20e3))*(1+factwaclp);
	paclp=0.7+1e-5*(pdynmc-20e3)*(1+factwaclp);

		//calculating three feedback gains
	double gainfb3=waclp*waclp*paclp/(dla*dmde);
	double gainfb2=(2*zaclp*waclp+paclp+dmq-dla/dvbec)/dmde;
	double gainfb1=(waclp*waclp+2.*zaclp*waclp*paclp+dma+dmq*dla/dvbec
				-gainfb2*dmde*dla/dvbec)/(dla*dmde)-gainp;

	//pitch loop acceleration control, pitch control command
	double fspb3=FSPCB[2];
    double zzd_new=AGRAV*ancomx+fspb3;
	zz=integrate(zzd_new,zzd,zz,int_step);
	zzd=zzd_new;
	double dqc=-gainfb1*(-fspb3)-gainfb2*qqcx*RAD+gainfb3*zz+gainp*zzd;
    double delecx=dqc*DEG;

	//diagnostic output
	GAINFP.build_vec3(gainfb1,gainfb2,gainfb3);
	//--------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[516].gets(zzd);
	hyper[517].gets(zz);
	//diagnostics
	hyper[524].gets_vec(GAINFP);
	hyper[504].gets(waclp);
	hyper[505].gets(zaclp);
	hyper[506].gets(paclp);

	return delecx;
}
///////////////////////////////////////////////////////////////////////////////
//Acceleration controller in lateral (yaw) plane
//Member function of class 'Hyper'
//Employs pole placement technique (no matrix inversion required) 
//Ref: Zipfel, p.416
//Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded yaw control deflection
//
//Return output
//		drcx = yaw control command - deg
//Parameter input
//		alcomx = lateral loadfactor command = g
//		int_step = integration step size - s
//		
//050104 Adopted from DRM sim, PZi
//060120 Added variable bandwidth, PZi
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

double Hyper::control_yaw_accel(double alcomx,double int_step)
{
	//local module-variables
	Matrix GAINFY(3,1);

	//localizing module-variables
	//input data
	double wacly=hyper[568].real();
	double zacly=hyper[569].real();
	double pacly=hyper[570].real();
	double gainy=hyper[571].real();
	double factwacly=hyper[574].real();
	//input from other modules
	double pdynmc=round6[57].real();
	int maero=hyper[100].integer();
	double dyb=hyper[150].real();
	double dnb=hyper[152].real();
	double dnr=hyper[153].real();
	double dndr=hyper[154].real();
	double dvbe=round6[225].real();
	double rrcx=hyper[322].real();
	Matrix FSPCB=hyper[334].vec();
	//state variables
	double yyd=hyper[514].real();
	double yy=hyper[515].real();
	//-------------------------------------------------------------------------
	//calculating close loop poles
	wacly=(0.1+0.5e-5*(pdynmc-20e3))*(1+factwacly);
	pacly=0.7+1e-5*(pdynmc-20e3)*(1+factwacly);

	//calculating three feedback gains
	double gainfb3=-wacly*wacly*pacly/(dyb*dndr);
	double gainfb2=(2*zacly*wacly+pacly+dnr+dyb/dvbe)/dndr;
	double gainfb1=(-wacly*wacly-2.*zacly*wacly*pacly+dnb+dnr*dyb/dvbe
				-gainfb2*dndr*dnb/dvbe)/(dyb*dndr)-gainy;

	//yaw loop acceleration controller, yaw control command
	double fspb2=FSPCB.get_loc(1,0);			
	double yyd_new=AGRAV*alcomx-fspb2;
	yy=integrate(yyd_new,yyd,yy,int_step);
	yyd=yyd_new;
    double drc=-gainfb1*fspb2-gainfb2*rrcx*RAD+gainfb3*yy+gainy*yyd;
    double drcx=drc*DEG;

	//diagnostic output
	GAINFY.build_vec3(gainfb1,gainfb2,gainfb3);
	//--------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[514].gets(yyd);
	hyper[515].gets(yy);
	//diagnostics
	hyper[572].gets_vec(GAINFY);
	hyper[568].gets(wacly);
	hyper[569].gets(zacly);
	hyper[570].gets(pacly);

	return drcx;
}





