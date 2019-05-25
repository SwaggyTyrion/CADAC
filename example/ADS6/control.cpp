///////////////////////////////////////////////////////////////////////////////
//FILE: 'control.cpp'
//Contains 'control' module of class 'Missile'
//
//030612 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'control' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[500-599]
//
//     maut= 0 No control
//        >= 1 Roll controller
//         = 2 Rate controller
//         = 3 Acceleration controller
//		   = 4 Acceleration and additional rate controller for thrust moment control
// 
//030612 Created by Peter H Zipfel
//080321 Added TVC rate damping loop, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::def_control()
{
	//Definition and initialization of module-variables
	missile[500].init("maut","int",0," >0:Roll; =2:Rate; =3:Accel controller","control","data","");
	missile[501].init("mfreeze","int",0,"=0:Unfreeze; =1:Freeze; increment for more","control","data","");
	missile[502].init("wacl",0,"Nat freq of accel closed loop complex pole - rad/s","control","dia","plot");
	missile[505].init("zacl",0,"Damping of accel close loop complex pole - ND","control","dia","");
	missile[506].init("pacl",0,"Close loop real pole - ND","control","dia","plot");
	missile[507].init("alimitx",0,"Total structural acceleration limiter - g's","control","data","");
	missile[508].init("dqlimx",0,"Pitch flap control limiter - deg","control","data","");
	missile[509].init("drlimx",0,"Yaw flap control limiter - deg","control","data","");
	missile[510].init("dplimx",0,"Roll command limiter - deg","control","data","");
	missile[511].init("phicomx",0,"Commanded roll angle - deg","control","data","");
	missile[512].init("wrcl",0,"Freq of roll closed loop complex pole - rad/s","control","dia","plot");
	missile[513].init("zrcl",0,"Damping of roll closed loop pole - ND","control","data","");
	missile[514].init("yyd",0,"Yaw feed-forward derivative variable- m/s","control","state","");
	missile[515].init("yy",0,"Yaw feed-forward integration variable- m/s","control","state","");
	missile[516].init("zzd",0,"Pitch feed-forward derivative variable- m/s","control","state","");
	missile[517].init("zz",0,"Pitch feed-forward integration variable- m/s","control","state","");
	missile[519].init("dpcx",0,"Roll flap command deflection - deg","control","out","scrn,");
	missile[520].init("dqcx",0,"Pitch flap command deflection - deg","control","out","scrn,");
	missile[521].init("drcx",0,"Yaw flap command deflection - deg","control","out","scrn,");
	missile[522].init("tp",0,"Time constant of roll rate controller - sec","control","data","");
	missile[524].init("GAINFB",0,0,0,"Feedback gain of rate, accel and control","control","diag","");
	missile[525].init("gainp",0,"Feed-forward gain - s^2/m","control","data","");
	missile[526].init("dqcx_rcs",0,"Pitch flap command for RCS - deg","control","out","");
	missile[527].init("drcx_rcs",0,"Yaw flap command for RCS - deg","control","out","");
	missile[528].init("qqcomx",0,"Pitch rate command deg/s","control","data","");
	missile[529].init("rrcomx",0,"Yaw rate command deg/s","control","data","");
	missile[531].init("gkp",0,"Gain of roll rate feedback - s","control","diag","");
	missile[532].init("gkphi",0,"Gain of roll angle feedback -","control","diag","");
	missile[548].init("factwrcl",0,"Factor to change roll bandwidth - ND","control","data","");
	missile[550].init("zetlagr",0,"Desired damping of closed rate loop - ND","control","data","");
	missile[560].init("zrate",0,"Zero pole of rate TR.FCT. - 1/rad","control","diag","plot");
	missile[561].init("grate",0,"Feedback gain of rate loop - ND","control","diag","");
	missile[562].init("wnlagr",0,"Nat freq of closed rate loop - rad/s","control","diag","plot");
	missile[566].init("wacl_bias",0,"Bias of closed loop frequency 'wacl' - ND","control","data","");
	missile[567].init("pacl_bias",0,"Bias of closed loop pole 'pacl' - ND","control","data","");
	missile[568].init("zacl_bias",0,"Bias of closed loop damping 'zacl' - ND","control","data","");
	//acceleration controller check-out 
	missile[569].init("ancomx_test",0,"Testing pitch accel. control - g's","control","data","plot");
	missile[570].init("alcomx_test",0,"Testing yaw accel. control - g's","control","data","plot");

}
///////////////////////////////////////////////////////////////////////////////
//'control' module
//Member function of class 'Missile'
// (1) Selects control configurations
//     maut= 0 No control
//        >= 1 Roll controller
//         = 2 Rate controller
//         = 3 Acceleration controller
//		   = 4 Acceleration and additional rate controller for thrust moment control
//
// (2) Freezes flight conditions for control response.
//     Frozen variables are: dvbe, mach, pdynmc, thrust and mass parameters
//     Invoked by setting  mfreeze=1 and canceled by mfreeze=0.
//     For sequential maneuvers increment mfreeze by "1"
//
//030612 Created by Peter H Zipfel
//080321 Added TVC rate damping loop, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::control(double int_step)
{
	//localizing module-variables
	//input data
	int maut=missile[500].integer();
	//-------------------------------------------------------------------------
	//return if no control
	if(maut==0) return;

	//calling roll position controller
	control_roll();

	//calling rate controller
	if(maut==2) control_rate();

	//calling acceleration controller
	if(maut==3) control_accel(int_step);

	//calling acceleration controller and rate controller
	if(maut==4){
		control_rate();
		control_accel(int_step);
	}
	//-------------------------------------------------------------------------
}	
///////////////////////////////////////////////////////////////////////////////
//Roll controller
//Member function of class 'Missile'
// (1) Calculates gains
// (2) Calculates commanded roll control
//
//170901 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::control_roll()
{
	//local module-variables
	double gkp(0);
	double gkphi(0);
	double dpcx(0);

	//localizing module-variables
	//input data
	double phicomx=missile[511].real();
	double wrcl=missile[512].real();
	double zrcl=missile[513].real();
	double tp=missile[522].real();
	double factwrcl=missile[548].real();
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double dlp=missile[150].real();
	double dld=missile[151].real();
	double thtblcx=missile[329].real();
	Matrix WBECB=missile[306].vec();
	double phiblcx=missile[335].real();
	//save value
//	double dpcx=missile[519].real();
	//-------------------------------------------------------------------------
	//Variable roll bandwidth
    wrcl=-0.8*dlp*(1+factwrcl);

	//calculating gains
	gkp=(2*zrcl*wrcl+dlp)/dld;
    gkphi=wrcl*wrcl/dld;

	//roll position control
	double pp=WBECB[0];
    double ephi=gkphi*(phicomx-phiblcx)*RAD;
    double dpc=ephi-gkp*pp;
    
	if(fabs(thtblcx)>88){
		//switch to rate control when getting close to vertical
		//roll rate gain to achieve closed loop time constant tp (specified in 'input.asc')
		double kp=(1/tp+dlp)/dld;
		dpcx=kp*pp*DEG;
	}
	else{
		dpcx=dpc*DEG;
	}

	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[519].gets(dpcx);
	//diagnostics
	missile[512].gets(wrcl);
	missile[531].gets(gkp);
	missile[532].gets(gkphi);
}
///////////////////////////////////////////////////////////////////////////////
//Rate controller for pitching and yawing
//Member function of class 'Missile'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded pitch and yaw control
//
//170830 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::control_rate()
{
	//local module-variables
	double zrate(0);
	double grate(0);
	double wnlagr(0);
	double dqcx(0);
	double drcx(0);
	double dqcx_rcs(0);
	double drcx_rcs(0);

	//localizing module-variables
	//input data
	double zetlagr=missile[550].real();
	//input from other modules
	double time=flat6[0].real();
	double dvbe=flat6[236].real();
	double dna=missile[145].real();
	double dnd=missile[146].real();
	double dma=missile[147].real();
	double dmq=missile[148].real();
	double dmd=missile[149].real();
	Matrix WBECB=missile[306].vec();
    double qqcomx=missile[528].real();
    double rrcomx=missile[529].real();
	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    zrate=dna/dvbe-dma*dnd/(dvbe*dmd);
    double aa=dna/dvbe-dmq;
    double bb=-dma-dmq*dna/dvbe;

	//feedback gain of rate loop, given desired close loop 'zetlager'
    double dum1=(aa-2*zetlagr*zetlagr*zrate);
    double dum2=aa*aa-4*zetlagr*zetlagr*bb;
    double radix=dum1*dum1-dum2;
    if(radix<0)radix=0;
	if(fabs(dmd)<SMALL)dmd=SMALL*sign(dmd);
    grate=(-dum1+sqrt(radix))/dmd;

    //diagnostics: natural frequency of closed rate loop
    double dum3=grate*dmd*zrate;
    radix=bb+dum3;
    if(radix<0)radix=0;
    wnlagr=sqrt(radix);

	//commanded pitch and yaw control fins
	double qq=WBECB[1];
	double rr=WBECB[2];
	dqcx=DEG*grate*qq;
    drcx=DEG*grate*rr;

    //output to RCS
	dqcx_rcs=dqcx;
	drcx_rcs=drcx;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[520].gets(dqcx);
	missile[521].gets(drcx);
	missile[526].gets(dqcx_rcs);
	missile[527].gets(drcx_rcs);
	//diagnostics
	missile[560].gets(zrate);
	missile[561].gets(grate);
	missile[562].gets(wnlagr);
}
///////////////////////////////////////////////////////////////////////////////
//Acceleration controller for normal and lateral planes
//Member function of class 'Missile'
//Employs pole placement technique (no matrix inversion required) 
//Feedback signals are: body rate (gyro) and acceleration (accel)
//
// (1) Calculates two feedback and one feed-forward gains
//     based on input of dominant closed loop conjugate complex
//     roots
// (2) Calculates the commanded pitch and control deflections
//
//030612 Created by Peter H Zipfel
//071219 changing gain calculations to pitch and yaw, PZi
//080403 Modified for alpha-beta aero model, PZi
//090731 Closed loop frequency is tracking open loop in pitch and yaw, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::control_accel(double int_step)
{
	//local variables
	double phi(0);
    double wn(0);

	//local module-variables
	Matrix GAINFB(3,1);
	double dqcx(0);
	double drcx(0);

	//localizing module-variables
	//input data
	double wacl=missile[502].real();
	double zacl=missile[505].real();
	double pacl=missile[506].real();
	double alimitx=missile[507].real();
	double gainp=missile[525].real();
	double factwacl=missile[546].real();
	double twcl=missile[565].real();
	double wacl_bias=missile[566].real();
	double pacl_bias=missile[567].real();
	double zacl_bias=missile[568].real();
	double ancomx_test=missile[569].real();
	double alcomx_test=missile[570].real();
	//input from other modules
	double time=flat6[0].real();
	double mach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double dvbe=flat6[236].real();
	double ancomx=missile[402].real();
	double alcomx=missile[403].real();
	double dna=missile[145].real();
	double dnd=missile[146].real();
	double dma=missile[147].real();
	double dmq=missile[148].real();
	double dmd=missile[149].real();
	double realq1=missile[160].real();
	double realq2=missile[161].real();
	double wnq=missile[162].real();
	double dnr=missile[169].real();
	double dyb=missile[170].real();
	double dnb=missile[171].real();
	double wnr=missile[172].real();
	double realr1=missile[175].real();
	double realr2=missile[176].real();
	Matrix FSPCB=missile[334].vec();
	Matrix WBECB=missile[306].vec();
	//state variables
	double yyd=missile[514].real();
	double yy=missile[515].real();
	double zzd=missile[516].real();
	double zz=missile[517].real();
	//-------------------------------------------------------------------------
	//testing acceleration controller without guidance commands
	ancomx+=ancomx_test;
	alcomx+=alcomx_test;

	//structural total acceleration limiter
	double aa=sqrt(alcomx*alcomx+ancomx*ancomx);
	if(aa>alimitx) aa=alimitx;
	if(fabs(ancomx)<SMALL&&fabs(alcomx)<SMALL)
		phi=0;
	else
		phi=atan2(ancomx,alcomx);
	alcomx=aa*cos(phi);
	ancomx=aa*sin(phi);
   
	//feedback gains calculated based on the closed loop pole specifications 'wacl', 'pacl', and 'zacl'
    //'zacl' alway set to optimal damping 0.7
    zacl=0.7*(1+zacl_bias);
	//'wacl' follows the first open loop pole 'realq1'='realr1'
    wacl=fabs(realq1)*(1+wacl_bias);
    //'pacl' follows the second open loop pole 'realq2'='realr2'
    pacl=(fabs(realq2)+35)*(1+pacl_bias);

	//*Pitch loop
    //gain calculation for pitch
	double gainfb3=wacl*wacl*pacl/(dna*dmd);
	double gainfb2=(2*zacl*wacl+pacl+dmq-dna/dvbe)/dmd;
	double gainfb1=(wacl*wacl+2*zacl*wacl*pacl+dma+dmq*dna/dvbe
				-gainfb2*dna*dmd/dvbe)/(dna*dmd)-gainp;

	//pitch loop acceleration controller, pitch control command
	double qq=WBECB[1];
	double fspb3=FSPCB[2];			
	double zzd_new=AGRAV*ancomx+fspb3;
	zz=integrate(zzd_new,zzd,zz,int_step);
	zzd=zzd_new;
	double dqc=-gainfb1*(-fspb3)-gainfb2*qq+gainfb3*zz+gainp*zzd;
	dqcx=dqc*DEG;

	//*Yaw loop
	//gain calculation for yaw
	gainfb3=-wacl*wacl*pacl/(dyb*dnd);
	gainfb2=(2*zacl*wacl+pacl+dnr+dyb/dvbe)/dnd;
	gainfb1=(-wacl*wacl-2*zacl*wacl*pacl+dnb+dnr*dyb/dvbe
				-gainfb2*dyb*dnd/dvbe)/(dyb*dnd)-gainp;

	//diagnostic of yaw-gains 
	GAINFB.build_vec3(gainfb1,gainfb2,gainfb3);

	//yaw loop acceleration controller, yaw control command
	double rr=WBECB[2];
	double fspb2=FSPCB[1];			
	double yyd_new=AGRAV*alcomx-fspb2;
	yy=integrate(yyd_new,yyd,yy,int_step);
	yyd=yyd_new;
	double drc=-gainfb1*fspb2-gainfb2*rr+gainfb3*yy+gainp*yyd;
	drcx=drc*DEG;
	//--------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[514].gets(yyd);
	missile[515].gets(yy);
	missile[516].gets(zzd);
	missile[517].gets(zz);
	//output to other modules
	missile[520].gets(dqcx);
	missile[521].gets(drcx);
	//diagnostics
	missile[502].gets(wacl);
	missile[505].gets(zacl);
	missile[506].gets(pacl);
	missile[524].gets_vec(GAINFB);
}
