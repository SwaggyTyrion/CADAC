///////////////////////////////////////////////////////////////////////////////
//FILE: 'control.cpp'
//Contains 'control' module of class 'Missile'
//
//030612 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of control module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[500-599]
//
//     maut= 0 No control
//        >= 1 Roll controller
//         = 2 Rate controller
//         = 3 Acceleration controller
// 
//030612 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_control()
{
	//Definition and initialization of module-variables
	missile[500].init("maut","int",0," =2:Rate; =3:Accel controller","control","data","");
	missile[501].init("mfreeze","int",0,"=0:Unfreeze; =1:Freeze; increment for more","control","data","");
	missile[504].init("wacl",0,"Nat freq of accel close loop complex pole - rad/s","control","data","");
	missile[505].init("zacl",0,"Damping of accel close loop complex pole - ND","control","data","");
	missile[506].init("pacl",0,"Close loop real pole - ND","control","data","");
	missile[507].init("alimit",0,"Total structural acceleration limiter - g's","control","data","");
	missile[508].init("dqlimx",0,"Pitch flap control limiter - deg","control","data","");
	missile[509].init("drlimx",0,"Yaw flap control limiter - deg","control","data","");
	missile[510].init("dplimx",0,"Roll command limiter - deg","control","data","");
	missile[511].init("phicomx",0,"Commanded roll angle - deg","control","data","plot");
	missile[512].init("wrcl",0,"Freq of roll closed loop complex pole - rad/s","control","data","");
	missile[513].init("zrcl",0,"Damping of roll closed loop pole - ND","control","data","");
	missile[514].init("yyd",0,"Yaw feed-forward derivative variable- m/s","control","state","");
	missile[515].init("yy",0,"Yaw feed-forward integration variable- m/s","control","state","");
	missile[516].init("zzd",0,"Pitch feed-forward derivative variable- m/s","control","state","");
	missile[517].init("zz",0,"Pitch feed-forward integration variable- m/s","control","state","");
	missile[519].init("dpcx",0,"Roll flap command deflection - deg","control","out","plot");
	missile[520].init("dqcx",0,"Pitch flap command deflection - deg","control","out","plot");
	missile[521].init("drcx",0,"Yaw flap command deflection - deg","control","out","plot");
	missile[524].init("GAINFB",0,0,0,"Feedback gain of rate, accel and control","control","diag","");
	missile[525].init("gainp",0,"Feed-forward gain - s^2/m","control","data","");
	missile[531].init("gkp",0,"Gain of roll rate feedback - s","control","diag","");
	missile[532].init("gkphi",0,"Gain of roll angle feedback -","control","diag","");
	missile[533].init("fspb2m",0,"Max pitch accel transient - m/s^2","control","diag","");
	missile[534].init("fspb2mt",0,"Stagetime at max pitch accel transient - s","control","diag","");
	missile[535].init("fspb3m",0,"Max yaw accel transient - m/s^2","control","diag","");
	missile[536].init("fspb3mt",0,"Stagetime at max yaw accel transient - s","control","diag","");
	missile[537].init("qqxm",0,"Max pitch rate transient - deg/s","control","diag","");
	missile[538].init("qqxmt",0,"Stagetime at max pitch rate transient - s","control","diag","");
	missile[539].init("rrxm",0,"Max yaw rate transient - deg/s","control","diag","");
	missile[540].init("rrxmt",0,"Stagetime at max yaw rate transient - s","control","diag","");
	missile[541].init("dqcxm",0,"Max pitch flap transient - deg","control","diag","");
	missile[542].init("dqcxmt",0,"Stagetime at max pitch flap transient - s","control","diag","");
	missile[543].init("drcxm",0,"Max yaw flap transient - deg","control","diag","");
	missile[544].init("drcxmt",0,"Stagetime at max yaw flap transient - s","control","diag","");
	missile[545].init("isetc2",0,"Flag to print freeze variables","control","init","");
	missile[546].init("factwacl",0,"Fact assoc with closed loop natural frequency","control","data","");
	missile[547].init("factzacl",0,"Fact assoc with closed loop damping","control","data","");
	missile[550].init("zetlagr",0,"Desired damping of closed rate loop ND","control","data","");
	missile[553].init("ratelimx",0,"Rate command limiter - deg/s","control","data","");
	missile[560].init("zrate",0,"Zero pole of rate TR.FCT. - 1/rad","control","diag","");
	missile[561].init("grate",0,"Feedback gain of rate loop - ND","control","diag","");
	missile[562].init("wnlagr",0,"Nat freq of closed rate loop - rad/s","control","diag","");
}
///////////////////////////////////////////////////////////////////////////////
//'control' module
//Member function of class 'Missile'
// (1) Selects control configurations
//     maut= 0 No control
//        >= 1 Roll controller
//         = 2 Rate controller
//         = 3 Acceleration controller
// (2) Freezes flight conditions for control response.
//     Frozen variables are: dvbe, vmach, pdynmc, thrust and mass parameters
//     Invoked by setting  mfreeze=1 and canceled by mfreeze=0.
//     For sequential maneuvers increment mfreeze by "1"
//
//030612 Created by Peter H Zipfel
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
	//-------------------------------------------------------------------------
}	
///////////////////////////////////////////////////////////////////////////////
//Roll controller
//Member function of class 'Missile'
// (1) Calculates gains
// (2) Calculates commanded roll control
//
//030612 Created by Peter H Zipfel
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
	//input from other modules
	double time=flat6[0].real();
	double phiblx=flat6[139].real();
	double dlp=missile[150].real();
	double dld=missile[151].real();
	double pp=flat6[155].real();
	//-------------------------------------------------------------------------
	//calculating gains
	gkp=(2.*zrcl*wrcl+dlp)/dld;
    gkphi=wrcl*wrcl/dld;

	//roll position control
    double ephi=gkphi*(phicomx-phiblx)*RAD;
    double dpc=ephi-gkp*pp;
    dpcx=dpc*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[519].gets(dpcx);
	//diagnostics
	missile[531].gets(gkp);
	missile[532].gets(gkphi);
}

///////////////////////////////////////////////////////////////////////////////
//Rate controller for pitching and yawing
//Member function of class 'Missile'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded pitch and yaw control
//
//030612 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::control_rate()
{
	//local module-variables
	double zrate(0);
	double grate(0);
	double wnlagr(0);
	double dqcx(0);
	double drcx(0);

	//localizing module-variables
	//input data
	double zetlagr=missile[550].real();
	//input from other modules
	double time=flat6[0].real();
	double qq=flat6[157].real();
	double rr=flat6[159].real();
	double dvbe=flat6[236].real();
	double dna=missile[145].real();
	double dnd=missile[146].real();
	double dma=missile[147].real();
	double dmq=missile[148].real();
	double dmd=missile[149].real();
	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    zrate=dna/dvbe-dma*dnd/(dvbe*dmd);
    double aa=dna/dvbe-dmq;
    double bb=-dma-dmq*dna/dvbe;

	//feecback gain of rate loop, given desired close loop 'zetlager'
    double dum1=(aa-2.*zetlagr*zetlagr*zrate);
    double dum2=aa*aa-4.*zetlagr*zetlagr*bb;
    double radix=dum1*dum1-dum2;
    if(radix<0.)radix=0.;
	if(fabs(dmd)<SMALL)dmd=SMALL*sign(dmd);
    grate=(-dum1+sqrt(radix))/(-dmd);

    //natural frequency of closed rate loop
    double dum3=grate*dmd*zrate;
    radix=bb+dum3;
    if(radix<0.)radix=0.;
    wnlagr=sqrt(radix);

	//commanded pitch and yaw control fins
	dqcx=DEG*grate*qq;
	drcx=DEG*grate*rr;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[520].gets(dqcx);
	missile[521].gets(drcx);
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
///////////////////////////////////////////////////////////////////////////////

void Missile::control_accel(double int_step)
{
	//local variables
	double phi(0);

	//local module-variables
	Matrix GAINFB(3,1);
	double dqcx(0);
	double drcx(0);

	//localizing module-variables
	//input data
	double wacl=missile[504].real();
	double zacl=missile[505].real();
	double pacl=missile[506].real();
	double alimit=missile[507].real();
	double gainp=missile[525].real();
	double factwacl=missile[546].real();
	double factzacl=missile[547].real();
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double ancomx=missile[402].real();
	double alcomx=missile[403].real();
	double dna=missile[145].real();
	double dma=missile[147].real();
	double dmq=missile[148].real();
	double dmd=missile[149].real();
	Matrix FSPB=flat6[230].vec();
	double dvbe=flat6[236].real();
	double qq=flat6[157].real();
	double rr=flat6[159].real();
	//state variables
	double yyd=missile[514].real();
	double yy=missile[515].real();
	double zzd=missile[516].real();
	double zz=missile[517].real();
	//-------------------------------------------------------------------------
	//structural total acceleration limiter
    double aa=sqrt(alcomx*alcomx+ancomx*ancomx);
    if(aa>alimit) aa=alimit;
    if(fabs(ancomx)<SMALL&&fabs(alcomx)<SMALL)
       phi=0.;
    else
       phi=atan2(ancomx,alcomx);
    alcomx=aa*cos(phi);
    ancomx=aa*sin(phi);

	//feedback gains based on natural frequency and damping of conjugate
	//complex poles of desired closed loop response
	//wacl(natural frequency) and zacl(damping) are functions of dynamic pressure
      wacl=(0.013*sqrt(pdynmc)+7.1)*(factwacl+1);
      zacl=(0.559e-3*sqrt(pdynmc)+0.232)*(factzacl+1);
      pacl=14;

	//gain calculation
	double gainfb3=wacl*wacl*pacl/(dna*dmd);
	double gainfb2=(2.*zacl*wacl+pacl+dmq-dna/dvbe)/dmd;
	double gainfb1=(wacl*wacl+2.*zacl*wacl*pacl+dma+dmq*dna/dvbe
				-gainfb2*dmd*dna/dvbe)/(dna*dmd)-gainp;
	GAINFB.build_vec3(gainfb1,gainfb2,gainfb3);

	//pitch loop acceleration controller, pitch control command
	double fspb3=FSPB.get_loc(2,0);			
    double zzd_new=AGRAV*ancomx+fspb3;
	zz=integrate(zzd_new,zzd,zz,int_step);
	zzd=zzd_new;
	double dqc=-gainfb1*(-fspb3)-gainfb2*qq+gainfb3*zz;
    dqcx=dqc*DEG;

	//yaw loop acceleration controller, yaw control command
	double fspb2=FSPB.get_loc(1,0);			
    double yyd_new=AGRAV*alcomx-fspb2;
	yy=integrate(yyd_new,yyd,yy,int_step);
	yyd=yyd_new;
    double drc=-gainfb1*fspb2-gainfb2*rr+gainfb3*yy;
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
	missile[504].gets(wacl);
	missile[505].gets(zacl);
	missile[506].gets(pacl);
	missile[524].gets_vec(GAINFB);
}
