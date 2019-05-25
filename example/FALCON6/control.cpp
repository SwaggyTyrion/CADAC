///////////////////////////////////////////////////////////////////////////////
//FILE: 'control.cpp'
//Contains 'control' module of class 'Plane'
//
//030731 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of control module-variables 
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[500-599]
// 
// maut= |mauty|mautp|
//
//        mauty= 0 no control, fixed control surfaces
//               2 yaw rate control - SAS
//               3 yaw acceleration control
//               4 heading angle control (inludes SAS) 
//
//              mautp= 0 no control, fixed control surfaces
//                     2 pitch rate control
//                     3 pitch acceleration control
//                     4 flight path angle control
//					   5 altitude hold control (includes pitch accel) 
// Roll control
//		maut > 0 roll control
//   			 mroll= 0 roll position control (default)
//					    1 roll rate control
//
//030731 Created by Peter H Zipfel
// /////////////////////////////////////////////////////////////////////////////
 
void Plane::def_control()
{
 	//Definition and initialization of module-variables
 	plane[500].init("maut","int",0,"maut=|mauty|mautp| see 'control' module ","control","data","");
 	plane[501].init("mroll","int",0,"=0:Roll postion; =1:Roll rate control","control","data",""); 
 	plane[503].init("mfreeze","int",0,"=0:Unfreeze; =1:Freeze; increment for more","control","data","");
	plane[504].init("waclp",0,"Nat freq of accel close loop complex pole - rad/s","control","data","");
	plane[505].init("zaclp",0,"Damping of accel close loop complex pole - ND","control","data","");
	plane[506].init("paclp",0,"Close loop real pole - ND","control","data","");
	plane[508].init("dalimx",0,"Aileron limiter - deg","control","data","");
	plane[509].init("delimx",0,"Elevator limiter - deg","control","data","");
	plane[510].init("drlimx",0,"Rudder limiter - deg","control","data","");
	plane[511].init("philimx",0,"Roll angle limiter - deg","control","data","");
	plane[512].init("wrcl",0,"Freq of roll closed loop complex pole - rad/s","control","data","");
	plane[513].init("zrcl",0,"Damping of roll closed loop pole - ND","control","data","");
	plane[514].init("yyd",0,"Yaw feed-foreward derivative variable- m/s","control","state","");
	plane[515].init("yy",0,"Yaw feed-foreward integration variable- m/s","control","state","");
	plane[516].init("zzd",0,"Pitch feed-foreward derivative variable- m/s","control","state","");
	plane[517].init("zz",0,"Pitch feed-foreward integration variable- m/s","control","state","");
	plane[519].init("delacx",0,"Aileron command deflection - deg","control","out","plot");
	plane[520].init("delecx",0,"Elevator command deflection - deg","control","out","plot");
	plane[521].init("delrcx",0,"Rudder  command deflection - deg","control","out","plot");
	plane[522].init("alcomx",0,"Lateral (horizontal) acceleration comand - g's","control","data","plot");
	plane[523].init("ancomx",0,"Pitch acceleration command - g's","control","data","plot");
	plane[524].init("GAINFP",0,0,0,"Feedback gains of pitch accel controller","control","diag","");
	plane[525].init("gainp",0,"Proportional gain in pitch acceleration loop - s^2/m","control","data","");
	plane[526].init("gainl",0,"Gain in lateral acceleration loop - rad/g's","control","data","");
	plane[527].init("altcom",0,"Altitude command - m","control","data","plot");
	plane[528].init("gainalt",0,"Altitude gain - 1/s","control","data","");
	plane[529].init("gainaltrate",0,"Altitude rate gain - 1/s","control","data","");
	plane[530].init("altrate",0,"Altitude rate - m/s","control","diag","");
	plane[531].init("gkp",0,"Gain of roll rate feedback - s","control","diag","");
	plane[532].init("gkphi",0,"Gain of roll angle feedback -","control","diag","");
	plane[533].init("fspb2m",0,"Max pitch accel transient - m/s^2","control","diag","");
	plane[534].init("fspb2mt",0,"Stagetime at max pitch accel transient - s","control","diag","");
	plane[535].init("fspb3m",0,"Max yaw accel transient - m/s^2","control","diag","");
	plane[536].init("fspb3mt",0,"Stagetime at max yaw accel transient - s","control","diag","");
	plane[537].init("qqxm",0,"Max pitch rate transient - deg/s","control","diag","");
	plane[538].init("qqxmt",0,"Stagetime at max pitch rate transient - s","control","diag","");
	plane[539].init("rrxm",0,"Max yaw rate transient - deg/s","control","diag","");
	plane[540].init("rrxmt",0,"Stagetime at max yaw rate transient - s","control","diag","");
	plane[541].init("dqcxm",0,"Max pitch flap transient - deg","control","diag","");
	plane[542].init("dqcxmt",0,"Stagetime at max pitch flap transient - s","control","diag","");
	plane[543].init("drcxm",0,"Max yaw flap transient - deg","control","diag","");
	plane[544].init("drcxmt",0,"Stagetime at max yaw flap transient - s","control","diag","");
	plane[545].init("isetc2",0,"Flag to print freeze variables","control","init","");
	plane[546].init("factwacl",0,"Fact assoc with closed loop natural frequency","control","data","");
	plane[547].init("factzacl",0,"Fact assoc with closed loop damping","control","data","");
	plane[548].init("tp",0,"Desired closed loop roll rate time constant - s","control","data","");
	plane[549].init("zetlagr",0,"Desired damping of closed rate loop ND","control","data","plot");
	plane[550].init("psivlcomx",0,"Heading command - deg","control","data","");
	plane[551].init("facthead",0,"Fact to reduce heading gain gainpsi*(1.+facthead) - ND","control","data","");
	plane[552].init("gainpsi",0,"Heading control gain - ND","control","diag","");
	plane[553].init("phicomx",0,"Roll angle command - deg","control","data","");
	plane[554].init("pcomx",0,"Roll rate command - deg/s","control","data","");
	plane[555].init("qcomx",0,"Pitch rate command - deg/s","control","data","");
	plane[556].init("rcomx",0,"Yaw rate command - deg/s","control","data","");
	plane[557].init("thtvlcomx",0,"Flight path angle command - deg","control","data","");
	plane[558].init("anlimpx",0,"Positive structural acceleration limiter - g's","control","data","");
	plane[559].init("anlimnx",0,"Neg structural accel limiter (data is positive) - g's","control","data","");
	plane[560].init("zrate",0,"Pole of yaw rate TR.FCT. - 1/rad","control","diag","");
	plane[561].init("grate",0,"Feedback gain of yaw rate loop - ND","control","diag","");
	plane[562].init("wnlagr",0,"Nat freq of closed yaw rate loop - rad/s","control","diag","");
	plane[563].init("pgam",0,"Loc of gamma close loop real pole (pos. stable)- rad/s","control","data","");
	plane[564].init("wgam",0,"Nat freq of gamma close loop complex pole (- rad/s","control","data","");
	plane[565].init("zgam",0,"Damping of gamma close loop complex pole - rad/s","control","data","");
	plane[566].init("GAINGAM",0,0,0,"Gamma fbck gain of q, theta and gamma","control","diag","");
	plane[567].init("gainff",0,"Gamma feed-forward gain","control","diag","");

}
///////////////////////////////////////////////////////////////////////////////
//'control' module
//Member function of class 'Plane'
// (1) maut= |mauty|mautp|
//
//            mauty= 0 no control, fixed control surfaces
//                   2 yaw rate control - SAS
//                   3 yaw acceleration control
//                   4 heading angle control (inludes SAS) 
//
//                  mautp= 0 no control, fixed control surfaces
//                         2 pitch rate control
//                         3 pitch acceleration control
//                         4 flight path angle control
//  		    		   5 altitude hold control (includes pitch accel)
// (2) Roll control
//		mroll= 0 roll position control (default)
//			   1 roll rate control
// (3) Freezes flight conditions for control response.
//     Frozen variables are: dvbe, vmach, pdynmc, vmass, thrust.
//     Invoked by setting  mfreeze=1 and canceled by mfreeze=0.
//     For sequential maneuvers increment mfreeze by "one"
//
//030731 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::control(double int_step)
{
	//local module-variables
	double delacx(0),delecx(0),delrcx(0);
	double phi(0);
	
	//localizing module-variables
	//input data
	int maut=plane[500].integer();
	int mroll=plane[501].integer();
	double dalimx=plane[508].real();
	double delimx=plane[509].real();
	double drlimx=plane[510].real();
	double philimx=plane[511].real();
	double alcomx=plane[522].real();
	double ancomx=plane[523].real();
	double altcom=plane[527].real();
	double psivlcomx=plane[550].real();
	double phicomx=plane[553].real();
	double pcomx=plane[554].real();
	double qcomx=plane[555].real();
	double rcomx=plane[556].real();
	double thtvlcomx=plane[557].real();
	double anlimpx=plane[558].real();
	double anlimnx=plane[559].real();
	//input from other modules
	double time=flat6[0].real();
	double gmax=plane[163].real();
	double gminx=plane[167].real();
	//-------------------------------------------------------------------------
	//return if no control
	if(maut==0) return;

	//decoding control flag
    int mauty=maut/10;
    int mautp=maut%10; 

	//calling yaw and pitch rate stabilizers, assuming zero rate input
	if(mauty==2) 
		 delrcx=control_yaw_rate(rcomx);
	if(mautp==2) 
		 delecx=control_pitch_rate(qcomx);

	//calling acceleration controller in normal- and lateral-plane
	if(mauty==3)
		phicomx=control_lateral_accel(alcomx);

	if(mautp==3){
		//limiting normal acceleration by max alpha or max structure load
		if(ancomx>gmax) ancomx=gmax;
		if(ancomx<gminx) ancomx=gminx;
		delecx=control_normal_accel(ancomx,int_step);
	}

	//calling vertical flight path angle controller
	if(mautp==4)
		delecx=control_gamma(thtvlcomx);
	//calling heading and SAS(rcomx=0) controllers
	if(mauty==4){
		phicomx=control_heading(psivlcomx);
		delrcx=control_yaw_rate(rcomx);
	}

	//calling altitude control
	if(mautp==5){
		ancomx=control_altitude(altcom);
		//limiting normal acceleration by pos and neg structural limiter
		if(ancomx>anlimpx) ancomx=anlimpx;
		if(ancomx<-anlimnx) ancomx=-anlimnx;

		//limiting normal acceleration by max alpha or max structure load
		if(ancomx>gmax)
			ancomx=gmax;
		if(ancomx<gminx)
			ancomx=gminx;
		delecx=control_normal_accel(ancomx,int_step);
	}

	//selecting roll-postion or roll-rate control
	if(mroll==0){
		if(fabs(phicomx)>philimx) phicomx=philimx*sign(phicomx);
		delacx=control_roll(phicomx);
	}
	else if(mroll==1)
		delacx=control_roll_rate(pcomx);

	//limiting control commands
	if(fabs(delacx)>dalimx)delacx=dalimx*sign(delacx);
	if(fabs(delecx)>delimx)delecx=delimx*sign(delecx);
	if(fabs(delrcx)>drlimx)delrcx=drlimx*sign(delrcx);
	//-------------------------------------------------------------------------
	//ouput to other modules
	plane[519].gets(delacx);
	plane[520].gets(delecx);
	plane[521].gets(delrcx);
	//diagnostics
	plane[523].gets(ancomx);
	plane[553].gets(phicomx);
}	
///////////////////////////////////////////////////////////////////////////////
//Roll controller
//Member function of class 'Plane'
//Pole placement technique, specifying closed loop conj complex poles
// (1) Calculates gains
// (2) Calculates commanded roll control
//
//Return output
//		delacx = roll control command - deg
//Parameter input
//		phicomx = roll command - deg
//
//011212 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_roll(double phicomx)
{
	//local module-variables
	double gkp(0);
	double gkphi(0);

	//localizing module-variables
	//input data
	double wrcl=plane[512].real();
	double zrcl=plane[513].real();
	//input from other modules
	double phiblx=flat6[139].real(); 
	double ppx=flat6[160].real();
	double dllp=plane[155].real();
	double dllda=plane[156].real();
	//-------------------------------------------------------------------------
	//calculating gains
	gkp=(2.*zrcl*wrcl+dllp)/dllda;
    gkphi=wrcl*wrcl/dllda;

	//roll position control
    double ephi=gkphi*(phicomx-phiblx)*RAD;
    double dpc=ephi-gkp*ppx*RAD;
    double delacx=dpc*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[531].gets(gkp);
	plane[532].gets(gkphi);

	return delacx;
}
///////////////////////////////////////////////////////////////////////////////
//Roll rate controller 
//Member function of class 'Plane'
// (1) Calculates roll rate gyro feedback gain
// (2) Calculates commanded roll control based on roll rate input
//Return ouput
//		delacx = roll control command - deg
//Parameter input
//		pcomx = roll rate command - deg/s
//
//021203 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_roll_rate(double pcomx)
{
	//localizing module-variables
	//input data
	double tp=plane[548].real();
	//input from other modules
	double ppx=flat6[160].real();
	double dllp=plane[155].real();
	double dllda=plane[156].real();
	//-------------------------------------------------------------------------
	//roll rate gain to achieve closed loop time constant tp
	double kp=(1/tp+dllp)/dllda;

	//commanded roll control fin
	double delacx=kp*(pcomx-ppx);
	//-------------------------------------------------------------------------
	return delacx;
}
///////////////////////////////////////////////////////////////////////////////
//SAS controller for pitch rate 
//Member function of class 'Plane'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded pitch control based on zero input
//
//Return ouput
//		delecx = pitch control command - deg
//Input parameter
//		qcomx = pitch rate command - deg/s
//
//021016 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_pitch_rate(double qcomx)
{
	//localizing module-variables
	//input data
	double zetlagr=plane[549].real();
	//input from other modules
	double dla=plane[145].real();
	double dlde=plane[146].real();
	double dma=plane[147].real();
	double dmq=plane[148].real();
	double dmde=plane[149].real();
	double qqx=flat6[161].real();
	double dvbe=flat6[236].real();
	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    double zrate=dla/dvbe-dma*dlde/(dvbe*dmde);
    double aa=dla/dvbe-dmq;
    double bb=-dma-dmq*dla/dvbe;

	//feecback gain of rate loop, given desired close loop 'zetlager'
    double dum1=(aa-2.*zetlagr*zetlagr*zrate);
    double dum2=aa*aa-4.*zetlagr*zetlagr*bb;
    double radix=dum1*dum1-dum2;
    if(radix<0.)radix=0.;
	if(fabs(dmde)<SMALL)dmde=SMALL*sign(dmde);
    double grate=(-dum1+sqrt(radix))/(-dmde);

	//commanded pitch control fin
	double delecx=grate*(qqx-qcomx);
	//-------------------------------------------------------------------------

	return delecx;
}
///////////////////////////////////////////////////////////////////////////////
//SAS controller for yaw rate 
//Member function of class 'Plane'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded yaw control based on zero input
//
//Return ouput
//		delrcx = yaw control command - deg
//Input parameter
//		rcomx = yaw rate command - deg/s
//
//021016 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_yaw_rate(double rcomx)
{
	//local module-variables
	double zrate(0);
	double grate(0);
	double wnlagr(0);

	//localizing module-variables
	//input data
	double zetlagr=plane[549].real();
	//input from other modules
	double dyb=plane[150].real();
	double dydr=plane[151].real();
	double dnb=plane[152].real();
	double dnr=plane[153].real();
	double dndr=plane[154].real();
	double rrx=flat6[162].real();
	double dvbe=flat6[236].real();
	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    zrate=-dyb/dvbe+dnb*dydr/(dvbe*dndr);
    double aa=-dyb/dvbe-dnr;
    double bb=dnb+dyb*dnr/dvbe;

	//feecback gain of rate loop, given desired close loop 'zetlager'
    double dum1=(aa-2.*zetlagr*zetlagr*zrate);
    double dum2=aa*aa-4.*zetlagr*zetlagr*bb;
    double radix=dum1*dum1-dum2;
    if(radix<0.)radix=0.;
	if(fabs(dndr)<SMALL) dndr=SMALL*sign(dndr);
    grate=(-dum1+sqrt(radix))/(-dndr);

    //natural frequency of closed rate loop
    double dum3=grate*dndr*zrate;
    radix=bb+dum3;
    if(radix<0.)radix=0.;
    wnlagr=sqrt(radix);

	//commanded yaw control fin
	double delrcx=grate*(rrx-rcomx);
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[560].gets(zrate);
	plane[561].gets(grate);
	plane[562].gets(wnlagr);

	return delrcx;
}
///////////////////////////////////////////////////////////////////////////////
//Acceleration controller in normal (pitch) plane
//Member function of class 'Plane'
//Employs pole placement technique (no matrix inversion required) 
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
///////////////////////////////////////////////////////////////////////////////

double Plane::control_normal_accel(double ancomx,double int_step)
{
	//local module-variables
	Matrix GAINFP(3,1);

	//localizing module-variables
	//input data
	double waclp=plane[504].real();
	double zaclp=plane[505].real();
	double paclp=plane[506].real();
	double gainp=plane[525].real();
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double dla=plane[145].real();
	double dma=plane[147].real();
	double dmq=plane[148].real();
	double dmde=plane[149].real();
	double qqx=flat6[161].real();
	Matrix FSPB=flat6[230].vec();
	double dvbe=flat6[236].real();
	//state variables
	double zzd=plane[516].real();
	double zz=plane[517].real();
	//-------------------------------------------------------------------------
	//10km, 0.55M q=5500 Pa: waclp=2, zaclp=0.7,paclp=5
	//1km, 0.6M q=21800 Pa: waclp=4, zaclp=0.3,paclp=10
	waclp=2+0.0001226*(pdynmc-5500);
	zaclp=0.7-0.0000245*(pdynmc-5500);
	paclp=5+0.0003067*(pdynmc-5500);
	
	//gain calculation
	double gainfb3=waclp*waclp*paclp/(dla*dmde);
	double gainfb2=(2.*zaclp*waclp+paclp+dmq-dla/dvbe)/dmde;
	double gainfb1=(waclp*waclp+2.*zaclp*waclp*paclp+dma+dmq*dla/dvbe
				-gainfb2*dmde*dla/dvbe)/(dla*dmde)-gainp;

	//pitch loop acceleration control, pitch control command
	double fspb3=FSPB[2];
    double zzd_new=AGRAV*ancomx+fspb3;
	zz=integrate(zzd_new,zzd,zz,int_step);
	zzd=zzd_new;
	double dqc=-gainfb1*(-fspb3)-gainfb2*qqx*RAD+gainfb3*zz+gainp*zzd;
    double delecx=dqc*DEG;

	//diagnostic output
	GAINFP.build_vec3(gainfb1,gainfb2,gainfb3);
	//--------------------------------------------------------------------------
	//loading module-variables
	//state variables
	plane[516].gets(zzd);
	plane[517].gets(zz);
	//diagnostics
	plane[524].gets_vec(GAINFP);

	return delecx;
}

///////////////////////////////////////////////////////////////////////////////
//Converting lateral acceleration command into roll angle command
//Member function of class 'Plane'
//
//Return output
//		phicomx = roll angle command - deg
//Parameter input
//		alcomx = lateral (horizontal) loadfactor command - g's
//		
//030620 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Plane::control_lateral_accel(double alcomx)
{
	//local variable
	double phicomx(0);

	//localizing module-variables
	//input data
	double gainl=plane[526].real();
	//input from other modules
	double grav=flat6[55].real();
	Matrix FSPB=flat6[230].vec();
	//-------------------------------------------------------------------------
	double fspb3=FSPB[2];

	//converting lateral acceleration into roll angle
	phicomx=-DEG*gainl*atan(alcomx)*sign(fspb3);
	return phicomx;
}
///////////////////////////////////////////////////////////////////////////////
//Flight path angle tracking (gamma-hold) control (mautp=4)
//employs pole placement technique
//feedback signals are: body rate (gyro), body pitch angle 
// and flight path angle 
//
//(1) Calculates three feedback and one feed-foreward gain
//    based on input of closed loop conjugate complex pair
//    and one real pole ('wgam', 'zgam', 'pgam').
//(2) Calculates the commanded pitch control deflections
//
//Return output:
//         delecx=Pitch flap command deflection - deg
//Parameter input:
//         thtvlcomx=Flight path angle command - deg
//
//020614 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double Plane::control_gamma(double thtvlcomx)
{
	//local variables
	Matrix AA(3,3);
	Matrix BB(3,1);
	Matrix DP(3,3);
	Matrix DD(3,1);
	Matrix HH(3,1);

	//local module-variables
	Matrix GAINGAM(3,1);
	double gainff=0;

	//localizing module-variables
	//input data
	double pgam=plane[563].real();
	double wgam=plane[564].real();
	double zgam=plane[565].real();
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double thtblx=flat6[138].real();
	double qqx=flat6[161].real();
	double thtvlx=flat6[241].real();
	double dvbe=flat6[236].real();
	double dla=plane[145].real();
	double dlde=plane[146].real();
	double dma=plane[147].real();
	double dmq=plane[148].real();
	double dmde=plane[149].real(); 
	//--------------------------------------------------------------------------

	//prevent division by zero
	if(dvbe==0)dvbe=dvbe;
	
	//building fundamental matrices (body rate, acceleration, fin deflection)
	AA.build_mat33(dmq,dma,-dma,1.,0.,0.,0.,dla/dvbe,-dla/dvbe);
	BB.build_vec3(dmde,0.,dlde/dvbe);

	//feedback gains from closed-loop pole placement
	double am=2.*zgam*wgam+pgam;
	double bm=wgam*wgam+2.*zgam*wgam*pgam;
	double cm=wgam*wgam*pgam;
	double v11=dmde;
	double v12=0.;
	double v13=dlde/dvbe;
	double v21=dmde*dla/dvbe-dlde*dma/dvbe;
	double v22=dmde;
	double v23=-dmq*dlde/dvbe;
	double v31=0.;
	double v32=v21;
	double v33=v21;
	DP.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33);
	DD.build_vec3(am+dmq-dla/dvbe,bm+dma+dmq*dla/dvbe,cm);
	Matrix DPI=DP.inverse();
	GAINGAM=DPI*DD;

	//steady-state feed-forward gain to achieve unit gamma response
	Matrix DUM33=AA-BB*~GAINGAM;
	Matrix IDUM33=DUM33.inverse();
	Matrix DUM3=IDUM33*BB;
	HH.build_vec3(0.,0.,1.);
	double denom=HH^DUM3;
	gainff=-1./denom;

	//pitch control command
	double thtc=gainff*thtvlcomx*RAD;
	double qqf=GAINGAM[0]*qqx*RAD;
	double thtblf=GAINGAM[1]*thtblx*RAD;
	double thtvlf=GAINGAM[2]*thtvlx*RAD;
	double delec=thtc-(qqf+thtblf+thtvlf);
	double delecx=delec*DEG;

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[566].gets_vec(GAINGAM);
	plane[567].gets(gainff);

	return delecx;
}

///////////////////////////////////////////////////////////////////////////////
// Heading hold control (mauty=4)
// Generates bank command for roll control
// Employs pole placement technique with the condition that the
//  real pole of the closed loop heading transfer function equals
//  the product of natural frequency and damping of the roll transfer function
//
// This subroutine performs the following functions:
// (1) Calculates the heading loop forward gain.
// (2) Outputs roll position command
//
// Return output:
//          phicomx=Roll position command - deg
// Parameter input:
//			psivdocx=Heading command - deg
//
//030522 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double Plane::control_heading(double psivlcomx)
{
	//local variable
	double phicomx(0);
	
	//local module-variables
	double gainpsi(0);

	//localizing module-variables
	//input data
	double wrcl=plane[512].real();
	double zrcl=plane[513].real();
	double facthead=plane[551].real();
	//input from other modules
	double grav=flat6[55].real();
	double dvbe=flat6[236].real();
	double psivlx=flat6[240].real();
	//--------------------------------------------------------------------------

	//calculating heading gain
	gainpsi=(dvbe/grav)*zrcl*wrcl*(1.-zrcl*zrcl)*(1.+facthead);

	//roll command
	phicomx=gainpsi*(psivlcomx-psivlx);

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[552].gets(gainpsi);

	return phicomx;
}
///////////////////////////////////////////////////////////////////////////////
// Altitude hold control (mautp=5)
// Generates accleration command for pitch acceleration autopilot,
//  correcting for bank angle
// MAUTP=5
//
// This subroutine performs the following functions:
// (1) Calculates the pitch acceleration command
//
// Return output
//          ancomx=Pich acceleration command - g's
// Parameter input
//			altcom=Altitude command - m
//
//030522 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double Plane::control_altitude(double altcom)
{
	//local variable
	double ancomx(0);
	
	//local module-variables
	double altrate(0);

	//localizing module-variables
	//input data
	double gainalt=plane[528].real();
	double gainaltrate=plane[529].real();
	//input from other modules
	double grav=flat6[55].real();
	double phiblx=flat6[139].real();
	Matrix VBEL=flat6[233].vec();
	double hbe=flat6[239].real();
	//--------------------------------------------------------------------------
	//altitude rate feedback
	altrate=-VBEL[2];

	//acceleration command
	double eh=gainalt*(altcom-hbe);
	if(phiblx==0) phiblx=SMALL;
	ancomx=(1./cos(phiblx*RAD))*(gainaltrate*(eh-altrate)+grav)/AGRAV;

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	plane[530].gets(altrate);

	return ancomx;
}





