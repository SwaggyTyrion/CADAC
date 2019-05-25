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
//               = 2 yaw rate control - SAS
//               = 3 yaw acceleration control (inludes SAS)
//               = 4 heading angle control (inludes SAS) 
//
//               mautp = 0 no control, fixed control surfaces
//                     = 2 pitch rate control
//                     = 3 pitch acceleration control
//                     = 4 flight path angle control
//					   = 5 altitude hold control (includes pitch accel) 
// Roll control
//		maut > 0 roll control
//   			 mroll= 0 roll position control (default)
//					  = 1 roll rate control
//
// 030520 Created by Peter H Zipfel
// /////////////////////////////////////////////////////////////////////////////
 
void Hyper::def_control()
{
 	//Definition and initialization of module-variables
 	hyper[500].init("maut","int",0,"maut=|mauty|mautp| see table","control","data","");
 	hyper[501].init("mroll","int",0,"=0:Roll postion; =1:Roll rate control","control","data",""); 
 	hyper[503].init("mfreeze","int",0,"=0:Unfreeze; =1:Freeze; increment for more","control","data","");
	hyper[504].init("waclp",0,"Nat freq of accel close loop complex pole - rad/s","control","data","");
	hyper[505].init("zaclp",0,"Damping of accel close loop complex pole - ND","control","data","");
	hyper[506].init("paclp",0,"Close loop real pole - ND","control","data","");
	hyper[507].init("alimitx",0,"Total structural acceleration limiter - g's","control","data","");
	hyper[508].init("dalimx",0,"Aileron limiter - deg","control","data","");
	hyper[509].init("delimx",0,"Elevator limiter - deg","control","data","");
	hyper[510].init("drlimx",0,"Rudder limiter - deg","control","data","");
	hyper[511].init("philimx",0,"Roll angle limiter - deg","control","data","");
	hyper[512].init("wrcl",0,"Freq of roll closed loop complex pole - rad/s","control","data","");
	hyper[513].init("zrcl",0,"Damping of roll closed loop pole - ND","control","data","");
	hyper[514].init("yyd",0,"Yaw feed-foreward derivative variable- m/s","control","state","");
	hyper[515].init("yy",0,"Yaw feed-foreward integration variable- m/s","control","state","");
	hyper[516].init("zzd",0,"Pitch feed-foreward derivative variable- m/s","control","state","");
	hyper[517].init("zz",0,"Pitch feed-foreward integration variable- m/s","control","state","");
	hyper[519].init("delacx",0,"Aileron command deflection - deg","control","out","plot");
	hyper[520].init("delecx",0,"Elevator command deflection - deg","control","out","plot");
	hyper[521].init("delrcx",0,"Rudder  command deflection - deg","control","out","plot");
	hyper[522].init("alcomx",0,"Lateral (horizontal) acceleration comand - g's","control","data","plot");
	hyper[523].init("ancomx",0,"Pitch acceleration command - g's","control","data","plot");
	hyper[524].init("GAINFP",0,0,0,"Feedback gains of pitch accel controller","control","diag","");
	hyper[525].init("gainp",0,"Proportional gain in pitch acceleration loop - s^2/m","control","data","");
	hyper[526].init("gainl",0,"Gain in lateral acceleration loop - rad/g's","control","data","");
	hyper[527].init("altcom",0,"Altitude command - m","control","data","");
	hyper[528].init("gainalt",0,"Altitude gain - 1/s","control","data","");
	hyper[529].init("gainaltrate",0,"Altitude rate gain - 1/s","control","data","");
	hyper[530].init("altrate",0,"Altitude rate - m/s","control","diag","");
	hyper[531].init("gkp",0,"Gain of roll rate feedback - s","control","diag","");
	hyper[532].init("gkphi",0,"Gain of roll angle feedback -","control","diag","");
	hyper[533].init("fspb2m",0,"Max pitch accel transient - m/s^2","control","diag","");
	hyper[534].init("fspb2mt",0,"Stagetime at max pitch accel transient - s","control","diag","");
	hyper[535].init("fspb3m",0,"Max yaw accel transient - m/s^2","control","diag","");
	hyper[536].init("fspb3mt",0,"Stagetime at max yaw accel transient - s","control","diag","");
	hyper[537].init("qqxm",0,"Max pitch rate transient - deg/s","control","diag","");
	hyper[538].init("qqxmt",0,"Stagetime at max pitch rate transient - s","control","diag","");
	hyper[539].init("rrxm",0,"Max yaw rate transient - deg/s","control","diag","");
	hyper[540].init("rrxmt",0,"Stagetime at max yaw rate transient - s","control","diag","");
	hyper[541].init("dqcxm",0,"Max pitch flap transient - deg","control","diag","");
	hyper[542].init("dqcxmt",0,"Stagetime at max pitch flap transient - s","control","diag","");
	hyper[543].init("drcxm",0,"Max yaw flap transient - deg","control","diag","");
	hyper[544].init("drcxmt",0,"Stagetime at max yaw flap transient - s","control","diag","");
	hyper[545].init("isetc2",0,"Flag to print freeze variables","control","init","");
	hyper[546].init("factwacl",0,"Fact assoc with closed loop natural frequency","control","data","");
	hyper[547].init("factzacl",0,"Fact assoc with closed loop damping","control","data","");
	hyper[548].init("tp",0,"Desired closed loop roll rate time constant - s","control","data","");
	hyper[549].init("zetlagr",0,"Desired damping of closed rate loop ND","control","data","");
	hyper[550].init("psivdcomx",0,"Heading command - deg","control","data","");
	hyper[551].init("facthead",0,"Fact to reduce heading gain gainpsi*(1+facthead) - ND","control","data","");
	hyper[552].init("gainpsi",0,"Heading control gain - ND","control","diag","");
	hyper[553].init("phicomx",0,"Roll angle command - deg","control","data","plot");
	hyper[554].init("pcomx",0,"Roll rate command - deg/s","control","data","");
	hyper[555].init("qcomx",0,"Pitch rate command - deg/s","control","data","");
	hyper[556].init("rcomx",0,"Yaw rate command - deg/s","control","data","");
	hyper[557].init("thtvdcomx",0,"Flight path angle command - deg","control","data","");
	hyper[560].init("zrate",0,"Pole of yaw rate TR.FCT. - 1/rad","control","diag","");
	hyper[561].init("grate",0,"Feedback gain of yaw rate loop - ND","control","diag","");
	hyper[562].init("wnlagr",0,"Nat freq of closed yaw rate loop - rad/s","control","diag","");
	hyper[563].init("pgam",0,"Loc of gamma close loop real pole (pos. stable)- rad/s","control","data","");
	hyper[564].init("wgam",0,"Nat freq of gamma close loop complex pole (- rad/s","control","data","");
	hyper[565].init("zgam",0,"Damping of gamma close loop complex pole - rad/s","control","data","");
	hyper[566].init("GAINGAM",0,0,0,"Gamma fbck gain of q, theta and gamma","control","diag","");
	hyper[567].init("gainff",0,"Gamma feed-forward gain","control","diag","");

}
///////////////////////////////////////////////////////////////////////////////
//'control' module
//Member function of class 'Hyper'
//
// maut = |mauty|mautp|
//
//         mauty = 0 no control, fixed control surfaces
//               = 2 yaw rate control - SAS
//               = 3 yaw acceleration control (inludes SAS)
//               = 4 heading angle control (inludes SAS) 
//
//               mautp = 0 no control, fixed control surfaces
//                     = 2 pitch rate control
//                     = 3 pitch acceleration control
//                     = 4 flight path angle control
//					   = 5 altitude hold control (includes pitch accel) 
// Roll control
//	   mroll = 0 roll position control (default)
//			   1 roll rate control
//
// * Freezes flight conditions for control response.
//     Frozen variables are: dvbe, vmach, pdynmc, vmass, thrust.
//     Invoked by setting  mfreeze=1 and canceled by mfreeze=0.
//     For sequential maneuvers increment mfreeze by "one"
//
//030520 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::control(double int_step)
{
	//local module-variables
	double delacx(0),delecx(0),delrcx(0);
	double phi(0);
	
	//localizing module-variables
	//input data
	int maut=hyper[500].integer();
	int mroll=hyper[501].integer();
	double alimitx=hyper[507].real();
	double dalimx=hyper[508].real();
	double delimx=hyper[509].real();
	double drlimx=hyper[510].real();
	double philimx=hyper[511].real();
	double alcomx=hyper[522].real();
	double ancomx=hyper[523].real();
	double altcom=hyper[527].real();
	double psivdcomx=hyper[550].real();
	double phicomx=hyper[553].real();
	double pcomx=hyper[554].real();
	double qcomx=hyper[555].real();
	double rcomx=hyper[556].real();
	double thtvdcomx=hyper[557].real();
	//input from other modules
	double time=round6[0].real();
	double gmax=hyper[163].real();
	double gminx=hyper[167].real();
	//-------------------------------------------------------------------------
	//return if no control
	if(maut==0) return;

	//decoding control flag
    int mauty=maut/10;
    int mautp=(maut%10); 


	//calling yaw and pitch rate stabilizers, assuming zero rate input
	if(mauty==2) 
		 delrcx=control_yaw_rate(rcomx);
	if(mautp==2) 
		 delecx=control_pitch_rate(qcomx);

	//calling acceleration controller in lateral-plane
	if(mauty==3){
		phicomx=control_lateral_accel(alcomx);
		delrcx=control_yaw_rate(rcomx);
	}

	//calling acceleration controller in normal-plane
	if(mautp==3){
		//limiting normal acceleration by max alpha or max structure load
		if(ancomx>gmax) ancomx=gmax;
		if(ancomx<gminx) ancomx=gminx;
		delecx=control_normal_accel(ancomx,int_step);
	}

	//calling vertical flight path angle controller
	if(mautp==4)
		delecx=control_gamma(thtvdcomx);
	//calling heading and SAS(rcomx=0) controllers
	if(mauty==4){
		phicomx=control_heading(psivdcomx);
		delrcx=control_yaw_rate(rcomx);
	}

	//calling altitude control
	if(mautp==5){
		ancomx=control_altitude(altcom);
		//limiting normal acceleration by max alpha or max structure load
		if(ancomx>gmax) ancomx=gmax;
		if(ancomx<gminx) ancomx=gminx;
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
	//output to other modules
	hyper[519].gets(delacx);
	hyper[520].gets(delecx);
	hyper[521].gets(delrcx);
	//diagnostics
	hyper[523].gets(ancomx);
	hyper[553].gets(phicomx);
}	
///////////////////////////////////////////////////////////////////////////////
//Roll controller
//Member function of class 'Hyper'
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

double Hyper::control_roll(double phicomx)
{
	//local module-variables
	double gkp(0);
	double gkphi(0);

	//localizing module-variables
	//input data
	double wrcl=hyper[512].real();
	double zrcl=hyper[513].real();
	//input from other modules
	double phibdcx=hyper[338].real(); 
	double ppcx=hyper[320].real();
	double dllp=hyper[155].real();
	double dllda=hyper[156].real();
	//-------------------------------------------------------------------------
	//calculating gains
	gkp=(2.*zrcl*wrcl+dllp)/dllda;
    gkphi=wrcl*wrcl/dllda;

	//roll position control
    double ephi=gkphi*(phicomx-phibdcx)*RAD;
    double dpc=ephi-gkp*ppcx*RAD;
    double delacx=dpc*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[531].gets(gkp);
	hyper[532].gets(gkphi);

	return delacx;
}
///////////////////////////////////////////////////////////////////////////////
//Roll rate controller 
//Member function of class 'Hyper'
// (1) Calculates roll rate gyro feedback gain
// (2) Calculates commanded roll control based on roll rate input
//Return output
//		delacx = roll control command - deg
//Parameter input
//		pcomx = roll rate command - deg/s
//
//021203 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Hyper::control_roll_rate(double pcomx)
{
	//localizing module-variables
	//input data
	double tp=hyper[548].real();
	//input from other modules
	double dllp=hyper[155].real();
	double dllda=hyper[156].real();
	double ppcx=hyper[320].real();
	//-------------------------------------------------------------------------
	//roll rate gain to achieve closed loop time constant tp
	double kp=(1/tp+dllp)/dllda;

	//commanded roll control fin
	double delacx=kp*(pcomx-ppcx);
	//-------------------------------------------------------------------------
	return delacx;
}
///////////////////////////////////////////////////////////////////////////////
//SAS controller for pitch rate 
//Member function of class 'Hyper'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded pitch control based on zero input
//
//Return output
//		delecx = pitch control command - deg
//Input parameter
//		qcomx = pitch rate command - deg/s
//
//021016 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Hyper::control_pitch_rate(double qcomx)
{
	//localizing module-variables
	//input data
	double zetlagr=hyper[549].real();
	//input from other modules
	double dla=hyper[145].real();
	double dlde=hyper[146].real();
	double dma=hyper[147].real();
	double dmq=hyper[148].real();
	double dmde=hyper[149].real();
	double qqcx=hyper[321].real();
	double dvbec=hyper[330].real();
	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    double zrate=dla/dvbec-dma*dlde/(dvbec*dmde);
    double aa=dla/dvbec-dmq;
    double bb=-dma-dmq*dla/dvbec;

	//feecback gain of rate loop, given desired close loop 'zetlager'
    double dum1=(aa-2.*zetlagr*zetlagr*zrate);
    double dum2=aa*aa-4.*zetlagr*zetlagr*bb;
    double radix=dum1*dum1-dum2;
    if(radix<0.)radix=0.;
	if(fabs(dmde)<SMALL)dmde=SMALL*sign(dmde);
    double grate=(-dum1+sqrt(radix))/(-dmde);

	//commanded pitch control fin
	double delecx=grate*(qqcx-qcomx);
	//-------------------------------------------------------------------------

	return delecx;
}
///////////////////////////////////////////////////////////////////////////////
//SAS controller for yaw rate 
//Member function of class 'Hyper'
// (1) Calculates rate gyro feedback gain
// (2) Calculates commanded yaw control based on zero input
//
//Return output
//		delrcx = yaw control command - deg
//Input parameter
//		rcomx = yaw rate command - deg/s
//
//021016 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Hyper::control_yaw_rate(double rcomx)
{
	//local module-variables
	double zrate(0);
	double grate(0);
	double wnlagr(0);

	//localizing module-variables
	//input data
	double zetlagr=hyper[549].real();
	//input from other modules
	double dyb=hyper[150].real();
	double dydr=hyper[151].real();
	double dnb=hyper[152].real();
	double dnr=hyper[153].real();
	double dndr=hyper[154].real();
	double rrcx=hyper[322].real();
	double dvbec=hyper[330].real();
	//-------------------------------------------------------------------------
	//parameters of open loop angular rate transfer function
    zrate=-dyb/dvbec+dnb*dydr/(dvbec*dndr);
    double aa=-dyb/dvbec-dnr;
    double bb=dnb+dyb*dnr/dvbec;

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
	double delrcx=grate*(rrcx-rcomx);
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[560].gets(zrate);
	hyper[561].gets(grate);
	hyper[562].gets(wnlagr);

	return delrcx;
}
///////////////////////////////////////////////////////////////////////////////
//Acceleration controller in normal (pitch) plane
//Member function of class 'Hyper'
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

double Hyper::control_normal_accel(double ancomx,double int_step)
{
	//local module-variables
	Matrix GAINFP(3,1);

	//localizing module-variables
	//input data
//!	int mfreeze=hyper[503].integer();
	double waclp=hyper[504].real();
	double zaclp=hyper[505].real();
	double paclp=hyper[506].real();
	double gainp=hyper[525].real();
	//input from other modules
	double time=round6[0].real();
	double pdynmc=round6[57].real();
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

	//gain calculation
	double gainfb3=waclp*waclp*paclp/(dla*dmde);
	double gainfb2=(2.*zaclp*waclp+paclp+dmq-dla/dvbec)/dmde;
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

	return delecx;
}
///////////////////////////////////////////////////////////////////////////////
//Converting lateral acceleration command into roll angle command
//Member function of class 'Hyper'
//
//
//Return output
//		phicomx = roll angle command - deg
//Parameter input
//		alcomx = lateral (horizontal) loadfactor command - g's
//		
//030620 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Hyper::control_lateral_accel(double alcomx)
{
	//local variable
	double phicomx(0);
	//localizing module-variables
	//input data
	double gainl=hyper[526].real();
	//input from other modules
	double grav=round6[63].real();
	Matrix FSPCB=hyper[334].vec();
	//-------------------------------------------------------------------------
	double fspb3=FSPCB[2];

	//converting lateral acceleration into roll angle
//	phicomx=-DEG*gainl*alcomx*sign(fspb3)/(fabs(fspb3/grav)+0.001);
	phicomx=-DEG*gainl*alcomx*sign(fspb3);//to supress limit cylces caused by pitch-roll coupling
//	phicomx=-DEG*gainl*atan(alcomx)*sign(fspb3);//mod 031204

	return phicomx;
}
///////////////////////////////////////////////////////////////////////////////
//Flight path angle tracking (gamma-hold) control (mautp=4)
//employs pole placement technique
//feedback signals are: body rate (gyro), body pitch angle (from INS)
// and flight path angle (from INS)
//
//(1) Calculates three feedback and one feed-foreward gain
//    based on input of closed loop conjugate complex pair
//    and one real pole ('wgam', 'zgam', 'pgam').
//(2) Calculates the commanded pitch control deflections
//
//Return output:
//         delecx=Pitch flap command deflection - deg
//Parameter input:
//         thtvdcomx=Flight path angle command - deg
//
//020614 Created by Peter H Zipfel 
///////////////////////////////////////////////////////////////////////////////

double Hyper::control_gamma(double thtvdcomx)
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
	double pgam=hyper[563].real();
	double wgam=hyper[564].real();
	double zgam=hyper[565].real();
	//input from other modules
	double time=round6[0].real();
	double pdynmc=round6[57].real();
	double dvbe=round6[225].real();
	double dla=hyper[145].real();
	double dlde=hyper[146].real();
	double dma=hyper[147].real();
	double dmq=hyper[148].real();
	double dmde=hyper[149].real(); 
	double qqcx=hyper[321].real();
	double dvbec=hyper[330].real();
	double thtvdcx=hyper[332].real();
	double thtbdcx=hyper[339].real();
	//--------------------------------------------------------------------------

	//prevent division by zero
	if(dvbec==0)dvbec=dvbe;
	
	//building fundamental matrices (body rate, acceleration, fin deflection)
	AA.build_mat33(dmq,dma,-dma,1.,0.,0.,0.,dla/dvbec,-dla/dvbec);
	BB.build_vec3(dmde,0.,dlde/dvbec);

	//feedback gains from closed-loop pole placement
	double am=2.*zgam*wgam+pgam;
	double bm=wgam*wgam+2.*zgam*wgam*pgam;
	double cm=wgam*wgam*pgam;
	double v11=dmde;
	double v12=0.;
	double v13=dlde/dvbec;
	double v21=dmde*dla/dvbec-dlde*dma/dvbec;
	double v22=dmde;
	double v23=-dmq*dlde/dvbec;
	double v31=0.;
	double v32=v21;
	double v33=v21;
	DP.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33);
	DD.build_vec3(am+dmq-dla/dvbec,bm+dma+dmq*dla/dvbec,cm);
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
	double thtc=gainff*thtvdcomx*RAD;
	double qqf=GAINGAM.get_loc(0,0)*qqcx*RAD;
	double thtbgf=GAINGAM.get_loc(1,0)*thtbdcx*RAD;
	double thtugf=GAINGAM.get_loc(2,0)*thtvdcx*RAD;
	double delec=thtc-(qqf+thtbgf+thtugf);
	double delecx=delec*DEG;

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[566].gets_vec(GAINGAM);
	hyper[567].gets(gainff);

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

double Hyper::control_heading(double psivdcomx)
{
	//local variable
	double phicomx(0);
	
	//local module-variables
	double gainpsi(0);

	//localizing module-variables
	//input data
	double wrcl=hyper[512].real();
	double zrcl=hyper[513].real();
	double facthead=hyper[551].real();
	//input from other modules
	double grav=round6[63].real();
	double dvbec=hyper[330].real();
	double psivdcx=hyper[333].real();
	//--------------------------------------------------------------------------

	//calculating heading gain
	gainpsi=(dvbec/grav)*zrcl*wrcl*(1.-zrcl*zrcl)*(1.+facthead);

	//roll command
	phicomx=gainpsi*(psivdcomx-psivdcx);

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[552].gets(gainpsi);

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

double Hyper::control_altitude(double altcom)
{
	//local variable
	double ancomx(0);
	
	//local module-variables
	double altrate(0);

	//localizing module-variables
	//input data
	double gainalt=hyper[528].real();
	double gainaltrate=hyper[529].real();
	//input from other modules
	double grav=round6[63].real();
	double altc=hyper[328].real();
	Matrix VBECD=hyper[329].vec();
	double phibdcx=hyper[338].real();
	//--------------------------------------------------------------------------
	//altitude rate feedback
	altrate=-VBECD[2];

	//acceleration command
	double eh=gainalt*(altcom-altc);
	if(phibdcx==0) phibdcx=SMALL;
	ancomx=(1./cos(phibdcx*RAD))*(gainaltrate*(eh-altrate)+grav)/AGRAV;

	//--------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[530].gets(altrate);

	return ancomx;
}





