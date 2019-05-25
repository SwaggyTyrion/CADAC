///////////////////////////////////////////////////////////////////////////////
//FILE: 'guidance.cpp'
//Contains 'guidance' module of class 'Missile'
//
//011214 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030409 Upgraded to SM Item32, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of guidance module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[400-499]
// 
//011214 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_guidance()
{
	//Definition and initialization of module-variables
	missile[400].init("mguid","int",0,"=0:None, =3:Pro-Nav, =6:Comp Pro-Nav","guidance","data","");
	missile[401].init("gnav",0,"Navigation gain - ND","guidance","data","");
	missile[402].init("ancomx",0,"Normal acceleration command - g's","guidance","out","scrn,plot");
	missile[403].init("alcomx",0,"Lateral acceleration command - g's","guidance","out","scrn,plot");
	missile[404].init("gn",0,"Guidance Gain - m/s","guidance","diag","");
	missile[405].init("mnav","int",0,"=0: Reset, =3:Update","guidance","data","");
	missile[406].init("apny",0,"Pronav acceleration along P2 axis - m/s^2","guidance","diag","plot");
	missile[407].init("apnz",0,"Pronav acceleration along P3 axis - m/s^2","guidance","diag","");
	missile[408].init("adely",0,"Vehicle longit accel correction term along P2 - m/s^2","guidance","diag","plot");
	missile[409].init("adelz",0,"Vehicle longit accel correction term along P3 - m/s^2","guidance","diag","");
	missile[410].init("all",0,"Vehicle lateral accel before limiting - m/s^2","guidance","diag","plot");
	missile[411].init("ann",0,"Vehicle normal accel before limiting - m/s^2","guidance","diag","");
	missile[412].init("epchta",0,"Epoch of target data receipt - sec","guidance","save","");
	missile[440].init("WOELC",0,0,0,"O LOS rate computed from kinematic data - rad/s","guidance","out","");
	missile[443].init("tgoc",0,"Time-to-go, computed - s","guidance","diag","plot");
	missile[444].init("dtbc",0,"Distance-to-target, computed - m","guidance","diag","plot");
	missile[445].init("dvtbc",0,"Closing speed, computed - m/s","guidance","diag","plot");
	missile[446].init("psiobcx",0,"Yaw LOS angle wrt missile - deg","guidance","diag","plot");
	missile[447].init("thtobcx",0,"Pitch LOS angle wrt missile - deg","guidance","diag","plot");
	missile[448].init("UTBLC",0,0,0,"LOS unit vector from extrapolated data - NA","guidance","out","");
	missile[455].init("STELC",0,0,0,"Target location, extrapol onboard missile - m","guidance","diag","");
	missile[458].init("STBLC",0,0,0,"Target wrt Missile position, extrapolated - m","guidance","diag","");
	missile[459].init("STELM",0,0,0,"Stored target position - m","guidance","save","");
	missile[460].init("VTELC",0,0,0,"Stored target velocity - m/s","guidance","save","");
}	
///////////////////////////////////////////////////////////////////////////////
//Guidance module
//Member function of class 'Missile'
// This function performs the following functions:
// (1) mguid= 0: No Guidance
//            3: Midcourse (Pro-Nav based on external information)
//               Set by input or in Module S1 if break lock occurred
//            6: Terminal (Pro-Nav based on LOS rates only)
//               Set in Module S1 when 'mseek=4'
// (2) Receives targeting data ('mnav=3') from third party and
//     extrapolates target position to current time
//
// Target data obtained from seeker module, which downloaded it from 'combus'
//	STEL, VTEL
//
//011214 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::guidance(Packet *combus,int num_vehicles)
{	
	//local module-variables
	Matrix STELC(3,1);
	Matrix STBLC(3,1);

	//localizing module-variables
	//input data 
	int mguid=missile[400].integer();
	int mnav=missile[405].integer();
	//getting saved value
	double epchta=missile[412].real();
	Matrix STELM=missile[459].vec(); 
	Matrix VTELC=missile[460].vec();
	//input from other modules
	double time=flat6[0].real();
	Matrix STEL=missile[2].vec(); 
	Matrix VTEL=missile[3].vec();
	Matrix SBEL=flat6[219].vec();
	//-------------------------------------------------------------------------
    //target tracking data receipt
	if(mnav==3){
		mnav=0;
		epchta=time;
		STELM=STEL;
		VTELC=VTEL;
	}
	//target extrapolation
    double dtime=time-epchta;
	Matrix DUM3=VTELC*dtime;
    STELC=STELM+DUM3;
    STBLC=STELC-SBEL;

	//midcourse guidance using external information
	if(mguid==3) guidance_mid(STBLC,VTELC);

	//terminal guidance 
	if(mguid==6) guidance_term();
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	missile[405].gets(mnav);
	missile[412].gets(epchta);
	missile[459].gets_vec(STELM);
	missile[460].gets_vec(VTELC);
	//diagnostics
	missile[455].gets_vec(STELC);
	missile[458].gets_vec(STBLC);
}	
///////////////////////////////////////////////////////////////////////////////
//Midcouse guidance
//Member function of class 'Missile'
//
// (1) Calculating LOS rate from third-party target info 
// (2) Calculating acceleration command based on pro-nav guidance law
//
//011214 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::guidance_mid(Matrix STBLC,Matrix VTELC)
{
	//local module-variables
	double dtbc(0);
	double psiobcx(0);
	double thtobcx(0);
	double dvtbc(0);
	double tgoc(0);
	double ancomx(0);
	double alcomx(0);
	Matrix WOELC(3,1);

	//localizing module-variables
	//input data
	double gnav=missile[401].real();
	//input from other modules
	Matrix TBL=flat6[120].mat();
	Matrix VBEL=flat6[233].vec();
	//-------------------------------------------------------------------------
	//line of sight kinematics
	dtbc=STBLC.absolute();

	//unit los vector
	Matrix UTBLC=STBLC*(1/dtbc);
	Matrix UTBBC=TBL*UTBLC;

	//LOS angles wrt missile body
	Matrix POLAR=UTBBC.pol_from_cart();
	psiobcx=POLAR.get_loc(1,0)*DEG;
	thtobcx=POLAR.get_loc(2,0)*DEG;

	//relative velocity
	Matrix VTBLC=VTELC-VBEL;

	//closing velocity
	dvtbc=fabs(UTBLC^VTBLC);

	//diagnostic: time-to-go
	tgoc=dtbc/dvtbc;

	//inertial los rates in local coordinates
	WOELC=UTBLC.skew_sym()*VTBLC*(1./dtbc);

	//acceleration command of proportional navigation
	Matrix AAPNB=TBL*WOELC.skew_sym()*UTBLC*gnav*dvtbc;

	//converting to missle conventions (LATAX in g's) 
	ancomx=-AAPNB.get_loc(2,0)/AGRAV;
	alcomx=AAPNB.get_loc(1,0)/AGRAV;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[402].gets(ancomx);
	missile[403].gets(alcomx);
	missile[440].gets_vec(WOELC);
	missile[448].gets_vec(UTBLC);
	//diagnostics
	missile[443].gets(tgoc);
	missile[444].gets(dtbc);
	missile[445].gets(dvtbc);
	missile[446].gets(psiobcx);
	missile[447].gets(thtobcx);
}	
///////////////////////////////////////////////////////////////////////////////
//Terminal guidance
//Member function of class 'Missile'
//
//(1) Calculates acceleration commands based on:
//        (a) LOS rates
//        (b) Velocity decay term
//
//(2) Limits acceleration commands by circular limiter
//
//011219 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::guidance_term()
{
	//local variables
	Matrix GRAVL(3,1);
	double phi(0);

	//local module-variables
	double ancomx(0);
	double alcomx(0);
	double gn(0); 
	double apny(0);
	double apnz(0);
	double adely(0);
	double adelz(0);
	double all(0);
	double ann(0);
	//localizing module-variables
	//input data
	double gnav=missile[401].real();	
	//input from other modules
	double time=flat6[0].real();
	Matrix STEL=missile[2].vec();
	Matrix VTEL=missile[3].vec();
	double thtpb=missile[247].real();
	double psipb=missile[248].real();
	double sigdpy=missile[287].real();
	double sigdpz=missile[288].real();
	Matrix TBL=flat6[120].mat();
	Matrix FSPB=flat6[230].vec();
	double gmax=missile[167].real();
	double trcode=missile[180].real();
	double trcvel=missile[182].real();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	//-------------------------------------------------------------------------
	//computing closing velocity
	Matrix SBTL=SBEL-STEL;
	double dbt=SBTL.absolute();
	double dum=(SBTL)^(VBEL-VTEL);
	double dcvel=fabs(dum/dbt);

	//checking for termination condition
	if(time>3.){
		if(dcvel<trcvel) trcode=1.;
	}
	//computing missile acceleration compensation term
	double fspcb1=FSPB.get_loc(0,0);
	adely=fspcb1*tan(psipb)/AGRAV;
    adelz=fspcb1*tan(thtpb)/(cos(psipb)*AGRAV);

	//gravitational bias term in g's
	GRAVL.build_vec3(0,0,1.);
	Matrix GRAVB=TBL*GRAVL;

	//acceleration commands along body axes
    gn=gnav*dcvel;
    apny=gn*sigdpz/(cos(psipb)*AGRAV);
    apnz=gn*(sigdpz*tan(thtpb)*tan(psipb)+sigdpy/cos(thtpb))/AGRAV;
    all=apny+adely-GRAVB.get_loc(1,0);
    ann=apnz+adelz+GRAVB.get_loc(2,0);

	//limiting acceleration commands by circular limiter
    double aa=sqrt(all*all+ann*ann);
    if(aa>gmax) aa=gmax;
    if((fabs(ann)<SMALL&&fabs(all)<SMALL))
       phi=0.;
    else{
		phi=atan2(ann,all);
	}
    alcomx=aa*cos(phi);
    ancomx=aa*sin(phi);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[402].gets(ancomx);
	missile[403].gets(alcomx);
	//diagnostics
	missile[404].gets(gn);  
	missile[406].gets(apny);
	missile[407].gets(apnz);
	missile[408].gets(adely);
	missile[409].gets(adelz);
	missile[410].gets(all); 
	missile[411].gets(ann); 
}	
