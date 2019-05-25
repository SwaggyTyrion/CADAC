///////////////////////////////////////////////////////////////////////////////
//FILE: 'guidance.cpp'
//Contains 'guidance' module of class 'Missile'
//
//011214 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030409 Upgraded to SM Item32, PZi
//081010 Adopted for GENSIM6, PZi
//100414 Modified for AGM6, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'guidance' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[400-499]
// (1) Guidance options
//     mguid = |mid|term|
// 		  	    mid = 0 no midcourse guidance	
//					  2 Line Guidance for air-to-ground 
//			          3 Pro-Nav against trageting from data link
//			          4 Pro-Nav against true target
//				    term = 0 no terminal guidance
//				           6 Compensated pro-nav (for IIR gimbaled sensor)
//
// (2) Receives targeting data (when 'mnav=3') via data link from aircraft
//     and extrapolates target position to current time
// 
//070914 Created by Peter H Zipfel
//081010 Adopted for GENSIM6, PZi
//100414 Modified for AGM6, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::def_guidance()
{
	//Definition and initialization of module-variables
	missile[400].init("mguid","int",0,"=|mid|term|, see table","guidance","data","");
	missile[401].init("gnav",0,"Navigation gain - ND","guidance","data","");
	missile[402].init("ancomx",0,"Normal acceleration command - g's","guidance","out","scrn,plot");
	missile[403].init("alcomx",0,"Lateral acceleration command - g's","guidance","out","scrn,plot");
	missile[404].init("gn",0,"Guidance Gain - m/s","guidance","diag","");
	missile[405].init("grav_bias",0,"Grav. bias for midcourse pro-nav (mguid=4) - g's","guidance","diag","");
	missile[406].init("apny",0,"Pronav acceleration along P2 axis - m/s^2","guidance","diag","");
	missile[407].init("apnz",0,"Pronav acceleration along P3 axis - m/s^2","guidance","diag","");
	missile[408].init("adely",0,"Vehicle longit accel correction term along P2 - m/s^2","guidance","diag","");
	missile[409].init("adelz",0,"Vehicle longit accel correction term along P3 - m/s^2","guidance","diag","");
	missile[410].init("all",0,"Vehicle lateral accel before limiting - m/s^2","guidance","diag","plot");
	missile[411].init("ann",0,"Vehicle normal accel before limiting - m/s^2","guidance","diag","plot");
	missile[412].init("epchta",0,"Epoch of target data receipt - sec","guidance","save","");
	missile[415].init("tgo_tgt_acrft",0,"TGO of red target wrt blue aircraft - s","guidance","diag","");

	missile[420].init("line_gain",0,"Line guidance gain - 1/s","guidance","data","");
	missile[421].init("nl_gain_fact",0,"Nonlinear gain factor - ND","guidance","data","");
	missile[422].init("decrement",0,"Distance decrement of line guid (63%) - m","guidance","data","");
	missile[423].init("dtac",0,"Distance target to aircraft, computed - m","guidance","diag","");
	missile[424].init("VBEO",0,0,0,"Vel of missile in LOS coord - m/s","guidance","diag","");
	missile[425].init("VBEF",0,0,0,"Vel of missile in LOA coord - m/s","guidance","diag","");
	missile[426].init("init_guide_line","int",true,"Initialization of line guide","guidance","init","");
	missile[427].init("thtflx",0,"Vertical LOA angle for air-to-ground - deg","guidance","data","");
	missile[428].init("SBTO",0,0,0,"True missile pos normal to LOS - m","guidance","diag","");
	missile[436].init("quad_pos",0,"Quadratic evaluation of position (=0 at blue missile) - m","guidance","data","");
	missile[437].init("quad_vel",0,"Quadratic evaluation of velocity (=0 at blue missile) - m/s","guidance","data","");
	missile[438].init("w1",0,"Position weight of cost function - 1/m^2","guidance","data","");
	missile[439].init("w3",0,"Acceleration weight of cost function - 1/(m/s^2)^2","guidance","data","");
	missile[440].init("WOELC",0,0,0,"O LOS rate computed from kinematic data - rad/s","guidance","out","");
	missile[443].init("tgoc",0,"Time-to-go, computed - s","guidance","diag","");
	missile[444].init("dtbc",0,"Distance-to-target, computed - m","guidance","diag","");
	missile[445].init("dvtbc",0,"Closing speed, computed - m/s","guidance","diag","");
	missile[446].init("psiobcx",0,"Yaw LOS angle wrt missile - deg","guidance","diag","plot");
	missile[447].init("thtobcx",0,"Pitch LOS angle wrt missile - deg","guidance","diag","plot");
	missile[448].init("UTBLC",0,0,0,"LOS unit vector from extrapolated data - NA","guidance","out","");

	missile[455].init("STELC",0,0,0,"Target location, extrapol onboard missile - m","guidance","diag","");
	missile[458].init("STBLC",0,0,0,"Target wrt Missile position, extrapolated - m","guidance","diag","");
	missile[459].init("STELM",0,0,0,"Stored target position - m","guidance","save","");
	missile[460].init("VTELC",0,0,0,"Stored target velocity - m/s","guidance","save","");
}	
///////////////////////////////////////////////////////////////////////////////
//'guidance' module
//Member function of class 'Missile'
// This function performs the following functions:
// (1) Guidance options
//
// (1) Guidance options
//     mguid = |mid|term|
// 		  	    mid = 0 no midcourse guidance	
//					  2 Line Guidance for air-to-ground ('nl_gain_fact' input)
//			          3 Pro-Nav; set by input or in 'sensor' module if break lock occurred
//			          4 Pro-Nav against true target
//				    term = 0 no terminal guidance
//				           6 Compensated pro-nav (for IIR gimbaled sensor)
//
// (2) Receives targeting data (when 'mnav=3') via data link from aircraft
//     and extrapolates target position to current time
//
//070914 Created by Peter H Zipfel
//081010 Adopted for GENSIM6, PZi
//100414 Modified for AGM6, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::guidance(Packet *combus,int num_vehicles)
{	
	//local variables
	Matrix ACBX(3,1);
	double phi(0);
	double tgok(0);

	//local module-variables
	Matrix STELC(3,1);
	Matrix STBLC(3,1);
	int mupdat(0);
	double all(0);
	double ann(0);
	Matrix UTBC(3,1);

	//localizing module-variables
	//input data 
	int mguid=missile[400].integer();
	int init_guide_line=missile[426].integer();
	//getting saved value
	double ancomx=missile[402].real();
	double alcomx=missile[403].real();
	double epchta=missile[412].real();
	double nl_gain_fact=missile[421].real();
	Matrix STELM=missile[459].vec(); 
	Matrix VTELC=missile[460].vec();
	//input from other modules
	double launch_time=flat6[9].real();
	double psiblx=flat6[137].real();
	int tgt_num=missile[1].integer();
	Matrix STEL=missile[2].vec(); 
	double gmax=missile[167].real();
	Matrix VBELC=missile[303].vec();
	Matrix SBELC=missile[304].vec();
	Matrix TBLC=missile[315].mat();
	int mnav=missile[750].integer();
	Matrix STCEL=missile[751].vec();
	Matrix VTCEL=missile[752].vec();
	Matrix SAEL=missile[754].vec();
	Matrix VAEL=missile[755].vec();
	//-------------------------------------------------------------------------
	//decoding guidance flag
    int guid_mid=mguid/10;
    int guid_term=(mguid%10);

    //target tracking data receipt
	if(mnav==3){
		epchta=launch_time;
		STELM=STCEL;
		VTELC=VTCEL;
	}
	//target extrapolation between datalink updates
    double dtime=launch_time-epchta;
	Matrix DUM3=VTELC*dtime;
    STELC=STELM+DUM3;
    STBLC=STELC-SBELC;

	//displacement vector of target wrt aircraft
	Matrix STALC=STELC-SAEL;
	//differential velocity of target wrt aircraft as observed from Earth frame
	Matrix VTAELC=VTELC-VAEL;

	//differential velocity of target wrt blue missile as observed from Earth frame
	Matrix VTBELC=VTELC-VBELC;

	//midcourse Line Guidance for air-to-ground
	if(guid_mid==2)
		ACBX=guidance_mid_line(STALC,STBLC,VBELC,nl_gain_fact);
	
	//midcourse pro-nav guidance against target coordinates from datalink (set 'mnav=3')
	if(guid_mid==3)
		ACBX=guidance_mid_pronav(STBLC,VTELC);

	//midcourse pro-nav guidance against true target (no datalink)
		//However, missile coordinates are corrupted by INS errors
	if(guid_mid==4){
		STBLC=STEL-SBELC;
		ACBX=guidance_mid_pronav(STBLC,VTELC);
	}
	//terminal compensated pro-nav guidance  
	if(guid_term==6)
		ACBX=guidance_term_comp();

	if(mguid>0){
		//latex (lateral acceleration demand)
		all=ACBX[1];
		ann=-ACBX[2];

		//limiting acceleration commands by circular limiter
		double aa=sqrt(all*all+ann*ann);
		if(aa>gmax) aa=gmax;
		if((fabs(ann)<SMALL&&fabs(all)<SMALL))
		   phi=0;
		else{
			phi=atan2(ann,all);
		}
		//acceleration commands in body axes, output
		alcomx=aa*cos(phi);
		ancomx=aa*sin(phi);
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	missile[405].gets(mnav);
	missile[412].gets(epchta);
	missile[421].gets(nl_gain_fact);
	missile[459].gets_vec(STELM);
	missile[460].gets_vec(VTELC);
	missile[426].gets(init_guide_line);
	//output to other modules
	missile[402].gets(ancomx);
	missile[403].gets(alcomx);
	missile[430].gets_vec(UTBC);
	//diagnostics
	missile[410].gets(all); 
	missile[411].gets(ann);
	missile[455].gets_vec(STELC);
	missile[458].gets_vec(STBLC);
}	
///////////////////////////////////////////////////////////////////////////////
//Midcourse Line Guidance
//Member function of class 'Missile'
// Ref: Zipfel, "Modeling and Simulation of Aerospace Vehicle Dynamics", AIAA 2007
//
// Calculates acceleration commands from velocity components normal to LOS
//  biased by the LOA between target and aircraft
//
// For air-to-air both LOA angles are calculated on-line (flag: thtflx==0)
// For air-to-ground the vertical LOA angle 'thtflx' is provided  by 'input.asc' (flag: thtflx!=0)
//
// Return output
//			ACBX = acceleration command in body coord - g's
// Parameter input
//			STALC(3x1) = target wrt aircraft displacement - m 
//			STBLC(3x1) = target wrt vehicle displacement - m 
//			VBELC(3x1) = missile inertial velocity - m/s
//			nl_gain_fact = variable non-linear gain factor
//
//070918 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Missile::guidance_mid_line(Matrix STALC,Matrix STBLC,Matrix VBELC,double nl_gain_fact )								   
{
	//local variables
	double vbeo2(0);
	double vbeo3(0);
	double vbef2(0);
	double vbef3(0);
	Matrix ACVX(3,1);
	Matrix ACBX(3,1);
	Matrix TFL(3,3);
	Matrix TOL(3,3);
	Matrix TVL(3,3);
	Matrix TBV(3,3);

	//local module variables
	double dtac(0);
	double dtbc(0);
	Matrix VBEO(3,1);
	Matrix VBEF(3,1);
	Matrix SBTO(3,1);

	//localizing module-variables
	//input data
	double line_gain=missile[420].real();
	double decrement=missile[422].real();
	double thtflx=missile[427].real();
	//input from other modules
	Matrix SBTL=missile[295].vec();
	double grav=flat6[55].real();
	Matrix TBLC=missile[315].mat();
	double thtvlcx=missile[332].real();
	double psivlcx=missile[333].real();
	//-------------------------------------------------------------------------
	//TM of LOA wrt local level axes
	Matrix POLAR=STALC.pol_from_cart();
	dtac=POLAR[0];
	double az_loa=POLAR[1];
	double el_loa=POLAR[2];

	//for air-to-air (default thtflx==0) TFL is calculated from on-line LOA angles
	if(thtflx==0)
		TFL=mat2tr(az_loa,el_loa);
	// for air-to-ground (thtflx!=0) vertical LOA angle is provided in 'input.asc' 
	else
		TFL=mat2tr(az_loa,thtflx*RAD);

	//TM of LOS wrt local level axes
	POLAR=STBLC.pol_from_cart();
	dtbc=POLAR[0];
	double az_los=POLAR[1];
	double el_los=POLAR[2];
	TOL=mat2tr(az_los,el_los);

	//TM of velocity vector wrt local level axes
	TVL=mat2tr(psivlcx*RAD,thtvlcx*RAD);
	TBV=TBLC*~TVL;

	//converting missile velocity to LOS and LOA coordinates
	VBEO=TOL*VBELC;
	vbeo2=VBEO[1];
	vbeo3=VBEO[2];

	VBEF=TFL*VBELC;
	vbef2=VBEF[1];
	vbef3=VBEF[2];

	//nonlinear gain
	double nl_gain=nl_gain_fact*(1-exp(-dtbc/decrement));

	//line guidance steering law, acceleration in velocity axes, - g's
	double algv1=grav*sin(thtvlcx*RAD)/AGRAV;
	double algv2=line_gain*(-vbeo2+nl_gain*vbef2)/AGRAV;
	double algv3=line_gain*((-vbeo3+nl_gain*vbef3)-grav*cos(thtvlcx*RAD))/AGRAV;
	
	//packing accelerations into vector
	ACVX.assign_loc(0,0,algv1);
	ACVX.assign_loc(1,0,algv2);
	ACVX.assign_loc(2,0,algv3);

	//converting acceleration command to body axes
	ACBX=TBV*ACVX;

	//diagnostic: displacement of the true missile pos. normal to the computed LOS
	SBTO=TOL*SBTL;
	//-------------------------------------------------------------------------
	//diagnostics
	missile[423].gets(dtac);
	missile[444].gets(dtbc);
	missile[424].gets_vec(VBEO);
	missile[425].gets_vec(VBEF);
	missile[428].gets_vec(SBTO);

	return ACBX;
}
///////////////////////////////////////////////////////////////////////////////
//Midcouse pro-nav guidance
//Member function of class 'Missile'
//
// (1) Calculating LOS rate from third-party target info 
// (2) Calculating acceleration command based on pro-nav guidance law
//
// Return output
//			ACBX = acceleration command in body coord - g's
// Parameter input
//			STBLC(3x1) = target wrt vehicle displacement - m 
//			VTELC(3x1) = target inertial velocity - m/s
//
//011214 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix Missile::guidance_mid_pronav(Matrix STBLC,Matrix VTELC)
{
	//local module-variables
	double dtbc(0);
	double psiobcx(0);
	double thtobcx(0);
	double dvtbc(0);
	double tgoc(0);
	Matrix WOELC(3,1);
	Matrix ACBX(3,1);

	//localizing module-variables
	//input data
	double gnav=missile[401].real();
	double grav_bias=missile[405].real();
	//input from other modules
	double grav=flat6[55].real();
	Matrix TBLC=missile[315].mat();
	Matrix VBELC=missile[303].vec();
	//-------------------------------------------------------------------------
	//line of sight kinematics
	dtbc=STBLC.absolute();

	//unit los vector
	Matrix UTBLC=STBLC*(1/dtbc);
	Matrix UTBBC=TBLC*UTBLC;

	//LOS angles wrt missile body
	Matrix POLAR=UTBBC.pol_from_cart();
	psiobcx=POLAR.get_loc(1,0)*DEG;
	thtobcx=POLAR.get_loc(2,0)*DEG;

	//relative velocity
	Matrix VTBLC=VTELC-VBELC;

	//closing velocity
	dvtbc=fabs(UTBLC^VTBLC);

	//diagnostic: time-to-go
	tgoc=dtbc/dvtbc;

	//inertial los rates in local coordinates
	WOELC=UTBLC.skew_sym()*VTBLC*(1/dtbc);

	//gravity bias
	Matrix GRAV_COMP(3,1);
	GRAV_COMP.build_vec3(0,0,grav_bias*grav);

	//acceleration command of proportional navigation
	ACBX=TBLC*((WOELC.skew_sym()*UTBLC*gnav*dvtbc-GRAV_COMP)*(1/AGRAV));
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[440].gets_vec(WOELC);
	missile[448].gets_vec(UTBLC);
	//diagnostics
	missile[443].gets(tgoc);
	missile[444].gets(dtbc);
	missile[445].gets(dvtbc);
	missile[446].gets(psiobcx);
	missile[447].gets(thtobcx);

	return ACBX;
}	
///////////////////////////////////////////////////////////////////////////////
//Terminal, compensated pro-nav guidance
//Member function of class 'Missile'
//
// Calculates acceleration commands based on:
//        (a) LOS rates
//        (b) Velocity decay term
//
// Return output
//			ACBX = acceleration command in body coord - g's
//
//011219 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix Missile::guidance_term_comp()
{
	//local variables
	Matrix GRAVL(3,1);
	double phi(0);
	Matrix ACBX(3,1);

	//local module-variables
	double gn(0); 
	double apny(0);
	double apnz(0);
	double adely(0);
	double adelz(0);

	//localizing module-variables
	//input data
	double gnav=missile[401].real();	
	//input from other modules
	double launch_time=flat6[9].real();
	Matrix STEL=missile[2].vec();
	Matrix VTEL=missile[3].vec();
	double psipb=missile[280].real();
	double thtpb=missile[281].real();
	double sigdpy=missile[291].real();
	double sigdpz=missile[292].real();
	Matrix TBLC=missile[315].mat();
	Matrix FSPCB=missile[334].vec();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	//-------------------------------------------------------------------------
	//computing closing velocity
	Matrix SBTL=SBEL-STEL;
	double dbt=SBTL.absolute();
	double dum=(SBTL)^(VBEL-VTEL);
	double dcvel=fabs(dum/dbt);

	//computing missile acceleration compensation term
	double fspcb1=FSPCB.get_loc(0,0);
	adely=fspcb1*tan(psipb)/AGRAV;
    adelz=fspcb1*tan(thtpb)/(cos(psipb)*AGRAV);

	//gravitational bias term in g's
	GRAVL.build_vec3(0,0,1);
	Matrix GRAVB=TBLC*GRAVL;

	//acceleration commands along body axes
    gn=gnav*dcvel;
    apny=gn*sigdpz/(cos(psipb)*AGRAV);
    apnz=gn*(sigdpz*tan(thtpb)*tan(psipb)+sigdpy/cos(thtpb))/AGRAV;
    double all=apny+adely-GRAVB.get_loc(1,0);
    double ann=apnz+adelz+GRAVB.get_loc(2,0);

	//acceleration command vector in body coordinates - g's
	ACBX.build_vec3(0,all,-ann);
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	missile[404].gets(gn);  
	missile[406].gets(apny);
	missile[407].gets(apnz);
	missile[408].gets(adely);
	missile[409].gets(adelz);

	return ACBX;
}
///////////////////////////////////////////////////////////////////////////////
//Terminal, pro-nav 
//Member function of class 'Missile'
//
// Calculates acceleration commands based on LOS rates
//
// Return output
//			ACBX = acceleration command in body coord - g's
//
//071115 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix Missile::guidance_term_pronav()
{
	//local variables
	Matrix GRAVL(3,1);
	double phi(0);
	Matrix ACBX(3,1);

	//local module-variables
	double gn(0); 
	double apny(0);
	double apnz(0);

	//localizing module-variables
	//input data
	double gnav=missile[401].real();	
	//input from other modules
	double launch_time=flat6[9].real();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	Matrix STEL=missile[2].vec();
	Matrix VTEL=missile[3].vec();
	Matrix TBLC=missile[315].mat();
	double psiob=missile[820].real();
	double thtob=missile[821].real();
	double psidon=missile[868].real();
	double thtdon=missile[869].real();
	//-------------------------------------------------------------------------
	//computing closing velocity (preliminary, must be replaced by realistic code)
	Matrix SBTL=SBEL-STEL;
	double dbt=SBTL.absolute();
	double dum=(SBTL)^(VBEL-VTEL);
	double dcvel=fabs(dum/dbt);

	//gravitational bias term in g's
	GRAVL.build_vec3(0,0,1.);
	Matrix GRAVB=TBLC*GRAVL;

	//acceleration commands along body axes
    gn=gnav*dcvel;
    apny=gn*psidon/(cos(psiob)*AGRAV);
    apnz=gn*(psidon*tan(thtob)*tan(psiob)+thtdon/cos(thtob))/AGRAV;
    double all=apny-GRAVB.get_loc(1,0);
    double ann=apnz+GRAVB.get_loc(2,0);

	//acceleration command vector in body coordinates - g's
	ACBX.build_vec3(0,all,-ann);
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	missile[404].gets(gn);  
	missile[406].gets(apny);
	missile[407].gets(apnz);

	return ACBX;
}
