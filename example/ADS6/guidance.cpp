///////////////////////////////////////////////////////////////////////////////
//FILE: 'guidance.cpp'
//Contains 'guidance' module of class 'Missile'
//
//011214 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030409 Upgraded to SM Item32, PZi
//081010 Adopted for GENSIM6, PZi
//170914 Modified for ADS6, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'guidance' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[400-499]
//081010 Adopted for GENSIM6, PZi
//170914 Modified for ADS6, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::def_guidance()
{
	//Definition and initialization of module-variables
	missile[400].init("mguide","int",0,"=|mid|term|, see table","guidance","data","");
	missile[402].init("ancomx",0,"Normal acceleration command - g's","guidance","out","scrn,plot");
	missile[403].init("alcomx",0,"Lateral acceleration command - g's","guidance","out","scrn,plot");
	missile[404].init("gnav_comp",0,"Compensated navigation gain - m/s","guidance","diag","plot");
	missile[405].init("grav_bias",1,"Gravity bias - g's","guidance","data","");
	missile[406].init("apnyx",0,"Pronav acceleration along P2 axis - g's","guidance","diag","");
	missile[407].init("apnzx",0,"Pronav acceleration along P3 axis - g's","guidance","diag","");
	missile[408].init("adely",0,"Vehicle longit accel correction term along P2 - m/s^2","guidance","diag","");
	missile[409].init("adelz",0,"Vehicle longit accel correction term along P3 - m/s^2","guidance","diag","");
	missile[410].init("allx",0,"Vehicle lateral accel before limiting - g's","guidance","diag","plot");
	missile[411].init("annx",0,"Vehicle normal accel before limiting - g's","guidance","diag","plot");
	missile[412].init("epchta",0,"Epoch of target data receipt - sec","guidance","save","");
	missile[413].init("gnav",0,"Navigation gain - ND","guidance","data","");
	missile[414].init("gnd",0,"Derivative of nav gain - 1/sec","guidance","state","");
	missile[415].init("gn",0,"State variable of nav gain - ND","guidance","state","plot");
	missile[416].init("tgnav",0,"Time constant of nav gain delay - sec","guidance","data","");
	missile[417].init("dcvel",0,"Closing speed for comp pro-nav (positive) - m/s","guidance","dia","plot");

	missile[420].init("line_gain",0,"Line guidance gain - 1/s","guidance","data","");
	missile[421].init("nl_gain_fact",0,"Nonlinear gain factor - ND","guidance","data","");
	missile[422].init("decrement",0,"Distance decrement of line guid (63%) - m","guidance","data","");
	missile[423].init("psiflx",0,"Heading line-of-attack angle - deg","guidance","diag","plot");
	missile[424].init("thtflx",0,"Pitch line-of-attack angle - deg","guidance","data","");
	missile[425].init("ip_sltrange",0,"Slant range to IP - m","guidance","dia","plot");
	missile[426].init("nl_gain",0,"Nonlinear gain - ND","guidance","dia","");
	missile[427].init("VBEO",0,0,0,"Vel of missile in LOS coord - m/s","guidance","diag","");
	missile[428].init("VBEF",0,0,0,"Vel of missile in LOA coord - m/s","guidance","diag","");
	missile[429].init("SIBLC",0,0,0,"IP wrt missile in local level coord - m","guidance","out","");
	missile[440].init("WOELC",0,0,0,"O LOS rate computed from kinematic data - rad/s","guidance","out","");
	missile[443].init("tgoc",0,"Time-to-go, computed - s","guidance","diag","");
	missile[444].init("dtbc",0,"Distance-to-target in midcourse - m","guidance","diag","");
	missile[445].init("dvtbc",0,"Closing speed, computed - m/s","guidance","diag","plot");
	missile[446].init("psiobcx",0,"Yaw LOS angle wrt missile - deg","guidance","diag","");
	missile[447].init("thtobcx",0,"Pitch LOS angle wrt missile - deg","guidance","diag","");
	missile[448].init("UTBLC",0,0,0,"LOS unit vector from extrapolated data - NA","guidance","out","");
}	
///////////////////////////////////////////////////////////////////////////////
//'guidance' module
//Member function of class 'Missile'
// This function performs the following functions:
// (1) Guidance options
//
// (1) Guidance options
//     mguide = |mid|term|
// 		  	     mid = 0 no midcourse guidance
//					   2 line guidance to IP
//			           3 Pro-Nav - not used
//				    term = 0 no terminal guidance
//				           6 Compensated pro-nav for IR seeker
//						   7 Pro-nav for RF seeker
//
//070914 Created by Peter H Zipfel
//081010 Adopted for GENSIM6, PZi
//170720 Included RF seeker guidance, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::guidance(Packet *combus,int num_vehicles,int vehicle_slot,double int_step)
{	
	//local variables
	Matrix ACBX(3,1);
	double phi(0);
	Variable *data_t;
	Matrix SIEL1(3,1);
	Matrix SIEL2(3,1);
	Matrix SIEL3(3,1);
	Matrix SIBLC(3,1);
	Matrix SBEL0(3,1);
	Matrix SIBL0(3,1);

	//local module-variables
	int mupdat(0);
	double allx(0);
	double annx(0);
	double ancomx(0);
	double alcomx(0);
	double psiflx(0);

	//localizing module-variables
	//input data 
	int mguide=missile[400].integer();
	double thtflx=missile[424].real();

	//input from other modules
	double launch_time=flat6[9].real();
	double sbel1=flat6[220].real();
	double sbel2=flat6[221].real();
	double sbel3=flat6[223].real();
	double psiblx=flat6[137].real();
	Matrix STEL=missile[2].vec(); 
	double gmax=missile[167].real();
	Matrix VBELC=missile[303].vec();
	Matrix SBELC=missile[304].vec();
	Matrix TBLC=missile[315].mat();
	Matrix STCEL=missile[751].vec();
	Matrix VTCEL=missile[752].vec();
	Matrix SAEL=missile[754].vec();
	Matrix VAEL=missile[755].vec();
	//-------------------------------------------------------------------------
	//decoding guidance flag
    int guid_mid=mguide/10;
    int guid_term=(mguide%10);

	//midcourse guidance
	if(guid_mid>=2)
	{
		//downloading from combus-package 'fire control radar f1' the IP coordinates
		string radar_id="f1";
		for(int i=0;i<num_vehicles;i++)
		{
			string id=combus[i].get_id();
			if (id==radar_id)
			{						
				//downloading IP coordinates from radar packet for missiles m1, m2, m3
				data_t=combus[i].get_data();
				SIEL1=data_t[4].vec();
				SIEL2=data_t[5].vec();
				SIEL3=data_t[6].vec();
			}
			//assigning IPs to missiles
			//!!!assumption: MISSILE6 objects are loaded first in 'input.asc' (usual order)
			//  and there is a one-on-one (missile-target) assignment for intercept
			if(vehicle_slot==0) SIBLC=SIEL1-SBELC;
			if(vehicle_slot==1) SIBLC=SIEL2-SBELC;
			if(vehicle_slot==2) SIBLC=SIEL3-SBELC;
		}
	}	
	//executing midcouse line guidance towards IP point
	if(guid_mid==2){
		//computing line-of-attack angles to IP
		SBEL0.build_vec3(sbel1,sbel2,sbel3);
		if(vehicle_slot==0) SIBL0=SIEL1-SBEL0;
		if(vehicle_slot==1) SIBL0=SIEL2-SBEL0;
		if(vehicle_slot==2) SIBL0=SIEL3-SBEL0;
		Matrix POLAR=SIBL0.pol_from_cart();
		psiflx=POLAR[1]*DEG;

		ACBX=guidance_line(SIBLC,psiflx,thtflx);
	}
	//executing midcouse pro-nav towards fixed IP point
	if(guid_mid==3)
		ACBX=guidance_mid_pronav(SIBLC);

	//terminal compensated pro-nav guidance for IR seeker 
	if(guid_term==6)
		ACBX=guidance_term_comp(int_step);

	//terminal compensated pro-nav guidance for RF seeker 
	if(guid_term==7)
		ACBX=guidance_term_pronav(int_step);

	//latex (lateral acceleration demand)
	allx=ACBX[1];
	annx=-ACBX[2];

	//limiting acceleration commands by circular limiter
    double aa=sqrt(allx*allx+annx*annx);
    if(aa>gmax) aa=gmax;
    if((fabs(annx)<SMALL&&fabs(allx)<SMALL))
       phi=0;
    else{
		phi=atan2(annx,allx);
	}
	//acceleration commands in body axes, output
    alcomx=aa*cos(phi);
    ancomx=aa*sin(phi);

	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[402].gets(ancomx);
	missile[403].gets(alcomx);
	//diagnostics
	missile[410].gets(allx); 
	missile[411].gets(annx);
	missile[423].gets(psiflx);
	missile[429].gets_vec(SIBLC);
}	
///////////////////////////////////////////////////////////////////////////////
//Line Guidance against stationary IP
//
//parameter input
//		SIBLC=IP wrt vehicle coordinates - m
//		psiflx= heading of LOA from north - deg
//		thtflx= elevation of LOA from target horizontal plane (up positive) - deg
//return output:
//		ACBX(3x1)=acceleration demanded by line guidance in body  coord. - g's
//
//171007 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Missile::guidance_line(Matrix SIBLC,double psiflx,double thtflx)
{
	//local variables 
	Matrix ACBX(3,1);
	Matrix TVL(3,3);
	Matrix TBV(3,3);

	//local module-variables
	double nl_gain(0);
	Matrix VBEO(3,1);
	Matrix VBEF(3,1);
	double ip_sltrange(0);

	//localizing module-variables
	//input data
	double line_gain=missile[420].real();
	double nl_gain_fact=missile[421].real();
	double decrement=missile[422].real();
	//input from other modules
	double grav=flat6[55].real();
	Matrix VBELC=missile[303].vec();
	Matrix TBLC=missile[315].mat();
	double thtvlcx=missile[332].real();
	double psivlcx=missile[333].real();
	//-------------------------------------------------------------------------
	//TM of LOA wrt local level axes
	Matrix TFL=mat2tr(psiflx*RAD,thtflx*RAD);

	//building TM of LOS wrt local level axes; also getting range-to-go to IP
	Matrix POLAR=SIBLC.pol_from_cart();
	ip_sltrange=POLAR[0];
	double psiol=POLAR[1];
	double thtol=POLAR[2];
	Matrix TOL=mat2tr(psiol,thtol);

	//converting geographic velocity to LOS and LOA coordinates
	VBEO=TOL*VBELC;
	double vbeo2=VBEO[1];
	double vbeo3=VBEO[2];

	VBEF=TFL*VBELC;
	double vbef2=VBEF[1];
	double vbef3=VBEF[2];

	//nonlinear gain
	nl_gain=nl_gain_fact*(1-exp(-ip_sltrange/decrement));

	//line guidance steering law
	double algv1=grav*sin(thtvlcx*RAD);
	double algv2=line_gain*(-vbeo2+nl_gain*vbef2);
	double algv3=line_gain*(-vbeo3+nl_gain*vbef3)-grav*cos(thtvlcx*RAD);

	//packing accelerations into vector 
	Matrix ALGV(3,1);
	ALGV.build_vec3(algv1,algv2,algv3);

	//TM of velocity vector wrt local level axes
	TVL=mat2tr(psivlcx*RAD,thtvlcx*RAD);
	TBV=TBLC*~TVL;

	//converting acceleration command to body axes
	ACBX=TBV*ALGV*(1/AGRAV);

	//-------------------------------------------------------------------------
	//loading dignostic module-variables
	missile[425].gets(ip_sltrange);
	missile[426].gets(nl_gain);
	missile[427].gets_vec(VBEO);
	missile[428].gets_vec(VBEF);

	return ACBX;
}

///////////////////////////////////////////////////////////////////////////////
//Midcouse pro-nav guidance
//Member function of class 'Missile'
//
// (1) Calculating LOS rate from third-party target info 
// (2) Calculating acceleration command based on pro-nav guidance law
//
//170928 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix Missile::guidance_mid_pronav(Matrix SIBLC)
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
	double gnav=missile[413].real();
	//input from other modules
	Matrix TBLC=missile[315].mat();
	Matrix VBELC=missile[303].vec();
	//-------------------------------------------------------------------------
	//line of sight kinematics
	dtbc=SIBLC.absolute();

	//unit los vector
	Matrix UTBLC=SIBLC*(1/dtbc);
	Matrix UTBBC=TBLC*UTBLC;

	//LOS angles wrt missile body
	Matrix POLAR=UTBBC.pol_from_cart();
	psiobcx=POLAR.get_loc(1,0)*DEG;
	thtobcx=POLAR.get_loc(2,0)*DEG;

	//closing velocity
	dvtbc=fabs(UTBLC^VBELC);

	//diagnostic: time-to-go
	tgoc=dtbc/dvtbc;

	//inertial los rates in local coordinates
	WOELC=UTBLC.skew_sym()*VBELC*(1/dtbc);

	//acceleration command of proportional navigation
	ACBX=TBLC*WOELC.skew_sym()*UTBLC*gnav*dvtbc*(1/AGRAV);
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
// Used for IR sensor with mguide = 6
//
//011219 Created by Peter H Zipfel
//171012 Gain shaping, PZi
///////////////////////////////////////////////////////////////////////////////
Matrix Missile::guidance_term_comp(double int_step)
{
	//local variables
	Matrix GRAVL(3,1);
	double phi(0);
	Matrix ACBX(3,1);

	//local module-variables
	double gnav_comp(0); 
	double apnyx(0);
	double apnzx(0);
	double adely(0);
	double adelz(0);

	//localizing module-variables
	//input data
	double grav_bias=missile[405].real();	
	double gnav=missile[413].real();	
	double tgnav=missile[416].real();	
	//input from other modules
	double launch_time=flat6[9].real();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	Matrix STEL=missile[2].vec();
	Matrix VTEL=missile[3].vec();
	double thtpb=missile[279].real();
	double psipb=missile[280].real();
	double sigdy=missile[291].real();
	double sigdz=missile[292].real();
	Matrix TBLC=missile[315].mat();
	Matrix FSPCB=missile[334].vec();
	//sate variables
	double gnd=missile[414].real();	
	double gn=missile[415].real();	

	//-------------------------------------------------------------------------
	//computing closing velocity
	//!!! this is a shortcut for the passive IR seeker
	// assumes that there is laser range-rate sensor
	Matrix SBTL=SBEL-STEL;
	double dbt=SBTL.absolute();
	double dum=(SBTL)^(VBEL-VTEL);
	double dcvel=dum/dbt;

	//computing missile acceleration compensation term
	double fspcb1=FSPCB.get_loc(0,0);
	adely=fspcb1*tan(psipb)/AGRAV;
    adelz=fspcb1*tan(thtpb)/(cos(psipb)*AGRAV);

	//gravitational bias term in g's
	GRAVL.build_vec3(0,0,grav_bias);
	Matrix GRAVB=TBLC*GRAVL;

	//ramping up nav gain (first order delay determined by time constant 'tgnav')
	if(tgnav){
		double gnd_new=(gnav-gn)/tgnav;
		gn=integrate(gnd_new,gnd,gn,int_step);
		gnd=gnd_new;
	}
	else
		gn=gnav;

	//acceleration commands along body axes
    gnav_comp=-gn*dcvel;
    apnyx=gnav_comp*sigdz/(cos(psipb)*AGRAV);
    apnzx=gnav_comp*(sigdz*tan(thtpb)*tan(psipb)+sigdy/cos(thtpb))/AGRAV;

    double allx=apnyx+adely-GRAVB.get_loc(1,0);
    double annx=apnzx+adelz+GRAVB.get_loc(2,0);

	//acceleration command vector in body coordinates - g's
	ACBX.build_vec3(0,allx,-annx);
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[414].gets(gnd);
	missile[415].gets(gn);  
	//diagnostics
	missile[404].gets(gnav_comp);  
	missile[406].gets(apnyx);
	missile[407].gets(apnzx);
	missile[408].gets(adely);
	missile[409].gets(adelz);
	missile[417].gets(dcvel);

	return ACBX;
}
///////////////////////////////////////////////////////////////////////////////
//Terminal, pro-nav for RF seeker 
//Member function of class 'Missile'
//
// Calculates acceleration commands based on LOS rates and closing velocity
// Used for RF sensor with mguide = 7
//
//170720 Created by Peter H Zipfel
//171012 Gain shaping, PZi
///////////////////////////////////////////////////////////////////////////////
Matrix Missile::guidance_term_pronav(double int_step)
{
	//local variables
	Matrix GRAVL(3,1);
	double phi(0);
	Matrix ACBX(3,1);

	//local module-variables
	double apnyx(0);
	double apnzx(0);
	double gnav_comp(0);

	//localizing module-variables
	//input data
	double grav_bias=missile[405].real();	
	double gnav=missile[413].real();	
	double tgnav=missile[416].real();	
	//input from other modules
	double launch_time=flat6[9].real();
	Matrix TBLC=missile[315].mat();
	double ddab=missile[842].real();
	double psisb=missile[864].real();
	double thtsb=missile[866].real();
	double lamdqb=missile[868].real();
	double lamdrb=missile[869].real();
	//state variables
	double gnd=missile[414].real();	
	double gn=missile[415].real();	
	//-------------------------------------------------------------------------

	//gravitational bias term in g's
	GRAVL.build_vec3(0,0,grav_bias);
	Matrix GRAVB=TBLC*GRAVL;

	//ramping up nav gain (first order delay determined by time constant 'tgnav')
	if(tgnav){
		double gnd_new=(gnav-gn)/tgnav;
		gn=integrate(gnd_new,gnd,gn,int_step);
		gnd=gnd_new;
	}
	else
		gn=gnav;

	//with closing speed
    gnav_comp=-gn*ddab;
	//acceleration commands along body axes
    apnyx=gnav_comp*lamdrb/(cos(psisb)*AGRAV);
    apnzx=gnav_comp*(lamdrb*tan(thtsb)*tan(psisb)+lamdqb/cos(thtsb))/AGRAV;
    double allx=apnyx-GRAVB.get_loc(1,0);
    double annx=apnzx+GRAVB.get_loc(2,0);

	//acceleration command vector in body coordinates - g's
	ACBX.build_vec3(0,allx,-annx);
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[414].gets(gnd);
	missile[415].gets(gn);  
	//diagnostics
	missile[404].gets(gnav_comp);  
	missile[406].gets(apnyx);
	missile[407].gets(apnzx);
	return ACBX;
}
