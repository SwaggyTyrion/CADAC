///////////////////////////////////////////////////////////////////////////////
//FILE: 'aerodynamics.cpp'
//
//Contains 'aerodynamics' module of class 'Plane'
//
//030627 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'aerodynamic' module-variables
//Module-variable locations are assigned to flat6[100-199]
//
//* Reference: (1) Stevens & Lewis, "Aircraft Control and
//  Simulation" Wiley-Interscience Publication, 1992.
//* Data based on: (2) Nguyen, J.T., et al., "Simulator Study of
//  Stall/Post-Stall Characteristics of a Fighter Airplane with
//  Relaxec Longitudinal Static Stability", NASA Tech. Paper 1538,
//  NASA, Washington, D.C. Dec. 1979.
//* Data tables valid up to Mach = 0.6 (possibly 0.7 Mach)
//Ref area = 300 ft^2, span = 30 ft, chord = 11.32 ft
//Ref area 'refa' = 27.87 m^2,  span 'refb' = 9.14 m, chord 'refc' = 3.45 m
//Ref c.g. xrcg= O.35*refc= 1.2075 m
//
//This module performs the following functions:
// (2) Sets reference values
// (1) Initializes counters for tables
//
//030627 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::def_aerodynamics()
{
	//Definition and initialization of module-variables
    plane[104].init("refa",0,"Reference area for aero coefficients - m^2","aerodynamics","init","");
    plane[105].init("refb",0,"Reference span for aero coefficients - m","aerodynamics","init","");
    plane[106].init("refc",0,"Reference cord for aero coefficients - m","aerodynamics","init","");
    plane[109].init("cd",0,"Drag coefficient - ND","aerodynamics","diag","");
    plane[110].init("cl",0,"Lift coefficient - ND","aerodynamics","diag","");
	//main force and moment coefficients
    plane[111].init("cxt",0,"X-force coefficient - ND","aerodynamics","out","");
    plane[112].init("cyt",0,"Side force coefficient - ND","aerodynamics","out","");
    plane[113].init("czt",0,"Z-force coefficient - ND","aerodynamics","out","");
    plane[114].init("clt",0,"Rolling moment coefficient - ND","aerodynamics","out","");
    plane[115].init("cmt",0,"Pitching moment coefficient - ND","aerodynamics","out","");
    plane[116].init("cnt",0,"Yawing moment coefficient - ND","aerodynamics","out","");
	//table look-up coefficients
	plane[120].init("cd0",0,"Reference drag coeff(alpha,mach) - ND","aerodynamics","diag","");
	plane[121].init("cda",0,"Delta drag force due to alpha(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[122].init("cl0",0,"Reference lift coeff(alpha,mach) - ND","aerodynamics","diag","");
	plane[123].init("cla",0,"Lift slope derivative(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[124].init("clde",0,"Lift force due to elevator (alpha.mach), - 1/deg","aerodynamics","diag","");
	plane[125].init("cyb",0,"Weather vane der wrt beta(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[126].init("cyda",0,"Side force due to aileron deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[127].init("cydr",0,"Side force due to rudder deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[128].init("cllb",0,"Dutch-roll deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[129].init("cllda",0,"Roll control effectiveness(alpha,mach), - 1/deg","aerodynamics","diag","");
	plane[130].init("cllp",0,"Roll damping deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	plane[131].init("cllr",0,"Roll moment due to yaw rate deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	plane[132].init("cm0",0,"Pitch moment coeff(alpha,mach) - ND","aerodynamics","diag","");
	plane[133].init("cma",0,"Pitch moment due to alpha deriv(alpha,mach) -1/deg","aerodynamics","diag","");
	plane[134].init("cmde",0,"Pitch control effectiveness(alpha,mach), - 1/deg","aerodynamics","diag","");
	plane[135].init("cmq",0,"Pitch dampning deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	plane[136].init("clnb",0,"Yaw moment deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[137].init("clnda",0,"Yaw moment due to aileron deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[138].init("clndr",0,"Yaw moment due to rudder deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	plane[139].init("clnp",0,"Yaw moment due to roll rate deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	plane[140].init("clnr",0,"Yaw damping deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
    plane[141].init("clldr",0,"Roll moment due to rudder - 1/deg","aerodynamics","diag","");
    plane[142].init("clovercd",0,"Lift over drag ratio - ND","aerodynamics","diag","");
	//calculated dimensional derivatives for autopilot
    plane[144].init("stmarg",0,"Static margin (+stable, -unstable) - caliber","aerodynamics","diag","plot");
    plane[145].init("dla",0,"Lift slope derivative - m/s^2","aerodynamics","out","");
    plane[146].init("dlde",0,"Lift elevator control derivative - m/s^2","aerodynamics","out","");
    plane[147].init("dma",0,"Pitch moment derivative - 1/s^2","aerodynamics","out","plot");
    plane[148].init("dmq",0,"Pitch damping derivative - 1/s","aerodynamics","out","");
    plane[149].init("dmde",0,"Pitch control derivative - 1/s^2","aerodynamics","out","plot");
    plane[150].init("dyb",0,"Side force derivative - m/s^2","aerodynamics","out","");
    plane[151].init("dydr",0,"Side force control derivative - m/s^2","aerodynamics","out","");
    plane[152].init("dnb",0,"Yawing moment derivative - 1/s^2","aerodynamics","out","");
    plane[153].init("dnr",0,"Yaw dampnig derivative - 1/s","aerodynamics","out","");
    plane[154].init("dndr",0,"Yaw control derivative - 1/s^2","aerodynamics","out","");
    plane[155].init("dllp",0,"Roll damping derivative - 1/s","aerodynamics","out","");
    plane[156].init("dllda",0,"Roll control derivative - 1/s^2","aerodynamics","out","");
    plane[157].init("cdrag",0,"Drag coefficient - ND","aerodynamics","out","");
    plane[158].init("clift",0,"Drag coefficient - ND","aerodynamics","diag","");
	//maneuver limits
    plane[160].init("alplimpx",0,"Maximum positive alpha permissible - deg","aerodynamics","data","");
    plane[163].init("gmax",0,"Max maneuverability limited by strct_pos_limitx- g's","aerodynamics","out","plot");
    plane[164].init("alplimnx",0,"Minimum neg alpha permissible (with neg sign) - deg","aerodynamics","data","");
    plane[167].init("gminx",0,"Min maneuverability limited by strct_neg_limitx - g's","aerodynamics","out","plot");
	//rigid vehicle dynamic modes
    plane[170].init("realp1",0,"First real root of airframe pitch dyn  - rad/s","aerodynamics","diag","plot");
    plane[171].init("realp2",0,"Second real root of airframe pitch dyn - rad/s","aerodynamics","diag","plot");
    plane[172].init("wnp",0,"Natural frequency of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
    plane[173].init("zetp",0,"Damping of airframe pitch dynamics - NA","aerodynamics","diag","plot");
    plane[174].init("rpreal",0,"Real part or mean value (real roots) of pitch  - rad/s","aerodynamics","diag","");
    plane[175].init("realy1",0,"First real root of airframe yaw dynamics - rad/s","aerodynamics","diag","plot");
    plane[176].init("realy2",0,"Second real root of airframe yaw dynamics - rad/s","aerodynamics","diag","plot");
    plane[177].init("wny",0,"Natural frequency of airframe yaw dynamics - rad/s","aerodynamics","diag","plot");
    plane[178].init("zety",0,"Damping of airframe yaw dynamics - NA","aerodynamics","diag","plot");
    plane[179].init("ryreal",0,"Real part or mean value (real roots) of yaw - rad/s","aerodynamics","diag","");
	//run termination conditions    
    plane[180].init("trcode",0,"Termination code number","aerodynamics","init","");
    plane[181].init("tmcode",0,"Dummy variable initialized to zero","aerodynamics","data","");
    plane[183].init("trmach",0,"Minimum Mach number","aerodynamics","data","");
    plane[184].init("trdynm",0,"Minimum dynamic pressure - Pa","aerodynamics","data","");
    plane[185].init("trload",0,"Minimum load capacity - g's","aerodynamics","data","");
    plane[186].init("tralppx",0,"Maximum positive angle of attack - deg","aerodynamics","data","");
    plane[187].init("tralpnx",0,"Minimum negative angle of attack - deg","aerodynamics","data","");
    plane[188].init("trbetx",0,"Maximum absolute value of sideslip angle - deg","aerodynamics","data","");
    plane[190].init("vmass",0,"Aircraft mass - kg","aerodynamics","init","");
    plane[191].init("IBBB",0,0,0,0,0,0,0,0,0,"Aircraft moment of inertia - kg*m^2","aerodynamics","init","");
    plane[192].init("eng_ang_mom",0,"Engine angular momentum - kg*m^2/sec","aerodynamics","init","");
    plane[193].init("xcg",0,"Actual c.g location - m","aerodynamics","data","");
    plane[194].init("xcgr",0,"Reference c.g location - m","aerodynamics","data","");
}	

///////////////////////////////////////////////////////////////////////////////
//Initialization of aerodynamic module 
//Member function of class 'Plane'
//Ref area 'refa' = 27.87 m^2,  span 'refb' = 9.14 m, chord 'refc' = 3.45 m
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::init_aerodynamics()
{
	//specifying reference values
	double refa(27.87);
	double refb(9.14);
	double refc(3.45);

	//F16 mass parameters (constant)
	double vmass=9496;
	Matrix IBBB(3,3);
	IBBB.assign_loc(0,0,12875);
	IBBB.assign_loc(2,0,-1331.4);
	IBBB.assign_loc(1,1,75673);
	IBBB.assign_loc(0,2,-1331.4);
	IBBB.assign_loc(2,2,85551);

	//engine angular momentum
	double eng_ang_mom(70000);

	//Run termination criteria
	//If any of the termination limits is violated a number code is
	//stored in 'trcond'. If mstop=1 the simulation will stop.
	//If mstop=0 (default) the simulation will continue, and additional
	//code numbers, if any, will enter on the left of 'trcond' until the
	//run is stopped by other means.
	//
	//	   code  term.cond. module				description
	//		 2     trmach     environment	minimum mach number
	//		 3     trdynm     environment	minimum dynamic pressure - Pa
	//		 4     trload     aerodynamics	minimum load factor - g's
	//		 5     tralppx    kinematics   maximum pos angle of attack - deg
	//		 6     tralpnx    kinematics   minimum neg angle of attack - deg
	//		 7     trbetx     kinematics   maximum sidelsip angle - deg

	double trmach(0.8);
	double trdynm(10.e+3);
	double trload(3);
	double tralppx(21);
	double tralpnx(-6);
	double trbetx(5);
	//
	double trcode(0);
	double tmcode(0);

	//loading module-variables
	plane[104].gets(refa);
	plane[105].gets(refb);
	plane[106].gets(refc);

	plane[180].gets(trcode);
	plane[181].gets(tmcode);
	plane[183].gets(trmach);
	plane[184].gets(trdynm);
	plane[185].gets(trload);
	plane[186].gets(tralppx);
	plane[187].gets(tralpnx);
	plane[188].gets(trbetx);
	plane[190].gets(vmass);
	plane[191].gets_mat(IBBB);
	plane[192].gets(eng_ang_mom);
}
///////////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Plane'
//
//This module performs the following functions:
// (1) Aerodynamic table look-up from file 'plane6_aero_deck.asc'
//     Ref area 'refa' = 27.87 m^2,  span 'refb' = 9.14 m, chord 'refc' = 3.45 m
// (2) Calculation of aero coefficients in body coordinates
// Note: Rolling moment from positive aileron deflection is negative
//		in high subsonic regime. This phenomena is caused by wing twist.
//		Therefore, the aircraft uses spoilers in this region. In this sim
//		the sign of 'clda' is reversed since spoiler are not implements
//
//030627 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::aerodynamics()
{
	//local module-variables
	double cd(0);
	double cxt(0);
	double cyt(0);
	double czt(0);
	double clt(0);
	double cmt(0);
	double cnt(0);
	double cd0(0);
	double cda(0);
	double cl0(0);
	double cla(0);
	double clde(0);
	double cyb(0);
	double cyda(0);
	double cydr(0);
	double cllb(0);
	double cllda(0);
	double clldr(0);
	double cllp(0);
	double cllr(0);
	double cm0(0);
	double cmde(0);
	double clnda(0);
	double clndr(0);
	double clnp(0);
	double clnr(0);
	double clift (0);
	double cdrag(0);
	double clovercd(0);
	double gmax(0);
	double gminx(0);

	//localizing module-variables
	//input data
	double alplimpx=plane[160].real();
	double alplimnx=plane[164].real();
	double xcg=plane[193].real();
	double xcgr=plane[194].real();
	//from initialization
	double refa=plane[104].real();
    double refb=plane[105].real();
    double refc=plane[106].real();
	double trcode=plane[180].real();
	double trload=plane[185].real();
	double vmass=plane[190].real();;			
	//input from other modules
	double time=flat6[0].real();
	double alphax=flat6[144].real();
	double betax=flat6[145].real();
	double vmach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double dvba=flat6[75].real();
	double ppx=flat6[160].real();
	double qqx=flat6[161].real();
	double rrx=flat6[162].real();
	double delax=plane[619].real();
	double delex=plane[620].real();
	double delrx=plane[621].real();
	//-------------------------------------------------------------------------
	//common parameters
	double c2v=refc/(2*dvba);
	double b2v=refb/(2*dvba);

	//axial force coefficient
	double cx=aerotable.look_up("cx_vs_elev_alpha",delex,alphax);
	double cxq=aerotable.look_up("cxq_vs_alpha",alphax);
	cxt=cx+c2v*cxq*qqx*RAD;

	//side force coefficient
	double cyr=aerotable.look_up("cyr_vs_alpha",alphax);

	double cyp=aerotable.look_up("cyp_vs_alpha",alphax);
	cyt=-0.02*betax+0.021*delax/20+0.086*delrx/30+b2v*(cyr*rrx*RAD+cyp*ppx*RAD);

	//down force coefficient
	double cz=aerotable.look_up("cz_vs_alpha",alphax);
	double czq=aerotable.look_up("czq_vs_alpha",alphax);
	czt=cz*(1-pow(betax*RAD,2))-0.19*delex/25+c2v*czq*qqx*RAD;

	//rolling moment coefficient
	double cl=aerotable.look_up("cl_vs_beta_alpha",betax,alphax);
	double cldr=aerotable.look_up("cldr_vs_beta_alpha",betax,alphax);
	double clda=aerotable.look_up("clda_vs_beta_alpha",betax,alphax);
	clda=-clda; //see note in header!
	double clr=aerotable.look_up("clr_vs_alpha",alphax);
	double clp=aerotable.look_up("clp_vs_alpha",alphax);
	clt=cl+clda*delax/20+cldr*delrx/30+b2v*(cllr*rrx*RAD+clp*ppx*RAD);

	//pitching moment coefficient
	double cm=aerotable.look_up("cm_vs_elev_alpha",delex,alphax);
	double cmq=aerotable.look_up("cmq_vs_alpha",alphax);
	cmt=cm+c2v*cmq*qqx*RAD+czt*(xcgr-xcg)/refc;

	//yawing moment coefficient
	double cn=aerotable.look_up("cn_vs_beta_alpha",betax,alphax);
	double cnda=aerotable.look_up("cnda_vs_beta_alpha",betax,alphax);
	double cndr=aerotable.look_up("cndr_vs_beta_alpha",betax,alphax);
	double cnr=aerotable.look_up("cnr_vs_alpha",alphax);
	double cnp=aerotable.look_up("cnp_vs_alpha",alphax);
	cnt=cn+cnda*delax/20+cndr*delrx/30-cyt*(xcgr-xcg)/refb
		+b2v*(cnr*rrx*RAD+cnp*ppx*RAD);

	//calculating the positive and negative load factors available
	double czp=aerotable.look_up("cz_vs_alpha",alplimpx);
	double czn=aerotable.look_up("cz_vs_alpha",alplimnx);
	double alpx=-czp*pdynmc*refa;
	double alnx=-czn*pdynmc*refa;
	double weight=vmass*AGRAV;
	gmax=alpx/weight;
	gminx=alnx/weight;

	//diagnostic: lift and drag coefficients
	double cosa=cos(alphax*RAD);
	double sina=sin(alphax*RAD);
	cdrag=-cxt*cosa-czt*sina;
	clift=cxt*sina-czt*cosa;
	clovercd=clift/cdrag;

	//preparing deriv (all moments start with'clxxx'to distinguish from lift'cl')
	clde=0.19/25;	//per deg
	cyb=-0.02*RAD;	//per deg
	cydr=-0.086/30; //per deg
	cmq=cmq;		//per rad/s
	clnr=b2v*cnr;	//per rad/s
	clndr=cndr/30;	//per deg
	cllp=b2v*clp;	//per rad/s
	cllda=clda/20;	//per deg

	//run-termination if max g capability is less than 'trload'
	if(gmax<trload)trcode=4;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	plane[111].gets(cxt);  
	plane[112].gets(cyt);
	plane[113].gets(czt);
	plane[114].gets(clt);	
	plane[115].gets(cmt);
	plane[116].gets(cnt);
	plane[157].gets(cdrag);
	plane[163].gets(gmax);
	plane[167].gets(gminx);
	//diagnostics
	plane[109].gets(cd);
	plane[110].gets(cl);
	plane[120].gets(cd0);
	plane[121].gets(cda);
	plane[122].gets(cl0);
	plane[123].gets(cla);
	plane[124].gets(clde);
	plane[125].gets(cyb);
	plane[126].gets(cyda);
	plane[127].gets(cydr);
	plane[128].gets(cllb);
	plane[129].gets(cllda);
	plane[130].gets(cllp);
	plane[131].gets(cllr);
	plane[132].gets(cm0);
	plane[134].gets(cmde);
	plane[135].gets(cmq);
	plane[137].gets(clnda);
	plane[138].gets(clndr);
	plane[139].gets(clnp);
	plane[140].gets(clnr);
	plane[141].gets(clldr);
	plane[142].gets(clovercd);
	plane[158].gets(clift);

	//-------------------------------------------------------------------------	
	//function call for derivative calculations
	aerodynamics_der();
  }	
///////////////////////////////////////////////////////////////////////////////
//On-line calculation of aerodynamic derivatives for aero-adaptive autopilot 
//Member function of class 'Plane'
//Input/output occurs over module-variables
//
//This subroutine performs the following functions:
// (1) calculates dimensional derivaties (rad, m, sec)
// (2) calculates the airframe's rigid modes
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::aerodynamics_der()
{
	//local module-variables
	double stmarg(0);
	double dla(0);
	double dlde(0);
	double dma(0);
	double dmq(0);
	double dmde(0);
	double dyb(0);
	double dydr(0);
	double dnb(0);
	double dnr(0);
	double dndr(0);
	double dllp(0);
	double dllda(0);
	double realp1(0);
	double realp2(0);
	double wnp(0);
	double zetp(0);
	double rpreal(0);
	double realy1(0);
	double realy2(0);
	double wny(0);
	double zety(0);
	double ryreal(0);
	double cma(0);
	double clnb(0);

	//localizing module-variables
	//from initialization
	double refa=plane[104].real();
    double refb=plane[105].real();
    double refc=plane[106].real();
	double xcg=plane[193].real();
	double xcgr=plane[194].real();
	//from other modules
	double time=flat6[0].real();
	double vmach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double dvba=flat6[75].real();
	double alphax=flat6[144].real();
	double betax=flat6[145].real();
    double vmass= plane[190].real();
	Matrix IBBB=plane[191].mat(); 
	double delex=plane[620].real();
	//from aerodynamics module		
	double cla=plane[123].real(); //cla=-cza interpolate from cz
	double clde=plane[124].real();//clde=0.19
	double cyb=plane[125].real();//const = -0.02*RAD (deg)
	double cydr=plane[127].real();//const = 0.086 (max value for delrx=30deg)
	double cllda=plane[129].real();//available from look-up
	double cllp=plane[130].real();//available from look-up
	double cmde=plane[134].real();//interpolate from cm
	double cmq=plane[135].real();//available from look-up
	double clndr=plane[138].real();//available from look-up
	double clnr=plane[140].real();//available from look-up
	//-------------------------------------------------------------------------
	//interpolating to get remaining nondimensional derivatives
	//lift slope derivative	 (per degree)
	double czp=aerotable.look_up("cz_vs_alpha",alphax+1.5);
	double czn=aerotable.look_up("cz_vs_alpha",alphax-1.5);
	double cza=(czp-czn)/3;
	cla=-cza;  

	//pitching moment due to alpha derivative (per degree)
	double cmp=aerotable.look_up("cm_vs_elev_alpha",delex,alphax+1.5);
	double cmn=aerotable.look_up("cm_vs_elev_alpha",delex,alphax-1.5);
	double dum=(cmp-cmn)/3;
	cma=dum+cza*(xcgr-xcg)/refc;

	//elevator control derivative (per degree)
	cmp=aerotable.look_up("cm_vs_elev_alpha",delex+1.5,alphax);
	cmn=aerotable.look_up("cm_vs_elev_alpha",delex-1.5,alphax);
	cmde=(cmp-cmn)/3;

	//yawing moment due to beta derivative (per degree)
	double cnp=aerotable.look_up("cn_vs_beta_alpha",betax+1.5,alphax);
	double cnn=aerotable.look_up("cn_vs_beta_alpha",betax-1.5,alphax);
	dum=(cnp-cnn)/3;
	clnb=dum-cyb*(xcgr-xcg)/refb;
	
	//dimensional derivatives
	//MOI components
	double ibbb11=IBBB.get_loc(0,0);
	double ibbb22=IBBB.get_loc(1,1);
	double ibbb33=IBBB.get_loc(2,2);

	//Dimensional derivatives for pitch plane (converted to 1/rad where required)
	double duml=(pdynmc*refa/vmass)/RAD;
	dla=duml*cla;
	dlde=duml*clde;
	double dumm=pdynmc*refa*refc/ibbb22;
    dma=dumm*cma/RAD;
    dmq=dumm*(refc/(2.*dvba))*cmq;
    dmde=dumm*cmde/RAD;

	//Dimensional derivatives in lateral plane (converted to 1/rad where required)
    double dumy=pdynmc*refa/vmass;
    dyb=dumy*cyb/RAD;
	dydr=dumy*cydr/RAD;
    double dumn=pdynmc*refa*refb/ibbb33;
    dnb=dumn*clnb/RAD;
    dnr=dumn*(refb/(2.*dvba))*clnr;
    dndr=dumn*clndr/RAD;

	//Dimensional derivatives in roll (converted to 1/rad where required)
	double dumll=pdynmc*refa*refb/ibbb11;
    dllp=dumll*(refb/(2.*dvba))*cllp;
    dllda=dumll*cllda/RAD;

	//static margin (per chord length 'refc')
	if(cla) stmarg=-cma/cla;

	//diagnostics: pitch plane roots		
	double a11=dmq;
	double a12=dma/dla;
	double a21=dla;
	double a22=-dla/dvba;
	
	double arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
	   wnp=0.;
	   zetp=0.;
	   double dum=a11+a22;
	   realp1=(dum+sqrt(arg))/2.;
	   realp2=(dum-sqrt(arg))/2.;
	   rpreal=(realp1+realp2)/2.;
	}
	else
	{
	   realp1=0.;
	   realp2=0.;
	   wnp=sqrt(a11*a22-a12*a21);
	   zetp=-(a11+a22)/(2.*wnp);
	   rpreal=-zetp*wnp;
	}
	
	//diagnostics: yaw plane roots		
	a11=dnr;
	a12=dnb/dyb;
	a21=-dyb;
	a22=dyb/dvba;
	
	arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
	   wny=0.;
	   zety=0.;
	   double dum=a11+a22;
	   realy1=(dum+sqrt(arg))/2.;
	   realy2=(dum-sqrt(arg))/2.;
	   ryreal=(realy1+realy2)/2.;
	}
	else
	{
	   realy1=0.;
	   realy2=0.;
	   wny=sqrt(a11*a22-a12*a21);
	   zety=-(a11+a22)/(2.*wny);
	   ryreal=-zety*wny;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
    plane[145].gets(dla);
    plane[146].gets(dlde);	
    plane[147].gets(dma);	
    plane[148].gets(dmq);	
    plane[149].gets(dmde);	
    plane[150].gets(dyb);	
    plane[151].gets(dydr);	
    plane[152].gets(dnb);	
    plane[153].gets(dnr);	
    plane[154].gets(dndr);	
    plane[155].gets(dllp);	
    plane[156].gets(dllda);	
	//diagnostics
	plane[133].gets(cma);
	plane[136].gets(clnb);
	plane[144].gets(stmarg);
    plane[170].gets(realp1);
    plane[171].gets(realp2);
    plane[172].gets(wnp);
    plane[173].gets(zetp);
    plane[174].gets(rpreal);
    plane[175].gets(realy1);
    plane[176].gets(realy2);
    plane[177].gets(wny);
    plane[178].gets(zety);
    plane[179].gets(ryreal);

  }
