///////////////////////////////////////////////////////////////////////////////
//FILE: 'aerodynamics.cpp'
//
//Contains 'aerodynamics' module of class 'Hyper'
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of aerodynamic module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[100-199]
// Ref: "Handbook of Intelligent Control", Chapter 11 "Flight
//       Propulsion, and Thermal Control of Advanced Aircraft and
//       Hypersonic Vehicles", Edited by D.A.White and D.A. Sofge,
//       Van Nostrand Reinhold,New York, NY, 1992
//
// This module performs the following functions:
// (1) Provides the aerodynamic tables of hypersonic vehicle GHAME
//     Ref area refa= 557.42 m^2
//     Ref lengths refb= 24.38m, refc= 22.86 m
//     Moment coefficients referenced to fixed c.g.
// (2) Provides an axial force coefficient for the Space Tug
// (3) Defines the run stopping conditions
//
// maero = 0 no aerodynamic forces
//		   1 GHAME aerodynamic forces and moments
//	       2 axial force coeff for the transfer vehicle
//
//030507 Created by Peter H Zipfel
//040301 Added transfer vehicle, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_aerodynamics()
{
	//Definition and initialization of module-variables
    hyper[100].init("maero","int",0,"=0: no aero; =1:GHAME; =2:transfer vehicle","aerodynamics","data","");
    hyper[104].init("refa",0,"Reference area for aero coefficients - m^2","aerodynamics","init","");
    hyper[105].init("refb",0,"Reference span for aero coefficients - m","aerodynamics","init","");
    hyper[106].init("refc",0,"Reference cord for aero coefficients - m","aerodynamics","init","");
	//main force and moment coefficients
    hyper[110].init("cd",0,"Drag  coefficient - ND","aerodynamics","dia","");
    hyper[111].init("cl",0,"Lift coefficient - ND","aerodynamics","dia","");
    hyper[112].init("cy",0,"Side force coefficient - ND","aerodynamics","out","");
    hyper[113].init("cll",0,"Rolling moment coefficient - ND","aerodynamics","out","");
    hyper[114].init("clm",0,"Pitching moment coefficient - ND","aerodynamics","out","");
    hyper[115].init("cln",0,"Yawing moment coefficient - ND","aerodynamics","out","");
    hyper[116].init("cx",0,"X-force coefficient - ND","aerodynamics","out","");
    hyper[117].init("cz",0,"Z-force coefficient - ND","aerodynamics","out","");
	//table look-up coefficients
	hyper[120].init("cd0",0,"Reference drag coeff(alpha,mach) - ND","aerodynamics","diag","");
	hyper[121].init("cda",0,"Delta drag force due to alpha(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[122].init("cl0",0,"Reference lift coeff(alpha,mach) - ND","aerodynamics","diag","");
	hyper[123].init("cla",0,"Lift slope derivative(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[124].init("clde",0,"Lift force due to elevator (alpha.mach), - 1/deg","aerodynamics","diag","");
	hyper[125].init("cyb",0,"Weather vane der wrt beta(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[126].init("cyda",0,"Side force due to aileron deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[127].init("cydr",0,"Side force due to rudder deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[128].init("cllb",0,"Dutch-roll deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[129].init("cllda",0,"Roll control effectiveness(alpha,mach), - 1/deg","aerodynamics","diag","");
	hyper[130].init("cllp",0,"Roll damping deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	hyper[131].init("cllr",0,"Roll moment due to yaw rate deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	hyper[132].init("cm0",0,"Pitch moment coeff(alpha,mach) - ND","aerodynamics","diag","");
	hyper[133].init("cma",0,"Pitch moment due to alpha deriv(alpha,mach) -1/deg","aerodynamics","diag","");
	hyper[134].init("cmde",0,"Pitch control effectiveness(alpha,mach), - 1/deg","aerodynamics","diag","");
	hyper[135].init("cmq",0,"Pitch dampning deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	hyper[136].init("clnb",0,"Yaw moment deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[137].init("clnda",0,"Yaw moment due to aileron deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[138].init("clndr",0,"Yaw moment due to rudder deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[139].init("clnp",0,"Yaw moment due to roll rate deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	hyper[140].init("clnr",0,"Yaw damping deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
    hyper[141].init("clldr",0,"Roll moment due to rudder - 1/deg","aerodynamics","diag","");
    hyper[142].init("clovercd",0,"Lift over drag ratio - ND","aerodynamics","diag","");
	//calculated dimensional derivatives for autopilot
    hyper[144].init("stmarg",0,"Static margin (+stable, -unstable) - caliber","aerodynamics","diag","plot");
    hyper[145].init("dla",0,"Lift slope derivative - m/s^2","aerodynamics","out","");
    hyper[146].init("dlde",0,"Lift elevator control derivative - m/s^2","aerodynamics","out","");
    hyper[147].init("dma",0,"Pitch moment derivative - 1/s^2","aerodynamics","out","");
    hyper[148].init("dmq",0,"Pitch damping derivative - 1/s","aerodynamics","out","");
    hyper[149].init("dmde",0,"Pitch control derivative - 1/s^2","aerodynamics","out","");
    hyper[150].init("dyb",0,"Side force derivative - m/s^2","aerodynamics","out","");
    hyper[151].init("dydr",0,"Side force control derivative - m/s^2","aerodynamics","out","");
    hyper[152].init("dnb",0,"Yawing moment derivative - 1/s^2","aerodynamics","out","");
    hyper[153].init("dnr",0,"Yaw dampnig derivative - 1/s","aerodynamics","out","");
    hyper[154].init("dndr",0,"Yaw control derivative - 1/s^2","aerodynamics","out","");
    hyper[155].init("dllp",0,"Roll damping derivative - 1/s","aerodynamics","out","");
    hyper[156].init("dllda",0,"Roll control derivative - 1/s^2","aerodynamics","out","");
	//maneuver limits
    hyper[160].init("alpplimx",0,"Maximum positive alpha permissible - deg","aerodynamics","data","");
    hyper[161].init("strct_pos_limitx",0,"Pos structural limiter - g's","aerodynamics","data","");
    hyper[162].init("gavail_pos",0,"Positive maneuver headroom in pitch - g's","aerodynamics","diag","plot");
    hyper[163].init("gmax",0,"Max maneuverability limited by strct_pos_limitx- g's","aerodynamics","out","plot");
    hyper[164].init("alpnlimx",0,"Minimum neg alpha permissible (with neg sign) - deg","aerodynamics","data","");
    hyper[165].init("strct_neg_limitx",0,"Neg structural limiter (with neg sign) - g's","aerodynamics","data","");
    hyper[166].init("gavail_neg",0,"Nagative manuever  headroom - g's","aerodynamics","diag","plot");
    hyper[167].init("gminx",0,"Min maneuverability limited by strct_neg_limitx - g's","aerodynamics","out","plot");
	//rigid vehicle dynamic modes
    hyper[170].init("realp1",0,"First real root of airframe pitch dyn  - rad/s","aerodynamics","diag","");
    hyper[171].init("realp2",0,"Second real root of airframe pitch dyn - rad/s","aerodynamics","diag","");
    hyper[172].init("wnp",0,"Natural frequency of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
    hyper[173].init("zetp",0,"Damping of airframe pitch dynamics - NA","aerodynamics","diag","");
    hyper[174].init("rpreal",0,"Real part or mean value (real roots) of pitch  - rad/s","aerodynamics","diag","");
    hyper[175].init("realy1",0,"First real root of airframe yaw dynamics - rad/s","aerodynamics","diag","");
    hyper[176].init("realy2",0,"Second real root of airframe yaw dynamics - rad/s","aerodynamics","diag","");
    hyper[177].init("wny",0,"Natural frequency of airframe yaw dynamics - rad/s","aerodynamics","diag","");
    hyper[178].init("zety",0,"Damping of airframe yaw dynamics - NA","aerodynamics","diag","");
    hyper[179].init("ryreal",0,"Real part or mean value (real roots) of yaw - rad/s","aerodynamics","diag","");
	//run termination conditions    
    hyper[180].init("trcode",0,"Termination code number","aerodynamics","init","");
    hyper[181].init("tmcode",0,"Dummy variable initialized to zero","aerodynamics","data","");
    hyper[183].init("trmach",0,"Minimum Mach number","aerodynamics","data","");
    hyper[184].init("trdynm",0,"Minimum dynamic pressure - Pa","aerodynamics","data","");
    hyper[185].init("trload",0,"Minimum load capacity - g's","aerodynamics","data","");
    hyper[186].init("tralp",0,"Maximum total angle of attack - rad","aerodynamics","data","");
	//transfer vehicle
    hyper[190].init("refa_st",0,"Cross sectional area of transfer vehicle - m^2","aerodynamics","init","");
    hyper[191].init("caa",0,"Axial force coefficient of transfer vehicle - ND","aerodynamics","init","");
}	

///////////////////////////////////////////////////////////////////////////////
//Initialization of aerodynamic module 
//Member function of class 'Hyper'
//Run stopping conditions
//     Ref Area refa= 557.42 m^2
//     Ref Lengths refb= 24.38 m, refc= 22.86 m
//     Moment coefficients referenced to fixed c.g.
//
// maero = 0 no aerodynamic forces
//		   1 GHAME aerodynamic forces and moments
//	       2 axial force coeff for the transfer vehicle
//
//030507 Created by Peter H Zipfel
//040301 Added transfer vehicle, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::init_aerodynamics()
{
	//local module variables
	double refa(0);
	double refb(0);
	double refc(0);
	double trmach(0);
	double trdynm(0);
	double trload(0);
	double tralppx(0);
	double tralpnx(0);
	double trbetx(0);
	double trcode(0);
	double tmcode(0);
	double refa_st(0);
	double caa(0); 

	//localizing module-variables
	//input data
	int maero=hyper[100].integer();
	//-------------------------------------------------------------------------
	//
	//for GHAME
	refa=557.42; //m^2
	refb=24.38;  //m
	refc=22.86;  //m

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
	//		 6     tralppx    kinematics   minimum neg angle of attack - deg
	//		 7     trbetx     kinematics   maximum sidelsip angle - deg

	trmach=0.8;
	trdynm=10.e+3;
	trload=3;
	tralppx=21;
	tralpnx=-3;
	trbetx=5;
	
	trcode=0;
	tmcode=0;

	//for transfer vehicle
	refa_st=7; //m^2
	caa=0.4; 

	//-------------------------------------------------------------------------

	//loading module-variables
	hyper[104].gets(refa);
	hyper[105].gets(refb);
	hyper[106].gets(refc);
	hyper[180].gets(trcode);
	hyper[181].gets(tmcode);
	hyper[183].gets(trmach);
	hyper[184].gets(trdynm);
	hyper[185].gets(trload);
	hyper[186].gets(tralppx);
	hyper[187].gets(tralpnx);
	hyper[188].gets(trbetx);
	hyper[190].gets(refa_st);
	hyper[191].gets(caa);
}
///////////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Hyper'
//
//This module performs the following functions:
// (1) Aerodynamic table look-up from file 'hyper6_aero_deck.asc'
//     Ref Area refa= 557.42 m^2
//     Ref Lengths refb= 24.38m, refc= 22.86 m
//     Moment coefficients referenced to fixed c.g.
// (2) Calculation of aero coefficients in body coordinates
//
//030507 Created by Peter H Zipfel
//040301 Added transfer vehicle, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::aerodynamics()
{
	//local module-variables
	double cd(0);
	double cy(0);
	double cl(0);
	double cx(0);
	double cz(0);
	double cll(0);
	double clm(0);
	double cln(0);
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
	double cma(0);
	double cmde(0);
	double cmq(0);
	double clnb(0);
	double clnda(0);
	double clndr(0);
	double clnp(0);
	double clnr(0);
	double clovercd(0);
	double gmax(0);
	double gminx(0);
	double gavail_pos(0);
	double gavail_neg(0);

	//localizing module-variables
	//input data
	int maero=hyper[100].integer();
	double alpplimx=hyper[160].real();
	double strct_pos_limitx=hyper[161].real();
	double alpnlimx=hyper[164].real();
	double strct_neg_limitx=hyper[165].real();
	//from initialization
	double refa=hyper[104].real();
    double refb=hyper[105].real();
    double refc=hyper[106].real();
    double refa_st=hyper[190].real();
    double caa=hyper[191].real();
	//input from other modules
	double time=round6[0].real();
	double alphax=round6[144].real();
	double betax=round6[145].real();
	double vmach=round6[56].real();
	double pdynmc=round6[57].real();
	double dvba=round6[75].real();
	double ppx=round6[160].real();
	double qqx=round6[161].real();
	double rrx=round6[162].real();
	double vmass=hyper[15].real();;			
	double trcode=hyper[180].real();
	double trload=hyper[185].real();
	double delax=hyper[619].real();
	double delex=hyper[620].real();
	double delrx=hyper[621].real();
	//-------------------------------------------------------------------------

	// GHAME aero-coefficients
	if(maero==1){
		//drag coefficient
		cd0=aerotable.look_up("cd0_vs_alpha_mach",alphax,vmach);
		cda=aerotable.look_up("cda_vs_alpha_mach",alphax,vmach);
		cd=cd0+cda*alphax;

		//lift coefficient
		cl0=aerotable.look_up("cl0_vs_alpha_mach",alphax,vmach);
		cla=aerotable.look_up("cla_vs_alpha_mach",alphax,vmach);
		clde=aerotable.look_up("clde_vs_alpha_mach",alphax,vmach);
		cl=cl0+cla*alphax+clde*delex;
		clovercd=fabs(cl/cd);

		//side force coefficient
		cyb=aerotable.look_up("cyb_vs_alpha_mach",alphax,vmach);
		cyda=aerotable.look_up("cyda_vs_alpha_mach",alphax,vmach);
		cydr=aerotable.look_up("cydr_vs_alpha_mach",alphax,vmach);
		cy=cyb*betax+cyda*delax+cydr*delrx;

		//rolling moment coeffient
		cllb=aerotable.look_up("cllb_vs_alpha_mach",alphax,vmach);
		cllda=aerotable.look_up("cllda_vs_alpha_mach",alphax,vmach);
		clldr=aerotable.look_up("clldr_vs_alpha_mach",alphax,vmach);
		cllp=aerotable.look_up("cllp_vs_alpha_mach",alphax,vmach);
		cllr=aerotable.look_up("cllr_vs_alpha_mach",alphax,vmach);
		cll=cllb*betax+cllda*delax+clldr*delrx+cllp*ppx*RAD*refb/(2*dvba)
			+cllr*rrx*RAD*refb/(2*dvba);

		//pitching moment coefficient
		cm0=aerotable.look_up("cm0_vs_alpha_mach",alphax,vmach);
		cma=aerotable.look_up("cma_vs_alpha_mach",alphax,vmach);
		cmde=aerotable.look_up("cmde_vs_alpha_mach",alphax,vmach);
		cmq=aerotable.look_up("cmq_vs_alpha_mach",alphax,vmach);
		clm=cm0+cma*alphax+cmde*delex+cmq*qqx*RAD*refc/(2.*dvba);	

		//yawing moment coefficient
		clnb=aerotable.look_up("clnb_vs_alpha_mach",alphax,vmach);
		clnda=aerotable.look_up("clnda_vs_alpha_mach",alphax,vmach);
		clndr=aerotable.look_up("clndr_vs_alpha_mach",alphax,vmach);
		clnp=aerotable.look_up("clnp_vs_alpha_mach",alphax,vmach);
		clnr=aerotable.look_up("clnr_vs_alpha_mach",alphax,vmach);
		cln=clnb*betax+clnda*delax+clndr*delrx+clnp*ppx*RAD*refb/(2.*dvba)
			+clnr*rrx*RAD*refb/(2.*dvba);
		
		//force coefficients in body coord
		// (cd, cy, cl are given in stability coord)
		double cosa=cos(alphax*	RAD);
		double sina=sin(alphax*RAD);
		cx=-cd*cosa+cl*sina;
		cz=-cd*sina-cl*cosa;

		//calculate load factor available for max alpha
		double cl0max=aerotable.look_up("cl0_vs_alpha_mach",alpplimx,vmach);
		double clamax=aerotable.look_up("cla_vs_alpha_mach",alpplimx,vmach);
		double clmax=cl0max+clamax*alpplimx;
		double almax=clmax*pdynmc*refa;
		double weight=vmass*AGRAV;
		gmax=almax/weight;
		if(gmax>=strct_pos_limitx)gmax=strct_pos_limitx; 
		double aload=cl*pdynmc*refa;
		double gg=aload/weight;
		gavail_pos=gmax-gg;

		//calculate load factor available for min alpha
		double cl0min=aerotable.look_up("cl0_vs_alpha_mach",alpnlimx,vmach);
		double clamin=aerotable.look_up("cla_vs_alpha_mach",alpnlimx,vmach);
		double clmin=cl0min+clamin*alpnlimx;
		double almin=clmin*pdynmc*refa;
		weight=vmass*AGRAV;
		gminx=almin/weight;
		if(gminx<=strct_neg_limitx)gminx=strct_neg_limitx; 
		aload=cl*pdynmc*refa;
		gg=aload/weight;
		gavail_neg=gminx-gg;

		//run-termination if max g capability is less than 'trload'
		if(gmax<trload)trcode=4;
	}

	//transfer vehicle aero coefficient
	if(maero==2){
		refa=refa_st;
		cx=-caa;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[104].gets(refa);
	hyper[112].gets(cy);
	hyper[113].gets(cll);	
	hyper[114].gets(clm);
	hyper[115].gets(cln);
	hyper[116].gets(cx);
	hyper[117].gets(cz);
	hyper[163].gets(gmax);
	hyper[167].gets(gminx);
	//diagnostics
	hyper[110].gets(cd);
	hyper[111].gets(cl);
	hyper[120].gets(cd0);
	hyper[121].gets(cda);
	hyper[122].gets(cl0);
	hyper[123].gets(cla);
	hyper[124].gets(clde);
	hyper[125].gets(cyb);
	hyper[126].gets(cyda);
	hyper[127].gets(cydr);
	hyper[128].gets(cllb);
	hyper[129].gets(cllda);
	hyper[130].gets(cllp);
	hyper[131].gets(cllr);
	hyper[132].gets(cm0);
	hyper[133].gets(cma);
	hyper[134].gets(cmde);
	hyper[135].gets(cmq);
	hyper[136].gets(clnb);
	hyper[137].gets(clnda);
	hyper[138].gets(clndr);
	hyper[139].gets(clnp);
	hyper[140].gets(clnr);
	hyper[141].gets(clldr);
	hyper[142].gets(clovercd);
	hyper[162].gets(gavail_pos);
	hyper[166].gets(gavail_neg);

	//-------------------------------------------------------------------------	
	//function call for derivative calculations
	if(maero==1){
		aerodynamics_der();
	}
}	
///////////////////////////////////////////////////////////////////////////////
//On-line calculation of aerodynamic derivatives for aero-adaptive autopilot 
//Member function of class 'Hyper'
//Input/output occurs over module-variables
//
//This subroutine performs the following functions:
// (1) calculates dimensional derivaties (rad, m, sec)
// (2) calculates the airframe's rigid modes
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::aerodynamics_der()
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

	//localizing module-variables
	//from initialization
	double refa=hyper[104].real();
    double refb=hyper[105].real();
    double refc=hyper[106].real();
	//from other modules
	double time=round6[0].real();
	double vmach=round6[56].real();
	double pdynmc=round6[57].real();
	double dvba=round6[75].real();
    double vmass= hyper[15].real();
	Matrix IBBB=hyper[18].mat(); 
	//from aerodynamics module		
	double cla=hyper[123].real();
	double clde=hyper[124].real();
	double cyb=hyper[125].real();
	double cydr=hyper[127].real();
	double cllda=hyper[129].real();
	double cllp=hyper[130].real();
	double cma=hyper[133].real();
	double cmde=hyper[134].real();
	double cmq=hyper[135].real();
	double clnb=hyper[136].real();
	double clndr=hyper[138].real();
	double clnr=hyper[140].real();
	//-------------------------------------------------------------------------
	//MOI components
	double ibbb11=IBBB.get_loc(0,0);
	double ibbb22=IBBB.get_loc(1,1);
	double ibbb33=IBBB.get_loc(2,2);
	//Dimensional derivatives for pitch plane (converted to 1/rad where requried)
	double duml=(pdynmc*refa/vmass)/RAD;
	dla=duml*cla;
	dlde=duml*clde;
	double dumm=pdynmc*refa*refc/ibbb22;
    dma=dumm*cma/RAD;
    dmq=dumm*(refc/(2.*dvba))*cmq;
    dmde=dumm*cmde/RAD;

	//Dimensional derivatives in plane (converted to 1/rad where requried)
    double dumy=pdynmc*refa/vmass;
    dyb=dumy*cyb/RAD;
	dydr=dumy*cydr/RAD;
    double dumn=pdynmc*refa*refb/ibbb33;
    dnb=dumn*clnb/RAD;
    dnr=dumn*(refb/(2.*dvba))*clnr;
    dndr=dumn*clndr/RAD;

	//Dimensional derivatives in roll (converted to 1/rad where requried)
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
    hyper[145].gets(dla);
    hyper[146].gets(dlde);	
    hyper[147].gets(dma);	
    hyper[148].gets(dmq);	
    hyper[149].gets(dmde);	
    hyper[150].gets(dyb);	
    hyper[151].gets(dydr);	
    hyper[152].gets(dnb);	
    hyper[153].gets(dnr);	
    hyper[154].gets(dndr);	
    hyper[155].gets(dllp);	
    hyper[156].gets(dllda);	
	//diagnostics
	hyper[144].gets(stmarg);
    hyper[170].gets(realp1);
    hyper[171].gets(realp2);
    hyper[172].gets(wnp);
    hyper[173].gets(zetp);
    hyper[174].gets(rpreal);
    hyper[175].gets(realy1);
    hyper[176].gets(realy2);
    hyper[177].gets(wny);
    hyper[178].gets(zety);
    hyper[179].gets(ryreal);
}
