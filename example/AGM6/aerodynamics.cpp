///////////////////////////////////////////////////////////////////////////////
//FILE: 'aerodynamics.cpp'
//
//Contains 'aerodynamics' module of class 'Missile'
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of aerodynamic module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[100-199]
// 
//Defining and initializing module-variables
//Aerodynamic model based on AGM6
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_aerodynamics()
{
	//Definition and initialization of module-variables
    missile[103].init("refl",0,"Reference length for moment derivatives - m","aerodynamics","init","");
    missile[104].init("refa",0,"Reference area for aero coefficients - m^2","aerodynamics","init","");
    missile[120].init("ca",0,"Axial force coefficient","aerodynamics","out","");
    missile[121].init("cy",0,"Side force coefficient","aerodynamics","out","");
    missile[122].init("cn",0,"Normal force coefficient","aerodynamics","out","");
    missile[123].init("cll",0,"Rolling moment coefficient","aerodynamics","out","");
    missile[124].init("clm",0,"Pitching moment coefficient","aerodynamics","out","");
    missile[125].init("cln",0,"Yawing moment coefficient","aerodynamics","out","");
    missile[130].init("cn0",0,"Normal force coeff as function of Mach and alpha","aerodynamics","diag","");
    missile[131].init("cnp",0,"Normal force coeff of roll attitude","aerodynamics","diag","");
    missile[132].init("clm0",0,"Pitching moment coeff as a function of Mach and alpha","aerodynamics","diag","");
    missile[133].init("clmp",0,"Pitching moment coeff of roll attitude","aerodynamics","diag","");
    missile[134].init("cyp",0,"Side force coeff of roll attitude","aerodynamics","diag","");
    missile[135].init("clnp",0,"Yawing moment coeff of roll attitude","aerodynamics","diag","");
    missile[136].init("ca0",0,"Axial force coeff at zero incidence","aerodynamics","diag","");
    missile[137].init("caa",0,"Axial force coeff alpha derivative - 1/deg","aerodynamics","diag","");
    missile[138].init("cad",0,"Axial force coeff of control surface drag - 1/deg^2","aerodynamics","diag","");
    missile[139].init("cndq",0,"Normal force coeff of pitch control deflection - 1/deg","aerodynamics","diag","");
    missile[140].init("clmdq",0,"Pitching moment coeff of pitch control deflec - 1/deg","aerodynamics","diag","");
    missile[141].init("clmq",0,"Pitching moment coeff pitch damping - 1/deg","aerodynamics","diag","");
    missile[142].init("cllap",0,"Rolling moment coeff of incidence angle - 1/deg^2","aerodynamics","diag","");
    missile[143].init("clldp",0,"Rolling moment coeff of roll control deflec - 1/deg","aerodynamics","diag","");
    missile[144].init("cllp",0,"Rolling moment coeff of roll damping - 1/deg","aerodynamics","diag","");
    missile[145].init("dna",0,"Normal force slope derivative - m/s^2","aerodynamics","out","");
    missile[146].init("dnd",0,"Pitch control force derivative - m/s^2","aerodynamics","out","");
    missile[147].init("dma",0,"Pitch moment derivative - 1/s^2","aerodynamics","out","");
    missile[148].init("dmq",0,"Pitch damping derivative - 1/s","aerodynamics","out","");
    missile[149].init("dmd",0,"Pitch control derivative - 1/s^2","aerodynamics","out","");
    missile[150].init("dlp",0,"Roll damping derivative - 1/s","aerodynamics","out","");
    missile[151].init("dld",0,"Roll control derivative - 1/s^2","aerodynamics","out","");
    missile[152].init("cna",0,"Normal force coeff in aeroballistic axes","aerodynamics","diag","");
    missile[153].init("cya",0,"Side force coeff in aeroballistic axes","aerodynamics","diag","");
    missile[154].init("clma",0,"Pitching moment coeff in aeroballistic axes","aerodynamics","diag","");
    missile[155].init("clna",0,"Yawing moment coeff in aeroballistic axes","aerodynamics","diag","");
    missile[159].init("stmarg",0,"Static margin (+stable, -unstable) - diameter","aerodynamics","diag","plot");
    missile[160].init("realq1",0,"First real root of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
    missile[161].init("realq2",0,"Second real root of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
    missile[162].init("wnq",0,"Natural frequency of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
    missile[163].init("zetq",0,"Damping of airframe pitch dynamics - NA","aerodynamics","diag","plot");
    missile[164].init("realp",0,"Real root of airframe roll dynamics - rad/s","aerodynamics","diag","");
    missile[165].init("alplimx",0,"Maximum total alpha permissible - deg","aerodynamics","data","");
    missile[166].init("gavail",0,"Maneuver headroom - g's","aerodynamics","diag","plot");
    missile[167].init("gmax",0,"Max maneuverability limited by ALPLIMX - g's","aerodynamics","diag","plot");
    missile[168].init("pqreal",0,"Real part or mean value (real roots) of airf dyn - rad","aerodynamics","diag","");
    missile[180].init("trcond","int",0,"Termination condition number","aerodynamics","init","plot");
    missile[181].init("tmcode",0,"Dummy variable initialized to zero","aerodynamics","data","");
    missile[182].init("trcvel",0,"Minimum closing speed - m/s","aerodynamics","data","");
    missile[183].init("trmach",0,"Minimum Mach number","aerodynamics","data","");
    missile[184].init("trdynm",0,"Minimum dynamic pressure - Pa","aerodynamics","data","");
    missile[185].init("trload",0,"Minimum load capacity - g's","aerodynamics","data","");
    missile[186].init("tralp",0,"Maximum total angle of attack - rad","aerodynamics","data","");
    missile[187].init("trtht",0,"Maximum pitch gimbal angle - rad","aerodynamics","data","");
    missile[189].init("trthtd",0,"Maximum pitch gimbal rate - rad/s","aerodynamics","data","");
    missile[190].init("trphid",0,"Maximum roll gimbal rate - rad/s","aerodynamics","data","");
    missile[191].init("trate",0,"Maximum tracking rate - rad/s","aerodynamics","data","");
}	

///////////////////////////////////////////////////////////////////////////////
//Initialization of aerodynamic module 
//Member function of class 'Missile'
//This module performs the following functions:
//
// (1)Aerodynamic tables are located in file 'AGM66_aero_deck'
//      Ref length refl = 0.5 m
//		Ref area   refa = 0.196 m^2
// (2) Defines the run stopping conditions
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::init_aerodynamics()
{
	//specifying reference values
	double refl(0.5);
	double refa(0.196);

	//Run termination criteria
	//If any of the termination limits is violated a number code is
	//stored in 'trcond'. If 'stop 1' the simulation will stop.
	//If 'stop 0' (default) the simulation will continue, and additional
	//code numbers, if any, will enter on the left of 'trcond' until the
	//run is stopped by other means.
	//
	//	   code		     		module			description
	//		 1     trortho    kinematics	maximum orthogonality error
	//		 2     trmach     environment   minimum Mach number
	//		 3     trdynm     environment   minimum dynamic pressure - Pa
	//		 4     trload     aerodynamics  minimum load factor - g's
	//		 5     tralp      kinematics	maximum total incidence angle - rad
	//		 6     trtht      sensor		maximum pitch gimbal angle - rad (value set in input.asc)
	//		 7     trthtd     sensor		maximum pitch gimbal rate - rad/s (value set in input.asc)
	//		 8     trphid     sensor		maximum roll gimbal rate  - rad/s (value set in input.asc)
	//		 9     trate      sensor		maximum tracking rate - rad/s (value set in input.asc)

	double trortho(10e-5);
	double trmach(0.4);
	double trdynm(10e+3);
	double trload(0.5);
	double tralp(1);
	int trcond(0);

	//loading module-variables
	missile[103].gets(refl);
	missile[104].gets(refa);
	missile[182].gets(trortho);
	missile[183].gets(trmach);
	missile[184].gets(trdynm);
	missile[185].gets(trload);
	missile[186].gets(tralp);
	missile[180].gets(trcond);
}
///////////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Missile'
// Calculates aerodynamic coefficients in aeroballistic coordinates
//  from look-up tables
// Converts them to body coordinates
// Aerodynamic model based on SRAAM6
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::aerodynamics()
{
	//local module-variables
	double ca0(0);
	double caa(0);
	double cad(0);
	double caoff(0);
	double ca(0);
	double cyp(0);
	double cydr(0);
	double cya(0);
	double clma(0);
	double cn0(0);
	double cnp(0);
	double cna(0);
	double cllap(0);
	double cllp(0);
	double clldp(0);
	double clm0(0);
	double clmp(0);
	double clmq(0);
	double clmdq(0);
	double clnp(0);
	double clna(0);
	double gmax(0);
	double gavail(0);
	double cy(0);
	double cn(0);
	double cll(0);
	double clm(0);
	double cln(0);

	//localizing module-variables
	//input data
	double alplimx=missile[165].real();
	//from initialization
	double refl=missile[103].real();
	double refa=missile[104].real();
	int trcond=missile[180].integer();
	double trload=missile[185].real();
	//from other modules
	double time=flat6[0].real();
	double vmach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double alppx=flat6[140].real();
	double phip=flat6[143].real();
	double ppx=flat6[160].real();
	double qqx=flat6[161].real();
	double rrx=flat6[162].real();
	double dvba=flat6[75].real();
	double vmass=missile[12].real();
	double alimit=missile[507].real();
	double dpx=missile[619].real();
	double dqx=missile[620].real();
	double drx=missile[621].real();
	//-------------------------------------------------------------------------
	//transforming control commands from body -> aeroballistic coord.
    double cphip=cos(phip);
    double sphip=sin(phip);
    double dqax=dqx*cphip-drx*sphip;
    double drax=dqx*sphip+drx*cphip;

	//transforming body rates from body -> aeroballistic coord.
    double qqax=qqx*cphip-rrx*sphip;
    double rrax=qqx*sphip+rrx*cphip;

	//looking up axial force coefficients
	ca0=aerotable.look_up("ca0_vs_mach",vmach);
	caa=aerotable.look_up("caa_vs_mach",vmach);
	cad=aerotable.look_up("cad_vs_mach",vmach);

	//axial force coefficient
	double deff=(fabs(dqax)+fabs(drax))/2;
    ca=ca0+caa*alppx+cad*deff*deff;

	//looking up side force coefficents in aeroballistic coord
	cyp=aerotable.look_up("cyp_vs_mach_alpha",vmach,alppx);
	cydr=aerotable.look_up("cndq_vs_mach",vmach);

	//side force coefficient
    double s4phi=sin(4*phip);
    cya=cyp*s4phi+cydr*drax;

	//looking up normal force coefficients in aeroballistic coord
	cn0=aerotable.look_up("cn0_vs_mach_alpha",vmach,alppx);
	cnp=aerotable.look_up("cnp_vs_mach_alpha",vmach,alppx);
	double cndq=aerotable.look_up("cndq_vs_mach",vmach);

	//normal force coefficient
    double s2phi=pow(sin(2*phip),2);
    cna=cn0+cnp*s2phi+cndq*dqax;

	//looking up rolling moment coefficients
	cllap=aerotable.look_up("cllap_vs_mach",vmach);
	cllp=aerotable.look_up("cllp_vs_mach",vmach);
	clldp=aerotable.look_up("clldp_vs_mach",vmach);

	//colling moment coefficient
    cll=cllap*alppx*alppx*s4phi+cllp*ppx*refl/(2*dvba)+clldp*dpx;

	//looking up pitching moment coefficients in aeroballistic coord
	clm0=aerotable.look_up("clm0_vs_mach_alpha",vmach,alppx);
	clmp=aerotable.look_up("clmp_vs_mach_alpha",vmach,alppx);
	clmq=aerotable.look_up("clmq_vs_mach",vmach);
	clmdq=aerotable.look_up("clmdq_vs_mach",vmach);

	//pitching moment coefficient
    clma=clm0+clmp*s2phi+clmq*qqax*refl/(2*dvba)+clmdq*dqax;

	//looking up yawing moment coefficients in aeroballistic coord
	clnp=aerotable.look_up("clnp_vs_mach_alpha",vmach,alppx);

	//yawing moment coefficient
    double clnr=clmq;
    double clndr=clmdq;
    clna=clnp*s4phi+clnr*rrax*refl/(2*dvba)+clndr*drax;

	//force coefficients in body axes
	cy=cya*cphip-cna*sphip;
	cn=cya*sphip+cna*cphip;

	//moment coefficient in body axes
	clm=clma*cphip+clna*sphip;
	cln=clna*cphip-clma*sphip;

	//calculate load factor available
	double cn0mx=aerotable.look_up("cn0_vs_mach_alpha",vmach,alplimx);
	double almx=cn0mx*pdynmc*refa;
	double weight=vmass*AGRAV;
	gmax=almx/weight;
	if(gmax>=alimit)gmax=alimit;
	double aload=cn0*pdynmc*refa;
	double gg=aload/weight;
	gavail=gmax-gg;

	//run-termination if max g capability is less than 'trload'
	if(gmax<trload)
		trcond=4;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[120].gets(ca);
	missile[121].gets(cy);
	missile[122].gets(cn);
	missile[123].gets(cll);	
	missile[124].gets(clm);
	missile[125].gets(cln);
	missile[167].gets(gmax);
	missile[180].gets(trcond);
	//diagnostics
	missile[130].gets(cn0);
	missile[131].gets(cnp);
	missile[132].gets(clm0);
	missile[133].gets(clmp);
	missile[134].gets(cyp);
	missile[136].gets(ca0);
	missile[135].gets(clnp);
	missile[137].gets(caa);
	missile[138].gets(cad);
	missile[139].gets(cndq);
	missile[140].gets(clmdq);
	missile[141].gets(clmq);
	missile[142].gets(cllap);
	missile[143].gets(clldp);
	missile[144].gets(cllp);
	missile[152].gets(cna);
	missile[153].gets(cya);
	missile[154].gets(clma);
	missile[155].gets(clna);
	missile[166].gets(gavail);
	//-------------------------------------------------------------------------	
	//function call for derivative calculations
	aerodynamics_der();
}	
///////////////////////////////////////////////////////////////////////////////
//On-line calculation of aerodynamic derivatives for aero-adaptive autopilot 
//Member function of class 'Missile'
//
//This subroutine performs the following functions:
//
// (1) calculates non-dimensional derivatives (radians)
// (2) calculates dimensional derivaties (rad, m, sec)
// (3) calculates the airframe roots
//
//100406 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::aerodynamics_der()
{
	//local module-variables
	double wnq(0);
	double zetq(0);
	double realq1(0);
	double realq2(0);
	double pqreal(0);
	double realp(0);

	//localizing module-variables
	//input data
	double alplimx=missile[165].real();
	//from initialization
	double refl=missile[103].real();
	double refa=missile[104].real();
	//saved values in case calculation is by-passed
	double dna=missile[145].real();
	double dnd=missile[146].real();
	double dma=missile[147].real();
	double dmq=missile[148].real();
	double dmd=missile[149].real();
	double dlp=missile[150].real();
	double dld=missile[151].real();
	double stmarg=missile[159].real();
	//from other modules
	double time=flat6[0].real();
	double vmach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double alppx=flat6[140].real();
	double phip=flat6[143].real();
	double dvba=flat6[75].real();
	double vmass=missile[12].real();
	double ai11=missile[16].real();
	double ai33=missile[17].real();
	//from aerodynamics module
	double cndq= missile[139].real();
	double clmdq=missile[140].real();
	double clmq= missile[141].real();
	double cllp= missile[144].real();
	double clldp=missile[143].real();
	//-------------------------------------------------------------------------
	//perform calculations only as long as alppx < alplimx-3
	if(alppx<(alplimx-3.))
	{
		//Non-dimensional derivatives
		//look up coeff at +- 3 deg, but not beyond tables
		double alpp=alppx+3;
		if(alpp<3)alpp=3;
		double alpm=alppx-3;
		if(alpm<0)alpm=0;

		//calculating normal force derivative wrt alpha 'cna'			
		double cn0p=aerotable.look_up("cn0_vs_mach_alpha",vmach,alpp);
		double cn0m=aerotable.look_up("cn0_vs_mach_alpha",vmach,alpm);		
		double dum=(cn0p-cn0m);
		double cna=DEG*dum/(alpp-alpm);

		//pitch control derivative
		double cnd=DEG*cndq;
		
		//calculating pitch moment derivative wrt alpha 'cma'
		double clm0p=aerotable.look_up("clm0_vs_mach_alpha",vmach,alpp);
		double clm0m=aerotable.look_up("clm0_vs_mach_alpha",vmach,alpm);
		double cma=DEG*(clm0p-clm0m)/(alpp-alpm);
		
		//other derivatives
		double cmq=DEG*clmq;
		double cmd=DEG*clmdq;
		double clp=DEG*cllp;
		double cld=DEG*clldp;

		//dimensional derivatives		
		double dumn=pdynmc*refa/vmass;
		dna=dumn*cna;
		dnd=dumn*cnd;
		
		double dumm=pdynmc*refa*refl/ai33;
		dma=dumm*cma;
		dmq=dumm*(refl/(2*dvba))*cmq;
		dmd=dumm*cmd;
		
		double duml=pdynmc*refa*refl/ai11;
		dlp=duml*(refl/(2.*dvba))*clp;
		dld=duml*cld;
		
		//static margin		
		stmarg=-cma/cna;
	}	
	//maneuver plane roots		
	double a11=dmq;
	double a12=dma/dna;
	double a21=dna;
	double a22=-dna/dvba;
	
	double arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
	   wnq=0.;
	   zetq=0.;
	   double dum=a11+a22;
	   realq1=(dum+sqrt(arg))/2.;
	   realq2=(dum-sqrt(arg))/2.;
	   pqreal=(realq1+realq2)/2.;
	}
	else
	{
	   realq1=0.;
	   realq2=0.;
	   wnq=sqrt(a11*a22-a12*a21);
	   zetq=-(a11+a22)/(2.*wnq);
	   pqreal=-zetq*wnq;
	}
	
	//roll rate root		
	realp=dlp;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[145].gets(dna);
	missile[146].gets(dnd);
	missile[147].gets(dma);
	missile[148].gets(dmq);
	missile[149].gets(dmd);
	missile[150].gets(dlp);
	missile[151].gets(dld);
	//diagnostics
	missile[159].gets(stmarg);
	missile[160].gets(realq1);
	missile[161].gets(realq2);
	missile[162].gets(wnq);
	missile[163].gets(zetq);
	missile[164].gets(realp);
	missile[168].gets(pqreal);
}