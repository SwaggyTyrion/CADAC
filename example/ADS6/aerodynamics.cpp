///////////////////////////////////////////////////////////////////////////////
//FILE: 'aerodynamics.cpp'
//
//Contains 'aerodynamics' module of class 'Missile'
//
//030601 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'aerodynamics' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[100-199]
// 
//170816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::def_aerodynamics()
{
	//Definition and initialization of module-variables
	missile[103].init("refl",0.25,"Reference length for moment derivatives - m","aerodynamics","init","");
	missile[104].init("refa",0.0491,"Reference area for aero coefficients - m^2","aerodynamics","init","");
	missile[120].init("ca",0,"Axial force coefficient - ND","aerodynamics","out","");
	missile[121].init("cy",0,"Side force coefficient - ND","aerodynamics","out","");
	missile[122].init("cn",0,"Normal force coefficient - ND","aerodynamics","out","");
	missile[123].init("cll",0,"Rolling moment coefficient - ND","aerodynamics","out","");
	missile[124].init("clm",0,"Pitching moment coefficient - ND","aerodynamics","out","");
	missile[125].init("cln",0,"Yawing moment coefficient - ND","aerodynamics","out","");
	missile[131].init("cyb",0,"Yaw force  derivative - 1/rad ","aerodynamics","save","");
	missile[132].init("clnb",0,"Yaw moment derivative - 1/rad ","aerodynamics","save","");
	missile[133].init("clnr",0,"Yaw damping derivative - 1/rad","aerodynamics","diag","");
	missile[134].init("clndr",0,"Yaw control derivatve - 1/deg","aerodynamics","diag","");
	missile[136].init("ca0",0,"Axial force coeff at zero incidence","aerodynamics","diag","");
	missile[138].init("cad",0,"Axial force coeff of control surface drag - 1/deg^2","aerodynamics","diag","");
	missile[139].init("cndq",0,"Normal force coeff of pitch control deflection - 1/deg","aerodynamics","diag","");
	missile[140].init("clmdq",0,"Pitch control derivative - 1/deg","aerodynamics","diag","");
	missile[141].init("clmq",0,"Pitching damping derivative - 1/rad","aerodynamics","diag","");
	missile[143].init("clldp",0,"Roll control derivative - 1/deg","aerodynamics","diag","");
	missile[144].init("cllp",0,"Roll damping derivative - 1/rad","aerodynamics","diag","");
	missile[145].init("dna",0,"Normal force slope derivative - m/s^2","aerodynamics","out","");
	missile[146].init("dnd",0,"Yaw control derivative - 1/s^2","aerodynamics","out","");
	missile[147].init("dma",0,"Pitch moment derivative - 1/s^2","aerodynamics","out","");
	missile[148].init("dmq",0,"Pitch damping derivative - 1/s","aerodynamics","out","");
	missile[149].init("dmd",0,"Pitch control derivative - 1/s^2","aerodynamics","out","");
	missile[150].init("dlp",0,"Roll damping derivative - 1/s","aerodynamics","out","");
	missile[151].init("dld",0,"Roll control derivative - 1/s^2","aerodynamics","out","");
	missile[152].init("cna",0,"Normal force derivative - 1/rad","aerodynamics","save","");
	missile[153].init("clma",0,"Pitch moment derivatve - 1/rad","aerodynamics","save","");
	missile[159].init("stmarg_pitch",0,"Static margin in pitch (+ stable, - unstable) - diameter","aerodynamics","save","plot");
	missile[160].init("realq1",0,"First real root of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
	missile[161].init("realq2",0,"Second real root of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
	missile[162].init("wnq",0,"Natural frequency of airframe in pitch plane - rad/s","aerodynamics","diag","plot");
	missile[163].init("zetq",0,"Damping of airframe in pitch plane - NA","aerodynamics","diag","plot");
	missile[164].init("realp",0,"Real root of airframe roll dynamics - rad/s","aerodynamics","diag","plot");
	missile[165].init("alplimx",40,"Maximum total alpha permissible - deg","aerodynamics","data","");
	missile[166].init("gavail",0,"Maneuver headroom - g's","aerodynamics","diag","plot");
	missile[167].init("gmax",0,"Max maneuverability limited by 'alplimx' - g's","aerodynamics","diag","plot");
	missile[168].init("pqreal",0,"Real part or mean value (real roots) of pitch dyn - rad","aerodynamics","diag","");
	missile[169].init("dnr",0,"Yaw damping derivative - 1/s","aerodynamics","out","");
	missile[170].init("dyb",0,"Yaw force slope derivative - m/s^2","aerodynamics","out","");
	missile[171].init("dnb",0,"Yaw moment derivative - 1/s^2","aerodynamics","out","");
	missile[172].init("wnr",0,"Natural frequency of airframe in yaw plane - rad/s","aerodynamics","diag","plot");
	missile[173].init("zetr",0,"Damping of airframe in yaw plane - NA","aerodynamics","diag","");
	missile[174].init("stmarg_yaw",0,"Static margin in yaw (+ stable, - unstable) - diameter","aerodynamics","save","plot");
	missile[175].init("realr1",0,"First real root of airframe yaw dynamics - rad/s","aerodynamics","diag","plot");
	missile[176].init("realr2",0,"Second real root of airframe yaw dynamics - rad/s","aerodynamics","diag","plot");
	missile[177].init("prreal",0,"Real part or mean value (real roots) of yaw dyn - rad","aerodynamics","diag","plot");	
	missile[180].init("trcond","int",0,"Termination condition number","aerodynamics","diag","");
	missile[182].init("trortho",0,"Maximum orthogonality error","aerodynamics","data","");
	missile[183].init("tralp",0,"Minimum Mach number","aerodynamics","data","");
	missile[184].init("trdynm",0,"Minimum dynamic pressure - Pa","aerodynamics","data","");
	missile[185].init("trload",0,"Minimum load capacity - g's","aerodynamics","data","");
}	
///////////////////////////////////////////////////////////////////////////////
//Initialization of 'aerodynamics' module 
//Member function of class 'Missile'
//This module performs the following functions:
//
// Defines the run stopping conditions
//
//170816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::init_aerodynamics()
{
	//Run termination criteria

	//If any of the termination limits is violated, a number code is
	//stored in 'trcond'. If also 'stop=1', the vehicle object is stopped.
	//If 'stop=0' (default), the simulation will continue, and additional
	//code numbers, if any, will overwrite 'trcond' until the
	//run is stopped by other means
	//
	//	  'trcond' term.cond.	module				description
	//
	//		 1     trortho	kinematics		maximum orthogonality error of direction cosine matrix overflow
	//		 2     tralp	kinematics		maximum total angle of attack overflow - rad
	//		 3     trdynm	environment		minimum dynamic pressure underflow - Pa
	//		 4     trload	aerodynamics    minimum available load factor underflow 'gmax' - g's 
	//following limits are provided in 'input.asc'
	//		 5     -		sensor			target not in field-of-view (IR or RF sensor) 
	//		 6     trtht	sensor			maximum pitch gimbal angle, or max FOV for strap-down sensors overflow - rad
	//		 7     trthtd	sensor			maximum pitch gimbal rate overflow - rad/s
	//		 8     trphid	sensor			maximum roll gimbal rate  overflow - rad/s
	//		 9     trate	sensor			maximum tracking rate overflow- rad/s

	double trortho(1e-4);
	double tralp(1.047);
	double trdynm(1e+4);
	double trload(0.001);//very small because SAM is launched from tube

	int trcond(0);

	//loading termination conditions
	missile[180].gets(trcond);
	missile[182].gets(trortho);
	missile[183].gets(tralp);
	missile[184].gets(trdynm);
	missile[185].gets(trload);
}
///////////////////////////////////////////////////////////////////////////////
//'aerodynamics' module
//Member function of class 'Missile'
// Calculates aerodynamic coefficients in aeroballistic coordinates from look-up tables
// Converts them to body coordinates
//
//170816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::aerodynamics()
{
	//local module-variables
	double deffx(0);
	double ca0(0);
	double cad(0);
	double cab(0);
	double ca(0);
	double cy0(0);
	double cydr(0);
	double cn0(0);
	double cndq(0);
	double cllp(0);
	double clldp(0);
	double cll0(0);
	double clm0(0);
	double clmq(0);
	double clmdq(0);
	double cln0(0);
	double clnb(0);
	double clnr(0);
	double clndr(0);
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
	double mach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double phip=flat6[143].real();
	double ppx=flat6[160].real();
	double qqx=flat6[161].real();
	double rrx=flat6[162].real();
	double dvbe=flat6[236].real();
	int    mprop=missile[10].integer();
	double mass=missile[12].real();
	double xcgref=missile[14].real();
	double xcg=missile[15].real();
	double alimitx=missile[507].real();
	double dpx=missile[619].real();
	double dqx=missile[620].real();
	double drx=missile[621].real();
	double delx1=missile[622].real();
	double delx2=missile[623].real();
	double delx3=missile[624].real();
	double delx4=missile[625].real();
	double alphax=flat6[144].real();
	double betax=flat6[145].real();
	//-------------------------------------------------------------------------
	//effective find deflection for drag calculation
	deffx=(fabs(delx1)+fabs(delx2)+fabs(delx3)+fabs(delx4))/4;

	//looking up axial force coefficients
	ca0=aerotable.look_up("ca0_vs_mach,betax,alphax",mach,betax,alphax);
	cad=aerotable.look_up("cad_vs_mach",mach); //(1/deg)
	//axial force coefficient
	ca=ca0+cad*deffx;

	//increasing base drag when motor is off
	if(mprop==0){
		cab=aerotable.look_up("cab_vs_mach",mach);
		ca+=cab;
	}
	//looking up side force coefficents
    cy0=aerotable.look_up("cy0_vs_mach,betax,alphax",mach,betax,alphax);
	cydr=aerotable.look_up("cydr_vs_mach,betax,alphax",mach,betax,alphax); //(1/deg)
	//side force coefficient
	cy=cy0+cydr*drx;

	//looking up normal force coefficients
	cn0=aerotable.look_up("cn0_vs_mach,betax,alphax",mach,betax,alphax);
	cndq=aerotable.look_up("cndq_vs_mach,betax,alphax",mach,betax,alphax); //(1/deg)
	//normal force coefficient
	cn=cn0+cndq*dqx;

	//looking up rolling moment coefficients
	cll0=aerotable.look_up("cll0_vs_mach,betax,alphax",mach,betax,alphax);
	cllp=aerotable.look_up("cllp_vs_mach",mach); //(1/rad)
	clldp=aerotable.look_up("clldp_vs_mach,betax,alphax",mach,betax,alphax); //(1/deg)
	//rolling moment coefficient
	cll=cll0+cllp*ppx*RAD*refl/(2*dvbe)+clldp*dpx;

	//looking up pitching moment coefficients
	clm0=aerotable.look_up("clm0_vs_mach,betax,alphax",mach,betax,alphax);
	clmq=aerotable.look_up("clmq_vs_mach",mach); //(1/rad)
	clmdq=aerotable.look_up("clmdq_vs_mach,betax,alphax",mach,betax,alphax); //(1/deg)
	//pitching moment coefficient
	clm=clm0+clmq*qqx*RAD*refl/(2*dvbe)+clmdq*dqx-cn/refl*(xcgref-xcg);

	//looking up yawing moment coefficients
	cln0=aerotable.look_up("cln0_vs_mach,betax,alphax",mach,betax,alphax);
	clnr=aerotable.look_up("clnr_vs_mach",mach); //(1/rad)
	clndr=aerotable.look_up("clndr_vs_mach,betax,alphax",mach,betax,alphax); //(1/deg)
	//yawing moment coefficient
	cln=cln0+clnr*rrx*RAD*refl/(2*dvbe)+clndr*drx-cy/refl*(xcgref-xcg);

	//computing maximum maneuverability 'gmax' available
	double cn0mx=aerotable.look_up("cn0_vs_mach,betax,alphax",mach,0,alplimx);
	double almx=cn0mx*pdynmc*refa;
 	double weight=mass*AGRAV;
	gmax=almx/weight;
	if(gmax>=alimitx)gmax=alimitx;

	double aload=sqrt(cn0*cn0+cy0*cy0)*pdynmc*refa;

	//computing maneuvering headroom available 'gavail'
	double gg=aload/weight;
	gavail=gmax-gg;
	if(gavail>alimitx)
		gavail=alimitx;
	if(gavail<0)
		gavail=0;

	//run-termination condition if max g capability is less than 'trload'
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
	//diagnostics
	missile[133].gets(clnr);
	missile[134].gets(clndr);
	missile[136].gets(ca0);
	missile[138].gets(cad);
	missile[139].gets(cndq);
	missile[140].gets(clmdq);
	missile[141].gets(clmq);
	missile[143].gets(clldp);
	missile[144].gets(cllp);
	missile[166].gets(gavail);
	missile[180].gets(trcond);
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
// (3) calculates the airframe dynamic roots
//
//170816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::aerodynamics_der()
{
	//local module-variables
	double dna(0);
	double dma(0);
	double dmq(0);
	double dmd(0);
	double dlp(0);
	double dld(0);
	double dnd(0);
	double dnr(0);
	double dyb(0);
	double dnb(0);
	double wnq(0);
	double zetq(0);
	double realq1(0);
	double realq2(0);
	double pqreal(0);
	double realp(0);
	double wnr(0);
	double zetr(0);
	double realr1(0);
	double realr2(0);
	double prreal(0);

	//localizing module-variables
	//input data
	double alplimx=missile[165].real();
	//from initialization
	double refl=missile[103].real();
	double refa=missile[104].real();
	//saved values in case calculation is by-passed
	double cyb=missile[131].real();
	double clnb=missile[132].real();
	double cna=missile[152].real();
	double clma=missile[153].real();
	double stmarg_pitch=missile[159].real();
	double stmarg_yaw=missile[174].real();
	//from other modules
	double time=flat6[0].real();
	double mach=flat6[56].real();
	double pdynmc=flat6[57].real();
	double alppx=flat6[140].real();
	double phip=flat6[143].real();
	double alphax=flat6[144].real();
	double betax=flat6[145].real();
	double dvbe=flat6[236].real();
	double mass=missile[12].real();
	double thrust=missile[13].real();
	double xcgref=missile[14].real();
	double xcg=missile[15].real();
	double ai11=missile[16].real();
	double ai33=missile[17].real();
	int mtvc=missile[700].integer();
	double parm=missile[709].real();
	double gtvc=missile[716].real();
	//from aerodynamics module
	double clnr= missile[133].real();
	double clndr= missile[134].real();
	double cndq= missile[139].real();
	double clmdq=missile[140].real();
	double clmq= missile[141].real();
	double cllp= missile[144].real();
	double clldp=missile[143].real();
	if(time>= 0.333){
		double stop(0);
	}
	//-------------------------------------------------------------------------
	//perform calculations only as long as total angle of attack 'alppx' < 'alplimx'-3
	if(alppx<(alplimx-3))
	{
		//non-dimensional derivatives in pitch
		//look up coeff at +- 3 deg, but not beyond tables
		double alphapx=fabs(alphax)+3;
		if(alphapx<3)alphapx=3;
		double alphamx=fabs(alphax)-3;
		if(alphamx<0)alphamx=0;

		//calculating longitudinal stability derivatives in body axes (dim: 1/rad)
		double cn0p=aerotable.look_up("cn0_vs_mach,betax,alphax",mach,betax,alphapx);
		double cn0m=aerotable.look_up("cn0_vs_mach,betax,alphax",mach,betax,alphamx);
		cna=(cn0p-cn0m)/((alphapx-alphamx)*RAD);

  		double clm0p=aerotable.look_up("clm0_vs_mach,betax,alphax",mach,betax,alphapx);
		double clm0m=aerotable.look_up("clm0_vs_mach,betax,alphax",mach,betax,alphamx);
		clma=(clm0p-clm0m)/((alphapx-alphamx)*RAD)-cna/refl*(xcgref-xcg);

		//non-dimensional derivatives in yaw
		//look up coeff at +- 3 deg, but not beyond tables
		double beta=fabs(betax);
		double betapx=fabs(betax)+3;
		if(betapx<3)betapx=3;
		double betamx=fabs(betax)-3;
		if(betamx<0)betamx=0;

		//calculating lateral stability derivatives in body axes (dim: 1/rad)
  		double cy0p=aerotable.look_up("cy0_vs_mach,betax,alphax",mach,betapx,alphax);
		double cy0m=aerotable.look_up("cy0_vs_mach,betax,alphax",mach,betamx,alphax);
		cyb=(cy0p-cy0m)/((betapx-betamx)*RAD);

  		double cln0p=aerotable.look_up("cln0_vs_mach,betax,alphax",mach,betapx,alphax);
		double cln0m=aerotable.look_up("cln0_vs_mach,betax,alphax",mach,betamx,alphax);
		clnb=(cln0p-cln0m)/((betapx-betamx)*RAD)-cyb/refl*(xcgref-xcg);	
	}

	//dimensional derivatives in pitch (all in 1/rad)
	//normal force dim derivative 'dna'
	dna=(pdynmc*refa/mass)*cna;

	//calculating pitch moment dim derivative wrt alpha 'dma'
	dma=(pdynmc*refa*refl/ai33)*clma;

	//pitch damping dim deriv 'dmq'
	dmq=DEG*(pdynmc*refa*refl/ai33)*(refl/(2*dvbe))*clmq;

	//pitch moment control dim derivative 'dmd'
	dmd=DEG*(pdynmc*refa*refl/ai33)*clmdq;

	//dimensional derivatives in yaw (all in 1/rad)
	//side force dim derivative 'dyb'
	dyb=(pdynmc*refa/mass)*cyb;

	//yawing moment dim derivative 'dnb'
	dnb=(pdynmc*refa*refl/ai33)*clnb;

	//yaw damping dim derivative 'dnr'
	dnr=DEG*(pdynmc*refa*refl/ai33)*(refl/(2*dvbe))*clnr;

	//yaw moment control dim derivative 'dnd'
	dnd=DEG*(pdynmc*refa*refl/ai33)*clndr;

	//dimensional derivatives in roll (all in 1/rad)
	//roll damping dim derivative 'dlp'
	dlp=DEG*(pdynmc*refa*refl/ai11)*(refl/(2*dvbe))*cllp;

	//roll control dim derivative 'dldp'
	dld=DEG*(pdynmc*refa*refl/ai11)*clldp;

	//* Pitch loop
	if(fabs(dna)>=SMALL){
		//static margin in pitch
		stmarg_pitch=-(dma/dna)*(ai33/(refl*mass));
		//fundamental matrix elements	
		double a11=dmq;
		double a12=dma/dna;
		double a21=dna;
		double a22=-dna/dvbe;
		//calculating pitch-plane poles
		double arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
		if(arg>=0)
		{
		   wnq=0;
		   zetq=0;
		   double dum=a11+a22;
		   realq1=(dum+sqrt(arg))/2;
		   realq2=(dum-sqrt(arg))/2;
		   pqreal=(realq1+realq2)/2;
		}
		else
		{
		   realq1=0;
		   realq2=0;
		   wnq=sqrt(a11*a22-a12*a21);
		   zetq=-(a11+a22)/(2*wnq);
		   pqreal=-zetq*wnq;
		}
	}
	//*Yaw loop
	if(fabs(dyb)>=SMALL){
		//static margin in yaw	
		stmarg_yaw=-(dnb/dyb)*(ai33/(refl*mass)); 
		//fundamental matrix elements		
		double a11=dnr; 
		double a12=dnb/dyb;
		double a21=-dyb;
		double a22=dyb/dvbe;
	
		//calculating yaw-plane poles 
		double arg=pow((a11+a22),2)-4*(a11*a22-a12*a21);
		if(arg>=0)
		{
		   wnr=0;
		   zetr=0;
		   double dum=a11+a22;
		   realr1=(dum+sqrt(arg))/2;
		   realr2=(dum-sqrt(arg))/2;
		   prreal=(realr1+realr2)/2;
		}
		else
		{
		   realr1=0;
		   realr2=0;
		   wnr=sqrt(a11*a22-a12*a21);
		   zetr=-(a11+a22)/(2*wnr);
		   prreal=-zetr*wnr;
		}
	}
	//roll rate root		
	realp=dlp;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[145].gets(dna);
	missile[147].gets(dma);
	missile[148].gets(dmq);
	missile[149].gets(dmd);
	missile[150].gets(dlp);
	missile[151].gets(dld);
	missile[146].gets(dnd);
	missile[169].gets(dnr);
	missile[170].gets(dyb);
	missile[171].gets(dnb);
	//saving values
	missile[131].gets(cyb);
	missile[132].gets(clnb);
	missile[152].gets(cna);
	missile[153].gets(clma);
	missile[159].gets(stmarg_pitch);
	missile[174].gets(stmarg_yaw);
	//diagnostics
	missile[160].gets(realq1);
	missile[161].gets(realq2);
	missile[162].gets(wnq);
	missile[163].gets(zetq);
	missile[164].gets(realp);
	missile[168].gets(pqreal);
	missile[172].gets(wnr);
	missile[173].gets(zetr);
	missile[175].gets(realr1);
	missile[176].gets(realr2);
	missile[177].gets(prreal);
}
