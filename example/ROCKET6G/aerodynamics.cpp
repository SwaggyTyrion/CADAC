///////////////////////////////////////////////////////////////////////////////
//FILE: 'aerodynamics.cpp'
//
//Contains 'aerodynamics' module of class 'Hyper'
//
//050103 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of aerodynamic module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[100-199]
//
// This module performs the following functions:
// (1) Calculates the aerodynamic force and moment coefficients from the aero_deck
// (2) Derives the dimensional derivatives for the flight controllers
//
// maero = 0 no aerodynamic forces
//		   11 Booster - 1 stage (last stage)
//         12 Booster - 2 stages
//         13 Booster - 3 stages (launch)
//
//050103 Created by Peter H Zipfel
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_aerodynamics()
{
	//Definition and initialization of module-variables
	hyper[100].init("maero","int",0,"=11: last stage; =12: 2 stages; =13: 3 stages","aerodynamics","data","");
	hyper[104].init("refa",0,"Reference area for aero coefficients - m^2","aerodynamics","init","");
	hyper[105].init("refd",0,"Reference length for aero coefficients - m","aerodynamics","init","");
	hyper[108].init("xcg_ref",0,"Reference cg location from nose - m","aerodynamics","init","");
	//main force and moment coefficients
	hyper[112].init("cy",0,"Side force coefficient - ND","aerodynamics","out","");
	hyper[113].init("cll",0,"Rolling moment coefficient - ND","aerodynamics","out","");
	hyper[114].init("clm",0,"Pitching moment coefficient - ND","aerodynamics","out","");
	hyper[115].init("cln",0,"Yawing moment coefficient - ND","aerodynamics","out","");
	hyper[116].init("cx",0,"X-force coefficient - ND","aerodynamics","out","");
	hyper[117].init("cz",0,"Z-force coefficient - ND","aerodynamics","out","");
	//table look-up coefficients 
	hyper[118].init("ca0",0,"Axial force coeff(Mach) - ND","aerodynamics","diag","");
	hyper[119].init("caa",0,"Delta axial force due to alpha(Mach) - ND","aerodynamics","diag","");
	hyper[120].init("cn0",0,"Normal force coeff(Mach,alpha) - ND","aerodynamics","diag","");
	hyper[121].init("clm0",0,"Pitch moment coeff(Mach,alpha) - ND","aerodynamics","diag","");
	hyper[122].init("clmq",0,"Pitch dampning deriv(Mach) - 1/deg","aerodynamics","diag","");
	hyper[123].init("cla",0,"Lift slope derivative(alpha,mach) - 1/deg","aerodynamics","out","");
	hyper[124].init("clde",0,"Lift force due to elevator (alpha.mach), - 1/deg","aerodynamics","diag","");
	hyper[125].init("cyb",0,"Weather vane der wrt beta(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[127].init("cydr",0,"Side force due to rudder deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[129].init("cllda",0,"Roll control effectiveness(alpha,mach), - 1/deg","aerodynamics","diag","");
	hyper[130].init("cllp",0,"Roll damping deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	hyper[133].init("cma",0,"Pitch moment due to alpha deriv(alpha,mach) -1/deg","aerodynamics","diag","");
	hyper[134].init("cmde",0,"Pitch control effectiveness(alpha,mach), - 1/deg","aerodynamics","diag","");
	hyper[135].init("cmq",0,"Pitch dampning deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	hyper[136].init("cnb",0,"Yaw moment deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[138].init("cndr",0,"Yaw moment due to rudder deriv(alpha,mach) - 1/deg","aerodynamics","diag","");
	hyper[140].init("cnr",0,"Yaw damping deriv(alpha,mach) - 1/rad","aerodynamics","diag","");
	//calculated dimensional derivatives for autopilot
	hyper[143].init("stmarg_yaw",0,"Static margin yaw (+stable, -unstable) - caliber","aerodynamics","diag","plot");
	hyper[144].init("stmarg_pitch",0,"Static margin pitch (+stable, -unstable) - caliber","aerodynamics","diag","plot");
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
	//rigid vehicle dynamic modes
	hyper[170].init("realp1",0,"First real root of airframe pitch dyn  - rad/s","aerodynamics","diag","plot");
	hyper[171].init("realp2",0,"Second real root of airframe pitch dyn - rad/s","aerodynamics","diag","");
	hyper[172].init("wnp",0,"Natural frequency of airframe pitch dynamics - rad/s","aerodynamics","diag","plot");
	hyper[173].init("zetp",0,"Damping of airframe pitch dynamics - NA","aerodynamics","diag","plot");
	hyper[174].init("rpreal",0,"Real part or mean value (real roots) of pitch  - rad/s","aerodynamics","diag","");
	hyper[175].init("realy1",0,"First real root of airframe yaw dynamics - rad/s","aerodynamics","diag","plot");
	hyper[176].init("realy2",0,"Second real root of airframe yaw dynamics - rad/s","aerodynamics","diag","");
	hyper[177].init("wny",0,"Natural frequency of airframe yaw dynamics - rad/s","aerodynamics","diag","plot");
	hyper[178].init("zety",0,"Damping of airframe yaw dynamics - NA","aerodynamics","diag","plot");
	hyper[179].init("ryreal",0,"Real part or mean value (real roots) of yaw - rad/s","aerodynamics","diag","");
	//run termination conditions    
	hyper[180].init("trcode",0,"Termination code number","aerodynamics","init","");
	hyper[181].init("tmcode",0,"Dummy variable initialized to zero","aerodynamics","data","");
	hyper[183].init("trmach",0,"Minimum Mach number","aerodynamics","data","");
	hyper[184].init("trdynm",0,"Minimum dynamic pressure - Pa","aerodynamics","data","");
	hyper[185].init("trload",0,"Minimum load capacity - g's","aerodynamics","data","");
	hyper[186].init("tralp",0,"Maximum total angle of attack - rad","aerodynamics","data","");
	//limiters
	hyper[187].init("alplimx",0,"Alpha limiter for vehicle - deg","aerodynamics","data","");
	hyper[188].init("alimitx",0,"Structural  limiter for vehicle - g's","aerodynamics","data","");
	hyper[189].init("gnavail",0,"G available in pitch for vehicle - g's","aerodynamics","diag","");
	hyper[190].init("gyavail",0,"G available in yaw for vehicle - g's","aerodynamics","diag","");
	hyper[191].init("gnmax",0,"Max g permissable in pitch for vehicle - g's","aerodynamics","out","plot");
	hyper[192].init("gymax",0,"Max g permissable in yaw for vehicle - g's","aerodynamics","out","plot");
}	
///////////////////////////////////////////////////////////////////////////////
//Initialization of aerodynamic module 
//Member function of class 'Hyper'
//Run stopping conditions
//
// maero = 0 no aerodynamic forces
//		   11 Booster - 1 stage (last stage)
//         12 Booster - 2 stages
//         13 Booster - 3 stages (launch)
//
//050103 Created by Peter H Zipfel
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::init_aerodynamics()
{
	//local module variables
	double trmach(0);
	double trdynm(0);
	double trload(0);
	double tralp(0);
	double trcode(0);
	double tmcode(0);
	//-------------------------------------------------------------------------
//Run termination criteria 
	//If any of the termination limits is violated a number code is
	//stored in 'trcond'. If mstop=1 the simulation will stop.
	//If mstop=0 (default) the simulation will continue, and additional
	//code numbers, if any, will enter on the left of 'trcond' until the
	//run is stopped by other means.
	//
	//	   code  term.cond. module				description
	//		 2     trmach     environment	minimum Mach number
	//		 3     trdynm     environment	minimum dynamic pressure - Pa
	//		 4     trload     aerodynamics	minimum load factor - g's
	//		 5     tralp      kinematics    maximum pos angle of attack - deg

	trmach=0.8;
	trdynm=10.e+3;
	trload=3;
	tralp=21;

	trcode=0;
	tmcode=0;
	//-------------------------------------------------------------------------
	//loading module-variables
	hyper[180].gets(trcode);
	hyper[181].gets(tmcode);
	hyper[183].gets(trmach);
	hyper[184].gets(trdynm);
	hyper[185].gets(trload);
	hyper[186].gets(tralp);
}
///////////////////////////////////////////////////////////////////////////////
//Aerodynamic module
//Member function of class 'Hyper'
//
//This module performs the following functions:
// (1) Aerodynamic table look-up from file 'aero_deck_SLV.asc'
// (2) Calculation of aero coefficients in body coordinates
//
//050103 Created by Peter H Zipfel
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::aerodynamics(double int_step)
{
	//local variable
	int thrust_on=false;
	double cn(0);
	double cn0mx(0);

	//local module-variables
	double ca0b(0);
  	double ca(0);
	double cya(0);
	double clma(0);
	double clna(0);
	double cna(0);
	double cy(0);
	double cx(0);
	double cz(0);
	double cll(0);
	double clm(0);
	double cln(0);
	double clde(0);
	double cyb(0);
	double cydr(0);
	double cllda(0);
	double cllp(0);
	double cmde(0);
	double cmq(0);
	double cnb(0);
	double cndr(0);
	double cnr(0);
	double ca0(0);
	double caa(0);
	double cn0(0);
	double clm0(0);
	double clmq(0);
	double gnavail(0);   
	double gyavail(0);
	double gnmax(0);
	double gymax(0);

	//localizing module-variables
	//input data
	int maero=hyper[100].integer();
	double nose_radius=hyper[192].real();
	double alplimx=hyper[187].real();
	double alimitx=hyper[188].real();
	int mwake=hyper[280].integer();
	double refa=hyper[104].real();
	double refd=hyper[105].real();
	double xcg_ref=hyper[108].real();
	//loading saved value
	double cla=hyper[123].real();
	double cma=hyper[133].real();
	double qn=hyper[191].real();
	double qndotmax=hyper[194].real();
	//input from other modules
	double time=round6[0].real();
	double alppx=round6[140].real();
	double phipx=round6[141].real();
	double alphax=round6[144].real();
	double betax=round6[145].real();
	double rho=round6[53].real();
	double vmach=round6[56].real();
	double pdynmc=round6[57].real();
	double tempk=round6[58].real();
	double dvba=round6[75].real();
	double ppx=round6[160].real();
	double qqx=round6[161].real();
	double rrx=round6[162].real();
	double alt=round6[221].real();
	int mprop=hyper[10].integer();
	double vmass=hyper[15].real();
	double xcg=hyper[17].real();
	double trcode=hyper[180].real();
	double trload=hyper[185].real();
	double delax=hyper[619].real();
	double delex=hyper[620].real();
	double delrx=hyper[621].real();
	double delx1=hyper[638].real();
	double delx2=hyper[639].real();
	double delx3=hyper[640].real();
	double delx4=hyper[641].real();
	double delx5=hyper[642].real();
	double delx6=hyper[643].real();
	//-------------------------------------------------------------------------
	//transforming body rates from body -> aeroballistic coord.
	double phip=phipx*RAD;
	double cphip=cos(phip);
	double sphip=sin(phip);
	double qqax=qqx*cphip-rrx*sphip;
	double rrax=qqx*sphip+rrx*cphip;

	//looking up axial force coefficients
	if(maero==13){
		ca0=aerotable.look_up("ca0slv3_vs_mach",vmach);
		caa=aerotable.look_up("caaslv3_vs_mach",vmach);
		ca0b=aerotable.look_up("ca0bslv3_vs_mach",vmach);
	}else if(maero== 12){
		ca0=aerotable.look_up("ca0slv2_vs_mach",vmach);
		caa=aerotable.look_up("caaslv2_vs_mach",vmach);
		ca0b=aerotable.look_up("ca0bslv2_vs_mach",vmach);
	}else if(maero==11){
	   ca0=aerotable.look_up("ca0slv2_vs_mach",vmach);
	   caa=aerotable.look_up("caaslv2_vs_mach",vmach);
	   ca0b=aerotable.look_up("ca0bslv2_vs_mach",vmach);
	}
	//axial force coefficient
	if(mprop) thrust_on=true;
	ca=ca0+caa*alppx+thrust_on*ca0b;

	//looking up normal force coefficients in aeroballistic coord
	if(maero==13){
	   cn0=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alppx);
	}else if (maero==12){
	   cn0=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alppx);
	}else if (maero==11){
	   cn0=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alppx);
	}	
	//normal force coefficient
	cna=cn0;

	//looking up pitching moment coefficients in aeroballistic coord
	if (maero==13){
	   clm0=aerotable.look_up("clm0slv3_vs_mach_alpha",vmach,alppx);
	   clmq=aerotable.look_up("clmqslv3_vs_mach",vmach);
	}else if(maero==12){
	   clm0=aerotable.look_up("clm0slv2_vs_mach_alpha",vmach,alppx);
	   clmq=aerotable.look_up("clmqslv2_vs_mach",vmach);
	}else if(maero==11){
	   clm0=aerotable.look_up("clm0slv1_vs_mach_alpha",vmach,alppx);
	   clmq=aerotable.look_up("clmqslv1_vs_mach",vmach);
	}	
	//pitching moment coefficient
	double clmaref=clm0+clmq*qqax*refd/(2.*dvba);
	clma=clmaref-cna*(xcg_ref-xcg)/refd;

	double alplx(0),alpmx(0);
	double cn0p(0),cn0m(0);
	double clm0p(0),clm0m(0);

	//Non-dimensional derivatives
	//look up coeff at +- 3 deg, but not beyond tables
	alplx=alppx+3.0;
	alpmx=alppx-3.0;
    if(alpmx<0.)alpmx=0.0;

	//calculating normal force dim derivative wrt alpha 'cla'			
	if(maero==13){
	   cn0p=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alplx);
	   cn0m=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alpmx);		
	}else if(maero==12){
	   cn0p=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alplx);
	   cn0m=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alpmx);		
	}else if(maero==11){
	   cn0p=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alplx);
	   cn0m=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alpmx);		
	}		
	//replacing value from previous cycle, only if within max alpha limit
	if(alplx<alplimx)
		cla=(cn0p-cn0m)/(alplx-alpmx);

	//calculating pitch moment dim derivative wrt alpha 'cma'
	if(maero== 13){
	   clm0p=aerotable.look_up("clm0slv3_vs_mach_alpha",vmach,alplx);
	   clm0m=aerotable.look_up("clm0slv3_vs_mach_alpha",vmach,alpmx);
	}else if(maero==12){
	   clm0p=aerotable.look_up("clm0slv2_vs_mach_alpha",vmach,alplx);
	   clm0m=aerotable.look_up("clm0slv2_vs_mach_alpha",vmach,alpmx);
	}else if(maero==11){
	   clm0p=aerotable.look_up("clm0slv1_vs_mach_alpha",vmach,alplx);
	   clm0m=aerotable.look_up("clm0slv1_vs_mach_alpha",vmach,alpmx);
	}
	//replacing value from previous cycle, only if within max alpha limit
	if(alppx<alplimx)
		cma=(clm0p-clm0m)/(alplx-alpmx)-cla*(xcg_ref-xcg)/refd;

	//converting force and moment coeff to body axes
	//force coefficients in body axes
	cx=-ca;
	cy=-cna*sphip;
	cz=-cna*cphip;
	//moment coefficient in body axes
	cll=0;
	clm=clma*cphip;
	cln=-clma*sphip;

	//calculate load factor available for max alpha
	//looking up normal force coefficients in aeroballistic coord
	if(maero==13){
		cn0mx=aerotable.look_up("cn0slv3_vs_mach_alpha",vmach,alplimx);
	}else if(maero==12){
		cn0mx=aerotable.look_up("cn0slv2_vs_mach_alpha",vmach,alplimx);
	}else if(maero==11){
		cn0mx=aerotable.look_up("cn0slv1_vs_mach_alpha",vmach,alplimx);
	}	
	double anlmx=cn0mx*pdynmc*refa;
	double weight=vmass*AGRAV;
	gnmax=anlmx/weight;
	if(gnmax>=alimitx)gnmax=alimitx; 
	double aloadn=cn*pdynmc*refa;
	double gng=aloadn/weight;
	gnavail=gnmax-gng;
	//same load factor in yaw plane
	gymax=gnmax;
	gyavail=gnavail;

	//converting output to be compatible with 'aerodynamics_der()'
	//force
	clde=0.0;
	cyb=-cla;
	cydr=0.0;
	//roll
	cllda=0.0;
	cllp=0.0;
	//pitch
	cmde=0.0;
	cmq=clmq;
	//yaw
	cnb=-cma;
	cndr=0.0;
	cnr=clmq;
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	hyper[191].gets(qn);
	hyper[194].gets(qndotmax);
	hyper[280].gets(mwake);
	//output to other modules
	hyper[104].gets(refa);
	hyper[112].gets(cy);
	hyper[113].gets(cll);
	hyper[114].gets(clm);
	hyper[115].gets(cln);
	hyper[116].gets(cx);
	hyper[117].gets(cz);
	hyper[191].gets(gnmax);
	hyper[192].gets(gymax);
	//output to 'aerodynamics_der()'
	hyper[123].gets(cla);
	hyper[124].gets(clde);
	hyper[125].gets(cyb);
	hyper[127].gets(cydr);
	hyper[129].gets(cllda);
	hyper[130].gets(cllp);
	hyper[133].gets(cma);
	hyper[134].gets(cmde);
	hyper[135].gets(cmq);
	hyper[136].gets(cnb);
	hyper[138].gets(cndr);
	hyper[140].gets(cnr);
	//diagnostics
	hyper[118].gets(ca0);
	hyper[119].gets(caa);
	hyper[120].gets(cn0);
	hyper[121].gets(clm0);
	hyper[122].gets(clmq);
	hyper[189].gets(gnavail);
	hyper[190].gets(gyavail);
	//-------------------------------------------------------------------------	
	//function call for derivative calculations
	aerodynamics_der();
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
	double stmarg_pitch(0);
	double stmarg_yaw(0);
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
	double refd=hyper[105].real();
	//from other modules
	double time=round6[0].real();
	double vmach=round6[56].real();
	double pdynmc=round6[57].real();
	double dvba=round6[75].real();
	double vmass= hyper[15].real();
	double xcg= hyper[17].real();
	double thrust= hyper[26].real();
	Matrix IBBB=hyper[18].mat(); 
	int mtvc=hyper[900].integer();
	double gtvc=hyper[908].real();
	double parm= hyper[909].real();
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
	double cnb=hyper[136].real();
	double cndr=hyper[138].real();
	double cnr=hyper[140].real();
	//-------------------------------------------------------------------------
	//MOI components
	double ibbb11=IBBB.get_loc(0,0);
	double ibbb22=IBBB.get_loc(1,1);
	double ibbb33=IBBB.get_loc(2,2);
	//Dimensional derivatives for pitch plane (converted to 1/rad where required)
	double duml=(pdynmc*refa/vmass)/RAD;
	dla=duml*cla;
	dlde=duml*clde;
	double dumm=pdynmc*refa*refd/ibbb22;
	dma=dumm*cma/RAD;
	dmq=dumm*(refd/(2*dvba))*cmq;
	dmde=dumm*cmde/RAD;

	//Dimensional derivatives in plane (converted to 1/rad where required)
	double dumy=pdynmc*refa/vmass;
	dyb=dumy*cyb/RAD;
	dydr=dumy*cydr/RAD;
	double dumn=pdynmc*refa*refd/ibbb33;
	dnb=dumn*cnb/RAD;
	dnr=dumn*(refd/(2*dvba))*cnr;
	dndr=dumn*cndr/RAD;

	//Dimensional derivatives in roll (converted to 1/rad where required)
	double dumll=pdynmc*refa*refd/ibbb11;
	dllp=dumll*(refd/(2*dvba))*cllp;
	dllda=dumll*cllda/RAD;

	//TVC control derivatives
	if(mtvc==1||mtvc==2||mtvc==3){
		//pitch plane
		dlde=gtvc*thrust/vmass;
		dmde=-(parm-xcg)*gtvc*thrust/IBBB.get_loc(2,2);
		//yaw plane
		dydr=dlde;
		dndr=dmde;
	}
	//static margin in pitch (per chord length 'refd')
	if(cla) stmarg_pitch=-cma/cla;

	//static margin in yaw (per span length 'refd')
	if(cyb) stmarg_yaw=-cnb/cyb;

	//diagnostics: pitch plane roots		
	double a11=dmq;
	double a12(0);
	if(dla)
		a12=dma/dla;
	double a21=dla;
	double a22=-dla/dvba;
	
	double arg=pow((a11+a22),2)-4.*(a11*a22-a12*a21);
	if(arg>=0.)
	{
		wnp=0.;
		zetp=0.;
		double dum=a11+a22;
		realp1=(dum+sqrt(arg))/2;
		realp2=(dum-sqrt(arg))/2;
		rpreal=(realp1+realp2)/2;
	}
	else
	{
		realp1=0.;
		realp2=0.;
		wnp=sqrt(a11*a22-a12*a21);
		zetp=-(a11+a22)/(2*wnp);
		rpreal=-zetp*wnp;
	}	
	//diagnostics: yaw plane roots		
	a11=dnr;
	if(dyb)
		a12=dnb/dyb;
	else
		a12=0;
	a21=-dyb;
	a22=dyb/dvba;
	arg=pow((a11+a22),2)-4*(a11*a22-a12*a21);
	if(arg>=0.)
	{
		wny=0.;
		zety=0.; 
		double dum=a11+a22;
		realy1=(dum+sqrt(arg))/2;
		realy2=(dum-sqrt(arg))/2;
		ryreal=(realy1+realy2)/2;
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
	hyper[143].gets(stmarg_yaw);
	hyper[144].gets(stmarg_pitch);
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

