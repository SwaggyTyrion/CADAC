///////////////////////////////////////////////////////////////////////////////
//FILE: 'newton.cpp'
//Contains 'newton' module of class 'Flat6'
//
//020513 Created from SRAAM6 by Peter H Zipfel
//030307 Upgraded to SM Item32, PZi
//060504 Picked up 'newton' module from DRMPN6, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'newton' module-variables 
//Member function of class 'Flat6'
//Module-variable locations are assigned to flat6[200-249]
// 
//020513 Created by Peter H Zipfel
//030319 Upgraded to SM Item32, PZi
//051018 Moved 'time' calculations from 'newton' to 'kinematics' module, PZi
///////////////////////////////////////////////////////////////////////////////
void Flat6::def_newton()
{
	//Definition and initialization of module-variables
    flat6[210].init("VBEBD",0,0,0," Derivative of missile velocity - m/s^2","newton","state","");
    flat6[213].init("VBEB",0,0,0,"Missile velocity in body axes - m/s","newton","state","");
    flat6[216].init("SBELD",0,0,0,"Derivative of missile position - m/s","newton","state","");
    flat6[219].init("SBEL",0,0,0,"Missile pos. wrt point E in local level axes - m","newton","state","scrn,plot,com");
    flat6[220].init("sbel1",0,"Initial north comp of SBEL - m","newton","data","");
    flat6[221].init("sbel2",0,"Initial east comp of SBEL - m","newton","data","");
    flat6[222].init("sbel3",0,"Initial down comp of SBEL - m","newton","data","");
    flat6[230].init("FSPB",0,0,0,"Specific force in body axes - m/s^2","newton","out","");
    flat6[233].init("VBEL",0,0,0,"Missile velocity in local level axes - m/s","newton","out","scrn,com");
    flat6[236].init("dvbe",0,"Missile speed - m/s","newton","in/out","scrn,plot");
    flat6[237].init("alpha0x",0,"Initial angle-of-attack - deg","newton","data","");
    flat6[238].init("beta0x",0,"Initial side slip angle - deg","newton","data","");
    flat6[239].init("hbe",0,"Height above ground - m","newton","out","scrn,plot");
    flat6[240].init("psivlx",0,"Heading angle - deg","newton","diag","scrn,plot");
    flat6[241].init("thtvlx",0,"Vertical flight path angle - deg","newton","diag","scrn,plot");
    flat6[244].init("anx",0,"Normal specific force component - g's","newton","diag","scrn,plot");
    flat6[245].init("ayx",0,"Side specific force component - g's","newton","diag","scrn,plot");
    flat6[246].init("ATB",0,0,0,"Tangential accel, solving Newton's Law - m/s^2","newton","diag","");
    flat6[247].init("mfreeze_newt","int",0,"Saving mfreeze value - ND","newton","save","");
    flat6[248].init("dvbef",0,"Saved speed under mfreeze=1 - m/s","newton","save","");
    flat6[249].init("SLEL",0,0,0,"Launch point - m","newton","out","");
}	

///////////////////////////////////////////////////////////////////////////////
//Initialization of 'newton' module
//Member function of class 'Flat6'
//Initializing missile position and velocities
// 
//020513 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat6::init_newton()
{	
	//local module-variables
	Matrix SBEL(3,1);
	Matrix VBEB(3,1);
	double hbe(0);
	Matrix SLEL(3,1);

	//localizing module-variables
	//input data
	double sbel1=flat6[220].real();
	double sbel2=flat6[221].real();
	double sbel3=flat6[222].real();
	double dvbe=flat6[236].real();
	double alpha0x=flat6[237].real();
	double beta0x=flat6[238].real();
	//input from other modules
	double time=flat6[0].real();
	Matrix TBL=flat6[120].mat();
	//initialization of vbeb
	double salp=sin(alpha0x*RAD);
	double calp=cos(alpha0x*RAD);
	double sbet=sin(beta0x*RAD);
	double cbet=cos(beta0x*RAD);
	//-------------------------------------------------------------------------
	//creating velicity vector in body coord
	VBEB.build_vec3(calp*cbet*dvbe,sbet*dvbe,salp*cbet*dvbe);

	//initialization of vbel
	Matrix VBEL=TBL.trans()*VBEB;

	//creating displacement vector in loval level coord
	SBEL.build_vec3(sbel1,sbel2,sbel3);

	//fixing and saving launch point
	SLEL=SBEL;

	//initialize hbe
	hbe=-SBEL.get_loc(2,0);
	//-------------------------------------------------------------------------
	//loading module-variables
	//initialized output
	flat6[213].gets_vec(VBEB);
	flat6[219].gets_vec(SBEL);
	flat6[233].gets_vec(VBEL);
	flat6[239].gets(hbe);
	//output to other modules
	flat6[249].gets_vec(SLEL);
}
///////////////////////////////////////////////////////////////////////////////
//'newton' module
//Member function of class 'Flat6'
//
//Solving the translational equations of motion using Newton's 2nd Law
//
//011127 Created by Peter H Zipfel
//051018 Moved 'time' calculations from 'newton' to 'kinematics' module, PZi
///////////////////////////////////////////////////////////////////////////////
void Flat6::newton(double int_step)
{
	//local variables
	Matrix GRAVL(3,1);
	double psivl(0);
	
	//local module-variables
	Matrix FSPB(3,1);
	Matrix VBEL(3,1);
	double hbe(0);
	double psivlx(0);
	double thtvlx(0);	
	double anx(0);
	double ayx(0);
	Matrix ATB(3,1);

	//localizing module-variables
	//initialization
	double dvbe=flat6[236].real();
	//getting saved value
	int mfreeze_newt=flat6[247].integer();
	double dvbef=flat6[248].real();
	//input from other modules
	double grav=flat6[55].real();
	Matrix TBL=flat6[120].mat();
	Matrix FAPB=flat6[200].vec();
	Matrix WBEB=flat6[163].vec();
	double vmass=missile[12].real();
	int mfreeze=missile[501].integer();
	//state variables
	Matrix VBEBD=flat6[210].vec();
	Matrix VBEB=flat6[213].vec();
	Matrix SBELD=flat6[216].vec();
	Matrix SBEL=flat6[219].vec();
	//-----------------------------------------------------------------------------
	//calculating tangent acceleration in body coord
	ATB=WBEB.skew_sym()*VBEB;

	//building the gravity vector
	GRAVL.build_vec3(0,0,grav);

	//integrating  acceleration in body coord to obtain velocity
	FSPB=FAPB*(1/vmass);
	Matrix VBEBD_NEW=FSPB-ATB+TBL*GRAVL;
	VBEB=integrate(VBEBD_NEW,VBEBD,VBEB,int_step);
	VBEBD=VBEBD_NEW;

	//integrating velocity in L-coord to obtain displacement
//	VBEL=TBL.trans()*VBEB;
	VBEL=~TBL*VBEB; //alternate transpose
	Matrix SBELD_NEW=VBEL;
	SBEL=integrate(SBELD_NEW,SBELD,SBEL,int_step);
	SBELD=SBELD_NEW;

	//calculating flight path angles
	double vbel1=VBEL.get_loc(0,0);
	double vbel2=VBEL.get_loc(1,0);
	double vbel3=VBEL.get_loc(2,0);
	if(vbel1==0.&&vbel2==0.) 
		psivl=0.;
	else
		psivl=atan2(vbel2,vbel1);
	double thtvl=atan2(-vbel3,sqrt(vbel1*vbel1+vbel2*vbel2));
	psivlx=psivl*DEG;
	thtvlx=thtvl*DEG;

	//missile speed
	dvbe=VBEL.absolute();

	//altitude above Earth
	hbe=-SBEL.get_loc(2,0);

	//diagnostics: accelerations achieved in g's
	anx=-FSPB.get_loc(2,0)/grav;
	ayx=FSPB.get_loc(1,0)/grav;

	//freeze variables for autopilot response calculations
	if(mfreeze==0)
		mfreeze_newt=0;
	else{
		if(mfreeze!=mfreeze_newt){
			mfreeze_newt=mfreeze;
			dvbef=dvbe;
		}
		dvbe=dvbef;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	flat6[210].gets_vec(VBEBD);
	flat6[213].gets_vec(VBEB);
	flat6[216].gets_vec(SBELD);
	flat6[219].gets_vec(SBEL);
	//saving values
	flat6[247].gets(mfreeze_newt);
	flat6[248].gets(dvbef);
	//output to other modules
	flat6[230].gets_vec(FSPB);
	flat6[233].gets_vec(VBEL);
	flat6[236].gets(dvbe);
	flat6[239].gets(hbe);
	//diagnostics
	flat6[240].gets(psivlx);
	flat6[241].gets(thtvlx);
	flat6[244].gets(anx);
	flat6[245].gets(ayx);
	flat6[246].gets_vec(ATB);
}
