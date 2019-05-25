///////////////////////////////////////////////////////////////////////////////
//FILE: 'forces.cpp'
//Contains 'forces' module of class 'Plane'
//
//030707 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of forces module-variables 
//Member function of class 'Plane'
//Module-variable locations are assigned to flat6[200-209]
// 
//Note that FAPM and FMB is entered into the flat6[] array because they are needed
// for the 'newton' and 'euler' modules, which are members of the 'Flat6' class
//
//030707 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::def_forces()
{
	//Definition and initialization of module-variables
    flat6[200].init("FAPB",0,0,0,"Aerodynamic and propulsive forces in body axes - N","forces","out","");
    flat6[201].init("FMB",0,0,0,"Aerodynamic and propulsive moments in body axes - N*m","forces","out","");
}	

///////////////////////////////////////////////////////////////////////////////
//Force module
//Member function of class 'Plane'
// Calculates the non-gravitational forces
// Calculates the aerodynamic moments
//
//030707 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::forces()
{
	//local module-variables
	Matrix FAPB(3,1);
	Matrix FMB(3,1);

	//localizing module-variables
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double thrust=plane[55].real(); 
	double refa=plane[104].real();
	double refb=plane[105].real();
	double refc=plane[106].real();
	double cxt=plane[111].real();
	double cyt=plane[112].real();
	double czt=plane[113].real();
	double clt=plane[114].real();
	double cmt=plane[115].real();
	double cnt=plane[116].real();
	//-------------------------------------------------------------------------
	//total non-gravitational forces
	FAPB[0]=pdynmc*refa*cxt+thrust;
	FAPB[1]=pdynmc*refa*cyt;
	FAPB[2]=pdynmc*refa*czt;
	
	//aerodynamic moment
	FMB[0]=pdynmc*refa*refb*clt;
	FMB[1]=pdynmc*refa*refc*cmt;
	FMB[2]=pdynmc*refa*refb*cnt;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	flat6[200].gets_vec(FAPB);
	flat6[201].gets_vec(FMB);
}	
