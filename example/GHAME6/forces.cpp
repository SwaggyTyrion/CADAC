///////////////////////////////////////////////////////////////////////////////
//FILE: 'forces.cpp'
//Contains 'forces' module of class 'Hyper'
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of forces module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to round6[200-209]
// 
//Note that FAPM and FMB is entered into the round6[] array because they are needed
// for the 'newton' and 'euler' modules, which are members of the 'Round6' class
//
//011126 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_forces()
{
	//Definition and initialization of module-variables
    round6[200].init("FAPB",0,0,0,"Aerodynamic and propulsive forces in body axes - N","forces","out","");
    round6[201].init("FMB",0,0,0,"Aerodynamic and propulsive moments in body axes - N*m","forces","out","");
}	

///////////////////////////////////////////////////////////////////////////////
//Force module
//Member function of class 'Hyper'
// Calculates the non-gravitational forces
// Calculates the aerodynamic moments
//
//030507 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::forces()
{
	//local module-variables
	Matrix FAPB(3,1);
	Matrix FMB(3,1);

	//localizing module-variables
	//input from other modules
	double time=round6[0].real();
	double pdynmc=round6[57].real();
	double thrust=hyper[26].real();
	Matrix FMRCS=hyper[64].vec();
	Matrix FARCS=hyper[84].vec();
	double refa=hyper[104].real();
    double refb=hyper[105].real();
    double refc=hyper[106].real();
	double cy=hyper[112].real();
	double cll=hyper[113].real();
	double clm=hyper[114].real();
	double cln=hyper[115].real();
	double cx=hyper[116].real();
	double cz=hyper[117].real();
	//-------------------------------------------------------------------------
	//total non-gravitational forces
	FAPB.assign_loc(0,0,pdynmc*refa*cx+thrust);
	FAPB.assign_loc(1,0,pdynmc*refa*cy);
	FAPB.assign_loc(2,0,pdynmc*refa*cz);
	FAPB+=FARCS;

	//aerodynamic moment
	FMB.assign_loc(0,0,pdynmc*refa*refb*cll);
	FMB.assign_loc(1,0,pdynmc*refa*refc*clm);
	FMB.assign_loc(2,0,pdynmc*refa*refb*cln);
	FMB+=FMRCS;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	round6[200].gets_vec(FAPB);
	round6[201].gets_vec(FMB);
}	
