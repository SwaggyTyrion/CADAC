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
//051207 Modified for TVC, PZi
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
	int mprop=hyper[10].integer();
	double thrust=hyper[26].real();
	int mrcs_moment=hyper[50].integer();
	int mrcs_force=hyper[51].integer();
	int mwake=hyper[280].integer();
	Matrix FMRCS=hyper[64].vec();
	Matrix FARCS=hyper[84].vec();
	double refa=hyper[104].real();
    double refd=hyper[105].real();
	double cy=hyper[112].real();
	double cll=hyper[113].real();
	double clm=hyper[114].real();
	double cln=hyper[115].real();
	double cx=hyper[116].real();
	double cz=hyper[117].real();
	int mtvc=hyper[900].integer();
	Matrix FPB=hyper[910].vec();
	Matrix FMPB=hyper[911].vec();
	//-------------------------------------------------------------------------
	//total non-gravitational forces
	FAPB.assign_loc(0,0,pdynmc*refa*cx);
	FAPB.assign_loc(1,0,pdynmc*refa*cy);
	FAPB.assign_loc(2,0,pdynmc*refa*cz);

	//aerodynamic moment
	FMB.assign_loc(0,0,pdynmc*refa*refd*cll);
	FMB.assign_loc(1,0,pdynmc*refa*refd*clm);
	FMB.assign_loc(2,0,pdynmc*refa*refd*cln);

	//adding thrust modified by TVC or otherwise just plain thrust if mprop > 0
	if(mtvc==1||mtvc==2||mtvc==3){
		FAPB=FAPB+FPB;
		FMB=FMB+FMPB;
	}
	else if(mprop)
		FAPB[0]=FAPB[0]+thrust;

	//adding force components from RCS
	if(mrcs_force==1||mrcs_force==2)
		FAPB=FAPB+FARCS;

	//adding moment components from RCS
	if(mrcs_moment>0&&mrcs_moment<=23)
		FMB=FMB+FMRCS;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	round6[200].gets_vec(FAPB);
	round6[201].gets_vec(FMB);
}	
