///////////////////////////////////////////////////////////////////////////////
//FILE: 'forces.cpp'
//Contains 'forces' module of class 'Missile'
//
//030601 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'forces' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to flat6[200-209]
// 
//Note that FAPM and FMB is entered into the flat6[] array because they are needed
// for the newton and euler modules, which are members of the 'Flat6' class
//
//030601 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::def_forces()
{
	//Definition and initialization of module-variables
    flat6[200].init("FAPB",0,0,0,"Aerodynamic and propulsive forces in body axes - N","forces","out","");
    flat6[201].init("FMB",0,0,0,"Aerodynamic and propulsive moments in body axes - N*m","forces","out","");
}	
///////////////////////////////////////////////////////////////////////////////
//'forces' module
//Member function of class 'Missile'
// Calculates the non-gravitational forces
// Calculates the aerodynamic moments
//
//030601 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Missile::forces()
{
	//local module-variables
	Matrix FAPB(3,1);
	Matrix FMB(3,1);

	//localizing module-variables
	//input from other modules
	double time=flat6[0].real();
	double pdynmc=flat6[57].real();
	double thrust=missile[13].real();
	Matrix FMRCS=missile[64].vec();
	Matrix FARCS=missile[84].vec();
	double refl=missile[103].real();
	double refa=missile[104].real();
	double ca=missile[120].real();
	double cy=missile[121].real();
	double cn=missile[122].real();
	double cll=missile[123].real();
	double clm=missile[124].real();
	double cln=missile[125].real();
	int mtvc=missile[700].integer();
	Matrix FPB=missile[710].vec();
	Matrix FMPB=missile[711].vec();
	//-------------------------------------------------------------------------
	//total non-gravitational forces
	FAPB[0]=-pdynmc*refa*ca;
	FAPB[1]=pdynmc*refa*cy;
	FAPB[2]=-pdynmc*refa*cn;

	//adding thrust forces if no TVC
	if(mtvc==0)
		FAPB[0]=FAPB[0]+thrust;
	else
		//adding thrust forces modulated by TVC
		FAPB+=FPB;

	//adding RCS forces if present
	FAPB+=FARCS;

	//total moments
	FMB[0]=pdynmc*refa*refl*cll;
	FMB[1]=pdynmc*refa*refl*clm;
	FMB[2]=pdynmc*refa*refl*cln;

	//adding thrust moments if TVC active
	if(mtvc!=0)
		FMB+=FMPB;

	//adding RCS moments if active
		FMB+=FMRCS;

	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	flat6[200].gets_vec(FAPB);
	flat6[201].gets_vec(FMB);
}	
