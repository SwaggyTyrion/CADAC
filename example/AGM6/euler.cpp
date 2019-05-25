///////////////////////////////////////////////////////////////////////////////
//FILE: 'euler.cpp'
//Contains 'euler' module of class 'Flat6'
//
//011128 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030319 Upgraded to SM Item32, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'euler' module-variables 
//Member function of class 'Flat6'
//Module-variable locations are assigned to flat6[150-199]
// 
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat6::def_euler()
{
	//Definition and initialization of module-variables
    flat6[154].init("ppd",0,"Body roll angular velocity derivative - rad/s^2","euler","state","");
    flat6[155].init("pp",0,"Body roll angular velocity - rad/s","euler","state","");
    flat6[156].init("qqd",0,"Body pitch angular velocity derivative - rad/s^2","euler","state","");
    flat6[157].init("qq",0,"Body pitch angular velocity - rad/s","euler","state","");
    flat6[158].init("rrd",0,"Body yaw angular velocity derivative - rad/s^2","euler","state","");
    flat6[159].init("rr",0,"Body yaw angular velocity - rad/s","euler","state","");
    flat6[160].init("ppx",0,"Body roll angular velocity in body axes - deg/s","euler","out","plot");
    flat6[161].init("qqx",0,"Body pitch angular velocity in body axes - deg/s","euler","out","plot");
    flat6[162].init("rrx",0,"Body yaw angular velocity in body axes - deg/s","euler","out","plot");
    flat6[163].init("WBEB",0,0,0,"Ang vel of veh wrt earth, body axes - rad/s","euler","diag","");
}	
///////////////////////////////////////////////////////////////////////////////
//'euler' module
//Member function of class 'Flat6'
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Flat6::euler(double int_step)
{
	//local module-variables
	double ppx(0),qqx(0),rrx(0);
	Matrix WBEB(3,1);

	//localizing module-variables
	//input from other modules
	Matrix FMB=flat6[201].vec();
	double ai11=missile[16].real();
	double ai33=missile[17].real();
	//state variables
	double ppd=flat6[154].real();
	double pp =flat6[155].real();
	double qqd=flat6[156].real();
	double qq =flat6[157].real();
	double rrd=flat6[158].real();
	double rr =flat6[159].real();
	//-------------------------------------------------------------------------
	//integrating angular velocity accelerations to get pp, qq, rr
	double fmb1=FMB.get_loc(0,0);
	double fmb2=FMB.get_loc(1,0);
	double fmb3=FMB.get_loc(2,0);

	double ppd_new=fmb1/ai11;
	pp=integrate(ppd_new,ppd,pp,int_step);
	ppd=ppd_new;

	double qqd_new=((ai33-ai11)*pp*rr+fmb2)/ai33;
	qq=integrate(qqd_new,qqd,qq,int_step);
	qqd=qqd_new;

	double rrd_new=(-(ai33-ai11)*pp*qq+fmb3)/ai33;
	rr=integrate(rrd_new,rrd,rr,int_step);
	rrd=rrd_new;

	//building angular velocity vector
	WBEB.build_vec3(pp,qq,rr);

	//angular rates in deg/s
	ppx=pp*DEG;
	qqx=qq*DEG;
	rrx=rr*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	flat6[154].gets(ppd);	
	flat6[155].gets(pp); 	
	flat6[156].gets(qqd);	
	flat6[157].gets(qq); 	
	flat6[158].gets(rrd);	
	flat6[159].gets(rr); 		
	//output to other modules
	flat6[160].gets(ppx);	
	flat6[161].gets(qqx);	
	flat6[162].gets(rrx);
	flat6[163].gets_vec(WBEB);
}