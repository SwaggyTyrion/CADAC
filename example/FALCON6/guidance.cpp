///////////////////////////////////////////////////////////////////////////////
//FILE: 'guidance.cpp'
//Contains 'guidance' module of class 'Plane'
//
//030816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of guidance module-variables
//Member function of class 'Plane'
//Module-variable locations are assigned to plane[400-499]
//
// mguid=|mguidl|mguidp|
//	      mguidl= 0 No lateral guidance (default)
//	            = 3 Lateral line guidance - waypoint guidance (input)
//			     mguidp= 0 No pitch guidance (default)
//					   = 3 Pitch line guidance (input)
//		
// * Waypoint guidance: mguid=30
//	 The vehicle flies lateral line guidance towards the waypoint,
//    given by 'swel1' and 'swel2' (third component is disregarded) with the heading 'psiflx'
//    Waypoints  are sequenced in 'input.asc'.
//	 'wp_flag' is the event criteria. It switches from +1 (closing) to -1 (fleeting)
//    based on closing velocity computation
// * Descent to IP: mguid=33
//	 The vehicle descends to an IP, given by 'swel1', 'swel2' and 'swel3', with the  prescribed
//	  heading 'psiflx' and glide slop 'thtflx'
// * Special case: Point guidance if line_gain=0. Vehicle flies pursuit guidance towards the 
//    waypoint or IP
//
//030816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Plane::def_guidance()
{
	//definition of module-variables
	plane[400].init("mguid","int",0,"Switch for guidance options - ND","guidance","data","scrn");
	plane[402].init("line_gain",0,"Line guidance gain - 1/s","guidance","data","");
	plane[403].init("nl_gain_fact",0,"Nonlinear gain factor - ND","guidance","data","");
	plane[404].init("decrement",0,"distance decrement - m","guidance","data","");
	plane[405].init("swel1",0,"North coordiante of way point - m","guidance","data","");
	plane[406].init("swel2",0,"East coordinate of way point - m","guidance","data","");
	plane[407].init("swel3",0,"Altitude of way point - m","guidance","data","");
	plane[408].init("psiflx",0,"Heading line-of-attack angle - deg","guidance","data","");
	plane[409].init("thtflx",0,"Pitch line-of-attack angle - deg","guidance","data","");
	plane[411].init("dwb",0,"Slant range to waypoint - m","guidance","diag","");
	plane[412].init("nl_gain",0,"Nonlinear gain - rad","guidance","diag","scrn,plot");
	plane[413].init("VBEO",0,0,0,"Vehicle velocity in LOS coordinats - m/s","guidance","diag","");
	plane[414].init("VBEF",0,0,0,"Vehicle velocity in LOA coordinats - m/s","guidance","diag","");
	plane[415].init("dwbh",0,"Ground range to waypoint - m","guidance","diag","scrn,plot");
	plane[416].init("SWBL",0,0,0,"Vehicle wrt waypoint/target in geo coor - m","guidance","diag","");
	plane[417].init("turn_min",0,"Minimum turning radius - m","guidance","diag","");
	plane[419].init("wp_flag","int",0,"=1:closing on trgt; =-1:fleeting; =0:outside - ND","guidance","diag","plot");
}
///////////////////////////////////////////////////////////////////////////////  
//Guidance module
//Member function of class 'Plane'
//
// mguid=|mguidl|mguidp|
//	      mguidl= 0 No lateral guidance (default)
//	            = 3 Lateral line guidance - waypoint guidance (input)
//			     mguidp= 0 No pitch guidance (default)
//					   = 3 Pitch line guidance (input)
//		
//030816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
	
void Plane::guidance()
{
	//local variables
	int mguidl(0),mguidp(0);
	Matrix SWEL(3,1);
	Matrix ALGV(3,1);

	//local module-variables
	double phicomx(0);
	double ancomx(0);
	double alcomx(0);
	double dwb(0);
	double dwbh(0);
	Matrix SWBL(3,1);

	//localizing module-variables
	//input data
	int mguid=plane[400].integer();
	double swel1=plane[405].real();
	double swel2=plane[406].real();
	double swel3=plane[407].real();
	double psiflx=plane[408].real();
	double thtflx=plane[409].real();
	//input from other modules
	double time=flat6[0].real();
	int halt=flat6[1].integer();
	double grav=flat6[55].real();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	double dvbe=flat6[236].real();
	double psivlx=flat6[240].real();
	double thtvlx=flat6[241].real();
	double philimx=plane[511].real();
	//-------------------------------------------------------------------------

	//returning if no guidance
	if(mguid==0) return;

	//decoding guidance switch
	mguidl=mguid/10;
	mguidp=mguid%10;

	//vehicle wrt waypoint displacement
	SWEL.build_vec3(swel1,swel2,swel3);
	SWBL=SWEL-SBEL;
	dwb=SWBL.absolute();
	dwbh=sqrt(SWBL[0]*SWBL[0]+SWBL[1]*SWBL[1]);

	//setting new-waypoint flag if vehicle is within 2x min turning radius
	int wp_flag=1;
	double turn_min=dvbe*dvbe/(grav*tan(philimx*RAD));

	if(dwbh<(2*turn_min)){
		double closing=(SWBL[0]*VBEL[0]+SWBL[1]*VBEL[1]);
		//waypoint flag: wp_flag= +1. (closing); wp_flag=-1. (fleeting)
         wp_flag=sign(closing);
//f16f11_2 - start
		if(wp_flag<0&&wp_flag==-1){
			double closest=dwb;
			if(mguid==30){
				cout<<" *** Closest horizontal approach to waypoint dwbh = "<<dwbh<<" m\n";
				cout<<"     Heading angle psivlx = "<<psivlx<<" deg\n";
			}
			if(mguidp==3||mguid==33){
				cout<<" *** Closest approach to IP dwb = "<<dwb<<" m\n";
				cout<<"     Heading angle psivlx = "<<psivlx<<" deg\n";
				cout<<"     Flight path angle thtvlx = "<<thtvlx<<" deg\n";
			}
			//stop run if halt=1
			if(halt) system("pause");
		}
//f16f11_2 - end
	}
	//calling line guidance
	ALGV=guidance_line(SWBL,psiflx,thtflx);
	if(mguidl==3)alcomx=ALGV[1]/grav;
	if(mguidp==3)ancomx=-ALGV[2]/grav;
	//-------------------------------------------------------------------------
	//loading module-variables
	plane[411].gets(dwb);
	plane[415].gets(dwbh);
	plane[416].gets_vec(SWBL);
	plane[417].gets(turn_min);
	plane[419].gets(wp_flag);
	plane[522].gets(alcomx);
	plane[523].gets(ancomx);
}
///////////////////////////////////////////////////////////////////////////////
//Guidance to a line
//Against stationary waypoints or IPs
//
//parameter input
//		SWBL=waypoint wrt vehicle coordinates - m
//		psiflx= heading of LOA from north - deg
//		thtflx= elevation of LOA from target horizontal plane (up positive) - deg
//return output:
//		ALGV(3x1)=acceleration demanded by line guidance in flight path  coord. - m/s^2
// where:
//		alcomx=ALGV[1]/grav, lateral acceleration command - g's
//		ancomx=-ALGV[2]/grav, normal acceleration command - g's
//
//030816 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Plane::guidance_line(Matrix SWBL,double psiflx,double thtflx)
{
	//local module-variables
	double nl_gain(0);
	Matrix VBEO(3,1);
	Matrix VBEF(3,1);

	//localizing module-variables
	//input data
	double line_gain=plane[402].real();
	double nl_gain_fact=plane[403].real();
	double decrement=plane[404].real();
	//input from other modules
	double time=flat6[0].real(); 
	double grav=flat6[55].real();
	Matrix VBEL=flat6[233].vec();
	double thtvlx=flat6[241].real();
	//-------------------------------------------------------------------------
	//TM of LOA wrt local level axes
	Matrix TFL=mat2tr(psiflx*RAD,thtflx*RAD);

	//building TM of LOS wrt local level axes; also getting range-to-go to waypoint
	Matrix POLAR=SWBL.pol_from_cart();
	double wp_sltrange=POLAR[0];
	double psiol=POLAR[1];
	double thtol=POLAR[2];
	Matrix TOL=mat2tr(psiol,thtol);

	//converting geographic velocity to LOS and LOA coordinates
	VBEO=TOL*VBEL;
	double vbeo2=VBEO[1];
	double vbeo3=VBEO[2];

	VBEF=TFL*VBEL;
	double vbef2=VBEF[1];
	double vbef3=VBEF[2];

	//nonlinear gain
	nl_gain=nl_gain_fact*(1-exp(-wp_sltrange/decrement));

	//line guidance steering law
	double algv1=grav*sin(thtvlx*RAD);
	double algv2=line_gain*(-vbeo2+nl_gain*vbef2);
	double algv3=line_gain*(-vbeo3+nl_gain*vbef3)-grav*cos(thtvlx*RAD);

	//packing accelerations into vector for output
	Matrix ALGV(3,1);
	ALGV.build_vec3(algv1,algv2,algv3);
	//-------------------------------------------------------------------------
	//loading dignostic module-variables
	plane[411].gets(wp_sltrange);
	plane[412].gets(nl_gain);
	plane[413].gets_vec(VBEO);
	plane[414].gets_vec(VBEF);

	return ALGV;
}

