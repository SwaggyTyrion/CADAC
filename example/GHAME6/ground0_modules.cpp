///////////////////////////////////////////////////////////////////////////////
//FILE: 'ground0_modules.cpp'
//Contains all modules of class 'Ground0'
//							kinematics()
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Defining of kinematics module-variables
//Member function of class 'Ground0'
//Module-variable locations are assigned to ground0[10-19]
//
//Initializing the module-variables
//		
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Ground0::def_kinematics()
{
	//defining module-variables
	ground0[0].init("time",0,"Vehicle time since launch - s","kinematics","diag","");
	ground0[11].init("lonx",0,"Vehicle longitude - deg","kinematics","data","");
	ground0[12].init("latx",0,"Vehicle latitude - deg","kinematics","data","");
	ground0[13].init("alt",0,"Vehicle altitude - m","kinematics","data","");
	ground0[14].init("SBII",0,0,0,"Inertial position - m ","kinematics","state","com");
	ground0[15].init("VBII",0,0,0,"Inertial velocity - m/s ","kinematics","state","com");
	ground0[16].init("dbi",0,"Distance from Earth center - m","kinematics","diag","com");
	ground0[17].init("TIG",0,0,0,0,0,0,0,0,0,"TM of inertial wrt geographic coordinates ","kinematics","init","");
	ground0[18].init("TGE",0,0,0,0,0,0,0,0,0,"Geographic wrt Earth - ND ","kinematics","out","");	
}

///////////////////////////////////////////////////////////////////////////////
//Initial calculations of kinematics module 
//Member function of class 'Ground0'
// 
//Initial calculations
//Initialization of time
//
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Ground0::init_kinematics(double sim_time,double not_used)
{
	//local variables
	Matrix TEG(3,3);
	Matrix TEI(3,3);
	Matrix SBIG(3,1);
	Matrix WEII(3,3);

	//local module-variables
	double time(0);
	double dbi(0);
	Matrix SBII(3,1);
	Matrix VBII(3,1);
	Matrix TGE(3,3);
	Matrix TIG(3,3);

	//localizing module-variables
	//input data
	double lonx=ground0[11].real();
	double latx=ground0[12].real();
	double alt=ground0[13].real();
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;

	//Earth's angular velocity skew-symmetric tensor in inertial coordinates
	WEII.assign_loc(0,1,-WEII3);
	WEII.assign_loc(1,0,WEII3);

	//calculating initial  position in Earth coordinates
	double radius=-(alt+REARTH);
	SBIG.assign_loc(2,0,radius);
	TGE=cad_tge(lonx*RAD,latx*RAD);
	TEG=TGE.trans();
	Matrix SBIE=TEG*SBIG;

	//calculating initial vehicle position in inertial coordinates
	// 'cad_tei' contains adjustment for celestial longitude of Greenwich merid. at time=0
	TEI=cad_tei(time);
	SBII=~TEI*SBIE;
	dbi=SBII.absolute();

	//initializing velocity state variable	
	VBII=WEII*SBII;

	//diagnostic: TM of inertial wrt geopraphic coordinates
	TIG=~TEI*TEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	ground0[0].gets(time);
	ground0[14].gets_vec(SBII);
	ground0[15].gets_vec(VBII);
	ground0[16].gets(dbi);
	ground0[17].gets_mat(TIG);
	ground0[18].gets_mat(TGE);
}
///////////////////////////////////////////////////////////////////////////////
//Kinematic module
//Member function of class 'Ground0'
//
//Calculating inertial position and velocity of a fixed point on the Earth
//
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Ground0::kinematics(double sim_time,double not_used1,double &not_used2,double &not_used3)
{
	//local variables
	Matrix TEG(3,3);
	Matrix TEI(3,3);
	Matrix SBIG(3,1);
	Matrix WEII(3,3);

	//local module-variables
	double time(0);
	double dbi(0);
	Matrix SBII(3,1);
	Matrix VBII(3,1);
	Matrix TGE(3,3);
	Matrix TIG(3,3);

	//localizing module-variables
	//input data
	double lonx=ground0[11].real();
	double latx=ground0[12].real();
	double alt=ground0[13].real();
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;

	//Earth's angular velocity skew-symmetric tensor in inertial coordinates
	WEII.assign_loc(0,1,-WEII3);
	WEII.assign_loc(1,0,WEII3);

	//calculating initial  position in Earth coordinates
	double radius=-(alt+REARTH);
	SBIG.assign_loc(2,0,radius);
	TGE=cad_tge(lonx*RAD,latx*RAD);
	TEG=TGE.trans();
	Matrix SBIE=TEG*SBIG;

	//calculating initial vehicle position in inertial coordinates
	// 'cad_tei' contains adjustment for celestial longitude of Greenwich merid. at time=0
	TEI=cad_tei(time);
	SBII=~TEI*SBIE;
	dbi=SBII.absolute();

	//initializing velocity state variable	
	VBII=WEII*SBII;

	//diagnostic: TM of inertial wrt geopraphic coordinates
	TIG=~TEI*TEG;

	//-------------------------------------------------------------------------
	//loading module-variables
	ground0[0].gets(time);
	ground0[14].gets_vec(SBII);
	ground0[15].gets_vec(VBII);
	ground0[16].gets(dbi);
	ground0[17].gets_mat(TIG);
	ground0[18].gets_mat(TGE);
}



