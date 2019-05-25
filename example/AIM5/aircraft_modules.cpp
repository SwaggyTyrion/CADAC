///////////////////////////////////////////////////////////////////////////////
//FILE: 'aircraft_modules.cpp'
//
//Contains all Modules of class 'Aircraft'
//						guidance()	aircraft[10-19]
//						control()	aircraft[20-38]
//						forces()	aircraft[39]
// Generally used variables are assigned to aircraft[0-9] 
//
//This is the target aircraft attacked by the air intercept missile
//
//Capability of target aircraft
//	(1) straight and level
//	(2) horizontal turn
//	(3) escape maneuver from red missile
//	(4) (1) through (3) can be supplemented with longitudinal acceleraton (see 'forces' module)
//
//070411 Created by Peter H Zipfel
//130725 Building AIM5, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'guidance' module-variables
//Member function of class 'Aircraft'
//Module-variable locations are assigned to aircraft[10-19]
//
// acft_option = 0 flying out only with initial conditions
//				 1 horizontal g-maneuver (specify 'gturn')
//				 2 inital fly out and escape maneuver
//
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::def_guidance()
{
	//Definition of module-variables
	aircraft[10].init("acft_option","int",0,"=0:steady; =1:hor g-manvr, alpha limtd; =2:escape - ND","guidance","data","");
	aircraft[11].init("guid_gain",0,"Guidance gain for escaping aircraft maneuvers - ND","guidance","data","");
	aircraft[12].init("ACOML",0,0,0,"Commanded accel in loc lev coord - m/s^2","guidance","out","");
	aircraft[13].init("gturn",0,"G-accel for horiz turn (+ right, - left) - g's","guidance","data","");
}
///////////////////////////////////////////////////////////////////////////////
//'guidance' module 
//Member function of class 'Aircraft' 
//Calculating guidance commands for combat and escape maneuvers
//
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::guidance(Packet *combus,int num_vehicles)
{
	//local variables
	Variable *data_t=NULL;
	int tgt_com_slot(0);

	//local module-variables
	Matrix ACOML(3,1);

	//localizing module-variables
	//input data
	int acft_option=aircraft[10].integer();
	double guid_gain=aircraft[11].real();
	double gturn=aircraft[13].real();
	//input from other modules
	double grav=flat3[11].real();
	Matrix TVL=flat3[24].mat(); 
	Matrix SAEL=flat3[26].vec();
	Matrix VAEL=flat3[27].vec();
	//-------------------------------------------------------------------------
	//steady, straight, horizontal flight
	if(acft_option==0){
		//gravity bias for horizontal flight
		ACOML.build_vec3(0,0,-grav);
	}
	//horizontal g-turn
	if(acft_option==1){
		Matrix ACOMV(3,3);
		ACOMV.build_vec3(0,gturn*grav,-grav);
		ACOML=~TVL*ACOMV;
	}
	//escape maneuver from red missile
	if(acft_option==2){
		//downloading from 'combus' red missile states
		int i(0);
		char number[4];	
		Matrix STEL(3,1);
		Matrix VTEL(3,1);
		//building target id = t(j+1); first red missile is used
		sprintf(number,"%i",1);
		string target_id="t"+string(number);
		//finding slot 'i' of target in 'combus' (same as in vehicle_list)
		for(i=0;i<num_vehicles;i++){
			string id=combus[i].get_id();
			if (id==target_id){
				//downloading data from target packet
				tgt_com_slot=i;
				data_t=combus[i].get_data();
				STEL=data_t[1].vec();
				VTEL=data_t[2].vec();
			}
		}
		//distance to missile
		Matrix SATL(3,1);
		//displacement vector of target aircraft wrt air intercept missile
		SATL=SAEL-STEL;
		double dab=SATL.absolute();

		//calculating guidance constant
		double dum=(VAEL.skew_sym()*VTEL).absolute();
		double gain=guid_gain*dum/dab;

		//unit vectors
		double dvbe=VTEL.absolute();
		Matrix UVTEL(3,1);
		Matrix UVAEL(3,1);
		UVTEL=VTEL*(1/dvbe);
		double dvae=VAEL.absolute();
		UVAEL=VAEL*(1/dvae);

		//escape maneuver; aircraft aligns with red missile velocity vector
		//acceleration command from cross product steering guidance law 
		Matrix EPSL=UVAEL.skew_sym()*UVTEL;
		ACOML=EPSL.skew_sym()*UVAEL*gain;

		//including g-bias
		Matrix GRAVL(3,1);
		GRAVL.build_vec3(0,0,-grav);
		ACOML+=GRAVL;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	aircraft[12].gets_vec(ACOML);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'control' module-variables
//Member function of class 'Aircraft'
//Module-variable locations are assigned to aircraft[20-39]
//
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::def_control()
{
	//Definition of module-variables
	aircraft[20].init("phiav",0,"Aircraft aircraft bank angle - rad","control","state","");
	aircraft[21].init("phiavd",0,"Derivative of bank angle - rad/s","control","state","");
	aircraft[22].init("tphi",0,"Time lag constant of bank angle - sec","control","data","");
	aircraft[23].init("philimx",0,"Bank angle limiter - deg","control","data","");
	aircraft[24].init("phiavx",0,"Bank angle - deg","control","out","");
	aircraft[25].init("phiavcx",0,"Commanded bank angle - deg","control","diag","");
	aircraft[26].init("anx",0,"Normal load factor - g's","control","state","");
	aircraft[27].init("anxd",0,"Normal load factor derivative - g's/s","control","state","");
	aircraft[28].init("tanx",0,"Time lag constant of normal load factor  - sec","control","data","");
	aircraft[29].init("alplimx",0,"Angle of attack limiter - deg","control","data","");
	aircraft[30].init("ancomx",0,"Commanded load factor - g's","control","diag","");
	aircraft[31].init("clalpha",0,"Aircraft lift slope - 1/deg","control","data","");
	aircraft[32].init("wingloading",0,"Aircraft wing loading - N/m^2","control","data","");
	//feedback path to 'Flat3'
	flat3[21].init("phiavout",0,"Aircraft aircraft bank angle - rad","control","out","");
}
///////////////////////////////////////////////////////////////////////////////
//'control' Module 
//Member function of class 'Aircraft' 
// Delays guidance commands
// Limits normal load factor and bank angle
// Outputs normal load factor and bank angle achieved by aircraft 
//
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::control(double int_step)
{
	//local variables
	double phiavc(0);

	//local module-variables
	double phiavcx(0);
	double phiavx(0);
	double ancomx(0);
	double phiavout(0);

	//localizing module-variables
	//input data
	double tphi=aircraft[22].real();
	double philimx=aircraft[23].real();
	double tanx=aircraft[28].real();
	double alplimx=aircraft[29].real();
	double clalpha=aircraft[31].real();
	double wingloading=aircraft[32].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	Matrix TVL=flat3[24].mat(); 
	int acft_option=aircraft[10].integer();
	Matrix ACOML=aircraft[12].vec();
	//state variables
	double phiav=aircraft[20].real();
	double phiavd=aircraft[21].real();
	double anx=aircraft[26].real();
	double anxd=aircraft[27].real();
	//-------------------------------------------------------------------------
	//converting accel command to aircraft coord
	Matrix ACOMV=TVL*ACOML;

	//calculating bank angle
	double acoma2=ACOMV.get_loc(1,0);
	double acoma3=ACOMV.get_loc(2,0);
	if(fabs(acoma2)<EPS&&fabs(acoma3)<EPS)
		phiavc=0.;
	else
		phiavc=atan2(acoma2,-acoma3);
	//diagnostic
	phiavcx=phiavc*DEG;
	
	//delaying bank angle
	if(tphi){
		double phiavd_new=(phiavc-phiav)/tphi;
		phiav=integrate(phiavd_new,phiavd,phiav,int_step);
		phiavd=phiavd_new;
	}
	else
		phiav=phiavc;

	//limiting bank angle
	phiavx=phiav*DEG;
	if(fabs(phiavx)>=philimx) phiavx=philimx*sign(phiavx);

	//outputting bank angle to 'newton' module
	phiavout=phiavx*RAD;

	//calculating normal load factor
	ancomx=sqrt(acoma2*acoma2+acoma3*acoma3)/grav;

	//delaying normal load factor
	if(tanx){
		double anxd_new=(ancomx-anx)/tanx;
		anx=integrate(anxd_new,anxd,anx,int_step);
		anxd=anxd_new;
	}
	else
		anx=ancomx;
	if(acft_option>0){
		//calculating acceleration limiter based on aircraft alpha limit
		double anlimx=pdynmc*clalpha*alplimx/wingloading;

		//limiting normal load factor
		if(fabs(anx)>=anlimx) anx=anlimx*sign(anx);
	}
  //-------------------------------------------------------------------------
	//loading module-variables
	//state variables and output to other modules
	aircraft[20].gets(phiav);
	aircraft[21].gets(phiavd);
	aircraft[26].gets(anx);
	aircraft[27].gets(anxd);
	//output to 'newton' module
	flat3[21].gets(phiavout);
	//diagnositcs
	aircraft[24].gets(phiavx);
	aircraft[25].gets(phiavcx);
	aircraft[30].gets(ancomx);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'force' module-variables
//Member function of class 'Aircraft'
//Module-variable locations are assigned to aircraft[39]
//
//Note that FSPA is entered into the 'flat3[20]' array because it is needed
// for the 'newton' module, which is a member of the 'Flat3' class
//
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::def_forces()
{
	//Definition of module-variables
	flat3[20].init("FSPA",0,0,0,"Specific force - m/s^2","forces","out","");
	aircraft[39].init("acc_longx",0,"Longitudinal acceleration - g's","forces","data","");
}
///////////////////////////////////////////////////////////////////////////////
//'force' Module 
//Member function of class 'Aircraft' 
//Provides the specific forces acting on the target aircraft
//
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::forces()
{
	//local module-variables
	Matrix FSPA(3,1);

	//localizing module-variables
	//input data
	double acc_longx=aircraft[39].real();
	//input from other modules
	double grav=flat3[11].real();
	double anx=aircraft[26].real();
	//-------------------------------------------------------------------------
	//calculating specific force in airplane coord
	double acoma1=acc_longx*grav;
	double acoma2=0;
	double acoma3=-anx*grav;
	FSPA.build_vec3(acoma1,acoma2,acoma3);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	flat3[20].gets_vec(FSPA);
}


