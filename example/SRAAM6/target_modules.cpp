///////////////////////////////////////////////////////////////////////////////
//FILE: 'target_modules.cpp'
//
//Contains all Modules of class 'Target'
//						guidance()
//						control()
//						forces()
//						intercept()
//
//020116 Created by Peter Zipfel
//031124 Updated, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of guidance module-variables
//Member function of class 'Target'
//Module-variable locations are assigned to target[10-19]
//
// tgt_option = 0 flying out, maintaining initial conditions
//			  = 1 horizontal constant g-turn (specify 'gturn') 
//			  =	2 inital fly out and, when mseek=4, escape maneuver 
//
//020116 Created by Peter Zipfel
//031124 Updated, PZi
///////////////////////////////////////////////////////////////////////////////

void Target::def_guidance()
{
	//Definition of module-variables
	target[1].init("msl_num","int",0,"Missile tail number attacking this tgt - ND","guidance","data","");
	target[10].init("tgt_option","int",0,"=0:steady manvr; =1 hor g-manvr; =2:escape - ND","guidance","data","");
	target[11].init("guid_gain",0,"Guidance gain for target maneuvers - ND","guidance","data","");
	target[12].init("ACOML",0,0,0,"Commanded accel in loc lev coord - m/s^2","guidance","out","");
	target[13].init("gturn",0,"G-accel for horiz turn (+ right, - left) - g's","guidance","data","");
}
///////////////////////////////////////////////////////////////////////////////
//Target guidance module 
//Member function of class 'Target' 
//Calculating guidance commands for combat and escape maneuvers
//
// tgt_option = 0 flying out, maintaining initial conditions
//			  = 1 horizontal constant g-turn (specify 'gturn') 
//			  =	2 inital fly out and, when mseek=4, escape maneuver 
//
//020116 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::guidance(Packet *combus,int num_vehicles)
{
	//local variables
	Variable *data_msl=NULL;

	//local module-variables
	Matrix ACOML(3,1);

	//localizing module-variables
	//input data
	int msl_num=target[1].integer();
	int tgt_option=target[10].integer();
	double guid_gain=target[11].real();
	double gturn=target[13].real();
	//input from other modules
	double grav=flat3[11].real();
	Matrix TVL=flat3[24].mat();
	Matrix SAEL=flat3[26].vec();
	Matrix VAEL=flat3[27].vec();
	//-------------------------------------------------------------------------
	//steady, straight, horizontal fly-out
	if(tgt_option==0){

		//gravity bias for horizontal flight
		ACOML.build_vec3(0,0,-grav);
	}

	//horizontal g-turn
	if(tgt_option==1){
		Matrix ACOMV(3,3);
		ACOMV.build_vec3(0,gturn*grav,-grav);
		ACOML=~TVL*ACOMV;
	}
	//escape maneuver from missile
	if(tgt_option==2)
	{
		//getting status of mseek
		data_msl=combus[msl_num-1].get_data();
		int mseek=data_msl[6].integer();

		//missile seeker active (mseek=4)
		if(mseek==4){

			//getting missile position and velocity from 'combus'
			Matrix SBEL(3,1);
			Matrix VBEL(3,1);
			SBEL=data_msl[2].vec();
			VBEL=data_msl[3].vec();

			//distance to missile
			Matrix SABL(3,1);
			SABL=SAEL-SBEL;
			double dab=SABL.absolute();

			//calculating guidance constant
			double dum=(VAEL.skew_sym()*VBEL).absolute();
			double gain=guid_gain*dum/dab;

			//unit vectors
			double dvbe=VBEL.absolute();
			Matrix UVBEL(3,1);
			Matrix UVAEL(3,1);
			UVBEL=VBEL*(1/dvbe);
			double dvae=VAEL.absolute();
			UVAEL=VAEL*(1/dvae);

			//escape maneuver; target aircraft aligns with missile velocity vector
			//acceleration comand from cross product steering guidance law 
			Matrix EPSL=UVAEL.skew_sym()*UVBEL;
			ACOML=EPSL.skew_sym()*UVAEL*gain;

			//including g-bias
			Matrix GRAVL(3,1);
			GRAVL.build_vec3(0,0,-grav);
			ACOML+=GRAVL;
		}
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	target[12].gets_vec(ACOML);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of control module-variables
//Member function of class 'Target'
//Module-variable locations are assigned to target[20-39]
//
//020116 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::def_control()
{
	//Definition of module-variables
	target[20].init("phiav",0,"Target aircraft bank angle - rad","control","state","");
	target[21].init("phiavd",0,"Derivative of bank angle - rad/s","control","state","");
	target[22].init("tphi",0.2,"Time lag constant of bank angle - sec","control","data","");
	target[23].init("philimx",120,"Bank angle limiter - deg","control","data","");
	target[24].init("phiavx",0,"Bank angle - deg","control","out","com");
	target[25].init("phiavcx",0,"Commanded bank angle - deg","control","diag","");
	target[26].init("anx",0,"Normal load factor - g's","control","state","com");
	target[27].init("anxd",0,"Normal load factor derivative - g's/s","control","state","");
	target[28].init("tanx",0.1,"Time lag constant of normal load factor  - sec","control","data","");
	target[29].init("alplimx",40,"Angle of attack limiter - deg","control","data","");
	target[30].init("ancomx",0,"Commanded load factor - g's","control","diag","");
	target[31].init("clalpha",0.0523,"Aircraft lift slope - 1/deg","control","data","");
	target[32].init("wingloading",3247,"Aircraft wing loading - N/m^2","control","data","");
	//feedback path to 'Flat3'
	flat3[21].init("phiavout",0,"Target aircraft bank angle - rad","control","out","");
}
///////////////////////////////////////////////////////////////////////////////
//Control Module 
//Member function of class 'Target' 
// Delays guidance commands
// limits normal load factor and bank angle
// outputs normal load factor and bank angle achieved by aircraft 
//
//020116 Created by Peter Zipfel
//030320 Upgraded to SM Item32, PZi
//030704 Replaced TAV by TVL, PZi
//030925 Updated, PZi
///////////////////////////////////////////////////////////////////////////////

void Target::control(double int_step)
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
	double tphi=target[22].real();
	double philimx=target[23].real();
	double tanx=target[28].real();
	double alplimx=target[29].real();
	double clalpha=target[31].real();
	double wingloading=target[32].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	Matrix TVL=flat3[24].mat();
	int tgt_option=target[10].integer();
	Matrix ACOML=target[12].vec();
	//state variables
	double phiav=target[20].real();
	double phiavd=target[21].real();
	double anx=target[26].real();
	double anxd=target[27].real();
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
	if(tgt_option>0){
		//calculating acceleration limiter based on aircraft alpha limit
		double anlimx=pdynmc*clalpha*alplimx/(wingloading*grav);

		//limiting normal load factor
		if(fabs(anx)>=anlimx) anx=anlimx*sign(anx);
	}
  //-------------------------------------------------------------------------
	//loading module-variables
	//state variables and output to other modules
	target[20].gets(phiav);
	target[21].gets(phiavd);
	target[26].gets(anx);
	target[27].gets(anxd);
	//ouput to 'newton' module
	flat3[21].gets(phiavout);
	//diagnositcs
	target[24].gets(phiavx);
	target[25].gets(phiavcx);
	target[30].gets(ancomx);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of force module-variables
//Member function of class 'Target'
//Module-variable locations are assigned to target[40-49]
//
//Note that FSPA is entered into the 'flat3[20]' array because it is needed
// for the 'newton' module, which is a member of the 'Flat3' class
//
//020116 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::def_forces()
{
	//Definition of module-variables
	flat3[20].init("FSPA",0,0,0,"Specific force - m/s^2","forces","out","");
	target[40].init("acc_longx",0,"Longitudinal acceleration - g's","forces","data","");
}

///////////////////////////////////////////////////////////////////////////////
//Force Module 
//Member function of class 'Target' 
//Provides the specific forces acting on the target aircaft
//
//020116 Created by Peter Zipfel
//030704 Removed 'phiav' transformation, PZi
///////////////////////////////////////////////////////////////////////////////

void Target::forces()
{
	//local module-variables
	Matrix FSPA(3,1);

	//localizing module-variables
	//input data
	double acc_longx=target[40].real();
	//input from other modules
	double grav=flat3[11].real();
	double anx=target[26].real();
	//-------------------------------------------------------------------------
	//calculating specific force in airplane coord
	double acoma1=acc_longx*grav;
	double acoma2=0;
	double acoma3=-anx*grav;
	FSPA.build_vec3(acoma1,acoma2,acoma3);
	//-------------------------------------------------------------------------
	//loading module-variables
	//ouput to other modules
	flat3[20].gets_vec(FSPA);
}
