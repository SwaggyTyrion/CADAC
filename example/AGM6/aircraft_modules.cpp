///////////////////////////////////////////////////////////////////////////////
//FILE: 'aircraft_modules.cpp'
//
//Contains all Modules of class 'Aircraft'
//						guidance()	aircraft[10-19]
//						control()	aircraft[20-38]
//						forces()	aircraft[39]
//						sensor()	aircraft[40-59]
// Generally used variables are assigned to aircraft[0-9] 
//
//This is the blue aircraft tracking the red target
//
//Capability of blue aircraft
//	(1) straight and level
//	(2) horizontal turn
//	(3) escape maneuver 
//	(4) (1) through (3) can be supplemented with longitudinal acceleraton (see 'forces' module)
//
//070411 Created by Peter H Zipfel
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
	//escape maneuver from red target
	if(acft_option==2){
		//downloading from 'combus' red target states
		int i(0);
		char number[4];	
		Matrix STEL(3,1);
		Matrix VTEL(3,1);
		//building target id = t(j+1); first red target is used
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
		//distance to target
		Matrix SATL(3,1);
		//displacement vector of blue aircraft wrt red target
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

		//escape maneuver; aircraft aligns with red target velocity vector
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
//Provides the specific forces acting on the blue aircraft
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
///////////////////////////////////////////////////////////////////////////////
//Definition of seeler module-variables 
//Member function of class 'Aircraft'
//Module-variable locations are assigned to aircraft[40-59]
//
//070523 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Aircraft::def_sensor()
{
	//Definition of module-variables
//	aircraft[40].init("track_on","int",0,"Aircraft targeting transmission, =0:off; =1:on - ND","sensor","data","");
	aircraft[41].init("init_flag","int",true,"Initialization flag, =0:off; =1:on - ND","sensor","data","");
	aircraft[42].init("track_epoch",0,"Tracking epoch - s","sensor","save","");
	aircraft[43].init("track_step",0,"Tracking time interval - s","sensor","data","");
	aircraft[44].init("target_num","int",0,"Target 3 in combus- ND","sensor","save","");
    aircraft[46].init("STCEL1",0,0,0,"Position track file of target #1","sensor","out","com");
    aircraft[47].init("VTCEL1",0,0,0,"Velocity track file of target #1","sensor","out","com");
    aircraft[48].init("STCEL2",0,0,0,"Position track file of target #2","sensor","out","com");
    aircraft[49].init("VTCEL2",0,0,0,"Velocity track file of target #2","sensor","out","com");
    aircraft[50].init("STCEL3",0,0,0,"Position track file of target #3","sensor","out","com");
    aircraft[51].init("VTCEL3",0,0,0,"Velocity track file of target #3","sensor","out","com");
    aircraft[52].init("STCEL4",0,0,0,"Position track file of target #4","sensor","out","");
    aircraft[53].init("VTCEL4",0,0,0,"Velocity track file of target #4","sensor","out","");
    aircraft[54].init("STCEL5",0,0,0,"Position track file of target #5","sensor","out","");
    aircraft[55].init("VTCEL5",0,0,0,"Velocity track file of target #5","sensor","out","");
	aircraft[56].init("dat_sigma",0,"1 sigma error of distance measurement - m","sensor","data","");
	aircraft[57].init("azat_sigma",0,"1 sigma error of azimuth measurement - rad","sensor","data","");
	aircraft[58].init("elat_sigma",0,"1 sigma error of elevation measurement - rad","sensor","data","");
	aircraft[59].init("vel_sigma",0,"1 sigma error of velocity component meas. - m/s","sensor","data","");
}
///////////////////////////////////////////////////////////////////////////////
//Sensor (Targeting Radar) Module 
//Member function of class 'Aircraft'
//
// * Measures LOS to target, corrupted by errors
// * Measures velocity of target, corrupted by errors
// * Loads measurements onto 'combus' 
//
// Notes:*The measurements are taken at large intervals wrt the 
//		  integration step. There is no correlation between measurements.
//		  Therefore, a Gaussian corruption taken at each measurement is good modeling
//        practice (instead of Markov).
//		 *The target measurements are stored in target track files, which are 
//        loaded onto 'combus', LIMITED TO THE FIRST 5 TARGETS in the 'vehicle_list'.
//        Each track file consists of target position and velocity vectors STCELx, VTCELx
//		  where 'x' is the sequential number of the target as loaded into 'input.asc'
//
//070523 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Aircraft::sensor(Packet *combus,int num_vehicles,double sim_time,double int_step)
{
	//local variables
	Variable *data_t;
	Matrix STEL(3,1);
	Matrix VTEL(3,1);
	Matrix SATL(3,1);
	Matrix POLAR(3,1);
	Matrix STCEL(3,1);
	Matrix VTCEL(3,1);
	Matrix SATCL(3,1);

	//local module-variables

	//localizing module-variables
	//input data
	int init_flag=aircraft[41].integer();
	int target_num=aircraft[44].integer();
	double track_step=aircraft[43].real();
	double dat_sigma=aircraft[56].real();
	double azat_sigma=aircraft[57].real();
	double elat_sigma=aircraft[58].real();
	double vel_sigma=aircraft[59].real();
	//getting saved data
	double track_epoch=aircraft[42].real();
	Matrix STCEL1=aircraft[46].vec();
	Matrix VTCEL1=aircraft[47].vec();
	Matrix STCEL2=aircraft[48].vec();
	Matrix VTCEL2=aircraft[49].vec();
	Matrix STCEL3=aircraft[50].vec();
	Matrix VTCEL3=aircraft[51].vec();
	Matrix STCEL4=aircraft[52].vec();
	Matrix VTCEL4=aircraft[53].vec();
	Matrix STCEL5=aircraft[54].vec();
	Matrix VTCEL5=aircraft[55].vec();
	//from other modules
	Matrix SAEL=flat3[26].vec();
	//-------------------------------------------------------------------------
	//storing tracking time initially 
	if(init_flag){
	init_flag=0;
		track_epoch=sim_time;
	}

	//measuring parameters of targets
	if(sim_time>= track_epoch){

		//next tracking epoch
		track_epoch=sim_time+track_step;

		//cycling through all vehicles to find the target 
		//initializing target counter
		target_num=1;
		//finding slot 'i' of target in 'combus' (same as in vehicle_list)
		for(int i=0;i<num_vehicles;i++){

			//downloading from 'combus' target variables
			//getting vehicle id
			string id=combus[i].get_id();
			//building target id = t(j+1)
			char number[4];	
			sprintf(number,"%i",target_num);
			string target_id="t"+string(number);

			if (id==target_id){

				//downloading data from target packet
				data_t=combus[i].get_data();
				STEL=data_t[2].vec();
				VTEL=data_t[3].vec();

				//true polar coordinates to target
				SATL=SAEL-STEL;
				POLAR=SATL.pol_from_cart();
				double dat=POLAR.get_loc(0,0);
				double azat=POLAR.get_loc(1,0);
				double elat=POLAR.get_loc(2,0);

				//corrupting true values to obtain polar postion measurements
				double dat_meas=dat+gauss(0,dat_sigma);
				double azat_meas=azat+gauss(0,azat_sigma);
				double elat_meas=elat+gauss(0,elat_sigma);

				//converting to cartesian coordinates
				SATCL.cart_from_pol(dat_meas,azat_meas,elat_meas);
				STCEL=SAEL-SATCL;

				//noise corruption of velocity measurements
				double vtcel1=VTEL.get_loc(0,0)+gauss(0,vel_sigma);
				double vtcel2=VTEL.get_loc(1,0)+gauss(0,vel_sigma);
				double vtcel3=VTEL.get_loc(2,0)+gauss(0,vel_sigma);
				VTCEL.build_vec3(vtcel1,vtcel2,vtcel3);
				
				//building track files
				switch(target_num){
				case 1:
					STCEL1=STCEL;
					VTCEL1=VTCEL;
					break;
				case 2:
					STCEL2=STCEL;
					VTCEL2=VTCEL;
					break;
				case 3:
					STCEL3=STCEL;
					VTCEL3=VTCEL;
					break;
				case 4:
					STCEL4=STCEL;
					VTCEL4=VTCEL;
					break;
				case 5:
					STCEL5=STCEL;
					VTCEL5=VTCEL;
					break;
				}
				//target counter
				target_num++;

				//discarding targets #6 and higher
				if(target_num>5)break;

			}//track file of one target loaded
		}//all track files loaded
	}//end of target data measurements 
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving data
	aircraft[41].gets(init_flag);
	aircraft[42].gets(track_epoch);
	aircraft[44].gets(target_num);
	aircraft[46].gets_vec(STCEL1);
	aircraft[47].gets_vec(VTCEL1);
	aircraft[48].gets_vec(STCEL2);
	aircraft[49].gets_vec(VTCEL2);
	aircraft[50].gets_vec(STCEL3);
	aircraft[51].gets_vec(VTCEL3);
	aircraft[52].gets_vec(STCEL4);
	aircraft[53].gets_vec(VTCEL4);
	aircraft[54].gets_vec(STCEL5);
	aircraft[55].gets_vec(VTCEL5);
}

