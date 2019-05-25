///////////////////////////////////////////////////////////////////////////////
//FILE: 'radar_modules.cpp'
//
//Contains all Modules of class 'Radar'
//	only module 'seeker()' is needed
//
//040517 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of seeler module-variables 
//Member function of class 'Radar'
//Module-variable locations are assigned to radar[10-29]
//
//040517 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Radar::def_seeker()
{
	//Definition of module-variables
	radar[10].init("radar_on","int",0,"Ground radar transmission, =0:off; =1:on - ND","seeker","data","");
	radar[11].init("init_flag","int",1,"Initialization flag, =0:off; =1:on - ND","seeker","data","");
	radar[12].init("track_epoch",0,"Tracking epoch - s","seeker","save","");
	radar[13].init("track_step",0,"Tracking time interval - s","seeker","data","");
	radar[14].init("target_num","int",0,"Target 3 in combus- ND","seeker","save","");
    radar[15].init("STCII1",0,0,0,"Position track file of target #1","seeker","save","com");
    radar[16].init("VTCII1",0,0,0,"Velocity track file of target #1","seeker","save","com");
    radar[17].init("STCII2",0,0,0,"Position track file of target #2","seeker","save","com");
    radar[18].init("VTCII2",0,0,0,"Velocity track file of target #2","seeker","save","com");
    radar[19].init("STCII3",0,0,0,"Position track file of target #3","seeker","save","com");
    radar[20].init("VTCII3",0,0,0,"Velocity track file of target #3","seeker","save","com");
    radar[21].init("STCII4",0,0,0,"Position track file of target #4","seeker","save","com");
    radar[22].init("VTCII4",0,0,0,"Velocity track file of target #4","seeker","save","com");
    radar[23].init("STCII5",0,0,0,"Position track file of target #5","seeker","save","com");
    radar[24].init("VTCII5",0,0,0,"Velocity track file of target #5","seeker","save","com");
	radar[25].init("dat_sigma",0,"1 sigma error of distance measurement - m","seeker","data","");
	radar[26].init("azat_sigma",0,"1 sigma error of azimuth measurement - rad","seeker","data","");
	radar[27].init("elat_sigma",0,"1 sigma error of elevation measurement - rad","seeker","data","");
	radar[28].init("vel_sigma",0,"1 sigma error of velocity component meas. - m/s","seeker","data","");
}
///////////////////////////////////////////////////////////////////////////////
//Seeker (Targeting Radar) Module 
//Member function of class 'Radar'
//
// * Measures LOS to satellite, corrupted by errors
// * Measures velocity of satellite, corrupted by errors
// * Loads measurements onto 'combus' 
//
// Notes:*The measurements are taken at greater intervals than the 
//		  integration step. There is no correlation between measurements.
//		  Therefore, a Gaussian corruption taken at each measurement is good modeling
//        practice (instead of Markov).
//		 *The satellite measurements are stored in target track files, which are 
//        loaded onto 'combus', LIMITED TO THE FIRST 5 TARGETS in the 'vehicle_list'.
//        Each track file consists of satellite position and velocity vectors STCIIx, VTCIIx
//		  where 'x' is the sequential number of the satellite as loaded in 'input.asc'
//
//040517 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Radar::seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)
{
	//local variables
	Variable *data_t;
	Matrix STII(3,1);
	Matrix VTII(3,1);
	Matrix SBTI(3,1);
	Matrix POLAR(3,1);
	Matrix STCII(3,1);
	Matrix VTCII(3,1);
	Matrix SBTCI(3,1);

	//local module-variables

	//localizing module-variables
	//input data
	int radar_on=radar[10].integer();
	int init_flag=radar[11].integer();
	int target_num=radar[14].integer();
	double track_step=radar[13].real();
	double dat_sigma=radar[25].real();
	double azat_sigma=radar[26].real();
	double elat_sigma=radar[27].real();
	double vel_sigma=radar[28].real();
	//getting saved data
	double track_epoch=radar[12].real();
	Matrix STCII1=radar[15].vec();
	Matrix VTCII1=radar[16].vec();
	Matrix STCII2=radar[17].vec();
	Matrix VTCII2=radar[18].vec();
	Matrix STCII3=radar[19].vec();
	Matrix VTCII3=radar[20].vec();
	Matrix STCII4=radar[21].vec();
	Matrix VTCII4=radar[22].vec();
	Matrix STCII5=radar[23].vec();
	Matrix VTCII5=radar[24].vec();
	//from other modules
	Matrix SBII=ground0[14].vec();
	//-------------------------------------------------------------------------
	//return if radar is not tracking
	if(!radar_on) return;

	//storing tracking time initially 
	if(init_flag){
	init_flag=0;
		track_epoch=sim_time;
	}

	//measuring parameters of satellites
	if(sim_time>= track_epoch){

		//next tracking epoch
		track_epoch=sim_time+track_step;

		//cycling through all vehicles to find the satellite 
		//initializing target counter
		target_num=1;
		//finding slot 'i' of satellite in 'combus' (same as in vehicle_list)
		for(int i=0;i<num_vehicles;i++){

			//downloading from 'combus' satellite variables
			//getting vehicle id
			string id=combus[i].get_id();
			//building target id = t(j+1)
			char number[4];	
			sprintf(number,"%i",target_num);
			string target_id="t"+string(number);

			if (id==target_id){

				//downloading data from satellite (target) packet
				data_t=combus[i].get_data();
				STII=data_t[1].vec();
				VTII=data_t[2].vec();

				//true polar coordinates to satellite
				SBTI=SBII-STII; 
				POLAR=SBTI.pol_from_cart();
				double dat=POLAR.get_loc(0,0);
				double azat=POLAR.get_loc(1,0);
				double elat=POLAR.get_loc(2,0);

				//corrupting true values to obtain polar postion measurements
				double dat_meas=dat+gauss(0,dat_sigma);
				double azat_meas=azat+gauss(0,azat_sigma);
				double elat_meas=elat+gauss(0,elat_sigma);

				//converting to cartesian coordiantes
				SBTCI.cart_from_pol(dat_meas,azat_meas,elat_meas);
				STCII=SBII-SBTCI;

				//noise corruption of velocity measurements
				double vtcii1=VTII.get_loc(0,0)+gauss(0,vel_sigma);
				double vtcii2=VTII.get_loc(1,0)+gauss(0,vel_sigma);
				double vtcii3=VTII.get_loc(2,0)+gauss(0,vel_sigma);
				VTCII.build_vec3(vtcii1,vtcii2,vtcii3);
				
				//building track files
				switch(target_num){
				case 1:
					STCII1=STCII;
					VTCII1=VTCII;
					break;
				case 2:
					STCII2=STCII;
					VTCII2=VTCII;
					break;
				case 3:
					STCII3=STCII;
					VTCII3=VTCII;
					break;
				case 4:
					STCII4=STCII;
					VTCII4=VTCII;
					break;
				case 5:
					STCII5=STCII;
					VTCII5=VTCII;
					break;
				}
				//target counter
				target_num++;

				//discarding satellites #6 and higher
				if(target_num>5)break;

			}//track file of one satellite loaded
		}//all track files loaded
	}//end of satellite data measurements
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving data
	radar[11].gets(init_flag);
	radar[12].gets(track_epoch);
	radar[14].gets(target_num);
	radar[15].gets_vec(STCII1);
	radar[16].gets_vec(VTCII1);
	radar[17].gets_vec(STCII2);
	radar[18].gets_vec(VTCII2);
	radar[19].gets_vec(STCII3);
	radar[20].gets_vec(VTCII3);
	radar[21].gets_vec(STCII4);
	radar[22].gets_vec(VTCII4);
	radar[23].gets_vec(STCII5);
	radar[24].gets_vec(VTCII5);
}
