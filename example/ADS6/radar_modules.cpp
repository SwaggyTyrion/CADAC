///////////////////////////////////////////////////////////////////////////////
//FILE: 'radar_modules.cpp'
//
//Contains all Modules of class 'Radar'
//						sensor()	radar[10-49]
//
// Generally used variables are assigned to radar[0-9] 
//
// Tracks targets, predicts intercept IP, and launches missiles
//
//170916 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of sensor module-variables 
//Member function of class 'Radar'
//Module-variable locations are assigned to radar[10-49]
//
//180101 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Radar::def_sensor()
{
	//Definition of module-variables
	radar[9].init("mtrack","int",0,"Tracking flag, =0:off; =1:rocket; =2:aircraft - ND","sensor","data","");
	radar[10].init("alt_engage",0,"Desired engagement altitude - m","sensor","data","");
	radar[11].init("init_flag","int",true,"Initialization flag, =0:off; =1:on - ND","sensor","init","");
	radar[12].init("track_epoch",0,"Tracking epoch - s","sensor","save","");
	radar[13].init("track_step",0,"Tracking time interval - s","sensor","data","");
	radar[14].init("rocket_num","int",0,"Rocket 3 in combus- ND","sensor","save","");
	radar[15].init("launch_delay",9999,"Launch delay of missile - sec","sensor","save","");
	radar[16].init("SIEL",0,0,0,"IP coord for missile - m","sensor","save","");
	radar[17].init("ip_alt_bias",0,"IP biased up for line guidance - m","sensor","data","");
	radar[18].init("lnch_dly_bias1",0,"Launch delay bias missile #1 - sec","sensor","data","");
	radar[19].init("lnch_dly_bias2",0,"Launch delay bias missile #2 - sec","sensor","data","");
	radar[20].init("lnch_dly_bias3",0,"Launch delay bias missile #3 - sec","sensor","data","");
	radar[26].init("dat_sigma",0,"1 sigma error of distance measurement - m","sensor","data","");
	radar[27].init("azat_sigma",0,"1 sigma error of azimuth measurement - rad","sensor","data","");
	radar[28].init("elat_sigma",0,"1 sigma error of elevation measurement - rad","sensor","data","");
	radar[29].init("vel_sigma",0,"1 sigma error of velocity component meas. - m/s","sensor","data","");
	radar[30].init("apo_flag","int",false,"Rocket has reached apogee - ND","sensor","save","");
	radar[31].init("apo_epoch",0,"'sim_time' at rocket apogee - s","sensor","save","");
	radar[32].init("lnch_delay_m1",0,"Missile #1 launch delay - s","sensor","out","com");
	radar[33].init("lnch_delay_m2",0,"Missile #2 launch delay - s","sensor","out","com");
	radar[34].init("lnch_delay_m3",0,"Missile #3 launch delay - s","sensor","out","com");
	radar[35].init("SIEL1",0,0,0,"IP coord for missile #1 - m","sensor","out","com");
	radar[36].init("SIEL2",0,0,0,"IP coord for missile #2 - m","sensor","out","com");
	radar[37].init("SIEL3",0,0,0,"IP coord for missile #3 - m","sensor","out","com");
	radar[40].init("aircraft_num","int",0,"Aircraft 3 in combus- ND","sensor","save","");
	radar[41].init("lethal_rng",0,"Lethal range of SAM - m","sensor","data","");
	radar[42].init("lethal_flag1","int",false,"AC1 enters lethal zone - ND","sensor","save","");
	radar[43].init("lethal_flag2","int",false,"AC2 enters lethal zone - ND","sensor","save","");
	radar[44].init("lethal_flag3","int",false,"AC3 enters lethal zone - ND","sensor","save","");
	radar[45].init("launch_delay1",9999,"Launch delay of missile #1 - sec","sensor","save","");
	radar[46].init("launch_delay2",9999,"Launch delay of missile #2 - sec","sensor","save","");
	radar[47].init("launch_delay3",9999,"Launch delay of missile #3 - sec","sensor","save","");
}
///////////////////////////////////////////////////////////////////////////////
//Sensor Radar Module 
//Member function of class 'Radar'
//
// * Radar tracks targets and missiles, gives missile launch commands, and uplinks IP coordinates to missiles
// * Measures LOS to targets, corrupted by errors
// * Measures LOS to missiles, corrupted by errors
// * Uploads to 'combus' missile launch times - expressed by 'launch_delay' - and IP coordinates 
// * Up to three rocket-targets or up to three aircraft-targets
//
// Notes for rocket-targets engagments:
//		*Radar tracks rockets starting from launch and, after their apogee,
//		  starts calculations and transmissions of missile launch times and IP coordinates
//		*The radar measurements are taken at large intervals wrt the 
//		  integration step. There is no correlation between measurements.
//		  Therefore, a Gaussian corruption taken at each measurement is good modeling
//        practice (instead of Markov).
//		*Missile launches follow the rocket sequence: First launched rocket is intercepted by first
//		  launched missile, etc. up to three engagements.
//		*Radar predicts the IP coordinates based on tabulated rocket and missile trajectories.
//		  The IP coordinates are corrected as the radar tracks deviations from the tabulated trajectories 
//		*During midcourse the missiles fly towards their IPs, received from the radar,
//		  until they get close enough to turn on their seekers.
//		*Missiles home-in on target autonomouly. 
//
// Notes for aircraft-targets engagements
//		*Radar tracks aircaft targets and determines when aircaft enters lethality zone.
//		*The radar measurements are taken at large intervals wrt the 
//		  integration step. There is no correlation between measurements.
//		  Therefore, a Gaussian corruption taken at each measurement is good modeling
//        practice (instead of Markov).
//		*Missile launches when target-aircraft enters lethality zone; first aircraft entering the lethality
//		  zone is attacked by first missile, etc., up to three engagements.
//		*Radar sends current aircraft-target coordinates to missile.
//      *During midcourse the missiles fly towards the uplinked aircraft-target coordinates,
//		  until they get close enough to turn on their seekers.
//		*Missiles home-in on target autonomouly. 
//
//180110 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Radar::sensor(Packet *combus,int num_vehicles,int vehicle_slot,double sim_time,double int_step)
{
	//local variables
	Variable *data_t;
	Variable *data_m;
	Matrix STEL(3,1);
	Matrix VTEL(3,1);
	Matrix STRL(3,1);
	Matrix POLAR(3,1);
	Matrix STCEL(3,1);
	Matrix VTCEL(3,1);
	Matrix STRCL(3,1);
	Matrix SBEL(3,1);
	Matrix SBRL(3,1);
	Matrix SBRCL(3,1);
	Matrix SBCEL(3,1);
	double dat(0);
	double azat(0);
	double elat(0);
	double dat_meas=(0);
	double azat_meas=(0);
	double elat_meas=(0);
	double siel1=(0);
	double siel2=(0);
	double siel3=(0);
	double vtcel1=(0);
	double vtcel2=(0);
	double vtcel3=(0);
	double alt_diff_rock(0);
	double alt_diff_misl(0);
	double delta_ip(0);
	double alt_ip(0);
	double ip_apo_time_rocket(0);
	double dtrc(0);

	//localizing module-variables
	//input data
	int mtrack=radar[9].integer();
	double alt_engage=radar[10].real();
	int init_flag=radar[11].integer();
	int rocket_num=radar[14].integer();
	double track_step=radar[13].real();
	double ip_alt_bias=radar[17].real();
	double lnch_dly_bias1=radar[18].real();
	double lnch_dly_bias2=radar[19].real();
	double lnch_dly_bias3=radar[20].real();
	double dat_sigma=radar[26].real();
	double azat_sigma=radar[27].real();
	double elat_sigma=radar[28].real();
	double vel_sigma=radar[29].real();
	int aircraft_num=radar[40].integer();
	double lethal_rng=radar[41].real();
	//initializing and then getting saved data
	double track_epoch=radar[12].real();
	double launch_delay=radar[15].real();
	Matrix SIEL=radar[16].vec();
	int apo_flag=radar[30].integer();
	double apo_epoch=radar[31].real();
	double lnch_delay_m1=radar[32].real();
	double lnch_delay_m2=radar[33].real();
	double lnch_delay_m3=radar[34].real();
	Matrix SIEL1=radar[35].vec();
	Matrix SIEL2=radar[36].vec();
	Matrix SIEL3=radar[37].vec();
	int lethal_flag1=radar[42].integer();
	int lethal_flag2=radar[43].integer();
	int lethal_flag3=radar[44].integer();
	double launch_delay1=radar[45].real();
	double launch_delay2=radar[46].real();
	double launch_delay3=radar[47].real();

	//from other modules
	Matrix SREL=flat0[14].vec();
	//-------------------------------------------------------------------------
	//storing tracking time initially 
	if(init_flag){
	init_flag=false;
		track_epoch=sim_time;
	}
	//selecting rocket tracking
	if(mtrack==1)
	{
		//**Measuring parameters of rockets
		if(sim_time>= track_epoch)
		{
			//next tracking epoch
			track_epoch=sim_time+track_step;

			//cycling through all vehicles to find the rocket 
			//initializing rocket counter
			rocket_num=1;
			//finding slot 'i' of rocket in 'combus' (same as in vehicle_list)
			for(int i=0;i<num_vehicles;i++)
			{
				//downloading from 'combus' rocket variables
				//getting vehicle id
				string id=combus[i].get_id();
				char number[4];	
				sprintf(number,"%i",rocket_num);
				string rocket_id="r"+string(number);

				if (id==rocket_id)
				{
					//downloading data from rocket packet
					//(though in 'Flat3' the letter 'A' represents the air-target, which comprises both rocket and aircraft
					// here we use for the rocket-target the letter 'T' 
					data_t=combus[i].get_data();
					STEL=data_t[4].vec();
					VTEL=data_t[5].vec();

					//true polar coordinates from rocket relative to radar
					STRL=STEL-SREL;
					POLAR=STRL.pol_from_cart();
					dat=POLAR.get_loc(0,0);
					azat=POLAR.get_loc(1,0);
					elat=POLAR.get_loc(2,0);
					//corrupting true values to obtain polar position measurements
					dat_meas=dat+gauss(0,dat_sigma);
					azat_meas=azat+gauss(0,azat_sigma);
					elat_meas=elat+gauss(0,elat_sigma);
					//converting to local level coordinates
					STRCL.cart_from_pol(dat_meas,azat_meas,elat_meas);
					//meausured rocket position wrt to earth reference point E in local-level coordinates
					STCEL=STRCL-SREL;

					//noise corruption of velocity measurements
					vtcel1=VTEL.get_loc(0,0)+gauss(0,vel_sigma);
					vtcel2=VTEL.get_loc(1,0)+gauss(0,vel_sigma);
					vtcel3=VTEL.get_loc(2,0)+gauss(0,vel_sigma);
					//measured rocket velocity wrt earth in local-level coordinates
					VTCEL.build_vec3(vtcel1,vtcel2,vtcel3);

					//* At rocket apogee calculating launch_delay for missile and initial IP coordinates
					//using: if VTEL[2] becomes negative apogee is encountered
					double vtcel3=VTCEL[2];
					if(vtcel3>0 && !apo_flag)
					{
						apo_flag=true;
						apo_epoch=sim_time;
						//intercept time 'ip_apo_time_rocket' =  time from apogee to IP
						// obtained from rocket trajectory
						double ip_apo_time_rocket=rocket_traj.look_up("apotime_vs_descent_altitude",alt_engage);
						//intercept time 'ip_time_missile' = time from ground to IP
						double ip_time_missile=missile_traj.look_up("time_vs_ascent_altitude",alt_engage);
						//missiles launch_delay uploaded to 'combus'
						launch_delay=apo_epoch+ip_apo_time_rocket-ip_time_missile;
				
						//determining all three IP coordinates from tabular rocket data
						siel1=rocket_traj.look_up("x_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel2=rocket_traj.look_up("y_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel3=-alt_engage;
						SIEL.build_vec3(siel1,siel2,siel3);
						//missiles IP coordinates uploaded to 'combus'
					}
					if(sim_time > launch_delay)
					{
						//**Refining IP every radar tracking interval
						//*Rocket IP altitude differential
						//acquiring actual rocket altitude
						double alt_rock_actual=-STCEL[2];
						//rocket time since apogee
						double apo_time_rocket=rocket_traj.look_up("apotime_vs_descent_altitude",alt_rock_actual);
						//predicted rocket altitude at time_rocket
						double alt_rock_predicted=-rocket_traj.look_up("z_vs_launch_time",apo_time_rocket+apo_epoch);
						//differential altitude of rocket
						alt_diff_rock=alt_rock_predicted-alt_rock_actual;

						//**Missile tracking & calculating IP altitude differential 
						//*Downloading missile coordinates from 'combus'
						//getting vehicle id
						string id=combus[i].get_id();
						char number[4];	
						sprintf(number,"%i",rocket_num);
						string missile_id="m"+string(number);
						if (id==missile_id)
						{
							data_m=combus[i].get_data();
							SBEL=data_m[3].vec();
						}
						//true polar coordinates to missile
						SBRL=SBEL-SREL;
						POLAR=SBRL.pol_from_cart();
						dat=POLAR.get_loc(0,0);
						azat=POLAR.get_loc(1,0);
						elat=POLAR.get_loc(2,0);
						//corrupting true values to obtain polar postion measurements
						dat_meas=dat+gauss(0,dat_sigma);
						azat_meas=azat+gauss(0,azat_sigma);
						elat_meas=elat+gauss(0,elat_sigma);
						//converting to local level coordinates
						SBRCL.cart_from_pol(dat_meas,azat_meas,elat_meas);
						//meausured missile wrt to earth reference point E in local-level coordinates
						SBCEL=SBRCL-SREL;

						//*Missile altitude differential
						//acquiring actual missile altitude
						double alt_misl_actual=-SBCEL[2];
						//missile time since launch given current altitude
						double time_missile=missile_traj.look_up("time_vs_ascent_altitude",alt_misl_actual);
						//predicted missile altitude at time_missile
						double alt_misl_predicted=missile_traj.look_up("alt_vs_launch_time",time_missile);
						//differential altitude of rocket
						alt_diff_misl=alt_misl_predicted-alt_misl_actual;
					}
					//uploading to 'combus' missile 'launch_delay' and IP coordinates 
					// with the pairing of rocket tail# = missile tail# 
					switch(rocket_num){
					case 1:
						//launch delay for missile #1 attacking rocket #1
						lnch_delay_m1=launch_delay+lnch_dly_bias1;

						//IP with updates
						//get new IP coordinates
						delta_ip=-alt_diff_rock-alt_diff_misl;
						//current IP altitude
						alt_ip=-SIEL[2];
						//new IP altitude 
						alt_ip+=delta_ip;
						//new IP coordinates
						ip_apo_time_rocket=rocket_traj.look_up("apotime_vs_descent_altitude",alt_ip);
						siel1=rocket_traj.look_up("x_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel2=rocket_traj.look_up("y_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel3=-alt_ip-ip_alt_bias;
						SIEL1.build_vec3(siel1,siel2,siel3);
						break;

					case 2:
						//launch delay for missile #2 attacking rocket #2
						lnch_delay_m2=launch_delay+lnch_dly_bias2;

						//IP with updates
						//get new IP coordinates
						delta_ip=-alt_diff_rock-alt_diff_misl;
						//current IP altitude
						alt_ip=-SIEL[2];
						//new IP altitude 
						alt_ip+=delta_ip;
						//new IP coordinates
						ip_apo_time_rocket=rocket_traj.look_up("apotime_vs_descent_altitude",alt_ip);
						siel1=rocket_traj.look_up("x_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel2=rocket_traj.look_up("y_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel3=-alt_ip-ip_alt_bias;
						SIEL2.build_vec3(siel1,siel2,siel3);
						break;

					case 3:
						//launch delay for missile #3 attacking rocket #3
						lnch_delay_m3=launch_delay+lnch_dly_bias3;

						//IP with updates
						//get new IP coordinates
						delta_ip=-alt_diff_rock-alt_diff_misl;
						//current IP altitude
						alt_ip=-SIEL[2];
						//new IP altitude 
						alt_ip+=delta_ip;
						//new IP coordinates
						ip_apo_time_rocket=rocket_traj.look_up("apotime_vs_descent_altitude",alt_ip);
						siel1=rocket_traj.look_up("x_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel2=rocket_traj.look_up("y_vs_launch_time",ip_apo_time_rocket+apo_epoch);
						siel3=-alt_ip-ip_alt_bias;
						SIEL3.build_vec3(siel1,siel2,siel3);
						break;
					}
					//rocket counter
					rocket_num++;

					//discarding rockets #4 and higher
					if(rocket_num>3)break;

				}//one rocket tracked and corresponding missile's launch signal and IP coordinates transmitted
			}//all rockets tracked
		}//end of measuring rocket parameters
	}//end of rocket tracking

	//selecting aircraft tracking
	else if(mtrack==2)
	{
		//**Measuring parameters of aircraft
		if(sim_time>= track_epoch)
		{
			//next tracking epoch
			track_epoch=sim_time+track_step;

			//cycling through all vehicles to find the aircraft 
			//initializing aircraft counter
			aircraft_num=1;
			//finding slot 'i' of aircraft in 'combus' (same as in vehicle_list)
			for(int i=0;i<num_vehicles;i++)
			{
				//*Downloading from 'combus' aircraft variables
				//getting vehicle id
				string id=combus[i].get_id();
				char number[4];	
				sprintf(number,"%i",aircraft_num);
				string aircraft_id="a"+string(number);

				if (id==aircraft_id)
				{
					//downloading data from aircraft packet
					//(though in 'Flat3' the letter 'A' represents the air-target, which comprises both aircraft and aircraft
					// here we use for the aircraft-target the letter 'T' 
					data_t=combus[i].get_data();
					STEL=data_t[4].vec();
					VTEL=data_t[5].vec();

					//true polar coordinates from aircraft relative to radar
					STRL=STEL-SREL;
					POLAR=STRL.pol_from_cart();
					dat=POLAR.get_loc(0,0);
					azat=POLAR.get_loc(1,0);
					elat=POLAR.get_loc(2,0);
					//corrupting true values to obtain polar position measurements
					dat_meas=dat+gauss(0,dat_sigma);
					azat_meas=azat+gauss(0,azat_sigma);
					elat_meas=elat+gauss(0,elat_sigma);
					//converting to local level coordinates
					STRCL.cart_from_pol(dat_meas,azat_meas,elat_meas);
					//calculating range of aircaft to radar
					dtrc=STRCL.absolute();
					//meausured aircraft position wrt to earth reference point E in local-level coordinates
					STCEL=STRCL-SREL;

					//calculating launch delay of missile #1 gainst aircraft #1
					if((dtrc<lethal_rng) && (aircraft_num==1) && (!lethal_flag1))
					{
						lethal_flag1=true;
						launch_delay1=sim_time;
					}
					//calculating launch delay of missile #2 gainst aircraft #2
					if((dtrc<lethal_rng) && (aircraft_num==2) && (!lethal_flag2))
					{
						lethal_flag2=true;
						launch_delay2=sim_time;
					}
					//calculating launch delay of missile #3 gainst aircraft #3
					if((dtrc<lethal_rng) && (aircraft_num==3) && (!lethal_flag3))
					{
						lethal_flag3=true;
						launch_delay3=sim_time;
					}
					//noise corruption of velocity measurements
					vtcel1=VTEL.get_loc(0,0)+gauss(0,vel_sigma);
					vtcel2=VTEL.get_loc(1,0)+gauss(0,vel_sigma);
					vtcel3=VTEL.get_loc(2,0)+gauss(0,vel_sigma);
					//measured aircraft velocity wrt earth in local-level coordinates
					VTCEL.build_vec3(vtcel1,vtcel2,vtcel3);

					//uploading to 'combus' missile 'launch_delay' and IP (aircraft-target) coordinates 
					// with pairing of aircraft tail# = missile tail# 
					switch(aircraft_num){
					case 1:
						//launch delay for missile #1 attacking aircraft #1
						//1st missile launch delay is issued (uploaded to 'combus')
						lnch_delay_m1=launch_delay1+lnch_dly_bias1;
						//IP coordinates
						SIEL1=STCEL;
						break;

					case 2:
						//launch delay for missile #2 attacking aircraft #2
						//2nd missile launch delay is issued (uploaded to 'combus')
						lnch_delay_m2=launch_delay2+lnch_dly_bias2;
						//IP coordinates
						SIEL2=STCEL;
						break;

					case 3:
						//launch delay for missile #3 attacking aircraft #3
						//t3rd missile launch delay is issued (uploaded to 'combus')
						lnch_delay_m3=launch_delay3+lnch_dly_bias3;
						//IP coordinates
						SIEL3=STCEL;
						break;
					}
					//aircraft counter
					aircraft_num++;

					//discarding aircraft #4 and higher
					if(aircraft_num>3)break;

				}//one aircraft tracked and corresponding missile's launch signal and IP coordinates transmitted
			}//all aircraft tracked
		}//end of measuring aircraft parameters
	}//end of aircraft tracking
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to combus
	radar[32].gets(lnch_delay_m1);
	radar[33].gets(lnch_delay_m2);
	radar[34].gets(lnch_delay_m3);
	radar[35].gets_vec(SIEL1);
	radar[36].gets_vec(SIEL2);
	radar[37].gets_vec(SIEL3);
	//saving data
	radar[11].gets(init_flag);
	radar[12].gets(track_epoch);
	radar[14].gets(rocket_num);
	radar[15].gets(launch_delay);
	radar[16].gets_vec(SIEL);
	radar[30].gets(apo_flag);
	radar[31].gets(apo_epoch);
	radar[40].gets(aircraft_num);
	radar[42].gets(lethal_flag1);
	radar[43].gets(lethal_flag2);
	radar[44].gets(lethal_flag3);
	radar[45].gets(launch_delay1);
	radar[46].gets(launch_delay2);
	radar[47].gets(launch_delay3);
}


