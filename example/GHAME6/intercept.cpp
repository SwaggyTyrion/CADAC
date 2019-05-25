///////////////////////////////////////////////////////////////////////////////
//FILE: 'intercept.cpp'
//Contains 'inercept' module of class 'Hyper'
//
//030619 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of intercept module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[650-699]
//
//		
//    event = |mseek|mguide|maut|rcs_moment|mprop|
//
//			   
//			   mseek = 0 Seeker turned off
//					 = 2 Seeker enabled
//					 = 3 Acquisition mode
//					 = 4 Seeker lock-on
//					 = 5 Seeker blind
//					 
//					 mguide = 30 line-guidance lateral, with maut 35 
//							= 03 line-guidance in pitch 
//							= 33 line-guidance lateral and in pitch, with maut 33
//							=  4 arc-guidance lateral, with maut 25
//							=  5 linear tangent guidance law (LTG) for rocket ascent   
//							=  6 terminal guidance to uplinked coordinates
//							=  7 terminal guidance with RF seeker
//						    =  8 glideslope rendez-vous guidance with data link targeting
// 
//						    maut = |mauty||mautp| 
//									mauty = 0 no control, fixed control surfaces
//										  = 2 yaw rate control - SAS
//										  = 3 yaw acceleration control (inludes SAS)
//										  = 4 heading angle control (inludes SAS) 
//										   mautp = 0 no control, fixed control surfaces
//												 = 2 pitch rate control
//												 = 3 pitch acceleration control
//												 = 4 flight path angle control
//												 = 5 altitude hold control (includes pitch accel)
//
//								 rcs_moment = |rcs_type||rcs_mode|
//											   rcs_type = 0 no RCS thrusting
//														= 1 proportional moment thrusters
//														= 2 on-off moment thrusters (Schmitt trigger)
//														 rcs_mode = 0 no control
//																  = 1 all angle control
//																  = 2 thrust vector direction and roll angle control
// 
//											mrpop = 0 No thrusting
//												  = 1 Hypersonic propulsion throttle command (input)
//												  = 2 Hypersonic propulsion autothrottle (input)
//												  = 3 Constant thrust rocket under LTG control (set in 'guidance' module)
//												  = 4 Constant thrust rocket (input or glideslope guidance initiated)
//
//030619 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_intercept()
{
	//definition of module-variables
	hyper[650].init("mintercept","int",0,"Mode switch - ND","intercept","data","");
	hyper[651].init("write","int",1,"True flag for writing miss to console - ND","intercept","init","");
	hyper[652].init("miss",0,"Miss distance - m","intercept","diag","plot");
	hyper[653].init("hit_time",0,"Intercept time - s","intercept","diag","");
	hyper[654].init("MISS_I",0,0,0,"Miss vector in inertial coord. - m","intercept","diag","plot");
	hyper[655].init("time_m",0,"Previous time - s","intercept","save","");
	hyper[656].init("SBTIM",0,0,0,"Previous displacment vector - m","intercept","save","");
	hyper[657].init("STMII",0,0,0,"Previous  taget displacement vector. - m","intercept","save","");
	hyper[658].init("SBMII",0,0,0,"Previous hyper displacement vector - m","intercept","save","");
	hyper[659].init("event","int",0,"|mturn|skr_type|skr_mode|mguid|mauty|mautp|mprop| - ND","intercept","diag","scrn,plot");
	hyper[660].init("dbt",0,"True distance between vehicle and target - m","intercept","diag","scrn,plot");
	hyper[661].init("MISS_H",0,0,0,"Miss vector in Hill coord. - m","intercept","diag","plot");
	hyper[662].init("MISS_L",0,0,0,"Miss vector in local level coord. - m","intercept","diag","plot");
}
///////////////////////////////////////////////////////////////////////////////
//Intercept module
//Member function of class 'Hyper'
//Determining closest approach to waypoints
//Module-variable locations are assigned to hyper[650-699]
//
//Parameter Input: 'vehicle_slot' is current 'Hyper' object
//Input from module-variable array: 'sat_num' satellite being approached, specified in 'input.asc'
//
//Console output: miss distance and associated parameters written to console
//
//030619 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::intercept(Packet *combus,int vehicle_slot,double int_step,char *title)
{
	//local variables
	//local module-variables
  	double miss(0);
	double hit_time(0);
	Matrix MISS_I(3,1);
	int event(0);
	double dbt(0);
	Matrix MISS_H(3,1);
	Matrix MISS_L(3,1);

	//localizing module-variables
	//initializations
	int write=hyper[651].integer();
	//input from other modules
	double time=round6[0].real();
	Matrix TBI=round6[121].mat();
	double alt=round6[221].real();
	double dvbe=round6[225].real();
	double dvbi=round6[226].real();
	double psivdx=round6[228].real();
	double thtvdx=round6[229].real();
	Matrix SBII=round6[235].vec();
	Matrix VBII=round6[236].vec();
	int headon_flag=round6[257].integer();
	int sat_num=hyper[1].integer();
	Matrix STII=hyper[2].vec();
	Matrix VTII=hyper[3].vec();
	int mprop=hyper[10].integer();
	int mrcs_moment=hyper[50].integer();
	int mseek=hyper[200].integer();
	int mguide=hyper[400].integer();
	double wp_lonx=hyper[405].real();
	double wp_latx=hyper[406].real();
	double wp_alt=hyper[407].real();
	Matrix SWBD=hyper[416].vec();
	int wp_flag=hyper[419].integer();
	int maut=hyper[500].integer();
	//restoring saved values
	double time_m=hyper[655].real();
	Matrix SBTIM=hyper[656].vec();
	Matrix STMII=hyper[657].vec();
	Matrix SBMII=hyper[658].vec();
//-----------------------------------------------------------------------------
	//building diagnostic event flag
    int mauty=maut/10;
    int mautp=(maut%10);
    int rcs_type=mrcs_moment/10;
    int rcs_mode=(mrcs_moment%10); 
	event=(int)(mseek*1000000+mguide*100000+mauty*10000+mautp*1000+rcs_type*100+rcs_mode*10+mprop);

	//LOS geometry
	Matrix STBI=STII-SBII;
	Matrix STBB=TBI*STBI;
	dbt=STBI.absolute();

	//Ground impact
	if((alt<=0)&&write)
	{
		write=0;

		//getting hyper #
		string id_hyper=combus[vehicle_slot].get_id();

		//writing miss information to console
		cout<<"\n"<<" ***"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
		cout<<"\n"<<" *** Ground impact or 'alt'= #QNAN  of Hyper_"<<id_hyper<<"   Time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivdx<<" deg      gamma = "<<thtvdx<<" deg\n\n";    

		//declaring hyper 'dead'
		combus[vehicle_slot].set_status(0);
	}
	//Waypoint hoizontal miss distance 
	if(mguide==4||mguide==30)
	{
		if(wp_flag==1)write=1;
		if(wp_flag==-1&&write)
		{
			write=0;

			double swbd1=SWBD[0];
			double swbd2=SWBD[1];
			double dwbh=sqrt(swbd1*swbd1+swbd2*swbd2);

			//getting hyper #
			string id_hyper=combus[vehicle_slot].get_id();

			//writing miss information to console
			cout<<" *** Hyper "<<id_hyper<<" passed waypoint at longitude = "
				<<wp_lonx<<" deg, latitude = "<<wp_latx<<" deg at time = "<<time<<" sec *** \n";
			cout<<"      SWBD-horizontal miss distance  = "<<dwbh<<" m north = "<<swbd1
				<<" m  east = "<<swbd2<<" m\n";   
			cout<<"      speed = "<<dvbe<<" m/s   heading = "<<psivdx<<" deg        gamma = "<<thtvdx<<" deg\n\n";
		}
	}

	//Terminal line/line or point/line guidance
	if(mguide==33)
	{
		//resetting flag after passage of waypoint
		if(wp_flag==1)write=1;

		//Line guidance: simple miss distance calculation at last integration, no interpolation
		if((alt<=wp_alt)&&write)
		{
			write=0;
			
			miss=SWBD.absolute();

			//getting hyper #
			string id_hyper=combus[vehicle_slot].get_id();

			//writing miss information to console
			cout<<"\n"<<" ***"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
			cout<<"\n"<<" *** Impact of Hyper_"<<id_hyper<<" on waypoint coord.: longitude = "<<wp_lonx<<" deg, latitude = "
							<<wp_latx<<" deg, altitude = "<<wp_alt<<" m\n";
			cout<<"      miss distance = "<<miss<<" m    intercept time = "<<time<<" sec\n";
			cout<<"      north = "<<SWBD.get_loc(0,0)<<" m      east = "<<SWBD.get_loc(1,0)
							<<" m        down = "<<SWBD.get_loc(2,0)<<" m\n";
			cout<<"      speed = "<<dvbe<<" m/s heading = "<<psivdx<<" deg     gamma = "<<thtvdx<<" deg\n\n";    

			//declaring hyper 'dead'
			combus[vehicle_slot].set_status(0);
		}
	}//end of terminal line/line or point/line guidance

	//data link or seeker guidance
	if(mguide==6||mguide==7||mguide==8)
	{
		//resetting flag after passage of waypoint
		if(wp_flag==-1)write=1;

		//entering sphere of satellite influence of 20e3 m 
		if(dbt<20e3)
		{		
			//unit LOS vector
			Matrix UTBI=STBI*(1./dbt);

			//relative velocity
			Matrix VTBI=VTII-VBII;

			//closing speed on satellite
			double closing_speed=UTBI^VTBI;
			
			//Intercept (closing speed becomes negative)
			//Miss is closest distance between hyper and satellite points; obtained by linear interpolation
			//between integration steps
			Matrix SBTI=STBI*(-1);
			if((closing_speed>0)&&write)
			{
				write=0;

				Matrix SBBMI=SBII-SBMII;
				Matrix STTMI=STII-STMII;

				//intercept time at point of closest approach
				hit_time=time_m-int_step*((SBBMI-STTMI)^SBTIM)/(SBBMI^SBBMI);

				//miss distance vector in inertial coordinates
				double tau=hit_time-time_m;
				MISS_I=(SBBMI-STTMI)*(tau/int_step)+SBTIM;
				miss=MISS_I.absolute();

				if(mguide==6||mguide==7){
					//miss distance in Hill coordinates
					//Hill base vectors
					Matrix UH1=STII.univec3();
					Matrix ANG_MOM=STII.skew_sym()*VTII;
					Matrix UH3=ANG_MOM.univec3();
					Matrix UH2=UH3.skew_sym()*UH1;
					//T.M. of Hill wrt inertial coordinates
					Matrix THI(3,3);
					THI.build_mat33(UH1[0],UH1[1],UH1[2],UH2[0],UH2[1],UH2[2],UH3[0],UH3[1],UH3[2]);
					//vehicle displacement in Hill coord at closest approach
					MISS_H=THI*MISS_I;

					//relative speed
					double relative_speed(0);
					double sat_dvbi=VTII.absolute();
					if(headon_flag)
						relative_speed=dvbi+sat_dvbi;
					else
						relative_speed=fabs(dvbi-sat_dvbi);

					//getting hyper # 
					string id_hyper=combus[vehicle_slot].get_id();

					//writing miss information to console
					cout<<"\n"<<" ***"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
					cout<<"\n"<<" *** Intercept of Vehicle_"<<id_hyper<<" and Satellite_t"<<sat_num<<" in Hill coordinates ***\n";
					cout<<"      miss distance = "<<miss<<" m    intercept time = "<<hit_time<<" sec\n";
					cout<<"      H1-dir = "<<MISS_H.get_loc(0,0)<<" m    H2-dir = "<<MISS_H.get_loc(1,0)
									<<" m      H3-dir = "<<MISS_H.get_loc(2,0)<<" m\n";
					cout<<"      speed dvbi = "<<dvbi<<" m/s   heading = "<<psivdx<<" deg     gamma = "<<thtvdx<<" deg\n";
					cout<<"      satellite speed dvbi = "<<sat_dvbi<<" m/s"<<"     relative speed = "<<relative_speed<<" m/s\n\n";
				}
				if(mguide==8){
					//miss distance in local level coordinates (assumes circular target satellite orbit)
					//TM of local level wrt inertial coordinates
					Matrix UL1I=VTII.univec3(); 
					Matrix UL3I=STII.univec3()*(-1);
					Matrix UL2I=UL3I.skew_sym()*UL1I;
					Matrix TLI(3,3);
					TLI.build_mat33(UL1I[0],UL1I[1],UL1I[2],UL2I[0],UL2I[1],UL2I[2],UL3I[0],UL3I[1],UL3I[2]);

					//vehicle displacement in local level coord at closest approach
					MISS_L=TLI*MISS_I;

					//relative speed
					double relative_speed(0);
					double sat_dvbi=VTII.absolute();
					if(headon_flag)
						relative_speed=dvbi+sat_dvbi;
					else
						relative_speed=fabs(dvbi-sat_dvbi);

					//getting hyper # 
					string id_hyper=combus[vehicle_slot].get_id();

					//writing miss information to console
					cout<<"\n"<<" ***"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
					cout<<"\n"<<" *** Intercept of Vehicle_"<<id_hyper<<" and Satellite_t"<<sat_num<<" in local level coordinates ***\n";
					cout<<"      miss distance = "<<miss<<" m    intercept time = "<<hit_time<<" sec\n";
					cout<<"      L1-dir = "<<MISS_L.get_loc(0,0)<<" m    L2-dir = "<<MISS_L.get_loc(1,0)
									<<" m      L3-dir = "<<MISS_L.get_loc(2,0)<<" m\n";
					cout<<"      speed dvbi = "<<dvbi<<" m/s   heading = "<<psivdx<<" deg     gamma = "<<thtvdx<<" deg\n";
					cout<<"      satellite speed dvbi = "<<sat_dvbi<<" m/s"<<"     relative speed = "<<relative_speed<<" m/s\n\n";
				}				
				//declaring hyper dead
				combus[vehicle_slot].set_status(0);
			}
			//save from previous cycle
			SBTIM=SBTI;
			STMII=STII;
			SBMII=SBII;
			time_m=time;
		}
	}//end of datalink or seeker guidance
//-----------------------------------------------------------------------------
	//loading module-variables
	//saving for next cycle
	hyper[651].gets(write);
	hyper[656].gets_vec(SBTIM);
	hyper[657].gets_vec(STMII);
	hyper[658].gets_vec(SBMII);
	hyper[655].gets(time_m);
	//diagnostics
	hyper[652].gets(miss);
	hyper[653].gets(hit_time);
	hyper[654].gets_vec(MISS_I);
	hyper[659].gets(event);
	hyper[660].gets(dbt);
	hyper[661].gets_vec(MISS_H);
	hyper[662].gets_vec(MISS_L);
}
