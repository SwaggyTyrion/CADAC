///////////////////////////////////////////////////////////////////////////////
//FILE: 'intercept.cpp'
//Contains the 'intercept' module of class 'Missile'
//
//030712 Created by Peter H Zipfel
//060508 Modified for SWEEP++, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'intercept' module-variables
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[650-699]
//
// Trajectory termination code	'lconv'=0 initialized
//										1 undefined
//										2 trajectory terminated with miss calculation
//										3 trajectory terminated at ground impact
//										4 trajectory stopped when termination code 'trcond' is set (and if 'stop=1')
//										5 trajectory halted when 'halt' is set in 'input.asc' ('halt=1')
// 'lconv' is used by SWEEP
//
//030714 Created by Peter H Zipfel
//060509 Modified for SWEEP++, PZi
//100414 Modified for AGM6, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::def_intercept()
{
	//definition of module-variables
	missile[650].init("mterm","int",0,"=0:miss magnitude; =1:in I-plane; =2:w/angle input - ND","intercept","data","");
	missile[651].init("write","int",true,"True flag for writing miss to console - ND","intercept","init","");
	missile[652].init("miss",0,"Miss distance - m","intercept","diag","plot");
	missile[653].init("hit_time",0,"Intercept time - s","intercept","diag","");
	missile[654].init("MISS_P",0,0,0,"Miss vector in target plane coor - m","intercept","diag","plot");
	missile[655].init("time_m",0,"Previous time - s","intercept","save","");
	missile[656].init("SBMTP",0,0,0,"Previous displacment vector. - m","intercept","save","");
	missile[659].init("mode","int",0,"Mode flags |mseek|mguid|maut|mprop|  - ND","intercept","diag","scrn");
	missile[660].init("dbt",0,"True distance between blue missile and target - m","intercept","diag","scrn,plot");
	missile[661].init("psiplx",0,"Yaw angle of target plane - deg","intercept","/diag/data","plot");
	missile[662].init("thtplx",0,"Pitch angle of target plane - deg","intercept","diag/data","plot");
	missile[663].init("critmax",100,"Critish max value for miss - m","intercept","diag/data","plot");
}
///////////////////////////////////////////////////////////////////////////////
//'intercept' module
//Member function of class 'Missile'
//Determining closest approach of missile and target points in target plane
//
//Parameter Input: 'vehicle_slot' is current 'Missile' object
//Input from module-variable array: 'tgt_com_slot' target being attacked, determined in 'sensor' module
//
//030714 Created by Peter H Zipfel
//060509 Modified for SWEEP++, PZi
//100414 Modified for AGM6, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::intercept(Packet *combus,int vehicle_slot,int num_vehicles,double int_step,char *title)
{
	//local module-variables
	int tgt_com_slot(0);
	Variable *data_t;
	double hit_time(0);
	Matrix MISS_P(3,1);
	double miss(0);
	int mode(0);
	Matrix SBTP(3,1);
	Matrix TPL(3,3);
	Matrix STEL(3,1); 
	Matrix VTEL(3,1);

	//localizing module-variables
	//input data
	int mterm=missile[650].integer();
	double psiplx=missile[661].real();
	double thtplx=missile[662].real();
	//input from other modules
	double time=flat6[0].real();
	int halt=flat6[4].integer();
	int stop=flat6[5].integer();
	int lconv=flat6[6].integer();
	Matrix TBL=flat6[120].mat();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	double dvbe=flat6[236].real();
	double hbe=flat6[239].real();
	double psivlx=flat6[240].real();
	double thtvlx=flat6[241].real();
	int tgt_num=missile[1].integer();
	int mprop=missile[10].integer();
	int trcond=missile[180].integer();
	int mseek=missile[200].integer();
	int mguid=missile[400].integer();
	int maut=missile[500].integer();
	//getting saved values
	int write=missile[651].integer();
	double time_m=missile[655].real();
	Matrix SBMTP=missile[656].vec();
	//-------------------------------------------------------------------------
	//decoding guidance flag
    int guid_mid=mguid/10;
    int guid_term=(mguid%10);

	//trajectory mode flags
	mode=10000*mseek+1000*guid_mid+100*guid_term+10*maut+mprop;

	//downloading from 'combus' target variables
	//building target id = t(j+1)
	char number[4];	
	sprintf(number,"%i",tgt_num);
	string target_id="t"+string(number);
	//finding slot 'i' of target in 'combus' (same as in vehicle_list)
	for(int i=0;i<num_vehicles;i++)
	{
		string id=combus[i].get_id();
		if (id==target_id)
		{						
			//downloading data from target packet
			tgt_com_slot=i;
			data_t=combus[i].get_data();
			STEL=data_t[2].vec();
			VTEL=data_t[3].vec();
		}
	}
	//true distance between missile and target
	Matrix STBL=STEL-SBEL;
	Matrix STBB=TBL*STBL;
	double dbt=STBL.absolute();

	//Termination of trajectory if 'trcond' is set and 'stop=1' 
	if(trcond&&stop){

		//setting flag for termination
		lconv=4;

		//writing information to console
		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();
		cout<<"\n"<<" *** Stopped Missile_"<<id_missl<<" with condition trcond = "<<trcond <<"   sim_time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s   altitude = "<<hbe<<" m     heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);
	}
	//Termination of trajectory if halt==1 
	if(halt){

		//setting flag for termination
		lconv=5;

		//writing information to console
		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();
		cout<<"\n"<<" *** Halt of Missile_"<<id_missl<<"   sim_time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s   altitude = "<<hbe<<" m     heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);
	}
	//Ground impact
	double alt=-SBEL.get_loc(2,0);
	if((alt<=0)&&write){	
		write=0;

		//setting flag for termination
		lconv=3;

		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();

		//writing information to console
//		cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
		cout<<"\n"<<" *** Ground impact of Missile_"<<id_missl<<" (or: #IND/#QNAN)  sim_time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);
	}

	//Impact on target plane (tilt of target plane specified by psiplx, thtplx 
	if(guid_mid==4||guid_term==6)
	{
		Matrix SBTL=SBEL-STEL;
		dbt=SBTL.absolute();

		//entering the target sphere of influence, set to 100m
		if(dbt<100)
		{
			//stop integration if missile c.m is below target plane
			// target plane is defined by 'Target::sael1, sael2, seal3' and input 'psiplx' and 'thtplx'
			//calculating TPL target plane wrt local level coortinate transformation
			Matrix TPL=mat2tr(psiplx*RAD,thtplx*RAD);
			//true missile wrt target position in target plane coordinates
			Matrix SBTP=TPL*SBTL;

			//if missile dips below target plane, interpolate between previous integration step
			double sbtp3=SBTP[2];
			if(sbtp3>0)
			{
				write=0;

				//setting flag for termination
				lconv=2;

				//end-of-run calculations
				//miss distance of piercing point B and target point T in target plane (zipfel book p.52)
				Matrix SBBMP=SBTP-SBMTP;
				Matrix STBMP=SBMTP*(-1);
				double stbmp3=STBMP[2];
				double sbbmp3=SBBMP[2];
				double dum=stbmp3/sbbmp3;
				MISS_P=SBBMP*dum-STBMP;
				miss=MISS_P.absolute();

				//interpolated time of hit
				hit_time=dum*int_step+time_m;

				//getting hyper # 
				string id_missile=combus[vehicle_slot].get_id();

				//writing miss information to console
				cout<<"\n"<<" ***"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
				cout<<"\n"<<" *** Intercept of Missile_"<<id_missile<<" and Target_t"<<tgt_num<<" in target plane coordinates ***\n";
				cout<<"   * Miss distance  = "<<miss<<" m    intercept time = "<<hit_time<<" sec\n";
				cout<<"      up on target plane  = "<<MISS_P.get_loc(0,0)<<" m    right on target plan   = "<<MISS_P.get_loc(1,0)<<" m\n";
				cout<<"      speed dvbe = "<<dvbe<<" m/s   heading = "<<psivlx<<" deg     gamma = "<<thtvlx<<" deg\n";

				//declaring hyper 'dead'
				combus[vehicle_slot].set_status(0);
			}
			//save from previous cycle
			time_m=time;
			SBMTP=SBTP;
		}
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	missile[651].gets(write);
	missile[653].gets(hit_time);
	missile[655].gets(time_m);
	missile[656].gets_vec(SBMTP);
	//diagnostics
	flat6[6].gets(lconv);
	missile[652].gets(miss);
	missile[654].gets_vec(MISS_P);
	missile[659].gets(mode);
	missile[660].gets(dbt);
}
