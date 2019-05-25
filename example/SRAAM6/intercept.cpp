///////////////////////////////////////////////////////////////////////////////
//FILE: 'intercept.cpp'
//Contains the 'intercept' module of class 'Missile'
//
//030712 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of intercept module-variables
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[650-699]
//		
//030714 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_intercept()
{
	//definition of module-variables
	missile[651].init("write","int",1,"True flag for writing miss to console - ND","intercept","init","");
	missile[652].init("miss",0,"Miss distance - m","intercept","diag","plot");
	missile[653].init("hit_time",0,"Intercept time - s","intercept","diag","");
	missile[654].init("MISS_L",0,0,0,"Miss vector in local level coord. - m","intercept","diag","");
	missile[655].init("time_m",0,"Previous time - s","intercept","save","");
	missile[656].init("SBTLM",0,0,0,"Previous displacment vector. - m","intercept","save","");
	missile[657].init("STMEL",0,0,0,"Previous aircraft taget displacment vector. - m","intercept","save","");
	missile[658].init("SBMEL",0,0,0,"Previous missile displacment vector. - m","intercept","save","");
	missile[659].init("mode","int",0,"Mode flags |mseek|mguid|maut|mprop|  - ND","intercept","diag","scrn");

}
///////////////////////////////////////////////////////////////////////////////
//Intercept module
//Member function of class 'Missile'
//Determining closest approach of missile and target points
//
//Parameter Input: 'vehicle_slot' is current 'Missile' object
//Input from module-variable array: 'tgt_com_slot' target being attacked, determined in 'seeker' module
//
//console output: miss distance and associated parameters written to console
//
//030714 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::intercept(Packet *combus,int vehicle_slot,double int_step,char *title)
{
	//local module-variables
	double hit_time(0);
	Matrix MISS_L(3,1);
	double miss(0);
	int mode(0);

	//localizing module-variables
	//input from other modules
	double time=flat6[0].real();
	int halt=flat6[1].integer();
	Matrix TBL=flat6[120].mat();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	double dvbe=flat6[236].real();
	double psivlx=flat6[240].real();
	double thtvlx=flat6[241].real();
	Matrix STEL=missile[2].vec(); 
	Matrix VTEL=missile[3].vec();
	int tgt_com_slot=missile[5].integer();
	int mprop=missile[50].integer();
	int mseek=missile[200].integer();
	int mguid=missile[400].integer();
	int maut=missile[500].integer();
	//getting saved values
	int write=missile[651].integer();
	double time_m=missile[655].real();
	Matrix SBTLM=missile[656].vec();
	Matrix STMEL=missile[657].vec();
	Matrix SBMEL=missile[658].vec();
	//-------------------------------------------------------------------------
	//trajectory mode flags
	mode=1000*mseek+100*mguid+10*maut+mprop;

	//LOS geometry
	Matrix STBL=STEL-SBEL;
	Matrix STBB=TBL*STBL;
	double dbt=STBL.absolute();

	//Termination of run if halt==1 
	if(halt){

		//writing information to console
		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();
		cout<<"\n"<<" *** Halt of Missile_"<<id_missl<<"   Time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    
		combus[vehicle_slot].set_status(0);
	}

	//Ground impact
	double alt=-SBEL.get_loc(2,0);
	if((alt<=0)&&write)
	{
		write=0;

		//getting missile #
		string id_missl=combus[vehicle_slot].get_id();

		//writing information to console
//		cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
		cout<<"\n"<<" *** Ground impact of Missile_"<<id_missl<<" (or: #IND/#QNAN)  Time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);
	}

	//Seeker/pronav
	if(mseek>=3)
	{
		//entering sphere of target influence of 100m 
		if(dbt<100)
		{		
			//unit LOS vector
			Matrix UTBL=STBL*(1./dbt);

			//relative velocity
			Matrix VTBL=VTEL-VBEL;

			//closing speed on target
			double closing_speed=UTBL^VTBL;
			
			//Intercept (closing speed becomes negative)
			//Miss is closest distance between missile and target points; obtained by linear interpolation
			//between integration steps
			Matrix SBTL=STBL*(-1);
			if((closing_speed>0)&&write)
			{
				write=0;

				Matrix SBBML=SBEL-SBMEL;
				Matrix STTML=STEL-STMEL;

				//intercept time at point of closest approach
				hit_time=time_m-int_step*((SBBML-STTML)^SBTLM)/(SBBML^SBBML);

				//miss distance vector in geographic coordinates
				double tau=hit_time-time_m;
				MISS_L=(SBBML-STTML)*(tau/int_step)+SBTLM;
				miss=MISS_L.absolute();

				//getting missile # and target #
				string id_targ=combus[tgt_com_slot].get_id();
				string id_missl=combus[vehicle_slot].get_id();

				//writing miss information to console
	//			cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
				cout<<"\n"<<" *** Intercept of Missile_"<<id_missl<<" and Target_"<<id_targ<<" ***\n";
				cout<<"      miss distance = "<<miss<<" m    intercept time = "<<hit_time<<" sec\n";
				cout<<"      north = "<<MISS_L.get_loc(0,0)<<" m      east = "<<MISS_L.get_loc(1,0)
								<<" m        down = "<<MISS_L.get_loc(2,0)<<" m\n";
				cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg       gamma = "<<thtvlx<<" deg\n\n";    
				
				//declaring missile and target 'dead (0)
				combus[tgt_com_slot].set_status(0);
				combus[vehicle_slot].set_status(0);
			}
			//save from previous cycle
			SBTLM=SBTL;
			STMEL=STEL;
			SBMEL=SBEL;
			time_m=time;
		}
	}//end of seeker/pronav
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	missile[651].gets(write);
	missile[653].gets(hit_time);
	missile[655].gets(time_m);
	missile[656].gets_vec(SBTLM);
	missile[657].gets_vec(STMEL);
	missile[658].gets_vec(SBMEL);
	//diagnostics
	missile[652].gets(miss);
	missile[654].gets_vec(MISS_L);
	missile[659].gets(mode);
}
