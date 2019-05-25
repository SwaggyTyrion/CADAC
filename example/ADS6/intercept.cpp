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
// mterm = 0 miss of closest approach
//		   1 miss in intercept plane, that is given by calculated aspect angles
//         2 miss in intercept plane, but aspect angles are provided in 'input.asc'
//
//  if(mterm=0) MISS vector is in L-coordinates
//  if(mterm>=1) MISS vector is in intercept plane P-coordinates (MISS1 right, MISS2 down)
//
// Trajectory termination code	'lconv'=0 initialized
//										1 undefined
//										2 trajectory terminated with miss calculation
//										3 trajectory terminated at ground impact
//										4 trajectory stopped when termination code 'trcond' is set (and if 'stop=1')
//										5 trajectory stopped when missile reached IP without terminal guidance
// 'lconv' is used by SWEEP
//
//030714 Created by Peter H Zipfel
//060509 Modified for SWEEP++, PZi
//171008 Modified for ADSim, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::def_intercept()
{
	//definition of module-variables
	missile[650].init("mterm","int",0,"=0:miss magnitude; =1:in I-plane; =2:w/angle input - ND","intercept","data","");
	missile[651].init("write","int",true,"True flag for writing miss to console - ND","intercept","init","");
	missile[652].init("miss",0,"Miss distance - m","intercept","diag","plot");
	missile[653].init("hit_time",0,"Intercept time - s","intercept","diag","");
	missile[654].init("MISS",0,0,0,"Miss vector in L (mterm=0) or P (mterm>=1) coor - m","intercept","diag","plot");
	missile[655].init("time_m",0,"Previous time - s","intercept","save","");
	missile[656].init("SBTLM",0,0,0,"Previous displacment vector. - m","intercept","save","");
	missile[657].init("STMEL",0,0,0,"Previous aircraft taget displacment vector. - m","intercept","save","");
	missile[658].init("SBMEL",0,0,0,"Previous missile displacment vector. - m","intercept","save","");
	missile[659].init("mode","int",0,"Mode flags |mseek|mguide|maut|mprop|  - ND","intercept","diag","scrn,plot");
	missile[660].init("dbt",0,"True distance between missile and target - m","intercept","diag","scrn,plot");
	missile[661].init("psiptx",0,"Yaw angle of intercept plane - deg","intercept","/diag/data","plot");
	missile[662].init("thtptx",0,"Pitch angle of intercept plane - deg","intercept","diag/data","plot");
}
///////////////////////////////////////////////////////////////////////////////
//'intercept' module
//Member function of class 'Missile'
//Determining closest approach of missile and target center of mass
//
// mterm = 0 miss of closest approach
//		   1 miss in intercept plane, that is given by calculated aspect angles
//         2 miss in intercept plane, but aspect angles are provided in 'input.asc'
//  if(mterm>=1) MISS vector is coordinated in intercept plane: MISS1 down, MISS2 right 
//
//Parameter Input: 'vehicle_slot' is current 'Missile' object
//Input from module-variable array: 'tgt_com_slot' target being attacked, determined in 'sensor' module
//
//* Intercept plane: the plane is normal to the differential  
//  velocity vector and contains the target center of mass.
//  Its x-axis is obtained by rotating
//  from the nose of the target through 'psiptx' and 'thtptx'.
//  Its y-axis remains in the target x,y-plane. 
//
//* Monte Carlo runs: for every sample run the intercept plane
//  will be oriented slightly different. Better, make two M.C. runs.
//  First run establishes the mean orientation angles 'psiptx','thtptx'
//  of the intercept plane. Then use 'mterm=2' to input these angles for
//  the actual M.C. run.
//
// 	|mode| = |skr_type| |skr_mode| |guid_mid| |guid_term| |maut|  |mprop|
//
//			 1 RF	    1 off	   0 none	  0 none	  0 none  0 off
//			 2 IR		2 enabled  2 line	  6 Comp-PN   1 roll  1 on
//						3 acq	   3 PN		  7 PN		  2 rate
//				        4 lock  						  3 acc
//						5 blind							  4 acc+
//								
//030714 Created by Peter H Zipfel
//060509 Modified for SWEEP++, PZi
//080422 Added miss calculations in intercept plane, PZi
//171008 Added IP intercept for ADSim, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::intercept(Packet *combus,int vehicle_slot,double int_step,char *title)
{
	//local module-variables
	double hit_time(0);
	Matrix MISS(3,1);
	double miss(0);
	int mode(0);
	Matrix SBTP(3,1);
	Matrix TPL(3,3);

	//localizing module-variables
	//input data
	int mterm=missile[650].integer();
	double psiptx=missile[661].real();
	double thtptx=missile[662].real();
	//input from other modules
	double time=flat6[0].real();
	int stop=flat6[5].integer();
	int lconv=flat6[6].integer();
	double pdynmc=flat6[57].real();
	Matrix TBL=flat6[120].mat();
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	double dvbe=flat6[236].real();
	double alt=flat6[239].real();
	double psivlx=flat6[240].real();
	double thtvlx=flat6[241].real();
	Matrix STEL=missile[2].vec(); 
	Matrix VTEL=missile[3].vec();
	int tgt_slot=missile[5].integer();
	int mprop=missile[10].integer();
	int trcond=missile[180].integer();
	int mseek=missile[200].integer();
	double dta=missile[277].real();
	int mguide=missile[400].integer();
	double ip_sltrange=missile[425].real();
	Matrix SIBLC=missile[429].vec(); 
	int maut=missile[500].integer();
	//getting saved values
	int write=missile[651].integer();
	double time_m=missile[655].real();
	Matrix SBTLM=missile[656].vec();
	Matrix STMEL=missile[657].vec();
	Matrix SBMEL=missile[658].vec();
	//-------------------------------------------------------------------------
	//decoding guidance flag
    int guid_mid=mguide/10;
    int guid_term=(mguide%10);

	//decoding mseek
	int skr_type=mseek/10;
	int skr_mode=mseek%10;

	//trajectory mode flags
	mode=100000*skr_type+10000*skr_mode+1000*guid_mid+100*guid_term+10*maut+mprop;

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
		cout<<"      speed = "<<dvbe<<" m/s   altitude = "<<alt<<" m     heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

		//declaring missile 'dead'
		combus[vehicle_slot].set_status(0);

		//diplaying banner on screen at the end of run
		scrn_banner();
	}
	//Termination of trajectory if IP has been reached without terminal guidance 
	if(ip_sltrange<500){

		//calculating closing speed on IP
		Matrix UIBL=SIBLC*(1/ip_sltrange);
		double closing_speed=UIBL^VBEL;
			
		if((closing_speed<0)&&write)
		{
			write=0;
			//setting flag for termination on IP
			lconv=5;

			//writing information to console
			//getting missile #
			string id_missl=combus[vehicle_slot].get_id();
			cout<<"\n"<<" *** Missile_"<<id_missl<<"  reached IP at sim_time = "<<time<<" sec ***\n";
			cout<<"      miss distance = "<<ip_sltrange<<" m \n";
			cout<<"      speed = "<<dvbe<<" m/s   altitude = "<<alt<<" m     heading = "<<psivlx<<" deg      gamma = "<<thtvlx<<" deg\n\n";    

			//declaring missile 'dead'
			combus[vehicle_slot].set_status(0);

			//diplaying banner on screen at the end of run
			scrn_banner();				
		}
	}
	//Ground impact
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
		
		//diplaying banner on screen at the end of run
		scrn_banner();
	}
	//Sensor guidance
	if(skr_mode==4)
	{
		//eliminate 'trcond' caused during launch phase
		trcond=0;

		//entering sphere of target influence of 500m 
		if(dbt<500)
		{		
			//unit LOS vector
			Matrix UTBL=STBL*(1/dbt);

			//differential velocity
			Matrix VTBEL=VTEL-VBEL;

			//closing speed on target, projected on LOS
			double closing_speed=UTBL^VTBEL;
			
			//between integration steps
			Matrix SBTL=STBL*(-1);

			//differential speed
			double diff_speed=VTBEL.absolute();

			//calculating head-on aspect angles of missile velocity vector wrt target velocity vector
			//heading and flight path angle of target
			Matrix POLAR=VTEL.pol_from_cart();
			double psivl=POLAR[1];
			double thtvl=POLAR[2];
			//T.M. of target wrt local level coordinates 
			Matrix TTL=mat2tr(psivl,thtvl);
			//differential velocity of target wrt missile in target body coordinates
			Matrix VTBET=TTL*VTBEL;
			//differential velocity of target wrt missile in missile body coordinates
			Matrix VTBEB=TBL*VTBEL;
			//head-on aspect angles of missile wrt target velocity vectors 
			POLAR=VTBEB.pol_from_cart();
			double az=POLAR[1];
			double el=POLAR[2];
			double aspazx=az*DEG;
			double aspelx=el*DEG;

			if((closing_speed>0)&&write)
			{
			//intercept (closing speed becomes negative)
				write=0;
				if(mterm==0){
					//miss is closest approach between missile and target c.g.; obtained by linear interpolation
					//setting flag for termination
					lconv=2;

					Matrix SBBML=SBEL-SBMEL;
					Matrix STTML=STEL-STMEL;

					//intercept time at point of closest approach
					hit_time=time_m-int_step*((SBBML-STTML)^SBTLM)/(SBBML^SBBML);

					//getting missile # and target #
					string id_targ=combus[tgt_slot].get_id();
					string id_missl=combus[vehicle_slot].get_id();

					//absolute miss vector in geographic coordinates and miss magnitude
					double tau=hit_time-time_m;
					MISS=(SBBML-STTML)*(tau/int_step)+SBTLM;
					miss=MISS.absolute();

					//angle between missile and target velocity vectors
					double thetax=angle(VTEL,VBEL)*DEG;
				
					//writing miss information to console
					//cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
					cout<<"\n"<<" *** Intercept of Missile_"<<id_missl<<" and target_"<<id_targ<<" ***\n";
					cout<<"      miss magnitude = "<<miss<<" m    intercept sim_time = "<<hit_time<<" sec\n";
					cout<<"      north = "<<MISS.get_loc(0,0)<<" m      east = "<<MISS.get_loc(1,0)
									<<" m        down = "<<MISS.get_loc(2,0)<<" m\n";
					cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg       gamma = "<<thtvlx<<" deg\n";    
					cout<<"      altitude at intercept = "<<alt<<" m   differential speed = "<<diff_speed<<" m/s\n";
					cout<<"      inclusive angle of vel. vectors = "<<thetax<<" deg  dynamic pressure = "<<pdynmc<<" Pa\n";
					cout<<"     Intercept condition 'trcond' = "<<trcond<<"\n\n";

					//diplaying banner on screen at the end of run
					scrn_banner();				
				}
				if(mterm>=0)
				{				
					//miss vector in intercept plane
					//setting flag for termination
					lconv=2;

					Matrix SBBML=SBEL-SBMEL;
					Matrix STTML=STEL-STMEL;

					//intercept time at point of closest approach
					hit_time=time_m-int_step*((SBBML-STTML)^SBTLM)/(SBBML^SBBML);

					//getting missile # and target #
					string id_targ=combus[tgt_slot].get_id();
					string id_missl=combus[vehicle_slot].get_id();

					if(mterm==1){
						//for MC: every run will have slightly different aspect angles
						//yaw angle of relative vel.vec. in intercept plane
						Matrix VBTET=VTBET*(-1);
						POLAR=VBTET.pol_from_cart();
						psiptx=POLAR[1]*DEG;
						thtptx=POLAR[2]*DEG-90;
					}
					if(mterm>=1){
						//for MC: average aspect angles 'psiptx' and 'thtptx' are provided by 'input.asc'

						// T.M. of intercept plane wrt local level
						Matrix TPT=mat2tr(psiptx*RAD,thtptx*RAD);
						TPL=TPT*TTL;
						SBTP=TPL*SBTL;
						Matrix SBBMP=TPL*SBBML;
						Matrix STBMP=SBBMP-SBTP;
						double ww=STBMP[2]/SBBMP[2];
						//miss vector in intercept plane coordinates
						MISS=SBBMP*ww-STBMP;

						miss=MISS.absolute();

						//angle between missile and target velocity vectors
						double thetax=angle(VTEL,VBEL)*DEG;

						//writing miss information to console
			//			cout<<"\n"<<" *** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
						cout<<"\n"<<" *** Intercept of Missile_"<<id_missl<<" and target_"<<id_targ<<" ***\n";
						cout<<"      miss in I-plane = "<<miss<<" m    intercept sim_time = "<<hit_time<<" sec\n";
						cout<<"      down = "<<MISS.get_loc(0,0)<<" m      right = "<<MISS.get_loc(1,0)<<" m\n";
						cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivlx<<" deg       gamma = "<<thtvlx<<" deg\n";    
						cout<<"      altitude at intercept = "<<alt<<" m   differential speed = "<<diff_speed<<" m/s\n";
						cout<<"      inclusive angle of vel. vectors = "<<thetax<<" deg  dynamic pressure = "<<pdynmc<<" Pa\n";
						cout<<"     Intercept condition 'trcond' = "<<trcond<<"\n\n";

						//diplaying banner on screen at the end of run
						scrn_banner();					
					}
				}
				//for debugging
				if(mterm==-1)
				{
					//getting missile # and target #
					string id_targ=combus[tgt_slot].get_id();
					string id_missl=combus[vehicle_slot].get_id();

					double miss=STBL.absolute();
					cout<<"\n"<<" *** Intercept of Missile_"<<id_missl<<" and target_"<<id_targ<<" ***\n";
					cout<<"      miss without interpolation = "<<miss<<" m\n";
				}
				//declaring missile and target 'dead (=0)
				combus[vehicle_slot].set_status(0);
				combus[tgt_slot].set_status(0);

			}//end of closing speed change

			//save from previous cycle
			SBTLM=SBTL;
			STMEL=STEL;
			SBMEL=SBEL;
			time_m=time;
		}//end of sphere of influence
	}//end of sensor guidance
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
	flat6[6].gets(lconv);
	missile[652].gets(miss);
	missile[654].gets_vec(MISS);
	missile[659].gets(mode);
	missile[660].gets(dbt);
	missile[661].gets(psiptx);
	missile[662].gets(thtptx);
}
