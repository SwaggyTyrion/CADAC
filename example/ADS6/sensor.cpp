///////////////////////////////////////////////////////////////////////////////
//FILE: 'sensor.cpp'
//
//Contains 'sensor' module of class 'Missile'
//
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'sensor' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned for 
//	IR seeker to missile[200-299]
//	RF seeker to missile[800-899]
// 
//170612 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_sensor()
{
	//Definition and initialization of module-variables
	//executive (to be used throughout the simulation)
	missile[2].init("STEL",0,0,0,"Position of rocket# - m","combus","","");
	missile[3].init("VTEL",0,0,0,"Velocity of rocket# - m","combus","","");
	missile[5].init("tgt_slot","int",0,"Target slot in combus ","combus","","");
	//common to IR and RF
    missile[200].init("mseek","int",0,"=1x: RF; =2x:IR - ND ","sensor","data","");
    missile[201].init("skr_dyn","int",0,"=0: Kinemtic, =1:Dynamic","sensor","data","");
    missile[202].init("isets1","int",0,"Sensor flag","sensor","init","");
    missile[203].init("epchac",0,"Epoch of start of sensor acquisition - s","sensor","init","");
    missile[204].init("fst_tgt_slot","int",0,"Slot of first rocket in 'combus' - ND","sensor","save","");
    missile[205].init("mtarget","int",0,"Target flag: =1:rocket; =2:aircraft - ND","sensor","data","");
	missile[207].init("dbtk",0,"Seeker missile-target distance - m","sensor","diag","plot,scrn");
    missile[279].init("thtpb",0,"Pitch pointing angle - rad","sensor","out","");
    missile[280].init("psipb",0,"Yaw pointing angle - rad","sensor","out","");
    missile[295].init("SBTL",0,0,0,"True missile wrt target displacement - m","sensor","diag","");
    missile[289].init("thtpbx",0,"Pitch pointing angle - deg","sensor","diag","plot");
    missile[290].init("psipbx",0,"Yaw pointing angle - deg","sensor","diag","plot");
	//IR gimbaled seeker 
    missile[231].init("dblind",0,"Blind range - m","sensor","data","");
    missile[232].init("ibreak","int",0,"Flag for sensor break-lock ND","sensor","init","");
	missile[233].init("racq_ir",0,"Acquisition range - m","sensor","data","");
    missile[234].init("dtimac_ir",0,"Rocket acquisition time - s","sensor","data","");
    missile[235].init("ehz",0,"Yaw error angle about z-axis - rad","sensor","dia","");
    missile[236].init("ehy",0,"Pitch error angle about y-axis - rad","sensor","dia","");
	missile[246].init("trtht",0,"Maximum pitch gimbal angle - rad","aerodynamics","data","");
	missile[247].init("trthtd",0,"Maximum pitch gimbal rate - rad/s","aerodynamics","data","");
	missile[248].init("trphid",0,"Maximum roll gimbal rate - rad/s","aerodynamics","data","");
	missile[249].init("trate",0,"Maximum tracking rate - rad/s","aerodynamics","data","");
	missile[250].init("gk",0,"K.F. gain - 1/s","sensor","data","");
    missile[251].init("zetak",0,"K.F. damping","sensor","data","");
    missile[252].init("wnk",0,"K.F. natural frequency - rad/s","sensor","data","");
	missile[253].init("biast",0,"Pitch gimbal bias errors - rad","sensor","data","");
    missile[254].init("randt",0,"Pitch gimbal random errors - rad","sensor","data","");
    missile[255].init("biasp",0,"Roll gimbal bias error - rad","sensor","data","");
    missile[256].init("randp",0,"Roll gimbal bias error - rad","sensor","data","");
    missile[257].init("wlq1d",0,"Pitch sight line spin derivative - rad/s^2","sensor","state","");
    missile[258].init("wlq1",0,"Pitch sight line spin - rad/s","sensor","state","");
    missile[259].init("wlqd",0,"Pitch pointing rate derivative - rad/s^2","sensor","state","");
    missile[260].init("wlq",0,"Pitch pointing rate - rad/s","sensor","state","");
    missile[261].init("wlr1d",0,"Yaw sight line spin derivative - rad/s^2","sensor","state","");
    missile[262].init("wlr1",0,"Yaw sight line spin - rad/s","sensor","state","");
    missile[263].init("wlrd",0,"Yaw pointing rate derivative - rad/s^2","sensor","state","");
    missile[264].init("wlr",0,"Yaw pointing rate - rad/s","sensor","state","");
    missile[265].init("wlq2d",0,"Second state variable deriv in K.F. - rad/s^3","sensor","state","");
    missile[266].init("wlq2",0,"Second state variable in K.F. - rad/s^2","sensor","state","");
    missile[267].init("wlr2d",0,"Second state variable der in K.F. - rad/s^3","sensor","state","");
    missile[268].init("wlr2",0,"Second state variable in K.F. - rad/s^2","sensor","state","");
    missile[269].init("fovyaw_ir",0,"Half yaw field-of-view at acquisition - rad","sensor","data","");
    missile[270].init("fovpitch_ir",0,"Half positive pitch field-of-view at acquis. - rad","sensor","data","");
    missile[272].init("daim",0,"Dist from targ to initiate aimpoint mode - m","sensor","data","");
    missile[273].init("BIASAI",0,0,0,"Bias error of aimpoint mode in target coor - m","sensor","data","");
    missile[274].init("BIASSC",0,0,0,"Bias error of hot spot mode in target coor - m","sensor","data","");
    missile[275].init("RANDSC",0,0,0,"Random error of hot spot mode in targ coor - m","sensor","data","");
    missile[276].init("epy",0,"Error of pointing in pitch - rad","sensor","diag","plot");
    missile[277].init("dta",0,"True distance between target and aircraft - m","sensor","out","plot");
    missile[278].init("epz",0,"Error of pointing in yaw - rad","sensor","diag","plot");
    missile[281].init("ththb",0,"Head pitch angle - rad","sensor","diag","");
    missile[282].init("phihb",0,"Head roll angle - rad","sensor","diag","");
    missile[283].init("TPB",0,0,0,0,0,0,0,0,0,"I/G TM of pointing axes wrt body axes","sensor","init","");
    missile[284].init("THB",0,0,0,0,0,0,0,0,0,"I/G TM of head axes wrt body axes","sensor","init","");
    missile[285].init("dvbtc",0,"Closing velocity computed by INS - m/s","sensor","diag","");
    missile[286].init("EAHH",0,0,0,"Aimpoint displacement wrt center of F.P. - rad","sensor","diag","");
    missile[287].init("EPHH",0,0,0,"Computer pointing error of sensor wrt center of F.P.","sensor","diag","");
    missile[288].init("EAPH",0,0,0,"Aimpoint to computer pointing displacement - rad","sensor","diag","");
    missile[291].init("sigdy",0,"Pitch LOS rate in pointing axes  - rad/s","sensor","out","plot");
    missile[292].init("sigdz",0,"Yaw LOS rate in pointing axes  - rad/s","sensor","out","plot");
    missile[293].init("biaseh",0,"Image blur and pixel bias errors - rad","sensor","data","");
    missile[294].init("randeh",0,"Image blur and pixel random errors - rad","sensor","data","");
	//RF gimbaled seeker
    missile[803].init("racq_rf",0,"RF seeker acquisition range - m","sensor","data","");
    missile[804].init("dtimac_rf",0,"RF seeker acquisition time - sec","sensor","data","");
	missile[805].init("temp_resx",290,"Temperature of resistor - K","sensor","data","");
	missile[807].init("biasaz",0,"Azimuth boresight error - rad","sensor","data","");
	missile[808].init("biasel",0,"Elevation boresight error - rad","sensor","data","");
	missile[809].init("freqghz",0,"Seeker operating freequency - GHz","sensor","data","");
	missile[810].init("rngegw",0,"Range gate width - m","sensor","data","");
	missile[811].init("thta_3db",0,"Nominal beam width - deg","sensor","data","");
	missile[812].init("powrs",0,"Seeker average power - W","sensor","data","");    
	missile[813].init("gainsdb",0,"Transmit gain - dB","sensor","data","");
	missile[814].init("gainmdb",0,"Receive gain - dB","sensor","data","");
	missile[815].init("tgt_rcs",0,"target radar cross section - m^2","sensor","data","");
	missile[816].init("rlatmodb",0,"Atmospheric loss - dB","sensor","data","");
	missile[817].init("rltotldb",0,"Total system loss - dB","sensor","data","");
	missile[818].init("dwltm",0,"Dwell time - s","sensor","data","");
	missile[819].init("rnoisfgdb",0,"Noise figure - dB","sensor","data","");
	missile[820].init("plc5",0,"Coeff.of poly.curve fit of power loss","sensor","data","");
	missile[821].init("plc4",0,"Coeff.of poly.curve fit of power loss","sensor","data","");
	missile[822].init("plc3",0,"Coeff.of poly.curve fit of power loss","sensor","data","");
	missile[823].init("plc2",0,"Coeff.of poly.curve fit of power loss","sensor","data","");    
	missile[824].init("plc1",0,"Coeff.of poly.curve fit of power loss","sensor","data","");
	missile[825].init("plc0",0,"Coeff.of poly.curve fit of power loss","sensor","data","");
	missile[826].init("biasgl1",0,"Glint Gaussian bias in target x-dir - m","sensor","data","");
	missile[827].init("biasgl2",0,"Glint Gaussian bias in target y-dir - m","sensor","data","");
	missile[828].init("biasgl3",0,"Glint Gaussian bias in target z-dir - m","sensor","data","");
	missile[829].init("randgl1",0,"Glint Markov noise in target x-dir - m","sensor","data","");
	missile[830].init("randgl2",0,"Glint Markov noise in target y-dir - m","sensor","data","");
	missile[831].init("randgl3",0,"Glint Markov noise in target z-dir - m","sensor","data","");
	missile[832].init("fovlim_rfx",0,"Half field of view limit (@ -1db) - deg","sensor","data","");
	missile[833].init("forlim_rfx",0,"Half field of regard limit - deg","sensor","data","");
	missile[834].init("gain_rf",0,"RF Gain in tracking loop - 1/sec","sensor","data","");
	missile[838].init("aztbx",0,"Azimuth target aspect angle - deg","sensor","dia","plot");
	missile[839].init("eltbx",0,"Elevation target aspect angle - deg","sensor","dia","plot");
	missile[841].init("dab",0," LOS distance - m","sensor","out","plot");
	missile[842].init("ddab",0,"LOS range-rate (negative) - m/s","sensor","out","plot");
	missile[843].init("pwr_loss_db",0,"Power loss due to look angle - dB","sensor","diag","");
	missile[844].init("snr_db",0,"Signal to noise ratio - dB","sensor","diag","plot");
	missile[845].init("onax",0,"Off-the-nose angle - deg","sensor","diag","plot");
	missile[855].init("epaz_rf_saved",0,"Azimuth error saved for Markov - rad","sensor","save","");
	missile[856].init("epel_rf_saved",0,"Elevation error saved for Markov - rad","sensor","save","");
	missile[857].init("range_rf_saved",0,"Range error saved for Markov - m","sensor","save","");
	missile[858].init("rate_rf_saved",0,"Range-rate error saved for Markov - m/s","sensor","save","");
	missile[860].init("epsic",0,"Yaw tracking error corrupted by glint - rad","sensor","dia","plot");
	missile[861].init("ethtc",0,"Pitch tracking error corrupted by glint - rad","sensor","dia","plot");
	missile[862].init("mepsit",0,"Monopulse error in yaw channel - rad","sensor","dia","");
	missile[863].init("methtt",0,"Monopulse error in pitch channel - rad","sensor","dia","");
    missile[864].init("psisb",0,"Yaw gimbal angle - rad","sensor","state","plot");
    missile[865].init("psisbd",0,"Yaw gimbal angle derivative - rad/s","sensor","state","");
    missile[866].init("thtsb",0,"Pitch gimbal angle - rad","sensor","state","plot");
    missile[867].init("thtsbd",0,"Pitch gimbal angle derivative - rad/s","sensor","state","");
    missile[868].init("lamdqb",0,"Pitch LOS rate in body coor  - rad/s","sensor","out","plot");
    missile[869].init("lamdrb",0,"Yaw LOS rate in body coor - rad/s","sensor","out","plot");
}	
///////////////////////////////////////////////////////////////////////////////
//Sensor module
//Member function of class 'Missile'
//
// Operation flag of sensor during execution
// mseek = |skr_type|skr_mode| 
//			skr_type  =	1: RF gimbaled seeker
//					  = 2: IR gimbaled seeker 
//					 skr_mode =	1: Sensor off 
//					            2: Sensor enabled (input, or set internally when break-lock occured)
//								3: Acquisition mode (set internally, when missile is within acquisition range)
//								4: Sensor lock-on (set internally, when acquisition time has elapsed)
//								5: Sensor within blind range (set internally). Output held constant (IR only)
//
// skr_dyn = 0 kinematic sensor						
//         = 1 dynamic sensor						
//													
// RF Seeker
//		This is a gimbaled seeker with outer gimbal pitching and inner gimbal yawing
//		The sensor operates in the Ku band
//		Used with 'mguide 7' 
//
// IR Seeker
//		This is a gimbaled seeker with outer gimbal rolling and inner gimbal pitching
//		The sensor operates in the 8.5-12.5 micro-meter wave band
//		Images of the target are created on its 128x128 focal plane array
//		It homes in on a selected hot spot
//		Used with 'mguide 6'
//
//170612 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor(Packet *combus,int num_vehicles,int vehicle_slot,double sim_time,double int_step)
{
	//local variables
	Variable *data_t;
	double ehz(0),ehy(0);
	int msl_slot(0);
	int tgt_slot(0);

	//local module-variables
	double dbtk(0);
	Matrix STEL(3,1);
	Matrix VTEL(3,1);
	Matrix SBTL(3,1);
	Matrix TTL(3,3);TTL.identity(); //shortcut, eventually should be subsribed from 'combus'
	double psipb(0),thtpb(0);
	double sigdy(0),sigdz(0);
	double ththb(0),phihb(0);	
	double psipbx(0),thtpbx(0);
	double dta(0);
	double dab(0), ddab(0);
	double aztbx(0),eltbx(0);
	double lamdrb(0),lamdqb(0);

	//localizing module-variables
	//input data
	int mseek=missile[200].integer();
	int skr_dyn=missile[201].integer();
	int mtarget=missile[205].integer();
	double racq_ir=missile[233].real();
	double dtimac_ir=missile[234].real();
	double fovyaw_ir=missile[269].real();
	double fovpitch_ir=missile[270].real();
	double racq_rf=missile[803].real();
	double dtimac_rf=missile[804].real();
	double fovlim_rfx=missile[832].real();
	double forlim_rfx=missile[833].real();
	//getting saved value
	int isets1=missile[202].integer();
	double epchac=missile[203].real();
	int fst_tgt_slot=missile[204].integer();
	double timeac=missile[238].real();
	Matrix THB=missile[284].mat();
	double epsic=missile[860].real();
	double ethtc=missile[861].real();

	//input from other modules
	int stop=flat6[5].integer();
	double time=flat6[0].real();
	Matrix SBEL=flat6[219].vec();
	int trcond=missile[180].integer();
	int mguide=missile[400].integer();
	//-------------------------------------------------------------------------
	//decoding mseek
	int skr_type=mseek/10;
	int skr_mode=mseek%10;

	//*downloading target coordinates
	//rocket-target
	if(mtarget==1)
	{
		//finding 1st rocket-target slot in 'combus'
		char number[4];
		sprintf(number,"%i",1);
		string first_rocket_id="r"+string(number);
		for(int i=0;i<num_vehicles;i++)
		{
			string id=combus[i].get_id();
			if (id==first_rocket_id) fst_tgt_slot=i;
		}
		//pairing rocket-target slot to 'this' missile
		//!!!assumption for 'input.asc': MISSILE6 objects are loaded first in  (usual order)
		//  and there is a one-on-one missile-rocket assignement (m1->r1, m2->r2, m3->r3)
		if(vehicle_slot==0) msl_slot=0; 
		if(vehicle_slot==1) msl_slot=1;
		if(vehicle_slot==2) msl_slot=2; 
		tgt_slot=fst_tgt_slot+msl_slot;
		//downloading from 'combus' target variables for seeker tracking
		data_t=combus[tgt_slot].get_data();
		STEL=data_t[4].vec();
		VTEL=data_t[5].vec();
		dta=data_t[10].real();
	}
	//aircraft-target
	else if(mtarget==2)
	{
		//finding 1st aircraft-target slot in 'combus'
		char number[4];
		sprintf(number,"%i",1);
		string first_aircraft_id="a"+string(number);
		for(int i=0;i<num_vehicles;i++)
		{
			string id=combus[i].get_id();
			if (id==first_aircraft_id) fst_tgt_slot=i;
		}
		//pairing aircraft-target slot to 'this' missile
		//!!!assumption for 'input.asc': MISSILE6 objects are loaded first in  (usual order)
		//  and there is a one-on-one missile-aircraft assignement (m1->a1, m2->a2, m3->a3)
		if(vehicle_slot==0) msl_slot=0; 
		if(vehicle_slot==1) msl_slot=1;
		if(vehicle_slot==2) msl_slot=2; 
		tgt_slot=fst_tgt_slot+msl_slot;	
		//downloading from 'combus' target variables for seeker tracking
		data_t=combus[tgt_slot].get_data();
		STEL=data_t[4].vec();
		VTEL=data_t[5].vec();
		dta=data_t[10].real();
	}

	//establishing true displacement vector of missile wrt target
	SBTL=SBEL-STEL;
	//seeker distance to target
	dbtk=SBTL.absolute();

	//***RF gimbaled seeker
	if(skr_type==1)
	{
		//RF seeker is enabled
		if(skr_mode==2){
			isets1=1;
			//is target within RF acquisition range
			if(dbtk<racq_rf)
				skr_mode=3;
		}
		//**RF seeker in acquisition mode
		if(skr_mode==3)
		{
			//initialization
			if(isets1==1){
				isets1=0;
				//starting clock for duration of acquisition
				epchac=time;
				sensor_kin(thtpb,psipb,sigdy,sigdz,lamdrb,lamdqb,ddab, SBTL,VTEL,dbtk);
			}
			//RF seeker acquisition(for dynamic seeker, target must also be in the field-of-regard)
			if(skr_dyn==1)
			{
				sensor_rf_dyn(lamdrb,lamdqb,dab,ddab,ethtc,epsic,aztbx,eltbx, SBTL,int_step);
				timeac=time-epchac;
				if(timeac>dtimac_rf){
					if((fabs(aztbx)<=forlim_rfx)&&(fabs(eltbx)<=forlim_rfx))
						skr_mode=4;
					else
						trcond=5;
				}
			}
			else{
				sensor_kin(thtpb,psipb,sigdy,sigdz,lamdrb,lamdqb,ddab, SBTL,VTEL,dbtk);

				timeac=time-epchac;
				if(timeac>dtimac_rf)
					skr_mode=4;
			}
		}
		//**RF seeker lock-on mode
		else if(skr_mode==4)
		{
			if(skr_dyn==1)
			{
				sensor_rf_dyn(lamdrb,lamdqb,dab,ddab,ethtc,epsic,aztbx,eltbx, SBTL,int_step);
			}
			else
			{
				sensor_kin(thtpb,psipb,sigdy,sigdz,lamdrb,lamdqb,ddab, SBTL,VTEL,dbtk);
			}
		}
		//diagnostics
		thtpbx=thtpb*DEG;
		psipbx=psipb*DEG;

	}//end of RF seeker

	//***IR gimbaled seeker
	if(skr_type==2)
	{
		//IR sensor is enabled
		if(skr_mode==2){
			isets1=1;
			//is target within IR acquisition range
			if(dbtk<racq_ir)
				skr_mode=3;
		}
		//**IR sensor in acquisition mode
		if(skr_mode==3){
			//initializing time counter, state variables and TM matrix 
			if(isets1==1){
				isets1=0;
				//starting clock for duration of acquisition
				epchac=time;
				sensor_kin(thtpb,psipb,sigdy,sigdz,lamdrb,lamdqb,ddab, SBTL,VTEL,dbtk);
				sensor_ir_uthpb(ththb,phihb, psipb,thtpb);
				THB=sensor_ir_thb(ththb,phihb);
			}
			//IR seeker acquisition(for dynamic sensor, target must be in field-of-view)
			if(skr_dyn==1)
			{
				sensor_ir_dyn(mseek,mguide,thtpb,psipb,sigdy,sigdz,ehz,ehy,THB, SBTL,dbtk,int_step);
			    timeac=time-epchac;
			   if(timeac>dtimac_ir){
					if((fabs(ehz)<=fovyaw_ir)&&(fabs(ehy)<=fovpitch_ir))
						skr_mode=4;
					else
						trcond=5;
			   }
			}
			else{
				sensor_kin(thtpb,psipb,sigdy,sigdz,lamdrb,lamdqb,ddab, SBTL,VTEL,dbtk);
				timeac=time-epchac;
				if(timeac>dtimac_ir)
					skr_mode=4;
			}
		}
		//**IR sensor lock-on mode
		if(skr_mode==4)
		{
			if(skr_dyn==1)
			{
				sensor_ir_dyn(mseek,mguide,thtpb,psipb,sigdy,sigdz,ehz,ehy,THB, SBTL,dbtk,int_step);
			}
			else
			{
				sensor_kin(thtpb,psipb,sigdy,sigdz,lamdrb,lamdqb,ddab, SBTL,VTEL,dbtk);
			}
		}
		//diagnostics
		thtpbx=thtpb*DEG;
		psipbx=psipb*DEG;

	}//end of IR seeker

	//reconstituting mseek
	mseek=10*skr_type+skr_mode;

	//-------------------------------------------------------------------------
	//*Common to RF and IR
	//output to other modules
	missile[2].gets_vec(STEL); 
	missile[3].gets_vec(VTEL);
	missile[5].gets(tgt_slot);
	missile[180].gets(trcond);
	missile[200].gets(mseek);
	missile[277].gets(dta);
	//saving for next cylcle
	missile[202].gets(isets1);
	missile[203].gets(epchac);
	missile[204].gets(fst_tgt_slot);
	missile[238].gets(timeac);
	//diagnostics
    missile[207].gets(dbtk);
	missile[289].gets(thtpbx);
	missile[290].gets(psipbx);
	missile[295].gets_vec(SBTL);

	//*RF sensor
	//output to other modules
	missile[842].gets(ddab);
	missile[868].gets(lamdqb);
	missile[869].gets(lamdrb);
	//saved values
	missile[860].gets(epsic);
	missile[861].gets(ethtc);
	//diagnostics
	missile[838].gets(aztbx);
	missile[839].gets(eltbx);
	missile[841].gets(dab);
	missile[842].gets(ddab);

	//*IIR sensor
	//ouput to other modules
    missile[400].gets(mguide);
	missile[279].gets(thtpb);
	missile[280].gets(psipb);
	missile[291].gets(sigdy);
	missile[292].gets(sigdz);

	//saving value for next cycle
	missile[284].gets_mat(THB);
	//diagnostics
	missile[200].gets(mseek);
	missile[235].gets(ehz);
	missile[236].gets(ehy);
}
///////////////////////////////////////////////////////////////////////////////
//RF seeker function
//Member function of class 'missile'
// (1) Corrupts the true target centroid by glint.
// (2) Introduces boresight error into the antenna azimuth and elevation measurements.
// (3) Introduces thermal noise into range and range-rate measurements.
//
// Argument Output (RF):
//                 lamdrb= Yaw LOS rate in body axes - rad/s
//                 lamdqb= Pitch LOS rate in body axes - rad/s 
//                 dab = range to target measurement - m
//                 ddab = range-rate to target measurement - m/s 
//                 ethtc = pitch tracking error - rad
//                 epsic = yaw tracking error - rad
//				   aztbx = azimuth target aspect angle - deg"
//				   eltbx = elevation target aspect angle - deg
// Parameter input:
//                 SBTL(3) = position of missile wrt target - m
//				   int_step = integration step size - s
//
//170722 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor_rf_dyn(double &lamdrb,double &lamdqb,double &dab,double &ddab, 
					    double &ethtc,double &epsic,double &aztbx, double &eltbx, Matrix SBTL,double int_step)
{	
	//local variables
	double aztb(0);
	double eltb(0);
	Matrix TPB(3,3);

	//local module-variables
	double pwr_loss_db(0);
	double snr_db(0);
	double onax(0);
	double mepsit(0);
	double methtt(0);

	//localizing module-variables
	//input data
	double temp_resx=missile[805].real();
	double biasaz=missile[807].real(); 
	double biasel=missile[808].real();
	double freqghz=missile[809].real();
	double rngegw=missile[810].real();
	double thta_3db=missile[811].real();
	double powrs=missile[812].real();
	double gainsdb=missile[813].real();
	double gainmdb=missile[814].real();
	double tgt_rcs=missile[815].real();
	double rlatmodb=missile[816].real();
	double rltotldb=missile[817].real();
	double dwltm=missile[818].real();
	double rnoisfgdb=missile[819].real();
	double plc5=missile[820].real();
	double plc4=missile[821].real();
	double plc3=missile[822].real();
	double plc2=missile[823].real();
	double plc1=missile[824].real();
	double plc0=missile[825].real();
	//sate variables
	double gain_rf=missile[834].real();
	double psisb=missile[864].real();
	double psisbd=missile[865].real();
	double thtsb=missile[866].real();
	double thtsbd=missile[867].real();
	//getting saved values
	double epaz_saved=missile[855].real();
	double epel_saved=missile[856].real();
	double range_saved=missile[857].real();
	double rate_saved=missile[858].real();
	//from other modules
	double time=flat6[0].real();
	Matrix TBL=flat6[120].mat();
	Matrix VBEL=flat6[233].vec();
	Matrix VTEL=missile[3].vec();
	Matrix WBECB=missile[306].vec();
	//-------------------------------------------------------------------------	
	//**Calculating target aspect angles relative to missile body
	Matrix STBL=SBTL*(-1);
	Matrix STBB=TBL*STBL;
	//calculating TM of 'pointing' (IR terminology) wrt body coordinates TPB
	//(idealized because of using STBB instead of SOBB)
	Matrix POLAR=STBB.pol_from_cart();
	aztb=POLAR[1];
	eltb=POLAR[2];
	//pointing TM
	TPB=mat2tr(aztb,eltb);
	//diagnostics:
	aztbx=aztb*DEG;
	eltbx=eltb*DEG;

	//**Corrupting LOS by glint
	Matrix SOTLC=sensor_rf_glint();
	Matrix SOBLC=SOTLC-SBTL;
	double dobc=SOBLC.absolute();	
	//range rate glint
	Matrix UOBLC=SOBLC.univec3();
	Matrix VTBL=VTEL-VBEL;
	double rdobc=UOBLC^VTBL;

	//**Building the inner gimbal wrt body TM (inner gimbal yaw 'psisb', outer gimbal pitch 'thtsb')
	//ref: Zipfel, Eq. (9.83)
	//'psisb' and 'thtsb' come from the previously saved state variable) 
	Matrix TSB(3,3);
	double cpsisb=cos(psisb);
	double spsisb=sin(psisb);
	double cthtsb=cos(thtsb);
	double sthtsb=sin(thtsb);
	TSB.assign_loc(0,0,cpsisb*cthtsb);
	TSB.assign_loc(0,1,spsisb);
	TSB.assign_loc(0,2,-cpsisb*sthtsb);
	TSB.assign_loc(1,0,-spsisb*cthtsb);
	TSB.assign_loc(1,1,cpsisb);
	TSB.assign_loc(1,2,spsisb*sthtsb);
	TSB.assign_loc(2,0,sthtsb);
	TSB.assign_loc(2,1,0);
	TSB.assign_loc(2,2,cthtsb);

	//**Calculating pitch and yaw tracking errors
	//expressing LOS corrupted by glint in inner gimbal axes
	Matrix SOBS=TSB*TBL*SOBLC;
	double sobs1=SOBS.get_loc(0,0);
	double sobs2=SOBS.get_loc(1,0);
	double sobs3=SOBS.get_loc(2,0);
	//tracking errors
	ethtc=atan2(-sobs3,sobs1);
	epsic=atan2(sobs2,sobs1);

	//**Computing signal to noise ratio
	//computing the angle between LOS and antenna centroid axis
	onax=sqrt(ethtc*ethtc+epsic*epsic)*DEG;
	//converting data from db to rational numbers
	double k_noise=PI/2;
	double wvelngth=(2.998e8)/(freqghz*10.e8);
	double gains=pow(10.,(gainsdb/10.));
	double gainm= pow(10.,(gainmdb/10.));
	double rnoisfg=pow(10.,(rnoisfgdb/10.));
	double rlatmo=pow(10.,(rlatmodb/10.));
	double rltotl=pow(10.,(rltotldb/10.));
	//getting off-the-nose transmitting power loss from curve
	double pwr_loss=-(plc5*10.e-11)*pow(onax,5)+(plc4*10.e-9)*pow(onax,4)
				-(plc3*10.e-7)*pow(onax,3)+(plc2*10.e-5)*pow(onax,2)
				-(plc1*10.e-3)*onax+plc0;
	if(pwr_loss>1)pwr_loss=1;
	//transmit power
	double powrt=powrs-pwr_loss;
	//signal power
	double ps=powrt*gains*rlatmo*rlatmo*tgt_rcs*gainm*wvelngth*wvelngth
				/(pow(4.*PI,3)*pow(dobc,4)*rltotl);
	double pn=KBOLTZ*temp_resx*rnoisfg*(1./dwltm);
	double snr=ps/pn;

	//**Modeling thermal noise
	double sigma_mp=sqrt(dwltm/int_step)*(thta_3db*RAD)/(k_noise*sqrt(snr)); 
	double nepsi_mp=markov(sigma_mp,100.,time,int_step,epaz_saved); //bcor = 100 Hz thermal noise bandwidth
	double netht_mp=markov(sigma_mp,100.,time,int_step,epel_saved); //bcor = 100 Hz thermal noise bandwidth

	//**Establishing total monopulse errors
	mepsit=biasaz+nepsi_mp;
	methtt=biasel+netht_mp;

	//**Getting inertial LOS rates
	//LOS rates in inner gimbal coordiantes
	double lamdrs=gain_rf*(epsic+mepsit);
	double lamdqs=gain_rf*(ethtc+methtt);

	//**Inertial LOS rates 'lamdqb', 'lamdrb'  in body coordinates (out to 'guidance_term_pronav()', mguide=7)
	Matrix WOES(3,1);
	WOES.build_vec3(0,lamdqs,lamdrs);
	Matrix WOEB=~TSB*WOES;
	lamdqb=WOEB.get_loc(1,0);
	lamdrb=WOEB.get_loc(2,0);

	//**Range  measurements with thermal noise
	//range thermal noise sigma
	double sigma_range=sqrt(dwltm/int_step)*rngegw/(k_noise*sqrt(snr));
	//range thermal noise
	double eps_range=markov(sigma_range,10.,time,int_step,range_saved); //bcor = 10 Hz thermal noise bandwidth
	//range measurement
	dab=dobc+eps_range;
	
	//**Range-rate measurements with thermal noise
	//velocity gate width
	double vgw = 200*wvelngth/2;
	//range-rate thermal sigma
	double sig_range_rate = sqrt(dwltm/int_step)*vgw/(k_noise*sqrt(snr));
	double eps_range_rate=markov(sig_range_rate,10.,time,int_step,rate_saved); //bcor = 10 Hz thermal noise bandwidth
	//range-rate measurement
	ddab=rdobc+eps_range_rate;

	//diagnostics
	pwr_loss_db=10.*log10(pwr_loss);
	snr_db=10.*log10(snr); 

	//**Seeker feedback loops
	//*Gyro compensation
	//converting strap-down gyro output WBECB from INS to inner gimbal coordinates
	Matrix WBES=TSB*WBECB;
	double wbes2=WBES.get_loc(1,0);
	double wbes3=WBES.get_loc(2,0);
	//compensating for yaw body rates using INS gyro
	psisbd=lamdrs-wbes3;
	//compensating for pitch body rates using INS gyro
	thtsbd=(lamdqs-wbes2)/cpsisb;

	//*Integrating to get gimbal angles
	//yaw gimbal angle
	double psisbd_new=psisbd;
	psisb=integrate(psisbd_new,psisbd,psisb,int_step);
	psisbd=psisbd_new;
	//pitch gimbal angle
	double thtsbd_new=thtsbd;
	thtsb=integrate(thtsbd_new,thtsbd,thtsb,int_step);
	thtsbd=thtsbd_new;
	
	//**Compatibiity with IR LOS rates in pointing axes
	//inertial LOS rate in pointing coordinates
	Matrix WOEP=TPB*WOEB;
	//Inertial LOS rates 'sigdy', 'sigdz' in pointing coordinates (out to 'guidance_term_comp()', mguide=6) 
	//pitch sight-line spin about y-pointing axis
	double sigdy=WOEP.get_loc(1,0);
	//yaw sight-line spin about z-pointing axis
	double sigdz=WOEP.get_loc(2,0);
	
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[864].gets(psisb);
	missile[865].gets(psisbd);
	missile[866].gets(thtsb);
	missile[867].gets(thtsbd);
	//saving values
	missile[855].gets(epaz_saved);
	missile[856].gets(epel_saved);
	missile[857].gets(range_saved);
	missile[858].gets(rate_saved);
	//output to other modules
	missile[291].gets(sigdy);
	missile[292].gets(sigdz);
	//diagnostics
	missile[843].gets(pwr_loss_db);
	missile[844].gets(snr_db);
	missile[845].gets(onax);
	missile[862].gets(mepsit);
	missile[863].gets(methtt);
}
///////////////////////////////////////////////////////////////////////////////
//Glint
//Member function of class 'missile'
//Simple glint model, corrupting the aimpoint along the three target axes
//
// This function performs the following functions:
// (1)  Generates a random bias displacement from true target position
// (2)  Generates a gaussian Markov displacement from true target position
// Return output:
//                 SOTL(3)=Displacement of apparent target centroid O from
//                         true target center of mass T in L coord. - m
//
//170713 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Missile::sensor_rf_glint()
{
	//local variables
	Matrix BIASGL(3,1);
	Matrix RANDGL(3,1);
	
	//localizing module-variables
	//input data
//	Matrix TTL=missile[4].mat();
	Matrix TTL(3,3);TTL.identity(); //shortcut, eventually has to comes from 'combus'
	double biasgl1=missile[826].real();
	double biasgl2=missile[827].real();
	double biasgl3=missile[828].real();
	double randgl1=missile[829].real();
	double randgl2=missile[830].real();
	double randgl3=missile[831].real();
	//-------------------------------------------------------------------------
	//calculating displacement of aimpoint by glint
	BIASGL.build_vec3(biasgl1,biasgl2,biasgl3);
	RANDGL.build_vec3(randgl1,randgl2,randgl3);
	Matrix SOTT=RANDGL+BIASGL;
	Matrix TLT=TTL.trans();
	Matrix SOTL=TLT*SOTT;
	//-------------------------------------------------------------------------
	return SOTL;
}
///////////////////////////////////////////////////////////////////////////////
//IIR sensor function
//Member function of class 'Missile'
// (1) Given true target relative geometry it determines inertial
//     LOS rates in pitch and yaw, corrupted by these errors:
//         Rocket Scintillation
//         Blur, pixel quatization and bias
//         Gimbal dynamics, quantization and bias
// (2) Determines Aimpoint off-set from computer determined sensor axis
//     in Focal Plane (F.P.) array EAPH(3)
// (3) Models Kalman Filter dynamics (generates inertial LOS rates)
// (4) Models strap-down gyro feedback and gimbal kinematics
// (5) Allows for aimpoint selection and correction.
//
// Argument Output:
//                 mseek: If break lock occured reset to 2 (acquisition)
//                 mguide: If break lock occured reset to 2 (midcourse)
//                 thtpb= Pitch pointing angle - rad (used in 'guidance')
//                 psipb= Yaw pointing angle - rad (used in 'guidance')
//                 sigdy= Pitch sight line spin - rad/s (used in 'guidance')
//                 sigdz= Yaw sight line spin - rad/ (used in 'guidance')
//                 ehz=	  Yaw sensor error angle - rad
//                 ehy=   Pitch sensor error angle - rad
//                 THB(3,3)= Transf matrix of head wrt body axes (saving for next cycle) 
// Argument Input:
//                 SBTL(3)= Position of missile wrt target - m
//                 dbtk=     Distance between missile and target - m
//				   int_step= Integration step size - s
//
//020102 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor_ir_dyn(int &mseek,int &mguide,double &thtpb,double &psipb,double &sigdy,
					   double &sigdz,double &ehz,double &ehy,Matrix &THB, Matrix SBTL,
					   double dbtk,double int_step)
{
	//local variables
	Matrix EAPP(3,1);
	Matrix U1PP(3,1),U1HH(3,1);
	Matrix WBEP(3,1);
	double thtpbd,psipbd;
	double ththbc,phihbc,phihbd;

	//local module-variables
	double epz(0),epy(0);
	double ththb(0),phihb(0);
	Matrix EAHH(3,1);
	Matrix EPHH(3,1);
	Matrix EAPH(3,1);

	//localizing module-variables
	//input data
	double dblind=missile[231].real();
	int ibreak=missile[232].integer();
	double trtht=missile[246].real();
	double trthtd=missile[247].real();
	double trphid=missile[248].real();
	double trate=missile[249].real();
	double gk=missile[250].real();
	double zetak=missile[251].real();
	double wnk=missile[252].real();
	double biast=missile[253].real();
	double randt=missile[254].real();
	double biasp=missile[255].real();
	double randp=missile[256].real();
	double biaseh=missile[293].real();
	double randeh=missile[294].real();
	//initialization
	Matrix TPB=missile[283].mat();
	//input from other modules
	Matrix TBL=flat6[120].mat();
	Matrix TTL=missile[3].mat();
	int trcond=missile[180].integer();
	Matrix WBECB=missile[306].vec();
	//state variables
	double wlq1d=missile[257].real();
	double wlq1=missile[258].real();
	double wlqd=missile[259].real();
	double wlq=missile[260].real();
	double wlr1d=missile[261].real();
	double wlr1=missile[262].real();
	double wlrd=missile[263].real();
	double wlr=missile[264].real();
	double wlq2d=missile[265].real();
	double wlq2=missile[266].real();
	double wlr2d=missile[267].real();
	double wlr2=missile[268].real();
	//diagnostic
	double time=flat6[0].real();
	//-------------------------------------------------------------------------
	//aimpoint modulation
	Matrix THL=THB*TBL;
	Matrix SBTH=THL*SBTL;
	Matrix SATH=sensor_ir_aimp(THL,TTL,dbtk);
	Matrix SABH=SATH-SBTH;

	//error angles
	double sabh1=SABH.get_loc(0,0);
	double sabh2=SABH.get_loc(1,0);
	double sabh3=SABH.get_loc(2,0);
    double ey=atan2(-sabh3,sabh1);
    double ez=atan2(sabh2,sabh1);
	//error angle corrupted by blur and bias
    ehy=ey+biaseh+randeh;
    ehz=ez+biaseh+randeh;
	EAHH.build_vec3(0,ehz,-ehy);
	//T.M. matrices
	Matrix TBH=THB.trans();//THB of previous integration cycle is used
	Matrix TPH=TPB*TBH; 
	Matrix THP=TPH.trans();
	//pointing error angles
	U1PP.build_vec3(1,0,0);
	U1HH.build_vec3(1,0,0);
	EPHH=THP*U1PP-U1HH;
	EAPH=EAHH-EPHH;
	EAPP=TPH*EAPH;
	epy=-EAPP.get_loc(2,0);
	epz=EAPP.get_loc(1,0);

	//sight line spin estimator (kalman filter represented by 2nd order lag)
	double wsq=wnk*wnk;
	double gg=gk*wsq;
	//yaw channel
	double wlr1d_new=wlr2;
	wlr1=integrate(wlr1d_new,wlr1d,wlr1,int_step);
	wlr1d=wlr1d_new;
	double wlr2d_new=gg*epz-2.*zetak*wnk*wlr1d-wsq*wlr1;
	wlr2=integrate(wlr2d_new,wlr2d,wlr2,int_step);
	wlr2d=wlr2d_new;
	//pitch channel
	double wlq1d_new=wlq2;
	wlq1=integrate(wlq1d_new,wlq1d,wlq1,int_step);
	wlq1d=wlq1d_new;
	double wlq2d_new=gg*epy-2.*zetak*wnk*wlq1d-wsq*wlq1;
	wlq2=integrate(wlq2d_new,wlq2d,wlq2,int_step);
	wlq2d=wlq2d_new;

	//output to guidance module: LOS rates in pointing coord
    sigdz=wlr1;
    sigdy=wlq1;

	//look angle control
	WBEP=TPB*WBECB;
	double wbep2=WBEP.get_loc(1,0);
	double wbep3=WBEP.get_loc(2,0);
	//yaw channel
	double wlrd_new=wlr1-wbep3;
	wlr=integrate(wlrd_new,wlrd,wlr,int_step);
	wlrd=wlrd_new;
	psipb=wlr;
	psipbd=wlrd;
	//pitch channel
	double wlqd_new=wlq1-wbep2;
	wlq=integrate(wlqd_new,wlqd,wlq,int_step);
	wlqd=wlqd_new;
    thtpb=wlq;
    thtpbd=wlqd;
	//calculating the TPB matrix
	TPB=mat2tr(psipb,thtpb);

	//caculating gimbal dynamics and THB for the next integration cycle
	sensor_ir_uthpb(ththbc,phihbc,psipb,thtpb);
    ththb=ththbc+biast+randt;
    phihb=phihbc+biasp+randp;
	THB=sensor_ir_thb(ththb,phihb);

	//flagging break-lock and blind range conditions 
	if(mseek==4){
		ibreak=0;
		phihbd=-thtpbd*sin(psipb);
		double eh=sqrt(ehy*ehy+ehz*ehz);
		if(fabs(ththb)>trtht){
		   trcond=6;
		   ibreak=1;
		}
		else if(fabs(thtpbd)>trthtd){
		   trcond=7;
		   ibreak=1;
		}
		else if(fabs(phihbd)>trphid){
		   trcond=8;
		   ibreak=1;
		}
		else if(eh>trate){
		   trcond=9;
		   ibreak=1;
		}
		if(ibreak==1){
		  //reaquisition
		  mseek=2;
		  mguide=50; 
		}
		if(dbtk<dblind) mseek=5;
    }
	
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[257].gets(wlq1d);
	missile[258].gets(wlq1);
	missile[259].gets(wlqd);
	missile[260].gets(wlq);
	missile[261].gets(wlr1d);
	missile[262].gets(wlr1);
	missile[263].gets(wlrd);
	missile[264].gets(wlr);
	missile[265].gets(wlq2d);
	missile[266].gets(wlq2);
	missile[267].gets(wlr2d);
	missile[268].gets(wlr2);
	//output to other modules
	missile[180].gets(trcond);
	//diagnostics
//	missile[234].gets(dba);
	missile[276].gets(epy);
	missile[278].gets(epz);
	missile[281].gets(ththb);
	missile[282].gets(phihb);
	missile[283].gets_mat(TPB);
	missile[286].gets_vec(EAHH);
	missile[287].gets_vec(EPHH);
	missile[288].gets_vec(EAPH);
}
///////////////////////////////////////////////////////////////////////////////
//Aimpoint selection and corruption function
//Member function of class 'Missile'
// (1) Introduces aimpoint tracking errors
// (2) Introduces hot spot jitter and bias errors
// Both are initiated at distance 'daim' from the target
//
// Return output:
//          SATH(3)=Aimpoint error in head axes (focal plane array)
//
// Argument input:
//          THL(3,3)=Tran Matrix of head wrt local level axes
//          dbtk=Distance of vehicle wrt target - m
//
//020102 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Missile::sensor_ir_aimp(Matrix THL,Matrix TTL,double dbtk)
{
	//local variables
	Matrix SATH(3,1);
	Matrix THT(3,3);
	
	//localizing module-variables
	//input data
	double daim=missile[272].real();
	Matrix BIASAI=missile[273].vec();
	Matrix BIASSC=missile[274].vec();
	Matrix RANDSC=missile[275].vec();
	//from other modules
	//-------------------------------------------------------------------------
	THT=THL*TTL.trans();
	if(dbtk<daim)
		//aimpoint update
		SATH=THT*BIASAI;
	else
		//hot spot mode
		SATH=THT*(BIASSC+RANDSC);
	return SATH;
	//-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
//Angle conversion function
//Member function of class 'Missile'
// Converts pointing angles (computer) to head angles (gimbals)
//
// Argument Output:
//          ththb=Gimbal head pitch angle - rad
//          phihb=Gimbal roll angle - rad
//
// Argument Input:
//          psipb=Yaw computer pointing angle - rad
//          thtpb=Pitch computer pointing angle - rad
//
//020102 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor_ir_uthpb(double &ththb,double &phihb, double psipb,double thtpb)
{
	//local variables
	double sinpsi,tantht;
	//-------------------------------------------------------------------------
      ththb=acos(cos(thtpb)*cos(psipb));
      sinpsi=sin(psipb);
      tantht=tan(thtpb);
      if(fabs(sinpsi)&&fabs(tantht)<SMALL)
        phihb=0.;
      else
        phihb=atan2(sinpsi,tantht);
	//-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
//THB tranformation matrix function
//Member function of class 'Missile'
// Calculates T.M. of head axes wrt body axes
// Argument Output
//          THB=Transformation matrix of head angles wrt missile body axes
// Argument Input:
//          ththb=Gimbal head pitch angle - rad
//          phihb=Gimbal roll angle - rad
//
//020102 Converted from SRAAM6 by Peter Zipfel
/////////////////////////////////////////////////////////////////////////////////

Matrix Missile::sensor_ir_thb(double tht,double phi)
{
	//local variables
	Matrix THB(3,3);
	//-------------------------------------------------------------------------
	THB.assign_loc(0,0,cos(tht));
    THB.assign_loc(2,0,sin(tht));
    THB.assign_loc(1,1,cos(phi));
    THB.assign_loc(1,2,sin(phi));

    THB.assign_loc(0,1,THB.get_loc(2,0)*THB.get_loc(1,2));
    THB.assign_loc(0,2,(-THB.get_loc(2,0))*THB.get_loc(1,1));
    THB.assign_loc(2,1,(-THB.get_loc(0,0))*THB.get_loc(1,2));
    THB.assign_loc(2,2,THB.get_loc(0,0)*THB.get_loc(1,1));
    THB.assign_loc(1,0,0.);

	return THB;
	//-------------------------------------------------------------------------
}
///////////////////////////////////////////////////////////////////////////////
//Kinematic sensor
//Member function of class 'Missile'
// (1)  Calculates error free LOS rates and angles
// (2)  Also used to initialize the dynamic sensor subroutine
//
// Argument Output:
//                 thtpb=Pitch pointing angle - rad
//                 psipb=Yaw pointing angle - rad
//                 sigdy=Pitch sight line spin - rad/s
//                 sigdz=Yaw sight line spin - rad/s
//				   lamdrb=Yaw LOS rate in body axes - rad/s
//				   lamdqb=Pitch LOS rate in body axes - rad/s
//				   ddab=Closing speed - m/s
// Argument Input:
//                 SBTL(3)=Position of missile wrt target - m
//				   VTEL(3)=Target velocity vector - m/s
//                 dbtk=Distance between missile and target - m
//
//011221 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor_kin(double &thtpb,double &psipb,double &sigdy,double &sigdz,double &lamdrb,double &lamdqb,double &ddab,
					   Matrix SBTL,Matrix VTEL,double dbtk)
{
	//localizing module-variables
	//from other modules
	Matrix TBL=flat6[120].mat();
	Matrix VBEL=flat6[233].vec();
	//-------------------------------------------------------------------------
	//LOS kinematics
    Matrix STBL=SBTL*(-1);
    Matrix STBB=TBL*STBL;
    Matrix UTBL=STBL/dbtk;

	//relative velocity
	Matrix VTBL=VTEL-VBEL;

	//closing velocity
	ddab=-fabs(UTBL^VTBL);

	//LOS rate output in body coordinates
	Matrix WOEB=TBL*UTBL.skew_sym()*VTBL/dbtk;
	lamdqb=WOEB.get_loc(1,0);
	lamdrb=WOEB.get_loc(2,0);

	//building pointing wrt body T.M.
	Matrix POLAR=STBB.pol_from_cart();
	psipb=POLAR.get_loc(1,0);
	thtpb=POLAR.get_loc(2,0);
	Matrix TPB=mat2tr(psipb,thtpb);
	//for 'guidance_term_pronav()' 
	double psisb=psipb;
	double thtsb=thtpb;

	//LOS rate output in pointing coordinates
	Matrix WOEP=TPB*WOEB;
	sigdy=WOEP.get_loc(1,0);
	sigdz=WOEP.get_loc(2,0);
	//-------------------------------------------------------------------------
	//output to other modules
	missile[864].gets(psisb);
	missile[866].gets(thtsb);
}
