///////////////////////////////////////////////////////////////////////////////
//FILE: 'seeker.cpp'
//
//Contains 'seeker' module of class 'Hyper'
//
//040518 Created  by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'seeker' module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[200-299]
// 
//Defining and initializing module-variables
// includes also satellite variables downloaded from 'combus'
//  and placed into reserved location hyper[0-9] (used here and in 'guidance()')
// 
//   mseek = 0 turned off
//		   = 2 enabled (input, or set internally if break-lock occurred)
//		   = 3 acquisition mode (set internally, when hyper is within 'racq')
//		   = 4 lock-on (set internally, when 'dtimac' has elapsed)
//		   = 5 within blind range (set internally); output held const
//
// skr_dyn = 0 kinematic seeker with ideal filter
//         = 1 dynamic seeker with dynamic filter
//
//040518 Created  by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_seeker()
{
	//Definition and initialization of module-variables
	hyper[2].init("STII",0,0,0,"Position of satellite#, downloaded from 'combus' - m","combus","","");
	hyper[3].init("VTII",0,0,0,"Velocity of satellite#, downloaded from 'combus' - m","combus","","");
	hyper[5].init("tgt_com_slot","int",0,"'This' target slot in combus - ND","combus","out","");
    hyper[200].init("mseek","int",0," =2:Enable, =3:Acq, =4:Lock","seeker","data/diag","com");
    hyper[201].init("skr_dyn","int",0,"=0: Kinemtic, =1:Dynamic","seeker","data","");
    hyper[202].init("isets1","int",0,"Seeker flag","seeker","init","");
    hyper[203].init("epchac",0,"Epoch of start of seeker acquisition - s","seeker","init","");
    hyper[204].init("ibreak","int",0,"Flag for seeker break-lock ND","seeker","init","");
	//RF seeker
    hyper[205].init("temp_resx",290,"Temperature of resistor - K","seeker","data","");
    hyper[206].init("dblind",0,"Blind range - m","seeker","data","");
    hyper[207].init("biasaz",0,"Azimuth boresight error - rad","seeker","data","");
    hyper[208].init("biasel",0,"Elevation boresight error - rad","seeker","data","");
    hyper[209].init("freqghz",0,"Seeker operating freequency - GHz","seeker","data","");
    hyper[210].init("rngegw",0,"Range gate width - m","seeker","data","");
    hyper[211].init("thta_3db",0,"Nominal beam width - deg","seeker","data","");
    hyper[212].init("powrs",0,"Seeker average power - W","seeker","data","");    
	hyper[213].init("gainsdb",0,"Transmit gain - dB","seeker","data","");
	hyper[214].init("gainmdb",0,"Receive gain - dB","seeker","data","");
	hyper[215].init("tgt_rcs",0,"Satellite radar cross section - m^2","seeker","data","");
	hyper[216].init("rlatmodb",0,"Atmospheric loss - dB","seeker","data","");
	hyper[217].init("rltotldb",0,"Total system loss - dB","seeker","data","");
	hyper[218].init("dwltm",0,"Dwell time - s","seeker","data","");
	hyper[219].init("rnoisfgd",0,"Noise figure - dB","seeker","data","");
	hyper[220].init("plc5",0,"Coeff.of poly.curve fit of power loss","seeker","data","");
	hyper[221].init("plc4",0,"Coeff.of poly.curve fit of power loss","seeker","data","");
	hyper[222].init("plc3",0,"Coeff.of poly.curve fit of power loss","seeker","data","");
	hyper[223].init("plc2",0,"Coeff.of poly.curve fit of power loss","seeker","data","");    
	hyper[224].init("plc1",0,"Coeff.of poly.curve fit of power loss","seeker","data","");
	hyper[225].init("plc0",0,"Coeff.of poly.curve fit of power loss","seeker","data","");
    hyper[226].init("biasgl1",0,"Glint Gaussian bias in satellite x-dir - m","seeker","data","");
    hyper[227].init("biasgl2",0,"Glint Gaussian bias in satellite y-dir - m","seeker","data","");
    hyper[228].init("biasgl3",0,"Glint Gaussian bias in satellite z-dir - m","seeker","data","");
    hyper[229].init("randgl1",0,"Glint Markov noise in satellite x-dir - m","seeker","data","");
    hyper[230].init("randgl2",0,"Glint Markov noise in satellite y-dir - m","seeker","data","");
    hyper[231].init("randgl3",0,"Glint Markov noise in satellite z-dir - m","seeker","data","");
    hyper[232].init("fovlimx",0,"Field of view limit - deg","seeker","data","");
    hyper[233].init("racq",0,"Seeker acquisition range - m","seeker","data","");
    hyper[234].init("dtimac",0,"Seeker and filter  acquisition time - s","seeker","data","");
    hyper[235].init("esfta",0,"Azimuth scale factor error - ND","seeker","data","");
    hyper[236].init("esfte",0,"Elevation scale factor error - ND","seeker","data","");
    hyper[237].init("dbtk",0,"Seeker hyper-satellite distance - m","seeker","diag","");
    hyper[238].init("timeac",0,"Saving acqisition time - s","seeker","save","");
    hyper[239].init("azabx",0,"Actual azimuth of antenna wrt body axes - deg","seeker","out","plot");
    hyper[240].init("elabx",0,"Actual elevation of antenna wrt body axes - deg","seeker","out","plot");
    hyper[241].init("dab",0,"Actual LOS distance - m","seeker","out","scrn,plot");
    hyper[242].init("ddab",0,"Actual LOS range-rate - m/s","seeker","out","scrn,plot");
    hyper[243].init("pwr_loss_db",0,"Power loss due to look angle - dB","seeker","diag","");
    hyper[244].init("snr_db",0,"Signal to noise ratio - dB","seeker","diag","");
    hyper[245].init("onax",0,"Off-the-nose angle - deg","seeker","diag","");
    hyper[246].init("epazt",0,"Total azimuth measurement error - rad","seeker","diag","");
    hyper[247].init("epelt",0,"Total elevation measurement error - rad","seeker","diag","");
    hyper[248].init("epaz_glnt",0,"Glint azimuth error - rad","seeker","diag","");
    hyper[249].init("epel_glnt",0,"Glint elevation error - rad","seeker","diag","");
	//Seeker filter
	hyper[250].init("init_filter","int",1,"Initializing matrices only once","seeker","init","");
	hyper[251].init("epchup",0,"Update epoch, =0: forcing first update","seeker","init","");
	hyper[252].init("ppos_skr",0,"Init 1sig pos values of cov matrix - m","seeker","data","");
	hyper[253].init("pvel_skr",0,"Init 1sig vel values of cov matrix - m/s","seeker","data","");
	hyper[254].init("psfct",0,"Init 1sig scale fct error of cov matrix - ND","seeker","data","");
	hyper[255].init("qpos_skr",0,"1sig pos values of process cov matrix - m","seeker","data","");
	hyper[256].init("qvel_skr",0,"1sig vel values of process cov matrix - m/s","seeker","data","");
	hyper[257].init("qsfct",0,"1sig scale fct error of process cov matrix - ND","seeker","data","");
	hyper[258].init("razab",0,"1sig azimuth value of meas spectral density - rad","seeker","data","");
	hyper[259].init("relab",0,"1sig elevation value of meas spectral density - rad","seeker","data","");
	hyper[260].init("rdab",0,"1sig range value of meas spectral density - m","seeker","data","");
	hyper[261].init("rddab",0,"1sig range-rate value of meas spectrall density - m/s","seeker","data","");
	hyper[262].init("dtimkf",0,"Kalman Filter update interval - s","seeker","data","");
	hyper[263].init("factp_skr",0,"Factor to modifiy initial P-matrix P(1+factp_skr)","seeker","data","");
	hyper[264].init("factq_skr",0,"Factor to modifiy the Q-matrix Q(1+factq_skr)","seeker","data","");
	hyper[265].init("factr_skr",0,"Factor to modifiy the R-matrix R(1+factr_skr)","seeker","data","");
	hyper[266].init("flag_out","int",1,"Flag limiting warnings to one output","seeker","init","");
	hyper[267].init("mupdt","int",0,"Update flag for Kalman filter - NA","seeker","diag","");
	hyper[268].init("esfcta",0,"Azimuth scale factor error - ND","seeker","data","");
	hyper[269].init("esfcte",0,"Elevation scale factor error - ND","seeker","data","");
	hyper[270].init("STBIK",0,0,0,"Estimated satellite wrt hyper position - m","seeker","out","");
	hyper[271].init("VTBIK",0,0,0,"Estimated satellite wrt hyper velocity - m/s","seeker","out","");
	hyper[272].init("SIGPOS",0,0,0,"Std dev of position error - m","seeker","diag","");
	hyper[273].init("SIGVEL",0,0,0,"Std dev of velocity error - m/s","seeker","diag","");
	hyper[274].init("semi_major",0,"Error ellipse major semi-axis - m","seeker","diag","plot");
	hyper[275].init("semi_minor",0,"Error ellipse minor semi-axis - m","seeker","diag","plot");
	hyper[276].init("ellipse_anglx",0,"Rotation angle of major axis - deg","seeker","diag","");
	hyper[277].init("ESTBI",0,0,0,"State position residual - m","seeker","diag","");
	hyper[278].init("EVTBI",0,0,0,"State velocity residual - m/s","seeker","diag","");
	hyper[279].init("eaz",0,"Measurement azimuth error - rad","seeker","diag","plot");
	hyper[280].init("eel",0,"Measurement elevation error - rad","seeker","diag","plot");
	hyper[281].init("edab",0,"Measurement range error - m","seeker","diag","plot");
	hyper[282].init("eddab",0,"Measurement range-rate error - m/s","seeker","diag","plot");
	hyper[283].init("SXH_SKR",0,0,0,"Target wrt vehicle position in inertial coord - m","seeker","save","");
	hyper[284].init("VXH_SKR",0,0,0,"Target wrt vehicle velocity in inertial coord - m/s","seeker","save","");
	hyper[285].init("SFH",0,0,0,"Scale factor state - ND","seeker","save","");
	hyper[286].init("PMAT1",0,0,0,0,0,0,0,0,0,"Covariance Matrix 1st row - mixed","seeker","save","");
	hyper[287].init("PMAT2",0,0,0,0,0,0,0,0,0,"Covariance Matrix 2nd row - mixed","seeker","save","");
	hyper[288].init("PMAT3",0,0,0,0,0,0,0,0,0,"Covariance Matrix 3rd row - mixed","seeker","save","");
	hyper[289].init("PMAT4",0,0,0,0,0,0,0,0,0,"Covariance Matrix 4th row - mixed","seeker","save","");
	hyper[290].init("PMAT5",0,0,0,0,0,0,0,0,0,"Covariance Matrix 5th row - mixed","seeker","save","");
	hyper[291].init("PMAT6",0,0,0,0,0,0,0,0,0,"Covariance Matrix 6th row - mixed","seeker","save","");
	hyper[292].init("PMAT7",0,0,0,0,0,0,0,0,0,"Covariance Matrix 7th row - mixed","seeker","save","");
	hyper[293].init("PMAT8",0,0,0,0,0,0,0,0,0,"Covariance Matrix 8th row - mixed","seeker","save","");
	hyper[294].init("dtim",0,"Timer for Kalman Filter update - s","seeker","save","");
	hyper[296].init("epaz_saved",0,"Azimuth  Markov noise - rad","seeker","save","");
	hyper[297].init("epel_saved",0,"Elevation  Markov noise - rad","seeker","save","");
	hyper[298].init("range_saved",0,"Range  Markov noise - m","seeker","save","");
	hyper[299].init("rate_saved",0,"Range-rate  Markov noise - m/s","seeker","save","");
}	

///////////////////////////////////////////////////////////////////////////////
//Seeker module
//Member function of class 'Hyper'
// 
//   mseek = 0 turned off
//		   = 2 enabled (input, or set internally if break-lock occured)
//		   = 3 acquisition mode (set internally, when hyper is within 'racq')
//		   = 4 lock-on (set internally, when 'dtimac' has elapsed)
//		   = 5 within blind range (set internally); output held const
//
// skr_dyn = 0 kinematic seeker with ideal filter
//         = 1 dynamic seeker with dynamic filter
//
// Filter:
//     Filter converts seeker measurements into inertial position and velocity
//     8 State extended Kalman filter for dynamic seeker.
//	   States: 3 positions, 3 velocities, 2 scale factors.
//	   Pos and vel are in local inertial coordinates
//	   Measurements: az & el angles, range, range-rate.
//	   Filter initialization during mseek=3
//	   Filter update during mseek=4 in 'dtimkf' time intervals.
//	   Filter update stops when seeker goes blind (mseek=5)
//
// Notes:
// (1) Value of 'racq' determines when Seeker is locked on target 
//
// (2) The satellite states are subscribed from 'combus'. They must be located in
//	   the 'combus Packet' at the following positions: STII @ 1 and VTII @ 2. 
//	   They are loaded into 'hyper[2]' and 'hyper[3]' for further use in 'this' Hyper vehicle 
//
//040518 Created  by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)
{
	//local variables
	Variable *data_t;
	double azab(0),elab(0),dab(0),ddab(0);
	double sigdy(0),sigdz(0);
	double ehz(0),ehy(0);
	Matrix STBBK(3,1);

	//local module-variables
	double azabx(0),elabx(0);
	double dbt(0),dbtk(0);
	Matrix STII(3,1);
	Matrix VTII(3,1);
	Matrix SBTI(3,1);
	int tgt_com_slot(0);
	double psiot1(0),thtot1(0);

	//localizing module-variables
	//input data
	int sat_num=hyper[1].integer();
	int mseek=hyper[200].integer();
	int skr_dyn=hyper[201].integer();
	int isets1=hyper[202].integer();
	double fovlimx=hyper[232].real();
	double racq=hyper[233].real();
	double dtimac=hyper[234].real();
	Matrix STBIK=hyper[270].vec();	
	Matrix VTBIK=hyper[271].vec(); 
	//getting saved value
	double epchac=hyper[203].real();
	double timeac=hyper[238].real();
	//input from other modules
	double time=round6[0].real();
	Matrix TBI=round6[121].mat();
	Matrix SBII=round6[235].vec();
	Matrix VBII=round6[236].vec();
	int mguide=hyper[400].integer();
	//-------------------------------------------------------------------------
	//downloading from 'combus' satellite variables
	//building satellite id = t(j+1)
	char number[4];	
	sprintf(number,"%i",sat_num);
	string target_id="t"+string(number);
	//finding slot 'i' of satellite in 'combus' (same as in vehicle_list)
	for(int i=0;i<num_vehicles;i++)
	{
		string id=combus[i].get_id();
		if (id==target_id)
		{						
			//downloading data from satellite packet
			tgt_com_slot=i;
			data_t=combus[i].get_data();
			STII=data_t[1].vec();
			VTII=data_t[2].vec();
		}
		//true distance to satellite
		SBTI=SBII-STII;
	}
	//seeker distance to satellite
	SBTI=SBII-STII;
	dbtk=SBTI.absolute();

	//seeker is enabled
	if(mseek==2){
		//within acquisition range
		isets1=1;
		if(dbtk<racq)mseek=3;
	}
	//seeker in acquisition mode
	if(mseek==3){
		//initialization
		if(isets1==1){
			isets1=0;
			epchac=time;
		}
		//acquisition(for dynamic seeker, satellite must be in field-of-view)
		if(skr_dyn==1)
		{
			seeker_rf(azab,elab,dab,ddab,mseek,mguide, fovlimx,SBTI,int_step);
			seeker_filter(STBIK,VTBIK ,azab,elab,dab,ddab,mseek,int_step);

			if((fabs(azab)<=fovlimx*RAD)&&(fabs(elab)<=fovlimx*RAD)){
			   timeac=time-epchac;
			   if(timeac>dtimac)
				   mseek=4;
			}
		}
		else{
			seeker_kin(azab,elab,dab,ddab, SBTI);			
			//ideal filter output  for kinematic seeker
			STBBK.cart_from_pol(dab,azab,elab);
			STBIK=TBI.trans()*STBBK;
			VTBIK=VTII-VBII;

			timeac=time-epchac;
			if(timeac>dtimac)
				mseek=4;
		}
	}
	//seeker lock-on
	if(mseek==4){
		if(skr_dyn==1){
			seeker_rf(azab,elab,dab,ddab,mseek,mguide, fovlimx,SBTI,int_step);			
			seeker_filter(STBIK,VTBIK ,azab,elab,dab,ddab,mseek,int_step);						  
		}
		else{
			seeker_kin(azab,elab,dab,ddab, SBTI);
			//ideal filter output  for kinematic seeker
			STBBK.cart_from_pol(dab,azab,elab);
			STBIK=TBI.trans()*STBBK;
			VTBIK=VTII-VBII;			
		}
		//diagnostics
		azabx=azab*DEG;
		elabx=elab*DEG;
	}
	//diagnostics
	azabx=azab*DEG;
	elabx=elab*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving value for next cycle
	hyper[203].gets(epchac);
	hyper[238].gets(timeac);
	//output to other modules
	hyper[2].gets_vec(STII); 
	hyper[3].gets_vec(VTII);
	hyper[5].gets(tgt_com_slot);
	hyper[400].gets(mguide);
	hyper[270].gets_vec(STBIK);	
	hyper[271].gets_vec(VTBIK); 
	//diagnostics
	hyper[200].gets(mseek);
	hyper[202].gets(isets1);
    hyper[237].gets(dbtk);
    hyper[239].gets(azabx);
    hyper[240].gets(elabx);
    hyper[241].gets(dab);
    hyper[242].gets(ddab);
//    hyper[277].gets(dbt);
	hyper[295].gets_vec(SBTI);

}
///////////////////////////////////////////////////////////////////////////////
//RF seeker function
//Member function of class 'Hyper'
// (1) Corrupts the true satellite centroid by glint.
// (2) Introduces boresight error into the antenna azimuth and elevation measurements.
// (3) Introduces thermal noise into range and range-rate measurements.
// (4) Resets to midcourse if break lock occurs.
// (5) Holds last measurement after blind range is reached.
//
// Parameter output:
//                 azab = Azimuth angle measurement - rad
//                 elab = elevation angle measurement - rad
//                 dab = range to satellite centroid measurement - m
//                 ddab = range rate to satellite measurement - m/s 
//                 mseek: if break lock occured, reset to 2 (acquisition)
//                 mguide: if break lock occured, reset to 6 (datalink based guidance)
// Parameter input:
//                 fovlimx = field-of-view limit - deg
//                 SBTI(3) = position of hyper wrt satellite - m
//				   int_step = integration step size - s 
//
//040518 Created  by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::seeker_rf(double &azab,double &elab,double &dab,double &ddab,int &mseek,
					   int &mguide, double fovlimx,Matrix SBTI,double int_step)
{
	//local variables
	Matrix U1B(3,1);

	//local module-variables
	double azobx(0),elobx(0);
	double pwr_loss_db(0);
	double snr_db(0);
	double onax(0);
	double epazt(0),epelt(0);

	//localizing module-variables
	//input data
	double temp_resx=hyper[205].real();
	double dblind=hyper[206].real();
	double biasaz=hyper[207].real(); 
	double biasel=hyper[208].real();
	double freqghz=hyper[209].real();
	double rngegw=hyper[210].real();
	double thta_3db=hyper[211].real();
	double powrs=hyper[212].real();
	double gainsdb=hyper[213].real();
	double gainmdb=hyper[214].real();
	double tgt_rcs=hyper[215].real();
	double rlatmodb=hyper[216].real();
	double rltotldb=hyper[217].real();
	double dwltm=hyper[218].real();
	double rnoisfgd=hyper[219].real();
	double plc5=hyper[220].real();
	double plc4=hyper[221].real();
	double plc3=hyper[222].real();
	double plc2=hyper[223].real();
	double plc1=hyper[224].real();
	double plc0=hyper[225].real();
	double esfta=hyper[235].real();
	double esfte=hyper[236].real();
	//getting saved values
	double epaz_saved=hyper[296].real();
	double epel_saved=hyper[297].real();
	double range_saved=hyper[298].real();
	double rate_saved=hyper[299].real();
	//from other modules
	double time=round6[0].real();
	Matrix TBI=round6[121].mat();
	Matrix VBII=round6[236].vec();
	Matrix VTII=hyper[3].vec();
	double trcode=hyper[180].real();
	//-------------------------------------------------------------------------	
	//corrupting LOS by glint
	Matrix SOTI=seeker_glint();
	Matrix SOBIC=SOTI-SBTI;
	Matrix SOBBC=TBI*SOBIC;
	
	//corrupted LOS wrt body axes 
	Matrix POLAR=SOBBC.pol_from_cart();
	double dobc=POLAR.get_loc(0,0);
	double azobc=POLAR.get_loc(1,0);
	double elobc=POLAR.get_loc(2,0);
	//forming unit LOS vector
	Matrix UOBIC=SOBIC*(1/dobc);
	
	//true LOS wrt body axes
    Matrix STBI=SBTI*(-1.);
    Matrix STBB=TBI*STBI;
	POLAR=STBB.pol_from_cart();
	double dob=POLAR.get_loc(0,0);
	double azob=POLAR.get_loc(1,0);
	double elob=POLAR.get_loc(2,0);
	//forming LOS unit vector
	Matrix UOBI=STBI*(1/dob);

	//computing off-the-nose LOS angle 'ona'
	U1B.build_vec3(1,0,0);
	Matrix UOBBC=TBI*UOBIC;
	double ona=angle(U1B,UOBBC);
	onax=ona*DEG;
	
	//signal to noise ratio
	//converting data from db to rational numbers
	double k_noise=PI/2.;
    double wvelngth=(2.998e8)/(freqghz*10.e8);
    double gains=pow(10.,(gainsdb/10.));
    double gainm= pow(10.,(gainmdb/10.));
    double rnoisfg=pow(10.,(rnoisfgd/10.));
    double rlatmo=pow(10.,(rlatmodb/10.));
    double rltotl=pow(10.,(rltotldb/10.));
	//getting off-the-nose transmitting power loss from curve
    double pwr_loss=-(plc5*10.e-11)*pow(onax,5)+(plc4*10.e-9)*pow(onax,4)
             -(plc3*10.e-7)*pow(onax,3)+(plc2*10.e-5)*pow(onax,2)
             -(plc1*10.e-3)*onax+plc0;
	if(pwr_loss>1.)pwr_loss=1.;
	//transmit power
	double powrt=powrs-pwr_loss;
	//signal power
    double ps=powrt*gains*rlatmo*rlatmo*tgt_rcs*gainm*wvelngth*wvelngth
             /(pow(4.*PI,3)*pow(dobc,4)*rltotl);
    double pn=KBOLTZ*temp_resx*rnoisfg*(1./dwltm);
	double snr=ps/pn;

	//closing velocity
	Matrix VTBI=VTII-VBII;

	//Range rate
	//corrupted by glint
	double rdobc=UOBIC^VTBI;
	//without glint
	double rdob=UOBI^VTBI;

	//monopulse angular measurement of antenna with glint, scale_factor,bias and noise errors
	//glint error
	double epaz_glnt=azobc-azob;
	double epel_glnt=elobc-elob;
	//thermal noise
	double sigma_mp=sqrt(dwltm/int_step)*(thta_3db*RAD)/(k_noise*sqrt(snr)); 
	double epaz_mp=markov(sigma_mp,100.,time,int_step,epaz_saved); //bcor = 100 Hz thermal noise bandwidth
	double epel_mp=markov(sigma_mp,100.,time,int_step,epel_saved); //bcor = 100 Hz thermal noise bandwidth
	//total monopulse errors
	epazt=epaz_glnt+biasaz+epaz_mp;
	epelt=epel_glnt+biasel+epel_mp;
	//monopulse measurements(true + error)
	azab=(1.+esfta)*azob+epazt;
	elab=(1.+esfte)*elob+epelt;

	//range  measurements with thermal noise
	//range thermal noise sigma
	double sigma_range=sqrt(dwltm/int_step)*rngegw/(k_noise*sqrt(snr));
	//range thermal noise
	double eps_range=markov(sigma_range,10.,time,int_step,range_saved); //bcor = 10 Hz thermal noise bandwidth
	//range measurement
	dab=dobc+eps_range;
	
	//range-rate measurements with thermal noise
	//velocity gate width
	double vgw = 200.*wvelngth/2.0;
	//range-rate thermal sigma
	double sig_range_rate = sqrt(dwltm/int_step)*vgw/(k_noise*sqrt(snr));
	double eps_range_rate=markov(sig_range_rate,10.,time,int_step,rate_saved); //bcor = 10 Hz thermal noise bandwidth
	//range-rate measurement
	ddab=rdobc+eps_range_rate;

	//flagging break-lock and blind range conditions; back to acquistion if outside FOW 
	if(mseek==4){
		if(fabs(azab)>fovlimx*RAD||fabs(elab)>fovlimx*RAD){
			trcode=6.;
			mseek=2;
			mguide=6;
		}
		if(dab<dblind)
			mseek=5;
    }

	//diagnostics
	pwr_loss_db=10.*log10(pwr_loss);
	snr_db=10.*log10(snr);
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	hyper[296].gets(epaz_saved);
	hyper[297].gets(epel_saved);
	hyper[298].gets(range_saved);
	hyper[299].gets(rate_saved);
	//output to other modules
	hyper[180].gets(trcode);
	//diagnostics
    hyper[243].gets(pwr_loss_db);
    hyper[244].gets(snr_db);
    hyper[245].gets(onax);
    hyper[246].gets(epazt);
    hyper[247].gets(epelt);
    hyper[248].gets(epaz_glnt);
    hyper[249].gets(epel_glnt);
}
///////////////////////////////////////////////////////////////////////////////
//Kinematic seeker
//Member function of class 'Hyper'
//  Generates ideal measurements of the strapdown seeker
//
// Parameter output:
//                 azob = Azimuth angle measurement - rad
//                 elob = Elevation angle measurement - rad
//                 dab = Range to satellite centroid measurement - m
//                 ddab = Range rate to satellite measurement - m/s
// Parameter input:
//                 SBTI(3) = Position of hyper wrt satellite - m
//
//040518 Created  by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::seeker_kin(double &azob,double &elob,double &dab,double &ddab, Matrix SBTI)
{
	//local variables
	Matrix STBI(3,1);
	Matrix STBB(3,1);
	Matrix UTBI(3,1);
	Matrix VTBI(3,1);
	Matrix POLAR(3,1);
	
	//localizing module-variables
	//from other modules
	Matrix VTII=hyper[3].vec();
	Matrix TBI=round6[121].mat();
	Matrix VBII=round6[236].vec();
	//-------------------------------------------------------------------------
	//forming LOS unit vector
    STBI=SBTI*(-1.);
    STBB=TBI*STBI;
	dab=STBI.absolute();
	double dum1=1./dab;
    UTBI=STBI*dum1;

	//computing relative velocity
	VTBI=VTII-VBII;

	//computing closing velocity
	ddab=UTBI^VTBI;

	//computing antenna centerline azimuth and elevation wrt body axes
	POLAR=STBB.pol_from_cart();
	azob=POLAR.get_loc(1,0);
	elob=POLAR.get_loc(2,0);
	//-------------------------------------------------------------------------
}

///////////////////////////////////////////////////////////////////////////////
//Glint
//Member function of class 'Hyper'
//Simple glint model, corrupting the aimpoint along the three satellite axes
//
// This function performs the following functions:
// (1)  Generates a random bias displacement from true satellite position
// (2)  Generates a gaussian Markov displacement from true satellite position
// Return output:
//                 SOTI(3)=Displacement of apparent satellite centroid O from
//                         true satellite center of mass T in L coord. - m
//
//040518 Created  by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::seeker_glint()
{
	//local variables
	Matrix BIASGL(3,1);
	Matrix RANDGL(3,1);
	
	//localizing module-variables
	//input data
//	Matrix TTI=hyper[4].mat();
	Matrix TTI(3,3);TTI.identity(); //shortcut, eventually comes from 'combus'
	double biasgl1=hyper[226].real();
	double biasgl2=hyper[227].real();
	double biasgl3=hyper[228].real();
	double randgl1=hyper[229].real();
	double randgl2=hyper[230].real();
	double randgl3=hyper[231].real();
	//-------------------------------------------------------------------------
	//calculating displacement of aimpoint by glint
	BIASGL.build_vec3(biasgl1,biasgl2,biasgl3);
	RANDGL.build_vec3(randgl1,randgl2,randgl3);
	Matrix SOTT=RANDGL+BIASGL;
	Matrix TIT=TTI.trans();
	Matrix SOTI=TIT*SOTT;
	//-------------------------------------------------------------------------
	return SOTI;
}
///////////////////////////////////////////////////////////////////////////////
//Seeker filter converts seeker measurements into inertial position and velocity
// 8 State extended Kalman filter for dynamic seeker.
// States: 3 positions, 3 velocities, 2 scale factors.
// Pos and vel are in inertial coordinates
// Measurements: az & el angles, range, range-rate.
// Filter initialization during mseek=3.
// Filter update during mseek=4 in 'dtimkf' time intervals.
// Filter update stops when seeker goes blind (mseek=5)
//
// Parameter output
//			STBIK(3) = target relative vehicle position from filtered seeker measurements - m 
//			VTBIK(3) = target relative vehicle velocity from filtered seeker measurements - m/s 
//
// Parameter input
//			azab = measured azimuth of antenna wrt body axes - rad
//			elab = measured elevation of antenna wrt body axes - rad
//			dab = measured LOS distance - m 
//			ddab = measured LOS range-rate - m/s
//			mseek = seeker mode switch
//			in_step = integration step - sec
//
// Debugging provision
//	Start of debugging can be set by initializing local variables 'debug_time_extrap(9999)' and 
//   'debug_time_update(9999)' with smaller values
// 
//040518 Created  by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Hyper::seeker_filter(Matrix &STBIK,Matrix &VTBIK
			,double azab,double elab,double dab,double ddab,int mseek,double int_step)
{
	//local variables
	double debug_time_extrap(9999); //set time for debugging purposes, otherwise large value
	double debug_time_update(9999); //set time for debugging purposes, otherwise large value
	int i(0);
	int j(0);
	int m(0);
	int n(0);
	Matrix STBBK(3,1);
	Matrix XH(8,1); //recursive, must be saved, separate into SXH_SKR(3), VXH_SKR(3), SFH(2),
	Matrix PMAT(8,8);  //recursive, must be saved, separated into 8 PMATx(3x3)
	static Matrix QQ(8,8); // initialization, constant, same for all objects, -> static ok
	static Matrix RR(4,4);// initialization, constant, same for all objects, -> static ok
	static Matrix FF(8,8);// initialization, constant, same for all objects, -> static ok
	static Matrix PHI(8,8);// initialization, constant, same for all objects, -> static ok
	static Matrix GAMDT(8,3);// initialization, constant, same for all objects, -> static ok
	Matrix XXT(8,1);
	Matrix XX(8,1);
	Matrix SHI(3,1);
	Matrix VHI(3,1);
	Matrix SHB(3,1);
	Matrix HH(4,8);
	Matrix ZH(4,1);
	Matrix ZK(4,1);
	Matrix UNIT(8,8);UNIT.identity();
	Matrix COVPL(3,3);
	Matrix COVPA(3,3);
	Matrix COV23(2,2);
	Matrix STBI(3,1);
	Matrix VTBI(3,1);

	//localizing module-variables
	//initialized and from previous cylce
	int init_filter=hyper[250].integer();
	double epchup=hyper[251].real();
	int flag_out=hyper[266].integer();
	int mupdt=hyper[267].integer();
	Matrix SIGPOS=hyper[272].vec();	
	Matrix SIGVEL=hyper[273].vec(); 
	double semi_major=hyper[274].real();
	double semi_minor=hyper[275].real();
	double ellipse_anglx=hyper[276].real();
	Matrix ESTBI=hyper[277].vec();
	Matrix EVTBI=hyper[278].vec();
	double eaz=hyper[279].real();
	double eel=hyper[280].real();
	double edab=hyper[281].real();
	double eddab=hyper[282].real();
	//input data
	double ppos_skr=hyper[252].real();
	double pvel_skr=hyper[253].real();
	double psfct=hyper[254].real();
	double qpos_skr=hyper[255].real();
	double qvel_skr=hyper[256].real();
	double qsfct=hyper[257].real();
	double razab=hyper[258].real();
	double relab=hyper[259].real();
	double rdab=hyper[260].real();
	double rddab=hyper[261].real();
	double dtimkf=hyper[262].real();
	double factp_skr=hyper[263].real();
	double factq_skr=hyper[264].real();
	double factr_skr=hyper[265].real();
	double esfcta=hyper[268].real();
	double esfcte=hyper[269].real();
	//input from other modules
	double time=round6[0].real();
	double grav=round6[11].real();
	Matrix TBI=round6[121].mat();
	Matrix TDI=round6[223].mat();
	Matrix SBII=round6[235].vec();
	Matrix VBII=round6[236].vec();
	Matrix STII=hyper[2].vec();
	Matrix VTII=hyper[3].vec();
	Matrix VBIIC=hyper[303].vec();
	Matrix SBIIC=hyper[304].vec();
	Matrix TBIC=hyper[315].mat();
	Matrix FSPCB=hyper[334].vec();
	//getting saved values
	Matrix SXH_SKR=hyper[283].vec();
	Matrix VXH_SKR=hyper[284].vec();
	Matrix SFH=hyper[285].vec();
	Matrix PMAT1=hyper[286].mat();
	Matrix PMAT2=hyper[287].mat();
	Matrix PMAT3=hyper[288].mat();
	Matrix PMAT4=hyper[289].mat();
	Matrix PMAT5=hyper[290].mat();
	Matrix PMAT6=hyper[291].mat();
	Matrix PMAT7=hyper[292].mat();
	Matrix PMAT8=hyper[293].mat();
	double dtim=hyper[294].real();
	//assembling saved state vector
	for(m=0;m<3;m++){
		XH.assign_loc(m,0,SXH_SKR.get_loc(m,0));
		XH.assign_loc(m+3,0,VXH_SKR.get_loc(m,0));
	}
	XH.assign_loc(6,0,SFH.get_loc(0,0));
	XH.assign_loc(7,0,SFH.get_loc(1,0));
	//assembling covariance matrix from saved 3x3 matrices
	Matrix VEC1=PMAT1.vec9_mat33();
	Matrix VEC2=PMAT2.vec9_mat33();
	Matrix VEC3=PMAT3.vec9_mat33();
	Matrix VEC4=PMAT4.vec9_mat33();
	Matrix VEC5=PMAT5.vec9_mat33();
	Matrix VEC6=PMAT6.vec9_mat33();
	Matrix VEC7=PMAT7.vec9_mat33();
	Matrix VEC8=PMAT8.vec9_mat33();
	for(n=0;n<8;n++){
		//VECx, x=0...7, is the x-th row of PMAT
		PMAT.assign_loc(0,n,VEC1.get_loc(n,0));
		PMAT.assign_loc(1,n,VEC2.get_loc(n,0));
		PMAT.assign_loc(2,n,VEC3.get_loc(n,0));
		PMAT.assign_loc(3,n,VEC4.get_loc(n,0));
		PMAT.assign_loc(4,n,VEC5.get_loc(n,0));
		PMAT.assign_loc(5,n,VEC6.get_loc(n,0));
		PMAT.assign_loc(6,n,VEC7.get_loc(n,0));
		PMAT.assign_loc(7,n,VEC8.get_loc(n,0));
	}
	//-------------------------------------------------------------------------
//	if(time>debug_time_update) cout<<"********************* Entrance **********************\n";
//	if(time>debug_time_update) cout<<"*** PMAT entering function ***\n";
//	if(time>debug_time_update) PMAT.print();
//	if(time>debug_time_update) cout<<"*** XH entering function ***\n";
//	if(time>debug_time_update) XH.print();
	
	//*** initialization
	if(mseek==3&&init_filter){
		init_filter=0;
		//state vector XH = satellite state - INS derived vehicle state (plus scale factor states)
		Matrix SH=STII-SBIIC;
		Matrix VH=VTII-VBIIC;
		XH.zero();
		for(i=0;i<3;i++){
			XH.assign_loc(i,0,SH.get_loc(i,0));
			XH.assign_loc(i+3,0,VH.get_loc(i,0));
		}
		XH.assign_loc(6,0,1);
		XH.assign_loc(7,0,1);
		//covariance matrix
		PMAT.zero();
		for(i=0;i<3;i++){
			PMAT.assign_loc(i,i,pow(ppos_skr*(1.+factp_skr),2));
			PMAT.assign_loc(i+3,i+3,pow(pvel_skr*(1.+factp_skr),2));
		}
		PMAT.assign_loc(6,6,pow(psfct*(1.+factp_skr),2));
		PMAT.assign_loc(7,7,pow(psfct*(1.+factp_skr),2));

		//dynamic error covariance matrix
		QQ.zero();
		for(i=0;i<3;i++){
			QQ.assign_loc(i,i,pow(qpos_skr*(1+factq_skr),2));
			QQ.assign_loc(i+3,i+3,pow(qvel_skr*(1+factq_skr),2));
		}
		QQ.assign_loc(6,6,pow(qsfct*(1+factq_skr),2));
		QQ.assign_loc(7,7,pow(qsfct*(1+factq_skr),2));
		//measurement noise covariance matrix
		RR.zero();
		RR.assign_loc(0,0,pow(razab*(1+factr_skr),2));
		RR.assign_loc(1,1,pow(relab*(1+factr_skr),2));
		RR.assign_loc(2,2,pow(rdab*(1+factr_skr),2));
		RR.assign_loc(3,3,pow(rddab*(1+factr_skr),2));
		//F matrix of dynamic process
		FF.zero();
		for(i=0;i<3;i++){
			FF.assign_loc(i,i+3,1);
		}
		//state transition matrix PHI
		Matrix EYE(8,8);
		PHI=EYE.identity()+FF*int_step;

		//control matrix
		Matrix GG(8,3);
		GG.assign_loc(3,0,1);
		GG.assign_loc(4,1,1);
		GG.assign_loc(5,2,1);
		GAMDT=GG*int_step;

		if(time>debug_time_extrap) cout<<"\n**************** Initialization Time = "<<time<<" ****************\n";
		if(time>debug_time_extrap) cout<<"*** PMAT Covariance matrix ***\n";
		if(time>debug_time_extrap) PMAT.print();
		if(time>debug_time_extrap) cout<<"*** PHI State transition matrix ***\n";
		if(time>debug_time_extrap) PHI.print();
		if(time>debug_time_extrap) cout<<"*** XH State ***\n";
		if(time>debug_time_extrap) XH.print();
	}
	//*** state and covariance matrix extrapolation
	if(mseek==4){

		double dtim=time-epchup;
		if(dtim>dtimkf)mupdt=1;

		//extrapolating the state vector
		Matrix FSPIC(3,1);FSPIC=~TBIC*FSPCB;
		Matrix GRAVD(3,1);GRAVD.build_vec3(0,0,grav);

		if(time>debug_time_extrap) cout<<"**********************************************************\n";
		if(time>debug_time_extrap) cout<<"**************** Extrapolation Time = "<<time<<" ***************\n";
		if(time>debug_time_extrap) cout<<"**********************************************************\n\n";
		if(time>debug_time_extrap) cout<<"********************* XH Extrapolation *******************\n";
		if(time>debug_time_extrap) cout<<"*** XH before Extrapolation\n";
		if(time>debug_time_extrap) XH.print();

		XH=PHI*XH-GAMDT*(FSPIC+~TDI*GRAVD);

		if(time>debug_time_extrap) cout<<"*** XH after Extrapolation \n";
		if(time>debug_time_extrap) XH.print();

		//extrapolating covariance matrix

		if(time>debug_time_extrap) cout<<"********************* P Extrapolation ********************\n";
		if(time>debug_time_extrap) cout<<"*** PMAT before Extrapolation\n";
		if(time>debug_time_extrap) PMAT.print();
		if(time>debug_time_extrap) cout<<"*** QQ\n";
		if(time>debug_time_extrap) QQ.print();

		PMAT=PHI*(PMAT+QQ*(int_step/2))*PHI.trans()+QQ*(int_step/2);      //<- iteration

		if(time>debug_time_extrap) cout<<"*** PMAT after Extrapolation\n";
		if(time>debug_time_extrap) PMAT.print();

		//diagnostic: state residuals=true-estimated state
		STBI=STII-SBII;
		VTBI=VTII-VBII;
		for(i=0;i<3;i++){
			XXT.assign_loc(i,0,STBI.get_loc(i,0));
			XXT.assign_loc(i+3,0,VTBI.get_loc(i,0));
		}
		XXT.assign_loc(6,0,1+esfcta);
		XXT.assign_loc(7,0,1+esfcte);
		XX=XXT-XH;

		//diagnostics: std deviation of position
		for(i=0;i<3;i++){
			if(PMAT.get_loc(i,i)>=0){
				SIGPOS.assign_loc(i,0,sqrt(PMAT.get_loc(i,i)));
			}
			else{
				if(flag_out==1){
					flag_out=0;
					cout<<" *** Warning: Negative position variance in EKF at time = "<<time<<'\n';
				}
			}
		}
		//diagnostics: std deviation of velocity
		for(i=0;i<3;i++){
			if(PMAT.get_loc(i+3,i+3)>=0){
				SIGVEL.assign_loc(i,0,sqrt(PMAT.get_loc(i+3,i+3)));
			}
			else{
				if(flag_out==1){
					flag_out=0;
					cout<<" *** Warning: Negative velocity variance in EKF at time = "<<time<<'\n';
				}
			}
		}
		if(time>debug_time_extrap) cout<<"********************* XX State Residual **********************\n";
		if(time>debug_time_extrap) cout<<"*** XXT True State ***\n";
		if(time>debug_time_extrap) XXT.print();
		if(time>debug_time_extrap) cout<<"*** XH Extrapolated State ***\n";
		if(time>debug_time_extrap) XH.print();
		if(time>debug_time_extrap) cout<<"*** XX=XXT-XH State residual ***\n";
		if(time>debug_time_extrap) XX.print();
	}
	//*** filter update epoch
	if(mseek==4&&mupdt==1){

		if(time>debug_time_update) cout<<"**********************************************************\n";
		if(time>debug_time_update) cout<<"******************** Update Time = "<<time<<"******************\n";
		if(time>debug_time_update) cout<<"**********************************************************\n\n";

		mupdt=0;
		epchup=time;

		//observation matrix HH
		for(i=0;i<3;i++){
			SHI.assign_loc(i,0,XH.get_loc(i,0));
			VHI.assign_loc(i,0,XH.get_loc(i+3,0));
		}
		SHB=TBIC*SHI;
		Matrix POLAR=SHB.pol_from_cart();
		double dtbh=POLAR.get_loc(0,0);
		double azh=POLAR.get_loc(1,0);
		double elh=POLAR.get_loc(2,0);
		double shb0=SHB.get_loc(0,0);
		double shb1=SHB.get_loc(1,0);
		double shb2=SHB.get_loc(2,0);
		double dtb01=sqrt(shb0*shb0+shb1*shb1);
		double shb02=shb0*shb0;
		double dsv=SHI^VHI;

		double ha1=(TBIC.get_loc(1,0)*shb0-TBIC.get_loc(0,0)*shb1)/shb02;
		double ha2=(TBIC.get_loc(1,1)*shb0-TBIC.get_loc(0,1)*shb1)/shb02;
		double ha3=(TBIC.get_loc(1,2)*shb0-TBIC.get_loc(0,2)*shb1)/shb02;

		double he1=(-TBIC.get_loc(2,0)*pow(dtb01,2)+shb2*(TBIC.get_loc(0,0)*shb0
			+TBIC.get_loc(1,0)*shb1))/pow(dtb01,3);
		double he2=(-TBIC.get_loc(2,1)*pow(dtb01,2)+shb2*(TBIC.get_loc(0,1)*shb0
			+TBIC.get_loc(1,1)*shb1))/pow(dtb01,3);
		double he3=(-TBIC.get_loc(2,2)*pow(dtb01,2)+shb2*(TBIC.get_loc(0,2)*shb0
			+TBIC.get_loc(1,2)*shb1))/pow(dtb01,3);                

		double ca7=XH.get_loc(6,0)/(1.+pow(SHB.get_loc(1,0)/SHB.get_loc(0,0),2));
		double ce8=XH.get_loc(7,0)/(1.+pow(-SHB.get_loc(2,0)/dtb01,2));

		HH.assign_loc(0,0,ca7*ha1);
        HH.assign_loc(0,1,ca7*ha2);
        HH.assign_loc(0,2,ca7*ha3);
        HH.assign_loc(1,0,ce8*he1);
        HH.assign_loc(1,1,ce8*he2);
        HH.assign_loc(1,2,ce8*he3);
        HH.assign_loc(2,0,XH.get_loc(0,0)/dtbh);
        HH.assign_loc(2,1,XH.get_loc(1,0)/dtbh);
        HH.assign_loc(2,2,XH.get_loc(2,0)/dtbh);

        double dtbh2=pow(dtbh,2);
        double dtbh3=pow(dtbh,3);

        HH.assign_loc(3,0,(dtbh2*XH.get_loc(3,0)-XH.get_loc(0,0)*dsv)/dtbh3);
        HH.assign_loc(3,1,(dtbh2*XH.get_loc(4,0)-XH.get_loc(1,0)*dsv)/dtbh3);
        HH.assign_loc(3,2,(dtbh2*XH.get_loc(5,0)-XH.get_loc(2,0)*dsv)/dtbh3);
        HH.assign_loc(3,3,HH.get_loc(2,0));
        HH.assign_loc(3,4,HH.get_loc(2,1));
        HH.assign_loc(3,5,HH.get_loc(2,2));
        HH.assign_loc(0,6,atan2(SHB.get_loc(1,0),SHB.get_loc(0,0)));
        HH.assign_loc(1,7,atan2(-SHB.get_loc(2,0),dtb01));

		if(time>debug_time_update) cout<<"**************** HH Observation Matrix ****************\n";
		if(time>debug_time_update) cout<<"*** HH\n";
		if(time>debug_time_update) HH.print();

		//filter gain calculations
		Matrix INV=HH*PMAT*~HH+RR;  // or HH.trans() 
		Matrix GK=PMAT*~HH*INV.inverse();

		//measurement residuals
		//extrapolated measurements
        ZH.assign_loc(0,0,azh);
        ZH.assign_loc(1,0,elh);
        ZH.assign_loc(2,0,dtbh);
        ZH.assign_loc(3,0,dsv/dtbh);
		//actual measurements 
        ZK.assign_loc(0,0,azab);
        ZK.assign_loc(1,0,elab);
        ZK.assign_loc(2,0,dab);
        ZK.assign_loc(3,0,ddab);
		//residual
		Matrix EZ=ZK-ZH;

		if(time>debug_time_update) cout<<"**************** EZ Measurement Residuals ****************\n";
		if(time>debug_time_update) cout<<"*** EZ\n";
		if(time>debug_time_update) EZ.print();

		//state update

		if(time>debug_time_update) cout<<"********************* XH Update **********************\n";
		if(time>debug_time_update) cout<<"*** XH before Update, time="<<time<<'\n';
		if(time>debug_time_update) XH.print();

		XH=XH+GK*EZ;

		if(time>debug_time_update) cout<<"*** XH after Update\n";
		if(time>debug_time_update) XH.print();

		for(i=0;i<3;i++){
			STBIK.assign_loc(i,0,XH.get_loc(i,0));
			VTBIK.assign_loc(i,0,XH.get_loc(i+3,0));
		}
		//covariance matrix update

		if(time>debug_time_update) cout<<"********************* P Update **********************\n";
		if(time>debug_time_update) cout<<"*** PMAT before Update\n";
		if(time>debug_time_update) PMAT.print();

		PMAT=(UNIT-GK*HH)*PMAT;

		if(time>debug_time_update) cout<<"*** PMAT after Update\n";
		if(time>debug_time_update) PMAT.print();

		//diagnostics: 1sigma error ellipse in plane normal to LOS vector
		//extracting position cov matrix
		for(i=0;i<3;i++){
			for(j=0;j<3;j++){
				COVPL.assign_loc(i,j,PMAT.get_loc(i,j));
			}
		}
		//LOS az & el
		POLAR=SHI.pol_from_cart();
		double azal=POLAR.get_loc(1,0);
		double elal=POLAR.get_loc(2,0);
		//T.M. between LOS and inertial coord
		Matrix TAI=mat2tr(azal,elal);					
		//converting position cov matrix to LOS coord
		COVPA=TAI*COVPL;
		//extracting 2-dim cov matrix in 2- and 3- directions
        COV23.assign_loc(0,0,COVPA.get_loc(1,1));
        COV23.assign_loc(1,1,COVPA.get_loc(2,2));
        COV23.assign_loc(0,1,COVPA.get_loc(1,2));
        COV23.assign_loc(1,0,COV23.get_loc(0,1));
		//getting bi-variate ellpise
		Matrix ELLIPSE=COV23.ellipse();	
		semi_major=ELLIPSE.get_loc(0,0);
		semi_minor=ELLIPSE.get_loc(1,0);
		ellipse_anglx=DEG*ELLIPSE.get_loc(2,0);

		//diagnostics: state residuals = true - est. state
		STBI=STII-SBII;
		VTBI=VTII-VBII;
		ESTBI=STBI-SHI;
		EVTBI=VTBI-VHI;

		if(time>debug_time_update) cout<<"********************* Pos State Residuals **********************\n";
		if(time>debug_time_update) cout<<"*** ESTBI\n";
		if(time>debug_time_update) ESTBI.print();

		//diagnostic: meas error = ideal - actual meas
		Matrix STBB=TBI*STBI;
		POLAR=STBB.pol_from_cart();
		double dabi=POLAR.get_loc(0,0);
		double azabi=POLAR.get_loc(1,0);
		double elabi=POLAR.get_loc(2,0);
		Matrix UTBI=STBI.univec3();
		double ddabi=UTBI^VTBI;
        eaz=azabi-azab;
        eel=elabi-elab;
        edab=dabi-dab;
        eddab=ddabi-ddab;
	}
//			if(time>debug_time_update) cout<<"********************* Exit **********************\n";
//			if(time>debug_time_update) cout<<"*** PMAT exiting fuction\n";
//			if(time>debug_time_update) PMAT.print();
//			if(time>debug_time_update) cout<<"*** XH exiting fuction\n";
//			if(time>debug_time_update) XH.print();
	//-------------------------------------------------------------------------
	//decomposing state vector for saving
	for(m=0;m<3;m++){
		SXH_SKR.assign_loc(m,0,XH.get_loc(m,0));
		VXH_SKR.assign_loc(m,0,XH.get_loc(m+3,0));
	}
	SFH.assign_loc(0,0,XH.get_loc(6,0));
	SFH.assign_loc(1,0,XH.get_loc(7,0));
	//decomposing covariance matrix for saving as 3x3 matrices
	for(n=0;n<8;n++){
		VEC1.assign_loc(n,0,PMAT.get_loc(0,n));
		VEC2.assign_loc(n,0,PMAT.get_loc(1,n));
		VEC3.assign_loc(n,0,PMAT.get_loc(2,n));
		VEC4.assign_loc(n,0,PMAT.get_loc(3,n));
		VEC5.assign_loc(n,0,PMAT.get_loc(4,n));
		VEC6.assign_loc(n,0,PMAT.get_loc(5,n));
		VEC7.assign_loc(n,0,PMAT.get_loc(6,n));
		VEC8.assign_loc(n,0,PMAT.get_loc(7,n));
	}
	PMAT1=VEC1.mat33_vec9();
	PMAT2=VEC2.mat33_vec9();
	PMAT3=VEC3.mat33_vec9();
	PMAT4=VEC4.mat33_vec9();
	PMAT5=VEC5.mat33_vec9();
	PMAT6=VEC6.mat33_vec9();
	PMAT7=VEC7.mat33_vec9();
	PMAT8=VEC8.mat33_vec9();
	//loading module-variables
	//saving values
	hyper[250].gets(init_filter);
	hyper[251].gets(epchup);
	hyper[266].gets(flag_out);
	hyper[267].gets(mupdt);
	hyper[283].gets_vec(SXH_SKR);
	hyper[284].gets_vec(VXH_SKR);
	hyper[285].gets_vec(SFH);
	hyper[286].gets_mat(PMAT1);
	hyper[287].gets_mat(PMAT2);
	hyper[288].gets_mat(PMAT3);
	hyper[289].gets_mat(PMAT4);
	hyper[290].gets_mat(PMAT5);
	hyper[291].gets_mat(PMAT6);
	hyper[292].gets_mat(PMAT7);
	hyper[293].gets_mat(PMAT8);
	hyper[294].gets(dtim);
	//saving for next cycle  and diagnostics
	hyper[272].gets_vec(SIGPOS);	
	hyper[273].gets_vec(SIGVEL); 
	hyper[274].gets(semi_major);
	hyper[275].gets(semi_minor);
	hyper[276].gets(ellipse_anglx);
	hyper[277].gets_vec(ESTBI);
	hyper[278].gets_vec(EVTBI);
	hyper[279].gets(eaz);
	hyper[280].gets(eel);
	hyper[281].gets(edab);
	hyper[282].gets(eddab);
}