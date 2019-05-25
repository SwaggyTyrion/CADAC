///////////////////////////////////////////////////////////////////////////////
//FILE: 'gps.cpp'
//Contains 'gps' module of class 'Hyper'
//
//040105 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of GPS/Filter module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[700-799]
//
//mgps	= 0 no GPS (default)
//		= 1 GPS initialized (input)
//		= 2 GPS filter extrapolation (internal)
//		= 3 GPS update to INS (internal;'ins' module resets mgps=2) 
//
//040105 Created by Peter H Zipfel
//141125 Constellation update to: Yuma Almanac Week 787 (21 Sep 2014), PZi
//150126 Added code to plot quadriga on GLOBE, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_gps()
{
	//definition of module-variables
	//GPS measurement
	hyper[700].init("mgps","int",0,"=0:no GPS; =1:init; =2:extrapol; =3:update - ND","gps","data","");
	hyper[702].init("almanac_time",0,"Time since almanac epoch at sim start - sec","gps","data","");
	hyper[703].init("del_rearth",0,"Delta to Earth's radius for GPS clear LOS signal reception - m","gps","data","");
	hyper[704].init("gdop",0,"Geometric dilution of precision - m","gps","diag","");
	hyper[705].init("gps_acqtime",0,"Acquisition time for GPS signal - s","gps","data","");
	hyper[706].init("gps_step",0,"GPS update interval - s","gps","data","");
	hyper[707].init("gps_epoch",0,"GPS update epoch time since launch - s","gps","save","");
	hyper[708].init("gps_acq","int",0,"=0: GPS not initialized; =1: initialized - ND ","gps","save","");
	hyper[709].init("ucfreq_noise",0,"User clock frequency error - m/s MARKOV","gps","data","");
	hyper[710].init("ucbias_error",0,"User clock bias error - m GAUSS","gps","data","scrn,plot");
	hyper[711].init("ucfreq_error",0,"User clock frequency error - m ","gps","diag","scrn,plot");
	hyper[713].init("ucfreqm",0,"User clock frequency state - m/s","gps","save","");
	hyper[714].init("pr1_bias",0,"Pseudo-range 1 bias - m GAUSS","gps","data","");
	hyper[715].init("pr2_bias",0,"Pseudo-range 2 bias - m GAUSS","gps","data","");
	hyper[716].init("pr3_bias",0,"Pseudo-range 3 bias - m GAUSS","gps","data","");
	hyper[717].init("pr4_bias",0,"Pseudo-range 4 bias - m GAUSS","gps","data","");	
	hyper[718].init("pr1_noise",0,"Pseudo-range 1 noise - m MARKOV","gps","data","");
	hyper[719].init("pr2_noise",0,"Pseudo-range 2 noise - m MARKOV","gps","data","");
	hyper[720].init("pr3_noise",0,"Pseudo-range 3 noise - m MARKOV","gps","data","");
	hyper[721].init("pr4_noise",0,"Pseudo-range 4 noise - m MARKOV","gps","data","");
	hyper[722].init("dr1_noise",0,"Delta-range 1 noise - m/s MARKOV","gps","data","");
	hyper[723].init("dr2_noise",0,"Delta-range 2 noise - m/s MARKOV","gps","data","");
	hyper[724].init("dr3_noise",0,"Delta-range 3 noise - m/s MARKOV","gps","data","");
	hyper[725].init("dr4_noise",0,"Delta-range 4 noise - m/s MARKOV","gps","data","");
	hyper[726].init("slotsum",0,"Sum of stored slot numbers of quadriga - ND","gps","save","");
	//Z150126  plotting quadriga
	hyper[727].init("lon1",0,"Longitude of 1st SV - deg","gps","dia","");
	hyper[728].init("lat1",0,"Latitude of 1st SV - deg","gps","dia","");
	hyper[729].init("alt1",0,"Altitude of 1st SV - m","gps","dia","");
	hyper[730].init("lon2",0,"Longitude of 2st SV - deg","gps","dia","");
	hyper[731].init("lat2",0,"Latitude of 2st SV - deg","gps","dia","");
	hyper[732].init("alt2",0,"Altitude of 2st SV - m","gps","dia","");
	hyper[733].init("lon3",0,"Longitude of 3st SV - deg","gps","dia","");
	hyper[734].init("lat3",0,"Latitude of 3st SV - deg","gps","dia","");
	hyper[735].init("alt3",0,"Altitude of 3st SV - m","gps","dia","");
	hyper[736].init("lon4",0,"Longitude of 4st SV - deg","gps","dia","");
	hyper[737].init("lat4",0,"Latitude of 4st SV - deg","gps","dia","");
	hyper[738].init("alt4",0,"Altitude of 4st SV - m","gps","dia","");
	//GPS filter
	hyper[750].init("uctime_cor",0,"User clock correlation time constant - s","gps","data","");
	hyper[751].init("ppos",0,"Init 1sig pos values of state cov matrix - m","gps","data","");
	hyper[752].init("pvel",0,"Init 1sig vel values of state cov matrix - m/s","gps","data","");
	hyper[753].init("pclockb",0,"Init 1sig clock bias error of state cov matrix - m","gps","data","");
	hyper[754].init("pclockf",0,"Init 1sig clock freq error of state cov matrix - m/s","gps","data","");
	hyper[755].init("qpos",0,"1sig pos values of process cov matrix - m","gps","data","");
	hyper[756].init("qvel",0,"1sig vel values of process cov matrix - m/s","gps","data","");
	hyper[757].init("qclockb",0,"1sig clock bias error of process cov matrix - m","gps","data","");
	hyper[758].init("qclockf",0,"1sig clock freq error of process cov matrix - m/s","gps","data","");
	hyper[759].init("rpos",0,"1sig pos value of meas cov matrix - m","gps","data","");
	hyper[760].init("rvel",0,"1sig vel value of meas cov matrix - m/s","gps","data","");
	hyper[761].init("factp",0,"Factor to modifiy initial P-matrix P(1+factp)","gps","data","");
	hyper[762].init("factq",0,"Factor to modifiy the Q-matrix Q(1+factq)","gps","data","");
	hyper[763].init("factr",0,"Factor to modifiy the R-matrix R(1+factr)","gps","data","");
	hyper[764].init("SXH",0,0,0,"Position state (inertial coor) - m","gps","out","plot");
	hyper[765].init("VXH",0,0,0,"Velocity  state (inertial coor) - m/s","gps","out","plot");
	hyper[766].init("CXH",0,0,0,"clock state (only two values) - m, m/s","gps","save","plot");
	hyper[767].init("PP1",0,0,0,0,0,0,0,0,0,"Covariance Matrix 1st row - mixed","gps","save","");
	hyper[768].init("PP2",0,0,0,0,0,0,0,0,0,"Covariance Matrix 2nd row - mixed","gps","save","");
	hyper[769].init("PP3",0,0,0,0,0,0,0,0,0,"Covariance Matrix 3rd row - mixed","gps","save","");
	hyper[770].init("PP4",0,0,0,0,0,0,0,0,0,"Covariance Matrix 4th row - mixed","gps","save","");
	hyper[771].init("PP5",0,0,0,0,0,0,0,0,0,"Covariance Matrix 5th row - mixed","gps","save","");
	hyper[772].init("PP6",0,0,0,0,0,0,0,0,0,"Covariance Matrix 6th row - mixed","gps","save","");
	hyper[773].init("PP7",0,0,0,0,0,0,0,0,0,"Covariance Matrix 7th row - mixed","gps","save","");
	hyper[774].init("PP8",0,0,0,0,0,0,0,0,0,"Covariance Matrix 8th row - mixed","gps","save","");
	hyper[775].init("std_pos",0,"Std deviation of position from P matrix - m","gps","diag","plot");
	hyper[776].init("std_vel",0,"Std deviation of velocity from P matrix - m/s","gps","diag","plot");
	hyper[777].init("std_ucbias",0,"Std deviation of user clock bias from P matrix - m","gps","diag","plot");
	//additional diagnostics
	hyper[778].init("gps_pos_meas",0,"GPS position measurement residuals - m","gps","diag","scrn,plot");
	hyper[779].init("gps_vel_meas",0,"GPS velocity measurement residuals - m/s","gps","diag","scrn,plot");
	hyper[780].init("state_pos",0,"State x absolute position value - m","gps","diag","scrn,plot");
	hyper[781].init("state_vel",0,"State x absolute velocity value - m","gps","diag","scrn,plot");
}
///////////////////////////////////////////////////////////////////////////////  
//GPS module
//Member function of class 'Hyper'
//
//mgps	= 0 no GPS (default)
//		= 1 GPS initialized (input)
//		= 2 GPS filter extrapolation (internal)
//		= 3 GPS update to INS (internal;'ins' module resets mgps=2) 
//
//* GPS measurements 
//  Four pseudo-ranges and four delta-ranges (range rates) are measured from the
//   four-satallite configuration (called quadriga) that has the lowest GDOP.
//  The GPS satellites, called space vehicles (SV), are initialized at their locations
//   given by the Yuma Almanac Week 787 (21 Sep 2014)
//  The 24 satellites are arranged in 6 circular orbits at intervals of 60 deg right ascension
//   and 55 deg inclination
//  The pseudo-range errors are ephemeris, ionospheric and tropospheric; receiver noise and bias
//  The delta-range errors are receiver dynamic noise
//	The user clock errors are bias and frequency           
//
//* Kalman filter
//  Eight states: 3 postion, 3 velocity, clock bias and frequency
//  Observation matrix is 8x8 and nonlinear (-> extended K.F.)
//	The position and velocity states update the INS nav solution
//	Clock bias is updated 
//
//040105 Created by Peter H Zipfel
//141125 Constellation update to: Yuma Almanac Week 787 (21 Sep 2014), PZi
///////////////////////////////////////////////////////////////////////////////
	
void Hyper::gps(double int_step)
{
	//local variables
	//GPS
	static double rsi(0); //constant, same for all objects, -> static ok
	static double wsi(0); //constant, same for all objects, -> static ok;
	static double incl(0); //constant, same for all objects, -> static ok
	static double sv_init_data[48];
	double ssii_quad[16]; //quadriga inertial coordinates and SV slot#
	double vsii_quad[12]; //quadriga inertial velocities
	double dtime_gps(0);
	double time_gps(0);
	double slot[4]={0,0,0,0}; //SV slot#  of quadriga
	Matrix ZZ(8,1);
	double slotm(0);
	Matrix PR_BIAS(4,1);	
	Matrix PR_NOISE(4,1);
	Matrix DR_NOISE(4,1);
	//filter
	static Matrix FF(8,8); //constant, same for all objects, -> static ok
	static Matrix PHI(8,8);//constant, same for all objects, -> static ok
	Matrix XH(8,1); //local
	Matrix RR(8,8); //local
	Matrix QQ(8,8); //local
	Matrix HH(8,8); //local
	Matrix PP(8,8);  //recursive, must be saved; separated into 8 PPx(3x3)
	int i(0);
	int j(0);
	int n(0);
	int m(0);
	//local module-variables
	double gdop(0);
	double ucfreq_error(0);
	double std_pos(0);
	double std_vel(0);
	double std_ucbias(0);
	//localizing module-variables
	//input data
	int mgps=hyper[700].integer();
	double almanac_time=hyper[702].real();
	double del_rearth=hyper[703].real();
	double gps_acqtime=hyper[705].real();
	double gps_step=hyper[706].real();
	double ucfreq_noise=hyper[709].real();
	double ucbias_error=hyper[710].real();
	double pr1_bias=hyper[714].real();
	double pr2_bias=hyper[715].real();
	double pr3_bias=hyper[716].real();
	double pr4_bias=hyper[717].real();	
	double pr1_noise=hyper[718].real();
	double pr2_noise=hyper[719].real();
	double pr3_noise=hyper[720].real();
	double pr4_noise=hyper[721].real();
	double dr1_noise=hyper[722].real();
	double dr2_noise=hyper[723].real();
	double dr3_noise=hyper[724].real();	
	double dr4_noise=hyper[725].real();
	//assemble bias and noise measurement vectors 
	PR_BIAS[0]=pr1_bias;
	PR_BIAS[1]=pr2_bias;
	PR_BIAS[2]=pr3_bias;
	PR_BIAS[3]=pr4_bias;
	PR_NOISE[0]=pr1_noise;
	PR_NOISE[1]=pr2_noise;
	PR_NOISE[2]=pr3_noise;
	PR_NOISE[3]=pr4_noise;
	DR_NOISE[0]=dr1_noise;
	DR_NOISE[1]=dr2_noise;
	DR_NOISE[2]=dr3_noise;
	DR_NOISE[3]=dr4_noise;
	//input data filter
	double uctime_cor=hyper[750].real();
	double ppos=hyper[751].real();
	double pvel=hyper[752].real();
	double pclockb=hyper[753].real();
	double pclockf=hyper[754].real();
	double qpos=hyper[755].real();
	double qvel=hyper[756].real();
	double qclockb=hyper[757].real();
	double qclockf=hyper[758].real();
	double rpos=hyper[759].real();
	double rvel=hyper[760].real();
	double factp=hyper[761].real();
	double factq=hyper[762].real();
	double factr=hyper[763].real();
	//getting saved values of gps
	double gps_epoch=hyper[707].real();
	int gps_acq=hyper[708].integer();
	double ucfreqm=hyper[713].real();
	double slotsum=hyper[726].real();
	double gps_pos_meas=hyper[778].real();
	double gps_vel_meas=hyper[779].real();

	//getting saved values of quadriga
	double lon1=hyper[727].real();
	double lat1=hyper[728].real();
	double alt1=hyper[729].real();
	double lon2=hyper[730].real();
	double lat2=hyper[731].real();
	double alt2=hyper[732].real();
	double lon3=hyper[733].real();
	double lat3=hyper[734].real();
	double alt3=hyper[735].real();
	double lon4=hyper[736].real();
	double lat4=hyper[737].real();
	double alt4=hyper[738].real();

	//input from other modules
	double time=round6[0].real();
	Matrix WBII=round6[166].vec();
	Matrix SBII=round6[235].vec();
	Matrix VBII=round6[236].vec();
	Matrix VBIIC=hyper[303].vec();
	Matrix SBIIC=hyper[304].vec();
	Matrix WBICI=hyper[305].vec();
	//getting saved values of filter
	Matrix SXH=hyper[764].vec();
	Matrix VXH=hyper[765].vec();
	Matrix CXH=hyper[766].vec();
	Matrix PP1=hyper[767].mat();
	Matrix PP2=hyper[768].mat();
	Matrix PP3=hyper[769].mat();
	Matrix PP4=hyper[770].mat();
	Matrix PP5=hyper[771].mat();
	Matrix PP6=hyper[772].mat();
	Matrix PP7=hyper[773].mat();
	Matrix PP8=hyper[774].mat();
	double state_pos=hyper[780].real();
	double state_vel=hyper[781].real();
	//assembling covariance matrix from saved 3x3 matrices
	Matrix VEC1=PP1.vec9_mat33();
	Matrix VEC2=PP2.vec9_mat33();
	Matrix VEC3=PP3.vec9_mat33();
	Matrix VEC4=PP4.vec9_mat33();
	Matrix VEC5=PP5.vec9_mat33();
	Matrix VEC6=PP6.vec9_mat33();
	Matrix VEC7=PP7.vec9_mat33();
	Matrix VEC8=PP8.vec9_mat33();
	for(int n=0;n<8;n++){
		//VECx, x=0...7, is the x-th row of PP (8th element is zero)
		PP.assign_loc(0,n,VEC1.get_loc(n,0));
		PP.assign_loc(1,n,VEC2.get_loc(n,0));
		PP.assign_loc(2,n,VEC3.get_loc(n,0));
		PP.assign_loc(3,n,VEC4.get_loc(n,0));
		PP.assign_loc(4,n,VEC5.get_loc(n,0));
		PP.assign_loc(5,n,VEC6.get_loc(n,0));
		PP.assign_loc(6,n,VEC7.get_loc(n,0));
		PP.assign_loc(7,n,VEC8.get_loc(n,0));
	}
	//-----------------------------------------------------------------------------
	//returning, if no gps
	if(mgps==0)
	{
		return;
	}

	//GPS initializations
	if(mgps==1)
	{
		//24 SVs initialization
		gps_sv_init(sv_init_data,rsi,wsi,incl);

		//filter initialization
		//covariance matrix
		for(i=0;i<3;i++){
			PP.assign_loc(i,i,pow(ppos*(1+factp),2));
			PP.assign_loc(i+3,i+3,pow(pvel*(1+factp),2));
		}
		PP.assign_loc(6,6,pow(pclockb*(1+factp),2));
		PP.assign_loc(7,7,pow(pclockf*(1+factp),2));

		//fundamental dynamic matrix of filter - constant throughout
		FF.assign_loc(0,3,1);
		FF.assign_loc(1,4,1);
		FF.assign_loc(2,5,1);
		FF.assign_loc(6,7,1);
		FF.assign_loc(7,7,-1/uctime_cor);

		//state transition matrix - constant throughout
		Matrix EYE(8,8);
		PHI=EYE.identity()+FF*int_step+FF*FF*(int_step*int_step/2);

		/*/diagnostic - start
		cout<<"PP = \n";
		PP.print();
		cout<<"FF = \n";
		FF.print();
		cout<<"PHI = \n";
		PHI.print();
		//diagnostic - end */
		
		//setting inital acquisition flag
		gps_acq=1;

		//initializing update clock
		gps_epoch=time;

		//initiating filter extrapolation
		mgps=2;

	}
	//user clock error growth and filter extrapolation 
	if(mgps==2)
	{
		if(gps_acq)
			//saving delay-time for GPS signal acquisition
			dtime_gps=gps_acqtime;
		else
			//saving delay-time for GPS update
			dtime_gps=gps_step;

		//checking when GPS update time has occured in order to initiate update 
		time_gps=time-gps_epoch;
		if(time_gps>=dtime_gps){
			mgps=3;
		}
		//*** user-clock frequency and bias error growth between updates ***
		// integrating 'ucfreq_noise' Markov process to
		//  obtain user-clock bias error 'ucbias_error' (trapezoidal integration)
		// user-clock bias is updated at filter update epoch
		ucfreq_error=ucfreq_noise;
		ucbias_error=ucbias_error+(ucfreq_error+ucfreqm)*(int_step/2);
		ucfreqm=ucfreq_error;

		//*** filter extrapolation ***
		//dynamic error covariance matrix
		for(i=0;i<3;i++){
			QQ.assign_loc(i,i,pow(qpos*(1+factq),2));
			QQ.assign_loc(i+3,i+3,pow(qvel*(1+factq),2));
		}
		QQ.assign_loc(6,6,pow(qclockb*(1+factq),2));
		QQ.assign_loc(7,7,pow(qclockf*(1+factq),2));
		
		//covariance estimate extrapolation
		PP=PHI*(PP+QQ*(int_step/2))*~PHI+QQ*(int_step/2);
		//diagnostics: st. deviations of the diagonals of the covariance matrix
		std_pos=sqrt(PP.get_loc(0,0));
		std_vel=sqrt(PP.get_loc(3,3));
		std_ucbias=sqrt(PP.get_loc(6,6));		
	}
	//filter update epoch
	if(mgps==3)
	{
		//cancelling initial GPS acquisition flag
		gps_acq=0;

		//resetting update clock
		time_gps=0;
		gps_epoch=time;

		//*** SV propagation and quadriga selection 'ssii_quad' (4 SVs with best GDOP) ***
		gps_quadriga(ssii_quad,vsii_quad,gdop,mgps, sv_init_data,rsi,wsi,incl,almanac_time,del_rearth,time,SBII);
		
		//Pseudo-range and range-rate measurements
		for(i=0;i<4;i++){
			//unpacking i-th SV inertial position
			Matrix SSII(3,1);
			for(j=0;j<3;j++){
				SSII[j]=*(ssii_quad+4*i+j);
			}
			//Z150126 - start
			//diagnostics: getting long, lat, alt of the four quadriga SVs for plotting in GLOBE 		
			double lon(0),lat(0),alt(0);
			cad_geo84_in(lon,lat,alt,SSII,time);
			if (i==0) {lon1=lon*DEG;lat1=lat*DEG;alt1=alt;};			 
			if (i==1) {lon2=lon*DEG;lat2=lat*DEG;alt2=alt;};				 
			if (i==2) {lon3=lon*DEG;lat3=lat*DEG;alt3=alt;};				 
			if (i==3) {lon4=lon*DEG;lat4=lat*DEG;alt4=alt;};			 
			//Z150126 - end
			
			//calculating true range to SV
			Matrix SSBI(3,1);
			SSBI=SSII-SBII;
			double dsb=SSBI.absolute();

			//measured pseudo-range
			double dsb_meas=dsb+PR_BIAS[i]+PR_NOISE[i]+ucbias_error;

			//unpacking i-th SV inertial velocity
			Matrix VSII(3,1);
			for(j=0;j<3;j++){
				VSII[j]=*(vsii_quad+3*i+j);
			}			
			//velocity of SV wrt user
			Matrix VSBI(3,1);
			VSBI=VSII-VBII-WBII.skew_sym()*SSBI;

			//calculating true range-rate to SV
			Matrix USSBI(3,1);
			USSBI=SSBI*(1/dsb);
			double dvsb=VSBI^USSBI;

			//measured delta-range rate
			double dvsb_meas=dvsb+DR_NOISE[i]+ucfreq_error;

			//INS derived range measurements
			Matrix SSBIC(3,1);
			SSBIC=SSII-SBIIC;
			double dsbc=SSBIC.absolute();

			//INS derived range-rate measurements
			//velocity of SV wrt user
			Matrix VSBIC(3,1);
			VSBIC=VSII-VBIIC-WBICI.skew_sym()*SSBIC;
			//calculating range-rate to SV
			Matrix USSBIC(3,1);
			USSBIC=SSBIC*(1/dsb);
			double dvsbc=VSBIC^USSBIC;

			//loading measurement residuals into measurement vector
			// ZZ[0->3] range meas resid of SV's; ZZ[4->7] range-rate meas resid of SV's
			ZZ[i]=dsb_meas-dsbc;
			ZZ[i+4]=dvsb_meas-dvsbc;

			//observation matrix of filter
			for(j=0;j<3;j++){
				HH.assign_loc(i,j,USSBI.get_loc(j,0));
				HH.assign_loc(i+4,j+3,USSBI.get_loc(j,0)*gps_step);
			}
			HH.assign_loc(i,6,1);
			HH.assign_loc(i+4,7,gps_step);

			//for diagnostics: loading the 4 SV slot # of the quadriga
			*(slot+i)=*(ssii_quad+4*i+3);
			//accumulating sum of slots
			slotm=slotm+slot[i];
		}
		//for diagnostics displaying the SV slot# of the quadriga on the console
		// but only if they have changed (i.e., sum of slot# has changed)
		if(slotsum!=slotm){
			slotsum=slotm;
			cout<<" *** GPS Quadriga slot # "<<slot[0]<<"  "<<slot[1]<<"  "<<slot[2]<<"  "<<slot[3]<<" ;  GDOP = "<<gdop<<" m ***\n";
		}
		//*** filter correction and update (to INS: 'SXH' and 'VXH') ***
		//filter gain
		Matrix KK(8,8);
		/*/z150612
		cout<<"PP = \n";
		PP.print();
		cout<<"QQ = \n";
		QQ.print();
		//z150612*/

		//measurement noise covariance matrix
		for(i=0;i<4;i++){
			RR.assign_loc(i,i,pow(rpos*(1+factr),2));
			RR.assign_loc(i+4,i+4,pow(rvel*(1+factr),2));
		}		
		//Kalman gain
		KK=PP*~HH*(HH*PP*~HH+RR).inverse();
		//state correction
		XH=KK*ZZ;
		//covariance correction for next cycle
		Matrix EYE(8,8);
		PP=(EYE.identity()-KK*HH)*PP;

		//clock error bias update
		ucbias_error=ucbias_error-XH.get_loc(6,0);

		/*/diagnostic print-out - start
		cout<<" *** Update Epoch ***\n";
		cout<<"Position and velocity measurement ZZ \n";
		ZZ.print();
		cout<<" Observation matrix HH = \n";
		HH.print();
		cout<<" Meas. cov. matrix RR = \n";
		RR.print();
		cout<<" Kalman gain KK = \n";
		KK.print();
		cout<<" State XH = \n";
		XH.print();
		cout<<"Updated covariance matrix PP = \n";
		PP.print();
		//diagnostic print-out - end*/

		//diagnostics of 1st SV of quadriga saved to plot file 
		gps_pos_meas=ZZ.get_loc(0,0);
		gps_vel_meas=ZZ.get_loc(4,0);

		//decomposing state vector for output
		for(int m=0;m<3;m++){
			SXH.assign_loc(m,0,XH.get_loc(m,0));
			VXH.assign_loc(m,0,XH.get_loc(m+3,0));
		}
		CXH.assign_loc(0,0,XH.get_loc(6,0));
		CXH.assign_loc(1,0,XH.get_loc(7,0));

		//diagnostic
		state_pos=SXH.absolute();
		state_vel=VXH.absolute();

	}//end of update
	//-----------------------------------------------------------------------------
	//decomposing covariance matrix for saving as 3x3 matrices
	for(int n=0;n<8;n++){
		VEC1.assign_loc(n,0,PP.get_loc(0,n));
		VEC2.assign_loc(n,0,PP.get_loc(1,n));
		VEC3.assign_loc(n,0,PP.get_loc(2,n));
		VEC4.assign_loc(n,0,PP.get_loc(3,n));
		VEC5.assign_loc(n,0,PP.get_loc(4,n));
		VEC6.assign_loc(n,0,PP.get_loc(5,n));
		VEC7.assign_loc(n,0,PP.get_loc(6,n));
		VEC8.assign_loc(n,0,PP.get_loc(7,n));
	}
	PP1=VEC1.mat33_vec9();
	PP2=VEC2.mat33_vec9();
	PP3=VEC3.mat33_vec9();
	PP4=VEC4.mat33_vec9();
	PP5=VEC5.mat33_vec9();
	PP6=VEC6.mat33_vec9();
	PP7=VEC7.mat33_vec9();
	PP8=VEC8.mat33_vec9();
	//loading module-variables
	//saving values of filter
	hyper[767].gets_mat(PP1);
	hyper[768].gets_mat(PP2);
	hyper[769].gets_mat(PP3);
	hyper[770].gets_mat(PP4);
	hyper[771].gets_mat(PP5);
	hyper[772].gets_mat(PP6);
	hyper[773].gets_mat(PP7);
	hyper[774].gets_mat(PP8);
	//output to other modules
	hyper[764].gets_vec(SXH);
	hyper[765].gets_vec(VXH);
	//saving values of GPS
	hyper[700].gets(mgps);
	hyper[707].gets(gps_epoch);
	hyper[708].gets(gps_acq);
	hyper[710].gets(ucbias_error);
	hyper[713].gets(ucfreqm);
	hyper[726].gets(slotsum);	
	//diagnostics
	hyper[704].gets(gdop);
	hyper[711].gets(ucfreq_error);
	hyper[766].gets_vec(CXH);
	hyper[775].gets(std_pos);
	hyper[776].gets(std_vel);
	hyper[777].gets(std_ucbias);
	hyper[778].gets(gps_pos_meas);
	hyper[779].gets(gps_vel_meas);
	hyper[780].gets(state_pos);
	hyper[781].gets(state_vel);
	//Z150126 plotting quadriga
	hyper[727].gets(lon1);
	hyper[728].gets(lat1);
	hyper[729].gets(alt1);
	hyper[730].gets(lon2);
	hyper[731].gets(lat2);
	hyper[732].gets(alt2);
	hyper[733].gets(lon3);
	hyper[734].gets(lat3);
	hyper[735].gets(alt3);
	hyper[736].gets(lon4);
	hyper[737].gets(lat4);
	hyper[738].gets(alt4);
}
///////////////////////////////////////////////////////////////////////////////
//Selection of the best four SVs (quadriga)
//Member function of class 'Hyper'
//Assumptions: (1) SVs on circular orbits at 55 deg inclination and  separated 
// by equal intervals of 60 deg right ascension (starting at GPS Almanac 224 week);
//
// parameter input:
//	*sv_init_data = 24x2 array: first column: right ascension- rad
//					            second column: argument of latitude - rad
//					stored in one-dimensional array 'sv_init_data[48]' in sequential rows
//	rsi = central radius of SV orbits - m
//	wsi = angular velocity of SVs - rad/s
//	incl = inclination of orbital planes - rad
//	almanac_time = time since almanac epoch at start of simulation (data) - sec
//	del_rearth = increase added to Earth's radius for GPS signal LOS calculations (data) - m
//	time = simulation time - sec
//	SBII = inertial coordinates of hypersonic vehicle - m
//
// parameter output:
//	*ssii_quad = inertial coordinates + slot# of each quadriga SV, stored sequentially - m
//	*vsii_quad = inertial velocities of each SV of the quadriga , stored sequentially - m/s
//	gdop = geometric dillution of precision of quadriga - m
//	mgps = set here to 1 (GPS initialization), if less than  4 SVs are visible 
//	
//040105 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::gps_quadriga(double *ssii_quad,double *vsii_quad,double &gdop,int &mgps 
						,const double *sv_init_data,const double &rsi,const double &wsi
						,const double &incl,double almanac_time,double del_rearth
						,double time,Matrix SBII)
{
	int i(0);
	int m(0);
	double sv_data[24][2];
	double slot[4]={0,0,0,0}; //SV slot#  of quadriga 
	int islot[4]={0,0,0,0}; //SV slot#  of quadriga stored as integer 

	int quad[4]={0,0,0,0}; //location of quadriga SVs in 'ssii_vis[visible_count]
	gdop=LARGE;

	//unpacking the one-dimensional array of SVs into sv_data[24][2]
	for (i=0;i<24;i++){
		for(int k=0;k<2;k++){
			sv_data[i][k]=*(sv_init_data+2*i+k);
		}
	}	
	//propagating the argument of latitude in time
	for(i=0;i<24;i++){
		sv_data[i][1]=sv_data[i][1]+(almanac_time+time)*wsi;
	}
	//conversion to inertial (J2000) coordinates
	double ssii[24][4];
	double sin_incl=sin(incl);
	double cos_incl=cos(incl);
	for(i=0;i<24;i++){
		ssii[i][0]=rsi*(cos(sv_data[i][0])*cos(sv_data[i][1])-sin(sv_data[i][0])*sin(sv_data[i][1])*cos_incl);
		ssii[i][1]=rsi*(sin(sv_data[i][0])*cos(sv_data[i][1])+cos(sv_data[i][0])*sin(sv_data[i][1])*cos_incl);
		ssii[i][2]=rsi*sin(sv_data[i][1])*sin_incl;
		ssii[i][3]=0;
		//last entry is a flag with the code:
		//=0:not visible; >0:visible (not int but double!), where the number is the SV slot# (1,2,3,...,24)		
	}
	//determining visible satellites
	Matrix SSII(3,1);
	bool visible=false;
	int visible_count(0);

	for(i=0;i<24;i++){
		visible=false;
		SSII[0]=ssii[i][0];
		SSII[1]=ssii[i][1];
		SSII[2]=ssii[i][2];

		//SV to user angle with vertex at Earth center
		double delta=angle(SSII,SBII);

		//grazing angle of SV beam with vertex at Earth center
		double epsilon=acos((REARTH+del_rearth)/rsi);

		//min radius of user to have clear line-of-site to SV
		double rmin=(REARTH+del_rearth)/cos(delta-epsilon);

		if(delta<epsilon){
			//SV is in the visibility cone
			ssii[i][3]=i+1;
			visible_count++;
		}
		else{
			//user is outside the visibility cone but high enough to have LOS to the SV
			//(rmin can go negative if delta-epsilon>90deg; this can happen when the user is
			// opposite (or nearly opposite) of the SV; this is always a no-visibility case)
			double dbi=SBII.absolute();
			if(rmin>0&&rmin<dbi){
				ssii[i][3]=i+1;
				visible_count++;
			}
		}
	}
	//re-acquiring GPS if not enough SVs visible (less than 4)
	if(visible_count<4){
		mgps=1;
		cout<<" *** Warning: only "<<visible_count<<" SV are visible, mgps set = 1 ***\n";
	}
	//selecting best 4 SVs if 4 or more are visible
	else{

		//repackage visible SVs into 'ssii_vis' single-dimensioned array
		//inertial displacement vector elements are stored sequentially in 'ssii_vis[4*visible_count]'
		// 'ssii_vis' has 3 inertial coordinates and SV slot# of all visible SVs
		double *ssii_vis;
		ssii_vis=new double[4*visible_count];
		int k(0);
		for(i=0;i<24;i++){
			if(ssii[i][3]>0){
				*(ssii_vis+k)=ssii[i][0];
				*(ssii_vis+k+1)=ssii[i][1];
				*(ssii_vis+k+2)=ssii[i][2];
				*(ssii_vis+k+3)=ssii[i][3];
				k=k+4;
			}
		}
		//selecting quadriga (four SVs) with smallest GDOP
		//i1, i2, i3, i4 are the SVs picked by the binomial combination
		int nm3=visible_count-3; //nm3=1 
		int nm2=visible_count-2; //nm2=2
		int nm1=visible_count-1; //nm1=3
		
		for(int i1=0;i1<nm3;i1++){
			for(int i2=i1+1;i2<nm2;i2++){
				for(int i3=i2+1;i3<nm1;i3++){
					for(int i4=i3+1;i4<visible_count;i4++){

						//pullling the quadriga inertial coordinates
						Matrix SSII1(3,1);
						Matrix SSII2(3,1);
						Matrix SSII3(3,1);
						Matrix SSII4(3,1);
						for(m=0;m<3;m++){
							SSII1[m]=*(ssii_vis+4*i1+m);
							SSII2[m]=*(ssii_vis+4*i2+m);
							SSII3[m]=*(ssii_vis+4*i3+m);
							SSII4[m]=*(ssii_vis+4*i4+m);
						}
						//calculating user wrt the SV displacement unit vectors
						Matrix UNI1=(SBII-SSII1).univec3();
						Matrix UNI2=(SBII-SSII2).univec3();
						Matrix UNI3=(SBII-SSII3).univec3();
						Matrix UNI4=(SBII-SSII4).univec3();

						//building the GPS 'H' matrix
						Matrix HGPS(4,4);HGPS.ones();					
						for(m=0;m<3;m++){
							HGPS.assign_loc(0,m,UNI1[m]);
							HGPS.assign_loc(1,m,UNI2[m]);
							HGPS.assign_loc(2,m,UNI3[m]);
							HGPS.assign_loc(3,m,UNI4[m]);
						}
						//calculating GDOP 
						Matrix COV(4,4);
						COV=(HGPS*HGPS.trans()).inverse();
						double gdop_local=sqrt(COV.get_loc(0,0)+COV.get_loc(1,1)+COV.get_loc(2,2)+COV.get_loc(3,3));

						//save slot # of quadriga SVs if GDOP has decreased
						if(gdop_local<gdop){
							gdop=gdop_local;
							quad[0]=i1;
							quad[1]=i2;
							quad[2]=i3;
							quad[3]=i4;
						}
					}
				}
			}
		}//end of picking quadriga amongst visible SVs

		//extracting "best" quadriga from visible SVs
		//and storing inertial coordinates of the four SVs and their slot# in ssii_quad[16]
		for(m=0;m<4;m++){
			for(int n=0;n<4;n++){
				*(ssii_quad+4*m+n)=*(ssii_vis+4*quad[m]+n);
			}
		}
		delete[] ssii_vis;

		//calculating inertial velocity of quadriga SVs
		// getting slot# of quadriga
		for(i=0;i<4;i++){
			slot[i]=*(ssii_quad+4*i+3);
			//casting into an int
			islot[i]=(int)slot[i];
		}
		//storing inertial velocities of the four SVs in vsii_quad[12]
		double sin_incl=sin(incl);
		double cos_incl=cos(incl);
		double vel=rsi*wsi;
		for(m=0;m<4;m++){
			int ii=islot[m]-1;  //reminder: slot#=1,2,3...,24
			*(vsii_quad+3*m+0)=vel*(-sin(sv_data[ii][1])*cos(sv_data[ii][0])-cos(sv_data[ii][1])*sin(sv_data[ii][0])*cos_incl);
			*(vsii_quad+3*m+1)=vel*(-sin(sv_data[ii][1])*sin(sv_data[ii][0])+cos(sv_data[ii][1])*cos(sv_data[ii][0])*cos_incl);
			*(vsii_quad+3*m+2)=vel*(cos(sv_data[ii][1])*sin_incl);
		}
	}//end of picking quadriga from 4 or more visible SVs
}
///////////////////////////////////////////////////////////////////////////////
//Initialization of the Space Vehicles (GPS satellites)
//Member function of class 'Hyper'
//Assumptions: all SVs on circular orbits at 55 deg inclination and  separated 
// by equal intervals of nominally 60 deg right ascension;
/*
	Yuma Almanac Week 787 (21 Sep 2014)

***	Right ascensions of orbital planes nearly equally spaced in 60 deg increments
    yet the individual right ascensions were averaged for each plane to get the 
	following mean values

	Plane A  322.4 deg
	Plane B   22.9 deg
	Plane C   83.2 deg
	Plane D  140.1 deg
	Plane E  199.4 deg
	Plane F  262.8 deg
*/
//
// parameter output:
//	*sv_init_data=24x2=48 array: alternating right ascension- rad and argument of latitude - rad
//					(the two-dimensional array is stored in sequential rows in a one-dimensional array)
//					              
//	rsi = central radius of SV orbits - m
//	wsi = angular velocity of SVs - rad/s
//
//140919 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::gps_sv_init(double *sv_init_data,double &rsi,double &wsi,double &incl)
{
	//(24x2) array of SVs: A - F planes 4 satellites each
	//data entry: right ascension (rad), argument of latitude (rad)
	//(the inclination of 55 deg is the same for all planes)
	//Console output refers to slot #. Slot # is the sequential table entry
	
	double sv_data[24][2]={

		5.63,	-1.600, //A-plane, slot #1
		5.63,	 2.115,	//				#2
		5.63,	-2.309, //		        #3
		5.63,	 0.319, //				#4

		0.40,	 1.063, //B-plane, slot #5
		0.40,   -1.342, //				#6
		0.40,    0.543,	//		        #7
		0.40,	 2.874,	//				#8

		1.45,	 1.705, //C-plane, slot #9
		1.45,   -2.841,	//				#10
		1.45,   -2.321,	//		        #11
		1.45,   -0.640,	//				#12

		2.45,	 1.941, //D-plane, slot #13
		2.45,	-0.147,	//				#14
		2.45,	 1.690,	//		        #15
		2.45,	 0.409,	//				#16

		3.48,	-0.571, //E-plane, slot #17
		3.48,	-2.988,	//				#18
		3.48,	 0.858,	//		        #19
		3.48,	 2.705,	//				#20

		4.59,	-0.7180, //F-plane,slot #21
		4.59,	 2.666,	 //				#22
		4.59,	-2.977,	 //		        #23
		4.59,	-0.2090	 //				#24
	};
	//loading SV data into 'sv_init_data' array by pointer arithmetic 
	for(int i=0;i<24;i++){
		for(int k=0;k<2;k++){
			*(sv_init_data+2*i+k)=sv_data[i][k];
		}
	}
	// Orbital radius (same for all SVs) = 26560 km
	rsi=26560000;

	//angluar velocity of SV in rad/sec
	//corresponds to a period of half a sidereal day)
	wsi=sqrt(GM/pow(rsi,3));

	//inclination consatant for all SVs = 55deg = 0.95986rad
	incl=0.95986;
}

