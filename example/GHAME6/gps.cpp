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
//		= 2 GPS filter extrapolation
//		= 3 GPS update to INS ('ins' module resets mgps=2) 
//
//040105 Created by Peter H Zipfel
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
	hyper[710].init("ucbias_error",0,"User clock bias error - m GAUSS","gps","data","");
	hyper[711].init("ucfreq_error",0,"User clock frequency error - m ","gps","diag","");
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
	hyper[730].init("slotsum",0,"Sum of stored slot numbers of quadriga - ND","gps","save","");
	//GPS filter
	hyper[750].init("uctime_cor",0,"User clock correlation time constant - s","gps","data","");
	hyper[751].init("ppos",0,"Init 1sig pos values of cov matrix - m","gps","data","");
	hyper[752].init("pvel",0,"Init 1sig vel values of cov matrix - m/s","gps","data","");
	hyper[753].init("pclockb",0,"Init 1sig clock bias error of cov matrix - m","gps","data","");
	hyper[754].init("pclockf",0,"Init 1sig clock freq error of cov matrix - m/s","gps","data","");
	hyper[755].init("qpos",0,"1sig pos values of process cov matrix - m","gps","data","");
	hyper[756].init("qvel",0,"1sig vel values of process cov matrix - m/s","gps","data","");
	hyper[757].init("qclockb",0,"1sig clock bias error of process cov matrix - m","gps","data","");
	hyper[758].init("qclockf",0,"1sig clock freq error of process cov matrix - m/s","gps","data","");
	hyper[759].init("rpos",0,"1sig pos value of meas spectral dens matrix - m","gps","data","");
	hyper[760].init("rvel",0,"1sig vel value of meas spectral dens matrix - m/s","gps","data","");
	hyper[761].init("factp",0,"Factor to modifiy initial P-matrix P(1+factp)","gps","data","");
	hyper[762].init("factq",0,"Factor to modifiy the Q-matrix Q(1+factq)","gps","data","");
	hyper[763].init("factr",0,"Factor to modifiy the R-matrix R(1+factr)","gps","data","");
	hyper[764].init("SXH",0,0,0,"Position state (inertial coor) - m","gps","out","plot");
	hyper[765].init("VXH",0,0,0,"Velocity  state (inertial coor) - m/s","gps","out","");
	hyper[766].init("CXH",0,0,0,"clock state (only two values) - m, m/s","gps","save","");
	hyper[767].init("PP1",0,0,0,0,0,0,0,0,0,"Covariance Matrix 1st row - mixed","gps","save","");
	hyper[768].init("PP2",0,0,0,0,0,0,0,0,0,"Covariance Matrix 2nd row - mixed","gps","save","");
	hyper[769].init("PP3",0,0,0,0,0,0,0,0,0,"Covariance Matrix 3rd row - mixed","gps","save","");
	hyper[770].init("PP4",0,0,0,0,0,0,0,0,0,"Covariance Matrix 4th row - mixed","gps","save","");
	hyper[771].init("PP5",0,0,0,0,0,0,0,0,0,"Covariance Matrix 5th row - mixed","gps","save","");
	hyper[772].init("PP6",0,0,0,0,0,0,0,0,0,"Covariance Matrix 6th row - mixed","gps","save","");
	hyper[773].init("PP7",0,0,0,0,0,0,0,0,0,"Covariance Matrix 7th row - mixed","gps","save","");
	hyper[774].init("PP8",0,0,0,0,0,0,0,0,0,"Covariance Matrix 8th row - mixed","gps","save","");
	hyper[775].init("std_pos",0,"Std deviation of position from P matrix - m","gps","diag","plot");
	hyper[776].init("std_vel",0,"Std deviation of velocity from P matrix - m/s","gps","diag","");
	hyper[777].init("std_ucbias",0,"Std deviation of user clock bias from P matrix - m","gps","diag","plot");
}
///////////////////////////////////////////////////////////////////////////////  
//GPS module
//Member function of class 'Hyper'
//
//mgps	= 0 no GPS (default)
//		= 1 GPS initialized (input)
//		= 2 GPS filter extrapolation
//		= 3 GPS update to INS (INS resets mgps=2) 
//
//* GPS measurements 
//  Four pseudo-ranges and four delta-ranges (range rates) are measured from that
//   four-satallite configuration (called quadriga) that has the lowest GDOP.
//  The GPS satellites, called space vehicles (SV), are initialized at their locations
//   given by the Almanac of the 224 week (50th week in 2003)
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
	int n(0);
	//filter
	Matrix PP(8,8);  //recursive, must be saved; separated into 8 PPx(3x3)
	static Matrix QQ(8,8); //constant, same for all objects, -> static ok
	static Matrix RR(8,8);//constant, same for all objects, -> static ok
	static Matrix FF(8,8);//constant, same for all objects, -> static ok
	static Matrix PHI(8,8);//constant, same for all objects, -> static ok
	Matrix XH(8,1);
	Matrix HH(8,8);

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
	double slotsum=hyper[730].real();
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
	/*/assembling saved state vector
	for(int m=0;m<3;m++){
		XH.assign_loc(m,0,SXH.get_loc(m,0));
		XH.assign_loc(m+3,0,VXH.get_loc(m,0));
	}
	XH.assign_loc(6,0,CXH.get_loc(0,0));
	XH.assign_loc(7,0,CXH.get_loc(1,0));*/
	//assembling covariance matrix from saved 3x3 matrices
	Matrix VEC1=PP1.vec9_mat33();
	Matrix VEC2=PP2.vec9_mat33();
	Matrix VEC3=PP3.vec9_mat33();
	Matrix VEC4=PP4.vec9_mat33();
	Matrix VEC5=PP5.vec9_mat33();
	Matrix VEC6=PP6.vec9_mat33();
	Matrix VEC7=PP7.vec9_mat33();
	Matrix VEC8=PP8.vec9_mat33();
	for(n=0;n<8;n++){
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
		int i(0);
		for(i=0;i<3;i++){
			PP.assign_loc(i,i,pow(ppos*(1.+factp),2));
			PP.assign_loc(i+3,i+3,pow(pvel*(1.+factp),2));
		}
		PP.assign_loc(6,6,pow(pclockb*(1.+factp),2));
		PP.assign_loc(7,7,pow(pclockf*(1.+factp),2));
		//dynamic error covariance matrix
		for(i=0;i<3;i++){
			QQ.assign_loc(i,i,pow(qpos*(1.+factq),2));
			QQ.assign_loc(i+3,i+3,pow(qvel*(1.+factq),2));
		}
		QQ.assign_loc(6,6,pow(qclockb*(1.+factq),2));
		QQ.assign_loc(7,7,pow(qclockf*(1.+factq),2));
		//measurement noise covariance matrix
		for(i=0;i<4;i++){
			RR.assign_loc(i,i,pow(rpos*(1.+factr),2));
			RR.assign_loc(i+4,i+4,pow(rvel*(1.+factr),2));
		}
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
		cout<<"QQ = \n";
		QQ.print();
		cout<<"RR = \n";
		RR.print();
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
		for(int i=0;i<4;i++){
			//unpacking i-th SV inertial position
			Matrix SSII(3,1);
			int j(0);
			for(j=0;j<3;j++){
				SSII[j]=*(ssii_quad+4*i+j);
			}
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

			//loading measurement residuals into measurement vector 'ZZ'
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
		KK=PP*~HH*(HH*PP*~HH+RR).inverse();
		//state correction		
		XH=KK*ZZ;
		//covariance correction
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
		cout<<" Kalman gain KK = \n";
		KK.print();
		cout<<" State XH = \n";
		XH.print();
		cout<<"Updated covariance matrix PP = \n";
		PP.print();
		//diagnostic print-out - end*/ 

	//decomposing state vector for output
	for(int m=0;m<3;m++){
		SXH.assign_loc(m,0,XH.get_loc(m,0));
		VXH.assign_loc(m,0,XH.get_loc(m+3,0));
	}
	CXH.assign_loc(0,0,XH.get_loc(6,0));
	CXH.assign_loc(1,0,XH.get_loc(7,0));

		//reset state
		XH.zero(); 
	}
	//-----------------------------------------------------------------------------
	//decomposing covariance matrix for saving as 3x3 matrices
	for(n=0;n<8;n++){
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
	hyper[730].gets(slotsum);	
	//diagnostics
	hyper[704].gets(gdop);
	hyper[711].gets(ucfreq_error);
	hyper[766].gets_vec(CXH);
	hyper[775].gets(std_pos);
	hyper[776].gets(std_vel);
	hyper[777].gets(std_ucbias);
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
	double sv_data[24][2];
	double slot[4]={0,0,0,0}; //SV slot#  of quadriga 
	int islot[4]={0,0,0,0}; //SV slot#  of quadriga stored as integer 

	int quad[4]={0,0,0,0}; //location of quadriga SVs in 'ssii_vis[visible_count]
	gdop=LARGE;

	//unpacking the one-dimensional array of SVs into sv_data[24][2]
	int i(0);
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
		ssii[i][3]=0; //=0; not visible; >0:visible (not int but double!); the number is the SV slot# (1,2,3,...,24)
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
			//user is in the visibility cone of the SV
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
		//selecting quadriga (four SVs) with best GDOP
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
						int m(0);
						for(m=0;m<3;m++){
							SSII1[m]=*(ssii_vis+4*i1+m);
							SSII2[m]=*(ssii_vis+4*i2+m);
							SSII3[m]=*(ssii_vis+4*i3+m);
							SSII4[m]=*(ssii_vis+4*i4+m);
						}
						//calculating user wrt the SV displacement vectors
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

						//save location of quadriga SVs in 'ssii_vis[visible_count]' if GDOP has decreased
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

		//extract quadriga from visible SVs
		//storing inertial coordinates of the four SVs and their slot# in ssii_quad[16]
		int m(0);
		for(m=0;m<4;m++){
			for(int n=0;n<4;n++){
				*(ssii_quad+4*m+n)=*(ssii_vis+4*quad[m]+n);
			}
		}
		delete[] ssii_vis;

		//calculating inertial velocity of quadriga SVs
		// getting slot# of quadriga
		for(int i=0;i<4;i++){
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
			*(vsii_quad+3*m+0)=vel*(-cos(sv_data[ii][0])*sin(sv_data[ii][1])-sin(sv_data[ii][0])*cos(sv_data[ii][1])*cos_incl);
			*(vsii_quad+3*m+1)=vel*(-sin(sv_data[ii][0])*sin(sv_data[ii][1])+cos(sv_data[ii][0])*cos(sv_data[ii][1])*cos_incl);
			*(vsii_quad+3*m+2)=vel*(cos(sv_data[ii][1])*sin_incl);
		}
	}//end of picking quadriga from 4 or more visible SVs
}
///////////////////////////////////////////////////////////////////////////////
//Initialization of the Space Vehicles (GPS satellites)
//Member function of class 'Hyper'
//Assumptions: all SVs on circular orbits at 55 deg inclination and  separated 
// by equal intervals of 60 deg right ascension;
/*
**********************************************
**************** Raw Data ********************
From GPS ALMANAC (http://www.schriever.af.mil/GPS/index.html)
Week 224, Time of Applicability(s):  589824.0000

******** Week  224 almanac for PRN-01 ********
ID:                         01
Health:                     000
Eccentricity:               0.5358695984E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9772440325
Rate of Right Ascen(r/s):  -0.7931758961E-008
SQRT(A)  (m 1/2):           5153.583496
Right Ascen at Week(rad):  -0.2613969176E+000
Argument of Perigee(rad):  -1.718055112
Mean Anom(rad):             0.3143769288E+000
Af0(s):                     0.3194808960E-003
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-02 ********
ID:                         02
Health:                     000
Eccentricity:               0.2312564850E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9318897334
Rate of Right Ascen(r/s):  -0.8000333246E-008
SQRT(A)  (m 1/2):           5153.740234
Right Ascen at Week(rad):   0.1713255804E+001
Argument of Perigee(rad):  -1.734657383
Mean Anom(rad):            -0.8683181390E+000
Af0(s):                    -0.2384185791E-003
Af1(s/s):                  -0.7275957614E-011
week:                        224

******** Week  224 almanac for PRN-03 ********
ID:                         03
Health:                     000
Eccentricity:               0.5226135254E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9292591960
Rate of Right Ascen(r/s):  -0.8183198006E-008
SQRT(A)  (m 1/2):           5153.590332
Right Ascen at Week(rad):   0.2785288165E+001
Argument of Perigee(rad):   0.525609004
Mean Anom(rad):             0.2240241871E+001
Af0(s):                     0.6961822510E-004
Af1(s/s):                   0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-04 ********
ID:                         04
Health:                     000
Eccentricity:               0.6592750549E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9623776015
Rate of Right Ascen(r/s):  -0.7726036106E-008
SQRT(A)  (m 1/2):           5153.699707
Right Ascen at Week(rad):  -0.2356570121E+001
Argument of Perigee(rad):  -0.131623866
Mean Anom(rad):            -0.6394168218E+000
Af0(s):                    -0.2384185791E-004
Af1(s/s):                  -0.7275957614E-011
week:                        224

******** Week  224 almanac for PRN-05 ********
ID:                         05
Health:                     000
Eccentricity:               0.5056381226E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9353112296
Rate of Right Ascen(r/s):  -0.8046049436E-008
SQRT(A)  (m 1/2):           5153.536133
Right Ascen at Week(rad):   0.1737647822E+001
Argument of Perigee(rad):   0.805981819
Mean Anom(rad):            -0.9686811540E+000
Af0(s):                     0.9536743164E-005
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-06 ********
ID:                         06
Health:                     000
Eccentricity:               0.6348609924E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9368092577
Rate of Right Ascen(r/s):  -0.8103194673E-008
SQRT(A)  (m 1/2):           5153.545410
Right Ascen at Week(rad):   0.2837112822E+001
Argument of Perigee(rad):  -2.034382848
Mean Anom(rad):             0.2146048859E+000
Af0(s):                    -0.1907348633E-005
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-07 ********
ID:                         07
Health:                     000
Eccentricity:               0.1262569427E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9384570886
Rate of Right Ascen(r/s):  -0.8080336578E-008
SQRT(A)  (m 1/2):           5153.636719
Right Ascen at Week(rad):   0.2808872745E+001
Argument of Perigee(rad):  -1.854389901
Mean Anom(rad):             0.2362527404E+001
Af0(s):                     0.6437301636E-003
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-08 ********
ID:                         08
Health:                     000
Eccentricity:               0.9044170380E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9635280871
Rate of Right Ascen(r/s):  -0.7691748964E-008
SQRT(A)  (m 1/2):           5153.645508
Right Ascen at Week(rad):   0.7949215026E+000
Argument of Perigee(rad):   2.354673992
Mean Anom(rad):             0.3739744793E+000
Af0(s):                     0.3957748413E-003
Af1(s/s):                  -0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-09 ********
ID:                         09
Health:                     000
Eccentricity:               0.1513195038E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9503214713
Rate of Right Ascen(r/s):  -0.7828897534E-008
SQRT(A)  (m 1/2):           5153.688965
Right Ascen at Week(rad):   0.7286667152E+000
Argument of Perigee(rad):   1.011534870
Mean Anom(rad):             0.1918262450E-001
Af0(s):                    -0.2098083496E-004
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-10 ********
ID:                         10
Health:                     000
Eccentricity:               0.5705356598E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9805696549
Rate of Right Ascen(r/s):  -0.7851755629E-008
SQRT(A)  (m 1/2):           5153.576660
Right Ascen at Week(rad):  -0.1321933036E+001
Argument of Perigee(rad):   0.223329027
Mean Anom(rad):            -0.2757309119E+001
Af0(s):                     0.3623962402E-004
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-11 ********
ID:                         11
Health:                     000
Eccentricity:               0.2485752106E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9096350277
Rate of Right Ascen(r/s):  -0.8251772291E-008
SQRT(A)  (m 1/2):           5153.636230
Right Ascen at Week(rad):  -0.2475294466E+001
Argument of Perigee(rad):  -0.080300673
Mean Anom(rad):             0.1021105771E+001
Af0(s):                     0.8296966553E-004
Af1(s/s):                   0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-13 ********
ID:                         13
Health:                     000
Eccentricity:               0.1942634583E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9814445034
Rate of Right Ascen(r/s):  -0.7897471819E-008
SQRT(A)  (m 1/2):           5153.705566
Right Ascen at Week(rad):  -0.2793170789E+000
Argument of Perigee(rad):   0.688518064
Mean Anom(rad):            -0.2585818231E+001
Af0(s):                    -0.3147125244E-004
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-14 ********
ID:                         14
Health:                     000
Eccentricity:               0.1491546631E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9759257678
Rate of Right Ascen(r/s):  -0.7943188009E-008
SQRT(A)  (m 1/2):           5153.711914
Right Ascen at Week(rad):  -0.2877015422E+000
Argument of Perigee(rad):  -1.384651728
Mean Anom(rad):             0.1784403152E+001
Af0(s):                    -0.1811981201E-004
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-15 ********
ID:                         15
Health:                     000
Eccentricity:               0.8505344391E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9693044835
Rate of Right Ascen(r/s):  -0.7668890869E-008
SQRT(A)  (m 1/2):           5153.697754
Right Ascen at Week(rad):  -0.2305287750E+001
Argument of Perigee(rad):   2.166080866
Mean Anom(rad):             0.2620043680E+000
Af0(s):                     0.1983642578E-003
Af1(s/s):                   0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-16 ********
ID:                         16
Health:                     000
Eccentricity:               0.1993179321E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9606219126
Rate of Right Ascen(r/s):  -0.7771752296E-008
SQRT(A)  (m 1/2):           5153.670410
Right Ascen at Week(rad):   0.1805347832E+001
Argument of Perigee(rad):  -1.567895021
Mean Anom(rad):            -0.6892715719E+000
Af0(s):                     0.1525878906E-004
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-17 ********
ID:                         17
Health:                     000
Eccentricity:               0.1596212387E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9713298175
Rate of Right Ascen(r/s):  -0.7657461821E-008
SQRT(A)  (m 1/2):           5153.022461
Right Ascen at Week(rad):  -0.2263880006E+001
Argument of Perigee(rad):  -2.833899177
Mean Anom(rad):             0.8404259791E+000
Af0(s):                     0.6389617920E-004
Af1(s/s):                   0.2182787284E-010
week:                        224

******** Week  224 almanac for PRN-18 ********
ID:                         18
Health:                     000
Eccentricity:               0.4451274872E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9641093220
Rate of Right Ascen(r/s):  -0.8034620388E-008
SQRT(A)  (m 1/2):           5153.582031
Right Ascen at Week(rad):  -0.1286785177E+001
Argument of Perigee(rad):  -2.946246043
Mean Anom(rad):            -0.1230849182E+001
Af0(s):                     0.0000000000E+000
Af1(s/s):                  -0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-20 ********
ID:                         20
Health:                     000
Eccentricity:               0.1921653748E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9642411485
Rate of Right Ascen(r/s):  -0.8023191341E-008
SQRT(A)  (m 1/2):           5153.710449
Right Ascen at Week(rad):  -0.1339053999E+001
Argument of Perigee(rad):   1.666497478
Mean Anom(rad):            -0.1954849914E+001
Af0(s):                    -0.1993179321E-003
Af1(s/s):                   0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-21 ********
ID:                         21
Health:                     000
Eccentricity:               0.8091449738E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9559420728
Rate of Right Ascen(r/s):  -0.7783181344E-008
SQRT(A)  (m 1/2):           5153.670410
Right Ascen at Week(rad):  -0.2330863584E+001
Argument of Perigee(rad):   2.939518398
Mean Anom(rad):             0.2280721587E+000
Af0(s):                     0.6866455078E-004
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-23 ********
ID:                         23
Health:                     000
Eccentricity:               0.1659488678E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9842667883
Rate of Right Ascen(r/s):  -0.7817468486E-008
SQRT(A)  (m 1/2):           5153.807617
Right Ascen at Week(rad):  -0.1270776874E+001
Argument of Perigee(rad):  -1.631301682
Mean Anom(rad):             0.2958480438E+001
Af0(s):                    -0.1134872437E-003
Af1(s/s):                  -0.4001776688E-010
week:                        224

******** Week  224 almanac for PRN-24 ********
ID:                         24
Health:                     000
Eccentricity:               0.9591102600E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9726181217
Rate of Right Ascen(r/s):  -0.7634603726E-008
SQRT(A)  (m 1/2):           5153.493652
Right Ascen at Week(rad):  -0.2331571777E+001
Argument of Perigee(rad):  -1.427159025
Mean Anom(rad):            -0.8662347564E-003
Af0(s):                     0.8201599121E-004
Af1(s/s):                   0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-25 ********
ID:                         25
Health:                     000
Eccentricity:               0.1083230972E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9433885972
Rate of Right Ascen(r/s):  -0.7931758961E-008
SQRT(A)  (m 1/2):           5153.535645
Right Ascen at Week(rad):   0.6800995203E+000
Argument of Perigee(rad):  -1.648405418
Mean Anom(rad):             0.7372736358E+000
Af0(s):                     0.5722045898E-004
Af1(s/s):                   0.0000000000E+000
week:                        224

******** Week  224 almanac for PRN-26 ********
ID:                         26
Health:                     000
Eccentricity:               0.1483392715E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9798446093
Rate of Right Ascen(r/s):  -0.7920329914E-008
SQRT(A)  (m 1/2):           5153.576660
Right Ascen at Week(rad):  -0.2762577310E+000
Argument of Perigee(rad):   0.539280384
Mean Anom(rad):             0.1803642327E+001
Af0(s):                     0.4796981812E-003
Af1(s/s):                  -0.7275957614E-011
week:                        224

******** Week  224 almanac for PRN-27 ********
ID:                         27
Health:                     000
Eccentricity:               0.1768207550E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9479006578
Rate of Right Ascen(r/s):  -0.7817468486E-008
SQRT(A)  (m 1/2):           5153.553223
Right Ascen at Week(rad):   0.7076613650E+000
Argument of Perigee(rad):  -2.232846855
Mean Anom(rad):            -0.7527418995E+000
Af0(s):                     0.8478164673E-003
Af1(s/s):                   0.1818989404E-010
week:                        224

******** Week  224 almanac for PRN-28 ********
ID:                         28
Health:                     000
Eccentricity:               0.8107662201E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9585546338
Rate of Right Ascen(r/s):  -0.7794610391E-008
SQRT(A)  (m 1/2):           5153.638184
Right Ascen at Week(rad):   0.1816219022E+001
Argument of Perigee(rad):  -2.431747913
Mean Anom(rad):            -0.2209888826E+001
Af0(s):                     0.8583068848E-005
Af1(s/s):                   0.3637978807E-011
week:                        224

******** Week  224 almanac for PRN-29 ********
ID:                         29
Health:                     000
Eccentricity:               0.8179187775E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9767946241
Rate of Right Ascen(r/s):  -0.7943188009E-008
SQRT(A)  (m 1/2):           5153.644531
Right Ascen at Week(rad):  -0.3083211502E+000
Argument of Perigee(rad):  -1.510251648
Mean Anom(rad):            -0.2128845130E+001
Af0(s):                     0.1878738403E-003
Af1(s/s):                   0.1091393642E-010
week:                        224

******** Week  224 almanac for PRN-30 ********
ID:                         30
Health:                     000
Eccentricity:               0.7350444794E-002
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9422560879
Rate of Right Ascen(r/s):  -0.7988904198E-008
SQRT(A)  (m 1/2):           5153.668457
Right Ascen at Week(rad):   0.1777820067E+001
Argument of Perigee(rad):   1.283633326
Mean Anom(rad):            -0.2010720745E+001
Af0(s):                     0.6875991821E-003
Af1(s/s):                   0.2182787284E-010
week:                        224

******** Week  224 almanac for PRN-31 ********
ID:                         31
Health:                     000
Eccentricity:               0.1201105118E-001
Time of Applicability(s):  589824.0000
Orbital Inclination(rad):   0.9379118064
Rate of Right Ascen(r/s):  -0.8103194673E-008
SQRT(A)  (m 1/2):           5153.588867
Right Ascen at Week(rad):   0.2807869066E+001
Argument of Perigee(rad):   0.952014093
Mean Anom(rad):             0.1222410415E+001
Af0(s):                     0.4339218140E-003
Af1(s/s):                   0.0000000000E+000
week:                        224
*******************************************************
*************** Reduced Data **************************
************* For Cicular Orbits **********************
*******************************************************
 True Anomaly = Mean Anomaly
 Argument of Latitude(rad) (u|0) = Argument of Perigee + Mean Anomaly
 
******** Week  224 almanac for PRN-01 Slot F4 ********
Orbital Inclination(rad):   0.9772440325
Right Ascen at Week(rad):  -0.2613969176E+000
Argument of Perigee(rad):  -1.718055112
Mean Anom(rad):             0.3143769288E+000
Argument of latitude(rad)  -1.403678183		(-80.42deg)

******** Week  224 almanac for PRN-02 Slot B5 ********<<<
Orbital Inclination(rad):   0.9318897334
Right Ascen at Week(rad):   0.1713255804E+001
Argument of Perigee(rad):  -1.734657383
Mean Anom(rad):            -0.8683181390E+000
Argument of latitude(rad)  -2.602976077		(-149.14deg

******** Week  224 almanac for PRN-03 Slot C2 ********
Orbital Inclination(rad):   0.9292591960
Right Ascen at Week(rad):   0.2785288165E+001
Argument of Perigee(rad):   0.525609004
Mean Anom(rad):             0.2240241871E+001
Argument of latitude(rad)	2.765850875		(158.43deg)

******** Week  224 almanac for PRN-04 Slot D4 ********
Orbital Inclination(rad):   0.9623776015
Right Ascen at Week(rad):  -0.2356570121E+001 (-135.0deg)
Argument of Perigee(rad):  -0.131623866
Mean Anom(rad):            -0.6394168218E+000
Argument of latitude(rad)  -0.771040687		(-44.18deg) 

******** Week  224 almanac for PRN-05 Slot B4 ********
Orbital Inclination(rad):   0.9353112296
Right Ascen at Week(rad):   0.1737647822E+001
Argument of Perigee(rad):   0.805981819
Mean Anom(rad):            -0.9686811540E+000
Argument of latitude(rad)  -0.162699335		(-9.32deg) 

******** Week  224 almanac for PRN-06 Slot C1 ********
Orbital Inclination(rad):   0.9368092577
Right Ascen at Week(rad):   0.2837112822E+001
Argument of Perigee(rad):  -2.034382848
Mean Anom(rad):             0.2146048859E+000
Argument of latitude(rad)	-1.819777963	(-104.27deg)

******** Week  224 almanac for PRN-07 Slot C4 ********
Orbital Inclination(rad):   0.9384570886
Right Ascen at Week(rad):   0.2808872745E+001
Argument of Perigee(rad):  -1.854389901
Mean Anom(rad):             0.2362527404E+001
Argument of latitude(rad)	0.508137503		(29.11deg)

******** Week  224 almanac for PRN-08 Slot A3 ********
Orbital Inclination(rad):   0.9635280871
Right Ascen at Week(rad):   0.7949215026E+000
Argument of Perigee(rad):   2.354673992
Mean Anom(rad):             0.3739744793E+000
Argument of latitude(rad)	2.728648471		(156.34deg)

******** Week  224 almanac for PRN-09 Slot A1 ********
Orbital Inclination(rad):   0.9503214713
Right Ascen at Week(rad):   0.7286667152E+000
Argument of Perigee(rad):   1.011534870
Mean Anom(rad):             0.1918262450E-001
Argument of latitude(rad)	1.030717495		(59.06deg)

******** Week  224 almanac for PRN-10 Slot E3 ********
Orbital Inclination(rad):   0.9805696549
Right Ascen at Week(rad):  -0.1321933036E+001
Argument of Perigee(rad):   0.223329027
Mean Anom(rad):            -0.2757309119E+001
Argument of latitude(rad)  -2.533980092		(-145.19deg)

******** Week  224 almanac for PRN-11 Slot D2 ********
Orbital Inclination(rad):   0.9096350277
Right Ascen at Week(rad):  -0.2475294466E+001 (-141.8 deg)
Argument of Perigee(rad):  -0.080300673
Mean Anom(rad):             0.1021105771E+001
Argument of latitude(rad)	0.940805098		(53.91deg)

******** Week  224 almanac for PRN-13 Slot F3 ********
Orbital Inclination(rad):   0.9814445034
Right Ascen at Week(rad):  -0.2793170789E+000
Argument of Perigee(rad):   0.688518064
Mean Anom(rad):            -0.2585818231E+001
Argument of latitude(rad)  -1.897300167		(-108.71deg) 

******** Week  224 almanac for PRN-14 Slot F1 ********
Orbital Inclination(rad):   0.9759257678
Right Ascen at Week(rad):  -0.2877015422E+000
Argument of Perigee(rad):  -1.384651728
Mean Anom(rad):             0.1784403152E+001
Argument of latitude(rad)	0.399751424		(22.91deg)

******** Week  224 almanac for PRN-15 Slot D5 ********<<<
Orbital Inclination(rad):   0.9693044835
Right Ascen at Week(rad):  -0.2305287750E+001
Argument of Perigee(rad):   2.166080866
Mean Anom(rad):             0.2620043680E+000
Argument of latitude(rad)	2.428085234		(139.12deg)

******** Week  224 almanac for PRN-16 Slot B1 ********
Orbital Inclination(rad):   0.9606219126
Right Ascen at Week(rad):   0.1805347832E+001
Argument of Perigee(rad):  -1.567895021
Mean Anom(rad):            -0.6892715719E+000
Argument of latitude(rad)  -2.257166593		(-129.34deg) 

******** Week  224 almanac for PRN-17 Slot D6 ********<<<
Orbital Inclination(rad):   0.9713298175
Rate of Right Ascen(r/s):  -0.7657461821E-008
Argument of Perigee(rad):  -2.833899177
Mean Anom(rad):             0.8404259791E+000
Argument of latitude(rad)  -1.9934473198	(-114.22deg) 

******** Week  224 almanac for PRN-18 Slot E4 ********
Orbital Inclination(rad):   0.9641093220
Right Ascen at Week(rad):  -0.1286785177E+001
Argument of Perigee(rad):  -2.946246043
Mean Anom(rad):            -0.1230849182E+001
Argument of latitude(rad)  -4.177095225	(-239.34=120.66deg) 

******** Week  224 almanac for PRN-19 missing ********

******** Week  224 almanac for PRN-20 Slot E1 ********
Orbital Inclination(rad):   0.9642411485
Right Ascen at Week(rad):  -0.1339053999E+001
Argument of Perigee(rad):   1.666497478
Mean Anom(rad):            -0.1954849914E+001
Argument of latitude(rad)  -0.288352436		(-16.52deg) 

******** Week  224 almanac for PRN-21 Slot D3 ********
Orbital Inclination(rad):   0.9559420728
Right Ascen at Week(rad):  -0.2330863584E+001 (-133.5 deg)
Argument of Perigee(rad):   2.939518398
Mean Anom(rad):             0.2280721587E+000
Argument of latitude(rad)	3.167590557		(181.49deg)

******** Week  224 almanac for PRN-22 missing ********

******** Week  224 almanac for PRN-23 Slot E5 ********<<<
Orbital Inclination(rad):   0.9842667883
Right Ascen at Week(rad):  -0.1270776874E+001
Argument of Perigee(rad):  -1.631301682
Mean Anom(rad):             0.2958480438E+001
Argument of latitude(rad)	1.327178756		(76.04deg)

******** Week  224 almanac for PRN-24 Slot D1 ********
Orbital Inclination(rad):   0.9726181217
Right Ascen at Week(rad):  -0.2331571777E+001 (-133.6 deg)
Argument of Perigee(rad):  -1.427159025
Mean Anom(rad):            -0.8662347564E-003
Argument of latitude(rad)  -1.428025259		(-81.82deg) 

******** Week  224 almanac for PRN-25 Slot A2 ********
Orbital Inclination(rad):   0.9433885972
Right Ascen at Week(rad):   0.6800995203E+000
Argument of Perigee(rad):  -1.648405418
Mean Anom(rad):             0.7372736358E+000
Argument of latitude(rad)  -0.911131782		(-52.21deg) 

******** Week  224 almanac for PRN-26 Slot F2 ********
Orbital Inclination(rad):   0.9798446093
Right Ascen at Week(rad):  -0.2762577310E+000
Argument of Perigee(rad):   0.539280384
Mean Anom(rad):             0.1803642327E+001
Argument of latitude(rad)	2.342922711		(134.24deg)

******** Week  224 almanac for PRN-27 Slot A4 ********
Orbital Inclination(rad):   0.9479006578
Right Ascen at Week(rad):   0.7076613650E+000
Argument of Perigee(rad):  -2.232846855
Mean Anom(rad):            -0.7527418995E+000
Argument of latitude(rad)  -2.985588755		(-171.07deg) 

******** Week  224 almanac for PRN-28 Slot B3 ********
Orbital Inclination(rad):   0.9585546338
Right Ascen at Week(rad):   0.1816219022E+001
Argument of Perigee(rad):  -2.431747913
Mean Anom(rad):            -0.2209888826E+001
Argument of latitude(rad)  -4.641636739		(-265.93=94.07deg) 

******** Week  224 almanac for PRN-29 Slot F5 ********<<<
Orbital Inclination(rad):   0.9767946241
Right Ascen at Week(rad):  -0.3083211502E+000
Argument of Perigee(rad):  -1.510251648
Mean Anom(rad):            -0.2128845130E+001
Argument of latitude(rad)  -3.639096778		(-208.52=151.48deg)

******** Week  224 almanac for PRN-30 Slot B2 ********
Orbital Inclination(rad):   0.9422560879
Right Ascen at Week(rad):   0.1777820067E+001
Argument of Perigee(rad):   1.283633326
Mean Anom(rad):            -0.2010720745E+001
Argument of latitude(rad)  -0.727087419		(-41.66deg)

******** Week  224 almanac for PRN-31 Slot C3 ********
Orbital Inclination(rad):   0.9379118064
Right Ascen at Week(rad):   0.2807869066E+001
Argument of Perigee(rad):   0.952014093
Mean Anom(rad):             0.1222410415E+001
Argument of latitude(rad)	2.174424508		(124.59deg)
*************************************************************

*** Averaged Right ascensions of orbital planes A, B, C, D, E, F
Orbital Plane D -2.360 rad -135.2  deg diff = 63.9 deg (wrt C)
Orbital Plane E -1.305 rad  -74.77 deg diff = 60.4 deg (wrt D)
Orbital Plane F -0.276 rad  -15.81 deg diff = 59.0 deg (wrt E)
Orbital Plane A  0.727 rad	 41.65 deg diff = 57.5 deg (wrt F)
Orbital Plane B  1.784 rad	102.2  deg diff = 60.5 deg (wrt A)
Orbital Plane C  2.809 rad  160.9  deg diff = 58.7 deg (wrt B)

***	Right ascensions of orbital planes equally spaced by 60 deg
Plane D -130 deg
Plane E  -70 deg
Plane F  -10 deg
Plane A  50 deg
Plane B 110 deg
Plane C 170 deg

>>> note, large deviation from actual orbits, but of the same order as the 
	spread of the individual right ascensions within one plane
*/
//
// parameter output:
//	*sv_init_data=24x2=48 array: alternating right ascension- rad and argument of latitude - rad
//					(the two-dimensional array is stored in sequential rows in a one-dimensional array)
//					              
//	rsi = central radius of SV orbits - m
//	wsi = angular velocity of SVs - rad/s
//
//040106 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::gps_sv_init(double *sv_init_data,double &rsi,double &wsi,double &incl)
{
	//(24x2) array of SVs: A - F planes 4 satellites each
	//data entry: right ascension (rad), argument of latitude (rad)
	// row# = slot# of SV in constellation
	//(inclination is the same for all planes) 

	double sv_data[24][2]={
		
		0.9947,  1.0371, //A-plane
		0.9947,  2.7284,
		0.9947, -2.9855,
		0.9947, -0.9112,

		1.9197,	 1.6417, //B-plane
		1.9197,	-2.2572,
		1.9197,	-0.7270,
		1.9197,	-0.1626,

		2.9668,	 0.5080, //C-plane
		2.9668,	 2.1743,
		2.9668,	 2.7649,
		2.9668,	-1.8197,

		4.0140,	 0.9408, //D-plane
		4.0140,	 2.4279,
		4.0140,	-3.1153,
		4.0140,	-0.7710,

		5.0611,	 1.3270, //E-plane
		5.0611,	 2.1057,
		5.0611,	-2.5338,
		5.0611,	-0.2883,

		6.1082,	 0.3998, //F-plane
		6.1082,	 2.3427,
		6.1082,	-1.8972,
		6.1082,	-1.4035
	};
	//loading SV data into 'sv_init_data' array by pointer arithmetic 
	for (int i=0;i<24;i++){
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

