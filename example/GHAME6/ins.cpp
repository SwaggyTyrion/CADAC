///////////////////////////////////////////////////////////////////////////////
//FILE: 'ins.cpp'
//Contains 'ins' module of class 'Hyper'
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of INS module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[300-349]
// 
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_ins()
{
	//Definition and initialization of module-variables
	hyper[300].init("mins","int",0,"D INS mode. =0:ideal INS; =1:with INS errors","ins","data","");
	hyper[301].init("frax",0,"Fractn to mod initial INS err state: XXO=XXO(1+frax)","ins","data","");
	hyper[303].init("VBIIC",0,0,0,"Computed body vel in earth coor - m/s","ins","out","");
	hyper[304].init("SBIIC",0,0,0,"Computed pos of body wrt earth reference point - m","ins","out","");
	hyper[305].init("WBICI",0,0,0,"Computed inertial body rate in inert coord - rad/s","ins","out","");
	hyper[306].init("WBICB",0,0,0,"Computed inertial body rate in body coord - rad/s","ins","out","");
	hyper[307].init("EWALKG",0,0,0,"Random walk - rad/sqrt(sec)","ins","data","");
	hyper[308].init("EUNBG",0,0,0,"Gyro cluster misalignment - rad","ins","data","");
	hyper[309].init("EMISG",gauss(0,1.1e-4),gauss(0,1.1e-4),gauss(0,1.1e-4),"Gyro  misalignmt - rad","ins","data","");
	hyper[310].init("ESCALG",gauss(0,2.e-5),gauss(0,2.e-5),gauss(0,2.e-5),"Gyro scale fctr - parts","ins","data","");
	hyper[311].init("EBIASG",gauss(0,1.e-6),gauss(0,1.e-6),gauss(0,1.e-6),"Gyro bias - rad/s","ins","data","");
	hyper[312].init("EUG",0,0,0,"Gyro spin axis accel sensitivity - rad/s","ins","diag","");
	hyper[313].init("EWG",0,0,0,"Gyro random walk errors - rad/s","ins","diag","");
	hyper[315].init("TBIC",0,0,0,0,0,0,0,0,0,"Comp trans matrix of body wrt earth coor - None","ins","out","");
	hyper[316].init("EWALKA",0,0,0,"Accel random bias - m/s2","ins","data","");
	hyper[317].init("EMISA",gauss(0,1.1e-4),gauss(0,1.1e-4),gauss(0,1.1e-4),"Accel misalignmt - rad","ins","data","");
	hyper[318].init("ESCALA",gauss(0,5.e-4),gauss(0,5.e-4),gauss(0,5.e-4),"Accel scale fctr  - parts","ins","data","");
	hyper[319].init("EBIASA",gauss(0,3.56e-3),gauss(0,3.56e-3),gauss(0,3.56e-3),"Accel bias - m/s2","ins","data","");
	hyper[320].init("ppcx",0,"INS computed roll rate - deg/s","ins","out","");
	hyper[321].init("qqcx",0,"INS computed pitch rate - deg/s","ins","out","");
	hyper[322].init("rrcx",0,"INS computed yaw rate - deg/s","ins","out","");
	hyper[324].init("EWBIB",0,0,0,"Error in angular vel of body wrt earth - rad/s","ins","diag","");
	hyper[325].init("EFSPB",0,0,0,"Error in specific force on body in body coor - N/kg","ins","diag","");
	hyper[326].init("loncx",0,"INS derived longitude - deg","ins","out","");
	hyper[327].init("latcx",0,"INS derived latitude - deg","ins","out","");
	hyper[328].init("altc",0,"INS derived altitude - m","ins","out","");
	hyper[329].init("VBECD",0,0,0,"Geodetic velocity - m/s","ins","out","");
	hyper[330].init("dvbec",0,"Computed body speed wrt earth - m/s","ins","out","");
	hyper[331].init("TDCI",0,0,0,0,0,0,0,0,0,"Comp T.M. of geodetic wrt inertial - None","ins","out","");
	hyper[332].init("thtvdcx",0,"INS computed vertical flight path angle - deg","ins","out","");
	hyper[333].init("psivdcx",0,"INS computed heading angle - deg","ins","out","");	
	hyper[334].init("FSPCB",0,0,0,"Computed specific force on body - N/kg","ins","out","");
	hyper[336].init("alphacx",0,"INS computed angle of attack - deg","ins","diag","");
	hyper[337].init("betacx",0,"INS computed sideslip angle - deg","ins","diag","");
	hyper[338].init("phibdcx",0,"INS computed geodetic Euler roll angle - deg","ins","out","");
	hyper[339].init("thtbdcx",0,"INS computed geodetic Euler pitch angle - deg","ins","out","");
	hyper[340].init("psibdcx",0,"INS computed geodetic Euler yaw angle - deg","ins","out","");
	hyper[341].init("alppcx",0,"INS computed total angle of attack - deg","ins","out","");
	hyper[342].init("phipcx",0,"INS computed aero roll angle - deg","ins","diag","");
	hyper[343].init("RICID",0,0,0,"INS tilt error derivative - rad","ins","state","");
	hyper[344].init("RICI",0,0,0,"INS tilt error - rad","ins","state","plot");
	hyper[345].init("EVBID",0,0,0,"INS vel error derivative - m/s","ins","state","");
	hyper[346].init("EVBI",0,0,0,"INS vel error - m/s","ins","state","");
	hyper[347].init("ESBID",0,0,0,"INS pos error derivative - m","ins","state","");
	hyper[348].init("ESBI",0,0,0,"INS pos error - m","ins","state","plot");
}	

///////////////////////////////////////////////////////////////////////////////
//INS initialization module
//Member function of class 'Hyper'
// Initializes INS error states using the Cholesky method
//
//mins	= 0 ideal INS (no errors)
//		= 1 space stabilized INS
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::init_ins()
{
	//local variables
	double sum(0);
	double APP0[9][9];
	double GAUSS[9];
	double XX0[9];

	//local module-variables
	Matrix ESBI(3,1);
	Matrix EVBI(3,1);
	Matrix RICI(3,1);

	//localizing module-variables
	//input data
	int mins=hyper[300].integer();
	double frax=hyper[301].real();
	//input from other modules
	//-------------------------------------------------------------------------
	//initialization without INS errors (perfect transfer alignment)	
	if(mins==0){
		//do nothing
	}else{
		//Initial covariance matrix  (GPS quality)
		//equipped aircraft. Units: meter, meter/sec, milli-rad.
		double PP0[9][9]={
		  20.701,     0.12317,    0.10541,
		  6.3213E-02, 2.2055E-03, 1.7234E-03,
		  1.0633E-03, 3.4941E-02,-3.5179E-02,
 
		  0.12317,    20.696,    -0.27174,
		  4.8366E-03, 5.9463E-02,-1.3367E-03,
		 -3.4903E-02, 2.6112E-03,-4.2663E-02,
 
		  0.10541,   -0.27174,    114.12,
		  5.6373E-04,-8.3147E-03, 5.4059E-02,
		  1.5496E-02, 7.6463E-02,-3.5302E-03,
 
		  6.3213E-02, 4.8366E-03, 5.6373E-04,
		  1.9106E-03, 8.0945E-05, 1.9810E-06,
		  2.5755E-04, 2.8346E-03,-5.6482E-04,
 
		  2.2055E-03, 5.9463E-02,-8.3147E-03,
		  8.0945E-05, 1.7201E-03,-1.5760E-05,
		 -2.8341E-03, 2.6478E-04,-1.0781E-03,
 
		  1.7234E-03,-1.3367E-03, 5.4059E-02,
		  1.9810E-06,-1.5760E-05, 3.0070E-03,
		  4.1963E-04,-1.3297E-04, 4.1190E-05,
 
		  1.0638E-03,-3.4903E-02, 1.5496E-02,
		  2.5755E-04,-2.8341E-03, 4.1963E-04,
		  5.4490E-02,-1.8695E-03, 8.9868E-04,
 
		  3.4941E-02, 2.6112E-03, 7.6463E-02,
		  2.8346E-03, 2.6478E-04,-1.3297E-04,
		 -1.8695E-03, 5.2819E-02, 1.0990E-02,
 
		 -3.5179E-02,-4.2663E-02,-3.5302E-03,
		 -5.6482E-04,-1.0781E-03, 4.1190E-05,
		  8.9868E-04, 1.0990E-02, 0.1291};

		//forming the Cholesky matrix as square root of PP0 for initialization
		  int i(0);
		for(i=0;i<9;i++){
			for(int j=0;j<9;j++){
				if(j<i){
					sum=0.;
					if(j>0){
						for(int k=0;k<j-1;k++)
							sum=sum+APP0[i][k]*APP0[j][k];
					}//endif
					if(APP0[j][j]==0.){
						APP0[i][j]=0.;
					}
					else{
						APP0[i][j]=(PP0[i][j]-sum)/APP0[j][j];
					}//endif
				}
				else if(j==i){
					sum=0.;
					if(i>0){
						for(int k=0;k<i-1;k++)
							sum=sum+APP0[i][k]*APP0[i][k];
					}//endif
					APP0[i][j]=sqrt(PP0[i][i]-sum);
				}
				else{
					APP0[i][j]=0.;
				}//endif
			}//10
		}//10
		//drawing Gaussian 9x1 vector with unit std deviation
		for(i=0;i<9;i++)
			GAUSS[i]=gauss(0,1);

		//multipying XX0=APP0*GAUSS

		for(i=0;i<9;i++)
		{
			double temp=0;
			for(int j=0;j<9;j++)
				temp=temp+APP0[i][j]*GAUSS[j];
			XX0[i]=temp;
		}
		//forming subvectors for initialization and converting tilt to radians
		ESBI.build_vec3(XX0[0],XX0[1],XX0[2]);
		ESBI*=(1.+frax);
		EVBI.build_vec3(XX0[3],XX0[4],XX0[5]);
		EVBI*=(1.+frax);
		RICI.build_vec3(XX0[6],XX0[7],XX0[8]);
		RICI*=(1.+frax)*.001;

	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//initializations
	hyper[344].gets_vec(RICI);
	hyper[346].gets_vec(EVBI);
	hyper[348].gets_vec(ESBI);
}	
///////////////////////////////////////////////////////////////////////////////
//INS module
//Member function of class 'Hyper'
//Error equations based on Zipfel,p.436, Figure 10.27
// space stabilized INS with GPS and star tracker updates
// when 'mstar'=3 star tracker update received
// when 'mgps'=3 GPS update received
//
//mins	= 0 ideal INS (no errors)
//		= 1 space stabilized INS
//
//030604 Created by Peter H Zipfel
//040130 GPS update inserted, PZi
//040212 Star tracker update inserted, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::ins(double int_step)
{
	//local variables
	double lonc(0),latc(0);
	double phipc(0);
	double psivdc(0),thtvdc(0);
	double psibdc(0),thtbdc(0),phibdc(0);
	double cthtbd(0);
	
	//local module-variables
	Matrix VBECB(3,1);
	Matrix VBECD(3,1);
	Matrix EFSPB(3,1);
	Matrix EWBIB(3,1);
	Matrix TBIC(3,3);
	Matrix TDCI(3,3);
	Matrix FSPCB(3,1);
	Matrix WBICI(3,1);
	Matrix WBICB(3,1);
	double dvbec(0);
	double thtvdcx(0);
	double psivdcx(0);
	double psibdcx(0);
	double thtbdcx(0);
	double phibdcx(0);
	double alphacx(0);
	double betacx(0);
	double loncx(0),latcx(0),altc(0);
	double alppcx(0),phipcx(0);
	double ppcx(0),qqcx(0),rrcx(0);

	//localizing module-variables
	//input data
	int mins=hyper[300].integer();
	Matrix EWALKA=hyper[316].vec();
	//initialization
	Matrix VBIIC=hyper[303].vec();
	Matrix SBIIC=hyper[304].vec();
	//input from other modules
	double time=round6[0].real();
	Matrix GRAVG=round6[62].vec();
	Matrix TBI=round6[121].mat();
	Matrix WBIB=round6[164].vec();
	Matrix WBII=round6[166].vec();
	Matrix SBII=round6[235].vec();
	Matrix VBII=round6[236].vec();
	Matrix FSPB=round6[239].vec();
	int mgps=hyper[700].integer();
	Matrix SXH=hyper[764].vec();
	Matrix VXH=hyper[765].vec();
	int mstar=hyper[800].integer();
	Matrix URIC=hyper[830].vec();
	//state variables
	Matrix RICID=hyper[343].vec();
	Matrix RICI=hyper[344].vec();
	Matrix EVBID=hyper[345].vec();
	Matrix EVBI=hyper[346].vec();
	Matrix ESBID=hyper[347].vec();
	Matrix ESBI=hyper[348].vec();
	//-------------------------------------------------------------------------
	//ideal INS output
	if(mins==0){
		TBIC=TBI;
		FSPCB=FSPB;
		WBICI=WBII;
		WBICB=WBIB;
		SBIIC=SBII;
		VBIIC=VBII;
	}
	else
	{
		//computing INS derived postion of hyper B wrt center of Earth I
		//(needed as 'ins_grav()' argument) 
		SBIIC=ESBI+SBII;

		//calculating attitude errors
		EWBIB=ins_gyro(WBICB, int_step);
		Matrix RICID_NEW=~TBI*EWBIB;
		RICI=integrate(RICID_NEW,RICID,RICI,int_step);
		RICID=RICID_NEW;

		//updating tilt with star tracker
		if(mstar==3){
			RICI=RICI-URIC;
			//returning flag to star tracker that update was completed
			mstar=2;
		}
		//computed transformation matrix
		Matrix UNI(3,3);UNI.identity();
		Matrix TIIC=UNI-RICI.skew_sym();
		TBIC=TBI*TIIC;

		//calculating velocity error
		//accelerometer error (bias,scale factor,misalignment)
		EFSPB=ins_accl();
		//acceleration measurement with random walk effect 
		FSPCB=EWALKA+EFSPB+FSPB;
		//gravitational error
		Matrix EGRAVI=ins_grav(ESBI,SBIIC);
		//integrating velocity error equation
		Matrix TICB=~TBIC;
		Matrix EVBID_NEW=TICB*EFSPB-RICI.skew_sym()*TICB*FSPCB+EGRAVI;
		EVBI=integrate(EVBID_NEW,EVBID,EVBI,int_step);
		EVBID=EVBID_NEW;

		//calculating position error
		Matrix ESBID_NEW=EVBI;
		ESBI=integrate(ESBID_NEW,ESBID,ESBI,int_step);
		ESBID=ESBID_NEW;

		//GPS upate
		if(mgps==3){
			//updating INS navigation output
			SBIIC=SBIIC-SXH;
			VBIIC=VBIIC-VXH;
			//resetting INS error states
			ESBI=ESBI-SXH;
			EVBI=EVBI-VXH;
			//returning flag to GPS that update was completed
			mgps=2;
		}
		//re-computing INS derived postion of hyper B wrt center of Earth I
		SBIIC=ESBI+SBII;
		//computing INS derived velocity of hyper B wrt inertial frame I
		VBIIC=EVBI+VBII;
		//computing INS derived body rates in inertial coord
		WBICI=~TBIC*WBICB;
	}
	//computing geographic velocity in body coordinates from INS
	Matrix VEIC(3,1);
	VEIC[0]=-WEII3*SBIIC[1];
	VEIC[1]=WEII3*SBIIC[0];
	VEIC[2]=0;
	Matrix VBEIC=VBIIC-VEIC;
	VBECB=TBIC*VBEIC;
	dvbec=VBECB.absolute();

	//decomposing computed body rates
	ppcx=WBICB[0]*DEG;
	qqcx=WBICB[1]*DEG;
	rrcx=WBICB[2]*DEG;

	//computing indidence angles from INS
	double alphac=atan2(VBECB[2],VBECB[0]);
	double betac=asin(VBECB[1]/dvbec);
	alphacx=alphac*DEG;
	betacx=betac*DEG;

	//incidence angles in load factor plane (aeroballistic)
	double dum=VBECB[0]/dvbec;
	if(fabs(dum)>1)
		dum=1*sign(dum);
	double alppc=acos(dum);

	if(VBECB[1]==0&&VBECB[2]==0)
		phipc=0.;
	//note: if vbeb2 is <EPS the value if phipc is forced to be 0 or PI
	//		to prevent oscillations
	else if(fabs(VBECB[1])<EPS)
		if(VBECB[2]>0) phipc=0;
		if(VBECB[2]<0) phipc=PI;
	else
		phipc=atan2(VBECB[1],VBECB[2]);
	alppcx=alppc*DEG;
	phipcx=phipc*DEG;

	//getting long,lat,alt from INS
	cad_geo84_in(lonc,latc,altc, SBIIC,time);			  

	//getting T.M. of geodetic wrt inertial coord
	TDCI=cad_tdi84(lonc,latc,altc,time);
	loncx=lonc*DEG;
	latcx=latc*DEG;

	//computing geodetic velocity from INS
	VBECD=TDCI*VBEIC;
	
	//computing flight path angles
	if(VBECD[0]==0&&VBECD[1]==0){
		psivdc=0;
		thtvdc=0;
	}
	else{
		psivdc=atan2(VBECD[1],VBECD[0]);
		thtvdc=atan2(-VBECD[2],sqrt(VBECD[0]*VBECD[0]+VBECD[1]*VBECD[1]));
	}
	psivdcx=psivdc*DEG;
	thtvdcx=thtvdc*DEG;

	//computing Euler angles from INS
	Matrix TBD=TBIC*~TDCI;
	double tbd13=TBD.get_loc(0,2);
	double tbd11=TBD.get_loc(0,0);
	double tbd33=TBD.get_loc(2,2);
	double tbd12=TBD.get_loc(0,1);
	double tbd23=TBD.get_loc(1,2);

	//note: when |tbd13| >= 1., thtbdc = +- pi/2, but cos(thtbdc) is
	//		forced to be a small positive number to prevent division by zero
	if(fabs(tbd13)<1.){
		thtbdc=asin(-tbd13);
		cthtbd=cos(thtbdc);
	}
	else{
		thtbdc=PI/2.*sign(-tbd13);
		cthtbd=EPS;
	}
	//note: to avoid rare errors: #IND (or NaN) of angles psibdc 
	//      and phibdc (acos, asin problem), cpsi and sphi are forced 
	//		to be always <= (1.-EPS)
	double cpsi=tbd11/cthtbd;
	if(fabs(cpsi)>1)
		cpsi=1*sign(cpsi);
	double cphi=tbd33/cthtbd;
	if(fabs(cphi)>1)
		cphi=1*sign(cphi);
	psibdc=acos(cpsi)*sign(tbd12);
	phibdc=acos(cphi)*sign(tbd23);
	psibdcx=DEG*psibdc;
	thtbdcx=DEG*thtbdc;
	phibdcx=DEG*phibdc;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	hyper[343].gets_vec(RICID);
	hyper[344].gets_vec(RICI); 
	hyper[345].gets_vec(EVBID);
	hyper[346].gets_vec(EVBI); 
	hyper[347].gets_vec(ESBID);
	hyper[348].gets_vec(ESBI);
	//output to other modules
	hyper[303].gets_vec(VBIIC);
	hyper[304].gets_vec(SBIIC);
	hyper[305].gets_vec(WBICI);
	hyper[306].gets_vec(WBICB);
	hyper[315].gets_mat(TBIC);
	hyper[320].gets(ppcx);
	hyper[321].gets(qqcx);
	hyper[322].gets(rrcx);
	hyper[326].gets(loncx);
	hyper[327].gets(latcx);
	hyper[328].gets(altc);
	hyper[329].gets_vec(VBECD);
	hyper[330].gets(dvbec);
	hyper[331].gets_mat(TDCI);
	hyper[332].gets(thtvdcx);
	hyper[333].gets(psivdcx);
	hyper[334].gets_vec(FSPCB);
	hyper[338].gets(phibdcx);
	hyper[339].gets(thtbdcx);
	hyper[340].gets(psibdcx);
	hyper[341].gets(alppcx);
	hyper[700].gets(mgps);
	hyper[800].gets(mstar);
	//diagnostics
	hyper[324].gets_vec(EWBIB);	
	hyper[325].gets_vec(EFSPB);
	hyper[336].gets(alphacx);
	hyper[337].gets(betacx);
	hyper[342].gets(phipcx);
}	
///////////////////////////////////////////////////////////////////////////////
//Gyro error model
//Member function of class 'Hyper'
//
// (1) Introduces gyro errors: Random walk
//                             Gyro cluster misalignment
//                             Scale factor error
//                             Bias error
//                             Mass unbalence
// (2) Outputs gyro measurements of body angular velocities 
//
// Return output
//			 EWBIB = Error in angular vel of body wrt earth - rad/s 
// Argument output
//			 WBICB = Computed ang vel of body wrt earth - rad/s
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::ins_gyro(Matrix &WBICB, double int_step)
{
	//local variables
	Matrix EWBIB(3,1);
	Matrix EMISCG(3,1);
	
	//local module-variables
	Matrix EUG(3,1);
	Matrix EWG(3,1);

	//localizing module-variables
	//input data
	Matrix EWALKG=hyper[307].vec();
	Matrix EUNBG=hyper[308].vec();
	Matrix EMISG=hyper[309].vec();
	Matrix ESCALG=hyper[310].vec();
	Matrix EBIASG=hyper[311].vec();
	//input from other modules
	Matrix WBIB=round6[164].vec();
	Matrix FSPB=round6[239].vec();
	//-------------------------------------------------------------------------
	//computing cluster misalignment error
	Matrix EGB=ESCALG.diamat_vec()+EMISG.skew_sym();
	EMISCG=EGB*WBIB;
	Matrix EMSBG=EBIASG+EMISCG;

	//computing gyro spin axis sensitivity (mass unbalance)
	EUG[0]=EUNBG[0]*FSPB[0];
	EUG[1]=EUNBG[1]*FSPB[1];
	EUG[2]=EUNBG[2]*FSPB[2];

	//computing random walk error
	EWG=EWALKG*(1./sqrt(int_step));

	//combining all uncertainties
	EWBIB=EMSBG+EUG+EWG;

	//gyro measured body rates
	WBICB=WBIB+EWBIB;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[312].gets_vec(EUG);
	hyper[313].gets_vec(EWG);

	return EWBIB;
}	
///////////////////////////////////////////////////////////////////////////////
//Accelerometer error model
//Member function of class 'Hyper'
// (1) Introduces accelerometer errors: Bias
//                                      Random bias
//                                      Scale factor error
//                                      Accelerometer cluster misalignment
// (2) Outputs acceleration measurements in body axes
//
//Return output
//			  EFSPB(3x1) = accelerometer cluster measurement errors - m/s^2
//
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::ins_accl()
{
	//localizing module-variables
	//input data
	Matrix EMISA=hyper[317].vec();
	Matrix ESCALA=hyper[318].vec();
	Matrix EBIASA=hyper[319].vec();
	//input from other modules
	Matrix FSPB=round6[239].vec();
	//-------------------------------------------------------------------------
	//computing accelerometer erros without random walk (done in 'ins()')
	Matrix EAB=ESCALA.diamat_vec()+EMISA.skew_sym();
	Matrix EFSPB=EBIASA+EAB*FSPB;
	//-------------------------------------------------------------------------
	return EFSPB;
}	
///////////////////////////////////////////////////////////////////////////////
//Gravitational error model
//Member function of class 'Hyper'
//Calculates first order gravitational errors
// Ref: Zipfel, p.435, Eq.10.94
// 
//Return output
//				EGRAVI(3x1) = Gravitational acceleration error - m/s^2
//Parameter input
//				ESBI(3x1) = Position error - m
//				SBIIC(3x1) = Computed inertial position - m
// 
//030604 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::ins_grav(Matrix ESBI,Matrix SBIIC)
{
	//local variable
	Matrix EGRAVI(3,1);

	//localizing module-variables
	//input from other modules
	double dbi=round6[230].real();
	//-------------------------------------------------------------------------
	double dbic=SBIIC.absolute();
	double ed=dbic-dbi;
	double dum=GM/pow(dbic,3);
	EGRAVI=ESBI*(-dum)-SBIIC*(3*ed*dum/dbic);

	return EGRAVI;
}	
