///////////////////////////////////////////////////////////////////////////////
//FILE: 'ins.cpp'
//Contains 'ins' module of class 'Missile'
//
//020513 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of INS module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[300-349]
//
//020513 Created by Peter H Zipfel
//180212 Providing three INS specification models, PZi
///////////////////////////////////////////////////////////////////////////////

void Missile::def_ins()
{
	//Definition and initialization of module-variables
	missile[300].init("mins","int",0,"INS mode =0:ideal INS; =1:ASpec; =2:BSpec; =3:GSpec","ins","data","");
	missile[301].init("frax",0,"Fractn to modify init INS err state: XXO=XXO(1+frax)","ins","data","");
	missile[302].init("hbem",0,"Computed body alt above SL - m","ins","out","");
	missile[303].init("VBELC",0,0,0,"Computed body vel in earth coor - m/s","ins","out","");
	missile[304].init("SBELC",0,0,0,"Computed pos of body wrt earth reference point - m","ins","out","");
	missile[306].init("WBECB",0,0,0,"Computed ang vel of body wrt earth - rad/s","ins","out","");
	missile[312].init("biasal",0,"Bias of altimeter measurement - m","ins","data","");
	missile[313].init("randal",0,"Noise of altimeter measurement - m","ins","data","");
	missile[314].init("ehbe",0,"Altimeter error - m","ins","out","");
	missile[315].init("TBLC",0,0,0,0,0,0,0,0,0,"Comp trans matrix of body wrt earth coor - None","ins","out","");
	missile[322].init("EUG",0,0,0,"Gyro spin axis accel sensitivity - rad/s","ins","dia","");
	missile[323].init("EWG",0,0,0,"Gyro random walk errors - rad/s","ins","dia","");
	missile[324].init("EWBEB",0,0,0,"Error in angular vel of body wrt earth - rad/s","ins","dia","");
	missile[325].init("EFSPB",0,0,0,"Error in specific force on body in body coor - N/kg","ins","dia","");
	missile[327].init("tanlat",0,"Tangent of latitude angle of body at release - ND","ins","init","");
	missile[328].init("thtblc",0,"Computed pitch angle - rad","ins","out","");
	missile[329].init("thtblcx",0,"Computed pitch angle - deg","ins","out","");
	missile[330].init("dvbec",0,"Computed body speed wrt earth - m/s","ins","out","");
	missile[331].init("thtvlc",0,"Computed vertical flight path angle - rad","ins","out","");
	missile[332].init("thtvlcx",0,"Computed vertical flight path angle - deg","ins","out","plot");
	missile[333].init("psivlcx",0,"Computed heading angle - deg","ins","out","plot");	
	missile[334].init("FSPCB",0,0,0,"Computed specific force on body - N/kg","ins","out","");
	missile[335].init("phiblcx",0,"Computed Euler roll angle - deg","ins","out","plot");
	missile[343].init("RECED",0,0,0,"INS tilt error derivative - rad","ins","state","");
	missile[344].init("RECE",0,0,0,"INS tilt error - rad","ins","state","");
	missile[345].init("EVBED",0,0,0,"INS vel error derivative - m/s","ins","state","");
	missile[346].init("EVBE",0,0,0,"INS vel error - m/s","ins","state","");
	missile[347].init("ESTTCD",0,0,0,"INS pos error derivative - m","ins","state","");
	missile[348].init("ESTTC",0,0,0,"INS pos error - m","ins","state","");	
}	
///////////////////////////////////////////////////////////////////////////////
//INS initialization module
//Member function of class 'Missile'
// Initializes INS error states using the Cholesky method
// 
// mins = 0 ideal INS
//        1 ASpec ring laser gyros
//		  2 BSpec new technology, high accuracy
//		  3 GSpec MEMS technology 
// 
//020513 Created by Peter Zipfel
//080604 Reformulated transfer alignment initialization using 'cholesky()' function, PZi
//180212 Providing three INS specification models, PZi
///////////////////////////////////////////////////////////////////////////////

void Missile::init_ins()
{
	//local module-variables
	Matrix ESTTC(3,1);
	Matrix EVBE(3,1);
	Matrix RECE(3,1);
	Matrix SBELC(3,1);
	Matrix VBELC(3,1);

	//localizing module-variables
	//input data
	int mins=missile[300].integer();
	double frax=missile[301].real();
	//input from other modules
	Matrix SBEL=flat6[219].vec();
	Matrix VBEL=flat6[233].vec();
	//-------------------------------------------------------------------------
	//with INS errors
		if(mins==1){
		//alpha-specs laser ring gyros low accuracy
		missile[308].init("EUNBG",0,0,0,"Gyro unbalance - (rad/s)/m/s^2)","ins","data","");
		missile[309].init("EMISG",gauss(0,1.1e-4),gauss(0,1.1e-4),gauss(0,1.1e-4),"Gyro cluster misalignmt vector - rad","ins","data","");
		missile[310].init("ESCALG",gauss(0,2.5e-5),gauss(0,2.5e-5),gauss(0,2.5e-5),"Gyro scale factor error - parts","ins","data","");
		missile[311].init("EBIASG",gauss(0,3.2e-6),gauss(0,3.2e-6),gauss(0,3.2e-6),"Gyro bias - rad/s","ins","data","");
		missile[316].init("EWALKA",gauss(0,8.35e-4),gauss(0,8.35e-4),gauss(0,8.35e-4),"Accel random walk - (m/s)/sqrt(s)","ins","data","");
		missile[317].init("EMISA",gauss(0,1.1e-4),gauss(0,1.1e-4),gauss(0,1.1e-4),"Accel cluster misalignmt - rad","ins","data","");
		missile[318].init("ESCALA",gauss(0,5e-4),gauss(0,5e-4),gauss(0,5e-4),"Accel scale factor error - parts","ins","data","");
		missile[319].init("EBIASA",gauss(0,3.56e-3),gauss(0,3.56e-3),gauss(0,3.56e-3),"Accel bias - m/s2","ins","data","");
	}else if(mins==2){
		//beta-specs high accuracy 
		missile[308].init("EUNBG",0,0,0,"Gyro unbalance - (rad/s)/m/s^2)","ins","data","");
		missile[309].init("EMISG",gauss(0,10e-5),gauss(0,10e-5),gauss(0,10e-5),"Gyro cluster misalignmt vector - rad","ins","data","");
		missile[310].init("ESCALG",gauss(0,1.5e-5),gauss(0,1.5e-5),gauss(0,1.5e-5),"Gyro scale factor error - parts","ins","data","");
		missile[311].init("EBIASG",gauss(0,1.5e-6),gauss(0,1.5e-6),gauss(0,1.5e-6),"Gyro bias - rad/s","ins","data","");
		missile[316].init("EWALKA",gauss(0,4.1e-5),gauss(0,4.1e-5),gauss(0,4.1e-5),"Accel random walk - (m/s)/sqrt(s)","ins","data","");
		missile[317].init("EMISA",gauss(0,0.54e-4),gauss(0,0.54e-4),gauss(0,0.54e-4),"Accel cluster misalignmt - rad","ins","data","");
		missile[318].init("ESCALA",gauss(0,2e-6),gauss(0,2e-6),gauss(0,2e-6),"Accel scale factor error - parts","ins","data","");
		missile[319].init("EBIASA",gauss(0,1.5e-3),gauss(0,1.5e-3),gauss(0,1.5e-3),"Accel bias - m/s2","ins","data","");
	}else if(mins==3){
		//gamma-specs MEMS low accuracy 
		missile[308].init("EUNBG",gauss(0,4.83e-7),gauss(0,4.83e-7),gauss(0,4.83e-7),"Gyro unbalance - (rad/s)/m/s^2)","ins","data","");
		missile[309].init("EMISG",gauss(0,50e-6),gauss(0,50e-6),gauss(0,50e-6),"Gyro cluster misalignmt vector - rad","ins","data","");
		missile[310].init("ESCALG",gauss(0,15e-5),gauss(0,15e-5),gauss(0,15e-5),"Gyro scale factor error - parts","ins","data","");
		missile[311].init("EBIASG",gauss(0,4.83e-6),gauss(0,4.83e-6),gauss(0,4.83e-6),"Gyro bias - rad/s","ins","data","");
		missile[316].init("EWALKA",gauss(0,5.08e-5),gauss(0,5.08e-5),gauss(0,5.08e-5),"Accel random walk - (m/s)/sqrt(s)","ins","data","");
		missile[317].init("EMISA",gauss(0,4.85e-4),gauss(0,4.85e-4),gauss(0,4.85e-4),"Accel cluster misalignmt - rad","ins","data","");
		missile[318].init("ESCALA",gauss(0,3e-6),gauss(0,3e-6),gauss(0,3e-6),"Accel scale factor error - parts","ins","data","");
		missile[319].init("EBIASA",gauss(0,9.81e-3),gauss(0,9.81e-3),gauss(0,9.81e-3),"Accel bias - m/s2","ins","data","");
	}
	//**Initialization (transfer alignment)
	//initialization without INS errors (perfect transfer alignment)	
	if(mins==0){
		SBELC=SBEL;
		VBELC=VBEL;
	}
	//initial covariance matrix after transfer alignment from a GPS
	//equipped aircraft; units: meter, meter/sec, milli-rad.
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
	  8.9868E-04, 1.0990E-02, 0.1291
	};
	//copying PP0 onto Matrix PP_INIT	
	Matrix PP_INIT(9,9);
	for(int p=0; p<9;p++){
		for(int q=0; q<9;q++){
			PP_INIT.assign_loc(p,q,PP0[p][q]);
		}
	}
	//getting square root of covariance matrix
	Matrix APP_INIT=PP_INIT.cholesky();

	//drawing Gaussian 9x1 vector with unit std deviation
	Matrix GAUSS_INIT(9,1);
	for(int r=0; r<9;r++){
		GAUSS_INIT.assign_loc(r,0,gauss(0,1));
	}
	//forming random initial state vector
	Matrix XX_INIT=APP_INIT*GAUSS_INIT;
	XX_INIT*=(1+frax);

	//forming subvectors for initialization and converting tilt to radians
	ESTTC.build_vec3(XX_INIT[0],XX_INIT[1],XX_INIT[2]);
	EVBE.build_vec3(XX_INIT[3],XX_INIT[4],XX_INIT[5]);
	RECE.build_vec3(XX_INIT[6],XX_INIT[7],XX_INIT[8]);
	RECE*=0.001;

	//initializing INS velocity and position vectors
	VBELC=EVBE+VBEL;
	SBELC=ESTTC+SBEL;
	//-------------------------------------------------------------------------
	//loading module-variables
	//initializations
	missile[303].gets_vec(VBELC);
	missile[344].gets_vec(RECE);
	missile[346].gets_vec(EVBE);
	missile[348].gets_vec(ESTTC);
	missile[304].gets_vec(SBELC);
}	
///////////////////////////////////////////////////////////////////////////////
//INS module
//Member function of class 'Missile'
//Error equations based on Zipfel,p.438,Eq.10.98
//
//020513 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::ins(double int_step)
{
	//local variables
	Matrix EWBEL(3,1);
	Matrix RECED_NEW(3,1);
	Matrix UNIT(3,3);UNIT.identity();
	Matrix TLLC(3,3);TLLC.identity();
	Matrix RERE(3,3);RERE.identity();
	Matrix EVBED_NEW(3,1);
	double psivlc(0);
	double cthtblc(0);

	
	//local module-variables
	Matrix EFSPB(3,1);
	Matrix EWBEB(3,1);
	Matrix TBLC(3,3);
	Matrix FSPCB(3,1);
	Matrix WBECB(3,1);
	Matrix SBELC(3,1);
	Matrix VBELC(3,1);
	double thtblc(0);
	double thtblcx(0);
	double dvbec(0);
	double thtvlc(0);
	double thtvlcx(0);
	double psivlcx(0);
	Matrix TLCB(3,3);
	double phiblcx(0);
	Matrix WALKA(3,1);

	//localizing module-variables
	//input data
	int mins=missile[300].integer();
	double tanlat=missile[327].real();
	Matrix EWALKA=missile[316].vec();
	//input from other modules
	double time=flat6[0].real();
	Matrix TBL=flat6[120].mat();
	Matrix TLB=flat6[148].mat();
	Matrix WBEB=flat6[163].vec();
	Matrix SBEL=flat6[219].vec();
	Matrix FSPB=flat6[230].vec();
	Matrix VBEL=flat6[233].vec();
	double dvbe=flat6[236].real();
	//state variables
	Matrix RECED=missile[343].vec();
	Matrix RECE=missile[344].vec();
	Matrix EVBED=missile[345].vec();
	Matrix EVBE=missile[346].vec();
	Matrix ESTTCD=missile[347].vec();
	Matrix ESTTC=missile[348].vec();
	//-------------------------------------------------------------------------
	//calling altitude measurement (not used in this module)
	ins_alt();

	//ideal INS output
	if(mins==0){
		TBLC=TBL;
		FSPCB=FSPB;
		WBECB=WBEB;
		SBELC=SBEL;
		VBELC=VBEL;
		dvbec=dvbe;
	}
	else{
	
		//calling instrument measurement errors
		EFSPB=ins_accl();
		EWBEB=ins_gyro(WBECB, int_step);

		//attitude error equations
		EWBEL=TLB*EWBEB;
		RECED_NEW.assign_loc(0,0,EWBEL.get_loc(0,0)+EVBE.get_loc(1,0)/REARTH);
		RECED_NEW.assign_loc(1,0,EWBEL.get_loc(1,0)-EVBE.get_loc(0,0)/REARTH);
		RECED_NEW.assign_loc(2,0,EWBEL.get_loc(2,0)-EVBE.get_loc(1,0)*tanlat/REARTH);
		RECE=integrate(RECED_NEW,RECED,RECE,int_step);
		RECED=RECED_NEW;

		//INS derived direction cosine matrix
		RERE=RECE.skew_sym();
		TLLC=RERE+UNIT;
		TBLC=TBL*TLLC;
		TLCB=TBLC.trans();

		//adding random walk effect to acceleration measurement
		double ewalka1=EWALKA[0];
		double ewalka2=EWALKA[1];
		double ewalka3=EWALKA[2];
		WALKA.build_vec3(uniform(-ewalka1,ewalka1),uniform(-ewalka2,ewalka2),uniform(-ewalka3,ewalka3));
		FSPCB=WALKA+EFSPB+FSPB;
		
		//integrating velocity error equation
		Matrix EF=TLCB*EFSPB-RERE*TLCB*FSPCB;
		double ef1=EF.get_loc(0,0);
		double ef2=EF.get_loc(1,0);
		double ef3=EF.get_loc(2,0)+2.*AGRAV*ESTTC.get_loc(2,0)/REARTH;
		EVBED_NEW.build_vec3(ef1,ef2,ef3);
		EVBE=integrate(EVBED_NEW,EVBED,EVBE,int_step);
		EVBED=EVBED_NEW;

		//integrating postion errors
		Matrix ESTTCD_NEW=EVBE;
		ESTTC=integrate(ESTTCD_NEW,ESTTCD,ESTTC,int_step);
		ESTTCD=ESTTCD_NEW;

		//computing INS derived postion of missile B wrt Earth ref point E
		//(sent to 'guidance' module)
		SBELC=ESTTC+SBEL;

		//computing INS derived velocity of missile B wrt Earth frame E
		VBELC=EVBE+VBEL;
		dvbec=VBELC.absolute();
	}
	//computing flight path angles
	double vbelc1=VBELC.get_loc(0,0);
	double vbelc2=VBELC.get_loc(1,0);
	double vbelc3=VBELC.get_loc(2,0);
	if(vbelc1==0&&vbelc2==0){
		psivlc=0;
		thtvlc=0;
	}
	else{
		psivlc=atan2(vbelc2,vbelc1);
		thtvlc=atan2(-vbelc3,sqrt(vbelc1*vbelc1+vbelc2*vbelc2));
	}
	psivlcx=psivlc*DEG;
	thtvlcx=thtvlc*DEG;

	//computing Euler pitch angle
	double tblc13=TBLC.get_loc(0,2);
	if(fabs(tblc13)<1){
		thtblc=asin(-tblc13);
		cthtblc=cos(thtblc);
	}
	else{
		thtblc=PI/2*sign(-tblc13);
		cthtblc=EPS;
	}
	thtblcx=thtblc*DEG;

	//computing Euler roll angle
	double tblc23=TBLC.get_loc(1,2);
	double tblc33=TBLC.get_loc(2,2);
	double cphic=tblc33/cthtblc;
	if(fabs(cphic)>=1)
		cphic=(1-EPS)*sign(cphic);
	double phiblc=acos(cphic)*sign(tblc23);

	phiblcx=phiblc*DEG;
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[343].gets_vec(RECED);
	missile[344].gets_vec(RECE); 
	missile[345].gets_vec(EVBED);
	missile[346].gets_vec(EVBE); 
	missile[347].gets_vec(ESTTCD);
	missile[348].gets_vec(ESTTC);
	//output to other modules
	missile[303].gets_vec(VBELC);
	missile[306].gets_vec(WBECB);
	missile[315].gets_mat(TBLC);
	missile[328].gets(thtblc);
	missile[329].gets(thtblcx);
	missile[330].gets(dvbec);
	missile[331].gets(thtvlc);
	missile[332].gets(thtvlcx);
	missile[333].gets(psivlcx);
	missile[334].gets_vec(FSPCB);
	missile[335].gets(phiblcx);
	missile[304].gets_vec(SBELC);
	//diagnostics
	missile[324].gets_vec(EWBEB);	
	missile[325].gets_vec(EFSPB); 
}	
///////////////////////////////////////////////////////////////////////////////
//Gyro error model
//Member function of class 'Missile'
//
// (1) Introduces gyro errors: Random walk
//                             Gyro cluster misalignment
//                             Scale factor error
//                             Bias error
//                             Mass unbalence
// (2) Outputs gyro measurements of body angular velocities 
//
// Return output EWBEB = Error in angular vel of body wrt earth - rad/s 
// Argument output WBECB = Computed ang vel of body wrt earth - rad/s
//
//
//020513 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Missile::ins_gyro(Matrix &WBECB, double int_step)
{
	//local variables
	Matrix EWBEB(3,1);
	
	//local module-variables
	Matrix EMISCG(3,1);
	Matrix EUG(3,1);
	Matrix EWG(3,1);

	//localizing module-variables
	//input data
	Matrix EWALKG=missile[307].vec();
	Matrix EUNBG=missile[308].vec();
	Matrix EMISG=missile[309].vec();
	Matrix ESCALG=missile[310].vec();
	Matrix EBIASG=missile[311].vec();
	//input from other modules
	Matrix FSPB=flat6[230].vec();
	Matrix WBEB=flat6[163].vec();
	//-------------------------------------------------------------------------
	//computing cluster misalignment error
	Matrix EGB=ESCALG.diamat_vec()+EMISG.skew_sym();
	EMISCG=EGB*WBEB;
	Matrix EMSBG=EBIASG+EMISCG;

	//computing gyro spin axis sensitivity (mass unbalance)
	double eug1=EUNBG.get_loc(0,0)*FSPB.get_loc(0,0);
	double eug2=EUNBG.get_loc(1,0)*FSPB.get_loc(1,0);
	double eug3=EUNBG.get_loc(2,0)*FSPB.get_loc(2,0);
	EUG.build_vec3(eug1,eug2,eug3);

	//computing random walk error
	EWG=EWALKG*(1/sqrt(int_step));

	//combining all uncertainties
	EWBEB=EMSBG+EUG+EWG;

	//gyro measured body rates
	WBECB=WBEB+EWBEB;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	missile[322].gets_vec(EUG);
	missile[364].gets_vec(EMISCG);
	missile[323].gets_vec(EWG);

	return EWBEB;
}	
///////////////////////////////////////////////////////////////////////////////
//Accelerometer error model
//Member function of class 'Missile'
// (1) Introduces accelerometer errors: Bias
//                                      Random bias
//                                      Scale factor error
//                                      Accelerometer cluster misalignment
// (2) Outputs acceleration measurements in body axes
//
//020513 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Missile::ins_accl()
{
	//localizing module-variables
	//input data
	Matrix EWALKA=missile[316].vec();
	Matrix EMISA=missile[317].vec();
	Matrix ESCALA=missile[318].vec();
	Matrix EBIASA=missile[319].vec();
	//input from other modules
	Matrix FSPB=flat6[230].vec();
	//-------------------------------------------------------------------------
	//computing accelerometer erros without random walk (done in 'ins()')
	Matrix EAB=ESCALA.diamat_vec()+EMISA.skew_sym();
	Matrix EFSPB=EBIASA+EAB*FSPB;
	//-------------------------------------------------------------------------
	return EFSPB;
}	
///////////////////////////////////////////////////////////////////////////////
//Altimeter error model
//Member function of class 'Missile'
// (1) Intruduces altimeter errors: Bias
//                                  Random
// (2) Outputs altimeter measurement above sea level
//
//020513 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::ins_alt()
{
	//local module-variables
	double ehbe=0;
	double hbem=0;

	//localizing module-variables
	//input data
	double biasal=missile[312].real();
	double randal=missile[313].real();
	//input from other modules
	double alt=flat6[239].real();
	//-------------------------------------------------------------------------
    ehbe=biasal+randal;
    hbem=alt+ehbe;	
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[302].gets(hbem);
	missile[314].gets(ehbe);
}	