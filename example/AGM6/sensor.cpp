///////////////////////////////////////////////////////////////////////////////
//FILE: 'sensor.cpp'
//
//Contains 'sensor' module of class 'Missile'
//
//011221 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030410 Upgraded to SM Item32, PZi
//081007 Modified for GENSIM6, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'sensor' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[200-299]
// 
//Defining and initializing module-variables
// includes also target variables downloaded from 'combus'
//  and placed into reserved location missile[0-9] (used here and in 'guidance' module)
// 
//	mseek =	0: Sensor turned off
//			2: Sensor enabled (input,or set internally if break-lock occured)
//			3: Acquis. mode (set internally, when missile is within 'racq')
//			4: Sensor lock-on (set internally, when 'dtimac' has elapsed)
//			5: Sensor within blind range (set internally). Output held const
//
// skr_dyn = 0 kinematic sensor
//           1 dynamic sensor
//
//020607 Created by Peter H Zipfel
//081007 Modified for GENSIM6, PZi
///////////////////////////////////////////////////////////////////////////////

void Missile::def_sensor()
{
	//Definition and initialization of module-variables
	missile[1].init("tgt_num","int",0,"Target tail # attacked by 'this' missile","combus","data","");
	missile[2].init("STEL",0,0,0,"Position of target#, downloaded from 'combus' - m","combus","","");
	missile[3].init("VTEL",0,0,0,"Velocity of target#, downloaded from 'combus' - m","combus","","");
	missile[5].init("tgt_com_slot","int",0,"'This' target slot in combus - ND","combus","out","");
    missile[200].init("mseek","int",0,"See table in module 'sensor'","sensor","data/diag","com");
    missile[201].init("skr_dyn","int",0,"=0: Kinemtic, =1:Dynamic","sensor","data","");
    missile[202].init("isets1","int",0,"Sensor flag","sensor","init","");
    missile[203].init("epchac",0,"Epoch of start of sensor acquisition - s","sensor","init","");
    missile[204].init("ibreak","int",0,"Flag for sensor break-lock ND","sensor","init","");
    missile[206].init("dblind",0,"Blind range - m","sensor","data","");
    missile[233].init("racq",0,"Acquisition range - m","sensor","data","");
    missile[234].init("dtimac",0,"Target acquisition time - s","sensor","data","");
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
    missile[269].init("fovyaw",0,"Half yaw field-of-view at acquisition - rad","sensor","data","");
    missile[270].init("fovpitch",0,"Half positive pitch field-of-view at acquis. - rad","sensor","data","");
    missile[271].init("dba",0,"Distance between active sensor and its aimpoint - m","sensor","diag","");
    missile[272].init("daim",0,"Dist from targ to initiate aimpoint mode - m","sensor","data","");
    missile[273].init("BIASAI",0,0,0,"Bias error of aimpoint mode in target coor - m","sensor","data","");
    missile[274].init("BIASSC",0,0,0,"Bias error of hot spot mode in target coor - m","sensor","data","");
    missile[275].init("RANDSC",0,0,0,"Random error of hot spot mode in targ coor - m","sensor","data","");
    missile[276].init("epy",0,"Error of pointing in pitch - rad","sensor","diag","plot");
    missile[278].init("epz",0,"Error of pointing in yaw - rad","sensor","diag","plot");
    missile[279].init("thtpb",0,"Pitch pointing angle - rad","sensor","out","plot");
    missile[280].init("psipb",0,"Yaw pointing angle - rad","sensor","out","plot");
    missile[281].init("ththb",0,"Head pitch angle - rad","sensor","diag","plot");
    missile[282].init("phihb",0,"Head roll angle - rad","sensor","diag","plot");
    missile[283].init("TPB",0,0,0,0,0,0,0,0,0,"I/G TM of pointing axes wrt body axes","sensor","init","");
    missile[284].init("THB",0,0,0,0,0,0,0,0,0,"I/G TM of head axes wrt body axes","sensor","init","");
    missile[285].init("dvbtc",0,"Closing velocity computed by INS - m/s","sensor","diag","");
    missile[286].init("EAHH",0,0,0,"Aimpoint displacement wrt center of F.P. - rad","sensor","diag","plot");
    missile[287].init("EPHH",0,0,0,"Computer pointing error of sensor wrt center of F.P.","sensor","diag","");
    missile[288].init("EAPH",0,0,0,"Aimpoint to computer pointing displacement - rad","sensor","diag","");
    missile[289].init("thtpbx",0,"Pitch pointing angle - deg","sensor","diag","");
    missile[290].init("psipbx",0,"Yaw pointing angle - deg","sensor","diag","");
    missile[291].init("sigdpy",0,"Pitch sight line spin - rad/s","sensor","out","plot");
    missile[292].init("sigdpz",0,"Yaw sight line spin - rad/s","sensor","out","plot");
    missile[293].init("biaseh",0,"Image blur and pixel bias errors - rad","sensor","data","");
    missile[294].init("randeh",0,"Image blur and pixel random errors - rad","sensor","data","");
    missile[295].init("SBTL",0,0,0,"True missile wrt target displacement - m","sensor","diag","scrn");
    missile[296].init("epaz_saved",0,"Saving Markov az-error - rad","sensor","save","");
    missile[297].init("epel_saved",0,"Saving Markov el-error - rad","sensor","save","");
    missile[298].init("range_saved",0,"Saving Markov range-error - m","sensor","save","");
    missile[299].init("rate_saved",0,"Saving Markov range-rate-error - m/s","sensor","save","");
}	
///////////////////////////////////////////////////////////////////////////////
//Sensor module
//Member function of class 'Missile'
//
//	mseek =	0: Sensor turned off
//			2: Sensor enabled (input,or set internally if break-lock occurred)
//			3: Acquis. mode (set internally, when missile is within 'racq')
//			4: Sensor lock-on (set internally, when 'dtimac' has elapsed)
//			5: Sensor within blind range (set internally). Output held const
//
// skr_dyn = 0 kinematic sensor
//           1 dynamic sensor
//
// Notes:
// (1) Value of 'racq' determines whether Sensor is locked-on before (LOBL) or after launch
//     For LOBL, the time delay 'dtimac' represents the sensor lock-out time
//
// (2) The target states are subscribed from 'combus'. They must be located in
//	   the 'combus Packet' at the following positions: STEL @ 2 and VTEL @ 3. 
//	   They are loaded into 'missile[2]' and 'missile[3]' for further use in 'this' missile 
//
//020605 Created by Peter H Zipfel
//021115 Incorporated IIR sensor, PZi
//081007 Modified for GENSIM6, PZi
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor(Packet *combus,int num_vehicles,double sim_time,double int_step)
{
	//local variables
	Variable *data_t;
	double sigdy(0),sigdz(0);
	double ehz(0),ehy(0);

	//local module-variables
	double dbt(0),dbtk(0);
	Matrix STEL(3,1);
	Matrix VTEL(3,1);
	Matrix SBTL(3,1);
	int tgt_com_slot(0);
	Matrix TTL(3,3);TTL.identity(); //shortcut, eventually should be subsribed from 'combus'
	double psipb(0),thtpb(0);
	double sigdpy(0),sigdpz(0);
	double ththb(0),phihb(0);	
	double psiot1(0),thtot1(0);
	double psipbx(0),thtpbx(0);

	//localizing module-variables
	//input data
	int tgt_num=missile[1].integer();
	int mseek=missile[200].integer();
	int skr_dyn=missile[201].integer();
	int isets1=missile[202].integer();
	double fovlimx=missile[232].real();
	double racq=missile[233].real();
	double dtimac=missile[234].real();
	double fovyaw=missile[269].real();
	double fovpitch=missile[270].real();
	double racq_irs=missile[806].real();
	double fovyawx_irs=missile[807].real();
	double fovpitchx_irs=missile[808].real();
	//getting saved value
	double epchac=missile[203].real();
	double timeac=missile[238].real();
	Matrix THB=missile[284].mat();
	//input from other modules
	double time=flat6[0].real();
	Matrix SBEL=flat6[219].vec();
	int trcond=missile[180].integer();
	int mguid=missile[400].integer();
	//-------------------------------------------------------------------------

	//downloading from 'combus' target variables
	//building target id = t(j+1)
	char number[4];	
	sprintf(number,"%i",tgt_num);
	string target_id="t"+string(number);
	//finding slot 'i' of target in 'combus' (same as in 'vehicle_list')
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
	//IIR gimbaled sensor
	//target aspect angles
	SBTL=SBEL-STEL;
	Matrix SBTT=TTL*SBTL;
	Matrix POLAR=SBTT.pol_from_cart();
	dbtk=POLAR.get_loc(0,0);
	psiot1=POLAR.get_loc(1,0);
	thtot1=POLAR.get_loc(2,0);

	//sensor is enabled
	if(mseek==2){
		//within acquisition range
		isets1=1;
		if(dbtk<racq)
			mseek=3;
	}
	//sensor in acquisition mode
	if(mseek==3){
		//initializing TM matrix, state variables and time counter
		if(isets1==1){
			sensor_ir_kin(thtpb,psipb,sigdy,sigdz, SBTL,VTEL,dbtk);
			sensor_ir_uthpb(ththb,phihb, psipb,thtpb);
			THB=sensor_ir_thb(ththb,phihb);
			isets1=0;
			epchac=time;
		}
		//acquisition(for dynamic sensor, target must be in field-of-view)
		if(skr_dyn==1){
			sensor_ir_dyn(mseek,mguid,thtpb,psipb,sigdy,sigdz,ehz,ehy,THB,trcond, SBTL,dbtk,int_step);
		   timeac=time-epchac;
		   if(timeac>dtimac){
				if((fabs(ehz)<=fovyaw)&&(fabs(ehy)<=fovpitch))
					mseek=4;
				else
					trcond=5;
		   }
		}
		else{
			sensor_ir_kin(thtpb,psipb,sigdy,sigdz, SBTL,VTEL,dbtk);
			timeac=time-epchac;
			if(timeac>dtimac)
				mseek=4;
		}
	}
	//sensor lock-on (dynamic or kinematic)
	if(mseek==4){
		if(skr_dyn==1){
			sensor_ir_dyn(mseek,mguid,thtpb,psipb,sigdy,sigdz,ehz,ehy,THB,trcond, SBTL,dbtk,int_step);
		}
		else{
			sensor_ir_kin(thtpb,psipb,sigdy,sigdz, SBTL,VTEL,dbtk);
		}

		//LOS rate output to the guidance module
		sigdpy=sigdy;
		sigdpz=sigdz;
	}
	thtpbx=thtpb*DEG;
	psipbx=psipb*DEG;

	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	missile[2].gets_vec(STEL); 
	missile[3].gets_vec(VTEL);
	missile[5].gets(tgt_com_slot);
	missile[180].gets(trcond);

	//output to other modules
	//IIR sensor
	missile[279].gets(thtpb);
	missile[280].gets(psipb);
	missile[291].gets(sigdpy);
	missile[292].gets(sigdpz);
    missile[400].gets(mguid);
	//saving value for next cycle
	missile[203].gets(epchac);
	missile[238].gets(timeac);
	missile[284].gets_mat(THB);
	//diagnostics
	missile[200].gets(mseek);
	missile[202].gets(isets1);
    missile[237].gets(dbtk);
	missile[289].gets(thtpbx);
	missile[290].gets(psipbx);
	missile[295].gets_vec(SBTL);
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
//                 sigdpz=Yaw sight line spin - rad/s
// Argument Input:
//                 SBTL(3)=Position of missile wrt target - m
//				   VTEL(3)=Target velocity vector - m/s
//                 dbtk=Distance between missile and target - m
//
//011221 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor_ir_kin(double &thtpb,double &psipb,double &sigdy,double &sigdz,
					   Matrix SBTL,Matrix VTEL,double dbtk)
{
	//local module-variables
	double dvbtc=0;

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
	dvbtc=fabs(UTBL^VTBL);

	//LOS rate output in body coordinates
	Matrix WOEB=TBL*UTBL.skew_sym()*VTBL/dbtk;

	//building pointing wrt body T.M.
	Matrix POLAR=STBB.pol_from_cart();
	psipb=POLAR.get_loc(1,0);
	thtpb=POLAR.get_loc(2,0);
	Matrix TPB=mat2tr(psipb,thtpb);

	//LOS rate output in pointing coordinates
	Matrix WOEP=TPB*WOEB;
	sigdy=WOEP.get_loc(1,0);
	sigdz=WOEP.get_loc(2,0);
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	missile[274].gets(dvbtc);
}
///////////////////////////////////////////////////////////////////////////////
//IIR sensor function
//Member function of class 'Missile'
// (1) Given true target relative geometry it determines inertial
//     LOS rates in pitch and yaw, corrupted by these errors:
//         Target Scintillation
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
//                 mguid: If break lock occured reset to 2 (midcourse)
//                 thtpb= Pitch pointing angle - rad
//                 psipb= Yaw pointing angle - rad
//                 sigdy= Pitch sight line spin - rad/s
//                 sigdz= Yaw sight line spin - rad/s
//                 ehz=	  Yaw sensor error angle - rad
//                 ehy=   Pitch sensor error angle - rad
//                 THB(3,3)= Transf matrix of head wrt body axes 
// Argument Input:
//                 SBTL(3)= Position of missile wrt target - m
//                 dbtk=     Distance between missile and target - m
//				   int_step= Integration step size - s
//
//020102 Converted from SRAAM6 by Peter Zipfel
//100526 Corrected 'trcond' pass-through, PZi
///////////////////////////////////////////////////////////////////////////////

void Missile::sensor_ir_dyn(int &mseek,int &mguid,double &thtpb,double &psipb,double &sigdy,
					   double &sigdz,double &ehz,double &ehy,Matrix &THB,int &trcond, Matrix SBTL,
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
	int ibreak=missile[204].integer();
	double dblind=missile[206].real();
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
	double trtht=missile[187].real();
	double trthtd=missile[189].real();
	double trphid=missile[190].real();
	double trate=missile[191].real();
	Matrix TBL=flat6[120].mat();
	Matrix TTL=missile[3].mat();
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
		  mseek=2;
		  mguid=40;
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
