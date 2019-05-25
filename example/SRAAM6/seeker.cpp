///////////////////////////////////////////////////////////////////////////////
//FILE: 'seeker.cpp'
//
//Contains 'seeker' module of class 'Missile'
//
//011221 Created from FORTRAN code SRAAM6 by Peter H Zipfel
//030410 Upgraded to SM Item32, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of seeker module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[200-299]
// 
//  mseek=0: No seeker
//       =2: Seeker enabled (input,or set internally if break-lock occured
//       =3: Acquis. mode (set internally, when missile is within 'racq')
//       =4: Seeker lock-on (set internally, when 'dtimac' has elapsed)
//       =5: Seeker within blind range (set internally). Output held const
//
//  ms1dyn=0: Kinematic seeker
//        =1: Dynamic seker
//
//Defining and initializing module-variables
// includes also target variables downloaded from 'combus'
// and placed into reserved location missile[0-9] (used here and in guidance())
// 
//011221 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::def_seeker()
{
	//Definition and initialization of module-variables
	missile[1].init("tgt_num","int",0,"Target tail # attacked by 'this' missile","combus","data","");
	missile[2].init("STEL",0,0,0,"Position of target, downloaded from 'combus' - m","combus","out","");
	missile[3].init("VTEL",0,0,0,"Velocity of target, downloaded from 'combus' - m","combus","out","");
	missile[4].init("TTL",1,0,0,0,1,0,0,0,1,"TM of target, downloaded from 'combus' - m","combus","","");
	missile[5].init("tgt_com_slot","int",0,"This target slot in combus - ND","combus","out","");
	missile[6].init("sht_num","int",0,"Shooter tail # launching 'this' missile","combus","data","");
	missile[7].init("SSEL",0,0,0,"Position of shooter, downloaded from 'combus' - m","combus","","");
	missile[8].init("dts",0,"Target-shooter distance - m","combus","out","scrn,plot");
	missile[9].init("STSL",0,0,0,"Velocity of shooter, downloaded from 'combus' - m","combus","out","");
    missile[200].init("mseek","int",0," =2:Enable, =3:Acquisition, =4:Lock","seeker","data/diag","com");
    missile[201].init("ms1dyn","int",0,"=0: Kinemtic, =1:Dynamic","seeker","data","");
    missile[202].init("isets1","int",0,"Seeker flag","seeker","init","");
    missile[203].init("epchac",0,"Epoch of start of seeker acquisition - s","seeker","save","");
    missile[204].init("ibreak","int",0,"Flag for seeker break-lock ND","seeker","init","");
    missile[206].init("dblind",0,"Blind range - m","seeker","data","");
    missile[207].init("racq",0,"Acquisition range - m","seeker","data","");
    missile[208].init("dtimac",0,"Time duration to acquire target - s","seeker","data","");
    missile[209].init("dbt",0,"true distance to target - m","seeker","diag","scrn,plot");
    missile[211].init("gk",0,"K.F. gain - 1/s","seeker","data","");
    missile[212].init("zetak",0,"K.F. damping","seeker","data","");
    missile[213].init("wnk",0,"K.F. natural frequency - rad/s","seeker","data","");
    missile[215].init("biast",0,"Pitch gimbal bias errors - rad","seeker","data","");
    missile[216].init("randt",0,"Pitch gimbal random errors - rad","seeker","data","");
    missile[217].init("biasp",0,"Roll gimbal bias error - rad","seeker","data","");
    missile[218].init("randp",0,"Roll gimbal bias error - rad","seeker","data","");
    missile[219].init("wlq1d",0,"Pitch sight line spin derivative - rad/s^2","seeker","state","");
    missile[220].init("wlq1",0,"Pitch sight line spin - rad/s","seeker","state","");
    missile[221].init("wlqd",0,"Pitch pointing rate derivative - rad/s^2","seeker","state","");
    missile[222].init("wlq",0,"Pitch pointing rate - rad/s","seeker","state","");
    missile[223].init("wlr1d",0,"Yaw sight line spin derivative - rad/s^2","seeker","state","");
    missile[224].init("wlr1",0,"Yaw sight line spin - rad/s","seeker","state","");
    missile[225].init("wlrd",0,"Yaw pointing rate derivative - rad/s^2","seeker","state","");
    missile[226].init("wlr",0,"Yaw pointing rate - rad/s","seeker","state","");
    missile[227].init("wlq2d",0,"Second state variable deriv in K.F. - rad/s^3","seeker","state","");
    missile[228].init("wlq2",0,"Second state variable in K.F. - rad/s^2","seeker","state","");
    missile[229].init("wlr2d",0,"Second state variable der in K.F. - rad/s^3","seeker","state","");
    missile[230].init("wlr2",0,"Second state variable in K.F. - rad/s^2","seeker","state","");
    missile[231].init("fovyaw",0,"Half yaw field-of-view at acquisition - rad","seeker","data","");
    missile[232].init("fovpitch",0,"Half positive pitch field-of-view at acquis. - rad","seeker","data","");
    missile[234].init("dba",0,"Distance between active seeker and its aimpoint - m","seeker","diag","");
    missile[235].init("daim",0,"Dist from targ to initiate aimpoint mode - m","seeker","data","");
    missile[236].init("BIASAI",1,0.5,0.2,"Bias error of aimpoint mode in target coor - m","seeker","data","");
    missile[239].init("BIASSC",0,0,0,"Bias error of hot spot mode in target coor - m","seeker","data","");
    missile[242].init("RANDSC",0,0,0,"Random error of hot spot mode in targ coor - m","seeker","data","");
    missile[245].init("epy",0,"Error of pointing in pitch - rad","seeker","diag","");
    missile[246].init("epz",0,"Error of pointing in yaw - rad","seeker","diag","");
    missile[247].init("thtpb",0,"Pitch pointing angle - rad","seeker","out","");
    missile[248].init("psipb",0,"Yaw pointing angle - rad","seeker","out","");
    missile[249].init("ththb",0,"Head pitch angle - rad","seeker","diag","");
    missile[250].init("phihb",0,"Head roll angle - rad","seeker","diag","");
    missile[251].init("ththbx",0,"Head pitch angle - deg","seeker","diag","scrn,plot");
    missile[252].init("phihbx",0,"Head roll angle - deg","seeker","diag","scrn,plot");
    missile[253].init("TPB",0,0,0,0,0,0,0,0,0,"I/G TM of pointing axes wrt body axes","seeker","init","");
    missile[262].init("THB",0,0,0,0,0,0,0,0,0,"I/G TM of head axes wrt body axes","seeker","init","");
    missile[271].init("timeac",0,"Time duration of seeker acquisition phase - s","seeker","diag","");
    missile[272].init("psiot1",0,"Azimuth of sensor LOS in target axes - rad","seeker","diag","");
    missile[273].init("thtot1",0,"Elevation of sensor LOS in target axes - rad","seeker","diag","");
    missile[274].init("dvbtc",0,"Closing velocity, computed - m/s","seeker","diag","");
    missile[276].init("EAHH",0,0,0,"Aimpoint displacement wrt center of F.P. - rad","seeker","diag","plot");
    missile[279].init("EPHH",0,0,0,"Computer pointing error of sensor wrt center of F.P.","seeker","diag","");
    missile[282].init("EAPH",0,0,0,"Aimpoint to computer pointing displacement - rad","seeker","diag","");
    missile[285].init("thtpbx",0,"Pitch pointing angle - deg","seeker","diag","scrn,plot");
    missile[286].init("psipbx",0,"Yaw pointing angle - deg","seeker","diag","scrn,plot");
    missile[287].init("sigdpy",0,"Pitch sight line spin - rad/s","seeker","out","plot");
    missile[288].init("sigdpz",0,"Yaw sight line spin - rad/s","seeker","out","plot");
    missile[289].init("biaseh",0,"Image blur and pixel bias errors - rad","seeker","data","");
    missile[290].init("randeh",0,"Image blur and pixel random errors - rad","seeker","data","");
    missile[291].init("psihlx",0,"Yaw angle of CL seeker wrt local axes - deg","seeker","diag","");
    missile[292].init("ththlx",0,"Pitch angle of CL seeker wrt local axes - deg","seeker","diag","");
    missile[293].init("phihlx",0,"Roll angle of CL seeker wrt local axes - deg","seeker","diag","");
    missile[294].init("SBTL",0,0,0,"Position of missile wrt target - m","seeker","diag","");
}	

///////////////////////////////////////////////////////////////////////////////
//Seeker module
//Member function of class 'Missile'
// (1) mseek=0: No seeker
//          =2: Seeker enabled (input,or set internally if break-lock occured
//          =3: Acquis. mode (set internally, when missile is within 'racq')
//          =4: Seeker lock-on (set internally, when 'dtimac' has elapsed)
//          =5: Seeker within blind range (set internally). Output held const
//
// (2) ms1dyn=0: Kinematic seeker
//           =1: Dynamic seker
//
// (3) Sets mguid=6(terminal guidance) when seeker is locked on (mseek=4)
//     Sets mguid=3(midcourse guidance) when seeker breaks lock (mseek=2)
//
// (4) Value of 'racq' determines wether Seeker is locked on before or after launch
//     For LOBL, the time delay 'dtimac' represents the seeker lock-out time
//
// RTI: The target states are subscribed from 'combus'. They must be located in
//		the 'combus Packet' at the following positions: STEL @ 1 and VTEL @ 2. 
//		They are loaded into missile[2] and missile[3] for futher use in 'this' missile 
//
//011221 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::seeker(Packet *combus,int num_vehicles,double int_step)
{
	//local variables
	double sigdy(0),sigdz(0);
	double ehz(0),ehy(0);
	Variable *data_t=NULL;
	int i(0);

	//local module-variables
	double psipb(0),thtpb(0);
	double sigdpy(0),sigdpz(0);
	double ththb(0),phihb(0);	
	double psiot1(0),thtot1(0);
	double psipbx(0),thtpbx(0);
	double dbt(0);
	double dts(0);
	double timeac(0);
	Matrix STEL(3,1);
	Matrix VTEL(3,1);
	Matrix SBTL(3,1);
	int tgt_com_slot(0);
	Matrix TTL(3,3);TTL.identity(); //shortcut, eventually comes from 'combus'
	Matrix SSEL(3,1);
	Matrix STSL(3,1);

	//localizing module-variables
	//input data
	int tgt_num=missile[1].integer();
	int sht_num=missile[6].integer();
	int mseek=missile[200].integer();
	int ms1dyn=missile[201].integer();
	int isets1=missile[202].integer();
	double racq=missile[207].real();
	double dtimac=missile[208].real();
	double fovyaw=missile[231].real();
	double fovpitch=missile[232].real();
	//getting saved value
	double epchac=missile[203].real();
	//from other modules
	double time=flat6[0].real();
	Matrix SBEL=flat6[219].vec();
	int mguid=missile[400].integer();
	//from previous integration cycle
	Matrix THB=missile[262].mat();
	//-------------------------------------------------------------------------
	//downloading from 'combus' shooter aircraft variables
	//building target id = t(j+1)
	char number[4];	
	sprintf(number,"%i",sht_num);
	string shooter_id="t"+string(number);
	//finding slot 'i' of shooter in 'combus' (same as in vehicle_list)
	for(i=0;i<num_vehicles;i++)
	{
		string id=combus[i].get_id();
		if (id==shooter_id)
		{						
			//downloading data from target packet
			tgt_com_slot=i;
			data_t=combus[i].get_data();
			SSEL=data_t[1].vec();
		}
	}
	//downloading from 'combus' target aircraft variables
	//building target id = t(j+1)
	sprintf(number,"%i",tgt_num);
	string target_id="t"+string(number);
	//finding slot 'i' of target in 'combus' (same as in vehicle_list)
	for(i=0;i<num_vehicles;i++)
	{
		string id=combus[i].get_id();
		if (id==target_id)
		{						
			//downloading data from target packet
			tgt_com_slot=i;
			data_t=combus[i].get_data();
			STEL=data_t[1].vec();
			VTEL=data_t[2].vec();
		}
	}
	//target wrt shooter displacement
	STSL=STEL-SSEL;
	dts=STSL.absolute();
	//target aspect angles
    SBTL=SBEL-STEL;
    Matrix SBTT=TTL*SBTL;
	Matrix POLAR=SBTT.pol_from_cart();
	dbt=POLAR.get_loc(0,0);
	psiot1=POLAR.get_loc(1,0);
	thtot1=POLAR.get_loc(2,0);

	//seeker is enabled
	if(mseek==2){
		//within acquisition range
		isets1=1;
		if(dbt<racq)mseek=3;
	}
	//seeker in acquisition mode
	if(mseek==3){
		//initializing TM matrix, state variables and time counter
		if(isets1==1){
            seeker_kin(thtpb,psipb,sigdy,sigdz, SBTL,VTEL,dbt);
            seeker_uthpb(ththb,phihb, psipb,thtpb);
            THB=seeker_thb(ththb,phihb);
            isets1=0;
            epchac=time;
		}
		//acquisition(for dynamic seeker, target must be in field-of-view)
		if(ms1dyn==1){
            seeker_dyn(mseek,mguid,thtpb,psipb,sigdy,sigdz,ehz,ehy,THB, SBTL,dbt,int_step);
            if((fabs(ehz)<=fovyaw)&&(fabs(ehy)<=fovpitch)){
               timeac=time-epchac;
               if(timeac>dtimac)mseek=4;
            }
		}
		else{
			seeker_kin(thtpb,psipb,sigdy,sigdz, SBTL,VTEL,dbt);
			timeac=time-epchac;
            if(timeac>dtimac)mseek=4;
		}
	}
	//seeker lock-on (dynamic or kinematic)
	if(mseek==4){
		mguid=6;
		if(ms1dyn==1){
            seeker_dyn(mseek,mguid,thtpb,psipb,sigdy,sigdz,ehz,ehy,THB, SBTL,dbt,int_step);
		}
		else{
            seeker_kin(thtpb,psipb,sigdy,sigdz, SBTL,VTEL,dbt);
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
	missile[7].gets_vec(SSEL); 
	missile[8].gets(dts);
	missile[9].gets_vec(STSL); 
	missile[247].gets(thtpb);
	missile[248].gets(psipb);
	missile[287].gets(sigdpy);
	missile[288].gets(sigdpz);
	missile[400].gets(mguid);
	//save for next integration cycle
	missile[262].gets_mat(THB);
	missile[203].gets(epchac);
	//diagnostics
	missile[200].gets(mseek);
	missile[202].gets(isets1);
	missile[209].gets(dbt);
	missile[271].gets(timeac);
	missile[272].gets(psiot1);
	missile[273].gets(thtot1);
	missile[285].gets(thtpbx);
	missile[286].gets(psipbx);
	missile[294].gets_vec(SBTL);
}
///////////////////////////////////////////////////////////////////////////////
//Kinematic seeker
//Member function of class 'Missile'
// (1)  Calculates error free LOS rates and angles
// (2)  Also used to initialize the dynamic seeker subroutine
//
// Argument Output:
//                 thtpb=Pitch pointing angle - rad
//                 psipb=Yaw pointing angle - rad
//                 sigdy=Pitch sight line spin - rad/s
//                 sigdpz=Yaw sight line spin - rad/s
// Argument Input:
//                 SBTL(3)=Position of missile wrt target - m
//				   VTEL(3)=Target velocity vector - m/s
//                 dbt=Distance between missile and target - m
//
//011221 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::seeker_kin(double &thtpb,double &psipb,double &sigdy,double &sigdz,
					   Matrix SBTL,Matrix VTEL,double dbt)
{
	
	//local module-variables
	double dvbtc(0);

	//localizing module-variables
	//from other modules
	Matrix TBL=flat6[120].mat();
	Matrix VBEL=flat6[233].vec();
	//-------------------------------------------------------------------------
	//LOS kinematics
    Matrix STBL=SBTL*(-1.);
    Matrix STBB=TBL*STBL;
	double dum1=1./dbt;
    Matrix UTBL=STBL*dum1;

	//relative velocity
	Matrix VTBL=VTEL-VBEL;

	//closing velocity
	dvbtc=fabs(UTBL^VTBL);

	//LOS rate output in body coordinates
	Matrix WOEB=TBL*UTBL.skew_sym()*VTBL*dum1;

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
//Dynamic seeker function
//Member function of class 'Missile'
// (1) Given true target relative geometry it determines inertial
//     LOS rates in pitch and yaw, corrupted by these errors:
//         Target Scintillation
//         Blur, pixel quatization and bias
//         Gimbal dynamics, quantization and bias
// (2) Determines Aimpoint off-set from computer determined seeker axis
//     in Focal Plane (F.P.) array EAPH(3)
// (3) Models Kalman Filter dynamics (generates inertial LOS rates)
// (4) Models strap-down gyro feedback and gimbal kinematics
// (5) Allows for aimpoint selection and correction.
//
// Argument Output:
//                 mseek: If break lock occured reset to 2 (acquisition)
//                 mguid: If break lock occured reset to 3 (midcourse)
//                 thtpb= Pitch pointing angle - rad
//                 psipb= Yaw pointing angle - rad
//                 sigdy= Pitch sight line spin - rad/s
//                 sigdz= Yaw sight line spin - rad/s
//                 ehz=	  Yaw seeker error angle - rad
//                 ehy=   Pitch seeker error angle - rad
//                 THB(3,3)= Transf matrix of head wrt body axes 
// Argument Input:
//                 SBTL(3)= Position of missile wrt target - m
//                 dbt=     Distance between missile and target - m
//				   int_step= Integration step size - s
//
//020102 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Missile::seeker_dyn(int &mseek,int &mguid,double &thtpb,double &psipb,double &sigdy,
					   double &sigdz,double &ehz,double &ehy,Matrix &THB, Matrix SBTL,
					   double dbt,double int_step)
{
	//local variables
	Matrix U1PP(3,1),U1HH(3,1);
	double ththbc,phihbc,phihbd;

	//local module-variables
	double epz(0),epy(0);
	double ththb(0),phihb(0);
	double ththbx(0),phihbx(0);
	Matrix EAHH(3,1);
	Matrix EPHH(3,1);
	Matrix EAPH(3,1);

	//localizing module-variables
	//input data
	int ibreak=missile[204].integer();
	double dblind=missile[206].real();
	double gk=missile[211].real();
	double zetak=missile[212].real();
	double wnk=missile[213].real();
	double biast=missile[215].real();
	double randt=missile[216].real();
	double biasp=missile[217].real();
	double randp=missile[218].real();
	double biaseh=missile[289].real();
	double randeh=missile[290].real();
	//initialization
	Matrix TPB=missile[253].mat();
	//from other modules
	Matrix TTL=missile[4].mat();
	Matrix TBL=flat6[120].mat();
	Matrix WBEB=flat6[163].vec();
	double trcode=missile[180].real();
	double trtht=missile[187].real();
	double trthtd=missile[189].real();
	double trphid=missile[190].real();
	double trate=missile[191].real();
	//state variables
	double wlq1d=missile[219].real();
	double wlq1=missile[220].real();
	double wlqd=missile[221].real();
	double wlq=missile[222].real();
	double wlr1d=missile[223].real();
	double wlr1=missile[224].real();
	double wlrd=missile[225].real();
	double wlr=missile[226].real();
	double wlq2d=missile[227].real();
	double wlq2=missile[228].real();
	double wlr2d=missile[229].real();
	double wlr2=missile[230].real();
	//diagnostic
	double time=flat6[0].real();
	//-------------------------------------------------------------------------
	//aimpoint modulation
	Matrix THL=THB*TBL;
	Matrix SBTH=THL*SBTL;
	Matrix SATH=seeker_aimp(THL,TTL,dbt);
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
	EAHH.build_vec3(0.,ehz,-ehy);
	//T.M. matrices
	Matrix TBH=THB.trans();//THB of previous integration cycle is used
	Matrix TPH=TPB*TBH; 
	Matrix THP=TPH.trans();
	//pointing error angles
	U1PP.build_vec3(1.,0.,0.);
	U1HH.build_vec3(1.,0.,0.);
	EPHH=THP*U1PP-U1HH;
	EAPH=EAHH-EPHH;
	Matrix EAPP=TPH*EAPH;
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

	//ouput to guidance module: LOS rates in pointing coord
    sigdz=wlr1;
    sigdy=wlq1;

	//look angle control
	Matrix WBEP=TPB*WBEB;
	double wbep2=WBEP.get_loc(1,0);
	double wbep3=WBEP.get_loc(2,0);
	//yaw channel
	double wlrd_new=wlr1-wbep3;
	wlr=integrate(wlrd_new,wlrd,wlr,int_step);
	wlrd=wlrd_new;
	psipb=wlr;
	double psipbd=wlrd;
	//pitch channel
	double wlqd_new=wlq1-wbep2;
	wlq=integrate(wlqd_new,wlqd,wlq,int_step);
	wlqd=wlqd_new;
    thtpb=wlq;
    double thtpbd=wlqd;
	//calculating the TPB matrix
	TPB=mat2tr(psipb,thtpb);

	//caculating gimbal dynamics and THB for the next integration cycle
	seeker_uthpb(ththbc,phihbc,psipb,thtpb);
    ththb=ththbc+biast+randt;
    phihb=phihbc+biasp+randp;
	THB=seeker_thb(ththb,phihb);

	//diagnostics
    ththbx=ththb*DEG;
    phihbx=phihb*DEG;

	//flagging break-lock and blind range conditions 
	if(mseek==4){
		ibreak=0;
		phihbd=-thtpbd*sin(psipb);
		double eh=sqrt(ehy*ehy+ehz*ehz);
		if(fabs(ththb)>trtht){
		   trcode=6.;
		   ibreak=1;
		}
		else if(fabs(thtpbd)>trthtd){
		   trcode=7.;
		   ibreak=1;
		}
		else if(fabs(phihbd)>trphid){
		   trcode=8.;
		   ibreak=1;
		}
		else if(eh>trate){
		   trcode=9.;
		   ibreak=1;
		}
		if(ibreak==1){
		  mseek=2;
		  mguid=3;
		}
		if(dbt<dblind) mseek=5;
    }
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	missile[219].gets(wlq1d);
	missile[220].gets(wlq1);
	missile[221].gets(wlqd);
	missile[222].gets(wlq);
	missile[223].gets(wlr1d);
	missile[224].gets(wlr1);
	missile[225].gets(wlrd);
	missile[226].gets(wlr);
	missile[227].gets(wlq2d);
	missile[228].gets(wlq2);
	missile[229].gets(wlr2d);
	missile[230].gets(wlr2);
	//diagnostics
	missile[245].gets(epy);
	missile[246].gets(epz);
	missile[249].gets(ththb);
	missile[250].gets(phihb);
    missile[251].gets(ththbx);
    missile[252].gets(phihbx);
	missile[253].gets_mat(TPB);
	missile[276].gets_vec(EAHH);
	missile[279].gets_vec(EPHH);
	missile[282].gets_vec(EAPH);
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
//          dbt=Distance of vehicle wrt target - m
//
//020102 Converted from SRAAM6 by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Missile::seeker_aimp(Matrix THL,Matrix TTL,double dbt)
{
	//local variables
	Matrix SATH(3,1);
	
	//localizing module-variables
	//input data
	double daim=missile[235].real();
	Matrix BIASAI=missile[236].vec();
	Matrix BIASSC=missile[239].vec();
	Matrix RANDSC=missile[242].vec();
	//from other modules
	//-------------------------------------------------------------------------
	Matrix THT=THL*TTL.trans();
	if(dbt<daim)
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

void Missile::seeker_uthpb(double &ththb,double &phihb, double psipb,double thtpb)
{
      ththb=acos(cos(thtpb)*cos(psipb));
      double sinpsi=sin(psipb);
      double tantht=tan(thtpb);
      if(fabs(sinpsi)&&fabs(tantht)<SMALL)
        phihb=0.;
      else
        phihb=atan2(sinpsi,tantht);
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

Matrix Missile::seeker_thb(double tht,double phi)
{
	//local variables
	Matrix THB(3,3);

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
}
