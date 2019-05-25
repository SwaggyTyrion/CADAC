///////////////////////////////////////////////////////////////////////////////
//FILE: 'aim_modules.cpp'
//
//Contains all modules of class 'Aim'
//						aerodynamics()	aim[10-49]
//						propulsion()	aim[50-74]
//						seeker()		aim[75-99]
//						guidance()		aim[100-124]
//						control()		aim[125-149]
//						forces()		aim[150-159]
//						intercept()		aim[160-174]
//
// generally used variables are assigned to aim[0-9] 
//
//070412 Created by Peter H Zipfel
//130725 Building AIM5, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'aerodynamics' module-variables
//Member function of class 'Aim'
//Module-variable locations are assigned to aim[10-49]
//
//070412 Created by Peter H Zipfel
//130727 Modified for AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::def_aerodynamics()
{
	//Definition of module-variables
	aim[11].init("area",0,"Reference area of aim - deg","aerodynamics","data","");
	aim[14].init("alpmax",0,"Maximum angle of attack - deg","aerodynamics","data","");
	aim[20].init("alppx",0,"Total anlge of attack - deg","aerodynamics","diag","scrn,plot");
	aim[21].init("phipx",0,"Aerodynamic roll angle - deg","aerodynamics","diag","scrn,plot");
	aim[22].init("cnpaim",0,"Normal force coeff in man.plane - ND","aerodynamics","diag","");
	aim[23].init("claim",0,"Lift force coeff in velocity coor - ND","aerodynamics","out","");
	aim[24].init("cdaim",0,"Drag coeff in velocity coor - ND","aerodynamics","out","");
	aim[25].init("caaim",0,"Axial force coeff in body coor - ND","aerodynamics","out","");
	aim[26].init("cyaim",0,"Side force coeff in body coor - ND","aerodynamics","out","");
	aim[27].init("cnaim",0,"Normal force coeff in body coor - ND","aerodynamics","out","");
	aim[28].init("cnalp",0,"Normal force derivative - 1/rad","aerodynamics","out","");
	aim[29].init("cybet",0,"Side force derivative - 1/rad","aerodynamics","out","");
	aim[30].init("gmax",0,"Max g permissible, given 'alpmax' - g's","aerodynamics","out","scrn,plot");
}
///////////////////////////////////////////////////////////////////////////////
//'aerodynamics' module 
//Member function of class 'Aim'
// (1) Lift and drag coefficients from table look-up 
// (2) Converting them to body axes
// (3) Aerodynamic derivatives for autopilot
// (4) Max g's permissible
// 
//070412 Created by Peter H Zipfel
//130727 Modified for AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::aerodynamics()
{
	//local variables
	double phip(0);
	//local module-variables
	double alppx(0);
	double phipx(0);
	double cnpaim(0);
	double claim(0);
	double cdaim(0);
	double caaim(0);
	double cyaim(0);
	double cnaim(0);
	double cnalp(0);
	double cybet(0);
	double gmax(0);
	double cnp_max(0);
	double cdaim_max(0);
	//localizing module-variables
	//input data
	double area=aim[11].real();
	double alphax=aim[143].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	double mach=flat3[14].real();
	int mprop=aim[50].integer();
	double mass=aim[61].real();
	double betax=aim[144].real();
	double alpmax=aim[14].real();
	//-------------------------------------------------------------------------
	//converting to aeroballistic coordinates
	double alpha=alphax*RAD;
	double beta=betax*RAD;
	double alpp=acos(cos(alpha)*cos(beta));
	double dum1=tan(beta);
	double dum2=sin(alpha);

	if(fabs(dum2)<SMALL)
		dum2=SMALL*sign(dum2);
	phip=atan2(dum1,dum2);

	//converting to degrees for output
	alppx=alpp*DEG;
	phipx=phip*DEG;

	//table look-up of lift and drag coefficient
	claim=aerotable.look_up("cl_aim_vs_alpha_mach",alppx,mach);
	if(mprop){
		cdaim=aerotable.look_up("cd_aim_on_vs_alpha_mach",alppx,mach);
	} else{
		cdaim=aerotable.look_up("cd_aim_off_vs_alpha_mach",alppx,mach);
	}
	//coefficients in body coordinates (guarding against negative values)
	double cos_alpha=cos(alpha);
	double sin_alpha=sin(alpha);
	caaim=cdaim*cos_alpha-claim*sin_alpha;
	cnpaim=cdaim*sin_alpha+claim*cos_alpha;
	cnaim=fabs(cnpaim)*cos(phip);
	cyaim=-fabs(cnpaim)*sin(phip);

	//calculating max g permissable corresponding to 'alpmax'
	double claim_max=aerotable.look_up("cl_aim_vs_alpha_mach",alpmax,mach);
	if(mprop){
		cdaim_max=aerotable.look_up("cd_aim_on_vs_alpha_mach",alpmax,mach);
	} else{
		cdaim_max=aerotable.look_up("cd_aim_off_vs_alpha_mach",alpmax,mach);
	}
	cnp_max=cdaim_max*sin(alpmax*RAD)+claim_max*cos(alpmax*RAD);

	double falphax=fabs(alphax);
	double fbetax=fabs(betax);
	if(falphax<10)
		cnalp=(0.123+0.013*falphax)*DEG;
	else
		cnalp=0.06*pow(falphax,0.625)*DEG;
	if(fbetax<10)
		cybet=-(0.123+0.013*fbetax)*DEG;
	else
		cybet=-0.06*pow(fbetax,0.625)*DEG;

	double normal_force=cnp_max*pdynmc*area;
	double weight=mass*grav;
	gmax=normal_force/weight;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	aim[20].gets(alppx);
	aim[21].gets(phipx);
	aim[22].gets(cnpaim);
	aim[23].gets(claim);
	aim[24].gets(cdaim);
	//output to other modules
	aim[25].gets(caaim);
	aim[26].gets(cyaim);
	aim[27].gets(cnaim);
	aim[28].gets(cnalp);
	aim[29].gets(cybet);
	aim[30].gets(gmax);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'propulsion' module-variables
//Member function of class 'Aim'
//Module-variable locations are assigned to aim[50-74]
//
// Numerical values are from AIM5
//
//070412 Created by Peter H Zipfel
//130727 Modified for AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::def_propulsion()
{
	aim[50].init("mprop","int",0,"Flag for propulsion modes - ND","propulsion","diag","");
	aim[51].init("pres_sl",101325,"Atmospheric pressure at SL - Pa","propulsion","data","");
	aim[52].init("aexit",0,"Nozzle exit area - m^2","propulsion","data","");
	aim[60].init("thrust",0,"Thrust at altitude - N","propulsion","out","scrn,plot");
	aim[61].init("mass",0,"Mass of missile - kg","propulsion","out","scrn,plot");
}
///////////////////////////////////////////////////////////////////////////////
//'propulsion' module
//Member function of class 'Aim'
//
// mrpop=0 motor off
//		 1 motor on
// 
// Based on AIM5
//	initial mass=63.8 kg
//
//070412 Created by Peter H Zipfel
//130727 Modified for AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::propulsion()
{
	//local variables

	//local module-variables
	double thrust_sl(0);
	double thrust(0);

	//localizing module-variables
	//input data
	int mprop=aim[50].integer();
	double pres_sl=aim[51].real();
	double aexit=aim[52].real();
	//input from other modules
	double time=flat3[0].real();
	double press=flat3[16].real();
	//restoring values
	double mass=aim[61].real();
	//-------------------------------------------------------------------------
	//TPo - will this work for multiple pulses?
	if(mprop==1){
		thrust_sl=proptable.look_up("thrust_vs_time",time);
		thrust=thrust_sl+(pres_sl-press)*aexit;
		mass=proptable.look_up("mass_vs_time",time);
	}
	if(time>0.0&&thrust_sl==0){
		mprop=0;
		thrust=0;
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	//output to other modules
	aim[50].gets(mprop);
	aim[60].gets(thrust);
	//saving values and output
	aim[61].gets(mass);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'seeker' module-variables
//Member function of class 'Aim'
//Module-variable locations are assigned to aim[75-99]
//
//070412 Created by Peter H Zipfel
//130727 Modified for AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::def_seeker()
{
	aim[1].init("acft_com_slot","int",0,"This aim slot in combus - ND","combus","out","");
	aim[2].init("VTEL",0,0,0,"Velocity of aircraft - m/s","combus","out","");
	aim[3].init("psivlx_acft",0,"Heading of aircraft - deg","combus","out","");
	aim[4].init("thtvlx_acft",0,"Flight path angle of aircraft - deg","combus","out","");
	aim[5].init("tgt_num","int",1,"Target tail # attacked by 'this' missile","combus","data","");
	aim[75].init("mseek","int",0,"Seeker: off=0, On=1 - ND","seeker","data","");
	aim[80].init("dta",0,"Distance between aim and aircraft - m","seeker","out","scrn,plot");
	aim[81].init("dvta",0,"Closing speed of aim wrt aircraft - m/s","seeker","out","");
	aim[82].init("tgo_aim",0,"T-GO for aim to reach aircraft - s","seeker","diag","");
	aim[83].init("los_azx",0,"Az.of LOS (airc-aim) wrt aim axes - deg","seeker","diag","");
	aim[84].init("los_elx",0,"El.of LOS (airc-aim) wrt aim axes - deg","seeker","diag","");
	aim[85].init("sigdy",0,"Pitch LOS rates in aim coor - rad/s","seeker","diag","");
	aim[86].init("sigdz",0,"Yaw LOS rates in aim coor - rad/s","seeker","diag","");
	aim[87].init("UTAA",0,0,0,"Unit LOS vector in missile coord - ND","seeker","out","");
	aim[88].init("WOEA",0,0,0,"Inertial LOS rate in missile coord - rad/s","seeker","out","");
	aim[89].init("STAL",0,0,0,"Aim aircraft wrt missile position - m","seeker","out","");
}
///////////////////////////////////////////////////////////////////////////////
//'seeker' module
//Kinematic seeker with unlimited field-of-view
//Locked on aircraft at start of run
//Member function of class 'Aim'
//Assuming offset location in 'combus' 'Packet' at: STEL -> 2, VTEL -> 3
//
//mseek=0 seeker off
//mseek=1 seeker on
//		
//070412 Created by Peter H Zipfel
//130727 Modified for AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)
{
	//local variables
	Matrix STEL(3,1);
	Variable *data_t=NULL;
	Matrix STAL(3,1);

	//local module-variables
	double dta(0);
	double dvta(0);
	double tgo_aim(0);
	double los_azx(0);
	double los_elx(0);
	double sigdy(0);
	double sigdz(0);
	Matrix UTAA(3,1);
	Matrix WOEA(3,1);
	int acft_com_slot(0);
	Matrix VTEL(3,1);
	double psivlx_acft(0);
	double thtvlx_acft(0);

	//localizing module-variables
	//input data
	int tgt_num=aim[5].integer();
	int mseek=aim[75].integer();
	//input from other modules
	Matrix TAL=flat3[22].mat();
	Matrix SAEL=flat3[26].vec();
	Matrix VAEL=flat3[27].vec();
	//-------------------------------------------------------------------------
	//downloading from 'combus' target aircraft variables 
	//building aim id = t(j+1)
	int i(0);
	char number[4];
	sprintf(number,"%i",tgt_num);
	string aircraft_id="a"+string(number);
	//finding slot 'i' of aircraft in 'combus' (same as in vehicle_list)
	for(i=0;i<num_vehicles;i++)
	{
		string id=combus[i].get_id();
		if (id==aircraft_id)
		{						
			//downloading data from aircraft packet
			data_t=combus[i].get_data();
			STEL=data_t[2].vec();
			VTEL=data_t[3].vec();
			psivlx_acft=data_t[4].real();
			thtvlx_acft=data_t[5].real();
			//save aircraft com slot
			acft_com_slot=i;
		}
	}
	if(mseek){
		//LOS kinematics of target aircraft  wrt missile
		//aircraft (T) wrt missile (A) position
		STAL=STEL-SAEL;
		//unit LOS vector
		dta=STAL.absolute();
		double dum=1/dta;
		Matrix UTAL=STAL*dum;
		UTAA=TAL*UTAL;
		//LOS angles wrt missile body axes
		Matrix POLAR=UTAA.pol_from_cart();
		los_azx=DEG*POLAR[1];
		los_elx=DEG*POLAR[2];
		//differential velocity of target aircraft wrt aim missile as observed from Earth in local level coor
		Matrix VTAEL=VTEL-VAEL;
		//closing velocity
		dvta=UTAL^VTAEL;
		//time-to-go
		tgo_aim=dta/fabs(dvta);

		//inertial LOS rates in missile body coordinates
		WOEA=TAL*UTAL.skew_sym()*VTAEL*dum;
		//diagnostic
		sigdy=WOEA[1];
		sigdz=WOEA[2];
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	aim[1].gets(acft_com_slot);
	aim[2].gets_vec(VTEL);
	aim[3].gets(psivlx_acft);
	aim[4].gets(thtvlx_acft);
	aim[81].gets(dvta);
	aim[87].gets_vec(UTAA);
	aim[88].gets_vec(WOEA);
	aim[89].gets_vec(STAL);
	//saving values and output
	aim[83].gets(los_azx);
	aim[84].gets(los_elx);
	//diagnostics
	aim[80].gets(dta);
	aim[82].gets(tgo_aim);
	aim[85].gets(sigdy);
	aim[86].gets(sigdz);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'guidance' module-variables
//Member function of class 'Aim'
//Module-variable locations are assigned to aim[100-124]
//
//	mguid = |manvr|mode|
//			  manvr = 1 decaying spiral maneuver	
//				    mode =1 pro-nav 
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aim::def_guidance()
{
	//Definition of module-variables
	aim[100].init("mguid","int",0,"=|manvr|mode| =11:|spiral|pronav|","guidance","data","");
	aim[101].init("gnav",0,"Proportional navigation gain - ND","guidance","data","");
	aim[102].init("tgo_manvr",0,"Evasive maneuver start - sec","guidance","data","");
	aim[103].init("amp_manvr",0,"Initial amplitude of maneuver - g's","guidance","data","");
	aim[104].init("frq_manvr",0,"Frequency of spiral maneuver - rad/s","guidance","data","");
	aim[105].init("tgo63_manvr",0,"Amplitude decay at 63% tgo - sec","guidance","data","");
	aim[106].init("annx",0,"Normal accel command, unrestricted - g's","guidance","diag","");
	aim[107].init("allx",0,"Lateral accel command, unrestricted  - g's","guidance","diag","");
	aim[108].init("an_manvr",0,"Normal spiral command  - g's","guidance","diag","");
	aim[109].init("al_manvr",0,"Lateral spiral command  - g's","guidance","diag","");
	aim[110].init("ancomx",0,"Aim normal acceleration command - g's","guidance","out","scrn,plot");
	aim[111].init("alcomx",0,"Aim lateral acceleration command - g's","guidance","out","scrn,plot");
}
///////////////////////////////////////////////////////////////////////////////
//'guidance' module 
//Member function of class 'Aim' 
//Calculating commands for autopilot
//
//Spiral decay maneuver
//  Starts at 'tgo_manvr' with amplitude 'amp_manvr' and frequency 'frq_manvr'.
//	Amplitude decays exponentially with the 63% time-to-go value 'tgo63_manvr' 
//
//070412 Created by Peter H Zipfel
//070905 Added spiral evasive maneuver, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::guidance(Packet *combus,int num_vehicles)
{
	//local variables
	double phi(0);

	//local module-variables
	double ancomx(0);
	double alcomx(0);
	double annx(0);
	double allx(0);
	double amp(0);
	double an_manvr(0);
	double al_manvr(0);

	//localizing module-variables
	//input data
	int mguid=aim[100].integer();
	double gnav=aim[101].real();
	double tgo_manvr=aim[102].real();
	double amp_manvr=aim[103].real();
	double frq_manvr=aim[104].real();
	double tgo63_manvr=aim[105].real();
	//input from other modules
	double grav=flat3[11].real();
	double gmax=aim[30].real();
	double dvta=aim[81].real();
	double tgo_aim=aim[82].real();
	Matrix UTAA=aim[87].vec();
	Matrix WOEA=aim[88].vec();
	//-------------------------------------------------------------------------
	//decoding guidance flag
	int guid_manvr=mguid/10;
	int guid_mode=(mguid%10);

	//pro-nav guidance
	if(guid_mode==1){
		//acceleration command in missile axes
		Matrix APNA=WOEA.skew_sym()*UTAA*gnav*fabs(dvta);
		annx=-APNA.get_loc(2,0)/grav;
		allx=APNA.get_loc(1,0)/grav;
	}
	//adding exponentially decaying spiral maneuver
	if(guid_manvr==1&&tgo_aim<tgo_manvr){
		amp=amp_manvr*(1-exp(-tgo_aim/tgo63_manvr));
		an_manvr=amp*sin(frq_manvr*tgo_aim);
		al_manvr=amp*cos(frq_manvr*tgo_aim);
		annx+=an_manvr;
		allx+=al_manvr;
	}
	//limiting acceleration commands by circular limiter
	double aax=sqrt(allx*allx+annx*annx);
	if(aax>gmax) aax=gmax;
	if((fabs(annx)<SMALL||fabs(allx)<SMALL))
		phi=0;
	else{
		phi=atan2(annx,allx);
	}
	alcomx=aax*cos(phi);
	ancomx=aax*sin(phi);
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	aim[106].gets(annx);
	aim[107].gets(allx);
	aim[108].gets(amp);
	aim[110].gets(ancomx);
	aim[111].gets(alcomx);
	//diagnostics
	aim[106].gets(annx);
	aim[107].gets(allx);
	aim[108].gets(an_manvr);
	aim[109].gets(al_manvr);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'control' module-variables
//Member function of class 'Aim'
//Module-variable locations are assigned to aim[125-149]
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aim::def_control()
{
	//Definition of module-variables
	aim[127].init("ta",0,"Ratio of prop/integral gain - ND","control","data","");
	aim[128].init("tr",0,"Rate loop time constant - sec","control","data","");
	aim[129].init("gacp",0,"Root locus gain of accel loop - rad/s2","control","data","");
	aim[130].init("tip",0,"Incidence lag time constant of aim - sec","control","diag","scrn,plot");
	aim[131].init("xi",0,"Integral feedback  - rad/s","control","state","");
	aim[132].init("xid",0,"Integral feedback derivative - rad/s^2 ","control","state","");
	aim[133].init("ratep",0,"Pitch rate  - rad/s","control","state","");
	aim[134].init("ratepd",0,"Pitch rate derivative  - rad/s^2","control","state","");
	aim[135].init("alp",0,"Angle of attack - rad","control","state","");
	aim[136].init("alpd",0,"Angle of attack derivative - rad/s","control","state","");
	aim[137].init("yi",0,"Integral feedback  - rad/s","control","state","");
	aim[138].init("yid",0,"Integral feedback derivative - rad/s^2 ","control","state","");
	aim[139].init("ratey",0,"Yaw rate  - rad/s","control","state","");
	aim[140].init("rateyd",0,"Yaw rate derivative  - rad/s^2","control","state","");
	aim[141].init("bet",0,"Sideslip angle - rad","control","state","");
	aim[142].init("betd",0,"Sideslip angle - rad/s","control","state","");
	aim[143].init("alphax",0,"Angle of attack of aim - deg","control","in/out","scrn,plot");
	aim[144].init("betax",0,"Sideslip angle of aim - deg","control","in/out","scrn.plot");
}
///////////////////////////////////////////////////////////////////////////////
//Initial calculations of 'control' module 
//Member function of class 'Aim'
// 
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aim::init_control()
{
	//localizing module-variables
	//input data
	double alphax=aim[143].real();
	double betax=aim[144].real();
	//-------------------------------------------------------------------------
	// initializing incidence angles
	double alp=alphax*RAD;
	double bet=betax*RAD;
	//-------------------------------------------------------------------------
	//initializing variables of 'control' module
	aim[135].gets(alp);
	aim[141].gets(bet);
}///////////////////////////////////////////////////////////////////////////////
//'control' module 
//Member function of class 'Aim' 
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aim::control(double int_step)
{
	//local module-variables
	double tip(0);
	double alphax(0);
	double betax(0);
	//localizing module-variables
	//input data
	double ta=aim[127].real();
	double tr=aim[128].real();
	double gacp=aim[129].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	double dvae=flat3[25].real();
	double area=aim[11].real();
	double alpmax=aim[14].real();
	double cyaim=aim[26].real();
	double cnaim=aim[27].real();
	double cnalp=aim[28].real();
	double cybet=aim[29].real();
	double thrust=aim[60].real();
	double mass=aim[61].real();
	double ancomx=aim[110].real();
	double alcomx=aim[111].real();
	//state variables
	double xi=aim[131].real();
	double xid=aim[132].real();
	double ratep=aim[133].real();
	double ratepd=aim[134].real();
	double alp=aim[135].real();
	double alpd=aim[136].real();
	double yi=aim[137].real();
	double yid=aim[138].real();
	double ratey=aim[139].real();
	double rateyd=aim[140].real();
	double bet=aim[141].real();
	double betd=aim[142].real();
	//-------------------------------------------------------------------------
	//Pitch acceleration controller
	//incidence lag time constant
	tip=dvae*mass/(pdynmc*area*fabs(cnalp)+thrust);
	//pitch specific force
	double fspz=-pdynmc*area*cnaim/mass;
	//P-I shaping
	double gr=gacp*tip*tr/dvae;
	double gi=gr/ta;
	double abez=-ancomx*grav;
	double ep=abez-fspz;
	double xid_new=gi*ep;
	xi=integrate(xid_new,xid,xi,int_step);
	xid=xid_new;
	double ratepc=-(ep*gr+xi);
	//pitch rate first order lag
	double ratepd_new=(ratepc-ratep)/tr;
	ratep=integrate(ratepd_new,ratepd,ratep,int_step);
	ratepd=ratepd_new;
	//incidence lag
	double alpd_new=(tip*ratep-alp)/tip;
	alp=integrate(alpd_new,alpd,alp,int_step);
	alpd=alpd_new;
	alphax=alp*DEG;
	//alpha limiter
	if(fabs(alphax)>alpmax) alphax=alpmax*sign(alphax);

	//Yaw acceleration controller
	//incidence lag time constant
	double tiy=dvae*mass/(pdynmc*area*fabs(cybet)+thrust);
	//yaw specific force
	double fspy=pdynmc*area*cyaim/mass;
	//P-I shaping
	gr=gacp*tiy*tr/dvae;
	gi=gr/ta;
	double abey=alcomx*grav;
	double ey=abey-fspy;
	double yid_new=gi*ey;
	yi=integrate(yid_new,yid,yi,int_step);
	yid=yid_new;
	double rateyc=(ey*gr+yi);
	//yaw rate first order lag
	double rateyd_new=(rateyc-ratey)/tr;
	ratey=integrate(rateyd_new,rateyd,ratey,int_step);
	rateyd=rateyd_new;
	//incidence lag
	double betd_new=-(tiy*ratey+bet)/tiy;
	bet=integrate(betd_new,betd,bet,int_step);
	betd=betd_new;
	betax=bet*DEG;
	//beta limiter (same as alpha limiter)
	if(fabs(betax)>alpmax) betax=alpmax*sign(betax);
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	aim[131].gets(xi);
	aim[132].gets(xid);
	aim[133].gets(ratep);
	aim[134].gets(ratepd);
	aim[135].gets(alp);
	aim[136].gets(alpd);
	aim[137].gets(yi);
	aim[138].gets(yid);
	aim[139].gets(ratey);
	aim[140].gets(rateyd);
	aim[141].gets(bet);
	aim[142].gets(betd);
	//output to other modules
	aim[143].gets(alphax);
	aim[144].gets(betax);
	//diagnostics
	aim[130].gets(tip);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'force' module-variables
//Member function of class 'Aim'
//Module-variable locations are assigned to aim[150-159]
//
//Note that FSPA is entered into the 'flat3[20]' array because it is needed
// for the 'newton' module, which is a member of the 'Flat3' class
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aim::def_forces()
{
	//Definition of module-variables
	flat3[20].init("FSPA",0,0,0,"Specific force in vehicle coor - m/s^2","forces","out","");
	aim[150].init("aax",0,"Axial acceleration of aim - g's","forces","diag","");
	aim[151].init("alx",0,"Yaw maneuver acceleration of aim - g's","forces","diag","scrn,plot");
	aim[152].init("anx",0,"Pitch maneuver acceleration of aim - g's","forces","diag","scrn,plot");
}
///////////////////////////////////////////////////////////////////////////////
//'force' module 
//Member function of class 'Aim' 
//Calculates forces acting on the missile
//
//070412 Created by Peter H Zipfel
//080606 Added fixed aim option, which requries g-bias in vertical direction, PZi
///////////////////////////////////////////////////////////////////////////////
void Aim::forces()
{
	//local module-variables
	Matrix FSPA(3,1);
	double aax(0);
	double alx(0);
	double anx(0);

	//localizing module-variables
	//input data
	double acc_longx=aim[40].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	double area=aim[11].real();
	double caaim=aim[25].real();
	double cyaim=aim[26].real();
	double cnaim=aim[27].real();
	double thrust=aim[60].real();
	double mass=aim[61].real();
	//-------------------------------------------------------------------------
	FSPA[0]=(thrust-caaim*pdynmc*area)/mass;
	FSPA[1]=(cyaim*pdynmc*area)/mass;
	FSPA[2]=(-cnaim*pdynmc*area)/mass;

	//diagnostics: accelerations in aim body coord
	aax=FSPA[0]/grav;
	alx=FSPA[1]/grav;
	anx=-FSPA[2]/grav;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	flat3[20].gets_vec(FSPA);
	//diagnostics
	aim[150].gets(aax);
	aim[151].gets(alx);
	aim[152].gets(anx);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'intercept' module-variables
//Member function of class 'Aim'
//Module-variable locations are assigned to aim[160-174]
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aim::def_intercept()
{
	//Definition of module-variables
	aim[161].init("aspazx",0,"Aspect azimuth of incoming missile - deg","intercept","diag","");
	aim[162].init("aspelx",0,"Aspect elevation of incoming missile - deg","intercept","diag","");
}
///////////////////////////////////////////////////////////////////////////////
//'intercept' module 
//Member function of class 'Aim' 
//Parameter Input: 'vehicle_slot' is current 'Aim' object
//Input from module-variable array: 'acft_com_slot' aircraft being attacked, determined in 'seeker' module
//
//Sign convention for incoming missile aspect angles wrt aircraft velocity vector
// Azimuth: positive if missile comes in from the right; negative if from left
// Elevation: positive if missile comes from above, negative if from below;
// Zero is at the positive  direction of the aircraft velocity vector
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aim::intercept(Packet *combus,int vehicle_slot,double int_step,char *title)
{
	//local module-variables
	Matrix TTL(3,3);//T.M. of aircraft wrt local level coordinates

	//localizing module-variables
	double aspazx(0);
	double aspelx(0);

	//input data
	//input from other modules
	double time=flat3[0].real();
	Matrix VAEL=flat3[27].vec();//missile
	int acft_com_slot=aim[1].integer();
	Matrix VTEL=aim[2].vec();//aircraft
	double psivlx_acft=aim[3].real();
	double thtvlx_acft=aim[4].real();
	double dta=aim[80].real();
	double dvta=aim[81].real();
	Matrix STAL=aim[89].vec();
	//-------------------------------------------------------------------------
	// displaying miss distance only if missile is inside sphere of aircraft
	if(dta<500){
		// point of closest approach
		if(dvta>0){
			//calculating aspect angles of incoming missile
			//aircraft T velocity relative to incoming missile A
			Matrix VTAEL=VTEL-VAEL;
			//differential speed of missile wrt aircraft
			double diff_speed=VTAEL.absolute();
			//T.M. of aircraft wrt local level coordinates
			TTL=mat2tr(psivlx_acft*RAD,thtvlx_acft*RAD);
			Matrix VTAT=TTL*VTAEL;
			Matrix POLAR=VTAT.pol_from_cart();
			//aspect angles
			double az=POLAR.get_loc(1,0);
			double el=POLAR.get_loc(2,0);
			aspazx=az*DEG;
			aspelx=el*DEG;

			//getting missile # and aircraft #
			string id_acft=combus[acft_com_slot].get_id();
			string id_aim=combus[vehicle_slot].get_id();

			//missile intercepted aircraft
			cout<<"\n"<<" $$$ Intercept of Missile_"<<id_aim<<" with Aircraft_"<<id_acft
				<<"   at sim_time = "<<time<<" sec $$$\n";
			cout<<"      miss distance = "<<dta<<" m     differential speed = "<<diff_speed<<" m/s \n";
			cout<<"      incoming missile azimuth = "<<aspazx<<" deg          elevation = "<<aspelx<<" deg \n\n";

			//missile and aircraft are set to be 'dead'
			combus[vehicle_slot].set_status(0);
			combus[acft_com_slot].set_status(0);
		}
	}	
}
