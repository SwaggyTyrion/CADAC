///////////////////////////////////////////////////////////////////////////////
//FILE: 'rocket_modules.cpp'
//
//Contains all modules of class 'Rocket'
//						aerodynamics()	rocket[10-49]
//						propulsion()	rocket[50-74]
//						sensor()		rocket[75-99]
//						guidance()		rocket[100-124]
//						control()		rocket[125-149]
//						forces()		rocket[150-159]
//						intercept()		rocket[160-174]
//
// universally used variables are assigned to rocket[0-9] 
//
//170802 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Definition of 'aerodynamics' module-variables
//Member function of class 'Rocket'
//Module-variable locations are assigned to rocket[10-49]
//
//070412 Created by Peter H Zipfel
//170910 Modified for SRBM, PZi
///////////////////////////////////////////////////////////////////////////////
void Rocket::def_aerodynamics()
{
	//Definition of module-variables
	rocket[11].init("area",0.636,"Reference area of rocket - deg","aerodynamics","data","");
	rocket[12].init("alpha_t0x",0,"Initial angle of attack of rocket - deg","aerodynamics","data","");
	rocket[13].init("beta_t0x",0,"Initial sideslip angle of rocket - deg","aerodynamics","data","");
	rocket[14].init("alpmax",0,"Maximum angle of attack - deg","aerodynamics","data","");
	rocket[20].init("alppx",0,"Total anlge of attack - deg","aerodynamics","diag","");
	rocket[21].init("phipx",0,"Aerodynamic roll angle - deg","aerodynamics","diag","");
	rocket[22].init("cnptgt",0,"Normal force coeff in man.plane - ND","aerodynamics","diag","");
	rocket[23].init("cltgt",0,"Lift force coeff in velocity coor - ND","aerodynamics","out","");
	rocket[24].init("cdtgt",0,"Drag coeff in velocity coor - ND","aerodynamics","out","");
	rocket[25].init("catgt",0,"Axial force coeff in body coor - ND","aerodynamics","out","");
	rocket[26].init("cytgt",0,"Side force coeff in body coor - ND","aerodynamics","out","");
	rocket[27].init("cntgt",0,"Normal force coeff in body coor - ND","aerodynamics","out","");
	rocket[28].init("cnalp",0,"Normal force derivative - 1/rad","aerodynamics","out","");
	rocket[29].init("cybet",0,"Side force derivative - 1/rad","aerodynamics","out","");
	rocket[30].init("gmax",0,"Max g permissible, given 'alpmax' - g's","aerodynamics","out","");
}
///////////////////////////////////////////////////////////////////////////////
//Initial calculations of 'aerodynamics' module 
//Member function of class 'Rocket'
// 
//170802 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::init_aerodynamics()
{
	//local module-variables
	double alphax(0);
	double betax(0);
	double cnalp(0);
	double cybet(0);

	//localizing module-variables
	//input data
	double alpha_t0x=rocket[12].real();
	double beta_t0x=rocket[13].real();
	//-------------------------------------------------------------------------
	//initializing incidence angles
	alphax=alpha_t0x;
	betax=beta_t0x;
	//providing derivatives of SRBM1 concept for the autopilot
	cnalp=7.468;
	cybet=-cnalp;
	//-------------------------------------------------------------------------
	//loading module-variables
	//ooutput to 'control' module
	rocket[28].gets(cnalp);
	rocket[29].gets(cybet);
	rocket[143].gets(alphax);
	rocket[144].gets(betax);

}
///////////////////////////////////////////////////////////////////////////////
//'aerodynamics' module 
//Member function of class 'Rocket'
// (1) Lift and drag coefficients from table look-up 
// (2) Converting them to body axes
// (3) Aerodynamic derivatives for autopilot
// (4) Max g's permissible
// 
//170802 Created by Peter H Zipfel
//170910 Modified for SRBM, PZi
///////////////////////////////////////////////////////////////////////////////
void Rocket::aerodynamics()
{
	//local variables
	double phip(0);
	//local module-variables
	double alppx(0);
	double phipx(0);
	double cnptgt(0);
	double cltgt(0);
	double cdtgt(0);
	double catgt(0);
	double cytgt(0);
	double cntgt(0);
	double gmax(0);
	double cnp_max(0);
	//localizing module-variables
	//input data
	double area=rocket[11].real();
	double alpmax=rocket[14].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	double mach=flat3[14].real();
	int mprop=rocket[50].integer();
	double mass=rocket[61].real();
	double alphax=rocket[143].real();
	double betax=rocket[144].real();
	//-------------------------------------------------------------------------
	//converting to aeroballistic coordinates
	double alpha=alphax*RAD;
	double beta=betax*RAD;
	double alpp=acos(cos(alpha)*cos(beta));
	double dum1=tan(beta);
	double dum2=sin(alpha);
	if((dum1*dum1>SMALL && dum2*dum2>SMALL))
		phip=atan2(dum1,dum2);
	//converting to degrees for output
	alppx=alpp*DEG;
	phipx=phip*DEG;

	//table look-up of lift and drag coefficient
	cltgt=aerotable.look_up("cltgt_vs_alpha_mach",alppx,mach);
	cdtgt=aerotable.look_up("cdtgt_vs_alpha_mach",alppx,mach);

	//coefficients in body coordinates (guarding against negative values)
	double cos_alpha=cos(alpha);
	double sin_alpha=sin(alpha);
	catgt=cdtgt*cos_alpha-cltgt*sin_alpha;
	//addding 10% base drag at motor burn-out
	if(mprop==0){
		catgt=catgt*1.1;}
	cnptgt=cdtgt*sin_alpha+cltgt*cos_alpha;
	cntgt=fabs(cnptgt)*cos(phip);
	cytgt=-fabs(cnptgt)*sin(phip);

	//calculating max g permissable corresponding to 'alpmax'
	double cltgt_max=aerotable.look_up("cltgt_vs_alpha_mach",alpmax,mach);
	double cdtgt_max=aerotable.look_up("cdtgt_vs_alpha_mach",alpmax,mach);
	cnp_max=cdtgt_max*sin(alpmax*RAD)+cltgt_max*cos(alpmax*RAD);

	double normal_force=cnp_max*pdynmc*area;
	double weight=mass*grav;
	gmax=normal_force/weight;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	rocket[20].gets(alppx);
	rocket[21].gets(phipx);
	rocket[22].gets(cnptgt);
	rocket[23].gets(cltgt);
	rocket[24].gets(cdtgt);
	//output to other modules
	rocket[25].gets(catgt);
	rocket[26].gets(cytgt);
	rocket[27].gets(cntgt);
	rocket[30].gets(gmax);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'propulsion' module-variables
//Member function of class 'Rocket'
//Module-variable locations are assigned to rocket[50-74]
//
// Initial numerical values are provided here
//
//170802 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::def_propulsion()
{
	rocket[50].init("mprop","int",0,"=0: no prop; =1: prop on - ND","propulsion","data/out","");
	rocket[51].init("pres_sl",101325,"Atmospheric pressure at SL - Pa","propulsion","data","");
	rocket[52].init("aexit",0.282,"Nozzle exit area - m^2","propulsion","data","");
	rocket[53].init("mass_launch",6000,"Mass at launch - kg","propulsion","data","");
	rocket[54].init("mass_fuel",4000,"fuel mass- kg","propulsion","data","");
	rocket[55].init("isp",230,"Specific impuls- sec","propulsion","data","");
	rocket[56].init("thrust_sl",128600,"Thrust at sea level - N","propulsion","data","");
	rocket[60].init("thrust",0,"Thrust at altitude - N","propulsion","out","com");
	rocket[61].init("mass",0,"Mass of missile - kg","propulsion","out","");
}
///////////////////////////////////////////////////////////////////////////////
//'propulsion' module
//Member function of class 'Rocket'
//
// mrpop=0 motor off
//		 1 motor on
// 
//	SRBM1 concept
//	 initial mass=6000 kg
//	 burnout mass=2000 kg
//   thrust=128,600 N
//   specific impulse=230 sec
//
//170802 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::propulsion()
{
	//local module-variables
	double thrust(0);

	//localizing module-variables
	//input data
	int mprop=rocket[50].integer();
	double pres_sl=rocket[51].real();
	double aexit=rocket[52].real();
	double mass_launch=rocket[53].real();
	double mass_fuel=rocket[54].real();
	double isp=rocket[55].real();
	double thrust_sl=rocket[56].real();
	//input from other modules
	double time=flat3[0].real();
	double launch_time=flat3[3].real();
	double press=flat3[16].real();
	//saving mass after burn-out
	double mass=rocket[61].real();
	//-------------------------------------------------------------------------
	//thrusting
	if(mprop==1){		
	//mass flow
	double mass_flow=thrust_sl/(isp*9.81);
	mass=mass_launch-mass_flow*launch_time;
	//correcting thrust for altitude
	thrust=thrust_sl+(pres_sl-press)*aexit;
	//burn out
		if(mass<=mass_launch-mass_fuel){
			thrust=0;
			mprop=0;
		}
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving value
	rocket[61].gets(mass);
	//output to other modules
	rocket[50].gets(mprop);
	rocket[60].gets(thrust);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'sensor' module-variables
//Member function of class 'Rocket'
//Module-variable locations are assigned to rocket[75-99]
//
//170911 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::def_sensor()
{
	rocket[75].init("mseek","int",0,"Flag for sensor modes - ND","sensor","data","");
	rocket[76].init("stel1",0,"'North' location of rocket-target - m","sensor","data","");
	rocket[77].init("stel2",0,"'East' location of rocket-target - m","sensor","data","");
	rocket[78].init("stel3",0,"'Down' location of rocket-target - m","sensor","data","");
	rocket[80].init("dta",0,"Distance between rocket-target and rocket - m","sensor","out","com");
	rocket[81].init("dvta",0,"Closing speed on target - m/s","sensor","out","com");
	rocket[82].init("tgo_tgt",9999,"T-GO for rocket to reach rocket-target - s","sensor","diag","");
	rocket[83].init("los_azx",0,"Az.of LOS (airc-tgt) wrt tgt axes - deg","sensor","diag","");
	rocket[84].init("los_elx",0,"El.of LOS (airc-tgt) wrt tgt axes - deg","sensor","diag","");
	rocket[85].init("sigdy",0,"Pitch LOS rates in tgt coor - rad/s","sensor","diag","");
	rocket[86].init("sigdz",0,"Yaw LOS rates in tgt coor - rad/s","sensor","diag","");
	rocket[87].init("UTAA",0,0,0,"Unit LOS vector in rocket coord - ND","sensor","out","");
	rocket[88].init("WOEA",0,0,0,"Inertial LOS rate in rocket coord - rad/s","sensor","out","");
	rocket[89].init("STAL",0,0,0,"Rocket wrt target position - m","sensor","out","");
}
///////////////////////////////////////////////////////////////////////////////
//'sensor' module
//Kinematic sensor with unlimited field-of-view against target coordinates
//Member function of class 'Rocket'
//
//'maut'	'mguide'	 'mseek'
//	0			0|0			0	all ballistic
//	1			0|0			0	ascent with 'ancomx_bias', ballistic exo, reentry 'alcomx' = 0 until impact
//	1		  0/1|1			1	ascent with 'ancomx_bias', ballistic exo, reentry seeker-guidance to rocket coord/spiral man. 	
//		
//170911 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::sensor(Packet *combus,int num_vehicles,int vehicle_slot,double sim_time,double int_step)
{
	//local variables
	Matrix STEL(3,1);
	Variable *data_t=NULL;
	Matrix STAL(3,1);

	//local module-variables
	double dta(0);
	double dvta(0);
	double tgo_tgt(0);
	double los_azx(0);
	double los_elx(0);
	double sigdy(0);
	double sigdz(0);
	Matrix UTAA(3,1);
	Matrix WOEA(3,1);
	Matrix VTEL(3,1);

	//localizing module-variables
	//input data
	int mseek=rocket[75].integer();
	double stel1=rocket[76].real();
	double stel2=rocket[77].real();
	double stel3=rocket[78].real();
	//input from other modules
	Matrix TAL=flat3[22].mat();
	Matrix SAEL=flat3[26].vec();
	Matrix VAEL=flat3[27].vec();
	double alt=flat3[36].real();
	int flag_exo=rocket[126].integer();
	double alt_endo=rocket[128].real();
	//-------------------------------------------------------------------------
	if(mseek && flag_exo && alt<alt_endo)
	{
		//target of rocket location wrt to point E
		STEL.build_vec3(stel1,stel2,stel3);
		//target wrt rocket displacement
		STAL=STEL-SAEL;
		//unit LOS vector
		Matrix UTAL=STAL.univec3();
		UTAA=TAL*UTAL;
		//LOS angles wrt rocket body axes
		Matrix POLAR=UTAA.pol_from_cart();
		los_azx=DEG*POLAR[1];
		los_elx=DEG*POLAR[2];
		//differential velocity of rocket wrt missile as observed from Earth in local level coor
		Matrix VTAEL=VAEL*(-1);
		//closing velocity
		dvta=UTAL^VTAEL;

		//time-to-go
		double abs_dvta=fabs(dvta);
		dta=STAL.absolute();
		if(abs_dvta>SMALL)
			tgo_tgt=dta/abs_dvta;
		else
			tgo_tgt=0;

		//inertial LOS rates in rocket body coordinates
		WOEA=TAL*UTAL.skew_sym()*VTAEL*(1/dta);
		//diagnostic
		sigdy=WOEA[1];
		sigdz=WOEA[2];
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	rocket[2].gets_vec(VTEL);
	rocket[80].gets(dta);
	rocket[81].gets(dvta);
	rocket[82].gets(tgo_tgt);
	rocket[87].gets_vec(UTAA);
	rocket[88].gets_vec(WOEA);
	rocket[89].gets_vec(STAL);
	//diagnostics
	rocket[85].gets(sigdy);
	rocket[86].gets(sigdz);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'guidance' module-variables
//Member function of class 'Rocket'
//Module-variable locations are assigned to rocket[100-124]
//
//170911 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::def_guidance()
{
	//Definition of module-variables
	rocket[100].init("mguide","int",0,"=1: pronav; =11: spiral+pronav","guidance","data","");
	rocket[101].init("gnav",0,"Proportional navigation gain - ND","guidance","data","");
	rocket[102].init("tgo_manvr",0,"Evasive maneuver start - sec","guidance","data","");
	rocket[103].init("amp_manvr",0,"Initial amplitude of maneuver - g's","guidance","data","");
	rocket[104].init("frq_manvr",0,"Frequency of spiral maneuver - rad/s","guidance","data","");
	rocket[105].init("tgo63_manvr",0,"Amplitude decay at 63% tgo - sec","guidance","data","");
	rocket[106].init("annx",0,"Normal accel command, unrestricted - g's","guidance","diag","");
	rocket[107].init("allx",0,"Lateral accel command, unrestricted  - g's","guidance","diag","");
	rocket[108].init("an_manvr",0,"Normal spiral command  - g's","guidance","diag","");
	rocket[109].init("al_manvr",0,"Lateral spiral command  - g's","guidance","diag","");
	rocket[110].init("ancomx",0,"Rocket normal acceleration command - g's","guidance","out","");
	rocket[111].init("alcomx",0,"Rocket lateral acceleration command - g's","guidance","out","");
}
///////////////////////////////////////////////////////////////////////////////
//'guidance' module 
//Member function of class 'Rocket' 
//Calculates commands for autopilot
//
//'maut'	'mguide'	 'mseek'
//	0			0|0			0	all ballistic
//	1			0|0			0	ascent with 'ancomx_bias', ballistic exo, reentry 'alcomx' = 0 until impact
//	1		  0/1|1			1	ascent with 'ancomx_bias', ballistic exo, reentry seeker-guidance to rocket coord/spiral man. 	
//		
//	mguide = |manvr|mode|
//			  manvr = 1 decaying spiral maneuver	
//				    mode =1 pro-nav 
//
//Spiral decay maneuver
//  Starts at 'tgo_manvr' with amplitude 'amp_manvr' and frequency 'frq_manvr'.
//	Amplitude decays exponentially with the 63% time-to-go value 'tgo63_manvr' 
//
//170911 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::guidance(Packet *combus,int num_vehicles,int vehicle_slot,double int_step)
{
	//local variables
	double phi(0);
	double amp(0);

	//local module-variables
	double ancomx(0);
	double alcomx(0);
	double annx(0);
	double allx(0);
	double an_manvr(0);
	double al_manvr(0);

	//localizing module-variables
	//input data
	int mguide=rocket[100].integer();
	double gnav=rocket[101].real();
	double tgo_manvr=rocket[102].real();
	double amp_manvr=rocket[103].real();
	double frq_manvr=rocket[104].real();
	double tgo63_manvr=rocket[105].real();
	//input from other modules
	double grav=flat3[11].real();
	double alt=flat3[36].real();
	double gmax=rocket[30].real();
	double dvta=rocket[81].real();
	double tgo_tgt=rocket[82].real();
	Matrix UTAA=rocket[87].vec();
	Matrix WOEA=rocket[88].vec();
	int flag_exo=rocket[126].integer();
	//-------------------------------------------------------------------------
	//decoding guidance flag
	int guid_manvr=mguide/10;
	int guid_mode=(mguide%10);

	//pro-nav guidance
	if(guid_mode==1){
		//acceleration command in missile axes
		Matrix APNA=WOEA.skew_sym()*UTAA*gnav*fabs(dvta);
		annx=-APNA.get_loc(2,0)/grav;
		allx=APNA.get_loc(1,0)/grav;
	}
	//adding exponentially decaying spiral maneuver
	if(guid_manvr==1 && flag_exo && tgo_tgt<tgo_manvr){
		amp=amp_manvr*(1-exp(-tgo_tgt/tgo63_manvr));
		an_manvr=amp*sin(frq_manvr*tgo_tgt);
		al_manvr=amp*cos(frq_manvr*tgo_tgt);
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
	rocket[110].gets(ancomx);
	rocket[111].gets(alcomx);
	//diagnostics
	rocket[106].gets(annx);
	rocket[107].gets(allx);
	rocket[108].gets(an_manvr);
	rocket[109].gets(al_manvr);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'control' module-variables
//Member function of class 'Rocket'
//Module-variable locations are assigned to rocket[125-149]
//
//070412 Created by Peter H Zipfel
//170910 Modified for SRBM, PZi
///////////////////////////////////////////////////////////////////////////////
void Rocket::def_control()
{
	//Definition of module-variables
	rocket[125].init("maut","int",0,"=0: ballistic; =1: ascent accel.control - ND","control","data","");
	rocket[126].init("flag_exo","int",false,"=0: endo-atmos; =1: exo-atmos- ND","control","data","");
	rocket[127].init("ancomx_bias",0,"Normal accel. bias- g's","control","data","");
	rocket[128].init("alt_endo",0,"Reentry altitude into the atmosphere - m","control","data","");
	rocket[130].init("tip",0,"Incidence lag time constant of rocket - sec","control","diag","");
	rocket[131].init("xi",0,"Integral feedback  - rad/s","control","state","");
	rocket[132].init("xid",0,"Integral feedback derivative - rad/s^2 ","control","state","");
	rocket[133].init("ratep",0,"Pitch rate  - rad/s","control","state","");
	rocket[134].init("ratepd",0,"Pitch rate derivative  - rad/s^2","control","state","");
	rocket[135].init("alp",0,"Angle of attack - rad","control","state","");
	rocket[136].init("alpd",0,"Angle of attack derivative - rad/s","control","state","");
	rocket[137].init("yi",0,"Integral feedback  - rad/s","control","state","");
	rocket[138].init("yid",0,"Integral feedback derivative - rad/s^2 ","control","state","");
	rocket[139].init("ratey",0,"Yaw rate  - rad/s","control","state","");
	rocket[140].init("rateyd",0,"Yaw rate derivative  - rad/s^2","control","state","");
	rocket[141].init("bet",0,"Sideslip angle - rad","control","state","");
	rocket[142].init("betd",0,"Sideslip angle - rad/s","control","state","");
	rocket[143].init("alphax",0,"Angle of attack - deg","control","out","com");
	rocket[144].init("betax",0,"Sideslip angle - deg","control","out","com");
}
///////////////////////////////////////////////////////////////////////////////
//'control' module 
//Member function of class 'Rocket'
//acceleration autopilot for ascent and reentry
//
//'maut'	'mguide'	 'mseek'
//	0			0|0			0	all ballistic
//	1			0|0			0	ascent with 'ancomx_bias', ballistic exo, reentry 'alcomx' = 0 until impact
//	1		  0/1|1			1	ascent with 'ancomx_bias', ballistic exo, reentry seeker-guidance to rocket coord/spiral man. 	
//		
//170911 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::control(double int_step)
{
	//local variables
	double tr(0);
	double gacp(0);
	double ta(0);
	//local module-variables
	double tip(0);
	double alphax(0);
	double betax(0);

	//localizing module-variables
	//input data
	int maut=rocket[125].integer();
	int flag_exo=rocket[126].integer();
	double ancomx_bias=rocket[127].real();
	double alt_endo=rocket[128].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	double dvae=flat3[25].real();
	double alt=flat3[36].real();
	double area=rocket[11].real();
	double alpmax=rocket[14].real();
	double cytgt=rocket[26].real();
	double cntgt=rocket[27].real();
	double cnalp=rocket[28].real();
	double cybet=rocket[29].real();
	double thrust=rocket[60].real();
	double mass=rocket[61].real();
	int mguide=rocket[100].integer();
	double ancomx=rocket[110].real();
	double alcomx=rocket[111].real();
	//state variables
	double xi=rocket[131].real();
	double xid=rocket[132].real();
	double ratep=rocket[133].real();
	double ratepd=rocket[134].real();
	double alp=rocket[135].real();
	double alpd=rocket[136].real();
	double yi=rocket[137].real();
	double yid=rocket[138].real();
	double ratey=rocket[139].real();
	double rateyd=rocket[140].real();
	double bet=rocket[141].real();
	double betd=rocket[142].real();
	//-------------------------------------------------------------------------
	if(alt>alt_endo){
		//setting flag when rocket goes exo-atmospheric
		// flag stays set even after rocket re-enters (value is saved)
		flag_exo=true;

		//resetting state variables to zero for reentry
		xi=0;
		xid=0;
		ratep=0;
		ratepd=0;
		alp=0;
		alpd=0;
		yi=0;
		yid=0;
		ratey=0;
		rateyd=0;
		bet=0;
		betd=0;
	}
	if(!flag_exo){
		//applying normal acceleration bias during ascent only, prior to reaching exo
		ancomx=ancomx_bias;}

	if(maut==1&&alt<alt_endo){
		//acceleration autopilot only active below 30km
		//online computing parameters as functions of dynamic pressure, values for SRBM1
		tr=((-2e-7)*pdynmc+0.22); //rate loop time constant
		gacp=pow((0.002*pdynmc),0.575)*(1-0.5); //root locus gain of accel loop	
		ta=2.2; //ratio of prop/integral gains

		//Pitch acceleration controller
		//incidence lag time constant
		tip=dvae*mass/(pdynmc*area*fabs(cnalp)+thrust);
		//pitch specific force
		double fspz=-pdynmc*area*cntgt/mass;
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
		double fspy=pdynmc*area*cytgt/mass;
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
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	rocket[131].gets(xi);
	rocket[132].gets(xid);
	rocket[133].gets(ratep);
	rocket[134].gets(ratepd);
	rocket[135].gets(alp);
	rocket[136].gets(alpd);
	rocket[137].gets(yi);
	rocket[138].gets(yid);
	rocket[139].gets(ratey);
	rocket[140].gets(rateyd);
	rocket[141].gets(bet);
	rocket[142].gets(betd);
	//saving value
	rocket[126].gets(flag_exo);
	//output to other modules
	rocket[143].gets(alphax);
	rocket[144].gets(betax);
	//diagnostics
	rocket[130].gets(tip);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'force' module-variables
//Member function of class 'Rocket'
//Module-variable locations are assigned to rocket[150-159]
//
//Note that FSPA is entered into the 'flat3[20]' array because it is needed
// for the 'newton' module, which is a member of the 'Flat3' class
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::def_forces()
{
	//Definition of module-variables
	flat3[20].init("FSPA",0,0,0,"Specific force in rocket coor - m/s^2","forces","out","");
	rocket[150].init("aax",0,"Axial acceleration of rocket - g's","forces","diag","");
	rocket[151].init("alx",0,"Yaw acceleration of rocket - g's","forces","diag","com");
	rocket[152].init("anx",0,"Pitch acceleration of rocket - g's","forces","diag","com");
}
///////////////////////////////////////////////////////////////////////////////
//'force' module 
//Member function of class 'Rocket' 
//Calculates forces acting on rocket
//
//070412 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::forces()
{
	//local module-variables
	Matrix FSPA(3,1);
	double aax(0);
	double alx(0);
	double anx(0);

	//localizing module-variables
	//input data
	double acc_longx=rocket[40].real();
	//input from other modules
	double grav=flat3[11].real();
	double pdynmc=flat3[13].real();
	double area=rocket[11].real();
	double catgt=rocket[25].real();
	double cytgt=rocket[26].real();
	double cntgt=rocket[27].real();
	double thrust=rocket[60].real();
	double mass=rocket[61].real();
	//-------------------------------------------------------------------------
	FSPA[0]=(thrust-catgt*pdynmc*area)/mass;
	FSPA[1]=(cytgt*pdynmc*area)/mass;
	FSPA[2]=(-cntgt*pdynmc*area)/mass;

	//diagnostics: accelerations in rocket body coord
	aax=FSPA[0]/grav;
	alx=FSPA[1]/grav;
	anx=-FSPA[2]/grav;
	//-------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	flat3[20].gets_vec(FSPA);
	//diagnostics
	rocket[150].gets(aax);
	rocket[151].gets(alx);
	rocket[152].gets(anx);
}
///////////////////////////////////////////////////////////////////////////////
//Definition of 'intercept' module-variables
//Member function of class 'Rocket'
//Module-variable locations are assigned to rocket[160-174]
//
//170913 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::def_intercept()
{
	//Definition of module-variables
	rocket[160].init("write","int",true,"Flag for writing miss to console - ND","intercept","init","");
}
///////////////////////////////////////////////////////////////////////////////
//'intercept' module 
//Member function of class 'Rocket' 
//calculates miss distance of rocket from target coordinates
//stops computer run
//
//170913 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Rocket::intercept(Packet *combus,int vehicle_slot,double int_step,char *title)
{
	//input data
	//input from other modules
	double time=flat3[0].real();
	double dvae=flat3[25].real();
	Matrix SAEL=flat3[26].vec();
	double psivlx=flat3[29].real();
	double thtvlx=flat3[30].real();
	double alt=flat3[36].real();
	double dta=rocket[80].real();
	double dvta=rocket[81].real();
	Matrix STAL=rocket[89].vec();
	int mguide=rocket[100].integer();
	//getting saved value
	int write=rocket[160].integer();
	//-------------------------------------------------------------------------
	// displaying miss distance of target relative to rocket coordinates
	if((dta<1000)&&(mguide>0)&&write){
		// point of closest approach
		if(dvta>0){
			write=0;
			double stal1=STAL[0];
			double stal2=STAL[1];
			double stal3=STAL[2];

			//rocket impact on target coordinates
			cout<<"\n"<<" *** Rocket impacted target coordinates at sim_time = "<<time<<" sec\n";
			cout<<"      miss distance = "<<dta<<" m \n";
			cout<<"      north miss = "<<stal1<<" m  east miss = "<<stal2<<" m down miss = "<<stal3<<" m \n\n";

			//terminating rocket
			combus[vehicle_slot].set_status(0);
		}
	}	
	// inpact on ground
	if(alt<0&&write){
		write=0;
		double sael1=SAEL[0];
		double sael2=SAEL[1];
		double sael3=SAEL[2];

		//rocket impact on target coordinates
		cout<<"\n"<<" *** Rocket impacted ground at sim_time = "<<time<<" sec\n";
		cout<<"      north = "<<sael1<<" m  east = "<<sael2<<" m  down = "<<sael3<<" m \n";
		cout<<"      speed = "<<dvae<<" m/s  heading = "<<psivlx<<" deg  gamma = "<<thtvlx<<" deg \n\n";

		//terminating rocket
		combus[vehicle_slot].set_status(0);
	}	
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving values
	rocket[160].gets(write);
}
