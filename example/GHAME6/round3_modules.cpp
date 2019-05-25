///////////////////////////////////////////////////////////////////////////////
//FILE: 'round3_modules.cpp'
//Contains all modules of class 'Round3'
//							'environment()'
//							'newton()'
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Defining of kinematics module-variables
//Member function of class 'Round3'
//Module-variable locations are assigned to round3[0-9]
//
//Initializing the module-variables
//		
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::def_kinematics()
{
	//defining module-variables
	round3[0].init("time",0,"Vehicle time since launch - s","kinematics","diag","");
}

///////////////////////////////////////////////////////////////////////////////
//Initial calculations of kinematics module 
//Member function of class 'Round3'
// 
//Initial calculations
//Initialization of time
//
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::init_kinematics(double sim_time,double not_used)
{
	//local module-variables
	double time(0);
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//-------------------------------------------------------------------------
	//loading module-variables
	round3[0].gets(time);
}
///////////////////////////////////////////////////////////////////////////////
//Kinematic module
//Member function of class 'Round3'
//
//Time calculations
//
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::kinematics(double sim_time,double not_used1,double &not_used2,double &not_used3)
{
	//local module-variables
	double time(0);
	//-------------------------------------------------------------------------
	//setting vehicle time to simulation time
	time=sim_time;
	//-------------------------------------------------------------------------
	//loading module-variables
	round3[0].gets(time);
}
///////////////////////////////////////////////////////////////////////////////
//Defining of environmental module-variables
//Member function of class 'Round3'
//Module-variable locations are assigned to round3[10-19]
//
//Initializing the module-variables
//		
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::def_environment()
{
	//defining module-variables
	round3[11].init("grav",0,"Gravitational acceleration - m/s^2","environment","out","");
	round3[12].init("rho",0,"Air density - kg/m^3","environment","out","");
	round3[13].init("pdynmc",0,"Dynamic pressure - Pa","environment","out","");
	round3[15].init("vsound",0,"Speed of sound - m/s","environment","diag","");
	round3[16].init("mach",0,"Mach number - ND","environment","out","");
}

///////////////////////////////////////////////////////////////////////////////
//Environment module
//Member function of class 'Round3' 
//
//US 1976 Standard Atmosphere (NASA Marshall)
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::environment(double int_step)
{	
	//local variables
	double tempk(0);
	double press(0);
	
	//localized module-variables	
	double grav(0);
	double rho(0); 
	double vsound(0);
	double mach(0);
	double pdynmc(0);

	//localizing module-variables
	double alt=round3[40].real();
	double dvbe=round3[41].real();
	//-------------------------------------------------------------------------
	//Newtonian gravitational acceleration
	grav=GM/pow((REARTH+alt),2);

	// US Standard Atmosphere 1976 (NASA Marshall)
	double alt_km=alt/1000;
	int check=us76_nasa2002(alt_km,&rho,&press,&tempk,&vsound);
	if(check)
			{cerr<<" *** Error: altitude is outside us76_nasa2002 atmosphere *** \n";system("pause");exit(1);}

	//derived parameters
	mach=fabs(dvbe/vsound);
	pdynmc=0.5*rho*pow(dvbe,2);
	//-------------------------------------------------------------------------
	//loading module-variables
	round3[11].gets(grav);
	round3[12].gets(rho);
	round3[13].gets(pdynmc);
	round3[15].gets(vsound);
	round3[16].gets(mach);
}

///////////////////////////////////////////////////////////////////////////////
//Definition of newton module-variables 
//Member function of class 'Round3'
//Module-variable locations are assigned to round3[20-59]
// 
//Initializing variables for the Newton Module.
//First variable 'round3[0]' must be 'time'
//
//  minit = 0 Initialization with geographic variables
//		  = 1 Initialization with orbital elements
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::def_newton()
{
	//Definition of module-variables
	round3[0].init("time",0,"Vehicle time since launch - s","newton","diag","");
	round3[20].init("minit","int",0,"initializing, =0:geographic; =1:orbital elements","newton","data","");
	round3[22].init("TGV",0,0,0,0,0,0,0,0,0,"TM of geographic wrt geo velocity coord - ND ","newton","init","");
	round3[23].init("TIG",0,0,0,0,0,0,0,0,0,"TM of inertial wrt geographic coordinates ","newton","init","");
	round3[26].init("dbi",0,"Distance from Earth center - m","newton","diag","scrn,com");
	round3[27].init("psivg",0,"Vehicle heading angle - rad","newton","out","");
	round3[28].init("thtvg",0,"Vehicle flight path angle - rad","newton","out","");
	round3[29].init("WEII",0,0,0,0,0,0,0,0,0,"Earth's angular velocity (skew-sym) - rad/s ","newton","init","");
	round3[30].init("SB0II",0,0,0,"Initial inertial position - m ","newton","init","");
	round3[31].init("SBEG",0,0,0,"Geographic position wrt ground point below launch - m ","newton","state","");
	round3[32].init("VBEG",0,0,0,"Geographic velocity - m ","newton","state","");
	round3[33].init("TGE",0,0,0,0,0,0,0,0,0,"Geographic wrt Earth - ND ","newton","out","");	
	round3[34].init("altx",0,"Vehicle altitude - km","newton","diag","");
	round3[35].init("SBII",0,0,0,"Inertial position - m ","newton","state","com");
	round3[36].init("VBII",0,0,0,"Inertial velocity - m/s ","newton","state","com");
	round3[37].init("ABII",0,0,0,"Inertial acceleration - m/s^2 ","newton","state","");
	round3[38].init("lonx",0,"Vehicle longitude - deg","newton","data/diag","scrn,com");
	round3[39].init("latx",0,"Vehicle latitude - deg","newton","data/diag","scrn,com");
	round3[40].init("alt",0,"Vehicle altitude - m","newton","data/out","scrn,com");
	round3[41].init("dvbe",0,"Vehicle speed - m/s","newton","out","scrn,com");
	round3[42].init("psivgx",0,"Vehicle heading angle - deg","newton","data/out","scrn,com");
	round3[43].init("thtvgx",0,"Vehicle flight path angle - deg","newton","data/out","scrn,com");
	round3[44].init("semi",0,"Semi-major axis of elliptical orbit - m","newton","data","");
	round3[45].init("ecc",0,"Eccentricity of elliptical orbit - ND","newton","data","");
	round3[46].init("inclx",0,"Inclination of orbital plane - deg","newton","data","");
	round3[47].init("lon_anodex",0,"Longitude of the ascending node - deg","newton","data","");
	round3[48].init("arg_perix",0,"Argument of periapsis - deg","newton","data","");
	round3[49].init("true_anomx",0,"True anomaly - deg","newton","data","");
	round3[50].init("arg_latx",0,"Argument of latitude - deg","newton","diag","com");
}

///////////////////////////////////////////////////////////////////////////////
//Initial calculations of newton module 
//Member function of class 'Round3'
// 
//Initial calculations.
//
//  minit = 0 Initialization with geographic variables
//		  = 1 Initialization with orbital elements
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::init_newton()
{
	//local variables
	double lon(0);
	double lat(0);
	double psivg(0);
	double thtvg(0);
	Matrix SBIG(3,1);
	Matrix TEG(3,3);
	Matrix TEI(3,3);
	int parabola_flag(0);

	//local module-variables
	double dbi(0);
	Matrix TGE(3,3);
	Matrix SBII(3,1);
	Matrix SB0II(3,1);
	Matrix TGV(3,3);
	Matrix TIG(3,3);
	Matrix VBEG(3,1);
	Matrix WEII(3,3);
	Matrix VBII(3,1);

	//localizing module-variables
	//input data
	int minit=round3[20].integer();
	double lonx=round3[38].real();
	double latx=round3[39].real();
	double alt=round3[40].real();
	double dvbe=round3[41].real();
	double psivgx=round3[42].real();
	double thtvgx=round3[43].real();
	double semi=round3[44].real();
	double ecc=round3[45].real();
	double inclx=round3[46].real();
	double lon_anodex=round3[47].real();
	double arg_perix=round3[48].real();
	double true_anomx=round3[49].real();
	//input form other modules
	double time=round3[0].real();
	//-------------------------------------------------------------------------
	//Earth's angular velocity skew-symmetric tensor in inertial coordinates
	WEII.assign_loc(0,1,-WEII3);
	WEII.assign_loc(1,0,WEII3);

	//initialization with geographic variables	
	if(minit==0){
		//calculating initial vehicle position in Earth coordinates
		double radius=-(alt+REARTH);
		SBIG.assign_loc(2,0,radius);
		TGE=cad_tge(lonx*RAD,latx*RAD);
		TEG=TGE.trans();
		Matrix SBIE=TEG*SBIG;

		//calculating initial vehicle position in inertial coordinates
		// 'cad_tei' contains adjustment for celestial longitude of Greenwich merid. at time=0
		TEI=cad_tei(time);
		SBII=~TEI*SBIE;

		psivg=psivgx*RAD;
		thtvg=thtvgx*RAD;

		//georgaphic velocity
		VBEG.cart_from_pol(dvbe,psivg,thtvg);
		
		//TM of inertial wrt geopraphic coordinates
		TIG=~TEI*TEG;

		//initializing velocity state variable	
		VBII=TIG*VBEG+WEII*SBII;

		//diagnostics: argument-of-latitude for orbital trajectory
		parabola_flag=cad_orb_in(semi,ecc,inclx,lon_anodex,arg_perix,true_anomx, SBII,VBII);

	}
	//initialization with orbital elements
	else if(minit==1){

		//inertial position and velocity
		parabola_flag=cad_in_orb(SBII,VBII,semi,ecc,inclx,lon_anodex,arg_perix,true_anomx);

		//coordinates on Earth
		cad_geo84_in(lon,lat,alt, SBII,time);			  

		//building TM of inertial wrt geopraphic coordinates 
		TEI=cad_tei(time);
		TGE=cad_tge(lon,lat);
		TIG=~TEI*~TGE;

		//geographic velocity
		VBEG=~TIG*(VBII-WEII*SBII);

		//angles and magnitude of geographic velocity
		Matrix POLAR=VBEG.pol_from_cart();
		dvbe=POLAR[0];
		psivg=POLAR[1];
		thtvg=POLAR[2];

		//initializing for plotting
		psivgx=psivg*DEG;
		thtvgx=thtvg*DEG;
		lonx=lon*DEG;
		latx=lat*DEG;
	}

	//saving initial position for later use
	SB0II=SBII;

	//TM of velocity wrt geographic coordinates
	Matrix TVG=mat2tr(psivg,thtvg);
	TGV=~TVG;

	//diagnostics
	dbi=SBII.absolute();
	//-------------------------------------------------------------------------
	//loading module-variables
	round3[0].gets(time);
	round3[22].gets_mat(TGV);
	round3[23].gets_mat(TIG);
	round3[26].gets(dbi);
	round3[29].gets_mat(WEII);
	round3[30].gets_vec(SB0II);
	round3[32].gets_vec(VBEG);
	round3[33].gets_mat(TGE);
	round3[35].gets_vec(SBII);
	round3[36].gets_vec(VBII);
	round3[38].gets(lonx);
	round3[39].gets(latx);
	round3[40].gets(alt);
	round3[41].gets(dvbe);
	round3[42].gets(psivgx);
	round3[43].gets(thtvgx);

}
///////////////////////////////////////////////////////////////////////////////
//Newton module
//Member function of class 'Round3'
//
//Solving the translational equations of motion using Newton's 2nd Law
//
//040506 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round3::newton(double int_step)
{
	//local variables
	Matrix GRAV(3,1);
	double lon(0);
	double lat(0);
	double semi(0);
	double ecc(0);
	double inclx(0);
	double lon_anodex(0);
	double arg_perix(0);

	//localized module-variables
	Matrix TGE(3,3);
	double dvbe(0);
	double psivg(0);
	double thtvg(0);
	double lonx(0);
	double latx(0);
	double alt(0);
	double psivgx(0);
	double thtvgx(0);
	double altx(0);
	double true_anomx(0);
	double arg_latx(0);
	double dbi(0);
	
	//localizing module-variables
	//input from initialization module
	Matrix TGV=round3[22].mat();
	Matrix TIG=round3[23].mat();
	Matrix WEII=round3[29].mat();
	//state variables
	Matrix SBEG=round3[31].vec();
	Matrix VBEG=round3[32].vec();
	Matrix SBII=round3[35].vec();
	Matrix VBII=round3[36].vec();
	Matrix ABII=round3[37].vec();
	//input from other module
	double time=round3[0].real();
	double grav=round3[11].real();
	Matrix FSPV=round3[21].vec();
	//-------------------------------------------------------------------------
	//building gravitational vector in geographic coordinates
	GRAV.assign_loc(2,0,grav);

	//integrating inertial state variables
	Matrix NEXT_ACC=TIG*((TGV*FSPV)+GRAV);
	Matrix NEXT_VEL=integrate(NEXT_ACC,ABII,VBII,int_step);
	SBII=integrate(NEXT_VEL,VBII,SBII,int_step);
	ABII=NEXT_ACC;
	VBII=NEXT_VEL;

	//getting lon, lat and alt
		cad_geo84_in(lon,lat,alt, SBII,time);			  
	lonx=lon*DEG;
	latx=lat*DEG;
	altx=alt/1000;
	
	//calculating TM of geographic wrt Earth coordinates
	TGE=cad_tge(lon,lat);

	//calculating TM of geographic wrt inertial coordinates
	Matrix TEI=cad_tei(time);
	Matrix TGI=TGE*TEI;

	//calculating geographic velocity VBEG=TGI*(VBII-(WEII*SBII));
	Matrix NEXT_VBEG=TGI*(VBII-(WEII*SBII));

	//and integrating to obtain geographic displacement wrt initial launch point E
	//(SBEG should only be used for diagnostics!)
	SBEG=integrate(NEXT_VBEG,VBEG,SBEG,int_step);
	VBEG=NEXT_VBEG;

	//getting speed, heading and flight path angle
	Matrix POLAR=VBEG.pol_from_cart();		
	dvbe=POLAR.get_loc(0,0);
	psivg=POLAR.get_loc(1,0);
	thtvg=POLAR.get_loc(2,0);
	psivgx=psivg*DEG;
	thtvgx=thtvg*DEG;

	//preparing TMs for output
	TIG=TGI.trans();
	Matrix TVG=mat2tr(psivg,thtvg);
	TGV=TVG.trans();

	//diagnostics: argument-of-latitude for orbital trajectory
	int parabola_flag=cad_orb_in(semi,ecc,inclx,lon_anodex,arg_perix,true_anomx, SBII,VBII);
	arg_latx=arg_perix+true_anomx;
	//diagnostics
	dbi=SBII.absolute();

	//-------------------------------------------------------------------------
	//loading module-variables
	//state variables
	round3[31].gets_vec(SBEG);
	round3[32].gets_vec(VBEG);
	round3[35].gets_vec(SBII);
	round3[36].gets_vec(VBII);
	round3[37].gets_vec(ABII);
	//output
	round3[0].gets(time);
	round3[27].gets(psivg);
	round3[28].gets(thtvg);
	round3[38].gets(lonx);
	round3[39].gets(latx);
	round3[22].gets_mat(TGV);
	round3[23].gets_mat(TIG);
	round3[33].gets_mat(TGE);
	round3[34].gets(altx);
	round3[40].gets(alt);
	round3[41].gets(dvbe);
	round3[42].gets(psivgx);
	round3[43].gets(thtvgx);
	//diagnostics
	round3[26].gets(dbi);
	round3[49].gets(true_anomx);
	round3[50].gets(arg_latx);
}
