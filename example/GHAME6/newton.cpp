///////////////////////////////////////////////////////////////////////////////
//FILE: 'newton.cpp'
//Contains 'newton' module of class 'Round6'
//
//030416 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'newton' module-variables 
//Member function of class 'Round6'
//Module-variable locations are assigned to round6[210-299]
// 
//Initializing variables for the newton module.
//
//030410 Created by Peter H Zipfel
//050222 Variable integration step size, PZi
///////////////////////////////////////////////////////////////////////////////

void Round6::def_newton()
{
	//Definition of module-variables
	round6[210].init("minit","int",0,"minit=0:geogr. initialization; =1:automated","newton","data","");
	round6[217].init("alpha0x",0,"Initial angle-of-attack - deg","newton","data","");
	round6[218].init("beta0x",0,"Initial sideslip angle - deg","newton","data","");
	round6[219].init("lonx",0,"Vehicle longitude - deg","newton","init,diag","scrn,plot,com");
	round6[220].init("latx",0,"Vehicle latitude - deg","newton","init,diag","scrn,plot,com");
	round6[221].init("alt",0,"Vehicle altitude - m","newton","init,out","scrn,plot,com");
	round6[222].init("TVD",0,0,0,0,0,0,0,0,0,"TM of geo velocity wrt geodetic coord - ND ","newton","out","");
	round6[223].init("TDI",0,0,0,0,0,0,0,0,0,"TM of geodetic wrt inertial  coordinates ","newton","init","");
	round6[225].init("dvbe",0,"Vehicle geographic speed - m/s","newton","init,out","scrn,plot,com");
	round6[226].init("dvbi",0,"Vehicle inertial speed - m/s","newton","out","scrn,plot,com");
	round6[227].init("WEII",0,0,0,0,0,0,0,0,0,"Earth's angular velocity (skew-sym) - rad/s ","newton","init","");
	round6[228].init("psivdx",0,"Vehicle heading angle - deg","newton","init,out","scrn,plot,com");
	round6[229].init("thtvdx",0,"Vehicle flight path angle - deg","newton","init,out","scrn,plot,com");
	round6[230].init("dbi",0,"Vehicle distance from center of Earth - m","newton","out","scrn,plot");
	round6[231].init("TGI",0,0,0,0,0,0,0,0,0,"TM of geocentric wrt inertial  coordinates ","newton","init","");
	round6[232].init("VBED",0,0,0,"Geographic velocity in geodetic coord - m/s ","newton","out","");
	round6[234].init("altx",0,"Vehicle altitude - kft","newton","diag","");
	round6[235].init("SBII",0,0,0,"Inertial position - m ","newton","state","com");
	round6[236].init("VBII",0,0,0,"Inertial velocity - m/s ","newton","state","com");
	round6[237].init("ABII",0,0,0,"Inertial acceleration - m/s^2 ","newton","save","");
	round6[238].init("grndtrck",0,"Vehicle ground track on Earth surface - m","newton","diag","");
	round6[239].init("FSPB",0,0,0,"Specifiv forece in body coord - m/s^2 ","newton","out","");
	round6[240].init("ayx",0,"Achieved side acceleration - g's","newton","diag","plot");
	round6[241].init("anx",0,"Achieved normal acceleration - g's","newton","diag","plot");
	round6[242].init("gndtrkmx",0,"Ground track - km","newton","diag","");
	round6[243].init("gndtrnmx",0,"Ground track - nm","newton","diag","");
	round6[244].init("latx_bias",0,"Bias to adjust waypoint latitude for intercept - deg","newton","data","");
	round6[245].init("dvbi_bias",0,"Bias to modify insertion speed - m/s","newton","data","");
	round6[246].init("dbi_bias",0,"Bias to modify insertion altitude - m","newton","data","");
    round6[247].init("mfreeze_newt","int",0,"Saving mfreze value - ND","newton","save","");
    round6[248].init("dvbef",0,"Saved speed when mfreeze=1 - m/s","newton","save","");
	round6[249].init("thtvdx_bias",0,"Bias to modify insertion flight path angle - m","newton","data","");
	round6[250].init("sat_semi",0,"Sat.semi-major axis of elliptical orbit - m","newton","data","");
	round6[251].init("sat_ecc",0,"Sat.eccentricity of elliptical orbit - ND","newton","data","");
	round6[252].init("sat_inclx",0,"Sat.inclination of orbital plane - deg","newton","data","");
	round6[253].init("sat_lon_anodex",0,"Sat.longitude of the ascending node - deg","newton","data","");
	round6[254].init("sat_arg_perix",0,"Sat.argument of periapsis - deg","newton","data","");
	round6[255].init("sat_true_anomx",0,"Sat.true anomaly - deg","newton","data","");
	round6[256].init("ranglex_l_t",0,"Range angle of hyper wrt to satellite at start - deg","newton","data","");
	round6[257].init("headon_flag","int",0,"headon_flag=1:head-on =0:tail-chase","newton","data","");
	round6[258].init("tgo_insertion",0,"Estimated time to intercept - sec","newton","data","");
}

///////////////////////////////////////////////////////////////////////////////
//Initial calculations of 'newton' module 
//Member function of class 'Round6'
// 
//Initial calculations of HYPER vehicle
//
//  minit = 0 Geographic initialization provided in 'input.asc'
//		  = 1 Automated initialization and insertion based on satellite orbit 
//
//** Automated mission planning based on: satellites orbital elements,'ranglex_l_t',and 'tgo_insertion' 
// * Automated initialization
//    Inputing the orbital elements of the satellite at time = 0
//    Calculating the initial conditions of the hyper vehicle 'longx', 'latx', 'psivdx',
//     based on input 'ranglex_l_t' = range angle of hyper wrt to satellite at start - deg 
//       (it has the same positive sense as the satellite's true anomaly)
//    Remaining initial conditions of the vehicle are specified in 'input.asc':
//     'alt', 'dvbe', 'alphax0', 'betax0'=0, 'thtvdx'  
//    Head-on, tail-chase options:
//     'headon_flag' = +1: head-on; = 0: tail-chase
//    Assumptions:
//     At launch, the vehicle's 'longx' and 'latx' are under the satellite orbit at a location
//      displaced by 'ranglex_l_t'
//     The launch heading angle 'psivdx' is the inertial heading biased by the component
//      of the earth's rotation
//     The launch Euler angle 'psibdx' is automatically set equal to 'psivdx'
//      (assuming 'betax'=0)
// * Automated insertion conditions for LTG guidance 
//    Inputing the orbital elements of the satellite at time = 0 (same as above) and
//     the estimated time to insertion 'tgo_insertion'
//    Calculating the insertion conditions 'dbi_desired', 'dvbi_desired', 'thtdvgx_desired'
//     using Kepler's equation to propagate the satellite through 'tgo_insertion'
// * Automated waypoint selection for arc-guidance         
//    The satellite postion, projected to insertion serves as waypoint for the hyper vehicle
//	   Given 'tgo_insertion' the satellites 'longx' and 'latx' is projected
//		 and used as waypoint 'wp_lonx','wp_latx' for the hyper vehicle 	
//
//030410 Created by Peter H Zipfel
//040501 Added automated mission planning, PZi
///////////////////////////////////////////////////////////////////////////////

void Round6::init_newton()
{
	//local variables
	Matrix VBEB(3,1);
	Matrix POLAR(3,1);

	//local module-variables
	Matrix WBII(3,3);
	double dbi(0);
	Matrix VBED(3,1);
	Matrix SBII(3,1);
	Matrix VBII(3,1);
	double dvbi(0);
	double psivdx(0);
	double thtvdx(0);
	double dbi_desired(0);
	double dvbi_desired(0);
	double thtvdx_desired(0);
	double wp_lonx(0);
	double wp_latx(0);
	double int_step_new(0);

	//localizing module-variables
	//input data
	int minit=round6[210].integer();  
	double dvbe=round6[225].real();  
	double lonx=round6[219].real();  
	double latx=round6[220].real();  
	double alt=round6[221].real();
	double latx_bias=round6[244].real();  
	double dvbi_bias=round6[245].real();  
	double dbi_bias=round6[246].real();  
	double thtvdx_bias=round6[249].real();  
	double sat_semi=round6[250].real();
	double sat_ecc=round6[251].real();
	double sat_inclx=round6[252].real();
	double sat_lon_anodex=round6[253].real();		
	double sat_arg_perix=round6[254].real();
	double sat_true_anomx=round6[255].real();
	double ranglex_l_t=round6[256].real();
	int headon_flag=round6[257].integer();  
	double tgo_insertion=round6[258].real();
	//input from other modules
	double time=round6[0].real();
	double psibdx=round6[137].real();
	double thtbdx=round6[138].real();
	double phibdx=round6[139].real();
	double alpha0x=round6[217].real();	
	double beta0x=round6[218].real();	
	//-----------------------------------------------------------------------------
	//Earth's angular velocity skew-symmetric matrix (3x3)
	Matrix WEII(3,3);
	WEII.assign_loc(0,1,-WEII3);
	WEII.assign_loc(1,0,WEII3);

	//automated mission planning
	if(minit==1){
		//true anomaly if satellite where overhead launch point
		double true_anomx=sat_true_anomx+ranglex_l_t;

		//calculating inertial postion and velocity of overhead point on sat orbit
		Matrix SOII(3,1);
		Matrix VOII(3,1);
		int parabola_flag=cad_in_orb(SOII,VOII,sat_semi,sat_ecc,sat_inclx,sat_lon_anodex,sat_arg_perix,true_anomx);

		//calculating long and lat of this overhead point, which become the vehicle's long and lat at start
		double lon(0);
		double lat(0);
		double alt_unused(0);
		cad_geo84_in(lon,lat,alt_unused,SOII,time);
		lonx=lon*DEG;
		latx=lat*DEG;

		//getting Euler yaw angle and setting it equal to heading (assuming beta=0)
		//building TM of inertial wrt geopraphic coordinates
		Matrix TEI=cad_tei(time);
		Matrix TGE=cad_tge(lon,lat);
		Matrix TIG=~TEI*~TGE;
		//geographic velocity 
		Matrix VOEG=~TIG*(VOII-WEII*SOII);
		//Euler yaw angle
		Matrix POLAR=VOEG.pol_from_cart();
		psibdx=POLAR[1]*DEG;
		if(headon_flag){
			psibdx=psibdx-180;
		}
		//calculating the insertion conditions for LTG guidance
		//satellite's inertial position and velocity at time = 0
		Matrix STII(3,1);
		Matrix VTII(3,1);
		parabola_flag=cad_in_orb(STII,VTII,sat_semi,sat_ecc,sat_inclx,sat_lon_anodex,sat_arg_perix,sat_true_anomx);
		Matrix SPII(3,1);
		Matrix VPII(3,1);
		//satellites projected inertial position and velocity at intercept
		int kepler_flag=cad_kepler(SPII,VPII, STII,VTII,tgo_insertion);
		//desired LTG conditions
		double dvbi_pdct=VPII.absolute();
		dbi_desired=SPII.absolute()+dbi_bias;
		dvbi_desired=dvbi_pdct+dvbi_bias;

		//flight path angle at insertion
		double dvbe_pdct(0);
		double psivdx_pdct(0);
		double thtvdx_pdct(0);
		cad_geo84vel_in(dvbe_pdct,psivdx_pdct,thtvdx_pdct,SPII,VPII,time);
		if(headon_flag)
			thtvdx_desired=0;
		else
			thtvdx_desired=thtvdx_pdct+thtvdx_bias;

		//projecting satellite lon/lat to insertion and use it as waypoint
		double lonp(0);
		double latp(0);
		double altp(0);
		cad_geo84_in(lonp,latp,altp,SPII,tgo_insertion);
		wp_lonx=lonp*DEG;
		double wp_latx_unbiased=latp*DEG;
		//biasing waypoint latitude in order to line up with target
		wp_latx=wp_latx_unbiased+latx_bias;

		//diagnostics: great circle distance on Earth between satellite and launch point 
		double lonc(0);
		double latc(0);
		double altc(0);
		cad_geo84_in(lonc,latc,altc, STII,time);			  
		double launch_distance=cad_distance(lon,lat,lonc,latc);			  

		//console output
		string sense;
		if(headon_flag)
			sense="AGAINST";
		else
			sense="WITH";
		cout<<'\n';
		cout<<" *** Automated mission planning; hyper vehicle flies "<<sense<< " satellite direction ***\n";
		cout<<"     * Input: range angle of launch wrt sat = "<<ranglex_l_t<<" deg \t\ttime to intercept = "<<tgo_insertion<<" s\n";
		cout<<"     * Hyper vehicle at start     longitude = "<<lonx<<" deg \tlatitude = "<<latx<<" deg \t\theading angle = "<<psibdx<<" deg\n";
		cout<<"     * Desired insertion conditions  radius = "<<dbi_desired<<" m \tinertial vel = "<<dvbi_desired<<" m/s\tflight path angle = "<<thtvdx_desired<<" deg\n";
		cout<<"     * Waypoint                   longitude = "<<wp_lonx<<" deg \tlatitude = "<<wp_latx<<" deg\n";
		cout<<"     * Satellite at start         longitude = "<<lonc*DEG<<" deg \tlatitude = "<<latc*DEG<<" deg \taltitude = "<<altc<<" m\n";
		cout<<"     * Satellite at insertion     longitude = "<<lonp*DEG<<" deg \tlatitude = "<<latp*DEG<<" deg \t\taltitude = "<<altp<<" m \n";
		cout<<"                                  heading = "<<psivdx_pdct<<" deg    flight path = "<<thtvdx_pdct<<" deg    geo speed = "<<dvbe_pdct<<" m/s    inrtl speed = "<<dvbi_pdct<<" m/s\n";
		//cout<<"     * Ground range vehicle->sat  at start: = "<<launch_distance<<" km \n";			
	}
	//converting geodetic lonx, latx, alt to SBII
	SBII=cad_in_geo84(lonx*RAD,latx*RAD,alt,time);
	dbi=SBII.absolute();
			
	//building geodetic velocity VBED(3x1) from  alpha, beta, and dvbe 
    double salp=sin(alpha0x*RAD);
    double calp=cos(alpha0x*RAD);
    double sbet=sin(beta0x*RAD);
    double cbet=cos(beta0x*RAD);
    double vbeb1=calp*cbet*dvbe;
    double vbeb2=sbet*dvbe;
    double vbeb3=salp*cbet*dvbe;
	VBEB.build_vec3(vbeb1,vbeb2,vbeb3);

	//building TBD (if minit=1, psibdx is calculated above, otherwise all angles from 'input.asc'
	Matrix TBD=mat3tr(psibdx*RAD,thtbdx*RAD,phibdx*RAD); 
	//Geodetic velocity
	VBED=~TBD*VBEB;

	//building inertial velocity
	Matrix TDI=cad_tdi84(lonx*RAD,latx*RAD,alt,time);
	Matrix TGI=cad_tgi84(lonx*RAD,latx*RAD,alt,time);
	VBII=~TDI*VBED+WEII*SBII;
	dvbi=VBII.absolute();

	//calculating geodetic flight path angles (plotting initialization)
	POLAR=VBED.pol_from_cart();
	psivdx=DEG*POLAR.get_loc(1,0);
	thtvdx=DEG*POLAR.get_loc(2,0);
//-----------------------------------------------------------------------------
	//loading module-variables
	//initializations
	round6[219].gets(lonx);
	round6[220].gets(latx);
	round6[223].gets_mat(TDI);
	round6[226].gets(dvbi);
	round6[227].gets_mat(WEII);
	round6[228].gets(psivdx);
	round6[229].gets(thtvdx);
	round6[230].gets(dbi);
	round6[231].gets_mat(TGI);
	round6[232].gets_vec(VBED);
	round6[235].gets_vec(SBII);
	round6[236].gets_vec(VBII);
	//output to other module
	round6[137].gets(psibdx);
	hyper[405].gets(wp_lonx);
	hyper[406].gets(wp_latx);
	hyper[441].gets(dbi_desired);
	hyper[442].gets(dvbi_desired);
	hyper[443].gets(thtvdx_desired);
}
///////////////////////////////////////////////////////////////////////////////
//Newton module
//Member function of class 'Round6'
//
//Solving the translational equations of motion using Newton's 2nd Law
//
//030410 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Round6::newton(double int_step)
{
	//local variables
	double lon(0);
	double lat(0);

	//local module-variables
	double dvbe(0);
	double dvbi(0);
	double lonx(0);
	double latx(0);
	double alt(0);
	double altx(0);
	double psivdx(0);
	double thtvdx(0);
	Matrix FSPB(3,1);
	double anx(0);
	double ayx(0);
	double dbi(0);
	double gndtrkmx(0);
	double gndtrnmx(0);
	Matrix TVD(3,3);
	Matrix VBED(3,1);
	
	//localizing module-variables
	//initializations
	Matrix TDI=round6[223].mat();
	Matrix TGI=round6[231].mat();
	Matrix WEII=round6[227].mat();
	//input data
	//getting saved vaules
	double grndtrck=round6[238].real();
	int mfreeze_newt=round6[247].integer();
	double dvbef=round6[248].real();
	//state variables
	Matrix SBII=round6[235].vec();
	Matrix VBII=round6[236].vec();
	Matrix ABII=round6[237].vec();
	//input from other modules
	double time=round6[0].real();
	Matrix GRAVG=round6[62].vec();
	Matrix TBI=round6[121].mat();	    
	Matrix FAPB=round6[200].vec();  	
	double vmass=hyper[15].real();
	int mfreeze=hyper[503].integer();
	//-----------------------------------------------------------------------------
	//integrating vehicle acceleration
	FSPB=FAPB*(1./vmass);
	Matrix NEXT_ACC=~TBI*FSPB+~TGI*GRAVG;
	Matrix NEXT_VEL=integrate(NEXT_ACC,ABII,VBII,int_step);
	SBII=integrate(NEXT_VEL,VBII,SBII,int_step);
	ABII=NEXT_ACC;
	VBII=NEXT_VEL;
	dvbi=VBII.absolute();
	dbi=SBII.absolute();

	//geodetic longitude, latitude and altitude
	cad_geo84_in(lon,lat,alt,SBII,time);
	TDI=cad_tdi84(lon,lat,alt,time);
	TGI=cad_tgi84(lon,lat,alt,time);
	lonx=lon*DEG;
	latx=lat*DEG;
	//altitude in kft for diagnostics
	altx=0.001*alt*FOOT;

	//geographic velocity in geodetic axes VBED(3x1) and flight path angles
	VBED=TDI*(VBII-WEII*SBII);
	Matrix POLAR=VBED.pol_from_cart();
	dvbe=POLAR[0];
	psivdx=DEG*POLAR[1];
	thtvdx=DEG*POLAR[2];

	//T.M. of geographic velocity wrt geodetic coordinates
	TVD=mat2tr(psivdx*RAD,thtvdx*RAD);

	//diagnostics: acceleration achieved
	ayx=FSPB[1]/AGRAV;
	anx=-FSPB[2]/AGRAV;

	//ground track travelled (10% accuracy, usually on the high side)
	double vbed1=VBED[0];
	double vbed2=VBED[1];
	grndtrck+=sqrt(vbed1*vbed1+vbed2*vbed2)*int_step*REARTH/dbi;
	gndtrkmx=0.001*grndtrck;
	gndtrnmx=NMILES*grndtrck;

	//freeze variables for autopilot response calculations
	if(mfreeze==0)
		mfreeze_newt=0;
	else{
		if(mfreeze!=mfreeze_newt){
			mfreeze_newt=mfreeze;
			dvbef=dvbe;
		}
		dvbe=dvbef;
	}	
//-----------------------------------------------------------------------------
	//loading module-variables
	//state variables
	round6[235].gets_vec(SBII);
	round6[236].gets_vec(VBII);
	round6[237].gets_vec(ABII);
	//saving values
	round6[238].gets(grndtrck);
	round6[247].gets(mfreeze_newt);
	round6[248].gets(dvbef);
	//output to other modules
	round6[219].gets(lonx);
	round6[220].gets(latx);
	round6[221].gets(alt);
	round6[222].gets_mat(TVD);
	round6[223].gets_mat(TDI);
	round6[225].gets(dvbe);
	round6[226].gets(dvbi);
	round6[231].gets_mat(TGI);
	round6[232].gets_vec(VBED);
	round6[239].gets_vec(FSPB);
	//diagnostics
	round6[228].gets(psivdx);
	round6[229].gets(thtvdx);
	round6[230].gets(dbi);
	round6[234].gets(altx);
	round6[240].gets(ayx);
	round6[241].gets(anx);
	round6[242].gets(gndtrkmx);
	round6[243].gets(gndtrnmx);
}
