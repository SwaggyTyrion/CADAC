///////////////////////////////////////////////////////////////////////////////
//FILE: 'guidance.cpp'
//Contains 'guidance' module of class 'Hyper'
//
//030616 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of guidance module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[400-499]
//
//	   mguide = 30 line-guidance lateral, with maut 35 
//		      = 03 line-guidance in pitch 
//			  = 33 line-guidance lateral and in pitch, with maut 33
//			  =  4 arc-guidance lateral, with maut 24 or 25
//			  =  5 linear tangent guidance law (LTG) for rocket ascent   
//			  =  6 terminal pro-nav guidance (with seeeker or uplinked tracking)
//			  =  7 terminal advanced guidance (AGL) with RF seeker (set in 'seeker' module if 'mseek==4)
//			  =  8 glideslope rendezvous guidance with data link targeting 
//		
//030616 Created by Peter H Zipfel
//040330 Added LTG guidance, PZi
//040518 Added AGL guidance, PZi
//040907 Added glideslope guidance, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_guidance()
{
	//definition of module-variables
	//line guidance
	hyper[400].init("mguide","int",0,"Guidance modes, see table","guidance","data","");
	hyper[402].init("line_gain",0,"Line guidance gain - 1/s","guidance","data","");
	hyper[403].init("nl_gain_fact",0,"Nonlinear gain factor - ND","guidance","data","");
	hyper[404].init("decrement",0,"distance decrement - m","guidance","data","");
	hyper[405].init("wp_lonx",0,"Longitude of way point - deg","guidance","data","");
	hyper[406].init("wp_latx",0,"Latitude of way point - deg","guidance","data","");
	hyper[407].init("wp_alt",0,"Altitude of way point - m","guidance","data","");
	hyper[408].init("psifdx",0,"Heading line-of-attack angle - deg","guidance","data","");
	hyper[409].init("thtfdx",0,"Pitch line-of-attack angle - deg","guidance","data","");
	hyper[410].init("point_gain",0,"Point guidance gain - 1/s","guidance","data","");
	hyper[411].init("wp_sltrange",999999,"Range to waypoint - m","guidance","diag","");
	hyper[412].init("nl_gain",0,"Nonlinear gain - rad","guidance","diag","");
	hyper[413].init("VBEO",0,0,0,"Vehicle velocity in LOS coordinats - m/s","guidance","diag","");
	hyper[414].init("VBEF",0,0,0,"Vehicle velocity in LOA coordinats - m/s","guidance","diag","");
	hyper[415].init("wp_grdrange",999999,"Ground range to waypoint - m","guidance","diag","");
	hyper[416].init("SWBD",0,0,0,"Vehicle wrt waypoint/target in geo coor - m","guidance","out","");
	hyper[417].init("rad_min",0,"Minimum arc radius, calculated - m","guidance","diag","");
	hyper[418].init("rad_geometric",0,"Geometric arc radius, calculated - m","guidance","diag","");
	hyper[419].init("wp_flag","int",0,"=1:closing on target; =-1:fleeting; =0:outside - ND","guidance","diag","");
	//basic pro-nav
	hyper[420].init("gnav",0,"Nav gain for pro-nav - ND","guidance","data","");
	hyper[421].init("aycomx",0,"Side acceleration command - g's","guidance","out","plot");
	hyper[422].init("azcomx",0,"Down acceleration command - g's","guidance","out","plot");
	hyper[423].init("tgoc",0,"Time-to-go using uplink coordinates - s","guidance","diag","plot");
	hyper[424].init("dtbc",0,"Range-to-go using uplink coordinates - m","guidance","diag","");
	hyper[425].init("psiobcx",0,"Yaw angle of LOS wrt body frame - deg","guidance","diag","");
	hyper[426].init("thtobcx",0,"Pitch angle of LOS wrt body frame - deg","guidance","diag","");
	hyper[427].init("SBTHC",0,0,0,"Vehicle position in Hill coordinates using INS - m","guidance","diag","plot");
	//LTG guidance
	hyper[428].init("init_flag","int",1,"Flag for initializing LTG flags - ND","guidance","init","");
	hyper[429].init("time_ltg",0,"Time since initiating LTG - s","guidance","diag","");
	hyper[430].init("UTBC",0,0,0,"Commanded unit thrust vector in body coor - ND","guidance","out","plot");
	hyper[431].init("RBIAS",0,0,0,"Range-to-be-gained bias - m","guidance","save","");
	hyper[432].init("beco_flag","int",0,"Boost engine cut-off flag - ND","guidance","diag","");
	hyper[434].init("inisw_flag","int",1,"Flag to initialize '..._intl()' - ND","guidance","init","");
	hyper[435].init("skip_flag","int",1,"Flag to delay output - ND","guidance","init","");
	hyper[436].init("ipas_flag","int",1,"Flag to initialize in '..._tgo()'  - ND","guidance","init","");
	hyper[437].init("ipas2_flag","int",1,"Flag to initialize in '..._trat()'  - ND","guidance","init","");
	hyper[438].init("print_flag","int",1,"Flag to cause print-out  - ND","guidance","init","");
	hyper[439].init("ltg_count","int",0,"Counter of LTG guidance cycles  - ND","guidance","save","");
	hyper[440].init("ltg_step",0,"LTG guidance time step - s","guidance","data","");
	hyper[441].init("dbi_desired",0,"Desired orbital end position - m","guidance","data","");
	hyper[442].init("dvbi_desired",0,"Desired orbital end velocity - m/s","guidance","data","");
	hyper[443].init("thtvdx_desired",0,"Desired orbital flight path angle - deg","guidance","data","");
	hyper[444].init("num_stages","int",0,"Number of stages in boost phase - s","guidance","data","");
	hyper[445].init("delay_ignition",0,"Delay of motor ignition after staging - s","guidance","data","");
	hyper[446].init("amin",0,"Minimum longitudinal acceleration - m/s^2","guidance","data","");

	hyper[450].init("char_time1",0,"Characteristic time 'tau' of stage 1 - s","guidance","data","");
	hyper[451].init("char_time2",0,"Characteristic time 'tau' of stage 2 - s","guidance","data","");
	hyper[452].init("char_time3",0,"Characteristic time 'tau' of stage 3 - s","guidance","data","");
	hyper[453].init("exhaust_vel1",0,"Exhaust velocity of stage 1 - m/s","guidance","data","");
	hyper[454].init("exhaust_vel2",0,"Exhaust velocity of stage 2 - m/s","guidance","data","");
	hyper[455].init("exhaust_vel3",0,"Exhaust velocity of stage 3 - m/s","guidance","data","");
	hyper[456].init("burnout_epoch1",0,"Burn out of stage 1 at 'time_ltg' - s","guidance","data","");
	hyper[457].init("burnout_epoch2",0,"Burn out of stage 2 at 'time_ltg' - s","guidance","data","");
	hyper[458].init("burnout_epoch3",0,"Burn out of stage 3 at 'time_ltg' - s","guidance","data","");
	hyper[459].init("lamd_limit",0,"Limiter on 'lamd' - 1/s","guidance","data","");
	hyper[460].init("RGRAV",0,0,0,"Postion loss due to gravity - m ","guidance","save","");
	hyper[461].init("RGO",0,0,0,"Range-to-go vector - m ","guidance","save","");
	hyper[462].init("VGO",0,0,0,"Velocity still to be gained - m/s","guidance","save","");
	hyper[463].init("SDII",0,0,0,"Desired inertial position - m","guidance","save","");
	hyper[464].init("UD",0,0,0,"Unit vector of SPII and SDII - ND","guidance","save","");
	hyper[465].init("UY",0,0,0,"Unit vector normal to traj plane - ND","guidance","save","");
	hyper[466].init("UZ",0,0,0,"Unit vector in traj plane, normal to SBBI_D - ND","guidance","save","");
	hyper[467].init("vgom",0,"Velocity to be gained magnitude - m/s","guidance","diag","");
	hyper[468].init("tgo",0,"Time to go to desired end state - s","guidance","save","");
	hyper[469].init("nst","int",0,"N-th stage number","guidance","save","");
	hyper[470].init("ULAM",0,0,0,"Unit thrust vector in VGO direction - ND","guidance","diag","");
	hyper[471].init("LAMD",0,0,0,"Thrust vector turning rate - 1/s","guidance","diag","");
	hyper[472].init("isp_fuel",0,"Specific impulse of thrusting motor - s","guidance","out","");
	hyper[473].init("burntime",0,"Total burntime of thrusting motor - s","guidance","out","");
	hyper[474].init("nstmax","int",0,"//# of stages needed to meet end state - ND","guidance","diag","");
	hyper[475].init("lamd",0,"Magnitude of LAMD - 1/s","guidance","diag","plot");
	hyper[476].init("dpd",0,"Distance of the predicted from the desired end-point - m","guidance","diag","");
	hyper[477].init("dbd",0,"Distance of vehicle from the desired end-point - m","guidance","diag","");
	//advanced guidance law
	hyper[478].init("gnavpn",0,"Nav gain for AGL pro-nav term - ND","guidance","data","");
	hyper[479].init("gnavps",0,"Nav gain for AGL pursuit term - ND","guidance","data","");
	//glideslope rendez-vous guidance
	hyper[480].init("gs_flag","int",1,"Flag for initializing glideslope guidance - ND","guidance","init","");
	hyper[481].init("time_gs",0,"Total time  on glideslope - s","guidance","data","");
	hyper[482].init("num_burns","int",0,"Number of firings on glideslope - ND","guidance","data","");
	hyper[483].init("closing_rate",0,"Terminal closing rate (negative for closing) - m/s","guidance","data","");
	hyper[484].init("orbital_rate",0,"Target orbital rate (circular orbit) - rad/s","guidance","data","");
	hyper[485].init("satl1",0,"Aimpoint x-component in downrange - m","guidance","data","");
	hyper[486].init("satl2",0,"Aimpoint y-component in crossrange - m","guidance","data","");
	hyper[487].init("satl3",0,"Aimpoint z-component towards Earth center - m","guidance","data","");
	hyper[488].init("dtime_gs",0,"Pulse interval between firings on glideslope - s","guidance","save","");
	hyper[489].init("length_gs",0,"Length of glideslope - m","guidance","save","");
	hyper[490].init("para_gs",0,"Decrement parameter of glideslope - 1/s","guidance","save","");
	hyper[491].init("UB0AL",0,0,0,"Initial glideslope unit vector in local coord - ND","guidance","save","");
	hyper[492].init("epoch_gs",0,"Epoch of when burn is initiated - s","guidance","save","");
	hyper[493].init("counter_gs","int",1,"Counter for number of firings  - ND","guidance","save","");
	hyper[494].init("VBTLM",0,0,0,"Relative velocity before burn - m/s","guidance","save","");
	hyper[495].init("DELTA_V",0,0,0,"Delta-v vector to be achieved by burn - m/s","guidance","save","");
	hyper[496].init("burn_flag","int",1,"Flag set while motor is burning - ND","guidance","save","");
	hyper[497].init("SBTL",0,0,0,"Vehicle position in local level coordinates - m","guidance","diag","plot");
	hyper[498].init("VBTL",0,0,0,"Vehicle velocity in local level coordinates - m","guidance","diag","");
	hyper[499].init("delta_v",0,"Delta_v magnitude to-be-achieved - m/s","guidance","diag","");

}
///////////////////////////////////////////////////////////////////////////////  
//Guidance module
//Member function of class 'Hyper'
//
//	   mguide = 30 line-guidance lateral, with maut 35 
//		      = 03 line-guidance in pitch 
//			  = 33 line-guidance lateral and in pitch, with maut 33
//			  =  4 arc-guidance lateral, with maut 24 or 25
//			  =  5 linear tangent guidance law (LTG) for rocket ascent   
//			  =  6 terminal pro-nav guidance (with seeeker or uplinked tracking)
//			  =  7 terminal advanced guidance (AGL) with RF seeker (set in 'seeker' module if 'mseek==4)
//			  =  8 glideslope rendezvous guidance with data link targeting 
//		
//030616 Created by Peter H Zipfel
//040330 Added LTG guidance, PZi
//040518 Added AGL guidance, PZi
//////////////////////////////////////////////////////////////////////////////
	
void Hyper::guidance(double int_step)
{
	//local variables
	Matrix ALGV(3,1);
	Matrix ACOMX(3,1);
	Matrix UTBBC(3,1);

	//local module-variables
	double phicomx(0);
	double ancomx(0);
	double alcomx(0);
	double aycomx(0);
	double azcomx(0);

	//localizing module-variables
	//input data
	int mguide=hyper[400].integer();
	int maut=hyper[500].integer();
	double wp_lonx=hyper[405].real();
	double wp_latx=hyper[406].real();
	double wp_alt=hyper[407].real();
	double psifdx=hyper[408].real();
	double thtfdx=hyper[409].real();
	double ltg_step=hyper[440].real();
	int gs_flag=hyper[480].integer();
	//restore saved variable
	int init_flag=hyper[428].integer();
	double time_ltg=hyper[429].real();
	Matrix UTBC=hyper[430].vec();
	int ltg_count=hyper[439].integer();
	//input from other modules
	double time=round6[0].real(); //for diagnostics only
	double grav=round6[63].real();
	int mprop=hyper[10].integer();
	int mseek=hyper[200].integer();
	Matrix STBIK=hyper[270].vec();
	Matrix VTBIK=hyper[271].vec();
	Matrix TBIC=hyper[315].mat();
	//-------------------------------------------------------------------------
	//returning,if no guidance
	if(mguide==0){
		alcomx=0;
		ancomx=0;
		UTBC.zero();
		return;
	}
	//lateral line guidance
	if(mguide==30){
		ALGV=guidance_line(wp_lonx,wp_latx,wp_alt,psifdx,thtfdx);
		alcomx=ALGV.get_loc(1,0)/grav;
	}
	//pitch line guidance
	if(mguide==3){
		ALGV=guidance_line(wp_lonx,wp_latx,wp_alt,psifdx,thtfdx);
		alcomx=0;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//line guidance lateral and pitch
	if(mguide==33){
		ALGV=guidance_line(wp_lonx,wp_latx,wp_alt,psifdx,thtfdx);
		alcomx=ALGV.get_loc(1,0)/grav;
		ancomx=-ALGV.get_loc(2,0)/grav;
	}
	//arc guidance lateral
	if(mguide==4){
		phicomx=guidance_arc(wp_lonx,wp_latx,wp_alt);
	}
	//LTG guidance, starting LTG clock
	if(mguide==5){
		//start LTG clock 
		if(init_flag){
			init_flag=0;
			time_ltg=0;
		}else{
			time_ltg+=int_step;
		}
		//calling LTG guidance every 'int(ltg_step/int_step)' times 
		ltg_count++;
		int ratio=(int)(ltg_step/int_step);
		//using the 'MOD' function of FORTRAN spelled out
		int ltg_flag= ltg_count-(int)(ltg_count/ratio)*ratio;
		if(!ltg_flag){
			Matrix UTIC=guidance_ltg(mprop,int_step,time_ltg);
			UTBC=TBIC*UTIC;
		}
	}
	//terminal pro-nav guidance
	if(mguide==6){
		ACOMX=guidance_pronav(UTBBC, STBIK,VTBIK);
		aycomx=ACOMX.get_loc(1,0);
		azcomx=ACOMX.get_loc(2,0);
		UTBC=UTBBC;
	}
	//terminal guidance AGL (Advanced Guidance Law) 
	if(mguide==7){
		ACOMX=guidance_AGL(UTBBC, STBIK,VTBIK);
		aycomx=ACOMX.get_loc(1,0);
		azcomx=ACOMX.get_loc(2,0);
		UTBC=UTBBC;
	}
	//glideslope rendez-vous guidance with data link targeting
	if(mguide==8){
		UTBC=guidance_glideslope(mprop,gs_flag,mseek);
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving variable
	hyper[428].gets(init_flag);
	hyper[429].gets(time_ltg);
	hyper[439].gets(ltg_count);
	hyper[480].gets(gs_flag);
	//output to other modules
	hyper[10].gets(mprop);
	hyper[421].gets(aycomx);
	hyper[422].gets(azcomx);
	hyper[430].gets_vec(UTBC);
	hyper[522].gets(alcomx);
	hyper[523].gets(ancomx);
	hyper[553].gets(phicomx);
	//diagnostics
}
///////////////////////////////////////////////////////////////////////////////
//Linear tangent guidance (LTG) for ascent
//mguide=1   
//   
//Ref: Jaggers, R.F."Multistage Linear Tangent Ascent Guidance
//     as Baselined for the Space Shuttle Vehicle", NASA MSC,
//     Internal Note MSC-EG-72-39, June 1972
//
//Uses optimal linear tangent guidance law for up to 3 stages   
//of constant rocket thrust   
//   
//Return output:   
//          UTIC(3) = unit thrust vector command in inertial coor. - ND
//Parameter input
//			int_step = integration step
//			time_ltg = time elapsed since start of LTG (mguide=1) 
//Nomenclature
//		Capitalized variables are 3x1 vectors, unless a capital 
//		 'N' is appended then the array is used to store information for each 
//		 of the n stages (max n=3)
//  
//040319 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_ltg(int &mprop,double int_step,double time_ltg)
{
	//local variables
	//Parameter output 'guidance_ltg_crct()'
	Matrix VMISS(3,1);	//velocity miss - m/s
	//Parameter input 'guidance_ltg_crct()'
	Matrix SPII(3,1);		//predicted inertial position vector - m
	Matrix VPII(3,1);		//predicted inertial velocity vector - m/s			
	//Paramter output of 'guidance_ltg_tgo()' 
	double tgop(0);		//time-to-go of previous computation cycle - s
	Matrix BURNTN(3,1);//burn time duration left in n-th stage -> used in '_igrl()' - s
	Matrix L_IGRLN(3,1); //velocity to-be-gained by n-th stage at 'time_ltg' -> used in '_igrl()' - m/s
	Matrix TGON(3,1);	//burn durations remaining, summed over n-th and previous stages, towards reaching end-state - s
	double l_igrl(0);	//L-integral (velocity to be gained - m/s)
	int nstmax(0);		//maximum # of stages needed , used in '_igrl()'
	//Parameter input/output of 'guidance_ltg_tgo()' 
	Matrix TAUN(3,1);	//characteristic time of stages - s
	//Parameter input of 'guidance_ltg_tgo()'
	Matrix VEXN(3,1);	//exhaust velocity of n-th stage - m/s
	 //burn out time of stages - s
	Matrix BOTN(4,1);	//burn-out time epoch of n-th stage, in 'time_ltg' clock time - s
					    //(0-th stage is dummy stage with BOTN[0]=0, has dimension BOTN(4,1))
	double vgom(0);		//desired velocity to be gained (absolute value) - m/s
	double amag1(0);	//absolute value of current vehicle acceleration - m/s^2
	//Parameter output 'guidance_ltg_igrl()'
	double s_igrl(0);	//thrust integral, postion - m   
	double j_igrl(0);	//thrust integral, velocit*time - m   
	double q_igrl(0);	//thrust integral, position*time - m*s   
	double h_igrl(0);	//thrust integral, velocity*time^2 - m*s   
	double p_igrl(0);	//thrust integral, position*time^2 - m*s^2
	double j_over_l(0);	//j_over_l= time remaining - s
	double tlam(0);		//time of thrust integraton - s 
	double qprime(0);	//conditioned q-integral - m*s
	//Parameter output 'guidance_ltg_trate()'
	Matrix ULAM(3,1);	//unit thrust vector in direction of VGO - ND
	Matrix LAMD(3,1);	//derivative of unit thrust vector - 1/s
	
	//local module-variables
	Matrix UTIC(3,1);
	double isp_fuel(0);
	double burntime(0);

	//localizing module-variables
	//input data
	double ltg_step=hyper[440].real();
	double dbi_desired=hyper[441].real();
	double dvbi_desired=hyper[442].real();
	double thtvdx_desired=hyper[443].real();
	int num_stages=hyper[444].integer();
	double delay_ignition=hyper[445].real();
	double amin=hyper[446].real();
	double char_time1=hyper[450].real();
	double char_time2=hyper[451].real();
	double char_time3=hyper[452].real();
	double exhaust_vel1=hyper[453].real();
	double exhaust_vel2=hyper[454].real();
	double exhaust_vel3=hyper[455].real();
	double burnout_epoch1=hyper[456].real();
	double burnout_epoch2=hyper[457].real();
	double burnout_epoch3=hyper[458].real();
	double lamd_limit=hyper[459].real();
	//initialized data
	int init_flag=hyper[428].integer();
	int beco_flag=hyper[432].integer();
	int inisw_flag=hyper[434].integer();
	int skip_flag=hyper[435].integer();
	int ipas2_flag=hyper[437].integer();
	int print_flag=hyper[438].integer();
	//restoring saved variables
	Matrix RBIAS=hyper[431].vec();
	Matrix RGRAV=hyper[460].vec();
	Matrix RGO=hyper[461].vec();
	Matrix VGO=hyper[462].vec();
	Matrix SDII=hyper[463].vec();
	Matrix UD=hyper[464].vec();
	Matrix UY=hyper[465].vec();
	Matrix UZ=hyper[466].vec();
	double tgo=hyper[468].real();
	int nst=hyper[469].integer();
	//input from other modules
	double time=round6[0].real(); 
	double grav=round6[63].real();
	double dbi=round6[230].real();
	double dvbi=round6[226].real();
	double lonx=round6[219].real();
	double latx=round6[220].real();
	double alt=round6[221].real();
	double dvbe=round6[225].real();
	double psivdx=round6[228].real();
	double thtvdx=round6[229].real();
	Matrix VBIIC=hyper[303].vec();
	Matrix SBIIC=hyper[304].vec();
	Matrix TBIC=hyper[315].mat();
	Matrix FSPCB=hyper[334].vec();
	//-------------------------------------------------------------------------	
	//preparing necessary variables
	Matrix ABII=~TBIC*FSPCB;
	amag1=ABII.absolute();
		
	//initializing
	if(inisw_flag){
		inisw_flag=0;

		//initializing predicted state to current state
		SPII=SBIIC;
		VPII=VBIIC;

		//calling corrector for initialization
		guidance_ltg_crct(SDII,UD,UY,UZ,VMISS //output
						  ,VGO		//input-output
						  ,dbi_desired,dvbi_desired,thtvdx_desired,SPII,VPII,SBIIC,VBIIC); //input
	}
	else
		//updating velocity to go	
		VGO=VGO-ABII*ltg_step;

	//velocity-to-go magnitude
	vgom=VGO.absolute();

	//building data vector of the stages
	TAUN.build_vec3(char_time1,char_time2,char_time3);
	VEXN.build_vec3(exhaust_vel1,exhaust_vel2,exhaust_vel3);

	//building array of burn-out time epochs of n-th stage, in 'time_ltg' clock time - s
	//burn-out epoch vector starts with dummy o-th stage then stages 1,2,3
	BOTN[0]=0;
	BOTN[1]=burnout_epoch1;
	BOTN[2]=burnout_epoch2;
	BOTN[3]=burnout_epoch3;

	//calling time-to-go function
	guidance_ltg_tgo(tgop,BURNTN,L_IGRLN,TGON,l_igrl,nstmax		//output							 
					 ,tgo,nst,TAUN				//input/output
					 ,VEXN,BOTN,delay_ignition,vgom,amag1,amin,time_ltg,num_stages);	//input
	//calling thrust integral function							 				
	guidance_ltg_igrl(s_igrl,j_igrl,q_igrl,h_igrl,p_igrl,j_over_l,tlam,qprime	//output							  
					  ,nst,nstmax,BURNTN,L_IGRLN,TGON,TAUN,VEXN,l_igrl,time_ltg);	//input
	//calling turning rate function							  
	guidance_ltg_trate(ULAM,LAMD,RGO		//output
					  ,ipas2_flag		//input-output
					  ,VGO,s_igrl,q_igrl,j_over_l,lamd_limit,vgom,time_ltg	//input					  
					  ,tgo,tgop,SDII,SBIIC,VBIIC,RBIAS,UD,UY,UZ,RGRAV);		//throughput to '_rtgo(()'									  	
					 
	//calculating thrust command vector
	Matrix TC=ULAM+LAMD*(time_ltg-tlam); //same as: TC=ULAM+LAMD*(-j_over_l)

	//calculating output thrust unit vector after skipping 10 'guid_step' delay (settling of transients)
	if(skip_flag){
		skip_flag++;
		if(skip_flag==10)
			skip_flag=0;
	}
	else
		UTIC=TC.univec3();

	//calling end-state predictor and corrector
	guidance_ltg_pdct(SPII,VPII,RGRAV,RBIAS		//output
					 ,LAMD,ULAM,l_igrl,s_igrl,j_igrl,q_igrl,h_igrl,p_igrl,j_over_l,qprime	//input
					 ,SBIIC,VBIIC,RGO,tgo);
	guidance_ltg_crct(SDII,UD,UY,UZ,VMISS	//output
					  ,VGO		//input-output
					  ,dbi_desired,dvbi_desired,thtvdx_desired,SPII,VPII,SBIIC,VBIIC);  //input

	//calculating specific impulse and burn duration of rocket motor in stage (used in propulsion module)
	isp_fuel=VEXN[nst-1]/AGRAV;
	burntime=BOTN[nst]-BOTN[nst-1]-delay_ignition;

	//igniting motor
	if(burntime>0)
		mprop=3;

	//cut-off logic
	if(tgo<10*int_step){
		beco_flag=1;
		mprop=0;
	}
	//boost engine cut-off print-out
	if(beco_flag&&print_flag){
		print_flag=0;
		cout<<" *** Boost engine cut-off time = "<<time<<" sec ***\n";
		cout<<"     Orbital position dbi = "<<dbi<<" m \tInertial speed dvbi = "<<dvbi<<" m/s \tFlight path angle thtvdx = "<<thtvdx<<" deg\n";
		cout<<"     Position error   ddb = "<<dbi_desired-dbi<<" m \t\tSpeed error    dvdb = "<<dvbi_desired-dvbi
			<<" m/s\tAngle error     thtvddbx = "<<thtvdx_desired-thtvdx<<" deg\n";
		cout<<"     Geodetic        lonx = "<<lonx<<" deg \tlatx = "<<latx<<" deg \talt = "<<alt<<" m\tpsivdx = "<<psivdx<<" deg \tspeed = "<<dvbe<<" m/s\n"; 
	}
	//-------------------------------------------------------------------------
	//loading diagnostic module-variables
	//output to other modules
	hyper[472].gets(isp_fuel);
	hyper[473].gets(burntime);
	//saving variables
	hyper[431].gets_vec(RBIAS);
	hyper[432].gets(beco_flag);
	hyper[434].gets(inisw_flag);
	hyper[435].gets(skip_flag);
	hyper[437].gets(ipas2_flag);
	hyper[438].gets(print_flag);
	hyper[460].gets_vec(RGRAV);
	hyper[461].gets_vec(RGO);
	hyper[462].gets_vec(VGO);
	hyper[463].gets_vec(SDII);
	hyper[464].gets_vec(UD);
	hyper[465].gets_vec(UY);
	hyper[466].gets_vec(UZ);
	hyper[468].gets(tgo);
	hyper[469].gets(nst);
	//diagnostics
	hyper[467].gets(vgom);
	hyper[470].gets_vec(ULAM);
	hyper[471].gets_vec(LAMD);
	hyper[474].gets(nstmax);
	
	//returning unit thrust vector command
	return UTIC;
}

///////////////////////////////////////////////////////////////////////////////
//Time-to-go to desired end-state
//
//Parameter output:   
//			tgop = time-to-go of previous computation cycle - s
//			BURNTN(3) = burn time duration left in n-th stage -> used in '_igrl()' - s
//			L_IGRLN(3) = velocity to-be-gained by n-th stage at 'time_ltg' -> used in '_igrl()' - m/s
//			TGON(3) = burn durations remaining, summed over n-th and previous stages, towards reaching end-state - s
//			l_igrl = L-integral (velocity-to-be-gained by remaining stages - m/s)
//			nstmax = maximum # of stages needed , used in '_igrl()'
//Parameter input/output:
//			tgo = time-to-go to desired end-state - s
//			nst = n-th stage number -> used in '_igrl()'
//		`	TAUN(3) = characteristic time of n-th stage - s
//Parameter input:
//			VEXN(3) = exhaust velocity of n-th stage - m/s
//		`	BOTN(4) = burn-out time epoch of n-th stage, in 'time_ltg' clock time - s
//					 (0-th stage is dummy stage with BOTN[0]=0, has dimension BOTN(4,1))
//			delay_ignition = delay of booster ignition after stating (input) -s
//			vgom = desired velocity to be gained (absolute value) - m/s
//			amag1 = absolute value of current vehicle acceleration - m/s^2
//			amin = minimum accleration for TAUN calcualtions (input) - m/s^2
//			time_ltg = time clock started when LTG is engaged -s
//			num_stages = number of stages (initialized in input)
//
//040322 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::guidance_ltg_tgo(double &tgop,Matrix &BURNTN,Matrix &L_IGRLN,Matrix &TGON,double &l_igrl,int &nstmax							 
							 ,double &tgo,int &nst,Matrix &TAUN
							 ,Matrix VEXN,Matrix BOTN,double delay_ignition,double vgom
							 ,double amag1,double amin,double time_ltg,int num_stages)
{
	//local variables
	int i(0);

	//localizing module-variables
	//restoring variables
	int ipas_flag=hyper[436].integer();

	//-------------------------------------------------------------------------
	//initializing to first stage
	if(ipas_flag)
		nst=1;
	//save previous time-to-go for use in '_rtgo()' ( initially: tgo=0)
	tgop=tgo;

	//initialize for-loop
	tgo=0;
	l_igrl=0;
	nstmax=num_stages;

	//advancing stage counter (outside for-loop, stage counter is nst=1,2,3)
	if((time_ltg>=BOTN[nst]))
		nst++;

	//looping through remaining stages (inside for-loop, stage counter is i=0,1,2)
	for(i=nst-1;i<nstmax;i++){  

		//if current stage, re-calculating characteristic time
		if(i==(nst-1))
			TAUN[nst-1]=TAUN[nst-1]-(time_ltg-BOTN[nst-1]); //first stage: no change (BOTN[0]=0)

		//if sufficient accel.and ignition of stage has occured then re-calculate 
		if((amag1>=amin)&&(time_ltg >(BOTN[nst-1]+delay_ignition)))
			TAUN[nst-1]=VEXN[nst-1]*(1/amag1); // re-calculating 'tau' using current acceleration

		//calculating remaining boost times
		if(i==(nst-1))
			//remaining burn time left in current stage
			BURNTN[i]=BOTN[i+1]-time_ltg;
		else
			//burn time in other stages (assuming 'delay_ignition'=0)			
			BURNTN[i]=BOTN[i+1]-BOTN[i];
		
		//calculating velocity gained of (n=i+1)-th stage ('L_IGRLN[i]')
		double dum1=TAUN[i];
		double dum2=BURNTN[i];
		L_IGRLN[i]=-VEXN[i]*log(1-dum2/dum1);

		//accumulating velocity to be gained for the remainder of the boost phase 
		l_igrl+=L_IGRLN[i];
		//if velocity-to-be-gained is less than desired
		if(l_igrl<vgom){

			//update tgo for remaining burn times left
			tgo+=BURNTN[i];

			//load into array: time to reach end-state for each stage (i=0,1,2)
			TGON[i]=tgo;
		}else{
			i++;
			break;
		}
	}
	//recomputing the velocity-to-be-gained to adjust to vgom
	nstmax=i; //After an ordinary or 'break' termination of the for-loop, 'i' is increased by 'one', 
				  // and is now equal to the number of stages needed to meet the end-state conditons.
				  // However, for the following arrays, where 'i' is the offset index,it must be decreased
				  // by 'one' to designate the last required stage 

	//subtracting from velocity-to-be-gained integral the last stage value 
	l_igrl=l_igrl-L_IGRLN[i-1];

	//re-calculating last stage boost time 
	double almx=vgom-l_igrl;
	L_IGRLN[i-1]=almx;
	double dum3=VEXN[i-1];
	BURNTN[i-1]=TAUN[i-1]*(1-exp(-almx/dum3));

	//updating tgo
	tgo+=BURNTN[i-1];  //accumulating burn intervals remaining until end-state
	TGON[i-1]=tgo; //loading burn duration left 

	//setting integral 'l_igrl' to desired velocity-to-be-gained
	l_igrl=vgom;

	if(ipas_flag){
		//initializing 'tgop', to be used in '_rtgo()'
		tgop=tgo; 
		//canceling initialization flag
		ipas_flag=0;
	}
	//-------------------------------------------------------------------------
	//saving variables
	hyper[436].gets(ipas_flag);
}
///////////////////////////////////////////////////////////////////////////////
//LTG thrust integrals
//
//Parameter output:
//			s_igrl = thrust integral, postion - m   
//			j_igrl = thrust integral, velocit*time - m   
//			q_igrl = thrust integral, position*time - m*s   
//			h_igrl = thrust integral, velocity*time^2 - m*s   
//			p_igrl = thrust integral, position*time^2 - m*s^2
//			j_over_l= time remaining - s
//			tlam = time of thrust integration - s 
//			qprime = conditioned q-integral - m*s
//Parameter input:
//			nst = n-th stage number - ND
//			num_stages = number of stages (input) - ND
//			BURNTN(3) = burn time duration left in n-th stage - s
//			L_IGRLN(3) = velocity to-be-gained by n-th stage at 'time_ltg' - m/s
//			TGON(3) = burn durations remaining, summed over n-th and previous stages, towards reaching end-state - s
//		`	TAUN(3) = characteristic time of n-th stage - s
//			VEXN(3) = exhaust velocity of n-th stage - m/s
//			l_igrl = L-integral (velocity to be gained - m/s)
//			time_ltg = time clock started when LTG is engaged -s
//
//040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::guidance_ltg_igrl(double &s_igrl,double &j_igrl,double &q_igrl,double &h_igrl
							  ,double &p_igrl,double &j_over_l,double &tlam,double &qprime
							  ,int nst,int nstmax,Matrix BURNTN,Matrix L_IGRLN,Matrix TGON
							  ,Matrix TAUN,Matrix VEXN,double l_igrl,double time_ltg)							  
{
	//local variables
	double ls_igrl(0);

	//calculating integrals for remaining boost stages
	for(int i=nst-1;i<nstmax;i++){
		double tb=BURNTN[i];
		double tga=TGON[i];
		double x=tb/TAUN[i];
		double a1(0);
		if(x==2)
			a1=1/(1-0.5*x*(1.001));//factor 1.001 used to prevent division by zero
		else
			a1=1/(1-0.5*x);
		double a2(0);
		if(x==1){
			//x=1 occurs when in '_tgo()': BURNTN[i-1]=TAUN[i-1]*(1-exp(-almx/dum3)); the exponent is very large
			//  and therefore BURNTN[i-1]=TAUN[i-1] -> caused by non-convergence of solution
			cerr<<" *** LTG Terminator: end-state cannot be reached *** \n";system("pause");exit(1);
		} 
		else
			a2=1/(1-x);
		double aa=VEXN[i]/TAUN[i];//longitudinal acceleration of booster - m's^2
		double ll_igrl=L_IGRLN[i];
		double a1x=4*a1-a2-3;
		double a2xsq=2*a2-4*a1+2;
		double sa=(aa*tb*tb/2)*(1+a1x/3+a2xsq/6);
		double ha=(aa*tb*tb*tb/3)*(1.+a1x*0.75+a2xsq*0.6);
		double ja=(aa*tb*tb/2)*(1+a1x*(2/3)+a2xsq/2);
		double qa=(aa*tb*tb*tb/6)*(1+a1x/2+a2xsq*0.3);
		double pa=(aa*tb*tb*tb*tb/12)*(1+a1x*0.6+a2xsq*0.4);

		//if not the current stage
		if(i!=nst-1){
			double t1=TGON[i-1];
			ha=ha+2*t1*ja+t1*t1*ll_igrl;
			ja=ja+t1*ll_igrl;
			pa=pa+2*t1*qa+t1*t1*sa;
			qa=qa+t1*sa;
		}
		ha=ja*tga-qa;
		sa=sa+ls_igrl*tb;
		qa=qa+j_igrl*tb; 
		pa=pa+h_igrl*tb;
		s_igrl=s_igrl+sa;
		q_igrl=q_igrl+qa;
		p_igrl=p_igrl+pa;
		h_igrl=h_igrl+ha;
		ls_igrl=ls_igrl+ll_igrl;
		j_igrl=j_igrl+ja;  
	}
	j_over_l=j_igrl/l_igrl;
	tlam=time_ltg+j_over_l; 
	qprime=q_igrl-s_igrl*j_over_l;
}
///////////////////////////////////////////////////////////////////////////////
//Turning rate and thrust vector calculations
//
//Parameter output:
//			ULAM(3) = unit thrust vector in direction of VGO - ND
//			LAMD(3) = derivative of unit thrust vector - 1/s
//			RGO(3) = range-to-go - m
//Parameter input/output:
//			ipas2_flag = initialization flag - ND
//Parameter input:
//			VGO(3) = velocity still to be gained - m/s 
//			ipas2_flag = initialization flag - ND   
//			s_igrl = thrust integral, postion - m
//			q_igrl = thrust integral, position*time - m*s   
//			j_over_l = time remaining - s
//			lamd_limit = LAMD components cannot exceed this limit (input) - 1/s
//			vgom = velocity to be gained - m/s
//			time_ltg = time clock started when LTG is engaged -s
//Parameter throughput to 'guidance_ltg_trate_rtgo(()':
//			tgo = time-to-go - s
//			tgop = previous time-to-go calculation - s
//			SDII(3) = desired range, defined in '_crct()' - m
//			SBIIC(3) = INS vehicle inertial position - m 
//			VBIIC(3) = INS vehicle inertial velosicy - m/s
//			RBIAS(3) = position bias vector, calculated in '_pdct()' - m 
//			UD(3) = unit vector of SPII and SDII - ND 
//			UY(3) = unit vector normal to trajectory plane - ND
//			UZ(3) = unit vector in traj plane, normal to desired inertial pos - ND
//Parameter modified in 'guidance_ltg_trate_rtgo(()':
//			RGRAV(3) = Postion loss due to gravity (modified in '_rtgo()') - m 
//
//040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::guidance_ltg_trate(Matrix &ULAM,Matrix &LAMD,Matrix &RGO	
							  ,int &ipas2_flag	
							  ,Matrix VGO,double s_igrl,double q_igrl	
							  ,double j_over_l,double lamd_limit,double vgom,double time_ltg
							  ,double tgo, double tgop,Matrix SDII,Matrix SBIIC,Matrix VBIIC,Matrix RBIAS
							  ,Matrix UD,Matrix UY,Matrix UZ,Matrix &RGRAV)
{
	//local module variables
	double lamd(0);
	//-------------------------------------------------------------------------
	//return if velocity-to-be-gained is zero
	if(vgom==0) return;

	//unit thrust vector in VGO direction
	ULAM=VGO.univec3();
	LAMD.zero();

	//initializing RGO at first pass
	if(ipas2_flag){
		ipas2_flag=0;
		RGO=ULAM*s_igrl; 
	}
	//calling function to get range-to-go (SBII_GO)
	guidance_ltg_trate_rtgo(RGO		//output
							,RGRAV //input-output
							,tgo,tgop,SDII,SBIIC,VBIIC,RBIAS,ULAM,UD,UY,UZ,s_igrl);	//input
	double denom=(q_igrl-s_igrl*j_over_l);
	if(denom!=0)
		LAMD=(RGO-ULAM*s_igrl)*(1/denom);
	else
		LAMD.zero();

	//setting limits on LAMD
	lamd=LAMD.absolute();
	if(lamd>=lamd_limit){
		Matrix ULMD=LAMD.univec3();
		LAMD=ULMD*lamd_limit;
	}
	//diagnostics:
	lamd=LAMD.absolute();
	//-------------------------------------------------------------------------
	//diagnostics
	hyper[475].gets(lamd);
}
///////////////////////////////////////////////////////////////////////////////
//Range-to-go calculation, called from 'guidance_ltg_trate()'
// * downrange is left free 
// * corrects for gravity effects and adds bias corrections 
//
//Parameter output:
//			RGO(3) = range-to-go vector - m
//Parameter input/output:
//			RGRAV(3) = Postion loss due to gravity - m 
//Parameter input:
//			tgo = time-to-go - s
//			tgop = previous time-to-go calculation - s
//			VGO(3) = velocity to be achieved - m/s
//			SDII(3) = desired range, defined in '_crct()' - m
//			SBIIC(3) = INS vehicle inertial position - m 
//			VBIIC(3) = INS vehicle inertial velosicy - m/s
//			RBIAS(3) = position bias vector, calculated in '_pdct()' - m 
//			ULAM(3) = unit thrust vector - ND
//			UD(3) = unit vector of SPII and SDII - ND 
//			UY(3) = unit vector normal to trajectory plane - ND
//			UZ(3) = unit vector in traj plane, normal to desired inertial pos - ND
//			s_igrl = thrust integral, postion - m   
//
//040326 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void  Hyper::guidance_ltg_trate_rtgo(Matrix &RGO
									 ,Matrix &RGRAV
									 ,double tgo, double tgop
									 ,Matrix SDII,Matrix SBIIC,Matrix VBIIC,Matrix RBIAS	
							 		 ,Matrix ULAM,Matrix UD,Matrix UY,Matrix UZ,double s_igrl)
{
	//correcting range-due-to-gravity
	RGRAV=RGRAV*(tgo/tgop)*(tgo/tgop);
	Matrix RGO_LOCAL=SDII-(SBIIC+VBIIC*tgo+RGRAV)-RBIAS;

	//calculating range-to-go vector component in (UD,UY)-plane
	double rgox=RGO_LOCAL^UD;
	double rgoy=RGO_LOCAL^UY;
	Matrix RGOXY=UD*rgox+UY*rgoy;

	//replacing down-range z-component
	double num=RGOXY^ULAM;
	double denom=ULAM^UZ;
	if(denom==0){
		cerr<<" *** Warning: divide by zero in 'guidance_ltg_trate_rtgo'; previous 'RGO' used *** \n";
		return;
	}
	double rgoz=(s_igrl-num)/denom;

	//(RGO is in the direction of ULAM with magnitude of s_igrl)
	RGO=RGOXY+UZ*rgoz; 
}
///////////////////////////////////////////////////////////////////////////////
//End-state predictor calculations
// * corrects for gravity effects and adds bias corrections 
//
//Parameter output:
//			SPII(3) = predicted inertial position vector - m
//			VPII(3) = predicted inertial velocity vector - m/s
//			RGRAV(3) = position loss due to gravity - m
//			RBIAS(3) = position bias -> used in '_rtgo()' - m			
//Parameter input:
//			LAMD(3) = derivative of unit thrust vector - 1/s
//			ULAM(3) = unit thrust vector in direction of VGO - ND
//			l_igrl = L-integral (velocity to be gained - m/s)
//			s_igrl = thrust integral, postion - m   
//			j_igrl = thrust integral, velocit*time - m   
//			q_igrl = thrust integral, position*time - m*s   
//			h_igrl = thrust integral, velocity*time^2 - m*s   
//			p_igrl = thrust integral, position*time^2 - m*s^2
//			j_over_l= time remaining - s
//			qprime = conditioned q-integral - m*s
//			SBIIC(3) = INS vehicle inertial position - m 
//			VBIIC(3) = INS vehicle inertial velosicy - m/s
//			RGO(3) = range-to-go - m
//			tgo = time-to-go - s 
//
//040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::guidance_ltg_pdct(Matrix &SPII,Matrix &VPII,Matrix &RGRAV,Matrix &RBIAS
							  ,Matrix LAMD,Matrix ULAM,double l_igrl,double s_igrl,double j_igrl
							  ,double q_igrl,double h_igrl,double p_igrl,double j_over_l,double qprime
							  ,Matrix SBIIC,Matrix VBIIC,Matrix RGO,double tgo)
{
	//local variables
	Matrix SBIIC2(3,1);
	Matrix VBIIC2(3,1);
	
	//velocity gained due to thrust
	double lmdsq=LAMD^LAMD;
	Matrix VTHRUST=ULAM*(l_igrl-0.5*lmdsq*(h_igrl-j_igrl*j_over_l));//Jackson, p.8

	//displacement gained due to thrust
	Matrix RTHRUST=ULAM*(s_igrl-0.5*lmdsq*(p_igrl-j_over_l*(q_igrl+qprime)))+LAMD*qprime;//Jackson, p.8

	//bias used in '-rtgo()'
	RBIAS=RGO-RTHRUST;

	//offsetting SBIIC, VBIIC to SBIIC1, VBIIC1 for gravity calculations
	Matrix SBIIC1=SBIIC-RTHRUST*0.1-VTHRUST*(tgo/30); //Jackson, p.23
	Matrix VBIIC1=VBIIC+RTHRUST*(1.2/tgo)-VTHRUST*0.1;//Jackson, p.23

	//calling Kepler utility to project to end state (two options available)
	int flag=cad_kepler(SBIIC2,VBIIC2,SBIIC1,VBIIC1,tgo);
//	int flag=cad_kepler1(SBIIC2,VBIIC2,SBIIC1,VBIIC1,tgo);
	if(flag){
		cerr<<" *** Warning: bad Kepler projection in 'guidance_ltg_pdct()' *** \n";} 
	//gravity corrections
	Matrix VGRAV=VBIIC2-VBIIC1;
	RGRAV=SBIIC2-SBIIC1-VBIIC1*tgo;

	//predicted state with gravity and thrust corrections
	SPII=SBIIC+VBIIC*tgo+RGRAV+RTHRUST;
	VPII=VBIIC+VGRAV+VTHRUST;
}
///////////////////////////////////////////////////////////////////////////////
//End-state corrector calculations
// * first called for intialization in 'guidance_ltg()'
//
//Parameter output:
//			SDII(3) = desired inertial position, defined here - m
//			UD(3) = unit vector of SPII and SDII - ND 
//			UY(3) = unit vector normal to trajectory plane - ND
//			UZ(3) = unit vector in traj plane, normal to desired inertial pos - ND
//			VMISS(3) = velocity miss - m/s
//
//Parameter input/output:
//			VGO(3) = velocity to be achieved - m/s 
//
//Parameter input:
//			dbi_desired = desired orbital end position (input) - m
//			dvbi_desired = desired end velocity (input) - m/s
//			SPII(3) = predicted inertial position vector - m
//			VPII(3) = predicted inertial velocity vector - m/s			
//			thtvdx_desired = desired flight path angle - deg  
//			SBIIC(3) = INS vehicle inertial position - m 
//			VBIIC(3) = INS vehicle inertial velosicy - m/s
//
//040325 Converted from FORTRAN by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::guidance_ltg_crct(Matrix &SDII,Matrix &UD,Matrix &UY,Matrix &UZ,Matrix &VMISS
							  ,Matrix &VGO
							  ,double dbi_desired,double dvbi_desired,double thtvdx_desired
							  ,Matrix SPII,Matrix VPII,Matrix SBIIC,Matrix VBIIC)
{
	//local variables
	Matrix VDII(3,1);

	//local module-variable
	double dpd(0);
	double dbd(0);
	
	//desired position
	UD=SPII.univec3();
	SDII=UD*dbi_desired;

	//generating unit base vectors ('%' overloaded Matrix operator)
	UY=VBIIC%SBIIC;
	UZ=UD%UY;

	//velocity-to-be-gained
	VDII=(UD*sin(thtvdx_desired*RAD)+UZ*cos(thtvdx_desired*RAD))*dvbi_desired;
	VMISS=VPII-VDII;
	VGO=VGO-VMISS;

	//diagnostics:
	//displacement of P wrt D (both lie always on the UD vector)
	double dpi=SPII.absolute();
	double ddi=SDII.absolute();
	dpd=dpi-ddi;
	//displacement of vehicle B wrt the desired end-point D
	double dbi=SBIIC.absolute();
	dbd=dbi-ddi;
	
	//-------------------------------------------------------------------------
	//diagnostics
	hyper[476].gets(dpd);
	hyper[477].gets(dbd);
}
///////////////////////////////////////////////////////////////////////////////
//Guidance on an arc through a waypoint (horizontal only)
//Waypoints are provided as: 'wp_lonx', 'wp_latx', 'wp_alt'(optional, default=0)
//applicable to:
//			mguide = 4 lateral arc-guidance 
//
//parameter input
//		wp_lonx=waypoint longitude - deg
//		wp_latx=waypoint latitude - deg
//		wp_alt=waypoint altitude - m (any value, e.g.:0)
//return output:
//	 phicomx = commanded bank angle - deg
//
//030616 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double Hyper::guidance_arc(double wp_lonx,double wp_latx,double wp_alt)
{
	//local variables
	double argument(0);
	double phicomx(0);
	double rad_dynamic(0);

	//local module-variables
	Matrix SWBD(3,1);
	int wp_flag(0);
	double wp_grdrange(0);
	double rad_min(0);
	double rad_geometric(0);

	//localizing module-variables
	//input from other modules
	double time=round6[0].real();
	double grav=round6[63].real();
	double phibdx=round6[139].real();
	Matrix SBIIC=hyper[304].vec();
	Matrix VBECD=hyper[329].vec();
	double dvbec=hyper[330].real();
	Matrix TDCI=hyper[331].mat();
	double psivdcx=hyper[333].real();
	Matrix FSPCB=hyper[334].vec();
	double alphacx=hyper[336].real();
	double philimx=hyper[511].real();
	//-----------------------------------------------------------------------------
	//converting waypoint coordinates
	Matrix SWII= cad_in_geo84(wp_lonx*RAD,wp_latx*RAD,wp_alt,time);

	//displacement of waypoint wrt hyper in geographic coord
	SWBD=TDCI*(SWII-SBIIC);

	//projection of displacement vector into horizontal plane, SH
	Matrix SH(3,1);
	SH[0]=SWBD[0];
	SH[1]=SWBD[1];
	SH[2]=0;

	//horizontal ground distance based on hyper geographic coordinates
	double dwbh=sqrt(SWBD[0]*SWBD[0]+SWBD[1]*SWBD[1]);

	//calculating azimuth angle of waypoint LOS wrt velocity vector, psiwvx
	//projection of velocity vector into horizontal plane, VH
	Matrix VH(3,1);
	VH[0]=VBECD[0];
	VH[1]=VBECD[1];
	VH[2]=0;

	//vector normal to arc plane, UV
	Matrix UV=VH.skew_sym()*SH;
	//steering angle, psiwvx 
	double psiwvx=DEG*angle(VH,SH);
	//steering angle with proper sign
	Matrix ZZ(3,1);
	ZZ[2]=1;
	psiwvx=psiwvx*sign(UV^ZZ);

	//getting acceleration component in the loadfactor plane
	double fspb3=FSPCB[2];

	//selecting guidance mode
	if(fabs(psiwvx)<90)
		//guiding on the arc through the waypoint
	{
		double num=-2*dvbec*dvbec*sin(psiwvx*RAD);
//		double denom=fspb3*dwbh;
		double denom=-grav*dwbh; //eliminates limit cycle, asssuming level flight
		if(denom!=0)
			argument=num/denom;
		if(fabs(asin(argument))<philimx*RAD)
			phicomx=DEG*asin(argument);
		else
			phicomx=philimx*sign(argument);
	}
	else
	{
		//making a minimum turn
		phicomx=philimx*sign(psiwvx);
	}
	//diagnostic: radii
	if((phibdx*fspb3)!=0)
		rad_dynamic=fabs(dvbec*dvbec/(fspb3*sin(phibdx*RAD)));
	if(psiwvx!=0) 
		rad_geometric=fabs(dwbh/(2*sin(psiwvx*RAD)));
		
	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	rad_min=dvbec*dvbec/(grav*tan(philimx*RAD));
	if(dwbh<2*rad_min)
		wp_flag=sign(VH^SH);
	else
		wp_flag=0;

	//diagnostic: ground range to waypoint
	wp_grdrange=dwbh; 
	//-----------------------------------------------------------------------------
	//loading module-variables
	hyper[415].gets(wp_grdrange); 
	hyper[416].gets_vec(SWBD); 
	hyper[417].gets(rad_min);
	hyper[418].gets(rad_geometric);
	hyper[419].gets(wp_flag);

	return phicomx;
}
///////////////////////////////////////////////////////////////////////////////
//Guidance to a line
//Against stationary waypoints or targets
//Waypoint are provided as: 'wp_lonx', 'wp_latx', 'wp_alt'
//applicable to :
//mguide	= 30 line-guidance lateral 
//			= 03 line-guidance in pitch 
//			= 33 line-guidance lateral and in pitch
//
//parameter input
//		wp_lonx=waypoint longitude - deg
//		wp_latx=waypoint latitude - deg
//		wp_alt=waypoint altitude - deg
//		psifdx= heading of LOA from north - deg
//		thtfdx= elevation of LOA from target horizontal plane (up positive) - deg
//return output:
//		ALGV(3x1)=acceleration demanded by line guidance in flight path  coord. - m/s^2
// where:
//		alcomx=ALGV(2)/grav, lateral acceleration command - g's
//		ancomx=-ALGV(3)/grav, normal acceleration command - g's
//
//030616 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_line(double wp_lonx,double wp_latx,double wp_alt,double psifdx,double thtfdx)
{
	//local module-variables
	double rad_min(0);
	double wp_sltrange(0);
	double wp_grdrange(0);
	double nl_gain(0);
	int wp_flag(0);
	Matrix VBEO(3,1);
	Matrix VBEF(3,1);
	Matrix SWBD(3,1);

	//localizing module-variables
	//input data
	double line_gain=hyper[402].real();
	double nl_gain_fact=hyper[403].real();
	double decrement=hyper[404].real();
	//input from other modules
	double time=round6[0].real(); 
	double grav=round6[63].real();
	Matrix SBIIC=hyper[304].vec();
	Matrix VBECD=hyper[329].vec();
	double dvbec=hyper[330].real();
	Matrix TDCI=hyper[331].mat();
	double thtvdcx=hyper[332].real();
	double philimx=hyper[511].real();
	//-----------------------------------------------------------------------------
	//TM of LOA wrt geographic axes
	Matrix TFD=mat2tr(psifdx*RAD,thtfdx*RAD);

	//converting waypoint to inertial coordinates
	Matrix SWII= cad_in_geo84(wp_lonx*RAD,wp_latx*RAD,wp_alt,time);

	//waypoint wrt hyper displacement in geographic coord (synthetic LOS)
	SWBD=TDCI*(SWII-SBIIC);

	//building TM of LOS wrt geographic axes; also getting range-to-go to waypoint
	Matrix POLAR=SWBD.pol_from_cart();
	wp_sltrange=POLAR[0];
	double psiod=POLAR[1];
	double thtod=POLAR[2];
	Matrix TOD=mat2tr(psiod,thtod);

	//ground range to waypoint (approximated by using hyper local-level plane)
	double swbg1=SWBD[0];
	double swbg2=SWBD[1];
	wp_grdrange=sqrt(swbg1*swbg1+swbg2*swbg2);

	//converting geographic hyper velocity to LOS and LOA coordinates
	VBEO=TOD*VBECD;
	double vbeo2=VBEO[1];
	double vbeo3=VBEO[2];

	VBEF=TFD*VBECD;
	double vbef2=VBEF[1];
	double vbef3=VBEF[2];

	//nonlinear gain
	nl_gain=nl_gain_fact*(1-exp(-wp_sltrange/decrement));

	//line guidance steering law
	double algv1=grav*sin(thtvdcx*RAD);
	double algv2=line_gain*(-vbeo2+nl_gain*vbef2);
	double algv3=line_gain*(-vbeo3+nl_gain*vbef3)-grav*cos(thtvdcx*RAD);

	//packing accelerations into vector for output
	Matrix ALGV(3,1);
	ALGV[0]=algv1;
	ALGV[1]=algv2;
	ALGV[2]=algv3;

	//setting way point flag (if within 2 times turning radius): closing (+1) or fleeting (-1)
	rad_min=dvbec*dvbec/(grav*tan(philimx*RAD));
	if(wp_grdrange<2*rad_min)
	{
		//projection of displacement vector into horizontal plane, SH
		Matrix SH(3,1);
		SH[0]=swbg1;
		SH[1]=swbg2;
		SH[2]=0;
		//projection of velocity vector into horizontal plane, VH
		Matrix VH(3,1);
		VH[0]=VBECD[0];
		VH[1]=VBECD[1];
		VH[2]=0;
		//setting flag eihter to +1 or -1
		wp_flag=sign(VH^SH);
	}
	else
		wp_flag=0;
	//-----------------------------------------------------------------------------
	//loading dignostic module-variables
	hyper[411].gets(wp_sltrange);
	hyper[412].gets(nl_gain);
	hyper[413].gets_vec(VBEO);
	hyper[414].gets_vec(VBEF);
	hyper[415].gets(wp_grdrange);
	hyper[416].gets_vec(SWBD);
	hyper[417].gets(rad_min);
	hyper[419].gets(wp_flag);

	return ALGV;
}
///////////////////////////////////////////////////////////////////////////////
//Terminal guidance using glideslope guidance
//Member function of class 'Hyper'
// Ref: Hablani, Tapper and Dana-Bashian, "Guidance and Relative Naviagtion for
//      Autonomous Rendezvous in a circular Orbit", AIAA J. G,C&D, Vol. 25,
//      No. 3, May-June 2002
//
// mguide = 8 
//
// Parameter input/ouput
//			gs_flag = flag for initializing glideslope guidance - ND 
// Parameter output
//			mprop = motor firing flag - ND
// Return output
//			UTB = Commanded unit thrust vector in body coor - ND
// Parameter input
//			mseek = seeker mode switch - ND
//
//040608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_glideslope(int &mprop,int &gs_flag,int mseek)
{
	//local variables
	Matrix SATL(3,1);
	double dvbic(0);
	Matrix SBTI(3,1);
	Matrix VBTI(3,1);
	Matrix UL1I(3,1),UL2I(3,1),UL3I(3,1);
	Matrix TLI(3,3);
	Matrix UTB(3,1);
	Matrix PHISS(3,3);
	Matrix PHISV(3,3);

	//local module variables
	Matrix SBTL(3,1);
	Matrix VBTL(3,1);
	double delta_v(0);

	//localizing module-variables
	//input data
	double time_gs=hyper[481].real();
	int num_burns=hyper[482].integer();
	double closing_rate=hyper[483].real();
	double orbital_rate=hyper[484].real();
	double satl1=hyper[485].real();
	double satl2=hyper[486].real();
	double satl3=hyper[487].real();
	//input from other modules
	double time=round6[0].real();
	double dvbi=round6[226].real();
	Matrix VBIIC=hyper[303].vec();
	Matrix SBIIC=hyper[304].vec();
	Matrix STCII=hyper[352].vec();
	Matrix VTCII=hyper[353].vec();
	Matrix TBIC=hyper[315].mat();
	//restoring saved variables
	double dtime_gs=hyper[488].real();
	double length_gs=hyper[489].real();
	double para_gs=hyper[490].real();
	Matrix UB0AL=hyper[491].vec();
	double epoch_gs=hyper[492].real();
	int counter_gs=hyper[493].integer();
	Matrix VBTLM=hyper[494].vec();
	Matrix DELTA_V=hyper[495].vec();
	int burn_flag=hyper[496].integer();

	//-------------------------------------------------------------------------
	//assembling displacement vector of aimpoint wrt to target in local level coordinates
	SATL.build_vec3(satl1,satl2,satl3);

	//inertial speed from INS
	dvbic=VBIIC.absolute();
	
	//TM of local level wrt inertial coordinates
	UL1I=VTCII.univec3(); 
	UL3I=STCII.univec3()*(-1);
	UL2I=UL3I.skew_sym()*UL1I;
	TLI.build_mat33(UL1I[0],UL1I[1],UL1I[2],UL2I[0],UL2I[1],UL2I[2],UL3I[0],UL3I[1],UL3I[2]);

	//datalink targeting
	SBTI=SBIIC-STCII;
	VBTI=VBIIC-VTCII;

	//vehicle position and velocity in local-level coordinates
	SBTL=TLI*SBTI;
	VBTL=TLI*VBTI;

	//initialization of glideslope parameters
	if(gs_flag){
		gs_flag=0;

		//pulse interval
		dtime_gs=time_gs/num_burns;

		//forming glideslope unit vector
		Matrix SB0AL=SBTL-SATL;
		UB0AL=SB0AL.univec3();

		//glideslope parameter: 'para_gs'
		length_gs=SB0AL.absolute();
		double rho_0_dot=UB0AL^VBTL;
		para_gs=(rho_0_dot-closing_rate)/length_gs;

		//other initializations
		epoch_gs=time;
		counter_gs=0;
	}

	//update epoch of glideslope guidance
	if(counter_gs<num_burns&&time>=(dtime_gs*counter_gs+epoch_gs)){
		
		//building state transition submatrices
		double w=orbital_rate;
		double wt=orbital_rate*dtime_gs;
		double swt=sin(wt);
		double cwt=cos(wt);
		PHISS.build_mat33(1,0,6*(wt-swt),0,cwt,0,0,0,4-3*cwt);
		PHISV.build_mat33(4*swt/w-3*dtime_gs,0,2*(1-cwt)/w,0,swt/w,0,-2*(1-cwt)/w,0,swt/w);

		//glideslope distance still to-be-travelled 'dlength'
		double dum=para_gs*dtime_gs*(counter_gs+1);
		double dlength=length_gs*exp(dum)+(closing_rate/para_gs)*(exp(dum)-1);

		//desired position
		Matrix SBCTL=SATL+UB0AL*dlength;

		//desired departure velocity
		Matrix VBTLP=PHISV.inverse()*(SBCTL-PHISS*SBTL);

		//delta-v to-be-supplied by burn
		DELTA_V=VBTLP-VBTL;

		//saving velocity 
		VBTLM=VBTL;

		//other updates
		counter_gs++;
		burn_flag=1;
	}
	//thrust unit vector in body coordinates to be sent to 'rcs' module
	UTB=TBIC*~TLI*DELTA_V.univec3();

	//remaining delta-v to achieve
	Matrix EV=DELTA_V+VBTLM-VBTL;
	delta_v=EV.absolute();

	//applying thrust after vehicle has nearly rotated to required attitude
	if(UTB[0]>0.9){

		//burning motor until delta-v is within 'closing_rate'
		if(delta_v>fabs(closing_rate)&&burn_flag){
			mprop=4;
		}
		else{
			mprop=0;
			burn_flag=0;
		}
	}
	//if seeker is in acquisition mode, point the 1B axis at the target
	if(mseek==3)
		UTB=TBIC*SBTI.univec3()*(-1);
	//-------------------------------------------------------------------------
	//saving variables
	hyper[488].gets(dtime_gs);
	hyper[489].gets(length_gs);
	hyper[490].gets(para_gs);
	hyper[491].gets_vec(UB0AL);
	hyper[492].gets(epoch_gs);
	hyper[493].gets(counter_gs);
	hyper[494].gets_vec(VBTLM);
	hyper[495].gets_vec(DELTA_V);
	hyper[496].gets(burn_flag);
	//diagnostics
	hyper[497].gets_vec(SBTL);
	hyper[498].gets_vec(VBTL);
	hyper[499].gets(delta_v);

	return UTB;
}
///////////////////////////////////////////////////////////////////////////////
//Proportional navigation guidance
//Member function of class 'Hyper'
//
// (1) Calculates LOS rate from uplinked coordinates and on-board INS
// (2) Calculates acceleration command based on pro-nav guidance law
// (3) Calculates direction of main thrust
//
// Parameter input
//			STBIK(3x1) = Displacment of target wrt hyper vehicle in inertial coord - m
//			VTBIK(3x1) = Velocity of target wrt hyper vehicle in inertial coord - m/s
//
// Return output
//			ACCOMX(3x1) = acceleration command in body coord - g's
// Parameter output
//			UTBBC(3x1) = Commanded unit thrust vector in body coor - ND  
//
//040520 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_pronav(Matrix &UTBBC, Matrix STBIK,Matrix VTBIK)
{
	//local variables
	Matrix STBIC(3,1);
	Matrix UTBIC(3,1);
	Matrix POLAR(3,1);
	Matrix VTBIC(3,1);
	Matrix AAPNB(3,1);
	Matrix WOIIC(3,1);
	Matrix ACCOMX(3,1);
	
	//local module-variables
	double dtbc(0);
	double psiobcx(0);
	double thtobcx(0);
	double dvtbc(0);
	double tgoc(0);
	Matrix SBTHC(3,1);

	//localizing module-variables
	//input data
	double gnav=hyper[420].real();
	//input from other modules
	Matrix STII=hyper[2].vec();
	Matrix VTII=hyper[3].vec();
	double time=round6[0].real();
	int mseek=hyper[200].integer();
	Matrix VBIIC=hyper[303].vec();
	Matrix SBIIC=hyper[304].vec();
	Matrix TBIC=hyper[315].mat();
	Matrix STCII=hyper[352].vec();
	Matrix VTCII=hyper[353].vec();
	//-------------------------------------------------------------------------
	//line of sight kinematics
	if(mseek>3){
		//from seeker
		STBIC=STBIK;
		VTBIC=VTBIK;
	}else{
		//from data link
		STBIC=STCII-SBIIC;
		VTBIC=VTCII-VBIIC;
	}
	//range to go
	dtbc=STBIC.absolute();

	//unit los vector
	// also used as commanded unit thrust vector in body coord (keeping angle of attack at zero)
	UTBIC=STBIC.univec3();
	UTBBC=TBIC*UTBIC;

	//LOS angles wrt hyper body
	POLAR=UTBBC.pol_from_cart();
	psiobcx=POLAR.get_loc(1,0)*DEG;
	thtobcx=POLAR.get_loc(2,0)*DEG;


	//closing velocity
	dvtbc=UTBIC^VTBIC;

	//time to go
	tgoc=fabs(dtbc/dvtbc);

	//inertial los rates in inertial coordinates
	WOIIC=UTBIC.skew_sym()*VTBIC*(1/dtbc);

	//acceleration command of proportional navigation
	AAPNB=TBIC*WOIIC.skew_sym()*UTBIC*gnav*fabs(dvtbc);

	//converting to  g's
	ACCOMX=AAPNB*(1/AGRAV);

	//for diagnostics: Hill coordinates
	//base vectors
	Matrix UH1=STII.univec3();
	Matrix ANG_MOM=STII.skew_sym()*VTII;
	Matrix UH3=ANG_MOM.univec3();
	Matrix UH2=UH3.skew_sym()*UH1;
	//T.M. of Hill wrt inertial coordinates
	Matrix THI(3,3);
	THI.build_mat33(UH1[0],UH1[1],UH1[2],UH2[0],UH2[1],UH2[2],UH3[0],UH3[1],UH3[2]);

	//for diagnostics: vehicle displacement in Hill coord
	Matrix SBTIC=STBIC*(-1);
	SBTHC=THI*SBTIC;
	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[423].gets(tgoc);
	hyper[424].gets(dtbc);
	hyper[425].gets(psiobcx);
	hyper[426].gets(thtobcx);
	hyper[427].gets_vec(SBTHC);
	
	return ACCOMX;
}
///////////////////////////////////////////////////////////////////////////////
//Terminal guidance using the advanced guidance law (AGL)
//Member function of class 'Hyper'
//
// Calculates acceleration commands based on relative position
//  and velocity of target wrt hyper vehicle in inertial coordinates.
//
// Parameter input
//			STBIK(3x1) = Displacment of target wrt hyper vehicle in inertial coord - m
//			VTBIK(3x1) = Velocity of target wrt hyper vehicle in inertial coord - m/s
//
// Return output
//			ACCOMX(3x1) = acceleration command in body coord - g's
// Parameter output
//			UTBBC(3x1) = Commanded unit thrust vector in body coor - ND  
//
//040608 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Matrix Hyper::guidance_AGL(Matrix &UTBBC, Matrix STBIK,Matrix VTBIK)
{
	//local variables
	double tgok(0);
	Matrix ACPURI(3,1);
	Matrix ACPNI(3,1);
	Matrix ACCOMX(3,1);
	Matrix UTBIK(3,1);

	//local module-variables
	double dtbc(0);
	double psiobcx(0);
	double thtobcx(0);
	double dvtbc(0);
	double tgoc(0);

	//localizing module-variables
	//input data
	double gnavpn=hyper[478].real();
	double gnavps=hyper[479].real();
	//input from other modules
	double time=round6[0].real();
	double pdynmc=round6[57].real();
	double gnmax=hyper[167].real();
	double dab=hyper[241].real();
	double ddab=hyper[242].real();
	Matrix TBIC=hyper[315].mat();
	//-------------------------------------------------------------------------

	//range to go
	dtbc=STBIK.absolute();

	//unit los vector
	// also used as commanded unit thrust vector in body coord (keeping angle of attack at zero)
	UTBIK=STBIK.univec3();
	UTBBC=TBIC*UTBIK;

	//LOS angles wrt hyper body
	Matrix POLAR=UTBBC.pol_from_cart();
	psiobcx=POLAR.get_loc(1,0)*DEG;
	thtobcx=POLAR.get_loc(2,0)*DEG;

	//closing velocity
	dvtbc=UTBIK^VTBIK;

	//time to go
	tgoc=fabs(dtbc/dvtbc);

	//acceleration command in inertial coordinates
	if(tgoc!=0){
		//pursuit term
		ACPURI=STBIK*(gnavps/(tgoc*tgoc));
		//proportional guidance term
		ACPNI=VTBIK*(gnavpn/tgoc);
	}
	//acceleration desired in body coordinates
	ACCOMX=TBIC*(ACPURI+ACPNI)*(1/AGRAV);

	//-------------------------------------------------------------------------
	//loading module-variables
	//diagnostics
	hyper[423].gets(tgoc);
	hyper[424].gets(dtbc);
	hyper[425].gets(psiobcx);
	hyper[426].gets(thtobcx);
	
	return ACCOMX;
}
