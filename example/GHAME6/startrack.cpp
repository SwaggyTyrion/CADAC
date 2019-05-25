///////////////////////////////////////////////////////////////////////////////
//FILE: 'startrack.cpp'
//Contains 'startrack' module of class 'Hyper'
//
//040212 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of star tracker module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[800-849]
//
//mstar	= 0 no startrack (default)
//		= 1 enabling startrack  (input)
//		= 2 delaying star track upates
//		= 3 sending tilt corrections to INS ('ins' module resets mstar=2) 
//
//040212 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_startrack()
{
	//definition of module-variables
	hyper[800].init("mstar","int",0,"=0:no star track; =1:init; =2:waiting; =3:update - ND","startrack","data","");
	hyper[802].init("star_el_min",0,"Minimum star elev angle from horizon - deg","startrack","data","");
	hyper[803].init("star_volume",0,"Volume of the parallelepiped of the star triad - ND","startrack","diag","");
	hyper[804].init("startrack_alt",0,"Altitude above which star tracking is possible - m","startrack","data","");
	hyper[805].init("star_acqtime",0,"Initial acquisition time for the star triad - s","startrack","data","");
	hyper[806].init("star_step",0,"Star fix update interval - s","startrack","data","");
	hyper[807].init("starfix_epoch",0,"Starfix update epoch time since launch - s","startrack","save","");
	hyper[808].init("star_acq","int",0,"=0: Startrack not initialized; =1: initialized - ND ","startrack","save","");
	hyper[809].init("star_slotsum",0,"Sum of stored slot numbers of triad - ND","startrack","save","");
	hyper[810].init("az1_bias",0,"Star azimuth error 1 bias - rad GAUSS","startrack","data","");
	hyper[811].init("az2_bias",0,"Star azimuth error 2 bias - rad GAUSS","startrack","data","");
	hyper[812].init("az3_bias",0,"Star azimuth error 3 bias - rad GAUSS","startrack","data","");
	hyper[813].init("az1_noise",0,"Star azimuth error 1 noise - rad MARKOV","startrack","data","");
	hyper[814].init("az2_noise",0,"Star azimuth error 2 noise - rad MARKOV","startrack","data","");
	hyper[815].init("az3_noise",0,"Star azimuth error 3 noise - rad MARKOV","startrack","data","");
	hyper[816].init("el1_bias",0,"Star elimuth error 1 bias - rad GAUSS","startrack","data","");
	hyper[817].init("el2_bias",0,"Star elimuth error 2 bias - rad GAUSS","startrack","data","");
	hyper[818].init("el3_bias",0,"Star elimuth error 3 bias - rad GAUSS","startrack","data","");
	hyper[819].init("el1_noise",0,"Star elimuth error 1 noise - rad MARKOV","startrack","data","");
	hyper[820].init("el2_noise",0,"Star elimuth error 2 noise - rad MARKOV","startrack","data","");
	hyper[821].init("el3_noise",0,"Star elimuth error 3 noise - rad MARKOV","startrack","data","");
	hyper[830].init("URIC",0,0,0,"Tilt corrections for INS","startrack","out","plot");
}
///////////////////////////////////////////////////////////////////////////////  
//Star tracker module
//Member function of class 'Hyper'
//
//mstar	= 0 no startrack (default)
//		= 1 enabling startrack  (input)
//		= 2 delaying star track upates
//		= 3 sending tilt corrections to INS ('ins' module resets mstar=2) 
//
//* Loads 25 bright stars from the star catalog
//* Separates out the stars that are visible from the vehicle
//* Picks those three stars (called triad) that provide the best measurements
//* Simulates the tracker errors by corrupting the true LOS
//* Calculates the INS tilt updates and sends them to the INS    
//
//040212 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
	
void Hyper::startrack()
{
	//local variables
	static double star_data[75];
	static string star_names[25];
	double usii_triad[12]; //star triad inertial coord and star slot#
	double dtime_star(0);
	double time_star(0);
	Matrix TRIAD_TRUE(3,3);
	Matrix TRIAD_MEAS(3,3);
	double slot[3]={0,0,0,}; //star slot#  of triad
	double slotm(0);
	Matrix AZ_BIAS(3,1);	
	Matrix AZ_NOISE(3,1);
	Matrix EL_BIAS(3,1);	
	Matrix EL_NOISE(3,1);

	//local module-variables
	double star_volume(0);

	//localizing module-variables
	//input data
	int mstar=hyper[800].integer();
	double star_el_min=hyper[802].real();
	double startrack_alt=hyper[804].real();
	double star_acqtime=hyper[805].real();
	double star_step=hyper[806].real();
	double az1_bias=hyper[810].real();
	double az2_bias=hyper[811].real();
	double az3_bias=hyper[812].real();
	double az1_noise=hyper[813].real();	
	double az2_noise=hyper[814].real();
	double az3_noise=hyper[815].real();
	double el1_bias=hyper[816].real();
	double el2_bias=hyper[817].real();
	double el3_bias=hyper[818].real();
	double el1_noise=hyper[819].real();
	double el2_noise=hyper[820].real();	
	double el3_noise=hyper[821].real();
	//assembling bias and noise measurement vectors 
	AZ_BIAS[0]=az1_bias;
	AZ_BIAS[1]=az2_bias;
	AZ_BIAS[2]=az3_bias;
	AZ_NOISE[0]=az1_noise;
	AZ_NOISE[1]=az2_noise;
	AZ_NOISE[2]=az3_noise;
	EL_BIAS[0]=el1_bias;
	EL_BIAS[1]=el2_bias;
	EL_BIAS[2]=el3_bias;
	EL_NOISE[0]=el1_noise;
	EL_NOISE[1]=el2_noise;
	EL_NOISE[2]=el3_noise;
	//getting saved values of startrack
	double starfix_epoch=hyper[807].real();
	int star_acq=hyper[808].integer();
	double star_slotsum=hyper[809].real();
	Matrix URIC=hyper[830].vec();
	//input from other modules
	double time=round6[0].real();
	Matrix TBI=round6[121].mat();
	double alt=round6[221].real();
	Matrix SBII=round6[235].vec();
	Matrix TBIC=hyper[315].mat();
	//-----------------------------------------------------------------------------
	//returning, if no star tracking
	if(mstar==0)
	{
		return;
	}
	//star tracker initialization
	if(mstar==1)
	{
		//Loading star catalog
		startrack_init(star_data,star_names);
		
		//setting inital acquisition flag
		star_acq=1;

		//initializing update clock
		starfix_epoch=time;

		//initiating star tracking if above dense atmosphere
		if(alt>startrack_alt)
			mstar=2;
	}
	//delay between updates; and initial star acquisition time delay 
	if(mstar==2)
	{
		if(star_acq)
			//saving delay-time for initial star acquisition
			dtime_star=star_acqtime;
		else
			//saving delay-time for star update
			dtime_star=star_step;

		//checking when starfix update time has occured in order to initiate update
		//provided vehicle is high enough
		time_star=time-starfix_epoch;
		if(time_star>=dtime_star&&alt>startrack_alt)
			mstar=3;		
	}	
	//INS update epoch
	if(mstar==3)
	{
		//cancelling initial star acquisition flag
		star_acq=0;

		//resetting update clock
		time_star=0;
		starfix_epoch=time;

		//getting star triad
		star_triad(usii_triad,star_volume, star_data,star_el_min,SBII);

		//shooting stars of the triad (measurement of unit vectors)
		for(int i=0;i<3;i++){
			int j(0);
			//unpacking i-th star's unit vector
			Matrix USII(3,1);
			for(j=0;j<3;j++){
				USII[j]=*(usii_triad+4*i+j);
			}
			//converting to body coord (using true TBI)
			Matrix USIB(3,1);			
			USIB=TBI*USII;
			//converting to true azimuth and elevation
			Matrix POLAR(3,1);
			POLAR=USIB.pol_from_cart();
			double az=POLAR[1];
			double el=POLAR[2];

			//measurements with measurement uncertainties 
			double az_meas=az+AZ_BIAS[i]+AZ_NOISE[i];
			double el_meas=el+EL_BIAS[i]+EL_NOISE[i];

			//converting measurement back to inertial unit vectors with INS information (using TBIC)
			//(here the tilt error of the INS enters the measurement)
			Matrix USIBM(3,1);
			USIBM.cart_from_pol(1,az_meas,el_meas);
			Matrix USIIM(3,1);
			USIIM=~TBIC*USIBM;

			//saving measurement vectors by columns
			for(j=0;j<3;j++){
				TRIAD_MEAS.assign_loc(j,i,USIIM.get_loc(j,0));
			}
			//saving true vectors by columns
			for(j=0;j<3;j++){
				TRIAD_TRUE.assign_loc(j,i,USII.get_loc(j,0));
			}
			//for diagnostics: loading the slot# of the triad
			*(slot+i)=*(usii_triad+4*i+3);
			//accumulating sum of slots
			slotm=slotm+slot[i];
		}
		//for diagnostics displaying the SV slot# of the triad on the console
		// but only if they have changed (sum of slot# has changed)
		if(star_slotsum!=slotm){
			star_slotsum=slotm;
			int slot1=(int)slot[0];
			int slot2=(int)slot[1];
			int slot3=(int)slot[2];
			cout<<" *** Star triad: "<<*(star_names+slot1-1)<<"  "
				<<*(star_names+slot2-1)<<"  "<<*(star_names+slot3-1)<<" ***\n";
		}
		//calculating the tilt corrections for the INS
		Matrix RDIFF(3,3);
		RDIFF=TRIAD_MEAS*TRIAD_TRUE.inverse();

		URIC[0]=RDIFF.get_loc(2,1); //phi-tilt
		URIC[1]=RDIFF.get_loc(0,2);	//theta-tilt
		URIC[2]=RDIFF.get_loc(1,0); //psi-tilt
	}
	//-----------------------------------------------------------------------------
	//loading module-variables
	//output to other modules
	hyper[830].gets_vec(URIC);
	//saving values 
	hyper[800].gets(mstar);
	hyper[807].gets(starfix_epoch);
	hyper[808].gets(star_acq);
	hyper[809].gets(star_slotsum);	
	//diagnostics
	hyper[803].gets(star_volume);
}

///////////////////////////////////////////////////////////////////////////////
//Selection of the best three stars (triad)
//Member function of class 'Hyper'
//
// Parameter input:
//	*star_data=25x3 array of 25 stars and their three unit vector components in the J2000 coord system
//					(the two-dimensional array is stored in sequential rows in a one-dimensional array)
//	star_el_min = minimum star elev angle from horizon - deg
//	SBII = inertial coordinates of hypersonic vehicle - m
//
// Parameter output:
//	*usii_triad(3x4) = unit vector inertial coord & slot# of each star of the triad , stored sequentially - ND
//	star_volume = volume of parallelepiped formed by the triad (max value=1) - ND
//	
//040211 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::star_triad(double *usii_triad,double star_volume, double *star_data,double star_el_min,Matrix SBII)
{
	double star_usii[25][3];
	double slot[3]={0,0,0}; //star slot#  of triad 
	int islot[4]={0,0,0}; //star slot#  of triad stored as integer 
	int triad[3]={0,0,0}; //location of triad stars in 'usii_vis[visible_count]
	int visible_count(0);
	int i(0);

	//unpacking the one-dimensional array of stars into star_usii[25][3]
	for (i=0;i<25;i++){
		for(int k=0;k<3;k++){
			star_usii[i][k]=*(star_data+3*i+k);
		}
	}	
	//determining visible stars slot #
	Matrix UBII(3,1);
	Matrix USII(3,1);
	bool visible=false;
	int vis_star_slot[25];//=0 if not visible, otherwise contains catalog slot#

	//vehicle's inertial unit vector
	UBII=SBII.univec3();

	for(i=0;i<25;i++){
		visible=false;
		USII[0]=star_usii[i][0];
		USII[1]=star_usii[i][1];
		USII[2]=star_usii[i][2];

		//star elevation angle from local horizontal (note: star is at infinity)
		double elstar=asin(USII^UBII);
		vis_star_slot[i]=0;
		if(elstar>star_el_min*RAD){
			visible_count++;
			//load slot# (starting at 1,2,3...) into 'int vis_star_slot[25]
			// a zero means it is not visible
			vis_star_slot[i]=i+1;
		}
	}
	//repackage visible stars into 'usii_vis' single-dimensioned array
	//inertial unit vector elements are stored sequentially in 'usii_vis[3*visible_count]'
	double *usii_vis;
	usii_vis=new double[3*visible_count];
	int k(0);
	for(i=0;i<25;i++){
		if(vis_star_slot[i]>0){
			*(usii_vis+k)=star_usii[i][0];
			*(usii_vis+k+1)=star_usii[i][1];
			*(usii_vis+k+2)=star_usii[i][2];
			k=k+3;
		}
	}
	//selecting triad (three stars) with maximum volume of their parallelepiped 
	//i1, i2, i3 are the stars picked by the binomial combination
	int nm2=visible_count-2; 
	int nm1=visible_count-1;
	
	for(int i1=0;i1<nm2;i1++){
		for(int i2=i1+1;i2<nm1;i2++){
			for(int i3=i2+1;i3<visible_count;i3++){

				//pullling the triad inertial coordinates
				Matrix USII1(3,1);
				Matrix USII2(3,1);
				Matrix USII3(3,1);
				for(int m=0;m<3;m++){
					USII1[m]=*(usii_vis+3*i1+m);
					USII2[m]=*(usii_vis+3*i2+m);
					USII3[m]=*(usii_vis+3*i3+m);
				}
				//calculating volume
				double volume_local(0);
				volume_local=fabs(USII1^(USII2.skew_sym()*USII3));

				//save location of triad stars in 'ssii_vis[visible_count]' if volume has increased
				if(volume_local>star_volume){
					star_volume=volume_local;
					triad[0]=i1; 
					triad[1]=i2; 
					triad[2]=i3; 
				}
			}
		}
	}//end of picking triads amongst visible stars

	//loading the visible catalog slot# from 'vis_star_slot[25]' into 'vis_star_slot_vis[visible_count]'
	// deleting all slots with 0 (invisible)
	int *vis_star_slot_vis;
	vis_star_slot_vis=new int[visible_count];
	int kk(0);
	for(i=0;i<25;i++){
		if(vis_star_slot[i]>0){
			vis_star_slot_vis[kk]=vis_star_slot[i];
			kk++;
		}
	}
	//extract triad (max volume) from visible stars
	//storing inertial coordinates of the three stars and their catalog slot# in usii_triad[12]
	// m = star index (0, 1, 2,...) in 'usii_vis' 
	for(int m=0;m<3;m++){
		for(int n=0;n<3;n++){
			*(usii_triad+4*m+n)=*(usii_vis+3*triad[m]+n);
		}
			*(usii_triad+4*m+3)=vis_star_slot_vis[triad[m]];
	}
	delete[] vis_star_slot_vis;
	delete[] usii_vis;	
}
///////////////////////////////////////////////////////////////////////////////
//Loading star catalog data
//Member function of class 'Hyper'
//
// parameter output:
//	*star_data=25x3 array of 25 stars and their three unit vector components in the J2000 coord system
//					(the two-dimensional array is stored in sequential rows in a one-dimensional array)
//	*star_names=25 array of the 25 star names loaded as strings 
//
//040210 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::startrack_init(double *star_data,string *star_names)
{
	//25 bright star catalog
	//unit vectors in J2000 coordinates

	double star_catalog_data[25][3]={
		-.179457, .947482,-.264715, //1  Sirius
		-.062053, .621699,-.780794, //2  Canopus
		-.395709,-.321011,-.860446, //3  Rigil Kent
		-.787739,-.518432, .332710, //4  Arcturus
		 .119339,-.770884, .625697, //5  Vega
		 .205167, .969392,-.134851, //6  Rigel
		 .141589, .680716, .718733, //7  Capella
		-.407741, .908325, .093239, //8  Procyon
		 .504096, .224174,-.834046, //9  Achernar
		-.434428,-.251576,-.864859, //10 Hadar
		 .449879,-.880088, .151836, //11 Altair
		 .355451, .890944, .282620, //12 Aslebaran
		-.479412,-.049965,-.876167, //13 Acrux
		 .032446, .991140, .128796, //14 Betelgeuse
		-.357911,-.827083,-.433397, //15 Antares
		-.923975,-.348220,-.158158, //16 Spica
		-.380630, .795326, .471781, //17 Pollux
		 .846646,-.247176,-.471268, //18 Fomalhout
		 .453017,-.541322, .708340, //19 Deneb
		-.511331,-.101246,-.853399, //20 Mimosa
		-.858250, .467448, .211893, //21 Regulus
		-.217999, .863108,-.455545, //22 Adhara
		 .161912, .980685, .109734, //23 Bellatrix
		-.103643,-.792588,-.600885, //24 Shaula
		 .140796, .866902, .478181  //25 El Nath
	};
	string star_catalog_names[25]={
		"Sirius",
		"Canopus",
		"Rigil Kent",
		"Arcturus",
		"Vega",
		"Rigel",
		"Capella",
		"Procyon",
		"Achernar",
		"Hadar",
		"Altair",
		"Aslebaran",
		"Acrux",
		"Betelgeuse",
		"Antares",
		"Spica",
		"Pollux",
		"Fomalhout",
		"Deneb",
		"Mimosa",
		"Regulus",
		"Adhara",
		"Bellatrix",
		"Shaula",
		"El Nath"
	};
	//loading star unit vectors into 'star_data' array by pointer arithmetic 
	int i(0);
	for (i=0;i<25;i++){
		for(int k=0;k<3;k++){
			*(star_data+3*i+k)=star_catalog_data[i][k];
		}
	}
	//loading star names into 'star_names' array by pointer arithmetic 
	for (i=0;i<25;i++){
		*(star_names+i)=star_catalog_names[i];
	}
}

