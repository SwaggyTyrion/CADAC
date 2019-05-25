///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_constants.hpp'
//
//Defines all global constant parameters for HYPER simulation
//
//030415 Created by Peter H zipfel
///////////////////////////////////////////////////////////////////////////////

#ifndef global_constants__HPP
#define global_constants__HPP

//global constants to be used in the simulation
//physical constants
double const REARTH=6370987.308;		//mean earth radius - m
double const WEII3=7.292115e-5;			//angular rotation of earth - rad/s
double const AGRAV=9.80675445;			//standard value of gravity acceleration - m/s^2 
double const G=6.673e-11;				//universal gravitational constant - Nm^2/kg^2 
double const EARTH_MASS=5.973332e24;	//mass of the earth - kg 
double const GM=3.9860044e14;			//gravitational parameter=G*EARTH_MASS - m^3/s^2 
double const C20=-4.8416685e-4;			//second degree zonal gravitational coefficient - ND
double const FLATTENING=3.33528106e-3;  //flattening of the Earth (WGS84) - ND
double const SMAJOR_AXIS=6378137;        //semi-major axis of Earth's ellipsoid (WGS84) - m 
double const GW_CLONG=0;				//Greenwich celestial longitude at start of simulation - rad
double const RGAS=287.053;				//ideal gas constant - J/(K*kg)=N*m/(K*kg) 
double const KBOLTZ=1.38e-23;			//Boltzmann's constant - Ws/K  
//numerical constants
double const PI=3.1415926536;			//circumference of unit diameter circle
double const EPS=1.e-10;				//machine precision error (type double)
double const SMALL=1e-7;				//small real number
int const    ILARGE=9999;				//large integer number
double const LARGE=1e10;				//large real number (type double)
//conversion factors
double const RAD=0.0174532925199432;	//conversion factor deg->rad
double const DEG=57.2957795130823;		//conversion factor rad->deg
double const FOOT=3.280834;				//conversion factor m->ft			
double const NMILES=5.399568e-4;		//conversion factor	m->nm	
//sizing of arrays
int const CHARN=31;						//character numbers in variable names
int const CHARL=200;					//character numbers in a line
//*** verify the following array sizes. If too small, dynamic memory allocation will fail! ***
int const NROUND6=300;					//size of 'round6' module-variable array
int const NHYPER=850;					//size of 'hyper' module-variable array
int const NROUND3=60;					//size of 'round3' module-variable array 
int const NSAT=20;						//size of 'satellite' module-variable array
int const NGROUND0=20;					//size of 'round3' module-variable array 
int const NRADAR=30;					//size of 'recce' module-variable array
int const NEVENT=20;					//max number of events
int const NVAR=50;						//max number of variables to be input at every event 
int const NMARKOV=20;					//max number of Markov noise variables
#endif