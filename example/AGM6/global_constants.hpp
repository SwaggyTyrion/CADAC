///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_constants.hpp'
//
//Defines all global constant parameters
//
//081010 Adapted to GENSIM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

#ifndef global_constants__HPP
#define global_constants__HPP

using namespace std;

//global constants to be used in the simulation
//physical constants
double const REARTH=6370987.308;		//mean earth radius - m
double const WEII3=7.292115e-5;			//angular rotation of earth - rad/s
double const AGRAV=9.80675445;			//standard value of gravity acceleration - m/s^2 
double const G=6.673e-11;				//universal gravitational constant - Nm^2/kg^2 
double const EARTH_MASS=5.973e24;		//mass of the earth - kg 
double const GAS_CON=53.35;				//universal gas constant for air in English units ft*lb/(R*lbm)
double const RGAS=287.053;				//ideal gas constant - J/(K*kg)=N*m/(K*kg) 
double const R=287.053;					//ideal gas constant - J/(K*kg)=N*m/(K*kg) 
double const KBOLTZ=1.38e-23;			//Boltzmann's constant - Ws/K  
//numerical constants
double const PI=3.1415927;				//circumference of unit diameter circle
double const EPS=1.e-10;				//machine precision error (type double)
double const SMALL=1.e-7;				//small real number
int const    ILARGE=9999;				//large integer number
//conversion factors
double const RAD=0.0174532925199432;	//conversion factor deg->rad
double const DEG=57.2957795130823;		//conversion factor rad->deg
double const KG=14.594;					//conversion factor slugs->kg
double const SLUGS=0.068521;			//conversion factor kg->slugs 
double const METER=0.3048;				//conversion factor ft->m
double const FOOT=3.280834;				//conversion factor m->ft			
double const NT=4.448;					//conversion factor lb->N
double const LB=0.22482;				//conversion factor N->lb
double const PA=47.88;					//conversion factor psf(lb/ft^2)->Pa
double const PSF=0.02088;				//conversion factor Pa->psf(lb/ft^2)
double const NM=1.3558;					//conversion factor ft*lb->N*m
double const FTLB=0.73746;				//conversion factor N*m->ft*lb
double const PSI=1.4504e-4;				//conversion factor Pa->PSI(lb/in^2);
double const CUBIN_TO_CUBFT=5.7870e-4;	//conversion factor in^3->ft^3
double const SQIN_TO_SQFT=6.94444e-3;	//conversion factor in^2->ft^2
double const IN_TO_FT=0.083333;			//conversion factor (1/12) in->ft
double const INCH=12;					//conversion factor ft->in
//sizing of char arrays
int const CHARN=31;						//character numbers in variable names
int const CHARL=200;					//character numbers in a line
//verify the following array sizes. If too small, dynamic memory allocation will fail!
int const NFLAT6=250;					//size of 'flat6' module-variable array
int const NMISSILE=900;					//size of 'misssile' module-variable array
int const NFLAT3=40;					//size of 'flat3' module-variable array 
int const NTARGET=20;					//size of 'target' module-variable array
int const NAIRCRAFT=60;					//size of 'aircraft' module-variable array
int const NEVENT=20;					//max number of events
int const NVAR=20;						//max number of variables to be input at every event 
int const NMARKOV=10;					//max number of Markov noise variables
#endif