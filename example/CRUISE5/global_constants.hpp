///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_constants.hpp'
//Defines all global constant parameters
//
//011128 Created by Peter H Zipfel
//060510 Updated from F16C for CRUISE, PZi
///////////////////////////////////////////////////////////////////////////////

#ifndef global_constants__HPP
#define global_constants__HPP

using namespace std;

//global constants to be used in the simulation
double const REARTH=6370987.308;		//mean earth radius - m
double const WEII3=7.292115e-5;			//angular rotation of earth - rad/s
double const RAD=0.0174532925199432;	//conversion factor deg->rad
double const DEG=57.2957795130823;		//conversion factor rad->deg
double const AGRAV=9.80675445;			//standard value of gravity acceleration - m/s^2 
double const G=6.673e-11;				//universal gravitational constant - Nm^2/kg^2 
double const EARTH_MASS=5.973e24;		//mass of the earth - kg 
double const R=287.053;					//ideal gas constant - m^2/(K*s^2) 
double const PI=3.1415927;				//circumference of unit diameter circle
double const EPS=1e-10;					//machine precision error
const double SMALL=1.e-7;				//small real number
int const ILARGE=9999;					//large integer number
double const BIG=1e10;					//big number
//sizing of arrays
int const CHARN=40;						//character numbers in variable names
int const CHARL=150;					//character numbers in a line
//verify the following array sizes. If too small, dynamic memory allocations may fail!
int const NROUND3=40;					//size of 'round3' module-variable array
int const NCRUISE=160;					//size of 'cruise' module-variable array 
int const NTARGET=20;					//size of 'target' module-variable array
int const NSATELLITE=20;				//size of 'satellite' module-variable array
int const NEVENT=25;					//max number of events
int const NVAR=15;						//max number of variables to be input at every event 

#endif