///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_constants.hpp'
//
//Defines all global constant parameters
//
//011128 Created by Peter H Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

#ifndef global_constants__HPP
#define global_constants__HPP

//global constants to be used in the simulation
const double REARTH=6370987.308;		//mean earth radius - m
const double WEII3=7.292115e-5;			//angular rotation of earth - rad/s
const double RAD=0.0174532925199432;	//conversion factor deg->rad
const double DEG=57.2957795130823;		//conversion factor rad->deg
const double FOOT=3.280834;				//conversion factor m->ft			
const double NT=4.448;					//conversion factor lb->N
const double PI=3.1415927;				//circumference of unit diameter circle
const double AGRAV=9.80675445;			//standard value of gravity acceleration - m/s^2 
const double G=6.673e-11;				//universal gravitational constant - Nm^2/kg^2 
const double EARTH_MASS=5.973e24;		//mass of the earth - kg 
const double R=287.053;					//ideal gas constant - m^2/(K*s^2) 
const double EPS=1.e-10;				//machine precision error (type double)
const double SMALL=1.e-7;				//small real number
int const ILARGE=9999;					//large integer number
//sizing of arrays
const int CHARN=31;						//character numbers in variable names
const int CHARL=121;					//character numbers in a line
//verify the following array sizes. If too small, dynamic memory allocation will fail!
const int NFLAT6=250;					//size of 'flat6' module-variable array
const int NPLANE=750;					//size of 'missile' module-variable array
const int NEVENT=20;					//max number of events
const int NVAR=15;						//max number of variables to be input at every event 
#endif