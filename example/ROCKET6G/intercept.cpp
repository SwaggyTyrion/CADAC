///////////////////////////////////////////////////////////////////////////////
//FILE: 'intercept.cpp'
//Contains 'inercept' module of class 'Hyper'
//
//030619 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of intercept module-variables
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[650-699]
//
//040619 Created by Peter H Zipfel
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_intercept()
{
	//definition of module-variables
	hyper[651].init("write","int",1,"True flag for writing miss to console - ND","intercept","init","");
	hyper[659].init("modes","int",0,"|mguide|maut|mprop| - ND","intercept","diag","scrn,plot");
}
///////////////////////////////////////////////////////////////////////////////
//Intercept module
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[650-699]
//
//Termination if impact on ground occurs
//
//030619 Created by Peter H Zipfel
//091214 Modified for ROCKET6, PZi
///////////////////////////////////////////////////////////////////////////////

void Hyper::intercept(Packet *combus,int num_vehicles,int vehicle_slot,double int_step,char *title)
{
	//local module-variables
	int modes(0);
	//localizing module-variables
	//initializations
	int write=hyper[651].integer();
	//input from other modules
	double time=round6[0].real();
	double alt=round6[221].real();
	double dvbe=round6[225].real();
	double psivdx=round6[228].real();
	double thtvdx=round6[229].real();
	int mprop=hyper[10].integer();
	int mguide=hyper[400].integer();
	int maut=hyper[500].integer();
//-----------------------------------------------------------------------------
	//decoding control flag
    int mauty=maut/10;
    int mautp=(maut%10);

	modes=(int)(mguide*1000+mauty*100+mautp*10+mprop);

	//ground impact
	if((alt<=0)&&write)
	{
		write=0;

		//getting hyper #
		string id_hyper=combus[vehicle_slot].get_id();

		//writing miss information to console
		cout<<"\n"<<" ***"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***";
		cout<<"\n"<<" *** Ground impact or 'alt'= #QNAN  of Hyper_"<<id_hyper<<"   Time = "<<time<<" sec ***\n";
		cout<<"      speed = "<<dvbe<<" m/s  heading = "<<psivdx<<" deg      gamma = "<<thtvdx<<" deg\n\n";    

		//declaring hyper 'dead'
		combus[vehicle_slot].set_status(0);
	}
//-----------------------------------------------------------------------------
	//loading module-variables
	//saving for next cycle
	hyper[651].gets(write);
	//diagnostics
	hyper[659].gets(modes);
}
