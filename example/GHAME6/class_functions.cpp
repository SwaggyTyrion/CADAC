///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_functions.cpp'
//
// Member functions of 'Cadac' class hierarchy
// Member functions of class 'Variable'
// 
//010628 Created by Peter H Zipfel
//030415 Adopted for HYPER, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
#include "global_header.hpp"

///////////////////////////////////////////////////////////////////////////////
//////////////// Member functions of 'Cadac' class hierarchy //////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Constructor initializing   
//
//010220 Created by Peter H Zipfel
//030415 Adopted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Round6::Round6()
{
	//creating module-variable array
	round6=new Variable[NROUND6];
	if(round6==0){cerr<<"*** Error: round6[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NROUND6;i++)round6[i].init("empty",0," "," "," "," ");
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//Reading input data from 'input.asc' and putting into 'round6' and 'hyper' arrays 
//Writing banners to screen, 'tabout.asc' and to 'traj.asc' files  
//
//Module-variable arrays:
//	hyper[NHYPER]		contains variables of modules under class 'Hyper'
//	hyper6[nhyper6]	contains variables of all modules with empty slots removed
//	scrn_hyper6[nscrn_hyper6]	contains variables to be displayed at screen and 'tabout.asc'
//	plot_hyper6[nplot_hyper6] contains variables to be plotted, i.e., written to 'traj.asc'
//	com_hyper6[ncom_hyper6] contains variables for communication among vehicles
//  event_ptr_list[NEVENT] event pointer list
//
//Index pointer arrays:	
//	round6_scrn_ind[round6_scrn_count];
//	hyper_scrn_ind[hyper_scrn_count];
//	round6_plot_ind[round6_plot_count];
//	hyper_plot_ind[hyper_plot_count];
//	round6_com_ind[round6_com_count];
//	hyper_com_ind[hyper_com_count];
//
//				  
//001220 Created by Peter H Zipfel
//030415 Adopted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Hyper::Hyper(Module *module_list,int num_modules,int num_satellite,int num_radar)
{
	//creating module-variable array
	hyper=new Variable[NHYPER];
	if(hyper==0){cerr<<"*** Error: hyper[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	int i(0);
	for(i=0;i<NHYPER;i++)hyper[i].init("empty",0," "," "," "," ");

	//calling initializer modules to build 'round6' and 'hyper' arrays
	//and make other initial calculations in the following sequence

	//call the module definitions -MOD: insert here new module definition function
	for (int j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="kinematics")&&(module_list[j].definition=="def"))
				def_kinematics();
			else if((module_list[j].name=="euler")&&(module_list[j].definition=="def"))
				def_euler();
			else if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="aerodynamics")&&(module_list[j].definition=="def"))
				def_aerodynamics();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();
			else if((module_list[j].name=="propulsion")&&(module_list[j].definition=="def"))
				def_propulsion();
			else if((module_list[j].name=="actuator")&&(module_list[j].definition=="def"))
				def_actuator();
			else if((module_list[j].name=="control")&&(module_list[j].definition=="def"))
				def_control();
			else if((module_list[j].name=="ins")&&(module_list[j].definition=="def"))
				def_ins();
			else if((module_list[j].name=="guidance")&&(module_list[j].definition=="def"))
				def_guidance();
			else if((module_list[j].name=="intercept")&&(module_list[j].definition=="def"))
				def_intercept();
			else if((module_list[j].name=="gps")&&(module_list[j].definition=="def"))
				def_gps();
			else if((module_list[j].name=="startrack")&&(module_list[j].definition=="def"))
				def_startrack();
			else if((module_list[j].name=="rcs")&&(module_list[j].definition=="def"))
				def_rcs();
			else if((module_list[j].name=="datalink")&&(module_list[j].definition=="def"))
				def_datalink();
			else if((module_list[j].name=="seeker")&&(module_list[j].definition=="def"))
				def_seeker();
		}
	//sizing module-variable arrays 'hyper6','scrn_hyper6','plot_hyper6' arrays
	//their dimensions are the protected data:'nhyper6','nscrn_hyper6','nplot_hyper6'
	sizing_arrays();

	//allocating dynamic memory to the module-variable arrays
	hyper6=new Variable[nhyper6];		
	if(!hyper6){cerr<<"*** Error: hyper6[] allocation failed *** \n";system("pause");exit(1);}

	scrn_hyper6=new Variable[nscrn_hyper6];
	if(!scrn_hyper6){cerr<<"*** Error: scrn_hyper6[] allocation failed *** \n";system("pause");exit(1);}

	plot_hyper6=new Variable[nplot_hyper6];
	if(!plot_hyper6){cerr<<"*** Error: plot_hyper6[] allocation failed *** \n";system("pause");exit(1);}

	com_hyper6=new Variable[ncom_hyper6];
	if(!com_hyper6){cerr<<"*** Error: com_hyper6[] allocation failed *** \n";system("pause");exit(1);}

	// allocating memory for the screen index arrays
	round6_scrn_ind=new int[round6_scrn_count];
	hyper_scrn_ind=new int[hyper_scrn_count];

	// allocating memory for the plot index arrays
	round6_plot_ind=new int[round6_plot_count];
	hyper_plot_ind=new int[hyper_plot_count];

	// allocating memory for the com index arrays
	round6_com_ind=new int[round6_com_count];
	hyper_com_ind=new int[hyper_com_count];

	//allocating memory for the 'grnd_range' array
	grnd_range=new double[num_satellite];

	//allocating memory to each event object in event object list
	for (i=0;i<NEVENT;i++)
		event_ptr_list[i]=new Event;

	//initializing the event array index
	nevent=0;
	event_total=0;

	//building 'hyper6' array (compacting and merging 'round6' and 'hyper' arrays)
	vehicle_array();

	//building 'scrn_hyper6' array from 'hyper6' array
	scrn_array();

	//building 'plot_hyper6' array from 'hyper6' array
	plot_array();

	//building the index arrays of the data to be written to the screen
	scrn_index_arrays();

	//building the index arrays of the data to be written to the 'ploti.asc' files
	plot_index_arrays();

	//building the index arrays of the data to be loaded into the packets of 'combus'
	com_index_arrays();

	//initializing the indices in the 'markov_list' to large integers
	for(i=0;i<NMARKOV;i++)
		markov_list[i].set_markov_round6_index(ILARGE);
	nmarkov=0;
}
///////////////////////////////////////////////////////////////////////////////
//Destructor deallocating dynamic memory
//				  
//010115 Created by Peter H Zipfel
//030415 Adopted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Hyper::~Hyper()
{
	delete [] hyper;
	delete [] hyper6;
	delete [] round6;
	delete [] scrn_hyper6;
	delete [] plot_hyper6;
	delete [] com_hyper6;
	delete [] round6_scrn_ind;
	delete [] hyper_scrn_ind;
	delete [] hyper_plot_ind;
	delete [] round6_com_ind;
	delete [] hyper_com_ind;
	delete [] grnd_range;
	delete [] &event_ptr_list;
}
///////////////////////////////////////////////////////////////////////////////
//Constructor allocating array memory and initializing  
//
//001220 Created by Peter H Zipfel
//030415 Adopted for HYPER, PZi
///////////////////////////////////////////////////////////////////////////////

Round3::Round3()
{
	//creating module-variable array
	round3=new Variable[NROUND3];
	if(round3==0){cerr<<"*** Error: round3[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NROUND3;i++)round3[i].init("empty",0," "," "," "," ");
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//				  
//010205 Created by Peter H Zipfel
//030415 Adopted for HYPER, PZi
///////////////////////////////////////////////////////////////////////////////

Satellite::Satellite(Module *module_list,int num_modules)
{
	//creating module-variable array
	satellite=new Variable[NSAT];
	if(satellite==0){cerr<<"*** Error: satellite[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NSAT;i++)satellite[i].init("empty",0," "," "," "," ");
	//calling initializer modules to build 'round3' and 'satellite' arrays
	//and make other initial calculations 

	//call the module definitions
	for (int j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();

		}
	//sizing module-variable array 'com_satellite3'
	sizing_arrays();

	// allocating memory for the com index arrays
	round3_com_ind=new int[round3_com_count];
	satellite_com_ind=new int[satellite_com_count];

	com_satellite3=new Variable[ncom_satellite3];
	if(!com_satellite3){cerr<<"*** Error: com_hyper6[] allocation failed *** \n";system("pause");exit(1);}

	//building the index arrays of the data to be loaded into the packets of 'combus'
	com_index_arrays();

}
///////////////////////////////////////////////////////////////////////////////
//Constructor allocating array memory and initializing  
//
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Ground0::Ground0()
{
	//creating module-variable array
	ground0=new Variable[NGROUND0];
	if(ground0==0){cerr<<"*** Error: ground0[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NGROUND0;i++) ground0[i].init("empty",0," "," "," "," ");
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//				  
//030110 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Radar::Radar(Module *module_list,int num_modules)
{
	//creating module-variable array
	radar=new Variable[NRADAR];
	if(radar==0){cerr<<"*** Error: radar[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NRADAR;i++)radar[i].init("empty",0," "," "," "," ");
	//calling initializer modules to build 'round3' and 'radar' arrays
	//and make other initial calculations 

	//call the module definitions
	for (int j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="kinematics")&&(module_list[j].definition=="def"))
				def_kinematics();
			else if((module_list[j].name=="seeker")&&(module_list[j].definition=="def"))
				def_seeker();
		}
	//sizing module-variable array 'com_radar3'
	sizing_arrays();

	// allocating memory for the com index arrays
	ground0_com_ind=new int[ground0_com_count]; 
	radar_com_ind=new int[radar_com_count];

	com_radar0=new Variable[ncom_radar0];
	if(!com_radar0){cerr<<"*** Error: com_hyper6[] allocation failed *** \n";system("pause");exit(1);}

	//building the index arrays of the data to be loaded into the packets of 'combus'
	com_index_arrays();

}
///////////////////////////////////////////////////////////////////////////////
//Destructor deallocating dynamic memory
//				  
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Satellite::~Satellite()
{
	delete [] satellite;
	delete [] round3;
	delete [] round3_com_ind;
	delete [] satellite_com_ind;
	delete [] com_satellite3;
}

///////////////////////////////////////////////////////////////////////////////
//Destructor deallocating dynamic memory
//				  
//040517 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Radar::~Radar()
{
	delete [] radar;
	delete [] ground0;
	delete [] ground0_com_ind;
	delete [] radar_com_ind;
	delete [] com_radar0;
}

///////////////////////////////////////////////////////////////////////////////
//////////////////// Members of class 'Vehicle' ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Constructor of class 'Vehicle'
//allocating dynamic memory for the array of pointers of type 'Cadac'
//and returning the pointer to array
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Vehicle::Vehicle(int number)
{
	capacity=number;
	vehicle_ptr=new Cadac *[capacity];
	if(vehicle_ptr==0){cerr<<"*** Error:'vehicle_ptr' allocation failed *** \n";system("pause");exit(1);}
	howmany=0;
}

///////////////////////////////////////////////////////////////////////////////
//Destructor of class 'Vehicle'
//de-allocating dynamic memory of the pointer-array of type 'Cadac'
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Vehicle::~Vehicle()
{
	delete [] vehicle_ptr;
}

///////////////////////////////////////////////////////////////////////////////
//Adding a vehicle pointer to the vehicle list
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Vehicle::add_vehicle(Cadac &pt)
{
	if(howmany<capacity)
		vehicle_ptr[howmany++]=&pt;			
}
///////////////////////////////////////////////////////////////////////////////
//Overloaded operator [] returns a 'Cadac' pointer
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Cadac * Vehicle::operator[](int position)
{
	if(position>=0 && position<howmany)
		return vehicle_ptr[position];
	else
	{
		cout<<"*** Bad value: "<<position<<'\n';
		return 0;
	}
}

///////////////////////////////////////////////////////////////////////////////
//Optaining size of vehicle list (total number of vehicles)
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int Vehicle::size()
{
	return howmany;
}

