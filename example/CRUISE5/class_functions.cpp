///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_functions.cpp'
// Member functions of 'Cadac' class hierarchy
// Member functions of class 'Variable'
// 
//010628 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
//060510 Updated from F16C for CRUISE, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
#include "global_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//////////////// Member functions of 'Cadac' class hierarchy //////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Constructor initializing   
//
//001220 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Round3::Round3()
{
	//creating module-variable array
	try{round3=new Variable[NROUND3];}
	catch(bad_alloc xa){cerr<<"*** Error: round3[] allocation failed ***\n";system("pause");exit(1);}

	//zeroing module-variable array
	for(int i=0;i<NROUND3;i++)round3[i].init("empty",0," "," "," "," ");
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//Reading input data from 'input.asc' and putting into 'round3' and 'cruise' arrays 
//Writing banners to screen, 'tabout.asc' and to 'traj.asc' files  
//
//Module-variable arrays:
//	cruise[NCRUISE]		contains variables of modules under class 'Cruise'
//	cruise3[ncruise3]	contains variables of all modules with empty slots removed
//	scrn_cruise3[nscrn_cruise3]	contains variables to be displayed at screen and 'tabout.asc'
//	plot_cruise3[nplot_cruise3] contains variables to be plotted, i.e., written to 'traj.asc'
//	com_cruise3[ncom_cruise3] contains variables for communication among vehicles
//  event_ptr_list[NEVENT] event pointer list
//
//Index pointer arrays:	
//	round3_scrn_ind[round3_scrn_count];
//	cruise_scrn_ind[cruise_scrn_count];
//	round3_plot_ind[round3_plot_count];
//	cruise_plot_ind[cruise_plot_count];
//	round3_com_ind[round3_com_count];
//	cruise_com_ind[cruise_com_count];
//
//001220 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Cruise::Cruise(Module *module_list,int num_modules,int num_target,int num_satellite)
{
	//initializing coutners
	int i(0);
	int j(0);

	//creating module-variable array
	try{cruise=new Variable[NCRUISE];}
	catch(bad_alloc xa){cerr<<"*** Error: cruise[] allocation failed ***\n";system("pause");exit(1);}

	//zeroing module-variable array
	for(i=0;i<NCRUISE;i++)cruise[i].init("empty",0," "," "," "," ");

	//calling initializer modules to build 'round3' and 'cruise' arrays
	// and make other initial calculations in the following sequence

	//call the module definitions -MOD
	for (j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="aerodynamics")&&(module_list[j].definition=="def"))
				def_aerodynamics();
			else if((module_list[j].name=="propulsion")&&(module_list[j].definition=="def"))
				def_propulsion();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();
			else if((module_list[j].name=="control")&&(module_list[j].definition=="def"))
				def_control();
			else if((module_list[j].name=="targeting")&&(module_list[j].definition=="def"))
				def_targeting();
			else if((module_list[j].name=="guidance")&&(module_list[j].definition=="def"))
				def_guidance();
			else if((module_list[j].name=="seeker")&&(module_list[j].definition=="def"))
				def_seeker();
			else if((module_list[j].name=="intercept")&&(module_list[j].definition=="def"))
				def_intercept();
		}

	//sizing module-variable arrays 'cruise3','scrn_cruise3','plot_cruise3' arrays
	//their dimensions are the protected data:'ncruise3','nscrn_cruise3','nplot_cruise3'
	sizing_arrays();

	//allocating dynamic memory to the module-variable arrays
	try{cruise3=new Variable[ncruise3];}		
	catch(bad_alloc xa){cerr<<"*** Error: cruise3[] allocation failed *** \n";system("pause");exit(1);}

	try{scrn_cruise3=new Variable[nscrn_cruise3];}
	catch(bad_alloc xa){cerr<<"*** Error: scrn_cruise3[] allocation failed *** \n";system("pause");exit(1);}

	try{plot_cruise3=new Variable[nplot_cruise3];}
	catch(bad_alloc xa){cerr<<"*** Error: plot_cruise3[] allocation failed *** \n";system("pause");exit(1);}

	try{com_cruise3=new Variable[ncom_cruise3];}
	catch(bad_alloc xa){cerr<<"*** Error: com_cruise3[] allocation failed *** \n";system("pause");exit(1);}

	// allocating memory for the screen index arrays
	round3_scrn_ind=new int[round3_scrn_count];
	cruise_scrn_ind=new int[cruise_scrn_count];

	// allocating memory for the plot index arrays
	round3_plot_ind=new int[round3_plot_count];
	cruise_plot_ind=new int[cruise_plot_count];

	// allocating memory for the com index arrays
	round3_com_ind=new int[round3_com_count];
	cruise_com_ind=new int[cruise_com_count];

	//allocating memory for the 'grnd_range' array
	grnd_range=new double[num_target];

	//allocating memory to each event object in event object list
	for (i=0;i<NEVENT;i++)
		try{event_ptr_list[i]=new Event;}
			catch(bad_alloc xa){cerr<<"*** Error: 'event' object allocation failed *** \n";system("pause");exit(1);}

	//allocating memory for satellite visibility array
	visibility=new Targeting[num_satellite];

	//initializing the event array index
	nevent=0;
	event_total=0;
	event_time=0;

	//building 'cruise3' array (compacting and merging 'round3' and 'cruise' arrays)
	vehicle_array();

	//building 'scrn_cruise3' array from 'cruise3' array
	scrn_array();

	//building 'plot_cruise3' array from 'cruise3' array
	plot_array();

	//building the index arrays of the data to be written to the screen
	scrn_index_arrays();

	//building the index arrays of the data to be written to the 'ploti.asc' files
	plot_index_arrays();

	//building the index arrays of the data to be loaded into the packets of 'combus'
	com_index_arrays();
}
///////////////////////////////////////////////////////////////////////////////
//Destructor deallocating dynamic memory
//				  
//010115 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Cruise::~Cruise()
{
	delete [] cruise;
	delete [] cruise3;
	delete [] round3;
	delete [] scrn_cruise3;
	delete [] plot_cruise3;
	delete [] com_cruise3;
	delete [] round3_scrn_ind;
	delete [] cruise_scrn_ind;
	delete [] cruise_plot_ind;
	delete [] round3_com_ind;
	delete [] cruise_com_ind;
	delete [] grnd_range;
	delete [] &event_ptr_list;
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//				  
//010205 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Target::Target(Module *module_list,int num_modules)
{
	//creating module-variable array
	try{target=new Variable[NTARGET];}
	catch(bad_alloc xa){cerr<<"*** Error: target[] allocation failed ***\n";system("pause");exit(1);}

	//zeroing module-variable array
	for(int i=0;i<NTARGET;i++)target[i].init("empty",0," "," "," "," ");
	//calling initializer modules to build 'round3' and 'target' arrays
	//and make other initial calculations 

	//call the module definitions
	for (int j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();
			else if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="intercept")&&(module_list[j].definition=="def"))
				def_intercept();
		}
	//sizing module-variable array 'com_target3'
	sizing_arrays();

	// allocating memory for the com index arrays
	round3_com_ind=new int[round3_com_count];
	target_com_ind=new int[target_com_count];

	try{com_target3=new Variable[ncom_target3];}
	catch(bad_alloc xa){cerr<<"*** Error: com_cruise3[] allocation failed *** \n";system("pause");exit(1);}

	//building the index arrays of the data to be loaded into the packets of 'combus'
	com_index_arrays();
}
///////////////////////////////////////////////////////////////////////////////
//Destructor deallocating dynamic memory
//				  
//010205 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Target::~Target()
{
	delete [] target;
	delete [] round3;
	delete [] round3_com_ind;
	delete [] target_com_ind;
	delete [] com_target3;
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//				  
//010810 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Satellite::Satellite(Module *module_list,int num_modules)
{
	//creating module-variable array
	try{satellite=new Variable[NSATELLITE];}
	catch(bad_alloc xa){cerr<<"*** Error: satellite[] allocation failed ***\n";system("pause");exit(1);}

	//zeroing module-variable array
	for(int i=0;i<NSATELLITE;i++)satellite[i].init("empty",0," "," "," "," ");
	//calling initializer modules to build 'round3' and 'satellite' arrays
	//and make other initial calculations 

	//call the module definitions
	for (int j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();
			else if((module_list[j].name=="seeker")&&(module_list[j].definition=="def"))
				def_seeker();
		}
	//sizing module-variable array 'com_satellite3'
	sizing_arrays();

	// allocating memory for the com index arrays
	round3_com_ind=new int[round3_com_count];
	satellite_com_ind=new int[satellite_com_count];

	try{com_satellite3=new Variable[ncom_satellite3];}
	catch(bad_alloc xa){cerr<<"*** Error: com_cruise3[] allocation failed *** \n";system("pause");exit(1);}

	//building the index arrays of the data to be loaded into the packets of 'combus'
	com_index_arrays();
}
///////////////////////////////////////////////////////////////////////////////
//Destructor deallocating dynamic memory
//				  
//010810 Created by Peter H Zipfel
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
	try{vehicle_ptr=new Cadac *[capacity];}
	catch(bad_alloc xa){cerr<<"*** Error:'vehicle_ptr' allocation failed *** \n";system("pause");exit(1);}
	howmany=0;
//	cerr<<">>> inside constructor of 'Vehicle' <<<\n";
}

///////////////////////////////////////////////////////////////////////////////
//Destructor of class 'Vehicle'
//de-allocating dynamic memory of the pointer-array of type 'Cadac'
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Vehicle::~Vehicle()
{
//	cerr<<">>> inside destructor of 'Vehicle' <<<\n";
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
//Overloading operator [] so that it returns a 'Cadac' pointer
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
//Obtaining size of vehicle list (total number of vehicles)
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int Vehicle::size()
{
	return howmany;
}

