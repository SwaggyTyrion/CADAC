///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_functions.cpp'
//
// Member functions of 'Cadac' class hierarchy
// Member functions of class 'Variable'
// 
//010628 Created by Peter H Zipfel
//130522 Adapted to MAGSIX simulation, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
#include "global_header.hpp"

///////////////////////////////////////////////////////////////////////////////
//////////////// Member functions of 'Cadac' class hierarchy //////////////////
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//Reading input data from 'input.asc' and putting into 'rotor' arrays 
//Writing banners to screen, 'tabout.asc' and to 'traj.asc' files  
//
//Module-variable arrays:
//	rotor[NROTOR]		contains variables of modules under class 'Rotor'
//	rotor6[nrotor6]	contains variables of all modules with empty slots removed
//	scrn_rotor6[nscrn_rotor6]	contains variables to be displayed at screen and 'tabout.asc'
//	plot_rotor6[nplot_rotor6] contains variables to be plotted, i.e., written to 'traj.asc'
//	com_rotor6[ncom_rotor6] contains variables for communication among vehicles
//  event_ptr_list[NEVENT] event pointer list
//
//Index pointer arrays:	
//	rotor_scrn_ind[rotor_scrn_count];
//	rotor_plot_ind[rotor_plot_count];
//	rotor_com_ind[rotor_com_count];
//
//				  
//001220 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
//130522 Adapted to MAGSIX simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Rotor::Rotor(Module *module_list,int num_modules)
{
	int i(0),j(0);
	//creating module-variable array
	rotor=new Variable[NROTOR];
	if(rotor==0){cerr<<"*** Error: rotor[] allocation failed ***\n";system("pause");system("pause");exit(1);}

	//zeroeing module-variable array
	for(i=0;i<NROTOR;i++)rotor[i].init("empty",0," "," "," "," ");

	//calling initializer modules to build 'rotor' arrays
	//and make other initial calculations in the following sequence

	//call the module definitions -MOD
	for (j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="trajectory")&&(module_list[j].definition=="def"))
				def_trajectory();
			else if((module_list[j].name=="attitude")&&(module_list[j].definition=="def"))
				def_attitude();
		}

	//sizing module-variable arrays 'rotor6','scrn_rotor6','plot_rotor6' arrays
	//their dimensions are the protected data:'nrotor6','nscrn_rotor6','nplot_rotor6'
	sizing_arrays();

	//allocating dynamic memory to the module-variable arrays
	rotor6=new Variable[nrotor6];		
	if(!rotor6){cerr<<"*** Error: rotor6[] allocation failed *** \n";system("pause");system("pause");exit(1);}

	scrn_rotor6=new Variable[nscrn_rotor6];
	if(!scrn_rotor6){cerr<<"*** Error: scrn_rotor6[] allocation failed *** \n";system("pause");exit(1);}

	plot_rotor6=new Variable[nplot_rotor6];
	if(!plot_rotor6){cerr<<"*** Error: plot_rotor6[] allocation failed *** \n";system("pause");exit(1);}

	com_rotor6=new Variable[ncom_rotor6];
	if(!com_rotor6){cerr<<"*** Error: com_rotor6[] allocation failed *** \n";system("pause");exit(1);}

	// allocating memory for the screen index arrays
	rotor_scrn_ind=new int[rotor_scrn_count];

	// allocating memory for the plot index arrays
	rotor_plot_ind=new int[rotor_plot_count];

	// allocating memory for the com index arrays
	rotor_com_ind=new int[rotor_com_count];

	//allocating memory to each event object in event object list
	for (i=0;i<NEVENT;i++)
		event_ptr_list[i]=new Event;

	//initializing the event array index
	nevent=0;
	event_total=0;

	//building 'rotor6' array (compacting 'rotor' arrays)
	vehicle_array();

	//building 'scrn_rotor6' array from 'rotor6' array
	scrn_array();

	//building 'plot_rotor6' array from 'rotor6' array
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

Rotor::~Rotor()
{
	delete [] rotor;
	delete [] rotor6;
	delete [] scrn_rotor6;
	delete [] plot_rotor6;
	delete [] com_rotor6;
	delete [] rotor_scrn_ind;
	delete [] rotor_plot_ind;
	delete [] rotor_com_ind;
	delete [] &event_ptr_list;
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
	double dum=0;//!
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

