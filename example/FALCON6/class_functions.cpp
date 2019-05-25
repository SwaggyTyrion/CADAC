///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_functions.cpp'
//
// Member functions of 'Cadac' class hierarchy
// Member functions of class 'Variable'
// 
//010628 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
#include "global_header.hpp"

///////////////////////////////////////////////////////////////////////////////
//////////////// Member functions of 'Cadac' class hierarchy //////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Constructor initializing   
//
//001220 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Flat6::Flat6()
{
	//creating module-variable array
	flat6=new Variable[NFLAT6];
	if(flat6==0){cerr<<"*** Error: flat6[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NFLAT6;i++)flat6[i].init("empty",0," "," "," "," ");
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//Reading input data from 'input.asc' and putting into 'flat6' and 'plane' arrays 
//Writing banners to screen, 'tabout.asc' and to 'traj.asc' files  
//
//Module-variable arrays:
//	plane[NPLANE]		contains variables of modules under class 'Plane'
//	plane6[nplane6]	contains variables of all modules with empty slots removed
//	scrn_plane6[nscrn_plane6]	contains variables to be displayed at screen and 'tabout.asc'
//	plot_plane6[nplot_plane6] contains variables to be plotted, i.e., written to 'traj.asc'
//	com_plane6[ncom_plane6] contains variables for communication among vehicles
//  event_ptr_list[NEVENT] event pointer list
//
//Index pointer arrays:	
//	flat6_scrn_ind[flat6_scrn_count];
//	plane_scrn_ind[plane_scrn_count];
//	flat6_plot_ind[flat6_plot_count];
//	plane_plot_ind[plane_plot_count];
//	flat6_com_ind[flat6_com_count];
//	plane_com_ind[plane_com_count];
//
//				  
//001220 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Plane::Plane(Module *module_list,int num_modules)
{
	int i(0),j(0);
	//creating module-variable array
	plane=new Variable[NPLANE];
	if(plane==0){cerr<<"*** Error: plane[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(i=0;i<NPLANE;i++)plane[i].init("empty",0," "," "," "," ");

	//calling initializer modules to build 'flat6' and 'plane' arrays
	//and make other initial calculations in the following sequence

	//call the module definitions -MOD
	for (j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="kinematics")&&(module_list[j].definition=="def"))
				def_kinematics();
			else if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="euler")&&(module_list[j].definition=="def"))
				def_euler();
			else if((module_list[j].name=="aerodynamics")&&(module_list[j].definition=="def"))
				def_aerodynamics();
			else if((module_list[j].name=="propulsion")&&(module_list[j].definition=="def"))
				def_propulsion();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();
			else if((module_list[j].name=="actuator")&&(module_list[j].definition=="def"))
				def_actuator();
			else if((module_list[j].name=="control")&&(module_list[j].definition=="def"))
				def_control();
			else if((module_list[j].name=="guidance")&&(module_list[j].definition=="def"))
				def_guidance();
		}

	//sizing module-variable arrays 'plane6','scrn_plane6','plot_plane6' arrays
	//their dimensions are the protected data:'nplane6','nscrn_plane6','nplot_plane6'
	sizing_arrays();

	//allocating dynamic memory to the module-variable arrays
	plane6=new Variable[nplane6];		
	if(!plane6){cerr<<"*** Error: plane6[] allocation failed *** \n";system("pause");exit(1);}

	scrn_plane6=new Variable[nscrn_plane6];
	if(!scrn_plane6){cerr<<"*** Error: scrn_plane6[] allocation failed *** \n";system("pause");exit(1);}

	plot_plane6=new Variable[nplot_plane6];
	if(!plot_plane6){cerr<<"*** Error: plot_plane6[] allocation failed *** \n";system("pause");exit(1);}

	com_plane6=new Variable[ncom_plane6];
	if(!com_plane6){cerr<<"*** Error: com_plane6[] allocation failed *** \n";system("pause");exit(1);}

	// allocating memory for the screen index arrays
	flat6_scrn_ind=new int[flat6_scrn_count];
	plane_scrn_ind=new int[plane_scrn_count];

	// allocating memory for the plot index arrays
	flat6_plot_ind=new int[flat6_plot_count];
	plane_plot_ind=new int[plane_plot_count];

	// allocating memory for the com index arrays
	flat6_com_ind=new int[flat6_com_count];
	plane_com_ind=new int[plane_com_count];

	//allocating memory to each event object in event object list
	for (i=0;i<NEVENT;i++)
		event_ptr_list[i]=new Event;

	//initializing the event array index
	nevent=0;
	event_total=0;

	//building 'plane6' array (compacting and merging 'flat6' and 'plane' arrays)
	vehicle_array();

	//building 'scrn_plane6' array from 'plane6' array
	scrn_array();

	//building 'plot_plane6' array from 'plane6' array
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

Plane::~Plane()
{
	delete [] plane;
	delete [] plane6;
	delete [] flat6;
	delete [] scrn_plane6;
	delete [] plot_plane6;
	delete [] com_plane6;
	delete [] flat6_scrn_ind;
	delete [] plane_scrn_ind;
	delete [] plane_plot_ind;
	delete [] flat6_com_ind;
	delete [] plane_com_ind;
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

