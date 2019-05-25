///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_functions.cpp'
//
// Member functions of 'Cadac' class hierarchy
// Member functions of class 'Variable'
// 
//010628 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
//081010 Adapted to GENSIM simulation, PZi
//130724 Building AIM5, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
#include "global_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//////////////// Member functions of 'Cadac' class hierarchy //////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Constructor allocating array memeory and initializing  
//
//001220 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
Flat3::Flat3()
{
	//creating module-variable array
	flat3=new Variable[NFLAT3];
	if(flat3==0){cerr<<"*** Error: flat3[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NFLAT3;i++)flat3[i].init("empty",0," "," "," "," ");
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//				  
//010205 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
//081010 Adapted to GENSIM simulation, PZi
//130724 Building AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
Aim::Aim(Module *module_list,int num_modules)
{
	//creating module-variable array
	aim=new Variable[NAIM];
	if(aim==0){cerr<<"*** Error: aim[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NAIM;i++)aim[i].init("empty",0," "," "," "," ");
	//calling initializer modules to build 'flat3' and 'aim' arrays
	//and make other initial calculations 

	//call the module definitions
	for (int j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="kinematics")&&(module_list[j].definition=="def"))
				def_kinematics();
			else if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="aerodynamics")&&(module_list[j].definition=="def"))
				def_aerodynamics();
			else if((module_list[j].name=="propulsion")&&(module_list[j].definition=="def"))
				def_propulsion();
			else if((module_list[j].name=="seeker")&&(module_list[j].definition=="def"))
				def_seeker();
			else if((module_list[j].name=="guidance")&&(module_list[j].definition=="def"))
				def_guidance();
			else if((module_list[j].name=="control")&&(module_list[j].definition=="def"))
				def_control();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();
			else if((module_list[j].name=="intercept")&&(module_list[j].definition=="def"))
				def_intercept();
		}

	//sizing module-variable array 'com_aim5'
	sizing_arrays();

	//allocating dynamic memory to the module-variable arrays
	aim5=new Variable[naim5];		
	if(!aim5){cerr<<"*** Error: aim5[] allocation failed *** \n";system("pause");exit(1);}

	scrn_aim5=new Variable[nscrn_aim5];
	if(!scrn_aim5){cerr<<"*** Error: scrn_aim5[] allocation failed *** \n";system("pause");exit(1);}

	plot_aim5=new Variable[nplot_aim5];
	if(!plot_aim5){cerr<<"*** Error: plot_aim5[] allocation failed *** \n";system("pause");exit(1);}

	com_aim5=new Variable[ncom_aim5];
	if(!com_aim5){cerr<<"*** Error: com_aim5[] allocation failed *** \n";system("pause");exit(1);}

	// allocating memory for the screen index arrays
	flat3_scrn_ind=new int[flat3_scrn_count];
	aim_scrn_ind=new int[aim_scrn_count];

	// allocating memory for the plot index arrays
	flat3_plot_ind=new int[flat3_plot_count];
	aim_plot_ind=new int[aim_plot_count];

	// allocating memory for the com index arrays
	flat3_com_ind=new int[flat3_com_count];
	aim_com_ind=new int[aim_com_count];


	//allocating memory to each event object in event object list
	for (int i=0;i<NEVENT;i++)
		event_ptr_list[i]=new Event;

	//initializing the event array index
	nevent=0;
	event_total=0;

	//building 'aim5' array (compacting and merging 'flat3' and 'aim' arrays)
	vehicle_array();

	//building 'scrn_aim5' array from 'aim5' array
	scrn_array();

	//building 'plot_aim5' array from 'aim5' array
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
//010205 Created by Peter H Zipfel
//011129 Adapted to SRAAM6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
Aim::~Aim()
{
	delete [] aim;
	delete [] aim5;
	delete [] flat3;
	delete [] scrn_aim5;
	delete [] plot_aim5;
	delete [] com_aim5;
	delete [] flat3_scrn_ind;
	delete [] aim_scrn_ind;
	delete [] aim_plot_ind;
	delete [] flat3_com_ind;
	delete [] aim_com_ind;
	delete [] &event_ptr_list;
}
///////////////////////////////////////////////////////////////////////////////
//Constructor initializing the modules and the module-variable arrays
//				  
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Aircraft::Aircraft(Module *module_list,int num_modules)
{
	//creating module-variable array
	aircraft=new Variable[NAIRCRAFT];
	if(aircraft==0){cerr<<"*** Error: aircraft[] allocation failed ***\n";system("pause");exit(1);}

	//zeroeing module-variable array
	for(int i=0;i<NAIRCRAFT;i++)aircraft[i].init("empty",0," "," "," "," ");
	//calling initializer modules to build 'flat3' and 'aircraft' arrays
	//and make other initial calculations 

	//call the module definitions
	for (int j=0;j<num_modules;j++)
		{
			if((module_list[j].name=="kinematics")&&(module_list[j].definition=="def"))
				def_kinematics();
			else if((module_list[j].name=="environment")&&(module_list[j].definition=="def"))
				def_environment();
			else if((module_list[j].name=="newton")&&(module_list[j].definition=="def"))
				def_newton();
			else if((module_list[j].name=="guidance")&&(module_list[j].definition=="def"))
				def_guidance();
			else if((module_list[j].name=="control")&&(module_list[j].definition=="def"))
				def_control();
			else if((module_list[j].name=="forces")&&(module_list[j].definition=="def"))
				def_forces();
		}
	//sizing module-variable array 'com_aircraft3'
	sizing_arrays();

	// allocating memory for the com index arrays
	flat3_com_ind=new int[flat3_com_count];
	if(!flat3_com_ind){cerr<<"*** Error: flat3_com_count[] allocation failed *** \n";system("pause");exit(1);}
	aircraft_com_ind=new int[aircraft_com_count];
	if(!aircraft_com_ind){cerr<<"*** Error: aircraft_com_count[] allocation failed *** \n";system("pause");exit(1);}
	com_aircraft3=new Variable[ncom_aircraft3];
	if(!com_aircraft3){cerr<<"*** Error: com_aircraft[] allocation failed *** \n";system("pause");exit(1);}

	//building the index arrays of the data to be loaded into the packets of 'combus'
	com_index_arrays();

}
///////////////////////////////////////////////////////////////////////////////
//Destructor deallocating dynamic memory
//				  
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Aircraft::~Aircraft()
{
	delete [] aircraft;
	delete [] flat3;
	delete [] flat3_com_ind;
	delete [] aircraft_com_ind;
	delete [] com_aircraft3;
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
	catch(bad_alloc xa){cerr<<"*** Allocation failure of 'vehicle_ptr' *** \n";system("pause");exit(1);}
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
//Optaining size of vehicle list (total number of vehicles)
//
//010626 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int Vehicle::size()
{
	return howmany;
}

