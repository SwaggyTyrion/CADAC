///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_hierarchy.hpp'
//Declares the derived classes 'Round3', 'Cruise''
// of the base class 'Cadac'
// and the global class 'Vehicles'
//
//030627 Created by Peter H Zipfel
//060511 Updated to latest CADAC++ standards, PZi
//060522 Inclusion of 'Satellite' object, PZi
//100505 Modified for GHAME3, PZi
///////////////////////////////////////////////////////////////////////////////

#ifndef cadac_class_hierarchy__HPP
#define cadac_class_hierarchy__HPP

#include "global_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Abstract base class: Cadac
//
//011128 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
//100505 Modified for GHAME3, PZi
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	char name[CHARN]; //vehicle object name
		
protected:
	//module-variable array of class 'Round3'
	//first 10 locations reserved for time and 'com' variables
	Variable *round3;

	//array of module-variables as defined in class 'Cruise'
	//first 10 locations are reserved for 'com' variables
	Variable *cruise;

public:
	//flag indicating an 'event' has occured
	bool event_epoch;

	//time elapsed in event 
	double event_time; //event_time

	virtual~Cadac(){};

	///////////////////////////////////////////////////////////////////////////
	//Constructor of class 'Cadac'
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	Cadac(){}

	///////////////////////////////////////////////////////////////////////////
	//Setting vehicle object name
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_name(char *vehicle_name) {strcpy(name,vehicle_name);}

	///////////////////////////////////////////////////////////////////////////
	//Getting vehicle object name
	//
	//010703 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_vname() {return name;}

	//////////////////////////executive functions /////////////////////////////
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,char *title)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,char *title,Document *doc_cruise3)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_cruise)=0;
	virtual Packet loading_packet(int num_cruise)=0;

	//module functions -MOD
	virtual void def_environment()=0;
	virtual void init_environment(double sim_time,double int_step)=0;
	virtual void environment(double sim_time,double event_time,double &int_step,double &out_fact)=0;
	virtual void def_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_newton()=0;
	virtual void init_newton()=0;
	virtual void newton(double int_step)=0;
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Round3
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitational acceleration and equations of motions
//Contains modules: environment and Newton's law
//
//011128 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
//100505 Modified for GHAME3, PZi
///////////////////////////////////////////////////////////////////////////////
class Round3:public Cadac
{
protected:			
	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *round3_scrn_ind; int round3_scrn_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *round3_plot_ind; int round3_plot_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *round3_com_ind; int round3_com_count;
public:
	Round3();
	virtual~Round3(){};

	//executive functions
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,char *title)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,char *title,Document *doc_cruise3)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_cruise)=0;
	virtual Packet loading_packet(int num_cruise)=0;

	//module functions
	virtual void def_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_forces()=0;
	virtual void forces()=0;

	//virtual functions to be defined in this class
	virtual void def_environment();
	virtual void init_environment(double sim_time,double int_step);
	virtual void environment(double sim_time,double event_time,double &int_step,double &out_fact);
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double int_step);
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Cruise
//
//Second level of derived class of the 'Cadac' hierarchy
//Models aerodynamics, propulsion, guidance and control 
//Contains Modules: aerodynamics, propulsion, forces, control, guidance 
//
//011128 Created by Peter H Zipfel
//030627 Adapted to CRUISE simulation, PZi
//100505 Modified for GHAME3, PZi
///////////////////////////////////////////////////////////////////////////////

class Cruise:public Round3
{
protected:
	//name of CRUISE3 vehicle object
	char cruise3_name[CHARL];

	//event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT];int nevent;
	//total number of envents for a vehicle object
	int event_total;

	//compacted array of all module-variables of vehicle object 'Cruise'
	Variable *cruise3;int ncruise3;

	//screen output array of module-variables of vehicle object 'Cruise'
	Variable *scrn_cruise3;int nscrn_cruise3;

	//plot output array of module-variables of vehicle object 'Cruise'
	Variable *plot_cruise3;int nplot_cruise3;

	//communications output array of module-variables of vehicle object 'Cruise'
	Variable *com_cruise3;int ncom_cruise3;

	//packet of data for each cruise vehicle
	Packet packet;

	//indicator array pointing to the module-variable which are to 
	// be written to the screen
	int *cruise_scrn_ind; int cruise_scrn_count;

	//indicator array pointing to the module-variable which are to 
	// be written to the 'ploti.asc' files
	int *cruise_plot_ind; int cruise_plot_count;

	//indicator array pointing to the module-variable which are to 
	// be written to 'combus' 'packets'
	int *cruise_com_ind; int cruise_com_count;

	//declaring Table pointer as temporary storage of a single table
	Table *table;
	//	declaring Datadeck 'aerotable' that stores all aerodynamic tables
	Datadeck aerotable;
	//	declaring Datadeck 'proptable' that stores all propulsion tables
	Datadeck proptable;

public:
	Cruise(){};
	Cruise(Module *module_list,int num_modules);
	virtual~Cruise();

	//executive functions
	virtual void sizing_arrays();
	virtual void vehicle_array();
	virtual void scrn_array();
	virtual void plot_array();
	virtual void scrn_banner();
	virtual void tabout_banner(ofstream &ftabout,char *title);
	virtual void tabout_data(ofstream &ftabout);
	virtual void vehicle_data(fstream &input);
	virtual void read_tables(char *file_name,Datadeck &datatable);
	virtual void scrn_index_arrays();
	virtual void scrn_data();
	virtual void plot_banner(ofstream &fplot,char *title);
	virtual void plot_index_arrays();
	virtual void plot_data(ofstream &fplot,bool merge);
	virtual void event(char *options);
	virtual void document(ostream &fdoc,char *title,Document *doc_cruise3);
	virtual void com_index_arrays();
	virtual Packet loading_packet_init(int num_cruise);
	virtual Packet loading_packet(int num_cruise);

	//module functions
	virtual void def_aerodynamics();
	virtual void aerodynamics(); 
	virtual void def_propulsion();
	virtual void init_propulsion();
	virtual void propulsion(double int_step);
	virtual void def_forces();
	virtual void forces();
  };
///////////////////////////////////////////////////////////////////////////////
////////////////////////// Global class 'Vehicle'//////////////////////////////
///////////// must be located after 'Cadac' hierarchy in this file ////////////
///////////////////////////////////////////////////////////////////////////////
//Class 'Vehicle'
//
//Global class for typifying the array of vehicle pointers
//
//010629 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Vehicle
{
private:
	int capacity;	//max number of vehicles permitted in vehicle list
	int howmany;	//actual number of vehicles in vehicle list 
	Cadac **vehicle_ptr; //'vehicle_ptr' is the pointer to an array of pointers of type 'Cadac'
public:
	Vehicle(int number=1);	//constructor, setting capacity, allocating dynamic memory
	virtual ~Vehicle();	//destructor, de-allocating dynamic memory
	void add_vehicle(Cadac &ptr);	//adding vehicle to list
	Cadac *operator[](int position);	//[] operator returns vehicle pointer
	int size();	//returning 'howmany' vehicles are stored in vehicle list
};

#endif