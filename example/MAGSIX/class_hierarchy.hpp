///////////////////////////////////////////////////////////////////////////////
//FILE: 'class_hierarchy.hpp'
//
//Contains the classes of the hierarchy of base class 'Cadac'
//
//130522 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_DEPRECATE
#ifndef cadac_class_hierarchy__HPP
#define cadac_class_hierarchy__HPP

#include "global_header.hpp"

///////////////////////////////////////////////////////////////////////////////
//Abstract base class: Cadac
//
//011128 Created by Peter H Zipfel
//130522 Adapted to MAGSIX simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	char name[CHARN]; //vehicle object name
		
protected:

	//array of module-variables as defined in class 'Rotor'
	//first ten locations are reserved for 'com' variables
	Variable *rotor;

public:
	//flag indicating an 'event' has occured
	bool event_epoch;

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
	virtual void document(ostream &fdoc,char *title,Document *doc_rotor6)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_rotor)=0;
	virtual Packet loading_packet(int num_rotor)=0;

	//module functions -MOD
	virtual void def_environment()=0;
	virtual void environment(double int_step)=0;
	virtual void def_trajectory()=0;
	virtual void init_trajectory()=0;
	virtual void trajectory(Packet *combus,int vehicle_slot,double sim_time,double int_step)=0;
	virtual void term_trajectory()=0;
	virtual void def_attitude()=0;
	virtual void init_attitude()=0;
	virtual void attitude(double int_step)=0;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Rotor
//
//Second level of derived class of the 'Cadac' hierarchy
//Models aerodynamics, propulsion, guidance and control 
//Contains Modules: aerodynamics, propulsion, forces, control, guidance 
//
//011128 Created by Peter H Zipfel
//130522 Adapted to MAGSIX simulation, PZi
///////////////////////////////////////////////////////////////////////////////

class Rotor:public Cadac
{
protected:
	//name of ROTOR vehicle object
	char rotor6_name[CHARL];

	//event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT];int nevent;
	//total number of envents for a vehicle object
	int event_total;

	//compacted array of all module-variables of vehicle object 'Rotor'
	Variable *rotor6;int nrotor6;

	//screen ouput array of module-variables of vehicle object 'Rotor'
	Variable *scrn_rotor6;int nscrn_rotor6;

	//plot ouput array of module-variables of vehicle object 'Rotor'
	Variable *plot_rotor6;int nplot_rotor6;

	//communications ouput array of module-variables of vehicle object 'Rotor'
	Variable *com_rotor6;int ncom_rotor6;

	//packet of data for each rotor vehicle
	Packet packet;

	//indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *rotor_scrn_ind; int rotor_scrn_count;

	//indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *rotor_plot_ind; int rotor_plot_count;

	//indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *rotor_com_ind; int rotor_com_count;

	//declaring Table pointer as temporary storage of a single table
	Table *table;
	//	declaring Datadeck 'aerotable' that stores all aero tables
	Datadeck aerotable;
	//	declaring Datadeck 'proptable' that stores all aero tables
	Datadeck proptable;

public:
	Rotor(){};
	Rotor(Module *module_list,int num_modules);
	virtual~Rotor();

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
	virtual void document(ostream &fdoc,char *title,Document *doc_rotor6);
	virtual void com_index_arrays();
	virtual Packet loading_packet_init(int num_rotor);
	virtual Packet loading_packet(int num_rotor);

	//module functions -MOD
	virtual void def_environment();
	virtual void environment(double int_step);
	virtual void def_trajectory();
	virtual void init_trajectory();
	virtual void trajectory(Packet *combus,int vehicle_slot,double sim_time,double int_step);
	virtual void term_trajectory();
	virtual void def_attitude();
	virtual void init_attitude();
	virtual void attitude(double int_step);
  };

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Global class 'Vehicle'//////////////////////////////
///////////// must be located after 'Cadac' hierarchy in this file (why?)//////
///////////////////////////////////////////////////////////////////////////////
//Class 'Vehicle'
//
//Global class for typifying the array of vehicle pointers
//
//010629 Created by Peter Zipfel
//030627 Adapted to MAGSIX simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Vehicle
{
private:
	int capacity;	//max number of vehicles permitted in vehicle list
	int howmany;	//actual number of vehicles in vehicle list
	//'vehicle_ptr' is the pointer to an array of pointers of type 'Cadac' 
	Cadac **vehicle_ptr;
public:
	Vehicle(int number=1);	//constructor, setting capacity, allocating dynamic memory
	virtual ~Vehicle();	//destructor, de-allocating dynamic memory
	void add_vehicle(Cadac &ptr);	//adding vehicle to list
	Cadac *operator[](int position);	//[] operator returns vehicle pointer
	int size();	//returning 'howmany' vehicles are stored in vehicle list
};

#endif