///////////////////////////////////////////////////////////////////////////////
//FILE: class_hierarchy.hpp
//
//Contains the classes of the hierarchy of base class 'Cadac'
//
//011128 Created by Peter H Zipfel
//081010 Modified for GENSIM6, PZi
//130724 Building AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_DEPRECATE
#ifndef cadac_class_hierarchy__HPP
#define cadac_class_hierarchy__HPP

#include "global_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Abstract base class: Cadac
//
//011128 Created by Peter H Zipfel
//070411 Included Aircraft class, PZi
//130724 Building AIM5, PZi
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	char name[CHARN]; //vehicle object name
		
protected:

	//module-variable array of class 'Flat3'
	//first 10 locations reserved for time and 'com' variables
	Variable *flat3;

	//Array of module-variables as defined in class 'Aim'
	//first ten locations are reserved for 'com' variables
	Variable *aim;

	//Array of module-variables as defined in class 'Aircraft'
	//first ten locations are reserved for 'com' variables
	Variable *aircraft;

public:
	//flag indicating an 'event' has occured
	bool event_epoch;

	//time elapsed in event 
	double event_time;

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
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_aircraft,int num_aim)=0;
	virtual Packet loading_packet(int num_aircraft,int num_aim)=0;

	//module functions -MOD
	virtual void def_environment()=0;
	virtual void environment()=0;
	virtual void def_kinematics()=0;
	virtual void init_kinematics(double sim_time)=0;
	virtual void kinematics(double sim_time,double event_time)=0;
	virtual void def_newton()=0;
	virtual void init_newton()=0;
	virtual void newton(double int_step)=0;
	virtual void def_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void propulsion()=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_control()=0;
	virtual void init_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance(Packet *combus,int num_vehicles)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Flat3
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitational acceleration and equations of motions
//Contains modules: environment, kinematics and Newton's law
//
//010116 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Flat3:public Cadac
{
protected:			

	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *flat3_scrn_ind; int flat3_scrn_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *flat3_com_ind; int flat3_com_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *flat3_plot_ind; int flat3_plot_count;

public:
	Flat3();
	virtual~Flat3(){};

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
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_aircraft,int num_aim)=0;
	virtual Packet loading_packet(int num_aircraft,int num_aim)=0;

	//module functions -MOD
	virtual void def_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void propulsion()=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_control()=0;
	virtual void init_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance(Packet *combus,int num_vehicles)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;

	//virtual functions to be declared in this class
	virtual void def_environment();
	virtual void environment();
	virtual void def_kinematics();
	virtual void init_kinematics(double sim_time);
	virtual void kinematics(double sim_time,double event_time);
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double int_step);
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class:Aim
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Flat3'
//Models AIM movements
//Contains Module 'forces'
//
//010205 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Aim:public Flat3
{
protected:
	//name of AIM5 vehicle object
	char aim5_name[CHARL];

	//Event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT];int nevent;

	//total number of envents for a vehicle object
	int event_total;

	//Compacted array of all module-variables of vehicle object 'Aim'
	Variable *aim5;int naim5;

	//Screen output array of module-variables of vehicle object 'Aim'
	Variable *scrn_aim5;int nscrn_aim5;

	//Plot output array of module-variables of vehicle object 'Aim'
	Variable *plot_aim5;int nplot_aim5;

	//Communications output array of module-variables of vehicle object 'Aim'
	Variable *com_aim5;int ncom_aim5;

	//Packet of data for each aim vehicle
	Packet packet;

	//indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *aim_scrn_ind; int aim_scrn_count;

	//indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *aim_plot_ind; int aim_plot_count;

	//indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *aim_com_ind; int aim_com_count;

	//declaring Table pointer as temporary storage of a single table
	Table *table;
	//declaring Datadeck 'aerotable' that stores all aero tables
	Datadeck aerotable;
	//declaring Datadeck 'proptable' that stores all aero tables
	Datadeck proptable;

public:
	Aim(){};
	Aim(Module *module_list,int num_modules);
	virtual~Aim();

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
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle);
	virtual void com_index_arrays();
	virtual Packet loading_packet_init(int num_aircraft,int num_aim);
	virtual Packet loading_packet(int num_aircraft,int num_aim);

	//module functions active
	virtual void def_aerodynamics();
	virtual void aerodynamics(); 
	virtual void def_propulsion();
	virtual void propulsion();
	virtual void def_seeker();
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step);
	virtual void def_control();
	virtual void init_control();
	virtual void control(double int_step);
	virtual void def_guidance();
	virtual void guidance(Packet *combus,int num_vehicles);
	virtual void def_forces();
	virtual void forces();
	virtual void def_intercept();
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title);
};
///////////////////////////////////////////////////////////////////////////////
//Derived class:Aircraft
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Flat3'
//Models aircraft flight path and aim track files
//Contains Module 'forces'
//
//070411 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Aircraft:public Flat3
{
protected:
	//name of AIRCRAFT3 vehicle object
	char aircraft3_name[CHARL];

	//Communications output array of module-variables of vehicle object 'Aircraft'
	Variable *com_aircraft3;int ncom_aircraft3;

	//Packet of data for eachaircraft vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *aircraft_com_ind; int aircraft_com_count;

public:
	Aircraft(){};
	Aircraft(Module *module_list,int num_modules);
	virtual~Aircraft();

	//executive functions dummy returns
	virtual void vehicle_array(){};
	virtual void scrn_array(){};
	virtual void plot_array(){};
	virtual void scrn_banner(){};
	virtual void tabout_banner(ofstream &ftabout,char *title){};
	virtual void tabout_data(ofstream &ftabout){};
	virtual void scrn_index_arrays(){};
	virtual void scrn_data(){};
	virtual void plot_banner(ofstream &fplot,char *title){};
	virtual void plot_index_arrays(){};
	virtual void plot_data(ofstream &fplot,bool merge){};
	virtual void event(char *options){};

	//executive functions active
	virtual void sizing_arrays();
	virtual void vehicle_data(fstream &input);
	virtual void read_tables(char *file_name,Datadeck &datatable){};
	virtual void com_index_arrays();
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle);
	virtual Packet loading_packet_init(int num_aircraft,int num_aim);
	virtual Packet loading_packet(int num_aircraft,int num_aim);

	//module function dummy returns -MOD
	virtual void def_aerodynamics(){};
	virtual void aerodynamics(){}; 
	virtual void def_propulsion(){};
	virtual void propulsion(){};
	virtual void def_actuator(){};
	virtual void init_control(){};
	virtual void def_seeker(){};
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step){};
	virtual void def_intercept(){};
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title){};

	//module functions active
	virtual void def_control();
	virtual void control(double int_step);
	virtual void def_guidance();
	virtual void guidance(Packet *combus,int num_vehicles);
	virtual void def_forces();
	virtual void forces();
};
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
////////////////////////// Global class 'Vehicle'//////////////////////////////
///////////// must be located after 'Cadac' hierarchy in this file (why?)//////
///////////////////////////////////////////////////////////////////////////////
//Class 'Vehicle'
//
//Global class for typifying the array of vehicle pointers
//
//010629 Created by Peter H Zipfel
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