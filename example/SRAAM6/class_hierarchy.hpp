///////////////////////////////////////////////////////////////////////////////
//FILE: class_hierarchy.hpp
//
//Contains the classes of the hierarchy of base class 'Cadac'
//
//011128 Created by Peter H Zipfel
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
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	char name[CHARN]; //vehicle object name
		
protected:
	//module-variable array of class 'Flat6'
	//first 10 locations reserved for time and 'com' variables
	Variable *flat6;

	//module-variable array of class 'Flat3'
	//first 10 locations reserved for time and 'com' variables
	Variable *flat3;

	//Array of module-variables as defined in class 'Missile'
	//first ten locations are reserved for 'com' variables
	Variable *missile;

	//Array of module-variables as defined in class 'Target'
	//first ten locations are reserved for 'com' variables
	Variable *target;

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
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_missile,int num_target)=0;
	virtual Packet loading_packet(int num_missile,int num_target)=0;

	//module functions -MOD
	virtual void def_environment()=0;
	virtual void environment()=0;
	virtual void def_kinematics()=0;
	virtual void init_kinematics()=0;
	virtual void kinematics(double int_step)=0;
	virtual void def_newton()=0;
	virtual void init_newton()=0;
	virtual void newton(double sim_time,double int_step)=0;
	virtual void def_euler()=0;
	virtual void euler(double int_step)=0;
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void propulsion()=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_tvc()=0;
	virtual void tvc(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance(Packet *combus,int num_vehicles)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;

};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Flat6
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitaional acceleration and equations of motions
//Contains modules: environment and Newton's law
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Flat6:public Cadac
{
protected:			
	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *flat6_scrn_ind; int flat6_scrn_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *flat6_plot_ind; int flat6_plot_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *flat6_com_ind; int flat6_com_count;
public:
	Flat6();
	virtual~Flat6(){};

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
	virtual Packet loading_packet_init(int num_missile,int num_target)=0;
	virtual Packet loading_packet(int num_missile,int num_target)=0;

	//module functions -MOD
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void propulsion()=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_tvc()=0;
	virtual void tvc(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance(Packet *combus,int num_vehicles)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;

	//virtual functions to be declared in this class
	virtual void def_kinematics();
	virtual void init_kinematics();
	virtual void kinematics(double int_step);
	virtual void def_euler();
	virtual void euler(double int_step);
	virtual void def_environment();
	virtual void environment();
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double sim_time,double int_step);

	//function in respective mdoule
	Matrix newton_tvl(double psivl,double thtvl);
	Matrix kinematics_tab(double alpp,double phip);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Missile
//
//Second level of derived class of the 'Cadac' hierarchy
//Models aerodynamics, propulsion, guidance and control 
//Contains Modules: aerodynamics, propulsion, forces, control, guidance 
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

class Missile:public Flat6
{
protected:
	//name of MISSILE6 vehicle object
	char missile6_name[CHARL];

	//Event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT];int nevent;
	//total number of envents for a vehicle object
	int event_total;

	//Compacted array of all module-variables of vehicle object 'Missile'
	Variable *missile6;int nmissile6;

	//Screen ouput array of module-variables of vehicle object 'Missile'
	Variable *scrn_missile6;int nscrn_missile6;

	//Plot ouput array of module-variables of vehicle object 'Missile'
	Variable *plot_missile6;int nplot_missile6;

	//Communications ouput array of module-variables of vehicle object 'Missile'
	Variable *com_missile6;int ncom_missile6;

	//Packet of data for each missile vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *missile_scrn_ind; int missile_scrn_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *missile_plot_ind; int missile_plot_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *missile_com_ind; int missile_com_count;

	//array of ground distances of 'Missile' object from all 'Target' objects
	double *grnd_range;

	//declaring Table pointer as temporary storage of a single table
	Table *table;
	//declaring Datadeck 'aerotable' that stores all aero tables
	Datadeck aerotable;
	//declaring Datadeck 'proptable' that stores all aero tables
	Datadeck proptable;

public:
	Missile(){};
	Missile(Module *module_list,int num_modules,int num_target);
	virtual~Missile();

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
	virtual Packet loading_packet_init(int num_missile,int num_target);
	virtual Packet loading_packet(int num_missile,int num_target);

	//module functions -MOD
	virtual void def_aerodynamics();
	virtual void init_aerodynamics();
	virtual void aerodynamics(); 
	virtual void def_propulsion();
	virtual void propulsion();
	virtual void def_forces();
	virtual void forces();
	virtual void def_actuator();
	virtual void actuator(double int_step);
	virtual void def_tvc();
	virtual void tvc(double int_step);
	virtual void def_control();
	virtual void control(double int_step);
	virtual void def_guidance();
	virtual void guidance(Packet *combus,int num_vehicles);
	virtual void def_seeker();
	virtual void seeker(Packet *combus,int num_vehicles,double int_step);
	virtual void def_intercept();
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title);

	//functions in respective modules
	void aerodynamics_der();
	void actuator_scnd(double int_step);
	void tvc_scnd(double &eta,double &zet,double etac,double zetc,double int_step);
	void control_roll();
	void control_rate();
	void control_accel(double int_step);
	void guidance_mid(Matrix SABLC,Matrix VAELC);
	void guidance_term();
	void seeker_kin(double &thtpb,double &psipb,double &sigdy,double &sigdz,
					   Matrix SBAL,Matrix VAEL,double dba);
	void seeker_dyn(int &mseek,int &mguid,double &thtpb,double &psipb,double &sigdy,
					   double &sigdz,double &ehz,double &ehy,Matrix &THB,Matrix SBAL,
					   double dba,double int_step);
	Matrix seeker_aimp(Matrix THL,Matrix TAL,double dba);
	void seeker_uthpb(double &ththb,double &phihb,double psipb,double thtpb);
	Matrix seeker_thb(double tht,double phi);
  };

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Flat3
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitaional acceleration and equations of motions
//Contains modules: environment and Newton's law
//
//010116 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
class Flat3:public Cadac
{
protected:			

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *flat3_com_ind; int flat3_com_count;
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
	virtual Packet loading_packet_init(int num_missile,int num_target)=0;
	virtual Packet loading_packet(int num_missile,int num_target)=0;

	//module functions -MOD
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_propulsion()=0;
	virtual void propulsion()=0;
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_tvc()=0;
	virtual void tvc(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance(Packet *combus,int num_vehicles)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;

	//dummy returns of unused modules
	virtual void def_kinematics(){};
	virtual void init_kinematics(){};
	virtual void kinematics(double int_step){};
	virtual void def_euler(){};
	virtual void euler(double int_step){};

	//virtual functions to be declared in this class
	virtual void def_environment();
	virtual void environment();
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double sim_time,double int_step);
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Target
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Flat3'
//Models target accelerations
//Contains Module 'forces'
//
//010205 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

class Target:public Flat3
{
protected:
	//name of TARGET3 vehicle object
	char target3_name[CHARL];

	//Communications ouput array of module-variables of vehicle object 'Target'
	Variable *com_target3;int ncom_target3;

	//Packet of data for each target vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *target_com_ind; int target_com_count;

public:
	Target(){};
	Target(Module *module_list,int num_modules);
	virtual~Target();

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
	virtual Packet loading_packet_init(int num_missile,int num_target);
	virtual Packet loading_packet(int num_missile,int num_target);

	//module function dummy returns -MOD
	virtual void def_aerodynamics(){};
	virtual void init_aerodynamics(){};
	virtual void aerodynamics(){}; 
	virtual void def_propulsion(){};
	virtual void propulsion(){};
	virtual void def_actuator(){};
	virtual void actuator(double int_step){};
	virtual void def_tvc(){};
	virtual void tvc(double int_step){};
	virtual void def_seeker(){};
	virtual void seeker(Packet *combus,int num_vehicles,double int_step){};
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
////////////////////////// Global class 'Vehicle'//////////////////////////////
///////////// must be located after 'Cadac' hierarchy in this file (why?)//////
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