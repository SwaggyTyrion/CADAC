///////////////////////////////////////////////////////////////////////////////
//FILE: class_hierarchy.hpp
//
//Contains the classes of the hierarchy of base class 'Cadac'
//
//011128 Created by Peter H Zipfel
//030415 Adapted to HYPER simulation, PZi
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
//030415 Adapted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Cadac
{
private:
	 //vehicle object name
	char name[CHARN];
		
protected:
	//module-variable array of class 'Round6'
	//first 10 locations reserved for time and 'com' variables
	Variable *round6;

	//module-variable array of class 'Round3'
	//first 10 locations reserved for time and 'com' variables
	Variable *round3;

	//module-variable array of class 'Ground0'
	//first 10 locations reserved for time and 'com' variables
	Variable *ground0;

	//Array of module-variables as defined in class 'Hyper'
	//first ten locations are reserved for 'com' variables
	Variable *hyper;

	//Array of module-variables as defined in class 'Satellite'
	//first ten locations are reserved for 'com' variables
	Variable *satellite;

	//Array of module-variables as defined in class 'Radar'
	//first ten locations are reserved for 'com' variables
	Variable *radar;

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
	virtual void tabout_banner(ofstream &ftabout,char *title,int &nmonte,int &nmc)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input,int nmonte)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void stat_data(ofstream &fstat,int nmc,int vehicle_slot)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_hyper,int num_satellite,int num_radar)=0;
	virtual Packet loading_packet(int num_hyper,int num_satellite,int num_radar)=0;
	virtual void markov_noise(double sim_time,double int_step,int nmonte)=0;

	//module functions -MOD
	virtual void def_newton()=0;
	virtual void init_newton()=0;
	virtual void newton(double int_step)=0;
	virtual void def_kinematics()=0;
	virtual void init_kinematics(double sim_time,double event_time)=0;
	virtual void kinematics(double sim_time,double event_time,double &int_step,double &out_fact)=0;
	virtual void init_euler()=0;
	virtual void def_euler()=0;
	virtual void euler(double int_step)=0;
	virtual void def_environment()=0;
	virtual void init_environment()=0;
	virtual void environment(double int_step)=0;
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_ins()=0;
	virtual void init_ins()=0;
	virtual void ins(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance(double int_step)=0;
	virtual void def_gps()=0;
	virtual void gps(double int_step)=0;
	virtual void def_startrack()=0;
	virtual void startrack()=0;
	virtual void def_rcs()=0;
	virtual void rcs()=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)=0;
	virtual void def_datalink()=0;
	virtual void datalink(Packet *combus,int num_vehicles)=0;
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Round6
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitaional acceleration and equations of motions
//Contains modules: 'environment', 'kinematics', 'newton', 'euler' 
//
//011128 Created by Peter H Zipfel
//030415 Adapted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Round6:public Cadac
{
protected:			
	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *round6_scrn_ind; int round6_scrn_count;

	//Indicator-array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *round6_plot_ind; int round6_plot_count;

	//Indicator-array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *round6_com_ind; int round6_com_count;
public:
	Round6();
	virtual~Round6(){};

	//executive functions 
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,char *title,int &nmonte,int &nmc)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input,int nmonte)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void stat_data(ofstream &fstat,int nmc,int vehicle_slot)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_hyper,int num_satellite,int num_radar)=0;
	virtual Packet loading_packet(int num_hyper,int num_satellite,int num_radar)=0;
	virtual void markov_noise(double sim_time,double int_step,int nmonte)=0;

	//module functions -MOD
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void def_ins()=0;
	virtual void init_ins()=0;
	virtual void ins(double int_step)=0;
	virtual void def_guidance()=0;
	virtual void guidance(double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;
	virtual void def_gps()=0;
	virtual void gps(double int_step)=0;
	virtual void def_startrack()=0;
	virtual void startrack()=0;
	virtual void def_rcs()=0;
	virtual void rcs()=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)=0;
	virtual void def_datalink()=0;
	virtual void datalink(Packet *combus,int num_vehicles)=0;

	//virtual functions to be declared in this class
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double int_step);
	virtual void def_kinematics();
	virtual void init_kinematics(double sim_time,double int_step);
	virtual void kinematics(double sim_time,double event_time,double &int_step,double &out_fact);
	virtual void def_euler();
	virtual void init_euler();
	virtual void euler(double int_step);
	virtual void def_environment();
	virtual void init_environment();
	virtual void environment(double int_step);

	//functions in respective modules
	Matrix environment_dryden(double dvba,double int_step); 
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Hyper
//
//Second level of derived class of the 'Cadac' hierarchy
//Models aerodynamics, propulsion, guidance and control 
//Contains Modules: 'aerodynamics', 'forces', 'propulsion', 'actuator', 'control',
// 'ins', 'guidance', 'gps', 'startrack', 'seeker', 'rcs', 'datalink'  
//
//011128 Created by Peter H Zipfel
//030415 Adapted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Hyper:public Round6
{
protected:
	//name of HYPER6 vehicle object
	char hyper6_name[CHARL];

	//Event list of 'Event' object pointers and actual number of events 
	Event *event_ptr_list[NEVENT];int nevent;
	//total number of envents for a vehicle object
	int event_total;

	//Compacted array of all module-variables of vehicle object 'Hyper'
	Variable *hyper6;int nhyper6;

	//Screen ouput array of module-variables of vehicle object 'Hyper'
	Variable *scrn_hyper6;int nscrn_hyper6;

	//Plot ouput array of module-variables of vehicle object 'Hyper'
	Variable *plot_hyper6;int nplot_hyper6;

	//Communications ouput array of module-variables of vehicle object 'Hyper'
	Variable *com_hyper6;int ncom_hyper6;

	//Packet of data for each hyper vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to the screen
	int *hyper_scrn_ind; int hyper_scrn_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to the 'ploti.asc' files
	int *hyper_plot_ind; int hyper_plot_count;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *hyper_com_ind; int hyper_com_count;

	//array of ground distances of 'Hyper' object from all 'Satellite' objects
	double *grnd_range;

	//array of module-variables that carry Markov process random values
	Markov markov_list[NMARKOV]; int nmarkov;

	//declaring Table pointer as temporary storage of a single table
	Table *table;
	//	declaring Datadeck 'aerotable' that stores all aero tables
	Datadeck aerotable;
	//	declaring Datadeck 'proptable' that stores all aero tables
	Datadeck proptable;

public:
	Hyper(){};
	Hyper(Module *module_list,int num_modules,int num_satellite,int num_radar);
	virtual~Hyper();

	//executive functions
	virtual void sizing_arrays();
	virtual void vehicle_array();
	virtual void scrn_array();
	virtual void plot_array();
	virtual void scrn_banner();
	virtual void tabout_banner(ofstream &ftabout,char *title,int &nmonte,int &nmc);
	virtual void tabout_data(ofstream &ftabout);
	virtual void vehicle_data(fstream &input,int nmonte);
	virtual void read_tables(char *file_name,Datadeck &datatable);
	virtual void scrn_index_arrays();
	virtual void scrn_data();
	virtual void plot_banner(ofstream &fplot,char *title);
	virtual void plot_index_arrays();
	virtual void plot_data(ofstream &fplot,bool merge);
	virtual void stat_data(ofstream &fstat,int nmc,int vehicle_slot);
	virtual void event(char *options);
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle);
	virtual void com_index_arrays();
	virtual Packet loading_packet_init(int num_hyper,int num_satellite,int num_radar);
	virtual Packet loading_packet(int num_hyper,int num_satellite,int num_radar);
	virtual void markov_noise(double sim_time,double int_step,int nmonte);

	//module functions -MOD
	virtual void def_aerodynamics();
	virtual void init_aerodynamics();
	virtual void aerodynamics(); 
	virtual void def_forces();
	virtual void forces();
	virtual void def_propulsion();
	virtual void init_propulsion();
	virtual void propulsion(double int_step);
	virtual void def_actuator();
	virtual void actuator(double int_step);
	virtual void def_control();
	virtual void control(double int_step);
	virtual void def_ins();
	virtual void init_ins();
	virtual void ins(double int_step);
	virtual void def_guidance();
	virtual void guidance(double int_step);
	virtual void def_intercept();
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title);
	virtual void def_gps();
	virtual void gps(double int_step);
	virtual void def_startrack();
	virtual void startrack();
	virtual void def_rcs();
	virtual void rcs();
	virtual void def_seeker();
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step);
	virtual void def_datalink();
	virtual void datalink(Packet *combus,int num_vehicles);

	//functions in respective modules
	void aerodynamics_der();

	Matrix actuator_scnd(Matrix ACTCX,double int_step);

	double control_roll(double phicomx);
	double control_roll_rate(double pcomx);
	double control_yaw_rate(double rcomx);
	double control_pitch_rate(double qcomx);
	double control_lateral_accel(double alcomx);
	double control_normal_accel(double ancomx,double int_step);
	double control_gamma(double thtvdcomx);
	double control_heading(double psivdcomx);
	double control_altitude(double altcom);

	Matrix ins_gyro(Matrix &WBECB, double int_step);
	Matrix ins_accl();
	Matrix ins_grav(Matrix ESBI,Matrix SBIIC);

	double guidance_arc(double wp_lonx,double wp_latx,double wp_alt);
	Matrix guidance_line(double wp_lonx,double wp_latx,double wp_alt,double psifdx,double thtfdx);
	Matrix guidance_ltg(int &mprop,double int_step,double time_ltg);
	void guidance_ltg_tgo(double &tgo, int &nst,int &num_stages, Matrix TAUN
						  ,Matrix VEXN,Matrix BOTN,double delay_ignition,double vgom
						  ,double amag1,double amin,double time_ltg);
	void guidance_ltg_tgo(double &tgop,Matrix &BURNTN,Matrix &L_IGRLN,Matrix &TGON,double &l_igrl,int &nstmax						  
						  ,double &tgo,int &nst,Matrix &TAUN
						  ,Matrix VEXN,Matrix BOTN,double delay_ignition,double vgom
						  ,double amag1,double amin,double time_ltg,int num_stages);
	void guidance_ltg_igrl(double &s_igrl,double &j_igrl,double &q_igrl,double &h_igrl
						   ,double &p_igrl,double &j_over_l,double &tlam,double &qprime
						   ,int nst,int nstmax,Matrix BURNTN,Matrix L_IGRLN,Matrix TGON
						   ,Matrix TAUN,Matrix VEXN,double l_igrl,double time_ltg);							  
	void guidance_ltg_trate(Matrix &ULAM,Matrix &LAMD,Matrix &RGO
							,int &ipas2
							,Matrix VGO,double s_igrl,double q_igrl
							,double j_over_l,double lamd_limit,double vgom,double time_ltg
							,double tgo, double tgop,Matrix SDII,Matrix SBIIC,Matrix VBIIC,Matrix RBIAS
							,Matrix UD,Matrix UY,Matrix UZ,Matrix &RGRAV);
	void guidance_ltg_trate_rtgo(Matrix &RGO
								 ,Matrix &RGRAV
								 ,double tgo, double tgop
								 ,Matrix SDII,Matrix SBIIC,Matrix VBIIC,Matrix RBIAS	
							 	 ,Matrix ULAM,Matrix UD,Matrix UY,Matrix UZ,double s_igrl);
	void guidance_ltg_pdct(Matrix &SPII,Matrix &VPII,Matrix &RGRAV,Matrix &RBIAS
						   ,Matrix LAMD,Matrix ULAM,double l_igrl,double s_igrl,double j_igrl
						   ,double q_igrl,double h_igrl,double p_igrl,double j_over_l,double qprime
						   ,Matrix SBIIC,Matrix VBIIC,Matrix RGO,double tgo);
	void guidance_ltg_crct(Matrix &SDII,Matrix &UD,Matrix &UY,Matrix &UZ,Matrix &VMISS
						   ,Matrix &VGO
						   ,double dbi_desired,double dvbi_desired,double thtvdx_desired
						   ,Matrix SPII,Matrix VPII,Matrix SBIIC,Matrix VBIIC);
	Matrix guidance_glideslope(int &mprop,int &gs_flag,int mseek);
	Matrix guidance_pronav(Matrix &UTBBC, Matrix STBIK,Matrix VTBIK);
	Matrix guidance_AGL(Matrix &UTBBC, Matrix STBIK,Matrix VTBIK);

	void gps_sv_init(double *sv_init_data,double &rsi,double &wsi,double &incl);
	void gps_quadriga(double *ssii_quad,double *vsii_quad,double &gdop,int &mgps 
						,const double *sv_init_data,const double &rsi,const double &wsi
						,const double &incl,double almanac_time,double del_rearth
						,double time,Matrix SBII);

	void startrack_init(double *star_data,string *star_names);
	void star_triad(double *usii_triad,double star_volume, double *star_data,double star_el_min,Matrix SBII);

	double rcs_prop(double input,double limiter);
	int rcs_schmitt(double input_new,double input,double dead_zone,double hysteresis);

	void seeker_rf(double &azab,double &elab,double &dab,double &ddab,int &mseek,
					   int &mguid, double fovlimx,Matrix SBTL,double int_step);
	Matrix seeker_glint();
	void seeker_kin(double &azob,double &elob,double &dab,double &ddab,Matrix SBTL);
	void seeker_filter(Matrix &STBIK,Matrix &VTBIK
			,double azab,double elabx,double dab,double ddab,int mseek,double int_step);
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Round3
//
//First derived class in the 'Cadac' hierarchy
//Models atmosphere, gravitaional acceleration and equations of motions
//Contains modules: 'environment' and 'newton'
//
//010116 Created by Peter H Zipfel
//030415 Adapted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Round3:public Cadac
{
protected:			

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
	virtual void tabout_banner(ofstream &ftabout,char *title,int &nmonte,int &nmc)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input,int nmonte)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void stat_data(ofstream &fstat,int nmc,int vehicle_slot)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_hyper,int num_satellite,int num_radar)=0;
	virtual Packet loading_packet(int num_hyper,int num_satellite,int num_radar)=0;
	virtual void markov_noise(double sim_time,double int_step,int nmonte)=0;

	//module functions -MOD
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void init_ins()=0;
	virtual void ins(double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)=0;

	virtual void def_datalink()=0;
	virtual void datalink(Packet *combus,int num_vehicles)=0;

	//dummy returns of unused modules
	virtual void def_euler(){};
	virtual void init_euler(){};
	virtual void euler(double int_step){};

	//virtual functions to be declared in this class
	virtual void def_kinematics();
	virtual void init_kinematics(double sim_time,double int_step);
	virtual void kinematics(double sim_time,double event_time,double &int_step,double &out_fact);
	virtual void def_newton();
	virtual void init_newton();
	virtual void newton(double int_step);
	virtual void def_environment();
	virtual void init_environment(){};
	virtual void environment(double int_step);
};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Satellite
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Round3'
//Models satellite accelerations
//Contains Module 'forces'
//
//010205 Created by Peter H Zipfel
//030415 Adapted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Satellite:public Round3
{
protected:
	//name of SAT3 vehicle object
	char satellite3_name[CHARL];

	//Communications ouput array of module-variables of vehicle object 'Satellite'
	Variable *com_satellite3;int ncom_satellite3;

	//Packet of data for each satellite vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *satellite_com_ind; int satellite_com_count;

public:
	Satellite(){};
	Satellite(Module *module_list,int num_modules);
	virtual~Satellite();

	//executive functions dummy returns
	virtual void vehicle_array(){};
	virtual void scrn_array(){};
	virtual void plot_array(){};
	virtual void scrn_banner(){};
	virtual void tabout_banner(ofstream &ftabout,char *title,int &nmonte,int &nmc){};
	virtual void tabout_data(ofstream &ftabout){};
	virtual void scrn_index_arrays(){};
	virtual void scrn_data(){};
	virtual void plot_banner(ofstream &fplot,char *title){};
	virtual void plot_index_arrays(){};
	virtual void plot_data(ofstream &fplot,bool merge){};
	virtual void stat_data(ofstream &fstat,int nmc,int vehicle_slot){};
	virtual void event(char *options){};
	virtual void markov_noise(double sim_time,double int_step,int nmonte){};

	//executive functions active
	virtual void sizing_arrays();
	virtual void vehicle_data(fstream &input,int nmonte);
	virtual void read_tables(char *file_name,Datadeck &datatable){};
	virtual void com_index_arrays();
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle);
	virtual Packet loading_packet_init(int num_hyper,int num_satellite,int num_radar);
	virtual Packet loading_packet(int num_hyper,int num_satellite,int num_radar);

	//module function dummy returns -MOD
	virtual void def_aerodynamics(){};
	virtual void init_aerodynamics(){};
	virtual void aerodynamics(){}; 
	virtual void def_propulsion(){};
	virtual void init_propulsion(){};
	virtual void propulsion(double int_step){};
	virtual void def_actuator(){};
	virtual void actuator(double int_step){};
	virtual void def_control(){};
	virtual void control(double int_step){};
	virtual void def_ins(){};
	virtual void init_ins(){};
	virtual void ins(double int_step){};
	virtual void def_guidance(){};
	virtual void guidance(double int_step){};
	virtual void def_gps(){};
	virtual void gps(double int_step){};
	virtual void def_startrack(){};
	virtual void startrack(){};
	virtual void def_rcs(){};
	virtual void rcs(){};
	virtual void def_intercept(){};
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title){};
	virtual void def_datalink(){};
	virtual void datalink(Packet *combus,int num_vehicles){};
	virtual void def_seeker(){};
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step){};
	virtual void def_filter(){};
	virtual void filter(double int_step){};

	//module functions -MOD
	virtual void def_forces();
	virtual void forces();
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Ground0
//
//First derived class in the 'Cadac' hierarchy
//Places Earth-fixed points
//Contains only the 'kinematics' module
//
//040517 Created by Peter H Zipfel
//030415 Adapted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Ground0:public Cadac
{
protected:			

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *ground0_com_ind; int ground0_com_count;
public:
	Ground0();
	virtual~Ground0(){};

	//executive functions 
	virtual void sizing_arrays()=0;
	virtual void vehicle_array()=0;
	virtual void scrn_array()=0;
	virtual void plot_array()=0;
	virtual void scrn_banner()=0;
	virtual void tabout_banner(ofstream &ftabout,char *title,int &nmonte,int &nmc)=0;
	virtual void tabout_data(ofstream &ftabout)=0;
	virtual void vehicle_data(fstream &input,int nmonte)=0;
	virtual void read_tables(char *file_name,Datadeck &datatable)=0;
	virtual void scrn_index_arrays()=0;
	virtual void scrn_data()=0;
	virtual void plot_banner(ofstream &fplot,char *title)=0;
	virtual void plot_index_arrays()=0;
	virtual void plot_data(ofstream &fplot,bool merge)=0;
	virtual void stat_data(ofstream &fstat,int nmc,int vehicle_slot)=0;
	virtual void event(char *options)=0;
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle)=0;
	virtual void com_index_arrays()=0;
	virtual Packet loading_packet_init(int num_hyper,int num_satellite,int num_radar)=0;
	virtual Packet loading_packet(int num_hyper,int num_satellite,int num_radar)=0;
	virtual void markov_noise(double sim_time,double int_step,int nmonte)=0;

	//module functions -MOD
	virtual void def_aerodynamics()=0;
	virtual void init_aerodynamics()=0;
	virtual void aerodynamics()=0; 
	virtual void def_forces()=0;
	virtual void forces()=0;
	virtual void def_propulsion()=0;
	virtual void init_propulsion()=0;
	virtual void propulsion(double int_step)=0;
	virtual void def_actuator()=0;
	virtual void actuator(double int_step)=0;
	virtual void def_control()=0;
	virtual void control(double int_step)=0;
	virtual void init_ins()=0;
	virtual void ins(double int_step)=0;
	virtual void def_seeker()=0;
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step)=0;
	virtual void def_intercept()=0;
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title)=0;

	virtual void def_datalink()=0;
	virtual void datalink(Packet *combus,int num_vehicles)=0;

	//dummy returns of unused modules
	virtual void def_newton(){};
	virtual void init_newton(){};
	virtual void newton(double int_step){};
	virtual void def_environment(){};
	virtual void init_environment(){};
	virtual void environment(double int_step){};
	virtual void def_euler(){};
	virtual void init_euler(){};
	virtual void euler(double int_step){};

	//virtual functions to be declared in this class
	virtual void def_kinematics();
	virtual void init_kinematics(double sim_time,double int_step);
	virtual void kinematics(double sim_time,double event_time,double &int_step,double &out_fact);

};
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Derived class: Radar
//
//Second level of derived class of the 'Cadac' hierarchy, branching from 'Ground0'
//Models radar on ground
//Contains module 'seeker'
//
//040505 Created by Peter H Zipfel
//030415 Adapted to HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
class Radar:public Ground0
{
protected:
	//name of RADAR3 vehicle object
	char radar0_name[CHARL];

	//Communications ouput array of module-variables of vehicle object 'Radar'
	Variable *com_radar0;int ncom_radar0;

	//Packet of data for each radar vehicle
	Packet packet;

	//Indicator array pointing to the module-variable which are to 
	//be written to 'combus' 'packets'
	int *radar_com_ind; int radar_com_count;

public:
	Radar(){};
	Radar(Module *module_list,int num_modules);
	virtual~Radar();

	//executive functions dummy returns
	virtual void vehicle_array(){};
	virtual void scrn_array(){};
	virtual void plot_array(){};
	virtual void scrn_banner(){};
	virtual void tabout_banner(ofstream &ftabout,char *title,int &nmonte,int &nmc){};
	virtual void tabout_data(ofstream &ftabout){};
	virtual void scrn_index_arrays(){};
	virtual void scrn_data(){};
	virtual void plot_banner(ofstream &fplot,char *title){};
	virtual void plot_index_arrays(){};
	virtual void plot_data(ofstream &fplot,bool merge){};
	virtual void stat_data(ofstream &fstat,int nmc,int vehicle_slot){};
	virtual void event(char *options){};
	virtual void markov_noise(double sim_time,double int_step,int nmonte){};

	//executive functions active
	virtual void sizing_arrays();
	virtual void vehicle_data(fstream &input,int nmonte);
	virtual void read_tables(char *file_name,Datadeck &datatable){};
	virtual void com_index_arrays();
	virtual void document(ostream &fdoc,char *title,Document *doc_vehicle);
	virtual Packet loading_packet_init(int num_hyper,int num_satellite,int num_radar);
	virtual Packet loading_packet(int num_hyper,int num_satellite,int num_radar);

	//module function dummy returns -MOD
	virtual void def_aerodynamics(){};
	virtual void init_aerodynamics(){};
	virtual void aerodynamics(){}; 
	virtual void def_propulsion(){};
	virtual void init_propulsion(){};
	virtual void propulsion(double int_step){};
	virtual void def_forces(){};
	virtual void forces(){};
	virtual void def_actuator(){};
	virtual void actuator(double int_step){};
	virtual void def_control(){};
	virtual void control(double int_step){};
	virtual void def_ins(){};
	virtual void init_ins(){};
	virtual void ins(double int_step){};
	virtual void def_guidance(){};
	virtual void guidance(double int_step){};
	virtual void def_gps(){};
	virtual void gps(double int_step){};
	virtual void def_startrack(){};
	virtual void startrack(){};
	virtual void def_rcs(){};
	virtual void rcs(){};
	virtual void def_intercept(){};
	virtual void intercept(Packet *combus,int vehicle_slot,double int_step,char *title){};
	virtual void def_datalink(){};
	virtual void datalink(Packet *combus,int num_vehicles){};
	virtual void def_filter(){};
	virtual void filter(double int_step){};

	//module functions -MOD
	virtual void def_seeker();
	virtual void seeker(Packet *combus,int num_vehicles,double sim_time,double int_step);
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
	Cadac **vehicle_ptr;
public:
	Vehicle(int number=1);	//constructor, setting capacity, allocating dynamic memory
	virtual ~Vehicle();	//destructor, de-allocating dynamic memory
	void add_vehicle(Cadac &ptr);	//adding vehicle to list
	Cadac *operator[](int position);	//[] operator returns vehicle pointer
	int size();	//returning 'howmany' vehicles are stored in vehicle list
};

#endif