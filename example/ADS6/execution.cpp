///////////////////////////////////////////////////////////////////////////////
///////////////////////   CADAC++ Simulation ADS6   ///////////////////////////
///////////////////////////////////////////////////////////////////////////////
//FILE: 'execution.cpp'
//
//Initializing and executing the simulation
//
//020923 Created by Peter H Zipfel
//030319 Upgraded to SM Item32, PZi
//070531 Inserted launch delay, PZi
//081010 Modified for GENSIM6, PZi
//150217 Compatible with MS VC++ 2013, PZi
//170809 Output in 'csv' format added, PZi
//170909 Added 'Radar', PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"
#include <ctime>

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//////////////// Definition of global function prototypes used in main() //////
///////////////////////////////////////////////////////////////////////////////

//acquiring the simmulation title
void acquire_title_options(fstream &input,char *title,char *options,int &nmonte,
						   int &iseed,int &nmc);

//acquiring the simulation run time
double acquire_endtime(fstream &input);

//numbering the modules
void number_modules(fstream &input,int &num);

//acquiring the calling order of the modules
void order_modules(fstream &input,int &num,Module *module_list);

//acquiring the number of vehicle objects
void number_objects(fstream &input,int &num_vehicles,int &num_missile,int &num_rocket,int &num_aircraft,int &num_radar);

//creating a type of vehicle object
Cadac *set_obj_type(fstream &input,Module *module_list,int num_modules,
				   int num_rocket,int num_aircraft,int num_radar);

//running the simulation
void execute(Vehicle &vehicle_list,Module *module_list,double sim_time,
			 double end_time,int num_vehicles,int num_modules,double plot_step,
			 double int_step,double scrn_step,double com_step,double traj_step,char *options,
			 ofstream &ftabout,ofstream *plot_ostream_list,Packet *combus,int *status,
			 int num_missile,int num_rocket,int num_aircraft,int num_radar,ofstream &ftraj,char *title,bool traj_merge,
			 int nmonte,int nmc,ofstream *stat_ostream_list,bool *stati_write_term,double *launch_delay_list);

// saving status of 'combus' vehicle objects
void combus_status(Packet *combus,int *status,int num_vehicles);

//getting timimg cycles for plotting, screen output and integration
void acquire_timing(fstream &input,double &plot_step,double &scn_step,double &com_step,
					double &traj_step,double &int_step);

//merging the 'ploti.asc' files onto 'plot.asc' 
void merge_plot_files(string *plot_file_list,int num_missile,char *title);

//merging the 'stati.asc' files onto 'stat.asc' 
void merge_stat_files(string *stat_file_list,int num_missile,char *title);

//writing 'combus' data on screen
void comscrn_data(Packet *combus,int num_vehicles,double sim_time);

//writing banner on 'traj.asc' file 
//void traj_banner(ofstream &ftraj,Packet *combus,char *title,int num_vehicles);
void traj_banner(ofstream &ftraj,Packet *combus,char *title,int num_vehicles,
				 int nmissile,int nrocket,int naircraft,int nradar);

//writing 'traj.asc' file of 'combus' data
void traj_data(ofstream &ftraj,Packet *combus,int num_vehicles,bool merge,double sim_time);

//Documenting 'input.asc' with module-variable definitions
void document_input(Document *doc_missile6,Document *doc_rocket5,Document *doc_aircraft3,Document *doc_radar0);

//writing 'plot' and 'traj' files in csv
void parse_plot_traj_csv(string *plot_files, int num_ucav, bool merge, string type);

///////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////  main()   //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Main function
//
//011128 Created by Peter H Zipfel
//070531 Inserted launch delay, PZi
///////////////////////////////////////////////////////////////////////////////

int main() 
{
	double sim_time=0; //simulation time, same as 'time'
	char title[CHARL]; //title from first line of 'input.asc'
	char options[CHARL]; //options from 'input.asc' for output
	Module *module_list=NULL; //list of modules and sequence as per 'input.asc'
	int num_modules=0; //number of modules
	double plot_step=0; //writing time step to 'plot.asc' and 'ploti.asc', i=1,2,3...
	double scrn_step=0; //writing time step to screen (console)
	double int_step=0; //integration step size 
	double com_step=0; //writing time step of 'combus' data to screen
	double traj_step=0; //writing time step of 'combus' to file 'traj.asc'
	int num_vehicles=0; //total number of vehicle objects
	int num_missile=0; //number of missile objects
	int num_rocket=0; //number of rocket objects
	int num_aircraft=0; //number of aircraft objects
	int num_radar=0; //number of radar objects
	Cadac *vehicle_type=NULL; //array of vehicle object pointers
	char vehicle_name[CHARN]; //name of each vehicle type
	double end_time=0; //run termination time from 'input.asc'
	string *plot_file_list=NULL; //array containing file names of 'ploti.asc', i=1,2,3...
	ofstream *plot_ostream_list=NULL; //array of output streams for 'ploti.asc', i=1,2,3...
	string *stat_file_list=NULL; //array containing file names of 'stati.asc', i=1,2,3...
	ofstream *stat_ostream_list=NULL; //array of output streams for 'stati.asc', i=1,2,3...
	Packet *combus=NULL; //communication bus container storing data for each vehicle object
					//in same sequence as 'vehicle_list'
	int *status=NULL; //array containing status of each vehicle object
	int nmonte=0; //number of MC runs to be executed
	int nmc=0; //MC counter
	int iseed=0; //seeding srand()
	bool one_traj_banner=true; //write just one banner on file 'traj.asc'
	bool *stati_write_term=NULL; //flag for writing impact data on 'stati.asc' once
	Document *doc_missile6=NULL;  //array for documenting MISSILE6 module-variables of 'input.asc'
	Document *doc_rocket5=NULL;  //array for documenting ROCKET5 module-variables of 'input.asc'
	Document *doc_aircraft3=NULL;  //array for documenting AIRCRAFT3 module-variables of 'input.asc'
	Document *doc_radar0=NULL;  //array for documenting RADAR0 module-variables of 'input.asc'
	bool document_missile6=false; //true if array doc_missile6 was created
	bool document_rocket5=false; //true if doc_rocket5 was created
	bool document_aircraft3=false; //true if doc_aircraft3 was created
	bool document_radar0=false; //true if doc_radar0 was created
	double launch_delay=0; //individual vehicle launch delay
	double *launch_delay_list=NULL;  //launch delay list

	///////////////////////////////////////////////////////////////////////////
	/////////////// Opening of files and creation of stream objects  //////////
	///////////////////////////////////////////////////////////////////////////

	//creating an input stream object and opening 'input.asc' file
	fstream input("input.asc");
	if(input.fail())
	{cerr<<"*** Error: File stream 'input.asc' failed to open (check spelling) ***\n";exit(1);}

	//creating an output stream object and opening 'tabout.asc' file
	ofstream ftabout("tabout.asc");
	if(!ftabout){cout<<" *** Error: cannot open 'tabout.asc' file *** \n";exit(1);}

	//creating an output stream object and opening 'doc.asc' file
	ofstream fdoc("doc.asc");
	if(!fdoc){cout<<" *** Error: cannot open 'doc.asc' file *** \n";exit(1);}

	//creating an output stream object and opening 'traj.asc' file
	ofstream ftraj("traj.asc");
	if(!ftraj){cout<<" *** Error: cannot open 'traj.asc' file *** \n";exit(1);}

	//creating file 'input_copy.asc' in local directory for use in 'document_input()'
	ofstream fcopy("input_copy.asc");
	if(!fcopy){cout<<" *** Error: cannot open 'input_copy.asc' file *** \n";exit(1);}

	///////////////////////////////////////////////////////////////////////////
	////////////////////////// Monte Carlo Loop ///////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	do
	{
		//initializing flags
		bool one_screen_banner=true; //write just one banner on screen
		bool document_missile=true; //write  to 'doc.asc' missile module-variables only once
		bool document_rocket=true; //write  to 'doc.asc' rocket module-variables only once
		bool document_aircraft=true; //write  to 'doc.asc' aircraft module-variables only once
		bool document_radar=true; //write  to 'doc.asc' radar module-variables only once
		bool traj_merge=false; //flag used in writing 'time=-1' endblock on 'traj.asc'
		
		//aqcuiring title statement and option selections
		acquire_title_options(input,title,options,nmonte,iseed,nmc);

		//initializing random number generator
		if(!nmc) srand(iseed); 

		//acquiring number of module 
		number_modules(input,num_modules);

		//dynamic memory allocation of array 'module_list'
		try{module_list=new Module[num_modules];} 
		catch(bad_alloc xa){cout<< "*** Allocation failure of 'module_list' ***\n";return 1;}

		try{module_list=new Module[num_modules];}		
		catch(bad_alloc xa){cout<< "*** Allocation failure of 'module_list' ***\n";return 1;}

					
		//acquiring calling order of module 
		order_modules(input,num_modules,module_list);
		
		//acquiring the time stepping
		acquire_timing(input,plot_step,scrn_step,int_step,com_step,traj_step);

		//acquiring number of vehicle objects from 'input.asc'
		number_objects(input,num_vehicles,num_missile,num_rocket,num_aircraft,num_radar);

		//creating the 'vehicle_list' object
		// at this point the constructor 'Vehicle' is called and memory is allocated
		Vehicle vehicle_list(num_vehicles);
		
		//allocating memory for 'ploti.asc' file streams, but do it only once
		if(!nmc){
			try{plot_ostream_list=new ofstream[num_vehicles];}
			catch(bad_alloc xa){cerr<<"*** Allocation failure of 'plot_ostream_list' *** \n";return 1;}
		}
		//allocating memory for 'ploti.asc' files, but do it only once
		if(!nmc){
			try{plot_file_list=new string[num_vehicles];}
			catch(bad_alloc xa){cerr<<"*** Allocation failure of 'plot_file_list' *** \n";return 1;}
		}
		//allocating memory for 'stati.asc' file streams, but do it only once
		if(!nmc){
			try{stat_ostream_list=new ofstream[num_vehicles];}
			catch(bad_alloc xa){cerr<<"*** Allocation failure of 'stat_ostream_list' *** \n";return 1;}
		}
		//allocating memory for 'stati.asc' files, but do it only once
		if(!nmc){
			try{stat_file_list=new string[num_vehicles];}
			catch(bad_alloc xa){cerr<<"*** Allocation failure of 'stat_file_list' *** \n";return 1;}
		}
		//allocating memory for 'combus'
		try{combus=new Packet[num_vehicles];}
		catch(bad_alloc xa){cerr<<"*** Allocation failure of 'combus' *** \n";return 1;}

		//allocating memory for 'status'
		try{status=new int[num_vehicles];}
		catch(bad_alloc xa){cerr<<"*** Allocation failure of 'status' *** \n";return 1;}

		//initialize 'status' to 'alive=1'
		for(int ii=0;ii<num_vehicles;ii++) status[ii]=1;

		//allocating memory for write-flag to 'stati.asc'
		try{stati_write_term=new bool[num_vehicles];}
		catch(bad_alloc xa){cerr<<"*** Allocation failure of 'stati_write_term' *** \n";return 1;}

		//allocating memory for launch delay list ';launch_delay_list' and initializing elements to zero
		try{launch_delay_list=new double[num_vehicles];}
		catch(bad_alloc xa){cerr<<"*** Allocation failure of 'launch_delay_list' *** \n";return 1;}
		for(int kk=0;kk<num_vehicles;kk++) launch_delay_list[kk]=0;

		//initialize 'stati_write_term' to true
		for(int ii=0;ii<num_vehicles;ii++) stati_write_term[ii]=true;

		///////////////////////////////////////////////////////////////////////
		////////////////// Initializing each vehicle object  ///////////////
		///////////////////////////////////////////////////////////////////////

		for(int i=0;i<num_vehicles;i++)
		{
			//Loading pointers of the vehicle-object classes ('Missile','Rocket','Aircraft','Radar') into 
			// the i-th location of 'vehicle_list'. Vehicle type is read in from 'input.asc' file. 
			//The loading process allocates dynamic memory at the pointer location
			// as required by the vehicle object 
			//The function returns the 'vehicle_type' as specified in 'input.asc' 
			//Furthermore, it passes 'module_list', 'num_modules','num_rocket'and'num_aircraft'
			// to the 'Missile', 'Rocket', 'Aircraft', and 'Radar' constructors
			vehicle_type=set_obj_type(input,module_list,num_modules,num_rocket,num_aircraft,num_radar);
 				
			//add vehicle to 'vehicle_list'
			vehicle_list.add_vehicle(*vehicle_type);

			//getting the name of the type of vehicle
			strcpy(vehicle_name,vehicle_list[i]->get_vname());

			//vehicle data and tables read from 'input.asc' 
			vehicle_list[i]->vehicle_data(input,nmonte);

			//executing initialization computations -MOD		
			for (int j=0;j<num_modules;j++)
			{
				if((module_list[j].name=="kinematics")&&(module_list[j].initialization=="init"))
					vehicle_list[i]->init_kinematics(sim_time,int_step,launch_delay);
				else if((module_list[j].name=="aerodynamics")&&(module_list[j].initialization=="init"))
					vehicle_list[i]->init_aerodynamics();
				else if((module_list[j].name=="ins")&&(module_list[j].initialization=="init"))
					vehicle_list[i]->init_ins();
				else if((module_list[j].name=="newton")&&(module_list[j].initialization=="init"))
					vehicle_list[i]->init_newton();
			}
			//populating the launch delay list
			launch_delay_list[i]=launch_delay;

			//writing banner to screen and file 'tabout.asc'
			if(!strcmp(vehicle_name,"MISSILE6")&&one_screen_banner)
			{
				one_screen_banner=false;
				if(strstr(options,"y_scrn"))
				{
					vehicle_list[i]->scrn_banner();
				}
				if(strstr(options,"y_tabout"))
				{	
					vehicle_list[i]->tabout_banner(ftabout,title,nmonte,nmc);
				}
			}
			//writing one block of data to screen after module initialization
			if(strstr(options,"y_scrn"))
			{
				vehicle_list[i]->scrn_data();
			}
			if(strstr(options,"y_tabout"))
			{	
				vehicle_list[i]->tabout_data(ftabout);
			}

			//executing 'plot' and 'stat' file initialization and documentation, but only at first run
			if(!nmc)
			{		
				//creating output stream objects in 'plot_ostream_list[i]' for every file "ploti.asc"
				//currently only the MISSILE6 plot files contain any data
				if(strstr(options,"y_plot"))
				{
					if(!strcmp(vehicle_name,"MISSILE6"))
					{
						char index[CHARN]; //
						string plotiasc; //plot file name
						const char *name;

						//building names for plot files
						sprintf(index,"%i",i+1);
						plotiasc="plot"+string(index)+".asc"; //using Standard Library string constructor
						plot_file_list[i]=plotiasc;
						name=plotiasc.c_str(); //using string member function to convert to char array 

						//creating output stream list 'plot_ostream_list[i]', each will write on file 'ploti.asc'
						plot_ostream_list[i].open(name); //'name' must be 'char' type

						//writing banner on 'ploti.asc'
						vehicle_list[i]->plot_banner(plot_ostream_list[i],title);				}
				}

				//creating output stream objects in 'stat_ostream_list[i]' for every file "stati.asc"
				//currently only the MISSILE6 stat files contain any data
				if(strstr(options,"y_stat"))
				{
					if(!strcmp(vehicle_name,"MISSILE6"))
					{
						char index[CHARN]; //
						string statiasc; //stat file name
						const char *name;

						//building names for stat files
						sprintf(index,"%i",i+1);
						statiasc="stat"+string(index)+".asc"; //using Standard Library string constructor
						stat_file_list[i]=statiasc;
						name=statiasc.c_str(); //using string member function to convert to char array 

						//creating output stream list 'stat_ostream_list[i]', each will write on file 'stati.asc'
						stat_ostream_list[i].open(name); //'name' must be 'char' type

						//writing banner on 'stati.asc'
						vehicle_list[i]->plot_banner(stat_ostream_list[i],title);				}
				}

				//composing documentation of 'flat6','missile','flat3','rocket', 'aircraft', 'flat0' and 'radar' module-variables
				//but do it only once for MISSILE6, ROCKET5, AIRCRAFT3, and RADAR0 objects
				if(strstr(options,"y_doc"))
				{
					if(!strcmp(vehicle_name,"MISSILE6")&&document_missile)
					{
						document_missile=false;
						int size=NFLAT6+NMISSILE;
						try{doc_missile6=new Document[size];}
						catch(bad_alloc xa){cerr<<"*** Allocation failure of 'doc_missile6' *** \n";return 1;}
						vehicle_list[i]->document(fdoc,title,doc_missile6);
						document_missile6=true;
					}
					if(!strcmp(vehicle_name,"ROCKET5")&&document_rocket)
					{
						document_rocket=false;
						int size=NFLAT3+NROCKET;
						try{doc_rocket5=new Document[size];}
						catch(bad_alloc xa){cerr<<"*** Allocation failure of 'doc_rocket5' *** \n";return 1;}
						vehicle_list[i]->document(fdoc,title,doc_rocket5);
						document_rocket5=true;
					}
					if(!strcmp(vehicle_name,"AIRCRAFT3")&&document_aircraft)
					{
						document_aircraft=false;
						int size=NFLAT3+NAIRCRAFT;
						try{doc_aircraft3=new Document[size];}
						catch(bad_alloc xa){cerr<<"*** Allocation failure of 'doc_aircraft3' *** \n";return 1;}
						vehicle_list[i]->document(fdoc,title,doc_aircraft3);
						document_aircraft3=true;
					}
					if(!strcmp(vehicle_name,"RADAR0")&&document_radar)
					{
						document_radar=false;
						int size=NFLAT0+NRADAR;
						try{doc_radar0=new Document[size];}
						catch(bad_alloc xa){cerr<<"*** Allocation failure of 'doc_radar0' *** \n";return 1;}
						vehicle_list[i]->document(fdoc,title,doc_radar0);
						document_radar0=true;
					}
				}
			}//end of initializing plot, stat files, and documentation

			//loading packets into 'combus' communication bus
			combus[i]=vehicle_list[i]->loading_packet_init(num_missile,num_aircraft,num_rocket,num_radar);

		} //end of initialization of vehicle object loop

		//writing 'combus' data to screen at time=0, after module initialization			
		if(strstr(options,"y_comscrn"))
		{
			comscrn_data(combus,num_vehicles,sim_time);
		}
		//writing banner for 'traj.asc' output file
		if(strstr(options,"y_traj")&&one_traj_banner)
		{
			one_traj_banner=false;
			traj_banner(ftraj,combus,title,num_vehicles,num_missile,num_rocket,num_aircraft,num_radar);

			//writing data after 'initial module' calculations
			traj_data(ftraj,combus,num_vehicles,traj_merge,sim_time);
		}

		//acuire ending time (last entry on 'input.asc')
		// and put file pointer at the beginning of 'input.asc' for MC repeat runs
		end_time=acquire_endtime(input);

		if(!nmc&&strstr(options,"y_doc")){
			//documenting input.asc (only once)
			document_input(doc_missile6,doc_rocket5,doc_aircraft3,doc_radar0);
			if(document_missile6)delete [] doc_missile6;
			if(document_rocket5)delete [] doc_rocket5;
			if(document_aircraft3)delete [] doc_aircraft3;
			if(document_radar0)delete [] doc_radar0;
		}

		///////////////////////////////////////////////////////////////////////	
		/////////////////////// Simulation Execution //////////////////////////
		///////////////////////////////////////////////////////////////////////	

		execute(vehicle_list,module_list,sim_time,
				 end_time,num_vehicles,num_modules,plot_step,
				 int_step,scrn_step,com_step,traj_step,options,ftabout,
				 plot_ostream_list,combus,status,num_missile,num_rocket,num_aircraft,num_radar,ftraj,title,
				 traj_merge,nmonte,nmc,stat_ostream_list,stati_write_term,launch_delay_list);

		//deallocating dynamic memory
		delete [] module_list;
		delete [] combus;
		delete [] status;
		delete [] stati_write_term;
		delete [] launch_delay_list;

		// If MONTE > 0 repeat 'nmonte' times
		nmc++;
	}	// at this point the destructor of the object 'Vehicle:: vehicle_list' is called 
	while(nmc<nmonte); 

	///////////////////////////////////////////////////////////////////////////	
	///////////////////////// End of Monte Carlo Loop /////////////////////////
	///////////////////////////////////////////////////////////////////////////	
	//////////////////////////// Post-Processing //////////////////////////////
	///////////////////////////////////////////////////////////////////////////	

	//Close input file streams
	input.close();
	fcopy.close();
	int f(0);

	//Close file streams
	ftabout.close();
	for(f=0;f<num_vehicles;f++) plot_ostream_list[f].close();
	fdoc.close();
	for(f=0;f<num_vehicles;f++) stat_ostream_list[f].close();
	ftraj.close();

	//merging 'ploti.asc' files into 'plot.asc'
	if(strstr(options,"y_merge")&&strstr(options,"y_plot"))
	{
		merge_plot_files(plot_file_list,num_missile,title);
	}
	//merging 'stati.asc' files into 'stat.asc'm using 'merge_plot_data' function
	//adding at the end: time=-1 and dummy block of data
	if(strstr(options,"y_merge")&&strstr(options,"y_stat"))
	{
		merge_stat_files(stat_file_list,num_missile,title);
	}
	//creating a comma separated value (.csv) file if 'y_csv' and either 'y_plot' or 'y_traj'
	if(strstr(options,"y_csv"))
	{
		if(strstr(options,"y_plot"))
		{
			if(strstr(options,"y_merge"))
			{
				parse_plot_traj_csv(plot_file_list,num_missile,1,"plot");
			}
			else{
				parse_plot_traj_csv(plot_file_list,num_missile,0,"plot");
			}
		}
		if(strstr(options,"y_traj"))
		{
			string* trajs=new string[1];
			trajs[0]="traj.asc";
			parse_plot_traj_csv(trajs,1,1,"traj");  //always merge traj into one file
			delete [] trajs;
		}
	}
	//deallocate dynamic memory
	delete [] plot_ostream_list;
	delete [] plot_file_list;
	delete [] stat_ostream_list;
	delete [] stat_file_list;

	//system("pause");
	return 0;
}
///////////////////////////////////////////////////////////////////////////////	
////////////////////////// End of main ////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////	

///////////////////////////////////////////////////////////////////////////////
//Executing the simulation
//
//Parameters:	&vehicle_list =	vehicle array - list of vehicle objects 
//								and their respective type (missile, rocket, aircraft, radar)
//								estabished by global function 'set_obj_type'
//				*module_list = module array - list of modules and their calling sequence
//								established by global function 'order_modules'
//				sim_time = simulation time; called 'time' in output		 
//				end_time = time to stop the simulation - read from 'input.asc' 
//								by global function 'acquire_end'
//				num_vehicles = total number of vehicles - read from 'input.asc' (VEHICLES #)
//								by global function 'number_vehicles'  				
//				num_modules = total number of modules - read from 'input.asc'
//								by global function 'number_modules'
//				plot_step = output writing interval to file 'traj.asc' - sec
//								read from 'input.asc' by global function 'acquire_timing'  				
//				int_step = integration step 
//								read from 'input.asc' by global function 'acquire_timing'  				
//				scrn_step = output writing interval to console - sec  				
//								read from 'input.asc' by global function 'acquire_timing'
//				com_step = output interval to communication bus 'combus'
//				traj_step = output interval to 'traj.asc'
//				*options = output option list
//				&ftabout = output file-stream to 'tabout.asc'
//				*plot_ostream_list = output file-steam list of 'ploti.asc' for each individual missile 
//								missile object
//				*combus = commumication bus
//				*status = health of vehicles
//				num_missile = number of 'Missile' objects
//				num_rocket = number of 'Rocket' objects
//				num_aircraft = number of 'Aircraft' objects
//				num_radar = number of 'Radar' objects
//				&ftraj = output file-stream to 'traj.asc'
//				*title = idenfication of run
//				traj_merge = flag for merging MC runs in 'traj.asc'
//				nmonte = max number of MC runs
//				nmc = current MC iteration number 
//				*stat_ostream_list = output file-steam list of 'stati.asc' for each individual missile 
//								missile object
//				*stati_write_term = flag for writing impact data on 'stati.asc' once
//				*launch_delay_list = launch delay list
//				  				
//011128 Created by Peter H Zipfel
//040705 Calculating 'event_time', PZi
//070531 Incrementing 'sim_time' in 'combus' until 'ENDTIME' is reached, PZi
//081010 Modified for GENSIM6, PZi
//170918 Modified for ADS6, PZi
///////////////////////////////////////////////////////////////////////////////
void execute(Vehicle &vehicle_list,Module *module_list,double sim_time,
			 double end_time,int num_vehicles,int num_modules,double plot_step,
			 double int_step,double scrn_step,double com_step,double traj_step,char *options,
			 ofstream &ftabout,ofstream *plot_ostream_list,Packet *combus,int *status,
			 int num_missile,int num_rocket,int num_aircraft,int num_radar,ofstream &ftraj,char *title,bool traj_merge,
			 int nmonte,int nmc,ofstream *stat_ostream_list,bool *stati_write_term,double *launch_delay_list)
{
	double scrn_time(0);
	double plot_time(0);
	double traj_time(0);
	double com_time(0);
	int vehicle_slot(0);
	bool increment_scrn_time(false);
	bool increment_plot_time(false);
	bool plot_merge(false);
	double out_fact(0);
	Variable *data_t;
	string radar_id="f1";
	double lnch_delay_m1(0);
	double lnch_delay_m2(0);
	double lnch_delay_m3(0);
	string missile_id1="m1";
	string missile_id2="m2";
	string missile_id3="m3";

	//integration loop
	while (sim_time<=(end_time+int_step))
	{
		//vehicle loop
		for (int i=0;i<num_vehicles;i++)
		{
			if(sim_time<launch_delay_list[i]);
				//vehicle is holding at initial point
			else			
			{
				//vehicle is progressing

				//slot occupied by current vehicle in 'vehicle_list[]'
				vehicle_slot=i;

				//watching for the next event			
				vehicle_list[i]->event(options);

				//fixing 'event_epoch' and initializing 'event_time'
				if(vehicle_list[i]->event_epoch)
					vehicle_list[i]->event_time=0;

				//continue only if vehicle is alive (health=1:alive; =-1:hit('rocket' only); =0:dead)
				int health=combus[i].get_status();
				if(health==1)
				{
					//refreshing Markov variables
					vehicle_list[i]->markov_noise(sim_time,int_step,nmonte);

					//module loop -MOD
					for(int j=0;j<num_modules;j++)
					{
						if(module_list[j].name=="environment")
							vehicle_list[i]->environment();
						else if(module_list[j].name=="kinematics")
							vehicle_list[i]->kinematics(sim_time,vehicle_list[i]->event_time,int_step,out_fact,combus,num_vehicles,vehicle_slot);
						else if(module_list[j].name=="newton")
							vehicle_list[i]->newton(int_step);
						else if(module_list[j].name=="euler")
							vehicle_list[i]->euler(int_step);
						else if(module_list[j].name=="aerodynamics")
							vehicle_list[i]->aerodynamics();
						else if(module_list[j].name=="propulsion")
							vehicle_list[i]->propulsion();
						else if(module_list[j].name=="forces")
							vehicle_list[i]->forces();
						else if(module_list[j].name=="actuator")
							vehicle_list[i]->actuator(int_step);
						else if(module_list[j].name=="tvc")
							vehicle_list[i]->tvc(int_step);
						else if(module_list[j].name=="rcs")
							vehicle_list[i]->rcs(int_step);
						else if(module_list[j].name=="control") 
							vehicle_list[i]->control(int_step);
						else if(module_list[j].name=="guidance")
							vehicle_list[i]->guidance(combus,num_vehicles,vehicle_slot,int_step);
						else if(module_list[j].name=="ins")
							vehicle_list[i]->ins(int_step);
						else if(module_list[j].name=="sensor")
							vehicle_list[i]->sensor(combus,num_vehicles,vehicle_slot,sim_time,int_step);
						else if(module_list[j].name=="intercept")
							vehicle_list[i]->intercept(combus,vehicle_slot,int_step,title);
					} //end of module loop

					//preserving 'health' status of vehicle objects
					combus_status(combus,status,num_vehicles);

					//loading data packet into 'combus' communication bus
					combus[i]=vehicle_list[i]->loading_packet(num_missile,num_aircraft,num_rocket,num_radar);

					//refreshing 'health' status of vehicle objects
					combus[i].set_status(status[i]);

				} //end of active vehicle loop

				//continuing incrementing 'sim_time' in combus packets until 'ENDTIME' is reached
				combus[i].set_data_variable(0,sim_time);

				//tracking time elapsed during event
				vehicle_list[i]->event_time+=int_step;

				//output to screen and/or 'tabout.asc'
				if(fabs(scrn_time-sim_time)<(int_step/2+EPS))
				{
					if(strstr(options,"y_scrn"))
					{
						vehicle_list[i]->scrn_data();
						if(i==(num_vehicles-1))increment_scrn_time=true;
					}

					if(strstr(options,"y_tabout"))
					{
						vehicle_list[i]->tabout_data(ftabout);
					}
					if(increment_scrn_time) scrn_time+=scrn_step*(1+out_fact);
				}

				//output to 'ploti.asc' file
				if(fabs(plot_time-sim_time)<(int_step/2+EPS))
				{
					if(strstr(options,"y_plot"))
					{
						vehicle_list[i]->plot_data(plot_ostream_list[i],plot_merge);
						if(i==(num_vehicles-1))increment_plot_time=true;
					}
					if(increment_plot_time) plot_time+=plot_step*(1+out_fact);
				}
				//output to 'stati.asc' file 
				if(strstr(options,"y_stat"))
				{
					if(vehicle_list[i]->event_epoch)
						vehicle_list[i]->stat_data(stat_ostream_list[i],nmc,i);
					if(!combus[i].get_status()&&stati_write_term[i])
					{
						stati_write_term[i]=false;
						vehicle_list[i]->stat_data(stat_ostream_list[i],nmc,i);
					}
				}
			}
			//download the launch delay values from Radar packet on 'combus' 
			// and store in 'launch_delay_list[i]' for m1, m2, m3
			// (requires that MISSILE6 objects precede RADAR0 object in 'input.asc'
			//   and assums the pairing of missiles and rocket targets m1->r1, m2->r2, m3->r3)  
			string id=combus[i].get_id();
			if (id==radar_id)
			{						
				//downloading launch delays for missiles m1, m2, m3
				data_t=combus[i].get_data();
				lnch_delay_m1=data_t[1].real();
				lnch_delay_m2=data_t[2].real();
				lnch_delay_m3=data_t[3].real();
			}
			if(i==0)launch_delay_list[0]=lnch_delay_m1;
			if(i==1)launch_delay_list[1]=lnch_delay_m2 ;
			if(i==2)launch_delay_list[2]=lnch_delay_m3 ;

		} //end of vehicle loop

		//outputting 'combus' to screen 
		if(fabs(com_time-sim_time)<(int_step/2+EPS))
		{
			if(strstr(options,"y_comscrn"))
			{
				comscrn_data(combus,num_vehicles,sim_time);
			}
			com_time+=com_step*(1+out_fact);
		}
		//outputting'combus' to 'traj.asc' file
		if(fabs(traj_time-sim_time)<(int_step/2+EPS))
		{
			if(strstr(options,"y_traj"))
			{
				traj_data(ftraj,combus,num_vehicles,traj_merge,sim_time);
			}
			traj_time+=traj_step*(1+out_fact);
		}
		//resetting output events
		increment_scrn_time=false;
		increment_plot_time=false;

		//advancing time
		sim_time+=int_step;

	} //end of integration loop

	//writing last integration out to 'ploti.asc' 
	//with time set to '-1' for multiple CADAC-Studio plots
	if(strstr(options,"y_plot"))
	{
		plot_merge=true;
		for (int i=0;i<num_vehicles;i++)
			vehicle_list[i]->plot_data(plot_ostream_list[i],plot_merge);
	}
	//writing last integration out to 'traj.asc' 
	//with time set to '-1' for multiple CADAC-Studio plots
	if(strstr(options,"y_traj"))
	{
		traj_merge=true;
		traj_data(ftraj,combus,num_vehicles,traj_merge,sim_time);
	}
} 


