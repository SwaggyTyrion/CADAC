///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_functions.cpp'
//
//Contains the global functions for the HYPER simulation.
//
//011129 Created by Peter H Zipfel
//030415 Adopted for HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Acquiring simulation title and option line from the input file 'input.asc'.
//Printing of title banner to screen
//
//Parameter output: *title, *options, &nmonte, &iseed
//
//Parameter input: &nmc
//
//011128 Created by Peter H Zipfel
//020919 Added 'document_input()', PZi
//030415 Adopted for HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void acquire_title_options(fstream &input,char *title,char *options,int &nmonte,int &iseed,int &nmc)
{ 
	char read[CHARN];
	char line_clear[CHARL];
	bool title_absent=true;
	int n(0);
	
	//read until 'OPTIONS' or if not encountered within 100 lines print error message
	do
	{
		n++;
		input>>read;
		if(ispunct(read[0])) input.getline(line_clear,CHARL,'\n');
		if (!strcmp(read,"TITLE"))
		{
			input.getline(title,CHARL,'\n');
			cout<<"\n"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<"\n";
			title_absent=false;
		}
		if (!strcmp(read,"MONTE"))
		{
			input>>nmonte;
			input>>iseed;
			cout<<" MONTE Run # "<<nmc+1<<'\n';
		}
	}while((strcmp(read,"OPTIONS"))&&(n<100));
	input.getline(options,CHARL,'\n');
	if(title_absent)
	{
		strcpy(title,"*** No Title found ***");
		cout<<"\n"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<"\n";

	}
	if(n==100) {cerr<<"*** Error: OPTIONS must be before MODULES; or: MONTE does not have a seed *** \n";system("pause");exit(1);}
}

////////////////////////////////////////////////////////////////////////////////
//Acquiring the number of modules from the input file. 
//
//Parameter output: &num, number of modules, (call-by-reference)
//					&file_prt, pointer to first module in 'input.asc' 
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void number_modules(fstream &input,int &num)
{
	char temp[CHARN];
	char line_clear[CHARL];
	num=0;
	int file_ptr=NULL;

	input>>temp;
	if (!strcmp(temp,"MODULES"))
	{
		input.getline(line_clear,CHARL,'\n');
		file_ptr=int(input.tellg());
		do
		{
			input>>temp;
			input.getline(line_clear,CHARL,'\n');
			num++;
		}while(strcmp(temp,"END"));
		num=num-1;
	}
	else
		cout<<"*** 'MODULES' must follow 'OPTIONS' line without comment lines *** \n";

	//reset file pointer to first module name
	input.seekg(file_ptr);
}
///////////////////////////////////////////////////////////////////
//Acquiring the ordering of the modules from the input file 'input.asc'.
//
//Argument output: *module_list, list of module names in the sequence of 'input.asc' 
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void order_modules(fstream &input,int &num,Module *module_list)
{	
	string temp;
	char module_type[CHARL];
	char line_clear[CHARL];


	for (int i=0;i<num;i++)		
	{
		input>>temp;
		module_list[i].name=temp;

		//reading the type of module functions present
		input.getline(module_type,CHARL,'\n');

		//initializing first
		module_list[i].definition="0";
		module_list[i].initialization="0";
		module_list[i].execution="0";
		module_list[i].termination="0";

		//loading the structure data
		if(strstr(module_type,"def"))module_list[i].definition="def";
		if(strstr(module_type,"init"))module_list[i].initialization="init";
		if(strstr(module_type,"exec"))module_list[i].execution="exec";
		if(strstr(module_type,"term"))module_list[i].termination="term";
	}
	input.getline(line_clear,CHARL,'\n');
}
///////////////////////////////////////////////////////////////////////////////
//Acquiring timing parameters
//
//Parameter output: plot_step, scrn_step, int_step
//
//010330 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void acquire_timing(fstream &input,double &plot_step,double &scrn_step,double &int_step,
					double &com_step,double &traj_step)
{
	char temp[CHARN];
	char line_clear[CHARL];
	plot_step=0;
	scrn_step=0;
	int_step=0;
	com_step=0;
	traj_step=0;

	input>>temp;
	if (!strcmp(temp,"TIMING"))
	{
		input.getline(line_clear,CHARL,'\n');
		do
		{
			input>>temp;
			if(!strcmp(temp,"plot_step"))input>>plot_step;
			if(!strcmp(temp,"scrn_step"))input>>scrn_step;
			if(!strcmp(temp,"int_step"))input>>int_step;
			if(!strcmp(temp,"com_step"))input>>com_step;
			if(!strcmp(temp,"traj_step"))input>>traj_step;
			input.getline(line_clear,CHARL,'\n');

		}while(strcmp(temp,"END"));
	}
	else
		cout<<"*** 'TIMING' must follow 'MODULES; NO blank lines between MODULES...END' ***\n";
}

///////////////////////////////////////////////////////////////////////////////
//Acquiring the number of vehicle objects from the input file 'input.asc'
//
//Parameter output: &num_vehicle, number of vehicles, (call-by-reference)
//					&num_hyper, number of hyper hypers					
//					&num_satellite, number of satellites					
//					&num_radar, number of radars					
//
//010330 Created by Peter H Zipfel
//020920 Added check for illigal '=' signs and missing numerical entries, PZi
//030415 Adopted for HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void number_objects(fstream &input,int &num_vehicles,int &num_hyper,int &num_satellite,int &num_radar)
{
	char read[CHARN];
	char line_clear[CHARL];
	num_vehicles=0;
	num_hyper=0;
	num_satellite=0;
	num_radar=0;
	int file_ptr=NULL;
	char comment[3]="//";

	//reading number of total vehicle objects	
	input>>read;
	if (!strcmp(read,"VEHICLES")){
		input>>num_vehicles;		
		input.getline(line_clear,CHARL,'\n');
	}
	else
		cout<<"*** 'VEHICLES' must follow 'TIMING' ***\n";

	//searching for # of hyper, satellite and radar objects
	//saving file pointer position
	file_ptr=int(input.tellg());

	do{
		input>>read;
		input.getline(line_clear,CHARL,'\n');
		if (!strcmp(read,"HYPER6")) num_hyper++;
		if (!strcmp(read,"SAT3")) num_satellite++;
		if (!strcmp(read,"RADAR0")) num_radar++;

	}while((num_hyper+num_satellite+num_radar)<num_vehicles);

	//resetting file pointer position
	input.seekg(file_ptr);

	//flagging illigal '=' signs
	int icount=0;
	while(!input.eof()){
		input>>read;
		if(strstr(read,comment)||!strcmp(read,"IF")){
			input.getline(line_clear,CHARL,'\n');
		}
		else{
			if(strstr(read,"=")){
				input.getline(line_clear,CHARL,'\n');
				icount++;
			}
		}
	}	

	//resetting file pointer position
	input.clear();
	input.seekg(file_ptr);

	//flagging missing numerical entries
	int vcount=0;
	while(!input.eof()){
		input>>read;
		if(strstr(read,comment)||isupper(read[0]))
			input.getline(line_clear,CHARL,'\n');
		else{
			input>>read;
			if(strstr(read,comment)){
				input.getline(line_clear,CHARL,'\n');
				vcount++;
			}
			else
				input.getline(line_clear,CHARL,'\n');
		}
	}
	if(icount){cout<<" *** Error: "<<icount<<" illigal '=' sign(s) found  in 'input.asc' ***\n";}
	if(vcount){cout<<" *** Error: "<<vcount<<" missing numerical value(s) in 'input.asc' ***\n";}
	if(icount||vcount){system("pause");exit(1);}

	//resetting file pointer position
	input.clear();
	input.seekg(file_ptr);
}

///////////////////////////////////////////////////////////////////////////////
//Allocating dynamic memory for an object by defining the appropriate pointer
//  
//Parameter output: *obj, type-of-vehicle pointer
//Arguments of object: module_list, num_modules, num_satellite, num_radar to be passed 
// to the constructorof 'Hyper', 'Satellite' and 'Radar'
//Return output: type, type-of-vehicle as defined in 'input.asc'
//
//011128 Created by Peter H Zipfel
//030415 Adopted for HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Cadac *set_obj_type(fstream &input,Module *module_list,int num_modules,int num_satellite,int num_radar)				   
{
	char line_clear[CHARL];
	char temp[CHARN];
	Cadac *obj=NULL;
	int file_ptr=NULL;

	//diagnostic: file pointer
	file_ptr=int(input.tellg());

	//bypassing comment lines
	do{
		input>>temp;
		if(ispunct(temp[0]))
			input.getline(line_clear,CHARL,'\n');
	}while(ispunct(temp[0]));

	if (!strcmp(temp,"HYPER6"))
	{
		//the pointer 'obj' is allocated the type 'Hyper' 
		obj=new Hyper(module_list,num_modules,num_satellite,num_radar); 
		if(obj==0){cerr<<"*** Error:'obj' allocation failed *** \n";system("pause");exit(1);}
		obj->set_name("HYPER6");
	}
	else if (!strcmp(temp,"SAT3"))
	{
		//the pointer 'obj' is allocated the type 'Satellite' 
		obj=new Satellite(module_list,num_modules);
		if(obj==0){cerr<<"*** Error:'obj' allocation failed *** \n";system("pause");exit(1);}
		obj->set_name("SAT3");
	}
	else if (!strcmp(temp,"RADAR0"))
	{
		//the pointer 'obj' is allocated the type 'Radar' 
		obj=new Radar(module_list,num_modules);
		if(obj==0){cerr<<"*** Error:'obj' allocation failed *** \n";system("pause");exit(1);}
		obj->set_name("RADAR0");
	}
	return obj;
}

///////////////////////////////////////////////////////////////////////////////
//Acquiring the simulation ending time from 'input.asc'
//
//011128 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

double acquire_endtime(fstream &input)
{	
	double num(0);
	char read[CHARN];
	char line_clear[CHARL];
	int file_ptr=NULL;

	//resetting file pointer to beginning
	file_ptr=int(input.tellg()); //note: for test only
	input.seekg(ios::beg);
	file_ptr=int(input.tellg()); //note: for test only
	do
	{
		file_ptr=int(input.tellg()); //note: for test only

		input>>read;
		if(strcmp(read,"ENDTIME"))
			input.getline(line_clear,CHARL);
	}while(strcmp(read,"ENDTIME"));

	input>>num;

	//placing file pointer at the beginning for MONTE looping
	input.seekg(ios::beg);

	return num;
}

///////////////////////////////////////////////////////////////////////////////
//Merging the 'ploti.asc' files onto 'plot.asc' 
//Compatible witch CADAC Studio plotting
//
//010117 Created by Peter H Zipfel
//011022 Changed treatment of plot_file_list;'num_vehicles'replaced by'num_hyper'PZi
//020304 Correction in first while loop, PZi
///////////////////////////////////////////////////////////////////////////////
void merge_plot_files(string *plot_file_list,int num_hyper,char *title)
{	
	char line_clear[CHARL];
	char line[CHARL];
	char buff[CHARN];
	int num_labels;
	int num_label_lines;
	int strip_lines;
	ifstream *file_istream_list;
	bool first_time=true;
	const char *name;
	int i(0);

	//Allocating memory for 'ploti.asc' file streams
	file_istream_list=new ifstream[num_hyper];
	if(file_istream_list==0)
		{cerr<<"*** Error: file_istream_list[] allocation failed *** \n";system("pause");exit(1);}

	ofstream fmerge("plot.asc");

	for(i=0;i<num_hyper;i++)
	{
		name=plot_file_list[i].c_str(); //conversion from 'string' to 'char' type
		file_istream_list[i].open(name);
	}

	//determining number of lines to be stripped of ploti.asc, i=1,2,3...
	ifstream fplot1("plot1.asc");
	fplot1.getline(line_clear,CHARL,'\n');
	fplot1>>buff;
	fplot1>>buff;
	fplot1>>num_labels;
	num_label_lines=num_labels/5;
	if(num_labels%5>0) num_label_lines=num_label_lines+1;
	strip_lines=num_label_lines+2;

	//copying first file 'plot1.asc' onto 'plot.asc'
	while(!file_istream_list[0].eof())
	{
		file_istream_list[0].getline(line,CHARL,'\n');
			if(first_time)
			{
				first_time=false;
//				if(file_istream_list[i].gcount()) fmerge<<line;
				fmerge<<"1"<<title<<"  "<< __DATE__ <<" "<< __TIME__;
			}
			else
				//copy only non-blank lines
				if(file_istream_list[0].gcount()) fmerge<<'\n'<<line;
	}
	//copying remaining files onto 'plot.asc'
	for(i=1;i<num_hyper;i++)
	{
		//discarding header lines
		int n=0;
		for(n=0;n<strip_lines;n++)
			file_istream_list[i].getline(line_clear,CHARL,'\n');

			//copying data lines
		while(!file_istream_list[i].eof())
		{
			file_istream_list[i].getline(line,CHARL,'\n');
			//copy only non-blank lines
			if(file_istream_list[i].gcount()) fmerge<<line<<'\n';
		}
	}
	//closing files and deallocating memory
	fmerge.close();
	for(i=0;i<num_hyper;i++) file_istream_list[i].close();
	delete [] file_istream_list;
}
///////////////////////////////////////////////////////////////////////////////
//Writing out banner of labels to 'traj.asc' from 'combus' module-variable array
//
//First label is time-since-launch-of-vehicle 'time', always at round6[0]
//five accross, unlimited down
//data field width 16 spaces, total width 80 spaces
//labels longer than 5 characters will be truncated to allow for vehicle 'id'
//to be appended, limiting hyper, satellite and radar vehicles to 8 char each (e.g.: SBEG1_t1)
//Accomodates 3x1 vectors
//
//010207 Created by Peter Zipfel
//011129 Adapted to Hyper6 simulation, PZi
//030110 Added Radar, PZi
//030415 Adopted for HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void traj_banner(ofstream &ftraj,Packet *combus,char *title,int num_vehicles,
				 int nhyper,int nsatellite,int nradar)
{
	char *buff1;
	char buff2[15];
	int label_length=10;
	int m(0);
	int p(0);
	int q(0);
	int r(0);

	bool first_time=true;
	string id;
	int i_hyper(0); //first hyper packet index in vehicle_list
	int i_satellite(0); //first satellite packet index in vehicle_list
	int i_radar(0); //first radar packet index in vehicle_list
	int ncom_hyper6; //number of module variables in 'com_hyper6'
	int ncom_satellite3; //number of module variables in 'com_satellite3'
	int ncom_radar0; //number of module variables in 'com_radar0'
	Variable *data_hyper; //Variable array stored in 'Packet data' of type Hyper
	Variable *data_satellite; //Variable array stored in 'Packet data' of type Satellite
	Variable *data_radar; //Variable array stored in 'Packet data' of type Radar

	ftraj<<"1"<<title<<" "<< __DATE__ <<" "<< __TIME__ <<"\n";

	//find first hyper packet index in 'combus'
	int i(0);
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		if(id=="h1")
			i_hyper=i;
	}
	//find first satellite packet index in 'combus'
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		if(id=="t1")
			i_satellite=i;
	}
	//find first radar packet index in 'combus'
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		if(id=="r1")
			i_radar=i;
	}
	//number of data elements in 'com_hyper', 'com_satellite' and 'com_radar'
	ncom_hyper6=combus[i_hyper].get_ndata();
	ncom_satellite3=combus[i_satellite].get_ndata();
	ncom_radar0=combus[i_radar].get_ndata();

	//picking the first data packet for label extraction
	data_hyper=combus[i_hyper].get_data();
	data_satellite=combus[i_satellite].get_data();
	data_radar=combus[i_radar].get_data();

	//determining the total number of vector variables in com_hyper6, com_satellite3 and com_radar0
	int j(0);
	for(j=0;j<ncom_hyper6;j++)
	{
		buff1=data_hyper[j].get_name();
		if(isupper(buff1[0])) p++;
	}
	for(j=0;j<ncom_satellite3;j++)
	{
		buff1=data_satellite[j].get_name();
		if(isupper(buff1[0])) q++;
	}
	for(j=0;j<ncom_radar0;j++)
	{
		buff1=data_radar[j].get_name();
		if(isupper(buff1[0])) r++; 
	}
	//number of variables (with vectors componentalized and only one 'time' label))
	int nvariables=ncom_hyper6*nhyper+ncom_satellite3*nsatellite+ncom_radar0*nradar
		+2*(p*nhyper+q*nsatellite+r*nradar)-(nhyper-1);
	
	ftraj<<"  0  0 " <<nvariables<<"\n";
	ftraj.setf(ios::left);


	//going through every packet and writing out the labels of all vehicel objects
	for(i=0;i<num_vehicles;i++)
	{
		//determining wether 'Hyper', 'Satellite' or 'Radar' object
		id=combus[i].get_id();
		if(!id.find("h"))
		//'Hyper' object
		{
			for(int k=0;k<ncom_hyper6;k++)
			{
				buff1=data_hyper[k].get_name();
				//printing 'time' only once
				if(!strcmp(buff1,"time")&&first_time)
				{
					first_time=false;
					ftraj.width(16);
					ftraj<<"time";
					m=1;
				}
				else if(strcmp(buff1,"time"))
				{
					//truncating if more than 10 characters
					strncpy(buff2,buff1,label_length);
					buff2[10]=0;
					//Vectors are recognized by upper case character 
					if(isupper(buff2[0]))
					{
						buff2[10]=0; 
						for(int n=1;n<4;n++)
						{				
							ftraj.width(strlen(buff2));
							ftraj<<buff2<<n<<"_";ftraj.width(14-strlen(buff2));ftraj<<id;
							m++;
							if(m>4){m=0;ftraj<<'\n';}
						}
					}
					else
					{
						ftraj.width(strlen(buff2));
						ftraj<<buff2<<"_";ftraj.width(15-strlen(buff2));ftraj<<id;
						m++;
						if(m>4){m=0;ftraj<<'\n';}
					}
				}
			}
		}//end of 'Hyper' objects
		else if(!id.find("t")) 
		//'Satellite' object
		{
			for(int k=0;k<ncom_satellite3;k++)
			{
				buff1=data_satellite[k].get_name();
				//printing 'time' only once
				if(!strcmp(buff1,"time")&&first_time)
				{
					first_time=false;
					ftraj.width(16);
					ftraj<<"time";
					m=1;
				}
				else if(strcmp(buff1,"time"))
				{
					//truncating if more than 10 characters
					strncpy(buff2,buff1,label_length);
					buff2[10]=0;
					//Vectors are recognized by upper case character 
					if(isupper(buff2[0]))
					{
						buff2[10]=0; 
						for(int n=1;n<4;n++)
						{				
							ftraj.width(strlen(buff2));
							ftraj<<buff2<<n<<"_";ftraj.width(14-strlen(buff2));ftraj<<id;
							m++;
							if(m>4){m=0;ftraj<<'\n';}
						}
					}
					else
					{
						ftraj.width(strlen(buff2));
						ftraj<<buff2<<"_";ftraj.width(15-strlen(buff2));ftraj<<id;
						m++;
						if(m>4){m=0;ftraj<<'\n';}
					}
				}
			}
		}//end of 'Satellite' objects
		else if(!id.find("r")) 
		//'Radar' object
		{
			for(int k=0;k<ncom_radar0;k++)
			{
				buff1=data_radar[k].get_name();
				//printing 'time' only once
				if(!strcmp(buff1,"time")&&first_time)
				{
					first_time=false;
					ftraj.width(16);
					ftraj<<"time";
					m=1;
				}
				else if(strcmp(buff1,"time"))
				{
					//truncating if more than 10 characters
					strncpy(buff2,buff1,label_length);
					buff2[10]=0;
					//Vectors are recognized by upper case character 
					if(isupper(buff2[0]))
					{
						buff2[10]=0; 
						for(int n=1;n<4;n++)
						{				
							ftraj.width(strlen(buff2));
							ftraj<<buff2<<n<<"_";ftraj.width(14-strlen(buff2));ftraj<<id;
							m++;
							if(m>4){m=0;ftraj<<'\n';}
						}
					}
					else
					{
						ftraj.width(strlen(buff2));
						ftraj<<buff2<<"_";ftraj.width(15-strlen(buff2));ftraj<<id;
						m++;
						if(m>4){m=0;ftraj<<'\n';}
					}
				}
			}
		}//end of 'Radar' objects
	}//end of 'combus' packet sequence	
	if((nvariables%5))ftraj<<"\n";
}

///////////////////////////////////////////////////////////////////////////////
//Writing data to 'traj.asc'
//
//Accomodates real, integers (printed as real) and 3x1 vectors 
//five accross, unlimited down
//data field 16 spaces, total width 80 spaces
//
//010212 Created by Peter Zipfel
//030415 Adopted for HYPER simulation, PZi
//050214 Added third object 'Radar', PZi
///////////////////////////////////////////////////////////////////////////////

void traj_data(ofstream &ftraj,Packet *combus,int num_vehicles,bool merge)
{
	char *integer;
	char *name;
	Matrix VEC(3,1);
	int k=0;
	Variable *data;
	int ndata;
	string id;
	int hyper_object;
	int satellite_object;
	int radar_object;
	
	ftraj.setf(ios::left);

	//first data entry is 'time'
	if(merge)
	//for merging files, 'time' at last entry must be '-1'
	{
		ftraj.width(16);
		ftraj<<"-1.0";
		k++;
	}
	else
	{
		Variable *data_c1=combus[0].get_data();
		double time=data_c1[0].real();
		ftraj.width(16);
		ftraj<<time;
		k=1;
	}
	//writing to 'traj.asc' the 'data' of the vehicle objects in the sequence their 
	//packets are stored in 'combus' (except for 'time')
	for(int i=0;i<num_vehicles;i++)
	{
		//determining whether 'Hyper','Satellite','Radar' object
		id=combus[i].get_id();
		hyper_object=id.find("h");
		satellite_object=id.find("t");
		radar_object=id.find("r");

		if(!hyper_object)
		//'Hyper' object
		{
			data=combus[i].get_data();
			ndata=combus[i].get_ndata();

			//writing communication variables to 'traj.asc'
			for(int j=1;j<ndata;j++)
			{

				integer=data[j].get_type();
				name=data[j].get_name();

				//checking for integer variables
				if(!strcmp(integer,"int"))
				{
					//casting integer to real variable
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<(double) data[j].integer();			
					k++;
				}
				//checking vor vectors
				else if(isupper(name[0]))
				{
					VEC=data[j].vec();

					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(0,0);
					k++; 
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(1,0);
					k++;
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(2,0);
					k++;
				}
				//left are real variables
				else
				{
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<data[j].real(); 
					k++;
				}
			}
		}//end of 'Hyper' object

		else if(!satellite_object)
		//'Satellite' object
		{
			data=combus[i].get_data();
			ndata=combus[i].get_ndata();

			//writing communication variables to 'traj.asc'
			for(int j=0;j<ndata;j++)
			{
				integer=data[j].get_type();
				name=data[j].get_name();

				//checking for integer variables
				if(!strcmp(integer,"int"))
				{
					//casting integer to real variable
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<(double) data[j].integer();			
					k++;
				}
				//checking vor vectors
				else if(isupper(name[0]))
				{
					VEC=data[j].vec();

					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(0,0);
					k++; 
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(1,0);
					k++;
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(2,0);
					k++;
				}
				//left are real variables
				else
				{
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<data[j].real(); 
					k++;
				}
			}

		}//end of 'Satellite' object

		else if(!radar_object)
		//'Radar' object
		{
			data=combus[i].get_data();
			ndata=combus[i].get_ndata();

			//writing communication variables to 'traj.asc'
			for(int j=0;j<ndata;j++)
			{
				integer=data[j].get_type();
				name=data[j].get_name();

				//checking for integer variables
				if(!strcmp(integer,"int"))
				{
					//casting integer to real variable
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<(double) data[j].integer();			
					k++;
				}
				//checking vor vectors
				else if(isupper(name[0]))
				{
					VEC=data[j].vec();

					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(0,0);
					k++; 
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(1,0);
					k++;
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<VEC.get_loc(2,0);
					k++;
				}
				//left are real variables
				else
				{
					if(k>4){k=0;ftraj<<'\n';}
					ftraj.width(16);
					ftraj<<data[j].real(); 
					k++;
				}
			}
		}//end of 'Radar' object
	}
	ftraj<<"\n";
}

///////////////////////////////////////////////////////////////////////////////
//Writing 'combus' data to screen
//
//010213 Created by Peter Zipfel
//030415 Adopted for HYPER simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void comscrn_data(Packet *combus,int num_vehicles)

{
	char *buff1;
	char buff2[15];
	int label_length=14;
	int m(0);
	int p(0);
	int q(0);
	int r(0);
	string id;
	int nhyper(0); //number of hyper packets 
	int nsatellite(0); //number of satellite packets 
	int nradar(0); //number of radar packets 
	int loc;
	int i_hyper(0); //first hyper packet index in vehicle_list
	int i_satellite(0); //first satellite packet index in vehicle_list
	int i_radar(0); //first radar packet index in vehicle_list
	int ncom_hyper6; //number of module variables in 'com_hyper6'
	int ncom_satellite3; //number of module variables in 'com_satellite3'
	int ncom_radar0; //number of module variables in 'com_radar0'
	Variable *data_hyper; //Variable array stored in 'Packet data' of type Hyper
	Variable *data_satellite; //Variable array stored in 'Packet data' of type Satellite
	Variable *data_radar; //Variable array stored in 'Packet data' of type Radar
	int ndata;
	Variable *data;
	char *integer;
	char *vector;
	Matrix VEC(3,1);
	int i(0);

	//find first hyper packet index in 'combus'
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		if(id=="h1")
			i_hyper=i;
	}
	//find first satellite packet index in 'combus'
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		if(id=="t1")
			i_satellite=i;
	}
	//find first radar packet index in 'combus'
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		if(id=="r1")
			i_radar=i;
	}
	//number of data elements in 'com_hyper' and 'com_satellite'
	ncom_hyper6=combus[i_hyper].get_ndata();
	ncom_satellite3=combus[i_satellite].get_ndata();
	ncom_radar0=combus[i_radar].get_ndata();

	//picking the first data packet for label extraction
	data_hyper=combus[i_hyper].get_data();
	data_satellite=combus[i_satellite].get_data();
	data_radar=combus[i_radar].get_data();

	//determining the number of hyper, satellite and radar packets
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		loc=id.find("h");
		if(!loc) nhyper++;			
	}	
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		loc=id.find("t");
		if(!loc) nsatellite++;			
	}	
	for(i=0;i<num_vehicles;i++)
	{
		id=combus[i].get_id();
		loc=id.find("r");
		if(!loc) nradar++;			
	}	

	//getting 'time' and diplaying it on the screen
	Variable *data_c1=combus[0].get_data();
	double time=data_c1[0].real();
	cout<<" time = ";cout.width(6);cout<<time;
	cout<<" ------------------------------------ combus ----------------------------------------------------------";
	cout.setf(ios::left);cout.width(16);cout<<"\n** HYPER6 **";m=1;

	//'Hyper' object labels (exclude 'time')
	int k(0);
	for(k=1;k<ncom_hyper6;k++)
	{
		buff1=data_hyper[k].get_name();
		//truncating if more than 14 characters
		strncpy(buff2,buff1,label_length);
		buff2[14]=0;
		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int n=1;n<4;n++)
			{				
				cout.width(strlen(buff2));
				cout<<buff2;cout.width(15-strlen(buff2));cout<<n;
				m++;
				if(m>7){m=0;cout<<'\n';}
			}
		}
		else
		{
			cout.width(15);
			cout<<buff2;
			m++;
			if(m>7){m=0;cout<<'\n';}
		}
	}
	cout<<'\n';

	//'Hyper' data output (exclude 'time')
	for(i=0;i<num_vehicles;i++)
	{
		//determining 'Hyper'object
		id=combus[i].get_id();
		loc=id.find("h");
		if(!loc)
		//'Hyper' object
		{
			p++;
			data=combus[i].get_data();
			ndata=combus[i].get_ndata();

			//write out label of i-th object
			cout<<"\n *** h_";cout.width(8);cout<<p;
			k=1;

			//writing communication variables to screen
			for(int j=1;j<ndata;j++)
			{
				integer=data[j].get_type();
				vector=data[j].get_name();

				//checking for integer variables
				if(!strcmp(integer,"int"))
				{
					//casting integer to real variable
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<(double) data[j].integer();			
					k++;
				}
				//checking vor vectors
				else if(isupper(vector[0]))
				{
					VEC=data[j].vec();

					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(0,0);
					k++; 
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(1,0);
					k++;
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(2,0);
					k++;
				}
				//left are real variables
				else
				{
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<data[j].real(); 
					k++;
				}
			}
		}//end of 'Hyper' object
	}//end of HYPER6 data

	cout.setf(ios::left);cout.width(17);cout<<"\n\n** SAT3 **";m=1;

	//'Satellite' object labels 
	for(k=0;k<ncom_satellite3;k++)
	{
		buff1=data_satellite[k].get_name();
		//truncating if more than 14 characters
		strncpy(buff2,buff1,label_length);
		buff2[14]=0;
		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int n=1;n<4;n++)
			{				
				cout.width(strlen(buff2));
				cout<<buff2;cout.width(15-strlen(buff2));cout<<n;
				m++;
				if(m>7){m=0;cout<<'\n';}
			}
		}
		else
		{
			cout.width(15);
			cout<<buff2;
			m++;
			if(m>7){m=0;cout<<'\n';}
		}
	}
	cout<<'\n';

	//'Satellite' data output 
	for(i=0;i<num_vehicles;i++)
	{
		//determining 'Satellite'object
		id=combus[i].get_id();
		loc=id.find("t");
		if(!loc)
		//'Satellite' object
		{
			q++;
			data=combus[i].get_data();
			ndata=combus[i].get_ndata();

			//write out label of i-th object
			cout<<"\n *** t_";cout.width(8);cout<<q;
			k=1;

			//writing communication variables to screen
			for(int j=0;j<ndata;j++)
			{
				integer=data[j].get_type();
				vector=data[j].get_name();

				//checking for integer variables
				if(!strcmp(integer,"int"))
				{
					//casting integer to real variable
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<(double) data[j].integer();			
					k++;
				}
				//checking vor vectors
				else if(isupper(vector[0]))
				{
					VEC=data[j].vec();

					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(0,0);
					k++; 
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(1,0);
					k++;
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(2,0);
					k++;
				}
				//left are real variables
				else
				{
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<data[j].real(); 
					k++;
				}
			}
		}//end of 'Satellite' object
	}//end of SAT3 data

	cout.setf(ios::left);cout.width(17);cout<<"\n\n** RADAR0 **";m=1;

	//'Radar' object labels 
	for(k=0;k<ncom_radar0;k++)
	{
		buff1=data_radar[k].get_name();
		//truncating if more than 14 characters
		strncpy(buff2,buff1,label_length);
		buff2[14]=0;
		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int n=1;n<4;n++)
			{				
				cout.width(strlen(buff2));
				cout<<buff2;cout.width(15-strlen(buff2));cout<<n;
				m++;
				if(m>7){m=0;cout<<'\n';}
			}
		}
		else
		{
			cout.width(15);
			cout<<buff2;
			m++;
			if(m>7){m=0;cout<<'\n';}
		}
	}
	cout<<'\n';

	//'Radar' data output 
	for(i=0;i<num_vehicles;i++)
	{
		//determining 'Radar'object
		id=combus[i].get_id();
		loc=id.find("r");
		if(!loc)
		//'Radar' object
		{
			r++;
			data=combus[i].get_data();
			ndata=combus[i].get_ndata();

			//write out label of i-th object
			cout<<"\n *** r_";cout.width(8);cout<<r;
			k=1;

			//writing communication variables to screen
			for(int j=0;j<ndata;j++)
			{
				integer=data[j].get_type();
				vector=data[j].get_name();

				//checking for integer variables
				if(!strcmp(integer,"int"))
				{
					//casting integer to real variable
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<(double) data[j].integer();			
					k++;
				}
				//checking vor vectors
				else if(isupper(vector[0]))
				{
					VEC=data[j].vec();

					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(0,0);
					k++; 
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(1,0);
					k++;
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<VEC.get_loc(2,0);
					k++;
				}
				//left are real variables
				else
				{
					if(k>7){k=0;cout<<'\n';}
					cout.width(15);
					cout<<data[j].real(); 
					k++;
				}
			}
		}//end of 'Radar' object
	}//end of RADAR0 data
	cout<<"\n               ------------------------------------------------------------------------------------------------------";
	cout<<"\n";	
}

///////////////////////////////////////////////////////////////////////////////
// Saving 'health' status of 'combus' vehicle objects in 'status[]'
//
//010402 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void combus_status(Packet *combus,int *status,int num_vehicles)
{
	//setting status flag array to zero if vehicle is dead
	for(int i=0;i<num_vehicles;i++)
		if(!combus[i].get_status()) status[i]=0;
}
///////////////////////////////////////////////////////////////////////////////
//Merging the 'stati.asc' files onto 'stat.asc' 
//Compatible witch CADAC Studio BIVAR program
//
//011029 Created by Peter H Zipfel
//011129 Adapted to Hyper6 simulation, PZi
//020304 Correction in first while loop, PZi
///////////////////////////////////////////////////////////////////////////////
void merge_stat_files(string *stat_file_list,int num_hyper,char *title)
{	
	char line_clear[CHARL];
	char line[CHARL];
	char buff[CHARN];
	int num_labels;
	int num_label_lines;
	int strip_lines;
	ifstream *file_istream_list;
	bool first_time=true;
	const char *name;
	int i(0);

	//Allocating memory for 'stati.asc' file streams
	file_istream_list=new ifstream[num_hyper];
	if(file_istream_list==0)
		{cerr<<"*** Error: file_istream_list[] allocation failed *** \n";system("pause");exit(1);}

	ofstream fmerge("stat.asc");

	for(i=0;i<num_hyper;i++)
	{
		name=stat_file_list[i].c_str(); //conversion from 'string' to 'char' type
		file_istream_list[i].open(name);
	}

	//determining number of lines to be stripped of stati.asc, i=1,2,3...
	ifstream fstat1("stat1.asc");
	fstat1.getline(line_clear,CHARL,'\n');
	fstat1>>buff;
	fstat1>>buff;
	fstat1>>num_labels;
	num_label_lines=num_labels/5;
	if(num_labels%5>0) num_label_lines=num_label_lines+1;
	strip_lines=num_label_lines+2;

	//copying first file 'stat1.asc' onto 'stat.asc'
	while(!file_istream_list[0].eof())
	{
		file_istream_list[0].getline(line,CHARL,'\n');
			if(first_time)
			{
				first_time=false;
				fmerge<<"1"<<title<<"  "<< __DATE__ <<" "<< __TIME__;
			}
			else
				//copy only non-blank lines
				if(file_istream_list[0].gcount()) fmerge<<'\n'<<line;
	}
	//copying remaining files onto 'stat.asc'
	for(i=1;i<num_hyper;i++)
	{
		//discarding header lines
		int n=0;
		for(n=0;n<strip_lines;n++)
			file_istream_list[i].getline(line_clear,CHARL,'\n');

			//copying data lines
		while(!file_istream_list[i].eof())
		{
			file_istream_list[i].getline(line,CHARL,'\n');
			//copy only non-blank lines
			if(file_istream_list[i].gcount()) fmerge<<line<<'\n';
		}
	}
	//closing files and deallocating memory
	fmerge.close();
	for(i=0;i<num_hyper;i++) file_istream_list[i].close();
	delete [] file_istream_list;
}

///////////////////////////////////////////////////////////////////////////////
///////////// Definition of Member functions of class 'Variable' //////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'real'
//
// private class member output: name, rval, def, mod, role, out
//
//001128 Created by Peter Zipfel
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////
void Variable::init(char *na,double rv,char *de,char *mo,char *ro,char *ou)
{

	if(!strcmp(name,"empty")==0) error[0]='*';
	strcpy(name,na);
	rval=rv;
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}
///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'int'
//
// privat class member output: name, type, ival, def, mod, role, out
//
//001128 Created by Peter Zipfel
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////
void Variable::init(char *na,char *ty,int iv,char *de,char *mo,char *ro,char *ou)
{
	if(!strcmp(name,"empty")==0) error[0]='*';
	strcpy(name,na);
	strcpy(type,ty);
	ival=iv;
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}
///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'Matrix' for 3x1 vectors
//
//private class member output: name, VEC, def, mod, role, out
//
//001128 Created by Peter Zipfel
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////

void Variable::init(char *na,double v1,double v2,double v3,char *de,char *mo,char *ro,char *ou)
{
	double *pbody;
	pbody=VEC.get_pbody();
	*pbody=v1;
	*(pbody+1)=v2;
	*(pbody+2)=v3;

	if(!strcmp(name,"empty")==0) error[0]='*';
	strcpy(name,na);
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}

///////////////////////////////////////////////////////////////////////////////
//Initialization of module-variables of type 'Matrix' for 3x3 matrices
//
//private class member output: name, MAT, def, mod, role, out
//
//001226 Created by Peter Zipfel
//020104 Corrected element assigment errors, PZi
//020909 Added error code handling, PZi
///////////////////////////////////////////////////////////////////////////////

void Variable::init(char *na,double v11,double v12,double v13,double v21,double v22,double v23,
					double v31,double v32,double v33,char *de,char *mo,char *ro,char *ou)
{
	double *pbody;
	pbody=MAT.get_pbody();
	*pbody=v11;
	*(pbody+1)=v12;
	*(pbody+2)=v13;
	*(pbody+3)=v21;
	*(pbody+4)=v22;
	*(pbody+5)=v23;
	*(pbody+6)=v31;
	*(pbody+7)=v32;
	*(pbody+8)=v33;

	if(!strcmp(name,"empty")==0) error[0]='*';
	strcpy(name,na);
	strcpy(def,de);
	strcpy(mod,mo);
	strcpy(role,ro);
	strcpy(out,ou);
}

///////////////////////////////////////////////////////////////////////////////
//Documenting 'input.asc' with module-variable definitions
// occurs if flag 'y_doc' is set
//
//020912 Created by Peter H Zipfel
//030415 Adopted for HYPER simulation, PZi
//050212 Firste 'do-while' criteria changed to look for 'eof', PZi
//////////////////////////////////////////////////////////////////////////////
void document_input(Document *doc_hyper6,Document *doc_satellite3,Document *doc_radar0)
{
	char buffl[CHARL];*buffl=NULL;
	char buffn[CHARN];*buffn=NULL;
	char buffo[CHARN];*buffo=NULL;
	char buff_stoch[CHARN];*buff_stoch=NULL;
	char numerical[CHARN];*numerical=NULL;
	char line_clear[CHARL];*line_clear=NULL;
	bool ident=false;
	bool def_found=false;

	//opening existing input.asc file
	fstream input1("input.asc");
	if(!input1){cout<<" *** Error: cannot open 'input.asc' file *** \n";system("pause");exit(1);}

	//opening new copy file
	fstream fcopy("input_copy.asc");
	if(!fcopy){cout<<" *** Error: cannot open 'input_copy.asc' file *** \n";system("pause");exit(1);}

	//copying 'input.asc' to 'input_copy.asc'
	do{
		input1.getline(buffl,CHARL,'\n');
		fcopy<<buffl<<'\n';
	}while(!input1.eof());

	//creating new output stream to file 'input.asc' and destroying all previous data
	ofstream input;
	input.open("input.asc",ios::out|ios::trunc);

	//reset file pointers to beginning
	fcopy.seekp(ios::beg);

	//copying from 'fcopy' stream to 'input' stream until 'VEHICLES' is reached
	do{
		fcopy.getline(buffl,CHARL,'\n');
		input<<buffl<<'\n';

	}while(!strstr(buffl,"VEHICLES"));

	//reading until STOP of input_copy file
	input.setf(ios::left);
	do{
		fcopy>>buffo;;//buffo gets HYPER6, SAT3, RADAR0, ENDTIME STOP and //comments

		//finding HYPER6 object
		if(!strcmp(buffo,"HYPER6")){
			input<<'\t'<<buffo;		//writes HYPER6 line
			fcopy.getline(line_clear,CHARL,'\n');
			input<<line_clear<<'\n';

			//inside HYPER6 loop until 'END' is reached
			do{
				fcopy>>buffn;
				int dum=1;
				//inserting whole line starting with key word IF
				if(!strcmp(buffn,"IF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<'\n';
					//set ident
					ident=true;
				}
				//inserting whole line starting with key word ENDIF
				else if(!strcmp(buffn,"ENDIF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<'\n';
					//set ident
					ident=false;
				}
				//reading and writing comment lines
				else if(ispunct(buffn[0])){
					//reading and writing comment lines
					if(ident)
						input<<"\t\t\t"<<buffn;
					else
						input<<"\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<'\n';
				}
				//inserting whole line starting with certain key words
				else if(!strcmp(buffn,"AERO_DECK")||!strcmp(buffn,"PROP_DECK")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<'\n';
				}
				//inserting 'END' with only one tab
				else if(!strcmp(buffn,"END")){
					input<<'\t'<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<'\n';
				}
				else{
					//jumping over stochastic key word
					if(!strcmp(buffn,"UNI")||!strcmp(buffn,"GAUSS")||!strcmp(buffn,"RAYL")
						||!strcmp(buffn,"EXP")||!strcmp(buffn,"MARKOV")){
							if(ident)
								input<<"\t\t\t\t"<<buffn;
							else
								input<<"\t\t\t"<<buffn;
							strcpy(buff_stoch,buffn);
							fcopy>>buffn;
							input<<" "<<buffn;
					}
					else{
						//inserting module-variable name
						if(ident)
							input<<"\t\t\t\t"<<buffn;
						else
							input<<"\t\t\t"<<buffn;
					}
					//jumping over numerical value,  twice for some stochastic variables
					fcopy>>numerical;
					input<<"  "<<numerical;
					if(!strcmp(buff_stoch,"UNI")||!strcmp(buff_stoch,"GAUSS")||!strcmp(buff_stoch,"MARKOV")){					
						fcopy>>numerical;
						input<<"  "<<numerical;
					}
					//getting defintion for module-variable stored in 'buffn'
					def_found=false; 
					for(int k=0;k<(NROUND6+NHYPER);k++){
						if(!strcmp(doc_hyper6[k].get_name(),"end_array"))break;
						if(!strcmp(doc_hyper6[k].get_name(),buffn)){
							fcopy.getline(line_clear,CHARL,'\n'); 							
							input<<"    //";
							if(!strcmp(doc_hyper6[k].get_type(),"int"))input<<"'int' ";
							input<<doc_hyper6[k].get_def();
							input<<"  module ";
							input<<doc_hyper6[k].get_mod();
							input<<'\n';
							*buff_stoch=NULL;
							def_found=true;
						}
					}
					//if 'name' has no defintion clear line
					if(!def_found){
						fcopy.getline(line_clear,CHARL,'\n');
						input<<"   //*** <<< Check spelling"<<'\n';
					}
				}
			}while(strcmp(buffn,"END"));
		}//end of HYPER6 has been reached

		else if(!strcmp(buffo,"SAT3")){
			input<<'\t'<<buffo;		//writes SAT3 line
			fcopy.getline(line_clear,CHARL,'\n');
			input<<line_clear<<endl;

			//inside SAT3 loop until 'END' is reached
			do{
				fcopy>>buffn;
				//inserting whole line starting with key word IF
				if(!strcmp(buffn,"IF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
					//set ident
					ident=true;
				}
				//inserting whole line starting with key word ENDIF
				else if(!strcmp(buffn,"ENDIF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
					//set ident
					ident=false;
				}
				//reading and writing comment lines
				else if(ispunct(buffn[0])){
					//reading and writing comment lines
					if(ident)
						input<<"\t\t\t"<<buffn;
					else
						input<<"\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				//inserting whole line starting with certain key words
				else if(!strcmp(buffn,"AERO_DECK")||!strcmp(buffn,"PROP_DECK")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				//inserting 'END' with only one tab
				else if(!strcmp(buffn,"END")){
					input<<'\t'<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				else{
					//jumping over stochastic key word
					if(!strcmp(buffn,"UNI")||!strcmp(buffn,"GAUSS")||!strcmp(buffn,"RAYL")
						||!strcmp(buffn,"EXP")||!strcmp(buffn,"MARKOV")){
							if(ident)
								input<<"\t\t\t\t"<<buffn;
							else
								input<<"\t\t\t"<<buffn;
							strcpy(buff_stoch,buffn);
							fcopy>>buffn;
							input<<" "<<buffn;
					}
					else{
						//inserting module-variable name
						if(ident)
							input<<"\t\t\t\t"<<buffn;
						else
							input<<"\t\t\t"<<buffn;
					}
					//jumping over numerical value,  twice for some stochastic variables
					fcopy>>numerical;
					input<<"  "<<numerical;
					if(!strcmp(buff_stoch,"UNI")||!strcmp(buff_stoch,"GAUSS")||!strcmp(buff_stoch,"MARKOV")){					
						fcopy>>numerical;
						input<<"  "<<numerical;
					}
					//getting defintion for module-variable stored in 'buffn'
					def_found=false; 
					for(int k=0;k<(NROUND3+NSAT);k++){
						if(!strcmp(doc_satellite3[k].get_name(),"end_array"))break;
						if(!strcmp(doc_satellite3[k].get_name(),buffn)){
							fcopy.getline(line_clear,CHARL,'\n'); 							
							input<<"    //";
							if(!strcmp(doc_satellite3[k].get_type(),"int"))input<<"'int' ";
							input<<doc_satellite3[k].get_def();
							input<<"  module ";
							input<<doc_satellite3[k].get_mod();
							input<<endl;
							*buff_stoch=NULL;
							def_found=true;
						}
					}
					//if 'name' has no defintion clear line
					if(!def_found){
						fcopy.getline(line_clear,CHARL,'\n');
						input<<"   //*** <<< Check spelling"<<endl;;
					}
				}
			}while(strcmp(buffn,"END"));
		}//end of SAT3 has been reached

		else if(!strcmp(buffo,"RADAR0")){
			input<<'\t'<<buffo;		//writes RADAR0 line
			fcopy.getline(line_clear,CHARL,'\n');
			input<<line_clear<<endl;

			//inside RADAR0 loop until 'END' is reached
			do{
				fcopy>>buffn;
				//inserting whole line starting with key word IF
				if(!strcmp(buffn,"IF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
					//set ident
					ident=true;
				}
				//inserting whole line starting with key word ENDIF
				else if(!strcmp(buffn,"ENDIF")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
					//set ident
					ident=false;
				}
				//reading and writing comment lines
				else if(ispunct(buffn[0])){
					//reading and writing comment lines
					if(ident)
						input<<"\t\t\t"<<buffn;
					else
						input<<"\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				//inserting whole line starting with certain key words
				else if(!strcmp(buffn,"AERO_DECK")||!strcmp(buffn,"PROP_DECK")){
					input<<"\t\t\t"<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				//inserting 'END' with only one tab
				else if(!strcmp(buffn,"END")){
					input<<'\t'<<buffn;
					fcopy.getline(line_clear,CHARL,'\n');
					input<<line_clear<<endl;
				}
				else{
					//jumping over stochastic key word
					if(!strcmp(buffn,"UNI")||!strcmp(buffn,"GAUSS")||!strcmp(buffn,"RAYL")
						||!strcmp(buffn,"EXP")||!strcmp(buffn,"MARKOV")){
							if(ident)
								input<<"\t\t\t\t"<<buffn;
							else
								input<<"\t\t\t"<<buffn;
							strcpy(buff_stoch,buffn);
							fcopy>>buffn;
							input<<" "<<buffn;
					}
					else{
						//inserting module-variable name
						if(ident)
							input<<"\t\t\t\t"<<buffn;
						else
							input<<"\t\t\t"<<buffn;
					}
					//jumping over numerical value,  twice for some stochastic variables
					fcopy>>numerical;
					input<<"  "<<numerical;
					if(!strcmp(buff_stoch,"UNI")||!strcmp(buff_stoch,"GAUSS")||!strcmp(buff_stoch,"MARKOV")){					
						fcopy>>numerical;
						input<<"  "<<numerical;
					}
					//getting defintion for module-variable stored in 'buffn'
					def_found=false; 
					for(int k=0;k<(NROUND3+NRADAR);k++){
						if(!strcmp(doc_radar0[k].get_name(),"end_array"))break;
						if(!strcmp(doc_radar0[k].get_name(),buffn)){
							fcopy.getline(line_clear,CHARL,'\n'); 							
							input<<"    //";
							if(!strcmp(doc_radar0[k].get_type(),"int"))input<<"'int' ";
							input<<doc_radar0[k].get_def();
							input<<"  module ";
							input<<doc_radar0[k].get_mod();
							input<<endl;
							*buff_stoch=NULL;
							def_found=true;
						}
					}
					//if 'name' has no defintion clear line
					if(!def_found){
						fcopy.getline(line_clear,CHARL,'\n');
						input<<"   //*** <<< Check spelling"<<endl;;
					}
				}
			}while(strcmp(buffn,"END"));
		}//end of RADAR0 has been reached

		//inserting last three key words
		else{
			input<<buffo;
			fcopy.getline(line_clear,CHARL,'\n');
 			input<<line_clear<<endl;
		}
		*buffl=NULL;
		*buffn=NULL;
		*buff_stoch=NULL;
		*numerical=NULL;
		*line_clear=NULL;
	}while(strcmp(buffo,"STOP"));

	*buffo=NULL;
	input.clear();
	fcopy.clear();
	input.close();
	fcopy.close();  
}
