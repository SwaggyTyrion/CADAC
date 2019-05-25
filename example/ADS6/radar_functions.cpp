///////////////////////////////////////////////////////////////////////////////
//FILE: 'radar_functions.cpp'
//
// Contains utilitiy functions for the 'Radar' class:
//		array sizing
//		writing data to output
//
//170916 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'com_radar'
// 
//Out to Radar:: flat0_com_count, radar_com_count, ncom_radar0 		 ,
//
//170916 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Radar::sizing_arrays()
{
	char *key4="com";
	int i(0);

	flat0_com_count=0;
	radar_com_count=0;

	//counting in 'flat0' array
 	for(i=0;i<NFLAT0;i++)
	{
		if(strstr(flat0[i].get_out(),key4))
			flat0_com_count++;
	}

	//counting in 'radar' array
 	for(i=0;i<NRADAR;i++)
	{
		if(strstr(radar[i].get_out(),key4))
			radar_com_count++;
	}

	//output to Radar::protected
	ncom_radar0=flat0_com_count+radar_com_count;
}
///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each vehicle object
//Assigning initial values to module-variables of 'flat0' and 'radar' arrays
//Reading trajectory decks
//
//The first vehicle 'RADAR0' of 'input.asc' reads until the first 'END' after
//'RADAR0'. The second vehicle object reads untile the second 'END', etc until
//the data for all vehicle objects are read
//
//Output:	radar0_name ('Radar' data member)
//			flat0[] data values (Flat0 data member)
//			radar[] data values ('Radar' data member)
//			markov_list[] ('Markov' list of variables)
//
//Limitation: real and integer variables only			 
//
//170916 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Radar::vehicle_data(fstream &input,int nmonte)
{
	char line_clear[CHARL];
	char read[CHARN];	//name of variable read from input.asc
	char *buff;			//name of module-variable
	double data;		//data of module-variable 
	char file_name[CHARN];	//name of data-deck file
	char *integer;
	int int_data;
	int file_ptr=NULL;
	int e=0;
	char *watchpoint=NULL;
	double value;
	double first;
	double second;
	char name1[CHARN];
	int kk(0);
	int i(0);
	int ii(0);

	input.getline(radar0_name,CHARL,'\n');

	//reading data until END is encountered
	do
	{
		//reading variable data into module-variable arrays
		input>>read;
		if(ispunct(read[0]))
		{
			input.getline(line_clear,CHARL,'\n');
		}
		else
		{
			for(i=0;i<NFLAT0;i++)
			{
				buff=flat0[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=flat0[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						flat0[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						flat0[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			for(i=0;i<NRADAR;i++)
			{
				buff=radar[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=radar[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						radar[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						radar[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			//reading trajectory data from SRBM_traj_deck.asc
			if(!strcmp(read,"SRBM_DECK")){
				//reading aerodeck file name
				input>>file_name;
				input.getline(line_clear,CHARL,'\n');

				read_tables(file_name,rocket_traj);
			}
			//reading trajectory data from SAM_traj_deck.asc
			if(!strcmp(read,"SAM_DECK")){
				//reading propdeck file name
				input>>file_name;
				input.getline(line_clear,CHARL,'\n');

				read_tables(file_name,missile_traj);
			}
			//loading values for random variables and building 'markov_list'
			//uniform distribution
			if(!strcmp(read,"UNI"))
			{
				input>>name1;
				input>>first;
				input>>second;
				if(!nmonte)
					value=(second-first)/2.;
				else
					value=uniform(first,second);

				//loading radom value into module-variable
				for(kk=0;kk<NFLAT0;kk++)
				{
					buff=flat0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat0[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NRADAR;kk++)
				{
					buff=radar[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						radar[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
			}
			//Gaussian distribution
			if(!strcmp(read,"GAUSS"))
			{
				input>>name1;
				input>>first;
				input>>second;
				if(!nmonte)
					value=first;
				else
					value=gauss(first,second);

				//loading radom value into module-variable
				for(kk=0;kk<NFLAT0;kk++)
				{
					buff=flat0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat0[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NRADAR;kk++)
				{
					buff=radar[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						radar[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
			}
			//Rayleigh distribution
			if(!strcmp(read,"RAYL"))
			{
				input>>name1;
				input>>first;
				if(!nmonte)
					value=first;
				else
					value=rayleigh(first);

				//loading radom value into module-variable
				for(kk=0;kk<NFLAT0;kk++)
				{
					buff=flat0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat0[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NRADAR;kk++)
				{
					buff=radar[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						radar[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
			}
			//exponential distribution
			if(!strcmp(read,"EXP"))
			{
				input>>name1;
				input>>first;
				if(!nmonte)
					value=first;
				else
					value=exponential(first);

				//loading radom value into module-variable
				for(kk=0;kk<NFLAT0;kk++)
				{
					buff=flat0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat0[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NRADAR;kk++)
				{
					buff=radar[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						radar[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
			}
			//Markov variable, initialize with Gaussian distribution
			if(!strcmp(read,"MARKOV"))
			{
				//reading input
				input>>name1;
				input>>first;
				input>>second;
				if(!nmonte)
					value=0;
				else
					value=gauss(0,first);

				//storing information in 'markov_list'
				markov_list[nmarkov].set_markov_sigma(first);
				markov_list[nmarkov].set_markov_bcor(second);
				markov_list[nmarkov].set_markov_saved(0.);
				markov_list[nmarkov].set_markov_status(true);

				//locating and storing module-variable index and initializing value
				for(ii=0;ii<NFLAT0;ii++)
				{
					buff=flat0[ii].get_name();
					if(!strcmp(buff,name1)) 
					{
						markov_list[nmarkov].set_markov_flat0_index(ii);
						flat0[ii].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(ii=0;ii<NRADAR;ii++)
				{
					buff=radar[ii].get_name();
					if(!strcmp(buff,name1)) 
					{
						markov_list[nmarkov].set_markov_vehicle_index(ii);
						radar[ii].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				nmarkov++;						
			}
		} //end of reading non-comment lines
	}while(strcmp(read,"END")); //reached 'END' of first vehicle object

	//flushing the line after END and starting new line
	input.getline(line_clear,CHARL,'\n'); 

	//diagnostic: file pointer
	file_ptr=int(input.tellg());
}
///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'flat0[]' and radar[] variables 
//that are output to 'combus' 'data'  
//
//Output: Radar::flat0_com_ind[], radar_com_ind[] 
//
//010210 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Radar::com_index_arrays()
{
	const char *test="com";
	int i(0);

	int k=0;
	for(i=0;i<NFLAT0;i++)
	{
		if(strstr(flat0[i].get_out(),test))
		{
			flat0_com_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NRADAR;i++)
	{
		if(strstr(radar[i].get_out(),test))
		{
			radar_com_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Initializing loading 'packet' of radar
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//differs from 'loading_packet' only by initializing 'status=1'
//
//010401 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Packet Radar::loading_packet_init(int num_missile,int num_aircraft,int num_rocket, int num_radar)
{
	string id;
	char object[4];
	static int c_count=0;
	int index(0);
	int i(0);
	
	c_count++;
	if(c_count==(num_radar+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="f"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<flat0_com_count;i++)
	{
		index=flat0_com_ind[i];
		com_radar0[i]=flat0[index];
	}
	for(int j=0;j<radar_com_count;j++)
	{
		index=radar_com_ind[j];
		com_radar0[i+j]=radar[index];
	}
	//refreshing the packet
	packet.set_id(id);
	packet.set_status(1);
	packet.set_data(com_radar0);
	packet.set_ndata(ncom_radar0);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Loading 'packet' of radar
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//
//010207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Packet Radar::loading_packet(int num_missile,int num_aircraft,int num_rocket,int num_radar)
{
	int index(0);
	int i(0);

	string id;
	char object[4];
	static int c_count=0;
	
	c_count++;
	if(c_count==(num_radar+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="f"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<flat0_com_count;i++)
	{
		index=flat0_com_ind[i];
		com_radar0[i]=flat0[index];
	}
	for(int j=0;j<radar_com_count;j++)
	{
		index=radar_com_ind[j];
		com_radar0[i+j]=radar[index];
	}
	//refreshing the packet
	packet.set_data(com_radar0);
	packet.set_ndata(ncom_radar0);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Composing documention on file 'doc.asc'
//Listing 'radar' module-variable arrays
//
//170916 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Radar::document(ostream &fdoc,char *title,Document *doc_radar0)
{
	int i(0);

	fdoc<<"\n\n                                       Radar Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	for(i=0;i<NRADAR;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(radar[i].get_name(),radar[j].get_name())&&strcmp(radar[i].get_name(),"empty"))
				radar[i].put_error(name_error_code);
		}				
		if(!strcmp(radar[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in radar[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(radar[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in radar[] array, see 'doc.asc' ***\n"; 

		fdoc<<radar[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(radar[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<radar[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<radar[i].get_name();
		}
		fdoc.width(54);fdoc<<radar[i].get_def();
		fdoc.width(13);fdoc<<radar[i].get_mod();
		fdoc.width(10);fdoc<<radar[i].get_role();
		fdoc<<radar[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}
	fdoc<<"\n\n                                       Flat0 Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	for(i=0;i<NFLAT0;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(flat0[i].get_name(),flat0[j].get_name())&&strcmp(flat0[i].get_name(),"empty"))
				flat0[i].put_error(name_error_code);
		}				
		if(!strcmp(flat0[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in flat0[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(flat0[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in flat0[] array, see 'doc.asc' ***\n"; 
	  
		fdoc<<flat0[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(flat0[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<flat0[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<flat0[i].get_name();
		}
		fdoc.width(54);fdoc<<flat0[i].get_def();
		fdoc.width(13);fdoc<<flat0[i].get_mod();
		fdoc.width(10);fdoc<<flat0[i].get_role();
		fdoc<<flat0[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}
	//building doc_radar0[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NRADAR;i++){
		if(strcmp(radar[i].get_name(),"empty")){
			doc_radar0[counter].put_doc_offset(counter);
			doc_radar0[counter].put_name(radar[i].get_name());
			doc_radar0[counter].put_type(radar[i].get_type());
			doc_radar0[counter].put_def(radar[i].get_def());
			doc_radar0[counter].put_mod(radar[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NFLAT0;i++){
		if(strcmp(flat0[i].get_name(),"empty")){
			doc_radar0[counter].put_doc_offset(counter);
			doc_radar0[counter].put_name(flat0[i].get_name());
			doc_radar0[counter].put_type(flat0[i].get_type());
			doc_radar0[counter].put_def(flat0[i].get_def());
			doc_radar0[counter].put_mod(flat0[i].get_mod());
			counter++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Reading tables from table decks
//
//Supports 1, 2, 3 dim tables stored seperately in data files
// Keying on SRBM_DECK and SAM_DECK in 'input.asc' this function  reads the tables
// from the data files and stores them in 'Radar::Datadeck rocket_traj' and 
// 'Radar::Datadeck missile_traj'.
// In the modules, table look up is carried out by, e.g.: 
// double value=rocket_traj.look_up(string name, double var1);  
//
// To add new tables, just include them in the files of  SRBM_DECK and SAM_DECK
// For debugging puposes un-comment the print out provision of the tables below
//
//170916 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Radar::read_tables(char *file_name,Datadeck &datatable)
{
	char line_clear[CHARL];
	char temp[CHARN];	//buffer for table data
	string table_deck_title;
	int table_count(0);
	int table_dim(0);
	double value(0);
	int file_ptr=NULL;
	int var_dim[3]={1,1,1,};
	int tt(0);

	//opening traj-deck file stream
	ifstream tbl_stream(file_name);

	if(tbl_stream.fail())
		{cerr<<"*** Error: File stream '"<<file_name<<"' failed to open (check spelling) ***\n";exit(1);}

	//determing the total # of table streams
	while(!tbl_stream.eof())
	{
		tbl_stream>>temp;
		if(!strcmp(temp,"TITLE")){
			tbl_stream.getline(line_clear,CHARL,'\n');
			table_deck_title=line_clear;
		}
		if(strstr(temp,"DIM"))
			table_count++;

	} //EOF reached of table streams

	//removing EOF bit (-1)
	tbl_stream.clear();
	//rewinding to beginning
	tbl_stream.seekg(ios::beg);

	//creating pointer array table 
	datatable.set_title(table_deck_title);
	datatable.set_capacity(table_count);
	datatable.alloc_mem();

	//discarding all entries until first DIM
	do{
		tbl_stream>>temp;
	}while(!strstr(temp,"DIM"));

	//loading tables one at a time
	for(int t=0;t<table_count;t++){

		//creating and allocating memory to object 'table'
		table=new Table;
			
		//extracting table dimension
		//at this point 'temp' is holding xDIM
		int dim_check(0);
		char dim_buff[2];
		int table_dim(0);
		strncpy(dim_buff,temp,1);
		//converting character to integer
		dim_check=sscanf(dim_buff,"%d",&table_dim);
		table->set_dim(table_dim);
								
		//extracting table name
		tbl_stream>>temp;
		table->set_name(temp);
		tbl_stream.getline(line_clear,CHARL,'\n');

		//extracting dimensions of independent variables
		var_dim[0]=1;var_dim[1]=1;var_dim[2]=1;
		for(tt=0;tt<table_dim;tt++){
			tbl_stream>>temp;
			tbl_stream>>var_dim[tt];
			if(tt==0)
				table->set_var1_dim(var_dim[tt]);
			if(tt==1)
				table->set_var2_dim(var_dim[tt]);
			if(tt==2)
				table->set_var3_dim(var_dim[tt]);
		}
		tbl_stream.getline(line_clear,CHARL,'\n');

		//allocating memory for variables and data arrays
		table->var1_values=new double [var_dim[0]];
		table->var2_values=new double [var_dim[1]];
		table->var3_values=new double [var_dim[2]];
		table->data=new double[var_dim[0]*var_dim[1]*var_dim[2]];

		//determining max number of rows of data
		int num_rows=var_dim[0];
		if(var_dim[0]<var_dim[1]) num_rows=var_dim[1];
		if(var_dim[2]>num_rows) num_rows=var_dim[2];

		//reading num_row of data 
		for(tt=0;tt<num_rows;tt++){

			//loading 1.variable values
			if(tt<var_dim[0]){
				tbl_stream>>value;
				table->set_var1_value(tt,value);
			}
			//loading 2.variable values, but bypass if default dimension one
			if(tt<var_dim[1]&&var_dim[1]!=1){
				tbl_stream>>value;
				table->set_var2_value(tt,value);
			}
			//loading 3.variable values, but bypass if default dimension one
			if(tt<var_dim[2]&&var_dim[2]!=1){
				tbl_stream>>value;
				table->set_var3_value(tt,value);
			}
			//loading tabular data, which in all cases has only 'var_dim[0]' rows
			if(tt<var_dim[0]){

				//read one row of data
				for(int ttt=0;ttt<var_dim[1]*var_dim[2];ttt++){
					tbl_stream>>value;
					table->set_data(tt*var_dim[1]*var_dim[2]+ttt,value);
				}
			}
		}//end of reading data

		//loading table into 'Datadeck' pointer array 'Table **tabel_ptr'
		datatable.set_counter(t);
		datatable.add_table(*table);
		tbl_stream>>temp; //reading next DIM entry
		
	}//end of 'for' loop, finished loading all tables
}