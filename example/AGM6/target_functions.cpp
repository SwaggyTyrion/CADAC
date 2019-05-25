///////////////////////////////////////////////////////////////////////////////
//FILE: 'target_functions.cpp'
//
// Contains utilitiy functions for the 'Target' class:
//		array sizing
//		writing data to output
//
//010206 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'com_target'
// 
//Out to Target:: flat3_com_count, target_com_count, ncom_target3 		 ,
//
//010207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Target::sizing_arrays()
{
	char *key4="com";
	int i(0);

	flat3_com_count=0;
	target_com_count=0;

	//counting in 'flat3' array
 	for(i=0;i<NFLAT3;i++)
	{
		if(strstr(flat3[i].get_out(),key4))
			flat3_com_count++;
	}

	//counting in 'target' array
 	for(i=0;i<NTARGET;i++)
	{
		if(strstr(target[i].get_out(),key4))
			target_com_count++;
	}

	//output to Target::protected
	ncom_target3=flat3_com_count+target_com_count;
}
///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each vehicle object
//Assigning initial values to module-variables of 'flat3' and 'target' arrays
//Reading aero and propulsion decks
//
//The first vehicle 'TARGET3' of 'input.asc' reads until the first 'END' after
//'TARGET3'. The second vehicle object reads untile the second 'END', etc until
//the data for all vehicle objects are read
//
//Output:	target3_name ('Target' data member)
//			flat3[] data values (Flat3 data member)
//			target[] data values ('Target' data member)
//			markov_list[] ('Markov' list of variables)
//
//Limitation: real and integer variables only (could be expanded to vectors)			 
//
//001230 Created by Peter H Zipfel
//010930 Added reading of random variables, PZi
///////////////////////////////////////////////////////////////////////////////
void Target::vehicle_data(fstream &input,int nmonte)
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

	input.getline(target3_name,CHARL,'\n');

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
			for(i=0;i<NFLAT3;i++)
			{
				buff=flat3[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=flat3[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						flat3[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						flat3[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			for(i=0;i<NTARGET;i++)
			{
				buff=target[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=target[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						target[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						target[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			//reading aero data from aero-deck file
			if(!strcmp(read,"AERO_DECK")){
				//reading aerodeck file name
				input>>file_name;
				input.getline(line_clear,CHARL,'\n');

				read_tables(file_name,aerotable);
			}
			//reading prop data from prop-deck file
			if(!strcmp(read,"PROP_DECK")){
				//reading propdeck file name
				input>>file_name;
				input.getline(line_clear,CHARL,'\n');

				read_tables(file_name,proptable);
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
				for(kk=0;kk<NFLAT3;kk++)
				{
					buff=flat3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NTARGET;kk++)
				{
					buff=target[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						target[kk].gets(value);
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
				for(kk=0;kk<NFLAT3;kk++)
				{
					buff=flat3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NTARGET;kk++)
				{
					buff=target[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						target[kk].gets(value);
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
				for(kk=0;kk<NFLAT3;kk++)
				{
					buff=flat3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NTARGET;kk++)
				{
					buff=target[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						target[kk].gets(value);
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
				for(kk=0;kk<NFLAT3;kk++)
				{
					buff=flat3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						flat3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NTARGET;kk++)
				{
					buff=target[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						target[kk].gets(value);
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
				for(ii=0;ii<NFLAT3;ii++)
				{
					buff=flat3[ii].get_name();
					if(!strcmp(buff,name1)) 
					{
						markov_list[nmarkov].set_markov_flat3_index(ii);
						flat3[ii].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(ii=0;ii<NTARGET;ii++)
				{
					buff=target[ii].get_name();
					if(!strcmp(buff,name1)) 
					{
						markov_list[nmarkov].set_markov_vehicle_index(ii);
						target[ii].gets(value);
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
//Building index array of those 'flat3[]' and target[] variables 
//that are output to 'combus' 'data'  
//
//Output: Target::flat3_com_ind[], target_com_ind[] 
//
//010210 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Target::com_index_arrays()
{
	const char *test="com";
	int i(0);

	int k=0;
	for(i=0;i<NFLAT3;i++)
	{
		if(strstr(flat3[i].get_out(),test))
		{
			flat3_com_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NTARGET;i++)
	{
		if(strstr(target[i].get_out(),test))
		{
			target_com_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Initializing loading 'packet' of target
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//differs from 'loading_packet' only by initializing 'status=1'
//
//010401 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Packet Target::loading_packet_init(int num_missile,int num_aircraft,int num_target)
{
	string id;
	char object[4];
	static int c_count=0;
	int index(0);
	int i(0);
	
	c_count++;
	if(c_count==(num_target+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="t"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<flat3_com_count;i++)
	{
		index=flat3_com_ind[i];
		com_target3[i]=flat3[index];
	}
	for(int j=0;j<target_com_count;j++)
	{
		index=target_com_ind[j];
		com_target3[i+j]=target[index];
	}
	//refreshing the packet
	packet.set_id(id);
	packet.set_status(1);
	packet.set_data(com_target3);
	packet.set_ndata(ncom_target3);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Loading 'packet' of target
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//
//010207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Packet Target::loading_packet(int num_missile,int num_aircraft,int num_target)
{
	int index(0);
	int i(0);

	string id;
	char object[4];
	static int c_count=0;
	
	c_count++;
	if(c_count==(num_target+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="t"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<flat3_com_count;i++)
	{
		index=flat3_com_ind[i];
		com_target3[i]=flat3[index];
	}
	for(int j=0;j<target_com_count;j++)
	{
		index=target_com_ind[j];
		com_target3[i+j]=target[index];
	}
	//refreshing the packet
	packet.set_data(com_target3);
	packet.set_ndata(ncom_target3);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Composing documention on file 'doc.asc'
//Listing 'target' module-variable arrays
//
//010214 Created by Peter H Zipfel
//020911 Added module-variable error flagging, PZi
///////////////////////////////////////////////////////////////////////////////

void Target::document(ostream &fdoc,char *title,Document *doc_target3)
{
	int i(0);

	fdoc<<"\n\n                                       Target Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	for(i=0;i<NTARGET;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(target[i].get_name(),target[j].get_name())&&strcmp(target[i].get_name(),"empty"))
				target[i].put_error(name_error_code);
		}				
		if(!strcmp(target[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in target[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(target[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in target[] array, see 'doc.asc' ***\n"; 

		fdoc<<target[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(target[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<target[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<target[i].get_name();
		}
		fdoc.width(54);fdoc<<target[i].get_def();
		fdoc.width(13);fdoc<<target[i].get_mod();
		fdoc.width(10);fdoc<<target[i].get_role();
		fdoc<<target[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}
	fdoc<<"\n\n                                       Flat3 Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	for(i=0;i<NFLAT3;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(flat3[i].get_name(),flat3[j].get_name())&&strcmp(flat3[i].get_name(),"empty"))
				flat3[i].put_error(name_error_code);
		}				
		if(!strcmp(flat3[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in flat3[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(flat3[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in flat3[] array, see 'doc.asc' ***\n"; 
	  
		fdoc<<flat3[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(flat3[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<flat3[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<flat3[i].get_name();
		}
		fdoc.width(54);fdoc<<flat3[i].get_def();
		fdoc.width(13);fdoc<<flat3[i].get_mod();
		fdoc.width(10);fdoc<<flat3[i].get_role();
		fdoc<<flat3[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}
	//building doc_target3[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NTARGET;i++){
		if(strcmp(target[i].get_name(),"empty")){
			doc_target3[counter].put_doc_offset(counter);
			doc_target3[counter].put_name(target[i].get_name());
			doc_target3[counter].put_type(target[i].get_type());
			doc_target3[counter].put_def(target[i].get_def());
			doc_target3[counter].put_mod(target[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NFLAT3;i++){
		if(strcmp(flat3[i].get_name(),"empty")){
			doc_target3[counter].put_doc_offset(counter);
			doc_target3[counter].put_name(flat3[i].get_name());
			doc_target3[counter].put_type(flat3[i].get_type());
			doc_target3[counter].put_def(flat3[i].get_def());
			doc_target3[counter].put_mod(flat3[i].get_mod());
			counter++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Reading tables from table decks
//
//Supports 1, 2, 3 dim tables stored seperately in data files
// Keying on AERO_DECK and PROP_DECK in 'input.asc' this function  reads the tables
// from the data files and stores them in 'Plane::Datadeck aerotable' and 
// 'Plane::Datadeck proptable'. In the modules, table look up is carried out by 
// double value=aerodeck.look_up(string name, double var1);  
// double value=aerodeck.look_up(string name, double var1,double var2);  
// double value=aerodeck.look_up(string name, double var1,double var2,double var3);  
//
// To add new tables, just include them in the files of  AERO_DECK and PROP_DECK
// For debugging puposes un-comment the print out provision of the tables below
//
//030721 Created by Peter H Zipfel
//031104 Corrected table diagnostic, PZi
//060505 Corrected line 1742 (changed 'else if' to 'if'), TPo
//070419 Included here in target_functions.cpp', PZi
///////////////////////////////////////////////////////////////////////////////
void Target::read_tables(char *file_name,Datadeck &datatable)
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

	//opening aero-deck file stream
	ifstream tbl_stream(file_name);

	if(tbl_stream.fail())
		{cerr<<"*** Error: File stream '"<<file_name<<"' failed to open (check spelling) ***\n";system("pause");exit(1);}

	//determing the total # of tbl_stream
	while(!tbl_stream.eof())
	{
		tbl_stream>>temp;
		if(!strcmp(temp,"TITLE")){
			tbl_stream.getline(line_clear,CHARL,'\n');
			table_deck_title=line_clear;
		}
		if(strstr(temp,"DIM"))
			table_count++;

	} //EOF reached of aero_deck

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