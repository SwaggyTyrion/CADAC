///////////////////////////////////////////////////////////////////////////////
//FILE: 'aircraft_functions.cpp'
//
// Contains utilitiy functions for the 'Aircraft' class:
//		array sizing
//		writing data to output
//
//010206 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each vehicle object
//Assigning initial values to module-variables of 'flat3' and 'aircraft' arrays
//Reading aero and propulsion decks
//
//The first vehicle 'AIRCRAFT3' of 'input.asc' reads until the first 'END' after
//'AIRCRAFT3'. The second vehicle object reads untile the second 'END', etc until
//the data for all vehicle objects are read
//
//Output:	aircraft3_name ('Aircraft' data member)
//			flat3[] data values (Flat3 data member)
//			aircraft[] data values ('Aircraft' data member)
//			markov_list[] ('Markov' list of variables)
//
//Limitation: real and integer variables only (could be expanded to vectors)			 
//
//001230 Created by Peter H Zipfel
//010930 Added reading of random variables, PZi
///////////////////////////////////////////////////////////////////////////////
void Aircraft::vehicle_data(fstream &input,int nmonte)
{

	char line_clear[CHARL];
	char read[CHARN];	//name of variable read from input.asc
	char *buff;			//name of module-variable
	double data;		//data of module-variable 
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

	input.getline(aircraft3_name,CHARL,'\n');

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
			for(i=0;i<NAIRCRAFT;i++)
			{
				buff=aircraft[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=aircraft[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						aircraft[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						aircraft[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
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
				for(kk=0;kk<NAIRCRAFT;kk++)
				{
					buff=aircraft[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						aircraft[kk].gets(value);
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
				for(kk=0;kk<NAIRCRAFT;kk++)
				{
					buff=aircraft[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						aircraft[kk].gets(value);
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
				for(kk=0;kk<NAIRCRAFT;kk++)
				{
					buff=aircraft[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						aircraft[kk].gets(value);
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
				for(kk=0;kk<NAIRCRAFT;kk++)
				{
					buff=aircraft[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						aircraft[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
			}
		} //end of reading non-comment lines
	}while(strcmp(read,"END")); //reached 'END' of first vehicle object

	//flushing the line after END and starting new line
	input.getline(line_clear,CHARL,'\n'); 

	//diagnostic: file pointer
	file_ptr=int(input.tellg());
}
///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'com_aircraft'
// 
//Out to Aircraft:: flat3_com_count, aircraft_com_count, ncom_aircraft3 		 ,
//
//010207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::sizing_arrays()
{
	char *key4="com";
	int i(0);

	flat3_com_count=0;
	aircraft_com_count=0;

	//counting in 'flat3' array
 	for(i=0;i<NFLAT3;i++)
	{
		if(strstr(flat3[i].get_out(),key4))
			flat3_com_count++;
	}

	//counting in 'aircraft' array
 	for(i=0;i<NAIRCRAFT;i++)
	{
		if(strstr(aircraft[i].get_out(),key4))
			aircraft_com_count++;
	}

	//output to Aircraft::protected
	ncom_aircraft3=flat3_com_count+aircraft_com_count;
}
///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'flat3[]' and aircraft[] variables 
//that are output to 'combus' 'data'  
//
//Output: Aircraft::flat3_com_ind[], aircraft_com_ind[] 
//
//010210 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
void Aircraft::com_index_arrays()
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
	for(i=0;i<NAIRCRAFT;i++)
	{
		if(strstr(aircraft[i].get_out(),test))
		{
			aircraft_com_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Initializing loading 'packet' of aircraft
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//differs from 'loading_packet' only by initializing 'status=1'
//
//010401 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Packet Aircraft::loading_packet_init(int num_missile,int num_aircraft,int num_target)
{
	string id;
	char object[4];
	static int c_count=0;
	int index(0);
	int i(0);
	
	c_count++;
	if(c_count==(num_aircraft+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="a"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<flat3_com_count;i++)
	{
		index=flat3_com_ind[i];
		com_aircraft3[i]=flat3[index];
	}
	for(int j=0;j<aircraft_com_count;j++)
	{
		index=aircraft_com_ind[j];
		com_aircraft3[i+j]=aircraft[index];
	}
	//refreshing the packet
	packet.set_id(id);
	packet.set_status(1);
	packet.set_data(com_aircraft3);
	packet.set_ndata(ncom_aircraft3);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Loading 'packet' of aircraft
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//
//010207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Packet Aircraft::loading_packet(int num_missile,int num_aircraft,int num_target)
{
	int index(0);
	int i(0);

/*	string id;
	char object[4];
	static int c_count=0;
	
	c_count++;
	if(c_count==(num_aircraft+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="t"+string(object);
*/
	//building 'data' array of module-variables
	for(i=0;i<flat3_com_count;i++)
	{
		index=flat3_com_ind[i];
		com_aircraft3[i]=flat3[index];
	}
	for(int j=0;j<aircraft_com_count;j++)
	{
		index=aircraft_com_ind[j];
		com_aircraft3[i+j]=aircraft[index];
	}
	//refreshing the packet
	packet.set_data(com_aircraft3);
	packet.set_ndata(ncom_aircraft3);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Composing documention on file 'doc.asc'
//Listing 'aircraft' module-variable arrays
//
//010214 Created by Peter H Zipfel
//020911 Added module-variable error flagging, PZi
///////////////////////////////////////////////////////////////////////////////
void Aircraft::document(ostream &fdoc,char *title,Document *doc_aircraft3)
{
	int i(0);

	fdoc<<"\n\n                                       Aircraft Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	for(i=0;i<NAIRCRAFT;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(aircraft[i].get_name(),aircraft[j].get_name())&&strcmp(aircraft[i].get_name(),"empty"))
				aircraft[i].put_error(name_error_code);
		}				
		if(!strcmp(aircraft[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in aircraft[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(aircraft[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in aircraft[] array, see 'doc.asc' ***\n"; 

		fdoc<<aircraft[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(aircraft[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<aircraft[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<aircraft[i].get_name();
		}
		fdoc.width(54);fdoc<<aircraft[i].get_def();
		fdoc.width(13);fdoc<<aircraft[i].get_mod();
		fdoc.width(10);fdoc<<aircraft[i].get_role();
		fdoc<<aircraft[i].get_out();
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
	//building doc_aircraft3[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NAIRCRAFT;i++){
		if(strcmp(aircraft[i].get_name(),"empty")){
			doc_aircraft3[counter].put_doc_offset(counter);
			doc_aircraft3[counter].put_name(aircraft[i].get_name());
			doc_aircraft3[counter].put_type(aircraft[i].get_type());
			doc_aircraft3[counter].put_def(aircraft[i].get_def());
			doc_aircraft3[counter].put_mod(aircraft[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NFLAT3;i++){
		if(strcmp(flat3[i].get_name(),"empty")){
			doc_aircraft3[counter].put_doc_offset(counter);
			doc_aircraft3[counter].put_name(flat3[i].get_name());
			doc_aircraft3[counter].put_type(flat3[i].get_type());
			doc_aircraft3[counter].put_def(flat3[i].get_def());
			doc_aircraft3[counter].put_mod(flat3[i].get_mod());
			counter++;
		}
	}
}
