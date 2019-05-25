///////////////////////////////////////////////////////////////////////////////
//FILE: 'satellite_functions.cpp'
// Contains utilitiy functions for the 'Satellite' class:
//		array sizing
//		writing data to output
//
//010810 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each vehicle object
//Assigning initial values to module-variables of 'round3' and 'satellite' arrays
//Reading aero and propulsion decks
//
//The first vehicle 'SATELLITE3' of 'input.asc' reads until the first 'END' after
//'SATELLITE3'. The second vehicle object reads untile the second 'END', etc until
//the data for all vehicle objects are read
//
//Output:	satellite3_name ('Satellite' data member)
//			round3[] data values (Round3 data member)
//			satellite[] data values ('Satellite' data member)
//
//Limitation: real and integer variables only (could be expanded to vectors)			 
//
//010810 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Satellite::vehicle_data(fstream &input)
{

	char line_clear[CHARL];
	char read[CHARN];	//name of variable read from input.asc
	char *buff;			//name of module-variable
	double data;		//data of module-variable 
	char *comment ="//";
	char *integer;
	int int_data;
	int file_ptr=NULL;
	int e(0);
	char *watchpoint=NULL;

	input.getline(satellite3_name,CHARL,'\n');

	//reading data until END is encountered
	do
	{
		//reading variable data into module-variable arrays
		input>>read;
		if(strpbrk(comment,read))
		{
			input.getline(line_clear,CHARL,'\n');
		}
		else
		{
			int i;
			for(i=0;i<NROUND3;i++)
			{
				buff=round3[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=round3[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						round3[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						round3[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			for(i=0;i<NSATELLITE;i++)
			{
				buff=satellite[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=satellite[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						satellite[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						satellite[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}

		} //end of reading non-comment lines
	}while(strcmp(read,"END")); //reached 'END' of first vehicle object

	//diagnostic: file pointer
	file_ptr=int(input.tellg());
}
///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'com_satellite'
// 
//Out to Satellite:: round3_com_count, satellite_com_count, ncom_satellite3 		 ,
//
//010810 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
void Satellite::sizing_arrays()
{
	char *key4="com";

	round3_com_count=0;
	satellite_com_count=0;
	int i(0);

	//counting in 'round3' array
 	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),key4))
			round3_com_count++;
	}

	//counting in 'satellite' array
 	for(i=0;i<NSATELLITE;i++)
	{
		if(strstr(satellite[i].get_out(),key4))
			satellite_com_count++;
	}

	//output to Satellite::protected
	ncom_satellite3=round3_com_count+satellite_com_count;
}

///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'round3[]' and satellite[] variables 
//that are output to 'combus' 'data'  
//
//Out to Satellite::round3_com_ind[], satellite_com_ind[] 
//
//010810 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Satellite::com_index_arrays()
{
	const char *test="com";
	int i(0);

	int k=0;
	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),test))
		{
			round3_com_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NSATELLITE;i++)
	{
		if(strstr(satellite[i].get_out(),test))
		{
			satellite_com_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Initializing loading 'packet' of satellite
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//differs from 'loading_packet' only by initializing 'status=1'
//
//010810 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Packet Satellite::loading_packet_init(int num_cruise,int num_target,int num_satellite)
{
	string id;
	char object[4];
	static int c_count=0;
	int index;
	int i(0);
	int j(0);
	
	c_count++;
	if(c_count==(num_satellite+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="s"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_satellite3[i]=round3[index];
	}
	for(j=0;j<satellite_com_count;j++)
	{
		index=satellite_com_ind[j];
		com_satellite3[i+j]=satellite[index];
	}
	//refreshing the packet
	packet.set_id(id);
	packet.set_status(1);
	packet.set_data(com_satellite3);
	packet.set_ndata(ncom_satellite3);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Loading 'packet' of satellite
//
//uses C-code 'sprintf' function to convert int to char
//
//010810 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Packet Satellite::loading_packet(int num_cruise,int num_target,int num_satellite)
{
	int index(0);
	int i(0);
	int j(0);

/*	string id;
	char object[4];
	static int c_count=0;
	
	c_count++;
	if(c_count==(num_satellite+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="t"+string(object);
*/
	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_satellite3[i]=round3[index];
	}
	for(j=0;j<satellite_com_count;j++)
	{
		index=satellite_com_ind[j];
		com_satellite3[i+j]=satellite[index];
	}
	//refreshing the packet
	packet.set_data(com_satellite3);
	packet.set_ndata(ncom_satellite3);

	return packet;
}

///////////////////////////////////////////////////////////////////////////////
//Composing documention on file 'doc.asc'
//Listing 'satellite' module-variable arrays
//
//010214 Created by Peter Zipfel
//020911 Added module-variable error flagging, PZi
//060929 Modification to accomodate C++8, PZi
///////////////////////////////////////////////////////////////////////////////

void Satellite::document(ostream &fdoc,char *title,Document *doc_satellite3)
{
	int i(0);
	int j(0);

	fdoc<<"\n\n                                       Satellite Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	for(i=0;i<NSATELLITE;i++)
	{
		for(j=0;j<i;j++){
			if(!strcmp(satellite[i].get_name(),satellite[j].get_name())&&strcmp(satellite[i].get_name(),"empty"))
				satellite[i].put_error(name_error_code);
		}				
		if(!strcmp(satellite[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in satellite[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(satellite[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in satellite[] array, see 'doc.asc' ***\n"; 

		fdoc<<satellite[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(satellite[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<satellite[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<satellite[i].get_name();
		}
		fdoc.width(54);fdoc<<satellite[i].get_def();
		fdoc.width(13);fdoc<<satellite[i].get_mod();
		fdoc.width(10);fdoc<<satellite[i].get_role();
		fdoc<<satellite[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}
	//building doc_satellite3[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NSATELLITE;i++){
		if(strcmp(satellite[i].get_name(),"empty")){
			doc_satellite3[counter].put_doc_offset(counter);
			doc_satellite3[counter].put_name(satellite[i].get_name());

//z060929			doc_satellite3[counter].put_type(satellite[i].get_type());
			char *dum;
			dum=satellite[i].get_type();
			if(*dum!=-51)
				doc_satellite3[counter].put_type(dum);
//z060929-end

			doc_satellite3[counter].put_def(satellite[i].get_def());
			doc_satellite3[counter].put_mod(satellite[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NROUND3;i++){
		if(strcmp(round3[i].get_name(),"empty")){
			doc_satellite3[counter].put_doc_offset(counter);
			doc_satellite3[counter].put_name(round3[i].get_name());

//z060929			doc_satellite3[counter].put_type(round3[i].get_type());
			char *dum;
			dum=round3[i].get_type();
			if(*dum!=-51)
				doc_satellite3[counter].put_type(dum);
//z060929-end

			doc_satellite3[counter].put_def(round3[i].get_def());
			doc_satellite3[counter].put_mod(round3[i].get_mod());
			counter++;
		}
	}
}
