///////////////////////////////////////////////////////////////////////////////
//FILE: 'target_functions.cpp'
// Contains utilitiy functions for the 'Target' class:
//		array sizing
//		writing data to output
//
//010206 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each vehicle object
//Assigning initial values to module-variables of 'round3' and 'target' arrays
//Reading aero and propulsion decks
//
//The first vehicle 'TARGET3' of 'input.asc' reads until the first 'END' after
// 'TARGET3'. The second vehicle object reads untile the second 'END', etc until
// the data for all vehicle objects are read
//
//Output:	target3_name ('Target' data member)
//			round3[] data values (Round3 data member)
//			target[] data values ('Target' data member)
//
//Limitation: real and integer variables only (could be expanded to vectors)			 
//
//001230 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::vehicle_data(fstream &input)
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
	int i(0);

	input.getline(target3_name,CHARL,'\n');

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
		} //end of reading non-comment lines
	}while(strcmp(read,"END")); //reached 'END' of first vehicle object

	//flushing the line after END and starting new line
	input.getline(line_clear,CHARL,'\n'); 

	//diagnostic: file pointer
	file_ptr=int(input.tellg());
}
///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'com_target'
// 
//Out to Target:: round3_com_count, target_com_count, ncom_target3 		 ,
//
//010207 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
void Target::sizing_arrays()
{
	char *key4="com";

	round3_com_count=0;
	target_com_count=0;
	int i(0);

	//counting in 'round3' array
 	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),key4))
			round3_com_count++;
	}
	//counting in 'target' array
 	for(i=0;i<NTARGET;i++)
	{
		if(strstr(target[i].get_out(),key4))
			target_com_count++;
	}
	//output to Target::protected
	ncom_target3=round3_com_count+target_com_count;
}

///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'round3[]' and target[] variables 
// that are output to 'combus' 'data'  
//
//Out to Target::round3_com_ind[], target_com_ind[] 
//
//010210 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

void Target::com_index_arrays()
{
	const char *test="com";
	int k(0);
	int i(0);

	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),test))
		{
			round3_com_ind[k]=i;
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
//Uses C-code 'sprintf' function to convert int to char
//Differs from 'loading_packet' only by initializing 'status=1'
//
//010401 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Packet Target::loading_packet_init(int num_cruise,int num_target,int num_satellite)
{
	string id;
	char object[4];
	static int c_count=0;
	int index;
	int i(0);
	int j(0);
	
	c_count++;
	if(c_count==(num_target+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="t"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_target3[i]=round3[index];
	}
	for(j=0;j<target_com_count;j++)
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
//Uses C-code 'sprintf' function to convert int to char
//
//010207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

Packet Target::loading_packet(int num_cruise,int num_target,int num_satellite)
{
	int index(0);
	int i(0);
	int j(0);

	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_target3[i]=round3[index];
	}
	for(j=0;j<target_com_count;j++)
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
//010214 Created by Peter Zipfel
//020911 Added module-variable error flagging, PZi
//060929 Modification to accomodate C++8, PZi
///////////////////////////////////////////////////////////////////////////////

void Target::document(ostream &fdoc,char *title,Document *doc_target3)
{
	int i(0);
	int j(0);

	fdoc<<"\n\n                                       Target Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	for(i=0;i<NTARGET;i++)
	{
		for(j=0;j<i;j++){
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
	//building doc_target3[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NTARGET;i++){
		if(strcmp(target[i].get_name(),"empty")){
			doc_target3[counter].put_doc_offset(counter);
			doc_target3[counter].put_name(target[i].get_name());

//z060929			doc_target3[counter].put_type(target[i].get_type());

			char *dum;
			dum=target[i].get_type();
			if(*dum!=-51)
				doc_target3[counter].put_type(dum);
//z060929-end

			doc_target3[counter].put_def(target[i].get_def());
			doc_target3[counter].put_mod(target[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NROUND3;i++){
		if(strcmp(round3[i].get_name(),"empty")){
			doc_target3[counter].put_doc_offset(counter);
			doc_target3[counter].put_name(round3[i].get_name());

//z060929			doc_target3[counter].put_type(round3[i].get_type());

			char *dum;
			dum=round3[i].get_type();
			if(*dum!=-51)
				doc_target3[counter].put_type(dum);
//z060929-end

			doc_target3[counter].put_def(round3[i].get_def());
			doc_target3[counter].put_mod(round3[i].get_mod());
			counter++;
		}
	}
}
