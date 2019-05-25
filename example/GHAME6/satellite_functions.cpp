///////////////////////////////////////////////////////////////////////////////
//FILE: 'satellite_functions.cpp'
//
// Contains utilitiy functions for the 'Satellite' class:
//		array sizing
//		writing data to output
//
//030404 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each vehicle object
//Assigning initial values to module-variables of 'round3' and 'satellite' arrays
//Reading aero and propulsion decks
//
//The first vehicle 'SAT3' of 'input.asc' reads until the first 'END' after
//'SAT3'. The second vehicle object reads untile the second 'END', etc until
//the data for all vehicle objects are read
//
//Output:	satellite3_name ('Satellite' data member)
//			round3[] data values (Round3 data member)
//			satellite[] data values ('Satellite' data member)
//			markov_list[] ('Markov' list of variables)
//
//Limitation: real and integer variables only (could be expanded to vectors)			 
//
//001230 Created by Peter Zipfel
//010930 Added reading of random variables, PZi
//030404 Adapted to HYPER6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Satellite::vehicle_data(fstream &input,int nmonte)
{

	char line_clear[CHARL];
	char read[CHARN];	//name of variable read from input.asc
	char *buff;			//name of module-variable
	double data(0);		//data of module-variable 
	char *integer;
	int int_data(0);
	int file_ptr=NULL;
	int e=0;
	char *watchpoint=NULL;
	double value(0);
	double first(0);
	double second(0);
	char name1[CHARN];

	input.getline(satellite3_name,CHARL,'\n');

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
			int i(0);
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
			for(i=0;i<NSAT;i++)
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
				int kk(0);
				for(kk=0;kk<NROUND3;kk++)
				{
					buff=round3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						round3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NSAT;kk++)
				{
					buff=satellite[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						satellite[kk].gets(value);
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
				int kk(0);
				for(kk=0;kk<NROUND3;kk++)
				{
					buff=round3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						round3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NSAT;kk++)
				{
					buff=satellite[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						satellite[kk].gets(value);
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
				int kk(0);
				for(kk=0;kk<NROUND3;kk++)
				{
					buff=round3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						round3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NSAT;kk++)
				{
					buff=satellite[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						satellite[kk].gets(value);
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
				int kk(0);
				for(kk=0;kk<NROUND3;kk++)
				{
					buff=round3[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						round3[kk].gets(value);
						input.getline(line_clear,CHARL,'\n');
					}				
				}
				for(kk=0;kk<NSAT;kk++)
				{
					buff=satellite[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						satellite[kk].gets(value);
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
//Determining dimensions of arrays: 'com_satellite'
// 
//Out to Satellite:: round3_com_count, satellite_com_count, ncom_satellite3 		 ,
//
//010207 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
void Satellite::sizing_arrays()
{
	char *key4="com";
	int i(0);

	round3_com_count=0;
	satellite_com_count=0;

	//counting in 'round3' array
 	for(i=0;i<NROUND3;i++)
	{
		if(strstr(round3[i].get_out(),key4))
			round3_com_count++;
	}

	//counting in 'satellite' array
 	for(i=0;i<NSAT;i++)
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
//Output: Satellite::round3_com_ind[], satellite_com_ind[] 
//
//010210 Created by Peter Zipfel
//030404 Adapted to HYPER6 simulation, PZi
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
	for(i=0;i<NSAT;i++)
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
//010401 Created by Peter H Zipfel
//030404 Adapted to HYPER6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
Packet Satellite::loading_packet_init(int num_hyper,int num_satellite,int num_radar)
{
	string id;
	char object[4];
	static int c_count=0;
	int index(0);
	int i(0);
	
	c_count++;
	if(c_count==(num_satellite+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="t"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_satellite3[i]=round3[index];
	}
	for(int j=0;j<satellite_com_count;j++)
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
//uses C-code 'sprintf' function to convert 'int' to 'char'
//
//010207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Packet Satellite::loading_packet(int num_hyper,int num_satellite,int num_radar)
{
	int index(0);
	int i(0);

	//building 'data' array of module-variables
	for(i=0;i<round3_com_count;i++)
	{
		index=round3_com_ind[i];
		com_satellite3[i]=round3[index];
	}
	for(int j=0;j<satellite_com_count;j++)
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
//030404 Adapted to HYPER6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Satellite::document(ostream &fdoc,char *title,Document *doc_satellite3)
{

	fdoc<<"\n\n                                       Satellite Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	int i(0);
	for(i=0;i<NSAT;i++)
	{
		for(int j=0;j<i;j++){
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
	fdoc<<"\n\n                                       Round3 Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	for(i=0;i<NROUND3;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(round3[i].get_name(),round3[j].get_name())&&strcmp(round3[i].get_name(),"empty"))
				round3[i].put_error(name_error_code);
		}				
		if(!strcmp(round3[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in round3[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(round3[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in round3[] array, see 'doc.asc' ***\n"; 
	  
		fdoc<<round3[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(round3[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<round3[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<round3[i].get_name();
		}
		fdoc.width(54);fdoc<<round3[i].get_def();
		fdoc.width(13);fdoc<<round3[i].get_mod();
		fdoc.width(10);fdoc<<round3[i].get_role();
		fdoc<<round3[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}
	//building doc_satellite3[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NSAT;i++){
		if(strcmp(satellite[i].get_name(),"empty")){
			doc_satellite3[counter].put_doc_offset(counter);
			doc_satellite3[counter].put_name(satellite[i].get_name());
			doc_satellite3[counter].put_type(satellite[i].get_type());
			doc_satellite3[counter].put_def(satellite[i].get_def());
			doc_satellite3[counter].put_mod(satellite[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NROUND3;i++){
		if(strcmp(round3[i].get_name(),"empty")){
			doc_satellite3[counter].put_doc_offset(counter);
			doc_satellite3[counter].put_name(round3[i].get_name());
			doc_satellite3[counter].put_type(round3[i].get_type());
			doc_satellite3[counter].put_def(round3[i].get_def());
			doc_satellite3[counter].put_mod(round3[i].get_mod());
			counter++;
		}
	}

}
