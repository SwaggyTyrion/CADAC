///////////////////////////////////////////////////////////////////////////////
//FILE: 'radar_functions.cpp'
//
// Contains utilitiy functions for the 'Radar' class:
//		array sizing
//		writing data to output
//
//040505 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each vehicle object
//Assigning initial values to module-variables of 'ground0' and 'radar' arrays
//Reading aero and propulsion decks
//
//The first vehicle 'RADAR0' of 'input.asc' reads until the first 'END' after
//'RADAR0'. The second vehicle object reads untile the second 'END', etc until
//the data for all vehicle objects are read
//
//Output:	radar0_name ('Radar' data member)
//			ground0[] data values (Ground0 data member)
//			radar[] data values ('Radar' data member)
//			markov_list[] ('Markov' list of variables)
//
//Limitation: real and integer variables only (could be expanded to vectors)			 
//
//001230 Created by Peter Zipfel
//010930 Added reading of random variables, PZi
//040505 Adopted for Radar, PZi
///////////////////////////////////////////////////////////////////////////////
void Radar::vehicle_data(fstream &input,int nmonte)
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
			int i(0);
			for(i=0;i<NGROUND0;i++)
			{
				buff=ground0[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=ground0[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						ground0[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						ground0[i].gets(data);
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
				for(kk=0;kk<NGROUND0;kk++)
				{
					buff=ground0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						ground0[kk].gets(value);
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
				int kk(0);
				for(kk=0;kk<NGROUND0;kk++)
				{
					buff=ground0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						ground0[kk].gets(value);
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
				int kk(0);
				for(kk=0;kk<NGROUND0;kk++)
				{
					buff=ground0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						ground0[kk].gets(value);
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
				int kk(0);
				for(kk=0;kk<NGROUND0;kk++)
				{
					buff=ground0[kk].get_name();
					if(!strcmp(buff,name1)) 
					{
						ground0[kk].gets(value);
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

		} //end of reading non-comment lines
	}while(strcmp(read,"END")); //reached 'END' of first vehicle object

	//flushing the line after END and starting new line
	input.getline(line_clear,CHARL,'\n'); 

	//diagnostic: file pointer
	file_ptr=int(input.tellg());
}
///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'com_radar'
// 
//Out to Radar:: ground0_com_count, radar_com_count, ncom_radar0 		 ,
//
//010207 Created by Peter Zipfel
//040505 Adopted for Radar, PZi
///////////////////////////////////////////////////////////////////////////////
void Radar::sizing_arrays()
{
	char *key4="com";
	int i(0);

	ground0_com_count=0;
	radar_com_count=0;

	//counting in 'ground0' array
 	for(i=0;i<NGROUND0;i++)
	{
		if(strstr(ground0[i].get_out(),key4))
			ground0_com_count++;
	}

	//counting in 'radar' array
 	for(i=0;i<NRADAR;i++)
	{
		if(strstr(radar[i].get_out(),key4))
			radar_com_count++;
	}

	//output to Radar::protected
	ncom_radar0=ground0_com_count+radar_com_count;
}

///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'ground0[]' and radar[] variables 
//that are output to 'combus' 'data'  
//
//Output: Radar::ground0_com_ind[], radar_com_ind[] 
//
//010210 Created by Peter Zipfel
//040505 Adopted for Radar, PZi
///////////////////////////////////////////////////////////////////////////////
void Radar::com_index_arrays()
{
	const char *test="com";
	int i(0);

	int k=0;
	for(i=0;i<NGROUND0;i++)
	{
		if(strstr(ground0[i].get_out(),test))
		{
			ground0_com_ind[k]=i;
			k++;
		}
	}

	int l=0;
	for( i=0;i<NRADAR;i++)
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
//040505 Adopted for Radar, PZi
///////////////////////////////////////////////////////////////////////////////
Packet Radar::loading_packet_init(int num_hyper,int num_satellite,int num_radar)
{
	string id;
	char object[4];
	static int c_count=0;
	int index(0);
	int i(0);
	
	c_count++;
	if(c_count==(num_radar+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="r"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<ground0_com_count;i++)
	{
		index=ground0_com_ind[i];
		com_radar0[i]=ground0[index];
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
//040505 Adopted for Radar, PZi
///////////////////////////////////////////////////////////////////////////////
Packet Radar::loading_packet(int num_hyper,int num_satellite,int num_radar)
{
	int index(0);
	int i(0);

	//building 'data' array of module-variables
	for(i=0;i<ground0_com_count;i++)
	{
		index=ground0_com_ind[i];
		com_radar0[i]=ground0[index];
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
//010214 Created by Peter Zipfel
//020911 Added module-variable error flagging, PZi
//040505 Adopted for Radar, PZi
///////////////////////////////////////////////////////////////////////////////
void Radar::document(ostream &fdoc,char *title,Document *doc_radar0)
{

	fdoc<<"\n\n                                       Radar Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	int i(0);
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
	fdoc<<endl;
	fdoc<<"\n\n                                       Ground0 Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	for(i=0;i<NGROUND0;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(ground0[i].get_name(),ground0[j].get_name())&&strcmp(ground0[i].get_name(),"empty"))
				ground0[i].put_error(name_error_code);
		}				
		if(!strcmp(ground0[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in ground0[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(ground0[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in ground0[] array, see 'doc.asc' ***\n"; 
	  
		fdoc<<ground0[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(ground0[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<ground0[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<ground0[i].get_name();
		}
		fdoc.width(54);fdoc<<ground0[i].get_def();
		fdoc.width(13);fdoc<<ground0[i].get_mod();
		fdoc.width(10);fdoc<<ground0[i].get_role();
		fdoc<<ground0[i].get_out();
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
	for(i=0;i<NGROUND0;i++){
		if(strcmp(ground0[i].get_name(),"empty")){
			doc_radar0[counter].put_doc_offset(counter);
			doc_radar0[counter].put_name(ground0[i].get_name());
			doc_radar0[counter].put_type(ground0[i].get_type());
			doc_radar0[counter].put_def(ground0[i].get_def());
			doc_radar0[counter].put_mod(ground0[i].get_mod());
			counter++;
		}
	}
}
