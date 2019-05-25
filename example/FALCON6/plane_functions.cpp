///////////////////////////////////////////////////////////////////////////////
//FILE: 'plane_functions.cpp'
//
// Contains utilitiy functions for the 'Plane' class:
//		array sizing
//		writing banners to output
//		writing data to output
//
//030627 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Determining dimensions of arrays: 'plane6', 'scrn_plane6', 'plot_plane6'
//and 'com_plane6'
// 
//Out to Plane:: nplane6, nscrn_plane6, nplot_plane6, ncom_plane6,
//	flat6_scrn_count, plane_scrn_count, flat6_plot_count, plane_plot_count 		 ,
//
//001212 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::sizing_arrays()
{
	const char *key1="empty";
	char *key2="scrn";
	char *key3="plot";
	char *key4="com";
	int flat6_full_count=0;
	int plane_full_count=0;
	int i(0);

	//initialize 'Flat6' and 'Plane' member variables
	flat6_scrn_count=0;
	plane_scrn_count=0;
	flat6_plot_count=0;
	plane_plot_count=0;
	flat6_com_count=0;
	plane_com_count=0;

	//counting in 'flat6' array
 	for(i=0;i<NFLAT6;i++)
	{
		if(strcmp(flat6[i].get_name(),key1))
			flat6_full_count++;
		if(strstr(flat6[i].get_out(),key2))
			flat6_scrn_count++;
		if(strstr(flat6[i].get_out(),key3))
			flat6_plot_count++;
		if(strstr(flat6[i].get_out(),key4))
			flat6_com_count++;
	}
	//counting in 'plane' array
 	for(i=0;i<NPLANE;i++)
	{
		if(strcmp(plane[i].get_name(),key1))
			plane_full_count++;
		if(strstr(plane[i].get_out(),key2))
			plane_scrn_count++;
		if(strstr(plane[i].get_out(),key3))
			plane_plot_count++;
		if(strstr(plane[i].get_out(),key4))
			plane_com_count++;
	}
	//output to Plane::protected
	nplane6=flat6_full_count+plane_full_count;
	nscrn_plane6=flat6_scrn_count+plane_scrn_count;
	nplot_plane6=flat6_plot_count+plane_plot_count;
	ncom_plane6=flat6_com_count+plane_com_count;
}

///////////////////////////////////////////////////////////////////////////////
//Building 'plane6' module-array by eliminating empty slots in 'flat6' and 'plane'
//and merging the two arrays 
//
//Output: Plane::plane6[] 
//
//001212 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::vehicle_array()
{
	const char *test="empty";
	int i(0);

	//load nonempty slots from flat6 array into plane6 array
	int k=0;
	for(i=0;i<NFLAT6;i++)
	{
		if(strcmp(flat6[i].get_name(),test))
		{
			plane6[k]=flat6[i];
			k++;
		}
	}	
	//load nonempty slots from plane array into plane6 array	
	int m=0;
	for(i=0;i<NPLANE;i++)
	{
		if(strcmp(plane[i].get_name(),test))
		{
			plane6[k+m]=plane[i];
			m++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Building 'scrn_plane6' module-array from 'plane6' array by keying on the word 'scrn'
//
//Output: Plane::scrn_plane6[] 
//
//001214 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::scrn_array()
{
	int k=0;
	char *buff;
	char *key="scrn";

	for(int i=0;i<nplane6;i++)
	{
		buff=plane6[i].get_out();
		if(strstr(buff,key))
		{
			scrn_plane6[k]=plane6[i];
			k++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Building 'plot_plane6' module-array from 'plane6' array by keying on the word 'plot'
//
//Output: Plane::plot_plane6[] 
//
//001214 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////
void Plane::plot_array()
{
	int k=0;
	char *buff;
	char *key="plot";

	for(int i=0;i<nplane6;i++)
	{
		buff=plane6[i].get_out();
		if(strstr(buff,key))
		{
			plot_plane6[k]=plane6[i];
			k++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//Writing out banner of labels to screen
//
//First label is time-since-launch-of-vehicle 'time', always at flat6[0]
//eight accross, unlimited down
//data field will be 15 spaces, total width 120 spaces
//labels longer than 14 characters will be truncated
//Accomodates 3x1 vectors
//
//010106 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::scrn_banner()
{
	char *buff1;
	char buff2[15];
	int label_length=14;
	int k=0;

	cout<<"\n Vehicle: PLANE6 "<<'\n'; 

	for(int i=0;i<nscrn_plane6;i++)
	{
		cout.setf(ios::left);

		buff1=scrn_plane6[i].get_name();

		//truncating if more than 14 characters
		strncpy(buff2,buff1,label_length);
		buff2[14]=0;

		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int i=1;i<4;i++)
			{				
				cout.width(strlen(buff2));
				cout<<buff2;cout.width(15-strlen(buff2));cout<<i;
				k++;
				if(k>7){k=0;cout<<'\n';}
			}
		}
		else
		{
			cout.width(15);
			cout<<buff2;
			k++;
			if(k>7){k=0;cout<<'\n';}
		}
	}
	cout<<"\n\n";
}
///////////////////////////////////////////////////////////////////////////////
//Writing out banner of labels to 'tabout.asc'
//
//First label is time-since-launch-of-vehicle 'time', always at flat6[0]
//eight accross, unlimited down
//data field will be 15 spaces, total width 120 spaces
//labels longer than 14 characters will be truncated
//Accomodates 3x1 vectors
//
//010114 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::tabout_banner(ofstream &ftabout,char *title)
{
	char *buff1;
	char buff2[15];
	int label_length=14;
	int k=0;

	ftabout<<"\n"<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<"\n";
	ftabout<<"\n Vehicle: PLANE6 "<<'\n';
	

	for(int i=0;i<nscrn_plane6;i++)
	{
		ftabout.setf(ios::left);

		buff1=scrn_plane6[i].get_name();

		//truncating if more than 14 characters
		strncpy(buff2,buff1,label_length);
		buff2[14]=0;

		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int i=1;i<4;i++)
			{				
				ftabout.width(strlen(buff2));
				ftabout<<buff2;ftabout.width(15-strlen(buff2));ftabout<<i;
				k++;
				if(k>7){k=0;ftabout<<'\n';}
			}
		}
		else
		{
			ftabout.width(15);
			ftabout<<buff2;
			k++;
			if(k>7){k=0;ftabout<<'\n';}
		}
	}
	ftabout<<"\n\n";
}
///////////////////////////////////////////////////////////////////////////////
//Reading input data from input file 'input.asc' for each plane vehicle object
//Assigning initial values to module-variables of 'flat6' and 'plane' arrays
//Reading aero and propulsion decks
//
//The first plane object 'input.asc' reads until the first 'END' after
//'PLANE6'. The second plane object reads untile the second 'END', etc until
//the data for all plane objects are read
//
//Output:	plane6_name ('Plane' data member)
//			flat6[] data values (Flat6 data member)
//			plane[] data values ('Plane' data member)
//			event_ptr_list[] ('Event' data members)
//
//Limitation: only real and integer variables can be read 			 
//
//001230 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
//030727 New table look-up scheme, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::vehicle_data(fstream &input)
{
	char line_clear[CHARL];
	char read[CHARN];	//name of variable read from input.asc
	char *buff=NULL;			//name of module-variable
	double data(0);		//data of module-variable 
	char file_name[CHARN];	//name of data-deck file
	char *integer=NULL;
	int int_data(0);
	char buff2[CHARN];
	int file_ptr=NULL;
	char name[CHARN];
	Variable *variable=NULL;
	char oper;
	double value(0);
	int e(0);
	char *watchpoint=NULL;
	int m(0),k(0),i(0);

	input.getline(plane6_name,CHARL,'\n');
	//reading data until 'END' is encountered
	do
	{
		//reading variable data into module-variable arrays after discarding comments
		input>>read;
		if(ispunct(read[0]))
		{
			input.getline(line_clear,CHARL,'\n');
		}
		else
		{
			for(i=0;i<NFLAT6;i++)
			{
				buff=flat6[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=flat6[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading integer value
						int_data=(int)data;
						flat6[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						flat6[i].gets(data);
						input.getline(line_clear,CHARL,'\n');
					}
				}				
			}
			for(i=0;i<NPLANE;i++)
			{
				buff=plane[i].get_name();
				if(!strcmp(buff,read)) 
				{
					input>>data;
					//checking for integers
					integer=plane[i].get_type();
					if(!strcmp(integer,"int"))
					{
						//loading interger value
						int_data=(int)data;
						plane[i].gets(int_data);
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						//loading real value
						plane[i].gets(data);
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

			//reading events into 'Event' pointer array 'event_ptr_list' of size NEVENT
			if(!strcmp(read,"IF"))
			{
				event_total++;
				//reading name of watch variable
				input>>name;
				//determining it's module-variable
				for(m=0;m<NFLAT6;m++)
				{
					buff=flat6[m].get_name();
					if(!strcmp(buff,name)) 
						variable=&flat6[m];
				}
				for(m=0;m<NPLANE;m++)
				{
					buff=plane[m].get_name();
					if(!strcmp(buff,name)) 
						variable=&plane[m];
				}

				//reading other criteria
				input>>oper;
				input>>value;
				event_ptr_list[e]->set_variable(variable);
				event_ptr_list[e]->set_value(value);
				event_ptr_list[e]->set_operator(oper);
				input.getline(line_clear,CHARL,'\n');

				//acquiring indices and values of event variables
				int el1=0; //element # in flat6[]
				int el2=0; //element # in plane[]
				event_ptr_list[e]->set_flat6_size(el1);
				event_ptr_list[e]->set_plane_size(el2);
				do
				{
					input>>buff2;
					if(ispunct(read[0]))
					{
						input.getline(line_clear,CHARL,'\n');
					}
					else
					{
						for(k=0;k<NFLAT6;k++)
						{
							buff=flat6[k].get_name();
							if(!strcmp(buff,buff2))
							{
								event_ptr_list[e]->set_flat6_index(el1,k);
								input>>data;
								event_ptr_list[e]->set_flat6_value(el1,data);
								input.getline(line_clear,CHARL,'\n');
								el1++;
								event_ptr_list[e]->set_flat6_size(el1);
								if(el1==NVAR)
								{
									cerr<<"*** Error: Check EVENTS (size of NVAR) *** \n";
									exit(1);
								}									
							}				
						}
						for(k=0;k<NPLANE;k++)
						{
							buff=plane[k].get_name();
							if(!strcmp(buff,buff2))
							{
								event_ptr_list[e]->set_plane_index(el2,k);
								input>>data;
								event_ptr_list[e]->set_plane_value(el2,data);
								input.getline(line_clear,CHARL,'\n');
								el2++;
								event_ptr_list[e]->set_plane_size(el2);
								if(el2==NVAR)
								{
									cerr<<"*** Error: Check EVENTS (size of NVAR) *** \n";
									exit(1);
								}									
							}				
						}
					}
				}while(strcmp(buff2,"ENDIF"));
				//increment event counter
				e++;	
			} //end of loading events

		} //end of reading non-comment lines
	}while(strcmp(read,"END")); //reached 'END' of vehicle object

	//flushing the line after END and starting new line
	input.getline(line_clear,CHARL,'\n'); 

	//diagnostic: file pointer
	file_ptr=int(input.tellg());
}
///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'flat6[]' and plane[] variables 
//that are output to screen  
//
//Output: Plane::flat6_scrn_ind[], plane_crn_ind[] 
//
//001213 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::scrn_index_arrays()
{
	const char *test="scrn";
	int i(0);

	int k=0;
	for(i=0;i<NFLAT6;i++)
	{
		if(strstr(flat6[i].get_out(),test))
		{
			flat6_scrn_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NPLANE;i++)
	{
		if(strstr(plane[i].get_out(),test))
		{
			plane_scrn_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Writing data to screen
//
//First label is time-since-launch-of-vehicle 'time', always at flat6[0]
//Accomodates real, integers (printed as real) and 3x1 vectors 
//eight accross, unlimited down
//data field 15 spaces, total width 120 spaces
//
//010112 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::scrn_data()
{

	int index;
	char *integer;
	char *vector;
	Matrix VEC(3,1);
	int k(0),i(0);
	
	cout<<plane6_name<<'\n';
	cout.setf(ios::left);

	//writing to screen the variables from the 'Flat6' class
	for(i=0;i<flat6_scrn_count;i++)
	{
		index=flat6_scrn_ind[i];
		//checking for integers
		integer=flat6[index].get_type();
		vector=flat6[index].get_name();
		if(!strcmp(integer,"int"))
		{
			cout.width(15);
			cout<<flat6[index].integer();			
			k++; if(k>7){k=0;cout<<'\n';}
		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=flat6[index].vec();

			cout.width(15);
			cout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;cout<<'\n';}
		}
		else
		{
			//real variables
			cout.width(15);
			cout<<flat6[index].real();
			k++; if(k>7){k=0;cout<<'\n';}
		}
	}
	//writing to screen the variables from the 'Plane' class
	for(i=0;i<plane_scrn_count;i++)
	{
		index=plane_scrn_ind[i];
		//checking for integers
		integer=plane[index].get_type();
		vector=plane[index].get_name();
		if(!strcmp(integer,"int"))
		{
			cout.width(15);
			cout<<plane[index].integer();			
			k++; if(k>7){k=0;cout<<'\n';}

		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=plane[index].vec();
			cout.width(15);
			cout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;cout<<'\n';}
			cout.width(15);
			cout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;cout<<'\n';}
		}
		else
		{
			//real variables
			cout.width(15);
			cout<<plane[index].real();
			k++; if(k>7){k=0;cout<<'\n';}
		}
	}
	cout<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Writing data to 'tabout.asc'
//
//First label is time-since-launch-of-vehicle 'time', must be at flat6[0]
//Accomodates real, integers (printed as real) and 3x1 vectors 
//eight accross, unlimited down
//data field 15 spaces, total width 120 spaces
//
//010114 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::tabout_data(ofstream &ftabout)
{

	int index;
	char *integer;
	char *vector;
	Matrix VEC(3,1);
	int k(0),i(0);
	
	ftabout<<plane6_name<<'\n';
	ftabout.setf(ios::left);

	//writing to 'tabout.asc' the variables from the 'Flat6' class
	for(i=0;i<flat6_scrn_count;i++)
	{
		index=flat6_scrn_ind[i];
		//checking for integers
		integer=flat6[index].get_type();
		vector=flat6[index].get_name();
		if(!strcmp(integer,"int"))
		{
			ftabout.width(15);
			ftabout<<flat6[index].integer();			
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=flat6[index].vec();

			ftabout.width(15);
			ftabout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
		else
		{
			//real variables
			ftabout.width(15);
			ftabout<<flat6[index].real(); 
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
	}
	//writing to 'tabout.asc' the variables from the 'Plane' class
	for(i=0;i<plane_scrn_count;i++)
	{
		index=plane_scrn_ind[i];
		//checking for integers
		integer=plane[index].get_type();
		vector=plane[index].get_name();
		if(!strcmp(integer,"int"))
		{
			ftabout.width(15);
			ftabout<<plane[index].integer();			
			k++; if(k>7){k=0;ftabout<<'\n';}

		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=plane[index].vec();
			ftabout.width(15);
			ftabout<<VEC.get_loc(0,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(1,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
			ftabout.width(15);
			ftabout<<VEC.get_loc(2,0);
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
		else
		{
			//real variables
			ftabout.width(15);
			ftabout<<plane[index].real();
			k++; if(k>7){k=0;ftabout<<'\n';}
		}
	}
	ftabout<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Writing out banner of labels to 'ploti.asc', i=1,2,3...
//
//First label is time-since-launch-of-vehicle 'time', always at flat6[0]
//five accross, unlimited down
//data field width 16 spaces, total width 80 spaces
//labels longer than 8 characters will be truncated
//Accomodates 3x1 vectors
//
//010115 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::plot_banner(ofstream &fplot,char *title)
{
	char *buff1;
	char buff2[15];
	int label_length=8;
	int k(0);
	int m(0);
	int i(0);

	fplot<<"1"<<title<<" '"<<plane6_name<<" ' "<< __DATE__ <<" "<< __TIME__ <<"\n";
	
	//determining the number vector variables
	for(i=0;i<nplot_plane6;i++)
	{
		buff1=plot_plane6[i].get_name();
		if(isupper(buff1[0])) m++;
	}
	//increase number of variables by vector components
	int nvariables=nplot_plane6+2*m;
	
	fplot<<"  0  0 " <<nvariables<<"\n"; 

	//writing banner to plot file 'ploti.asc'
	for(i=0;i<nplot_plane6;i++)
	{
		fplot.setf(ios::left);

		buff1=plot_plane6[i].get_name();

		//truncating if more than 8 characters
		strncpy(buff2,buff1,label_length);
		buff2[8]=0;

		//Vectors are recognized by upper case character 
		if(isupper(buff2[0]))
		{
			for(int j=1;j<4;j++)
			{				
				fplot.width(strlen(buff2));
				fplot<<buff2;fplot.width(16-strlen(buff2));fplot<<j;
				k++;
				if(k>4){k=0;fplot<<'\n';}
			}
		}
		else
		{
			fplot.width(16);
			fplot<<buff2;
			k++;
			if(k>4){k=0;fplot<<'\n';}
		}
	}
	if((nvariables%5))fplot<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'flat6[]' and plane[] variables that are  
//output to 'ploti.asc'
//
//Output: Plane::flat6_plot_ind[], plane_plot_ind[] 
//
//001213 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::plot_index_arrays()
{
	const char *test="plot";
	int k(0),i(0);

	for(i=0;i<NFLAT6;i++)
	{
		if(strstr(flat6[i].get_out(),test))
		{
			flat6_plot_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NPLANE;i++)
	{
		if(strstr(plane[i].get_out(),test))
		{
			plane_plot_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Writing data to 'ploti.asc', i=1,2,3...
//
//Accomodates real, integers (printed as real) and 3x1 vectors 
//five accross, unlimited down
//data field 16 spaces, total width 80 spaces
//
//010116 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::plot_data(ofstream &fplot,bool merge)
{

	int index;
	char *integer;
	char *vector;
	Matrix VEC(3,1);
	int k(0),i(0);
	
	fplot.setf(ios::left);

	//writing to 'ploti.asc' the variables from the 'Flat6' class
	for(i=0;i<flat6_plot_count;i++)
	{
		index=flat6_plot_ind[i];
		//checking for integers
		integer=flat6[index].get_type();
		vector=flat6[index].get_name();
		if(!strcmp(integer,"int"))
		{
			//casting integer to real variable
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<(double) flat6[index].integer();			
			k++;
		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=flat6[index].vec();

			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(0,0);
			k++; 
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(1,0);
			k++;
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(2,0);
			k++;
		}
		else
		{
			//real variables
			if(merge&&(index==0))
				//for merging files, time at last entry must be '-1'
			{
				fplot.width(16);
				fplot<<"-1.0";
				k++;
			}
			else
			{
				if(k>4){k=0;fplot<<'\n';}
				fplot.width(16);
				fplot<<flat6[index].real(); 
				k++;
			}
		}
	}
	//writing to 'ploti.asc' the variables from the 'Plane' class
	for(i=0;i<plane_plot_count;i++)
	{
		index=plane_plot_ind[i];
		//checking for integers
		integer=plane[index].get_type();
		vector=plane[index].get_name();
		if(!strcmp(integer,"int"))
		{
			//casting integer to real variable
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<(double) plane[index].integer();			
			k++;

		}
		//checking vor vectors
		else if(isupper(vector[0]))
		{
			VEC=plane[index].vec();
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(0,0);
			k++;
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(1,0);
			k++;
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<VEC.get_loc(2,0);
			k++;
		}
		else
		{
			//real variables
			if(k>4){k=0;fplot<<'\n';}
			fplot.width(16);
			fplot<<plane[index].real();
			k++;
		}
	}
	fplot<<"\n";
}
///////////////////////////////////////////////////////////////////////////////
//Watching for and executing events
// 
//Max number of events set by global constant NEVENT
//Max number of variables in any event set by global constant NVAR
//Each event has to be surrounded in 'input.asc' by 'IF' and 'ENDIF'
//Event criteria and variables can be 'double' or 'int'
//New variable-values are subsituted before calling modules
//
//010125 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::event(char *options)
{
	Variable *watch_variable_ptr;
	double current_value=0;
	int current_value_int=0;
	double crit_value=0;
	int crit_value_int=0;
	char oper;
	char *intest=NULL;
	int i(0);

	//initializing event flag ('Plane' member)
	event_epoch=false;

	//returning if no events occur for this vehicle object
	//or if all events have already occurred
	if(!event_total)return;
	
	//getting watch variable's current value and critical value
	watch_variable_ptr=event_ptr_list[nevent]->get_variable();
	crit_value=event_ptr_list[nevent]->get_value();

	//testing for integer
	intest=watch_variable_ptr->get_type();
	if(!strcmp(intest,"int"))
	{
		current_value_int=watch_variable_ptr->integer();
		crit_value_int=(int)crit_value;
	}
	else
		current_value=watch_variable_ptr->real();
	//getting relational operator
	oper=event_ptr_list[nevent]->get_operator();

	//checking if event occurred
	if(oper=='<')
	{
		if(!strcmp(intest,"int"))
		{
			if(current_value_int<crit_value_int)
				event_epoch=true;
		}
		else
		{
			if(current_value<crit_value)
				event_epoch=true;
		}
	}
	else if(oper=='=')
	{
		if(!strcmp(intest,"int"))
		{
			if(current_value_int==crit_value_int)
				event_epoch=true;
		}
		else
		{
			if(current_value==crit_value)event_epoch=true;
		}
	}
	else if(oper=='>')
	{
		if(!strcmp(intest,"int"))
		{
			if(current_value_int>crit_value_int)
				event_epoch=true;
		}
		else
		{
			if(current_value>crit_value)
				event_epoch=true;
		}
	}

	//loading new variables
	if(event_epoch)
	{
		int *flat6_index_list;
		double *flat6_value_list;
		int flat6_size;
		int *plane_index_list;
		double *plane_value_list;
		int plane_size;
		int index;
		char *integer;
		double value;
		int value_int;

		flat6_index_list=event_ptr_list[nevent]->get_flat6_indices();
		flat6_value_list=event_ptr_list[nevent]->get_flat6_values();
		flat6_size=event_ptr_list[nevent]->get_flat6_size();

		plane_index_list=event_ptr_list[nevent]->get_plane_indices();
		plane_value_list=event_ptr_list[nevent]->get_plane_values();
		plane_size=event_ptr_list[nevent]->get_plane_size();

		for(i=0;i<flat6_size;i++)
		{
			index=flat6_index_list[i];
			value=flat6_value_list[i];

			integer=flat6[index].get_type();
			if(!strcmp(integer,"int"))
			{
				value_int=(int)value;
				flat6[index].gets(value_int);
			}
				flat6[index].gets(value); // new value assigned
		}
		for(i=0;i<plane_size;i++)
		{
			index=plane_index_list[i];
			value=plane_value_list[i];

			integer=plane[index].get_type();
			if(!strcmp(integer,"int"))
			{
				value_int=(int)value;
				plane[index].gets(value_int);
			}
				plane[index].gets(value); // new value assigned
		}
				
		//writing event message to console
		double time=flat6[0].real();
		char *name=watch_variable_ptr->get_name();
		if(strstr(options,"y_events"))
		{
			cout<<" *** Event #"<<nevent+1<<'\t'<<plane6_name<<'\t'<<"time = "<<time
				<<"\tsec;  criteria:  "<<name<<" "<<oper<<" "<<crit_value<<"\t***\n";
		}
		//increment event number
		nevent++;
		//reset 'event_total' to zero after last event has occured
		if(nevent==event_total)event_total=0;	
	}
}
///////////////////////////////////////////////////////////////////////////////
//Composing documention on file 'doc.asc'
//Listing 'plane' and 'flat6' module-variable arrays
//
//010126 Created by Peter Zipfel
//011129 Adapted to PLANE6 simulation, PZi
//020911 Added module-variable error flagging, PZi
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::document(ostream &fdoc,char *title,Document *doc_plane6)
{
	fdoc<<"*********************************************************************************************************************\n";
	fdoc<<"********************************************** PLANE6 **************************************************************\n";
	fdoc<<"*********************************************************************************************************************\n";
	fdoc<<"\n"<<"*** "<<title<<"   "<< __DATE__ <<" "<< __TIME__ <<" ***\n\n";
//	fdoc<<"01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456\n";
	fdoc<<"\n\n                                       Plane Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                    DEFINITION                       |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";

	char name_error_code[]="A";
	int i(0),j(0);
	for(i=0;i<NPLANE;i++)
	{
		for(j=0;j<i;j++){
			if(!strcmp(plane[i].get_name(),plane[j].get_name())&&strcmp(plane[i].get_name(),"empty"))
				plane[i].put_error(name_error_code);
		}				
		if(!strcmp(plane[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in plane[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(plane[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in plane[] array, see 'doc.asc' ***\n"; 

				fdoc<<plane[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(plane[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<plane[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<plane[i].get_name();
		}
		fdoc.width(54);fdoc<<plane[i].get_def();
		fdoc.width(13);fdoc<<plane[i].get_mod();
		fdoc.width(10);fdoc<<plane[i].get_role();
		fdoc<<plane[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}

	fdoc<<"\n\n                                       Flat6 Module-Variable Array \n\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	fdoc<<"|LOC|        NAME       |                   DEFINITION                        |   MODULE   | PURPOSE |    OUTPUT    |\n";
	fdoc<<"---------------------------------------------------------------------------------------------------------------------\n";
	  
	for(i=0;i<NFLAT6;i++)
	{
		for(int j=0;j<i;j++){
			if(!strcmp(flat6[i].get_name(),flat6[j].get_name())&&strcmp(flat6[i].get_name(),"empty"))
				flat6[i].put_error(name_error_code);
		}				
		if(!strcmp(flat6[i].get_error(),"A")) cout<<" *** Error code 'A': duplicate name in flat6[] array, see 'doc.asc' ***\n"; 
		if(!strcmp(flat6[i].get_error(),"*")) cout<<" *** Error code '*': duplicate location in flat6[] array, see 'doc.asc' ***\n"; 
		
		fdoc<<flat6[i].get_error();
		fdoc.setf(ios::left);
		fdoc.width(4);fdoc<<i;
		if(!strcmp(flat6[i].get_type(),"int"))
		{
			fdoc.width(15);fdoc<<flat6[i].get_name();
			fdoc.width(5);fdoc<<" int ";
		}
		else
		{
			fdoc.width(20);fdoc<<flat6[i].get_name();
		}
		fdoc.width(54);fdoc<<flat6[i].get_def();
		fdoc.width(13);fdoc<<flat6[i].get_mod();
		fdoc.width(10);fdoc<<flat6[i].get_role();
		fdoc<<flat6[i].get_out();
		fdoc<<"\n";
		if(!((i+1)%10))fdoc<<"----------------------------------------------------------------------------------------------------------------------\n";			
	}

	//building doc_plane6[] for documenting 'input.asc' and eliminating 'empty' slots
	int counter=0;
	for(i=0;i<NPLANE;i++){
		if(strcmp(plane[i].get_name(),"empty")){
			doc_plane6[counter].put_doc_offset(counter);
			doc_plane6[counter].put_name(plane[i].get_name());
			doc_plane6[counter].put_type(plane[i].get_type());
			doc_plane6[counter].put_def(plane[i].get_def());
			doc_plane6[counter].put_mod(plane[i].get_mod());
			counter++;
		}
	}
	for(i=0;i<NFLAT6;i++){
		if(strcmp(flat6[i].get_name(),"empty")){
			doc_plane6[counter].put_doc_offset(counter);
			doc_plane6[counter].put_name(flat6[i].get_name());
			doc_plane6[counter].put_type(flat6[i].get_type());
			doc_plane6[counter].put_def(flat6[i].get_def());
			doc_plane6[counter].put_mod(flat6[i].get_mod());
			counter++;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//Building index array of those 'flat6[]' and plane[] variables 
//that are output to 'combus' 'data'  
//
//Output: Plane::flat6_com_ind[], plane_com_ind[] 
//
//010210 Created by Peter Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

void Plane::com_index_arrays()
{
	const char *test="com";

	int k(0),i(0);
	for(i=0;i<NFLAT6;i++)
	{
		if(strstr(flat6[i].get_out(),test))
		{
			flat6_com_ind[k]=i;
			k++;
		}
	}
	int l=0;
	for(i=0;i<NPLANE;i++)
	{
		if(strstr(plane[i].get_out(),test))
		{
			plane_com_ind[l]=i;
			l++;
		}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Initializing loading 'packet' with 'PLANE6' data
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//differs from 'loading_packet' only by initializing 'status=1'
//
//Output by 'return packet'
//
//010401 Created by Peter H Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Packet Plane::loading_packet_init(int num_plane)
{
	string id;
	char object[4];
	static int c_count=0;
	int index;
	int i(0),j(0);
	
	c_count++;
	if(c_count==(num_plane+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="m"+string(object);

	//building 'data' array of module-variables
	for(i=0;i<flat6_com_count;i++)
	{
		index=flat6_com_ind[i];
		com_plane6[i]=flat6[index]; 
	}
	for(j=0;j<plane_com_count;j++)
	{
		index=plane_com_ind[j];
		com_plane6[i+j]=plane[index];
	}
	//refreshing the packet
	packet.set_id(id);
	packet.set_status(1);
	packet.set_data(com_plane6);
	packet.set_ndata(ncom_plane6);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Loading 'packet' with 'PLANE6' data
//
//uses C-code 'sprintf' function to convert 'int' to 'char'
//
//010206 Created by Peter H Zipfel
//030627 Adapted to PLANE6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

Packet Plane::loading_packet(int num_plane)
{
	int index;
	int i(0),j(0);

/*	string id;
	char object[4];
	static int c_count=0;
	
	c_count++;
	if(c_count==(num_plane+1))c_count=1;
	sprintf(object,"%i",c_count);
	id="m"+string(object);
*/
	//building 'data' array of module-variables
	for(i=0;i<flat6_com_count;i++)
	{
		index=flat6_com_ind[i];
		com_plane6[i]=flat6[index];
	}
	for(j=0;j<plane_com_count;j++)
	{
		index=plane_com_ind[j];
		com_plane6[i+j]=plane[index];
	}
	//refreshing the packet
//	packet.set_id(id);
	packet.set_data(com_plane6);
	packet.set_ndata(ncom_plane6);

	return packet;
}
///////////////////////////////////////////////////////////////////////////////
//Reading tables from table decks
//
//Supports 1, 2, 3 dim tables stored seperately in data decks
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
///////////////////////////////////////////////////////////////////////////////
void Plane::read_tables(char *file_name,Datadeck &datatable)
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

	//opening data-deck file stream
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

	} //EOF reached of data-deck

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
		else if(var_dim[2]>num_rows) num_rows=var_dim[2];

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

	/*/////////////////////// DIAGNOSTICS //////////////////////////////////////
	//Diagnostic display of tables on console
	int num_tables=datatable.get_capacity();
	string deck_title=datatable.get_title();
	cout<<"\n ********************* Data Deck *********************\n";
	cout<<"TITLE "<<deck_title<<"\n\n";

	for(int ti=0;ti<num_tables;ti++){
		int tbl_dim=datatable[ti]->get_dim();

		if(tbl_dim==1){
			string tbl_name=datatable[ti]->get_name();
			cout<<tbl_dim<<"DIM "<<tbl_name<<'\n';
			int var1_dim=datatable[ti]->get_var1_dim();
			cout<<"NX1 "<<var1_dim<<'\n';
			for(int tii=0;tii<var1_dim;tii++){
				cout<<datatable[ti]->var1_values[tii]<<"\t"<<datatable[ti]->data[tii]<<endl;
			}
		cout<<'\n';
		}//end of 1-dim tables

		if(tbl_dim==2){
			string tbl_name=datatable[ti]->get_name();
			cout<<tbl_dim<<"DIM "<<tbl_name<<'\n';
			int var1_dim=datatable[ti]->get_var1_dim();
			int var2_dim=datatable[ti]->get_var2_dim();
			cout<<"NX1 "<<var1_dim<<"   NX2 "<<var2_dim<<'\n';

			//find maximum number of rows
			int num_rows=var1_dim;
			if(num_rows<var2_dim)
				num_rows=var2_dim;

			for(int tii=0;tii<num_rows;tii++){
				if(tii<var1_dim)
					cout<<datatable[ti]->var1_values[tii]<<"\t";
				else 
					cout<<"*\t";
				if(tii<var2_dim) 
					cout<<datatable[ti]->var2_values[tii]<<"\t";
				else 
					cout<<"*\t";

				if(tii<var1_dim){
					for(int tk=0;tk<var2_dim;tk++)
						cout<<datatable[ti]->data[tk+tii*var2_dim]<<'\t';
					cout<<endl;
				}
				else{
					for(int tk=0;tk<var2_dim;tk++)
						cout<<"*\t";
					cout<<endl;
				}

			}
		cout<<'\n';
		}//end of 2-dim tables

		if(tbl_dim==3){
			string tbl_name=datatable[ti]->get_name();
			cout<<tbl_dim<<"DIM "<<tbl_name<<'\n';
			int var1_dim=datatable[ti]->get_var1_dim();
			int var2_dim=datatable[ti]->get_var2_dim();
			int var3_dim=datatable[ti]->get_var3_dim();
			cout<<"NX1 "<<var1_dim<<"   NX2 "<<var2_dim<<"   NX3 "<<var3_dim<<'\n';

			//find maximum number of rows
			int num_rows=var1_dim;
			if(num_rows<var2_dim)
				num_rows=var2_dim;
			if(num_rows<var3_dim)
				num_rows=var3_dim;

			for(int tii=0;tii<num_rows;tii++){
				if(tii<var1_dim)
					cout<<datatable[ti]->var1_values[tii]<<"\t";
				else 
					cout<<"*\t";
				if(tii<var2_dim) 
					cout<<datatable[ti]->var2_values[tii]<<"\t";
				else 
					cout<<"*\t";
				if(tii<var3_dim) 
					cout<<datatable[ti]->var3_values[tii]<<"\t";
				else 
					cout<<"*\t";

				if(tii<var1_dim){
					for(int tk=0;tk<var2_dim*var3_dim;tk++)
						cout<<datatable[ti]->data[tk+tii*var2_dim*var3_dim]<<'\t';
					cout<<endl;
				}
				else{
					for(int tk=0;tk<var2_dim*var3_dim;tk++)
						cout<<"*\t";
					cout<<endl;
				}
			}
		cout<<'\n';
		}//end of 3-dim tables
	}//end of diagnostic table print-out
	/*//////////////////////////////////////////////////////////////////////////
}