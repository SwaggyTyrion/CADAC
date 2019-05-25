///////////////////////////////////////////////////////////////////////////////
//FILE: 'global_header.hpp'
//
//Global structures and classes: 'Module', 'Variable', 'File', 'Event', 'Packet'
// with inline member function definitions
//
//001206 Created by Peter Zipfel
//030404 Adapted to HYPER6 simulation, PZi
///////////////////////////////////////////////////////////////////////////////

#define _CRT_SECURE_NO_DEPRECATE
#ifndef global_header__HPP
#define global_header__HPP

#include <fstream>
#include <string>		
#include "utility_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Structure 'Module'
//
//Provides the structure for the modules of the simulation
//
//010107 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
struct Module
{
	string name;
	string definition;
	string initialization;
	string execution;
	string termination;
};

///////////////////////////////////////////////////////////////////////////////
//Class 'Variable'
//Establishing module-variables as type Variable
//Provides the class for the variables used in modules
//
//001125 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

class Variable 
{
private:
	char name[CHARN]; //label of variable
	char type[CHARN]; //type of variable 'int'; default is real
	double rval;	  //real value, double
	int ival;         //integer value, int
	Matrix VEC;		  //vector 
	Matrix MAT;		  //matrix 
	char def[CHARL];  //definition and units
	char mod[CHARN];  //module where variable is calculated
	char role[CHARN]; //role that variable pays: 'data', 'state', 'diag', 'out'
	char out[CHARN];  //output for: 'scrn', 'plot', 'com'
	char error[2];	  //error code '*' = SAME LOCATION multiple, overwritten definitions
					  //           'A' = SAME NAME assigned to multiple locations 
public:
	Variable()
	{
		VEC.dimension(3,1);MAT.dimension(3,3);
		strcpy(name,"empty");
		error[0]=' ';error[1]='\0';
		int dum=1;
	}; 
	~Variable(){};
//////////////////////////// Protopypes ///////////////////////////////////////
void init(char *na,double rv,char *de,char *mo,char *ro,char *ou);
void init(char *na,char *ty,int iv,char *de,char *mo,char *ro,char *ou);
void init(char *na,double v1,double v2,double v3,char *de,char *mo,char *ro,char *ou);
void init(char *na,double v11,double v12,double v13,double v21,double v22,double v23,
					double v31,double v32,double v33,char *de,char *mo,char *ro,char *ou);

////////////////////////// Inline Functions ///////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'name' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_name(){return name;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'type' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_type(){return type;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining value (of type 'double') from module-variable array to local variable
	//Example: double thrust_com=hyper[13].real();
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double real(){return rval;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining value (of type 'int') from module-variable array to local variable
	//Example: int mprop=hyper[10].integer();
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int integer(){return ival;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining vector value (of type 'Matrix') from module-variable array to local variable
	//Example: Matrix FSPV=round6[200].vec();
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Matrix vec(){return VEC;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining matrix value (of type 'Matrix') from module-variable array to local variable
	//Example: Matrix TGV=round6[22].mat();
	//
	//001226 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Matrix mat(){return MAT;}
	
	///////////////////////////////////////////////////////////////////////////
	//Four-times overloaded function gets()
	//Loads module-variable onto module-variable array
	//Overloaded for real, integer and vector variables
	//Example: hyper[10].gets(mprop);
	//
	//001128 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double gets(double rv)
	{
		rval=rv;
		return rval;
	}
	
	double gets(int iv)
	{
		ival=iv;
		return ival;
	} 
	Matrix gets_vec(Matrix VE)
	{
		VEC=VE;
		return VEC;
	} 

	Matrix gets_mat(Matrix MA)
	{
		MAT=MA;
		return MAT;
	} 

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'def' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_def(){return def;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'mod' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_mod(){return mod;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'role' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_role(){return role;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'out' from module-variable array 
	//
	//001213 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_out(){return out;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'error' code from module-variable array 
	//
	//020909 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_error(){return error;}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'error' code into module-variable  
	//Error code must be single caracter
	//020911 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_error(char *error_code){strcpy(error,error_code);}
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Class 'Event'
//
//Provides the 'Event' class declaration
//
//010119 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Event
{
private:
	Variable *watch_variable_ptr;	//pointer to variable to be watched
	double watch_value;			//numerical value for comparison (integers will be converted later)
	char event_operator;		// < , > , =  three options of relational opertors	
	int round6_indices[NVAR];	//new variables to be read from 'round6[]'
	double round6_values[NVAR];	//new values to be given to variables 
	int round6_size;				//size (number) of variables from round6 array
	int hyper_indices[NVAR];	//new variables to be read from 'hyper[]'
	double hyper_values[NVAR];	//new values to be given to variables
	int hyper_size;				//size (number) of variables from hyper array
public:
	Event(){};
	~Event(){};

	///////////////////////////////////////////////////////////////////////////
	//Setting watch variable 
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_variable(Variable *variable)
	{
		watch_variable_ptr=variable;
	}

	///////////////////////////////////////////////////////////////////////////
	//Setting watch value
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_value(double value){watch_value=value;}

	///////////////////////////////////////////////////////////////////////////
	//Setting event criterion (relational operator)
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_operator(char criterion){event_operator=criterion;}

	///////////////////////////////////////////////////////////////////////////
	//Setting index array of 'round6' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_round6_index(int element,int index){round6_indices[element]=index;}

	///////////////////////////////////////////////////////////////////////////
	//Setting values of new 'round6' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_round6_value(int element,double value){round6_values[element]=value;}

	///////////////////////////////////////////////////////////////////////////
	//Setting size of new 'round6' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_round6_size(int size){round6_size=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting index array of 'hyper' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_hyper_index(int element,int index){hyper_indices[element]=index;}

	///////////////////////////////////////////////////////////////////////////
	//Setting values of 'hyper' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_hyper_value(int element,double value){hyper_values[element]=value;}

	///////////////////////////////////////////////////////////////////////////
	//Setting size of new 'hyper' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_hyper_size(int size){hyper_size=size;}

	///////////////////////////////////////////////////////////////////////////
	//Getting watch variable
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Variable *get_variable(){return watch_variable_ptr;}

	///////////////////////////////////////////////////////////////////////////
	//Getting value of watch variable
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double get_value(){return watch_value;}

	///////////////////////////////////////////////////////////////////////////
	//Getting operator of event criterion
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char get_operator(){return event_operator;}

	///////////////////////////////////////////////////////////////////////////
	//Getting index array of 'round6' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int *get_round6_indices(){return round6_indices;}

	///////////////////////////////////////////////////////////////////////////
	//Getting values of new 'round6' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double *get_round6_values(){return round6_values;}

	///////////////////////////////////////////////////////////////////////////
	//Getting size of new 'round6' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_round6_size(){return round6_size;}

	///////////////////////////////////////////////////////////////////////////
	//Getting index array of 'hyper' module-variable array that are used for event
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int *get_hyper_indices(){return hyper_indices;}

	///////////////////////////////////////////////////////////////////////////
	//Getting values of new 'hyper' event variables
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double *get_hyper_values(){return hyper_values;}

	///////////////////////////////////////////////////////////////////////////
	//Getting size of new 'hyper' event variable array
	//
	//010123 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_hyper_size(){return hyper_size;}
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Class 'Packet'
//Provides the 'Packet' class declaration
//Packets are data clusters; one for each vehicle object, used in 'combus'
//
//010206 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Packet
{
private:
	string id;			//identification of vehicle object
	int status;			//alive=1, dead=0. hit=-1 (satellites only), 
	int ndata;			//number of module-variables in data array
	Variable *data;		//array of module-variables identified by "com" 
public:
	Packet(){};
	~Packet(){};
	///////////////////////////////////////////////////////////////////////////
	//Setting packet 'id'
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_id(string identification){id=identification;}

	///////////////////////////////////////////////////////////////////////////
	//Setting packet 'status'
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_status(int vehicle_status){status=vehicle_status;}

	///////////////////////////////////////////////////////////////////////////
	//Setting number of module variables in data
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_ndata(int vehicle_ndata){ndata=vehicle_ndata;}

	///////////////////////////////////////////////////////////////////////////
	//Setting packet 'data'
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_data(Variable *vehicle_d){data=vehicle_d;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'id' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	string get_id(){return id;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'status' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_status(){return status;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'ndata' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_ndata(){return ndata;}

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'data' from packet 
	//
	//010207 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	Variable *get_data(){return data;}
};

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Class 'Markov'
//
//Provides the 'Markov' class declaration
//
//010924 Created by Peter Zipfel
//020723 Included 'saved' private data member, PZi
///////////////////////////////////////////////////////////////////////////////
class Markov
{
private:
	int round6_index;	//markov variable index of 'round6[]'
	int vehicle_index;	//markov variable index of vehicle array ('hyper[]','satellite[]','radar[]')
	double sigma;		//markov sigma value 
	double bcor;		//markov beta correlation value
	double saved;		//markov previous value
	bool status;		//markov noise status: on(true)/off(false)
public:
	Markov(){};
	~Markov(){};

	///////////////////////////////////////////////////////////////////////////
	//Setting index of markov variable of 'round6[]'
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_markov_round6_index(int index){round6_index=index;}

	///////////////////////////////////////////////////////////////////////////
	//Setting index of markov variable index of vehicle array ''hyper[]','satellite[]','radar[]'
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_markov_vehicle_index(int index){vehicle_index=index;}

	///////////////////////////////////////////////////////////////////////////
	//Setting markov sigma value
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_markov_sigma(double sig){sigma=sig;}

	///////////////////////////////////////////////////////////////////////////
	//Setting markov correlation value
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_markov_bcor(double bc){bcor=bc;}

	///////////////////////////////////////////////////////////////////////////
	//Setting markov saved value
	//
	//020723 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_markov_saved(double svd){saved=svd;}

	///////////////////////////////////////////////////////////////////////////
	//Setting markov variable status
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_markov_status(bool st){status=st;}

	///////////////////////////////////////////////////////////////////////////
	//Getting index of markov variable of 'round6[]'
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_markov_round6_index(){return round6_index;}

	///////////////////////////////////////////////////////////////////////////
	//Getting index of markov variable index of vehicle array 'hyper[]','satellite[]','radar[]'
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_markov_vehicle_index(){return vehicle_index;}

	///////////////////////////////////////////////////////////////////////////
	//Getting markov sigma value
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double get_markov_sigma(){return sigma;}

	///////////////////////////////////////////////////////////////////////////
	//Getting markov correlation value
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double get_markov_bcor(){return bcor;}

	///////////////////////////////////////////////////////////////////////////
	//Getting markov saved value
	//
	//020723 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	double get_markov_saved(){return saved;}

	///////////////////////////////////////////////////////////////////////////
	//Getting markov variable status
	//
	//010924 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	bool get_markov_status(){return status;}
};

///////////////////////////////////////////////////////////////////////////////
//Class 'Document'
//stores a subset of module-variable for documentation
//
//020913 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////

class Document 
{
private:
	int doc_offset;	  //marks the offset of the last entry
	char name[CHARN]; //label of variable
	char type[CHARN]; //type of variable 'int'; default is real
	char def[CHARL];  //definition and units
	char mod[CHARN];  //module where variable is calculated		
public:
	Document()
	{
		strcpy(name,"end_array");
	}
	~Document(){};

	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'doc_offset' 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_doc_offset(){return doc_offset;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'name' from module-variable 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_name(){return name;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'type' from module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_type(){return type;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'def' from module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_def(){return def;}
	
	///////////////////////////////////////////////////////////////////////////
	//Obtaining 'mod' from module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	char *get_mod(){return mod;}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'doc_offset' 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_doc_offset(int mark){doc_offset=mark;}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'name' of module-variable 
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_name(char *na){strcpy(name,na);}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'type' of module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_type(char *ty){strcpy(type,ty);}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'def' of module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_def(char *de){strcpy(def,de);}
	
	///////////////////////////////////////////////////////////////////////////
	//Putting 'mod' of module-variable  
	//
	//020913 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////
	void put_mod(char *mo){strcpy(mod,mo);}
	
};
///////////////////////////////////////////////////////////////////////////////
//Class 'Table'
//Stores table data
//
//030710 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Table 
{
private:
	string name;	//table name 										
	int 	dim;	//table dimension (1,2, or 3)										
	int		var1_dim;	// variable 1 dimension	
	int		var2_dim;	// variable 2 dimension	
	int		var3_dim;	// variable 3 dimension	
public:
	double *var1_values;  //values of variable 1 
	double *var2_values;  //values of variable 2 
	double *var3_values;  //values of variable 3
	double *data; // table data values packaged in one array

	Table(){}
	virtual ~Table()
	{
		delete var1_values;
		delete var2_values;
		delete var3_values;
		delete data;
	}

	///////////////////////////////////////////////////////////////////////////
	//Getting dimension of table
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_dim(){return dim;}

	///////////////////////////////////////////////////////////////////////////
	//Getting name of table
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	string get_name(){return name;}

	///////////////////////////////////////////////////////////////////////////
	//Getting 1. independent variable dimension
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_var1_dim(){return var1_dim;}

	///////////////////////////////////////////////////////////////////////////
	//Getting 2. independent variable dimension
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_var2_dim(){return var2_dim;}

	///////////////////////////////////////////////////////////////////////////
	//Getting 3. independent variable dimension
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_var3_dim(){return var3_dim;}
	
	///////////////////////////////////////////////////////////////////////////
	//Setting dimension of table
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_dim(int table_dim){dim=table_dim;}

	///////////////////////////////////////////////////////////////////////////
	//Setting name of table
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_name(string tbl_name){name=tbl_name;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 1. independent variable dimension
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var1_dim(int size){var1_dim=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 2. independent variable dimension
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var2_dim(int size){var2_dim=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 3. independent variable dimension
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var3_dim(int size){var3_dim=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 1. independent variable values
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var1_value(int offset,double value){
		var1_values[offset]=value;	
	}
	///////////////////////////////////////////////////////////////////////////
	//Setting 2. independent variable values
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var2_value(int offset,double value){
		var2_values[offset]=value;	
	}
	///////////////////////////////////////////////////////////////////////////
	//Setting 3. independent variable values
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var3_value(int offset,double value){
		var3_values[offset]=value;	
	}
	///////////////////////////////////////////////////////////////////////////
	//Setting tablular data values 
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_data(int offset,double value){
		data[offset]=value;	
	}
};

///////////////////////////////////////////////////////////////////////////////
//Class 'Datadeck'
//provides the 'Datadeck' class declaration for table look-ups
//
//030710 Created by Peter Zipfel
///////////////////////////////////////////////////////////////////////////////
class Datadeck
{
private:
	string title; //title of data deck
	int capacity; //total number of tables
	int tbl_counter; //table counter
	Table **table_ptr; //table_ptr is pointer to a pointer array of type 'Table'

public:

	Datadeck(){}
	virtual ~Datadeck(){ delete [] table_ptr;}

	///////////////////////////////////////////////////////////////////////////////
	//Allocating memory  table deck title 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void alloc_mem(){table_ptr=new Table *[capacity];}		

	///////////////////////////////////////////////////////////////////////////////
	//Setting table deck title 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void set_title(string deck_title){title=deck_title;}

	///////////////////////////////////////////////////////////////////////////////
	//Getting table deck title 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	string get_title(){return title;}

	///////////////////////////////////////////////////////////////////////////////
	//Setting total number of tables 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void set_capacity(int table_numbers){capacity=table_numbers;}

	///////////////////////////////////////////////////////////////////////////////
	//Getting total number of tables 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	int get_capacity(){return capacity;}

	///////////////////////////////////////////////////////////////////////////////
	//Setting table counter 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void set_counter(int count){tbl_counter=count;}

	///////////////////////////////////////////////////////////////////////////////
	//Getting table counter 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	int get_counter(){return tbl_counter;}

	///////////////////////////////////////////////////////////////////////////////
	//Adding a table pointer to the table list 
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void add_table(Table &pt)
	{
		if(tbl_counter<capacity){
			table_ptr[tbl_counter]=&pt;
		}
	}
	///////////////////////////////////////////////////////////////////////////////
	//Overloaded operator [] returns a 'Table' pointer
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	Table * operator[](int slot)
	{
		if(slot>=0 && slot<capacity)
			return table_ptr[slot];
		else
		{
			cout<<"*** Bad pointer value of table deck: "<<slot<<'\n';
			return 0;
		}
	}

	///////////////////////////////////////////////////////////////////////////////
	//Getting table pointer 
	//030717 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	Table * get_tbl(int slot){return table_ptr[slot];}

	///////////////////////////////////////////////////////////////////////////////
	//Single independent variable look-up
	//constant extrapolation at the upper end, slope extrapolation at the lower end
	//
	//030717 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	double look_up(string name,double value1);

	///////////////////////////////////////////////////////////////////////////////
	//Two independent variables look-up
	// Constant extrapolation at the upper end, slope extrapolation at the lower end
	//
	//030717 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	double look_up(string name,double value1,double value2);

	///////////////////////////////////////////////////////////////////////////////
	//Three independent variables look-up
	//constant extrapolation at the upper end, slope extrapolation at the lower end
	//
	//030723 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	double look_up(string name,double value1,double value2,double value3);

	///////////////////////////////////////////////////////////////////////////////
	//Table index finder
	//This is a binary search method it is O(lgN)
	//* Returns array locater of the discrete_variable just below variable
	//* Keeps max or min array locater if variable is outside those max or min  
	//
	//010628 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	int find_index(int max,double value,double *list);

	///////////////////////////////////////////////////////////////////////////////
	//Linear one-dimensional interpolation
	// Constant extrapolation beyond max values of X1
	// Slope extrapolation beyond min values of X1
	//
	//030717 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	double interpolate(int ind,int ind2,int slot,double val);

	///////////////////////////////////////////////////////////////////////////////
	//Linear, two-dimensional interpolation
	// Constant extrapolation beyond max values of X1 and X2
	// Slope extrapolation beyond min values of X1 and X2
	//
	//030718 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	double interpolate(int ind10,int ind11,int ind20,int ind21,int slot,double value1,
						double value2);																

	///////////////////////////////////////////////////////////////////////////////
	//Linear, three-dimensional interpolation
	//Constant extrapolation beyond max values of X1, X2 and X3
	//Slope extrapolation beyond min values of X1, X2 and X3
	//
	//030723 Created by Peter Zipfel
	///////////////////////////////////////////////////////////////////////////////
	double interpolate(int ind10,int ind11,int ind20,int ind21,int ind30,int ind31,
								 int slot,double value1,double value2,double value3);
																					
};
#endif
