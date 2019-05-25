///////////////////////////////////////////////////////////////////////////////
//FILE: 'utility_header.hpp'
// Declares the classes 'Matrix', 'Table','Datadeck',    
//
//Module utility functions:
//	mat2tr
//	mat3tr
//	cadtge
//	cadtei
//	cadsph
//	cadtbv
//	cadine
//	sign
//	angle
//Table look-up, classes 'Table' and 'Datadeck'
//Integration
//US76 Atmosphere
//
//010628 Created by Peter H Zipfel
//020829 Dynamically dimensioned utilities, PZi
//030319 Added US76 atmosphere, PZi
//030424 General matrix integration, PZi
//030519 Overloaded operator [] for vector, PZi
//030725 Removed all reference to table look-up structure, PZi
//060510 Updated from F16C for CRUISE, PZi
///////////////////////////////////////////////////////////////////////////////

//preventing warnings in MS C++8 for not using security enhanced CRT functions 
#define _CRT_SECURE_NO_DEPRECATE

#ifndef utility_header__HPP
#define utility_header__HPP

#include <iostream>
#include <cmath>
#include "global_constants.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
/////////////////////// 'Matrix' member functions /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//One dimensional and two dimensional arrays of any dimension of type 'double'
//Class 'Matrix' 
// dynamically allocated matrix size
// pointer arithmetic
//
//020826 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
 
class Matrix
{
private:
	//number of rows
	int num_row;
	//number of columns
	int num_col;
	//total number of elements
	int num_elem;
	//pointer to array
	double *pbody;

public:
	//default constructors
	Matrix();
	//overloaded constructor
	Matrix(int row_size,int col_size);
	//copy constructor
	Matrix(const Matrix &MAT);
	//deconstructor
	~Matrix();

	//printing matrix array to console
	void print();

	//Returns absolute value of a vector VEC
	//Example: value=VEC.absolute();
	double absolute();

	//Returns the adjoint
	//Example: BMAT = AMAT.adjoint();
	Matrix adjoint();

	//Assigns a value to a matrix element (offset!)
	//Example: MAT.assign_loc(r,c,val); ((r+1)th-row, (c+1)th-col)
	void assign_loc(const int &r, const int &c, const double &val);

	//Builds a 3x1 vector from three parameters
	//Example: VEC.build_vec3(v1,v2,v3)
	Matrix & build_vec3(const double &v1,const double &v2,const double &v3);

	//Builds a 3x3 matrix from nine paramters arranged in rows
	//Example: MAT.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33)
	Matrix & build_mat33(const double &v11,const double &v12,const double &v13
						 ,const double &v21,const double &v22,const double &v23
					     ,const double &v31,const double &v32,const double &v33);
	
	//Calculates Cartesian vector from polar coordinates
	// |V1|             | cos(elevation)*cos(azimuth)|
	// |V2| = magnitude*|cos(elevation)*sin(azimuth) |
	// |V3|		        |	  -sin(elevation)        |
	//Example: VEC.cart_from_pol(magnitude,azimuth,elevation); 	
	Matrix &cart_from_pol(const double &magnitude,const double &azimuth
						   ,const double &elevation);
						
	//Returns column vector of column # 
	//Example: VEC = MAT.col_vec(2); (2nd column!)
	Matrix col_vec(const int &col);

	//Returns the determinant
	//Example: det = MAT.determinant();
	double determinant();

	//Returns nxn diagonal matrix  from nx1 vector 
	//Example: DIAMAT=VEC.diamat_vec()
	Matrix diamat_vec();

	//Returns nx1 diagonal vector from nxn matrix
	//Example: VEC=MAT.diavec_mat();
	Matrix diavec_mat();

	//Dimensions a matrix of size row x col
	//Example: MAT.dimension(3,3);
	void dimension(int row,int col);

	//Bi-variate ellipse
	//calculating major and minor semi-axes of ellipse and rotation angle 
	//    from the symmetrical pos semi-definite MAT(2x2) matrix
	//major_semi_axis = ELLIPSE.get_loc(0,0);
	//minor_semi_axis = ELLIPSE.get_loc(1,0);
	//angle      = ELLIPSE.get_loc(2,0);
	//
	//Example: ELLIPSE = MAT.ellipse();
	Matrix ellipse();

	//Returns the number of columns of matrix MAT
	//Example: nc = MAT.get_cols();
	int get_cols();

	//Returns offset index given row# and col#
	//Example: i = MAT.get_index(2,3); (2nd row, 3rd coloumn)
	int get_index(const int &row,const int &col);

	//Returns the value at offset-row 'r' offset-col 'c' of MAT
	//Example: value = MAT.get_loc(2,1); (3rd row, 2nd column)
	double get_loc(const int &r,const int &c);

	//Returns the pointer to MAT
	//Example: ptr = MAT.get_pbody();
	double * get_pbody();

	//Returns the number of rows in the matrix
	//Example: nr = MAT.get_rows();
	int get_rows();

	//Builds a square identity matrix of object 'Matrix MAT'
	Matrix &identity();
	
	//Returns the inverse of a square matrix AMAT
	//Example: INVERSE = AMAT.inverse();
	Matrix inverse();

	//Returns 3x3 matrix row-wise from 9x1 vector
	//Example: MAT=VEC.	mat33_vec9();
	Matrix mat33_vec9();
	
	//Forms  matrix MAT with all elements '1.' from object MAT(num_row,num_col)
	//Example: MAT.ones();
	Matrix & ones();

	//Inequality relational operator, returns true or false
	//returns true if elements differ by more than EPS
	//Example: if(AMAT!=BMAT){...};
	bool operator!=(const Matrix &B);

	//Scalar multiplication operator (scalar element by element multiplication)
	//Note: scalar must be the second operand
	//Example: CMAT = AMAT * b;
	Matrix operator*(const double &b);

	//Multiplication operator, returns matrix product 
	//Example: CMAT = AMAT * BMAT;
	Matrix operator*(const Matrix &B);

	//Scalar multiplication assignment operator (scalar element by element multiplication)
	//Example: AMAT *= b; meaning: AMAT = AMAT * b
	Matrix & operator*=(const double &b);

	//Multiplication assignment operator 
	//Example: AMAT *= BMAT; meaning: AMAT = AMAT * BMAT; 
	Matrix & operator*=(const Matrix &B);

	//Scalar Addition operator (scalar added to each element)
	//Note: scalar must be the second operand
	//Example: CMAT = AMAT + b; 
	Matrix operator+(const double &b);

	//Addition operator, returns matrix addition
	//Operands must be conformal
	//Example: CMAT = AMAT + BMAT;
	Matrix operator+(const Matrix &B);

	//Scalar addition assignment operator (scalar added to each element)
	//Example: AMAT += b; meaning: AMAT = AMAT + b 
	Matrix & operator+=(const double &b);
	
	//Matrix addition assignment operator
	//Example: AMAT += BMAT; meaning: AMAT = AMAT + BMAT;
	Matrix & operator+=(const Matrix &B);

	//Scalar subtraction operator (scalar element by element substraction)
	//Note: scalar must be the second operand
	//Example: CMAT = AMAT - b;
	Matrix operator-(const double &b);

	//Subtraction operator, returns matrix subtraction
	//Example: CMAT = AMAT - BMAT;
	Matrix operator-(const Matrix &B);

	//Scalar subtraction assignment operator (scalar subtracted from each element)
	//Example: AMAT -= b; meaning: AMAT = AMAT - b
	Matrix & operator-=(const double &b);

	//Matrix subtraction assignment operator
	//Example: AMAT -= BMAT; meaning: AMAT = AMAT - BMAT;
	Matrix & operator-=(const Matrix &B);

	//Assignment operator (deep copy) 
	//Example: AMAT = BMAT; also: AMAT = BMAT = CMAT;
	Matrix &operator=(const Matrix &B);

	//Equality relational operator, returns true or false
	//returns true if elements differ by less than EPS
	//Example: if(AMAT==BMAT){...};
	bool operator==(const Matrix &B);

	//Returns the component(i) from vector VEC[i] or assigns a value to component VEC[i]
	//Examples: comp_i=VEC[i]; VEC[i]=comp_i;
	double & operator [](const int &r);

	//Scalar product operator (any combination of row or column vectors)  
	//Example: value = AMAT ^ BMAT;  
	double operator^(const Matrix &B);

	//Alternate transpose Aji < Aij
	//(same as 'Matrix trans()') 
	//Example: BMAT = ~AMAT;
	Matrix operator~();

	//Returns polar from cartesian coordinates
	// magnitude = POLAR(0,0) = |V|
	// azimuth   = POLAR(1,0) = atan2(V2,V1)
	// elevation = POLAR(2,0) = atan2(-V3,sqrt(V1^2+V2^2)
	//Example: POLAR = VEC.pol_from_cart();
	Matrix pol_from_cart();

	//Returns row vector of row # 
	//Example: VEC = MAT.row_vec(2); (2nd row!)
	Matrix row_vec(const int &row);

	//Returns the skew-symmetric matrix MAT from a 3-dim vector VEC
	//			| 0 -c  b|		|a|
	//			| c  0 -a| <--	|b|
	//			|-b  a  0|		|c|
	//
	//Example: MAT = VEC.skew_sym();
	Matrix skew_sym();

	//Returns the sub matrix after 'row'  and 'col' have been ommitted 
	//Example: BMAT = AMAT.sub_matrix(1,3); (deleting first row and third column!) 
	Matrix sub_matrix(const int &row, const int &col);

	//Returns the stranspose of a matrix Aji < Aij
	//(same as 'Matrix operator~()' 
	//Example: BMAT = AMAT.trans();
	Matrix trans();

	//Returns unit vector from 3x1 vector
	//Example: UVEC=VEC.univec3();
	Matrix univec3();

	//Returns 9x1 vector from 3x3 matrix 
	//Example: VEC=MAT.vec9_mat33();
	Matrix vec9_mat33();

	//Forms a zero matrix MAT from object MAT(num_row,num_col)
	//Example: MAT.zero();
	Matrix &zero();
};
///////////////////////////////////////////////////////////////////////////////
////////////////// Module utility functions ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//Returns the T.M. of the psivg -> thtvg sequence
Matrix mat2tr(const double &psivg,const double &thtvg);

//Returns the Euler T.M. of the psi->tht->phi sequence
Matrix mat3tr(const double &psi,const double &tht,const double &phi);

//Returns the T.M. of geographic wrt earth coordinates
Matrix cadtge(double lon,double lat);

//Returns the T.M. of earth wrt inertial coordinates 
Matrix cadtei(double simulation_time);

//Returns lon, lat, alt from inertial displacement vector
Matrix cadsph(Matrix SBIE);

//Returns the T.M of body wrt velocity coordintates (3 DoF)
//Suitable for 3 DoF bank-to-turn simulations only
Matrix cadtbv(double phi,double alpha);

//Returns inertial coordinates from longitude, latitude and altitude
Matrix cadine(double lon,double lat,double alt,double time);

//Returns the sign of the function
//Example: value_signed=value*sign(variable) 
int sign(const double &variable);

//Returns the angle between two 3x1 vectors
//Example: theta=angle(VEC1,VEC2);
double angle(Matrix VEC1,Matrix VEC2);

///////////////////////////////////////////////////////////////////////////////
////////// Table look-up and interpolation function declarations //////////////
///////////////////////////////////////////////////////////////////////////////
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
	int dim;	//table dimension (1,2, or 3)										
	int	var1_dim;	// variable 1 dimension	
	int	var2_dim;	// variable 2 dimension	
	int	var3_dim;	// variable 3 dimension	
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
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_dim(){return dim;}

	///////////////////////////////////////////////////////////////////////////
	//Getting name of table
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	string get_name(){return name;}

	///////////////////////////////////////////////////////////////////////////
	//Getting 1. independent variable dimension
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_var1_dim(){return var1_dim;}

	///////////////////////////////////////////////////////////////////////////
	//Getting 2. independent variable dimension
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_var2_dim(){return var2_dim;}

	///////////////////////////////////////////////////////////////////////////
	//Getting 3. independent variable dimension
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	int get_var3_dim(){return var3_dim;}
	
	///////////////////////////////////////////////////////////////////////////
	//Setting dimension of table
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_dim(int table_dim){dim=table_dim;}

	///////////////////////////////////////////////////////////////////////////
	//Setting name of table
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_name(string tbl_name){name=tbl_name;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 1. independent variable dimension
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var1_dim(int size){var1_dim=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 2. independent variable dimension
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var2_dim(int size){var2_dim=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 3. independent variable dimension
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var3_dim(int size){var3_dim=size;}

	///////////////////////////////////////////////////////////////////////////
	//Setting 1. independent variable values
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var1_value(int offset,double value){
		var1_values[offset]=value;	
	}
	///////////////////////////////////////////////////////////////////////////
	//Setting 2. independent variable values
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var2_value(int offset,double value){
		var2_values[offset]=value;	
	}
	///////////////////////////////////////////////////////////////////////////
	//Setting 3. independent variable values
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_var3_value(int offset,double value){
		var3_values[offset]=value;	
	}
	///////////////////////////////////////////////////////////////////////////
	//Setting tablular data values
	//
	//030710 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////
	void set_data(int offset,double value){
		data[offset]=value;	
	}
};

///////////////////////////////////////////////////////////////////////////////
//Class 'Datadeck'
//Provides the 'Datadeck' class declaration for table look-ups
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
	//Allocating memory  for table_ptr array
	//
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void alloc_mem(){table_ptr=new Table *[capacity];}		

	///////////////////////////////////////////////////////////////////////////////
	//Setting table deck title
	//
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void set_title(string deck_title){title=deck_title;}

	///////////////////////////////////////////////////////////////////////////////
	//Getting table deck title
	//
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	string get_title(){return title;}

	///////////////////////////////////////////////////////////////////////////////
	//Setting total number of tables
	//
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void set_capacity(int table_numbers){capacity=table_numbers;}

	///////////////////////////////////////////////////////////////////////////////
	//Getting total number of tables
	//
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	int get_capacity(){return capacity;}

	///////////////////////////////////////////////////////////////////////////////
	//Setting table counter
	//
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	void set_counter(int count){tbl_counter=count;}

	///////////////////////////////////////////////////////////////////////////////
	//Getting table counter
	//
	//030711 Created by Peter H Zipfel
	///////////////////////////////////////////////////////////////////////////////
	int get_counter(){return tbl_counter;}

	///////////////////////////////////////////////////////////////////////////////
	//Adding a table pointer to the table list
	//
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
	//
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
	//
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

///////////////////////////////////////////////////////////////////////////////
////////////////////  Integration functions  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//Integration of scalar state variable
//Modified Euler Midpoint method 
//Example first order lag:
//			phid_new=(phic-phi)/tphi;
//			phi=integrate(phid_new,phid,phi,int_step);
//			phid=phid_new;
double integrate(const double &dydx_new,const double &dydx,const double &y,const double &int_step);

//Integration of Matrix MAT(r,c) 
Matrix integrate(Matrix &DYDX_NEW,Matrix &DYDX,Matrix &Y,const double int_step);

///////////////////////////////////////////////////////////////////////////////
////////////////////  US Standard Atmosphere 1976 /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// 
//Calculates the atmospheric properties density pressure and temperature 
// up to 85 km.
void atmosphere76(double &rho,double &press,double &tempk, const double balt);

#endif