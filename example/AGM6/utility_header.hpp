///////////////////////////////////////////////////////////////////////////////
//FILE: 'utility_header.hpp'
//
//'Matrix' class member functions
//			cholesky
// Module utility functions:
//		    mat2tr
//			mat3tr
//			sign
//			angle
// Stochastic functions
//			unituni
//			uniform
//			gauss
//			markov
//			rayleigh
//			exponential
// Table look-up
// Integration
// US76 Atmosphere
//
//010628 Created by Peter H Zipfel
//020723 In 'markov' replaced static variable by '&value_saved', PZi
//020724 Scrubbed down, PZi
//020829 Dynamically dimensioned utilities, PZi
//030319 Added US76 atmosphere, PZi
//030519 Added Overloaded operator [] for vector, PZi
//030925 Corrected error in assignment operator, PZi
//071029 Added 'cholesky', PZi
//071106 Added diadic product operator %, PZi
//071106 Added scalar division operator /, PZi
///////////////////////////////////////////////////////////////////////////////

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
	//destructor
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

	//Builds a 5x1 vector from five parameters
	//Example: VEC5.build_vec5(v1,v2,v3,v4,v5);
	///////////////////////////////////////////////////////////////////////////////
	Matrix & build_vec5(const double &v1,const double &v2,const double &v3,
								const double &v4,const double &v5);

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

	//Returns square root matrix of square matrix MAT
	//Example: SQRTMAT=MAT.cholesky();
	Matrix cholesky();
						
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

	//Scalar division operator
	//Example: CMAT = AMAT / b;
	Matrix operator/(const double &b);

	//Multiplication operator, returns matrix product 
	//Example: CMAT = AMAT * BMAT;
	Matrix operator*(const Matrix &B);

	//Diadic multiplication operator, returns the matrix of two vectors
	//Example: MAT = VEC1 % VEC2;
	Matrix operator%(const Matrix &B);

	//Scalar multiplication assignment operator (scalar element by element multiplication)
	//Example: AMAT *= b; meaning: AMAT = AMAT * b
	Matrix & operator*=(const double &b);

	//Multiplication assignment operator 
	//Example: AMAT *= BMAT; meaning: AMAT = AMAT * BMAT; 
	Matrix & operator*=(const Matrix &B);

	//Scalar Addition operator (scalar element by element addition)
	//Note: scalar must be the second operand
	//Example: CMAT = AMAT + b; 
	Matrix operator+(const double &b);

	//Addition operator, returns matrix addition
	//Example: CMAT = AMAT + BMAT;
	Matrix operator+(const Matrix &B);

	//Scalar addition assignment operator (scalar element by element addition)
	//Example: AMAT += b; meaning: AMAT = AMAT + b 
	Matrix & operator+=(const double &b);
	
	//Matrix addition assignment operator
	//Example: AMAT += BMAT; meaning: AMAT = AMAT + BMAT;
	Matrix & operator+=(const Matrix &B);

	//Scalar substraction operator (scalar element by element substraction)
	//Note: scalar must be the second operand
	//Example: CMAT = AMAT - b;
	Matrix operator-(const double &b);

	//Substraction operator, returns matrix substraction
	//Example: CMAT = AMAT - BMAT;
	Matrix operator-(const Matrix &B);

	//Scalar substraction assignment operator (scalar element by element substraction)
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

	//Alternate transpose Aij=>Aji
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

	//Returns the skew-symmetric matrix from a 3-dim vector VEC
	//			| 0 -c  b|		|a|
	//			| c  0 -a| <--	|b|
	//			|-b  a  0|		|c|
	//
	//Example: MAT = VEC.skew_sym();
	Matrix skew_sym();

	//Returns the sub matrix after 'row'  and 'col' have been ommitted 
	//Example: BMAT = AMAT.sub_matrix(1,3); (deleting first row and third column!) 
	Matrix sub_matrix(const int &row, const int &col);

	//Returns the stranspose of a matrix Aji <- Aij
	//Example: BMAT = AMAT.trans();
	Matrix trans();

	//Returns unit vector from 3x1 vector
	//Example: UVEC=VEC.univec3();
	Matrix univec3();

	//Returns 9x1 vector from 3x3 matrix row-wise
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

//Returns the sign of the function
//Example: value_signed=value*sign(variable) 
int sign(const double &variable);

//Returns the angle between two 3x1 vectors
double angle(Matrix VEC1,Matrix VEC2);

///////////////////////////////////////////////////////////////////////////////
////////////////////// Stochastic functions ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//Generating uniform random distribution between 0-1 based on C function rand()
double unituni();

//Generating uniform random distribution between 'min' and 'max'
double uniform(double min,double max);

//Generating a standard distribution with 'mean' and 'sig' std deviation
//Ref Numerical Recipies, p 289, 1992 Cambridge University Press
//Function unituni() is a CADAC++ utility
//
//parameter input:
//			min = standard deviation of Gaussian distribution - unit of variable
//			mean = mean value of Gaussian distribution - unit of variable
//return output:
//			value = value of variable - unit of variable
double gauss(double mean,double sig);

//Generating a time-correlated Gaussian variable with zero mean
//Ref: CADAC Subroutine CNT_GAUSS
//Function gauss() is CADAC++ utility
//
//parameter input:
//			sigma = standard deviation of Gaussian distribution - unit of variable
//			bcor = beta time correlation coefficient - 1/s (Hz)
//			time = simulation time - s
//			intstep = integration step size - s
//return output:
//			value = value of variable - unit of variable
double markov(double sigma,double bcor,double time,double intstep,double &value_saved);

//Generating a Rayleigh distribution with peak value of pdf = 'mode'
//Ref: Tybrin, "CADAC Program documentation", June 2000 and source code CADX3.FOR
//Function unituni() is a CADAC++ utility
//
//parameter input:
//			mode= mode (peak value of pdf) of Rayleigh distribution - unit of variable
//return output:
//			value=value of variable - unit of variable
//
//The mean of the distribution is: mean = mode * (pi/2)
//The variance is: variance = mode^2 * (2 - pi/2)
double rayleigh(double mode);

//Generating an exponential distribution with a given mean density
//Ref:
// Tybrin, "CADAC Program documentation", June 2000 and source code CADX3.FOR
// Numerical Recipies, p 287, 1992 Cambridge University Press
//Function unituni() is a CADAC++ utility
//
//parameter input:
//			density = # of events per unit of variable (in the mean)
//return output:
//			value = units of variable to be traversed until next event occurs
//
//The variance is density^2
double exponential(double density);

///////////////////////////////////////////////////////////////////////////////
////////////////////  Integration functions  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//
//Integration of scalar state variable
//Modified Euler method 
//Example first order lag:
//			phid_new=(phic-phi)/tphi;
//			phi=integrate(phid_new,phid,phi,int_step);
//			phid=phid_new;
double integrate(const double &dydx_new,const double &dydx,const double &y,const double &int_step);
///////////////////////////////////////////////////////////////////////////////
//Integration of Matrix MAT(r,c) 
Matrix integrate(Matrix &DYDX_NEW,Matrix &DYDX,Matrix &Y,const double int_step);

///////////////////////////////////////////////////////////////////////////////
////////////////////  US Standard Atmosphere 1976 /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// 
// *Calculates the atmospheric properties density pressure and temperature 
//	up to 85 km.
void atmosphere76(double &rho,double &press,double &tempk, const double balt);

#endif
