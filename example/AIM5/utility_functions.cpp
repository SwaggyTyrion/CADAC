///////////////////////////////////////////////////////////////////////////////
// FILE: 'utility_functions.cpp'
//
//'Matrix' class member functions
//			cholesky
// Module utility functions:
//		    mat2tr
//			mat3tr
//			sign
//			angle
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

#include <fstream>
#include <cmath>
#include "utility_header.hpp"
#include "global_header.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
/////////////////////// 'Matrix' member functions /////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//One dimensional and two dimensional arrays of any size of type 'double'
//
//020826 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
//Constructors
///////////////////////////////////////////////////////////////////////////////
Matrix::Matrix(){}

Matrix::Matrix(int row_size,int col_size)
{
//	cout<<" >>> constructing >>>\n";

	num_row=row_size;
	num_col=col_size;

	pbody=NULL;

	//allocating memory
	num_elem=row_size*col_size;
	pbody=new double[num_elem];
	if(pbody==0){cerr<<"*** Error: Matrix memory allocation failed ***\n";system("pause");exit(1);}

	//initializing array to zero
	for(int i=0;i<num_elem;i++)
		*(pbody+i)=0;
}
Matrix::Matrix(const Matrix &MAT)
{
//	cout<<" >>> copy constructing >>>\n";

	num_row=MAT.num_row;
	num_col=MAT.num_col;
	num_elem=MAT.num_elem;
	pbody=new double[num_elem];
	if(pbody==0){cerr<<"*** Error: Matrix memory allocation failed ***\n";system("pause");exit(1);}

	//copying
	for(int i=0;i<num_elem;i++)
		*(pbody+i)=(*(MAT.pbody+i));
}
///////////////////////////////////////////////////////////////////////////////
//Destructor
///////////////////////////////////////////////////////////////////////////////
Matrix::~Matrix()
{
//	cout<<" <<< destructing <<<\n";
	delete [] pbody;
}	
///////////////////////////////////////////////////////////////////////////////
//Printing matrix to console
///////////////////////////////////////////////////////////////////////////////
void Matrix::print()
{
	double *pmem=pbody;

	//outside loop rows, inside loop columns
	for(int i=0;i<num_row;i++){
		for(int j=0;j<num_col;j++){
			cout<<*pbody<<"\t";
			pbody++;
		}
		cout<<'\n';
	}
	//resetting pointer
	pbody=pmem;
	cout<<"\n\n";
}
///////////////////////////////////////////////////////////////////////////////
//Absolute value of vector
//Example: avalue = VEC.absolute();
///////////////////////////////////////////////////////////////////////////////
double Matrix::absolute() 
{
	if(num_row>1&&num_col>1){cerr<<" *** Warning: not a vector 'Matrix::absolute()' *** \n";}
	double ret=0;
	
	for(int i=0;i<num_elem;i++) 
		ret+=(*(pbody+i))*(*(pbody+i));
	ret=sqrt(ret);

	return ret;
}
///////////////////////////////////////////////////////////////////////////////
//Adjoint matrix (same as determinant procedure however the matrix element
//is NOT multiplied into each cofactor)
//Example: BMAT = AMAT.adjoint();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::adjoint()
{
	if(!(num_row==num_col))
	{cerr<<" *** Error: matrix not square 'Matrix::adjoint()' *** \n";system("pause");exit(1);}
	if((num_row==1)&&(num_col==1))
	{cerr<<" *** Error: only one element 'Matrix::adjoint()' *** \n";system("pause");exit(1);}

	Matrix RESULT(num_row,num_col);

	for(int i=0;i<num_elem;i++){
		//row #
		int row=i/num_col+1;
		//column #
		int col=i%num_col+1;

		if (((row+col)%2)==0)
			*(RESULT.pbody+i)=sub_matrix(row,col).determinant();
		else
			*(RESULT.pbody+i)=(-1.0)*sub_matrix(row,col).determinant();
	}
	return RESULT.trans();
}
//////////////////////////////////////////////////////////////////////////////
//Assigns a value to a matrix element (offset!)
//Example: MAT.assign_loc(r,c,val); ((r+1)th-row, (c+1)th-col)
///////////////////////////////////////////////////////////////////////////////
void Matrix::assign_loc(const int &r, const int &c, const double &val)
{
	if(r>num_row-1||c>num_col-1)
	{cerr<<" *** Error: location outside array 'Matrix::assign_loc()' *** \n";system("pause");exit(1);}

	//assigning value
	int offset=num_col*(r)+c;
	*(pbody+offset)=val;	
}
///////////////////////////////////////////////////////////////////////////////
//Builds a 3x1 vector from three parameters
//Example: VEC.build_vec3(v1,v2,v3);
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::build_vec3(const double &v1,const double &v2,const double &v3)
{
	num_row=3;
	num_col=1;
	*pbody=v1;
	*(pbody+1)=v2;
	*(pbody+2)=v3;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Builds a 5x1 vector from five parameters
//Example: VEC5.build_vec5(v1,v2,v3,v4,v5);
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::build_vec5(const double &v1,const double &v2,const double &v3,
							const double &v4,const double &v5)
{
	num_row=5;
	num_col=1;
	*pbody=v1;
	*(pbody+1)=v2;
	*(pbody+2)=v3;
	*(pbody+3)=v4;
	*(pbody+4)=v5;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Builds a 3x3 matrix from nine paramters arranged in rows
//Example: MAT.build_mat33(v11,v12,v13,v21,v22,v23,v31,v32,v33);
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::build_mat33(const double &v11,const double &v12,const double &v13
						 ,const double &v21,const double &v22,const double &v23
					     ,const double &v31,const double &v32,const double &v33)
{
	num_row=3;
	num_col=3;
	*pbody=v11;    *(pbody+1)=v12;*(pbody+2)=v13;
	*(pbody+3)=v21;*(pbody+4)=v22;*(pbody+5)=v23;
	*(pbody+6)=v31;*(pbody+7)=v32;*(pbody+8)=v33;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Calulates Cartesian from polar coordinates
//|V1|             | cos(elevation)*cos(azimuth)|
//|V2| = magnitude*|cos(elevation)*sin(azimuth) |
//|V3|		       |	  -sin(elevation)       |
//
//Example: VEC.cart_from_pol(magnitude,azimuth,elevation); 	
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::cart_from_pol(const double &magnitude,const double &azimuth
						   ,const double &elevation)
{
	*pbody=magnitude*(cos(elevation)*cos(azimuth));
	*(pbody+1)=magnitude*(cos(elevation)*sin(azimuth));
	*(pbody+2)=magnitude*(sin(elevation)*(-1.0));

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Returns square root matrix of square matrix MAT
//Example: SQRTMAT=MAT.cholesky();
//071029 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::cholesky()
{
	if(!(num_row==num_col))
	{cerr<<" *** Error: matrix not square 'Matrix::cholesky()' *** \n";system("pause");exit(1);}
	
	Matrix SQRTMAT(num_row,num_col);
	double sum(0);
	int dim=num_row;

	for(int i=0;i<dim;i++){
		for(int j=0;j<dim;j++){
			//off-diagonal elements
			if(j<i){
				sum=0;
				if(j>0){
					for(int k=0;k<j;k++)
						sum+=(*(SQRTMAT.pbody+i*dim+k))*(*(SQRTMAT.pbody+j*dim+k));
				}
				if(*(SQRTMAT.pbody+j*dim+j)==0)
					*(SQRTMAT.pbody+i*dim+j)=0;
				else
					*(SQRTMAT.pbody+i*dim+j)=(*(pbody+i*dim+j)-sum)/(*(SQRTMAT.pbody+j*dim+j));
			}
			//diagonal elements
			else if(j==i){
				sum=0;
				if(i>0){
					for(int k=0;k<i;k++)
						sum+=*(SQRTMAT.pbody+i*dim+k)*(*(SQRTMAT.pbody+i*dim+k));
				}
				*(SQRTMAT.pbody+i*dim+j)=sqrt(*(pbody+i*dim+i)-sum);
			}
			else{
				*(SQRTMAT.pbody+i*dim+j)=0;
			}
		}
	}
return SQRTMAT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns column vector of column # 
//Example: VEC = MAT.col_vec(2); (2nd column!)
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::col_vec(const int &col)
{
	if(col<=0||col>num_col)
	{cerr<<" *** Error: column outside array 'Matrix::col_vec()' *** \n";system("pause");exit(1);}
	
	Matrix RESULT(num_row,1);

	for(int i=0;i<num_row;i++){
		int offset=i*num_col+col-1;
		*(RESULT.pbody+i)=(*(pbody+offset));
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the determinant		
//Determinant recursive procedure
//Example: det = MAT.determinant();
///////////////////////////////////////////////////////////////////////////////
double Matrix::determinant()
{
	if(!(num_row==num_col))
	{cerr<<" *** Error: matrix not square 'Matrix::determinant()' *** \n";system("pause");exit(1);}
	
	double result=0;

	//base case of a single matrix element
	if ((num_col==1)&&(num_row==1))
		return *pbody;

	//second base case of a 2x2 matrix
	else if ((num_col==2)&&(num_row==2))
		return (*pbody)*(*(pbody+3))-(*(pbody+1))*(*(pbody+2));

	else
	{
		for(int j=0;j<num_col;j++)
		{
			//use cofactors and submatricies to finish for nxn
			if ((j%2)==0)
			{
				//odd column (numbered!)
				result+=sub_matrix(1,j+1).determinant()*(*(pbody+j));
			}
				else
			{
				//even column (numbered!)
				result+=(-1.0)*sub_matrix(1,j+1).determinant()*(*(pbody+j));
			}
		}
	}	
	return result;
}
///////////////////////////////////////////////////////////////////////////////
//Returns nxn diagonal matrix  from nx1 vector 
//Example: DIAMAT=VEC.diamat_vec()
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::diamat_vec()
{
	if(num_col!=1)
	{cerr<<" *** Error: not a vector 'Matrix::diagmat_vec()' *** \n";system("pause");exit(1);}

	Matrix RESULT(num_row,num_row);
	for(int i=0;i<num_row;i++){
		int offset=i*num_row+i;
		*(RESULT.pbody+offset)=(*(pbody+i));
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns nx1 diagonal vector from nxn matrix
//Example: VEC=MAT.diavec_mat();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::diavec_mat()
{
	if(!(num_row==num_col))
	{cerr<<" *** Error: matrix not square 'Matrix::diavec_mat()' *** \n";system("pause");exit(1);}
	
	Matrix RESULT(num_row,1);
	for(int i=0;i<num_row;i++){
		int offset=i*num_row+i;
		*(RESULT.pbody+i)=(*(pbody+offset));
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Dimensions a matrix of size row x col
//Only used to initialize arrays in class 'Variable'
//Example: MAT.dimension(3,3);
///////////////////////////////////////////////////////////////////////////////
void Matrix::dimension(int row,int col)
{
	num_row=row;
	num_col=col;

	pbody=NULL;

	//allocating memory
	num_elem=row*col;
	pbody=new double[num_elem];
	if(pbody==0){cerr<<"*** Error: memory allocation failed 'Matrix::dimension()' ***\n";system("pause");exit(1);}

	//initializing array to zero
	for(int i=0;i<num_elem;i++)
		*(pbody+i)=0;
}
///////////////////////////////////////////////////////////////////////////////
//Bi-variate ellipse
//calculating major and minor semi-axes of ellipse and rotation angle 
//    from the symmetrical pos semi-definite MAT(2x2) matrix
//coordinate axes orientation:
//          ^ 1-axis
//          |
//          |
//          |---> 2-axis
//
//angle is measured from 1st coordinate axis to the right
//
//major_semi_axis = ELLIPSE.get_loc(0,0);
//minor_semi_axis = ELLIPSE.get_loc(1,0);
//angle      = ELLIPSE.get_loc(2,0);
//
//Example: ELLIPSE = MAT.ellipse();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::ellipse()
{
	Matrix ELLIPSE(3,1);
	double dum=0;
	double dum1=0;
	double dum2=0;
	double ama=0;
	double ami=0;
	double phi=0;
	double ak1=0;
	double ak2=0;
	Matrix X1V(2,1); //major principal axes of ellipse 
	Matrix X2V(2,1); //minor principal axes of ellipse 

	double a11=*pbody;
	double a22=*(pbody+3);
	double a12=*(pbody+1);
	double a1122=a11+a22;
	double aq1122=a1122*a1122;
	dum1=aq1122-4.*(a11*a22-a12*a12);
	if(dum1>=0)dum2=sqrt(dum1);

	//major and minor semi-axes of ellipse
	ama=(a1122+dum2)/2.;
	ami=(a1122-dum2)/2.;
	ELLIPSE.assign_loc(0,0,ama);
	ELLIPSE.assign_loc(1,0,ami);
	if(ama==ami)return ELLIPSE;

	//angle of orientation of major axis wrt first principal axis
	if(a11-ama!=0){
         dum1=-a12/(a11-ama);
         ak1=sqrt(1./(1.+dum1*dum1));
         X1V.assign_loc(0,0,dum1*ak1);
         X1V.assign_loc(1,0,ak1);
         dum=dum1*ak1;
         if(fabs(dum)>1.) dum=1.*sign(dum);
         phi=acos(dum);
		 ELLIPSE.assign_loc(2,0,phi);
	}
	else{
         dum1=-a12/(a22-ama);
         ak1=sqrt(1./(1.+dum1*dum1));
         X1V.assign_loc(0,0,ak1);
         X1V.assign_loc(1,0,dum1*ak1);
         if(fabs(ak1)>1.) ak1=1.*sign(ak1);
         phi=acos(ak1);
		 ELLIPSE.assign_loc(2,0,phi);
	}
	//second principal axis - not used
	if(a11-ami!=0){
         dum2=-a12/(a11-ami);
         ak2=sqrt(1./(1.+dum2*dum2));
         X2V.assign_loc(0,0,dum2*ak2);
         X2V.assign_loc(1,0,ak2);
	}
	else{
         dum2=-a12/(a22-ami);
         ak2=sqrt(1./(1.+dum2*dum2));
         X2V.assign_loc(0,0,ak2);
         X2V.assign_loc(1,0,dum2*ak2);
	}
	return ELLIPSE;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the number of columns of matrix MAT
//Example: nc = MAT.get_cols();
///////////////////////////////////////////////////////////////////////////////
int Matrix::get_cols()
{
	return num_col;
}
///////////////////////////////////////////////////////////////////////////////
//Returns offset-index given row# and col#
//Example: i = MAT.get_index(2,3); (2nd row, 3rd column)
//////////////////////////////////////////////////////////////////////////////
int Matrix::get_index(const int &row, const int &col)
{
	int index;
		index=(row-1)*num_col+col-1;
	return index;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the value at offset-row 'r' offset-col 'c' of MAT
//Example: value = MAT.get_loc(2,1); (3rd row, 2nd column)
///////////////////////////////////////////////////////////////////////////////
double Matrix::get_loc(const int &r,const int &c)
{
	if((r<num_row)&&(c<num_col))
	return *(pbody+r*num_col+c);		
	else
	{
		{cout<<"*** Error: invalid matrix location 'Matrix::get_loc()' *** ";system("pause");exit(1);}
		return 0;
	}
}
///////////////////////////////////////////////////////////////////////////////
//Returns the number of rows in the matrix
//Example: nr = MAT.get_rows();
///////////////////////////////////////////////////////////////////////////////
int Matrix::get_rows()
{
	return num_row;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the pointer to MAT
//Example: ptr = MAT.get_pbody();
///////////////////////////////////////////////////////////////////////////////
double * Matrix::get_pbody()
{
	return pbody;
}
///////////////////////////////////////////////////////////////////////////////
//Builds a square identity matrix of object 'Matrix MAT'
//Example:	MAT.identity();
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::identity()
{
	if (num_row==num_col)
	{
		for(int r=0;r<num_row;r++)
			*(pbody+r*num_row+r)=1.;
	}
	else
	{cout<<"*** Error: matrix not square 'Matrix::identiy()'*** ";system("pause");exit(1);}

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the inverse of a square matrix AMAT 
//Inversion  INVERSE =(1/det(A))*Adj(A)
//Example: INVERSE = AMAT.inverse();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::inverse()
{
	if (num_col!=num_row)
	{cerr<<" *** Error: not a square matrix 'Matrix::inverse()' *** \n";system("pause");exit(1);}

	Matrix RESULT(num_row,num_col);
	double d=0;

	d=determinant();
	if (d==0)
	{cerr<<" *** Error: singular! 'Matrix::inverse()' *** \n";system("pause");exit(1);}

	d=1./d;
	RESULT=adjoint();
	RESULT=RESULT*d;

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns 3x3 matrix row-wise from 9x1 vector
//Example: MAT=VEC.	mat33_vec9();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::mat33_vec9()
{
	if(!(num_row==9 && num_col==1))
	{cerr<<" *** Error: vector not 9 x 1 'Matrix::mat33_vec9()' *** \n";system("pause");exit(1);}
	
	Matrix RESULT(3,3);
	for(int i=0;i<9;i++){
		*(RESULT.pbody+i)=*(pbody+i);
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Forms  matrix MAT with all elements '1.' from object MAT(num_row,num_col)
//Example: MAT.ones();
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::ones()
{
	for(int r=0;r<num_elem;r++)
		*(pbody+r)=1.;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Inequality relational operator, returns true or false 
//returns true if elements differ by more than EPS
//Example: if(AMAT!=BMAT){...};
///////////////////////////////////////////////////////////////////////////////
bool Matrix::operator!=(const Matrix &B)
{
	//check dimensions
	if (num_col!=B.num_col)
			return true;
	else if
		(num_row!=B.num_row)
			return true;

	for (int i=0;i<num_elem;i++){
			//check to see if values differ by more or less than EPS
			if ((*(pbody+i)-(*(B.pbody+i)))>EPS)
				return true;
			else if ((*(pbody+i)-(*(B.pbody+i)))<(-1.*EPS))
				return true;
		}
	return false;
}
///////////////////////////////////////////////////////////////////////////////
//Scalar multiplication operator
//Note: scalar must be the second operand
//Example: CMAT = AMAT * b;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator*(const double &b)
{
	Matrix RESULT(num_row,num_col);

	for (int i=0;i<num_elem;i++)
		*(RESULT.pbody+i)=*(pbody+i)*b;

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Scalar division operator
//Note: scalar must be the second operand
//Example: CMAT = AMAT / b;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator/(const double &b)
{
	Matrix RESULT(num_row,num_col);

	for (int i=0;i<num_elem;i++)
		*(RESULT.pbody+i)=(*(pbody+i))/b;

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Multiplication operator, returns matrix product
// associative but not commutative
//Example: CMAT = AMAT * BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator*(const Matrix &B)
{
	//create resultant matrix
	Matrix RESULT(num_row,B.num_col);
	int r=0; int c=0;

	//check for proper dimensions
	if (num_col!=B.num_row)
	{cout<<"*** Error: incompatible dimensions 'Matrix::operator*()' *** ";system("pause");exit(1);}

	for(int i=0;i<RESULT.num_elem;i++){
		r=i/B.num_col;
		c=i%B.num_col;
		for (int k=0; k<num_col;k++){
			*(RESULT.pbody+i)+= *(pbody+k+num_col*r)*(*(B.pbody+k*B.num_col+c));
		}
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Diadic multiplication operator, returns the matrix of two vectors
// both vectors are entered as column vectors
//Example: MAT = VEC1 % VEC2;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator%(const Matrix &B)
{
	//create resultant matrix
	Matrix RESULT(num_row,B.num_row);

	for(int i=0;i<num_row;i++){
		for (int k=0;k<B.num_row;k++){
			*(RESULT.pbody+B.num_row*i+k)= *(pbody+i)*(*(B.pbody+k));
		}
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Scalar multiplication assignment operator (scalar element by element multiplication)
//Example: AMAT *= b; meaning: AMAT = AMAT * b
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::operator*=(const double &b)
{
	for (int i=0;i<num_elem;i++)
		*(pbody+i)=*(pbody+i)*b;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Matrix multiplication assignment operator
//matrix B in argument must be square
//Example: AMAT *= BMAT; meaning: AMAT = AMAT * BMAT; 
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::operator*=(const Matrix &B)
{
	//create resultant matrix
	Matrix RESULT(num_row,B.num_col);
	int i(0);

	//check for proper dimensions
	if (num_col!=B.num_row)
	{cout<<"*** Error: incompatible dimensions 'Matrix::operator*=()' *** ";system("pause");exit(1);}

	//check for squareness of B
	if (B.num_col!=B.num_row)
	{cout<<"*** Error: Second matrix is not square 'Matrix::operator*=()' *** ";system("pause");exit(1);}

	for(i=0;i<RESULT.num_elem;i++){
		int r=i/B.num_col;
		int c=i%B.num_col;
		for (int k=0; k<num_col;k++){
			*(RESULT.pbody+i)+= *(pbody+k+num_col*r)*(*(B.pbody+k*B.num_col+c));
		}
	}
	num_col=RESULT.num_col;
	num_row=RESULT.num_row;
	num_elem=num_row*num_col;
	for (i=0;i<num_elem;i++)
		*(pbody+i)=*(RESULT.pbody+i);

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Scalar Addition operator (scalar element by element addition)
//Note: scalar must be the second operand
//Example: CMAT = AMAT + b; 
///////////////////////////////////////////////////////////////////////////////
Matrix  Matrix::operator+(const double &b)
{
	Matrix RESULT(num_row,num_col);

	for (int i=0;i<num_elem;i++)
		*(RESULT.pbody+i)=*(pbody+i)+b;

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Addition operator, returns matrix addition
//Example: CMAT = AMAT + BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator+(const Matrix &B)
{
	Matrix RESULT(num_row,num_col);

	if ((num_col!=B.num_col)||(num_row!=B.num_row))
	{cout<<"*** Error: matrices have different dimensions 'Matrix::operator +' *** ";system("pause");exit(1);}

	for (int i=0;i<num_elem;i++)
		*(RESULT.pbody+i)=*(pbody+i)+(*(B.pbody+i));

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Scalar addition assignment operator (scalar element by element addition)
//Example: AMAT += b; meaning: AMAT = AMAT + b 
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::operator+=(const double &b)
{
	for (int i=0;i<num_elem;i++)
		*(pbody+i)=*(pbody+i)+b;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Matrix addition assignment operator
//Example: AMAT += BMAT; meaning: AMAT = AMAT + BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::operator+=(const Matrix &B)
{
	if ((num_col!=B.num_col)||(num_row!=B.num_row))
	{cout<<"*** Error: matrices have different dimensions 'Matrix::operator +=' *** ";system("pause");exit(1);}

	for (int i=0;i<num_elem;i++)
		*(pbody+i)=*(pbody+i)+(*(B.pbody+i));

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Scalar substraction operator (scalar element by element substraction)
//Note: scalar must be the second operand
//Example: CMAT = AMAT - b;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator-(const double &b)
{
	Matrix RESULT(num_row,num_col);
	for (int i=0;i<num_elem;i++)
		*(RESULT.pbody+i)=*(pbody+i)-b;

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Substraction operator, returns matrix substraction
//Example: CMAT = AMAT - BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator-(const Matrix &B)
{
	Matrix RESULT(num_row,num_col);

	if ((num_col!=B.num_col)||(num_row!=B.num_row))
	{cout<<"*** Error: matrices have different dimensions 'Matrix::operator -' *** ";system("pause");exit(1);}
	for (int i=0;i<num_elem;i++)
		*(RESULT.pbody+i)=*(pbody+i)-*(B.pbody+i);
	
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Scalar substraction assignment operator (scalar element by element substraction)
//Example: AMAT -= b; meaning: AMAT = AMAT - b
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::operator-=(const double &b)
{
	for (int i=0;i<num_elem;i++)
		*(pbody+i)=*(pbody+i)-b;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Matrix subtraction assignment operator
//Example: AMAT -= BMAT; meaning: AMAT = AMAT - BMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::operator-=(const Matrix &B)
{
	if ((num_col!=B.num_col)||(num_row!=B.num_row))
	{cout<<"*** Error: matrices have different dimensions 'Matrix::operator +=' *** ";system("pause");exit(1);}

	for (int i=0;i<num_elem;i++)
		*(pbody+i)=*(pbody+i)-(*(B.pbody+i));

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Assignment operator (deep copy)
//Example: AMAT = BMAT; also: AMAT = BMAT = CMAT;
//Actually: AMAT.operator=(BMAT); also: AMAT.operator=(BMAT.operator=(CMAT));
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::operator=(const Matrix &B)
{
	if((num_row != B.num_row)||(num_col != B.num_col))
	{cerr<<" *** Error: incompatible dimensions 'Matrix::operator=()' *** \n";system("pause");exit(1);}

	delete [] pbody;
	num_elem=B.num_elem;
	num_row=B.num_row;
	num_col=B.num_col;
	pbody=new double[num_elem];

	for (int i=0;i<num_elem;i++)
		*(pbody+i)=(*(B.pbody+i));

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
//Equality relational operator
//returns true if elements differ by less than EPS
//Example: if(AMAT==BMAT){...};
///////////////////////////////////////////////////////////////////////////////
bool Matrix::operator==(const Matrix &B)
{
	//check dimensions
	if (num_col!=B.num_col)
			return false;
	else if
		(num_row!=B.num_row)
			return false;

	for (int i=0;i<num_elem;i++){
			//check to see if values differ by more or less than EPS
			if ((*(pbody+i)-(*(B.pbody+i)))>EPS)
				return false;
			else if ((*(pbody+i)-(*(B.pbody+i)))<(-1.*EPS))
				return false;
		}
	return true;
}
///////////////////////////////////////////////////////////////////////////////
//Extracting components from vector with offset operator []
//returns the component(i) from vector VEC[i] or assigns a value to component VEC[i]
//Examples: comp_i=VEC[i]; VEC[i]=comp_i;
///////////////////////////////////////////////////////////////////////////////
double & Matrix::operator[](const int &r)
{
	if((r<num_row)&&(num_col=1))
		return *(pbody+r);		
	else
	{
		{cout<<"*** Error: invalid matrix location,'Matrix::operator[]' *** ";system("pause");exit(1);}
	}
}
///////////////////////////////////////////////////////////////////////////////
//Scalar product operator (any combination of row or column vectors)  
//Example: value = AMAT ^ BMAT;  
///////////////////////////////////////////////////////////////////////////////
double Matrix::operator^(const Matrix &B)
{
	//initialize the result
	double result=0;

	//check dimensions
	bool one=false;
	bool dim=false;
	//true if both arrays have dimension '1'
	if((num_row==1||num_col==1)&&(B.num_row==1||B.num_col==1))one=true;
	//true if both arrays have at least one equal dimension
	if((num_row==B.num_row||num_row==B.num_col)&&(num_col==B.num_col||num_col==B.num_row))dim=true;
	if(!one||!dim)
	{cerr<<" *** Error: incompatible dimensions 'Matrix::operator^()' *** \n";system("pause");exit(1);}

	for (int i=0;i<num_row;i++)
			result+=*(pbody+i)*(*(B.pbody+i));
	return result;
}
///////////////////////////////////////////////////////////////////////////////
//Alternate transpose Aji <- Aij			
//Example: BMAT = ~AMAT;
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::operator~()
{
	Matrix RESULT(num_col, num_row);
	int i=0; //offset for original matrix
	int j=0; //offset for transposed matrix

	for (int r=0;r<num_row;r++){
		for(int c=0;c<num_col;c++){
			//offset for transposed
			j=c*num_row+r;
			*(RESULT.pbody+j)=*(pbody+i);
			i++;j++;
		}
	}			
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns polar from cartesian coordinates
// magnitude = POLAR(0,0) = |V|
// azimuth   = POLAR(1,0) = atan2(V2,V1)
// elevation = POLAR(2,0) = atan2(-V3,sqrt(V1^2+V2^2)
//Example: POLAR = VEC.pol_from_cart();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::pol_from_cart() 
{
	double d=0;
	double azimuth=0;
	double elevation=0;
	double denom;
	Matrix POLAR(3,1);
	
	double v1=(*pbody);
	double v2=(*(pbody+1));
	double v3=(*(pbody+2));

	d=sqrt(v1*v1+v2*v2+v3*v3);
	azimuth=atan2(v2,v1);

	denom=sqrt(v1*v1+v2*v2);
	if(denom>0)
		elevation=atan2(-v3,denom);
	else{
		if(v3>0) elevation=-PI/2.;
		if(v3<0) elevation=PI/2.;
		if(v3==0) elevation=0;
	}
	
	*POLAR.pbody=d;
	*(POLAR.pbody+1)=azimuth;
	*(POLAR.pbody+2)=elevation;

	return POLAR;
}
///////////////////////////////////////////////////////////////////////////////
//Returns row vector of row # 
//Example: VEC = MAT.row_vec(2); (2nd row!)
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::row_vec(const int &row)
{
	if(row<=0||row>num_row)
	{cerr<<" *** Error: row outside array 'Matrix::row_vec()' *** \n";system("pause");exit(1);}
	
	Matrix RESULT(1,num_col);

	for(int i=0;i<num_col;i++){
		int offset=(row-1)*num_col+i;
		*(RESULT.pbody+i)=(*(pbody+offset));
	}
	return RESULT;
}

///////////////////////////////////////////////////////////////////////////////
//Returns the skew-symmetric matrix from a 3-dim vector VEC
//			| 0 -c  b|		|a|
//			| c  0 -a| <--	|b|
//			|-b  a  0|		|c|
//
//Example: MAT = VEC.skew_sym();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::skew_sym()
{
	Matrix RESULT(3,3);
	//check for proper dimensions
	if (num_col!=1||num_row!=3)
	{cout<<"*** Error: not a 3x1 column vector 'Matrix::skew_sym()' *** ";system("pause");exit(1);}
	
	*(RESULT.pbody+5)=-(*pbody);
	*(RESULT.pbody+7)=(*pbody);
	*(RESULT.pbody+2)=(*(pbody+1));
	*(RESULT.pbody+6)=-(*(pbody+1));
	*(RESULT.pbody+1)=-(*(pbody+2));
	*(RESULT.pbody+3)=(*(pbody+2));

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the sub matrix after 'row'  and 'col' have been ommitted 
//Example: BMAT = AMAT.sub_matrix(1,3); (deleting first row and third column!) 
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::sub_matrix(const int &row, const int &col)
{ 
	if((row>num_row)||(col>num_col))
	{cerr<<" *** Error: row or column outside array 'Matrix::sub_matrix()' *** \n";system("pause");exit(1);}
	if(row==0||col==0)
	{cerr<<" *** Error: row/col are numbered not offset 'Matrix::sub_matrix()' *** \n";system("pause");exit(1);}

	//create return matrix
	Matrix RESULT(num_row-1,num_col-1);
	//start and stop of skipping matrix elements 
	int skip_start=(row-1)*num_col;
	int skip_end=skip_start+num_col;

	//initialize RESULT offset j
	int j=0;

	for (int i=0;i<num_elem;i++){
		//skip elements of row to be removed
		if((i<skip_start)||(i>=skip_end)){
			//offset of column element to be removed
			int offset_col=(col-1)+(i/num_col)*num_col;
			//skip elements of col to be removed
			if(i!=offset_col){
				*(RESULT.pbody+j)=*(pbody+i);
				j++;
			}
		}
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Transpose Aji <- Aij			
//Example: BMAT = AMAT.trans();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::trans()
{
	Matrix RESULT(num_col, num_row);
	int i=0; //offset for original matrix
	int j=0; //offset for transposed matrix

	for (int r=0;r<num_row;r++){
		for(int c=0;c<num_col;c++){
			//offset for transposed
			j=c*num_row+r;
			*(RESULT.pbody+j)=*(pbody+i);
			i++;j++;
		}
	}			
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns unit vector from 3x1 vector
//Example: UVEC=VEC.univec3();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::univec3()
{
	Matrix RESULT(3,1);
	//check for proper dimensions
	if (num_col!=1||num_row!=3)
	{cout<<" *** Error: not a 3x1 column vector in 'Matrix::univec()' *** \n";system("pause");exit(1);}

	double v1=(*pbody);
	double v2=(*(pbody+1));
	double v3=(*(pbody+2));
	double d=sqrt(v1*v1+v2*v2+v3*v3);

	//if VEC is zero than the unit vector is also a zero vector
	if(d==0){		
		*RESULT.pbody=0;
		*(RESULT.pbody+1)=0;
		*(RESULT.pbody+2)=0;
	}
	else{
		*RESULT.pbody=v1/d;
		*(RESULT.pbody+1)=v2/d;
		*(RESULT.pbody+2)=v3/d;
	}	
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns 9x1 vector from 3x3 matrix row-wise
//Example: VEC=MAT.vec9_mat33();
///////////////////////////////////////////////////////////////////////////////
Matrix Matrix::vec9_mat33()
{
	if(!(num_row==3 && num_col==3))
	{cerr<<" *** Error: matrix not 3 x 3 'Matrix::vec9_mat33()' *** \n";system("pause");exit(1);}
	
	Matrix RESULT(9,1);
	for(int i=0;i<9;i++){
		*(RESULT.pbody+i)=*(pbody+i);
	}
	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
//Forms a zero matrix MAT from object MAT(num_row,num_col)
//Example: MAT.zero();
///////////////////////////////////////////////////////////////////////////////
Matrix & Matrix::zero()
{
	for(int i=0;i<num_elem;i++)
		*(pbody+i)=0;

	return *this;
}
///////////////////////////////////////////////////////////////////////////////
////////////////// Module utility functions ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//Returns the T.M. of the psivg -> thtvg sequence
//
//010628 Created by Peter H Zipfel
////////////////////////////////////////////////////////////////////////////////

Matrix mat2tr(const double &psivg,const double &thtvg)
{
	Matrix AMAT(3,3);
	AMAT.assign_loc(0,2,-sin(thtvg));
	AMAT.assign_loc(1,0,-sin(psivg));
	AMAT.assign_loc(1,1,cos(psivg));
	AMAT.assign_loc(2,2,cos(thtvg));
	AMAT.assign_loc(0,0,(AMAT.get_loc(2,2) * AMAT.get_loc(1,1)));
	AMAT.assign_loc(0,1,(-AMAT.get_loc(2,2) * AMAT.get_loc(1,0)));
	AMAT.assign_loc(2,0,(-AMAT.get_loc(0,2) * AMAT.get_loc(1,1)));
	AMAT.assign_loc(2,1,(AMAT.get_loc(0,2) * AMAT.get_loc(1,0)));
	AMAT.assign_loc(1,2,0);

	return AMAT;
}
////////////////////////////////////////////////////////////////////////////////
//Returns the T.M. of the psi->tht->phi sequence
//Euler angle transformation matrix of flight mechanics
//
//011126 Created by Peter H Zipfel
////////////////////////////////////////////////////////////////////////////////

Matrix mat3tr(const double &psi,const double &tht,const double &phi)
{
	double spsi=sin(psi);
	double cpsi=cos(psi);
	double stht=sin(tht);
	double ctht=cos(tht);
	double sphi=sin(phi);
	double cphi=cos(phi);

	Matrix AMAT(3,3);
	AMAT.assign_loc(0,0,cpsi*ctht);
	AMAT.assign_loc(1,0,cpsi*stht*sphi-spsi*cphi);
	AMAT.assign_loc(2,0,cpsi*stht*cphi+spsi*sphi);
	AMAT.assign_loc(0,1,spsi*ctht);
	AMAT.assign_loc(1,1,spsi*stht*sphi+cpsi*cphi);
	AMAT.assign_loc(2,1,spsi*stht*cphi-cpsi*sphi);
	AMAT.assign_loc(0,2,-stht);
	AMAT.assign_loc(1,2,ctht*sphi);
	AMAT.assign_loc(2,2,ctht*cphi);

	return AMAT;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the sign of the variable
//Example: value_signed=value*sign(variable) 
//010824 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int sign(const double &variable)
{
	int sign=0;
	if(variable<0)sign=-1;
	if(variable>=0)sign=1;

	return sign;
}
///////////////////////////////////////////////////////////////////////////////
//Returns the angle between two 3x1 vectors
//Example: theta=angle(VEC1,VEC2);
//010824 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double angle(Matrix VEC1,Matrix VEC2)
{
	double argument;
	double scalar=VEC1^VEC2;
	double abs1=VEC1.absolute();
	double abs2=VEC2.absolute();

	double dum=abs1*abs2;
	if(abs1*abs2>EPS)
		argument=scalar/dum;
	else
		argument=1.;
	if(argument>1.) argument=1.;
	if(argument<-1.) argument=-1.;

	return acos(argument);
}
///////////////////////////////////////////////////////////////////////////////
//////////////// Table look-up and interpolation functions ////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Single independent variable look-up
//Constant extrapolation at the upper end, slope extrapolation at the lower end
//
//030717 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double Datadeck::look_up(string name,double value1)
{
	//finding slot of table in table pointer array (Table **table_ptr) 
	int slot(-1);
	string tbl_name;
	do{
		slot++;
		tbl_name=get_tbl(slot)->get_name();
	}while(name!=tbl_name);

		//getting table index locater of discrete value just below of variable value
	int var1_dim=get_tbl(slot)->get_var1_dim();
	int loc1=find_index(var1_dim-1,value1,get_tbl(slot)->var1_values);

	//using max discrete value if value is outside table
	if (loc1==(var1_dim-1)) return get_tbl(slot)->data[loc1];
		
	return interpolate(loc1,loc1+1,slot,value1);
}
///////////////////////////////////////////////////////////////////////////////
//Two independent variables look-up
//constant extrapolation at the upper end, slope extrapolation at the lower end
//
//030717 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double Datadeck::look_up(string name,double value1,double value2)
{
	//finding slot of table in table pointer array (Table **table_ptr) 
	int slot(-1);
	string tbl_name;
	do{
		slot++;
		tbl_name=get_tbl(slot)->get_name();
	}while(name!=tbl_name);
	
	//getting table index (off-set) locater of discrete value just below or equal of the variable value
	int var1_dim=get_tbl(slot)->get_var1_dim();
	int loc1=find_index(var1_dim-1,value1,get_tbl(slot)->var1_values);

	int var2_dim=get_tbl(slot)->get_var2_dim();
	int loc2=find_index(var2_dim-1,value2,get_tbl(slot)->var2_values);
		
	return interpolate(loc1,loc1+1,loc2,loc2+1,slot,value1,value2);
}
///////////////////////////////////////////////////////////////////////////////
//Three independent variables look-up
//constant extrapolation at the upper end, slope extrapolation at the lower end
//
//030723 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double Datadeck::look_up(string name,double value1,double value2,double value3)
{
	//finding slot of table in table pointer array (Table **table_ptr) 
	int slot(-1);
	string tbl_name;
	do{
		slot++;
		tbl_name=get_tbl(slot)->get_name();
	}while(name!=tbl_name);
	
	//getting table index locater of discrete value just below of variable value
	int var1_dim=get_tbl(slot)->get_var1_dim();
	int loc1=find_index(var1_dim-1,value1,get_tbl(slot)->var1_values);

	int var2_dim=get_tbl(slot)->get_var2_dim();
	int loc2=find_index(var2_dim-1,value2,get_tbl(slot)->var2_values);
		
	int var3_dim=get_tbl(slot)->get_var3_dim();
	int loc3=find_index(var3_dim-1,value3,get_tbl(slot)->var3_values);
		
	return interpolate(loc1,loc1+1,loc2,loc2+1,loc3,loc3+1,slot,value1,value2,value3);
}
///////////////////////////////////////////////////////////////////////////////
//Table index finder
//This is a binary search method it is O(lgN)
// * Returns array locater (offset index) of the discrete_variable just below variable
// * Keeps max or min array locater if variable is outside those max or min  
//
//030717 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
int Datadeck::find_index(int max,double value,double *list)
{
	if(value>=list[max])
		return max;
	else if (value<=list[0]){
		return 0;
	}
	else{
		int index=0;
		int mid;
		while(index<=max){
			mid=(index+max)/2;		//integer division
			if(value<list[mid])
				max=mid-1;
			else if(value>list[mid])
				index=mid+1;
			else
				return mid;
		}
		return max;
	}
}
///////////////////////////////////////////////////////////////////////////////
//Linear one-dimensional interpolation
//Data deck must contain table in the following format:
//
// X1       Table Values(X1)
//
// X11		Y11
// X12		Y12
// X13		Y13
//           
// * Constant extrapolation beyond max values of X1
// * Slope extrapolation beyond min values of X1
//
//030717 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double Datadeck::interpolate(int ind1,int ind2,int slot,double val)
{
	double dx(0),dy(0);
	double dumx(0);

	double diff=val-get_tbl(slot)->var1_values[ind1];
	dx=get_tbl(slot)->var1_values[ind2]-get_tbl(slot)->var1_values[ind1];
	dy=get_tbl(slot)->data[ind2]-get_tbl(slot)->data[ind1];

	if(dx>EPS) dumx=diff/dx;
	dy=dumx*dy;

	return get_tbl(slot)->data[ind1]+dy;
}
///////////////////////////////////////////////////////////////////////////////
//Linear, two-dimensional interpolation
//File must contain table in the following form:
//
//  X1  X2  //Table Values(X1-row, X2-column)
//            ---------------
//  X11 X21   |Y11  Y12  Y13| 
//  X12 X22   |Y21  Y22  Y23|    <- data
//  X13 X23   |Y31  Y32  Y33| 
//  X14       |Y41  Y42  Y43| 
//            ---------------
//Constant extrapolation beyond max values of X1 and X2
//Slope extrapolation beyond min values of X1 and X2
//
//030718 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double Datadeck::interpolate(int ind10,int ind11,int ind20,int ind21,int slot,double value1,
					double value2)																
{
	double dx1(0),dx2(0);
	double dumx1(0),dumx2(0);

	int var1_dim=get_tbl(slot)->get_var1_dim();
	int var2_dim=get_tbl(slot)->get_var2_dim();

	double diff1=value1-get_tbl(slot)->var1_values[ind10];;
	double diff2=value2-get_tbl(slot)->var2_values[ind20];

	if(ind10==(var1_dim-1)) //Assures constant upper extrapolation of first variable
		ind11=ind10;
	else
		dx1=get_tbl(slot)->var1_values[ind11]-get_tbl(slot)->var1_values[ind10];

	if(ind20==(var2_dim-1)) //Assures constant upper extrapolation of second variable
		ind21=ind20;
	else
		dx2=get_tbl(slot)->var2_values[ind21]-get_tbl(slot)->var2_values[ind20];

	if(dx1>EPS) dumx1=diff1/dx1;		
	if(dx2>EPS) dumx2=diff2/dx2;
		
	double y11=get_tbl(slot)->data[ind10*var2_dim+ind20];
	double y12=get_tbl(slot)->data[ind10*var2_dim+ind21];
	double y21=get_tbl(slot)->data[ind11*var2_dim+ind20];
	double y22=get_tbl(slot)->data[ind11*var2_dim+ind21];
	double y1=dumx1*(y21-y11)+y11;
	double y2=dumx1*(y22-y12)+y12;

	return dumx2*(y2-y1)+y1;
}
///////////////////////////////////////////////////////////////////////////////
//Linear, three-dimensional interpolation
//File must contain table in the following form:
//
//  X1  X2  X3    Table Values(X1-row, X2-block, X3-column) <- don't type (illustration only)
//
//                (X1 x X3) (X1 x X3) (X1 x X3) (X1 x X3)	<- don't type 
//				   for X21   for X22   for X23   for X24	<- don't type 
//               -----------------------------------------
//  X11 X21 X31  |Y111 Y112|Y121 Y122|Y131 Y132|Y141 Y142|  
//  X12 X22 X32  |Y211 Y212|Y221 Y222|Y231 Y232|Y241 Y242|  <- data; don't type: '|'
//  X13 X23      |Y311 Y312|Y321 Y322|Y331 Y332|Y341 Y342| 
//      X24      ----------------------------------------- 
//               
//Constant extrapolation beyond max values of X1, X2 and X3
//Slope extrapolation beyond min values of X1, X2 and X3
//
//030723 Created and corrected by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
double Datadeck::interpolate(int ind10,int ind11,int ind20,int ind21,int ind30,int ind31,
							 int slot,double value1,double value2,double value3)
{
	double dx1(0),dx2(0),dx3(0);
	double dumx1(0),dumx2(0),dumx3(0);

	int var1_dim=get_tbl(slot)->get_var1_dim();
	int var2_dim=get_tbl(slot)->get_var2_dim();
	int var3_dim=get_tbl(slot)->get_var3_dim();

	double diff1=value1-get_tbl(slot)->var1_values[ind10];;
	double diff2=value2-get_tbl(slot)->var2_values[ind20];
	double diff3=value3-get_tbl(slot)->var3_values[ind30];

	if(ind10==(var1_dim-1)) //Assures constant upper extrapolation of first variable
		ind11=ind10;
	else
		dx1=get_tbl(slot)->var1_values[ind11]-get_tbl(slot)->var1_values[ind10];

	if(ind20==(var2_dim-1)) //Assures constant upper extrapolation of second variable
		ind21=ind20;
	else
		dx2=get_tbl(slot)->var2_values[ind21]-get_tbl(slot)->var2_values[ind20];

	if(ind30==(var3_dim-1)) //Assures constant upper extrapolation of third variable
		ind31=ind30;
	else
		dx3=get_tbl(slot)->var3_values[ind31]-get_tbl(slot)->var3_values[ind30];

	if(dx1>EPS) dumx1=diff1/dx1;		
	if(dx2>EPS) dumx2=diff2/dx2;
	if(dx3>EPS) dumx3=diff3/dx3;
	//int ind10,int ind11,int ind20,int ind21,int ind30,int ind31
	//      i        i+1        j         j+1       k        k+1
	// Use innner x1 and outer variable x3 for 2DIM interpolation, middle variable x2 is parameter
	// For parameter ind20
	double y11=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind20*var3_dim+ind30];
	double y12=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind20*var3_dim+ind30+var2_dim*var3_dim];
	double y31=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind20*var3_dim+ind31];
	double y32=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind20*var3_dim+ind31+var2_dim*var3_dim];
	//2DIM interpolation
	double y1=dumx1*(y12-y11)+y11;
	double y3=dumx1*(y32-y31)+y31;
	double y21=dumx3*(y3-y1)+y1;

	// For parameter ind21
	y11=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind21*var3_dim+ind30];
	y12=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind21*var3_dim+ind30+var2_dim*var3_dim];
	y31=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind21*var3_dim+ind31];
	y32=get_tbl(slot)->data[ind10*var2_dim*var3_dim+ind21*var3_dim+ind31+var2_dim*var3_dim];
	//2DIM interpolation
	y1=dumx1*(y12-y11)+y11;
	y3=dumx1*(y32-y31)+y31;
	double y22=dumx3*(y3-y1)+y1;

	//1DIM interpolation between the middle variable 
	return dumx2*(y22-y21)+y21;
}
///////////////////////////////////////////////////////////////////////////////
////////////////////  Integration functions  //////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//Integration of scalar state variable
//Modified Euler method
//Example first order lag:
//			phid_new=(phic-phi)/tphi;
//			phi=integrate(phid_new,phid,phi,int_step);
//			phid=phid_new;
//050203 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
//double integrate(double dydx_new,double dydx,double y,double int_step)
double integrate(const double &dydx_new,const double &dydx,const double &y,const double &int_step)
{
	return y+(dydx_new+dydx)*int_step/2;
}
///////////////////////////////////////////////////////////////////////////////
//Integration of Matrix MAT(r,c) 
//
//030424 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
Matrix integrate(Matrix &DYDX_NEW,Matrix &DYDX,Matrix &Y,const double int_step)
{
	int nrow=Y.get_rows();int nrow1=DYDX_NEW.get_rows();int nrow2=DYDX.get_rows();
	int ncol=Y.get_cols();int ncol1=DYDX_NEW.get_cols();int ncol2=DYDX.get_cols();

	if(nrow!=nrow1||nrow!=nrow2)
		{cerr<<" *** Error: incompatible row-dimensions in 'integrate()' *** \n";system("pause");exit(1);}
	if(ncol!=ncol1||ncol!=ncol2)
		{cerr<<" *** Error: incompatible column-dimensions in 'integrate()' *** \n";system("pause");exit(1);}

	Matrix RESULT(nrow,ncol);
	for(int r=0;r<nrow;r++)
		for(int c=0;c<ncol;c++)
			RESULT.assign_loc(r,c,integrate(DYDX_NEW.get_loc(r,c)
			,DYDX.get_loc(r,c),Y.get_loc(r,c),int_step));

	return RESULT;
}
///////////////////////////////////////////////////////////////////////////////
// US Standard Atmosphere 1976
// *Calculates the atmospheric properties density pressure and temperature 
//	up to 85 km.
// *Extrapolation above 71 km and beyond 85 km is carried out from 71 km altitude 
// Ref: Public Domain Aeronautical Software (see Web) Fortran Code
//
// Argument Output:
//					rho=Air density - kg/m^3
//					press= Air static pressure - Pa
//					tempk= Air temperature - degKelvin
// Argument Input:
//					balt= Geometrical altitude above S.L. - m
//
// 030318 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
#include <cmath>

void atmosphere76(double &rho,double &press,double &tempk, const double balt)
{
	double rearth(6369.0); //radius of the earth - km
	double gmr(34.163195); //gas constant
	double rhosl(1.22500); //sea level density - kg/m^3
	double pressl(101325.); //sea level pressure - Pa
	double tempksl(288.15); //sea level temperature - dK

	double htab[8]={0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.852}; //altitude
	double ttab[8]={288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946}; //temperture
	double ptab[8]={1.0, 2.233611e-1, 5.403295e-2, 8.5666784e-3, 1.0945601e-3,
					6.6063531e-4, 3.9046834e-5, 3.68501e-6};  //pressure
	double gtab[8]={-6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0, 0.0};   //temperture gradient

	double delta(0);

	//convert geometric (m) to geopotential altitude (km)
	double alt=balt/1000; 
	double h=alt*rearth/(alt+rearth);

	//binary search determines altitude table entry i below actual altitude 
	int i(0); //offset of first value in table
	int j(7); //offset of last value in table
	for( ; ; ){
	  int k=(i+j)/2;     //integer division
	  if(h<htab[k])
	    j=k;
	  else
	    i=k;
	  if(j<=(i+1))break;
	}

	//normalized temperature 'theta' from table look-up and gradient interpolation
	double tgrad=gtab[i];
	double tbase=ttab[i];
	double deltah=h-htab[i];
	double tlocal=tbase+tgrad*deltah;
	double theta=tlocal/ttab[0]; 
	
	//normalized pressure from hydrostatic equations 
	if(tgrad==0)
	  delta=ptab[i]*exp(-gmr*deltah/tbase);
	else
	  delta=ptab[i]*pow((tbase/tlocal),(gmr/tgrad));

	//normalized density
	double sigma=delta/theta;

	//output
	rho=rhosl*sigma;
	press=pressl*delta;
	tempk=tempksl*theta;
}

