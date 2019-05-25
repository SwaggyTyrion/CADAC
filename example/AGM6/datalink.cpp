///////////////////////////////////////////////////////////////////////////////
//Definition of 'datalink' module-variables 
//Member function of class 'Missile'
//Module-variable locations are assigned to missile[750-774]
// 
//Defining and initializing module-variables
// includes also target track file downloaded from 'combus'
// 
//070523 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

using namespace std;

///////////////////////////////////////////////////////////////////////////////
void Missile::def_datalink()
{
	//Definition and initialization of module-variables
	missile[750].init("mnav","int",0,"=0:no transmission; =3:data transmission - ND","datalink","out","");
    missile[751].init("STCEL",0,0,0,"Measured target coordinates - m","datalink","out","");
    missile[752].init("VTCEL",0,0,0,"Measured target velocity - m/s","datalink","out","");
    missile[753].init("tgt_pos",0,"Target position wrt to reference point E - m","datalink","save","");
    missile[754].init("SAEL",0,0,0,"Aircraft position - m","datalink","out","");
    missile[755].init("VAEL",0,0,0,"Aircraft velocity - m/s","datalink","out","");
}	
///////////////////////////////////////////////////////////////////////////////
//Datalink module
//Member function of class 'Missile'
//
// (1) The target track file is subscribed from 'combus'. It must be located in
//	   the 'combus Packet' at the following offsets: STCELx @ 2x+5 and VTCEL1 @ 2x+6,
//	   where x = 1,2,3,4,5 are the five track files published by 'Aircraft' on the 'combus Packet'
//     (x is currently limited to 5 targets; additional targets do not produce a track file). 
//     Note, x = tgt_num, the target tail number in the sequence established by 'input.asc'. 
//     'tgt_num' identifies to the MISSILE6 object in 'input.asc' the target to be attacked. 
// (2) 'Aircraft' publishes  continuously the target tracking files. The datalink of 
//     'this' missile receives the track file of the target identified by 'tgt_num'.
//     When 'Aircraft' takes a new fix, the data link recognizes that the position of 
//     the target has changed and alerts the Guidance Module  by sending out the mnav=3
//     flag, which, once received, is reset to zero by the 'guidance' module 
//
//070523 Created by Peter H Zipfel
//070914 Added aircraft position and velocity to datalink, PZi
///////////////////////////////////////////////////////////////////////////////
void Missile::datalink(Packet *combus,int num_vehicles)
{
	//local variables
	Variable *data_t=NULL;
	double tgt_pos_new(0);

	//local module-variables
	int mnav=0;

	//localizing module-variables
	//input data
	int tgt_num=missile[1].integer();
	//getting saved value
	Matrix STCEL=missile[751].vec();
	Matrix VTCEL=missile[752].vec();
	Matrix SAEL=missile[754].vec();
	Matrix VAEL=missile[755].vec();
	double tgt_pos=missile[753].real();
	//from other modules
	double time=flat6[0].real();// for debugging
	//-------------------------------------------------------------------------
	//downloading from 'combus' the target track file
	string aircft_id="a1";

	//finding slot 'i' of aircraft in 'combus' (same as in vehicle_list)
	for(int i=0;i<num_vehicles;i++)
	{
		string id=combus[i].get_id();
		if (id==aircft_id)
		{						
			//downloading track file of target identified by 'tgt_num'
			data_t=combus[i].get_data();
			STCEL=data_t[2*tgt_num+5].vec();
			VTCEL=data_t[2*tgt_num+6].vec();

			//downloading aircraft position and velocity
			SAEL=data_t[2].vec();
			VAEL=data_t[3].vec();
		}
		//datalink sets update flag (mnav=3) if target position has changed
		tgt_pos_new=STCEL.absolute();
		if(fabs(tgt_pos-tgt_pos_new)>EPS){
			mnav=3;
			tgt_pos=tgt_pos_new;
		}
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving value
	missile[753].gets(tgt_pos);
	//output to other modules
	missile[750].gets(mnav);
	missile[751].gets_vec(STCEL); 
	missile[752].gets_vec(VTCEL);
	missile[754].gets_vec(SAEL); 
	missile[755].gets_vec(VAEL);
}
