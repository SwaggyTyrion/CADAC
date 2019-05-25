///////////////////////////////////////////////////////////////////////////////
//FILE: 'datalink.cpp'
//
//Contains 'datalink' module of class 'Hyper'
//
//040518 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

#include "class_hierarchy.hpp"

///////////////////////////////////////////////////////////////////////////////
//Definition of 'datalink' module-variables 
//Member function of class 'Hyper'
//Module-variable locations are assigned to hyper[350-399]
// 
//Defining and initializing module-variables
// includes also satellite track file downloaded from 'combus'
// 
//030207 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::def_datalink()
{
	//Definition and initialization of module-variables
	hyper[1].init("sat_num","int",0,"Satellite tail # intercepted by 'this' hyper vehicle","combus","data","");
	hyper[351].init("mnav","int",0,"=0:no transmission; =3:data transmission - ND","datalink","out","");
    hyper[352].init("STCII",0,0,0,"Measured satellite coordinates - m","datalink","out","");
    hyper[353].init("VTCII",0,0,0,"Measured satellite velocity - m/s","datalink","out","");
    hyper[354].init("tgt_pos",0,"Target position wrt to reference point E - m","datalink","save","");
}	

///////////////////////////////////////////////////////////////////////////////
//Datalink module
//Member function of class 'Hyper'
//
// (1) The satellite track file is subscribed from 'combus'. It must be located in
//	   the 'combus Packet' at the following offsets: STCIIx @ 2x+1 and VTCII1 @ 2x+2,
//	   where x = 1,2,3,4,5 are the five track files published by 'Radar' on the 'combus Packet'
//     (x is currently limited to 5 targets; additional targets do not produce a track file). 
//     Note, x = sat_num, the satellite number in the sequence established by 'input.asc'. 
//     'sat_num' identifies to the HYPER6 object in 'input.asc' the satellite to be intercepted. 
// (2) 'Radar' publishes  continuously the satellite tracking files. The datalink of 
//     'this' hyper receives the track file of the satellite identified by 'sat_num'.
//     When 'Radar' takes a new fix, the data link recognizes that the position of 
//     the satellite has changed and alerts the guidance module  by sending out the mnav=3
//     flag, which, once received, is reset to zero by the guidance module 
//
//040518 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////

void Hyper::datalink(Packet *combus,int num_vehicles)
{
	//local variables
	Variable *data_t=NULL;
	double tgt_pos_new(0);

	//local module-variables
	int mnav(0);
	//localizing module-variables
	//input data
	int sat_num=hyper[1].integer();
	//getting saved value
	Matrix STCII=hyper[352].vec();
	Matrix VTCII=hyper[353].vec();
	double tgt_pos=hyper[354].real();
	//from other modules
	double time=round6[0].real();
	int mguid=hyper[400].integer();
	//-------------------------------------------------------------------------
	//downloading from 'combus' the satellite track file
	string radar_id="r1";

	//finding slot 'i' of radar in 'combus' (same as in vehicle_list)
	for(int i=0;i<num_vehicles;i++)
	{
		string id=combus[i].get_id();
		if (id==radar_id)
		{						
			//downloading track file of satellite identified by 'sat_num'
			data_t=combus[i].get_data();
			STCII=data_t[2*sat_num+1].vec();
			VTCII=data_t[2*sat_num+2].vec();
		}
		//datalink sets update flag (mnav=3) if satellite position has changed
		tgt_pos_new=STCII.absolute();
		if(fabs(tgt_pos-tgt_pos_new)>EPS){
			mnav=3;
			tgt_pos=tgt_pos_new;
		}
	}
	//-------------------------------------------------------------------------
	//loading module-variables
	//saving value
	hyper[354].gets(tgt_pos);
	//output to other modules
	hyper[351].gets(mnav);
	hyper[352].gets_vec(STCII); 
	hyper[353].gets_vec(VTCII);
}
