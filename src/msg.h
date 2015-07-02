#ifndef MSG_H
#define MSG_H
/**** generic, internal msg stuff ****/

//enum msgproto : used to dispatch to/from correct txworkers
// WARNING - adding protos heedlessly will break things (at least pmsg code and txblocks !)
enum msgproto {	MP_ISO=0,	//9141 or 14230
	MP_CAN=1,	//CAN or iso15765
	MP_SCI=2,	//SCI J2610 (not impl)
	MP_J1850=3, //VPW or PWM (not impl)
	MP_INVALID=3
	};


#endif

