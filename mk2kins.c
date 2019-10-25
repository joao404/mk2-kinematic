/*****************************************************************
* Description: mk2_kinematics.c
*   Kinematics for mk2 typed robots with parallel kinematics
*   Set the params using HAL to fit your robot
*	No Check, if Motor can actually reach given angle 
*
*
*	A-Axis Joint[0] Basicmotor
*	B-Axis Joint[1] Armmotor
*	C-Axis Joint[2] Wristmotor
*	
*
*
* Author: Marcel Maage
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2019 All rights reserved.
*
* Last change:
*******************************************************************
*/

#include "rtapi_math.h"
#include "posemath.h"
#include "kinematics.h"             /* decls for kinematicsForward, etc. */


#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

#define L1 135.0
#define L2 57.0
#define L3 135.0
#define L4 204.0
#define L5 147.0
#define L6 88.5

#define WerkzeugoffsetRadius 41.2
#define WerkzeugoffsetHoehe 0
#define WerkzeugoffsetLinksVersatz 5.0
#define XOffset 80.0 //Hoehe vom Boden bis zum Drehpunkt von B
#define YOffset 80.0 //Hoehe vom Boden bis zum Drehpunkt von B
#define ZOffset 92.8 //Hoehe vom Boden bis zum Drehpunkt von B
//derzeit nicht in Verwendung

//#define iA 2.0 //Uebersetzung
//#define Aoffset 90//45 Grad sind 90 Grad beim Servohorn

#define XR  0.0
#define YR  0.0
#define ZR  0.0

/*
static struct data {
    hal_s32_t joints[EMCMOT_MAX_JOINTS];
} *data;
*/

struct haldata
{
    hal_float_t *l1, *l2, *l3, *l4, *l5, *l6, *wo, *wr, *x0, *y0, *z0, *iA, *iB, *iC;
} *haldata;


bool servo_joints_act[3]={false,false,false};


int kinematicsForward(const double * joint,
                      EmcPose * world,
                      const KINEMATICS_FORWARD_FLAGS * fflags,
                      KINEMATICS_INVERSE_FLAGS * iflags)
{

	double phiA,phiB,phiC;
	double radius,height;



	phiA=joint[0];
	phiB=joint[1];
	phiC=joint[2];
	
	//Change if Servojoint
	if(servo_joints_act[0])
		phiA=45.0-(phiA-90.0)/(double)*(haldata->iA);
	if(servo_joints_act[1])
		phiB=180-phiB;
	if(servo_joints_act[2])
		phiC=90-phiC;
	
	//Grad to Rad
    phiA=phiA/180.0*PM_PI;  
    phiB=phiB/180.0*PM_PI;
    phiC=phiC/180.0*PM_PI;

    
    
    //Calculating Z and Radius

    radius = (double)(*(haldata->l1)) * cos(phiB) + (double)(*(haldata->l5)) * cos( - phiC);
	height = (double)(*(haldata->l1)) * sin(phiB) + (double)(*(haldata->l5)) * sin( - phiC);
	phiA+=tan((double)*(haldata->wo)/((double)*(haldata->wr)+radius));
	radius=sqrt((radius+(double)*(haldata->wr))*(radius+(double)*(haldata->wr)) + (double)*(haldata->wo)*(double)*(haldata->wo));//effective Radius for xy
    //Serial.println(Radius);
    //Serial.println(Hoehe);
    
    world->tran.z = height + WerkzeugoffsetHoehe - (double)*(haldata->z0);
      

    //Calculating X and Y

    //world->tran.x= XR + (radius + (double)*(haldata->wr)) * cos(phiA) - (double)*(haldata->x0);
    //world->tran.y= YR + (radius + (double)*(haldata->wr)) * sin(phiA) - (double)*(haldata->y0);
	world->tran.x= XR + radius * cos(phiA) - (double)*(haldata->x0);
    world->tran.y= YR + radius * sin(phiA) - (double)*(haldata->y0);
	world->a = joint[3];
    world->b = joint[4];
    world->c = joint[5];
    world->u = joint[6];
    world->v = joint[7];
    world->w = joint[8];

	

   
   /* return 0 and exit */
   return 0;
}

int kinematicsInverse(const EmcPose * world,
                      double * joint,
                      const KINEMATICS_INVERSE_FLAGS * iflags,
                      KINEMATICS_FORWARD_FLAGS * fflags)
{
	
	double phiA,phiB,phiC;
	double X,Y,Z;
	
	double radius,height,alpha1,bf_2,alpha2,beta2;
	
	X=world->tran.x + (double)*(haldata->x0);
	Y=world->tran.y + (double)*(haldata->y0);
	Z=world->tran.z + (double)*(haldata->z0);
	
	
	radius = sqrt(X*X+Y*Y-(double)*(haldata->wo)*(double)*(haldata->wo))- (double)*(haldata->wr);
    height = Z-WerkzeugoffsetHoehe;
	
	//Calculating phiA
    phiA=atan2((Y-YR),(X-XR))-atan2((double)*(haldata->wo),radius+(double)*(haldata->wr));
    //Conversion to Grad
    phiA=phiA*180.0/PM_PI;

	//Calulating phiB and phiC
    	
	alpha1 = atan2(height,radius);
    bf_2 = radius*radius + height*height;
    alpha2 = acos((bf_2 + (double)*(haldata->l1)*(double)*(haldata->l1) - (double)*(haldata->l5)*(double)*(haldata->l5))/(2*sqrt(bf_2)*(double)*(haldata->l1)));
    phiB = /*PM_PI/2 -*/ (alpha1 + alpha2);
    beta2 = acos(((double)*(haldata->l1)*(double)*(haldata->l1) + (double)*(haldata->l5)*(double)*(haldata->l5) - bf_2)/(2*(double)*(haldata->l1)*(double)*(haldata->l5)));
    phiC = PM_PI - beta2 - alpha1 - alpha2;
	
	
	 //Conversion to Grad
    phiB=phiB*180.0/PM_PI;
    phiC=phiC*180.0/PM_PI;
	
	
	//Conversion Grad to ServoGrad
	if(servo_joints_act[0])
		phiA=90+(45.0 - phiA)*(double)*(haldata->iA);
	if(servo_joints_act[1])
		phiB=180-phiB;
	if(servo_joints_act[2])
		phiC=90-phiC;
	
	joint[0]=phiA;
	joint[1]=phiB;
	joint[2]=phiC;
	joint[3] = world->a;
    joint[4] = world->b;
    joint[5] = world->c;
    joint[6] = world->u;
    joint[7] = world->v;
    joint[8] = world->w;
	
    return 0;
}

int kinematicsHome(EmcPose * world,
                   double * joint,
                   KINEMATICS_FORWARD_FLAGS * fflags,
                   KINEMATICS_INVERSE_FLAGS * iflags)
{
	*fflags = 0;
    *iflags = 0;
  /* use joints, set world */
  return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
//  return KINEMATICS_FORWARD_ONLY;
  return KINEMATICS_BOTH;
}

static char *servo_joints = "0123";
RTAPI_MP_STRING(servo_joints, "Joints that are Servos");

static int set_servo_joint(void) {
    while(*servo_joints) {
	switch(*servo_joints) {
	    case '0': servo_joints++; servo_joints_act[0]=true; rtapi_print_msg(RTAPI_MSG_INFO,"mk2kins: Joint 0 is Servo\n");
	    case '1': servo_joints++; servo_joints_act[1]=true; rtapi_print_msg(RTAPI_MSG_INFO,"mk2kins: Joint 1 is Servo\n");
	    case '2': servo_joints++; servo_joints_act[2]=true; rtapi_print_msg(RTAPI_MSG_INFO,"mk2kins: Joint 2 is Servo\n");
	    case ' ': case '\t': servo_joints++;
		default: break;
	}
	/*
	rtapi_print_msg(RTAPI_MSG_ERR,
		"trivkins: ERROR: Invalid character '%c' in servo_joints\n",
		*servo_joints);
		return -1;
		*/
		return 0;
    }
    return 0;
}



EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);

int comp_id;

int rtapi_app_main(void) {
    int res=0;
    

    comp_id = hal_init("mk2kins");
    if(comp_id < 0) res = comp_id;

    if(res == 0)
    {
        haldata = hal_malloc(sizeof(struct haldata));
        res = !haldata;
    }

	if (!haldata) goto error;
	
	if((res = hal_pin_float_new("mk2kins.L1", HAL_IO, &(haldata->l1), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("mk2kins.L2", HAL_IO, &(haldata->l2), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("mk2kins.L3", HAL_IO, &(haldata->l3), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("mk2kins.L4", HAL_IO, &(haldata->l4), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("mk2kins.L5", HAL_IO, &(haldata->l5), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.L6", HAL_IO, &(haldata->l6), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.WO", HAL_IO, &(haldata->wo), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.WR", HAL_IO, &(haldata->wr), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.X0", HAL_IO, &(haldata->x0), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.Y0", HAL_IO, &(haldata->y0), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.Z0", HAL_IO, &(haldata->z0), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.iA", HAL_IO, &(haldata->iA), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.iB", HAL_IO, &(haldata->iB), comp_id)) < 0) goto error;
	if((res = hal_pin_float_new("mk2kins.iC", HAL_IO, &(haldata->iC), comp_id)) < 0) goto error;


	//Detecting which axes are Servos
	set_servo_joint();


	/*
	haldata->l1=L1;
	haldata->l2=L2;
	haldata->l3=L3;
	haldata->l4=L4;
	haldata->l5=L5;
	haldata->l6=L6;
	haldata->wo=WO;
	haldata->wr=WR;
	haldata->z0=Z0;
	*/

	/*
    
    haldata = hal_malloc(sizeof(struct haldata));
    if (!haldata) goto error;

    if((res = hal_pin_float_new("pumakins.A2", HAL_IO, &(haldata->a2), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("pumakins.A3", HAL_IO, &(haldata->a3), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("pumakins.D3", HAL_IO, &(haldata->d3), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("pumakins.D4", HAL_IO, &(haldata->d4), comp_id)) < 0) goto error;
    if((res = hal_pin_float_new("pumakins.D6", HAL_IO, &(haldata->d6), comp_id)) < 0) goto error;

    PUMA_A2 = DEFAULT_PUMA560_A2;
    PUMA_A3 = DEFAULT_PUMA560_A3;
    PUMA_D3 = DEFAULT_PUMA560_D3;
    PUMA_D4 = DEFAULT_PUMA560_D4;
    PUMA_D6 = DEFAULT_PUMA560_D6;
	*/
    hal_ready(comp_id);
	return 0;
    
error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
