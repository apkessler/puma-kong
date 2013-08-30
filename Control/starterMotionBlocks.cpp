////////////////////////////////
// startMotionBlocks.cpp
// Team Ducks in A Rho (DIAR)
////////////////////////////////

/////////////////////////// INCLUDES ///////////////////////////////
#include "Scl_control_wrapper\remote_task\CCS225RemoteTask.hpp"
#include "Scl_control_wrapper\PrLib\legacy_PR_matrix\PrMatrix.h"
#include "Scl_control_wrapper\PrLib\legacy_PR_matrix\PrVector.h"
#include "Scl_control_wrapper\PrLib\legacy_PR_matrix\PrVector3.h"
#include <math.h>



// These are the includes for shared memory.
#include <Windows.h>
#include "data_struct.h"


#define SAFETY_SPHERE_RADIUS 0.85

//Zero the X
#define XC_X 0.001
#define XC_Y 0.219
#define XC_Z 0.542
#define XC_W 0.778
#define XC_VX 0.014
#define XC_VY 0.620
#define XC_VZ 0.103

//Zero the Y
#define YC_X -0.102
#define YC_Y 0.000
#define YC_Z 0.558
#define YC_W 0.703
#define YC_VX 0.338
#define YC_VY 0.620
#define YC_VZ 0.087


//Zero the Z
#define ZC_X 0.844
#define ZC_Y 0.052
#define ZC_Z 0.000
#define ZC_W  0.641
#define ZC_VX -0.146
#define ZC_VY 0.724
#define ZC_VZ -0.209

//dead position
#define Q0_DEAD (6.233*3.14/180)
#define Q1_DEAD (37.976*3.14/180)
#define Q2_DEAD (67.516*3.14/180)
#define Q3_DEAD (168.387*3.14/180)
#define Q4_DEAD (-63.514*3.14/180)
#define Q5_DEAD (174.037*3.14/180)

/*
#define Q0_FIRST (155.274*3.14/180)
#define Q1_FIRST (-5.772*3.14/180)
#define Q2_FIRST (-34.68*3.14/180)
#define Q3_FIRST (0.052*3.14/180)
#define Q4_FIRST (95.036*3.14/180)
#define Q5_FIRST (-33.158*3.14/180)
*/

#define Q0_DEFAULT (9.125*3.14/180)
#define Q1_DEFAULT (12.453*3.14/180)
#define Q2_DEFAULT (28.994*3.14/180)
#define Q3_DEFAULT (-10.339*3.14/180)
#define Q4_DEFAULT (50.333*3.14/180)
#define Q5_DEFAULT (5.341*3.14/180)

#define X_DEFAULT 0.706
#define Y_DEFAULT 0.266
#define Z_DEFAULT 0.266

////////////////////// GLOBAL VARIABLES ////////////////////////////
PrVector des_pos (6), des_posOp(7), tempJoint(6), tempOp(7);
float buffer = 1*M_PI/180; //[degrees] This sets the allowable difference between joint pos and desired pos
float bufferOp = 0.001; //This sets the allowable difference between op pos and desired op pos
float bufferQuat = 0.01; //This sets the allowable difference between op quaternians and desired op quaternians
static int MotionIndex =  3;
float t0;
scl::CS225ControlMode t_mode;

float xdes = 0.706;
float ydes = 0.266;
float zdes = 0.266;

float xobst = 10;
float yobst = 10;
float zobst = 10;

float distance_to_obstacle = 10;

float EEx, EEy, EEz; //End effector position

bool initializing = false;
bool paused = false;
//For shared memory
HANDLE hMapFile;
shared_data* s_mem;

//////////////////// FUNCTION PROTOTYPES //////////////////////////
bool toocloseto(void);
int initReadSharedMemory(void);
void readFromSharedMemory(void);
void runaway(void);

/***************************************************************/

/**
 *	
 METHOD TO SET ROBOT IN MOTION:
 bool SCS225Task::setNewMotion(CS225ControlMode mode, PrVector &des_pos)

 \params: mode
			Can be one of the following:
				NJHOLD, JHOLD,
				NJMOVE,	JMOVE,
				NJGOTO,	JGOTO,
				NJTRACK, JTRACK,
				NHOLD,	HOLD,
				NGOTO, GOTO,
				NTRACK, TRACK

		  des_pos
			This is where you set your desired position for this motion. Number of elements is 6 for joint space motion,
			7 for operational space motion.

			CAUTION: All pure angles must be expressed in radians. You can use the predefined symbol M_PI for pi.
 
 \example usage: 
		  PrVector my_vector (7); //set desired values to my_vector
		  data_->setNewMotion(TRACK, my_vector); 

 */

namespace scl
{
	void CCS225RemoteTask::initMotionBlock()
	{

		printf("---------- STARTING MOTION BLOCK ----------\r\n");
		MotionIndex = 4;
		//TODO: Your motion initialization code goes here
		
		data_->vmax = 0.5;
		updateRemoteConstants();
		
		if(initReadSharedMemory())
		{
			printf("Shared memory init failed.\r\n");
		}
		else
		{
			printf("Shared memory init succeeded.\r\n");
		}

		// TODO:
		// GET PUMA-Kong initialized to initial position:
		// Hand pointing up, set desired EE orientation for entire motion block
		/*{
		//Define desired initial position [degrees]:
		des_pos[0] = 0.0*3.14/180.0;	//These are from HW4
		des_pos[1] = -55.0*3.14/180.0;	//These are from HW4
		des_pos[2] = 200.0*3.14/180.0;	//These are from HW4
		des_pos[3] = 0.0*3.14/180.0;	//These are from HW4
		des_pos[4] = -55.0*3.14/180.0;	//These are from HW4
		des_pos[5] = 0.0*3.14/180.0;	//These are from HW4
		
		printf("Setting initial position...\r\n");

		t_mode = NJGOTO;
		data_->setNewMotion(t_mode, des_pos); //Set initial position. Internally set gv.qd to des_pos.
		}*/
		
		//Set desired position:
		des_pos[0] = Q0_DEFAULT;
		des_pos[1] = Q1_DEFAULT;
		des_pos[2] = Q2_DEFAULT;
		des_pos[3] = Q3_DEFAULT;
		des_pos[4] = Q4_DEFAULT;
		des_pos[5] = Q5_DEFAULT;

		initializing = true;

		printf("Setting initial position to default...\n\r");
		
		t_mode = JGOTO;
		
		data_->setNewMotion(t_mode, des_pos); //Set initial position
	
	}

	void CCS225RemoteTask::performNextMotion()
	{	
		float desired_distance;

		switch (MotionIndex)
		{	
			case 0: 
				{
					printf("Loading X calibration point...\n\r");
					//Set desired position:
					des_posOp[0] = XC_X;
					des_posOp[1] = XC_Y;
					des_posOp[2] = XC_Z;
					des_posOp[3] = XC_W;
					des_posOp[4] = XC_VX;
					des_posOp[5] = XC_VY;
					des_posOp[6] = XC_VZ;

					t_mode = GOTO;
					data_->setNewMotion(t_mode, des_posOp); //Set initial position

				}break;

				case 1: 
				{
					printf("Loading Y calibration point...\n\r");

					//Set desired position:
					des_posOp[0] = YC_X;
					des_posOp[1] = YC_Y;
					des_posOp[2] = YC_Z;
					des_posOp[3] = YC_W;
					des_posOp[4] = YC_VX;
					des_posOp[5] = YC_VY;
					des_posOp[6] = YC_VZ;


					t_mode = GOTO;
					data_->setNewMotion(t_mode, des_posOp); //Set initial position

				}break;

				case 2: 
				{
				
					//Set desired position:
					des_posOp[0] = ZC_X;
					des_posOp[1] = ZC_Y;
					des_posOp[2] = ZC_Z;
					des_posOp[3] = ZC_W;
					des_posOp[4] = ZC_VX;
					des_posOp[5] = ZC_VY;
					des_posOp[6] = ZC_VZ;


					printf("Loading Z calibration point...\n\r");

					t_mode = GOTO;
					data_->setNewMotion(t_mode, des_posOp); //Set initial position

				}break;

				case 3:
				{
						//Set desired position:
					des_posOp[0] = X_DEFAULT;
					des_posOp[1] = Y_DEFAULT;
					des_posOp[2] = Z_DEFAULT;
					des_posOp[3] = 0.707;			//this is forward... up is 1,0,0,0
					des_posOp[4] = 0;
					des_posOp[5] = 0.707;
					des_posOp[6] = 0;



					printf("Loading default point...\n\r");

					t_mode = GOTO;
					data_->setNewMotion(t_mode, des_posOp); //Set initial position


				}break;
				case 4: // The main tracking Next Motion
				{
					// Read in target's position
					printf("Going to default\n\r");
					readFromSharedMemory(); //update xdes, ydes, zdes

					//t0 = sutil::CSystemClock::getSimTime();
					
					/*
					// Read in obstacle 1's position
					obs1_x = get_obs1_x();
					obs1_y = get_obs1_y();
					obs1_z = get_obs1_z();

					if ( toocloseto(1) )
					{
						//adjust xdes, ydes, zdes
					}
					*/

					//TODO:
					//Set desired position:
					des_pos[0] = Q0_DEFAULT;
					des_pos[1] = Q1_DEFAULT;
					des_pos[2] = Q2_DEFAULT;
					des_pos[3] = Q3_DEFAULT;
					des_pos[4] = Q4_DEFAULT;
					des_pos[5] = Q5_DEFAULT;

					t_mode = JGOTO;
					data_->setNewMotion(t_mode, des_pos); //Set next position
				}break;
			case 5: // The main tracking Next Motion
				{
					readFromSharedMemory(); //update xdes, ydes, zdes and xobst, yobst, zobst

					if(toocloseto()){
						runaway();
					}
					else
					{	
						// Read in target's position
						printf("In normal operation...\n\r");

						//t0 = sutil::CSystemClock::getSimTime();
	
						//TODO:
						//Set desired position:
						
					}

					desired_distance = sqrt(pow(xdes,2) + pow(ydes,2) + pow(zdes,2));

					if(desired_distance > SAFETY_SPHERE_RADIUS)
					{
						xdes = xdes / (desired_distance / SAFETY_SPHERE_RADIUS);
						ydes = ydes / (desired_distance / SAFETY_SPHERE_RADIUS);
						zdes = zdes / (desired_distance / SAFETY_SPHERE_RADIUS);
					}

					des_posOp[0] = xdes;
					des_posOp[1] = ydes;
					des_posOp[2] = zdes;
					des_posOp[3] = 0.707;			//this is forward... up is 1,0,0,0
					des_posOp[4] = 0;
					des_posOp[5] = 0.707;
					des_posOp[6] = 0;

					t_mode = GOTO;
					data_->setNewMotion(t_mode, des_posOp); //Set next position
				}break;

				case 6: // The main tracking Next Motion
				{
					des_pos[0] = Q0_DEAD;
					des_pos[1] = Q1_DEAD;
					des_pos[2] = Q2_DEAD;
					des_pos[3] = Q3_DEAD;
					des_pos[4] = Q4_DEAD;
					des_pos[5] = Q5_DEAD;

					t_mode = JGOTO;
					data_->setNewMotion(t_mode, des_pos); //Set next position
				}break;
			default:
				{
					printf("ERROR: You shouldn't be here --> all hell broke loose.\r\n");
				}

		} // End of switch (MotionIndex)

	}

	bool CCS225RemoteTask::isMotionComplete(const GlobalVariables &gv)
	{
		bool returnVal = true;
		/*
		static int cycleCounter = 0;
		static unsigned int cycleTimeOut = 50000; // This is how many times isMotionComplete will be called to goto desired position

		// cycleCounter is number of cycles through following code since we last updated desired position
		// it will be used to change trajectory as the target moves in real time.
		cycleCounter ++;
		
		//printf("x:%f, y:%f, z:%f\n\r",gv.xd[0],gv.xd[1],gv.xd[2]);
		*/
		// Make EE position modular
		EEx = gv.x[0];
		EEy = gv.x[1];
		EEz = gv.x[2];
		
	
		if (MotionIndex >= 0 && MotionIndex < 5) //if we're calibrating
		{
			if (initializing)
			{
				initializing = false;
				returnVal = true;
			}
			else if (_kbhit())
			{	
				if (_getch() == 'c')
				{
				printf("Got a key! Moving on. \r\n");
				returnVal = true;
				MotionIndex++;
				}
			
			}
			else
			{
				//printf("Waiting for key...\r\n");
				returnVal = false;
			}
		}

			//// Make EE position modular
			//EEx = gv.x[0];
			//EEy = gv.x[1];
			//EEz = gv.x[2];

			//if(data_->isMotionOpspace(data_->mode))
			//{
			//	tempOp = gv.x - gv.xd; //Set tempOp vector to be the difference of op pos and desired op pos
			//	for (int ii = 0; ii<7; ii++) 
			//	{
			//		if ( ii < 3)
			//		{
			//			printf("gv.xd = %.3f \r\n", gv.xd[2]);
			//			if ( abs(tempOp[ii]) < bufferOp ) 
			//				returnVal = true;
			//			else
			//			{
			//				returnVal = false;
			//				ii = 7;
			//			}
			//		}	
			//	/*
			//	//
			//	//else
			//	//{
			//	//	if ( abs(tempOp[ii]) < bufferQuat ) returnVal = true;
			//	//	else
			//	//	{
			//	//		returnVal = false;
			//	//		printf("ii orient = %d \r\n",ii);
			//	//		ii = 7;
			//	//	}
			//	//
			//	//}
			//	//
			//	*/
			//	} //end FOR
			//}
			//else
			//{
			//	tempJoint = gv.q - gv.qd; //Set temp vector to be the difference of joint pos and desired pos
			//
			//	for (int jj = 0; jj<6; jj++) 
			//	{
			//		if ( abs(tempJoint[jj]) < buffer ) 
			//			returnVal = true;
			//		else
			//		{
			//			returnVal = false;
			//			jj = 6;
			//		}
			//	}
			//}
		else if(MotionIndex == 5) //if we're not calibrating, then just return true
		{
			if (paused)
			{
				returnVal = false; 

				if (_kbhit())
				{
					if (_getch() == 'p')
					{
					printf("----------------- RESUMING ------------------\r\n");
					returnVal = true;
					paused = false;
					}
				}
			}
			else
			{
				returnVal = true;

				if (_kbhit())
				{
					//float a;
					//a = _getch();
					if (_getch() == 'p')
					{
					printf("----------------- PAUSED ------------------\r\n");
					returnVal = false;
					paused = true;
					}
					else
					{
					data_->vmax = 0.2;
					updateRemoteConstants();
					printf("----------------- KONG DIED! ------------------\r\n");
					returnVal = true;
					MotionIndex++;
					}
				}
			}

		}
		/*
		//
	 //   if (cycleCounter > cycleTimeOut)
		////if(gv.curTime > (t0+.1))
		//{
		//	printf("timeout!\n\r");
		//	returnVal = true;
		//	cycleCounter = 0;
		//}

		//printf("Checking motion block, returning true\r\n");
		//returnVal = true;
		*/


		/***********************************************************
		DO NOT CHANGE THE REMAINING CODE
		/***********************************************************/
		
		if(returnVal) 
			performNextMotion();
		
		return returnVal;
	}
}


//  ----------------------------------------------------------------//
//  ---------- CUSTOM FUNCTIONS ------------------------------------//
//  ----------------------------------------------------------------//

bool toocloseto(void)
{
	float obst_min = 0.50;
	bool returnVal = false;
	// Test if end effector is too close to the obstacle
	
	distance_to_obstacle = 0.001 + sqrt( pow((EEx - xobst),2) + pow((EEy - yobst),2) + pow((EEz - zobst),2) );
	if (distance_to_obstacle < obst_min)
	{
		printf("TOO CLOSE!\n\r");
		returnVal = true;
	}
	return returnVal;
}

void runaway(void)
{
	float scale = 0.60;
	xdes = xdes + (scale * (EEx - xobst) / distance_to_obstacle);
	ydes = ydes + (scale * (EEy - yobst) / distance_to_obstacle);
	zdes = zdes + (scale * (EEz - zobst) / distance_to_obstacle);
	return;
}


int initReadSharedMemory(void)
{

	hMapFile = OpenFileMapping(
                FILE_MAP_ALL_ACCESS,   // read/write access
                FALSE,                 // do not inherit the name
                SHARED_MEM_NAME);      // name of mapping object 

	if (hMapFile == NULL)
	{
		printf("Could not open file mapping object (%d).\n",GetLastError());
		return -1;
	} 

	s_mem = (shared_data*) MapViewOfFile(hMapFile,   // handle to map object
                    FILE_MAP_ALL_ACCESS, // read/write permission
                    0,                   
                    0,                   
                    SM_SIZE);

	if (s_mem == NULL) 
	{ 
		printf("Could not map view of file (%d).\n", GetLastError()); 

		CloseHandle(hMapFile);
		return -1;
	}


	return 0;
}


void readFromSharedMemory(void)
{
	//float xtarg,ytarg,ztarg;
	xdes = (float) s_mem->x1;
	ydes = (float) s_mem->y1;
	zdes = (float) s_mem->z1;
	xobst = (float) s_mem->x2;
	yobst = (float) s_mem->y2;
	zobst = (float) s_mem->z2;
	/*if(sqrt(pow(xtarg,2) + pow(ytarg,2) + pow(ztarg,2)) < SAFETY_SPHERE_RADIUS)
	{
		xdes = xtarg;
		ydes = ytarg;
		zdes = ztarg;
		printf("In sphere! Setting based on memory : %.02f, %.02f, %.02f\r\n", xdes,ydes,zdes);		
	}
	else
	{
		printf("Not in sphere. Using the last known position: %.02f, %.02f, %.02f\r\n", xdes,ydes,zdes);
	}*/
}