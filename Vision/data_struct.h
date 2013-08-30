#ifndef DATA_STRUCT
#define DATA_STRUCT

struct shared_data
{
	double x1;
	double y1;
	double z1;

	
	double x2;
	double y2;
	double z2;
};

#define SM_SIZE sizeof(shared_data)
#define SHARED_MEM_NAME TEXT("Shared_Mem")


#endif