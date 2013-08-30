/////////////////////////////////////////////
// DIAR_Kinect: main.cpp
//
// This is the main file, interfacing the Kinect
// with OpenCV via OpenNI and performing operations.
//
// Andrew Kessler 5/30/13
/////////////////////////////////////////////

#include <opencv\highgui.h>
#include <opencv\cv.h>
#include <iostream>
#include <stdio.h>
#include "data_struct.h"
#include <Windows.h>
#include <tchar.h>

using namespace cv;
using namespace std;


/////////////////////// Constants ///////////////////////////////////

//How many frames without an object can pass before we give up tracking an object.
#define MISSED_COUNT_MAX 7

//Calibration offsets
#define  X_OFFSET   2.133
#define	 Y_OFFSET   0.015
#define  Z_OFFSET   0.149

//max number of objects to be detected in frame
#define  MAX_NUM_OBJECTS 50

//default capture width and height
#define  FRAME_WIDTH   640
#define  FRAME_HEIGHT  480

//minimum and maximum object area
#define MIN_OBJECT_AREA  100
#define MAX_OBJECT_AREA  (FRAME_HEIGHT*FRAME_WIDTH/1.5)

#define X_DEFAULT 0.706
#define Y_DEFAULT 0.266
#define Z_DEFAULT 0.266

#define X2_DEFAULT 10
#define Y2_DEFAULT 10
#define Z2_DEFAULT 10 

//names that will appear at the top of each window
const string windowName3 = "After Morphological Operations";
const string controlWindow = "Controls";


/////////////////// Global Variables ///////////////////////////////

//initial min and max HSV filter values.
//these will be changed using trackbars

typedef struct 
{
	int H_MIN;
	int H_MAX;
	int S_MIN;
	int S_MAX;
	int V_MIN;
	int V_MAX;
	int D_MIN;
	int D_MAX;
	int ID;
} Threshold_t; 


Mat depthMap; //the depth data
Mat bgrImage; //the raw color  feed in RGB
Mat hsvImage;  //the raw color feed in HSV 

Mat depthVisible; //the depth map, scaled so that it provides a visible imasge
Mat depth1Segmented; //the binary mask resulting from depth segmentation
Mat color1Segmented; //the binary mask resulting from color segmentation
Mat thresh1Image; //the binary mask resulting from [(depth mask) AND (color mask)].

Mat depth2Segmented; //the binary mask resulting from depth segmentation
Mat color2Segmented; //the binary mask resulting from color segmentation
Mat thresh2Image; //the binary mask resulting from [(depth mask) AND (color mask)].

VideoCapture capture; //the video capture object
FILE* pFile;

//some boolean variables for different functionality
bool trackObjects = true;
bool useMorphOps = true;
bool tuning = false;

bool tracking1;
bool tracking2;
int missedCount1;
int missedCount2;

double last_x1 = X_DEFAULT;
double last_y1 = Y_DEFAULT;
double last_z1 = Z_DEFAULT;

double last_x2 = X2_DEFAULT;
double last_y2 = Y2_DEFAULT;
double last_z2 = Z2_DEFAULT;

//For shared memory
HANDLE hMapFile;
shared_data* s_mem;

//These store the threshold settings
Threshold_t color1Settings;
Threshold_t color2Settings;

//currently unused...
Mat T; //the transformation matrix

/////////////////// Function Declarations ////////////////////////////////////

//File read/write
void saveSettingsToFile(Threshold_t&, string);
void readSettingsFromFile(Threshold_t&, string);

//Trackbar callback - leave empty
void on_trackbar( int, void* );

//Convert integer to string
string intToString(int);

//Create the windows that have the setting trackbars
void createTrackbars(Threshold_t&);

//Kinect interface functions
void initKinect(void);
void printCameraInfo(void);

//OpenCV Image Manipulation Functions
void grabFrames(void);
void morphOps(Mat &thresh);
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed);
void drawObject(int x, int y, int z, Mat &frame, int colorID);
void drawWindows(void);

//Shared memory functions
int initSharedMemory(void);
void writeToSharedMemory(double, double, double,int);

//Convert pixel x,y,z to meter x,y,z
void convertBetweenFrames(int x, int y, int z, int colorID);



///////////////////////////// Main Function ///////////////////////////////////
int main( int argc, char* argv[] )
{
	
	
	if (initSharedMemory())
	{
		printf("Error: Couldn't open shared memory. \r\n");
		while(1){}
	}
	else
	{
		printf("Succeeded in opening shared memory. \r\n");
	}
	// END SHARED MEMORY SETUP

	color1Settings.ID = 1;
	color2Settings.ID = 2;
	createTrackbars(color1Settings); //create slider bars for HSV filtering
	createTrackbars(color2Settings); //create slider bars for HSV filtering

	initKinect();
   
	
	//ATTEMPT TO LOAD TRANSFORMATION MATRIX
		
	FileStorage fs("conversion_data.xml",FileStorage::READ);
	if (!fs.isOpened())
	{
		cout << "No file conversion_data.xml was found." << endl;
		while(1){}
	}

	fs["T"] >> T;
	cout << "Loaded transformation matrix...\r\n" << T << endl;
	fs.release();

	//END ATTEMPT TO LOAD TRANSFORMATION MATRIX


    while(1)
    {
  
		grabFrames();
		drawWindows();
		string colorStr;
     	//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		char c  = (char) waitKey(30);

		switch (c)
		{
		case '!':
			printf("Color1 settings file name to write >> ");
			getline(cin, colorStr);
			saveSettingsToFile(color1Settings, colorStr);
			break;
		case '@':
			printf("Color2 settings file name to write >> ");
			getline(cin, colorStr);
			saveSettingsToFile(color2Settings, colorStr);
			break;
		case '1':
			printf("Enter color 1 to load >> ");
			getline(cin, colorStr);
			readSettingsFromFile(color1Settings, colorStr);
			printf("Exiting read in from file...\r\n");
			break;
		case '2':
			printf("Enter color 2 to load >> ");
			getline(cin, colorStr);
			readSettingsFromFile(color2Settings, colorStr);
			printf("Exiting read in from file...\r\n");
			break;

		case 'm':
			useMorphOps = !useMorphOps;
			cout << "Morphological operations enabled = " << useMorphOps << endl;
			break;
		case 't':
			trackObjects = !trackObjects;
			cout << "Track objects enabled = " << trackObjects << endl;
			break;
		case 'w':
			tuning = !tuning;
			cout << "Tuning windows shown = " << tuning << endl;
			break;
		case 'q':
			cout << "Quitting!" << endl;
				return 0;
		}
    }

    return 0;
}
/*
 * Takes the current trackbar settings and writes them to a file.
 */
void saveSettingsToFile(Threshold_t &colorSettings, string clrStr)
{
	printf("Saving HSV settings...");

	FileStorage fs(clrStr + "_data.xml",FileStorage::WRITE);

	if (!fs.isOpened())
	{
		cout << "Error creating file " << clrStr << "_data.xml" << "." << endl;
		return;
	}

	fs << "H_MIN" << colorSettings.H_MIN;
	fs << "H_MAX" << colorSettings.H_MAX;

	fs << "S_MIN" << colorSettings.S_MIN;
	fs << "S_MAX" << colorSettings.S_MAX;

	fs << "V_MIN" << colorSettings.V_MIN;
	fs << "V_MAX" << colorSettings.V_MAX;

	fs << "D_MIN" << colorSettings.D_MIN;
	fs << "D_MAX" << colorSettings.D_MAX;

	fs.release();
	printf("done!\r\n");
}

/* 
 * Loads the hsv_data.xml file and writes settings to the trackbars. 
 */
void readSettingsFromFile(Threshold_t &colorSettings, string clrStr)
{
	printf("Reading HSV settings...\r\n");

	FileStorage fs(clrStr + "_data.xml",FileStorage::READ);

	string name = "Color " + intToString(colorSettings.ID) + " Settings";

	if (!fs.isOpened())
	{
		cout << "No file " << clrStr << "_data.xml" << " was found." << endl;
		return;
	}
	cout << "opened file " << clrStr <<"_data.xml...";

	fs["H_MIN"] >> colorSettings.H_MIN;
	fs["H_MAX"] >> colorSettings.H_MAX;

	fs["S_MIN"] >> colorSettings.S_MIN;
	fs["S_MAX"] >> colorSettings.S_MAX;

	fs["V_MIN"] >> colorSettings.V_MIN;
	fs["V_MAX"] >> colorSettings.V_MAX;

	fs["D_MIN"] >> colorSettings.D_MIN;
	fs["D_MAX"] >> colorSettings.D_MAX;
	
	setTrackbarPos("H_MIN",name,colorSettings.H_MIN);
	setTrackbarPos("H_MAX",name,colorSettings.H_MAX);
	setTrackbarPos("S_MIN",name,colorSettings.S_MIN);
	setTrackbarPos("S_MAX",name,colorSettings.S_MAX);
	setTrackbarPos("V_MIN",name,colorSettings.V_MIN);
	setTrackbarPos("V_MAX",name,colorSettings.V_MAX);
	setTrackbarPos("D_MIN",name,colorSettings.D_MIN);
	setTrackbarPos("D_MAX",name,colorSettings.D_MAX);

	fs.release();

	printf("done!\r\n");


}

/* 
 * This function looks for objects in the thresholded image.
 */
void trackFilteredObject(Mat threshold, Mat &cameraFeed, int colorID){

	Mat temp;
	threshold.copyTo(temp);
	bool* tracking_ptr;
	int* missedCount_ptr;
	double* lastX_ptr;
	double* lastY_ptr;
	double* lastZ_ptr;

	int x = 0, y = 0;

	if (colorID == 1)
	{
		tracking_ptr = &tracking1;
		missedCount_ptr = &missedCount1;
		lastX_ptr = &last_x1;
		lastY_ptr = &last_y1;
		lastZ_ptr = &last_z1;
	}
	else
	{
		tracking_ptr = &tracking2;
		missedCount_ptr = &missedCount2;
		lastX_ptr = &last_x2;
		lastY_ptr = &last_y2;
		lastZ_ptr = &last_z2;
	}



	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;

	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) 
	{
		int numObjects = hierarchy.size();
       
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
       if(numObjects<MAX_NUM_OBJECTS)
	   {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) 
			{

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 400px^2 then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
				{
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;

				}
				else 
				{
					objectFound = false;
				}

			}
			//let user know you found an object
			if(objectFound == true)
			{
				int z = (int)  depthMap.at<unsigned short>(Point(x,y));
	
				convertBetweenFrames(x,y,z, colorID);
			

				//putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
				
				//draw object location on screen
				drawObject(x,y,z, cameraFeed, colorID);
			}
		}
	   else 
	   {
			putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
			objectFound = false;
	   }
	}


	if (objectFound)
	{
		(*missedCount_ptr) = 0;
		(*tracking_ptr) = true;

	}
	else //object not found 
	{
		if (*tracking_ptr) //if it was tracking an object
		{
			if ((*missedCount_ptr) < MISSED_COUNT_MAX)
			{
				writeToSharedMemory((*lastX_ptr),(*lastY_ptr),(*lastZ_ptr),colorID);
				(*missedCount_ptr) = (*missedCount_ptr)+ 1;
			}
			else //to many missed tracks
			{
				//printf("Giving up on this object.\r\n");
				(*tracking_ptr) = false;
				writeToSharedMemory(X_DEFAULT,Y_DEFAULT,Z_DEFAULT, colorID);
			}

		}
		else
		{
			//printf("Not tracking - providing defaults. \r\n");
			writeToSharedMemory(X_DEFAULT,Y_DEFAULT,Z_DEFAULT, colorID);
		}
	}

}

/*
 * Callback function for trackbars. Currently unimplemented.
 */
void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}
string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

/* 
 * This function creates the window that has the trackbar that control the HSV 
 * thresholds.
 */
void createTrackbars(Threshold_t& settings){
	//create window for trackbars
	string name = "Color " + intToString(settings.ID) + " Settings";
    namedWindow(name,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];

	sprintf( TrackbarName, "H_MIN", settings.H_MIN);
	sprintf( TrackbarName, "H_MAX", settings.H_MAX);
	sprintf( TrackbarName, "S_MIN", settings.S_MIN);
	sprintf( TrackbarName, "S_MAX", settings.S_MAX);
	sprintf( TrackbarName, "V_MIN", settings.V_MIN);
	sprintf( TrackbarName, "V_MAX", settings.V_MAX);
	sprintf( TrackbarName, "D_MIN", settings.D_MIN);
	sprintf( TrackbarName, "D_MAX", settings.D_MAX);
	
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar( "H_MIN", name, &(settings.H_MIN), 256, on_trackbar );
    createTrackbar( "H_MAX", name, &(settings.H_MAX), 256, on_trackbar );
    createTrackbar( "S_MIN", name, &(settings.S_MIN), 256, on_trackbar );
    createTrackbar( "S_MAX", name, &(settings.S_MAX), 256, on_trackbar );
    createTrackbar( "V_MIN", name, &(settings.V_MIN), 256, on_trackbar );
    createTrackbar( "V_MAX", name, &(settings.V_MAX), 256, on_trackbar );
	createTrackbar( "D_MIN", name, &(settings.D_MIN), 256, on_trackbar );
	createTrackbar( "D_MAX", name, &(settings.D_MAX), 256, on_trackbar );

}

/*
 * This function draws the crosshair symbol over the tracked object, along with the pixel 
 * coordinates as text.
 */
void drawObject(int x, int y, int z, Mat &frame, int colorID){

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!
	
	Scalar clr;
	switch (colorID)
	{
	case 1:
		clr = Scalar(0,255,0);
		break;
	case 2:
		clr = Scalar(0,0,255);
		break;
	}

	circle(frame,Point(x,y),20,clr,2);
	line(frame,Point(x,y-5),Point(x,y-25),clr,2);
	line(frame,Point(x,y+5),Point(x,y+25),clr,2);
	line(frame,Point(x-5,y),Point(x-25,y),clr,2);
	line(frame,Point(x+5,y),Point(x+25,y),clr,2);

	putText(frame,intToString(x)+","+intToString(y)+","+intToString(z),Point(x,y+30),1,1,clr,2);

}

/*
 * This function applies morphological operations to the thresholded image.
 * It runs two erodes to reduce noise, and two dilates to enlarge remaining 
 * white space.
 */
 void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
	


}

/*
 * This function queries the Kinect for some settings, and prints them to the screen.
 */
void printCameraInfo(void)
{

	 // Print some avalible device settings.
    cout << "\nDepth generator output mode:" << endl <<
            "FRAME_WIDTH      " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
            "FRAME_HEIGHT     " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
            "FRAME_MAX_DEPTH  " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
            "FPS              " << capture.get( CV_CAP_PROP_FPS ) << endl <<
            "REGISTRATION     " << capture.get( CV_CAP_PROP_OPENNI_REGISTRATION ) << endl;
    if( capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT ) )
    {
        cout <<
            "\nImage generator output mode:" << endl <<
            "FRAME_WIDTH   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
            "FRAME_HEIGHT  " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
            "FPS           " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;
    }
    else
    {
        cout << "\nDevice doesn't contain image generator." << endl;
    }

}

/* 
 * This function attemps to open the Kinect. If successful, it prints the camera info.
 */
void initKinect()
{
	cout << "Device opening ...";
   
	capture.open( CV_CAP_OPENNI );

    cout << "done." << endl;

    if( !capture.isOpened() )
    {
        cout << "Can not open a capture object with the Kinect. Did you plug it in?" << endl;
		while(1){}
	}
	
	printCameraInfo();
}

/*
 * This function extracts the depth and color images from the most recent camera grab,
 * and applies the necessary thresholding operations. The resulting masks are ANDed 
 * together and then passed through morphological operations to eliminate noise. 
 * Finally, the treated image is passed to an object tracking algorithm.
 */
void grabFrames(void)
{
	
        if( !capture.grab() )
        {
            cout << "Can not grab images." << endl;
        }
        else
        {
			//Capture depth
            capture.retrieve( depthMap, CV_CAP_OPENNI_DEPTH_MAP );
            const float scaleFactor = 0.05f;
            //Convert the depth image to a helpful map
		    depthMap.convertTo( depthVisible, CV_8UC1, scaleFactor );
           
			//Capture the Color Image as RGB
            capture.retrieve( bgrImage, CV_CAP_OPENNI_BGR_IMAGE );
			cvtColor(bgrImage,hsvImage,COLOR_BGR2HSV); //convert frame from BGR to HSV colorspace
			

			////////////////// COLOR 1 //////////////////////////////
			inRange(depthVisible,Scalar(color1Settings.D_MIN),Scalar(color1Settings.D_MAX),depth1Segmented);
		
			//filter HSV image between values and store filtered image to threshold matrix
			inRange(hsvImage,Scalar(color1Settings.H_MIN,color1Settings.S_MIN,color1Settings.V_MIN),Scalar(color1Settings.H_MAX,color1Settings.S_MAX,color1Settings.V_MAX),color1Segmented);
		
			//Combine the two masks.
			bitwise_and(depth1Segmented,color1Segmented,thresh1Image);

			//perform morphological operations on thresholded image to eliminate noise
			if(useMorphOps)
				morphOps(thresh1Image);

			
			//pass in thresholded frame to our object tracking function
			//this function will return the x and y coordinates of the
			//filtered object
			if(trackObjects)
				trackFilteredObject(thresh1Image,bgrImage,1);

			
			////////////////// COLOR 2 //////////////////////////////
			inRange(depthVisible,Scalar(color2Settings.D_MIN),Scalar(color2Settings.D_MAX),depth2Segmented);
		
			//filter HSV image between values and store filtered image to threshold matrix
			inRange(hsvImage,Scalar(color2Settings.H_MIN,color2Settings.S_MIN,color2Settings.V_MIN),Scalar(color2Settings.H_MAX,color2Settings.S_MAX,color2Settings.V_MAX),color2Segmented);
		
			//Combine the two masks.
			bitwise_and(depth2Segmented,color2Segmented,thresh2Image);

			//perform morphological operations on thresholded image to eliminate noise
			if(useMorphOps)
				morphOps(thresh2Image);

			//pass in thresholded frame to our object tracking function
			//this function will return the x and y coordinates of the
			//filtered object
			if(trackObjects)
				trackFilteredObject(thresh2Image,bgrImage,2);
		}
}

/*
 * This function draws image windows to the screen. The 4-in-1 window of threshold
 * images is only drawn if the <tuning> parameter is TRUE. The <tuning> parameter 
 * can be toggled at run-time by pressing the 'w' key.
 */
void drawWindows()
{
	//Draw the images to the screen
	if (tuning)
	{
		Mat all;
		Mat col1, col2, col3;
	
		putText(color1Segmented,"Color 1Thresh.",Point(0,50),1,2,Scalar(255),2);
		putText(thresh1Image,"Combined 1 Thresh.",Point(0,50),1,2,Scalar(255),2);
		putText(depth1Segmented,"Depth 1 Thresh.",Point(0,50),1,2,Scalar(255),2);

		putText(color2Segmented,"Color 2 Thresh.",Point(0,50),1,2,Scalar(255),2);
		putText(thresh2Image,"Combined 2 Thresh.",Point(0,50),1,2,Scalar(255),2);
		putText(depth2Segmented,"Depth 2 Thresh.",Point(0,50),1,2,Scalar(255),2);
		
		vconcat(color1Segmented, color2Segmented, col1);
		vconcat(depth1Segmented, depth2Segmented, col2);
		vconcat(thresh1Image, thresh2Image, col3);

		hconcat(col1,col2, all);
		hconcat(all, col3, all);
		resize(all,all, Size(1280,640));

		imshow("All",all);
	}
		resize(bgrImage,bgrImage, Size(0,0), 1.5, 1.5);
		imshow("Color Image",bgrImage);
}

/*
 * This function writes the passed values x,y,z to shared memory.
 */
void writeToSharedMemory(double x, double y, double z, int colorID)
{

	switch (colorID)
	{
	case 1:
		s_mem->x1 = x;
		s_mem->y1 = y;
		s_mem->z1 = z;
		printf("C1: %+.03f, %+.03f, %+.03f\r\n",x, y, z);
		break;
	case 2:
		if(x == X_DEFAULT) //overrride the default
		{
			x = X2_DEFAULT;
			y = Y2_DEFAULT;
			z = Z2_DEFAULT;
		}
		
		s_mem->x2 = x;
		s_mem->y2 = y;
		s_mem->z2 = z;
	//	printf("C2: %+.03f, %+.03f, %+.03f\r\n",x, y, z);
		break;
	}
	
	
	
}

/* 
 * This function initializes the shared memory between the vision system and
 * the PUMA controller.
 */
int initSharedMemory(void)
{
	// BEGIN SET UP OF SHARED MEMORY
    hMapFile = CreateFileMapping(
                 INVALID_HANDLE_VALUE,   // use paging file
                 NULL,                   // default security 
                 PAGE_READWRITE,         // read/write access
                 0,                      // maximum object size (high-order DWORD) 
                 SM_SIZE,                // maximum object size (low-order DWORD)  
                 SHARED_MEM_NAME);       // name of mapping object
	
	if (hMapFile == NULL) 
    { 
       _tprintf(TEXT("Could not create file mapping object (%d).\n"), 
             GetLastError());
       return 1;
    }
	else
	{
		_tprintf(TEXT("File mapping object created\n"));
	}

	 s_mem = (shared_data*) MapViewOfFile(hMapFile,   // handle to map object
                        FILE_MAP_ALL_ACCESS, // read/write permission
                        0,                   
                        0,                   
                        SM_SIZE); 

	if (s_mem == NULL) 
    { 
      _tprintf(TEXT("Could not map view of file (%d).\n"), 
             GetLastError()); 

       CloseHandle(hMapFile);

      return 1;
    
	}

	return 0;


}

/*
 * This function takes the [x,y,z] coordinates of an object in pixels, and converts it to
 * a real world point in the robot's frame. This is then written to the shared memory.
 */
void convertBetweenFrames(int x, int y, int z, int colorID)
{
	
	double x_mtr, y_mtr, z_mtr;

	//Convert z (depth) to meters
	z_mtr = z/1000.0;
	
	//Convert x & y to meters
	int x_center = x - 320;
	int y_center = y - 240;
	
	x_mtr = (x_center * z_mtr * 12.0 / 17.5 / 320.0 / 1.147);
	y_mtr = (y_center * z_mtr * 12.0 / 17.5 / 240.0 / 1.550 );
 
	//printf("KF: %+.03f %+.03f %+.03f\n\r",x_mtr,y_mtr,z_mtr);

	Mat kf =(Mat_<double>(3,1) << x_mtr, y_mtr, z_mtr);	

	//Convert from Kinect frame to Robot frame
		
	double x_trans = X_OFFSET - z_mtr;
	double y_trans =  x_mtr - Y_OFFSET;
	double z_trans = Z_OFFSET - y_mtr;
	
	if (colorID == 1)
	{
		last_x1 = x_trans;
		last_y1 = y_trans;
		last_z1 = z_trans;
	}
	else
	{
		last_x2 = x_trans;
		last_y2 = y_trans;
		last_z2 = z_trans;
	}


	//This is the write to shared memory...
	writeToSharedMemory(x_trans,y_trans,z_trans, colorID );

}

