// this file include main function
#include "stdafx.h"
#include "cvStructuredLight.hpp"
#include "cvScanProCam.h"
#include "cvUtilProCam.h"

#include <sys/stat.h>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{

    cout << "hi" <<endl;

	IplImage** proj_gray_codes = NULL;
	int gray_ncols, gray_nrows;
	int gray_colshift, gray_rowshift;

	int width = 800; int height = 600;
	generateGrayCodes(width /*sl_params->proj_w*/, height /*sl_params->proj_h*/, proj_gray_codes, 
		gray_ncols, gray_nrows, gray_colshift, gray_rowshift, 
		true /*sl_params->scan_cols*/, true /*sl_params->scan_rows*/);

	char dir[100], buf[100], buf2[1000];
	sprintf(dir, "%d x %d", width, height);
	mkdir(dir, 0776);

	for(int i=0; i<(gray_ncols+gray_nrows+1); i++)
	{
		sprintf(buf, "%s/code_%d.png", dir, i);
		cvSaveImage(buf, proj_gray_codes[i]);
		cvReleaseImage(&proj_gray_codes[i]);
	}

	delete[] proj_gray_codes;
// Codes below are for the 
//	IplImage** proj_gray_codes_S = NULL;
//	generateGrayCodes_S(width /*sl_params->proj_w*/, height /*sl_params->proj_h*/, proj_gray_codes_S, 
//		gray_ncols, gray_nrows, gray_colshift, gray_rowshift, 
//		true /*sl_params->scan_cols*/, true /*sl_params->scan_rows*/);
/*
	sprintf(dir, "%d x %d : shifting", width, height);
	mkdir(dir, 0776);

	for(int i=0; i<=(gray_ncols+gray_nrows+12); i++)
	{
		sprintf(buf2, "%s/code_%d.png", dir, i);
		cvSaveImage(buf2, proj_gray_codes_S[i]);
		cvReleaseImage(&proj_gray_codes_S[i]);
	}

	delete[] proj_gray_codes;
*/

	// input calibration

	struct slParams sl_params; //	configuration
	struct slCalib sl_calib; //	calibration
	
	readConfiguration(NULL, &sl_params);	
		// Allocate storage for calibration parameters.
	
	int cam_nelems                  = sl_params.cam_w*sl_params.cam_h;
	int proj_nelems                 = sl_params.proj_w*sl_params.proj_h;
    sl_calib.cam_intrinsic_calib    = false;
	sl_calib.proj_intrinsic_calib   = false;
	sl_calib.procam_extrinsic_calib = false;
	sl_calib.cam_intrinsic          = cvCreateMat(3,3,CV_32FC1); 
	sl_calib.cam_distortion         = cvCreateMat(5,1,CV_32FC1); 
	sl_calib.cam_extrinsic          = cvCreateMat(2, 3, CV_32FC1);
	sl_calib.proj_intrinsic         = cvCreateMat(3, 3, CV_32FC1);
	sl_calib.proj_distortion        = cvCreateMat(5, 1, CV_32FC1);
	sl_calib.proj_extrinsic         = cvCreateMat(2, 3, CV_32FC1);
	sl_calib.cam_center             = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.proj_center            = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.cam_rays               = cvCreateMat(3, cam_nelems, CV_32FC1);
	sl_calib.proj_rays              = cvCreateMat(3, proj_nelems, CV_32FC1);
	sl_calib.proj_column_planes     = cvCreateMat(sl_params.proj_w, 4, CV_32FC1);
	sl_calib.proj_row_planes        = cvCreateMat(sl_params.proj_h, 4, CV_32FC1);

	double cam_intrinsic[3][3] = { {6799.891745995248, 0, 1684.663925292664}, {0, 6819.065266606978, 897.2080419838242}, {0, 0, 1}};
	double cam_distortion[5] = { -0.1081234940885747, 1.019777262201034, -0.01496156727613678, 0.007012687213256888, 0};
	cvInitMatHeader(sl_calib.cam_intrinsic, 3, 3, CV_32FC1, cam_intrinsic);
	cvInitMatHeader(sl_calib.cam_distortion, 5, 1, CV_32FC1, cam_distortion);

	double cam_rotation[9] = {-0.9465088474336322, -0.02181055404986991, -0.3219399034942621, 
		0.03029318697689328, -0.9993127627683679, -0.02136176469851316, 
		-0.3212527424826753, -0.0299716849758128, 0.9465190825055085 
		};
	double cam_translation[3] = {208.4992551331917, -9.241636763329311, 63.17988709955341};

	CvMat* r = cvCreateMat(1, 3, CV_32FC1);
	CvMat* t = cvCreateMat(1, 3, CV_32FC1);
	CvMat* R = cvCreateMat(3, 3, CV_64F);
	cvInitMatHeader(R, 3, 3, CV_32FC1, cam_rotation);
	cvInitMatHeader(r, 1, 3, CV_32FC1, cam_translation);
	cvRodrigues2(R, r, NULL); // rotation vector
	for(int i=0; i<3; i++) CV_MAT_ELEM(sl_calib.cam_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(sl_calib.cam_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

	double proj_intrinsic[3][3] = { {1583.136672775252, 0, 516.2688867516777},
		{0, 2105.254975139322, 519.7798700367449},
		{0, 0, 1} };
	double proj_distortion[5] = { 0.04757942626277912, -0.4940169148610814, 0.01733058285795064, 0.01007747086733519, 0 };

	cvInitMatHeader(sl_calib.proj_intrinsic, 3, 3, CV_32FC1, proj_intrinsic);
	cvInitMatHeader(sl_calib.proj_distortion, 5, 1, CV_32FC1, proj_distortion);

	double proj_rotation[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};
	double proj_translation[3] = {0, 0, 0};
	cvInitMatHeader(R, 3, 3, CV_32FC1, proj_rotation);
	cvInitMatHeader(r, 1, 3, CV_32FC1, proj_translation);
	cvRodrigues2(R, r, NULL); // rotation vector
	for(int i=0; i<3; i++) CV_MAT_ELEM(sl_calib.proj_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(sl_calib.proj_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

/*
	프로젝터가 원점 ! -> no extrinsic value
*/
	// Initialize background model.

	sl_calib.background_depth_map = cvCreateMat(sl_params.cam_h, sl_params.cam_w, CV_32FC1);
	sl_calib.background_image     = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U, 3);
	sl_calib.background_mask      = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U, 1);
	cvSet(sl_calib.background_depth_map, cvScalar(FLT_MAX));
	cvZero(sl_calib.background_image);
	cvSet(sl_calib.background_mask, cvScalar(255));


	// read imange

	IplImage** cam_gray_codes = NULL; char temp [100];
	cam_gray_codes = new IplImage* [2*(gray_ncols+gray_nrows+1)];
	for(int i=0; i<2*(gray_ncols+gray_nrows+1); i++)
	{
		sprintf(temp, "./Face_6mp_01/800x600/%02d.png", i);
		cam_gray_codes[i] = cvLoadImage(temp); //// TODO : read png files & need to modify array size
	}		

	// Decode gray codes

	IplImage* gray_decoded_cols = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_16U, 1);
	IplImage* gray_decoded_rows = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_16U, 1);
	IplImage* gray_mask /* what is this ? */ = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U,  1);
	decodeGrayCodes(sl_params.proj_w, sl_params.proj_h,
					cam_gray_codes /* image read by camera */, 
					gray_decoded_cols, gray_decoded_rows, gray_mask,
					gray_ncols, gray_nrows, 
					gray_colshift, gray_rowshift, 
					sl_params.thresh);
	
	// reconstruction

	printf("Reconstructing the point cloud and the depth map...\n");
	CvMat *points  = cvCreateMat(3, sl_params.cam_h*sl_params.cam_w, CV_32FC1);
	CvMat *colors  = cvCreateMat(3, sl_params.cam_h*sl_params.cam_w, CV_32FC1);
	CvMat *mask    = cvCreateMat(1, sl_params.cam_h*sl_params.cam_w, CV_32FC1);
	reconstructStructuredLight(&sl_params, &sl_calib, 
							   cam_gray_codes[0],
		                       gray_decoded_cols, gray_decoded_rows, sl_calib.background_mask,
							   points, colors, sl_calib.background_depth_map, mask);


/*	// Parse command line arguments.
	printf("[Structured Lighting for 3D Scanning]\n");
	char configFile[1024];
	if(argc == 1)
		strcpy(configFile, "./config.xml");
	else
		strcpy(configFile, argv[1]);

	// Read structured lighting parameters from configuration file.
	struct slParams sl_params;
	FILE* pFile = fopen(configFile, "r");
	if(pFile != NULL){
		fclose(pFile);
		printf("Reading configuration file \"%s\"...\n", configFile);
		readConfiguration(configFile, &sl_params);
	}
	else{
		printf("ERROR: Could not open configuration file \"%s\"!\n", configFile);
		printf("Press any key to exit.\n");
		_getch();
		return -1;
	}

	// Initialize capture from any detected device.
	printf("Initializing camera and projector...\n"); 
	if(sl_params.Logitech_9000){
		printf("Enabling Bayer mode for Logitech QuickCam 9000...\n");
		system("Bayer.exe 1 10 > nul");
	}
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
	if(capture == NULL){
		printf("ERROR: No camera was detected by OpenCV!\n");
		printf("Press any key to exit.\n");
		_getch();
		return -1;
	}

	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH,  sl_params.cam_w);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, sl_params.cam_h);
	IplImage* cam_frame = cvQueryFrame2(capture, &sl_params);
	if(cam_frame == NULL){
		printf("ERROR: No frame was available!\n");
		printf("Press any key to exit.\n");
		_getch();
		return -1;
	}
	
	// Create fullscreen window (for controlling projector display).
	cvNamedWindow("projWindow", CV_WINDOW_AUTOSIZE);
	IplImage* proj_frame = cvCreateImage(cvSize(sl_params.proj_w, sl_params.proj_h), IPL_DEPTH_8U, 3);
	cvSet(proj_frame, cvScalar(0, 0, 0));
	cvShowImage("projWindow", proj_frame);
	cvMoveWindow("projWindow", -sl_params.proj_w-7, -33);
	cvWaitKey(1);
	
	// Create output directory (clear previous scan first).
	printf("Creating output directory (overwrites existing object data)...\n");
	char str[1024];
	_mkdir(sl_params.outdir);
	sprintf(str, "%s\\%s", sl_params.outdir, sl_params.object);
	_mkdir(str);
	sprintf(str, "rd /s /q \"%s\\%s\"", sl_params.outdir, sl_params.object);
	system(str);
	sprintf(str, "%s\\%s", sl_params.outdir, sl_params.object);
	if(_mkdir(str) != 0){
		printf("ERROR: Cannot open output directory!\n");
		printf("Press any key to exit.\n");
		_getch();
		return -1;
	}
	
	// Allocate storage for calibration parameters.
	struct slCalib sl_calib;
	int cam_nelems                  = sl_params.cam_w*sl_params.cam_h;
	int proj_nelems                 = sl_params.proj_w*sl_params.proj_h;
    sl_calib.cam_intrinsic_calib    = false;
	sl_calib.proj_intrinsic_calib   = false;
	sl_calib.procam_extrinsic_calib = false;
	sl_calib.cam_intrinsic          = cvCreateMat(3,3,CV_32FC1);
	sl_calib.cam_distortion         = cvCreateMat(5,1,CV_32FC1);
	sl_calib.cam_extrinsic          = cvCreateMat(2, 3, CV_32FC1);
	sl_calib.proj_intrinsic         = cvCreateMat(3, 3, CV_32FC1);
	sl_calib.proj_distortion        = cvCreateMat(5, 1, CV_32FC1);
	sl_calib.proj_extrinsic         = cvCreateMat(2, 3, CV_32FC1);
	sl_calib.cam_center             = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.proj_center            = cvCreateMat(3, 1, CV_32FC1);
	sl_calib.cam_rays               = cvCreateMat(3, cam_nelems, CV_32FC1);
	sl_calib.proj_rays              = cvCreateMat(3, proj_nelems, CV_32FC1);
	sl_calib.proj_column_planes     = cvCreateMat(sl_params.proj_w, 4, CV_32FC1);
	sl_calib.proj_row_planes        = cvCreateMat(sl_params.proj_h, 4, CV_32FC1);
	
	// Load intrinsic camera calibration parameters (if found).
	char str1[1024], str2[1024];
	sprintf(str1, "%s\\calib\\cam\\cam_intrinsic.xml",  sl_params.outdir);
	sprintf(str2, "%s\\calib\\cam\\cam_distortion.xml", sl_params.outdir);
	if( ((CvMat*)cvLoad(str1) != 0) && ((CvMat*)cvLoad(str2) != 0) ){
		sl_calib.cam_intrinsic  = (CvMat*)cvLoad(str1);
		sl_calib.cam_distortion = (CvMat*)cvLoad(str2);
		sl_calib.cam_intrinsic_calib = true;
		printf("Loaded previous intrinsic camera calibration.\n");
	}
	else
		printf("Camera has not been intrinsically calibrated!\n");

	// Load intrinsic projector calibration parameters (if found);
	sprintf(str1, "%s\\calib\\proj\\proj_intrinsic.xml",  sl_params.outdir);
	sprintf(str2, "%s\\calib\\proj\\proj_distortion.xml", sl_params.outdir);
	if( ((CvMat*)cvLoad(str1) != 0) && ((CvMat*)cvLoad(str2) != 0) ){
		sl_calib.proj_intrinsic  = (CvMat*)cvLoad(str1);
		sl_calib.proj_distortion = (CvMat*)cvLoad(str2);
		sl_calib.proj_intrinsic_calib = true;
		printf("Loaded previous intrinsic projector calibration.\n");
	}
	else
		printf("Projector has not been intrinsically calibrated!\n");

	// Load extrinsic projector-camera parameters (if found).
	sprintf(str1, "%s\\calib\\proj\\cam_extrinsic.xml",  sl_params.outdir);
	sprintf(str2, "%s\\calib\\proj\\proj_extrinsic.xml", sl_params.outdir);
	if( (sl_calib.cam_intrinsic_calib && sl_calib.proj_intrinsic_calib) &&
		( ((CvMat*)cvLoad(str1) != 0) && ((CvMat*)cvLoad(str2) != 0) ) ){
		sl_calib.cam_extrinsic  = (CvMat*)cvLoad(str1);
		sl_calib.proj_extrinsic = (CvMat*)cvLoad(str2);
		sl_calib.procam_extrinsic_calib = true;
		evaluateProCamGeometry(&sl_params, &sl_calib);
		printf("Loaded previous extrinsic projector-camera calibration.\n");
	}
	else
		printf("Projector-camera system has not been extrinsically calibrated!\n");

	// Initialize background model.
	sl_calib.background_depth_map = cvCreateMat(sl_params.cam_h, sl_params.cam_w, CV_32FC1);
	sl_calib.background_image     = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U, 3);
	sl_calib.background_mask      = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U, 1);
	cvSet(sl_calib.background_depth_map, cvScalar(FLT_MAX));
	cvZero(sl_calib.background_image);
	cvSet(sl_calib.background_mask, cvScalar(255));

	// Initialize scan counter (used to index each scan iteration).
	int scan_index = 0;

	// Process user input, until 'ESC' is pressed.
	int cvKey = NULL;
	while(1){

		// Display a black projector image by default.
		cvSet(proj_frame, cvScalar(0, 0, 0));
		cvShowImage("projWindow", proj_frame);
		cvWaitKey(1);

		// Parse keystroke.
		if(cvKey == 27){
			printf("\n> Writing configuration file \"%s\"...\n", configFile);
			writeConfiguration(configFile, &sl_params);
			if(sl_params.Logitech_9000){
				printf("> Disabling Bayer mode for Logitech QuickCam 9000...\n");
				system("Bayer.exe 0 > nul");
			}
			printf("> Exiting application...\n");
			break;
		}
		else if(cvKey == 's'){
			printf("\n> Running scanner (view %d)...\n", ++scan_index);
			runStructuredLight(capture, &sl_params, &sl_calib, scan_index);
			cvKey = NULL;
		}
		else if(cvKey == 'b'){
			printf("\n> Scanning background...\n");
			cvSet(sl_calib.background_depth_map, cvScalar(FLT_MAX));
			cvZero(sl_calib.background_image);
			cvSet(sl_calib.background_mask, cvScalar(255));
			runBackgroundCapture(capture, &sl_params, &sl_calib);
			cvKey = NULL;
		}
		else if(cvKey == 'r'){
			printf("\n> Resetting background...\n");
			cvSet(sl_calib.background_depth_map, cvScalar(FLT_MAX));
			cvZero(sl_calib.background_image);
			cvSet(sl_calib.background_mask, cvScalar(255));
			cvKey = NULL;
		}
		else if(cvKey == 'c'){
			printf("\n> Calibrating camera...\n");
			runCameraCalibration(capture, &sl_params, &sl_calib);
			cvKey = NULL;
		}
		else if(cvKey == 'p'){
			printf("\n> Calibrating projector...\n");
			runProjectorCalibration(capture, &sl_params, &sl_calib, false);
			cvKey = NULL;
		}
		else if(cvKey == 'a'){
			printf("\n> Calibrating camera and projector simultaneously...\n");
			runProjectorCalibration(capture, &sl_params, &sl_calib, true);
			cvKey = NULL;
		}
		else if(cvKey == 'e'){
			printf("\n> Calibrating projector-camera alignment...\n");
			runProCamExtrinsicCalibration(capture, &sl_params, &sl_calib);
			cvKey = NULL;
		}

		// Display prompt.
		if(cvKey == NULL){
			printf("\nPress the following keys for the corresponding functions.\n");
			printf("'S': Run scanner\n");
			printf("'B': Estimate background\n");
			printf("'R': Reset background\n");
			printf("'C': Calibrate camera\n");
			printf("'P': Calibrate projector\n");
			printf("'A': Calibrate camera and projector simultaneously\n");
			printf("'E': Calibrate projector-camera alignment\n");
			printf("'ESC': Exit application\n");
		}

		// Get keystroke.
		cvKey = _getch();
	}

	// Release allocated resources.
	cvReleaseMat(&sl_calib.cam_intrinsic);
	cvReleaseMat(&sl_calib.cam_distortion);
	cvReleaseMat(&sl_calib.cam_extrinsic);
	cvReleaseMat(&sl_calib.proj_intrinsic);
	cvReleaseMat(&sl_calib.proj_distortion);
	cvReleaseMat(&sl_calib.proj_extrinsic);
	cvReleaseMat(&sl_calib.cam_center);
	cvReleaseMat(&sl_calib.proj_center);
	cvReleaseMat(&sl_calib.cam_rays);
	cvReleaseMat(&sl_calib.proj_rays);
	cvReleaseMat(&sl_calib.proj_column_planes);
	cvReleaseMat(&sl_calib.proj_row_planes);
	cvReleaseImage(&proj_frame);
	cvReleaseMat(&sl_calib.background_depth_map);
	cvReleaseImage(&sl_calib.background_image);
	cvReleaseImage(&sl_calib.background_mask);

	// Exit without errors.
	cvReleaseCapture(&capture);
	cvDestroyWindow("projWindow");
	*/
	return 0;
    
}