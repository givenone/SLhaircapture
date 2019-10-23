// this file include main function
#include "stdafx.h"
#include "cvStructuredLight.hpp"
#include "cvScanProCam.h"
#include "cvUtilProCam.h"
#include "cvCalibrateProCam.h"

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
    sl_calib.cam_intrinsic_calib    = true;
	sl_calib.proj_intrinsic_calib   = true;
	sl_calib.procam_extrinsic_calib = true;
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

//	double center[3] = {0, 0, 0};
//	cvInitMatHeader(sl_calib.proj_center, 3, 3, CV_32FC1, center); this is done in "evaluate pro cam in calibrateprocam.cpp"
	
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
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib.cam_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib.cam_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

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
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib.proj_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib.proj_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

	cvReleaseMat(&r);
	cvReleaseMat(&t);
	cvReleaseMat(&R);

	// precalcuate using calibration data

	evaluateProCamGeometry(&sl_params, &sl_calib); 
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
	
	// display result

//	displayDecodingResults(gray_decoded_cols, gray_decoded_rows, gray_mask, sl_params);

	// reconstruction

	printf("Reconstructing the point cloud and the depth map...\n");
	CvMat *points  = cvCreateMat(3, sl_params.cam_h*sl_params.cam_w, CV_32FC1);
	CvMat *colors  = cvCreateMat(3, sl_params.cam_h*sl_params.cam_w, CV_32FC1);
	CvMat *depth_map = cvCreateMat(sl_params.cam_h, sl_params.cam_w, CV_32FC1);
	CvMat *mask    = cvCreateMat(1, sl_params.cam_h*sl_params.cam_w, CV_32FC1);
	reconstructStructuredLight(&sl_params, &sl_calib, 
							   cam_gray_codes[1],
		                       gray_decoded_cols, gray_decoded_rows, gray_mask,
							   points, colors, depth_map, mask);

	// display depth map
//	displayDepthMap(depth_map, gray_mask, &sl_params);

	// Create output directory (if output enabled).
	char str[1024], outputDir[1024]; int scanindex = 1;
	if(sl_params.save){
		sprintf(outputDir, "%s\\%s\\%0.2d", sl_params.outdir, sl_params.object, scanindex);
		mkdir(dir, 0776);

		printf("Saving the depth map...\n");
		
		IplImage* depth_map_image = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U, 1);
		for(int r=0; r<sl_params.cam_h; r++){
			for(int c=0; c<sl_params.cam_w; c++){
				char* depth_map_image_data = (char*)(depth_map_image->imageData + r*depth_map_image->widthStep);
				if(mask->data.fl[sl_params.cam_w*r+c])
					depth_map_image_data[c] = 
						255-int(255*(depth_map->data.fl[sl_params.cam_w*r+c]-sl_params.dist_range[0])/
							(sl_params.dist_range[1]-sl_params.dist_range[0]));
				else
					depth_map_image_data[c] = 0;
			}
		}
		CvMat* dist_range = cvCreateMat(1, 2, CV_32FC1);
		cvmSet(dist_range, 0, 0, sl_params.dist_range[0]);
		cvmSet(dist_range, 0, 1, sl_params.dist_range[1]);
		char str[1024];
		sprintf(str, "%s\\depth_map.png", outputDir);
		cvSaveImage(str, depth_map_image);
		sprintf(str, "%s\\depth_map_range.xml", outputDir);
		cvSave(str, dist_range);
		cvReleaseImage(&depth_map_image);
		cvReleaseMat(&dist_range);
	}

	// Save the texture map.
	printf("Saving the texture map...\n");
	sprintf(str, "%s\\%s\\%s_%0.2d.png", sl_params.outdir, sl_params.object, sl_params.object, scanindex);
	cvSaveImage(str, cam_gray_codes[0]);

	// Save the point cloud.
	printf("Saving the point cloud...\n");
	sprintf(str, "%s\\%s\\%s_%0.2d.wrl", sl_params.outdir, sl_params.object, sl_params.object, scanindex);
	if(savePointsVRML(str, points, NULL, colors, mask)){
		printf("Scanning was not successful and must be repeated!\n");
		return -1;
	}

	return 0;
    
}