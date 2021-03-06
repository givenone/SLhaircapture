// this file include main function\
//  Reference:
//    Douglas Lanman and Gabriel Taubin
//     "Build Your Own 3D Scanner: 3D Photography for Beginners"
//     ACM SIGGRAPH 2009 Course Notes
//
// Author:
//   SEO JUNWON
//   Seoul National University
//   December 2019
#include "stdafx.h"
#include "cvStructuredLight.hpp"
#include "cvScanProCam.h"
#include "cvUtilProCam.h"
#include "cvCalibrateProCam.h"

#include <sys/stat.h>

using namespace std;
using namespace cv;

void initialize_background(struct slParams *sl_params, struct slCalib *sl_calib)
{
	sl_calib->background_depth_map = cvCreateMat(sl_params->cam_h, sl_params->cam_w, CV_32FC1);
	sl_calib->background_image     = cvCreateImage(cvSize(sl_params->cam_w, sl_params->cam_h), IPL_DEPTH_8U, 3);
	sl_calib->background_mask      = cvCreateImage(cvSize(sl_params->cam_w, sl_params->cam_h), IPL_DEPTH_8U, 1);
	cvSet(sl_calib->background_depth_map, cvScalar(FLT_MAX));
	cvZero(sl_calib->background_image);
	cvSet(sl_calib->background_mask, cvScalar(255));

	cout << "Initialized Background Model" << endl;
}

void config(struct slParams *sl_params, struct slCalib *sl_calib)
{
	int cam_nelems                  = sl_params->cam_w*sl_params->cam_h;
	int proj_nelems                 = sl_params->proj_w*sl_params->proj_h;
    sl_calib->cam_intrinsic_calib    = true;
	sl_calib->proj_intrinsic_calib   = true;
	sl_calib->procam_extrinsic_calib = true;
	sl_calib->cam_intrinsic          = cvCreateMat(3,3,CV_32FC1); 
	sl_calib->cam_distortion         = cvCreateMat(5,1,CV_32FC1); 
	sl_calib->cam_extrinsic          = cvCreateMat(2, 3, CV_32FC1);
	sl_calib->proj_intrinsic         = cvCreateMat(3, 3, CV_32FC1);
	sl_calib->proj_distortion        = cvCreateMat(5, 1, CV_32FC1);
	sl_calib->proj_extrinsic         = cvCreateMat(2, 3, CV_32FC1);
	sl_calib->cam_center             = cvCreateMat(3, 1, CV_32FC1);
	sl_calib->proj_center            = cvCreateMat(3, 1, CV_32FC1);
	sl_calib->cam_rays               = cvCreateMat(3, cam_nelems, CV_32FC1);
	sl_calib->proj_rays              = cvCreateMat(3, proj_nelems, CV_32FC1);
	sl_calib->proj_column_planes     = cvCreateMat(sl_params->proj_w, 4, CV_32FC1);
	sl_calib->proj_row_planes        = cvCreateMat(sl_params->proj_h, 4, CV_32FC1);


// This is the calibration data of 800 x 600 projectors and camera
	float cam_intrinsic[3][3] ={ {6799.891745995248, 0, 1684.663925292664}, {0, 6819.065266606978, 897.2080419838242}, {0, 0, 1}};
	float cam_distortion[5] = { -0.1081234940885747, 1.019777262201034, -0.01496156727613678, 0.007012687213256888, 0};

	for(int i=0; i<3; i++) 
	{
		for(int j=0; j<3; j++)
		{
			CV_MAT_ELEM(*sl_calib->cam_intrinsic, float, i, j) = cam_intrinsic[i][j];
		}
	}
	for(int i=0; i<5; i++) CV_MAT_ELEM(*sl_calib->cam_distortion, float, i, 0) = cam_distortion[i];

	float cam_rotation[9] = {-0.9465088474336322, -0.02181055404986991, -0.3219399034942621, 
		0.03029318697689328, -0.9993127627683679, -0.02136176469851316, 
		-0.3212527424826753, -0.0299716849758128, 0.9465190825055085 
		};
	float cam_translation[3] = {208.4992551331917, -9.241636763329311, 63.17988709955341};

	CvMat* r = cvCreateMat(1, 3, CV_32FC1);
	CvMat* t = cvCreateMat(1, 3, CV_32FC1);
	CvMat* R = cvCreateMat(3, 3, CV_64F);
	cvInitMatHeader(R, 3, 3, CV_32FC1, cam_rotation);
	cvInitMatHeader(t, 1, 3, CV_32FC1, cam_translation);
	cvRodrigues2(R, r, NULL); // rotation vector
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

	float proj_intrinsic[3][3] = { {1583.136672775252, 0, 516.2688867516777},
		{0, 2105.254975139322, 519.7798700367449},
		{0, 0, 1} };
	float proj_distortion[5] = { 0.04757942626277912, -0.4940169148610814, 0.01733058285795064, 0.01007747086733519, 0 };
	
	for(int i=0; i<3; i++) 
	{
		for(int j=0; j<3; j++)
		{
			CV_MAT_ELEM(*sl_calib->proj_intrinsic, float, i, j) = proj_intrinsic[i][j];
		}
	}
	for(int i=0; i<5; i++) CV_MAT_ELEM(*sl_calib->proj_distortion, float, i, 0) = proj_distortion[i];


	float proj_rotation[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};
	float proj_translation[3] = {0, 0, 0};
	cvInitMatHeader(R, 3, 3, CV_32FC1, proj_rotation);
	cvInitMatHeader(t, 1, 3, CV_32FC1, proj_translation);
	cvRodrigues2(R, r, NULL); // rotation vector
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->proj_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->proj_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

	cvReleaseMat(&r);
	cvReleaseMat(&t);
	cvReleaseMat(&R);
/* 
*
*
This is the Calibration data of the 1280 x 720 projector & camera
*
*
	float cam_intrinsic[3][3] ={ {5117.365927251628, 0, 1538.135121459372}, {0, 5116.093143162058, 1054.770061292614}, {0, 0, 1}};
	float cam_distortion[5] = { -0.08890846682500254, 1.230751792208493, 0.0001003592840032126, -0.001586782878333563, 0};

	for(int i=0; i<3; i++) 
	{
		for(int j=0; j<3; j++)
		{
			CV_MAT_ELEM(*sl_calib->cam_intrinsic, float, i, j) = cam_intrinsic[i][j];
		}
	}
	for(int i=0; i<5; i++) CV_MAT_ELEM(*sl_calib->cam_distortion, float, i, 0) = cam_distortion[i];

	float cam_rotation[9] = {-0.9306290660731862, -0.01079867427799931, -0.3658044969838218, 
		0.001625296207708427, -0.9996766576441286, 0.02537594478439811, 
		-0.3659602434582019, 0.02302105113371866, 0.9303457053228715 
		};
	float cam_translation[3] = {236.7497969219404, 3.627605952228119, 23.48881580363077};

	CvMat* r = cvCreateMat(1, 3, CV_32FC1);
	CvMat* t = cvCreateMat(1, 3, CV_32FC1);
	CvMat* R = cvCreateMat(3, 3, CV_64F);
	cvInitMatHeader(R, 3, 3, CV_32FC1, cam_rotation);
	cvInitMatHeader(t, 1, 3, CV_32FC1, cam_translation);
	cvRodrigues2(R, r, NULL); // rotation vector
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->cam_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

	float proj_intrinsic[3][3] = { {964.7284899784138, 0, 558.6978366123717},
		{0, 1924.130089363177, 358.8594595895041},
		{0, 0, 1} };
	float proj_distortion[5] = { 0.07534034442346636, 0.005389851836038228, -0.00009209299757556229, 0.001663718345445374, 0 };
	
	for(int i=0; i<3; i++) 
	{
		for(int j=0; j<3; j++)
		{
			CV_MAT_ELEM(*sl_calib->proj_intrinsic, float, i, j) = proj_intrinsic[i][j];
		}
	}
	for(int i=0; i<5; i++) CV_MAT_ELEM(*sl_calib->proj_distortion, float, i, 0) = proj_distortion[i];


	float proj_rotation[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};
	float proj_translation[3] = {0, 0, 0};
	cvInitMatHeader(R, 3, 3, CV_32FC1, proj_rotation);
	cvInitMatHeader(t, 1, 3, CV_32FC1, proj_translation);
	cvRodrigues2(R, r, NULL); // rotation vector
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->proj_extrinsic, float, 0, i) = (float)cvmGet(r, 0, i);
	for(int i=0; i<3; i++) CV_MAT_ELEM(*sl_calib->proj_extrinsic, float, 1, i) = (float)cvmGet(t, 0, i);

	cvReleaseMat(&r);
	cvReleaseMat(&t);
	cvReleaseMat(&R);
*/


	cout << "read configuration" << endl;
}

void save_codes(int width, int height, IplImage**& proj_gray_codes, int& gray_ncols, int& gray_nrows,
					  int& gray_colshift, int& gray_rowshift)
{
	

	char dir[100], buf[100], buf2[1000];
	sprintf(dir, "%d x %d", width, height);
	mkdir(dir, 0776);

	for(int i=0; i<2*(gray_ncols+gray_nrows+1); i++)
	{
		cout << i << endl;
		sprintf(buf, "%s/code_%d.png", dir, i);
		cvSaveImage(buf, proj_gray_codes[i]);
		cout << i << endl;
		cvReleaseImage(&proj_gray_codes[i]);
		cout << i << endl;
	}

	delete[] proj_gray_codes;
	cout << "normal pattern done" << endl;

	IplImage** proj_gray_codes_S = NULL;
	generateGrayCodes_S(width, height, proj_gray_codes_S, 
		gray_ncols, gray_nrows, gray_colshift, gray_rowshift, 
		true , true);

	cout <<  gray_ncols <<' '<<gray_nrows << endl;
	sprintf(dir, "%d x %d : shifting", width, height);
	mkdir(dir, 0776);

	for(int i=0; i<=(gray_ncols+gray_nrows+12); i++)
	{
		cout << i;
		sprintf(buf2, "%s/code_%d.png", dir, i);
		cvSaveImage(buf2, proj_gray_codes_S[i]);
		cvReleaseImage(&proj_gray_codes_S[i]);
	}

	delete[] proj_gray_codes;
}


int save(struct slParams *sl_params, CvMat *depth_map, CvMat *points, CvMat *mask, CvMat *colors, IplImage **cam_gray_codes)
{
	
	char str[1024], outputDir[1024]; int scanindex = 3;
	if(sl_params->save){

		sprintf(outputDir, "%s", sl_params->outdir);
		cout << "saving into" << outputDir<< endl;
		mkdir(outputDir, 0776);
		sprintf(outputDir, "%s/%s", sl_params->outdir, sl_params->object);
		mkdir(outputDir, 0776);
		
		printf("Saving the depth map...\n");
		
		IplImage* depth_map_image = cvCreateImage(cvSize(sl_params->cam_w, sl_params->cam_h), IPL_DEPTH_8U, 1);
		for(int r=0; r<sl_params->cam_h; r++){
			for(int c=0; c<sl_params->cam_w; c++){
				char* depth_map_image_data = (char*)(depth_map_image->imageData + r*depth_map_image->widthStep);
				if(mask->data.fl[sl_params->cam_w*r+c])
					depth_map_image_data[c] = 
						255-int(255*(depth_map->data.fl[sl_params->cam_w*r+c]-sl_params->dist_range[0])/
							(sl_params->dist_range[1]-sl_params->dist_range[0]));
				else
					depth_map_image_data[c] = 0;
			}
		}
		CvMat* dist_range = cvCreateMat(1, 2, CV_32FC1);
		cvmSet(dist_range, 0, 0, sl_params->dist_range[0]);
		cvmSet(dist_range, 0, 1, sl_params->dist_range[1]);
		char str[1024];

		sprintf(str, "%s/depth_map.png", outputDir);
		cvSaveImage(str, depth_map_image);
		sprintf(str, "%s/depth_map_range.xml", outputDir);
		cvSave(str, dist_range);
		cvReleaseImage(&depth_map_image);
		cvReleaseMat(&dist_range);
	}

	// Save the point cloud.
	printf("Saving the point cloud...\n");
	sprintf(str, "%s/%s/%s_%0.2d.ply", sl_params->outdir, sl_params->object, sl_params->object, scanindex);
	if(savePointsVRML(str, points, NULL, colors, mask)){
		printf("Scanning was not successful and must be repeated!\n");
		return -1;
	}
}

int main(int argc, char* argv[])
{
	if(argc == 1)
	{
		no_shfiting();
	}
	else
	{
		shifting();
	}
	
	return 0;
    
}

void no_shfiting()
{
	cout << "hi" <<endl;

	IplImage** proj_gray_codes = NULL;
	int gray_ncols, gray_nrows, gray_colshift, gray_rowshift;

// input calibration
	struct slParams sl_params; //	configuration
	struct slCalib sl_calib; //	calibration
// Allocate storage for calibration parameters.
	readConfiguration(NULL, &sl_params);	

	int width = sl_params.proj_w; int height = sl_params.proj_h;

	generateGrayCodes(width, height, proj_gray_codes, gray_ncols, gray_nrows, gray_colshift, gray_rowshift, 
		true , true);
	
	config(&sl_params, &sl_calib);

	evaluateProCamGeometry(&sl_params, &sl_calib); 

	cout << "evaluated Projector Camera Geometry" << endl;

	// Initialize background model.

	initialize_background(&sl_params, &sl_calib);
	
	// read imange

	IplImage** cam_gray_codes = NULL; char temp [100];
	cam_gray_codes = new IplImage* [50];
	cout << gray_ncols << ' ' << gray_nrows <<endl;
	for(int i=0; i<2*(gray_ncols+gray_nrows+1); i++)
	{
		sprintf(temp, sl_params.image_format, i);
		//sprintf(temp, "./hair_1280/1280x720/%02d.bmp", i);
		//sprintf(temp, "./hair_800/inverse_pattern/%d.bmp", i);
		//sprintf(temp, "./Face_6mp_01/800x600/%02d.png", i);
		cam_gray_codes[i] = cvLoadImage(temp);
	}		

	cout << "Successfully read all images" << endl;

	// Decode gray codes

	IplImage* gray_decoded_cols = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_16U, 1);
	IplImage* gray_decoded_rows = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_16U, 1);
	IplImage* gray_mask = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U,  1);
	decodeGrayCodes(sl_params.proj_w, sl_params.proj_h,
					cam_gray_codes, // image read by camera 
					gray_decoded_cols, gray_decoded_rows, gray_mask,
					gray_ncols, gray_nrows, 
					gray_colshift, gray_rowshift, 
					sl_params.thresh);
	
	cout << "Succesfully decoded gray codes" << endl;

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

	cout << "finished reconstruction !" << endl;

	// Create output directory and save
	save(&sl_params, depth_map, points, mask, colors, cam_gray_codes);
}

void shifting()
{
	cout << "hi :: this is shifting-based decoding" <<endl;

	IplImage** proj_gray_codes = NULL;
	int gray_ncols, gray_nrows, gray_colshift, gray_rowshift;

	// input calibration
	struct slParams sl_params; //	configuration
	struct slCalib sl_calib; //	calibration
	
	// Allocate storage for calibration parameters.
	readConfiguration(NULL, &sl_params);	

	int width = sl_params.proj_w; int height = sl_params.proj_h;

	generateGrayCodes(width, height, proj_gray_codes, gray_ncols, gray_nrows, gray_colshift, gray_rowshift, 
		true , true);
		
	config(&sl_params, &sl_calib);

	evaluateProCamGeometry(&sl_params, &sl_calib); 

	cout << "evaluated Projector Camera Geometry" << endl;

	// Initialize background model.

	initialize_background(&sl_params, &sl_calib);
	
	// read imange

	IplImage** cam_gray_codes = NULL; char temp [100];
	cam_gray_codes = new IplImage* [2 * gray_ncols + 8];
	cout << gray_ncols << ' ' << gray_nrows <<endl;

	for(int i=0; i<2 * gray_ncols; i++)
	{
		sprintf(temp, sl_params.image_format_S, i);
		cam_gray_codes[i] = cvLoadImage(temp);
	}		

	for(int i= 1; i <= 8 ; i++)
	{
		sprintf(temp, sl_params.shifting_format, i);
		cam_gray_codes[i + 2 * gray_ncols - 1] = cvLoadImage(temp);
	}

	cout << "Successfully read all images" << endl;

	// Decode gray codes

	IplImage* gray_decoded_cols = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_16U, 1);
	IplImage* gray_decoded_rows = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_16U, 1);
	IplImage* gray_mask = cvCreateImage(cvSize(sl_params.cam_w, sl_params.cam_h), IPL_DEPTH_8U,  1);
	
	decodeGrayCodes_S(sl_params.proj_w, sl_params.proj_h,
					cam_gray_codes, // image read by camera 
					gray_decoded_cols, gray_decoded_rows, gray_mask,
					gray_ncols, gray_nrows, 
					gray_colshift, gray_rowshift, 
					sl_params.thresh);
	
	cout << "Succesfully decoded gray codes" << endl;

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

	cout << "finished reconstruction !" << endl;

	// Create output directory (if output enabled).
	save(&sl_params, depth_map, points, mask, colors, cam_gray_codes);
}

