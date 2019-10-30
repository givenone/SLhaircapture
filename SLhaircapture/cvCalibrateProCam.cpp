#include "stdafx.h"
#include "cvStructuredLight.hpp"
#include "cvCalibrateProCam.h"
#include "cvUtilProCam.h"


// Evaluate geometry of projector-camera optical rays and planes.
int evaluateProCamGeometry(struct slParams* sl_params, struct slCalib* sl_calib){

	// Check for input errors (no intrinsic or extrinsic calibration, etc.).
	if(!sl_calib->cam_intrinsic_calib || !sl_calib->proj_intrinsic_calib || !sl_calib->procam_extrinsic_calib){
		printf("ERROR: Intrinsic and extrinsic projector-camera calibration must be completed first!\n");
		printf("Projector-camera geometry calculations were not successful and must be repeated.\n");
		return -1;
	}

	// Extract extrinsic calibration parameters.
	CvMat* r                = cvCreateMat(1, 3, CV_32FC1);
	CvMat* cam_rotation     = cvCreateMat(3, 3, CV_32FC1);
	CvMat* cam_translation  = cvCreateMat(3, 1, CV_32FC1);
	CvMat* proj_rotation    = cvCreateMat(3, 3, CV_32FC1);
	CvMat* proj_translation = cvCreateMat(3, 1, CV_32FC1);
	cvGetRow(sl_calib->cam_extrinsic, r, 0);
	cvRodrigues2(r, cam_rotation, NULL);
	for(int i=0; i<3; i++)
		cvmSet(cam_translation, i, 0, cvmGet(sl_calib->cam_extrinsic, 1, i));
	cvGetRow(sl_calib->proj_extrinsic, r, 0);
	cvRodrigues2(r, proj_rotation, NULL);
	for(int i=0; i<3; i++)
		cvmSet(proj_translation, i, 0, cvmGet(sl_calib->proj_extrinsic, 1, i));
	cvReleaseMat(&r);

	// Determine centers of projection.
	// Note: All positions are in coordinate system of the first camera.
	//cvSet(sl_calib->cam_center, cvScalar(0));
	//cvGEMM(proj_rotation, proj_translation, -1, NULL, 0, sl_calib->proj_center, CV_GEMM_A_T);
	//cvGEMM(cam_rotation, sl_calib->proj_center, 1, cam_translation, 1, sl_calib->proj_center, 0);

	// our syste : All positions are in coordinate system of the projector 
	cvSet(sl_calib->proj_center, cvScalar(0));
	cvGEMM(cam_rotation, cam_translation, -1, NULL, 0, sl_calib->cam_center, CV_GEMM_A_T);
	cvGEMM(proj_rotation, sl_calib->cam_center, 1, proj_translation, 1, sl_calib->cam_center, 0);

	// Pre-compute optical rays for each camera pixel.
	int    cam_nelems        = sl_params->cam_w*sl_params->cam_h;
	CvMat* cam_dist_points   = cvCreateMat(cam_nelems, 1, CV_32FC2);
	CvMat* cam_undist_points = cvCreateMat(cam_nelems, 1, CV_32FC2);
	for(int r=0; r<sl_params->cam_h; r++)
		for(int c=0; c<sl_params->cam_w; c++)
			cvSet1D(cam_dist_points, (sl_params->cam_w)*r+c, cvScalar(float(c), float(r)));
	cvUndistortPoints(cam_dist_points, cam_undist_points, sl_calib->cam_intrinsic, sl_calib->cam_distortion, NULL, NULL);

	for(int i=0; i<cam_nelems; i++){
		CvScalar pd = cvGet1D(cam_undist_points, i);
		
		float norm = (float)sqrt(pow(pd.val[0],2)+pow(pd.val[1],2)+1.0);
		sl_calib->cam_rays->data.fl[i]              = (float)pd.val[0]/norm;
		sl_calib->cam_rays->data.fl[i+  cam_nelems] = (float)pd.val[1]/norm;
		sl_calib->cam_rays->data.fl[i+2*cam_nelems] = (float)1.0/norm;
	}
	cvReleaseMat(&cam_dist_points);
	cvReleaseMat(&cam_undist_points);

	// Pre-compute optical rays for each projector pixel.
	int    proj_nelems        = sl_params->proj_w*sl_params->proj_h;
	CvMat* proj_dist_points   = cvCreateMat(proj_nelems, 1, CV_32FC2);
	CvMat* proj_undist_points = cvCreateMat(proj_nelems, 1, CV_32FC2);
	for(int r=0; r<sl_params->proj_h; r++)
		for(int c=0; c<sl_params->proj_w; c++)
			cvSet1D(proj_dist_points, (sl_params->proj_w)*r+c, cvScalar(float(c), float(r)));
	
	cvUndistortPoints(proj_dist_points, proj_undist_points, sl_calib->proj_intrinsic, sl_calib->proj_distortion, NULL, NULL);

	printMat(sl_calib->proj_intrinsic);
	for(int i=0; i<proj_nelems; i++){
		CvScalar pd = cvGet1D(proj_undist_points, i);
		float norm = (float)sqrt(pow(pd.val[0],2)+pow(pd.val[1],2)+1.0);
		//printf("%f %f\n", pd.val[0], pd.val[1]);
		sl_calib->proj_rays->data.fl[i]               = (float)pd.val[0]/norm;
		sl_calib->proj_rays->data.fl[i+  proj_nelems] = (float)pd.val[1]/norm;
		sl_calib->proj_rays->data.fl[i+2*proj_nelems] = (float)1.0/norm;
	}
	cvReleaseMat(&proj_dist_points);
	cvReleaseMat(&proj_undist_points);

	// Rotate projector optical rays into the camera coordinate system.
	CvMat* R = cvCreateMat(3, 3, CV_32FC1);
	cvGEMM(cam_rotation, proj_rotation, 1, NULL, 0, R, CV_GEMM_B_T);
	cvGEMM(R, sl_calib->proj_rays, 1, NULL, 0, sl_calib->proj_rays, 0);
	cvReleaseMat(&R);

	// Evaluate scale factor (to assist in plane-fitting).
	float scale = 0;
	for(int i=0; i<3; i++)
		scale += pow((float)sl_calib->proj_center->data.fl[i],(float)2.0);
	scale = sqrt(scale);

	// Estimate plane equations describing every projector column.
    // Note: Resulting coefficient vector is in camera coordinate system.
	for(int c=0; c<sl_params->proj_w; c++){
		CvMat* points = cvCreateMat(sl_params->proj_h+1, 3, CV_32FC1);
		for(int ro=0; ro<sl_params->proj_h; ro++){
			int ri = (sl_params->proj_w)*ro+c;
			for(int i=0; i<3; i++)
				points->data.fl[3*ro+i] = 
					sl_calib->proj_center->data.fl[i] + scale*sl_calib->proj_rays->data.fl[ri+proj_nelems*i];
		}
		for(int i=0; i<3; i++)
			points->data.fl[3*sl_params->proj_h+i] = sl_calib->proj_center->data.fl[i];
		float plane[4];
		cvFitPlane(points, plane);
		for(int i=0; i<4; i++)
			sl_calib->proj_column_planes->data.fl[4*c+i] = plane[i];
		cvReleaseMat(&points);
	}

	// Estimate plane equations describing every projector row.
    // Note: Resulting coefficient vector is in camera coordinate system.
	for(int r=0; r<sl_params->proj_h; r++){
		CvMat* points = cvCreateMat(sl_params->proj_w+1, 3, CV_32FC1);
		for(int co=0; co<sl_params->proj_w; co++){
			int ri = (sl_params->proj_w)*r+co;
			for(int i=0; i<3; i++)
				points->data.fl[3*co+i] = 
					sl_calib->proj_center->data.fl[i] + scale*sl_calib->proj_rays->data.fl[ri+proj_nelems*i];
		}
		for(int i=0; i<3; i++)
			points->data.fl[3*sl_params->proj_w+i] = sl_calib->proj_center->data.fl[i];
		float plane[4];
		cvFitPlane(points, plane);
		for(int i=0; i<4; i++)
			sl_calib->proj_row_planes->data.fl[4*r+i] = plane[i];
		cvReleaseMat(&points);
	}
	
	// Release allocated resources.
	cvReleaseMat(&cam_rotation);
	cvReleaseMat(&cam_translation);
	cvReleaseMat(&proj_rotation);
	cvReleaseMat(&proj_translation);

	// Return without errors.
	return 0;
}