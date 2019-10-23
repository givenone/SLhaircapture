// cvUtilProCam.h: Implementations of auxiliary functions for structured lighting.
//
// Overview:
//   This file defines auxiliary functions for implementing structured lighting. Functions
//   include basic operations, such as the base-2 logarithm, as well as geometric algorithms, 
//   including fitting planes to 3D points and intersecting lines with other lines and planes.
//
// Details:
//   Please read the SIGGRAPH 2009 course notes for additional details.
//
//     Douglas Lanman and Gabriel Taubin
//     "Build Your Own 3D Scanner: 3D Photography for Beginners"
//     ACM SIGGRAPH 2009 Course Notes
//
// Author:
//   Douglas Lanman
//   Brown University
//   July 2009
#include "stdafx.h"
#include "cvStructuredLight.hpp"
#include "cvUtilProCam.h"

// Fit a hyperplane to a set of ND points.
// Note: Input points must be in the form of an NxM matrix, where M is the dimensionality.
//       This function finds the best-fit plane P, in the least-squares
//       sense, between the points (X,Y,Z). The resulting plane P is described
//       by the coefficient vector W, where W(1)*X + W(2)*Y +W(3)*Z = W(3), for
//       (X,Y,Z) on the plane P.
void cvFitPlane(const CvMat* points, float* plane){

	// Estimate geometric centroid.
	int nrows = points->rows;
	int ncols = points->cols;
	int type  = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for(int c=0; c<ncols; c++){
		for(int r=0; r<nrows; r++)
			centroid->data.fl[c] += points->data.fl[ncols*r+c];
		centroid->data.fl[c] /= nrows;
	}
	
	// Subtract geometric centroid from each point.
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for(int r=0; r<nrows; r++)
		for(int c=0; c<ncols; c++)
			points2->data.fl[ncols*r+c] = points->data.fl[ncols*r+c] - centroid->data.fl[c];
	
	// Evaluate SVD of covariance matrix.
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);
	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T); 
	cvSVD(A, W, NULL, V, CV_SVD_V_T);

	// Assign plane coefficients by singular vector corresponding to smallest singular value.
	plane[ncols] = 0;
	for(int c=0; c<ncols; c++){
		plane[c] = V->data.fl[ncols*(ncols-1)+c];
		plane[ncols] += plane[c]*centroid->data.fl[c];
	}

	// Release allocated resources.
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}

// Find intersection between a 3D plane and a 3D line.
// Note: Finds the point of intersection of a line in parametric form 
//       (i.e., containing a point Q and spanned by the vector V, with 
//       a plane W defined in implicit form. Note, this function does 
//       not handle certain "degenerate" cases, since they do not occur
//       in practice with the structured lighting configuration.
void intersectLineWithPlane3D(const float* q, 
							  const float* v, 
							  const float* w,
							  float* p, 
							  float& depth){

	// Evaluate inner products.
	float n_dot_q = 0, n_dot_v = 0;
	for(int i=0; i<3; i++){
		n_dot_q += w[i]*q[i];
		n_dot_v += w[i]*v[i];
	}

	// Evaluate point of intersection P.
	depth = (w[3]-n_dot_q)/n_dot_v;
	for(int i=0; i<3; i++)
		p[i] = q[i] + depth*v[i];
}

// Find closest point to two 3D lines.
// Note: Finds the closest 3D point between two 3D lines defined in parametric
///      form (i.e., containing a point Q and spanned by the vector V). Note, 
//       this function does not handle certain "degenerate" cases, since they 
//       do not occur in practice with the structured lighting configuration.
void intersectLineWithLine3D(const float* q1, 
							 const float* v1, 
							 const float* q2,
							 const float* v2,
							 float* p){

	// Define intermediate quantities.
	float q12[3], v1_dot_v1 = 0, v2_dot_v2 = 0, v1_dot_v2 = 0, q12_dot_v1 = 0, q12_dot_v2 = 0;
	for(int i=0; i<3; i++){
		q12[i]      =  q1[i]-q2[i];
		v1_dot_v1  +=  v1[i]*v1[i];
		v2_dot_v2  +=  v2[i]*v2[i];
		v1_dot_v2  +=  v1[i]*v2[i];
		q12_dot_v1 += q12[i]*v1[i];
		q12_dot_v2 += q12[i]*v2[i];
	}

	// Calculate scale factors.
	float s, t, denom;
	denom = v1_dot_v1*v2_dot_v2 - v1_dot_v2*v1_dot_v2;
	s =  (v1_dot_v2/denom)*q12_dot_v2 - (v2_dot_v2/denom)*q12_dot_v1;
	t = -(v1_dot_v2/denom)*q12_dot_v1 + (v1_dot_v1/denom)*q12_dot_v2;

	// Evaluate closest point.
	for(int i=0; i<3; i++)
		p[i] = ( (q1[i]+s*v1[i]) + (q2[i]+t*v2[i]) )/2;
}

// Save a VRML-formatted point cloud.
int savePointsVRML(char* filename, 
				   CvMat* points,
				   CvMat* normals,
				   CvMat* colors,
				   CvMat* mask){

	// Open output file and create header.
	FILE* pFile = fopen(filename, "w");
	if(pFile == NULL){
		fprintf(stderr,"ERROR: Cannot open VRML file!\n");
		return -1;
	}
	fprintf(pFile, "#VRML V2.0 utf8\n");
	fprintf(pFile, "Shape {\n");
	fprintf(pFile, " geometry IndexedFaceSet {\n");

	// Output points (i.e., indexed face set vertices).
	// Note: Flip y-component for compatibility with Java-based viewer.
	if(points != NULL){
		fprintf(pFile, "  coord Coordinate {\n");
		fprintf(pFile, "   point [\n");
		for(int c=0; c<points->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
				for(int r=0; r<points->rows; r++){
					if(r != 1)
						fprintf(pFile, "    %f ",  points->data.fl[c + points->cols*r]);
					else
						fprintf(pFile, "    %f ", -points->data.fl[c + points->cols*r]);
				}
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "   ]\n");
		fprintf(pFile, "  }\n");
	}

	// Output normals (if provided).
	// Note: Flips normals, for compatibility with Java-based viewer.
	if(normals != NULL){
		fprintf(pFile, "  normalPerVertex TRUE\n");
		fprintf(pFile, "  normal Normal {\n");
		fprintf(pFile, "   vector [\n");
		for(int c=0; c<normals->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
				for(int r=0; r<normals->rows; r++)
					fprintf(pFile, "    %f ", -normals->data.fl[c + normals->cols*r]);
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "   ]\n");
		fprintf(pFile, "  }\n");
	}

	// Output colors (if provided).
	// Note: Assumes input is an 8-bit RGB color array.
	if(colors != NULL){
		fprintf(pFile, "  colorPerVertex TRUE\n");
		fprintf(pFile, "  color Color {\n");
		fprintf(pFile, "   color [\n");
		for(int c=0; c<colors->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
				for(int r=0; r<colors->rows; r++)
					fprintf(pFile, "    %f ", colors->data.fl[c + colors->cols*r]);
				fprintf(pFile, "\n");
			}
		}
		fprintf(pFile, "   ]\n");
		fprintf(pFile, "  }\n");
	}

	// Create footer and close file.
	fprintf(pFile, " }\n");
	fprintf(pFile, "}\n");
	if(fclose(pFile) != 0){
		printf("ERROR: Cannot close VRML file!\n");
		return -1;
	}

	// Return without errors.
	return 0;
}


// Read XML-formatted configuration file.
void readConfiguration(const char* filename, struct slParams* sl_params){

	// Open file storage for XML-formatted configuration file.
	//CvFileStorage* fs = cvOpenFileStorage(filename, 0, CV_STORAGE_READ);

	// Read output directory and object (or sequence) name.

	strcpy(sl_params->outdir, "output");
	strcpy(sl_params->object, "output");
	sl_params->save = true;

	// Read camera parameters.
	sl_params->cam_w         =  2048;
	sl_params->cam_h         =  3072;

	// Read projector parameters.
	sl_params->proj_w      = 600;
	sl_params->proj_h      = 800;
//	sl_params->proj_invert = false;

	// Read camera and projector gain parameters.
	sl_params->cam_gain  = 50;//cvReadIntByName(fs, m, "camera_gain",     50);
	sl_params->proj_gain = 50;//cvReadIntByName(fs, m, "projector_gain",  50);
/*	
	// Read distortion model parameters.
	m = cvGetFileNodeByName(fs, 0, "distortion_model");
	sl_params->cam_dist_model[0]  = (cvReadIntByName(fs, m, "enable_tangential_camera",          0) != 0);
	sl_params->cam_dist_model[1]  = (cvReadIntByName(fs, m, "enable_6th_order_radial_camera",    0) != 0);
	sl_params->proj_dist_model[0] = (cvReadIntByName(fs, m, "enable_tangential_projector",       0) != 0);
	sl_params->proj_dist_model[1] = (cvReadIntByName(fs, m, "enable_6th_order_radial_projector", 0) != 0);

	// Read camera calibration chessboard parameters.
	m = cvGetFileNodeByName(fs, 0, "camera_chessboard");
	sl_params->cam_board_w    =        cvReadIntByName(fs,  m, "interior_horizontal_corners",    8);
	sl_params->cam_board_h    =        cvReadIntByName(fs,  m, "interior_vertical_corners",      6);
	sl_params->cam_board_w_mm = (float)cvReadRealByName(fs, m, "square_width_mm",             30.0);
	sl_params->cam_board_h_mm = (float)cvReadRealByName(fs, m, "square_height_mm",            30.0);
*/	
	// Read scanning and reconstruction parameters.

	// **** mode 1이면 ray-plane or ray-ray 2이면 ray-ray
	sl_params->mode                    =  2;      // cvReadIntByName(fs,  m, "mode",                               2);
	sl_params->scan_cols               =  true; //      (cvReadIntByName(fs,  m, "reconstruct_columns",                1) != 0);
	sl_params->scan_rows               =  true; //      (cvReadIntByName(fs,  m, "reconstruct_rows",                   1) != 0);

	sl_params->thresh                  = 32;

	sl_params->dist_range[0]           = (float) 0.0; //cvReadRealByName(fs, m, "minimum_distance_mm",              0.0);
	sl_params->dist_range[1]           = (float) 1.0e4;//cvReadRealByName(fs, m, "maximum_distance_mm",            1.0e4);
	sl_params->dist_reject             = (float) 10.0; //cvReadRealByName(fs, m, "maximum_distance_variation_mm",   10.0);
	sl_params->background_depth_thresh = (float) 20; //cvReadRealByName(fs, m, "minimum_background_distance_mm",  20.0);

	// Enable both row and column scanning, if "ray-ray" reconstruction mode is enabled.
	if(sl_params->mode == 2){
		sl_params->scan_cols = true;
		sl_params->scan_rows = true;
	}
}

