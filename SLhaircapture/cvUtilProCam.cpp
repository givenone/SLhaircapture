// cvUtilProCam.h: Implementations of auxiliary functions for structured lighting.
//
// Overview:
//   This file defines auxiliary functions for implementing structured lighting. Functions
//   include basic operations, such as the base-2 logarithm, as well as geometric algorithms, 
//   including fitting planes to 3D points and intersecting lines with other lines and planes.
//
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

	//printf("%f ",depth);
	//printf("%f\n", depth);
	//printf("%f %f %f, %f %f %f,// %f %f %f,%f\n", q[0], q[1], q[2], v[0], v[1], v[2], w[0], w[1], w[2], w[3]);
	//printf("%f %f %f\n", p[0], p[1], p[2]);
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
	fprintf(pFile, "ply\n");
	fprintf(pFile, "format ascii 1.0\n");
	fprintf(pFile, "comment made by givenone\n");

	if(points != NULL){
		int cnt = 0;
		for(int c=0;c<points->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0) cnt++;
		}
		fprintf(pFile, "element vertex %d\n", cnt);		
		fprintf(pFile, "property float x\nproperty float y\nproperty float z\n");
		fprintf(pFile, "end_header\n");
		for(int c=0; c<points->cols; c++){
			if(mask == NULL || mask->data.fl[c] != 0){
				
				//fprintf(pFile, "a ");
				
				for(int r=0; r<points->rows; r++){
					if(r != 1)
						fprintf(pFile, "    %f ",  points->data.fl[c + points->cols*r]);
					else
						fprintf(pFile, "    %f ", points->data.fl[c + points->cols*r]);
				}
				fprintf(pFile, "\n");
			}
		}
	}

	// Output normals (if provided).
	// Note: Flips normals, for compatibility with Java-based viewer.
/*	if(normals != NULL){
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
	fprintf(pFile, "}\n");*/
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
	strcpy(sl_params->image_format, "./hair_800/inverse_pattern/%d.bmp");
	strcpy(sl_params->image_format_S,"./hair_800/shifting_revised/%d.bmp");
	strcpy(sl_params->shifting_format,"./hair_800/shifting_revised/shift_%d.bmp");

	sl_params->save = true;

	// Read camera parameters.
	sl_params->cam_w         =  3072;
	sl_params->cam_h         =  2048;

	// Read projector parameters.
	sl_params->proj_w      = 800;
	sl_params->proj_h      = 600;


	// Read scanning and reconstruction parameters.

	// **** mode 1이면 ray-plane or ray-ray 2이면 ray-ray
	sl_params->mode                    =  1;      
	sl_params->scan_cols               =  true; 
	sl_params->scan_rows               =  false;

	sl_params->thresh                  = 7;

	sl_params->dist_range[0]           = (float) -850.0;
	sl_params->dist_range[1]           = (float) -600.0;
	sl_params->dist_reject             = (float) 10.0;
	sl_params->background_depth_thresh = (float) 20.0;

	// Enable both row and column scanning, if "ray-ray" reconstruction mode is enabled.
	if(sl_params->mode == 2){
		sl_params->scan_cols = false;
		sl_params->scan_rows = true;
	}
}


void printMat(CvMat* mat)
{
	for(int j=0; j<mat->cols; j++)  printf("%10d",j+1);
	printf("\n");
    for(int i=0; i<mat->rows; i++)
    {
        for(int j=0; j<mat->cols; j++)
        {
            printf("%10.2f",cvGet2D(mat,i,j).val[0]);
        }
		printf("\n");
    }
}