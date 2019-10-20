// cvScanProCam.cpp: Functions for structured light scanning.
//
// Overview:
//   This file implements the functions for structured lighting. The current implementation
//   supports a single projector-camera pair. The projector-camera system must be calibrated 
//   prior to running the scanning function. A 3D point cloud and depth map are recovered by
//   optical triangulation. Three scanning modes are implemented, including: (1) encoding only 
//   the projector columns, (2) encoding only the projector rows, (3) encoding both rows and 
//   columns. Two reconstruction methods are implemented, including: (1) "ray-plane" 
//   triangulation and (2) "ray-ray" triangulation. In the former, each optical ray from the 
//   camera is intersected with the corresponding projector column and/or row. In the later,
//   the corresponding optical rays from the camera and projector are intersected; in this 
//   case, the 3D point is assigned as the closest point to the two (generally skewed) rays.
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
#include "cvScanProCam.h"
//#include "cvUtilProCam.h"

// Generate Gray codes.
int generateGrayCodes(int width, int height, 
					  IplImage**& gray_codes, 
					  int& n_cols, int& n_rows,
					  int& col_shift, int& row_shift, 
					  bool sl_scan_cols, bool sl_scan_rows){

	// Determine number of required codes and row/column offsets.
	if(sl_scan_cols){
		n_cols = (int)ceil(log2(width));
		col_shift = (int)floor((pow(2.0,n_cols)-width)/2);
	}
	else{
		n_cols = 0;
		col_shift = 0;
	}
	if(sl_scan_rows){
		n_rows = (int)ceil(log2(height));
		row_shift = (int)floor((pow(2.0,n_rows)-height)/2);
	}
	else{
		n_rows = 0;
		row_shift = 0;
	}	

	// Allocate Gray codes.
	gray_codes = new IplImage* [n_cols+n_rows+1];
	for(int i=0; i<(n_cols+n_rows+1); i++)
		gray_codes[i] = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
	int step = gray_codes[0]->widthStep/sizeof(uchar);

	// Define first code as a white image.
	cvSet(gray_codes[0], cvScalar(255));

	// Define Gray codes for projector columns.
	for(int c=0; c<width; c++){
		for(int i=0; i<n_cols; i++){
			uchar* data = (uchar*)gray_codes[i+1]->imageData;
			if(i>0)
				data[c] = (((c+col_shift) >> (n_cols-i-1)) & 1)^(((c+col_shift) >> (n_cols-i)) & 1);
			else
				data[c] = (((c+col_shift) >> (n_cols-i-1)) & 1);
			data[c] *= 255;
			for(int r=1; r<height; r++)
				data[r*step+c] = data[c];	
		}
	}

	// Define Gray codes for projector rows.
	for(int r=0; r<height; r++){
		for(int i=0; i<n_rows; i++){
			uchar* data = (uchar*)gray_codes[i+n_cols+1]->imageData;
			if(i>0)
				data[r*step] = (((r+row_shift) >> (n_rows-i-1)) & 1)^(((r+row_shift) >> (n_rows-i)) & 1);
			else
				data[r*step] = (((r+row_shift) >> (n_rows-i-1)) & 1);
			data[r*step] *= 255;
			for(int c=1; c<width; c++)
				data[r*step+c] = data[r*step];	
		}
	}

	// Return without errors.
	return 0;
}

// Generate Shifting Gray codes.
int generateGrayCodes_S(int width, int height, 
					  IplImage**& gray_codes, 
					  int& n_cols, int& n_rows,
					  int& col_shift, int& row_shift, 
					  bool sl_scan_cols, bool sl_scan_rows){

	// Determine number of required codes and row/column offsets.
	if(sl_scan_cols){
		n_cols = (int)ceil(log2(width));
		col_shift = (int)floor((pow(2.0,n_cols)-width)/2);
	}
	else{
		n_cols = 0;
		col_shift = 0;
	}
	if(sl_scan_rows){
		n_rows = (int)ceil(log2(height));
		row_shift = (int)floor((pow(2.0,n_rows)-height)/2);
	}
	else{
		n_rows = 0;
		row_shift = 0;
	}	
	// Allocate Gray codes. 
	// First, Second is all white and all black
	int n_cols_s = n_cols - 1; 
	int n_rows_s = n_rows - 1;
	gray_codes = new IplImage* [n_cols+n_rows+1];
	for(int i=0; i<(n_cols_s+n_rows_s+ 16); i++)
		gray_codes[i] = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
	int step = gray_codes[0]->widthStep/sizeof(uchar);

	// Define first code as a white image.
	cvSet(gray_codes[0], cvScalar(255));
	// Define second code as a black image.
	cvSet(gray_codes[1], cvScalar(0));


	// Define Gray codes for projector columns.
	for(int c=0; c<width; c++){
		for(int i=0; i<n_cols_s; i++){
			uchar* data = (uchar*)gray_codes[i+2]->imageData;
			if(i>0)
				data[c] = (((c+col_shift) >> (n_cols-i-1)) & 1)^(((c+col_shift) >> (n_cols-i)) & 1);
			else
				data[c] = (((c+col_shift) >> (n_cols-i-1)) & 1);
			data[c] *= 255;
			for(int r=1; r<height; r++)
				data[r*step+c] = data[c];	
		}
	}
	for(int i=0; i<7; i++)
	{
		for(int c=0; c<width; c++){
			uchar* data = (uchar*)gray_codes[i+n_cols_s+2]->imageData;
			int index = (c + width - 1) % width;
			data[c] = ((uchar*)gray_codes[n_cols_s+1+i] -> imageData)[index];
			for(int r=1; r<height; r++)
				data[r*step + c] = data[c];
		}
	}
	uchar* data = (uchar*)gray_codes[n_cols_s+2]->imageData;

	// Define Gray codes for projector rows.
	for(int r=0; r<height; r++){
		for(int i=0; i<n_rows_s; i++){
			uchar* data = (uchar*)gray_codes[i+n_cols_s+9]->imageData;
			if(i>0)
				data[r*step] = (((r+row_shift) >> (n_rows-i-1)) & 1)^(((r+row_shift) >> (n_rows-i)) & 1);
			else
				data[r*step] = (((r+row_shift) >> (n_rows-i-1)) & 1);
			data[r*step] *= 255;
			for(int c=1; c<width; c++)
				data[r*step+c] = data[r*step];	
		}
	}
	for(int i=0; i<7; i++)
	{
		for(int r=0; r<height; r++){
			uchar* data = (uchar*)gray_codes[i+n_rows_s+n_cols_s+9]->imageData;
			int index = (r + height - 1) % height;
			data[r*step] = ((uchar*)gray_codes[n_cols_s+n_rows_s+8+i] -> imageData)[index*step];
			for(int c=1; c<width; c++)
				data[r*step+c] = data[r*step];
		}
	}
	// Return without errors.
	return 0;
}

// can find other functions in "tutorial" 