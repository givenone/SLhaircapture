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
#include "cvUtilProCam.h"

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
	gray_codes = new IplImage* [2*(n_cols+n_rows+1)];
	for(int i=0; i<2*(n_cols+n_rows+1); i++)
		gray_codes[i] = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1);
	int step = gray_codes[0]->widthStep/sizeof(uchar);

	// Define first code as a white image.
	cvSet(gray_codes[0], cvScalar(255));
	cvSet(gray_codes[1], cvScalar(0));

	// Define Gray codes for projector columns.
	for(int c=0; c<width; c++){
		for(int i=0; i<2*n_cols; i+=2){
			uchar* data = (uchar*)gray_codes[i+2]->imageData;
			if(i>0)
				data[c] = (((c+col_shift) >> (n_cols-i/2-1)) & 1)^(((c+col_shift) >> (n_cols-i/2)) & 1);
			else
				data[c] = (((c+col_shift) >> (n_cols-i/2-1)) & 1);
			data[c] *= 255;
			for(int r=1; r<height; r++)
				data[r*step+c] = data[c];
			cvSubRS(gray_codes[i+2], cvScalar(255), gray_codes[i+3]); 	
		}
	}

	// Define Gray codes for projector rows.
	for(int r=0; r<height; r++){
		for(int i=0; i<2*n_rows; i+=2){
			uchar* data = (uchar*)gray_codes[i+2*n_cols+2]->imageData;
			if(i>0)
				data[r*step] = (((r+row_shift) >> (n_rows-i/2-1)) & 1)^(((r+row_shift) >> (n_rows-i/2)) & 1);
			else
				data[r*step] = (((r+row_shift) >> (n_rows-i/2-1)) & 1);
			data[r*step] *= 255;
			for(int c=1; c<width; c++)
				data[r*step+c] = data[r*step];	
			cvSubRS(gray_codes[i+2*n_cols+2], cvScalar(255), gray_codes[i+2*n_cols+3]); 	
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

// Decode Gray codes.
int decodeGrayCodes(int proj_width, int proj_height,
					IplImage**& gray_codes /* by cammera */, 
					IplImage*& decoded_cols,
					IplImage*& decoded_rows,
					IplImage*& mask,
					int& n_cols, int& n_rows,
					int& col_shift, int& row_shift, 
					int sl_thresh){

	// Extract width and height of images.
	int cam_width  = gray_codes[0]->width;
	int cam_height = gray_codes[0]->height;

	cout << "cam_width : " << cam_width << endl << "cam_height : " << cam_height << endl;

	// Allocate temporary variables.
	IplImage* gray_1      = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U,  1);
	IplImage* gray_2      = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U,  1);
	IplImage* bit_plane_1 = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U,  1);
	IplImage* bit_plane_2 = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U,  1);
	IplImage* temp        = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U,  1);
	IplImage* thresh_hold = cvCreateImage(cvSize(cam_width, cam_height), IPL_DEPTH_8U,  1);

	// Initialize image mask (indicates reconstructed pixels).
	cvSet(mask, cvScalar(0));

	cvCvtColor(gray_codes[0], gray_1, CV_RGB2GRAY);
	cvCvtColor(gray_codes[1], gray_2, CV_RGB2GRAY);

	cout << 1 << endl;
	cvAbsDiff(gray_1, gray_2, temp);
	cvCmpS(temp, sl_thresh, temp, CV_CMP_GE);
	cout << 1.5 << endl;
	cvOr(temp, mask, mask);

	cout << 2 << endl;
	cvAdd(gray_1, gray_2, thresh_hold);

	cvSet(temp, cvScalar(2));
	cvDiv(thresh_hold, temp, thresh_hold); // mean of 2 images.

	cout << 3 << endl;
	// Decode Gray codes for projector columns.
	cvZero(decoded_rows);
	for(int i=0; i<n_rows; i++){

		// Decode bit-plane and update mask.
		cvCvtColor(gray_codes[i+2], gray_1, CV_RGB2GRAY);
		cvCmp(gray_1, thresh_hold, bit_plane_2, CV_CMP_GE);

		// Convert from gray code to decimal value.
		if(i>0)
			cvXor(bit_plane_1, bit_plane_2, bit_plane_1);
		else
			cvCopy(bit_plane_2, bit_plane_1);
		cvAddS(decoded_rows, cvScalar(pow(2.0,n_rows-i-1)), decoded_rows, bit_plane_1);
	}
	cout << 4 << endl;
	cvSubS(decoded_rows, cvScalar(row_shift), decoded_rows);

	cout << 5 << endl;
	// Decode Gray codes for projector rows.
	cvZero(decoded_cols);
	for(int i=0; i<n_cols; i++){

		// Decode bit-plane and update mask.
		cvCvtColor(gray_codes[2+n_rows+i], gray_1, CV_RGB2GRAY);
		cvCmp(gray_1, thresh_hold, bit_plane_2, CV_CMP_GE);

		// Convert from gray code to decimal value.
		if(i>0)
			cvXor(bit_plane_1, bit_plane_2, bit_plane_1);
		else
			cvCopy(bit_plane_2, bit_plane_1);
		cvAddS(decoded_cols, cvScalar(pow(2.0,n_cols-i-1)), decoded_cols, bit_plane_1);
	}
	cvSubS(decoded_cols, cvScalar(row_shift), decoded_cols);
	cout << 6 << endl;
	// Eliminate invalid column/row estimates.
    // Note: This will exclude pixels if either the column or row is missing or erroneous.
	cvCmpS(decoded_cols, proj_width-1,  temp, CV_CMP_LE);
	cvAnd(temp, mask, mask);
	cvCmpS(decoded_cols, 0,  temp, CV_CMP_GE);
	cvAnd(temp, mask, mask);
	cvCmpS(decoded_rows, proj_height-1, temp, CV_CMP_LE);
	cvAnd(temp, mask, mask);
	cvCmpS(decoded_rows, 0,  temp, CV_CMP_GE);
	cvAnd(temp, mask, mask);
	cvNot(mask, temp);
	cvSet(decoded_cols, cvScalar(NULL), temp);
	cvSet(decoded_rows, cvScalar(NULL), temp);

	// Free allocated resources.
	cvReleaseImage(&gray_1);
	cvReleaseImage(&gray_2);
	cvReleaseImage(&bit_plane_1);
	cvReleaseImage(&bit_plane_2);
	cvReleaseImage(&temp);
	cvReleaseImage(&thresh_hold);
	// Return without errors.
	return 0;
}

// Reconstruct the point cloud and the depth map from a structured light sequence.
int reconstructStructuredLight(struct slParams* sl_params, 
					           struct slCalib* sl_calib,
							   IplImage*& texture_image,
							   IplImage*& gray_decoded_cols, 
							   IplImage*& gray_decoded_rows, 
						       IplImage*& gray_mask,
							   CvMat*&    points,
							   CvMat*&    colors,
							   CvMat*&    depth_map,
							   CvMat*&    mask){
	/* depth map, colors, point, mask를 만들어내는 함수 */

	// Define pointers to various image data elements (for fast pixel access).
	int cam_nelems                 = sl_params->cam_w*sl_params->cam_h;
	int proj_nelems                = sl_params->proj_w*sl_params->proj_h;
	uchar*  background_mask_data   = (uchar*)sl_calib->background_mask->imageData;
	int     background_mask_step   = sl_calib->background_mask->widthStep/sizeof(uchar);
	uchar*  gray_mask_data         = (uchar*)gray_mask->imageData;
	int     gray_mask_step         = gray_mask->widthStep/sizeof(uchar);
	ushort* gray_decoded_cols_data = (ushort*)gray_decoded_cols->imageData;
	int     gray_decoded_cols_step = gray_decoded_cols->widthStep/sizeof(ushort);
	ushort* gray_decoded_rows_data = (ushort*)gray_decoded_rows->imageData;
	int     gray_decoded_rows_step = gray_decoded_rows->widthStep/sizeof(ushort);

	// Create a temporary copy of the background depth map.
	CvMat* background_depth_map = cvCloneMat(sl_calib->background_depth_map);

	// By default, disable all pixels. -> mask가 0일 때는 disable 상태임.
	cvZero(mask);

	// Reconstruct point cloud and depth map.
	for(int r=0; r<sl_params->cam_h; r++){
		for(int c=0; c<sl_params->cam_w; c++){

			// Reconstruct current point, if mask is non-zero.
			if(gray_mask_data[r*gray_mask_step+c]){

				// Reconstruct using either "ray-plane" or "ray-ray" triangulation.
				if(sl_params->mode == 1){

					// Allocate storage for row/column reconstructed points and depths.
					float point_cols[3], point_rows[3];
					float depth_cols, depth_rows;
				
					// Intersect camera ray with corresponding projector column.
					if(sl_params->scan_cols){
						float q[3], v[3], w[4];
						int rc = (sl_params->cam_w)*r+c;
						for(int i=0; i<3; i++){
							q[i] = sl_calib->cam_center->data.fl[i];
							v[i] = sl_calib->cam_rays->data.fl[rc+cam_nelems*i];
						}
						int corresponding_column = gray_decoded_cols_data[r*gray_decoded_cols_step+c];
						for(int i=0; i<4; i++)
							w[i] = sl_calib->proj_column_planes->data.fl[4*corresponding_column+i];
						intersectLineWithPlane3D(q, v, w, point_cols, depth_cols);
					}
					// point와 depth를 triangulation을 이용하여 계산. point는 만나는 점의 위치고, depth는 float 값.

					// Intersect camera ray with corresponding projector row.
					if(sl_params->scan_rows){
						float q[3], v[3], w[4];
						int rc = (sl_params->cam_w)*r+c;
						for(int i=0; i<3; i++){
							q[i] = sl_calib->cam_center->data.fl[i];
							v[i] = sl_calib->cam_rays->data.fl[rc+cam_nelems*i];
						}
						int corresponding_row = gray_decoded_rows_data[r*gray_decoded_rows_step+c];
						for(int i=0; i<4; i++)
							w[i] = sl_calib->proj_row_planes->data.fl[4*corresponding_row+i];
						intersectLineWithPlane3D(q, v, w, point_rows, depth_rows);

						if(point_rows[0] == 0 && point_rows[1] == 0 && point_rows[2] == 0)
						{
						//	cout << c << ' ' << r << ' ' << q[0] << q[1] << q[2] << endl << v[0] << v[1] << v[2] << endl << w[0] << w[1] << w[2] << v << w ;
						}
					}

					// Average points of intersection (if row and column scanning are both enabled).
					// Note: Eliminate any points that differ between row and column reconstructions.
					if( sl_params->scan_cols && sl_params->scan_rows){
						if(abs(depth_cols-depth_rows) < sl_params->dist_reject){
							depth_map->data.fl[sl_params->cam_w*r+c] = (depth_cols+depth_rows)/2;
							for(int i=0; i<3; i++)
								points->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = (point_cols[i]+point_rows[i])/2;
						}
						else
							gray_mask_data[r*gray_mask_step+c] = 0;
					}
					else if(sl_params->scan_cols){
						depth_map->data.fl[sl_params->cam_w*r+c] = depth_cols;
						for(int i=0; i<3; i++)
							points->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = point_cols[i];
					}
					else if(sl_params->scan_rows){
						depth_map->data.fl[sl_params->cam_w*r+c] = depth_rows;
						for(int i=0; i<3; i++)
							points->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = point_rows[i];
					}
					else
						gray_mask_data[r*gray_mask_step+c] = 0;
				}
				else{

					// Reconstruct surface using "ray-ray" triangulation.
					int corresponding_column = gray_decoded_cols_data[r*gray_decoded_cols_step+c];
					int corresponding_row    = gray_decoded_rows_data[r*gray_decoded_rows_step+c];
					float q1[3], q2[3], v1[3], v2[3], point[3], depth = 0;
					int rc_cam  = (sl_params->cam_w)*r+c;
					int rc_proj = (sl_params->proj_w)*corresponding_row+corresponding_column;
					for(int i=0; i<3; i++){
						q1[i] = sl_calib->cam_center->data.fl[i];
						q2[i] = sl_calib->proj_center->data.fl[i];
						v1[i] = sl_calib->cam_rays->data.fl[rc_cam+cam_nelems*i];
						v2[i] = sl_calib->proj_rays->data.fl[rc_proj+proj_nelems*i];
					}
					intersectLineWithLine3D(q1, v1, q2, v2, point);
					for(int i=0; i<3; i++)
						depth += v1[i]*(point[i]-q1[i]);
					depth_map->data.fl[rc_cam] = depth;
					for(int i=0; i<3; i++)
						points->data.fl[rc_cam+cam_nelems*i] = point[i];
				}

				// Assign color using provided texture image.
				// Note: Color channels are ordered as RGB, rather than OpenCV's default BGR.
				uchar* texture_image_data = (uchar*)(texture_image->imageData + r*texture_image->widthStep);
				for(int i=0; i<3; i++)
					colors->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = (float)texture_image_data[3*c+(2-i)]/(float)255.0;

				// Update valid pixel mask (e.g., points will only be saved if valid).
				mask->data.fl[sl_params->cam_w*r+c] = 1;

				// Reject any points outside near/far clipping planes.
				
				if(depth_map->data.fl[sl_params->cam_w*r+c] < sl_params->dist_range[0] ||
				   depth_map->data.fl[sl_params->cam_w*r+c] > sl_params->dist_range[1]){
					gray_mask_data[r*gray_mask_step+c] = 0;
					mask->data.fl[sl_params->cam_w*r+c] = 0;
					depth_map->data.fl[sl_params->cam_w*r+c] = 0;
					for(int i=0; i<3; i++)
						points->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = 0;
					for(int i=0; i<3; i++)
						colors->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = 0;
				}

				// Reject background points.
				// Note: Currently only uses depth to determine foreground vs. background pixels.
				float depth_difference = 
					background_depth_map->data.fl[sl_params->cam_w*r+c] - 
					depth_map->data.fl[sl_params->cam_w*r+c];
				if(depth_difference < sl_params->background_depth_thresh && 
				   gray_mask_data[r*gray_mask_step+c] && 
				   background_mask_data[r*background_mask_step+c]){
					gray_mask_data[r*gray_mask_step+c] = 0;
					mask->data.fl[sl_params->cam_w*r+c] = 0;
					depth_map->data.fl[sl_params->cam_w*r+c] = 0;
					for(int i=0; i<3; i++)
						points->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = 0;
					for(int i=0; i<3; i++)
						colors->data.fl[sl_params->cam_w*r+c+cam_nelems*i] = 0;
				}
			}
		}
	}

	// Release allocated resources.
	cvReleaseMat(&background_depth_map);

	// Return without errors.
	return 0;
}

