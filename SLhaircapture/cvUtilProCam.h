void cvFitPlane(const CvMat* points, float* plane);

void intersectLineWithPlane3D(const float* q, const float* v, const float* w, float* p, float& depth);

void intersectLineWithLine3D(const float* q1, const float* v1, const float* q2, const float* v2, float* p);

int savePointsVRML(char* filename, CvMat* points, CvMat* normals, CvMat* colors, CvMat* mask);

void readConfiguration(const char* filename, struct slParams* sl_params);

