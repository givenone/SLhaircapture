# SLhaircapture 2019 fall SNU 창의적통합설계 with Morpheus 3D

# How to use

- to install
```bash
. make
```

- to run normal version of scanner
```bash
./scanner 
```
- to run shifting version of scanner
```bash
./scanner shifting
```

# Things you should change

- [cvUtilProCam.cpp](./SLhaircapture/cvUtilProCam.cpp)

* This lines
```cpp


	strcpy(sl_params->outdir, //TODO : output directory name);
	strcpy(sl_params->object, //TODO : output file name);
	strcpy(sl_params->image_format, //TODO : image file format string );
	strcpy(sl_params->image_format_S,//TODO : shifting based normal image format string);
	strcpy(sl_params->shifting_format,//TODO : shiting based shifting image format string);

	// Read camera parameters.
	sl_params->cam_w         =  3072;
	sl_params->cam_h         =  2048;

	// Read projector parameters.
	sl_params->proj_w      = 800;
	sl_params->proj_h      = 600;
```
