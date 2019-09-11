

This was intended as a simple example how to use camera, but mophed into a utility program with a number of options to visualise cameras and calibration data.


Call camera without parameters to get a list of options.

The following call reads a coordinate list file (option -p) and overlays the projected points onto an image (option -ov). 
The output is written into an image (option wov). Tip: Use '-wov -' to get an image viewer opened with the result.

Linux:
./camera -p ../../../data/IVCI/calibration/cam01.wim -ov ../../../data/IVCI/calibration/cam01/00000584.jpg -wov - ../../../data/IVCI/calibration/cam01_01.ycam 

Visual Studio command arguments for mistl_camera:
-p ../data/IVCI/calibration/cam01.wim -ov ../data/IVCI/calibration/cam01/00000584.jpg -wov - ../data/IVCI/calibration/cam01_01.ycam 

WARNING / FIXME: data/IVCI/calibration/cam01_01.ycam does not exist in SVN.


The camera parameters are read from the last argument. An object transformation is applied to the 3D points before projection (option -otrans). 

The utility also allows to select a particular frame (option -seq).



### visualize camera:
./camera -vis FactoryCameraCalibration_02.obj FactoryCameraCalibration_02.ycam

### visualize camera with open scene graph viewer
osgviewer FactoryCameraCalibration_00.obj FactoryCameraCalibration_01.obj FactoryCameraCalibration_02.obj


