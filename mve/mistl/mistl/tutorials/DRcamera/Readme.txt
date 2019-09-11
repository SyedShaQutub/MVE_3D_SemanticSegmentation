

This was intended as a simple example how to use camera, but mophed into a utility program with a number of options to visualise cameras and calibration data.


Call camera without parameters to get a list of options.

The following call reads a coordinate list file (option -p) and overlays the projected points onto an image (option -ov). The output is written into an image (option wov). Tip: Use '-wov -' to get an image viewer opened with the result.

./camera -p ../../../data/IVCI/calibration/cam01.wim -ov ../../../data/IVCI/calibration/cam01/00000584.jpg -wov - ../../../data/IVCI/calibration/cam01_01.ycam 


The camera parameters are read from the last argument. An object transformation is applied to the 3D points before projection (option -otrans). 

The utility also allows to select a particular frame (option -seq).



### Producing DR-Camera file out of 3 camera files in *.yml, *.cahv or *.ycam format
./DRcamera -o newFactoryCameraCalibration.yml cam_%02d.ycam

### Producing ycam files from factory:
./DRcamera -o camFile_%02d.ycam -convdr <path to input file>/FactoryCameraCalibration.yml



