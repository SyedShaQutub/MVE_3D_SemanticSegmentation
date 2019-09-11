

MODULES=camera  cameraToYcam  math  opencvcamera  reproduce3d  test  testCalibration  testOcv2Mistl  undistortion  vector



clean : $(foreach lib,$(MODULES),clean_$(lib))

clean_% :
	@cd $*; \
        $(MAKE) $(OPTS) clean
