
LIBDIR=../lib
AR=ar
MODULE=mistl
MAKEDEPEND= gcc -M

LIBNAME=lib$(MODULE).a
OBJ=Camera.o ReadCamera.o WriteCamera.o OCV_Camera.o Error.o ReadCoordinateFile.o MultiCameraCalibration.o Line3.o \
	KLTracker.o FeatureValidation.o MultiCamTracking.o MultiCameraCalibrationAdd.o WriteOBJ.o VisualizeCamera.o \
	MultiCameraCalibrationOptimize.o UnDistortImage.o RunCalibration.o CameraYAML.o TransformYAML.o DRCalibrate.o \
	DR_CameraArrayYAML.o EvaluateCalibration.o Log.o Quality.o NiceDataDump.o CSVDataDump.o FeatureList.o MultiCamTrackingFeatures.o RunCalibrationHistogram.o AuxilaryFunctions.o DistortionKP.o Vector3.o
	
# MultiCamTracking.o 
DEP	:= $(OBJ:%.o=%.dep)

## Targets
# first target is executed by default if no target is specified
all: lib dep

CC      = /usr/bin/gcc
DEBUG	= -g -DDEBUG -Wall
OPTIMIZE = -O3
CFLAGS  = $(DEBUG) $(OPTIMIZE)  -funroll-loops  -I../include -I.. -std=c++0x -DDLM_1_3 -DUSE_OPENCV
LDFLAGS = 


dep: $(DEP)

%.dep: %.cc
	$(MAKEDEPEND) $(CFLAGS) $< -o $@ $(LOCALLIBINCL)


%.o: %.cc
	$(CC) $(CFLAGS) -c $<


lib: $(LIBDIR)/$(LIBNAME)

## targets rules for static libraries
$(LIBDIR)/$(LIBNAME): $(OBJ)
	@echo "ARCHIVING static library $(LIBNAME) ..."
	@$(AR) $(ARFLAGS) $@ $(OBJ)
#	$(AR) ruvs $(LIBNAME) `lorder $(OBJ)|tsort`
	@echo "done $@ !"

clean:
	rm *.o *.a *.dep

-include $(DEP)
