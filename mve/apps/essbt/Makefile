MVE_ROOT := ../..
TARGET := $(shell basename `pwd`)
include ${MVE_ROOT}/Makefile.inc


#debug: CXXFLAGS += -I${MVE_ROOT}/libs ${OPENMP} -g
OPENCV_CFLAGS = `pkg-config --cflags opencv`
OPENCV_LIBS = `pkg-config --libs opencv`

CXXFLAGS += -pg -I${MVE_ROOT}/libs ${OPENCV_CFLAGS} ${OPENMP} -g -shared-libgcc -DNDEBUG
LDLIBS += -pg -lpng -ltiff -ljpeg -ltbb ${OPENCV_LIBS} ${OPENMP}

SOURCES := $(wildcard [^_]*.cc) 
${TARGET}: ${SOURCES:.cc=.o} libmve_essbt.a libmve.a libmve_util.a

clean:

	${RM} ${TARGET} *.o Makefile.dep

.PHONY: clean
