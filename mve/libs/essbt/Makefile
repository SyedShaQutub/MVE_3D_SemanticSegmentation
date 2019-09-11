MVE_ROOT = ../..
TARGET := libmve_essbt.a 
include ${MVE_ROOT}/Makefile.inc

OPENCV_CFLAGS = `pkg-config --cflags opencv`
OPENCV_LIBS = `pkg-config --libs opencv`

CXXFLAGS += -pg -fPIC -I${MVE_ROOT}/libs  ${OPENCV_CFLAGS} ${OPENMP}
LDLIBS += -pg -lpng -ltiff -ljpeg ${OPENCV_LIBS} ${OPENMP}

SOURCES := $(wildcard [^_]*.cc)
${TARGET}: ${SOURCES:.cc=.o}
	$(AR) rcs $@ $^

clean:
	${RM} $(TARGET) *.o Makefile.dep

.PHONY: clean
