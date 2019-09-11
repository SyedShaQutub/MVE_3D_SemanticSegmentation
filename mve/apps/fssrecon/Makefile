MVE_ROOT := ../..
TARGET := $(shell basename `pwd`)
include ${MVE_ROOT}/Makefile.inc

#debug: CXXFLAGS += -I${MVE_ROOT}/libs ${OPENMP} -g

CXXFLAGS += -I${MVE_ROOT}/libs ${OPENMP} -g
LDLIBS += -lpng -ltiff -ljpeg ${OPENMP}

SOURCES := $(wildcard [^_]*.cc)
${TARGET}: ${SOURCES:.cc=.o} libmve_fssr.a libmve.a libmve_util.a

clean:
	${RM} ${TARGET} *.o Makefile.dep

.PHONY: clean
