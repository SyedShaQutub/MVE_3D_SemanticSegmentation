TEMPLATE = lib
CONFIG += staticlib
CONFIG -= qt

DESTDIR	= lib
CONFIG(debug, debug|release):OBJECTS_DIR = ./tmp/debug
CONFIG(debug, debug|release):TARGET = Cored
CONFIG(release, debug|release):OBJECTS_DIR = ./tmp/release
CONFIG(release, debug|release):TARGET = Core
CONFIG(release, debug|release):DEFINES += NDEBUG

HEADERS = \
FileSequence.h \
FixedMatrix.h \
FixedVector.h \
SmartData.h

SOURCES = 

INCLUDEPATH += ../

  
