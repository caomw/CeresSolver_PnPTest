#############################################################################
# Makefile for building: PnP
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
CFLAGS        = -m64 -pipe -O2 -w -D_REENTRANT $(DEFINES)
CXXFLAGS      = -m64 -pipe -Wno-unused-parameter -std=c++0x -O2 -w -D_REENTRANT $(DEFINES)
INCPATH       = -I/usr/local/include/opencv -I/usr/local/include -I/usr/include/eigen3 -I.
LINK          = g++
LFLAGS        = -m64 -Wl,-O1
LIBS          = $(SUBLIBS)  -L/usr/lib/x86_64-linux-gnu -lglog -lgflags -lceres -llapack -lcamd -lamd -lccolamd -lcolamd -lcholmod -fopenmp -lgomp -lblas /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_contrib.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_features2d.so /usr/local/lib/libopencv_flann.so /usr/local/lib/libopencv_gpu.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_legacy.so /usr/local/lib/libopencv_ml.so /usr/local/lib/libopencv_nonfree.so /usr/local/lib/libopencv_objdetect.so /usr/local/lib/libopencv_ocl.so /usr/local/lib/libopencv_photo.so /usr/local/lib/libopencv_stitching.so /usr/local/lib/libopencv_superres.so /usr/local/lib/libopencv_ts.a /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videostab.so -lrt -lm -ldl -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = pnp.cpp 
OBJECTS       = pnp.o
DIST          = PnP.pro
QMAKE_TARGET  = PnP
DESTDIR       = 
TARGET        = PnP

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: PnP.pro  
	$(QMAKE) -o Makefile PnP.pro

qmake:  FORCE
	@$(QMAKE) -o Makefile PnP.pro

dist: 
	@$(CHK_DIR_EXISTS) .tmp/PnP1.0.0 || $(MKDIR) .tmp/PnP1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) .tmp/PnP1.0.0/ && $(COPY_FILE) --parents pnp.cpp .tmp/PnP1.0.0/ && (cd `dirname .tmp/PnP1.0.0` && $(TAR) PnP1.0.0.tar PnP1.0.0 && $(COMPRESS) PnP1.0.0.tar) && $(MOVE) `dirname .tmp/PnP1.0.0`/PnP1.0.0.tar.gz . && $(DEL_FILE) -r .tmp/PnP1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


check: first

####### Compile

pnp.o: pnp.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o pnp.o pnp.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

