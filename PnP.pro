NAME = PnP
QT       -= core gui

CONFIG(release, debug|release) {
    TARGET = $$NAME
    CONFIG += warn_off
}
else {
    TARGET = $${NAME}_d
    CONFIG += console
}

CONFIG += c++11
QMAKE_CXXFLAGS += -Wno-unused-parameter -std=c++0x


SOURCES += pnp.cpp

##########################################################################
#MISC
LIBS += -lglog
LIBS += -lgflags

CONFIG += link_pkgconfig
##########################################################################
#OPENCV
PKGCONFIG += opencv

##########################################################################
#EIGEN
PKGCONFIG += eigen3

##########################################################################
#CERES
LIBS += -lceres
# If Ceres was built with Suitesparse:
LIBS += -llapack -lcamd -lamd -lccolamd -lcolamd -lcholmod
# If Ceres was built with OpenMP:
LIBS += -fopenmp -lpthread -lgomp -lm
LIBS += -lblas







