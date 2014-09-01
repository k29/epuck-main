#-------------------------------------------------
#
# Project created by QtCreator 2013-09-08T16:54:33
#
#-------------------------------------------------

QT       += core gui

QT += widgets

TARGET = epuck
TEMPLATE = app


LIBS += -L/usr/lib `pkg-config --libs opencv` -ltbb -lcvblob
LIBS += -L/usr/local/lib

SOURCES += main.cpp\
        mainwindow.cpp \
    cameraworker.cpp \
    serial.cpp \
    camcapture.cpp \
    featuredetection.cpp \
    beliefstate.cpp \
    algoworker.cpp

HEADERS  += mainwindow.h \
    cameraworker.h \
    serial.h \
    imgproc.h \
    camcapture.h \
    commondefs.h \
    defines.h \
    featuredetection.h \
    beliefstate.h \
    algoworker.h

FORMS    += mainwindow.ui

