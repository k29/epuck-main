#-------------------------------------------------
#
# Project created by QtCreator 2012-09-12T22:43:27
#
#-------------------------------------------------

QT       += core gui

TARGET = trial1
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp\
        camcapture.cpp\
        makelut.cpp

HEADERS  += mainwindow.h \
    camcapture.h \
    imgproc.h
CONFIG += qwt
FORMS    += mainwindow.ui
LIBS += -L/usr/local/lib -lqwt
LIBS += -L/usr/local/lib -lQtSvg `pkg-config --libs opencv`
#INCLUDEPATH += /usr/include/flycapture /usr/include/qwt /usr/include/qt4
CONFIG += static
static {
CONFIG += static
DEFINES += STATIC
message("Static build.")
}

