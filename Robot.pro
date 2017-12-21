#-------------------------------------------------
#
# Project created by QtCreator 2016-10-11T11:02:22
#
#-------------------------------------------------

QT       += widgets opengl xml serialport

TARGET = Robot
TEMPLATE = lib

DEFINES += ROBOT_LIBRARY

SOURCES += robot.cpp

HEADERS += robot.h\
        robot_global.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

PROJNAME = Robot
INSTTYPE = MOD

INCLUDEPATH += /usr/include/openni2
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui
LIBS += -lOpenNI2
LIBS += -lurg_c

INCLUDEPATH += /home/rsys/Documents/NiTE-2.0.0/Include
# LIBS += /home/rsys/Documents/NiTE-2.0.0/Redist/libNiTE2.so



include(RobotSDK_Main.pri)
