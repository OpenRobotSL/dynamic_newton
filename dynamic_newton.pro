TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    RobotDynamics.cpp \
    test_7_dof.cpp

HEADERS += \
    RobotDynamics.h
