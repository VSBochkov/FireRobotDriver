QT += core
QT += network
QT += serialport
QT -= gui

TARGET = FireRobotDriver
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    driver.cpp

HEADERS += \
    driver.h

