QT  += sql \
       widgets \

TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle

CONFIG(debug, debug|release) {
    DESTDIR = ../../exe
} else {
    DESTDIR = ../../exe
}

#QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
#QMAKE_CXXFLAGS += -I/usr/include/xenomai
#QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

#QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
#QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt

QMAKE_CXXFLAGS += -O1

INCLUDEPATH += \
#    /usr/include/xenomai/cobalt \
#    /usr/include/xenomai \
#    /usr/include/xenomai/alchemy \
    /usr/include/eigen3 \
    /usr/include/rbdl \
    ../../../share/Headers \

LIBS    += \
#    -lalchemy \
#    -lcopperplate \
#    -lcobalt \
    -lpthread \
    -lrt \
#    -lpcan \
    -L/usr/local/lib/x86_64-linux-gnu \
    -lrbdl \

SOURCES += \
    RBDataBase.cpp \
#    RBCAN.cpp \
    RBCAN_new.cpp \
    RBSPI2CAN.cpp \
    RBProcessManager.cpp \
    RBFTSensor.cpp \
    RBIMUSensor.cpp \
#    RBSmartPower.cpp \
#    RBOpticFlowSensor.cpp \
#    RBFOGSensor.cpp \
    RBRawLAN.cpp \
#    RBELMO.cpp \
#    RBMotorControllerCOCOA.cpp \
    HydraulicActuatorController.cpp \
    HydraulicActuatorDataConverting.cpp \
    _main.cpp \
    HydraulicPumpController.cpp
#    RBClock.cpp

HEADERS += \
    RBDataBase.h \
    RBLog.h \
    RBDataType.h \
#    RBCAN.h \
    RBCAN_new.h \
    RBSPI2CAN.h \
    RBProcessManager.h \
    RBFTSensor.h \
    RBIMUSensor.h \
#    RBSmartPower.h \
#    RBOpticFlowSensor.h \
#    RBFOGSensor.h \
    RBRawLAN.h \
    ExternHeader.h \
#    RBELMO.h \
#    RBMotorControllerCOCOA.h \
    HydraulicActuatorController.h \
    HydraulicActuatorDataConverting.h \
#    RBClock.h
    RBThread.h \
    IKF.h \
    HydraulicPumpController.h
