TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle

QT       += core network

QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt


INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy \
    /usr/include/eigen3 \
    ../../../share/Headers

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L/usr/local/lib/x86_64-linux-gnu -lrbdl


SOURCES += main.cpp \


HEADERS += \
    BasicFiles/BasicJoint.h \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicMath.h \
    BasicFiles/BasicMatrix.h \

DISTFILES +=







