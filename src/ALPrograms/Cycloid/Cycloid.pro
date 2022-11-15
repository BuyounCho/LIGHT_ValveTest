TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle



QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt


INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy \
    ../../../share/Headers \
    /usr/include/eigen3\

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L/usr/local/lib/ -lrbdl


SOURCES += main.cpp \
    swfunc.cpp \
    myvariables.cpp \
    ManualCAN.cpp \
    QuadProg++.cc \
    Array.cc

HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    myvariables.h \
    main.h \
    ManualCAN.h \
    js_rbdl.h \
    Array.hh \
    QuadProg++.hh


