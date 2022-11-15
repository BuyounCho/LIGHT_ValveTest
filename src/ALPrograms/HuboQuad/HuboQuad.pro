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
    ../../../share/Headers\
    /usr/include/eigen3

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L/usr/local/lib/ -lrbdl


SOURCES += main.cpp \
    ManualCAN.cpp \
    basicsetting.cpp \
    Gaits/ow_quad_general.cpp \
    Gaits/ow_quad_others.cpp \
    Gaits/ow_quad_trot.cpp \
    Gaits/ow_quad_wave.cpp \
    Array.cc \
    QuadProg++.cc \
    Gaits/ow_quad_qp.cpp \
    Gaits/ow_pvddx.cpp \
    Gaits/ow_quad_standing.cpp \
    Gaits/ow_quad_startstop.cpp \
    Gaits/ow_quad_slope_adjust.cpp \
    Gaits/ow_quad_wave2.cpp \
    Gaits/ow_quad_slope_adjust_waves.cpp \
    calc_QP.cpp \
    Gaits/ow_quad_pronk.cpp \
    Gaits/ow_quad_demo.cpp \
    Gaits/ow_quad_flytrot.cpp \
    ow_onedofmpc.cpp

HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    Oinverse.h \
    OW_RBDL.h \
    ow_quad.h \
    Otypes.h \
    joint_inverse.h \
    joint_inverse_SW.h \
    ManualCAN.h \
    Array.hh \
    ow_cplex.h \
    QuadProg++.hh \
    Gaits/ow_pvddx.h \
    ow_onedofmpc.h

DISTFILES += \
    memo


