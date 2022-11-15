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


SOURCES += \
    _main.cpp \
    QP_BasicFiles/Array.cc \
    QP_BasicFiles/QuadProg++.cc \
    qpSWIFT_BasicFiles/amd/src/amd_1.c \
    qpSWIFT_BasicFiles/amd/src/amd_2.c \
    qpSWIFT_BasicFiles/amd/src/amd_aat.c \
    qpSWIFT_BasicFiles/amd/src/amd_control.c \
    qpSWIFT_BasicFiles/amd/src/amd_defaults.c \
    qpSWIFT_BasicFiles/amd/src/amd_dump.c \
    qpSWIFT_BasicFiles/amd/src/amd_global.c \
    qpSWIFT_BasicFiles/amd/src/amd_info.c \
    qpSWIFT_BasicFiles/amd/src/amd_order.c \
    qpSWIFT_BasicFiles/amd/src/amd_postorder.c \
    qpSWIFT_BasicFiles/amd/src/amd_post_tree.c \
    qpSWIFT_BasicFiles/amd/src/amd_preprocess.c \
    qpSWIFT_BasicFiles/amd/src/amd_valid.c \
    qpSWIFT_BasicFiles/src/Auxilary.c \
    qpSWIFT_BasicFiles/src/ldl.c \
    qpSWIFT_BasicFiles/src/Prime.c \
    qpSWIFT_BasicFiles/src/RUNQP.c \
    qpSWIFT_BasicFiles/src/timer.c \
    QUAD_Jointsetmodel_function.cpp \
    QUAD_BasicFunction.cpp \
    QUAD_Task_function.cpp

HEADERS += \
    BasicFiles/BasicJoint.h \
    BasicFiles/BasicSetting.h \
    QP_BasicFiles/QuadProg++.hh \
    QP_BasicFiles/Array.hh \
    qpSWIFT_BasicFiles/amd/include/amd.h \
    qpSWIFT_BasicFiles/amd/include/amd_internal.h \
    qpSWIFT_BasicFiles/amd/include/SuiteSparse_config.h \
    qpSWIFT_BasicFiles/include/GlobalOptions.h \
    qpSWIFT_BasicFiles/include/ldl.h \
    qpSWIFT_BasicFiles/include/Matrices.h \
    qpSWIFT_BasicFiles/include/Prime.h \
    qpSWIFT_BasicFiles/include/timer.h \
    QUAD_jointsetmodel.h \
    QUAD_BasicFunction.h

DISTFILES +=







