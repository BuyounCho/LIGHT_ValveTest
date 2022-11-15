TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle

QT       += core network

QMAKE_CXXFLAGS += -O3

#QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
#QMAKE_CXXFLAGS += -I/usr/include/xenomai
#QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

#QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
#QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt

INCLUDEPATH += \
#    /usr/include/xenomai/cobalt \
#    /usr/include/xenomai \
#    /usr/include/xenomai/alchemy \
    /usr/include/eigen3 \
    ../../../share/Headers \

LIBS    += \
#    -lalchemy \
#    -lcopperplate \
#    -lcobalt \
    -lpthread \
    -lrt \
#    -lpcan \
#    -L/usr/local/lib/x86_64-linux-gnu \
    -lrbdl \

SOURCES += \
    ManualCAN.cpp \
    QP_BasicFiles/Array.cc \
    QP_BasicFiles/QuadProg++.cc \
    LIGHT_functions.cpp \
    LIGHT_jointlevel_function.cpp \
    LIGHT_motionlevel_function.cpp \
    LIGHT_robotmodel_function.cpp \
    LIGHT_variables.cpp \
    LIGHT_dynamics.cpp \
    LIGHT_kinematics.cpp \
    LIGHT_savedata.cpp \
    LIGHT_systemid_function.cpp \
    LIGHT_walking_function.cpp \
    LIGHT_pumpreference_generator.cpp \
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
    _main.cpp \

HEADERS += \
    ManualCAN.h \
    BasicFiles/BasicJoint.h \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicMath.h \
    BasicFiles/BasicMatrix.h \
    BasicFiles/RBThread.h \
    QP_BasicFiles/QuadProg++.hh \
    QP_BasicFiles/Array.hh \
    LIGHT_commands.h \
    LIGHT_dynamics.h \
    LIGHT_info.h \
    LIGHT_kinematics.h \
    LIGHT_qp.h \
    LIGHT_robotmodel.h \
    LIGHT_var_and_func.h \
    LIGHT_motion.h \
    LIGHT_motion.h \
    qpSWIFT_BasicFiles/amd/include/amd.h \
    qpSWIFT_BasicFiles/amd/include/amd_internal.h \
    qpSWIFT_BasicFiles/amd/include/SuiteSparse_config.h \
    qpSWIFT_BasicFiles/include/GlobalOptions.h \
    qpSWIFT_BasicFiles/include/ldl.h \
    qpSWIFT_BasicFiles/include/Matrices.h \
    qpSWIFT_BasicFiles/include/Prime.h \
    qpSWIFT_BasicFiles/include/timer.h \
    LIGHT_savedata.h \


