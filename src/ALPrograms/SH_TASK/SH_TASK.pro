TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle

QT       += core network

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
    ../../../share/Headers

LIBS    += \
#    -lalchemy \
#    -lcopperplate \
#    -lcobalt \
    -lpthread \
    -lrt \
#    -lpcan \
#    -L/usr/local/lib/x86_64-linux-gnu
    -lrbdl


SOURCES += \
    LIGHT_jointsetmodel_function.cpp \
    _main.cpp \
    SH_TASK_ValveIdentification.cpp

HEADERS += \
    BasicFiles/BasicJoint.h \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicMath.h \
    BasicFiles/BasicMatrix.h \
    ../LIGHTWalking/LIGHT_info.h \
    ../LIGHTWalking/LIGHT_var_and_func.h \
    LIGHT_jointsetmodel.h \
    SH_TASK_BasicFunction.h

DISTFILES +=







