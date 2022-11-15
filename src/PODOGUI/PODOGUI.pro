#-------------------------------------------------
#
# Project created by QtCreator 2016-02-17T10:05:39
#
#-------------------------------------------------

QT += widgets \
      core gui sql network opengl \

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG(debug, debug|release) {
    DESTDIR = ../../exe
} else {
    DESTDIR = ../../exe
}


TARGET = PODOGUI
TEMPLATE = app


###
INCLUDEPATH += \
    ../../share/Headers/RBModel

LIBS += \
    -L../../share/Libs/ -lRBModel
###


SOURCES += \
    _main.cpp \
    GUIMainWindow.cpp \
    LAN/RBTCPClient.cpp \
    LAN/RBTCPServer.cpp \
    BasicFiles/RBDataBase.cpp \
    BasicFiles/LANDialog.cpp \
    BasicFiles/PODOALDialog.cpp \
    BasicFiles/JointDialog.cpp \
    BasicFiles/SensorDialog.cpp \
    JoyStick/RBJoystick.cpp \
    qcustomplot.cpp \
    HCBSettingDialog.cpp \
#    PumpControlDialog.cpp \
#    LIGHTWalkingDialog.cpp \
#    LIGHTWalkingDialog2.cpp \
#    SH_TASK_Dialog.cpp \
#    LIGHTWalkingDialog_GainSetting.cpp \
    ValvePerfTest_Dialog.cpp \

HEADERS  += \
    GUIMainWindow.h \
    CommonHeader.h \
    LAN/RBLANCommon.h \
    LAN/RBLog.h \
    LAN/RBTCPClient.h \
    LAN/RBTCPServer.h \
    BasicFiles/RBDataBase.h \
    BasicFiles/RBDataType.h \
    BasicFiles/RBLog.h \
    BasicFiles/LANDialog.h \
    BasicFiles/PODOALDialog.h \
    BasicFiles/JointDialog.h \
    BasicFiles/SensorDialog.h \
    JoyStick/joystickclass.h \
    JoyStick/joystickvariable.h \
    qcustomplot.h \
    HCBSettingDialog.h \
#    PumpControlDialog.h \
#    LIGHTWalkingDialog.h \
#    LIGHTWalkingDialog2.h \
#    SH_TASK_Dialog.h \
#    LIGHTWalkingDialog_GainSetting.h \
    ValvePerfTest_Dialog.h \

FORMS    += \
    GUIMainWindow.ui \
    BasicFiles/LANDialog.ui \
    BasicFiles/PODOALDialog.ui \
    BasicFiles/JointDialog.ui \
    BasicFiles/SensorDialog.ui \
    HCBSettingDialog.ui \
#    PumpControlDialog.ui \
#    LIGHTWalkingDialog.ui \
#    LIGHTWalkingDialog2.ui \
#    SH_TASK_Dialog.ui \
#    LIGHTWalkingDialog_GainSetting.ui \
    ValvePerfTest_Dialog.ui \

