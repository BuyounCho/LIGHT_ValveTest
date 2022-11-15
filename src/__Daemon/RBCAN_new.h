#ifndef RB_CAN_H
#define RB_CAN_H

#include <vector>
#include <QVector>
#include <QByteArray>
#include <iostream>

#include "HydraulicActuatorController.h"
#include "HydraulicActuatorDataConverting.h"
#include "HydraulicPumpController.h"
#include "RBIMUSensor.h"

typedef struct{
    unsigned char header;
    unsigned char dlc;
    unsigned short id;
    unsigned char data[8];
}ST_CAN;


class rb_can
{
public:
    rb_can();

    static pthread_mutex_t mutex_reference_VC[MAX_VC];
    static pthread_mutex_t mutex_reference_PC[MAX_PC];

    static ST_CAN  reference_msg_VC[MAX_VC];
    static ST_CAN  reference_msg_PC[MAX_PC];

    static QVector<ST_CAN> general_send_msgs1;
    static QVector<ST_CAN> general_send_msgs2;
    static QVector<ST_CAN> general_recv_msgs;


    static bool set_reference_msg_VC(int i);
    static bool set_reference_msg_PC(int i);
    static void write_general_msg(ST_CAN mb, int CH);

};

#endif // RB_CAN_H
