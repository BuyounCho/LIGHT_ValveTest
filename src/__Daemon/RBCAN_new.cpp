#include "RBCAN_new.h"

using namespace std;

static rb_can obj;

extern ValveController _DEV_VC[MAX_VC];
extern PumpController  _DEV_PC[MAX_PC];

pthread_mutex_t rb_can::mutex_reference_VC[MAX_VC];
pthread_mutex_t rb_can::mutex_reference_PC[MAX_PC];
ST_CAN  rb_can::reference_msg_VC[MAX_VC];
ST_CAN  rb_can::reference_msg_PC[MAX_PC];
QVector<ST_CAN> rb_can::general_send_msgs1;
QVector<ST_CAN> rb_can::general_send_msgs2;
QVector<ST_CAN> rb_can::general_recv_msgs;

rb_can::rb_can()
{

    for(int i=0; i<MAX_VC; i++){
        switch(_DEV_VC[i].CAN_CHANNEL) {
        case 0:
            reference_msg_VC[i].header = 0x89;
            break;
        case 1:
            reference_msg_VC[i].header = 0x77;
            break;
        case 2:
            reference_msg_VC[i].header = 0x89;
            break;
        case 3:
            reference_msg_VC[i].header = 0x77;
            break;
        default:
            std::cout << "CAN Channel Setting Error!" << std::endl;
            break;
        }
        pthread_mutex_init(&mutex_reference_VC[i], NULL);
        //        set_reference_msg(i);
    }

    for(int i=0; i<MAX_PC; i++){
        switch(_DEV_PC[i].CAN_CHANNEL) {
        case 0:
            reference_msg_PC[i].header = 0x89;
            break;
        case 1:
            reference_msg_PC[i].header = 0x77;
            break;
        case 2:
            reference_msg_PC[i].header = 0x89;
            break;
        case 3:
            reference_msg_PC[i].header = 0x77;
            break;
        default:
            std::cout << "CAN Channel Setting Error!" << std::endl;
            break;
        }
        pthread_mutex_init(&mutex_reference_PC[i], NULL);
        //        set_reference_msg(i);
    }

}

bool rb_can::set_reference_msg_VC(int i)
{
    // i : Valve Controller Board Number
    // if there are commanded reference, return true.
    // else, return false.
    static int16_t ref_pos_old[MAX_VC];
    static int16_t ref_vel_old[MAX_VC];
    static int16_t ref_force_pulse_old[MAX_VC];
    static int16_t ref_pressure_old[MAX_VC];
    static int16_t ref_valvepos_old[MAX_VC];
    static int16_t ref_PWM_old[MAX_VC];

    switch(sharedREF->ValveCtrlMode[i]) {
    case ValveControlMode_Null: // Valve Control Mode : Null
    {
        return false;
    }
    case ValveControlMode_PosOrFor: // Valve Control Mode : Position or Force Control
    {
        // @ joint : maximum 100 deg(or mm) > multiply 200
        //           maximum 1000 deg/s(or mm/s) multiply 20
        // @ force sensor : maximum 4096 pulse > multiply 10
        // @ force sensor : maximum 210 bar > multiply 100
        int16_t ref_pos = (int16_t)(_DEV_VC[i].Joints[0].HCB_Ref.ReferencePosition*200.0);
        int16_t ref_vel = (int16_t)(_DEV_VC[i].Joints[0].HCB_Ref.ReferenceVelocity*20.0);
        int16_t ref_force_pulse = (int16_t)(_DEV_VC[i].Joints[0].HCB_Ref.ReferenceForceTorque*_DEV_VC[i].PULSE_PER_FORCETORQUE*10.0);
        int16_t ref_pressure = (int16_t)(_DEV_VC[i].Joints[0].HCB_Ref.ReferencePumpPressure*100.0);

        if(sharedREF->PumpSupplyPressureChange) { // Variable Supply Pressure
            reference_msg_VC[i].dlc = 8;
            reference_msg_VC[i].id = (short)_DEV_VC[i].ID_SEND_POSVEL; // Position and Velocity Reference
            reference_msg_VC[i].data[0] = (unsigned char)(ref_pos & 0xFF);
            reference_msg_VC[i].data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
            reference_msg_VC[i].data[2] = (unsigned char)(ref_vel & 0xFF);
            reference_msg_VC[i].data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
            reference_msg_VC[i].data[4] = (unsigned char)(ref_force_pulse & 0xFF);
            reference_msg_VC[i].data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
            reference_msg_VC[i].data[6] = (unsigned char)(ref_pressure & 0xFF);
            reference_msg_VC[i].data[7] = (unsigned char)((ref_pressure>>8) & 0xFF);
            ref_pos_old[i] = ref_pos;
            ref_vel_old[i] = ref_vel;
            ref_force_pulse_old[i] = ref_force_pulse;
            ref_pressure_old[i] = ref_pressure;
        } else { // Constant Supply Pressure
            reference_msg_VC[i].dlc = 8;
            reference_msg_VC[i].id = (short)_DEV_VC[i].ID_SEND_POSVEL; // Position and Velocity Reference
            reference_msg_VC[i].data[0] = (unsigned char)(ref_pos & 0xFF);
            reference_msg_VC[i].data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
            reference_msg_VC[i].data[2] = (unsigned char)(ref_vel & 0xFF);
            reference_msg_VC[i].data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
            reference_msg_VC[i].data[4] = (unsigned char)(ref_force_pulse & 0xFF);
            reference_msg_VC[i].data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
            reference_msg_VC[i].data[6] = 0;
            reference_msg_VC[i].data[7] = 0;
            ref_pos_old[i] = ref_pos;
            ref_vel_old[i] = ref_vel;
            ref_force_pulse_old[i] = ref_force_pulse;
        }
        return true;
    }
    case ValveControlMode_Opening: // Valve Control Mode : Valve Opening
    {
        int16_t ref_valvepos = (int16_t)(_DEV_VC[i].Joints[0].HCB_Ref.ReferenceValvePos);

        reference_msg_VC[i].id = (short)_DEV_VC[i].ID_SEND_VALVEPOSnPWM;
        reference_msg_VC[i].dlc = 4;
        reference_msg_VC[i].data[0] = (unsigned char)(ref_valvepos & 0x000000FF);
        reference_msg_VC[i].data[1] = (unsigned char)((ref_valvepos>>8) & 0x000000FF);
        reference_msg_VC[i].data[2] = 0;
        reference_msg_VC[i].data[3] = 0;
        ref_valvepos_old[i] = ref_valvepos;
        return true;
    }
    case ValveControlMode_PWM: // Valve Control Mode : Valve input voltage
    {
        int16_t ref_PWM = (int16_t)(_DEV_VC[i].Joints[0].HCB_Ref.ReferencePWM);

        reference_msg_VC[i].id = (short)_DEV_VC[i].ID_SEND_VALVEPOSnPWM;
        reference_msg_VC[i].dlc = 4;
        reference_msg_VC[i].data[0] = 0;
        reference_msg_VC[i].data[1] = 0;
        reference_msg_VC[i].data[2] = (unsigned char)(ref_PWM & 0x000000FF);
        reference_msg_VC[i].data[3] = (unsigned char)((ref_PWM>>8) & 0x000000FF);
        ref_PWM_old[i] = ref_PWM;
        return true;
    }
    case ValveControlMode_UtilMode: // Valve Control Mode : Find Home
    {
        return false;
    }
    default:
        return false;
    }
}

bool rb_can::set_reference_msg_PC(int i)
{
    // i : Pump Controller Number
    // if there are commanded reference, return true.
    // else, return false.

    int16_t ref_n = (int16_t)_DEV_PC[i].ReferencePumpVelocity;
    int16_t ref_l = (int16_t)_DEV_PC[i].ReferencePumpVelocity_last;

    _DEV_PC[i].ReferencePumpVelocity_last = _DEV_PC[i].ReferencePumpVelocity;

    switch(sharedREF->PumpCtrlMode[i]) {
    case PumpControlMode_Null:
    {
//        reference_msg_PC[i].id = 999; // dummy ID
//        reference_msg_PC[i].dlc = 1;
//        reference_msg_PC[i].data[0] = 0;
//        return true;
        return false;
    }
    case PumpControlMode_Interpolation:
    {
//        reference_msg_PC[i].id = 999; // dummy ID
//        reference_msg_PC[i].dlc = 1;
//        reference_msg_PC[i].data[0] = 0;
//        return true;
        return false;
    }
    case PumpControlMode_ActiveControl:
    {
        reference_msg_PC[i].id = (short)_DEV_PC[i].ID_SEND_VELOCITY;
        reference_msg_PC[i].dlc = 2;
        reference_msg_PC[i].data[0] = (unsigned char)(ref_n & 0x000000FF);
        reference_msg_PC[i].data[1] = (unsigned char)((ref_n>>8) & 0x000000FF);
        return true;
    }
    default:
        return false;
    }

//    if(ref_n!=ref_l)
//    {

//    } else {
//        return false;
//    }
}


void rb_can::write_general_msg(ST_CAN mb, int CH) {
    ST_CAN msg = mb;
    if(CH == 0) {
        msg.header = 0x89;
        general_send_msgs1.push_back(msg);
    }
    else if(CH == 1) {
        msg.header = 0x77;
        general_send_msgs1.push_back(msg);
    }
    else if(CH == 2) {
        msg.header = 0x89;
        general_send_msgs2.push_back(msg);
    }
    else if(CH == 3) {
        msg.header = 0x77;
        general_send_msgs2.push_back(msg);
    } else {
        std::cout << "Channel Setting Error! (Channel : " << CH << ")" << std::endl;
    }
}
