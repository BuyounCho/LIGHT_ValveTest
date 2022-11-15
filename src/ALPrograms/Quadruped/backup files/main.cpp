#include "BasicFiles/BasicSetting.h"
#include "swtypes.h"
#include "ManualCAN.h"
#include <unistd.h>
#include "BasicMatrix.h"
#include "joint_inverse_SW.h"
//#include "cmatrix"

using namespace std;

// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
RBCORE_SHM_SENSOR       sharedSEN2;
pUSER_SHM               userData;

JointControlClass       *jCon;

int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;

// Parameters and functions --------
BoardIdChParams BP;
MovePosParams MP;
MoveVelParams MV;
CurrentSetParams CS;
PWMSetParams PS;
MoveVelParams MVlm;
CurrentSetParams CSlm;
EVALSetParams ES;
SineReferenceSetParams SS;
ProtectionParams PT;
SaveParams SP;
ONELEGFKIK Oii;
int OPERATION_MODE = 0;
unsigned long rt_count = 0;
unsigned long cntforcountsleep = 0;
void countsleep(int microsec)
{
    cntforcountsleep = 0;
    while(microsec/RT_TIMER_PERIOD_MS/1000.0>cntforcountsleep)
    {
        usleep(1*1000);//1ms sleep
    }
}
double GetJointAngleEnc(int Jnum)
{
    return sharedSEN->ENCODER[MC_ID_CH_Pairs[Jnum].id][MC_ID_CH_Pairs[Jnum].ch].CurrentPosition;
}
void SetCurrent(){
    sharedREF->COCOACurrent_REF[PODO_NO][RHR] = CS.current;
}
void SetPWM(){
    sharedREF->JointFFpwm[PODO_NO][MC_GetID(RHR)][MC_GetCH(RHR)] = PS.pwm;
}
double Vel2DAC(double speed, double velocityParam){
    return speed/(velocityParam*360/60);
}
double Cur2DAC(double current, double currentParam){
    return (current*100)/(11.07*currentParam); // CSMS-30BT Rated Torque = 11.07
}

// Main --------
int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "Quadruped");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

//    jCon->SetMotionOwner(0);
    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
        case POS_SET_MOVE:
        {
            FILE_LOG(logSUCCESS) << "Command POS_SET_MOVE received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            MP.period = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MP.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            MP.angle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            MP.mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];

//            MP.run = true;

            jCon->SetMovePosJoint(RHR, MP.angle, MP.period*1000, MP.smooth*1000, MP.mode);
            rt_count = 0;
            SP.saveEndIndex = (unsigned long)(MP.period/(0.001*RT_TIMER_PERIOD_MS));
            OPERATION_MODE = OPERATION_POS_MOVE;

//            countsleep((MP.period + 0.1)*1000*1000);
//            printf("Move done\n");

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case POS_STOP_MOVE:
        {
            FILE_LOG(logSUCCESS) << "Command POS_STOP_MOVE received..";
//            jCon->SetJointRefAngle(RHR,GetJointAngleEnc(RHR));
//            jCon->SetJointRefAngle(RHR,sharedSEN->ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference);
//            jCon->SetAllMotionOwner();

//            MP.run = false;

            jCon->SetStopPosJoint(RHR);
            SP.saveEndIndex = rt_count + (unsigned long)(MP.smooth/(0.001*RT_TIMER_PERIOD_MS));
//            SP.SaveFile(1);
//            OPERATION_MODE = OPERATION_NONE;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case POS_SET_REPETITIVE_MOVE:
        {
            FILE_LOG(logSUCCESS) << "Command POS_SET_REPETITIVE_MOVE received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            MP.period = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MP.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            MP.initialAngle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            MP.finalAngle = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
            MP.mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];

            MP.repetitiveFlag = true;
            MP.repetitiveCount = 0;

//            MP.run = true;

//            jCon->SetMovePosJoint(RHR, MP.angle, MP.period*1000, MP.smooth*1000, MP.mode);
            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_POS_MOVE_REPETITIVE;

//            countsleep((MP.period + 0.1)*1000*1000);
//            printf("Move done\n");

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case POS_STOP_REPETITIVE_MOVE:
        {
            FILE_LOG(logSUCCESS) << "Command POS_STOP_REPETITIVE_MOVE received..";

            MP.repetitiveFlag = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case VEL_SET_JOGGING:
        {
            FILE_LOG(logSUCCESS) << "Command VEL_SET_JOGGING received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            MV.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MV.speed = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            jCon->SetMoveJoint(RHR, MV.speed, MV.smooth*1000, MOVE_ABSOLUTE);
            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_VEL_JOGGING;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case VEL_STOP_JOGGING:
        {
            FILE_LOG(logSUCCESS) << "Command VEL_STOP_JOGGING received..";

            MV.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MV.speed = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

//            MP.run = false;

            jCon->SetMoveJoint(RHR,MV.speed,MV.smooth*1000, MOVE_ABSOLUTE);
            SP.saveEndIndex = rt_count + (unsigned long)(MV.smooth/(0.001*RT_TIMER_PERIOD_MS));
//            SP.SaveFile(2);
//            OPERATION_MODE = OPERATION_NONE;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case CURRENT_SET_CMD:
        {
            FILE_LOG(logSUCCESS) << "Command CURRENT_SET_CMD received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            CS.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            CS.current = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            int tar_id = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];

//            CS.setAllCurrentFlag = false;

            jCon->SetMoveQCurrent(tar_id,CS.current,CS.smooth*1000, MOVE_ABSOLUTE);
            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_CUR_SET_CMD;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case CURRENT_SET_CMD_ALL:
        {
            FILE_LOG(logSUCCESS) << "Command CURRENT_SET_CMD_ALL received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            for(int i=0;i<10;i++){
                CS.currents[i] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
            }
            CS.duration = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[10];
            CS.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[11];
            CS.durationIndex = (unsigned long)(CS.duration/(0.001*RT_TIMER_PERIOD_MS));
            CS.durationCount = 0;

            CS.setAllCurrentFlag = true;
            CS.currentCount = 0;

//            jCon->SetMoveQCurrent(RHR,CS.current,CS.smooth*1000, MOVE_ABSOLUTE);
            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_CUR_SET_CMD_ALL;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case CURRENT_STOP_CMD:
        {
            FILE_LOG(logSUCCESS) << "Command CURRENT_STOP_CMD received..";

            CS.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            CS.current = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            CS.setAllCurrentFlag = false;
//            CS.currentCount = 0;

            jCon->SetMoveQCurrent(RHR,CS.current,CS.smooth*1000, MOVE_ABSOLUTE);
            SP.saveEndIndex = rt_count + (unsigned long)(CS.smooth/(0.001*RT_TIMER_PERIOD_MS));

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case PWM_SET_CMD:
        {
            FILE_LOG(logSUCCESS) << "Command PWM_SET_CMD received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            PS.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            PS.pwm = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

//            PS.setAllPWMFlag = false;

            jCon->SetMovePWM(RHR,PS.pwm,PS.smooth*1000, MOVE_ABSOLUTE);
            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_PWM_SET_CMD;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case PWM_SET_CMD_ALL:
        {
            FILE_LOG(logSUCCESS) << "Command PWM_SET_CMD_ALL received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            for(int i=0;i<10;i++){
                PS.pwms[i] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
            }
            PS.duration = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[10];
            PS.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[11];
            PS.durationIndex = (unsigned long)(PS.duration/(0.001*RT_TIMER_PERIOD_MS));
            PS.durationCount = 0;

            PS.setAllPWMFlag = true;
            PS.pwmCount = 0;

//            jCon->SetMovePWM(RHR,PS.pwm,PS.smooth*1000, MOVE_ABSOLUTE);
            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_PWM_SET_CMD_ALL;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case PWM_STOP_CMD:
        {
            FILE_LOG(logSUCCESS) << "Command PWM_STOP_CMD received..";

            PS.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            PS.pwm = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            jCon->SetMovePWM(RHR,PS.pwm,PS.smooth*1000, MOVE_ABSOLUTE);
            SP.saveEndIndex = rt_count + (unsigned long)(PS.smooth/(0.001*RT_TIMER_PERIOD_MS));
//            SP.SaveFile(4);
//            OPERATION_MODE = OPERATION_NONE;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }

        case VEL_SET_JOGGING_LM:
        {
            FILE_LOG(logSUCCESS) << "Command VEL_SET_JOGGING_LM received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            MVlm.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MVlm.speed = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            MVlm.velocityParam = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            jCon->SetMoveJoint(RKN, MVlm.speed, MVlm.smooth*1000, MOVE_ABSOLUTE);
            jCon->SetMoveDAC(RKN, Vel2DAC(MVlm.speed, MVlm.velocityParam), MVlm.smooth*1000, MOVE_ABSOLUTE);

            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_VEL_JOGGING_LM;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case VEL_STOP_JOGGING_LM:
        {
            FILE_LOG(logSUCCESS) << "Command VEL_STOP_JOGGING_LM received..";

            MVlm.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MVlm.speed = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            MVlm.velocityParam = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            jCon->SetMoveJoint(RKN, MVlm.speed, MVlm.smooth*1000, MOVE_ABSOLUTE);
            jCon->SetMoveDAC(RKN, Vel2DAC(MVlm.speed, MVlm.velocityParam), MVlm.smooth*1000, MOVE_ABSOLUTE);

            SP.saveEndIndex = rt_count + (unsigned long)(MVlm.smooth/(0.001*RT_TIMER_PERIOD_MS));

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case CURRENT_SET_CMD_LM:
        {
            FILE_LOG(logSUCCESS) << "Command CURRENT_SET_CMD_LM received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            CSlm.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            CSlm.current = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            CSlm.currentParam = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

//            CS.setAllCurrentFlag = false;

            jCon->SetMoveQCurrent(RKN, CSlm.current, CSlm.smooth*1000, MOVE_ABSOLUTE);
            jCon->SetMoveDAC(RKN, Cur2DAC(CSlm.current,CSlm.currentParam), CSlm.smooth*1000, MOVE_ABSOLUTE);

            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_CUR_SET_CMD_LM;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case CURRENT_SET_CMD_ALL_LM:
        {
            FILE_LOG(logSUCCESS) << "Command CURRENT_SET_CMD_ALL_LM received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            for(int i=0;i<10;i++){
                CSlm.currents[i] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
            }
            CSlm.duration = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[10];
            CSlm.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[11];
            CSlm.currentParam = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[12];
            CSlm.durationIndex = (unsigned long)(CSlm.duration/(0.001*RT_TIMER_PERIOD_MS));
            CSlm.durationCount = 0;

            CSlm.setAllCurrentFlag = true;
            CSlm.currentCount = 0;

//            jCon->SetMoveQCurrent(RKN,CSlm.current,CSlm.smooth*1000, MOVE_ABSOLUTE);
            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_CUR_SET_CMD_ALL_LM;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case CURRENT_STOP_CMD_LM:
        {
            FILE_LOG(logSUCCESS) << "Command CURRENT_STOP_CMD_LM received..";

            CSlm.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            CSlm.current = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            CSlm.currentParam = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            CSlm.setAllCurrentFlag = false;
//            CS.currentCount = 0;

            jCon->SetMoveQCurrent(RKN,CSlm.current,CSlm.smooth*1000, MOVE_ABSOLUTE);
            jCon->SetMoveDAC(RKN, Cur2DAC(CSlm.current,CSlm.currentParam), CSlm.smooth*1000, MOVE_ABSOLUTE);
            SP.saveEndIndex = rt_count + (unsigned long)(CSlm.smooth/(0.001*RT_TIMER_PERIOD_MS));

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }

        case FOC_NULLING:
        {
            FILE_LOG(logSUCCESS) << "Command FOC_NULLING received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            int id, ch, onOff;
            id = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            ch = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            onOff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];

//            FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }

            if(id==-1){
                for(int i=0;i<NO_OF_JOINTS;i++){
                    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = i;
                    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1; //On
                    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FOC_NULLING;
                    while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                    {
                        usleep(500*1000); //Wait for 0.5 sec
                    }
                    usleep(50*1000);
                }
            }
            else{
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1; //On
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FOC_NULLING;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(2100*1000);
                }
                usleep(50*1000);
            }

            FILE_LOG(logSUCCESS) << "FOC NULLING DONE";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case FOC_CONTROL_ENABLE:
        {
            FILE_LOG(logSUCCESS) << "Command FOC_CONTROL_ENABLE received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            int id, ch, onOff, mode;
            id = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            ch = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            onOff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
            mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];

//            BP.id = id;
//            BP.ch = ch;

            /*
            FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            */
            /*
            FILE_LOG(logSUCCESS) << "2: FET OFF";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            FILE_LOG(logSUCCESS) << "3: FET ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//FET ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "4: DRIVE MOTOR FOC NULLING";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FOC_NULLING;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(2100*1000);
            }
            usleep(50*1000);
            */

            /*
            FILE_LOG(logSUCCESS) << "4: UPDATE REF";
            for(int i=0;i<NO_OF_JOINTS;i++)
            {
                double enc = sharedSEN->ENCODER[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch].CurrentPosition;
                jCon->SetJointRefAngle(i,enc);
            }
            */

            switch(mode){
            case SIXSTEP_POSITION_CONTROL:
            {
                FILE_LOG(logSUCCESS) << "5: UPDATE REF";
                double ref = sharedSEN->ENCODER[id][ch].CurrentPosition;
                jCon->SetJointRefAngle(id,ref);
                /*
                FILE_LOG(logSUCCESS) << "6: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                */
//                FILE_LOG(logSUCCESS) << "6: SIXSTEP POSITION CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_POSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "SIXSTEP POSITION CONTROL ON";
                break;
            }
            case SIXSTEP_CURRENT_CONTROL:
            {
                FILE_LOG(logSUCCESS) << "5: UPDATE REF";
                double ref = 0.0;
                jCon->SetJointRefQCurrent(id,ref);
                FILE_LOG(logSUCCESS) << "6: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
//                FILE_LOG(logSUCCESS) << "6: SIXSTEP CURRENT CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "SIXSTEP CURRENT CONTROL ON";
                break;
            }
            case SIXSTEP_CURRENT_POSITION_CONTROL:
            {
                FILE_LOG(logSUCCESS) << "5: UPDATE REF";
                double ref = sharedSEN->ENCODER[id][ch].CurrentPosition;
                jCon->SetJointRefAngle(id,ref);
                FILE_LOG(logSUCCESS) << "6: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
//                FILE_LOG(logSUCCESS) << "6: SIXSTEP CURRENT POSITION CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENTPOSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "SIXSTEP CURRENT POSITION CONTROL ON";
                break;
            }
            /*
            case FOC_NULLING:
            {
//                FILE_LOG(logSUCCESS) << "6: FOC NULLING";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FOC_NULLING;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(2100*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC NULLING DONE";
                break;
            }
            */
            case FOC_POSITION_CONTROL:
            {
                FILE_LOG(logSUCCESS) << "5: UPDATE REF";
                double ref = sharedSEN->ENCODER[id][ch].CurrentPosition;
                jCon->SetJointRefAngle(id,ref);
                FILE_LOG(logSUCCESS) << "6: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
//                FILE_LOG(logSUCCESS) << "6: FOC POSITION CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_POSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC POSITION CONTROL ON";
                break;
            }
            case FOC_VELOCITY_CONTROL:
            {
                FILE_LOG(logSUCCESS) << "5: REF DISABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = false;//off
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "6: UPDATE REF";
                double ref = sharedSEN->ENCODER[id][ch].CurrentPosition;
                jCon->SetJointRefAngle(id,ref);
//                  FILE_LOG(logSUCCESS) << "6: FOC VELOCITY CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_POSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1; // Velocity mode
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_COCOA_CHANGE_POS_MODE;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "7: UPDATE REF";
                jCon->SetJointRefAngle(id,0.0);
                FILE_LOG(logSUCCESS) << "8: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC VELOCITY CONTROL ON";


                /*
                FILE_LOG(logSUCCESS) << "5: UPDATE REF";
                double ref = 0.0;
                jCon->SetJointRefAngle(RHR,ref);
//                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//All
//                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_RESET;
//                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
//                {
//                    usleep(50*1000);
//                }
//                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "6: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
//                FILE_LOG(logSUCCESS) << "6: FOC VELOCITY CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_VELOCITY;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC VELOCITY CONTROL ON";
                */
                break;
            }
            case FOC_CURRENT_CONTROL:
            {
                FILE_LOG(logSUCCESS) << "5: UPDATE REF";
                double ref = 0.0;
                jCon->SetJointRefQCurrent(id,ref);
                FILE_LOG(logSUCCESS) << "6: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
//                FILE_LOG(logSUCCESS) << "6: FOC CURRENT CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC CURRENT CONTROL ON";
                break;
            }
            case FOC_PWM_CONTROL:
            {
                FILE_LOG(logSUCCESS) << "5: UPDATE REF";
                double ref = 0.0;
                jCon->SetJointRefPWM(id,ref);
                FILE_LOG(logSUCCESS) << "6: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
//                FILE_LOG(logSUCCESS) << "6: FOC PWM CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_PWM;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC PWM CONTROL ON";
                break;
            }

            default:
                break;
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case FOC_CONTROL_DISABLE:
        {
            FILE_LOG(logSUCCESS) << "Command FOC_CONTROL_DISABLE received..";

            int id, ch, onOff, mode;
            id = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            ch = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            onOff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
            mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];

            switch(mode){
            case SIXSTEP_POSITION_CONTROL:
            {
//                double ref = sharedSEN->ENCODER[id][ch].CurrentPosition;
//                jCon->SetJointRefAngle(id,ref);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_POSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "SIXSTEP POSITION CONTROL OFF";
                break;
            }
            case SIXSTEP_CURRENT_CONTROL:
            {
//                double ref = 0.0;
//                jCon->SetJointRefQCurrent(id,ref);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "SIXSTEP CURRENT CONTROL OFF";
                break;
            }
            case SIXSTEP_CURRENT_POSITION_CONTROL:
            {
//                double ref = sharedSEN->ENCODER[id][ch].CurrentPosition;
//                jCon->SetJointRefAngle(id,ref);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENTPOSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "SIXSTEP CURRENT POSITION CONTROL OFF";
                break;
            }
            case FOC_POSITION_CONTROL:
            {
//                double ref = sharedSEN->ENCODER[id][ch].CurrentPosition;
//                jCon->SetJointRefAngle(id,ref);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_POSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC POSITION CONTROL OFF";
                break;
            }
            case FOC_VELOCITY_CONTROL:
            {
//                double ref = 0.0;
//                jCon->SetJointRefAngle(id,ref);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_VELOCITY;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC VELOCITY CONTROL OFF";
                break;
            }
            case FOC_CURRENT_CONTROL:
            {
//                double ref = 0.0;
//                jCon->SetJointRefQCurrent(id,ref);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC CURRENT CONTROL OFF";
                break;
            }
            case FOC_PWM_CONTROL:
            {
//                double ref = 0.0;
//                jCon->SetJointRefPWM(id,ref);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_PWM;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "FOC PWM CONTROL OFF";
                break;
            }
            default:
                break;
            }

//            FILE_LOG(logSUCCESS) << "2: FET OFF";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);

            SP.saveEndIndex = rt_count;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case FOC_CONTROL_ENABLE_LM:
        {
            FILE_LOG(logSUCCESS) << "Command FOC_CONTROL_ENABLE_LM received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            int id, ch, onOff, mode;
            id = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            ch = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            onOff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
            mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];

            /*
            FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            FILE_LOG(logSUCCESS) << "2: FET OFF";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;//all
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;//all
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            FILE_LOG(logSUCCESS) << "3: FET ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;//all
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;//all
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//FET ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "4: UPDATE REF";
            for(int i=0;i<NO_OF_JOINTS;i++)
            {
                double enc = sharedSEN->ENCODER[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch].CurrentPosition;
                jCon->SetJointRefAngle(i,enc);
            }
            FILE_LOG(logSUCCESS) << "5: REF ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            */

            switch(mode){
            case VELOCITY_CONTROL:
            {
                double ref = 0.0;
                jCon->SetJointRefAngle(RKN,ref);
                jCon->SetJointRefDAC(RKN,ref);
                jCon->SetJointRefQCurrent(RKN,ref); // Protection
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "VELOCITY CONTROL ON";
                break;
            }
            case CURRENT_CONTROL:
            {
                double ref = 0.0;
                jCon->SetJointRefQCurrent(RKN,ref);
                jCon->SetJointRefDAC(RKN,ref);
                jCon->SetJointRefAngle(RKN,ref); // Protection
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "CURRENT CONTROL ON";
                break;
            }
            default:
                break;
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case FOC_CONTROL_DISABLE_LM:
        {
            FILE_LOG(logSUCCESS) << "Command FOC_CONTROL_DISABLE_LM received..";

            int id, ch, onOff, mode;
            id = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            ch = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            onOff = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2];
            mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3];

            switch(mode){
            case VELOCITY_CONTROL:
            {
                double ref = 0.0;
                jCon->SetMoveJoint(RKN, ref, MVlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, ref,  MVlm.smooth*1000, MOVE_ABSOLUTE);

//                cout << "t00 " << endl;
                while(jCon->Joints[RKN]->AskMoveFlag4()==STILL_MOVING){
//                    usleep(50*1000);
                }
//                cout << "t01 " << endl;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "VELOCITY CONTROL OFF";
                break;
            }
            case CURRENT_CONTROL:
            {
                double ref = 0.0;
                jCon->SetMoveQCurrent(RKN, ref, CSlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, ref,  CSlm.smooth*1000, MOVE_ABSOLUTE);

//                cout << "t03 " << endl;
                while(jCon->Joints[RKN]->AskMoveFlag4()==STILL_MOVING){
//                    usleep(50*1000);
                }
//                cout << "t04 " << endl;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = onOff;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "CURRENT CONTROL OFF";
                break;
            }
            default:
                break;
            }

            SP.saveEndIndex = rt_count;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }

        case EVAL_RUN_VELOCITY_CURRENT:
        {
            FILE_LOG(logSUCCESS) << "Command EVAL_RUN_VELOCITY_CURRENT received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            /*
            FILE_LOG(logSUCCESS) << "2: FET OFF";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            FILE_LOG(logSUCCESS) << "3: FET ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//FET ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "4: DRIVE MOTOR FOC NULLING";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FOC_NULLING;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(2100*1000);
            }
            usleep(50*1000);
            */
            FILE_LOG(logSUCCESS) << "5: REF DISABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = false;//off
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "6: UPDATE REF";
            double ref = sharedSEN->ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
            jCon->SetJointRefAngle(RHR,ref);
//                  FILE_LOG(logSUCCESS) << "6: FOC VELOCITY CONTROL ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1; //On
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_POSITION;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1; // Velocity mode
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_COCOA_CHANGE_POS_MODE;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "7: UPDATE REF";
            jCon->SetJointRefAngle(RHR,0.0);
            FILE_LOG(logSUCCESS) << "8: REF ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "9: DRIVE MOTOR FOC VELOCITY CONTROL ON";
            /*
            FILE_LOG(logSUCCESS) << "5: RESET REF";
            jCon->SetJointRefAngle(RHR,0.0);
            jCon->SetJointRefQCurrent(RKN,0.0);
            jCon->SetJointRefDAC(RKN,0.0);
//            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//All
//            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_RESET;
//            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
//            {
//                usleep(50*1000);
//            }
//            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "6: REF ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "7: DRIVE MOTOR FOC VELOCITY CONTROL ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_VELOCITY;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            */
            FILE_LOG(logSUCCESS) << "10: LOAD MOTOR CURRENT CONTROL ON";
//            FILE_LOG(logSUCCESS) << "8: LOAD MOTOR CURRENT CONTROL ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RKN);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RKN);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);

            FILE_LOG(logSUCCESS) << "INITIALIZATION DONE";
            FILE_LOG(logSUCCESS) << "EVALUATION START";

            MV.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MV.speed =  sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            for(int i=2;i<12;i++){
                CSlm.currents[i-2] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
            }
            CSlm.duration =     sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[12];
            CSlm.smooth =       sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[13];
            CSlm.currentParam = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[14];
            CSlm.durationIndex = (unsigned long)(CSlm.duration/(0.001*RT_TIMER_PERIOD_MS));
            CSlm.durationCount = 0;
            CSlm.setAllCurrentFlag = true;
            CSlm.currentCount = 0;

//            ES.startTorqueIndex = (unsigned long)(MV.smooth/(0.001*RT_TIMER_PERIOD_MS));
            ES.speedUpPhase = false;
            ES.torquePhase = false;
            ES.speedDownPhase = false;

            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_EVAL_VELCOTIY_CURRENT;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case EVAL_RUN_CURRENT_VELOCITY:
        {
            FILE_LOG(logSUCCESS) << "Command EVAL_RUN_CURRENT_VELOCITY received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            /*
            FILE_LOG(logSUCCESS) << "2: FET OFF";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            FILE_LOG(logSUCCESS) << "3: FET ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//FET ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "4: DRIVE MOTOR FOC NULLING";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FOC_NULLING;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(2100*1000);
            }
            usleep(50*1000);
            */
            FILE_LOG(logSUCCESS) << "5: RESET REF";
            jCon->SetJointRefQCurrent(RHR,0.0);
            jCon->SetJointRefAngle(RKN,0.0);
            jCon->SetJointRefDAC(RKN,0.0);
            FILE_LOG(logSUCCESS) << "6: REF ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "7: DRIVE MOTOR FOC CURRENT CONTROL ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "8: LOAD MOTOR VELOCITY CONTROL ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RKN);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RKN);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);

            FILE_LOG(logSUCCESS) << "INITIALIZATION DONE";
            FILE_LOG(logSUCCESS) << "EVALUATION START";

            MVlm.smooth = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            MVlm.speed =  sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            MVlm.velocityParam = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];

            for(int i=3;i<13;i++){
                CS.currents[i-3] = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
            }
            CS.duration =     sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[13];
            CS.smooth =       sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[14];
            CS.durationIndex = (unsigned long)(CS.duration/(0.001*RT_TIMER_PERIOD_MS));
            CS.durationCount = 0;
            CS.setAllCurrentFlag = true;
            CS.currentCount = 0;

//            ES.startTorqueIndex = (unsigned long)(MVlm.smooth/(0.001*RT_TIMER_PERIOD_MS));
            ES.speedUpPhase = false;
            ES.torquePhase = false;
            ES.speedDownPhase = false;

            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_EVAL_CURRENT_VELOCITY;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case EVAL_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command EVAL_STOP received..";

            int mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];

            switch(mode){
            case VELOCITY_CURRENT:
            {
                ES.speedUpPhase = false;
                ES.torquePhase = false;
                ES.speedDownPhase = false;
//                CSlm.setAllCurrentFlag = false;
                jCon->SetMoveQCurrent(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
                while(jCon->Joints[RKN]->AskMoveFlag4()==STILL_MOVING){
                    // Wait for torque being zero
                }
                jCon->SetMoveJoint(RHR,0.0,MV.smooth*1000, MOVE_ABSOLUTE);
                SP.saveEndIndex = rt_count + (unsigned long)(MV.smooth/(0.001*RT_TIMER_PERIOD_MS));
//                cout << "rt_count = " << rt_count << endl;
//                cout << "SP.saveEndIndex = " << SP.saveEndIndex << endl;
//                SP.saveEndIndex = rt_count;

                FILE_LOG(logSUCCESS) << "DRIVE MOTOR FOC VELOCITY CONTROL OFF";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//Off
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_VELOCITY;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "LOAD MOTOR CURRENT CONTROL OFF";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RKN);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RKN);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//Off
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);

                break;
            }
            case CURRENT_VELOCITY:
            {
                ES.speedUpPhase = false;
                ES.torquePhase = false;
                ES.speedDownPhase = false;
//                CS.setAllCurrentFlag = false;
                jCon->SetMoveQCurrent(RHR, 0.0, CS.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RHR, 0.0, CS.smooth*1000, MOVE_ABSOLUTE);
                while(jCon->Joints[RHR]->AskMoveFlag4()==STILL_MOVING){
                    // Wait for torque being zero
                }
                jCon->SetMoveJoint(RKN,0.0,MVlm.smooth*1000, MOVE_ABSOLUTE);
                SP.saveEndIndex = rt_count + (unsigned long)(MVlm.smooth/(0.001*RT_TIMER_PERIOD_MS));

                FILE_LOG(logSUCCESS) << "DRIVE MOTOR FOC CURRENT CONTROL OFF";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//Off
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "LOAD MOTOR VELOCITY CONTROL OFF";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RKN);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RKN);
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//Off
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                break;
            }
            default:
                break;
            }

//            FILE_LOG(logSUCCESS) << "2: FET OFF";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);

            FILE_LOG(logSUCCESS) << "EVALUATION is Stopped";
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case LOGGING_DATA:
        {
            FILE_LOG(logSUCCESS) << "Command LOGGING_DATA received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "2: REF ENABLE";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            FILE_LOG(logSUCCESS) << "3: FOC CURRENT CONTROL ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1; //All
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1; //On
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);


            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_LOGGING_DATA;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case LOGGING_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command LOGGING_STOP received..";

            SP.saveEndIndex = rt_count;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case SINE_REFERENCE_START:
        {
            FILE_LOG(logSUCCESS) << "Command SINE_REFERENCE_START received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            /*
            int id = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            int ch = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
            {
                FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                FILE_LOG(logSUCCESS) << "2: UPDATE REF";
                double ref = 0.0;
                jCon->SetJointRefQCurrent(RHR,ref);
                FILE_LOG(logSUCCESS) << "3: REF ENABLE";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                FILE_LOG(logSUCCESS) << "4: FOC CURRENT CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = id;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = ch;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1; //On
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
            }
            */

            SS.repetitiveFlag = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            SS.amplitude_high = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            SS.frequency_high = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

            if(SS.repetitiveFlag==false){
                jCon->SetMoveSine(RHR, SS.amplitude_high, 1000/SS.frequency_high, MOVE_ABSOLUTE);
            }

            rt_count = 0;
            SP.saveEndIndex = SP.num_data_idx;
            OPERATION_MODE = OPERATION_SINE_REFERENCE_SET_CMD;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case SINE_REFERENCE_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command SINE_REFERENCE_STOP received..";

//            SP.saveEndIndex = rt_count;
            SS.repetitiveFlag = false;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case PERFORMANCE_TEST_MOTION:
        {
            FILE_LOG(logSUCCESS) << "Command PERFORMANCE_TEST_MOTION received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            char mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            if(mode == 0){
                cout<<"GOTO HOME POSITION"<<endl;
                int mtime = 3000;
                jCon->SetMoveJoint(RHR, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RHP, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RKN, 0.0, mtime, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LHR, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LHP, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LKN, 0.0, mtime, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(RSR, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RSP, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(REB, 0.0, mtime, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LSR, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LSP, 0.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LEB, 0.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHR, Oii.q2m_rhr(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHP, 0.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LHR, Oii.q2m_lhr(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LHP, 0.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(RSR, Oii.q2m_rhr(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RSP, 0.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(REB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LSR, Oii.q2m_lhr(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LSP, 0.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LEB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

                usleep(mtime*1000);
            }else if(mode == 1){
                cout<<"GOTO POSE POSITION"<<endl;
                int mtime = 3000;
                jCon->SetMoveJoint(RHR, -5.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RHP, 50.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RKN, 0.0, mtime, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LHR, 5.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LHP, 50.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LKN, 0.0, mtime, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(RSR, -5.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RSP, 50.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(REB, 0.0, mtime, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LSR, 5.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LSP, 50.0, mtime, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LEB, 0.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHP, 50.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LHR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LHP, 50.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(RSR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RSP, 50.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(REB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LSR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LSP, 50.0, mtime, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LEB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

                usleep(mtime*1000);
            }else if(mode == 2){
                int mtime = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                int mnum = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
                double mscale = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                cout<<"REPEAT mstime ="<<mtime<<" and num ="<<mnum<<" and scale"<<mscale<<endl;

                double rot = 40.*12./360.*mscale;
                double tt = mtime*2/1000.;
                double freq = 1/tt*2*3.141592;
                double rpm = freq*rot*60;
                cout<<"MAX RPM : "<<rpm<<endl;

                for(int k=0;k<mnum;k++){
                    jCon->SetMoveJoint(RHR, -10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RKN, (0-40*mscale), mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LHR, 10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LKN, (0-40*mscale), mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(RSR, -10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(REB, (0-40*mscale), mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LSR, 10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LEB, (0-40*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHR, Oii.q2m_rhr(-10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RKN, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LHR, Oii.q2m_lhr(10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LKN, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(RSR, Oii.q2m_rhr(-10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(REB, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LSR, Oii.q2m_lhr(10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LEB, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
                    usleep(mtime*1000);

                    jCon->SetMoveJoint(RHR, -5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RHP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RKN, 0.0, mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LHR, 5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LHP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LKN, 0.0, mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(RSR, -5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RSP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(REB, 0.0, mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LSR, 5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LSP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LEB, 0.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LHR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LHP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(RSR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RSP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(REB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LSR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LSP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LEB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
                    usleep(mtime*1000);
                }

            }else if(mode == 3){
                int mtime = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                int mnum = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
                double mscale = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                cout<<"REPEAT mstime ="<<mtime<<" and num ="<<mnum<<" and scale"<<mscale<<endl;

                double rot = 40.*12./360.*mscale;
                double tt = mtime*2/1000.;
                double freq = 1/tt*2*3.141592;
                double rpm = freq*rot*60;
                cout<<"MAX RPM : "<<rpm<<endl;

                // init
                jCon->SetMoveJoint(RHR, -10.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RHP, (50+25*mscale), 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RKN, (0-40*mscale), 2000, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LSR, 10.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LSP, (50+25*mscale), 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LEB, (0-40*mscale), 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHR, Oii.q2m_rhr(-10.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHP, (50+25*mscale), 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RKN, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LHR, Oii.q2m_lhr(10.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LHP, (50+25*mscale), 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LKN, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
                usleep(2000*1000);

                for(int k=0;k<mnum;k++){
                    jCon->SetMoveJoint(RHR, -5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RHP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RKN, 0.0, mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LHR, 10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LKN, (0-40*mscale), mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(RSR, -10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(REB, (0-40*mscale), mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LSR, 5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LSP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LEB, 0.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LHR, Oii.q2m_lhr(10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LKN, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(RSR, Oii.q2m_rhr(-10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(REB, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LSR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LSP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LEB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

                    usleep(mtime*1000);
                    jCon->SetMoveJoint(RHR, -10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RKN, (0-40*mscale), mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LHR, 5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LHP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LKN, 0.0, mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(RSR, -5.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(RSP, 50.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(REB, 0.0, mtime, MOVE_ABSOLUTE);

                    jCon->SetMoveJoint(LSR, 10.0, mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
                    jCon->SetMoveJoint(LEB, (0-40*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHR, Oii.q2m_rhr(-10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RHP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RKN, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LHR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LHP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(RSR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(RSP, 50.0, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(REB, Oii.q2m_kn(0.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);

//                    jCon->SetMoveJoint(LSR, Oii.q2m_lhr(10.0*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LSP, (50+25*mscale), mtime, MOVE_ABSOLUTE);
//                    jCon->SetMoveJoint(LEB, Oii.q2m_kn((0-40*mscale)*D2Rf)*R2Df, mtime, MOVE_ABSOLUTE);
                    usleep(mtime*1000);
                }
                // finish
                jCon->SetMoveJoint(RHR, -5.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RHP, 50.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RKN, 0.0, 2000, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LHR, 5.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LHP, 50.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LKN, 0.0, 2000, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(RSR, -5.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RSP, 50.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(REB, 0.0, 2000, MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LSR, 5.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LSP, 50.0, 2000, MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LEB, 0.0, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RHP, 50.0, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LHR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LHP, 50.0, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LKN, Oii.q2m_kn(0.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(RSR, Oii.q2m_rhr(-5.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RSP, 50.0, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(REB, Oii.q2m_kn(0.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LSR, Oii.q2m_lhr(5.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LSP, 50.0, 2000, MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LEB, Oii.q2m_kn(0.0*D2Rf)*R2Df, 2000, MOVE_ABSOLUTE);
                usleep(2000*1000);
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case PERFORMANCE_TEST_CURRENT:
        {
            FILE_LOG(logSUCCESS) << "Command PERFORMANCE_TEST_CURRENT received..";
            int t_bno = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            double t_ref = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            cout<<"Ref BNO = "<<t_bno<<" and Ref Current = "<<t_ref<<endl;
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}


//==============================//
// Task Thread
//==============================//
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
//        cout << "OPERATION_MODE = " << OPERATION_MODE << endl;
        switch (OPERATION_MODE) {
        case OPERATION_NONE:
        {
            break;
        }
        case OPERATION_POS_MOVE:
        {
            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference; // Reference position
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // DM Encoder position
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // DM Encoder velocity
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition; // LM Encoder position
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity; // LM Encoder velocity

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "MP.period = " << MP.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(1);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllPosJoint();
            break;
        }
        case OPERATION_POS_MOVE_REPETITIVE:
        {
            if(jCon->Joints[RHR]->AskMoveFlag1()==MOVE_DONE){
                if(MP.repetitiveFlag==true){
                    if(MP.repetitiveCount%2==0){
                        jCon->SetMovePosJoint(RHR, MP.finalAngle, MP.period*1000, MP.smooth*1000, MP.mode);
                    }else{
                        jCon->SetMovePosJoint(RHR, MP.initialAngle, MP.period*1000, MP.smooth*1000, MP.mode);
                    }
                    MP.repetitiveCount++;
                }else{
                    SP.saveEndIndex = rt_count;
                }
            }

            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference; // Reference position
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // Encoder position
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // Encoder velocity
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition; // LM Encoder position
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity; // LM Encoder velocity

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "MP.period = " << MP.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(1);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllPosJoint();
            break;
        }
        case OPERATION_VEL_JOGGING:
        {
            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference; // Reference velocity
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // Encoder position
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // Encoder velocity
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition; // LM Encoder position
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity; // LM Encoder velocity
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;  // DM Torque
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;  // LM Torque

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "MV.smooth = " << MP.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(2);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllJoint();
            break;
        }
        case OPERATION_CUR_SET_CMD:
        {
            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
//            SP.DataBuf[1][SP.saveIndex]= sharedREF->COCOAQCurrent_REF[PODO_NO][RHR]; // Reference current
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentQCurrentReference; // Reference current
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent; // Board current
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;           // DM Torque
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;          // LM Torque

//            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // Encoder position
//            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // Encoder velocity
//            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition; // LM Encoder position
//            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity; // LM Encoder velocity

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "CS.smooth = " << CS.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(3);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllQCurrent();
            break;
        }
        case OPERATION_CUR_SET_CMD_ALL:
        {
            //// Increasing step sequence
            if(jCon->Joints[RHR]->AskMoveFlag2()==MOVE_DONE){
                if(CS.setAllCurrentFlag==true){
                    CS.durationCount++;
                    if(CS.durationCount>=CS.durationIndex){
                        if(CS.currentCount<10){
                            jCon->SetMoveQCurrent(RHR, CS.currents[CS.currentCount], CS.smooth*1000, MOVE_ABSOLUTE);
                            CS.currentCount++;
                        }else{
                            jCon->SetMoveQCurrent(RHR, 0.0, PT.ratioStop*CS.smooth*1000, MOVE_ABSOLUTE);
                            CS.setAllCurrentFlag = false;
                        }
//                        CS.currentCount++;
                        CS.durationCount = 0;
                    }
                }else{
                    CS.currentCount = 0;
                    SP.saveEndIndex = rt_count;
                }
            }

            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
//            SP.DataBuf[1][SP.saveIndex]= sharedREF->COCOAQCurrent_REF[PODO_NO][RHR]; // Reference current
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentQCurrentReference; // Reference current
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;           // Board current
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;           // DM Torque
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;           // LM Torque
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;          // DM Encoder position
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;          // DM Encoder velocity
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition;          // LM Encoder position
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentVelocity;          // LM Encoder velocity

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "CS.smooth = " << CS.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(3);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllQCurrent();
            break;
        }
        case OPERATION_PWM_SET_CMD:
        {
            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
//            SP.DataBuf[1][SP.saveIndex]= sharedREF->JointFFpwm[PODO_NO][MC_GetID(RHR)][MC_GetCH(RHR)]; // Reference PWM
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].PWMffout; // Board PWM ??
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].PWMin; // Board PWM ??
//            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // Encoder position
//            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // Encoder velocity

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "PS.smooth = " << PS.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(4);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllPWM();
            break;
        }
        case OPERATION_PWM_SET_CMD_ALL:
        {
            //// Increasing step sequence
            if(jCon->Joints[RHR]->AskMoveFlag3()==MOVE_DONE){
                if(PS.setAllPWMFlag==true){
                    PS.durationCount++;
                    if(PS.durationCount>=PS.durationIndex){
                        if(PS.pwmCount<10){
                            jCon->SetMovePWM(RHR, PS.pwms[PS.pwmCount], PS.smooth*1000, MOVE_ABSOLUTE);
                            PS.pwmCount++;
                        }else{
                            jCon->SetMovePWM(RHR, 0.0, PS.smooth*1000, MOVE_ABSOLUTE);
                            PS.setAllPWMFlag = false;
                        }
//                        PS.pwmCount++;
                        PS.durationCount = 0;
                    }
                }else{
                    PS.pwmCount = 0;
                    SP.saveEndIndex = rt_count;
                }
            }

            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
//            SP.DataBuf[1][SP.saveIndex]= sharedREF->JointFFpwm[PODO_NO][MC_GetID(RHR)][MC_GetCH(RHR)]; // Reference PWM
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].PWMffout; // Board PWM ??
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].PWMin; // Board PWM ??
//            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // Encoder position
//            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // Encoder velocity

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "PS.smooth = " << PS.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(4);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllPWM();
            break;
        }
        case OPERATION_VEL_JOGGING_LM:
        {
            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentReference; // Reference velocity
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition; // DM Encoder position
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentVelocity; // DM Encoder velocity
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition; // LM Encoder position
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentVelocity; // LM Encoder velocity
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentCurrentReference; // Reference DAC
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;           // DM Torque
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;          // LM Torque

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "MV.smooth = " << MP.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(5);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllJoint();
//            jCon->MoveAllDAC();
            break;
        }
        case OPERATION_CUR_SET_CMD_LM:
        {
            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
//            SP.DataBuf[1][SP.saveIndex]= sharedREF->COCOAQCurrent_REF[PODO_NO][RKN]; // Reference current
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentQCurrentReference; // Reference current
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentCurrent; // Board current
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentCurrentReference; // Reference DAC
//            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition; // Encoder position
//            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentVelocity; // Encoder velocity
//            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition; // LM Encoder position
//            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentVelocity; // LM Encoder velocity
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;           // DM Torque
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;          // LM Torque

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "CS.smooth = " << CS.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(6);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllQCurrent();
//            jCon->MoveAllDAC();
            break;
        }
        case OPERATION_CUR_SET_CMD_ALL_LM:
        {
            //// Increasing step sequence
            if(jCon->Joints[RKN]->AskMoveFlag2()==MOVE_DONE){
                if(CSlm.setAllCurrentFlag==true){
                    CSlm.durationCount++;
                    if(CSlm.durationCount>=CSlm.durationIndex){
                        if(CSlm.currentCount<10){
                            jCon->SetMoveQCurrent(RKN, CSlm.currents[CSlm.currentCount], CSlm.smooth*1000, MOVE_ABSOLUTE);
                            jCon->SetMoveDAC(RKN, Cur2DAC(CSlm.currents[CSlm.currentCount],CSlm.currentParam), CSlm.smooth*1000, MOVE_ABSOLUTE);
                            CSlm.currentCount++;
                        }else{
                            jCon->SetMoveQCurrent(RKN, 0.0, PT.ratioStop*CSlm.smooth*1000, MOVE_ABSOLUTE);
                            jCon->SetMoveDAC(RKN, 0.0, PT.ratioStop*CSlm.smooth*1000, MOVE_ABSOLUTE);
                            CSlm.setAllCurrentFlag = false;
                        }
//                        CSlm.currentCount++;
                        CSlm.durationCount = 0;
                    }
                }else{
                    CSlm.currentCount = 0;
                    SP.saveEndIndex = rt_count;
                }
            }

            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
//            SP.DataBuf[1][SP.saveIndex]= sharedREF->COCOAQCurrent_REF[PODO_NO][RKN]; // Reference current
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentQCurrentReference; // Reference current
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentCurrentReference;  // Reference DAC
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;           // DM Torque
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;           // LM Torque
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition;          // DM Encoder position
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentVelocity;          // DM Encoder velocity
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentPosition;          // LM Encoder position
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentVelocity;          // LM Encoder velocity


            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
//                cout << "rt_count = " << rt_count << endl;
//                cout << "CS.smooth = " << CS.period << endl;
//                cout << "Goal_count = " << SP.saveEndIndex << endl;
                SP.SaveFile(6);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllQCurrent();
//            jCon->MoveAllDAC();
            break;
        }
        case OPERATION_EVAL_VELCOTIY_CURRENT:
        {
            /*
            if(rt_count==0){ // Increasing speed to desired velocity
                jCon->SetMoveJoint(RHR, MV.speed, MV.smooth*1000, MOVE_ABSOLUTE);
            }

            if(rt_count >= ES.startTorqueIndex){ // Desired velocity achieved
                if(jCon->Joints[RKN]->AskMoveFlag2()==MOVE_DONE){
                    if(CSlm.setAllCurrentFlag==true){
                        CSlm.durationCount++;
                        if(CSlm.durationCount>=CSlm.durationIndex){
                            CSlm.durationCount = 0;
                            if(CSlm.currentCount<10){
                                jCon->SetMoveQCurrent(RKN, CSlm.currents[CSlm.currentCount], CSlm.smooth*1000, MOVE_ABSOLUTE);
                                jCon->SetMoveDAC(RKN, Cur2DAC(CSlm.currents[CSlm.currentCount],CSlm.currentParam), CSlm.smooth*1000, MOVE_ABSOLUTE);
                                CSlm.currentCount++;
                            }else{
                                jCon->SetMoveQCurrent(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
                                jCon->SetMoveDAC(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
                                CSlm.setAllCurrentFlag = false;
                                CSlm.currentCount = 0;
                            }
                        }
                    }else{
                        jCon->SetMoveJoint(RHR, 0.0, MV.smooth*1000, MOVE_ABSOLUTE);
                    }
                }
            }

            if(CSlm.setAllCurrentFlag==false && jCon->Joints[RHR]->AskMoveFlag()==MOVE_DONE){
                SP.saveEndIndex = rt_count;
            }
            */


            if(rt_count==0){
                jCon->SetMoveJoint(RHR, MV.speed, MV.smooth*1000, MOVE_ABSOLUTE);
                ES.speedUpPhase = true;
            }

            if(ES.speedUpPhase==true && jCon->Joints[RHR]->AskMoveFlag()==MOVE_DONE){
                ES.speedUpPhase = false;
                ES.torquePhase = true;
            }

            if(ES.torquePhase==true){
                if(jCon->Joints[RKN]->AskMoveFlag2()==MOVE_DONE){
                    if(CSlm.setAllCurrentFlag==true){
                        CSlm.durationCount++;
                        if(CSlm.durationCount>=CSlm.durationIndex){
                            CSlm.durationCount = 0;
                            if(CSlm.currentCount<10){
                                jCon->SetMoveQCurrent(RKN, CSlm.currents[CSlm.currentCount], CSlm.smooth*1000, MOVE_ABSOLUTE);
                                jCon->SetMoveDAC(RKN, Cur2DAC(CSlm.currents[CSlm.currentCount],CSlm.currentParam), CSlm.smooth*1000, MOVE_ABSOLUTE);
                                CSlm.currentCount++;
                            }else{
                                jCon->SetMoveQCurrent(RKN, 0.0, PT.ratioStop*CSlm.smooth*1000, MOVE_ABSOLUTE);
                                jCon->SetMoveDAC(RKN, 0.0, PT.ratioStop*CSlm.smooth*1000, MOVE_ABSOLUTE);
                                CSlm.setAllCurrentFlag = false;
                                CSlm.currentCount = 0;
                            }
                        }
                    }else{
                        ES.torquePhase = false;
                        jCon->SetMoveJoint(RHR, 0.0, MV.smooth*1000, MOVE_ABSOLUTE);
                        ES.speedDownPhase = true;
                    }
                }
            }

            if(ES.speedDownPhase==true && jCon->Joints[RHR]->AskMoveFlag()==MOVE_DONE){
                ES.speedDownPhase = false;
                SP.saveEndIndex = rt_count;
            }

            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;         // Reference Velocity
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;          // Encoder DM Position
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;          // Encoder DM Velocity
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;          // Encoder LM Position
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;          // Encoder LM Velocity
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentQCurrentReference; // Reference Torque
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentCurrentReference;  // Reference DAC[V]
            SP.DataBuf[9][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;           // DM Torque
            SP.DataBuf[10][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;          // LM Torque

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
                FILE_LOG(logSUCCESS) << "EVALUATION DONE";
                SP.SaveFile(7);
                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_EVAL_CURRENT_VELOCITY:
        {
            /*
            if(rt_count==0){ // Increasing speed to desired velocity
                jCon->SetMoveJoint(RKN, MVlm.speed, MVlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, Vel2DAC(MVlm.speed, MVlm.velocityParam), MVlm.smooth*1000, MOVE_ABSOLUTE);
            }

            if(rt_count >= ES.startTorqueIndex){ // Desired velocity achieved
                if(jCon->Joints[RHR]->AskMoveFlag2()==MOVE_DONE){
                    if(CS.setAllCurrentFlag==true){
                        CS.durationCount++;
                        if(CS.durationCount>=CS.durationIndex){
                            CS.durationCount = 0;
                            if(CS.currentCount<10){
                                jCon->SetMoveQCurrent(RHR, CS.currents[CS.currentCount], CS.smooth*1000, MOVE_ABSOLUTE);
                                CS.currentCount++;
                            }else{
                                jCon->SetMoveQCurrent(RHR, 0.0, CS.smooth*1000, MOVE_ABSOLUTE);
                                CS.setAllCurrentFlag = false;
                                CS.currentCount = 0;
                            }
                        }
                    }else{
                        jCon->SetMoveJoint(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
                        jCon->SetMoveDAC(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
                    }
                }
            }

            if(CS.setAllCurrentFlag==false && jCon->Joints[RKN]->AskMoveFlag4()==MOVE_DONE){
                SP.saveEndIndex = rt_count;
            }
            */

            if(rt_count==0){
                jCon->SetMoveJoint(RKN, MVlm.speed, MVlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, Vel2DAC(MVlm.speed, MVlm.velocityParam), MVlm.smooth*1000, MOVE_ABSOLUTE);
                ES.speedUpPhase = true;
            }

            if(ES.speedUpPhase==true && jCon->Joints[RKN]->AskMoveFlag()==MOVE_DONE){
                ES.speedUpPhase = false;
                ES.torquePhase = true;
            }

            if(ES.torquePhase==true){
                if(jCon->Joints[RHR]->AskMoveFlag2()==MOVE_DONE){
                    if(CS.setAllCurrentFlag==true){
                        CS.durationCount++;
                        if(CS.durationCount>=CS.durationIndex){
                            CS.durationCount = 0;
                            if(CS.currentCount<10){
                                jCon->SetMoveQCurrent(RHR, CS.currents[CS.currentCount], CS.smooth*1000, MOVE_ABSOLUTE);
                                CS.currentCount++;
                            }else{
                                jCon->SetMoveQCurrent(RHR, 0.0, PT.ratioStop*CS.smooth*1000, MOVE_ABSOLUTE);
                                CS.setAllCurrentFlag = false;
                                CS.currentCount = 0;
                            }
                        }
                    }else{
                        ES.torquePhase = false;
                        jCon->SetMoveJoint(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
                        jCon->SetMoveDAC(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
                        ES.speedDownPhase = true;
                    }
                }
            }

            if(ES.speedDownPhase==true && jCon->Joints[RKN]->AskMoveFlag()==MOVE_DONE){
                ES.speedDownPhase = false;
                SP.saveEndIndex = rt_count;
            }

            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrentReference;  // Reference Current
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;          // Encoder DM Position
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;          // Encoder DM Velocity
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;          // Encoder LM Position
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;          // Encoder LM Velocity
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentReference;         // Reference Velocity
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentCurrentReference;  // Reference DAC[V]
            SP.DataBuf[9][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;           // DM Torque
            SP.DataBuf[10][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;          // LM Torque

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
                FILE_LOG(logSUCCESS) << "EVALUATION DONE";
                SP.SaveFile(8);
                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_LOGGING_DATA:
        {
            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RKN)][MC_GetCH(RKN)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHR)][MC_GetCH(LHR)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RSR)][MC_GetCH(RSR)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RSP)][MC_GetCH(RSP)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[9][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(REB)][MC_GetCH(REB)].CurrentCurrent;           // DM Board Active Current
            SP.DataBuf[10][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(LSR)][MC_GetCH(LSR)].CurrentCurrent;          // DM Board Active Current
            SP.DataBuf[11][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(LSP)][MC_GetCH(LSP)].CurrentCurrent;          // DM Board Active Current
            SP.DataBuf[12][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(LEB)][MC_GetCH(LEB)].CurrentCurrent;          // DM Board Active Current

            if(rt_count >= SP.saveEndIndex){
                FILE_LOG(logSUCCESS) << "3: FOC CURRENT CONTROL OFF";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1; //All
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0; //Off
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;

                FILE_LOG(logSUCCESS) << "LOGGING DATA DONE";
                SP.SaveFile(9);
                OPERATION_MODE = OPERATION_NONE;
            }

            SP.saveIndex++;
            if(SP.saveIndex>SP.num_data_idx){
                SP.saveIndex=0;
            }

            break;
        }
        case OPERATION_SINE_REFERENCE_SET_CMD:
        {
            if(jCon->Joints[RHR]->AskMoveFlag5()==MOVE_DONE){
                if(SS.repetitiveFlag==true){
                    jCon->SetMoveSine(RHR, SS.amplitude_high, 1000/SS.frequency_high, MOVE_ABSOLUTE);
                    SS.repetitiveCount++;
                }else{
                    SP.saveEndIndex = rt_count;
                }
            }

            double rt_time = (double)(rt_count*0.001*RT_TIMER_PERIOD_MS);
            SP.DataBuf[0][SP.saveIndex]= rt_time;
            SP.DataBuf[1][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentQCurrentReference; // Reference current
            SP.DataBuf[2][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;  // Board current
            SP.DataBuf[3][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // Encoder position
            SP.DataBuf[4][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // Encoder velocity

            SP.DataBuf[5][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition; // LM Encoder position
            SP.DataBuf[6][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity; // LM Encoder velocity

            SP.DataBuf[7][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;  // DM Torque
            SP.DataBuf[8][SP.saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;  // LM Torque

            SP.saveIndex++;
            if(SP.saveIndex>=SP.num_data_idx){
                SP.saveIndex=0;
            }

            if(rt_count >= SP.saveEndIndex){
                SP.SaveFile(10);
                OPERATION_MODE = OPERATION_NONE;
            }
//            jCon->MoveAllSine();
            break;
        }

        default:
            break;
        }


        /*
        if(fabs(sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity)>=PT.limit_vel_DM){
            FILE_LOG(logSUCCESS) << "Drive Motor Velocity Exceeds the Velocity Limit!!";
//            cout << "DM Velocity = " << sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity << endl;
//            cout << "DM Velocity limit = " << PT.limit_vel_DM << endl;
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;

            if(jCon->Joints[RKN]->AskMoveFlag()==STILL_MOVING){
                jCon->SetMoveJoint(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
            }
        }
        if(fabs(sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity)>=PT.limit_vel_LM){
            FILE_LOG(logSUCCESS) << "Load Motor Velocity Exceeds the Velocity Limit!!";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;

            if(jCon->Joints[RKN]->AskMoveFlag()==STILL_MOVING){
                jCon->SetMoveQCurrent(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, 0.0, MVlm.smooth*1000, MOVE_ABSOLUTE);
            }
        }
        */
        if(fabs(sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1)>=PT.limit_torque_DM){
            FILE_LOG(logSUCCESS) << "Drive Torque Exceeds the Torque Limit!!";
//            if(jCon->Joints[RHR]->AskMoveFlag2()==STILL_MOVING){
//                jCon->SetMoveQCurrent(RHR, 0.0, CS.smooth*1000, MOVE_ABSOLUTE);
//            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;

            if(jCon->Joints[RKN]->AskMoveFlag2()==STILL_MOVING){
                jCon->SetMoveQCurrent(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
            }
        }
        if(fabs(sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2)>=PT.limit_torque_LM){
            FILE_LOG(logSUCCESS) << "Load Torque Exceeds the Torque Limit!!";
//            if(jCon->Joints[RHR]->AskMoveFlag4()==STILL_MOVING){
//                jCon->SetMoveQCurrent(RHR, 0.0, CS.smooth*1000, MOVE_ABSOLUTE);
//            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = MC_GetID(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = MC_GetCH(RHR);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;

            if(jCon->Joints[RKN]->AskMoveFlag2()==STILL_MOVING){
                jCon->SetMoveQCurrent(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
                jCon->SetMoveDAC(RKN, 0.0, CSlm.smooth*1000, MOVE_ABSOLUTE);
            }
        }

        jCon->MoveAllPosJoint(); //position
        jCon->MoveAllJoint();    //velocity
        jCon->MoveAllQCurrent(); //current
        jCon->MoveAllPWM();      //pwm
        jCon->MoveAllDAC();      //dac
        jCon->MoveAllSine();     //sine reference
        rt_count++;
        cntforcountsleep++;
        rt_task_suspend(&rtTaskCon);
    }
}

//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 10*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                sharedSEN2 = *sharedSEN;
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}
