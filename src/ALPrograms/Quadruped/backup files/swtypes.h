#ifndef SWTYPES_H
#define SWTYPES_H
#include "BasicMatrix.h"

//==============================//
// Command
//==============================//
enum DM_CONTROL_COMMAND
{
    EMPTY_DM = 0,
    SIXSTEP_POSITION_CONTROL,
    SIXSTEP_CURRENT_CONTROL,
    SIXSTEP_CURRENT_POSITION_CONTROL,
    FOC_POSITION_CONTROL,
    FOC_VELOCITY_CONTROL,
    FOC_CURRENT_CONTROL,
    FOC_PWM_CONTROL,
    SINE_REFERENCE_CONTROL
};

enum LM_CONTROL_COMMAND
{
    EMPTY_LM = 0,
    VELOCITY_CONTROL,
    CURRENT_CONTROL
};

enum EVAL_CONTROL_COMMAND
{
    EMPTY_EVAL = 0,
    VELOCITY_CURRENT,
    CURRENT_VELOCITY
};

enum CONTROL_MODE_COMMAND
{
    POS_SET_MOVE = 100,
    POS_STOP_MOVE,
    POS_SET_REPETITIVE_MOVE,
    POS_STOP_REPETITIVE_MOVE,
    VEL_SET_JOGGING,
    VEL_STOP_JOGGING,
    CURRENT_SET_CMD,
    CURRENT_SET_CMD_ALL,
    CURRENT_STOP_CMD,
    PWM_SET_CMD,
    PWM_SET_CMD_ALL,
    PWM_STOP_CMD,
    VEL_SET_JOGGING_LM,
    VEL_STOP_JOGGING_LM,
    CURRENT_SET_CMD_LM,
    CURRENT_SET_CMD_ALL_LM,
    CURRENT_STOP_CMD_LM,
    FOC_NULLING,
    FOC_CONTROL_ENABLE,
    FOC_CONTROL_DISABLE,
    FOC_CONTROL_ENABLE_LM,
    FOC_CONTROL_DISABLE_LM,
    EVAL_RUN_VELOCITY_CURRENT,
    EVAL_RUN_CURRENT_VELOCITY,
    EVAL_STOP,
    UPDATE_PROTECTION_PARAMETER,
    LOGGING_DATA,
    LOGGING_STOP,
    SINE_REFERENCE_START,
    SINE_REFERENCE_STOP,
    PERFORMANCE_TEST_MOTION,
    PERFORMANCE_TEST_CURRENT,
};
//==============================//
// Real Time Thread Command
//==============================//
enum RT_COMMAND
{
    OPERATION_NONE = 0,
    OPERATION_POS_MOVE,
    OPERATION_POS_MOVE_REPETITIVE,
    OPERATION_VEL_JOGGING,
    OPERATION_CUR_SET_CMD,
    OPERATION_CUR_SET_CMD_ALL,
    OPERATION_PWM_SET_CMD,
    OPERATION_PWM_SET_CMD_ALL,

    OPERATION_VEL_JOGGING_LM,
    OPERATION_CUR_SET_CMD_LM,
    OPERATION_CUR_SET_CMD_ALL_LM,

    OPERATION_EVAL_VELCOTIY_CURRENT,
    OPERATION_EVAL_CURRENT_VELOCITY,

    OPERATION_LOGGING_DATA,

    OPERATION_SINE_REFERENCE_SET_CMD
};

//==============================//
// Structures
//==============================//
struct BoardIdChParams
{
    int          id;
    int          ch;
};

struct MovePosParams
{
    double          period;
    double          smooth;
    double          angle;
    double          initialAngle;
    double          finalAngle;
    unsigned int    mode;
    bool            repetitiveFlag;
    unsigned long   repetitiveCount;
//    MovePosParams(){ratioStop = 5;}
};

struct MoveVelParams
{
    double smooth;
    double speed;
    double velocityParam; // for CSMS-30BT
    MoveVelParams(){
        smooth = 0.5;
        speed = 0.0;
    }
};

struct CurrentSetParams
{
    double          current;
    double          currents[10];
    unsigned long   currentCount;
    double          smooth;
    double          duration;
    unsigned long   durationIndex;
    unsigned long   durationCount;
    bool            setAllCurrentFlag;
    double          currentParam; // for CSMS-30BT

    CurrentSetParams(){
        current = 0.0;
        for(int i=0;i<10;i++){
            currents[i] = 0.0;
        }
        currentCount = 0;
        smooth = 0.5;
        duration = 1.0;
        durationIndex = 0;
        durationCount = 0;
        setAllCurrentFlag = false;
    }
};

struct PWMSetParams
{
    double          pwm;
    double          pwms[10];
    unsigned long   pwmCount;
    double          smooth;
    double          duration;
    unsigned long   durationIndex;
    unsigned long   durationCount;
    bool            setAllPWMFlag;

    PWMSetParams(){
        pwm = 0.0;
        for(int i=0;i<10;i++){
            pwms[i] = 0.0;
        }
        pwmCount = 0;
        smooth = 0.5;
        duration = 1.0;
        durationIndex = 0;
        durationCount = 0;
        setAllPWMFlag = false;
    }
};

struct EVALSetParams
{
    unsigned long   startTorqueIndex;
    bool            speedUpPhase;
    bool            torquePhase;
    bool            speedDownPhase;

    EVALSetParams(){
        startTorqueIndex = 0;
        speedUpPhase = false;
        torquePhase = false;
        speedDownPhase = false;
    }
};

struct SineReferenceSetParams
{
    double          amplitude_high;
    double          frequency_high;
    double          amplitude_low;
    double          frequency_low;

    bool            repetitiveFlag;
    unsigned long   repetitiveCount;

    SineReferenceSetParams(){
        amplitude_high = 0.0;
        frequency_high = 0.0;
        amplitude_low = 0.0;
        frequency_low = 0.0;
        repetitiveFlag = false;
        repetitiveCount = 0;
    }
};

struct ProtectionParams
{
    double limit_vel_DM;
    double limit_vel_LM;
    double limit_acc_DM;
    double limit_acc_LM;
    double limit_cur_DM;
    double limit_cur_LM;
    double limit_torque_DM;
    double limit_torque_LM;
    int    ratioStop;

    ProtectionParams(){
        limit_torque_DM = 9.0; //[Nm]
        limit_torque_LM = 90.0; //[Nm]
        ratioStop = 2;
    }
};

struct SaveParams
{
    int saveFlag;
    unsigned long saveIndex;
    unsigned long saveEndIndex;
    unsigned long num_data_type;
    unsigned long num_data_idx;
    double DataBuf[100][150000];

    SaveParams()
    {
        saveFlag=0;
        saveIndex=0;
        saveEndIndex=0;
        num_data_type = 100;
        num_data_idx = 150000;
        DataBuf[100][150000];
    }

    void SaveFile(int fnum = 0)
    {
        printf("Experiment finished %d type: %d\n",saveIndex,fnum);
        FILE* fp;
        unsigned int i, j;
        if(fnum==0){
            fp = fopen("SW_Efficiency.txt", "w");
        }else if(fnum==1){
            fp = fopen("SW_Position.txt", "w");
        }else if(fnum==2){
            fp = fopen("SW_Velocity.txt", "w");
        }else if(fnum==3){
            fp = fopen("SW_Current.txt", "w");
        }else if(fnum==4){
            fp = fopen("SW_PWM.txt", "w");
        }else if(fnum==5){
            fp = fopen("SW_Velocity_LM.txt", "w");
        }else if(fnum==6){
            fp = fopen("SW_Current_LM.txt", "w");
        }else if(fnum==7){
            fp = fopen("SW_Eval_Velocity_Current.txt", "w");
        }else if(fnum==8){
            fp = fopen("SW_Eval_Current_Velocity.txt", "w");
        }else if(fnum==9){
            fp = fopen("SW_Logging_Data.txt", "w");
        }else if(fnum==10){
            fp = fopen("SW_Sine_Reference.txt", "w");
        }

        for(i=0 ; i<saveIndex ; i++)
        {
            for(j=0 ; j<num_data_type ; j++){
                fprintf(fp, "%f\t", DataBuf[j][i]);
            }
            fprintf(fp, "\n");
        }
        fclose(fp);

        saveIndex=0;
        saveFlag=0;

        printf("Save done\n");
    }
};

#endif // SWTYPES_H
