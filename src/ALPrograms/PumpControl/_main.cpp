#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicJoint.h"
#include "PumpController_BasicFunctions.h"
#include "PumpControl_ControlFunctions.h"

#include <unistd.h>
#include <cmath>
#include <chrono>
#include <iostream>
using namespace std::chrono;
using namespace std;

#define PODO_AL_NAME       "PumpControl"

#define REALPUMP

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
//JointControlClass       *jCon;


// Program variable
int isTerminated;
int     __IS_WORKING = false;

int     PODO_NO = -1;

bool Flag_ActiveControl = false;

char _BNO = 0;

double P_sine_mag = 0.0;
double P_sine_per = 0.0;
double P_sine_num = 0.0;

double Ps_fin = 1.0;
double Ps_des[MAX_PREVIEW+1] = {Ps_min,0.0,};
double Ps_now = Ps_min+10.0;
char   PsRef_type = 0;
char   PsRef_type_transition = 0; // -1 : sharedREF >> thisAL, 1 : sharedREF << thisAL

double t = 0.0;
double t_change = 10.0;

double InputVoltage = 105.0;
double InitSpeed = 0.0;
double InitTime = 10.0;

double wp_ref_max = PUMPSPEED_MAX/100.0*InputVoltage*0.95; // [rpm] 0.90 = 90% limit
double wp_ref_min = PUMPSPEED_MIN; // [rpm]

double wp_now = 0.0;
double wdp_now = 0.0;
double wp_ref = 0.0;

extern double Qflow_ref;
extern double Qflow_ref_JointVel;
extern double Qflow_ref_Leakage;
extern double Qflow_ref_Accumulator;
extern double Qflow_ref_Feedback;

double IDX = 0.0;

int _OPERATION_MODE = 0;
enum _OPERATION_MODE_TYPE
{
    _OPERATION_NO = 200,
    _OPERATION_ACTIVE_CONTROL_INIT,
    _OPERATION_PRESSURE_CHANGE_LINEAR,
};


bool            save_PutDataFlag = false;
bool            save_ActivateFlag = false;
unsigned int    save_Index = 0;
float           save_Buf[SAVEKIND][SAVENUM];
float           temp_data[SAVEKIND];

void            save_PutData(unsigned int cur_Index);
void            save_File(char* filecomment = "");
void            save_File_forID(int _PWM, int direction);

int main(int argc, char *argv[])
{

    // =================== Program initiated ===================
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, PODO_AL_NAME);

    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

//    jCon->RefreshToCurrentReference();
//    jCon->SetAllMotionOwner();

//    // Pump Duty Reference Disable
//    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0] = 0;   // Pump Reference Disable
//    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_PUMP_ACTIVE_VELOCITY_CONTROL_ONOFF;

    FILE_LOG(logSUCCESS) << "Setting Complete!!";

    // =========================================================

    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){  

        case PUMP_CONTROL_SAVE_DATA:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]) {
                save_PutDataFlag = true;
                FILE_LOG(logWARNING) << "SAVING AL DATA START!!";
            } else {
                if(save_PutDataFlag) {
                    if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == NULL) {
                        save_File();
                    } else {
                        save_File(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                    }
                    save_PutDataFlag = false;
                    save_ActivateFlag = false;
                    FILE_LOG(logERROR) << "SAVING AL DATA END!!";
                } else {
                    FILE_LOG(logERROR) << "There is no saved data...";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
            break;
        }

        case PUMP_CONTROL_PRESREF_SELECTION:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == -1) { // Ps_ref : From This AL
                if(PsRef_type == 0) {
                    FILE_LOG(logERROR) << "Pres. Ref. Mode Error! (already this AL)";
                } else if(PsRef_type == 1) {
                    PsRef_type_transition = -1;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. will be obtained from thisAL.";
                }
            } else if (sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){ // Ps_ref : From Shared Memory
                if(PsRef_type == 0) {
                    PsRef_type_transition = 1;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. will be obtained from SharedREF.";
                } else if(PsRef_type == 1) {
                    FILE_LOG(logERROR) << "Pres. Ref. Mode Error! (already SharedREF)";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        case PUMP_ACTIVE_CONTROL_ONOFF:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) { // Active Control On!
                InitSpeed = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                InitTime = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                InputVoltage = sharedSEN->PUMP[0].CurrentSettingVoltage;

                wp_ref_max = PUMPSPEED_MAX/100.0*InputVoltage*0.90; // 0.90 = 90% limit
                wp_ref_min = PUMPSPEED_MIN;

                for(int i=0;i<=MAX_PREVIEW;i++) {
                    Ps_des[i] = Ps_min;
                }
                if(sharedREF->PumpCtrlMode[0] == PumpControlMode_ActiveControl) {
                    FILE_LOG(logERROR) << "Active Duty Control is already Activated!!";
                } else {
                    FILE_LOG(logWARNING) << "Active Duty Control Initialize...";
                    sharedREF->PumpCtrlMode_Command[0] = PumpControlMode_ActiveControl;
                    _OPERATION_MODE = _OPERATION_ACTIVE_CONTROL_INIT;
                }
            } else if (sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0) { // Active Control OFF!
                if(sharedREF->PumpCtrlMode[0] == PumpControlMode_ActiveControl) {
                    FILE_LOG(logWARNING) << "Active Duty Control is terminated...";
                    sharedREF->PumpCtrlMode_Command[0] = PumpControlMode_Interpolation;
                    usleep(100*1000);
                    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0] = 0;
                    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET;
                    Flag_ActiveControl = false;
                    _OPERATION_MODE = _OPERATION_NO;
                } else {
                    FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
                }
                sharedREF->PumpVelocityReference[0] = 0.0;
                FILE_LOG(logERROR) << "Pump Control Is Stopped!!";
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        case PUMP_ACTIVE_CONTROL_MPC_ONOFF:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) { // MPC On!
                if(sharedREF->PumpCtrlMode[0] == PumpControlMode_ActiveControl) {
                    FILE_LOG(logSUCCESS) << "MPC On!";
                    sharedREF->Flag_PumpControlMPC = true;
                } else {
                    FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
                }
            } else if (sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0) { // MPC OFF!
                if(sharedREF->PumpCtrlMode[0] == PumpControlMode_ActiveControl) {
                    FILE_LOG(logWARNING) << "MPC Off!";
                    sharedREF->Flag_PumpControlMPC = false;
                } else {
                    FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        case PUMP_ACTIVE_CONTROL_LINEAR:
        {
            if(Flag_ActiveControl) {
                Ps_fin = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                t_change = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                _OPERATION_MODE = _OPERATION_PRESSURE_CHANGE_LINEAR;
            } else {
                FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        default:
            break;
        }
    }
    sharedCMD->ALTHREAD_ONOFF_SIGNAL[PODO_NO] = false;
    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}


//==============================//
// Task Thread
//==============================//
int CNT_Display = 0;
bool Flag_DispOverRunTime = true;
void *RBTaskThread(void *)
{
    while(__IS_WORKING)
    {

        system_clock::time_point CurrentTime = system_clock::now();

//        static system_clock::time_point LastTime = CurrentTime;
//        microseconds t_interval = duration_cast<std::chrono::microseconds>(CurrentTime - LastTime);
//        LastTime = CurrentTime;
//        FILE_LOG(logDEBUG1) << "Time Interval : "<< t_interval.count() <<" usecond(s).";

#ifndef REALPUMP
        // MPC Performance Checking ======================
        static double t_SIM = 0.0;
        double dT_SIM = SYS_DT_PUMPING;

        int N_window_SIM = 25;
        double dT_window_SIM = 0.060;

        static double Ps_SIM = Ps_min+5.0;
        static VectorNd X_MPC_SIM = VectorNd::Zero(N_window_SIM);
        static VectorNd U_MPC_SIM = VectorNd::Zero(2*N_window_SIM);

        if(t_SIM == 0.0) {
            save_PutDataFlag = true;
            FILE_LOG(logWARNING) << "SAVING AL DATA START!!";
            X_MPC_SIM = Ps_SIM*VectorNd::Ones(N_window_SIM);
            for(int i=0;i<N_window_SIM;i++) {
                U_MPC_SIM(2*i) = wp_ref_min/60.0*2.0*PI;
            }
        } else if(t_SIM >= 10.0 && t_SIM < 10.0+SYS_DT_PUMPING) {
            strcpy(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR, "PumpMPCTest");
            save_ActivateFlag = true;
        }

        FILE_LOG(logINFO) << "[Simulation] Ps (t = " << t_SIM << ") : " << Ps_SIM;

        VectorNd Psdes_Future_SIM = VectorNd::Zero(N_window_SIM+1);
        VectorNd Qdes_Future_SIM = VectorNd::Zero(N_window_SIM+1);

        PumpPressure_VirtualRefMotion(t_SIM,dT_window_SIM,Psdes_Future_SIM,Qdes_Future_SIM);
        PumpPressureController_LinearMPC(N_window_SIM, dT_window_SIM,
                                         Ps_SIM, Psdes_Future_SIM, Qdes_Future_SIM,
                                         X_MPC_SIM,U_MPC_SIM);
        double wp_now = U_MPC_SIM(0); // unit : [rad/s]
        double Ps_next;
        PumpPressure_VirtualDynamics(Ps_SIM, wp_now, Qdes_Future_SIM(0), SYS_DT_PUMPING, Ps_next);
        Ps_SIM = Ps_next;

        t_SIM += dT_SIM;

        temp_data[11] = t_SIM;
        temp_data[12] = Ps_SIM;
        temp_data[13] = Psdes_Future_SIM(0);
        temp_data[14] = wp_now;
        temp_data[15] = Qdes_Future_SIM(0);
#else

        // Read Sensor Data ===============================================================================
        Ps_now = sharedSEN->PUMP[0].CurrentPressure;
//        Ps_now = Ps_min+Ps_margin;
        wp_now = sharedSEN->PUMP[0].CurrentVelocity; // [rpm]

        // Instant Pump Reference Generation  =============================================================
        switch(_OPERATION_MODE)
        {
        case _OPERATION_NO:
            break;
        case _OPERATION_ACTIVE_CONTROL_INIT:
        {
            static double TempSpeed = 0.0;
            double Ps_start = Ps_min+Ps_margin;

            if(Ps_now < Ps_start) {
                wp_ref = TempSpeed;
                if(TempSpeed<InitSpeed){
                    TempSpeed = TempSpeed + (InitSpeed/InitTime)*SYS_DT_PUMPING;
                } else {
                    TempSpeed = InitSpeed;
                }
                Ps_des[0] = Ps_min;
            } else {
                TempSpeed = 0.0;
                Ps_fin = Ps_min;
                t_change = 1.0;
                Flag_ActiveControl = true;
                _OPERATION_MODE = _OPERATION_PRESSURE_CHANGE_LINEAR;
            }
            break;
        }
        case _OPERATION_PRESSURE_CHANGE_LINEAR:
        {
            double dT_window = sharedREF->dT_PrevPump;
            double P_next, dP_next;
            for(int i=0;i<=MAX_PREVIEW;i++) {
                double t_view = SYS_DT_PUMPING + dT_window*(double)i;
                if(t+t_view <=t_change) {
                    linear_trajectory_pressure(t_change-t, t_view, Ps_des[0], Ps_fin, P_next, dP_next);
                    Ps_des[i] = P_next;
                } else {
                    Ps_des[i] = Ps_fin;
                }
            }

            t += SYS_DT_PUMPING;
            if (t >= t_change) {
                FILE_LOG(logSUCCESS) << "Pressure Change is Finished!!";
                t = 0.0;
                _OPERATION_MODE = _OPERATION_NO;
            }
            break;
        }
        default:
            break;
        }

        // Future Pump Reference Generation  =============================================================
        VectorNd Ps_des_window = VectorNd::Zero(sharedREF->N_PrevPump+1);
        VectorNd Qact_des_window = VectorNd::Zero(sharedREF->N_PrevPump+1);
        for(int i=0;i<=sharedREF->N_PrevPump;i++) {
            double Ps_ref = 0.0; // [bar]
            double Ps_AL = Ps_des[i];
            double Ps_SM = sharedREF->RequiredPressureReference_Future[i];

            static double alpha_trans = 0.0;
            if(PsRef_type_transition == 1) {
                double alpha_trans_prev = alpha_trans + (double)i*sharedREF->dT_PrevPump/5.0;
                if(alpha_trans_prev>1.0) alpha_trans_prev = 1.0;
                Ps_ref = alpha_trans_prev*Ps_SM + (1.0-alpha_trans_prev)*Ps_AL;
                alpha_trans = alpha_trans + SYS_DT_PUMPING/5.0/(double)(sharedREF->N_PrevPump+1);
                if(alpha_trans > 1.0) {
                    alpha_trans = 1.0;
                    PsRef_type_transition = 0;
                    PsRef_type = 1;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. Transition Done! : ThisAL >> SharedREF";
                }
            } else if (PsRef_type_transition == -1) {
                double alpha_trans_prev = alpha_trans - (double)i*sharedREF->dT_PrevPump/5.0;
                if(alpha_trans_prev<0.0) alpha_trans_prev = 0.0;
                Ps_ref = alpha_trans_prev*Ps_SM + (1.0-alpha_trans_prev)*Ps_AL;
                alpha_trans = alpha_trans - SYS_DT_PUMPING/5.0/(double)(sharedREF->N_PrevPump+1);
                if(alpha_trans < 0.0) {
                    alpha_trans = 0.0;
                    PsRef_type_transition = 0;
                    PsRef_type = 0;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. Transition Done! : SharedREF >> ThisAL";
                }
            } else if (PsRef_type_transition == 0) {
                if(PsRef_type == 0) {
                    Ps_ref = Ps_AL;
                } else if (PsRef_type == 1) {
                    Ps_ref = Ps_SM;
                }
            }
            Ps_ref = Ps_ref + Ps_margin;

            if(Ps_ref < Ps_min) {
                Ps_ref = Ps_min;
            } else if (Ps_ref > Ps_max) {
                Ps_ref = Ps_max;
            }
            Ps_des_window(i) = Ps_ref;

            Qact_des_window(i) = sharedREF->RequiredFlowrateReference_Future[i];
        }

        sharedREF->PumpPressureReference[0] = Ps_des_window(0);

        // Pump Speed Controller Selection ==============================================================

        if(Flag_ActiveControl) {
            if(sharedREF->Flag_PumpControlMPC) {
                int N_window = sharedREF->N_PrevPump;
                double dT_window = sharedREF->dT_PrevPump;

                static VectorNd X_MPC = Ps_min*VectorNd::Ones(N_window);
                static VectorNd U_MPC = VectorNd::Zero(2*N_window);
                PumpPressureController_LinearMPC(N_window, dT_window,
                                                 Ps_now, Ps_des_window, Qact_des_window,
                                                 X_MPC,U_MPC);
                wp_ref = U_MPC(0)*60.0/(2.0*PI); // rad/s >> rpm
            } else {
                double u_sol = 0.0;
                double Ps_des_in = Ps_des_window(0);
                double dPs_des_in = (Ps_des_window(1)-Ps_des_window(0))/sharedREF->dT_PrevPump;
                PumpFlowControl_Active_LIGHT2(Ps_now,Ps_des_in, dPs_des_in, u_sol);
                wp_ref = u_sol;
            }
        }

        double fcut_wp = 20.0;
        double alpha_wp = 1.0/(1.0+2.0*PI*fcut_wp*SYS_DT_PUMPING);
        static double wp_ref_fil = 0.0;
        wp_ref_fil = alpha_wp*wp_ref_fil + (1.0-alpha_wp)*wp_ref;

        double wp_ref_sat;
        if(wp_ref_fil < wp_ref_min) {
            wp_ref_sat = wp_ref_min;
        } else if (wp_ref_fil > wp_ref_max) {
            wp_ref_sat = wp_ref_max;
        } else {
            wp_ref_sat = wp_ref_fil;
        }
        sharedREF->PumpVelocityReference[0] = wp_ref_sat;

#endif

        // Debugging ===================================================================================

        if (CNT_Display>=100) {
//            cout << "Ps_des : "<< sharedREF->PumpPressureReference[0] << endl;
//            FILE_LOG(logDEBUG) << "Qflow_ref_JointVel : " << Qflow_ref_JointVel;
//            FILE_LOG(logDEBUG) << "Qflow_ref_Leakage : " << Qflow_ref_Leakage;
//            FILE_LOG(logDEBUG) << "Qflow_ref_Accumulator : " << Qflow_ref_Accumulator;
//            FILE_LOG(logDEBUG) << "Qflow_ref_Feedback : " << Qflow_ref_Feedback;
            CNT_Display = 0;
        } CNT_Display++;

        if(save_PutDataFlag == true){
            if(save_ActivateFlag == true || save_Index == (SAVENUM-1)) {
                if(save_Index == (SAVENUM-1)) {
                    FILE_LOG(logERROR) << "Save Buffer is Full!!";
                    sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] = NULL;
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_SAVE_DATA;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = false;
            } else {
                save_PutData(save_Index);
                save_Index++;
            }
        } else {
            if(save_ActivateFlag == true) {
                save_ActivateFlag = false;
                FILE_LOG(logERROR) << "There is no saved data...";
            }
        }

        //////======================================================================================

        system_clock::time_point CurrentTime2 = system_clock::now();
        microseconds t_TotalCycle = duration_cast<std::chrono::microseconds>(CurrentTime2 - CurrentTime);
//        FILE_LOG(logDEBUG1) << "Time Interval : "<< t_TotalCycle.count() <<" usecond(s).";

        temp_data[16] = t_TotalCycle.count(); // unit : [us]

        static int OverTimeCnt_6000ms = 0;
        if(t_TotalCycle.count() > 6000) {
            OverTimeCnt_6000ms++;
            if(Flag_DispOverRunTime)
            {
                FILE_LOG(logDEBUG1) << "OverTimeCnt (6000ms) : "<< OverTimeCnt_6000ms;
            }
        }

        //////======================================================================================
//        rt_task_suspend(&rtTaskCon);

        sharedCMD->ALTHREAD_ONOFF_SIGNAL[PODO_NO] = true;
        while(1) {
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) {
//                jCon->JointUpdate();
                sharedCMD->SYNC_SIGNAL[PODO_NO] = false;
                break;
            } else {
                usleep(2);
            }
        }
    }
    sharedCMD->ALTHREAD_ONOFF_SIGNAL[PODO_NO] = false;
}
//==============================//


//==============================//
// Flag Thread
//==============================//
int CntFlagThread = 0;
void *RBFlagThread(void *)
{
//    rt_task_set_periodic(NULL, TM_NOW, 40*1000);

//    while(__IS_WORKING)
//    {
//        rt_task_wait_period(NULL);

////        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) { // Daemon : 250Hz, PumpControl AL : 125Hz
////            jCon->JointUpdate();
////            if(CntFlagThread%2 == 1) {
////                rt_task_resume(&rtTaskCon);
////                CntFlagThread = 1;
////            } else {
//////                rt_task_resume(&rtTaskCon);
////            }
////            CntFlagThread++;
////        }

//        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
//            jCon->JointUpdate();
//            rt_task_resume(&rtTaskCon);
//        }
//    }

}

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = SYS_DT_PUMPING;

    save_Buf[1][cur_Index] = Ps_now; // Ps_now [bar]
    save_Buf[2][cur_Index] = wp_now; // W_now [rpm]
    save_Buf[3][cur_Index] = sharedSEN->PUMP[0].CurrentTemperature; // Wire Temperature [deg]

    save_Buf[4][cur_Index] = Qflow_ref;
    save_Buf[5][cur_Index] = Qflow_ref_JointVel;
    save_Buf[6][cur_Index] = Qflow_ref_Leakage;
    save_Buf[7][cur_Index] = Qflow_ref_Accumulator;
    save_Buf[8][cur_Index] = Qflow_ref_Feedback;

    save_Buf[9][cur_Index] = sharedREF->PumpPressureReference[0]; // [bar]
    save_Buf[10][cur_Index] = sharedREF->PumpVelocityReference[0]; // [rpm]

    save_Buf[11][cur_Index] = temp_data[11];
    save_Buf[12][cur_Index] = temp_data[12];
    save_Buf[13][cur_Index] = temp_data[13];
    save_Buf[14][cur_Index] = temp_data[14];
    save_Buf[15][cur_Index] = temp_data[15];
    save_Buf[16][cur_Index] = temp_data[16];
}

void save_File(char *filecomment)
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "DataLog/PumpControl_AL_Data/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();
    const char* filetype = ".txt";
    char filename[100];

    strcpy(filename, filedate);
    if(!filecomment[0] == NULL) {
        strcat(filename, "_");
        strcat(filename, filecomment);
    }
    strcat(filename, filetype);

    FILE* fp;
    unsigned int i, j;
    fp = fopen(filename, "w");

    for(i=0 ; i<save_Index ; i++)
    {
        for(j=0 ; j<SAVEKIND ; j++){
            fprintf(fp, "%f\t", save_Buf[j][i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);

    save_Index=0;
    std::cout << "Saved Filename : " << filename << std::endl;
}


//////=====================================================================================

