#include "SH_TASK_BasicFunction.h"
#include "LIGHT_jointsetmodel.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

extern JointControlClass       *jCon;


void LIGHTJointSet::ValveIdentification_ParameterSet(string _DATE, int _DIR,
                                                     int _OPEN_MIN, int _OPEN_RESOL, int _OPEN_MAX,
                                                     double _Ps_MIN, double _Ps_RESOL, double _Ps_MAX)
{
    ValveId_Date = _DATE;
    ValveId_Direction = _DIR;
    ValveId_OpenMin = _OPEN_MIN;
    ValveId_OpenResol = _OPEN_RESOL;
    ValveId_OpenMax = _OPEN_MAX;
    ValveId_PressureMin = _Ps_MIN;
    ValveId_PressureResol = _Ps_RESOL;
    ValveId_PressureMax = _Ps_MAX;
}


enum PRES_CHANGE_STAGESET{
    PRES_CHANGE_PARAMETER_SETTING = 0,
    PRES_CHANGE_READY,
    PRES_CHANGE_VALVEID,
    PRES_CHANGE_TERMINATE,
};
int PRES_CHANGE_CurrentStage = PRES_CHANGE_PARAMETER_SETTING;

bool LIGHTJointSet::ValveIdentification_VariablePressure(bool _ONOFF)
{
    static int DIR;
    static double Ps_min;
    static double Ps_resol;
    static double Ps_max;
    static double Ps_term;
    static double Ps_ini;
    static double Ps_des;

    static double n = 0.0;

    static double t = 0.0;
    static double t_set = 0.02;
    static double t_ready = 5.0;
    static double t_term = 5.0;

    static bool FlagForcedStop = false;

    switch(PRES_CHANGE_CurrentStage) {
    case PRES_CHANGE_PARAMETER_SETTING:
    {
        if(t == 0.0) {
            DIR = ValveId_Direction;
            Ps_min = ValveId_PressureMin;
            Ps_resol = ValveId_PressureResol;
            Ps_max = ValveId_PressureMax;
            Ps_term = Ps_min;
        }

        t = t + SYS_DT_ACTTEST;
        if(t>=t_set) {
            t = 0.0;
            PRES_CHANGE_CurrentStage = PRES_CHANGE_READY;
        }
        break;
    }
    case PRES_CHANGE_READY:
    {
        if(t == 0.0) {
            Ps_ini = sharedREF->PumpPressureReference[0];
            Ps_des = Ps_min + n*Ps_resol;
            FILE_LOG(logSUCCESS) << "Pressure Change : " << Ps_ini << "bar >> " << Ps_des << "bar";
        }

        double Ps = Ps_ini + (Ps_des-Ps_ini)*t/t_ready;

        sharedREF->LoadPressureReference_Future[0][0] = Ps;
        sharedREF->LoadPressureReference_Future[1][0] = Ps;

        t = t + SYS_DT_ACTTEST;
        if(t>=t_ready) {
            t = 0.0;
            PRES_CHANGE_CurrentStage = PRES_CHANGE_VALVEID;
        }
        break;
    }
    case PRES_CHANGE_VALVEID:
    {
        bool FINISH;
        if(DIR > 0) // Positive Direction
            FINISH = ValveIdentification_Positive((int)Ps_des,_ONOFF);
        else if (DIR < 0) // Negative Direction
            FINISH = ValveIdentification_Negative((int)Ps_des,_ONOFF);
        else
            FILE_LOG(logERROR) << "Direction Setting Error!";

        if(FINISH) {
            if(Ps_des < Ps_max) {
                n = n + 1.0;
                PRES_CHANGE_CurrentStage = PRES_CHANGE_READY;
            } else {
                PRES_CHANGE_CurrentStage = PRES_CHANGE_TERMINATE;
            }
        }
        break;
    }
    case PRES_CHANGE_TERMINATE:
    {
        if(t == 0.0) {
            Ps_ini = sharedREF->PumpPressureReference[0];
            Ps_des = Ps_min;
            FILE_LOG(logERROR) << "Valve ID, Terminate...";
        }

        double Ps = Ps_ini + (Ps_des-Ps_ini)*t/t_ready;

        sharedREF->LoadPressureReference_Future[0][0] = Ps;
        sharedREF->LoadPressureReference_Future[1][0] = Ps;

        t = t + SYS_DT_ACTTEST;
        if(t>=t_term) {
            t = 0.0;
            PRES_CHANGE_CurrentStage = PRES_CHANGE_PARAMETER_SETTING;
            FILE_LOG(logSUCCESS) << "Valve ID, Done!!";
            return true;
        }
        break;
    }
    default:
        break;
    }

    if(!_ONOFF && !FlagForcedStop) { // all stop
        FILE_LOG(logERROR) << "Pressure Change, Forced Stopped!";
        FlagForcedStop = true;
        PRES_CHANGE_CurrentStage = PRES_CHANGE_TERMINATE;
        t = 0.0;
    }
    return false;
}


enum VALVE_ID_STAGESET{
    VALVE_ID_PARAMETER_SETTING = 0,
    VALVE_ID_READY,
    VALVE_ID_OPEN,
    VALVE_ID_TERMINATE,
};
int VALVE_ID_CurrentStage = VALVE_ID_PARAMETER_SETTING;

bool LIGHTJointSet::ValveIdentification_Positive(int Ps, bool _ONOFF)
{
    static string Date;
    static int Xv_min;
    static int Xv_resol;
    static int Xv_max;
    static int Xv;

    static double t = 0.0;
    static double t_set = 0.02;
    static double t_ready = 1.0;
    static double t_openwait = 3.0;
    static double t_term = 1.0;
    static int n = 0;

    double q_ready = 5.0*D2R;
    double q_term = 90.0*D2R;

    static double q, dq, ddq;

    static bool FlagForcedStop = false;

    switch(VALVE_ID_CurrentStage) {
    case VALVE_ID_PARAMETER_SETTING: {
        if(t == 0.0) {
            FILE_LOG(logWARNING) << "Valve ID, Parameters Setting...";
            Xv_min = ValveId_OpenMin;
            Xv_resol = ValveId_OpenResol;
            Xv_max = ValveId_OpenMax;
            Date = ValveId_Date;
        }

        t = t + SYS_DT_ACTTEST;
        if(t>=t_set) {
            t = 0.0;
            VALVE_ID_CurrentStage = VALVE_ID_READY;
            FILE_LOG(logWARNING) << "Valve ID, Ready...";
        }
        break;
    }
    case VALVE_ID_READY: {
        if(t == 0.0) {
            q = Qnow(0);
            dq = 0.0;
            ddq = 0.0;

            // Reference Mode Change : Position Control
            jCon->SetJointRefAngle(0,q*R2D);
            jCon->SetJointRefAngVel(0, 0.0);
            jCon->SetJointRefTorque(0, 0.0);
            sharedREF->ValveCtrlMode_Command[0] = ValveControlMode_PosOrFor;
        }

        double q_next, dq_next, ddq_next;
        fifth_trajectory_oneaxis(t_ready-t, SYS_DT_ACTTEST,
                                 q, dq, ddq,
                                 q_ready, 0.0, 0.0,
                                 q_next, dq_next, ddq_next);
        q = q_next;
        dq = dq_next;
        ddq = ddq_next;
        Qref(0) = q;
        dQref(0) = dq;
        ddQref(0) = ddq;

        t = t + SYS_DT_ACTTEST;
        if(t>=t_ready) {
            t = 0.0;
            VALVE_ID_CurrentStage = VALVE_ID_OPEN;
        }
        break;
    }
    case VALVE_ID_OPEN: {
        if(t == 0.0) {
            save_Flag_forID = true;
            Xv = Xv_min + n*Xv_resol;
            FILE_LOG(logWARNING) << "Valve ID, Opening : " << Xv;

            // Reference Mode Change : Valve Opening
            jCon->SetRefValvePos(0, Xv); // -1000~1000
            sharedREF->ValveCtrlMode_Command[0] = ValveControlMode_Opening;
        }

        jCon->SetRefValvePos(0, Xv);

        t = t + SYS_DT_ACTTEST;
        if((Qnow(0) > 85.0*D2R)||(Qnow(0) < 0.0*D2R)||(t>=t_openwait)) {

            save_Flag_forID = false;
            save_File_forID(Date,Ps,Xv,1);

            if(Xv<Xv_max) {
                n++;
                VALVE_ID_CurrentStage = VALVE_ID_READY;
            } else {
                VALVE_ID_CurrentStage = VALVE_ID_TERMINATE;
            }
            t = 0.0;
        }
        break;
    }
    case VALVE_ID_TERMINATE: {
        if(t == 0.0) {
            q = Qnow(0);
            dq = 0.0;
            ddq = 0.0;

            // Reference Mode Change : Position Control
            jCon->SetJointRefAngle(0,q*R2D);
            jCon->SetJointRefAngVel(0, 0.0);
            jCon->SetJointRefTorque(0, 0.0);
            sharedREF->ValveCtrlMode_Command[0] = ValveControlMode_PosOrFor;
        }

        double q_next, dq_next, ddq_next;
        fifth_trajectory_oneaxis(t_set-t, SYS_DT_ACTTEST,
                                 q, dq, ddq,
                                 q_term, 0.0, 0.0,
                                 q_next, dq_next, ddq_next);
        q = q_next;
        dq = dq_next;
        ddq = ddq_next;
        Qref(0) = q;
        dQref(0) = dq;
        ddQref(0) = ddq;

        t = t + SYS_DT_ACTTEST;
        if(t>=t_term) {
            VALVE_ID_CurrentStage = VALVE_ID_PARAMETER_SETTING;
            t = 0.0;
            n = 0;
            FlagForcedStop = false;
            return true;
        }
        break;
    }
    default:
        break;
    }

    if(!_ONOFF && !FlagForcedStop) { // all stop
        FILE_LOG(logERROR) << "Valve ID, Forced Stopped!";
        FlagForcedStop = true;
        VALVE_ID_CurrentStage = VALVE_ID_TERMINATE;
        t = 0.0;
    }
    return false;
}

bool LIGHTJointSet::ValveIdentification_Negative(int Ps, bool _ONOFF)
{
    static string Date;
    static int Xv_min;
    static int Xv_resol;
    static int Xv_max;
    static int Xv;

    static double t = 0.0;
    static double t_set = 0.02;
    static double t_ready = 1.0;
    static double t_openwait = 3.0;
    static double t_term = 1.0;
    static int n = 0;

    double q_ready = 83.0*D2R;
    double q_term = 0.0*D2R;

    static double q, dq, ddq;

    static bool FlagForcedStop = false;

    switch(VALVE_ID_CurrentStage) {
    case VALVE_ID_PARAMETER_SETTING: {
        if(t == 0.0) {
            FILE_LOG(logWARNING) << "Valve ID, Parameters Setting...";
            Xv_min = ValveId_OpenMin;
            Xv_resol = ValveId_OpenResol;
            Xv_max = ValveId_OpenMax;
            Date = ValveId_Date;
        }

        t = t + SYS_DT_ACTTEST;
        if(t>=t_set) {
            t = 0.0;
            VALVE_ID_CurrentStage = VALVE_ID_READY;
            FILE_LOG(logWARNING) << "Valve ID, Ready...";
        }
        break;
    }
    case VALVE_ID_READY: {
        if(t == 0.0) {
            q = Qnow(0);
            dq = 0.0;
            ddq = 0.0;

            // Reference Mode Change : Position Control
            jCon->SetJointRefAngle(0,q*R2D);
            jCon->SetJointRefAngVel(0, 0.0);
            jCon->SetJointRefTorque(0, 0.0);
            sharedREF->ValveCtrlMode_Command[0] = ValveControlMode_PosOrFor;
        }

        double q_next, dq_next, ddq_next;
        fifth_trajectory_oneaxis(t_ready-t, SYS_DT_ACTTEST,
                                 q, dq, ddq,
                                 q_ready, 0.0, 0.0,
                                 q_next, dq_next, ddq_next);
        q = q_next;
        dq = dq_next;
        ddq = ddq_next;
        Qref(0) = q;
        dQref(0) = dq;
        ddQref(0) = ddq;

        t = t + SYS_DT_ACTTEST;
        if(t>=t_ready) {
            t = 0.0;
            VALVE_ID_CurrentStage = VALVE_ID_OPEN;
        }
        break;
    }
    case VALVE_ID_OPEN: {
        if(t == 0.0) {
            save_Flag_forID = true;
            Xv = Xv_min + n*Xv_resol;
            FILE_LOG(logWARNING) << "Valve ID, Opening : " << -Xv;

            // Reference Mode Change : Valve Opening
            jCon->SetRefValvePos(0, -Xv); // -1000~1000
            sharedREF->ValveCtrlMode_Command[0] = ValveControlMode_Opening;
        }

        jCon->SetRefValvePos(0, -Xv);

        t = t + SYS_DT_ACTTEST;
        if((Qnow(0) > 85.0*D2R)||(Qnow(0) < 3.0*D2R)||(t>=t_openwait)) {

            save_Flag_forID = false;
            save_File_forID(Date,Ps,-Xv,-1);

            if(Xv<Xv_max) {
                n++;
                VALVE_ID_CurrentStage = VALVE_ID_READY;
            } else {
                VALVE_ID_CurrentStage = VALVE_ID_TERMINATE;
            }
            t = 0.0;
        }
        break;
    }
    case VALVE_ID_TERMINATE: {
        if(t == 0.0) {
            q = Qnow(0);
            dq = 0.0;
            ddq = 0.0;

            // Reference Mode Change : Position Control
            jCon->SetJointRefAngle(0,q*R2D);
            jCon->SetJointRefAngVel(0, 0.0);
            jCon->SetJointRefTorque(0, 0.0);
            sharedREF->ValveCtrlMode_Command[0] = ValveControlMode_PosOrFor;
        }

        double q_next, dq_next, ddq_next;
        fifth_trajectory_oneaxis(t_set-t, SYS_DT_ACTTEST,
                                 q, dq, ddq,
                                 q_term, 0.0, 0.0,
                                 q_next, dq_next, ddq_next);
        q = q_next;
        dq = dq_next;
        ddq = ddq_next;
        Qref(0) = q;
        dQref(0) = dq;
        ddQref(0) = ddq;

        t = t + SYS_DT_ACTTEST;
        if(t>=t_term) {
            VALVE_ID_CurrentStage = VALVE_ID_PARAMETER_SETTING;
            t = 0.0;
            n = 0;
            FlagForcedStop = false;
            return true;
        }
        break;
    }
    default:
        break;
    }

    if(!_ONOFF && !FlagForcedStop) { // all stop
        FILE_LOG(logERROR) << "Valve ID, Forced Stopped!";
        FlagForcedStop = true;
        VALVE_ID_CurrentStage = VALVE_ID_TERMINATE;
        t = 0.0;
    }
    return false;
}

// ========================================================================================


void save_PutData_forID(unsigned int cur_Index, int BNO)
{
    // System Period
    save_Buf_forID[0][cur_Index] = SYS_DT_ACTTEST;

    save_Buf_forID[1][cur_Index] = sharedSEN->ENCODER[BNO][0].CurrentAngle;
    save_Buf_forID[2][cur_Index] = sharedSEN->ENCODER[BNO][0].CurrentAngVel;
    save_Buf_forID[3][cur_Index] = sharedSEN->ENCODER[BNO][0].CurrentValvePos;
    save_Buf_forID[4][cur_Index] = sharedSEN->ENCODER[BNO][0].CurrentRefValvePos;

    save_Buf_forID[5][cur_Index] = sharedSEN->PUMP[0].CurrentPressure;
}

void save_File_forID(string Date, int _Ps, int _OPEN, int direction)
{
    char file_name[60] = "ValveID_Data/";
    char file_name_pos[10] = "Positive/";
    char file_name_neg[10] = "Negative/";
    char file_name_plus[10] = "OPEN_p";
    char file_name_minus[10] = "OPEN_m";
    char file_name_extension[5] = ".txt";

    FILE* fp;
    unsigned int i, j;

    const char* file_date = Date.c_str();
    strcat(file_name,file_date);
    if(mkdir(file_name, 0776) == -1 && errno != EEXIST) {
        FILE_LOG(logERROR) << "Directory (Date) Error!";
    } else {
        if (direction > 0) {
            strcat(file_name,file_name_pos);
        } else if (direction < 0) {
            strcat(file_name,file_name_neg);
        }
        if(mkdir(file_name, 0776) == -1 && errno != EEXIST) {
            FILE_LOG(logERROR) << "Directory (Opening Direction) Error!";
        } else {
            char file_name_pres[12];
            sprintf(file_name_pres,"Ps_%dbar/",_Ps);
            strcat(file_name,file_name_pres);
            if(mkdir(file_name, 0776) == -1 && errno != EEXIST) {
                FILE_LOG(logERROR) << "Directory (Pressure) Error!";
            } else {
                if(_OPEN>=0) {
                    strcat(file_name,file_name_plus);
                } else {
                    strcat(file_name,file_name_minus);
                }
                char file_name_open[10];
                sprintf(file_name_open,"%d",abs(_OPEN));
                strcat(file_name,file_name_open);
                strcat(file_name,file_name_extension);

                fp = fopen(file_name, "w");
                for(i=0 ; i<save_Index_forID ; i++)
                {
                    for(j=0 ; j<SAVEKIND_FORID ; j++){
                        fprintf(fp, "%f\t", save_Buf_forID[j][i]);
                    }
                    fprintf(fp, "\n");
                }
                fclose(fp);
                save_Index_forID = 0;
                save_Flag_forID = 0;
            }
        }
    }
}

