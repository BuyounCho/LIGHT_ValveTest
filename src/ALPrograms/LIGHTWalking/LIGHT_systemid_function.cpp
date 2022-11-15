#include "LIGHT_motion.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;

extern LIGHT_InvKinematics_INFO INFO_InvKin;

extern LIGHTWholeBody LIGHT;

//extern bool SaveFlagOn_SYS_ID(int _DataType);
//extern bool SaveFlagOFFandSaveData_SYS_ID();
extern double _OPERATION_FREQ_NOW;
enum SYSID_STAGE {
    SYSID_INIT = 0,
    SYSID_PROCESSING,
    SYSID_TERMINATE
};
char SYSID_CurrentStage = SYSID_INIT;
Vector3d SYSID_Xini_CoM;
double f_init = 0.1;

extern bool _OPERATION_MODE_CHANGED;

bool LIGHTWholeMotions::SYSID_CoMRef2CoM(int SYSID_TYPE, double CoM_Mag, int f_max, double r_freq) {

    static double t_temp;
    static double f_now;
    static double t_total;
    static int n_now;

    // XorY = 0 : X-direction
    // XorY = 1 : Y-direction
    // DorS = 0 : DSP
    // DorS = 1 : SSP
    // RorL = 0 : Reference Frame is RightFoot
    // RorL = 1 : Reference Frame is LeftFoot
    bool XorY, DorS, RorL;
    if(SYSID_TYPE == 0) {
        XorY = false;   DorS = false;   RorL = false;
    } else if(SYSID_TYPE == 1) {
        XorY = true;   DorS = false;   RorL = false;
    } else if(SYSID_TYPE == 2) {
        XorY = false;   DorS = false;   RorL = true;
    } else if(SYSID_TYPE == 3) {
        XorY = true;   DorS = false;   RorL = true;
    } else if(SYSID_TYPE == 4) {
        XorY = false;   DorS = true;   RorL = false;
    } else if(SYSID_TYPE == 5) {
        XorY = true;   DorS = true;   RorL = false;
    } else if(SYSID_TYPE == 6) {
        XorY = false;   DorS = true;   RorL = true;
    } else if(SYSID_TYPE == 7) {
        XorY = true;   DorS = true;   RorL = true;
    }

    switch(SYSID_CurrentStage) {
    case SYSID_INIT:
    {
        if (TimeIsZero()) {
            if(RorL == false) {
                LIGHT.ChangeRefFrame(LIGHT.REFFRAME_RF);

                if(DorS == false) {
                    if(LIGHT.IsCurRefState_DSP())
                    {
                        LIGHT.CurRefStateIs_RDSP();
//                        SetSupportControlGain_DSP();
                    } else {
                        FILE_LOG(logERROR) << "Current State is not DSP.";
                        FILE_LOG(logERROR) << "System Identification is stopped.";
                        return true;
                    }
                } else {
                    if(LIGHT.IsCurRefState_RSSP())
                    {
//                        SetSupportControlGain_RSSP();
                    } else {
                        FILE_LOG(logERROR) << "Current State is not RSSP.";
                        FILE_LOG(logERROR) << "System Identification is stopped.";
                        return true;
                    }
                }
                // Set Cost Weight for Kinematics Problem
                INFO_InvKin.WeightSet_RFBaseMotion();

                SYSID_Xini_CoM = LIGHT.Xref_RF2CoM;
            } else {
                if(DorS == false) {
                    if(LIGHT.IsCurRefState_DSP())
                    {
                        LIGHT.CurRefStateIs_LDSP();
//                        SetSupportControlGain_DSP();
                    } else {
                        FILE_LOG(logERROR) << "Current State is not DSP.";
                        FILE_LOG(logERROR) << "System Identification is stopped.";
                        return true;
                    }
                } else {
                    if(LIGHT.IsCurRefState_LSSP())
                    {
//                        SetSupportControlGain_LSSP();
                    } else {
                        FILE_LOG(logERROR) << "Current State is not LSSP.";
                        FILE_LOG(logERROR) << "System Identification is stopped.";
                        return true;
                    }
                }
                // Set Cost Weight for Kinematics Problem
                INFO_InvKin.WeightSet_LFBaseMotion();

                SYSID_Xini_CoM = LIGHT.Xref_LF2CoM;
            }

            FILE_LOG(logSUCCESS) << " =================================";
            FILE_LOG(logSUCCESS) << " CoM Magnitude : " << CoM_Mag;
            FILE_LOG(logSUCCESS) << " Maximum Freq. : " << f_max;
            FILE_LOG(logSUCCESS) << " Delta Freq. : " << r_freq;
            FILE_LOG(logSUCCESS) << " =================================";

            f_now = f_init; // Initialize Frequency
            TimeLimitSet(3.0);

            FILE_LOG(logWARNING) << " CoM Model Identification Initialize... ";
        }

        // =============== Pelvis >> RF Orientation Trajectory ===============
        Submotion_Pelvis2RF_Ori(TimeNow(),t_total,I3);
        // =============== Pelvis >> LF Orientation Trajectory ===============
        Submotion_Pelvis2LF_Ori(TimeNow(),t_total,I3);

        if(RorL == false) {
            // =============== RF >> LF Position Trajectory ===============
            Submotion_RF2LF_Pos(TimeLeft(), LIGHT.Xref_RF2LF, zv, zv);
            // =============== RF >> CoM Position Trajectory ===============
            Vector3d _Xdes_RF2CoM, _dXdes_RF2CoM, _ddXdes_RF2CoM;
            if(!XorY) { // X-direction
                _Xdes_RF2CoM(0) = SYSID_Xini_CoM(0);
                _dXdes_RF2CoM(0) = CoM_Mag*(2.0*PI*f_now);
                _ddXdes_RF2CoM(0) = 0.0;
                _Xdes_RF2CoM(1) = SYSID_Xini_CoM(1);
                _dXdes_RF2CoM(1) = 0.0;
                _ddXdes_RF2CoM(1) = 0.0;
                _Xdes_RF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_RF2CoM(2) = 0.0;
                _ddXdes_RF2CoM(2) = 0.0;
            } else { // Y-direction
                _Xdes_RF2CoM(0) = SYSID_Xini_CoM(0);
                _dXdes_RF2CoM(0) = 0.0;
                _ddXdes_RF2CoM(0) = 0.0;
                _Xdes_RF2CoM(1) = SYSID_Xini_CoM(1);
                _dXdes_RF2CoM(1) = CoM_Mag*(2.0*PI*f_now);
                _ddXdes_RF2CoM(1) = 0.0;
                _Xdes_RF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_RF2CoM(2) = 0.0;
                _ddXdes_RF2CoM(2) = 0.0;
            }
            Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, _dXdes_RF2CoM, _ddXdes_RF2CoM);
        } else {
            // =============== LF >> RF Position Trajectory ===============
            Submotion_LF2RF_Pos(TimeLeft(), LIGHT.Xref_LF2RF, zv, zv);
            // =============== LF >> CoM Position Trajectory ===============
            Vector3d _Xdes_LF2CoM, _dXdes_LF2CoM, _ddXdes_LF2CoM;
            if(!XorY) { // X-direction
                _Xdes_LF2CoM(0) = SYSID_Xini_CoM(0);
                _dXdes_LF2CoM(0) = CoM_Mag*(2.0*PI*f_now);
                _ddXdes_LF2CoM(0) = 0.0;
                _Xdes_LF2CoM(1) = SYSID_Xini_CoM(1);
                _dXdes_LF2CoM(1) = 0.0;
                _ddXdes_LF2CoM(1) = 0.0;
                _Xdes_LF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_LF2CoM(2) = 0.0;
                _ddXdes_LF2CoM(2) = 0.0;
            } else { // Y-direction
                _Xdes_LF2CoM(0) = SYSID_Xini_CoM(0);
                _dXdes_LF2CoM(0) = 0.0;
                _ddXdes_LF2CoM(0) = 0.0;
                _Xdes_LF2CoM(1) = SYSID_Xini_CoM(1);
                _dXdes_LF2CoM(1) = CoM_Mag*(2.0*PI*f_now);
                _ddXdes_LF2CoM(1) = 0.0;
                _Xdes_LF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_LF2CoM(2) = 0.0;
                _ddXdes_LF2CoM(2) = 0.0;
            }
            Submotion_LF2CoM_Pos(TimeLeft(), _Xdes_LF2CoM, _dXdes_LF2CoM, _ddXdes_LF2CoM);
        }

        TimeUpdate();
        if (TimeCheck()) {
            TimeReset();
            SYSID_CurrentStage = SYSID_PROCESSING;
        }
        return false;
    }
    case SYSID_PROCESSING:
    {
        if (TimeIsZero()) {
//            SaveFlagOn_SYS_ID(SYSID_TYPE);
            t_temp = 0.0;
            t_total = 0.0;
            f_now = f_init;
            n_now = 1;
            double f_temp = f_now;
            while(f_temp <= f_max)
            {
                t_total += (double)1.0/f_temp;
                f_temp = f_temp*r_freq;
            }

//            TimeLimitSet(t_total);
            FILE_LOG(logSUCCESS) << " CoMREF to ZMP ID (INC)";
            FILE_LOG(logSUCCESS) << " Total Time : " << t_total;
            FILE_LOG(logSUCCESS) << " Current Frequency : " << f_now;
        }

        _OPERATION_FREQ_NOW = f_now;
        if(RorL == false) {
            Vector3d _Xdes_RF2CoM, _dXdes_RF2CoM, _ddXdes_RF2CoM;
            if(!XorY) { // X-direction
                _Xdes_RF2CoM(0) = SYSID_Xini_CoM(0)+CoM_Mag*sin(2.0*PI*f_now*t_temp);
                _dXdes_RF2CoM(0) = CoM_Mag*cos(2.0*PI*f_now*t_temp)*(2.0*PI*f_now);
                _ddXdes_RF2CoM(0) = -CoM_Mag*sin(2.0*PI*f_now*t_temp)*(2.0*PI*f_now)*(2.0*PI*f_now);
                _Xdes_RF2CoM(1) = SYSID_Xini_CoM(1);
                _dXdes_RF2CoM(1) = 0.0;
                _ddXdes_RF2CoM(1) = 0.0;
                _Xdes_RF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_RF2CoM(2) = 0.0;
                _ddXdes_RF2CoM(2) = 0.0;
            } else { // Y-direction
                _Xdes_RF2CoM(0) = SYSID_Xini_CoM(0);
                _dXdes_RF2CoM(0) = 0.0;
                _ddXdes_RF2CoM(0) = 0.0;
                _Xdes_RF2CoM(1) = SYSID_Xini_CoM(1)+CoM_Mag*sin(2.0*PI*f_now*t_temp);
                _dXdes_RF2CoM(1) = CoM_Mag*cos(2.0*PI*f_now*t_temp)*(2.0*PI*f_now);
                _ddXdes_RF2CoM(1) = -CoM_Mag*sin(2.0*PI*f_now*t_temp)*(2.0*PI*f_now)*(2.0*PI*f_now);
                _Xdes_RF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_RF2CoM(2) = 0.0;
                _ddXdes_RF2CoM(2) = 0.0;
            }
            Submotion_RF2CoM_Pos(SYS_DT_WALKING, _Xdes_RF2CoM, _dXdes_RF2CoM, _ddXdes_RF2CoM);
        } else {
            Vector3d _Xdes_LF2CoM, _dXdes_LF2CoM, _ddXdes_LF2CoM;
            if(!XorY) { // X-direction
                _Xdes_LF2CoM(0) = SYSID_Xini_CoM(0)+CoM_Mag*sin(2.0*PI*f_now*t_temp);
                _dXdes_LF2CoM(0) = CoM_Mag*cos(2.0*PI*f_now*t_temp)*(2.0*PI*f_now);
                _ddXdes_LF2CoM(0) = -CoM_Mag*sin(2.0*PI*f_now*t_temp)*(2.0*PI*f_now)*(2.0*PI*f_now);
                _Xdes_LF2CoM(1) = SYSID_Xini_CoM(1);
                _dXdes_LF2CoM(1) = 0.0;
                _ddXdes_LF2CoM(1) = 0.0;
                _Xdes_LF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_LF2CoM(2) = 0.0;
                _ddXdes_LF2CoM(2) = 0.0;
            } else { // Y-direction
                _Xdes_LF2CoM(0) = SYSID_Xini_CoM(0);
                _dXdes_LF2CoM(0) = 0.0;
                _ddXdes_LF2CoM(0) = 0.0;
                _Xdes_LF2CoM(1) = SYSID_Xini_CoM(1)+CoM_Mag*sin(2.0*PI*f_now*t_temp);
                _dXdes_LF2CoM(1) = CoM_Mag*cos(2.0*PI*f_now*t_temp)*(2.0*PI*f_now);
                _ddXdes_LF2CoM(1) = -CoM_Mag*sin(2.0*PI*f_now*t_temp)*(2.0*PI*f_now)*(2.0*PI*f_now);
                _Xdes_LF2CoM(2) = SYSID_Xini_CoM(2);
                _dXdes_LF2CoM(2) = 0.0;
                _ddXdes_LF2CoM(2) = 0.0;
            }
            Submotion_LF2CoM_Pos(SYS_DT_WALKING, _Xdes_LF2CoM, _dXdes_LF2CoM, _ddXdes_LF2CoM);
        }

        t_temp += SYS_DT_WALKING;
        TimeUpdate();
        if(f_now > f_max) {
            TimeReset();
//            SaveFlagOFFandSaveData_SYS_ID();
            SYSID_CurrentStage = SYSID_TERMINATE;
        } else {
            if(t_temp >= 1.0/f_now) {
                t_temp = 0.0;
                f_now = f_now*r_freq;
                FILE_LOG(logSUCCESS) << " Current Frequency : " << f_now;
                n_now++;
            }
        }
        return false;
    }
    case SYSID_TERMINATE:
    {
        if (TimeIsZero()) {
            TimeLimitSet(2.0);
            FILE_LOG(logWARNING) << " CoM Model Identification Terminate... ";
        }

        if(RorL == false) {
            Submotion_RF2CoM_Pos(TimeLeft(), SYSID_Xini_CoM, zv, zv);
        } else {
            Submotion_LF2CoM_Pos(TimeLeft(), SYSID_Xini_CoM, zv, zv);
        }

        TimeUpdate();
        if (TimeCheck()) {
            TimeReset();
            SYSID_CurrentStage = SYSID_INIT;
            FILE_LOG(logSUCCESS) << " System ID Finish! ";
            return true;
        }
        return false;
    }

    default :
        return false;
    }
}
