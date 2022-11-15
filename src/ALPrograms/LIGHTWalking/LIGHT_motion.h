#ifndef LIGHT_MOTION_H
#define LIGHT_MOTION_H

#include "LIGHT_qp.h"
#include "LIGHT_commands.h"
#include "LIGHT_var_and_func.h"
#include "LIGHT_robotmodel.h"
#include "LIGHT_dynamics.h"
#include "LIGHT_kinematics.h"

#include "ManualCAN.h"

class LIGHTWholeMotions
{

public:
    double _t; // operation time
    double _tlimit; // operation time
    bool   FlagTimeUpdate;

public:
    bool StanceFoot;
    enum Foot_ID {
        RIGHTFOOT = -1,
        DOUBLEFOOT = 0,
        LEFTFOOT = 1,
    };

    enum _PREVIEW_STEPS {
        PREVIEW_STEPS = 4
    };

    struct StepData {
        bool Occupied;              // Is this Buffer occupied?
        int NextSupportPhase;       // SUPPORTCONTROL_RDSP = 1
                                    // SUPPORTCONTROL_LDSP = 2
                                    // SUPPORTCONTROL_RSSP = 3
                                    // SUPPORTCONTROL_LSSP = 4

        double t_STEP;              // Step Time
        Vector3d X_STEP;            // Step Foot Desitination Position on ground (ZMP Destination)
        Vector3d X_STEP_adj;        // Additional Step Foot Adjustment for push recovery
        Vector3d X_OFFSET;          // Capture Offset from Step Position Center
        double Yaw_STEP;            // Step Foot Yaw Orientation

        Vector3d V_DES;             // Desired moving velocity
        Vector3d ZMP;               // ZMP reference w.r.t 'Current Step Frame'

        bool DSP_Transition;        // Is the DSP transition included?
        double t_DSP;

        bool FinalStep;             // Is this step a final step?
    } SDB[PREVIEW_STEPS];           // StepDateButter, Maximum Number of Preview Steps : 4
    Vector3d    ZMP_CurSTEP;
    double      t_CurSTEP;
    int         CurStepSupportPhase;
    bool        CurFlagDSP;
    Vector3d    ZMP_forStepOver;
    int        SupportFootChangeFlag; // -1 : Change to Right Frame, 1 : Change to Left Frame, 0 : none

    void StepInterferenceCheck(int i);
    void StepAdjustment();
    void StepAdjustment_XDirDecoupled();

    Vector3d Xref_CoM;
    Vector3d dXref_CoM;
    Vector3d Xref_CP;
    Vector3d dXref_CP;
    Vector3d Xref_ZMP;

    inline int GetFinalStep() {
        for(int i=0;i<PREVIEW_STEPS;i++) { if(SDB[i].FinalStep == true) return i; }
        return -1;
    }

    void StepDataBuffer_Clear(int _N = 0);
    
    void StepDataBuffer_Initialize_LDSP2RDSP(double _StepTime, double _X_Offset, double _Y_Offset);
    void StepDataBuffer_Initialize_RDSP2LDSP(double _StepTime, double _X_Offset, double _Y_Offset);
    void StepDataBuffer_Initialize_RFSwingUp(double _StepTime, double _X_Offset, double _Y_Offset);
    void StepDataBuffer_Initialize_RFSwingDown(double _StepTime, double _DSPTime, double _X_Step, double _Y_Step, double _Yaw_Step);
    void StepDataBuffer_Initialize_LFSwingUp(double _StepTime, double _X_Offset, double _Y_Offset);
    void StepDataBuffer_Initialize_LFSwingDown(double _StepTime, double _DSPTime, double _X_Step, double _Y_Step, double _Yaw_Step);
    void StepDataBuffer_Initialize_Walking(Vector3d _Initial_Stance,
                                          double _StepTime, double _DSPTime, double _StanceWidth, double _X_Offset, double _Y_Offset, double _Yaw_Offset, double _X_Step, double _Y_Step, double _Yaw_Step);
    void StepDataBuffer_AddStep(double _StepTime, double _DSPTime, double _StanceWidth, double _X_Offset, double _Y_Offset, double _Yaw_Offset, double _X_Step, double _Y_Step, double _Yaw_Step);
    void StepDataBuffer_PushOutStep();

    bool StepDataBuffer_Update();
    bool StepDataBuffer_Update_RDSP2RSSP();
    bool StepDataBuffer_Update_RDSP2LDSP();
    bool StepDataBuffer_Update_RDSP2LSSP();
    bool StepDataBuffer_Update_LDSP2LSSP();
    bool StepDataBuffer_Update_LDSP2RDSP();
    bool StepDataBuffer_Update_LDSP2RSSP();
    bool StepDataBuffer_Update_RSSP2RDSP();
    bool StepDataBuffer_Update_RSSP2LDSP();
    bool StepDataBuffer_Update_RSSP2LSSP();
    bool StepDataBuffer_Update_LSSP2LDSP();
    bool StepDataBuffer_Update_LSSP2RDSP();
    bool StepDataBuffer_Update_LSSP2RSSP();

    int JoyStickCommand_StartWalk, JoyStickCommand_StopWalk;
    double JoyStickCommand_StepX, JoyStickCommand_StepY, JoyStickCommand_StepYaw;

    LIGHT_QP QP_xCoMRef4Walking;
    LIGHT_QP QP_yCoMRef4Walking;
    double wn_X_pattern;
    double zeta_X_pattern;
    double wn_Y_pattern;
    double zeta_Y_pattern;
    int N_horizon;
    double dT_horizon;

    void GetFutureStance(double _t,
                         int& StanceLeg, Vector3d& StancePosition, double &StanceZAngle,
                         bool& StepOn, Vector3d& StepPosition, double& StepZAngle);
    Vector3d GetFutureVelocityRef(double _t);
    Vector3d GetFutureStepPosition(double _t);

    void Generate_CPandZMP_BasicPattern(int _N_Prev, double _dT_Prev,
                                        MatrixNd &_CPref_horizon, MatrixNd &_ZMPref_horizon);
    void Generate_ZMP_BasicPattern(int _N_Prev, double _dT_Prev,
                                   MatrixNd &_ZMPref_horizon);

    bool Generate_CPref_withMPC(int _N_Prev, double _dT_Prev,
                                MatrixNd &_CPref_horizon, MatrixNd &_ZMPref_horizon, MatrixNd &_CoMref_horizon);
    bool Generate_CPref_withMPC_SimplifiednDecoupled(bool _Init, int _N_Prev, double _dT_Prev,
                                                     MatrixNd& _CPref_horizon, MatrixNd& _ZMPref_horizon, MatrixNd& _CoMref_horizon);

    bool Generate_CoMPattern(Vector3d Xpre_CoM, Vector3d dXpre_CoM,
                             Vector3d &u_CoM, Vector3d &u_dCoM, Vector3d &u_ddCoM,
                             Vector3d &Xnext_CoM, Vector3d &dXnext_CoM, Vector3d &ddXnext_CoM);

    void GenerateWalkingPattern_Formulation(Vector3d Xref_CoM, Vector3d dXref_CoM,
                                            Vector3d u_CoM, Vector3d u_dCoM, Vector3d u_ddCoM);
    void GenerateWalkingPattern_Update(Vector3d Xnow_CoM, Vector3d dXnow_CoM,
                                       Vector3d u_CoM, Vector3d u_dCoM, Vector3d u_ddCoM,
                                       MatrixNd& _CPref, MatrixNd& _ZMPref);
    bool GenerateWalkingPattern_Solve(Vector3d Xnow_CoM, Vector3d dXnow_CoM,
                                      Vector3d &u_CoM, Vector3d &u_dCoM, Vector3d &u_ddCoM,
                                      Vector3d &Xnext_CoM, Vector3d &dXnext_CoM, Vector3d &ddXnext_CoM);

    /////////////////////////////////////////////////////////////////////////////////////////////

    //    LIGHTWholeMotions() : Walking_MPC(*this), Walking_PVC(*this) {
    //        _t = 0.0;
    //        _tlimit = 10000.0;
    //        StanceFoot = RIGHTFOOT;
    //    }

    void TimeReset(void) { _t = 0.0; FlagTimeUpdate = true; }
    void TimeUpdate(void) { if(FlagTimeUpdate){_t = _t+SYS_DT_WALKING;}}
    double TimeNow(void) {return _t;}
    void   TimeShow(void) { std::cout << "t : " << _t << std::endl; }
    double TimeLimit(void) {return _tlimit;}
    double TimeLeft(void) {return (_tlimit - _t);}
    void TimeLimitSet(double limit_time) {_tlimit = limit_time;}
    bool TimeCheck(void) {
        if(_t>_tlimit) {
            FlagTimeUpdate = false;
            return true;
        } else {return false;}
    }
    bool TimeIsZero(void) {
        if(_t == 0.0) {return true;}
        else{return false;}
    }
    bool Flag_OperationInit = false;

    void ClearDesiredMotions();

    ///////////////////////////////////////////////////////////////
    ///  [Sub Motions]
    /// 1. Pelvis2RF Orientation
    /// 2. Pelvis2RF Position
    /// 3. Pelvis2LF Orientation
    /// 4. Pelvis2LF Position
    ///
    /// 5. RF2Pelvis Orientation
    /// 6. RF2CoM    Position
    /// 7. RF2LF     Orientation
    /// 8. RF2LF     Position
    ///
    /// 9. LF2Pelvis Orientation
    /// 10. LF2CoM   Position
    /// 11. LF2RF    Orientation
    /// 12. LF2RF    Position
    ///
    /// 13. RightLeg Joint
    /// 14. LeftLeg  Joint
    /// 15. Waist    Joint
    ///////////////////////////////////////////////////////////////
    void Orientation_Trajectory(double t, double T, Matrix3d Rini, Matrix3d Rdes, Matrix3d &Rnext);
    void Position_5th_Trajectory(double Tmove, Vector3d Xini, Vector3d dXini, Vector3d ddXini, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes, Vector3d &Xnext, Vector3d &dXnext, Vector3d &ddXnext);
    void Position_3rd_Trajectory(double Tmove, Vector3d Xini, Vector3d dXini, Vector3d Xdes, Vector3d dXdes, Vector3d &Xnext, Vector3d &dXnext);
    void Joint_5th_Trajectory(double Tmove, double th_ini, double dth_ini, double ddth_ini, double th_des, double dth_des, double ddth_des, double &th_next, double &dth_next, double &ddth_next);
    void Joint_3rd_Trajectory(double Tmove, double th_ini, double dth_ini, double th_des, double dth_des, double &th_next, double &dth_next);

    void Submotion_Global2Pel_Ori(double t, double T, Matrix3d Rdes);
    void Submotion_Global2RF_Ori(double t, double T, Matrix3d Rdes);
    void Submotion_Global2LF_Ori(double t, double T, Matrix3d Rdes);
    void Submotion_Pelvis2RF_Ori(double t, double T, Matrix3d Rdes);
    void Submotion_Pelvis2LF_Ori(double t, double T, Matrix3d Rdes);

    void Submotion_Global2RF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);
    void Submotion_Global2LF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);
    void Submotion_Pelvis2RF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);
    void Submotion_Pelvis2LF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);
    void Submotion_RF2CoM_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);
    void Submotion_RF2LF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);
    void Submotion_LF2CoM_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);
    void Submotion_LF2RF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes);

    void Submotion_Global2RFswing_forWalking(double t_swing, double RotZdes);
    void Submotion_Global2LFswing_forWalking(double t_swing, double RotZdes);
    void Submotion_LF2RFSwing_forWalking(double t_Swing, Vector3d _Xdes_LF2RF, double z_SwingUp);
    void Submotion_RF2LFSwing_forWalking(double t_Swing, Vector3d _Xdes_RF2LF, double z_SwingUp);
    void Submotion_LF2RFSwingUp(double t_Swing, Vector3d _Xdes_RF2LF);
    void Submotion_LF2RFSwingDown(double t_Swing, Vector3d _Xdes_RF2LF);
    void Submotion_RF2LFSwingUp(double t_Swing, Vector3d _Xdes_RF2LF);
    void Submotion_RF2LFSwingDown(double t_Swing, Vector3d _Xdes_RF2LF);

    void Submotion_RightLeg_Q(double TIME, VectorNd Qdes, VectorNd dQdes, VectorNd ddQdes);
    void Submotion_LeftLeg_Q(double TIME, VectorNd Qdes, VectorNd dQdes, VectorNd ddQdes);
    void Submotion_Waist_Q(double TIME, VectorNd Qdes, VectorNd dQdes, VectorNd ddQdes);

    ///////////////////////////////////////////////////////////////
    ///  Basic Operation Set

    bool JointSpaceMove_ALL(double _TIME, MatrixNd _Qdes);
    bool WorkSpaceMove_ALL(double _TIME, Matrix3d _Rdes_RF, Vector3d _Xdes_RF, Matrix3d _Rdes_LF, Vector3d _Xdes_LF);
    bool CoM_Move_RDSP(double _TIME,
                       Matrix3d _Rdes_Global2Pel,
                       Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                       Vector3d _Xdes_RF2CoM, Vector3d _dXdes_RF2CoM, Vector3d _ddXdes_RF2CoM);
    bool CoM_Move_LDSP(double _TIME,
                       Matrix3d _Rdes_Global2Pel,
                       Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                       Vector3d _Xdes_LF2CoM, Vector3d _dXdes_LF2CoM, Vector3d _ddXdes_LF2CoM);
    bool CoM_Move_RSSP(double _TIME,
                       Matrix3d _Rdes_Global2Pel,
                       Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                       Vector3d _Xdes_RF2CoM, Vector3d _dXdes_RF2CoM, Vector3d _ddXdes_RF2CoM,
                       Vector3d _Xdes_RF2LF, Vector3d _dXdes_RF2LF, Vector3d _ddXdes_RF2LF);
    bool CoM_Move_LSSP(double _TIME,
                       Matrix3d _Rdes_Global2Pel,
                       Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                       Vector3d _Xdes_LF2CoM, Vector3d _dXdes_LF2CoM, Vector3d _ddXdes_LF2CoM,
                       Vector3d _Xdes_LF2RF, Vector3d _dXdes_LF2RF, Vector3d _ddXdes_LF2RF);

    enum SUPPORTPHASE_TRANSITION_SET {
        SUPPORTPHASE_TRANSITION_NOACT = 0,
        SUPPORTPHASE_TRANSITION_RDSP_RSSP,
        SUPPORTPHASE_TRANSITION_RSSP_RDSP,
        SUPPORTPHASE_TRANSITION_RDSP_LDSP,
        SUPPORTPHASE_TRANSITION_LDSP_RDSP,
        SUPPORTPHASE_TRANSITION_LDSP_LSSP,
        SUPPORTPHASE_TRANSITION_LSSP_LDSP,
    };
    int SupportPhase_Transition(int _TR_MODE);

    ////////////////////////////////////////////////////////////////////////////////////////
    /// Support Control Set

    enum SUPPORTCONTROL_MODESET {
        SUPPORTCONTROL_NOACT = 0,
        SUPPORTCONTROL_RDSP = -2,
        SUPPORTCONTROL_RSSP = -1,
        SUPPORTCONTROL_LSSP = 1,
        SUPPORTCONTROL_LDSP = 2,
        SUPPORTCONTROL_FLOATING,
        SUPPORTCONTROL_DIRECTTORQUE,
    };

    double CoM_wn_X;
    double CoM_wn_Y;
    double CoM_zeta_X;
    double CoM_zeta_Y;
    void SetLeadCompensateGain(double _wn_X, double _wn_Y, double _zeta_X, double _zeta_Y);

    double WeightCompensation_DSP;
    Vector3d Freq_body_pos_DSP, Zeta_body_pos_DSP;
    Vector3d Freq_body_ori_DSP, Zeta_body_ori_DSP;
    Vector3d Freq_foot_pos_DSP, Zeta_foot_pos_DSP;
    Vector3d Freq_foot_ori_DSP, Zeta_foot_ori_DSP;
    void SetSupportControlGain_DSP(double _WeightCompensation,
                                   Vector3d _Freq_body_pos, Vector3d _Zeta_body_pos,
                                   Vector3d _Freq_body_ori, Vector3d _Zeta_body_ori,
                                   Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                   Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori);

    double WeightCompensation_RSSP;
    Vector3d Freq_body_pos_RSSP, Zeta_body_pos_RSSP;
    Vector3d Freq_body_ori_RSSP, Zeta_body_ori_RSSP;
    Vector3d Freq_foot_pos_RSSP, Zeta_foot_pos_RSSP;
    Vector3d Freq_foot_ori_RSSP, Zeta_foot_ori_RSSP;
    void SetSupportControlGain_RSSP(double _WeightCompensation,
                                       Vector3d _Freq_body_pos, Vector3d _Zeta_body_pos,
                                       Vector3d _Freq_body_ori, Vector3d _Zeta_body_ori,
                                       Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                       Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori);

    double WeightCompensation_LSSP;
    Vector3d Freq_body_pos_LSSP, Zeta_body_pos_LSSP;
    Vector3d Freq_body_ori_LSSP, Zeta_body_ori_LSSP;
    Vector3d Freq_foot_pos_LSSP, Zeta_foot_pos_LSSP;
    Vector3d Freq_foot_ori_LSSP, Zeta_foot_ori_LSSP;
    void SetSupportControlGain_LSSP(double _WeightCompensation,
                                       Vector3d _Freq_body_pos, Vector3d _Zeta_body_pos,
                                       Vector3d _Freq_body_ori, Vector3d _Zeta_body_ori,
                                       Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                       Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori);


    Vector3d Freq_foot_pos_Float, Zeta_foot_pos_Float;
    Vector3d Freq_foot_ori_Float, Zeta_foot_ori_Float;
    void SetSupportControlGain_Float(Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                     Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori);

    bool SupportControl_DSP_byWBD(char _RefFrame);
    bool SupportControl_SSP_RF_byWBD();
    bool SupportControl_SSP_LF_byWBD();
    bool SupportControl_Float_byWBD();

    bool SupportControl_Integrated_byWBD(int _RefFrame, bool _IsRFContact, bool _IsLFContact);

    bool SupportControl_DirectTorque();
    bool SupportControl_Noact();

    //////////////////////////////////////////////////////////////////////////////////////
    /// Deflected Pelvis Compensation + Swing Foot Compensation

    bool SwingFootComp_Float();
    bool SwingFootComp_RF();
    bool SwingFootComp_LF();

    // Swing Foot Compensation
    bool Flag_CoMHeightCtrl = true;
    bool Flag_CoMSwayCtrl = true;
    double CoMHeightControl(double Damping, double Spring, double Integral);

    //////////////////////////////////////////////////////////////////////////////////////
    /// System Identification

    bool SYSID_CoMRef2CoM(int SYSID_TYPE, double CoM_Mag, int f_max, double r_freq);

    //////////////////////////////////////////////////////////////////////////////////////
    /// LIGHT Motion Sets

    bool CoM_Move_RDSP_smooth(double _TIME, double _Yoffset, bool _CtrlOn);
    bool CoM_Move_LDSP_smooth(double _TIME, double _Yoffset, bool _CtrlOn);

    bool RFSwingUp_Dynamic(double _TIME, Vector3d _ZMP_OFFSET, double _Yaw_LF2RF);
    bool RFSwingDown_Dynamic(double _TIME, Vector3d _Xdes_LF2RF, double _Yaw_LF2RF);
    bool LFSwingUp_Dynamic(double _TIME, Vector3d _ZMP_OFFSET, double _Yaw_RF2LF);
    bool LFSwingDown_Dynamic(double _TIME, Vector3d _Xdes_RF2LF, double _Yaw_RF2LF);

    bool AirWalking(double _PERIOD, double _MAG, int _NUM);

    bool StaticWalking(double _STEP_LENGTH, double _STEP_TIME, int _STEP_NUM);

    bool Squat_DSP(double _HEIGHT, double _TIME, double _NUM);
    bool Squat_SSP(double _HEIGHT, double _TIME, double _NUM);

    bool Walk(double _StepTime, double _DSPTime, double _StanceWidth, int _StepNumber,
              double _X_Offset, double _Y_Offset, double _Yaw_Offset,
              double _X_Step, double _Y_Step, double _Yaw_Step, double _StanceOffset,
              bool _Flag_Terminate, int &CurStepNumber);

    bool JumpTest(double _InitHeight, double _MaxHeight, double _InitBodyAngle, double _FinBodyAngle,
                  double _RobotMass, double _JumpTime, double _Spring, double _ReturnTime,
                  double _CoMxOffset, double _CoMxVelocity);

    bool FullTaskScenario();

    //////////////////////////////////////////////////////////////////////////////////////
    /// Loading and Setting Future Supply Pressure and Flow Rate

    void Calc_FuturePressureReference(double _t_now, double _T_set, double _Ps_des);
    bool Calc_FuturePressureReference_WithWalkPattern(int _N_Prev, double _dT_Prev,
                                                      Vector3d *_CPref_horizon, Vector3d *_ZMPref_horizon, Vector3d *_CoMref_horizon);

    bool IsLoaded_FuturePumpReference_FullTaskScenario = false;
    VectorNd Ps_Future_FullTaskScenario;
    VectorNd Qp_Future_FullTaskScenario;
    void Load_FuturePumpReference_FullTaskScenario();

    LIGHTWholeMotions() {
        _t = 0.0;
        _tlimit = 100000.0;
        FlagTimeUpdate = true;
        StanceFoot = RIGHTFOOT;

        JoyStickCommand_StartWalk = false;
        JoyStickCommand_StopWalk = false;
        JoyStickCommand_StepX = 0.0;
        JoyStickCommand_StepY = 0.0;
        JoyStickCommand_StepYaw = 0.0;

        CoM_wn_X = 15.0;
        CoM_wn_Y = 15.0;
        CoM_zeta_X = 0.6;
        CoM_zeta_Y = 0.6;

        WeightCompensation_DSP = 1.00;
        Freq_body_pos_DSP = Vector3d::Ones()*0.70;
        Freq_body_ori_DSP = Vector3d::Ones()*1.10;
        Freq_foot_pos_DSP = Vector3d::Ones()*0.50;
        Freq_foot_ori_DSP = Vector3d::Ones()*0.50;
        Zeta_body_pos_DSP = Vector3d::Ones()*0.60;
        Zeta_body_ori_DSP = Vector3d::Ones()*0.60;
        Zeta_foot_pos_DSP = Vector3d::Ones()*0.60;
        Zeta_foot_ori_DSP = Vector3d::Ones()*0.60;

        WeightCompensation_RSSP = 1.00;
        Freq_body_pos_RSSP = Vector3d::Ones()*0.50;
        Freq_body_ori_RSSP = Vector3d::Ones()*1.10;
        Freq_foot_pos_RSSP = Vector3d::Ones()*1.00;
        Freq_foot_ori_RSSP = Vector3d::Ones()*0.50;
        Zeta_body_pos_RSSP = Vector3d::Ones()*0.50;
        Zeta_body_ori_RSSP = Vector3d::Ones()*0.50;
        Zeta_foot_pos_RSSP = Vector3d::Ones()*0.50;
        Zeta_foot_ori_RSSP = Vector3d::Ones()*0.50;

        WeightCompensation_LSSP = WeightCompensation_RSSP;
        Freq_body_pos_LSSP = Freq_body_pos_RSSP;
        Freq_body_ori_LSSP = Freq_body_ori_RSSP;
        Freq_foot_pos_LSSP = Freq_foot_pos_RSSP;
        Freq_foot_ori_LSSP = Freq_foot_ori_RSSP;
        Zeta_body_pos_LSSP = Zeta_body_pos_RSSP;
        Zeta_body_ori_LSSP = Zeta_body_ori_RSSP;
        Zeta_foot_pos_LSSP = Zeta_foot_pos_RSSP;
        Zeta_foot_ori_LSSP = Zeta_foot_ori_RSSP;

        Freq_foot_pos_Float = Freq_foot_pos_RSSP;
        Freq_foot_ori_Float = Freq_foot_ori_RSSP;
        Zeta_foot_pos_Float = Zeta_foot_pos_RSSP;
        Zeta_foot_ori_Float = Zeta_foot_ori_RSSP;
    }

};

#endif // LIGHT_MOTION_H
