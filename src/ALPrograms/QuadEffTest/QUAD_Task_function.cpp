#include "QUAD_jointsetmodel.h"

// =============================================================================

bool QUADJointSet::ArmTask_MoveJoint()
{
    static int CurrentStage = ArmTask_PARAMETER_SETTING;
    static double t = 0.0;
    double t_set = 3.0;

    static double Qini, dQini, ddQini;

    for(int idx_prev=0;idx_prev<=sharedREF->N_window;idx_prev++) {
        int CurrentStage_prev = CurrentStage;
        double t_prev = t + (double)idx_prev*sharedREF->dT_window;

        switch(CurrentStage_prev) {
        case ArmTask_PARAMETER_SETTING:
            if (t_prev < t_set) {
                if(t_prev == 0.0) {
                    Qini = Qref_arm;
                    dQini = dQref_arm;
                    ddQini = ddQref_arm;
                }

                Qdes_arm(0,idx_prev) = Qini;
                dQdes_arm(0,idx_prev) = dQini;
                ddQdes_arm(0,idx_prev) = ddQini;
                Tdes_arm(0,idx_prev) = m_ArmTask*l_ArmTask*l_ArmTask*ddQini + m_ArmTask*l_ArmTask*g_const*cos(Qini);
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_set;
                    CurrentStage = ArmTask_PROCESS;
                }
                t_prev = t_prev - t_set;
            }

        case ArmTask_PROCESS:
            if (t_prev < t_ArmTask) {
                double _q, _dq, _ddq;
                PolyTrajectory_5th(t_ArmTask,t_prev,
                               Qini,dQini,ddQini,
                               q_ArmTask,0.0,0.0,
                               _q, _dq, _ddq);
                Qdes_arm(0,idx_prev) = _q;
                dQdes_arm(0,idx_prev) = _dq;
                ddQdes_arm(0,idx_prev) = _ddq;
                Tdes_arm(0,idx_prev) = m_ArmTask*l_ArmTask*l_ArmTask*_ddq + m_ArmTask*l_ArmTask*g_const*cos(_q);
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_ArmTask;
                    CurrentStage = ArmTask_FINISH;
                }
                t_prev = t_prev - t_ArmTask;
            }

        case ArmTask_FINISH:
            Qdes_arm(0,idx_prev) = q_ArmTask;
            dQdes_arm(0,idx_prev) = 0.0;
            ddQdes_arm(0,idx_prev) = 0.0;
            Tdes_arm(0,idx_prev) =  m_ArmTask*l_ArmTask*g_const*cos(q_ArmTask);

            if(idx_prev==0) {
                t = 0.0;
                CurrentStage = ArmTask_PARAMETER_SETTING;
                return true;
            }
        }
    }

    t = t + SYS_DT;
    return false;
}

bool QUADJointSet::ArmTask_SineWave()
{
    static int CurrentStage = ArmTask_PARAMETER_SETTING;
    static double t = 0.0;
    double t_set = 3.0;

    static double Qini, dQini, ddQini;
    for(int idx_prev=0;idx_prev<=sharedREF->N_window;idx_prev++) {
        int CurrentStage_prev = CurrentStage;
        double t_prev = t + (double)idx_prev*sharedREF->dT_window;

        switch(CurrentStage_prev) {
        case ArmTask_PARAMETER_SETTING:
            if (t_prev < t_set) {
                if(t_prev == 0.0) {
                    Qini = Qref_arm;
                    dQini = dQref_arm;
                    ddQini = ddQref_arm;
                }

                Qdes_arm(0,idx_prev) = Qini;
                dQdes_arm(0,idx_prev) = dQini;
                ddQdes_arm(0,idx_prev) = ddQini;
                Tdes_arm(0,idx_prev) = m_ArmTask*l_ArmTask*l_ArmTask*ddQini + m_ArmTask*l_ArmTask*g_const*cos(Qini);
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_set;
                    CurrentStage = ArmTask_PROCESS;
                }
                t_prev = t_prev - t_set;
            }

        case ArmTask_PROCESS:
            if (t_prev < (double)n_ArmTask*t_ArmTask) {
                double _q, _dq, _ddq;
                _q = Qini + q_ArmTask*(1.0-cos(2.0*PI*t_prev/t_ArmTask))/2.0;
                _dq = q_ArmTask*(2.0*PI/t_ArmTask)*sin(2.0*PI*t_prev/t_ArmTask)/2.0;
                _ddq = q_ArmTask*(2.0*PI/t_ArmTask)*(2.0*PI/t_ArmTask)*sin(2.0*PI*t_prev/t_ArmTask)/2.0;
                Qdes_arm(0,idx_prev) = _q;
                dQdes_arm(0,idx_prev) = _dq;
                ddQdes_arm(0,idx_prev) = _ddq;
                Tdes_arm(0,idx_prev) = m_ArmTask*l_ArmTask*l_ArmTask*_ddq + m_ArmTask*l_ArmTask*g_const*cos(_q);
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_ArmTask;
                    CurrentStage = ArmTask_FINISH;
                }
                t_prev = t_prev - t_ArmTask;
            }

        case ArmTask_FINISH:
            Qdes_arm(0,idx_prev) = Qini;
            dQdes_arm(0,idx_prev) = 0.0;
            ddQdes_arm(0,idx_prev) = 0.0;
            Tdes_arm(0,idx_prev) =  m_ArmTask*l_ArmTask*g_const*cos(Qini);
            if(idx_prev==0) {
                t = 0.0;
                CurrentStage = ArmTask_PARAMETER_SETTING;
                return true;
            }
        }
    }

    t = t + SYS_DT;
    return false;
}

// =============================================================================

bool QUADJointSet::JointSpaceMove()
{
    static int CurrentStage = JointSpaceMove_PARAMETER_SETTING;
    static double t = 0.0;
    double t_set = 3.0;

    static VectorNd Qini, dQini, ddQini;

    for(int idx_prev=0;idx_prev<=sharedREF->N_window;idx_prev++) {
        int CurrentStage_prev = CurrentStage;
        double t_prev = t + (double)idx_prev*sharedREF->dT_window;

        switch(CurrentStage_prev) {
        case JointSpaceMove_PARAMETER_SETTING:
            if (t_prev < t_set) {
                if(t_prev == 0.0) {
                    CtrlSpace = JointSpace;
                    Flag_InvKin = true;
                    Flag_InvDyn = true;

                    Qini = Qref;
                    dQini = dQref;
                    ddQini = ddQref;
                }

                Qdes.col(idx_prev) = Qini;
                dQdes.col(idx_prev) = dQini;
                ddQdes.col(idx_prev) = ddQini;

                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_set;
                    CurrentStage = JointSpaceMove_PROCESS;
                }
                t_prev = t_prev - t_set;
            }

        case JointSpaceMove_PROCESS:
            if (t_prev < t_JointSpaceMove) {
                for(int i=0;i<n_dof;i++) {
                    double _q, _dq, _ddq;
                    PolyTrajectory_5th(t_JointSpaceMove,t_prev,
                                   Qini(i),dQini(i),ddQini(i),Q_JointSpaceMove(i),0.0,0.0,
                                   _q, _dq, _ddq);
                    Qdes(i,idx_prev) = _q;
                    dQdes(i,idx_prev) = _dq;
                    ddQdes(i,idx_prev) = _ddq;
                }
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_JointSpaceMove;
                    CurrentStage = JointSpaceMove_FINISH;
                }
                t_prev = t_prev - t_JointSpaceMove;
            }

        case JointSpaceMove_FINISH:
            Qdes.col(idx_prev) = Q_JointSpaceMove;
            dQdes.col(idx_prev) = VectorNd::Zero(n_dof);
            ddQdes.col(idx_prev) = VectorNd::Zero(n_dof);

            if(idx_prev==0) {
                t = 0.0;
                CurrentStage = JointSpaceMove_PARAMETER_SETTING;
                return true;
            }
        }
    }

    t = t + SYS_DT;
    return false;
}

// =============================================================================

bool QUADJointSet::TestJoints_MoveJoint()
{
    static int CurrentStage = TestJoints_PARAMETER_SETTING;
    static double t = 0.0;
    double t_set = 3.0;

    static VectorNd Qini, dQini, ddQini;

    for(int idx_prev=0;idx_prev<=sharedREF->N_window;idx_prev++) {
        int CurrentStage_prev = CurrentStage;
        double t_prev = t + (double)idx_prev*sharedREF->dT_window;

        switch(CurrentStage_prev) {
        case TestJoints_PARAMETER_SETTING:
            if (t_prev < t_set) {
                if(t_prev == 0.0) {
                    CtrlSpace = JointSpace;
                    Flag_InvKin = true;
                    Flag_InvDyn = true;

                    Qini = Qref;
                    dQini = dQref;
                    ddQini = ddQref;
                }

                Qdes.col(idx_prev) = Qini;
                dQdes.col(idx_prev) = dQini;
                ddQdes.col(idx_prev) = ddQini;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_set;
                    CurrentStage = TestJoints_PROCESS;
                }
                t_prev = t_prev - t_set;
            }

        case TestJoints_PROCESS:
            if (t_prev < t_TestJoints) {
                double _q, _dq, _ddq;
                PolyTrajectory_5th(t_TestJoints,t_prev,
                               Qini(idx_TestJoints),dQini(idx_TestJoints),ddQini(idx_TestJoints),
                               q_TestJoints,0.0,0.0,
                               _q, _dq, _ddq);
                Qdes(idx_TestJoints,idx_prev) = _q;
                dQdes(idx_TestJoints,idx_prev) = _dq;
                ddQdes(idx_TestJoints,idx_prev) = _ddq;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_TestJoints;
                    CurrentStage = TestJoints_FINISH;
                }
                t_prev = t_prev - t_TestJoints;
            }

        case TestJoints_FINISH:
            Qdes(idx_TestJoints,idx_prev) = q_TestJoints;
            dQdes(idx_TestJoints,idx_prev) = 0.0;
            ddQdes(idx_TestJoints,idx_prev) = 0.0;

            if(idx_prev==0) {
                t = 0.0;
                CurrentStage = TestJoints_PARAMETER_SETTING;
                return true;
            }
        }
    }

    t = t + SYS_DT;
    return false;
}

bool QUADJointSet::TestJoints_SineWave()
{
    static int CurrentStage = TestJoints_PARAMETER_SETTING;
    static double t = 0.0;
    double t_set = 3.0;

    static VectorNd Qini, dQini, ddQini;

    for(int idx_prev=0;idx_prev<=sharedREF->N_window;idx_prev++) {
        int CurrentStage_prev = CurrentStage;
        double t_prev = t + (double)idx_prev*sharedREF->dT_window;

        switch(CurrentStage_prev) {
        case TestJoints_PARAMETER_SETTING:
            if (t_prev < t_set) {
                if(t_prev == 0.0) {
                    CtrlSpace = JointSpace;
                    Flag_InvKin = true;
                    Flag_InvDyn = true;

                    Qini = Qref;
                    dQini = dQref;
                    ddQini = ddQref;
                }

                Qdes.col(idx_prev) = Qini;
                dQdes.col(idx_prev) = dQini;
                ddQdes.col(idx_prev) = ddQini;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_set;
                    CurrentStage = TestJoints_PROCESS;
                }
                t_prev = t_prev - t_set;
            }

        case TestJoints_PROCESS:
            if (t_prev < (double)n_TestJoints*t_TestJoints) {
                double _q, _dq, _ddq;
                _q = Qini(idx_TestJoints) + q_TestJoints*(1.0-cos(2.0*PI*t_prev/t_TestJoints))/2.0;
                _dq = q_TestJoints*(2.0*PI/t_TestJoints)*sin(2.0*PI*t_prev/t_TestJoints)/2.0;
                _ddq = q_TestJoints*(2.0*PI/t_TestJoints)*(2.0*PI/t_TestJoints)*sin(2.0*PI*t_prev/t_TestJoints)/2.0;
                Qdes(idx_TestJoints,idx_prev) = _q;
                dQdes(idx_TestJoints,idx_prev) = _dq;
                ddQdes(idx_TestJoints,idx_prev) = _ddq;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_TestJoints;
                    CurrentStage = TestJoints_FINISH;
                }
                t_prev = t_prev - t_TestJoints;
            }

        case TestJoints_FINISH:
            Qdes(idx_TestJoints,idx_prev) = Qini(idx_TestJoints);
            dQdes(idx_TestJoints,idx_prev) = 0.0;
            ddQdes(idx_TestJoints,idx_prev) = 0.0;

            if(idx_prev==0) {
                t = 0.0;
                CurrentStage = TestJoints_PARAMETER_SETTING;
                return true;
            }
        }
    }

    t = t + SYS_DT;
    return false;
}

// =============================================================================


bool QUADJointSet::SquatMotion()
{
    static int CurrentStage = Squat_PARAMETER_SETTING;
    static double t = 0.0;
    double t_set = 3.0;
    double t_ready = 3.0;
    double t_fin = 1000.0;

    Vector3d Xdes_Ready(0.0,0.0,-0.55);
    static Vector3d Xini, dXini, ddXini;

    static bool FlagForcedStop = false;

//    if(CheckTerminateFlag() && !FlagForcedStop) { // all stop
//        FILE_LOG(logERROR) << "Squat Motion, Forced Stopped!";
//        FlagForcedStop = true;
//        Squat_CurrentStage = Squat_FINISH;
//        t = 0.0;
//    }

    for(int idx_prev=0;idx_prev<=sharedREF->N_window;idx_prev++) {
        int CurrentStage_prev = CurrentStage;
        double t_prev = t + (double)idx_prev*sharedREF->dT_window;

        switch(CurrentStage_prev) {
        case Squat_PARAMETER_SETTING:
            if (t_prev < t_set) {
                if(t_prev == 0.0) {
                    CtrlSpace = TaskSpace;
                    Flag_InvKin = true;
                    Flag_InvDyn = true;

                    Xini = Xref_Foot;
                    dXini = dXref_Foot;
                    ddXini = ddXref_Foot;
                    FILE_LOG(logWARNING) << "Squat Motion, Parameters Setting...";
                }

                Xdes_Foot.col(idx_prev) = Xini;
                dXdes_Foot.col(idx_prev) = dXini;
                ddXdes_Foot.col(idx_prev) = ddXini;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_set;
                    CurrentStage = Squat_READY;
                    FILE_LOG(logWARNING) << "Squat Motion, Ready...";
                }
                t_prev = t_prev - t_set;
            }

        case Squat_READY:
            if (t_prev < t_ready) {
                Vector3d _X, _dX, _ddX;
                PolyTrajectory_Vector_3rd(t_ready,t_prev,
                                     Xini, dXini, Xdes_Ready, zv,
                                     _X, _dX, _ddX);
                Xdes_Foot.col(idx_prev) = _X;
                dXdes_Foot.col(idx_prev) = _dX;
                ddXdes_Foot.col(idx_prev) = _ddX;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_ready;
                    CurrentStage = Squat_PROCESS;
                    save_Flag = true;
                    FILE_LOG(logSUCCESS) << "Squat Motion, Go!";
                }
                t_prev = t_prev - t_ready;
            }

        case Squat_PROCESS:
            t_fin = (double)(n_SquatMotion-1)*t_SquatMotion + 0.75*t_SquatMotion;
            if (t_prev < t_fin) {
                Vector3d Xdes_Squat = Xdes_Ready + X_SquatMotion*(1.0-cos(2.0*PI*t_prev/t_SquatMotion))/2.0;
                Vector3d dXdes_Squat = X_SquatMotion*(2.0*PI/t_SquatMotion)*(sin(2.0*PI*t_prev/t_SquatMotion))/2.0;
                Vector3d ddXdes_Squat = X_SquatMotion*(2.0*PI/t_SquatMotion)*(2.0*PI/t_SquatMotion)*(cos(2.0*PI*t_prev/t_SquatMotion))/2.0;

                Xdes_Foot.col(idx_prev) = Xdes_Squat;
                dXdes_Foot.col(idx_prev) = dXdes_Squat;
                ddXdes_Foot.col(idx_prev) = ddXdes_Squat;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_fin;
                    CurrentStage = Squat_TERMINATE;
                    FILE_LOG(logWARNING) << "Squat Motion, terminate...";
                }
                t_prev = t_prev - t_fin;
            }

        case Squat_TERMINATE:
            t_fin = 0.5*t_SquatMotion;
            if (t_prev < t_fin) {
                Vector3d _X, _dX, _ddX;
                PolyTrajectory_Vector_3rd(t_fin-t_prev,SYS_DT,
                                     Xdes_Foot.col(idx_prev), dXdes_Foot.col(idx_prev), Xdes_Ready, zv,
                                     _X, _dX, _ddX);

                Xdes_Foot.col(idx_prev) = _X;
                dXdes_Foot.col(idx_prev) = _dX;
                ddXdes_Foot.col(idx_prev) = _ddX;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_fin;
                    CurrentStage = Squat_FINISH;
                    save_Flag = false;
                    save_File("SquatMotion");
                    FILE_LOG(logSUCCESS) << "Squat Motion, Finish!";
                }
                t_prev = t_prev - t_fin;
            }

        case Squat_FINISH:
            Xdes_Foot.col(idx_prev) = Xdes_Ready;
            dXdes_Foot.col(idx_prev) = zv;
            ddXdes_Foot.col(idx_prev) = zv;

            if(idx_prev==0) {
                t = 0.0;
                CurrentStage = Squat_PARAMETER_SETTING;
                FlagForcedStop = false;
                return true;
            }

        }
    }
    t = t + SYS_DT;
    return false;
}

// =============================================================================


bool QUADJointSet::SwingMotion()
{
    static int CurrentStage = Swing_PARAMETER_SETTING;
    static int CurrentSwingStage = StepForward;
    static int n_swing = 0;
    static double t = 0.0;
    double t_set = 3.0;
    double t_ready = 3.0;
    double t_fin = 1000.0;

    Vector3d Xdes_Ready(0.0,0.0,-0.55);
    static Vector3d Xini, dXini, ddXini;

    static Vector3d X_stepforward_ini;
    static Vector3d dX_stepforward_ini;
    static Vector3d ddX_stepforward_ini;
    static Vector3d X_stepforward_fin;
    static Vector3d dX_stepforward_fin;
    static Vector3d ddX_stepforward_fin;

    static Vector3d X_stepback_ini;
    static Vector3d dX_stepback_ini;
    static Vector3d X_stepback_fin;
    static Vector3d dX_stepback_fin;

    static bool FlagForcedStop = false;

//    if(CheckTerminateFlag() && !FlagForcedStop) { // all stop
//        FILE_LOG(logERROR) << "Squat Motion, Forced Stopped!";
//        FlagForcedStop = true;
//        Squat_CurrentStage = Squat_FINISH;
//        t = 0.0;
//    }

    for(int idx_prev=0;idx_prev<=sharedREF->N_window;idx_prev++) {
        int CurrentStage_prev = CurrentStage;
        int CurrentSwingStage_prev = CurrentSwingStage;
        int n_swing_prev = n_swing;
        double t_prev = t + (double)idx_prev*sharedREF->dT_window;

        switch(CurrentStage_prev) {
        case Swing_PARAMETER_SETTING:
            if (t_prev < t_set) {
                if(t_prev == 0.0) {
                    CtrlSpace = TaskSpace;
                    Flag_InvKin = true;
                    Flag_InvDyn = true;

                    X_stepforward_ini = Vector3d(-x_SwingMotion/2.0, 0.0, -0.55);
                    dX_stepforward_ini = Vector3d(-0.6*x_SwingMotion/t_SwingMotion, 0.0, 0.25);
                    ddX_stepforward_ini = Vector3d(5.0*x_SwingMotion/t_SwingMotion/t_SwingMotion, 0.0, 5.0*z_SwingMotion/(t_SwingMotion/2.0)/(t_SwingMotion/2.0));
                    X_stepforward_fin = Vector3d(x_SwingMotion/2.0, 0.0, -0.55);
                    dX_stepforward_fin = Vector3d(-0.6*x_SwingMotion/t_SwingMotion,0.0,-0.25);
                    ddX_stepforward_fin = Vector3d(-5.0*x_SwingMotion/t_SwingMotion/t_SwingMotion, 0.0, 5.0*z_SwingMotion/(t_SwingMotion/2.0)/(t_SwingMotion/2.0));

                    X_stepback_ini = Vector3d(x_SwingMotion/2.0, 0.0, -0.55);
                    dX_stepback_ini = Vector3d(-0.6*x_SwingMotion/t_SwingMotion, 0.0, -0.00);
                    X_stepback_fin = Vector3d(-x_SwingMotion/2.0, 0.0, -0.55);
                    dX_stepback_fin = Vector3d(-0.6*x_SwingMotion/t_SwingMotion, 0.0, -0.00);

                    Xini = Xref_Foot;
                    dXini = dXref_Foot;
                    ddXini = ddXref_Foot;
                    FILE_LOG(logWARNING) << "Swing Motion, Parameters Setting...";
                }
                Xdes_Foot.col(idx_prev) = Xini;
                dXdes_Foot.col(idx_prev) = dXini;
                ddXdes_Foot.col(idx_prev) = ddXini;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_set;
                    CurrentStage = Swing_READY;
                    FILE_LOG(logWARNING) << "Swing Motion, Ready...";
                }
                t_prev = t_prev - t_set;
            }

        case Swing_READY:
            if (t_prev < t_ready) {
                Vector3d _X, _dX, _ddX;
                PolyTrajectory_Vector_3rd(t_ready-t_prev,SYS_DT,
                                     Xdes_Foot.col(idx_prev), dXdes_Foot.col(idx_prev), Xdes_Ready, zv,
                                     _X, _dX, _ddX);
                Xdes_Foot.col(idx_prev) = _X;
                dXdes_Foot.col(idx_prev) = _dX;
                ddXdes_Foot.col(idx_prev) = _ddX;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_ready;
                    CurrentStage = Swing_INIT;
                    save_Flag = true;
                    FILE_LOG(logSUCCESS) << "Squat Motion, Go!";
                }
                t_prev = t_prev - t_ready;
            }

        case Swing_INIT:
            t_fin = t_SwingMotion;
            if (t_prev < t_fin) {
                Vector3d _X, _dX, _ddX;
                PolyTrajectory_Vector_3rd(t_fin-t_prev,SYS_DT,
                                     Xdes_Foot.col(idx_prev), dXdes_Foot.col(idx_prev), X_stepback_fin, dX_stepback_fin,
                                     _X, _dX, _ddX);

                Xdes_Foot.col(idx_prev) = _X;
                dXdes_Foot.col(idx_prev) = _dX;
                ddXdes_Foot.col(idx_prev) = _ddX;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_fin;
                    CurrentStage = Swing_SWING;
                    CurrentSwingStage = StepForward;
                }
                t_prev = t_prev - t_fin;
                CurrentSwingStage_prev = StepForward;
            }

        case Swing_SWING:
        {
            bool StepPhaseDecision = false;
            while(!StepPhaseDecision){
                switch(CurrentSwingStage_prev) {
                case StepForward:
                    t_fin = t_SwingMotion;
                    if (t_prev < t_fin) {
                        Vector3d Xn, dXn, ddXn;
                        PolyTrajectory_Vector_5th(t_fin, t_prev,
                                             X_stepforward_ini,dX_stepforward_ini,ddX_stepforward_ini,X_stepforward_fin,dX_stepforward_fin,ddX_stepforward_fin,
                                             Xn, dXn, ddXn);

                        Xdes_Foot.col(idx_prev) = Xn;
                        dXdes_Foot.col(idx_prev) = dXn;
                        ddXdes_Foot.col(idx_prev) = ddXn;
                        StepPhaseDecision = true;
                        break;
                    } else {
                        if(idx_prev==0) {
                            t = t - t_fin;
                            n_swing++;
                            FILE_LOG(logSUCCESS) << "Swing Motion, Step : " << n_swing;
                            if(n_swing >= n_SwingMotion) {
                                CurrentStage = Swing_TERMINATE;
                                StepPhaseDecision = true;
                                FILE_LOG(logWARNING) << "Swing Motion, terminate...";
                                break;
                            } else {
                                CurrentSwingStage = StepBack;
                            }
                        }
                        t_prev = t_prev - t_fin;
                        n_swing_prev++;
                        if(n_swing_prev >= n_SwingMotion) {
                            StepPhaseDecision = true;
                            break;
                        } else {
                            CurrentSwingStage_prev = StepBack;
                        }
                    }
                case StepBack:
                    t_fin = t_SwingMotion;
                    if (t_prev < t_fin) {
                        Vector3d Xn, dXn, ddXn;
                        PolyTrajectory_Vector_3rd(t_fin, t_prev,
                                             X_stepback_ini,dX_stepback_ini,X_stepback_fin,dX_stepback_fin,
                                             Xn, dXn, ddXn);

                        Xdes_Foot.col(idx_prev) = Xn;
                        dXdes_Foot.col(idx_prev) = dXn;
                        ddXdes_Foot.col(idx_prev) = ddXn;
                        StepPhaseDecision = true;
                        break;
                    } else {
                        if(idx_prev==0) {
                            t = t - t_fin;
                            CurrentSwingStage = StepForward;
                        }
                        t_prev = t_prev - t_fin;
                        CurrentSwingStage_prev = StepForward;
                    }
                }
            }
            if(n_swing_prev < n_SwingMotion) {
                break;
            }
        }
        case Swing_TERMINATE:
            t_fin = 1.0*t_SwingMotion;
            if (t_prev < t_fin) {
                Vector3d _X, _dX, _ddX;
                PolyTrajectory_Vector_3rd(t_fin-t_prev,SYS_DT,
                                     Xdes_Foot.col(idx_prev), dXdes_Foot.col(idx_prev), Xdes_Ready, zv,
                                     _X, _dX, _ddX);

                Xdes_Foot.col(idx_prev) = _X;
                dXdes_Foot.col(idx_prev) = _dX;
                ddXdes_Foot.col(idx_prev) = _ddX;
                break;
            } else {
                if(idx_prev==0) {
                    t = t - t_fin;
                    CurrentStage = Swing_FINISH;
                    save_Flag = false;
                    save_File("SwingMotion");
                    FILE_LOG(logSUCCESS) << "Swing Motion Finish!!";
                }
                t_prev = t_prev - t_fin;
            }

        case Swing_FINISH:
            Xdes_Foot.col(idx_prev) = Xdes_Ready;
            dXdes_Foot.col(idx_prev) = zv;
            ddXdes_Foot.col(idx_prev) = zv;

            if(idx_prev==0) {
                t = 0.0;
                CurrentStage = Squat_PARAMETER_SETTING;
                CurrentSwingStage = StepForward;
                n_swing = 0;
                FlagForcedStop = false;
                return true;
            }

        }
    }
    t = t + SYS_DT;
    return false;
}


