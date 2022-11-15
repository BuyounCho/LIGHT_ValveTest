#ifndef _QUAD_JOINTSETMODEL_H
#define _QUAD_JOINTSETMODEL_H

#include "QUAD_BasicFunction.h"

#include "rbdl/rbdl.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

enum CtrlSpace_SET{
    JointSpace = 0,
    TaskSpace
};


// QUAD's Joint Class
class QUADJointSet
{
public:
    Model* Robot;

    bool Flag_Error;
    int n_dof;

    ///// state and reference ///////////////////////////////////////////////////////////////

    // Joint State
    VectorNd Qnow;
    VectorNd dQnow;
    VectorNd Tnow;

    VectorNd Qref;
    VectorNd dQref;
    VectorNd ddQref;
    VectorNd Tref;

    MatrixNd Qdes;
    MatrixNd dQdes;
    MatrixNd ddQdes;
    MatrixNd Tdes;

    double Qnow_arm;
    double dQnow_arm;
    double T_arm;

    double Qref_arm;
    double dQref_arm;
    double ddQref_arm;
    double Tref_arm;

    MatrixNd Qdes_arm;
    MatrixNd dQdes_arm;
    MatrixNd ddQdes_arm;
    MatrixNd Tdes_arm;

    // Actuator State
    MatrixNd Sdes;
    MatrixNd dSdes;
    MatrixNd Fdes;

    // Task Space State
    Vector3d Xnow_Foot;
    Vector3d dXnow_Foot;
    Vector3d Fnow_Foot;

    Vector3d Xref_Foot;
    Vector3d dXref_Foot;
    Vector3d ddXref_Foot;
    Vector3d Fref_Foot;

    MatrixNd Xdes_Foot;
    MatrixNd dXdes_Foot;
    MatrixNd ddXdes_Foot;
    MatrixNd Fdes_Foot;

    int CtrlSpace;
    bool Flag_InvKin;
    bool Flag_InvDyn;

    ////// leg model and parameter ////////////////////////////////////////////////////////

    Body b_BASE, b_ARM, b_RHP, b_RKN;
    Joint j_BASE, j_ARM, j_RHP, j_RKN;
    int n_BASE, n_ARM, n_RHP, n_RKN;

    double m_BASE, m_ARM, m_RHP, m_RKN;
    Vector3d c_BASE, c_ARM, c_RHP, c_RKN;
    Matrix3d I_BASE, I_ARM, I_RHP, I_RKN;

    Vector3d base2HAND;
    Vector3d RHP2RKN;
    Vector3d RKN2EE;

public:

    QUADJointSet()
    {
        n_dof = 2;
        Flag_Error = false;

        Robot = new Model();
        Robot->gravity = Vector3d(0,0,-9.81);

        base2HAND = Vector3d(0.400,0.0,0.0);
        RHP2RKN = Vector3d(0.350,0.0,0.0);
        RKN2EE = Vector3d(-0.300,0.0,-0.150);

        m_BASE = 10.0;
        m_ARM = 1.0;
        m_RHP = 3.0;
        m_RKN = 1.0;

        c_BASE = zv;
        c_ARM = 0.5*base2HAND;
        c_RHP = 0.5*RHP2RKN;
        c_RKN = 0.5*RKN2EE;

        I_BASE = 0.1*Matrix3d::Identity();
        I_ARM = 0.1*Matrix3d::Identity();
        I_RHP = 0.1*Matrix3d::Identity();
        I_RKN = 0.01*Matrix3d::Identity();

        b_BASE = Body(m_BASE,c_BASE,I_BASE); // mass (double) , mass center (vector), inertia (matrix)
//        b_ARM = Body(m_ARM,c_ARM,I_ARM); // mass (double) , mass center (vector), inertia (matrix)
        b_RHP = Body(m_RHP,c_RHP,I_RHP);
        b_RKN = Body(m_RKN,c_RKN,I_RKN);

        j_BASE = Joint(JointTypeFixed);
//        j_ARM = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum :
        j_RHP = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 0
        j_RKN = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 1

        n_BASE = Robot->AddBody(0,Xtrans(zv),j_BASE,b_BASE, "PEL");
//        n_ARM = Robot->AddBody(n_BASE,Xtrans(zv),j_ARM,b_ARM, "ARM");
        n_RHP = Robot->AddBody(n_BASE,Xtrans(zv),j_RHP,b_RHP, "RHP");
        n_RKN = Robot->AddBody(n_RHP,Xtrans(RHP2RKN),j_RKN,b_RKN, "RKN");

        // //////////////////////////////////////////////////////////
        // Robot states
        // //////////////////////////////////////////////////////////

        Qnow = VectorNd::Zero(n_dof);  // position
        dQnow = VectorNd::Zero(n_dof); // velocity
        Tnow = VectorNd::Zero(n_dof);  // torque

        Qref = VectorNd::Zero(n_dof);
        dQref = VectorNd::Zero(n_dof);
        ddQref = VectorNd::Zero(n_dof);
        Tref = VectorNd::Zero(n_dof);

        Qdes = MatrixNd::Zero(n_dof,MAX_PREVIEW+1);
        dQdes = MatrixNd::Zero(n_dof,MAX_PREVIEW+1);
        ddQdes = MatrixNd::Zero(n_dof,MAX_PREVIEW+1);
        Tdes = MatrixNd::Zero(n_dof,MAX_PREVIEW+1);

        Qnow_arm = 0.0;
        dQnow_arm = 0.0;
        T_arm = 0.0;

        Qref_arm = 0.0;
        dQref_arm = 0.0;
        ddQref_arm = 0.0;
        Tref_arm = 0.0;

        Qdes_arm = MatrixNd::Zero(1,MAX_PREVIEW+1);
        dQdes_arm = MatrixNd::Zero(1,MAX_PREVIEW+1);
        ddQdes_arm = MatrixNd::Zero(1,MAX_PREVIEW+1);
        Tdes_arm = MatrixNd::Zero(1,MAX_PREVIEW+1);

        Sdes = MatrixNd::Zero(n_dof,MAX_PREVIEW+1);
        dSdes = MatrixNd::Zero(n_dof,MAX_PREVIEW+1);
        Fdes = MatrixNd::Zero(n_dof,MAX_PREVIEW+1);

        Xnow_Foot = Vector3d::Zero();
        dXnow_Foot = Vector3d::Zero();
        Fnow_Foot = Vector3d::Zero();

        Xref_Foot = Vector3d::Zero();
        dXref_Foot = Vector3d::Zero();
        ddXref_Foot = Vector3d::Zero();
        Fref_Foot = Vector3d::Zero();

        Xdes_Foot = MatrixNd::Zero(3,MAX_PREVIEW+1);
        dXdes_Foot = MatrixNd::Zero(3,MAX_PREVIEW+1);
        ddXdes_Foot = MatrixNd::Zero(3,MAX_PREVIEW+1);
        Fdes_Foot = MatrixNd::Zero(3,MAX_PREVIEW+1);

        CtrlSpace = JointSpace;
        Flag_InvKin = false;
        Flag_InvDyn = false;

    }

    ///// state update
    void UpdateSensorData();
    void UpdateStates();
    void UpdateReferenceStates();

    void SetReference_Joint();
    void SetReference_Actuator();

    ///// Kinematics and Dynamics
    void InverseKinematics();
    void InverseKinematics_PumpReference();
    void InverseDynamics();

    ///// Joint State >> Actuator(Cylinder or Rotary) State ////////////////////////////
    ///// For reference generation
    void Joint2Rotary_HipPitch(double theta, double dtheta, double T,
                                   double &S, double &dS, double &F);
    void Joint2Rotary_Knee(double theta, double dtheta, double T,
                              double &S, double &dS, double &F);


    ///// Arm Task Functions //////////////////////////////////////////////////////////////
public:
    enum ARM_OPERATION_SET
    {
        ARM_OPERATION_NoAct = 0,
        ARM_OPERATION_MoveJoint,
        ARM_OPERATION_SineWave,
    };

private:
    int CurrentArmOpeartion = ARM_OPERATION_NoAct;
    bool CurrentArmOperation_TerminateFlag = false;

public:
    void SetArmOpeartion(int _OP) { CurrentArmOpeartion = _OP; }
    int CheckArmOpeartion(void) { return CurrentArmOpeartion; }
    void ResetArmOpeartion() { CurrentArmOpeartion = ARM_OPERATION_NoAct; }

    void TerminateArmOpeartion() { CurrentArmOperation_TerminateFlag = true; }
    bool CheckArmTerminateFlag() { return CurrentArmOperation_TerminateFlag; }
    void ResetArmTerminateFlag() { CurrentArmOperation_TerminateFlag = false; }

    // TestJoints ------------------------------------------
private:
    enum ArmTask_STAGESET{
        ArmTask_PARAMETER_SETTING = 0,
        ArmTask_PROCESS,
        ArmTask_FINISH,
    };
    int         n_ArmTask;
    double      t_ArmTask;
    double      q_ArmTask;
    double      m_ArmTask;
    double      l_ArmTask;
public:
    void SetParameter_ArmTask(int _N, double _T, double _Q, double _M, double _L) {
        n_ArmTask = _N;
        t_ArmTask = _T;
        q_ArmTask = _Q;
        m_ArmTask = _M;
        l_ArmTask = _L;
    }
    bool ArmTask_MoveJoint();
    bool ArmTask_SineWave();

    ///// Quad's Leg Task Functions //////////////////////////////////////////////////////////////

    // Quad's Leg Operation Set -------------------------------------------
public:
    enum OPERATION_SET
    {
        OPERATION_NoAct = 0,
        OPERATION_JointSpaceMove,

        OPERATION_TestJoints_MoveJoint,
        OPERATION_TestJoints_SineWave,

        OPERATION_Squat,
        OPERATION_Swing,
    };

private:
    int CurrentOpeartion = OPERATION_NoAct;
    bool CurrentOperation_TerminateFlag = false;

public:
    void SetOpeartion(int _OP) { CurrentOpeartion = _OP; }
    int CheckOpeartion(void) { return CurrentOpeartion; }
    void ResetOpeartion() { CurrentOpeartion = OPERATION_NoAct; }

    void TerminateOpeartion() { CurrentOperation_TerminateFlag = true; }
    bool CheckTerminateFlag() { return CurrentOperation_TerminateFlag; }
    void ResetTerminateFlag() { CurrentOperation_TerminateFlag = false; }

    // JointSpaceMove ------------------------------------------
public:
    enum JointSpaceMove_STAGESET{
        JointSpaceMove_PARAMETER_SETTING = 0,
        JointSpaceMove_PROCESS,
        JointSpaceMove_FINISH,
    };
    double      t_JointSpaceMove;
    VectorNd    Q_JointSpaceMove;
public:
    void SetParameter_JointSpaceMove(double _T, VectorNd _Qdes) {
        t_JointSpaceMove = _T;
        Q_JointSpaceMove = _Qdes;
    }
    bool JointSpaceMove();

    // TestJoints ------------------------------------------
private:
    enum TestJoints_STAGESET{
        TestJoints_PARAMETER_SETTING = 0,
        TestJoints_PROCESS,
        TestJoints_FINISH,
    };
    int         idx_TestJoints;
    int         n_TestJoints;
    double      t_TestJoints;
    double      q_TestJoints;
public:
    void SetParameter_TestJoints(int _I, int _N, double _T, double _Q) {
        idx_TestJoints = _I;
        n_TestJoints = _N;
        t_TestJoints = _T;
        q_TestJoints = _Q;
    }
    bool TestJoints_MoveJoint();
    bool TestJoints_SineWave();

    // Squat ------------------------------------------
private:
    enum Squat_STAGESET{
        Squat_PARAMETER_SETTING = 0,
        Squat_READY,
        Squat_PROCESS,
        Squat_TERMINATE,
        Squat_FINISH,
    };
    int         n_SquatMotion;
    double      t_SquatMotion;
    Vector3d    X_SquatMotion;
public:
    void SetParameter_SquatMotion(double _T, Vector3d _X, int _N) {
        n_SquatMotion = _N;
        t_SquatMotion = _T;
        X_SquatMotion = _X;
    }
    bool SquatMotion();

    // Swing ------------------------------------------
private:
    enum Swing_STAGESET{
        Swing_PARAMETER_SETTING = 0,
        Swing_READY,
        Swing_INIT,
        Swing_SWING,
        Swing_TERMINATE,
        Swing_FINISH,
    };
    enum Swing_PHASESET{
        StepForward = 0,
        StepBack,
    };
    int         n_SwingMotion;
    double      t_SwingMotion;
    double      x_SwingMotion;
    double      z_SwingMotion;
public:
    void SetParameter_SwingMotion(double _T, double _X, double _Z, int _N)
    {
        n_SwingMotion = _N;
        t_SwingMotion = _T;
        x_SwingMotion = _X;
        z_SwingMotion = _Z;
    }
    bool SwingMotion();

};

#endif // _QUAD_JOINTSETMODEL_H
