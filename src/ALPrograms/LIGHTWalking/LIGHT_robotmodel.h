#ifndef LIGHT_HOPPING_ROBOTMODEL
#define LIGHT_HOPPING_ROBOTMODEL

#include "rbdl/rbdl.h"
#include "LIGHT_var_and_func.h"
#include "LIGHT_info.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

extern void PrintHere(int n = 0);

class LIGHTWholeBody
{
private:
    // All generalized coordinates of whole body [base pos & ori(6)+ joint angle(n)]

public:
    Model* Robot;
    Model* Robot_FixedBase;
    INFO_LIGHT* LIGHT_Info;

    enum REFFRAME_ID {
        REFFRAME_RF     = -1,
        REFFRAME_PEL    =  0,
        REFFRAME_LF     =  1,
        REFFRAME_GLOBAL =  99,
    };
    int Cur_RefFrame = REFFRAME_PEL;
    void ChangeRefFrame(int _RefFrame) {
        if(Cur_RefFrame != REFFRAME_LF && _RefFrame == REFFRAME_LF) {
            Cur_RefFrame = REFFRAME_LF;
            Xnow_Pel = -Xnow_Pel2LF;
            dXnow_Pel = -dXnow_Pel2LF;
            Qnow.segment(PELVIS_POS_QNUMSTART,3) = Xnow_Pel;
            dQnow.segment(PELVIS_POS_QNUMSTART,3) = dXnow_Pel;
        } else if (Cur_RefFrame != REFFRAME_RF && _RefFrame == REFFRAME_RF) {
            Cur_RefFrame = REFFRAME_RF;
            Xnow_Pel = -Xnow_Pel2RF;
            dXnow_Pel = -dXnow_Pel2RF;
            Qnow.segment(PELVIS_POS_QNUMSTART,3) = Xnow_Pel;
            dQnow.segment(PELVIS_POS_QNUMSTART,3) = dXnow_Pel;
        } else if (Cur_RefFrame != REFFRAME_PEL && _RefFrame == REFFRAME_PEL) {
            Cur_RefFrame = REFFRAME_PEL;
            Xnow_Pel = zv;
            dXnow_Pel = zv;
            Qnow.segment(PELVIS_POS_QNUMSTART,3) = Xnow_Pel;
            dQnow.segment(PELVIS_POS_QNUMSTART,3) = dXnow_Pel;
        }
    }

    enum REFSTATE_ID {
        REFSTATE_RDSP      =  0,
        REFSTATE_LDSP      =  1,
        REFSTATE_RSSP      =  2,
        REFSTATE_LSSP      =  3,
        REFSTATE_FLOAT     =  4,
        REFSTATE_NOACT     =  99
    };
    int Cur_RefState = REFSTATE_FLOAT;
    int CheckCurrentRefState() {return Cur_RefState;}
    void CurRefStateIs_RDSP() {
        int Prev_RefState = Cur_RefState;
        Cur_RefState = REFSTATE_RDSP;
        ChangeRefFrame(REFFRAME_RF);
        ChangeRefFrame_ReferenceStates(Prev_RefState, Cur_RefState);
        ContactOn_RF();
        ContactOn_LF();
    }
    void CurRefStateIs_LDSP() {
        int Prev_RefState = Cur_RefState;
        Cur_RefState = REFSTATE_LDSP;
        ChangeRefFrame(REFFRAME_LF);
        ChangeRefFrame_ReferenceStates(Prev_RefState, Cur_RefState);
        ContactOn_RF();
        ContactOn_LF();
    }
    void CurRefStateIs_RSSP() {
        int Prev_RefState = Cur_RefState;
        Cur_RefState = REFSTATE_RSSP;
        ChangeRefFrame(REFFRAME_RF);
        ChangeRefFrame_ReferenceStates(Prev_RefState, Cur_RefState);
        ContactOn_RF();
        ContactOff_LF();
    }
    void CurRefStateIs_LSSP() {
        int Prev_RefState = Cur_RefState;
        Cur_RefState = REFSTATE_LSSP;
        ChangeRefFrame(REFFRAME_LF);
        ChangeRefFrame_ReferenceStates(Prev_RefState, Cur_RefState);
        ContactOff_RF();
        ContactOn_LF();
    }
    void CurRefStateIs_FLOAT() {
        int Prev_RefState = Cur_RefState;
        Cur_RefState = REFSTATE_FLOAT;
        ChangeRefFrame(REFFRAME_PEL);
        ChangeRefFrame_ReferenceStates(Prev_RefState, Cur_RefState);
        ContactOff_RF();
        ContactOff_LF();
    }
    void CurRefStateIs_NOACT() {
        int Prev_RefState = Cur_RefState;
        Cur_RefState = REFSTATE_NOACT;
        ChangeRefFrame_ReferenceStates(Prev_RefState, Cur_RefState);
        ContactOff_RF();
        ContactOff_LF();
    }
    bool IsCurRefState_Float() {
        if(Cur_RefState == REFSTATE_FLOAT||Cur_RefState == REFSTATE_NOACT) {return true;}
        else {return false;}
    }
    bool IsCurRefState_DSP() {
        if(Cur_RefState == REFSTATE_RDSP||Cur_RefState == REFSTATE_LDSP) {return true;}
        else {return false;}
    }
    bool IsCurRefState_SSP() {
        if(Cur_RefState == REFSTATE_RSSP||Cur_RefState == REFSTATE_LSSP) {return true;}
        else {return false;}
    }
    bool IsCurRefState_RSP() {
        if(Cur_RefState == REFSTATE_RDSP||Cur_RefState == REFSTATE_RSSP) {return true;}
        else {return false;}
    }
    bool IsCurRefState_LSP() {
        if(Cur_RefState == REFSTATE_LDSP||Cur_RefState == REFSTATE_LSSP) {return true;}
        else {return false;}
    }
    bool IsCurRefState_RDSP() {
        if(Cur_RefState == REFSTATE_RDSP) {return true;}
        else {return false;}
    }
    bool IsCurRefState_LDSP() {
        if(Cur_RefState == REFSTATE_LDSP) {return true;}
        else {return false;}
    }
    bool IsCurRefState_RSSP() {
        if(Cur_RefState == REFSTATE_RSSP) {return true;}
        else {return false;}
    }
    bool IsCurRefState_LSSP() {
        if(Cur_RefState == REFSTATE_LSSP) {return true;}
        else {return false;}
    }
    Matrix3d R_Transit;
    Vector3d r_Transit;

    bool Flag_PelvisPositionEstimation = true;
    bool Flag_Error = false;
    bool Flag_StateCheck = true;
    bool Flag_UnnormalStates = false;
    bool Flag_CoMStateObserve = false;
    bool Flag_KinematicCompensation = false;

    void CoMStateObserveStart() {Flag_CoMStateObserve = true;}
    void CoMStateObserveFinish() {Flag_CoMStateObserve = false;}
    void KinematicCompensationStart() {Flag_KinematicCompensation = true;}
    void KinematicCompensationFinish() {Flag_KinematicCompensation = false;}

    int n_dof = LIGHT_DOF;
    int n_act = LIGHT_ACT_DOF;
    int n_con = 3*2; // Contact point * 3D

    ///// state and reference

    // ** Important! ** /////////////////////////////////////////
    // Robot has 6+n(#joint) DoF.
    // 6 DoF comes from pelvis position X,Y,Z(Q[0],Q[1],Q[2])  (Don't confuse the order of position and orientation)
    // and orientation written in quaternion X,Y,Z(Q[3],Q[4],Q[5]).
    // The last W component of quaternion is added as last element of Q(6+n) << (6+n+1)th component
    // [ Reference : https://bitbucket.org/rbdl/rbdl/issues/44/no-way-to-set-base-link-state-for-floating ]

    // Joint Angle
    // Q(0~2) : Translation X, Y, and Z
    // Q(3~5) : Quaternion x,y,z element
    // Q(end) : Quaternion w element
    VectorNd Qnow;
    VectorNd dQnow;
    VectorNd Qnow_ZeroPelvis;
    VectorNd dQnow_ZeroPelvis;
    VectorNd Tnow;
    Vector3d Textnow_RF_byFT; // Estimated external torque/force by FT Sensor
    Vector3d Fextnow_RF_byFT;
    Vector3d Textnow_LF_byFT;
    Vector3d Fextnow_LF_byFT;
    Vector3d Textnow_RF_byJTS; // Estimated external torque/force by Joint Torque Sensor
    Vector3d Fextnow_RF_byJTS;
    Vector3d Textnow_LF_byJTS;
    Vector3d Fextnow_LF_byJTS;

    // Joint-level Desired Position
    VectorNd Qdes;
    VectorNd dQdes;
    VectorNd ddQdes;
    VectorNd Tdes;
    Vector3d Textdes_RF;
    Vector3d Fextdes_RF;
    Vector3d Textdes_LF;
    Vector3d Fextdes_LF;

    // Joint-level Reference
    VectorNd Qref;
    VectorNd dQref;
    VectorNd ddQref;
    VectorNd Qref_Comp;
    VectorNd dQref_Comp;
    VectorNd Qref_ZeroPelvis;
    VectorNd dQref_ZeroPelvis;
    VectorNd ddQref_ZeroPelvis;
    VectorNd Tref;
    VectorNd Tref_Comp; // Torque Compensation
    Vector3d Textref_RF;
    Vector3d Fextref_RF;
    Vector3d Textref_LF;
    Vector3d Fextref_LF;

    // Actuator-level Reference
    VectorNd Sref;
    VectorNd dSref;
    VectorNd ddSref;
    VectorNd Fref;

    VectorNd Qmax;
    VectorNd dQmax;
    VectorNd ddQmax;
    VectorNd Qmin;
    VectorNd dQmin;
    VectorNd ddQmin;

    VectorNd dQCon_ub;
    VectorNd dQCon_lb;
    VectorNd ddQCon_ub;
    VectorNd ddQCon_lb;

    ///////////////////////////////////////////////////////////////

    // Position and Orientation (work space)

    // World >> Pelvis
    Matrix3d Rnow_Pel; // (World > Pelvis) Orientation (othogonal 3*3 matrix)
    Vector3d Wnow_Pel; // (World > Pelvis) Angular Velocity (vector+angle)
    Vector3d Xnow_Pel;  // (World > Pelvis) Linear Position
    Vector3d dXnow_Pel; // (World > Pelvis) Linear Velocity

    // World >> Pelvis
    Vector3d Xnow_CoM;  // (World > CoM) Linear Position
    Vector3d dXnow_CoM; // (World > CoM) Linear Velocity

    // World >> RightFoot
    Matrix3d Rnow_RF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wnow_RF; // Orientation (vector+angle)
    Vector3d Xnow_RF;
    Vector3d dXnow_RF;

    // World >> LeftFoot
    Matrix3d Rnow_LF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wnow_LF; // Orientation (vector+angle)
    Vector3d Xnow_LF;
    Vector3d dXnow_LF;

    // Pelvis >> Center of Mass in Torso
    Vector3d Xnow_Pel2Torso;
    Vector3d dXnow_Pel2Torso;

    // Pelvis >> RightFoot
    Matrix3d Rnow_Pel2RF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wnow_Pel2RF; // Orientation (vector+angle)
    Vector3d Xnow_Pel2RF;
    Vector3d dXnow_Pel2RF;

    // Pelvis >> LeftFoot
    Matrix3d Rnow_Pel2LF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wnow_Pel2LF; // Orientation (vector+angle)
    Vector3d Xnow_Pel2LF;
    Vector3d dXnow_Pel2LF;

    // RightFoot >> LeftFoot
    Vector3d Xnow_RF2LF;
    Vector3d dXnow_RF2LF;

    // RightFoot >> CoM
    Vector3d Xnow_RF2CoM;
    Vector3d dXnow_RF2CoM;

    // LeftFoot >> RightFoot
    Vector3d Xnow_LF2RF;
    Vector3d dXnow_LF2RF;

    // LeftFoot >> CoM
    Vector3d Xnow_LF2CoM;
    Vector3d dXnow_LF2CoM;

    ///////////////////////////////////////////////////////////////

    // World >> Pelvis
    Matrix3d Rdes_Pel; // (World > Pelvis) Orientation (othogonal 3*3 matrix)
    Vector3d Wdes_Pel; // (World > Pelvis) Angular Velocity (vector+angle)
    Vector3d dWdes_Pel; // (World > Pelvis) Angular Acceleration (vector+angle)
    Vector3d Xdes_Pel;  // (World > Pelvis) Linear Position
    Vector3d dXdes_Pel; // (World > Pelvis) Linear Velocity
    Vector3d ddXdes_Pel; // (World > Pelvis) Linear Acceleration

    // World >> CoM
    Vector3d Xdes_CoM;   // (World > CoM) Linear Position
    Vector3d dXdes_CoM;  // (World > CoM) Linear Velocity
    Vector3d ddXdes_CoM; // (World > CoM) Linear Acceleration (input)

    // World >> RightFoot
    Matrix3d Rdes_RF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wdes_RF; // Orientation (vector+angle)
    Vector3d dWdes_RF; // Orientation (vector+angle)
    Vector3d Xdes_RF;
    Vector3d dXdes_RF;
    Vector3d ddXdes_RF;

    // World >> LeftFoot
    Matrix3d Rdes_LF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wdes_LF; // Orientation (vector+angle)
    Vector3d dWdes_LF; // Orientation (vector+angle)
    Vector3d Xdes_LF;
    Vector3d dXdes_LF;
    Vector3d ddXdes_LF;

    // Pelvis >> RightFoot
    Matrix3d Rdes_Pel2RF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wdes_Pel2RF; // Orientation (vector+angle)
    Vector3d dWdes_Pel2RF; // Orientation (vector+angle)
    Vector3d Xdes_Pel2RF;
    Vector3d dXdes_Pel2RF;
    Vector3d ddXdes_Pel2RF;

    // Pelvis >> LeftFoot
    Matrix3d Rdes_Pel2LF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wdes_Pel2LF; // Orientation (vector+angle)
    Vector3d dWdes_Pel2LF; // Orientation (vector+angle)
    Vector3d Xdes_Pel2LF;
    Vector3d dXdes_Pel2LF;
    Vector3d ddXdes_Pel2LF;

    // RightFoot >> LeftFoot
    Vector3d Xdes_RF2LF;
    Vector3d dXdes_RF2LF;
    Vector3d ddXdes_RF2LF;

    // RightFoot >> CoM
    Vector3d Xdes_RF2CoM;
    Vector3d dXdes_RF2CoM;
    Vector3d ddXdes_RF2CoM;

    // LeftFoot >> RightFoot
    Vector3d Xdes_LF2RF;
    Vector3d dXdes_LF2RF;
    Vector3d ddXdes_LF2RF;

    // LeftFoot >> CoM
    Vector3d Xdes_LF2CoM;
    Vector3d dXdes_LF2CoM;
    Vector3d ddXdes_LF2CoM;

    ///////////////////////////////////////////////////////////////

    // World >> Pelvis
    Matrix3d Rref_Pel; // (World > Pelvis) Orientation (othogonal 3*3 matrix)
    Vector3d Wref_Pel; // (World > Pelvis) Angular Velocity (vector+angle)
    Vector3d dWref_Pel; // (World > Pelvis) Angular Acceleration (vector+angle)
    Vector3d Xref_Pel;  // (World > Pelvis) Linear Position
    Vector3d dXref_Pel; // (World > Pelvis) Linear Velocity
    Vector3d ddXref_Pel; // (World > Pelvis) Linear Acceleration

    // World >> Pelvis
    Vector3d Xref_CoM;   // (World > CoM) Linear Position
    Vector3d dXref_CoM;  // (World > CoM) Linear Velocity
    Vector3d ddXref_CoM; // (World > CoM) Linear Acceleration (input)

    // World >> RightFoot
    Matrix3d Rref_RF; // Orientation (othogonal 3*3 matrix)
    Matrix3d Rref_RF_last; // Orientation (othogonal 3*3 matrix)
    Vector3d Wref_RF; // Orientation (vector+angle)
    Vector3d dWref_RF; // Orientation (vector+angle)
    Vector3d Xref_RF;
    Vector3d dXref_RF;
    Vector3d ddXref_RF;

    // World >> LeftFoot
    Matrix3d Rref_LF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wref_LF; // Orientation (vector+angle)
    Vector3d dWref_LF; // Orientation (vector+angle)
    Vector3d Xref_LF;
    Vector3d dXref_LF;
    Vector3d ddXref_LF;

    // Pelvis >> RightFoot
    Matrix3d Rref_Pel2RF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wref_Pel2RF; // Orientation (vector+angle)
    Vector3d dWref_Pel2RF; // Orientation (vector+angle)
    Vector3d Xref_Pel2RF;
    Vector3d dXref_Pel2RF;
    Vector3d ddXref_Pel2RF;

    // Pelvis >> LeftFoot
    Matrix3d Rref_Pel2LF; // Orientation (othogonal 3*3 matrix)
    Vector3d Wref_Pel2LF; // Orientation (vector+angle)
    Vector3d dWref_Pel2LF; // Orientation (vector+angle)
    Vector3d Xref_Pel2LF;
    Vector3d dXref_Pel2LF;
    Vector3d ddXref_Pel2LF;

    // RightFoot >> LeftFoot
    Vector3d Xref_RF2LF;
    Vector3d dXref_RF2LF;
    Vector3d ddXref_RF2LF;

    // RightFoot >> CoM
    Vector3d Xref_RF2CoM;
    Vector3d dXref_RF2CoM;
    Vector3d ddXref_RF2CoM;

    // LeftFoot >> RightFoot
    Vector3d Xref_LF2RF;
    Vector3d dXref_LF2RF;
    Vector3d ddXref_LF2RF;

    // LeftFoot >> CoM
    Vector3d Xref_LF2CoM;
    Vector3d dXref_LF2CoM;
    Vector3d ddXref_LF2CoM;

    bool CoMzIsPelz;

    ///////////////////////////////////////////////////////////////

    Vector3d K_RF_ori;
    Vector3d K_RF_pos;
    Vector3d K_LF_ori;
    Vector3d K_LF_pos;
    Vector3d D_RF_ori;
    Vector3d D_RF_pos;
    Vector3d D_LF_ori;
    Vector3d D_LF_pos;

    Vector3d K_RF_ori_OnContact;
    Vector3d K_RF_pos_OnContact;
    Vector3d K_LF_ori_OnContact;
    Vector3d K_LF_pos_OnContact;
    Vector3d D_RF_ori_OnContact;
    Vector3d D_RF_pos_OnContact;
    Vector3d D_LF_ori_OnContact;
    Vector3d D_LF_pos_OnContact;

    Vector3d K_RF_ori_OffContact;
    Vector3d K_RF_pos_OffContact;
    Vector3d K_LF_ori_OffContact;
    Vector3d K_LF_pos_OffContact;
    Vector3d D_RF_ori_OffContact;
    Vector3d D_RF_pos_OffContact;
    Vector3d D_LF_ori_OffContact;
    Vector3d D_LF_pos_OffContact;

    double Kp_AnkleTorqueComp;
    double Ki_AnkleTorqueComp;

    //////////////////////////////////////////////////////////////

    Vector3d Znow_DSP_RF;
    Vector3d Znow_DSP_LF;
    Vector3d Znow_SSP_RF;
    Vector3d Znow_SSP_LF;

private:
    bool ContactState_RF; // false : Uncontacted, true : Contacted
    bool ContactState_LF; // false : Uncontacted, true : Contacted
public:
    bool IsContacted_RF() {return ContactState_RF;}
    bool IsContacted_LF() {return ContactState_LF;}
    void ContactOn_RF() {ContactState_RF = true;}
    void ContactOff_RF() {ContactState_RF = false;}
    void ContactOn_LF() {ContactState_LF = true;}
    void ContactOff_LF() {ContactState_LF = false;}

    Quaternion Quat_Ini_RF;
    Quaternion Quat_Ini_LF;

    Vector3d CPnow;
    Vector3d CPdes;
    Vector3d dCPdes;
    Vector3d CPref;
    Vector3d dCPref;
    Vector3d CPlast;

    Vector3d SavingVector[50];

    ////// robot model and parameter /////////////////////////

    // body mass
    double m_robot;

    // Link Parameters
    Body b_link[LIGHT_ACT_DOF+1];
    Joint j_link[LIGHT_ACT_DOF+1];
    int n_link[LIGHT_ACT_DOF+1];
    double m_link[LIGHT_ACT_DOF+1];
    Vector3d c_link[LIGHT_ACT_DOF+1];
    Matrix3d I_link[LIGHT_ACT_DOF+1];

    Vector3d pel2torso;

    Vector3d pel2rhr;
    Vector3d rhr2rhy;
    Vector3d rhy2rhp;
    Vector3d rhp2rkn;
    Vector3d rkn2rap;
    Vector3d rap2rar;
    Vector3d rar2EE;
    Matrix3d R_rar2EE;

    Vector3d pel2lhr;
    Vector3d lhr2lhy;
    Vector3d lhy2lhp;
    Vector3d lhp2lkn;
    Vector3d lkn2lap;
    Vector3d lap2lar;
    Vector3d lar2EE;
    Matrix3d R_lar2EE;

    // Fixed base(Pelvis) information for test
    Joint j_pel_F;
    int n_link_F[LIGHT_ACT_DOF+1];

public:

    LIGHTWholeBody()
    {
        LIGHT_Info = new INFO_LIGHT();
        Robot = new Model();
        Robot_FixedBase = new Model();

        CoMzIsPelz = true; // true : CoM Height is considered as Pelvis Height
                            // false : CoM Height is not related to Pelvis Height

        // //////////////////////////////////////////////////////////
        // Robot model setting by RBDL
        //    >> Updated to LIGHT_ver2.0 (210430)
        // //////////////////////////////////////////////////////////
        Robot->gravity = Vector3d(0,0,-9.81);
        Robot_FixedBase->gravity = Vector3d(0,0,-9.81);

        /////////////////////////////////////////////

        //right leg
        m_link[RHR] = LIGHT_Info->m_rhr;
        m_link[RHY] = LIGHT_Info->m_rhy;
        m_link[RHP] = LIGHT_Info->m_rhp;
        m_link[RKN] = LIGHT_Info->m_rkn;
        m_link[RAP] = LIGHT_Info->m_rap;
        m_link[RAR] = LIGHT_Info->m_rar;

        c_link[RHR] = LIGHT_Info->c_rhr;
        c_link[RHY] = LIGHT_Info->c_rhy;
        c_link[RHP] = LIGHT_Info->c_rhp;
        c_link[RKN] = LIGHT_Info->c_rkn;
        c_link[RAP] = LIGHT_Info->c_rap;
        c_link[RAR] = LIGHT_Info->c_rar;

        I_link[RHR] = LIGHT_Info->I_rhr;
        I_link[RHY] = LIGHT_Info->I_rhy;
        I_link[RHP] = LIGHT_Info->I_rhp;
        I_link[RKN] = LIGHT_Info->I_rkn;
        I_link[RAP] = LIGHT_Info->I_rap;
        I_link[RAR] = LIGHT_Info->I_rar;

        b_link[RHR] = Body(m_link[RHR],c_link[RHR],I_link[RHR]); //mass (double) , mass center (vector), inertia (matrix)
        b_link[RHY] = Body(m_link[RHY],c_link[RHY],I_link[RHY]);
        b_link[RHP] = Body(m_link[RHP],c_link[RHP],I_link[RHP]);
        b_link[RKN] = Body(m_link[RKN],c_link[RKN],I_link[RKN]);
        b_link[RAP] = Body(m_link[RAP],c_link[RAP],I_link[RAP]);
        b_link[RAR] = Body(m_link[RAR],c_link[RAR],I_link[RAR]);

        j_link[RHR] = Joint(JointTypeRevolute, Vector3d (1.0, 0.0, 0.0)); // Qnum : 6  (Qsize : 8)
        j_link[RHY] = Joint(JointTypeRevolute, Vector3d (0.0, 0.0, 1.0)); // Qnum : 7  (Qsize : 9)
        j_link[RHP] = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 8  (Qsize : 10)
        j_link[RKN] = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 9  (Qsize : 11)
        j_link[RAP] = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 10 (Qsize : 12)
        j_link[RAR] = Joint(JointTypeRevolute, Vector3d (0.6487, 0.0, 0.7611)); // Qnum : 11 (Qsize : 13)

        ////////////////////////////////////////////

        //left leg
        m_link[LHR] = LIGHT_Info->m_lhr;
        m_link[LHY] = LIGHT_Info->m_lhy;
        m_link[LHP] = LIGHT_Info->m_lhp;
        m_link[LKN] = LIGHT_Info->m_lkn;
        m_link[LAP] = LIGHT_Info->m_lap;
        m_link[LAR] = LIGHT_Info->m_lar;

        c_link[LHR] = LIGHT_Info->c_lhr;
        c_link[LHY] = LIGHT_Info->c_lhy;
        c_link[LHP] = LIGHT_Info->c_lhp;
        c_link[LKN] = LIGHT_Info->c_lkn;
        c_link[LAP] = LIGHT_Info->c_lap;
        c_link[LAR] = LIGHT_Info->c_lar;

        I_link[LHR] = LIGHT_Info->I_lhr;
        I_link[LHY] = LIGHT_Info->I_lhy;
        I_link[LHP] = LIGHT_Info->I_lhp;
        I_link[LKN] = LIGHT_Info->I_lkn;
        I_link[LAP] = LIGHT_Info->I_lap;
        I_link[LAR] = LIGHT_Info->I_lar;

        //mass (double) , mass center (vector), inertia (matrix)
        b_link[LHR] = Body(m_link[LHR],c_link[LHR],I_link[LHR]);
        b_link[LHY] = Body(m_link[LHY],c_link[LHY],I_link[LHY]);
        b_link[LHP] = Body(m_link[LHP],c_link[LHP],I_link[LHP]);
        b_link[LKN] = Body(m_link[LKN],c_link[LKN],I_link[LKN]);
        b_link[LAP] = Body(m_link[LAP],c_link[LAP],I_link[LAP]);
        b_link[LAR] = Body(m_link[LAR],c_link[LAR],I_link[LAR]);

        j_link[LHR] = Joint(JointTypeRevolute, Vector3d (1.0, 0.0, 0.0)); // Qnum : 12 (Qsize : 14)
        j_link[LHY] = Joint(JointTypeRevolute, Vector3d (0.0, 0.0, 1.0)); // Qnum : 13 (Qsize : 15)
        j_link[LHP] = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 14 (Qsize : 16)
        j_link[LKN] = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 15 (Qsize : 17)
        j_link[LAP] = Joint(JointTypeRevolute, Vector3d (0.0, 1.0, 0.0)); // Qnum : 16 (Qsize : 18)
        j_link[LAR] = Joint(JointTypeRevolute, Vector3d (0.6487, 0.0, 0.7611)); // Qnum : 17 (Qsize : 19)

        /////////////////////////////////////////////

        // Waist
        m_link[WST] = LIGHT_Info->m_torso;
        c_link[WST] = LIGHT_Info->c_torso;
        I_link[WST] = LIGHT_Info->I_torso;
        b_link[WST] = Body(m_link[WST],c_link[WST],I_link[WST]);
        j_link[WST] = Joint(JointTypeRevoluteZ);  // Qnum : 18 (Qsize : 20)

        // Pelvis (Base Link)
        m_link[PEL] = LIGHT_Info->m_pel;
        c_link[PEL] = LIGHT_Info->c_pel;
        I_link[PEL] = LIGHT_Info->I_pel;
        b_link[PEL] = Body(m_link[PEL],c_link[PEL],I_link[PEL]); // mass (double) , mass center (vector), inertia (matrix)
        j_link[PEL] = Joint(JointTypeFloatingBase); // Qnum : 0~5, and last element (Qsize : 7)

        m_robot = 0.0;
        for(int i=0;i<LIGHT_ACT_DOF+1;i++) {
            m_robot += m_link[i];
        }

        ////////// link and joint setting ///////////////////////////////////

        //pel to RF
        pel2rhr = LIGHT_Info->offset_pel2rhr;
        rhr2rhy = LIGHT_Info->offset_rhr2rhy;
        rhy2rhp = LIGHT_Info->offset_rhy2rhp;
        rhp2rkn = LIGHT_Info->offset_rhp2rkn;
        rkn2rap = LIGHT_Info->offset_rkn2rap;
        rap2rar = LIGHT_Info->offset_rap2rar;
        rar2EE = LIGHT_Info->offset_rar2EE;
        R_rar2EE = LIGHT_Info->offset_R_rar2EE;

        //pel to LF
        pel2lhr = LIGHT_Info->offset_pel2lhr;
        lhr2lhy = LIGHT_Info->offset_lhr2lhy;
        lhy2lhp = LIGHT_Info->offset_lhy2lhp;
        lhp2lkn = LIGHT_Info->offset_lhp2lkn;
        lkn2lap = LIGHT_Info->offset_lkn2lap;
        lap2lar = LIGHT_Info->offset_lap2lar;
        lar2EE = LIGHT_Info->offset_lar2EE;
        R_lar2EE = LIGHT_Info->offset_R_lar2EE;

        pel2torso = LIGHT_Info->offset_pel2torso;

        // (Important!!) Generalized Coordinate Number
        // 0~5 : Pelvis coordinate (Position and Orientation(x,y,z))
        // 6~11 : Right leg
        // 12~17 : Left leg
        // 18 : Waist
        // 19 : Pelvis coordinate (Orientation(w))
        // pel (Robot Frame)
        n_link[PEL] = Robot->AddBody(0,Xtrans(zv),j_link[PEL],b_link[PEL],"PEL");

        // pel to right leg
        n_link[RHR] = Robot->AddBody(n_link[PEL],Xtrans(pel2rhr),j_link[RHR],b_link[RHR],"RHR");
        n_link[RHY] = Robot->AddBody(n_link[RHR],Xtrans(rhr2rhy),j_link[RHY],b_link[RHY],"RHY");
        n_link[RHP] = Robot->AddBody(n_link[RHY],Xtrans(rhy2rhp),j_link[RHP],b_link[RHP],"RHP");
        n_link[RKN] = Robot->AddBody(n_link[RHP],Xtrans(rhp2rkn),j_link[RKN],b_link[RKN],"RKN");
        n_link[RAP] = Robot->AddBody(n_link[RKN],Xtrans(rkn2rap),j_link[RAP],b_link[RAP],"RAP");
        n_link[RAR] = Robot->AddBody(n_link[RAP],Xtrans(rap2rar),j_link[RAR],b_link[RAR],"RAR");

        // pel to left leg
        n_link[LHR] = Robot->AddBody(n_link[PEL],Xtrans(pel2lhr),j_link[LHR],b_link[LHR],"LHR");
        n_link[LHY] = Robot->AddBody(n_link[LHR],Xtrans(lhr2lhy),j_link[LHY],b_link[LHY],"LHY");
        n_link[LHP] = Robot->AddBody(n_link[LHY],Xtrans(lhy2lhp),j_link[LHP],b_link[LHP],"LHP");
        n_link[LKN] = Robot->AddBody(n_link[LHP],Xtrans(lhp2lkn),j_link[LKN],b_link[LKN],"LKN");
        n_link[LAP] = Robot->AddBody(n_link[LKN],Xtrans(lkn2lap),j_link[LAP],b_link[LAP],"LAP");
        n_link[LAR] = Robot->AddBody(n_link[LAP],Xtrans(lap2lar),j_link[LAR],b_link[LAR],"LAR");

        // pel to torso
        n_link[WST] = Robot->AddBody(n_link[PEL],Xtrans(pel2torso),j_link[WST],b_link[WST],"WST");

        //////////////////////////////////////////////////////////////////////////

        // this joint model is for test version of fixed pelvis robot
        // Fixed Pelvis
        j_pel_F = Joint(JointTypeFixed); //
        n_link_F[PEL] = Robot_FixedBase->AddBody(0,Xtrans(zv),j_pel_F,b_link[PEL],"PEL_F");

        // pel to right leg
        n_link_F[RHR] = Robot_FixedBase->AddBody(n_link_F[PEL],Xtrans(pel2rhr),j_link[RHR],b_link[RHR],"RHR_F");
        n_link_F[RHY] = Robot_FixedBase->AddBody(n_link_F[RHR],Xtrans(rhr2rhy),j_link[RHY],b_link[RHY],"RHY_F");
        n_link_F[RHP] = Robot_FixedBase->AddBody(n_link_F[RHY],Xtrans(rhy2rhp),j_link[RHP],b_link[RHP],"RHP_F");
        n_link_F[RKN] = Robot_FixedBase->AddBody(n_link_F[RHP],Xtrans(rhp2rkn),j_link[RKN],b_link[RKN],"RKN_F");
        n_link_F[RAP] = Robot_FixedBase->AddBody(n_link_F[RKN],Xtrans(rkn2rap),j_link[RAP],b_link[RAP],"RAP_F");
        n_link_F[RAR] = Robot_FixedBase->AddBody(n_link_F[RAP],Xtrans(rap2rar),j_link[RAR],b_link[RAR],"RAR_F");

        // pel to left leg
        n_link_F[LHR] = Robot_FixedBase->AddBody(n_link_F[PEL],Xtrans(pel2lhr),j_link[LHR],b_link[LHR],"LHR_F");
        n_link_F[LHY] = Robot_FixedBase->AddBody(n_link_F[LHR],Xtrans(lhr2lhy),j_link[LHY],b_link[LHY],"LHY_F");
        n_link_F[LHP] = Robot_FixedBase->AddBody(n_link_F[LHY],Xtrans(lhy2lhp),j_link[LHP],b_link[LHP],"LHP_F");
        n_link_F[LKN] = Robot_FixedBase->AddBody(n_link_F[LHP],Xtrans(lhp2lkn),j_link[LKN],b_link[LKN],"LKN_F");
        n_link_F[LAP] = Robot_FixedBase->AddBody(n_link_F[LKN],Xtrans(lkn2lap),j_link[LAP],b_link[LAP],"LAP_F");
        n_link_F[LAR] = Robot_FixedBase->AddBody(n_link_F[LAP],Xtrans(lap2lar),j_link[LAR],b_link[LAR],"LAR_F");

        // pel to torso
        n_link_F[WST] = Robot_FixedBase->AddBody(n_link_F[PEL],Xtrans(pel2torso),j_link[WST],b_link[WST],"WST_F");


        // //////////////////////////////////////////////////////////
        // Robot states
        // //////////////////////////////////////////////////////////

        // ** Important! ** /////////////////////////////////////////
        // Robot has 6+n(#joint) DoF.
        // 6 DoF comes from pelvis position X,Y,Z(Q[0],Q[1],Q[2])  (Don't confuse the order of position and orientation)
        // and orientation written in quaternion X,Y,Z(Q[3],Q[4],Q[5]).
        // The last W component quaternion is added as last element of Q(6+n) << (6+n+1)th component
        // [ Reference : https://bitbucket.org/rbdl/rbdl/issues/44/no-way-to-set-base-link-state-for-floating ]
        /////////////////////////////////////////////////////////////

        Qnow = VectorNd::Zero(LIGHT_DOF+1); // '+1' is for quaternion
        Qnow(QNUM_END) = 1.0;
        dQnow = VectorNd::Zero(LIGHT_DOF);
        Qnow_ZeroPelvis = VectorNd::Zero(LIGHT_DOF+1); // '+1' is for quaternion
        Qnow_ZeroPelvis(QNUM_END) = 1.0;
        dQnow_ZeroPelvis = VectorNd::Zero(LIGHT_DOF);
        Tnow = VectorNd::Zero(LIGHT_ACT_DOF);
        Textnow_RF_byFT = Vector3d::Zero();
        Fextnow_RF_byFT = Vector3d::Zero();
        Textnow_LF_byFT = Vector3d::Zero();
        Fextnow_LF_byFT = Vector3d::Zero();
        Textnow_RF_byJTS = Vector3d::Zero();
        Fextnow_RF_byJTS = Vector3d::Zero();
        Textnow_LF_byJTS = Vector3d::Zero();
        Fextnow_LF_byJTS = Vector3d::Zero();

        Qdes = VectorNd::Zero(LIGHT_DOF+1); // '+1' is for quaternion
        Qdes(QNUM_END) = 1.0;
        dQdes = VectorNd::Zero(LIGHT_DOF);
        ddQdes = VectorNd::Zero(LIGHT_DOF);
        Tdes = VectorNd::Zero(LIGHT_ACT_DOF);
        Textdes_RF = Vector3d::Zero();
        Fextdes_RF = Vector3d::Zero();
        Textdes_LF = Vector3d::Zero();
        Fextdes_LF = Vector3d::Zero();

        Qref = VectorNd::Zero(LIGHT_DOF+1); // '+1' is for quaternion
        Qref(QNUM_END) = 1.0;
        dQref = VectorNd::Zero(LIGHT_DOF);
        ddQref = VectorNd::Zero(LIGHT_DOF);
        Qref_ZeroPelvis = VectorNd::Zero(LIGHT_DOF+1); // '+1' is for quaternion
        Qref_ZeroPelvis(QNUM_END) = 1.0;
        dQref_ZeroPelvis = VectorNd::Zero(LIGHT_DOF);
        ddQref_ZeroPelvis = VectorNd::Zero(LIGHT_DOF);
        Tref = VectorNd::Zero(LIGHT_ACT_DOF);
        Tref_Comp = VectorNd::Zero(LIGHT_ACT_DOF);
        Textref_RF = Vector3d::Zero();
        Fextref_RF = Vector3d::Zero();
        Textref_LF = Vector3d::Zero();
        Fextref_LF = Vector3d::Zero();

        Sref = VectorNd::Zero(LIGHT_ACT_DOF);
        dSref = VectorNd::Zero(LIGHT_ACT_DOF);
        ddSref = VectorNd::Zero(LIGHT_ACT_DOF);
        Fref = VectorNd::Zero(LIGHT_ACT_DOF);

        Qref_Comp = VectorNd::Zero(LIGHT_ACT_DOF);
        dQref_Comp = VectorNd::Zero(LIGHT_ACT_DOF);

        Znow_DSP_RF = Vector3d::Zero();
        Znow_DSP_LF = Vector3d::Zero();
        Znow_SSP_RF = Vector3d::Zero();
        Znow_SSP_LF = Vector3d::Zero();

        ContactState_RF = false;
        ContactState_LF = false;

        CPnow = Vector3d::Zero();
        CPdes = Vector3d::Zero();
        dCPdes = Vector3d::Zero();
        CPref = Vector3d::Zero();
        dCPref = Vector3d::Zero();
        CPlast = Vector3d::Zero();

        for (int idx=0;idx<30;idx++) {
            SavingVector[idx] = Vector3d::Zero();
        }

        // //////////////////////////////////////////////////////////
        // Joint Limits (Angle, Velocity, Acceleration)
        // //////////////////////////////////////////////////////////

        Qmax = LIGHT_Info->Qmax;
        Qmin = LIGHT_Info->Qmin;
        dQmax = LIGHT_Info->dQmax;
        dQmin = LIGHT_Info->dQmin;
        ddQmax = LIGHT_Info->ddQmax;
        ddQmin = LIGHT_Info->ddQmin;

        dQCon_ub = 10000.0*MatrixNd::Ones(LIGHT_DOF,1);
        dQCon_lb = -10000.0*MatrixNd::Ones(LIGHT_DOF,1);
        ddQCon_ub = 10000.0*MatrixNd::Ones(LIGHT_DOF,1);
        ddQCon_lb = -10000.0*MatrixNd::Ones(LIGHT_DOF,1);

        //////////////////////////////////////////////////////////////

        Kp_AnkleTorqueComp = 0.02;
        Ki_AnkleTorqueComp = 1.50;

//        K_RF_ori_OnContact(0) = 100.0; // Nm/rad
//        K_RF_ori_OnContact(1) = 100.0;
//        K_RF_ori_OnContact(2) = 100.0;
//        K_RF_pos_OnContact(0) = 100.0; // N/m
//        K_RF_pos_OnContact(1) = 100.0;
//        K_RF_pos_OnContact(2) = 100.0;
//        D_RF_ori_OnContact(0) = 10.0;
//        D_RF_ori_OnContact(1) = 10.0;
//        D_RF_ori_OnContact(2) = 10.0;
//        D_RF_pos_OnContact(0) = 10.0;
//        D_RF_pos_OnContact(1) = 10.0;
//        D_RF_pos_OnContact(2) = 10.0;

        K_RF_ori_OnContact(0) = 300.0; // Nm/rad
        K_RF_ori_OnContact(1) = 300.0;
        K_RF_ori_OnContact(2) = 50.0;
        K_RF_pos_OnContact(0) = 300.0; // N/m
        K_RF_pos_OnContact(1) = 300.0;
        K_RF_pos_OnContact(2) = 100.0;
        D_RF_ori_OnContact(0) = 15.0;
        D_RF_ori_OnContact(1) = 15.0;
        D_RF_ori_OnContact(2) = 5.0;
        D_RF_pos_OnContact(0) = 15.0;
        D_RF_pos_OnContact(1) = 15.0;
        D_RF_pos_OnContact(2) = 5.0;

        K_RF_ori_OffContact(0) = 200.0; // Nm/rad
        K_RF_ori_OffContact(1) = 200.0;
        K_RF_ori_OffContact(2) = 200.0;
        K_RF_pos_OffContact(0) = 300.0; // N/m
        K_RF_pos_OffContact(1) = 300.0;
        K_RF_pos_OffContact(2) = 300.0;
        D_RF_ori_OffContact(0) = 10.0;
        D_RF_ori_OffContact(1) = 10.0;
        D_RF_ori_OffContact(2) = 10.0;
        D_RF_pos_OffContact(0) = 10.0;
        D_RF_pos_OffContact(1) = 10.0;
        D_RF_pos_OffContact(2) = 10.0;

//        K_RF_ori_OffContact(0) = 50.0; // Nm/rad
//        K_RF_ori_OffContact(1) = 50.0;
//        K_RF_ori_OffContact(2) = 50.0;
//        K_RF_pos_OffContact(0) = 50.0; // N/m
//        K_RF_pos_OffContact(1) = 50.0;
//        K_RF_pos_OffContact(2) = 50.0;
//        D_RF_ori_OffContact(0) = 10.0;
//        D_RF_ori_OffContact(1) = 10.0;
//        D_RF_ori_OffContact(2) = 10.0;
//        D_RF_pos_OffContact(0) = 10.0;
//        D_RF_pos_OffContact(1) = 10.0;
//        D_RF_pos_OffContact(2) = 10.0;

//        K_RF_ori_OffContact(0) = 500.0; // Nm/rad
//        K_RF_ori_OffContact(1) = 500.0;
//        K_RF_ori_OffContact(2) = 100.0;
//        K_RF_pos_OffContact(0) = 500.0; // N/m
//        K_RF_pos_OffContact(1) = 500.0;
//        K_RF_pos_OffContact(2) = 100.0;
//        D_RF_ori_OffContact(0) = 20.0;
//        D_RF_ori_OffContact(1) = 20.0;
//        D_RF_ori_OffContact(2) = 20.0;
//        D_RF_pos_OffContact(0) = 20.0;
//        D_RF_pos_OffContact(1) = 20.0;
//        D_RF_pos_OffContact(2) = 20.0;

        K_LF_ori_OnContact = K_RF_ori_OnContact;
        K_LF_pos_OnContact = K_RF_pos_OnContact;
        D_LF_ori_OnContact = D_RF_ori_OnContact;
        D_LF_pos_OnContact = D_RF_pos_OnContact;

        K_LF_ori_OffContact = K_RF_ori_OffContact;
        K_LF_pos_OffContact = K_RF_pos_OffContact;
        D_LF_ori_OffContact = D_RF_ori_OffContact;
        D_LF_pos_OffContact = D_RF_pos_OffContact;
    }

    ///// Error Check
    void CheckStates();

    ///// state update
    void UpdateSensorData();
    void UpdateStates();
    void UpdateReferenceStates(bool UpdateEnable);
    void ChangeRefFrame_ReferenceStates(int _PreviousFrame, int _CurrentFrame);

    void SetFootStiffnDamp_RF(VectorNd _StiffnDamp);
    void SetFootStiffnDamp_LF(VectorNd _StiffnDamp);
    void UpdateFootStiffnDamp(bool UpdateEnable);

    void SetReference_Joint(bool UpdateEnable, bool PelvisCompensation=true);
    void SetReference_Actuator(bool UpdateEnable);
    void SetActuatorStiffnDamp(bool UpdateEnable);

    void SetJointLimit_dQ(); // Limit the joint velocity with ang,vel,acc constraints
    void SetJointLimit_ddQ(); // Limit the joint velocity with ang,vel,acc constraints

    ///// Kinematics and Dynamics
    void InverseKinematics();

    void InverseDynamics();
    void InverseDynamics_RightSupport();
    void InverseDynamics_FixedBase();

    // state estimate by using forward kinematics
    Matrix3d CalcWorld2BodyOrientation(Model &model, const Math::VectorNd &Q, const unsigned int body_id, bool update_kinematics = false) {
        Matrix3d R = CalcBodyWorldOrientation(model, Q, body_id, update_kinematics);
        return R.transpose();
    }

    void UpdateJacobFoot();
    void CalcFextwithTorque();

    void CalcFoot2ZMPwith6FT();
    void CalcFoot2ZMPwithTorque();

    MatrixNd CalcCoMJacobian(MatrixNd _Q);
    MatrixNd CalcCoMJacobian6D(MatrixNd _Q);
    Vector3d CalcCoMPosition(MatrixNd _Q);
    Vector3d CalcCoMVelocity(MatrixNd _Q,MatrixNd _dQ);
    Vector3d CalcCoMAcceleration(MatrixNd _Q,MatrixNd _dQ,MatrixNd _ddQ);
    Vector3d CalcCoMdJdQ(MatrixNd _Q, MatrixNd _dQ);

    ///// Joint State >> Actuator(Cylinder or Rotary) State
    ///// For reference generation
    void    Joint2Actuator_HipRoll(double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_HipYaw(double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_HipPitch(double theta_m, double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_Knee(double theta_m, double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_Ankle(double theta_m, double theta, double dtheta, double Tp,
                                 double phi_m, double phi, double dphi, double Tr,
                                 VectorNd &S, VectorNd &dS, VectorNd &F);
    void    Joint2Actuator_WaistYaw(double theta, double dtheta, double T, double &S, double &dS, double &F);

    double  Angle2CylinderPos_Hip(double theta);
    double  AngularVel2CylinderVel_Hip(double theta, double dtheta);
    double  Torque2CylinderForce_Hip(double theta, double T);
    double  Angle2CylinderPos_Knee(double theta);
    double  AngularVel2CylinderVel_Knee(double theta, double dtheta);
    double  Torque2CylinderForce_Knee(double theta, double T);
    VectorNd  Angle2CylinderPos_Ankle(double theta, double phi);
    VectorNd  AngularVel2CylinderVel_Ankle(double theta, double phi, double dtheta, double dphi);
    VectorNd  Torque2CylinderForce_Ankle(double theta, double phi, double T_p, double T_r);

    void    CalcTorqueLimit_Hip(int RL, double& Tmax, double& Tmin);
    void    CalcTorqueLimit_Knee(int RL, double& Tmax, double& Tmin);
    void    CalcTorqueLimit_Ankle(int RL, double& Tpitchmax, double& Tpitchmin, double& Trollmax, double& Trollmin);

    void GenerateSupplyPressureReference();

};

#endif // LIGHT_HOPPING_ROBOTMODEL
