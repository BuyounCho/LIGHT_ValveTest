#include "LIGHT_dynamics.h"
#include "LIGHT_motion.h"

extern void PrintHere(int n);

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Vector3d OrientationError(Matrix3d Rdes, Matrix3d R);
void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B);
void Matrix4QP_Addition(MatrixNd& A_cost, VectorNd& B_cost, MatrixNd A_temp, VectorNd B_temp);
bool Dynamics_SolutionCheck(VectorNd T_sol);

///////////////////////////////////////////////////////////////////////////////////////////////////
////////// Force Control Functions /////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

// ======================== SupportControl Gain Setting ============================= //

void LIGHTWholeMotions::SetLeadCompensateGain(double _wn_X, double _wn_Y, double _zeta_X, double _zeta_Y)
{
    CoM_wn_X = _wn_X;
    CoM_wn_Y = _wn_Y;
    CoM_zeta_X = _zeta_X;
    CoM_zeta_Y = _zeta_Y;
}

void LIGHTWholeMotions::SetSupportControlGain_DSP(double _WeightCompensation,
                                                  Vector3d _Freq_body_pos, Vector3d _Zeta_body_pos,
                                                  Vector3d _Freq_body_ori, Vector3d _Zeta_body_ori,
                                                  Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                                  Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori)
{
    WeightCompensation_DSP = _WeightCompensation;
    Freq_body_pos_DSP = _Freq_body_pos;
    Zeta_body_pos_DSP = _Zeta_body_pos;
    Freq_body_ori_DSP = _Freq_body_ori;
    Zeta_body_ori_DSP = _Zeta_body_ori;
    Freq_foot_pos_DSP = _Freq_foot_pos;
    Zeta_foot_pos_DSP = _Zeta_foot_pos;
    Freq_foot_ori_DSP = _Freq_foot_ori;
    Zeta_foot_ori_DSP = _Zeta_foot_ori;
}

void LIGHTWholeMotions::SetSupportControlGain_RSSP(double _WeightCompensation,
                                                   Vector3d _Freq_body_pos, Vector3d _Zeta_body_pos,
                                                   Vector3d _Freq_body_ori, Vector3d _Zeta_body_ori,
                                                   Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                                   Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori)
{
    WeightCompensation_RSSP = _WeightCompensation;
    Freq_body_pos_RSSP = _Freq_body_pos;
    Zeta_body_pos_RSSP = _Zeta_body_pos;
    Freq_body_ori_RSSP = _Freq_body_ori;
    Zeta_body_ori_RSSP = _Zeta_body_ori;
    Freq_foot_pos_RSSP = _Freq_foot_pos;
    Zeta_foot_pos_RSSP = _Zeta_foot_pos;
    Freq_foot_ori_RSSP = _Freq_foot_ori;
    Zeta_foot_ori_RSSP = _Zeta_foot_ori;
}

void LIGHTWholeMotions::SetSupportControlGain_LSSP(double _WeightCompensation,
                                                   Vector3d _Freq_body_pos, Vector3d _Zeta_body_pos,
                                                   Vector3d _Freq_body_ori, Vector3d _Zeta_body_ori,
                                                   Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                                   Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori)
{
    WeightCompensation_LSSP = _WeightCompensation;
    Freq_body_pos_LSSP = _Freq_body_pos;
    Zeta_body_pos_LSSP = _Zeta_body_pos;
    Freq_body_ori_LSSP = _Freq_body_ori;
    Zeta_body_ori_LSSP = _Zeta_body_ori;
    Freq_foot_pos_LSSP = _Freq_foot_pos;
    Zeta_foot_pos_LSSP = _Zeta_foot_pos;
    Freq_foot_ori_LSSP = _Freq_foot_ori;
    Zeta_foot_ori_LSSP = _Zeta_foot_ori;
}

void LIGHTWholeMotions::SetSupportControlGain_Float(Vector3d _Freq_foot_pos, Vector3d _Zeta_foot_pos,
                                                    Vector3d _Freq_foot_ori, Vector3d _Zeta_foot_ori)
{
    Freq_foot_pos_Float = _Freq_foot_pos;
    Zeta_foot_pos_Float = _Zeta_foot_pos;
    Freq_foot_ori_Float = _Freq_foot_ori;
    Zeta_foot_ori_Float = _Zeta_foot_ori;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
/// Support Control by Whole-Body Dynamics
////////////////////////////////////////////////////////////////////////////////////////////////////

bool LIGHTWholeMotions::SupportControl_DSP_byWBD(char _RefFrame)
{
    //    LIGHT.ContactOn_RF();
    //    LIGHT.ContactOn_LF();

    // RefFrame : which foot frame is reference frame?
    //            -1 : right, 1 : left
    if(_RefFrame == LIGHT.REFFRAME_RF) {
        LIGHT.Xdes_LF2CoM = LIGHT.Xdes_RF2CoM - LIGHT.Xdes_RF2LF;
        LIGHT.dXdes_LF2CoM = LIGHT.dXdes_RF2CoM - LIGHT.dXdes_RF2LF;
    } else if (_RefFrame == LIGHT.REFFRAME_LF) {
        LIGHT.Xdes_RF2CoM = LIGHT.Xdes_LF2CoM - LIGHT.Xdes_LF2RF;
        LIGHT.dXdes_RF2CoM = LIGHT.dXdes_LF2CoM - LIGHT.dXdes_LF2RF;
    } else {
        FILE_LOG(logERROR) << "[Error] Reference frame is not set.";
        return false;
    }

    VectorNd ddQ_sol = VectorNd::Zero(LIGHT_DOF);
    VectorNd Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);
    VectorNd Ext_RF_sol = VectorNd::Zero(6);
    VectorNd Ext_LF_sol = VectorNd::Zero(6);

    ////////////////////////////////////////////////////////////////////////////////////
    //// Optimization Problem Formulation (QP) ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    bool QPSolver_On = true;
    if (QPSolver_On) {

        system_clock::time_point t_StartTime = system_clock::now();

        LIGHT_QP QP_Dyn;
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable(44) : [ddQ(19) JointT(13) ExtRF(6) ExtLF(6)]
        // Equality Constraints(0) : X
        // Inequality Constraints(10) : RightFoot_ZMP(4)+LeftFoot_ZMP(4)+GRFz(2)

        const int size_ddQ = LIGHT_DOF; // 19
        const int size_JntTorque = LIGHT_ACT_DOF; // 13
        const int size_ExtForce = 2*6; // Double Support, 12
        const int idx_ddQ = 0;
        const int idx_JntTorque = size_ddQ;
        const int idx_Ext_RF = idx_JntTorque + size_JntTorque;
        const int idx_Ext_LF = idx_Ext_RF+6;
        int n_var   = size_ddQ+size_JntTorque+size_ExtForce;

        /////// Obtain Jacobian Matrices for Formulation /////////////////////////////////////////////////////////////////

        UpdateKinematics(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, VectorNd::Zero(LIGHT_DOF));

        MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[RAR], LIGHT.rar2EE, J_RF, false);
        MatrixNd J_LF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[LAR], LIGHT.lar2EE, J_LF, false);
        MatrixNd J_Pel = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[PEL], zv, J_Pel, false);
        MatrixNd Jv_CoM = LIGHT.CalcCoMJacobian(LIGHT.Qnow);

        MatrixNd Jv_RF2CoM = MatrixNd::Zero(3,size_ddQ);
        Jv_RF2CoM.block(0,0,1,size_ddQ) = Jv_CoM.block(0,0,1,size_ddQ)-J_RF.block(3,0,1,size_ddQ);
        Jv_RF2CoM.block(1,0,1,size_ddQ) = Jv_CoM.block(1,0,1,size_ddQ)-J_RF.block(4,0,1,size_ddQ);
        Jv_RF2CoM.block(2,0,1,size_ddQ) = J_Pel.block(5,0,1,size_ddQ)-J_RF.block(5,0,1,size_ddQ);
        MatrixNd Jv_LF2CoM = MatrixNd::Zero(3,size_ddQ);
        Jv_LF2CoM.block(0,0,1,size_ddQ) = Jv_CoM.block(0,0,1,size_ddQ)-J_LF.block(3,0,1,size_ddQ);
        Jv_LF2CoM.block(1,0,1,size_ddQ) = Jv_CoM.block(1,0,1,size_ddQ)-J_LF.block(4,0,1,size_ddQ);
        Jv_LF2CoM.block(2,0,1,size_ddQ) = J_Pel.block(5,0,1,size_ddQ)-J_LF.block(5,0,1,size_ddQ);

        VectorNd ddQ_Zero = VectorNd::Zero(LIGHT_DOF);
        VectorNd dJdQ_RF = VectorNd::Zero(6);
        dJdQ_RF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[RAR], LIGHT.rar2EE, false);
        VectorNd dJdQ_LF = VectorNd::Zero(6);
        dJdQ_LF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[LAR], LIGHT.lar2EE, false);
        VectorNd dJdQ_Pel = VectorNd::Zero(6);
        dJdQ_Pel = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[PEL], zv, false);
        Vector3d dJdQv_CoM = LIGHT.CalcCoMdJdQ(LIGHT.Qnow,LIGHT.dQnow);

        Vector3d dJdQv_RF2CoM;
        dJdQv_RF2CoM(0) = dJdQv_CoM(0) - dJdQ_RF(3);
        dJdQv_RF2CoM(1) = dJdQv_CoM(1) - dJdQ_RF(4);
        if(LIGHT.CoMzIsPelz) {
            dJdQv_RF2CoM(2) = dJdQ_Pel(5) - dJdQ_RF(5);
        } else {
            dJdQv_RF2CoM(2) = dJdQv_CoM(2) - dJdQ_RF(5);
        }

        Vector3d dJdQv_LF2CoM;
        dJdQv_LF2CoM(0) = dJdQv_CoM(0) - dJdQ_LF(3);
        dJdQv_LF2CoM(1) = dJdQv_CoM(1) - dJdQ_LF(4);
        if(LIGHT.CoMzIsPelz) {
            dJdQv_LF2CoM(2) = dJdQ_Pel(5) - dJdQ_LF(5);
        } else {
            dJdQv_LF2CoM(2) = dJdQv_CoM(2) - dJdQ_LF(5);
        }

        system_clock::time_point t_CalcJacob = system_clock::now();
        microseconds dt_CalcJacob = duration_cast<std::chrono::microseconds>(t_CalcJacob - t_StartTime);

        //////// Cost Functions ////////////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;
        Matrix3d Zeta = Matrix3d::Zero();
        Matrix3d Wn = Matrix3d::Zero();

        // 0-1. Foot Contact Consistent  ///////////////////////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 12;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
//        A_temp.block(0,idx_ddQ,6,size_ddQ) = J_RF;
//        B_temp.block(0,0,6,1) = -dJdQ_RF;
//        A_temp.block(6,idx_ddQ,6,size_ddQ) = J_LF;
//        B_temp.block(6,0,6,1) = -dJdQ_LF;

        if(_RefFrame == LIGHT.REFFRAME_RF) {
            VectorNd A_ref = VectorNd::Zero(6);

            Zeta(0,0) = Zeta_foot_ori_DSP(0);
            Zeta(1,1) = Zeta_foot_ori_DSP(1);
            Zeta(2,2) = Zeta_foot_ori_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_ori_DSP(2);
            A_ref.segment(0,3) = 2.0*Zeta*Wn*(zv-LIGHT.Wnow_LF) + Wn*Wn*OrientationError(I3,LIGHT.Rnow_LF);
            Zeta(0,0) = Zeta_foot_pos_DSP(0);
            Zeta(1,1) = Zeta_foot_pos_DSP(1);
            Zeta(2,2) = Zeta_foot_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_DSP(2);
            A_ref.segment(3,3) = 2.0*Zeta*Wn*(zv-LIGHT.dXnow_LF) + Wn*Wn*(LIGHT.Xref_RF2LF-LIGHT.Xnow_LF);
            A_temp.block(0,idx_ddQ,6,size_ddQ) = J_LF;
            B_temp.block(0,0,6,1) = A_ref-dJdQ_LF;

            A_ref = VectorNd::Zero(6);
            Zeta(0,0) = Zeta_foot_ori_DSP(0);
            Zeta(1,1) = Zeta_foot_ori_DSP(1);
            Zeta(2,2) = Zeta_foot_ori_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_ori_DSP(2);
            A_ref.segment(0,3) = 2.0*Zeta*Wn*(zv-LIGHT.Wnow_RF) + Wn*Wn*OrientationError(I3,LIGHT.Rnow_RF);
            Zeta(0,0) = Zeta_foot_pos_DSP(0);
            Zeta(1,1) = Zeta_foot_pos_DSP(1);
            Zeta(2,2) = Zeta_foot_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_DSP(2);
            A_ref.segment(3,3) = 2.0*Zeta*Wn*(zv-LIGHT.dXnow_RF) + Wn*Wn*(zv-LIGHT.Xnow_RF);
            A_temp.block(6,idx_ddQ,6,size_ddQ) = J_RF;
            B_temp.block(6,0,6,1) = A_ref-dJdQ_RF;

        } else if (_RefFrame == LIGHT.REFFRAME_LF) {
            VectorNd A_ref = VectorNd::Zero(6);

            Zeta(0,0) = Zeta_foot_ori_DSP(0);
            Zeta(1,1) = Zeta_foot_ori_DSP(1);
            Zeta(2,2) = Zeta_foot_ori_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_ori_DSP(2);
            A_ref.segment(0,3) = 2.0*Zeta*Wn*(zv-LIGHT.Wnow_RF) + Wn*Wn*OrientationError(I3,LIGHT.Rnow_RF);
            Zeta(0,0) = Zeta_foot_pos_DSP(0);
            Zeta(1,1) = Zeta_foot_pos_DSP(1);
            Zeta(2,2) = Zeta_foot_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_DSP(2);
            A_ref.segment(3,3) = 2.0*Zeta*Wn*(zv-LIGHT.dXnow_RF) + Wn*Wn*(LIGHT.Xref_LF2RF-LIGHT.Xnow_RF);
            A_temp.block(0,idx_ddQ,6,size_ddQ) = J_RF-J_LF;
            B_temp.block(0,0,6,1) = A_ref-(dJdQ_RF-dJdQ_LF);

            A_ref = VectorNd::Zero(6);
            Zeta(0,0) = Zeta_foot_ori_DSP(0);
            Zeta(1,1) = Zeta_foot_ori_DSP(1);
            Zeta(2,2) = Zeta_foot_ori_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_ori_DSP(2);
            A_ref.segment(0,3) = 2.0*Zeta*Wn*(zv-LIGHT.Wnow_LF) + Wn*Wn*OrientationError(I3,LIGHT.Rnow_LF);
            Zeta(0,0) = Zeta_foot_pos_DSP(0);
            Zeta(1,1) = Zeta_foot_pos_DSP(1);
            Zeta(2,2) = Zeta_foot_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_DSP(2);
            A_ref.segment(3,3) = 2.0*Zeta*Wn*(zv-LIGHT.dXnow_LF) + Wn*Wn*(zv-LIGHT.Xnow_LF);
            A_temp.block(6,idx_ddQ,6,size_ddQ) = J_LF;
            B_temp.block(6,0,6,1) = A_ref-dJdQ_LF;
        } else {
            FILE_LOG(logERROR) << "Reference Frame Error!!";
            return false;
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 1-1. Pelvis Orientation(3) //////////////////////////////////////////////////////////////////////
        W_temp = 300.0;
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        Zeta(0,0) = Zeta_body_ori_DSP(0);
        Zeta(1,1) = Zeta_body_ori_DSP(1);
        Zeta(2,2) = Zeta_body_ori_DSP(2);
        Wn(0,0) = 2.0*PI*Freq_body_ori_DSP(0);
        Wn(1,1) = 2.0*PI*Freq_body_ori_DSP(1);
        Wn(2,2) = 2.0*PI*Freq_body_ori_DSP(2);
        Vector3d dW_ref = LIGHT.dWdes_Pel
                + 2.0*Zeta*Wn*(LIGHT.Wdes_Pel-LIGHT.Wnow_Pel)
                + Wn*Wn*OrientationError(LIGHT.Rdes_Pel,LIGHT.Rnow_Pel);

        A_temp.block(0,idx_ddQ,3,size_ddQ) = J_Pel.block(0,0,3,size_ddQ);
        B_temp.block(0,0,3,1) = (dW_ref-dJdQ_Pel.segment(0,3));

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;


        // 2-1. RF to CoM Position (3) /////////////////////////////////////////////////////////////////////
//        if(_RefFrame == LIGHT.REFFRAME_RF)
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_pos_DSP(0);
            Zeta(1,1) = Zeta_body_pos_DSP(1);
            Zeta(2,2) = Zeta_body_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_body_pos_DSP(2);
            Vector3d A_ref;
            A_ref(0) = LIGHT.ddXdes_RF2CoM(0) + 2.0*Zeta(0,0)*Wn(0,0)*(LIGHT.dXdes_RF2CoM(0)-LIGHT.dXnow_RF2CoM(0)) + Wn(0,0)*Wn(0,0)*(LIGHT.Xdes_RF2CoM(0)-LIGHT.Xnow_RF2CoM(0));
            A_ref(1) = LIGHT.ddXdes_RF2CoM(1) + 2.0*Zeta(1,1)*Wn(1,1)*(LIGHT.dXdes_RF2CoM(1)-LIGHT.dXnow_RF2CoM(1)) + Wn(1,1)*Wn(1,1)*(LIGHT.Xdes_RF2CoM(1)-LIGHT.Xnow_RF2CoM(1));
            A_ref(2) = LIGHT.ddXdes_RF2CoM(2) + 2.0*Zeta(2,2)*Wn(2,2)*(LIGHT.dXdes_RF2CoM(2)-LIGHT.dXnow_RF2CoM(2)) + Wn(2,2)*Wn(2,2)*(LIGHT.Xdes_RF2CoM(2)-LIGHT.Xnow_RF2CoM(2));

            A_temp.block(0,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(0);
            A_temp.block(1,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(1);
            A_temp.block(2,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(2);
            B_temp(0,0) = (A_ref(0)-dJdQv_RF2CoM(0));
            B_temp(1,0) = (A_ref(1)-dJdQv_RF2CoM(1));
            B_temp(2,0) = (A_ref(2)-dJdQv_RF2CoM(2));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 2-2. LF to CoM Position (3) /////////////////////////////////////////////////////////////////////
//        if(_RefFrame == LIGHT.REFFRAME_LF)
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_pos_DSP(0);
            Zeta(1,1) = Zeta_body_pos_DSP(1);
            Zeta(2,2) = Zeta_body_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_body_pos_DSP(2);
            Vector3d A_ref;
            A_ref(0) = LIGHT.ddXdes_LF2CoM(0) + 2.0*Zeta(0,0)*Wn(0,0)*(LIGHT.dXdes_LF2CoM(0)-LIGHT.dXnow_LF2CoM(0)) + Wn(0,0)*Wn(0,0)*(LIGHT.Xdes_LF2CoM(0)-LIGHT.Xnow_LF2CoM(0));
            A_ref(1) = LIGHT.ddXdes_LF2CoM(1) + 2.0*Zeta(1,1)*Wn(1,1)*(LIGHT.dXdes_LF2CoM(1)-LIGHT.dXnow_LF2CoM(1)) + Wn(1,1)*Wn(1,1)*(LIGHT.Xdes_LF2CoM(1)-LIGHT.Xnow_LF2CoM(1));
            A_ref(2) = LIGHT.ddXdes_LF2CoM(2) + 2.0*Zeta(2,2)*Wn(2,2)*(LIGHT.dXdes_LF2CoM(2)-LIGHT.dXnow_LF2CoM(2)) + Wn(2,2)*Wn(2,2)*(LIGHT.Xdes_LF2CoM(2)-LIGHT.Xnow_LF2CoM(2));

            A_temp.block(0,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(0);
            A_temp.block(1,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(1);
            A_temp.block(2,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(2);
            B_temp(0,0) = (A_ref(0)-dJdQv_LF2CoM(0));
            B_temp(1,0) = (A_ref(1)-dJdQv_LF2CoM(1));
            B_temp(2,0) = (A_ref(2)-dJdQv_LF2CoM(2));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 3. Acceleration Minimization
        W_temp = 30.0;
        temp_size_cost = LIGHT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 4. Joint Torque Minimization ///////////////////////////////////////
        W_temp = 0.01;
        temp_size_cost = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        B_temp(RHP) = 0.5*LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // DSP Right HipPich Torque
        B_temp(LHP) = 0.5*LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // DSP Left HipPich Torque
        B_temp(RKN) = -60.0; // DSP Right Knee Torque
        B_temp(LKN) = -60.0; // DSP Right Knee Torque
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 5. External Force/Torque Minimization
        W_temp = 0.01;
        temp_size_cost = 12;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_Ext_RF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        if(_RefFrame == LIGHT.REFFRAME_RF) {
            Vector3d Step = LIGHT.Xref_RF2LF;
            Vector3d CoM = LIGHT.Xref_RF2CoM;
            double m = LIGHT.LIGHT_Info->m_robot;
            double alpha = Step.dot(CoM)/Step.dot(Step);
            if(alpha>1.0) alpha = 1.0;
            else if(alpha<0.0) alpha = 0.0;
            B_temp(5) = (1.0-alpha)*m*g_const; // RFz
            B_temp(11) = alpha*m*g_const; // LFz
        } else {
            Vector3d Step = LIGHT.Xref_LF2RF;
            Vector3d CoM = LIGHT.Xref_LF2CoM;
            double m = LIGHT.LIGHT_Info->m_robot;
            double alpha = Step.dot(CoM)/Step.dot(Step);
            if(alpha>1.0) alpha = 1.0;
            else if(alpha<0.0) alpha = 0.0;
            B_temp(5) = alpha*m*g_const; // RFz
            B_temp(11) = (1.0-alpha)*m*g_const; // LFz
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 6. Torque Solution Change Minimization
        W_temp = 0.005;
        temp_size_cost = size_JntTorque;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque,size_JntTorque,size_JntTorque) = MatrixNd::Identity(size_JntTorque,size_JntTorque);
        B_temp.block(0,0,size_JntTorque,1) = LIGHT.Tref;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 7. External Torque/Force Change Minimization
        W_temp = 0.005;
        temp_size_cost = size_ExtForce;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_Ext_RF,size_ExtForce,size_ExtForce) = MatrixNd::Identity(size_ExtForce,size_ExtForce);
        B_temp.block(0,0,3,1) = LIGHT.Textref_RF;
        B_temp.block(3,0,3,1) = LIGHT.Fextref_RF;
        B_temp.block(6,0,3,1) = LIGHT.Textref_LF;
        B_temp.block(9,0,3,1) = LIGHT.Fextref_LF;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // ////////////////////////////////////////////////////////////
        // Cost Function End

        n_cost = idx_cost;

//        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
//        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

        system_clock::time_point t_DefCostFn = system_clock::now();
        microseconds dt_DefCostFn = duration_cast<std::chrono::microseconds>(t_DefCostFn - t_CalcJacob);

        //////// Equality Constraints (Aq=B) /////////////////////////////////////////////////////////////
        /// Optimization Variables : U = [ddQ(n+6);
        ///                               JointTorque(n);
        ///                               ExternalTorque_RightFoot(3);  ExternalForce_RightFoot(3);
        ///                               ExternalTorque_LeftFoot(3);   ExternalForce_LeftFoot(3);
        ///
        /// 1) Equation of Motion :
        ///  Equation of Motion (for jumping with linear guide)
        ///  EoM is Fundamental equation for the system
        ///  This equation is based on physical phenomenom.
        ///  So, the solution must satisfy EoM  >>>  "Equality Constraint"
        ///
        ///                                        r Jacobian of RF Contact point
        ///  M(Q)*ddQ + C(Q,dQ) + G(Q) = S*Tau + JT_RF(Q)*Fext_RF + JT_LF(Q)*Fext_LF
        ///                                 L Joint Torque
        ///  >> [M(Q) -S -JT_RF(q) -JT_LF(q)]*[ddQ;Tau;Fext_RF;Fext_LF] = -C(Q,dQ) - G(Q)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ = 0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        // 1. Equation of Motion ////////////////////////////////////////////////////////////
        temp_size_equ = size_ddQ;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);

        // Make mass matrix (M)
        MatrixNd tempM = MatrixNd::Zero(size_ddQ,size_ddQ);
        CompositeRigidBodyAlgorithm(*LIGHT.Robot,LIGHT.Qnow,tempM,false);
        // Make Selection matrix (S)
        MatrixNd tempS = MatrixNd::Zero(size_ddQ,size_JntTorque);
        tempS.block(6, 0, size_JntTorque, size_JntTorque) = MatrixNd::Identity(size_JntTorque,size_JntTorque);
        // Make nonlinear matrix (C+G)
        VectorNd tempN = VectorNd::Zero(size_ddQ,1);
        NonlinearEffects(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, tempN);

        A_temp.block(0,idx_ddQ,size_ddQ,size_ddQ) = tempM;
        A_temp.block(0,idx_JntTorque,size_ddQ,size_JntTorque) = -tempS;
        A_temp.block(0,idx_Ext_RF,size_ddQ,6) = -J_RF.transpose();
        A_temp.block(0,idx_Ext_LF,size_ddQ,6) = -J_LF.transpose();
        B_temp = -tempN;
        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        system_clock::time_point t_DefEquCon = system_clock::now();
        microseconds dt_DefEquCon = duration_cast<std::chrono::microseconds>(t_DefEquCon - t_DefCostFn);

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ;
        VectorNd B_inequ;

        // ZMP Constraints *************************
        double ZMPx_RF_min = -LIGHT.LIGHT_Info->ZMPlimit_heel;
        double ZMPx_RF_max = LIGHT.LIGHT_Info->ZMPlimit_toe;
        double ZMPy_RF_min = -LIGHT.LIGHT_Info->ZMPlimit_outside;
        double ZMPy_RF_max = LIGHT.LIGHT_Info->ZMPlimit_inside;
        double ZMPx_LF_min = -LIGHT.LIGHT_Info->ZMPlimit_heel;
        double ZMPx_LF_max = LIGHT.LIGHT_Info->ZMPlimit_toe;
        double ZMPy_LF_min = -LIGHT.LIGHT_Info->ZMPlimit_inside;
        double ZMPy_LF_max = LIGHT.LIGHT_Info->ZMPlimit_outside;

        // 1) Right Foot, ZMP_x_max : T(RAP) / (R * Fz) < ZMP_x_max
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAP,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = -ZMPx_RF_max*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 2) Right Foot, ZMP_x_min : T(RAP) / (R * Fz) > ZMP_x_min
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAP,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = ZMPx_RF_min*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 3) Right Foot, ZMP_y_min : T(RAR) < -ZMP_y_min * (R * Fz)
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAR,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = ZMPy_RF_min*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 4) Right Foot, ZMP_y_max : -ZMP_y_max * (R * Fz) < T(RAR)
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAR,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = -ZMPy_RF_max*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 5) Left Foot, ZMP_x_max : T(LAP) / (R * Fz) < ZMP_x_max
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAP,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = -ZMPx_LF_max*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 6) Left Foot, ZMP_x_min : T(LAP) / (R * Fz) > ZMP_x_min
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAP,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = ZMPx_LF_min*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 7) Left Foot, ZMP_y_min : T(LAR) < -ZMP_y_min * (R * Fz)
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAR,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = ZMPy_LF_min*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 8) Left Foot, ZMP_y_max : -ZMP_y_max * (R * Fz) < T(RAR)
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAR,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = -ZMPy_LF_max*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // Fz Constraints (Positive Force) *************************
        double Fz_min = 10.0;

        // 9) RightFoot Fz>=30N (Downward)
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_Ext_RF+5,1,1) = -MatrixNd::Identity(1,1);
        B_temp.block(0,0,1,1) << -Fz_min;
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 10) LeftFoot Fz>=30N (Downward)
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_Ext_LF+5,1,1) = -MatrixNd::Identity(1,1);
        B_temp.block(0,0,1,1) << -Fz_min;
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // Friction Constraints *************************
        double mu = 0.4;
        if(_RefFrame == LIGHT.REFFRAME_RF) {
            // 11) Left Foot X-direction :  Fx_LF / Fz_LF < mu  ||  Fx_LF / Fz_LF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_LF+3) = 1.0;
            A_temp(0,idx_Ext_LF+5) = -mu;
            A_temp(1,idx_Ext_LF+3) = -1.0;
            A_temp(1,idx_Ext_LF+5) = -mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 12) Left Foot X-direction :  Fy_LF / Fz_LF < mu  ||  Fy_LF / Fz_LF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_LF+4) = 1.0;
            A_temp(0,idx_Ext_LF+5) = -mu;
            A_temp(1,idx_Ext_LF+4) = -1.0;
            A_temp(1,idx_Ext_LF+5) = -mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        } else if(_RefFrame == LIGHT.REFFRAME_LF) {
            // 11) Right Foot X-direction :  Fx_RF / Fz_RF < mu  ||  Fx_RF / Fz_RF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_RF+3) = 1.0;
            A_temp(0,idx_Ext_RF+5) = -mu;
            A_temp(1,idx_Ext_RF+3) = -1.0;
            A_temp(1,idx_Ext_RF+5) = -mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 12) Right Foot X-direction :  Fy_RF / Fz_RF < mu  ||  Fy_RF / Fz_RF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_RF+4) = 1.0;
            A_temp(0,idx_Ext_RF+5) = -mu;
            A_temp(1,idx_Ext_RF+4) = -1.0;
            A_temp(1,idx_Ext_RF+5) = -mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        // Torque Limit Constraints *************************
        VectorNd Tmax = VectorNd::Zero(12);
        VectorNd Tmin = VectorNd::Zero(12);

        Tmax(RHR) = 200.0;
        Tmin(RHR) = -200.0;
        Tmax(RHY) = 100.0;
        Tmin(RHY) = -100.0;
        LIGHT.CalcTorqueLimit_Hip(RIGHTLEG,Tmax(RHP),Tmin(RHP));
        LIGHT.CalcTorqueLimit_Knee(RIGHTLEG,Tmax(RKN),Tmin(RKN));
        LIGHT.CalcTorqueLimit_Ankle(RIGHTLEG,Tmax(RAP),Tmin(RAP),Tmax(RAR),Tmin(RAR));

        Tmax(LHR) = 200.0;
        Tmin(LHR) = -200.0;
        Tmax(LHY) = 100.0;
        Tmin(LHY) = -100.0;
        LIGHT.CalcTorqueLimit_Hip(LEFTLEG,Tmax(LHP),Tmin(LHP));
        LIGHT.CalcTorqueLimit_Knee(LEFTLEG,Tmax(LKN),Tmin(LKN));
        LIGHT.CalcTorqueLimit_Ankle(LEFTLEG,Tmax(LAP),Tmin(LAP),Tmax(LAR),Tmin(LAR));

        // 13) Torque Upper Limit
        temp_size_inequ = 12;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque,12,12) = MatrixNd::Identity(12,12);
        B_temp = Tmax;
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 14) Torque Lower Limit
        temp_size_inequ = 12;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque,12,12) = -MatrixNd::Identity(12,12);
        B_temp = -Tmin;
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        n_inequ = idx_inequ;

        system_clock::time_point t_DefInEquC = system_clock::now();
        microseconds dt_DefInEquC = duration_cast<std::chrono::microseconds>(t_DefInEquC - t_DefEquCon);

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_Dyn.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_Dyn.make_COST(A_cost, B_cost);
        QP_Dyn.make_EQ(A_equ,B_equ);
        QP_Dyn.make_IEQ(A_inequ,B_inequ);

        system_clock::time_point t_SetQPSolv = system_clock::now();
        microseconds dt_SetQPSolv = duration_cast<std::chrono::microseconds>(t_SetQPSolv - t_DefInEquC);

        ///////// Solve the problem ////////////////////////////////////////////////////////////////

        switch(QP_Dyn.WhichSolver()) {
        case SolverIsQuadProg:
        {
            // 1. Solve.
            VectorNd QP_sol = QP_Dyn.solve_QP();
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_DOF,LIGHT_ACT_DOF);
            Ext_RF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF,6);
            Ext_LF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF+6,6);
//            FILE_LOG(logINFO) << "DSP Cost : ";
//            for(int i=0;i<n_cost;i++) {
//                FILE_LOG(logINFO) << "idx (" << i << ") : " << (A_cost.row(i)*QP_sol-B_cost.row(i));
//            }
            break;
        }
        case SolverIsQPSwift:
        {
            qp_int n_var = QP_Dyn.NUMCOLS;
            qp_int n_equ = QP_Dyn.NUMEQ;
            qp_int n_inequ = QP_Dyn.NUMINEQ;

            qp_int   P_nnz,A_nnz,G_nnz;
            qp_real  *P_x = NULL;
            qp_int   *P_i = NULL,*P_p = NULL;
            qp_real  *q_x = NULL;
            qp_real  *A_x = NULL;
            qp_int   *A_i = NULL, *A_p = NULL;
            qp_real  *b_x = NULL;
            qp_real  *G_x = NULL;
            qp_int   *G_i = NULL, *G_p = NULL;
            qp_real  *h_x = NULL;

            // 1-0. Convert cost function to sparse form .
            MatrixNd P = QP_Dyn.P_cost;
            VectorNd q = QP_Dyn.Q_cost;
            P_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(P);
            P_x = new qp_real[P_nnz];
            P_i = new qp_int[P_nnz];
            P_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
            q_x = new qp_real[n_var];
            QP_Dyn.ConvertVector2Array_QPswift(q,n_var,q_x);

            // 1-1. Convert equality constraint to sparse form .
            if(n_equ > 0) {
                A_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_equ);
                A_x = new qp_real[A_nnz];
                A_i = new qp_int[A_nnz];
                A_p = new qp_int[n_var+1];
                QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_equ,A_x,A_i,A_p);
                b_x = new qp_real[n_equ];
                QP_Dyn.ConvertVector2Array_QPswift(B_equ,n_equ,b_x);
            }

            // 1-2. Convert inequality constraint to sparse form .
            G_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_inequ);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_inequ,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_Dyn.ConvertVector2Array_QPswift(B_inequ,n_inequ,h_x);

            // 2. Set Parameters and solve.
            QPswift      *myQP;
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                            P_p,P_i,P_x,
                            A_p,A_i,A_x,
                            G_p,G_i,G_x,
                            q_x,h_x,b_x,
                            0.0,nullptr);
            QP_SOLVE(myQP);
            VectorNd QP_sol = VectorNd::Zero(n_var);
            QP_Dyn.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_DOF,LIGHT_ACT_DOF);
            Ext_RF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF,6);
            Ext_LF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF+6,6);

            // 3. destruction allocated memory
            QP_CLEANUP(myQP);
            delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
            delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
            delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
            break;
        }
        default:
            FILE_LOG(logERROR) << "[Dynamics Error] QP Solver is not set!";
            return false;
        }

        system_clock::time_point t_SolveQPro = system_clock::now();
        microseconds dt_SolveQPro = duration_cast<std::chrono::microseconds>(t_SolveQPro - t_SetQPSolv);

        // Check Solving time for QP_Dyn
        if(false) {
            FILE_LOG(logDEBUG) << "dt_CalcJacob : "<< dt_CalcJacob.count() << " us";
            FILE_LOG(logDEBUG) << "dt_DefCostFn : "<< dt_DefCostFn.count() << " us";
            FILE_LOG(logDEBUG) << "dt_DefEquCon : "<< dt_DefEquCon.count() << " us";
            FILE_LOG(logDEBUG) << "dt_DefInEquC : "<< dt_DefInEquC.count() << " us";
            FILE_LOG(logDEBUG) << "dt_SetQPSolv : "<< dt_SetQPSolv.count() << " us";
            FILE_LOG(logDEBUG) << "dt_SolveQPro : "<< dt_SolveQPro.count() << " us";
        }
    } else {
        ddQ_sol = VectorNd::Zero(LIGHT_DOF);
        Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);
        Ext_RF_sol = VectorNd::Zero(6);
        Ext_LF_sol = VectorNd::Zero(6);
    }

    LIGHT.ddQref = ddQ_sol;
    LIGHT.Textref_RF = Ext_RF_sol.segment(0,3);
    LIGHT.Fextref_RF = Ext_RF_sol.segment(3,3);
    LIGHT.Textref_LF = Ext_LF_sol.segment(0,3);
    LIGHT.Fextref_LF = Ext_LF_sol.segment(3,3);
    LIGHT.Tref = Torque_sol;

    return true;
}

bool LIGHTWholeMotions::SupportControl_SSP_RF_byWBD()
{
    //    LIGHT.ContactOn_RF();
    //    LIGHT.ContactOff_LF();

    VectorNd ddQ_sol = VectorNd::Zero(LIGHT_DOF);
    VectorNd Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);
    VectorNd Ext_RF_sol = VectorNd::Zero(6);

    ////////////////////////////////////////////////////////////////////////////////////
    //// Optimization Problem Formulation (QP) ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    bool QPSolver_On = true;
    if (QPSolver_On) {

        LIGHT_QP QP_Dyn;
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable(38) : [ddQ(19) JointT(13) ExtRF(6)]
        // Equality Constraints(0) : X
        // Inequality Constraints(5) : RightFoot_ZMP(4)+GRFz(1)

        const int size_ddQ = LIGHT_DOF; // 19
        const int size_JntTorque = LIGHT_ACT_DOF; // 13
        const int size_ExtForce = 6; // Single Support Support

        const int idx_ddQ = 0;
        const int idx_JntTorque = size_ddQ;
        const int idx_Ext_RF = idx_JntTorque + size_JntTorque;
        int n_var   = size_ddQ+size_JntTorque+size_ExtForce;

        /////// Obtain Jacobian Matrices for Formulation /////////////////////////////////////////////////////////////////

        UpdateKinematics(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, VectorNd::Zero(LIGHT_DOF));

        MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[RAR], LIGHT.rar2EE, J_RF, false);
        MatrixNd J_LF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[LAR], LIGHT.lar2EE, J_LF, false);
        MatrixNd J_Pel = MatrixNd::Zero(6,size_ddQ);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[PEL], zv, J_Pel, false);

        MatrixNd Jv_CoM = LIGHT.CalcCoMJacobian(LIGHT.Qnow);
        MatrixNd Jv_RF2CoM = MatrixNd::Zero(3,size_ddQ);
        Jv_RF2CoM.block(0,0,1,size_ddQ) = Jv_CoM.block(0,0,1,size_ddQ)-J_RF.block(3,0,1,size_ddQ);
        Jv_RF2CoM.block(1,0,1,size_ddQ) = Jv_CoM.block(1,0,1,size_ddQ)-J_RF.block(4,0,1,size_ddQ);
        Jv_RF2CoM.block(2,0,1,size_ddQ) = J_Pel.block(5,0,1,size_ddQ)-J_RF.block(5,0,1,size_ddQ);

        VectorNd ddQ_Zero = VectorNd::Zero(LIGHT_DOF);
        VectorNd dJdQ_RF = VectorNd::Zero(6);
        dJdQ_RF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[RAR], LIGHT.rar2EE, false);
        VectorNd dJdQ_LF = VectorNd::Zero(6);
        dJdQ_LF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[LAR], LIGHT.lar2EE, false);
        VectorNd dJdQ_Pel = VectorNd::Zero(6);
        dJdQ_Pel = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[PEL], zv, false);
        Vector3d dJdQv_CoM = LIGHT.CalcCoMdJdQ(LIGHT.Qnow,LIGHT.dQnow);

        Vector3d dJdQv_RF2CoM;
        dJdQv_RF2CoM(0) = dJdQv_CoM(0) - dJdQ_RF(3);
        dJdQv_RF2CoM(1) = dJdQv_CoM(1) - dJdQ_RF(4);
        if(LIGHT.CoMzIsPelz) {
            dJdQv_RF2CoM(2) = dJdQ_Pel(5) - dJdQ_RF(5);
        } else {
            dJdQv_RF2CoM(2) = dJdQv_CoM(2) - dJdQ_RF(5);
        }

        //////// Cost Functions /////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;
        Matrix3d Zeta = Matrix3d::Zero();
        Matrix3d Wn = Matrix3d::Zero();

        // 0. Foot Contact Consistent  ///////////////////////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 6;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_ddQ,6,size_ddQ) = J_RF;
        B_temp.block(0,0,6,1) = -dJdQ_RF;
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 1. Pelvis Orientation(3) //////////////////////////////////////////////////////////////////////
        {
            W_temp = 300.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_ori_RSSP(0);
            Zeta(1,1) = Zeta_body_ori_RSSP(1);
            Zeta(2,2) = Zeta_body_ori_RSSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_ori_RSSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_ori_RSSP(1);
            Wn(2,2) = 2.0*PI*0.5;
            Vector3d dW_ref = LIGHT.dWdes_Pel
                    + 2.0*Zeta*Wn*(LIGHT.Wdes_Pel-LIGHT.Wnow_Pel)
                    + Wn*Wn*OrientationError(LIGHT.Rdes_Pel,LIGHT.Rnow_Pel);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_Pel.block(0,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (dW_ref-dJdQ_Pel.segment(0,3));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 2. RF to CoM Position (3) /////////////////////////////////////////////////////////////////////
        {
            W_temp = 300.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_pos_RSSP(0);
            Zeta(1,1) = Zeta_body_pos_RSSP(1);
            Zeta(2,2) = Zeta_body_pos_RSSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_pos_RSSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_pos_RSSP(1);
            Wn(2,2) = 2.0*PI*Freq_body_pos_RSSP(2);
            Vector3d A_ref;
            A_ref(0) = LIGHT.ddXdes_RF2CoM(0) + 2.0*Zeta(0,0)*Wn(0,0)*(LIGHT.dXdes_RF2CoM(0)-LIGHT.dXnow_RF2CoM(0)) + Wn(0,0)*Wn(0,0)*(LIGHT.Xdes_RF2CoM(0)-LIGHT.Xnow_RF2CoM(0));
            A_ref(1) = LIGHT.ddXdes_RF2CoM(1) + 2.0*Zeta(1,1)*Wn(1,1)*(LIGHT.dXdes_RF2CoM(1)-LIGHT.dXnow_RF2CoM(1)) + Wn(1,1)*Wn(1,1)*(LIGHT.Xdes_RF2CoM(1)-LIGHT.Xnow_RF2CoM(1));
            A_ref(2) = LIGHT.ddXdes_RF2CoM(2) + 2.0*Zeta(2,2)*Wn(2,2)*(LIGHT.dXdes_RF2CoM(2)-LIGHT.dXnow_RF2CoM(2)) + Wn(2,2)*Wn(2,2)*(LIGHT.Xdes_RF2CoM(2)-LIGHT.Xnow_RF2CoM(2));

            A_temp.block(0,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(0);
            A_temp.block(1,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(1);
            A_temp.block(2,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(2);
            B_temp(0,0) = (A_ref(0)-dJdQv_RF2CoM(0));
            B_temp(1,0) = (A_ref(1)-dJdQv_RF2CoM(1));
            B_temp(2,0) = (A_ref(2)-dJdQv_RF2CoM(2));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 3. LF Orientation(3) //////////////////////////////////////////////////////////////////////
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_ori_RSSP(0);
            Zeta(1,1) = Zeta_foot_ori_RSSP(1);
            Zeta(2,2) = Zeta_foot_ori_RSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_RSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_RSSP(1);
            Wn(2,2) = 2.0*PI*0.5;
            Vector3d dW_ref = LIGHT.dWdes_LF
                    + 2.0*Zeta*Wn*(LIGHT.Wdes_LF-LIGHT.Wnow_LF)
                    + Wn*Wn*OrientationError(LIGHT.Rdes_LF,LIGHT.Rnow_LF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_LF.block(0,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (dW_ref-dJdQ_LF.segment(0,3));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 4. RF2LF Position (3) /////////////////////////////////////////////////////////////////////
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_pos_RSSP(0);
            Zeta(1,1) = Zeta_foot_pos_RSSP(1);
            Zeta(2,2) = Zeta_foot_pos_RSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_RSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_RSSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_RSSP(2);
            Vector3d A_ref = LIGHT.ddXdes_RF2LF
                    + 2.0*Zeta*Wn*(LIGHT.dXdes_RF2LF-LIGHT.dXnow_RF2LF)
                    + Wn*Wn*(LIGHT.Xdes_RF2LF-LIGHT.Xnow_RF2LF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_LF.block(3,0,3,size_ddQ)-J_RF.block(3,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (A_ref-(dJdQ_LF.segment(3,3)-dJdQ_RF.segment(3,3)));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 5. Acceleration Minimization ///////////////////////////////////////
        W_temp = 0.1;
        temp_size_cost = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ+FLOATING_BASE_DOF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        A_temp(RHY,idx_ddQ+FLOATING_BASE_DOF+RHY) = 100.0;
        A_temp(LHY,idx_ddQ+FLOATING_BASE_DOF+LHY) = 100.0;
        A_temp(WST,idx_ddQ+FLOATING_BASE_DOF+WST) = 100.0;
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 6. Torque Minimization ///////////////////////////////////////
        W_temp = 0.01;
        temp_size_cost = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        A_temp(RHY,idx_JntTorque+RHY) = 100.0;
        A_temp(LHY,idx_JntTorque+LHY) = 100.0;
        A_temp(WST,idx_JntTorque+WST) = 100.0;
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        B_temp(RHR) = -65.0;  // RSSP Right HipRoll Torque
        B_temp(RHP) = LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // RSSP Right HipPich Torque
        B_temp(RKN) = -116.0; // RSSP Right Knee Torque
        B_temp(LHR) = 23.0;   // RSSP Left HipRoll Torque
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

//        // 6-2. Hip Pitch Equalization ///////////////////////////////////////
//        W_temp = 0.1;
//        temp_size_cost = 1;
//        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
//        A_temp(0,idx_JntTorque+RHP) = 1.0;
//        A_temp(0,idx_JntTorque+LHP) = 1.0;
//        B_temp(0) = LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // LSSP Right HipPich Torque
//        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
//        idx_cost += temp_size_cost;

        // 7. External Force/Torque Minimization
        W_temp = 0.01;
        temp_size_cost = 6;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_Ext_RF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        A_temp(2,idx_Ext_RF+2) = 100.0;
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        B_temp(5) = LIGHT.LIGHT_Info->m_robot*g_const; // RFz
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 9. Torque Solution Change Minimization
        W_temp = 0.005;
        temp_size_cost = size_JntTorque;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_JntTorque,size_JntTorque,size_JntTorque) = MatrixNd::Identity(size_JntTorque,size_JntTorque);
        B_temp.block(0,0,size_JntTorque,1) = LIGHT.Tref + LIGHT.Tref_Comp;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 10. External Torque/Force Change Minimization
        W_temp = 0.005;
        temp_size_cost = size_ExtForce;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_Ext_RF,size_ExtForce,size_ExtForce) = MatrixNd::Identity(size_ExtForce,size_ExtForce);
        B_temp.block(0,0,3,1) = LIGHT.Textref_RF;
        B_temp.block(3,0,3,1) = LIGHT.Fextref_RF;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        n_cost = idx_cost;

//        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
//        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

        //////// Equality Constraints (Aq=B) /////////////////////////////////////////////////////////////////
        /// Optimization Variables : U = [ddQ(n+6);
        ///                               JointTorque(n);
        ///                               ExternalTorque_RightFoot(3);  ExternalForce_RightFoot(3);
        ///                               ExternalTorque_LeftFoot(3);   ExternalForce_LeftFoot(3);
        ///
        /// 1) Equation of Motion :
        ///  Equation of Motion (for jumping with linear guide)
        ///  EoM is Fundamental equation for the system
        ///  This equation is based on physical phenomenom.
        ///  So, the solution must satisfy EoM  >>>  "Equality Constraint"
        ///
        ///                                        r Jacobian of RF Contact point
        ///  M(Q)*ddQ + C(Q,dQ) + G(Q) = S*Tau + JT_RF(Q)*Fext_RF + JT_LF(Q)*Fext_LF
        ///                                 L Joint Torque
        ///  >> [M(Q) -S -JT_RF(q)]*[ddQ;Tau;Fext_RF] = -C(Q,dQ) - G(Q)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ = 0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        // 1) Equation of Motion ////////////////////////////////////////////////////////////
        temp_size_equ = size_ddQ;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);

        // Make mass matrix (M)
        MatrixNd tempM = MatrixNd::Zero(size_ddQ,size_ddQ);
        CompositeRigidBodyAlgorithm(*LIGHT.Robot,LIGHT.Qnow,tempM,false);
        // Make Selection matrix (S)
        MatrixNd tempS = MatrixNd::Zero(size_ddQ,size_JntTorque);
        tempS.block(6, 0, size_JntTorque, size_JntTorque) = MatrixNd::Identity(size_JntTorque,size_JntTorque);
        // Make nonlinear matrix (C+G)
        VectorNd tempN = VectorNd::Zero(size_ddQ,1);
        NonlinearEffects(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, tempN);

        A_temp.block(0,idx_ddQ,size_ddQ,size_ddQ) = tempM;
        A_temp.block(0,idx_JntTorque,size_ddQ,size_JntTorque) = -tempS;
        A_temp.block(0,idx_Ext_RF,size_ddQ,6) = -J_RF.transpose();
        B_temp = -tempN;
        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ;
        VectorNd B_inequ;

        // ZMP Constraints *************************
        double ZMPx_RF_min = -LIGHT.LIGHT_Info->ZMPlimit_heel;
        double ZMPx_RF_max = LIGHT.LIGHT_Info->ZMPlimit_toe;
        double ZMPy_RF_min = -LIGHT.LIGHT_Info->ZMPlimit_outside;
        double ZMPy_RF_max = LIGHT.LIGHT_Info->ZMPlimit_inside;

        // 1) Right Foot, ZMP_x_max : T(RAP) / (R * Fz) < ZMP_x_max
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAP,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = -ZMPx_RF_max*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 2) Right Foot, ZMP_x_min : T(RAP) / (R * Fz) > ZMP_x_min
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAP,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = ZMPx_RF_min*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 3) Right Foot, ZMP_y_min : T(RAR) / (R * Fz) > ZMP_y_min
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAR,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = -ZMPy_RF_max*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 4) Right Foot, ZMP_y_max : T(RAR) / (R * Fz) > ZMP_y_max
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+RAR,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_RF+3,1,3) = ZMPy_RF_min*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // Fz Constraints (Positive Force) *************************
        double Fz_min = 30.0;

        //        // 5) RightFoot Fz>=100 (Downward)
        //        temp_size_inequ = 1;
        //        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        //        A_temp.block(0,idx_Ext_RF+5,1,1) = -MatrixNd::Identity(1,1);
        //        B_temp.block(0,0,1,1) << -Fz_min;
        //        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        //        idx_inequ += temp_size_inequ;

//        // Torque Limit Constraints *************************
//        VectorNd Tmax = VectorNd::Zero(12);
//        VectorNd Tmin = VectorNd::Zero(12);

//        Tmax(RHR) = 200.0;
//        Tmin(RHR) = -200.0;
//        Tmax(RHY) = 100.0;
//        Tmin(RHY) = -100.0;
//        LIGHT.CalcTorqueLimit_Hip(RIGHTLEG,Tmax(RHP),Tmin(RHP));
//        LIGHT.CalcTorqueLimit_Knee(RIGHTLEG,Tmax(RKN),Tmin(RKN));
//        LIGHT.CalcTorqueLimit_Ankle(RIGHTLEG,Tmax(RAP),Tmin(RAP),Tmax(RAR),Tmin(RAR));

//        Tmax(LHR) = 200.0;
//        Tmin(LHR) = -200.0;
//        Tmax(LHY) = 100.0;
//        Tmin(LHY) = -100.0;
//        LIGHT.CalcTorqueLimit_Hip(LEFTLEG,Tmax(LHP),Tmin(LHP));
//        LIGHT.CalcTorqueLimit_Knee(LEFTLEG,Tmax(LKN),Tmin(LKN));
//        LIGHT.CalcTorqueLimit_Ankle(LEFTLEG,Tmax(LAP),Tmin(LAP),Tmax(LAR),Tmin(LAR));

//        // 6) Torque Upper Limit
//        temp_size_inequ = 12;
//        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//        A_temp.block(0,idx_JntTorque,12,12) = MatrixNd::Identity(12,12);
//        B_temp = Tmax;
//        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
//        idx_inequ += temp_size_inequ;

//        // 7) Torque Lower Limit
//        temp_size_inequ = 12;
//        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//        A_temp.block(0,idx_JntTorque,12,12) = -MatrixNd::Identity(12,12);
//        B_temp = -Tmin;
//        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
//        idx_inequ += temp_size_inequ;

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_Dyn.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_Dyn.make_COST(A_cost, B_cost);
        QP_Dyn.make_EQ(A_equ,B_equ);
        QP_Dyn.make_IEQ(A_inequ,B_inequ);

        ///////// Solve the problem //////////////////////////////////////////////////////////////////////

        switch(QP_Dyn.WhichSolver()) {
        case SolverIsQuadProg:
        {
            // 1. Solve.
            VectorNd QP_sol = QP_Dyn.solve_QP();
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_DOF,LIGHT_ACT_DOF);
            Ext_RF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF,6);
//            FILE_LOG(logINFO) << "Cost : ";
//            for(int i=0;i<n_cost;i++) {
//                FILE_LOG(logINFO) << "idx (" << i << ") : " << (A_cost.row(i)*QP_sol-B_cost.row(i));
//            }
            break;
        }
        case SolverIsQPSwift:
        {
            qp_int n_var = QP_Dyn.NUMCOLS;
            qp_int n_equ = QP_Dyn.NUMEQ;
            qp_int n_inequ = QP_Dyn.NUMINEQ;

            qp_int   P_nnz,A_nnz,G_nnz;
            qp_real  *P_x = NULL;
            qp_int   *P_i = NULL,*P_p = NULL;
            qp_real  *q_x = NULL;
            qp_real  *A_x = NULL;
            qp_int   *A_i = NULL, *A_p = NULL;
            qp_real  *b_x = NULL;
            qp_real  *G_x = NULL;
            qp_int   *G_i = NULL, *G_p = NULL;
            qp_real  *h_x = NULL;

            // 1-0. Convert cost function to sparse form .
            MatrixNd P = QP_Dyn.P_cost;
            VectorNd q = QP_Dyn.Q_cost;
            P_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(P);
            P_x = new qp_real[P_nnz];
            P_i = new qp_int[P_nnz];
            P_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
            q_x = new qp_real[n_var];
            QP_Dyn.ConvertVector2Array_QPswift(q,n_var,q_x);

            // 1-1. Convert equality constraint to sparse form .
            if(n_equ > 0) {
                A_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_equ);
                A_x = new qp_real[A_nnz];
                A_i = new qp_int[A_nnz];
                A_p = new qp_int[n_var+1];
                QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_equ,A_x,A_i,A_p);
                b_x = new qp_real[n_equ];
                QP_Dyn.ConvertVector2Array_QPswift(B_equ,n_equ,b_x);
            }

            // 1-2. Convert inequality constraint to sparse form .
            G_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_inequ);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_inequ,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_Dyn.ConvertVector2Array_QPswift(B_inequ,n_inequ,h_x);

            // 2. Set Parameters for solving.
            QPswift      *myQP;
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                            P_p,P_i,P_x,
                            A_p,A_i,A_x,
                            G_p,G_i,G_x,
                            q_x,h_x,b_x,
                            0.0,nullptr);
            QP_SOLVE(myQP);
            VectorNd QP_sol = VectorNd::Zero(n_var);
            QP_Dyn.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_DOF,LIGHT_ACT_DOF);
            Ext_RF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF,6);

            // 3. destruction allocated memory
            QP_CLEANUP(myQP);
            delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
            delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
            delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
            break;
        }
        default:
            FILE_LOG(logERROR) << "[Kinematics Error] QP Solver is not set!";
            return false;
        }

    } else {
        ddQ_sol = VectorNd::Zero(LIGHT_DOF);
        Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);
        Ext_RF_sol = VectorNd::Zero(6);
    }

    LIGHT.ddQref = ddQ_sol;
    LIGHT.Textref_RF = Ext_RF_sol.segment(0,3);
    LIGHT.Fextref_RF = Ext_RF_sol.segment(3,3);
    LIGHT.Textref_LF = zv;
    LIGHT.Fextref_LF = zv;
    LIGHT.Tref = Torque_sol;

    return true;
}

bool LIGHTWholeMotions::SupportControl_SSP_LF_byWBD()
{
    //    LIGHT.ContactOff_RF();
    //    LIGHT.ContactOn_LF();

    VectorNd ddQ_sol = VectorNd::Zero(LIGHT_DOF);
    VectorNd Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);
    VectorNd Ext_LF_sol = VectorNd::Zero(6);

    ////////////////////////////////////////////////////////////////////////////////////
    //// Optimization Problem Formulation (QP) ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    bool QPSolver_On = true;
    if (QPSolver_On) {

        LIGHT_QP QP_Dyn;
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable(38) : [ddQ(19) JointT(13) ExtLF(6)]
        // Equality Constraints(0) : X
        // Inequality Constraints(5) : LeftFoot_ZMP(4)+GRFz(1)

        const int size_ddQ = LIGHT_DOF; // 19
        const int size_JntTorque = LIGHT_ACT_DOF; // 13
        const int size_ExtForce = 6; // Single Support Support

        const int idx_ddQ = 0;
        const int idx_JntTorque = size_ddQ;
        const int idx_Ext_LF = idx_JntTorque + size_JntTorque;
        int n_var   = size_ddQ+size_JntTorque+size_ExtForce;

        /////// Obtain Jacobian Matrices for Formulation /////////////////////////////////////////////////////////////////

        UpdateKinematics(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, VectorNd::Zero(LIGHT_DOF));

        MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[RAR], LIGHT.rar2EE, J_RF, false);
        MatrixNd J_LF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[LAR], LIGHT.lar2EE, J_LF, false);
        MatrixNd J_Pel = MatrixNd::Zero(6,size_ddQ);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[PEL], zv, J_Pel, false);

        MatrixNd Jv_CoM = LIGHT.CalcCoMJacobian(LIGHT.Qnow);
        MatrixNd Jv_LF2CoM = MatrixNd::Zero(3,size_ddQ);
        Jv_LF2CoM.block(0,0,1,size_ddQ) = Jv_CoM.block(0,0,1,size_ddQ)-J_LF.block(3,0,1,size_ddQ);
        Jv_LF2CoM.block(1,0,1,size_ddQ) = Jv_CoM.block(1,0,1,size_ddQ)-J_LF.block(4,0,1,size_ddQ);
        Jv_LF2CoM.block(2,0,1,size_ddQ) = J_Pel.block(5,0,1,size_ddQ)-J_LF.block(5,0,1,size_ddQ);

        VectorNd ddQ_Zero = VectorNd::Zero(LIGHT_DOF);
        VectorNd dJdQ_RF = VectorNd::Zero(6);
        dJdQ_RF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[RAR], LIGHT.rar2EE, false);
        VectorNd dJdQ_LF = VectorNd::Zero(6);
        dJdQ_LF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[LAR], LIGHT.lar2EE, false);
        VectorNd dJdQ_Pel = VectorNd::Zero(6);
        dJdQ_Pel = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[PEL], zv, false);
        Vector3d dJdQv_CoM = LIGHT.CalcCoMdJdQ(LIGHT.Qnow,LIGHT.dQnow);

        Vector3d dJdQv_LF2CoM;
        dJdQv_LF2CoM(0) = dJdQv_CoM(0) - dJdQ_LF(3);
        dJdQv_LF2CoM(1) = dJdQv_CoM(1) - dJdQ_LF(4);
        if(LIGHT.CoMzIsPelz) {
            dJdQv_LF2CoM(2) = dJdQ_Pel(5) - dJdQ_LF(5);
        } else {
            dJdQv_LF2CoM(2) = dJdQv_CoM(2) - dJdQ_LF(5);
        }

        //////// Cost Functions /////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;
        Matrix3d Zeta = Matrix3d::Zero();
        Matrix3d Wn = Matrix3d::Zero();

        // 0) Foot Contact Consistent  ///////////////////////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 6;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_ddQ,6,size_ddQ) = J_LF;
        B_temp.block(0,0,6,1) = -dJdQ_LF;
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 1. Pelvis Orientation(3) //////////////////////////////////////////////////////////////////////
        {
            W_temp = 300.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_ori_LSSP(0);
            Zeta(1,1) = Zeta_body_ori_LSSP(1);
            Zeta(2,2) = Zeta_body_ori_LSSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_ori_LSSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_ori_LSSP(1);
            Wn(2,2) = 2.0*PI*Freq_body_ori_LSSP(2);
            Vector3d dW_ref = LIGHT.dWdes_Pel
                    + 2.0*Zeta*Wn*(LIGHT.Wdes_Pel-LIGHT.Wnow_Pel)
                    + Wn*Wn*OrientationError(LIGHT.Rdes_Pel,LIGHT.Rnow_Pel);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_Pel.block(0,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (dW_ref-dJdQ_Pel.segment(0,3));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 2. LF to CoM Position (3) /////////////////////////////////////////////////////////////////////
        {
            W_temp = 300.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_pos_LSSP(0);
            Zeta(1,1) = Zeta_body_pos_LSSP(1);
            Zeta(2,2) = Zeta_body_pos_LSSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_pos_LSSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_pos_LSSP(1);
            Wn(2,2) = 2.0*PI*Freq_body_pos_LSSP(2);
            Vector3d A_ref;
            A_ref(0) = LIGHT.ddXdes_LF2CoM(0) + 2.0*Zeta(0,0)*Wn(0,0)*(LIGHT.dXdes_LF2CoM(0)-LIGHT.dXnow_LF2CoM(0)) + Wn(0,0)*Wn(0,0)*(LIGHT.Xdes_LF2CoM(0)-LIGHT.Xnow_LF2CoM(0));
            A_ref(1) = LIGHT.ddXdes_LF2CoM(1) + 2.0*Zeta(1,1)*Wn(1,1)*(LIGHT.dXdes_LF2CoM(1)-LIGHT.dXnow_LF2CoM(1)) + Wn(1,1)*Wn(1,1)*(LIGHT.Xdes_LF2CoM(1)-LIGHT.Xnow_LF2CoM(1));
            A_ref(2) = LIGHT.ddXdes_LF2CoM(2) + 2.0*Zeta(2,2)*Wn(2,2)*(LIGHT.dXdes_LF2CoM(2)-LIGHT.dXnow_LF2CoM(2)) + Wn(2,2)*Wn(2,2)*(LIGHT.Xdes_LF2CoM(2)-LIGHT.Xnow_LF2CoM(2));

            A_temp.block(0,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(0);
            A_temp.block(1,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(1);
            A_temp.block(2,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(2);
            B_temp(0,0) = (A_ref(0)-dJdQv_LF2CoM(0));
            B_temp(1,0) = (A_ref(1)-dJdQv_LF2CoM(1));
            B_temp(2,0) = (A_ref(2)-dJdQv_LF2CoM(2));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 3. RF Orientation(3) //////////////////////////////////////////////////////////////////////
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_ori_LSSP(0);
            Zeta(1,1) = Zeta_foot_ori_LSSP(1);
            Zeta(2,2) = Zeta_foot_ori_LSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_LSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_LSSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_ori_LSSP(2);
            Vector3d dW_ref = LIGHT.dWdes_RF
                    + 2.0*Zeta*Wn*(LIGHT.Wdes_RF-LIGHT.Wnow_RF)
                    + Wn*Wn*OrientationError(LIGHT.Rdes_RF,LIGHT.Rnow_RF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_RF.block(0,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (dW_ref-dJdQ_RF.segment(0,3));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 4. LF2RF Position (3) /////////////////////////////////////////////////////////////////////
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_pos_LSSP(0);
            Zeta(1,1) = Zeta_foot_pos_LSSP(1);
            Zeta(2,2) = Zeta_foot_pos_LSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_LSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_LSSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_LSSP(2);
            Vector3d A_ref = LIGHT.ddXdes_LF2RF
                    + 2.0*Zeta*Wn*(LIGHT.dXdes_LF2RF-LIGHT.dXnow_LF2RF)
                    + Wn*Wn*(LIGHT.Xdes_LF2RF-LIGHT.Xnow_LF2RF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_RF.block(3,0,3,size_ddQ)-J_LF.block(3,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (A_ref-(dJdQ_RF.segment(3,3)-dJdQ_LF.segment(3,3)));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 5. Acceleration Minimization ///////////////////////////////////////
        W_temp = 0.1;
        temp_size_cost = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ+FLOATING_BASE_DOF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        A_temp(RHY,idx_ddQ+FLOATING_BASE_DOF+RHY) = 100.0;
        A_temp(LHY,idx_ddQ+FLOATING_BASE_DOF+LHY) = 100.0;
        A_temp(WST,idx_ddQ+FLOATING_BASE_DOF+WST) = 100.0;
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 6. Torque Minimization /////////////////0//////////////////////
        W_temp = 0.01;
        temp_size_cost = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        A_temp(RHY,idx_JntTorque+RHY) = 100.0;
        A_temp(LHY,idx_JntTorque+LHY) = 100.0;
        A_temp(WST,idx_JntTorque+WST) = 100.0;
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        B_temp(RHR) = -23.0;  // LSSP Right HipRoll Torque
        B_temp(LHR) = 65.0;   // LSSP Left HipRoll Torque
        B_temp(LHP) = LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // LSSP Right HipPich Torque
        B_temp(LKN) = -116.0; // LSSP Left Knee Torque
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

//        // 6-2. Hip Pitch Equalization ///////////////////////////////////////
//        W_temp = 0.1;
//        temp_size_cost = 1;
//        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
//        A_temp(0,idx_JntTorque+RHP) = 1.0;
//        A_temp(0,idx_JntTorque+LHP) = 1.0;
//        B_temp(0) = LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // LSSP Right HipPich Torque
//        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
//        idx_cost += temp_size_cost;

        // 7. External Force/Torque Minimization
        W_temp = 0.01;
        temp_size_cost = 6;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_Ext_LF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        A_temp(2,idx_Ext_LF+2) = 100.0;
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        B_temp(5) = LIGHT.LIGHT_Info->m_robot*g_const; // LFz
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 8. Torque Solution Change Minimization
        W_temp = 0.005;
        temp_size_cost = size_JntTorque;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_JntTorque,size_JntTorque,size_JntTorque) = MatrixNd::Identity(size_JntTorque,size_JntTorque);
        B_temp.block(0,0,size_JntTorque,1) = LIGHT.Tref + LIGHT.Tref_Comp;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 9. External Torque/Force Change Minimization
        W_temp = 0.005;
        temp_size_cost = size_ExtForce;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_Ext_LF,size_ExtForce,size_ExtForce) = MatrixNd::Identity(size_ExtForce,size_ExtForce);
        B_temp.block(0,0,3,1) = LIGHT.Textref_LF;
        B_temp.block(3,0,3,1) = LIGHT.Fextref_LF;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        n_cost = idx_cost;

        //        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
        //        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;


        //////// Equality Constraints (Aq=B) /////////////////////////////////////////////////////////////////
        /// Optimization Variables : U = [ddQ(n+6);
        ///                               JointTorque(n);
        ///                               ExternalTorque_RightFoot(3);  ExternalForce_RightFoot(3);
        ///                               ExternalTorque_LeftFoot(3);   ExternalForce_LeftFoot(3);
        ///
        /// 1) Equation of Motion :
        ///  Equation of Motion (for jumping with linear guide)
        ///  EoM is Fundamental equation for the system
        ///  This equation is based on physical phenomenom.
        ///  So, the solution must satisfy EoM  >>>  "Equality Constraint"
        ///
        ///                                        r Jacobian of RF Contact point
        ///  M(Q)*ddQ + C(Q,dQ) + G(Q) = S*Tau + JT_RF(Q)*Fext_RF + JT_LF(Q)*Fext_LF
        ///                                 L Joint Torque
        ///  >> [M(Q) -S -JT_RF(q)]*[ddQ;Tau;Fext_RF] = -C(Q,dQ) - G(Q)
        ///
        /// 2) Foot Contact Consistent :
        /// dx_foot = J*dQ = 0
        /// >> ddx_foot = J_foot*ddQ + dJ_foot*dQ = 0
        /// >> [J_foot 0 0 0]*[ddQ;Tau;Fext_RF;Fext_LF] = -dJ_foot*dq
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ = 0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        // 0-1) Equation of Motion ////////////////////////////////////////////////////////////
        temp_size_equ = size_ddQ;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);

        // Make mass matrix (M)
        MatrixNd tempM = MatrixNd::Zero(size_ddQ,size_ddQ);
        CompositeRigidBodyAlgorithm(*LIGHT.Robot,LIGHT.Qnow,tempM,false);
        // Make Selection matrix (S)
        MatrixNd tempS = MatrixNd::Zero(size_ddQ,size_JntTorque);
        tempS.block(6, 0, size_JntTorque, size_JntTorque) = MatrixNd::Identity(size_JntTorque,size_JntTorque);
        // Make nonlinear matrix (C+G)
        VectorNd tempN = VectorNd::Zero(size_ddQ,1);
        NonlinearEffects(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, tempN);

        A_temp.block(0,idx_ddQ,size_ddQ,size_ddQ) = tempM;
        A_temp.block(0,idx_JntTorque,size_ddQ,size_JntTorque) = -tempS;
        A_temp.block(0,idx_Ext_LF,size_ddQ,6) = -J_LF.transpose();
        B_temp = -tempN;
        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ;
        VectorNd B_inequ;

        // ZMP Constraints *************************
        double ZMPx_LF_min = -LIGHT.LIGHT_Info->ZMPlimit_heel;
        double ZMPx_LF_max = LIGHT.LIGHT_Info->ZMPlimit_toe;
        double ZMPy_LF_min = -LIGHT.LIGHT_Info->ZMPlimit_inside;
        double ZMPy_LF_max = LIGHT.LIGHT_Info->ZMPlimit_outside;

        // 1) Left Foot, ZMP_x_max : T(LAP) / (R * Fz) < ZMP_x_max
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAP,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = -ZMPx_LF_max*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 2) Left Foot, ZMP_x_min : T(LAP) / (R * Fz) > ZMP_x_min
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAP,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = ZMPx_LF_min*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 3) Left Foot, ZMP_y_min : T(LAR) / (R * Fz) > ZMP_y_min
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAR,1,1) = -MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = -ZMPy_LF_max*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 4) Left Foot, ZMP_y_max : T(LAR) / (R * Fz) > ZMP_y_max
        temp_size_inequ = 1;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_JntTorque+LAR,1,1) = MatrixNd::Identity(1,1);
        A_temp.block(0,idx_Ext_LF+3,1,3) = ZMPy_LF_min*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
        B_temp = VectorNd::Zero(1);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // Fz Constraints (Positive Force) *************************
        double Fz_min = 30.0;

        //        // 5) LeftFoot Fz>=100N (Downward)
        //        temp_size_inequ = 1;
        //        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        //        A_temp.block(0,idx_Ext_LF+5,1,1) = -MatrixNd::Identity(1,1);
        //        B_temp.block(0,0,1,1) << -Fz_min;
        //        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        //        idx_inequ += temp_size_inequ;

//        // Torque Limit Constraints *************************
//        VectorNd Tmax = VectorNd::Zero(12);
//        VectorNd Tmin = VectorNd::Zero(12);

//        Tmax(RHR) = 200.0;
//        Tmin(RHR) = -200.0;
//        Tmax(RHY) = 100.0;
//        Tmin(RHY) = -100.0;
//        LIGHT.CalcTorqueLimit_Hip(RIGHTLEG,Tmax(RHP),Tmin(RHP));
//        LIGHT.CalcTorqueLimit_Knee(RIGHTLEG,Tmax(RKN),Tmin(RKN));
//        LIGHT.CalcTorqueLimit_Ankle(RIGHTLEG,Tmax(RAP),Tmin(RAP),Tmax(RAR),Tmin(RAR));

//        Tmax(LHR) = 200.0;
//        Tmin(LHR) = -200.0;
//        Tmax(LHY) = 100.0;
//        Tmin(LHY) = -100.0;
//        LIGHT.CalcTorqueLimit_Hip(LEFTLEG,Tmax(LHP),Tmin(LHP));
//        LIGHT.CalcTorqueLimit_Knee(LEFTLEG,Tmax(LKN),Tmin(LKN));
//        LIGHT.CalcTorqueLimit_Ankle(LEFTLEG,Tmax(LAP),Tmin(LAP),Tmax(LAR),Tmin(LAR));

//        // 6) Torque Upper Limit
//        temp_size_inequ = 12;
//        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//        A_temp.block(0,idx_JntTorque,12,12) = MatrixNd::Identity(12,12);
//        B_temp = Tmax;
//        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
//        idx_inequ += temp_size_inequ;

//        // 7) Torque Lower Limit
//        temp_size_inequ = 12;
//        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//        A_temp.block(0,idx_JntTorque,12,12) = -MatrixNd::Identity(12,12);
//        B_temp = -Tmin;
//        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
//        idx_inequ += temp_size_inequ;

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_Dyn.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_Dyn.make_COST(A_cost, B_cost);
        QP_Dyn.make_EQ(A_equ,B_equ);
        QP_Dyn.make_IEQ(A_inequ,B_inequ);

        ////////// Solve the problem ////////////////////////////////////////////////////////////////

        switch(QP_Dyn.WhichSolver()) {
        case SolverIsQuadProg:
        {
            // 1. Solve.
            VectorNd QP_sol = QP_Dyn.solve_QP();
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_DOF,LIGHT_ACT_DOF);
            Ext_LF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF,6);
            break;
        }
        case SolverIsQPSwift:
        {
            qp_int n_var = QP_Dyn.NUMCOLS;
            qp_int n_equ = QP_Dyn.NUMEQ;
            qp_int n_inequ = QP_Dyn.NUMINEQ;

            qp_int   P_nnz,A_nnz,G_nnz;
            qp_real  *P_x = NULL;
            qp_int   *P_i = NULL,*P_p = NULL;
            qp_real  *q_x = NULL;
            qp_real  *A_x = NULL;
            qp_int   *A_i = NULL, *A_p = NULL;
            qp_real  *b_x = NULL;
            qp_real  *G_x = NULL;
            qp_int   *G_i = NULL, *G_p = NULL;
            qp_real  *h_x = NULL;

            // 1-0. Convert cost function to sparse form .
            MatrixNd P = QP_Dyn.P_cost;
            VectorNd q = QP_Dyn.Q_cost;
            P_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(P);
            P_x = new qp_real[P_nnz];
            P_i = new qp_int[P_nnz];
            P_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
            q_x = new qp_real[n_var];
            QP_Dyn.ConvertVector2Array_QPswift(q,n_var,q_x);

            // 1-1. Convert equality constraint to sparse form .
            if(n_equ > 0) {
                A_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_equ);
                A_x = new qp_real[A_nnz];
                A_i = new qp_int[A_nnz];
                A_p = new qp_int[n_var+1];
                QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_equ,A_x,A_i,A_p);
                b_x = new qp_real[n_equ];
                QP_Dyn.ConvertVector2Array_QPswift(B_equ,n_equ,b_x);
            }

            // 1-2. Convert inequality constraint to sparse form .
            G_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_inequ);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_inequ,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_Dyn.ConvertVector2Array_QPswift(B_inequ,n_inequ,h_x);

            // 2. Set Parameters and solve.
            QPswift      *myQP;
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                            P_p,P_i,P_x,
                            A_p,A_i,A_x,
                            G_p,G_i,G_x,
                            q_x,h_x,b_x,
                            0.0,nullptr);
            QP_SOLVE(myQP);
            VectorNd QP_sol = VectorNd::Zero(n_var);
            QP_Dyn.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_DOF,LIGHT_ACT_DOF);
            Ext_LF_sol = QP_sol.segment(LIGHT_DOF+LIGHT_ACT_DOF,6);

            // 3. destruction allocated memory
            QP_CLEANUP(myQP);
            delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
            delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
            delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
            break;
        }
        default:
            FILE_LOG(logERROR) << "[Kinematics Error] QP Solver is not set!";
            return false;
        }

    } else {
        ddQ_sol = VectorNd::Zero(LIGHT_DOF);
        Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);
        Ext_LF_sol = VectorNd::Zero(6);
    }

    LIGHT.ddQref = ddQ_sol;
    LIGHT.Textref_RF = zv;
    LIGHT.Fextref_RF = zv;
    LIGHT.Textref_LF = Ext_LF_sol.segment(0,3);
    LIGHT.Fextref_LF = Ext_LF_sol.segment(3,3);
    LIGHT.Tref = Torque_sol;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// Foot Operational Space Impedance Control
////////////////////////////////////////////////////////////////////////////////////////////////////

bool LIGHTWholeMotions::SupportControl_Float_byWBD()
{
    //    LIGHT.ContactOff_RF();
    //    LIGHT.ContactOff_LF();

    VectorNd ddQ_sol = VectorNd::Zero(LIGHT_ACT_DOF);
    VectorNd Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);

    ////////////////////////////////////////////////////////////////////////////////////
    //// Optimization Problem Formulation (QP) ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    bool QPSolver_On = true;
    if (QPSolver_On) {

        LIGHT_QP QP_Dyn(SolverIsQPSwift);
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable(26) : [ddQ(13) JointT(13)] (Not Floating Base!)
        // Equality Constraints(0) : X
        // Inequality Constraints(13) : TorqueLimit(13)

        const int size_ddQ = LIGHT_ACT_DOF; // 13
        const int size_JntTorque = LIGHT_ACT_DOF; // 13

        const int idx_ddQ = 0;
        const int idx_JntTorque = size_ddQ;
        int n_var   = size_ddQ+size_JntTorque;

        /////// Obtain Jacobian Matrices for Formulation /////////////////////////////////////////////////////////////////
        VectorNd Qnow_F = LIGHT.Qnow.segment(6,LIGHT_ACT_DOF);
        VectorNd dQnow_F = LIGHT.dQnow.segment(6,LIGHT_ACT_DOF);
        UpdateKinematics(*LIGHT.Robot_FixedBase, Qnow_F, dQnow_F, VectorNd::Zero(LIGHT_ACT_DOF));

        MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_ACT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot_FixedBase, Qnow_F, LIGHT.n_link_F[RAR], LIGHT.rar2EE, J_RF, false);
        MatrixNd J_LF = MatrixNd::Zero(6,LIGHT_ACT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot_FixedBase, Qnow_F, LIGHT.n_link_F[LAR], LIGHT.lar2EE, J_LF, false);

        VectorNd ddQ_Zero = VectorNd::Zero(LIGHT_ACT_DOF);
        VectorNd dJdQ_RF = VectorNd::Zero(6);
        dJdQ_RF = CalcPointAcceleration6D(*LIGHT.Robot_FixedBase, Qnow_F, dQnow_F, ddQ_Zero, LIGHT.n_link_F[RAR], LIGHT.rar2EE, false);
        VectorNd dJdQ_LF = VectorNd::Zero(6);
        dJdQ_LF = CalcPointAcceleration6D(*LIGHT.Robot_FixedBase, Qnow_F, dQnow_F, ddQ_Zero, LIGHT.n_link_F[LAR], LIGHT.lar2EE, false);

        //////// Cost Functions /////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;
        Matrix3d Zeta = Matrix3d::Zero();
        Matrix3d Wn = Matrix3d::Zero();

        // 1. Pelvis to RF Orientation (3) /////////////////////////////////////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        Zeta(0,0) = Zeta_foot_ori_Float(0);
        Zeta(1,1) = Zeta_foot_ori_Float(1);
        Zeta(2,2) = Zeta_foot_ori_Float(2);
        Wn(0,0) = 2.0*PI*Freq_foot_ori_Float(0);
        Wn(1,1) = 2.0*PI*Freq_foot_ori_Float(1);
        Wn(2,2) = 2.0*PI*Freq_foot_ori_Float(2);
        Vector3d dW_ref = zv
                + 2.0*Zeta*Wn*(zv-LIGHT.Wnow_Pel2RF)
                + Wn*Wn*OrientationError(I3,LIGHT.Rnow_Pel2RF);

        A_temp.block(0,idx_ddQ,3,size_ddQ) = J_RF.block(0,0,3,size_ddQ);
        B_temp.block(0,0,3,1) = (dW_ref-dJdQ_RF.segment(0,3));

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 2. Pelvis to RF Position (3) /////////////////////////////////////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        Zeta(0,0) = Zeta_foot_pos_Float(0);
        Zeta(1,1) = Zeta_foot_pos_Float(1);
        Zeta(2,2) = Zeta_foot_pos_Float(2);
        Wn(0,0) = 2.0*PI*Freq_foot_pos_Float(0);
        Wn(1,1) = 2.0*PI*Freq_foot_pos_Float(1);
        Wn(2,2) = 2.0*PI*Freq_foot_pos_Float(2);
        Vector3d A_ref = LIGHT.ddXref_Pel2RF
                + 2.0*Zeta*Wn*(LIGHT.dXref_Pel2RF-LIGHT.dXnow_Pel2RF)
                + Wn*Wn*(LIGHT.Xref_Pel2RF-LIGHT.Xnow_Pel2RF);

        A_temp.block(0,idx_ddQ,3,size_ddQ) = J_RF.block(3,0,3,size_ddQ);
        B_temp.block(0,0,3,1) = (A_ref-dJdQ_RF.segment(3,3));

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 3. Pelvis to LF Orientation (3) //////////////////////////////////////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        Zeta(0,0) = Zeta_foot_ori_Float(0);
        Zeta(1,1) = Zeta_foot_ori_Float(1);
        Zeta(2,2) = Zeta_foot_ori_Float(2);
        Wn(0,0) = 2.0*PI*Freq_foot_ori_Float(0);
        Wn(1,1) = 2.0*PI*Freq_foot_ori_Float(1);
        Wn(2,2) = 2.0*PI*Freq_foot_ori_Float(2);
        dW_ref = zv
                + 2.0*Zeta*Wn*(zv-LIGHT.Wnow_Pel2LF)
                + Wn*Wn*OrientationError(I3,LIGHT.Rnow_Pel2LF);

        A_temp.block(0,idx_ddQ,3,size_ddQ) = J_LF.block(0,0,3,size_ddQ);
        B_temp.block(0,0,3,1) = (dW_ref-dJdQ_LF.segment(0,3));

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 4. Pelvis to LF Position (3) /////////////////////////////////////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        Zeta(0,0) = Zeta_foot_pos_Float(0);
        Zeta(1,1) = Zeta_foot_pos_Float(1);
        Zeta(2,2) = Zeta_foot_pos_Float(2);
        Wn(0,0) = 2.0*PI*Freq_foot_pos_Float(0);
        Wn(1,1) = 2.0*PI*Freq_foot_pos_Float(1);
        Wn(2,2) = 2.0*PI*Freq_foot_pos_Float(2);
        A_ref = LIGHT.ddXref_Pel2LF
                + 2.0*Zeta*Wn*(LIGHT.dXref_Pel2LF-LIGHT.dXnow_Pel2LF)
                + Wn*Wn*(LIGHT.Xref_Pel2LF-LIGHT.Xnow_Pel2LF);

        A_temp.block(0,idx_ddQ,3,size_ddQ) = J_LF.block(3,0,3,size_ddQ);
        B_temp.block(0,0,3,1) = (A_ref-dJdQ_LF.segment(3,3));

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 5. Waist Acceleration and Waist,Hip Torque Minimization ///////////////////////////////////////
        W_temp = 100.0;
        temp_size_cost = 2;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp(0,idx_ddQ+WST) = 1.0;
        A_temp(1,idx_JntTorque+WST) = 1.0;
        B_temp.block(0,0,2,1) = VectorNd::Zero(2);

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 6. Torque Solution Change Minimization
        W_temp = 1e-1;
        temp_size_cost = size_JntTorque;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        A_temp.block(0,idx_JntTorque,size_JntTorque,size_JntTorque) = MatrixNd::Identity(size_JntTorque,size_JntTorque);
        B_temp.block(0,0,size_JntTorque,1) = LIGHT.Tref;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        n_cost = idx_cost;

//        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
//        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

        //////// Equality Constraints (Aq=B) /////////////////////////////////////////////////////////////////
        /// Optimization Variables : U = [ddQ(n+6);
        ///                               JointTorque(n);
        ///                               ExternalTorque_RightFoot(3);  ExternalForce_RightFoot(3);
        ///                               ExternalTorque_LeftFoot(3);   ExternalForce_LeftFoot(3);
        ///
        /// 1) Equation of Motion :
        ///  Equation of Motion (for jumping with linear guide)
        ///  EoM is Fundamental equation for the system
        ///  This equation is based on physical phenomenom.
        ///  So, the solution must satisfy EoM  >>>  "Equality Constraint"
        ///
        ///  M(Q)*ddQ + C(Q,dQ) + G(Q) = S*Tau + J^T*F_ext
        ///                                 L Joint Torque
        ///  >> [M(Q) -S]*[ddQ;Tau] = -C(Q,dQ) - G(Q) + J^T*F_ext
        ///
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ = 0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        // 1) Equation of Motion ////////////////////////////////////////////////////////////
        temp_size_equ = size_ddQ;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);

        // Make mass matrix (M)
        MatrixNd tempM = MatrixNd::Zero(size_ddQ,size_ddQ);
        CompositeRigidBodyAlgorithm(*LIGHT.Robot_FixedBase,Qnow_F,tempM,false);
        // Make Selection matrix (S)
        MatrixNd tempS = MatrixNd::Identity(size_ddQ,size_JntTorque);
        // Make nonlinear matrix (C+G)
        VectorNd tempN = VectorNd::Zero(size_ddQ,1);
        NonlinearEffects(*LIGHT.Robot_FixedBase,Qnow_F,dQnow_F,tempN);

        A_temp.block(0,idx_ddQ,size_ddQ,size_ddQ) = tempM;
        A_temp.block(0,idx_JntTorque,size_ddQ,size_JntTorque) = -tempS;
        VectorNd ExtForce_RF = VectorNd::Zero(6);
        ExtForce_RF.segment(0,3) = LIGHT.Textdes_RF;
        ExtForce_RF.segment(3,3) = LIGHT.Fextdes_RF;
        VectorNd ExtForce_LF = VectorNd::Zero(6);
        ExtForce_LF.segment(0,3) = LIGHT.Textdes_LF;
        ExtForce_LF.segment(3,3) = LIGHT.Fextdes_LF;
        B_temp = -tempN + J_RF.transpose()*ExtForce_RF + J_LF.transpose()*ExtForce_LF;
        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////////////
        ///
        ///
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ = MatrixNd::Zero(n_inequ,n_var);
        VectorNd B_inequ = MatrixNd::Zero(n_inequ,1);
        MatrixNd temp_Ai;
        VectorNd temp_Bi;

        //        // 1) Joint Velocity Limit (angle, velocity, accelration)
        //        temp_size_inequ = 2*n_var;
        //        Resize_TempMatrix(temp_size_inequ,n_var,temp_Ai,temp_Bi);

        //        temp_Ai.block(0,0,n_var,n_var) = -MatrixNd::Identity(n_var,n_var);
        //        temp_Ai.block(n_var,0,n_var,n_var) = MatrixNd::Identity(n_var,n_var);
        //        temp_Bi.block(0,0,n_var,1) = 1e+4*VectorNd::Ones(n_var);
        //        temp_Bi.block(n_var,0,n_var,1) = 1e+4*VectorNd::Ones(n_var);

        //        Matrix4QP_Addition(A_inequ,B_inequ,temp_Ai,temp_Bi);
        //        idx_inequ += temp_size_inequ;

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_Dyn.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_Dyn.make_COST(A_cost, B_cost);
        QP_Dyn.make_EQ(A_equ,B_equ);
        QP_Dyn.make_IEQ(A_inequ,B_inequ);

        ///////// Solve the problem //////////////////////////////////////////////////////////////////////

        switch(QP_Dyn.WhichSolver()) {
        case SolverIsQuadProg:
        {
            // 1. Solve.
            VectorNd QP_sol = QP_Dyn.solve_QP();
            ddQ_sol = QP_sol.segment(0,LIGHT_ACT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_ACT_DOF,LIGHT_ACT_DOF);

            break;
        }
        case SolverIsQPSwift:
        {
            qp_int n_var = QP_Dyn.NUMCOLS;
            qp_int n_equ = QP_Dyn.NUMEQ;
            qp_int n_inequ = QP_Dyn.NUMINEQ;

            qp_int   P_nnz,A_nnz,G_nnz;
            qp_real  *P_x = NULL;
            qp_int   *P_i = NULL,*P_p = NULL;
            qp_real  *q_x = NULL;
            qp_real  *A_x = NULL;
            qp_int   *A_i = NULL, *A_p = NULL;
            qp_real  *b_x = NULL;
            qp_real  *G_x = NULL;
            qp_int   *G_i = NULL, *G_p = NULL;
            qp_real  *h_x = NULL;

            // 1-0. Convert cost function to sparse form .
            MatrixNd P = QP_Dyn.P_cost;
            VectorNd q = QP_Dyn.Q_cost;
            P_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(P);
            P_x = new qp_real[P_nnz];
            P_i = new qp_int[P_nnz];
            P_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
            q_x = new qp_real[n_var];
            QP_Dyn.ConvertVector2Array_QPswift(q,n_var,q_x);

            // 1-1. Convert equality constraint to sparse form .
            if(n_equ > 0) {
                A_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_equ);
                A_x = new qp_real[A_nnz];
                A_i = new qp_int[A_nnz];
                A_p = new qp_int[n_var+1];
                QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_equ,A_x,A_i,A_p);
                b_x = new qp_real[n_equ];
                QP_Dyn.ConvertVector2Array_QPswift(B_equ,n_equ,b_x);
            }

            // 1-2. Convert inequality constraint to sparse form .
            G_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_inequ);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_inequ,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_Dyn.ConvertVector2Array_QPswift(B_inequ,n_inequ,h_x);

            // 2. Set Parameters and solve.
            QPswift      *myQP;
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                            P_p,P_i,P_x,
                            A_p,A_i,A_x,
                            G_p,G_i,G_x,
                            q_x,h_x,b_x,
                            0.0,nullptr);
            QP_SOLVE(myQP);
            VectorNd QP_sol = VectorNd::Zero(n_var);
            QP_Dyn.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
            ddQ_sol = QP_sol.segment(0,LIGHT_ACT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_ACT_DOF,LIGHT_ACT_DOF);

            //            if (myQP->stats->Flag == QP_OPTIMAL){
            //                PRINT("\nOptimal Solution Found\n");
            //                PRINT("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup)*1000.0);
            //                PRINT("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
            //                PRINT("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
            //                PRINT("Iterations     : %d\n\n", myQP->stats->IterationCount);
            //            }
            //            if (myQP->stats->Flag == QP_MAXIT){
            //                PRINT("\nMaximum Iterations reached\n");
            //                PRINT("Solve Time     : %f ms\n", myQP->stats->tsolve*1000.0);
            //                PRINT("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
            //                PRINT("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
            //                PRINT("Iterations     : %d\n\n", myQP->stats->IterationCount);
            //            }

            //            if (myQP->stats->Flag == QP_FATAL){
            //                PRINT("\nUnknown Error Detected\n\n");
            //            }

            //            if (myQP->stats->Flag == QP_KKTFAIL){
            //                PRINT("\nLDL Factorization fail\n\n");
            //            }

            //            std::cout << "QP solution"  << std::endl << QP_sol.transpose()<< std::endl;
            //            std::cout << "Analytic solution"  << std::endl << (-(QP_Dyn.P_cost+MatrixNd::Identity(n_var,n_var)*1e-6).inverse()*QP_Dyn.Q_cost).transpose()<< std::endl;


            // 3. destruction allocated memory
            QP_CLEANUP(myQP);
            delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
            delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
            delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
            break;
        }
        case SolverIsAnalytic:
        {
            // 1. Solve the problem.
            VectorNd QP_sol = -pseudoInverse(QP_Dyn.P_cost)*QP_Dyn.Q_cost;
            ddQ_sol = QP_sol.segment(0,LIGHT_ACT_DOF);
            Torque_sol = QP_sol.segment(LIGHT_ACT_DOF,LIGHT_ACT_DOF);
            break;
        }
        default:
            FILE_LOG(logERROR) << "[Kinematics Error] QP Solver is not set!";
            return false;
        }

    } else {
        VectorNd Qnow_F = LIGHT.Qnow.segment(6,LIGHT_ACT_DOF);
        VectorNd dQnow_F = LIGHT.dQnow.segment(6,LIGHT_ACT_DOF);
        UpdateKinematics(*LIGHT.Robot_FixedBase, Qnow_F, dQnow_F, VectorNd::Zero(LIGHT_ACT_DOF));

        // Make nonlinear matrix (C+G)
        VectorNd tempN = VectorNd::Zero(LIGHT_ACT_DOF,1);
        NonlinearEffects(*LIGHT.Robot_FixedBase,Qnow_F,dQnow_F,tempN);

        ddQ_sol = VectorNd::Zero(LIGHT_ACT_DOF);
        Torque_sol = tempN; // Gravity Compensation
    }

    LIGHT.ddQref.segment(FLOATING_BASE_DOF,LIGHT_ACT_DOF) = ddQ_sol;
    LIGHT.Textref_RF = zv;
    LIGHT.Fextref_RF = zv;
    LIGHT.Textref_LF = zv;
    LIGHT.Fextref_LF = zv;
    LIGHT.Tref = Torque_sol;

    return true;
}





////////////////////////////////////////////////////////////////////////////////////////////////////
/// Whole-body Dynamics is
////////////////////////////////////////////////////////////////////////////////////////////////////

bool LIGHTWholeMotions::SupportControl_Integrated_byWBD(int _RefFrame, bool _IsRFContact, bool _IsLFContact)
{
    // RefFrame : which foot frame is reference frame?
    //            -1 : right, 1 : left
    if(_RefFrame == LIGHT.REFFRAME_RF) {
        LIGHT.Xdes_LF2CoM = LIGHT.Xdes_RF2CoM - LIGHT.Xdes_RF2LF;
        LIGHT.dXdes_LF2CoM = LIGHT.dXdes_RF2CoM - LIGHT.dXdes_RF2LF;
    } else if (_RefFrame == LIGHT.REFFRAME_LF) {
        LIGHT.Xdes_RF2CoM = LIGHT.Xdes_LF2CoM - LIGHT.Xdes_LF2RF;
        LIGHT.dXdes_RF2CoM = LIGHT.dXdes_LF2CoM - LIGHT.dXdes_LF2RF;
    } else {
        FILE_LOG(logERROR) << "[Error] Reference frame is not set.";
        return false;
    }

    VectorNd ddQ_sol = VectorNd::Zero(LIGHT_DOF);
    VectorNd Ext_RF_sol = VectorNd::Zero(6);
    VectorNd Ext_LF_sol = VectorNd::Zero(6);
    VectorNd Torque_sol = VectorNd::Zero(LIGHT_ACT_DOF);

    ////////////////////////////////////////////////////////////////////////////////////
    //// Optimization Problem Formulation (QP) ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    bool QPSolver_On = true;
    if (QPSolver_On) {

        system_clock::time_point t_StartTime = system_clock::now();

        LIGHT_QP QP_Dyn;
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable(44) : [ddQ(19) JointT(13) ExtRF(6) ExtLF(6)]
        // Equality Constraints(0) : X
        // Inequality Constraints(10) : RightFoot_ZMP(4)+LeftFoot_ZMP(4)+GRFz(2)

        const int size_ddQ = LIGHT_DOF; // 19
        const int size_ExtForce = 2*6; // Double Support, 12
        const int size_JntTorque = LIGHT_ACT_DOF; // 13
        const int idx_ddQ = 0;
        const int idx_Ext_RF = idx_ddQ + size_ddQ;
        const int idx_Ext_LF = idx_Ext_RF+6;

        int n_var   = size_ddQ+size_ExtForce;

        /////// Obtain Jacobian Matrices for Formulation /////////////////////////////////////////////////////////////////

        UpdateKinematics(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, VectorNd::Zero(LIGHT_DOF));

        MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[RAR], LIGHT.rar2EE, J_RF, false);
        MatrixNd J_LF = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[LAR], LIGHT.lar2EE, J_LF, false);
        MatrixNd J_Pel = MatrixNd::Zero(6,LIGHT_DOF);
        CalcPointJacobian6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.n_link[PEL], zv, J_Pel, false);
        MatrixNd Jv_CoM = LIGHT.CalcCoMJacobian(LIGHT.Qnow);

        MatrixNd Jv_RF2CoM = MatrixNd::Zero(3,size_ddQ);
        Jv_RF2CoM.block(0,0,1,size_ddQ) = Jv_CoM.block(0,0,1,size_ddQ)-J_RF.block(3,0,1,size_ddQ);
        Jv_RF2CoM.block(1,0,1,size_ddQ) = Jv_CoM.block(1,0,1,size_ddQ)-J_RF.block(4,0,1,size_ddQ);
        if(LIGHT.CoMzIsPelz) {
            Jv_RF2CoM.block(2,0,1,size_ddQ) = J_Pel.block(5,0,1,size_ddQ)-J_RF.block(5,0,1,size_ddQ);
        } else {
            Jv_RF2CoM.block(2,0,1,size_ddQ) = Jv_CoM.block(2,0,1,size_ddQ)-J_RF.block(5,0,1,size_ddQ);
        }
        MatrixNd Jv_LF2CoM = MatrixNd::Zero(3,size_ddQ);
        Jv_LF2CoM.block(0,0,1,size_ddQ) = Jv_CoM.block(0,0,1,size_ddQ)-J_LF.block(3,0,1,size_ddQ);
        Jv_LF2CoM.block(1,0,1,size_ddQ) = Jv_CoM.block(1,0,1,size_ddQ)-J_LF.block(4,0,1,size_ddQ);
        if(LIGHT.CoMzIsPelz) {
            Jv_LF2CoM.block(2,0,1,size_ddQ) = J_Pel.block(5,0,1,size_ddQ)-J_LF.block(5,0,1,size_ddQ);
        } else {
            Jv_LF2CoM.block(2,0,1,size_ddQ) = Jv_CoM.block(2,0,1,size_ddQ)-J_LF.block(5,0,1,size_ddQ);
        }
        VectorNd ddQ_Zero = VectorNd::Zero(LIGHT_DOF);
        VectorNd dJdQ_RF = VectorNd::Zero(6);
        dJdQ_RF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[RAR], LIGHT.rar2EE, false);
        VectorNd dJdQ_LF = VectorNd::Zero(6);
        dJdQ_LF = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[LAR], LIGHT.lar2EE, false);
        VectorNd dJdQ_Pel = VectorNd::Zero(6);
        dJdQ_Pel = CalcPointAcceleration6D(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, ddQ_Zero, LIGHT.n_link[PEL], zv, false);
        Vector3d dJdQv_CoM = LIGHT.CalcCoMdJdQ(LIGHT.Qnow,LIGHT.dQnow);

        Vector3d dJdQv_RF2CoM;
        dJdQv_RF2CoM(0) = dJdQv_CoM(0) - dJdQ_RF(3);
        dJdQv_RF2CoM(1) = dJdQv_CoM(1) - dJdQ_RF(4);
        if(LIGHT.CoMzIsPelz) {
            dJdQv_RF2CoM(2) = dJdQ_Pel(5) - dJdQ_RF(5);
        } else {
            dJdQv_RF2CoM(2) = dJdQv_CoM(2) - dJdQ_RF(5);
        }

        Vector3d dJdQv_LF2CoM;
        dJdQv_LF2CoM(0) = dJdQv_CoM(0) - dJdQ_LF(3);
        dJdQv_LF2CoM(1) = dJdQv_CoM(1) - dJdQ_LF(4);
        if(LIGHT.CoMzIsPelz) {
            dJdQv_LF2CoM(2) = dJdQ_Pel(5) - dJdQ_LF(5);
        } else {
            dJdQv_LF2CoM(2) = dJdQv_CoM(2) - dJdQ_LF(5);
        }

        system_clock::time_point t_CalcJacob = system_clock::now();
        microseconds dt_CalcJacob = duration_cast<std::chrono::microseconds>(t_CalcJacob - t_StartTime);

        //////// Equality Constraints (Aq=B) /////////////////////////////////////////////////////////////
        /// Optimization Variables : U = [ddQ(n+6);
        ///                               ExternalTorque_RightFoot(3);  ExternalForce_RightFoot(3);
        ///                               ExternalTorque_LeftFoot(3);   ExternalForce_LeftFoot(3);
        ///
        /// 1) Equation of Motion :
        ///  Equation of Motion (for jumping with linear guide)
        ///  EoM is Fundamental equation for the system
        ///  This equation is based on physical phenomenom.
        ///  So, the solution must satisfy EoM  >>>  "Equality Constraint"
        ///
        ///                                        r Jacobian of RF Contact point
        ///  M(Q)*ddQ + C(Q,dQ) + G(Q) = S*Tau + JT_RF(Q)*Fext_RF + JT_LF(Q)*Fext_LF
        ///                                 L Joint Torque
        ///  >> [M(Q) -S -JT_RF(q) -JT_LF(q)]*[ddQ;Tau;Fext_RF;Fext_LF] = -C(Q,dQ) - G(Q)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ = 0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        // 1. Equation of Motion ////////////////////////////////////////////////////////////
        temp_size_equ = 6;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);

        // Make mass matrix (M) and decouple it
        MatrixNd tempM = MatrixNd::Zero(size_ddQ,size_ddQ);
        CompositeRigidBodyAlgorithm(*LIGHT.Robot,LIGHT.Qnow,tempM,false);
        MatrixNd Mu = tempM.block(0,0,6,size_ddQ); // Upper inertia matrix (Base)
        MatrixNd Ml = tempM.block(6,0,size_ddQ-6,size_ddQ);  // Upper inertia matrix (Joints)

        // Make nonlinear matrix (C+G) decouple it
        VectorNd tempN = VectorNd::Zero(size_ddQ,1);
        NonlinearEffects(*LIGHT.Robot, LIGHT.Qnow, LIGHT.dQnow, tempN);
        VectorNd Nu = tempN.segment(0,6);
        VectorNd Nl = tempN.segment(6,size_ddQ-6);

        // Decouple Jacobian Matrix
        MatrixNd J_RFu_T = J_RF.transpose().block(0,0,6,6);
        MatrixNd J_RFl_T = J_RF.transpose().block(6,0,size_ddQ-6,6);
        MatrixNd J_LFu_T = J_LF.transpose().block(0,0,6,6);
        MatrixNd J_LFl_T = J_LF.transpose().block(6,0,size_ddQ-6,6);

        // Make Equality Constraints
        A_temp.block(0,idx_ddQ,6,size_ddQ) = Mu;
        A_temp.block(0,idx_Ext_RF,6,6) = J_RFu_T;
        A_temp.block(0,idx_Ext_LF,6,6) = J_LFu_T;
        B_temp = -Nu;

        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        system_clock::time_point t_DefEquCon = system_clock::now();
        microseconds dt_DefEquCon = duration_cast<std::chrono::microseconds>(t_DefEquCon - t_CalcJacob);

        //////// Cost Functions ////////////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;
        Matrix3d Zeta = Matrix3d::Zero();
        Matrix3d Wn = Matrix3d::Zero();
        Vector3d dW_ref, A_ref;

        // 0-1. Foot Contact Consistent (RF) ///////////////////////////////////////////////////////
        if(_IsRFContact) {
            W_temp = 100.0;
            temp_size_cost = 6;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_ori_DSP(0);
            Zeta(1,1) = Zeta_foot_ori_DSP(1);
            Zeta(2,2) = Zeta_foot_ori_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_ori_DSP(2);
            dW_ref = 2.0*Zeta*Wn*(zv-LIGHT.Wnow_RF);

            Zeta(0,0) = Zeta_foot_pos_DSP(0);
            Zeta(1,1) = Zeta_foot_pos_DSP(1);
            Zeta(2,2) = Zeta_foot_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_DSP(2);
            A_ref = 2.0*Zeta*Wn*(zv-LIGHT.dXnow_RF);

            A_temp.block(0,idx_ddQ,6,size_ddQ) = J_RF;
            B_temp.block(0,0,3,1) = dW_ref-dJdQ_RF.segment(0,3);
            B_temp.block(3,0,3,1) = A_ref-dJdQ_RF.segment(3,3);
            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 0-2. Foot Contact Consistent (LF) ///////////////////////////////////////////////////////
        if(_IsLFContact) {
            W_temp = 100.0;
            temp_size_cost = 6;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_ori_DSP(0);
            Zeta(1,1) = Zeta_foot_ori_DSP(1);
            Zeta(2,2) = Zeta_foot_ori_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_ori_DSP(2);
            dW_ref = 2.0*Zeta*Wn*(zv-LIGHT.Wnow_LF);

            Zeta(0,0) = Zeta_foot_pos_DSP(0);
            Zeta(1,1) = Zeta_foot_pos_DSP(1);
            Zeta(2,2) = Zeta_foot_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_DSP(2);
            A_ref = 2.0*Zeta*Wn*(zv-LIGHT.dXnow_LF);

            A_temp.block(0,idx_ddQ,6,size_ddQ) = J_LF;
            B_temp.block(0,0,3,1) = dW_ref-dJdQ_LF.segment(0,3);
            B_temp.block(3,0,3,1) = A_ref-dJdQ_LF.segment(3,3);
            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 1-1. Pelvis Orientation(3) //////////////////////////////////////////////////////////////////////
        W_temp = 200.0;
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

        Zeta(0,0) = Zeta_body_ori_DSP(0);
        Zeta(1,1) = Zeta_body_ori_DSP(1);
        Zeta(2,2) = Zeta_body_ori_DSP(2);
        Wn(0,0) = 2.0*PI*Freq_body_ori_DSP(0);
        Wn(1,1) = 2.0*PI*Freq_body_ori_DSP(1);
        Wn(2,2) = 2.0*PI*Freq_body_ori_DSP(2);
        dW_ref = LIGHT.dWdes_Pel
                + 2.0*Zeta*Wn*(LIGHT.Wdes_Pel-LIGHT.Wnow_Pel)
                + Wn*Wn*OrientationError(LIGHT.Rdes_Pel,LIGHT.Rnow_Pel);

        A_temp.block(0,idx_ddQ,3,size_ddQ) = J_Pel.block(0,0,3,size_ddQ);
        B_temp.block(0,0,3,1) = (dW_ref-dJdQ_Pel.segment(0,3));

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 2-1. RF to CoM Position (3) /////////////////////////////////////////////////////////////////////
        if(_IsRFContact) {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_pos_DSP(0);
            Zeta(1,1) = Zeta_body_pos_DSP(1);
            Zeta(2,2) = Zeta_body_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_body_pos_DSP(2);
            A_ref(0) = LIGHT.ddXdes_RF2CoM(0) + 2.0*Zeta(0,0)*Wn(0,0)*(LIGHT.dXdes_RF2CoM(0)-LIGHT.dXnow_RF2CoM(0)) + Wn(0,0)*Wn(0,0)*(LIGHT.Xdes_RF2CoM(0)-LIGHT.Xnow_RF2CoM(0));
            A_ref(1) = LIGHT.ddXdes_RF2CoM(1) + 2.0*Zeta(1,1)*Wn(1,1)*(LIGHT.dXdes_RF2CoM(1)-LIGHT.dXnow_RF2CoM(1)) + Wn(1,1)*Wn(1,1)*(LIGHT.Xdes_RF2CoM(1)-LIGHT.Xnow_RF2CoM(1));
            A_ref(2) = LIGHT.ddXdes_RF2CoM(2) + 2.0*Zeta(2,2)*Wn(2,2)*(LIGHT.dXdes_RF2CoM(2)-LIGHT.dXnow_RF2CoM(2)) + Wn(2,2)*Wn(2,2)*(LIGHT.Xdes_RF2CoM(2)-LIGHT.Xnow_RF2CoM(2));

            A_temp.block(0,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(0);
            A_temp.block(1,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(1);
            A_temp.block(2,idx_ddQ,1,size_ddQ) = Jv_RF2CoM.row(2);
            B_temp(0,0) = (A_ref(0)-dJdQv_RF2CoM(0));
            B_temp(1,0) = (A_ref(1)-dJdQv_RF2CoM(1));
            B_temp(2,0) = (A_ref(2)-dJdQv_RF2CoM(2));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 2-2. LF to CoM Position (3) /////////////////////////////////////////////////////////////////////
        if(_IsLFContact) {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_body_pos_DSP(0);
            Zeta(1,1) = Zeta_body_pos_DSP(1);
            Zeta(2,2) = Zeta_body_pos_DSP(2);
            Wn(0,0) = 2.0*PI*Freq_body_pos_DSP(0);
            Wn(1,1) = 2.0*PI*Freq_body_pos_DSP(1);
            Wn(2,2) = 2.0*PI*Freq_body_pos_DSP(2);
            A_ref(0) = LIGHT.ddXdes_LF2CoM(0) + 2.0*Zeta(0,0)*Wn(0,0)*(LIGHT.dXdes_LF2CoM(0)-LIGHT.dXnow_LF2CoM(0)) + Wn(0,0)*Wn(0,0)*(LIGHT.Xdes_LF2CoM(0)-LIGHT.Xnow_LF2CoM(0));
            A_ref(1) = LIGHT.ddXdes_LF2CoM(1) + 2.0*Zeta(1,1)*Wn(1,1)*(LIGHT.dXdes_LF2CoM(1)-LIGHT.dXnow_LF2CoM(1)) + Wn(1,1)*Wn(1,1)*(LIGHT.Xdes_LF2CoM(1)-LIGHT.Xnow_LF2CoM(1));
            A_ref(2) = LIGHT.ddXdes_LF2CoM(2) + 2.0*Zeta(2,2)*Wn(2,2)*(LIGHT.dXdes_LF2CoM(2)-LIGHT.dXnow_LF2CoM(2)) + Wn(2,2)*Wn(2,2)*(LIGHT.Xdes_LF2CoM(2)-LIGHT.Xnow_LF2CoM(2));

            A_temp.block(0,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(0);
            A_temp.block(1,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(1);
            A_temp.block(2,idx_ddQ,1,size_ddQ) = Jv_LF2CoM.row(2);
            B_temp(0,0) = (A_ref(0)-dJdQv_LF2CoM(0));
            B_temp(1,0) = (A_ref(1)-dJdQv_LF2CoM(1));
            B_temp(2,0) = (A_ref(2)-dJdQv_LF2CoM(2));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 3-1. LF Orientation and Position (6) //////////////////////////////////////////////////////////////////////
        if(_IsRFContact == true && _IsLFContact == false)
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_ori_RSSP(0);
            Zeta(1,1) = Zeta_foot_ori_RSSP(1);
            Zeta(2,2) = Zeta_foot_ori_RSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_RSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_RSSP(1);
            Wn(2,2) = 2.0*PI*0.5;
            dW_ref = LIGHT.dWdes_LF
                    + 2.0*Zeta*Wn*(LIGHT.Wdes_LF-LIGHT.Wnow_LF)
                    + Wn*Wn*OrientationError(LIGHT.Rdes_LF,LIGHT.Rnow_LF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_LF.block(0,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (dW_ref-dJdQ_LF.segment(0,3));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;

            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_pos_RSSP(0);
            Zeta(1,1) = Zeta_foot_pos_RSSP(1);
            Zeta(2,2) = Zeta_foot_pos_RSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_RSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_RSSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_RSSP(2);
            A_ref = LIGHT.ddXdes_RF2LF
                    + 2.0*Zeta*Wn*(LIGHT.dXdes_RF2LF-LIGHT.dXnow_RF2LF)
                    + Wn*Wn*(LIGHT.Xdes_RF2LF-LIGHT.Xnow_RF2LF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_LF.block(3,0,3,size_ddQ)-J_RF.block(3,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (A_ref-(dJdQ_LF.segment(3,3)-dJdQ_RF.segment(3,3)));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 3-2. RF Orientation and Position (6) //////////////////////////////////////////////////////////////////////
        if(_IsLFContact == true && _IsRFContact == false)
        {
            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_ori_LSSP(0);
            Zeta(1,1) = Zeta_foot_ori_LSSP(1);
            Zeta(2,2) = Zeta_foot_ori_LSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_ori_LSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_ori_LSSP(1);
            Wn(2,2) = 2.0*PI*0.5;
            dW_ref = LIGHT.dWdes_RF
                    + 2.0*Zeta*Wn*(LIGHT.Wdes_RF-LIGHT.Wnow_RF)
                    + Wn*Wn*OrientationError(LIGHT.Rdes_RF,LIGHT.Rnow_RF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_RF.block(0,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (dW_ref-dJdQ_RF.segment(0,3));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;

            W_temp = 200.0;
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            Zeta(0,0) = Zeta_foot_pos_LSSP(0);
            Zeta(1,1) = Zeta_foot_pos_LSSP(1);
            Zeta(2,2) = Zeta_foot_pos_LSSP(2);
            Wn(0,0) = 2.0*PI*Freq_foot_pos_LSSP(0);
            Wn(1,1) = 2.0*PI*Freq_foot_pos_LSSP(1);
            Wn(2,2) = 2.0*PI*Freq_foot_pos_LSSP(2);
            A_ref = LIGHT.ddXdes_LF2RF
                    + 2.0*Zeta*Wn*(LIGHT.dXdes_LF2RF-LIGHT.dXnow_LF2RF)
                    + Wn*Wn*(LIGHT.Xdes_LF2RF-LIGHT.Xnow_LF2RF);

            A_temp.block(0,idx_ddQ,3,size_ddQ) = J_RF.block(3,0,3,size_ddQ)-J_LF.block(3,0,3,size_ddQ);
            B_temp.block(0,0,3,1) = (A_ref-(dJdQ_RF.segment(3,3)-dJdQ_LF.segment(3,3)));

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 4. Acceleration Minimization
        W_temp = 20.0;
        temp_size_cost = size_ddQ;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
        B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 5. External Force/Torque Minimization
        if (_IsRFContact == true && _IsLFContact == true) { // DSP
            W_temp = 0.5;
            temp_size_cost = 12;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);

            A_temp.block(0,idx_Ext_RF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
            B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
            if(_RefFrame == LIGHT.REFFRAME_RF) {
                Vector3d Step = LIGHT.Xnow_RF2LF;
                Vector3d CoM = LIGHT.Xnow_RF2CoM;
                double m = LIGHT.LIGHT_Info->m_robot;
                double alpha = Step.dot(CoM)/Step.dot(Step);
                if(alpha>1.0) alpha = 1.0;
                else if(alpha<0.0) alpha = 0.0;
                B_temp(5) = -(1.0-alpha)*m*g_const; // RFz
                B_temp(11) = -alpha*m*g_const; // LFz
            } else {
                Vector3d Step = LIGHT.Xnow_LF2RF;
                Vector3d CoM = LIGHT.Xnow_LF2CoM;
                double m = LIGHT.LIGHT_Info->m_robot;
                double alpha = Step.dot(CoM)/Step.dot(Step);
                if(alpha>1.0) alpha = 1.0;
                else if(alpha<0.0) alpha = 0.0;
                B_temp(5) = -alpha*m*g_const; // RFz
                B_temp(11) = -(1.0-alpha)*m*g_const; // LFz
            }

            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        } else if(_IsRFContact == false && _IsLFContact == true) { // SSP_LF
            // RF : Zero Reaction Force
            W_temp = 100.0;
            temp_size_cost = 6;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_RF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
            B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
            B_temp(5) = 10.0;
            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;

            // LF : Support Body
            W_temp = 0.5;
            temp_size_cost = 6;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_LF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
            B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
            B_temp(5) = -LIGHT.LIGHT_Info->m_robot*g_const; // LFz
            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        } else if (_IsRFContact == true && _IsLFContact == false) { // SSP_RF
            // RF : Support Body
            W_temp = 0.5;
            temp_size_cost = 6;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_RF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
            B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
            B_temp(5) = -LIGHT.LIGHT_Info->m_robot*g_const; // RFz
            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;

            // LF : Zero Reaction Force
            W_temp = 100.0;
            temp_size_cost = 6;
            Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_LF,temp_size_cost,temp_size_cost) = MatrixNd::Identity(temp_size_cost,temp_size_cost);
            B_temp.block(0,0,temp_size_cost,1) = VectorNd::Zero(temp_size_cost);
            B_temp(5) = 10.0;
            Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
            idx_cost += temp_size_cost;
        }

        // 6. Joint Torque Minimization
        W_temp = 0.5;
        temp_size_cost = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ,LIGHT_ACT_DOF,size_ddQ) = Ml;
        A_temp.block(0,idx_Ext_RF,LIGHT_ACT_DOF,6) = J_RFl_T;
        A_temp.block(0,idx_Ext_LF,LIGHT_ACT_DOF,6) = J_LFl_T;
        B_temp.block(0,0,LIGHT_ACT_DOF,1) = -Nl;
        if(_IsRFContact == true && _IsLFContact == true) {
            B_temp(RHP) += 0.5*LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // DSP Right HipPich Torque
            B_temp(LHP) += 0.5*LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // DSP Left HipPich Torque
            B_temp(RKN) += -60.0; // DSP Right Knee Torque
            B_temp(LKN) += -60.0; // DSP Right Knee Torque
        } else if (_IsRFContact == true && _IsLFContact == false) {
            B_temp(RHR) = -65.0;  // RSSP Right HipRoll Torque
            B_temp(RHP) = LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // RSSP Right HipPich Torque
            B_temp(RKN) = -116.0; // RSSP Right Knee Torque
            B_temp(LHR) = 23.0;   // RSSP Left HipRoll Torque
        } else if (_IsRFContact == false && _IsLFContact == true) {
            B_temp(RHR) = -23.0;  // LSSP Right HipRoll Torque
            B_temp(LHR) = 65.0;   // LSSP Left HipRoll Torque
            B_temp(LHP) = LIGHT.LIGHT_Info->c_torso(0)*LIGHT.LIGHT_Info->m_torso*g_const;  // LSSP Right HipPich Torque
            B_temp(LKN) = -116.0; // LSSP Left Knee Torque
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 7. External Torque/Force Solution Change Minimization
        W_temp = 0.2;
        temp_size_cost = size_ExtForce;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_Ext_RF,size_ExtForce,size_ExtForce) = MatrixNd::Identity(size_ExtForce,size_ExtForce);
        B_temp.block(0,0,3,1) = LIGHT.Textref_RF;
        B_temp.block(3,0,3,1) = LIGHT.Fextref_RF;
        B_temp.block(6,0,3,1) = LIGHT.Textref_LF;
        B_temp.block(9,0,3,1) = LIGHT.Fextref_LF;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 8. Joint Torque Solution Minimization
        W_temp = 0.2;
        temp_size_cost = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ,LIGHT_ACT_DOF,size_ddQ) = Ml;
        A_temp.block(0,idx_Ext_RF,LIGHT_ACT_DOF,6) = J_RFl_T;
        A_temp.block(0,idx_Ext_LF,LIGHT_ACT_DOF,6) = J_LFl_T;
        B_temp.block(0,0,LIGHT_ACT_DOF,1) = LIGHT.Tref-Nl;

        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // ------------------------------------------------------
        // Cost Function End

        n_cost = idx_cost;

//        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
//        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

        system_clock::time_point t_DefCostFn = system_clock::now();
        microseconds dt_DefCostFn = duration_cast<std::chrono::microseconds>(t_DefCostFn - t_DefEquCon);

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ;
        VectorNd B_inequ;

        // ZMP Constraints *************************
        double ZMPx_RF_min = -LIGHT.LIGHT_Info->ZMPlimit_heel;
        double ZMPx_RF_max = LIGHT.LIGHT_Info->ZMPlimit_toe;
        double ZMPy_RF_min = -LIGHT.LIGHT_Info->ZMPlimit_outside;
        double ZMPy_RF_max = LIGHT.LIGHT_Info->ZMPlimit_inside;
        double ZMPx_LF_min = -LIGHT.LIGHT_Info->ZMPlimit_heel;
        double ZMPx_LF_max = LIGHT.LIGHT_Info->ZMPlimit_toe;
        double ZMPy_LF_min = -LIGHT.LIGHT_Info->ZMPlimit_inside;
        double ZMPy_LF_max = LIGHT.LIGHT_Info->ZMPlimit_outside;

        if(_IsRFContact) {
            // 1) Right Foot, ZMP_x_max : T(RAP) / (R * Fz) < ZMP_x_max
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_RF+0,1,3) = LIGHT.Rnow_RF.transpose().block(1,0,1,3);
            A_temp.block(0,idx_Ext_RF+3,1,3) = ZMPx_RF_max*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 2) Right Foot, ZMP_x_min : T(RAP) / (R * Fz) > ZMP_x_min
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_RF+0,1,3) = -LIGHT.Rnow_RF.transpose().block(1,0,1,3);
            A_temp.block(0,idx_Ext_RF+3,1,3) = -ZMPx_RF_min*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 3) Right Foot, ZMP_y_min : T(RAR) < -ZMP_y_min * (R * Fz)
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_RF+0,1,3) = LIGHT.Rnow_RF.transpose().block(0,0,1,3);
            A_temp.block(0,idx_Ext_RF+3,1,3) = -ZMPy_RF_min*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 4) Right Foot, ZMP_y_max : -ZMP_y_max * (R * Fz) < T(RAR)
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_RF+0,1,3) = -LIGHT.Rnow_RF.transpose().block(0,0,1,3);
            A_temp.block(0,idx_Ext_RF+3,1,3) = ZMPy_RF_max*LIGHT.Rnow_RF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        if(_IsLFContact) {
            // 5) Left Foot, ZMP_x_max : T(LAP) / (R * Fz) < ZMP_x_max
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_LF+0,1,3) = LIGHT.Rnow_LF.transpose().block(1,0,1,3);
            A_temp.block(0,idx_Ext_LF+3,1,3) = ZMPx_LF_max*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 6) Left Foot, ZMP_x_min : T(LAP) / (R * Fz) > ZMP_x_min
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_LF+0,1,3) = -LIGHT.Rnow_LF.transpose().block(1,0,1,3);
            A_temp.block(0,idx_Ext_LF+3,1,3) = -ZMPx_LF_min*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 7) Left Foot, ZMP_y_min : T(LAR) < -ZMP_y_min * (R * Fz)
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_LF+0,1,3) = LIGHT.Rnow_LF.transpose().block(0,0,1,3);
            A_temp.block(0,idx_Ext_LF+3,1,3) = -ZMPy_LF_min*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 8) Left Foot, ZMP_y_max : -ZMP_y_max * (R * Fz) < T(RAR)
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_Ext_LF+0,1,3) = -LIGHT.Rnow_LF.transpose().block(0,0,1,3);
            A_temp.block(0,idx_Ext_LF+3,1,3) = ZMPy_LF_max*LIGHT.Rnow_LF.transpose().block(2,0,1,3);
            B_temp = VectorNd::Zero(1);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        // Fz Constraints (Positive Force) *************************
        double Fz_min = -10.0;

        if(_IsRFContact) {
            // 9) RightFoot Fz<=-10N (Downward External Force)
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//            A_temp.block(0,idx_Ext_RF+5,1,1) = -MatrixNd::Identity(1,1);
            A_temp.block(0,idx_Ext_RF+3,1,3) = LIGHT.Rnow_RF.transpose().block(2,0,1,3);
            B_temp.block(0,0,1,1) << Fz_min;
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        if(_IsLFContact) {
            // 10) LeftFoot Fz>=10N (Upward External Force)
            temp_size_inequ = 1;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//            A_temp.block(0,idx_Ext_LF+5,1,1) = MatrixNd::Identity(1,1);
            A_temp.block(0,idx_Ext_LF+3,1,3) = LIGHT.Rnow_LF.transpose().block(2,0,1,3);
            B_temp.block(0,0,1,1) << Fz_min;
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        // Friction Constraints *************************
        double mu = 0.5;
        if(_IsRFContact) {
            // 11) Left Foot X-direction :  Fx_LF / Fz_LF < mu  ||  Fx_LF / Fz_LF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_RF+3) = -1.0;
            A_temp(0,idx_Ext_RF+5) = mu;
            A_temp(1,idx_Ext_RF+3) = 1.0;
            A_temp(1,idx_Ext_RF+5) = mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 12) Left Foot X-direction :  Fy_LF / Fz_LF < mu  ||  Fy_LF / Fz_LF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_RF+4) = -1.0;
            A_temp(0,idx_Ext_RF+5) = mu;
            A_temp(1,idx_Ext_RF+4) = 1.0;
            A_temp(1,idx_Ext_RF+5) = mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        if(_IsLFContact) {
            // 11) Right Foot X-direction :  Fx_RF / Fz_RF < mu  ||  Fx_RF / Fz_RF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_LF+3) = -1.0;
            A_temp(0,idx_Ext_LF+5) = mu;
            A_temp(1,idx_Ext_LF+3) = 1.0;
            A_temp(1,idx_Ext_LF+5) = mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;

            // 12) Right Foot X-direction :  Fy_RF / Fz_RF < mu  ||  Fy_RF / Fz_RF > -mu
            temp_size_inequ = 2;
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp(0,idx_Ext_LF+4) = -1.0;
            A_temp(0,idx_Ext_LF+5) = mu;
            A_temp(1,idx_Ext_LF+4) = 1.0;
            A_temp(1,idx_Ext_LF+5) = mu;
            B_temp = VectorNd::Zero(2);
            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        // Torque Limit Constraints *************************
        VectorNd Tmax = VectorNd::Zero(LIGHT_ACT_DOF);
        VectorNd Tmin = VectorNd::Zero(LIGHT_ACT_DOF);

        Tmax(RHR) = 200.0;
        Tmin(RHR) = -200.0;
        Tmax(RHY) = 100.0;
        Tmin(RHY) = -100.0;
        LIGHT.CalcTorqueLimit_Hip(RIGHTLEG,Tmax(RHP),Tmin(RHP));
        LIGHT.CalcTorqueLimit_Knee(RIGHTLEG,Tmax(RKN),Tmin(RKN));
        LIGHT.CalcTorqueLimit_Ankle(RIGHTLEG,Tmax(RAP),Tmin(RAP),Tmax(RAR),Tmin(RAR));
        Tmax(LHR) = 200.0;
        Tmin(LHR) = -200.0;
        Tmax(LHY) = 100.0;
        Tmin(LHY) = -100.0;
        LIGHT.CalcTorqueLimit_Hip(LEFTLEG,Tmax(LHP),Tmin(LHP));
        LIGHT.CalcTorqueLimit_Knee(LEFTLEG,Tmax(LKN),Tmin(LKN));
        LIGHT.CalcTorqueLimit_Ankle(LEFTLEG,Tmax(LAP),Tmin(LAP),Tmax(LAR),Tmin(LAR));
        Tmax(WST) = 50.0;
        Tmin(WST) = -50.0;

        // 13) Torque Upper Limit
        temp_size_inequ = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ,LIGHT_ACT_DOF,size_ddQ) = Ml;
        A_temp.block(0,idx_Ext_RF,LIGHT_ACT_DOF,6) = -J_RFl_T;
        A_temp.block(0,idx_Ext_LF,LIGHT_ACT_DOF,6) = -J_LFl_T;
        B_temp = Tmax - Nl;
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        // 14) Torque Lower Limit
        temp_size_inequ = LIGHT_ACT_DOF;
        Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_ddQ,LIGHT_ACT_DOF,size_ddQ) = -Ml;
        A_temp.block(0,idx_Ext_RF,LIGHT_ACT_DOF,6) = J_RFl_T;
        A_temp.block(0,idx_Ext_LF,LIGHT_ACT_DOF,6) = J_LFl_T;
        B_temp = -(Tmin - Nl);
        Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
        idx_inequ += temp_size_inequ;

        n_inequ = idx_inequ;

        system_clock::time_point t_DefInEquC = system_clock::now();
        microseconds dt_DefInEquC = duration_cast<std::chrono::microseconds>(t_DefInEquC - t_DefEquCon);

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_Dyn.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_Dyn.make_COST(A_cost, B_cost);
        QP_Dyn.make_EQ(A_equ,B_equ);
        QP_Dyn.make_IEQ(A_inequ,B_inequ);

        system_clock::time_point t_SetQPSolv = system_clock::now();
        microseconds dt_SetQPSolv = duration_cast<std::chrono::microseconds>(t_SetQPSolv - t_DefInEquC);

        ///////// Solve the problem ////////////////////////////////////////////////////////////////

        switch(QP_Dyn.WhichSolver()) {
        case SolverIsQuadProg:
        {
            // 1. Solve.
            VectorNd QP_sol = QP_Dyn.solve_QP();
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Ext_RF_sol = QP_sol.segment(LIGHT_DOF,6);
            Ext_LF_sol = QP_sol.segment(LIGHT_DOF+6,6);

            Resize_TempMatrix(LIGHT_ACT_DOF,n_var,A_temp,B_temp);
            A_temp.block(0,idx_ddQ,LIGHT_ACT_DOF,size_ddQ) = Ml;
            A_temp.block(0,idx_Ext_RF,LIGHT_ACT_DOF,6) = J_RFl_T;
            A_temp.block(0,idx_Ext_LF,LIGHT_ACT_DOF,6) = J_LFl_T;
            Torque_sol = A_temp*QP_sol + Nl;

//            FILE_LOG(logINFO) << "[QP_sol]";
////            FILE_LOG(logINFO) << "ddQ : " << ddQ_sol.transpose();
//            FILE_LOG(logINFO) << "Ext_RF_sol : " << Ext_RF_sol.transpose();
//            FILE_LOG(logINFO) << "Ext_LF_sol : " << Ext_LF_sol.transpose();

//            FILE_LOG(logINFO) << "DSP Cost : ";
//            for(int i=0;i<n_cost;i++) {
//                FILE_LOG(logINFO) << "idx (" << i << ") : " << (A_cost.row(i)*QP_sol-B_cost.row(i));
//            }
            break;
        }
        case SolverIsQPSwift:
        {
            qp_int n_var = QP_Dyn.NUMCOLS;
            qp_int n_equ = QP_Dyn.NUMEQ;
            qp_int n_inequ = QP_Dyn.NUMINEQ;

            qp_int   P_nnz,A_nnz,G_nnz;
            qp_real  *P_x = NULL;
            qp_int   *P_i = NULL,*P_p = NULL;
            qp_real  *q_x = NULL;
            qp_real  *A_x = NULL;
            qp_int   *A_i = NULL, *A_p = NULL;
            qp_real  *b_x = NULL;
            qp_real  *G_x = NULL;
            qp_int   *G_i = NULL, *G_p = NULL;
            qp_real  *h_x = NULL;

            // 1-0. Convert cost function to sparse form .
            MatrixNd P = QP_Dyn.P_cost;
            VectorNd q = QP_Dyn.Q_cost;
            P_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(P);
            P_x = new qp_real[P_nnz];
            P_i = new qp_int[P_nnz];
            P_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
            q_x = new qp_real[n_var];
            QP_Dyn.ConvertVector2Array_QPswift(q,n_var,q_x);

            // 1-1. Convert equality constraint to sparse form .
            if(n_equ > 0) {
                A_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_equ);
                A_x = new qp_real[A_nnz];
                A_i = new qp_int[A_nnz];
                A_p = new qp_int[n_var+1];
                QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_equ,A_x,A_i,A_p);
                b_x = new qp_real[n_equ];
                QP_Dyn.ConvertVector2Array_QPswift(B_equ,n_equ,b_x);
            }

            // 1-2. Convert inequality constraint to sparse form .
            G_nnz = QP_Dyn.GetNumberOfNonZero_QPswift(A_inequ);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_Dyn.ConvertMatrixA_Full2CCS_QPswift(A_inequ,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_Dyn.ConvertVector2Array_QPswift(B_inequ,n_inequ,h_x);

            // 2. Set Parameters and solve.
            QPswift      *myQP;
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                            P_p,P_i,P_x,
                            A_p,A_i,A_x,
                            G_p,G_i,G_x,
                            q_x,h_x,b_x,
                            0.0,nullptr);
            QP_SOLVE(myQP);
            VectorNd QP_sol = VectorNd::Zero(n_var);
            QP_Dyn.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
            ddQ_sol = QP_sol.segment(0,LIGHT_DOF);
            Ext_RF_sol = QP_sol.segment(LIGHT_DOF,6);
            Ext_LF_sol = QP_sol.segment(LIGHT_DOF+6,6);

            Resize_TempMatrix(LIGHT_ACT_DOF,n_var,A_temp,B_temp);
            A_temp.block(0,idx_ddQ,LIGHT_ACT_DOF,size_ddQ) = Ml;
            A_temp.block(0,idx_Ext_RF,LIGHT_ACT_DOF,6) = J_RFl_T;
            A_temp.block(0,idx_Ext_LF,LIGHT_ACT_DOF,6) = J_LFl_T;
            Torque_sol = A_temp*QP_sol + Nl;

            // 3. destruction allocated memory
            QP_CLEANUP(myQP);
            delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
            delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
            delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
            break;
        }
        default:
            FILE_LOG(logERROR) << "[Dynamics Error] QP Solver is not set!";
            return false;
        }

        system_clock::time_point t_SolveQPro = system_clock::now();
        microseconds dt_SolveQPro = duration_cast<std::chrono::microseconds>(t_SolveQPro - t_SetQPSolv);

        // Check Solving time for QP_Dyn
        if(false) {
            FILE_LOG(logDEBUG) << "dt_CalcJacob : "<< dt_CalcJacob.count() << " us";
            FILE_LOG(logDEBUG) << "dt_DefCostFn : "<< dt_DefCostFn.count() << " us";
            FILE_LOG(logDEBUG) << "dt_DefEquCon : "<< dt_DefEquCon.count() << " us";
            FILE_LOG(logDEBUG) << "dt_DefInEquC : "<< dt_DefInEquC.count() << " us";
            FILE_LOG(logDEBUG) << "dt_SetQPSolv : "<< dt_SetQPSolv.count() << " us";
            FILE_LOG(logDEBUG) << "dt_SolveQPro : "<< dt_SolveQPro.count() << " us";
        }
    }

    LIGHT.ddQref = ddQ_sol;
    LIGHT.Textref_RF = Ext_RF_sol.segment(0,3);
    LIGHT.Fextref_RF = Ext_RF_sol.segment(3,3);
    LIGHT.Textref_LF = Ext_LF_sol.segment(0,3);
    LIGHT.Fextref_LF = Ext_LF_sol.segment(3,3);
    LIGHT.Tref = Torque_sol;

    return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
/// Directly command torque
////////////////////////////////////////////////////////////////////////////////////////////////////

bool LIGHTWholeMotions::SupportControl_DirectTorque()
{
    LIGHT.ContactOn_RF();
    LIGHT.ContactOn_LF();

    // Setting Joint Torque
    LIGHT.Textref_RF = zv;
    LIGHT.Fextref_RF = zv;
    LIGHT.Textref_LF = zv;
    LIGHT.Fextref_LF = zv;

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// No Control
////////////////////////////////////////////////////////////////////////////////////////////////////

bool LIGHTWholeMotions::SupportControl_Noact()
{
    LIGHT.ContactOff_RF();
    LIGHT.ContactOff_LF();

    // Setting Joint Torque
    LIGHT.ddQref = VectorNd::Zero(LIGHT_DOF);
    LIGHT.Textref_RF = zv;
    LIGHT.Fextref_RF = zv;
    LIGHT.Textref_LF = zv;
    LIGHT.Fextref_LF = zv;
    LIGHT.Tref = VectorNd::Zero(LIGHT_ACT_DOF);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sub-Functions
////////////////////////////////////////////////////////////////////////////////////////////////////

Vector3d OrientationError(Matrix3d Rdes, Matrix3d R) {

    // Orientation Error
    Matrix3d err_R = Rdes*R.transpose(); // error in rotation matrix
    double err_W_theta = (err_R.trace()-1.0)/2.0;
    if (err_W_theta >= 1.0) {err_W_theta = 1.0 - 1e-6;}
    else if (err_W_theta <= -1.0) {err_W_theta = -1.0 + 1e-6;}
    err_W_theta = acos(err_W_theta);

    Vector3d err_W_vec;
    if(fabs(err_W_theta) < 1e-6) {
        err_W_vec = Vector3d::Zero(3,1);
    } else {
        err_W_vec(0,0) = 1.0/2.0/sin(err_W_theta)*(err_R(2,1)-err_R(1,2));
        err_W_vec(1,0) = 1.0/2.0/sin(err_W_theta)*(err_R(0,2)-err_R(2,0));
        err_W_vec(2,0) = 1.0/2.0/sin(err_W_theta)*(err_R(1,0)-err_R(0,1));
    }

    return err_W_theta*err_W_vec;
}

void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B) {
    A.resize(m,n);
    A = MatrixNd::Zero(m,n);
    B.resize(m,1);
    B = MatrixNd::Zero(m,1);
}

void Matrix4QP_Addition(MatrixNd& A_base, VectorNd& B_base, MatrixNd A_new, VectorNd B_new) {
    int m_now = A_base.rows();
    int n_new = A_new.cols();
    int m_new = A_new.rows();

    A_base.conservativeResize(m_now+m_new,n_new);
    B_base.conservativeResize(m_now+m_new,1);
    A_base.block(m_now,0,m_new,n_new) = A_new;
    B_base.block(m_now,0,m_new,1) = B_new;
}

bool Dynamics_SolutionCheck(VectorNd Tsol) {
    // true : Flag_Error On
    // false : Flag_Error Off

    if(Tsol.rows()!=13) {
        FILE_LOG(logERROR) << " Size mismatch error. " << endl;
        return true;
    }

    VectorNd Tmin = VectorNd::Zero(13);
    Tmin << -100.0, -200.0, -200.0, -350.0, -150.0, -150.0,
            -100.0, -200.0, -200.0, -350.0, -150.0, -150.0,
            -300.0;
    VectorNd Tmax = VectorNd::Zero(13);
    Tmax << 100.0, 200.0, 200.0, 350.0, 150.0, 150.0,
            100.0, 200.0, 200.0, 350.0, 150.0, 150.0,
            300.0;

    for(int i=0;i<13;i++) {
        if((Tmin(i))>Tsol(i)){
            FILE_LOG(logERROR) << " [Torque limit over] T(" << i << ") : " << Tsol(i) << endl;
            //            FILE_LOG(logERROR) << " [Torque limit over] T(" << i << ") : " << Tsol << endl;
            return true;
        }
        if((Tmax(i))<Tsol(i)){
            FILE_LOG(logERROR) << " [Torque limit over] T(" << i << ") : " << Tsol(i) << endl;
            //            FILE_LOG(logERROR) << " [Torque limit over] T(" << i << ") : " << Tsol << endl;
            return true;
        }
    }
    return false;
}

