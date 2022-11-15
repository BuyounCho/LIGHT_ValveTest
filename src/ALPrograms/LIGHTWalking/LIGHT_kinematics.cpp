#include "LIGHT_kinematics.h"

extern void PrintHere(int n);

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;

extern LIGHT_InvKinematics_INFO INFO_InvKin;
extern LIGHT_InvKinematics_SUB_INFO INFO_InvKin_SUB;

extern Vector3d OrientationError(Matrix3d Rdes, Matrix3d R);
extern void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B);
extern void Matrix4QP_Addition(MatrixNd& A_cost, VectorNd& B_cost, MatrixNd A_temp, VectorNd B_temp);

void LIGHTWholeBody::InverseKinematics()
{
//    LIGHT_QP QP_Kin(SolverIsQPSwift);
    LIGHT_QP QP_Kin(SolverIsQuadProg);
    MatrixNd temp_A;
    VectorNd temp_B;

    //////// Set Variable length, number of equality constraints and inequality constraints ///////
    /// Optimization Variables : U = [W_Pelvis(3);   dX_Pelvis(3);
    ///                               W_RightLeg(3); dX_RightLeg(3);
    ///                               W_LeftLeg(3);  dX_LeftLeg(3);
    ///                               Waist(1)]
    ///////////////////////////////////////////////////////////////////////////////////////////////
    int size_W_pel = 3;
    int size_V_pel = 3;
    int size_dQ_RF = 6;
    int size_dQ_LF = 6;
    int size_dQ_WST = 1;
    int idx_W_pel = 0;
    int idx_V_pel = idx_W_pel+size_W_pel;
    int idx_dQ_RF = idx_V_pel+size_V_pel;
    int idx_dQ_LF = idx_dQ_RF+size_dQ_RF;
    int idx_dQ_WST = idx_dQ_LF+size_dQ_LF;
    int n_var = size_W_pel+size_V_pel+size_dQ_RF+size_dQ_LF+size_dQ_WST; // 19

    UpdateKinematics(*LIGHT.Robot, LIGHT.Qref, LIGHT.dQref, VectorNd::Zero(LIGHT_DOF));

    ///////// Cost Function (min 1/2*(Hq-F)^2) /////////////////////////////////////////////////////////////////
    /// Optimization Variables : U = [W_Pelvis(3);   dX_Pelvis(3);
    ///                               dq_RightLeg(6); dq_LeftLeg(6);
    ///                               dq_Waist(1)]
    ///
    /// (COST) = min 1/2*{(W_BODY*|Pelvis(6)-Pelvis_ref(6)|^2
    ///
    ///                   +W_Pel2RF_Ang*|Ang_Pel2RF(3)-Ang_Pel2RF_ref(3)|^2
    ///                   +W_Pel2RF_Lin*|Lin_link[PEL]2RF(3)-Lin_link[PEL]2RF_ref(3)|^2
    ///                   +W_RF_dQ*|dQ_RightLeg(6)-dQ_RightLeg_ref(6)|^2
    ///
    ///                   +W_Pel2LF_Ang*|Ang_Pel2LF(3)-Ang_Pel2LF_ref(3)|^2
    ///                   +W_Pel2LF_Lin*|Lin_link[PEL]2LF(3)-Lin_link[PEL]2LF_ref(3)|^2
    ///                   +W_LF_dQ*|dQ_LeftLeg(6)-dQ_LeftLeg_ref(6)|^2
    ///
    ///                   +W_RF2Pel_Ang*|Ang_RF2Pel(3)-Ang_RF2Pel_ref(3)|^2
    ///                   +W_RF2Pel_Lin*|Lin_RF2Pel(3)-Lin_RF2Pel_ref(3)|^2
    ///                   +W_RF2LF_Ang*|Ang_RF2LF(3)-Ang_RF2LF_ref(3)|^2
    ///                   +W_RF2LF_Lin*|Lin_RF2LF(3)-Lin_RF2LF_ref(3)|^2
    ///                   +W_RF2CoM*|RF2CoM(3)-RF2CoM(3)|^2
    ///
    ///                   +W_LF2Pel_Ang*|Ang_LF2Pel(3)-Ang_LF2Pel_ref(3)|^2
    ///                   +W_LF2Pel_Lin*|Lin_LF2Pel(3)-Lin_LF2Pel_ref(3)|^2
    ///                   +W_LF2RF_Ang*|Ang_LF2RF(3)-Ang_LF2RF_ref(3)|^2
    ///                   +W_LF2RF_Lin*|Lin_LF2RF(3)-Lin_LF2RF_ref(3)|^2
    ///                   +W_LF2CoM*|LF2CoM(3)-LF2CoM(3)|^2
    ///
    ///                   +W_WST_dQ*|dQ_WST(1)-dQ_WST_ref(1)|^2
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int n_cost = 0;
    int idx_cost = 0;
    int temp_size_cost = 0;
    MatrixNd A_cost;
    VectorNd B_cost;

    // 0) Global to Pelvis Orientation (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_PEL_Ori] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_PEL_Ori];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(6,n_var);
        CalcPointJacobian6D(*Robot, Qref, n_link[PEL], zv, J, false);
        MatrixNd Jw_Pel = J.block(0,0,3,n_var);

        // Orientation Error
        double wn = 2.0*PI*3.0;
        Vector3d W_des = Wdes_Pel + wn*OrientationError(Rdes_Pel,Rref_Pel);

        temp_A.block(0,0,size_W_pel,n_var) = W*Jw_Pel;
        temp_B.block(0,0,size_W_pel,1) = W*W_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }


    // 1) Global to Pelvis Position (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_PEL_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_PEL_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd Jv_Pel = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[PEL], zv, Jv_Pel, false);

        // Position Error
        double wn = 2.0*PI*3.0;
        Vector3d V_des = dXdes_Pel + wn*(Xdes_Pel-Xref_Pel);

        temp_A.block(0,0,size_V_pel,n_var) = W*Jv_Pel;
        temp_B.block(0,0,size_V_pel,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 2) Global to RF Orientation (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_RF_Ori] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_RF_Ori];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(6,n_var); // Orientation(3)
        CalcPointJacobian6D(*Robot, Qref, n_link[RAR], rar2EE, J,1);

        double wn = 2.0*PI*3.0;
        MatrixNd W_des = Wdes_RF + wn*OrientationError(Rdes_RF,Rref_RF);

        temp_A.block(0,0,3,n_var) = W*J.block(0,0,3,n_var);
        temp_B.block(0,0,3,1) = W*W_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 3) Global to LF Orientation (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_LF_Ori] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_LF_Ori];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(6,n_var); // Orientation(3)
        CalcPointJacobian6D(*Robot, Qref, n_link[LAR], lar2EE, J,1);

        double wn = 2.0*PI*3.0;
        MatrixNd W_des = Wdes_LF + wn*OrientationError(Rdes_LF,Rref_LF);

        temp_A.block(0,0,3,n_var) = W*J.block(0,0,3,n_var);
        temp_B.block(0,0,3,1) = W*W_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 4) Global to RF Position (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_RF_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_RF_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd Jv_RF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[RAR], rar2EE, Jv_RF,1); // RF Position Jacobian

        double wn = 2.0*PI*3.0;
        Vector3d V_des = dXdes_RF + wn*(Xdes_RF-Xref_RF);

        temp_A.block(0,0,3,n_var) = W*Jv_RF;
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 5) Global to LF Position (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_LF_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_LF_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd Jv_LF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[LAR], lar2EE, Jv_LF,1); // LF Position Jacobian

        double wn = 2.0*PI*3.0;
        Vector3d V_des = dXdes_LF + wn*(Xdes_LF-Xref_LF);

        temp_A.block(0,0,3,n_var) = W*Jv_LF;
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 6) Pelvis to RF Orientation (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_Pel2RF_Ori] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_Pel2RF_Ori];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(3,n_var); // Orientation(3)
        MatrixNd J_Pel = MatrixNd::Zero(6,n_var);
        MatrixNd J_RF = MatrixNd::Zero(6,n_var);
        CalcPointJacobian6D(*Robot, Qref_ZeroPelvis, n_link[PEL], zv, J_Pel,1);
        CalcPointJacobian6D(*Robot, Qref_ZeroPelvis, n_link[RAR], rar2EE, J_RF,1);
        J = (J_RF.block(0,0,3,n_var)-J_Pel.block(0,0,3,n_var));

        double wn = 2.0*PI*3.0;
        MatrixNd W_des = Wdes_Pel2RF + wn*OrientationError(Rdes_Pel2RF,Rref_Pel2RF);

        temp_A.block(0,0,3,n_var) = W*J;
        temp_B.block(0,0,3,1) = W*W_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 7) Pelvis to LF Orientation (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_Pel2LF_Ori] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_Pel2LF_Ori];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(3,n_var); // Orientation(3)
        MatrixNd J_Pel = MatrixNd::Zero(6,n_var);
        MatrixNd J_LF = MatrixNd::Zero(6,n_var);
        CalcPointJacobian6D(*Robot, Qref_ZeroPelvis, n_link[PEL], zv, J_Pel,1);
        CalcPointJacobian6D(*Robot, Qref_ZeroPelvis, n_link[LAR], lar2EE, J_LF,1);
        J = (J_LF.block(0,0,3,n_var)-J_Pel.block(0,0,3,n_var));

        double wn = 2.0*PI*3.0;
        Vector3d W_des = Wdes_Pel2LF + wn*OrientationError(Rdes_Pel2LF,Rref_Pel2LF);

        temp_A.block(0,0,3,n_var) = W*J;
        temp_B.block(0,0,3,1) = W*W_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 8) Pelvis to RF Position (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_Pel2RF_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_Pel2RF_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(3,n_var); // Position(3)
        MatrixNd Jv_Pel = MatrixNd::Zero(3,n_var);
        MatrixNd Jv_RF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref_ZeroPelvis, n_link[PEL], zv, Jv_Pel,1); // Pelvis Position Jacobian
        CalcPointJacobian(*Robot, Qref_ZeroPelvis, n_link[RAR], rar2EE, Jv_RF,1); // RF Position Jacobian
        J = (Jv_RF-Jv_Pel);

        double wn = 2.0*PI*3.0;
        Vector3d V_des = dXdes_Pel2RF + wn*(Xdes_Pel2RF-Xref_Pel2RF);

        temp_A.block(0,0,3,n_var) = W*J;
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 9) Pelvis to LF Position (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_Pel2LF_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_Pel2LF_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(3,n_var); // Position(3)
        MatrixNd Jv_Pel = MatrixNd::Zero(3,n_var);
        MatrixNd Jv_LF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref_ZeroPelvis, n_link[PEL], zv, Jv_Pel,1); // Pelvis Position Jacobian
        CalcPointJacobian(*Robot, Qref_ZeroPelvis, n_link[LAR], lar2EE, Jv_LF,1); // LF Position Jacobian
        J = (Jv_LF-Jv_Pel);

        MatrixNd V_des = MatrixNd::Zero(3,1);
        double wn = 2.0*PI*3.0;
        V_des = dXdes_Pel2LF + wn*(Xdes_Pel2LF-Xref_Pel2LF);

        temp_A.block(0,0,3,n_var) = W*J;
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 10) RightFoot to CoM Position
    // x,y position : CoM position
    // z position : RightFoot to Pelvis Height
    if(INFO_InvKin.Weight[INFO_InvKin.W_RF2CoM_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_RF2CoM_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd Jv_RF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[RAR], rar2EE, Jv_RF,1); // RF Position Jacobian
        MatrixNd Jv_Pel = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[PEL], zv, Jv_Pel,1); // RF Position Jacobian
        MatrixNd Jv_RF2CoM = CalcCoMJacobian(Qref)-Jv_RF;
        MatrixNd Jv_RF2Pel = Jv_Pel-Jv_RF;

        double wn = 2.0*PI*3.0;
        Vector3d V_des;
        V_des(0) = dXdes_RF2CoM(0) + wn*(Xdes_RF2CoM(0)-Xref_RF2CoM(0));
        V_des(1) = dXdes_RF2CoM(1) + wn*(Xdes_RF2CoM(1)-Xref_RF2CoM(1));
        V_des(2) = dXdes_RF2CoM(2) + wn*(Xdes_RF2CoM(2)-Xref_RF2CoM(2));
        temp_A.block(0,0,1,n_var) = W*Jv_RF2CoM.row(0);
        temp_A.block(1,0,1,n_var) = W*Jv_RF2CoM.row(1);
        temp_A.block(2,0,1,n_var) = W*Jv_RF2Pel.row(2);
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 11) RightFoot to LeftFoot Position (3)
    if(INFO_InvKin.Weight[INFO_InvKin.W_RF2LF_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_RF2LF_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(3,n_var); // Position(3)
        MatrixNd Jv_RF = MatrixNd::Zero(3,n_var);
        MatrixNd Jv_LF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[RAR], rar2EE, Jv_RF,1); // RF Position Jacobian
        CalcPointJacobian(*Robot, Qref, n_link[LAR], lar2EE, Jv_LF,1); // LF Position Jacobian
        J = (Jv_LF-Jv_RF);

        double wn = 2.0*PI*3.0;
        MatrixNd V_des = dXdes_RF2LF + wn*(Xdes_RF2LF-Xref_RF2LF);

        temp_A.block(0,0,3,n_var) = W*J;
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 12) LeftFoot to CoM Position
    if(INFO_InvKin.Weight[INFO_InvKin.W_LF2CoM_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_LF2CoM_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd Jv_LF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[LAR], lar2EE, Jv_LF,1); // LF Position Jacobian
        MatrixNd Jv_Pel = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[PEL], zv, Jv_Pel,1); // RF Position Jacobian
        MatrixNd Jv_LF2CoM = CalcCoMJacobian(Qref)-Jv_LF;
        MatrixNd Jv_LF2Pel = Jv_Pel-Jv_LF;

        double wn = 2.0*PI*3.0;
        Vector3d V_des;
        V_des(0) = dXdes_LF2CoM(0) + wn*(Xdes_LF2CoM(0)-Xref_LF2CoM(0));
        V_des(1) = dXdes_LF2CoM(1) + wn*(Xdes_LF2CoM(1)-Xref_LF2CoM(1));
        V_des(2) = dXdes_LF2CoM(2) + wn*(Xdes_LF2CoM(2)-Xref_LF2CoM(2));

        temp_A.block(0,0,1,n_var) = W*Jv_LF2CoM.row(0);
        temp_A.block(1,0,1,n_var) = W*Jv_LF2CoM.row(1);
//        temp_A.block(2,0,1,n_var) = W*Jv_LF2CoM.row(2);
        temp_A.block(2,0,1,n_var) = W*Jv_LF2Pel.row(2);
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 13) LeftFoot to RightFoot Position
    if(INFO_InvKin.Weight[INFO_InvKin.W_LF2RF_Pos] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_LF2RF_Pos];
        temp_size_cost = 3;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd J = MatrixNd::Zero(3,n_var); // Position(3)
        MatrixNd Jv_LF = MatrixNd::Zero(3,n_var);
        MatrixNd Jv_RF = MatrixNd::Zero(3,n_var);
        CalcPointJacobian(*Robot, Qref, n_link[LAR], lar2EE, Jv_LF,1); // LF Position Jacobian
        CalcPointJacobian(*Robot, Qref, n_link[RAR], rar2EE, Jv_RF,1); // RF Position Jacobian
        J = (Jv_RF-Jv_LF);

        double wn = 2.0*PI*3.0;
        MatrixNd V_des = dXdes_LF2RF + wn*(Xdes_LF2RF-Xref_LF2RF);

        temp_A.block(0,0,3,n_var) = W*J;
        temp_B.block(0,0,3,1) = W*V_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 14) Right Leg Joint (6)
    if(INFO_InvKin.Weight[INFO_InvKin.W_RF_dQ] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_RF_dQ];
        temp_size_cost = 6;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        double wn = 2.0*PI*3.0;
        MatrixNd dQ_RF_ref = dQdes.block(RL_QNUMSTART,0,LEG_DOF,1)+wn*(Qdes.block(RL_QNUMSTART,0,LEG_DOF,1)-Qref.block(RL_QNUMSTART,0,LEG_DOF,1));
        temp_A.block(0,idx_dQ_RF,size_dQ_RF,size_dQ_RF) = W*MatrixNd::Identity(size_dQ_RF,size_dQ_RF);
        temp_B.block(0,0,size_dQ_RF,1) = W*dQ_RF_ref;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 15) Left Leg Joint (6)
    if(INFO_InvKin.Weight[INFO_InvKin.W_LF_dQ] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_LF_dQ];
        temp_size_cost = 6;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        double wn = 2.0*PI*3.0;
        MatrixNd dQ_LF_des = dQdes.block(LL_QNUMSTART,0,LEG_DOF,1)+wn*(Qdes.block(LL_QNUMSTART,0,LEG_DOF,1)-Qref.block(LL_QNUMSTART,0,LEG_DOF,1));
        temp_A.block(0,idx_dQ_LF,size_dQ_LF,size_dQ_LF) = W*MatrixNd::Identity(size_dQ_LF,size_dQ_LF);
        temp_B.block(0,0,size_dQ_LF,1) = W*dQ_LF_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 16) Waist Joint (1)
    if(INFO_InvKin.Weight[INFO_InvKin.W_WST_dQ] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_WST_dQ];
        temp_size_cost = 1;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        double wn = 2.0*PI*1.0;
        MatrixNd dQ_WST_des = dQdes.block(WST_QNUMSTART,0,WST_DOF,1)+wn*(Qdes.block(WST_QNUMSTART,0,WST_DOF,1)-Qref.block(WST_QNUMSTART,0,WST_DOF,1));
        temp_A.block(0,idx_dQ_WST,size_dQ_WST,size_dQ_WST) = W*MatrixNd::Identity(size_dQ_WST,size_dQ_WST);
        temp_B.block(0,0,size_dQ_WST,1) = W*dQ_WST_des;

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

    // 17) Yaw Momentum Minimization (1)
    if(INFO_InvKin.Weight[INFO_InvKin.W_YawMomentum_Minimize] != 0.0) {
        double W = INFO_InvKin.Weight[INFO_InvKin.W_YawMomentum_Minimize];
        temp_size_cost = 1;
        Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

        MatrixNd MJ = MatrixNd::Zero(3,n_var);
        for(int i=0;i<LIGHT_ACT_DOF+1;i++) {
            MatrixNd M = MatrixNd::Zero(3,6);
            Matrix3d R = CalcWorld2BodyOrientation(*Robot, Qref, n_link[i], false);
            M.block(0,0,3,3) = R*I_link[i]*R.transpose();
            M.block(0,3,3,3) = m_link[i]*VectorCross2Matrix(CalcBodyToBaseCoordinates(*Robot, Qref, n_link[i], c_link[i], false));

            MatrixNd J = MatrixNd::Zero(6,n_var);
            CalcPointJacobian6D(*Robot, Qref, n_link[i], c_link[i], J, false);
            MJ += M*J;
        }
        temp_A.block(0,0,temp_size_cost,n_var) = W*MJ.row(2);

        Matrix4QP_Addition(A_cost,B_cost,temp_A,temp_B);
        idx_cost += temp_size_cost;
    }

//            Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
//            std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

    n_cost = idx_cost;

    //////// Equality Constraints (Aq<=B) ////////////////////////////////////////////////////
    /// Optimization Variables : U = [W_Pelvis(3);   dX_Pelvis(3);
    ///                               W_RightLeg(3); dX_RightLeg(3);
    ///                               W_LeftLeg(3);  dX_LeftLeg(3);
    ///                               Waist(1)]
    ///
    ///    (No Equality Constraints in kinematics)
    //////////////////////////////////////////////////////////////////////////////////////////////
    int n_equ = 0;
    int idx_equ = 0;
    int temp_size_equ = 0;
    MatrixNd A_equ;
    VectorNd B_equ;
    MatrixNd temp_Ae;
    VectorNd temp_Be;

    n_equ = idx_equ;

    //////// Inequality Constraints (Aq<=B) ////////////////////////////////////////////////////
    /// Optimization Variables : U = [W_Pelvis(3);   dX_Pelvis(3);
    ///                               W_RightLeg(3); dX_RightLeg(3);
    ///                               W_LeftLeg(3);  dX_LeftLeg(3);
    ///                               Waist(1)]
    ///
    /// 1) Joint Velocity Limit (angle, velocity, accelration)
    //////////////////////////////////////////////////////////////////////////////////////////////
    int n_inequ = 0;
    int idx_inequ = 0;
    int temp_size_inequ = 0;
    MatrixNd A_inequ;
    VectorNd B_inequ;
    MatrixNd temp_Ai;
    VectorNd temp_Bi;

    // 1) Joint Velocity Limit (angle, velocity, accelration)
    temp_size_inequ = 2*n_var;
    Resize_TempMatrix(temp_size_inequ,n_var,temp_Ai,temp_Bi);

    SetJointLimit_dQ();
    temp_Ai.block(0,0,n_var,n_var) = -MatrixNd::Identity(n_var,n_var);
    temp_Ai.block(n_var,0,n_var,n_var) = MatrixNd::Identity(n_var,n_var);
    temp_Bi.block(0,0,n_var,1) = -this->dQCon_lb;
    temp_Bi.block(n_var,0,n_var,1) = this->dQCon_ub;

    Matrix4QP_Addition(A_inequ,B_inequ,temp_Ai,temp_Bi);
    idx_inequ += temp_size_inequ;

    n_inequ = idx_inequ;

    ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

    QP_Kin.setNums(n_var,n_cost,n_equ,n_inequ);
    QP_Kin.make_COST(A_cost, B_cost);
    int m = A_cost.transpose().cols();
    int n = A_cost.transpose().rows();
    QP_Kin.make_EQ(A_equ,B_equ);
    QP_Kin.make_IEQ(A_inequ,B_inequ);

    ////////////////////////////////////////////////////////////////////////////////////////////////

//    std::cout << "Analytic solution"  << std::endl << (-(QP_Kin.P_cost+MatrixNd::Identity(n_var,n_var)*1e-6).inverse()*QP_Kin.Q_cost).transpose()<< std::endl;

    //
    switch(QP_Kin.WhichSolver()) {
    case SolverIsQuadProg:
    {
        // 1. Solve.
        dQref = QP_Kin.solve_QP();
        break;
    }
    case SolverIsQPSwift:
    {
        qp_int n_var = QP_Kin.NUMCOLS;
        qp_int n_equ = QP_Kin.NUMEQ;
        qp_int n_inequ = QP_Kin.NUMINEQ;

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
        MatrixNd P = QP_Kin.P_cost;
        VectorNd q = QP_Kin.Q_cost;
        P_nnz = QP_Kin.GetNumberOfNonZero_QPswift(P);
        P_x = new qp_real[P_nnz];
        P_i = new qp_int[P_nnz];
        P_p = new qp_int[n_var+1];
        QP_Kin.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
        q_x = new qp_real[n_var];
        QP_Kin.ConvertVector2Array_QPswift(q,n_var,q_x);

        // 1-1. Convert equality constraint to sparse form .
        if(n_equ > 0) {
            A_nnz = QP_Kin.GetNumberOfNonZero_QPswift(A_equ);
            A_x = new qp_real[A_nnz];
            A_i = new qp_int[A_nnz];
            A_p = new qp_int[n_var+1];
            QP_Kin.ConvertMatrixA_Full2CCS_QPswift(A_equ,A_x,A_i,A_p);
            b_x = new qp_real[n_equ];
            QP_Kin.ConvertVector2Array_QPswift(B_equ,n_equ,b_x);
        }

        // 1-2. Convert inequality constraint to sparse form .
        G_nnz = QP_Kin.GetNumberOfNonZero_QPswift(A_inequ);
        G_x = new qp_real[G_nnz];
        G_i = new qp_int[G_nnz];
        G_p = new qp_int[n_var+1];
        QP_Kin.ConvertMatrixA_Full2CCS_QPswift(A_inequ,G_x,G_i,G_p);
        h_x = new qp_real[n_inequ];
        QP_Kin.ConvertVector2Array_QPswift(B_inequ,n_inequ,h_x);

        // 2. Set Parameters and solve.
        QPswift      *myQP;
        myQP = QP_SETUP(n_var,n_inequ,n_equ,
                                P_p,P_i,P_x,
                                A_p,A_i,A_x,
                                G_p,G_i,G_x,
                                q_x,h_x,b_x,
                                0.0,nullptr);
        QP_SOLVE(myQP);
        QP_Kin.ConvertArray2Vector_QPswift(myQP->x, n_var, dQref);


//        if (myQP->stats->Flag == QP_OPTIMAL){
//            PRINT("\nOptimal Solution Found\n");
//            PRINT("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup)*1000.0);
//            PRINT("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
//            PRINT("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
//            PRINT("Iterations     : %d\n\n", myQP->stats->IterationCount);
//        }
//        if (myQP->stats->Flag == QP_MAXIT){
//            PRINT("\nMaximum Iterations reached\n");
//            PRINT("Solve Time     : %f ms\n", myQP->stats->tsolve*1000.0);
//            PRINT("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
//            PRINT("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
//            PRINT("Iterations     : %d\n\n", myQP->stats->IterationCount);
//        }

//        if (myQP->stats->Flag == QP_FATAL){
//            PRINT("\nUnknown Error Detected\n\n");
//        }

//        if (myQP->stats->Flag == QP_KKTFAIL){
//            PRINT("\nLDL Factorization fail\n\n");
//        }

        // 3. destruction allocated memory
        QP_CLEANUP(myQP);
        delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
        delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
        delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
        break;
    }
    default:
        FILE_LOG(logERROR) << "[Kinematics Error] QP Solver is not set!";
        return;
    }

//    if(Flag_KinematicCompensation) {
//        double K_acc = 0.0;
//        dQref_Comp(FLOATING_BASE_DOF+RHR) = dQref_Comp(FLOATING_BASE_DOF+RHR) + K_acc*ddQref(FLOATING_BASE_DOF+RHR)*SYS_DT_WALKING;
//        dQref_Comp(FLOATING_BASE_DOF+LHR) = dQref_Comp(FLOATING_BASE_DOF+LHR) + K_acc*ddQref(FLOATING_BASE_DOF+LHR)*SYS_DT_WALKING;
//        dQref_Comp = 0.999*dQref_Comp;
//    } else {
//        dQref_Comp = VectorNd::Zero(LIGHT_DOF);
//    }
//    dQref = dQref + dQref_Comp;

    // Q(0~2) elements are Pelvis Position.
    Qref.segment(0,3) = Qref.segment(0,3) + dQref.segment(0,3)*SYS_DT_WALKING;

    // Q(3~5) elements are Pelvis Orientation(Quaternion). So, we should convert angular velocity to quaternion.
    Quaternion W_Pelvis(dQref(3),dQref(4),dQref(5),0.0);
    Quaternion Quat_Pelvis(Qref(3),Qref(4),Qref(5),Qref(QNUM_END));
    Quaternion dQuat_Pelvis = W_Pelvis*Quat_Pelvis;
    Quat_Pelvis += 0.5*SYS_DT_WALKING*dQuat_Pelvis;
    Qref(PELVIS_ORI_QNUMSTART+0) = Quat_Pelvis(0);
    Qref(PELVIS_ORI_QNUMSTART+1) = Quat_Pelvis(1);
    Qref(PELVIS_ORI_QNUMSTART+2) = Quat_Pelvis(2);
    Qref(QNUM_END) = Quat_Pelvis(3);

    // Q(6~end-1) elements are Joint angle.
    Qref.segment(ACT_QNUMSTART,LIGHT_ACT_DOF) = Qref.segment(ACT_QNUMSTART,LIGHT_ACT_DOF)
            + dQref.segment(ACT_QNUMSTART,LIGHT_ACT_DOF)*SYS_DT_WALKING;

    Qref_ZeroPelvis = Qref;
    Qref_ZeroPelvis.segment(0,6) = VectorNd::Zero(6);
    Qref_ZeroPelvis(QNUM_END) = 1.0;
    dQref_ZeroPelvis = dQref;
    dQref_ZeroPelvis.segment(0,6) = VectorNd::Zero(6);

}

