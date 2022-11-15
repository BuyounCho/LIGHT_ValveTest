#include "QUAD_jointsetmodel.h"

void QUADJointSet::UpdateSensorData()
{
    VectorNd Qnew = VectorNd::Zero(n_dof);
    VectorNd dQnew = VectorNd::Zero(n_dof);

    static VectorNd Qo = VectorNd::Zero(n_dof);
    static VectorNd Qoo = VectorNd::Zero(n_dof);
    static VectorNd dQo = VectorNd::Zero(n_dof);
    static VectorNd dQoo = VectorNd::Zero(n_dof);

    ///// Joint Angle(Encoder) ///////////////////////////////////////////////////////////

    Qnew(0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHR].id][MC_ID_CH_Pairs[RHR].ch].CurrentAngle*D2R;
    dQnew(0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHR].id][MC_ID_CH_Pairs[RHR].ch].CurrentAngVel*D2R;
    Qnew(1) = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentAngle*D2R;
    dQnew(1) = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHP].id][MC_ID_CH_Pairs[RHP].ch].CurrentAngVel*D2R;

    // LowPass Filtering
    double f_cut_enc = 25.0; // not good under 10.0!
    Qnow = Qnew;
    dQnow = Vector_LPF_2nd(dQoo,dQo,dQnew,f_cut_enc);

    Qoo = Qo;
    Qo = Qnow;
    dQoo = dQo;
    dQo = dQnow;

    Qnow_arm = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHY].id][MC_ID_CH_Pairs[RHY].ch].CurrentAngle*D2R;
    dQnow_arm = sharedSEN->ENCODER[MC_ID_CH_Pairs[RHY].id][MC_ID_CH_Pairs[RHY].ch].CurrentAngVel*D2R;
}

void QUADJointSet::UpdateStates()
{
    /////////// End Effector Information (in WorkSpace) Update //////////////////////////////////////////////////
    UpdateKinematics(*Robot, Qnow, dQnow, VectorNd::Zero(n_dof));

    Xnow_Foot = CalcBodyToBaseCoordinates(*Robot, Qnow, n_RKN, RKN2EE, false);
    dXnow_Foot = CalcPointVelocity(*Robot, Qnow, dQnow, n_RKN, RKN2EE, false);
}

void QUADJointSet::UpdateReferenceStates()
{
    /////////// End Effector Information (in WorkSpace) Update //////////////////////////////////////////////////
    UpdateKinematics(*Robot, Qref, dQref, VectorNd::Zero(n_dof));

    Xref_Foot = CalcBodyToBaseCoordinates(*Robot, Qref, n_RKN, RKN2EE, false);
    dXref_Foot = CalcPointVelocity(*Robot, Qref, dQref, n_RKN, RKN2EE, false);
}

void QUADJointSet::SetReference_Joint()
{   
    // Joint Position Reference Update
    jCon->SetJointRefAngle(RHY, Qref_arm*R2D);
    jCon->SetJointRefAngle(RHR, Qref(0)*R2D);
    jCon->SetJointRefAngle(RHP, Qref(1)*R2D);

    // Joint Velocity Reference
    jCon->SetJointRefAngVel(RHY, dQref_arm*R2D);
    jCon->SetJointRefAngVel(RHR, dQref(0)*R2D);
    jCon->SetJointRefAngVel(RHP, dQref(1)*R2D);

    // Joint Torque Reference
    jCon->SetJointRefTorque(RHY, Tref_arm);
    jCon->SetJointRefTorque(RHR, Tref(0));
    jCon->SetJointRefTorque(RHP, Tref(1));
}


void QUADJointSet::SetReference_Actuator()
{
    double _Qref, _dQref, _Tref;
    double _Sref, _dSref, _Fref;

    // Arm Pitch /////////////////////////////////////////////
    _Qref = Qref_arm; // joint 'reference' angle
    _dQref = dQref_arm; // joint angular velocity
    _Tref = Tref_arm; // joint torque

    jCon->SetJointRefActPos(RHY, _Qref*R2D);
    jCon->SetJointRefActVel(RHY, _dQref*R2D);
    jCon->SetJointRefActForce(RHY, _Tref);

    // Hip Pitch /////////////////////////////////////////////
    _Qref = Qref(0); // joint 'reference' angle
    _dQref = dQref(0); // joint angular velocity
    _Tref = Tref(0); // joint torque
    Joint2Rotary_HipPitch(_Qref,_dQref,_Tref,_Sref,_dSref,_Fref);

    jCon->SetJointRefActPos(RHR, _Sref*R2D);
    jCon->SetJointRefActVel(RHR, _dSref*R2D);
    jCon->SetJointRefActForce(RHR, _Fref);

    // Knee Pitch Linkage /////////////////////////////////////////////
    _Qref = Qref(1); // joint 'reference' angle
    _dQref = dQref(1); // joint angular velocity
    _Tref = Tref(1); // joint torque
    Joint2Rotary_Knee(_Qref,_dQref,_Tref,_Sref,_dSref,_Fref);

    jCon->SetJointRefActPos(RHP, _Sref*R2D);
    jCon->SetJointRefActVel(RHP, _dSref*R2D);
    jCon->SetJointRefActForce(RHP, _Fref);
}

///////////////////////////////////////////////
///   [ Inverse Kinematics ]
///  Task Space >> Joint Space
/// ///////////////////////////////////////////

void QUADJointSet::InverseKinematics()
{
    if(Flag_InvKin)
    {
        if(CtrlSpace == TaskSpace) {
            QuadEffTest_QP  QP_Kin(SolverIsQPSwift);
            MatrixNd temp_A;
            VectorNd temp_B;

            int n_var = n_dof; // hip, knee

            UpdateKinematics(*Robot, Qref, dQref, VectorNd::Zero(n_dof));

            ///////// Cost Function (min 1/2*(Hq-F)^2) /////////////////////////////////////////////////////////////////
            /// Optimization Variables : U = [W_Pelvis(3);   dX_Pelvis(3);
            ///                               dq_RightLeg(6); dq_LeftLeg(6);
            ///                               dq_Waist(1)]
            ///
            /// (COST) = min 1/2*{(W_BODY*|Pelvis(6)-Pelvis_ref(6)|^2
            ///
            ///                   +W_Pel2RF_Ang*|Ang_Pel2RF(3)-Ang_Pel2RF_ref(3)|^2
            ///                   +W_Pel2RF_Lin*|Lin_Pel2RF(3)-Lin_Pel2RF_ref(3)|^2
            ///                   +W_RF_dQ*|dQ_RightLeg(6)-dQ_RightLeg_ref(6)|^2
            ///
            ///                   +W_Pel2LF_Ang*|Ang_Pel2LF(3)-Ang_Pel2LF_ref(3)|^2
            ///                   +W_Pel2LF_Lin*|Lin_Pel2LF(3)-Lin_Pel2LF_ref(3)|^2
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

            double W_tracking = 1.0;

            // 1. End-effector Tracking
            temp_size_cost = 3;
            Resize_TempMatrix(temp_size_cost,n_var,temp_A,temp_B);

            MatrixNd J = MatrixNd::Zero(3,n_var);
            CalcPointJacobian(*Robot, Qref, n_RKN, RKN2EE, J, false);

            double wn = 2.0*PI*5.0;
            Vector3d V_des = dXdes_Foot.col(0) + wn*(Xdes_Foot.col(0)-Xref_Foot);
            temp_A.block(0,0,3,n_var) = J;
            temp_B.block(0,0,3,1) = V_des;

            Matrix4QP_Addition(A_cost,B_cost,W_tracking*temp_A,W_tracking*temp_B);
            idx_cost += temp_size_cost;

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

            // 1) Joint Angle Limit
            temp_size_inequ = 2*n_var;
            Resize_TempMatrix(temp_size_inequ,n_var,temp_Ai,temp_Bi);

            VectorNd Q_lb = VectorNd::Zero(n_dof);
            VectorNd Q_ub = VectorNd::Zero(n_dof);
            Q_lb << 0.0, -110.0*D2R;
            Q_ub << 95.0*D2R, 0.0;

            VectorNd dQ_lb,dQ_ub;
            dQ_lb = (Q_lb-Qref)/SYS_DT;
            dQ_ub = (Q_ub-Qref)/SYS_DT;

            temp_Ai.block(0,0,n_var,n_var) = -MatrixNd::Identity(n_var,n_var);
            temp_Ai.block(n_var,0,n_var,n_var) = MatrixNd::Identity(n_var,n_var);
            temp_Bi.block(0,0,n_var,1) = -dQ_lb;
            temp_Bi.block(n_var,0,n_var,1) = dQ_ub;

            Matrix4QP_Addition(A_inequ,B_inequ,temp_Ai,temp_Bi);
            idx_inequ += temp_size_inequ;

            n_inequ = idx_inequ;

            ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

            QP_Kin.setNums(n_var,n_cost,n_equ,n_inequ);
            QP_Kin.make_COST(A_cost, B_cost);
            QP_Kin.make_EQ(A_equ,B_equ);
            QP_Kin.make_IEQ(A_inequ,B_inequ);

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

            Qref = Qref + dQref*SYS_DT;
        } else {
            Qref = Qdes.col(0);
            dQref = dQdes.col(0);
            ddQref = ddQdes.col(0);
        }
    }

    Qref_arm = Qdes_arm(0,0);
    dQref_arm = dQdes_arm(0,0);
    ddQref_arm = ddQdes_arm(0,0);
}


void QUADJointSet::InverseKinematics_PumpReference()
{
    if(Flag_InvKin)
    {
        if(CtrlSpace == TaskSpace) {

            int n_var = n_dof;

            dQdes.col(0) = dQref;
            Qdes.col(0) = Qref;

            MatrixNd Xrec = MatrixNd::Zero(3,sharedREF->N_window);
            for(int i = 0; i < sharedREF->N_window; i++) {
                Vector3d Xprev = CalcBodyToBaseCoordinates(*Robot, Qdes.col(i), n_RKN, RKN2EE, true);

                MatrixNd J = MatrixNd::Zero(3,n_var);
                CalcPointJacobian(*Robot, Qref, n_RKN, RKN2EE, J, false);

                double k = 1.0;
                Vector3d V_des = dXdes_Foot.col(i+1) + k*(Xdes_Foot.col(i)-Xprev);
//                Vector3d V_des = dXdes_Foot.col(i+1);

                MatrixNd JTJ = J.transpose()*J;
                MatrixNd J_linv = JTJ.inverse()*J.transpose();

                dQdes.col(i+1) = J_linv*V_des;
                Qdes.col(i+1) = Qdes.col(i) + dQdes.col(i+1)*sharedREF->dT_window;

                Xrec.col(i) = Xprev;
            }

//            FILE_LOG(logINFO) << "dQdes : " <<dQdes.transpose();

//            static int cnt_disp = 0;
//            if(cnt_disp%100==0) {
//                FILE_LOG(logDEBUG2) << "Xdes : " << Xdes_Foot.row(2).transpose();
//                FILE_LOG(logDEBUG2) << "Error : " << Xdes_Foot.row(2).block(0,0,1,sharedREF->N_window-1).transpose()-Xrec.row(2).transpose();
//            } cnt_disp++;
        }

        for(int i = 0; i <= sharedREF->N_window; i++) {
            Joint2Rotary_HipPitch(Qdes(0,i),dQdes(0,i),Tdes(0,i),Sdes(0,i),dSdes(0,i),Fdes(0,i));
            double rA_Hip = 9595.0*1e-3; // mm^2*m
            double Pl_Hip = fabs(Fdes(0,i)/rA_Hip*10.0); // MPa >> bar
            double Qact_Hip = fabs(dSdes(0,i)*rA_Hip/1000.0*60.0); // rad/s >> L/min

            sharedREF->LoadPressureReference_Future[i][RHR] = Pl_Hip;
            sharedREF->ActFlowrateReference_Future[i][RHR] = Qact_Hip;

            Joint2Rotary_Knee(Qdes(1,i),dQdes(1,i),Tdes(1,i),Sdes(1,i),dSdes(1,i),Fdes(1,i));
            double rA_Knee = 9595.0*1e-3; // mm^2*m
            double Pl_Knee = fabs(Fdes(1,i)/rA_Knee*10.0); // MPa >> bar
            double Qact_Knee = fabs(dSdes(1,i)*rA_Knee/1000.0*60.0); // rad/s >> L/min

            sharedREF->LoadPressureReference_Future[i][RHP] = Pl_Knee;
            sharedREF->ActFlowrateReference_Future[i][RHP] = Qact_Knee;

        }

    } else {
        for(int i = 0; i <= sharedREF->N_window; i++) {
            sharedREF->LoadPressureReference_Future[i][RHR] = 0.0;
            sharedREF->ActFlowrateReference_Future[i][RHR] = 0.0;
            sharedREF->LoadPressureReference_Future[i][RHP] = 0.0;
            sharedREF->ActFlowrateReference_Future[i][RHP] = 0.0;
        }
    }

    for(int i = 0; i <= sharedREF->N_window; i++) {
        double rA_Arm = 9595.0*1e-3; // mm^2*m
        double Pl_Arm = fabs(Tdes_arm(0,i)/rA_Arm*10.0); // MPa >> bar
        double Qact_Arm = fabs(dQdes_arm(0,i)*rA_Arm/1000.0*60.0); // rad/s >> L/min

        sharedREF->LoadPressureReference_Future[i][RHY] = Pl_Arm;
        sharedREF->ActFlowrateReference_Future[i][RHY] = Qact_Arm;
    }
}

void QUADJointSet::InverseDynamics()
{
    if(Flag_InvDyn) {

    } else {

    }
}

///////////////////////////////////////////////
///   [ Joint Info >> Actuator Info ]
/// ///////////////////////////////////////////

//  Hip Joint ////////////////////////////////////////////////////////////
void  QUADJointSet::Joint2Rotary_HipPitch(double theta, double dtheta, double T,
                                        double &S, double &dS, double &F)
{
    S = theta;
    dS = dtheta;
    F = T;
}


//  Knee Joint ////////////////////////////////////////////////////////////
void  QUADJointSet::Joint2Rotary_Knee(double theta, double dtheta, double T,
                                        double &S, double &dS, double &F)
{
    // input : knee angle (contraction~extension) [rad]
    // output : rotary angle  [rad]

    theta = -theta; // theta is negative direction
    if(theta < -1.0*D2R) theta = -1.0*D2R;
    else if (theta > 110.0*D2R) theta = 110.0*D2R;
    dtheta = -dtheta;
    T = -T;

    double d1 = 0.350;
    double d2 = 0.060;
    double d3 = 0.350;
    double d4 = 0.058;
    double delta = (20.0)*D2R; // knee angle offset
    double gamma = (28.0)*D2R; // knee ~ d4 link angle

    double S_ini = (37.03)*D2R; // initial actuator angle offset (fully contracted)
    double Ls = sqrt(d1*d1+d4*d4+2.0*d1*d4*cos(theta+delta+gamma)); // cos(PI-(theta+delta+gamma)) = -cos(theta+delta+gamma)
    double phi2 = acos((Ls*Ls+d2*d2-d3*d3)/(2.0*Ls*d2));
    double phi1 = acos((Ls*Ls+d1*d1-d4*d4)/(2.0*Ls*d1));

    S = phi2 - phi1 - S_ini;

    // Jacobian for joint >> actuator
    double dLs = -d1*d4*sin(theta+delta+gamma)/Ls;
    double dphi2 = (d2*cos(phi2)-Ls)*dLs/(d2*Ls*sin(phi2));
    double dphi1 = (d1*cos(phi1)-Ls)*dLs/(d1*Ls*sin(phi1));
    double J = dphi2 - dphi1;

    dS = J*dtheta;
    F = T/J;
}
