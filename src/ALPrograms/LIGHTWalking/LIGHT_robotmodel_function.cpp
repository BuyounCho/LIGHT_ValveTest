//#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicJoint.h"
#include "LIGHT_robotmodel.h"
#include <iostream>

extern LIGHTWholeBody        LIGHT;
extern JointControlClass       *jCon;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

extern INFO_LIGHT LIGHT_Info;

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern int                     PODO_NO;
extern int                     __IS_CHOREONOID;

extern bool IsWalking;

extern VectorNd Vector_LPF(VectorNd v, VectorNd v_new, double f_cut);
extern VectorNd Vector_LPF_2nd(VectorNd voo, VectorNd vo, VectorNd v_new, double f_cut);
Vector3d ZMP_calc_global(Vector3d _pRF, Matrix3d _R_RF, Vector3d _F_RF, Vector3d _M_RF, Vector3d _pLF, Matrix3d _R_LF, Vector3d _F_LF,Vector3d _M_LF);

//==============================//
// Update Variables and States
//==============================//

VectorNd Qo = VectorNd::Zero(LIGHT_DOF+1);
VectorNd Qoo = VectorNd::Zero(LIGHT_DOF+1);
VectorNd dQo = VectorNd::Zero(LIGHT_DOF);
VectorNd dQoo = VectorNd::Zero(LIGHT_DOF);
VectorNd To = VectorNd::Zero(LIGHT_ACT_DOF);
VectorNd Too = VectorNd::Zero(LIGHT_ACT_DOF);
Vector3d TextRFo = Vector3d::Zero();
Vector3d TextRFoo = Vector3d::Zero();
Vector3d FextRFo = Vector3d::Zero();
Vector3d FextRFoo = Vector3d::Zero();
Vector3d TextLFo = Vector3d::Zero();
Vector3d TextLFoo = Vector3d::Zero();
Vector3d FextLFo = Vector3d::Zero();
Vector3d FextLFoo = Vector3d::Zero();

void LIGHTWholeBody::UpdateSensorData()
{

    // Q(0~2) : Pelvis Translation
    // Q(3~5) : Pelvis Orientation (Quaternion x,y,z)
    // Q(6~11) : Right Leg
    // Q(12~17) : Left Leg
    // Q(18) : Waist
    // Q(19) : Pelvis Orientation (Quaternion w)

    VectorNd Qnew = VectorNd::Zero(LIGHT_DOF+1);
    VectorNd dQnew = VectorNd::Zero(LIGHT_DOF);
    VectorNd Tnew = VectorNd::Zero(LIGHT_ACT_DOF);

    ///// Joint Angle(Encoder) ///////////////////////////////////////////////////////////

    for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
        Qnew(ACT_QNUMSTART+idx) = sharedSEN->ENCODER[MC_ID_CH_Pairs[idx].id][MC_ID_CH_Pairs[idx].ch].CurrentAngle*D2R;
        dQnew(ACT_QNUMSTART+idx) = sharedSEN->ENCODER[MC_ID_CH_Pairs[idx].id][MC_ID_CH_Pairs[idx].ch].CurrentAngVel*D2R;
    }

    // LowPass Filtering
    double f_cut_enc = 25.0; // not good under 10.0!
    Qnow.segment(ACT_QNUMSTART,LIGHT_ACT_DOF) = Qnew.segment(ACT_QNUMSTART,LIGHT_ACT_DOF);
    dQnow.segment(ACT_QNUMSTART,LIGHT_ACT_DOF) = Vector_LPF_2nd(dQoo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF),
                                                               dQo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF),
                                                               dQnew.segment(ACT_QNUMSTART,LIGHT_ACT_DOF),
                                                               f_cut_enc);

    Qoo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF) = Qo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF);
    Qo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF) = Qnow.segment(ACT_QNUMSTART,LIGHT_ACT_DOF);
    dQoo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF) = dQo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF);
    dQo.segment(ACT_QNUMSTART,LIGHT_ACT_DOF) = dQnow.segment(ACT_QNUMSTART,LIGHT_ACT_DOF);

    Qnow_ZeroPelvis = Qnow;
    Qnow_ZeroPelvis.segment(0,6) = VectorNd::Zero(6);
    Qnow_ZeroPelvis(QNUM_END) = 1.0;
    dQnow_ZeroPelvis = dQnow;
    dQnow_ZeroPelvis.segment(0,6) = VectorNd::Zero(6);

    ///// Pelvis Orientation Estimation (IMU) //////////////////////////////////////////////////////
    // This quaternion is projected to Yaw rotation zero
    double q0 = sharedSEN->IMU[0].Q[0];
    double q1 = sharedSEN->IMU[0].Q[1];
    double q2 = sharedSEN->IMU[0].Q[2];
    double q3 = sharedSEN->IMU[0].Q[3];

    Matrix3d Rnow_Global2Pel; // Pelvis frame orientation w.r.t the global frame.
    Matrix3d Rnow_Global2Pel_proj;
    Rnow_Global2Pel(0,0) = 1.0-2.0*(q2*q2+q3*q3);
    Rnow_Global2Pel(0,1) = 2.0*(q1*q2-q3*q0);
    Rnow_Global2Pel(0,2) = 2.0*(q0*q2+q1*q3);
    Rnow_Global2Pel(1,0) = 2.0*(q1*q2+q3*q0);
    Rnow_Global2Pel(1,1) = 1.0-2.0*(q1*q1+q3*q3);
    Rnow_Global2Pel(1,2) = 2.0*(q2*q3-q0*q1);
    Rnow_Global2Pel(2,0) = 2.0*(q1*q3-q0*q2);
    Rnow_Global2Pel(2,1) = 2.0*(q0*q1+q2*q3);
    Rnow_Global2Pel(2,2) = 1.0-2.0*(q2*q2+q1*q1);

    if(Cur_RefFrame == REFFRAME_PEL) {
        Qnew(PELVIS_ORI_QNUMSTART+0) = q1;
        Qnew(PELVIS_ORI_QNUMSTART+1) = q2;
        Qnew(PELVIS_ORI_QNUMSTART+2) = q3;
        Qnew(QNUM_END) = q0;
        dQnew(PELVIS_ORI_QNUMSTART+0) = sharedSEN->IMU[0].Wx_G;
        dQnew(PELVIS_ORI_QNUMSTART+1) = sharedSEN->IMU[0].Wy_G;
        dQnew(PELVIS_ORI_QNUMSTART+2) = sharedSEN->IMU[0].Wz_G;
        Rnow_Global2Pel_proj = Rnow_Global2Pel;
    } else if (Cur_RefFrame == REFFRAME_RF) {
        // Assumption : Right Foot does not move in absolute frame (No slip!)
        UpdateKinematics(*Robot, Qnow, dQnow, VectorNd::Zero(LIGHT_DOF));
        Matrix3d Rnow_Pel2RF_Kin = CalcWorld2BodyOrientation(*Robot, Qnow, n_link[PEL], false).transpose()*(CalcWorld2BodyOrientation(*Robot, Qnow, n_link[RAR],false)*R_rar2EE);
        Matrix3d Rnow_Global2RF = Rnow_Global2Pel*Rnow_Pel2RF_Kin;

        double psi = atan2(Rnow_Global2RF(1,0), Rnow_Global2RF(0,0));   // z-axis (Yaw)
        Matrix3d Rnow_YawProj(cos(psi), -sin(psi), 0.0,
                              sin(psi), cos(psi) , 0.0,
                              0.0     , 0.0      , 1.0);
        Rnow_Global2Pel_proj = Rnow_YawProj.transpose()*Rnow_Global2Pel;

        Qnew(QNUM_END) = sqrt(1.0+Rnow_Global2Pel_proj(0,0)+Rnow_Global2Pel_proj(1,1)+Rnow_Global2Pel_proj(2,2))/2.0;
        Qnew(PELVIS_ORI_QNUMSTART+0) = (Rnow_Global2Pel_proj(2,1)-Rnow_Global2Pel_proj(1,2))/4.0/Qnew(QNUM_END);
        Qnew(PELVIS_ORI_QNUMSTART+1) = (Rnow_Global2Pel_proj(0,2)-Rnow_Global2Pel_proj(2,0))/4.0/Qnew(QNUM_END);
        Qnew(PELVIS_ORI_QNUMSTART+2) = (Rnow_Global2Pel_proj(1,0)-Rnow_Global2Pel_proj(0,1))/4.0/Qnew(QNUM_END);

        Vector3d W_B(sharedSEN->IMU[0].Wx_B,sharedSEN->IMU[0].Wy_B,sharedSEN->IMU[0].Wz_B);
        Vector3d W_G = Rnow_Global2Pel_proj*W_B;
        dQnew(PELVIS_ORI_QNUMSTART+0) = W_G(0);
        dQnew(PELVIS_ORI_QNUMSTART+1) = W_G(1);
        dQnew(PELVIS_ORI_QNUMSTART+2) = W_G(2);
    } else if (Cur_RefFrame == REFFRAME_LF) {
        // Assumption : Left Foot does not move in absolute frame (No slip!)
        UpdateKinematics(*Robot, Qnow, dQnow, VectorNd::Zero(LIGHT_DOF));
        Matrix3d Rnow_Pel2LF_Kin = CalcWorld2BodyOrientation(*Robot, Qnow, n_link[PEL], false).transpose()*CalcWorld2BodyOrientation(*Robot, Qnow, n_link[LAR],false)*R_lar2EE;
        Matrix3d Rnow_Global2LF = Rnow_Global2Pel*Rnow_Pel2LF_Kin;

        double psi = atan2(Rnow_Global2LF(1,0), Rnow_Global2LF(0,0));   // z-axis (Yaw)
        Matrix3d Rnow_YawProj(cos(psi), -sin(psi), 0.0,
                              sin(psi), cos(psi) , 0.0,
                              0.0     , 0.0      , 1.0);
        Rnow_Global2Pel_proj = Rnow_YawProj.transpose()*Rnow_Global2Pel;

        Qnew(QNUM_END) = sqrt(1.0+Rnow_Global2Pel_proj(0,0)+Rnow_Global2Pel_proj(1,1)+Rnow_Global2Pel_proj(2,2))/2.0;
        Qnew(PELVIS_ORI_QNUMSTART+0) = (Rnow_Global2Pel_proj(2,1)-Rnow_Global2Pel_proj(1,2))/4.0/Qnew(QNUM_END);
        Qnew(PELVIS_ORI_QNUMSTART+1) = (Rnow_Global2Pel_proj(0,2)-Rnow_Global2Pel_proj(2,0))/4.0/Qnew(QNUM_END);
        Qnew(PELVIS_ORI_QNUMSTART+2) = (Rnow_Global2Pel_proj(1,0)-Rnow_Global2Pel_proj(0,1))/4.0/Qnew(QNUM_END);

        Vector3d W_B(sharedSEN->IMU[0].Wx_B,sharedSEN->IMU[0].Wy_B,sharedSEN->IMU[0].Wz_B);
        Vector3d W_G = Rnow_Global2Pel_proj*W_B;
        dQnew(PELVIS_ORI_QNUMSTART+0) = W_G(0);
        dQnew(PELVIS_ORI_QNUMSTART+1) = W_G(1);
        dQnew(PELVIS_ORI_QNUMSTART+2) = W_G(2);
    } else if (Cur_RefFrame == REFFRAME_GLOBAL) {
        Qnew(PELVIS_ORI_QNUMSTART+0) = q1;
        Qnew(PELVIS_ORI_QNUMSTART+1) = q2;
        Qnew(PELVIS_ORI_QNUMSTART+2) = q3;
        Qnew(QNUM_END) = q0;
        dQnew(PELVIS_ORI_QNUMSTART+0) = sharedSEN->IMU[0].Wx_G;
        dQnew(PELVIS_ORI_QNUMSTART+1) = sharedSEN->IMU[0].Wy_G;
        dQnew(PELVIS_ORI_QNUMSTART+2) = sharedSEN->IMU[0].Wz_G;
        Rnow_Global2Pel_proj = Rnow_Global2Pel;
    }

//    Qnew(PELVIS_ORI_QNUMSTART+0) = q1;
//    Qnew(PELVIS_ORI_QNUMSTART+1) = q2;
//    Qnew(PELVIS_ORI_QNUMSTART+2) = q3;
//    Qnew(QNUM_END) = q0;
//    dQnew(PELVIS_ORI_QNUMSTART+0) = sharedSEN->IMU[0].Wx_G;
//    dQnew(PELVIS_ORI_QNUMSTART+1) = sharedSEN->IMU[0].Wy_G;
//    dQnew(PELVIS_ORI_QNUMSTART+2) = sharedSEN->IMU[0].Wz_G;

    // LowPass Filtering
    double f_cut_pelori = 25.0; // not good under 10.0!
    Qnow.segment(PELVIS_ORI_QNUMSTART,3) = Qnew.segment(PELVIS_ORI_QNUMSTART,3);
    Qnow(QNUM_END) = Qnew(QNUM_END);
    dQnow.segment(PELVIS_ORI_QNUMSTART,3) = Vector_LPF_2nd(dQoo.segment(PELVIS_ORI_QNUMSTART,3),
                                                           dQo.segment(PELVIS_ORI_QNUMSTART,3),
                                                           dQnew.segment(PELVIS_ORI_QNUMSTART,3),
                                                           f_cut_pelori);

    dQoo.segment(PELVIS_ORI_QNUMSTART,3) = dQo.segment(PELVIS_ORI_QNUMSTART,3);
    dQo.segment(PELVIS_ORI_QNUMSTART,3) = dQnow.segment(PELVIS_ORI_QNUMSTART,3);

    ///// Pelvis Position Estimation (IMU+Encoder+Accelerometer) ////////////////////////////////////
    if(Flag_PelvisPositionEstimation) {
        if(Cur_RefFrame == REFFRAME_PEL) {
            Qnew(PELVIS_POS_QNUMSTART+0) = 0.0;
            Qnew(PELVIS_POS_QNUMSTART+1) = 0.0;
            Qnew(PELVIS_POS_QNUMSTART+2) = 0.0;
            dQnew(PELVIS_POS_QNUMSTART+0) = 0.0;
            dQnew(PELVIS_POS_QNUMSTART+1) = 0.0;
            dQnew(PELVIS_POS_QNUMSTART+2) = 0.0;
        } else if (Cur_RefFrame == REFFRAME_RF) {
            // Assumption : Right Foot does not move in absolute frame (No slip!)
            UpdateKinematics(*Robot, Qnow, dQnow, VectorNd::Zero(LIGHT_DOF));
            Vector3d Xnow_Pel_Kin = CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[PEL], zv, false)
                                    - CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[RAR], rar2EE,false); // Right Foot > Pelvis Position in absolute frame
            Vector3d Vnow_Pel_Kin = CalcPointVelocity(*Robot, Qnow, dQnow, n_link[PEL], zv,false)
                                    - CalcPointVelocity(*Robot, Qnow, dQnow, n_link[RAR], rar2EE,false);  // Right Foot > Pelvis Velocity in absolute frame
            Vector3d U_Acc(sharedSEN->IMU[0].Ax_B,sharedSEN->IMU[0].Ay_B,sharedSEN->IMU[0].Az_B);
            Vector3d U_Grav(0.0,0.0,g_const);
            Vector3d Anow_Pel_Acc = Rnow_Global2Pel_proj*U_Acc-U_Grav;

            Vector3d Xnow_Pel_Old = Qnow.segment(PELVIS_POS_QNUMSTART,3);
            Vector3d Vnow_Pel_Old = dQnow.segment(PELVIS_POS_QNUMSTART,3);

            double K = 2.0*3.1415*25.0*SYS_DT_WALKING;
            Vector3d Vnow_Pel_Update = Vnow_Pel_Old + K*(Vnow_Pel_Kin - Vnow_Pel_Old) + Anow_Pel_Acc*SYS_DT_WALKING;
            dQnew.segment(PELVIS_POS_QNUMSTART,3) = Vnow_Pel_Update;

            K = 2.0*3.1415*25.0*SYS_DT_WALKING;
            Vector3d Xnow_Pel_Update = Xnow_Pel_Old + K*(Xnow_Pel_Kin - Xnow_Pel_Old) + Vnow_Pel_Update*SYS_DT_WALKING;
            Qnew.segment(PELVIS_POS_QNUMSTART,3) = Xnow_Pel_Update;

        } else if (Cur_RefFrame == REFFRAME_LF) {
            // Assumption : Right Foot does not move in absolute frame (No slip!)
            UpdateKinematics(*Robot, Qnow, dQnow, VectorNd::Zero(LIGHT_DOF));
            Vector3d Xnow_Pel_Kin = CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[PEL], zv, false)
                                    - CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[LAR], lar2EE,false);  // Left Foot > Pelvis Position in absolute frame
            VectorNd Vnow_Pel_Kin = CalcPointVelocity(*Robot, Qnow, dQnow, n_link[PEL], zv,false)
                                    - CalcPointVelocity(*Robot, Qnow, dQnow, n_link[LAR], lar2EE,false); // Left Foot > Pelvis Velocity in absolute frame
            Vector3d U_Acc(sharedSEN->IMU[0].Ax_B,sharedSEN->IMU[0].Ay_B,sharedSEN->IMU[0].Az_B);
            Vector3d U_Grav(0.0,0.0,g_const);
            Vector3d Anow_Pel_Acc = Rnow_Global2Pel_proj*U_Acc-U_Grav;

            double K = 2.0*3.1415*25.0*SYS_DT_WALKING;
            Vector3d Vnow_Pel_Old = dQnow.segment(PELVIS_POS_QNUMSTART,3);
            Vector3d Vnow_Pel_Update = Vnow_Pel_Old + K*(Vnow_Pel_Kin - Vnow_Pel_Old) + Anow_Pel_Acc*SYS_DT_WALKING;
            dQnew.segment(PELVIS_POS_QNUMSTART,3) = Vnow_Pel_Update;

            K = 2.0*3.1415*25.0*SYS_DT_WALKING;
            Vector3d Xnow_Pel_Old = Qnow.segment(PELVIS_POS_QNUMSTART,3);
            Vector3d Xnow_Pel_Update = Xnow_Pel_Old + K*(Xnow_Pel_Kin - Xnow_Pel_Old) + Vnow_Pel_Update*SYS_DT_WALKING;
            Qnew.segment(PELVIS_POS_QNUMSTART,3) = Xnow_Pel_Update;

        } else if (Cur_RefFrame == REFFRAME_GLOBAL) {
            Qnew(PELVIS_POS_QNUMSTART+0) = 0.0;
            Qnew(PELVIS_POS_QNUMSTART+1) = 0.0;
            Qnew(PELVIS_POS_QNUMSTART+2) = 0.0;
            dQnew(PELVIS_POS_QNUMSTART+0) = 0.0;
            dQnew(PELVIS_POS_QNUMSTART+1) = 0.0;
            dQnew(PELVIS_POS_QNUMSTART+2) = 0.0;
        }
    }

    // LowPass Filtering
//    double f_cut_pelpos = 25.0; // not good under 10.0!
//    Qnow.segment(PELVIS_POS_QNUMSTART,3) = Vector_LPF_2nd(Qoo.segment(PELVIS_POS_QNUMSTART,3),
//                                                           Qo.segment(PELVIS_POS_QNUMSTART,3),
//                                                           Qnew.segment(PELVIS_POS_QNUMSTART,3),
//                                                           f_cut_pelpos);
//    dQnow.segment(PELVIS_POS_QNUMSTART,3) = Vector_LPF_2nd(dQoo.segment(PELVIS_POS_QNUMSTART,3),
//                                                           dQo.segment(PELVIS_POS_QNUMSTART,3),
//                                                           dQnew.segment(PELVIS_POS_QNUMSTART,3),
//                                                           f_cut_pelpos);
    Qnow.segment(PELVIS_POS_QNUMSTART,3) = Qnew.segment(PELVIS_POS_QNUMSTART,3);
    dQnow.segment(PELVIS_POS_QNUMSTART,3) = dQnew.segment(PELVIS_POS_QNUMSTART,3);

    Qoo.segment(PELVIS_POS_QNUMSTART,3) = Qo.segment(PELVIS_POS_QNUMSTART,3);
    Qo.segment(PELVIS_POS_QNUMSTART,3) = Qnow.segment(PELVIS_POS_QNUMSTART,3);
    dQoo.segment(PELVIS_POS_QNUMSTART,3) = dQo.segment(PELVIS_POS_QNUMSTART,3);
    dQo.segment(PELVIS_POS_QNUMSTART,3) = dQnow.segment(PELVIS_POS_QNUMSTART,3);

    ///// Joint Torque ///////////////////////////////////////////////////////////

    for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
        Tnew(idx) = sharedSEN->ENCODER[MC_ID_CH_Pairs[idx].id][MC_ID_CH_Pairs[idx].ch].CurrentTorque;
    }

    double f_cut_JTS = 25.0; // not good under 10.0!
    Tnow = Vector_LPF_2nd(Too,To,Tnew,f_cut_JTS);
    Too = To;
    To = Tnow;

    ///// Foot Torque&Force Sensor ///////////////////////////////////////////////////////////

    double f_cut_EXT = 25.0; // not good under 10.0!

    // Foot FT sensor
    Vector3d TextRF_new(sharedSEN->FT[0].Mx,sharedSEN->FT[0].My,sharedSEN->FT[0].Mz);
    Vector3d FextRF_new(sharedSEN->FT[0].Fx,sharedSEN->FT[0].Fy,sharedSEN->FT[0].Fz);
    Vector3d TextLF_new(sharedSEN->FT[1].Mx,sharedSEN->FT[1].My,sharedSEN->FT[1].Mz);
    Vector3d FextLF_new(sharedSEN->FT[1].Fx,sharedSEN->FT[1].Fy,sharedSEN->FT[1].Fz);
    Textnow_RF_byFT = Vector_LPF_2nd(TextRFoo,TextRFo,TextRF_new,f_cut_EXT);
    Fextnow_RF_byFT = Vector_LPF_2nd(FextRFoo,FextRFo,FextRF_new,f_cut_EXT);
    Textnow_LF_byFT = Vector_LPF_2nd(TextLFoo,TextLFo,TextLF_new,f_cut_EXT);
    Fextnow_LF_byFT = Vector_LPF_2nd(FextLFoo,FextLFo,FextLF_new,f_cut_EXT);
    TextRFoo = TextRFo;
    TextRFo = Textnow_RF_byFT;
    FextRFoo = FextRFo;
    FextRFo = Fextnow_RF_byFT;
    TextLFoo = TextLFo;
    TextLFo = Textnow_LF_byFT;
    FextLFoo = FextLFo;
    FextLFo = Fextnow_LF_byFT;

    /////////// ZMP Information (w.r.t each foot origin) Update //////////////////////////////////////////////////

    CalcFoot2ZMPwith6FT(); // Contact detection and ZMP Calculation

}

void LIGHTWholeBody::UpdateStates()
{

    /////////// End Effector Information (in WorkSpace) Update //////////////////////////////////////////////////    

    UpdateKinematics(*Robot, Qnow, dQnow, VectorNd::Zero(LIGHT_DOF));

    // World Origin > Pelvis
    Rnow_Pel = CalcWorld2BodyOrientation(*Robot, Qnow, n_link[PEL], false);
    Xnow_Pel = CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[PEL], zv, false);
    MatrixNd Vnow_Pel = CalcPointVelocity6D(*Robot, Qnow, dQnow, n_link[PEL], zv, false);
    Wnow_Pel = Vnow_Pel.block(0,0,3,1);
    dXnow_Pel = Vnow_Pel.block(3,0,3,1);

    // World Origin > Right Foot
    Rnow_RF = CalcWorld2BodyOrientation(*Robot, Qnow, n_link[RAR], false)*R_rar2EE;
    Xnow_RF = CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[RAR], rar2EE,false);
    MatrixNd Vnow_RF = CalcPointVelocity6D(*Robot, Qnow, dQnow, n_link[RAR], rar2EE,false);
    Wnow_RF = Vnow_RF.block(0,0,3,1);
    dXnow_RF = Vnow_RF.block(3,0,3,1);

    // World Origin > Left Foot
    Rnow_LF = CalcWorld2BodyOrientation(*Robot, Qnow, n_link[LAR], false)*R_lar2EE;
    Xnow_LF = CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[LAR], lar2EE,false);
    MatrixNd Vnow_LF = CalcPointVelocity6D(*Robot, Qnow, dQnow, n_link[LAR], lar2EE,false);
    Wnow_LF = Vnow_LF.block(0,0,3,1);
    dXnow_LF = Vnow_LF.block(3,0,3,1);

    // World Origin > CoM
    Xnow_CoM = CalcCoMPosition(Qnow);
    dXnow_CoM = CalcCoMVelocity(Qnow,dQnow);

    // Right Foot > Left Foot (Position)
    Xnow_RF2LF = (Xnow_LF-Xnow_RF);
    dXnow_RF2LF = (dXnow_LF-dXnow_RF);

    // Right Foot > CoM (Position)
    Xnow_RF2CoM = (Xnow_CoM-Xnow_RF);
    dXnow_RF2CoM = (dXnow_CoM-dXnow_RF);
    if(CoMzIsPelz) {
        Xnow_RF2CoM(2) = Xnow_Pel(2)-Xnow_RF(2);
        dXnow_RF2CoM(2) = dXnow_Pel(2)-dXnow_RF(2);
    }

    // Left Foot > Right Foot (Position)
    Xnow_LF2RF = (Xnow_RF-Xnow_LF);
    dXnow_LF2RF = (dXnow_RF-dXnow_LF);

    // Left Foot > CoM (Position)
    Xnow_LF2CoM = (Xnow_CoM-Xnow_LF);
    dXnow_LF2CoM = (dXnow_CoM-dXnow_LF);
    if(CoMzIsPelz) {
        Xnow_LF2CoM(2) = Xnow_Pel(2)-Xnow_LF(2);
        dXnow_LF2CoM(2) = dXnow_Pel(2)-dXnow_LF(2);
    }

    Xnow_Pel2Torso = CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[WST], c_link[WST], false) - Xnow_Pel;
    dXnow_Pel2Torso = CalcPointVelocity(*Robot, Qnow, dQnow, n_link[WST], c_link[WST], false) - dXnow_Pel;

    ///////// Wholebody Momentum ///////////////////////////////////////////////////////////////

//    Vector3d Lv = Vector3d::Zero();
//    Vector3d Lw = Vector3d::Zero();

//    for(int i=0;i<14;i++) {
//        VectorNd V = CalcPointVelocity6D(*Robot, Qnow, dQnow, n_link[i], c_link[i], false);

//        // Angular Momentum
//        Matrix3d R = CalcWorld2BodyOrientation(*Robot, Qnow, n_link[i], false);
//        Lw += R*I_link[i]*R.transpose()*V.segment(0,3);

//        // Linear Momentum
//        Lv += m_link[i]*VectorCross2Matrix(CalcBodyToBaseCoordinates(*Robot, Qnow, n_link[i], c_link[i], false))*V.segment(3,3);
//    }

//    static int idx_disp = 0;
//    if(idx_disp%20 == 0)
//    {
//        FILE_LOG(logINFO) << Lw(2)+Lv(2);
//        idx_disp = 0;
//    } idx_disp++;

    ///////// States from Pelvis ///////////////////////////////////////////////////////////////

    UpdateKinematics(*Robot, Qnow_ZeroPelvis, dQnow_ZeroPelvis, VectorNd::Zero(LIGHT_DOF));

    // Pelvis > Right Foot
    Rnow_Pel2RF = CalcWorld2BodyOrientation(*Robot, Qnow_ZeroPelvis, n_link[RAR], false)*R_rar2EE;
    Xnow_Pel2RF = CalcBodyToBaseCoordinates(*Robot, Qnow_ZeroPelvis, n_link[RAR], rar2EE,false);
    MatrixNd Vnow_Pel2RF = CalcPointVelocity6D(*Robot, Qnow_ZeroPelvis, dQnow_ZeroPelvis, n_link[RAR], rar2EE,false);
    Wnow_Pel2RF = Vnow_Pel2RF.block(0,0,3,1);
    dXnow_Pel2RF = Vnow_Pel2RF.block(3,0,3,1);

    // Pelvis > Left Foot
    Rnow_Pel2LF = CalcWorld2BodyOrientation(*Robot, Qnow_ZeroPelvis, n_link[LAR],false)*R_lar2EE;
    Xnow_Pel2LF = CalcBodyToBaseCoordinates(*Robot, Qnow_ZeroPelvis, n_link[LAR], lar2EE,false);
    MatrixNd Vnow_Pel2LF = CalcPointVelocity6D(*Robot, Qnow_ZeroPelvis, dQnow_ZeroPelvis, n_link[LAR], lar2EE,false);
    Wnow_Pel2LF = Vnow_Pel2LF.block(0,0,3,1);
    dXnow_Pel2LF = Vnow_Pel2LF.block(3,0,3,1);

    // Pelvis > Torso CoM
    Xnow_Pel2LF = CalcBodyToBaseCoordinates(*Robot, Qnow_ZeroPelvis, n_link[LAR], lar2EE,false);

}

void LIGHTWholeBody::UpdateReferenceStates(bool UpdateEnable)
{
    if(UpdateEnable) {

        ///// ========== Leg Position Reference in WorkSpace Update ============= /////

        UpdateKinematics(*Robot, Qref, dQref, VectorNd::Zero(LIGHT_DOF));

        // Global Origin > Pelvis
        Rref_Pel = CalcWorld2BodyOrientation(*Robot, Qref, n_link[PEL], false);
        Xref_Pel = CalcBodyToBaseCoordinates(*Robot, Qref, n_link[PEL], zv, false);
        MatrixNd Vref_Pel = CalcPointVelocity6D(*Robot, Qref, dQref, n_link[PEL], zv, false);
        Wref_Pel = Vref_Pel.block(0,0,3,1);
        dXref_Pel = Vref_Pel.block(3,0,3,1);
        MatrixNd Aref_Pel = CalcPointAcceleration6D(*Robot, Qref, dQref, ddQref, n_link[PEL], zv, false);
        dWref_Pel = Aref_Pel.block(0,0,3,1);
        ddXref_Pel = Aref_Pel.block(3,0,3,1);

        // Global Origin > CoM
        Xref_CoM = CalcCoMPosition(Qref);
        dXref_CoM = CalcCoMVelocity(Qref,dQref);
        ddXref_CoM = CalcCoMAcceleration(Qref,dQref,ddQref);

        // Global Origin > Right Foot
        Rref_RF = CalcWorld2BodyOrientation(*Robot, Qref, n_link[RAR],false)*R_rar2EE;
        Xref_RF = CalcBodyToBaseCoordinates(*Robot, Qref, n_link[RAR], rar2EE,false);
        MatrixNd Vref_RF = CalcPointVelocity6D(*Robot, Qref, dQref, n_link[RAR], rar2EE,false);
        Wref_RF = Vref_RF.block(0,0,3,1);
        dXref_RF = Vref_RF.block(3,0,3,1);
        MatrixNd Aref_RF = CalcPointAcceleration6D(*Robot, Qref, dQref, ddQref, n_link[RAR], rar2EE,false);
        dWref_RF = Aref_RF.block(0,0,3,1);
        ddXref_RF = Aref_RF.block(3,0,3,1);

        // Global Origin > Left Foot
        Rref_LF = CalcWorld2BodyOrientation(*Robot, Qref, n_link[LAR],false)*R_lar2EE;
        Xref_LF = CalcBodyToBaseCoordinates(*Robot, Qref, n_link[LAR], lar2EE,false);
        MatrixNd Vref_LF = CalcPointVelocity6D(*Robot, Qref, dQref, n_link[LAR], lar2EE,false);
        Wref_LF = Vref_LF.block(0,0,3,1);
        dXref_LF = Vref_LF.block(3,0,3,1);
        MatrixNd Aref_LF = CalcPointAcceleration6D(*Robot, Qref, dQref, ddQref, n_link[LAR], lar2EE,false);
        dWref_LF = Aref_LF.block(0,0,3,1);
        ddXref_LF = Aref_LF.block(3,0,3,1);

        // Right Foot > Left Foot
        Xref_RF2LF = (Xref_LF-Xref_RF);
        dXref_RF2LF = (dXref_LF-dXref_RF);
        ddXref_RF2LF = (ddXref_LF-ddXref_RF);

        // Right Foot > CoM (Position)
        Xref_RF2CoM = (Xref_CoM-Xref_RF);
        dXref_RF2CoM = (dXref_CoM-dXref_RF);
        ddXref_RF2CoM = (ddXref_CoM-ddXref_RF);
        if(CoMzIsPelz) {
            Xref_RF2CoM(2) = Xref_Pel(2)-Xref_RF(2);
            dXref_RF2CoM(2) = dXref_Pel(2)-dXref_RF(2);
            ddXref_RF2CoM(2) = ddXref_Pel(2)-ddXref_RF(2);
        }

        // Left Foot > Right Foot
        Xref_LF2RF = (Xref_RF-Xref_LF);
        dXref_LF2RF = (dXref_RF-dXref_LF);
        ddXref_LF2RF = (ddXref_RF-ddXref_LF);

        // Left Foot > CoM (Position)
        Xref_LF2CoM = (Xref_CoM-Xref_LF);
        dXref_LF2CoM = (dXref_CoM-dXref_LF);
        ddXref_LF2CoM = (ddXref_CoM-ddXref_LF);
        if(CoMzIsPelz) {
            Xref_LF2CoM(2) = Xref_Pel(2)-Xref_LF(2);
            dXref_LF2CoM(2) = dXref_Pel(2)-dXref_LF(2);
            ddXref_LF2CoM(2) = ddXref_Pel(2)-ddXref_LF(2);
        }

        Vector3d Lv = Vector3d::Zero();
        Vector3d Lw = Vector3d::Zero();

        for(int i=0;i<LIGHT_ACT_DOF+1;i++) {
            VectorNd V = CalcPointVelocity6D(*Robot, Qref, dQref, n_link[i], c_link[i], false);

            // Angular Momentum
            Matrix3d R = CalcWorld2BodyOrientation(*Robot, Qref, n_link[i], false);
            Lw += R*I_link[i]*R.transpose()*V.segment(0,3);

            // Linear Momentum
            Lv += m_link[i]*VectorCross2Matrix(CalcBodyToBaseCoordinates(*Robot, Qref, n_link[i], c_link[i], false))*V.segment(3,3);
        }

        static int idx_disp = 0;
        if(idx_disp%20 == 0)
        {
//            FILE_LOG(logINFO) << "Angular Momentum : " << Lw(2)+Lv(2);
            idx_disp = 0;
        } idx_disp++;

        ///// ========================================================================== /////

        UpdateKinematics(*Robot, Qref_ZeroPelvis, dQref_ZeroPelvis, VectorNd::Zero(LIGHT_DOF));

        // Pelvis > Right Foot
        Rref_Pel2RF = CalcWorld2BodyOrientation(*Robot, Qref_ZeroPelvis, n_link[RAR],false)*R_rar2EE;
        Xref_Pel2RF = CalcBodyToBaseCoordinates(*Robot, Qref_ZeroPelvis, n_link[RAR], rar2EE,false);
        MatrixNd Vref_Pel2RF = CalcPointVelocity6D(*Robot, Qref_ZeroPelvis, dQref_ZeroPelvis, n_link[RAR], rar2EE,false);
        Wref_Pel2RF = Vref_Pel2RF.block(0,0,3,1);
        dXref_Pel2RF = Vref_Pel2RF.block(3,0,3,1);
        MatrixNd Aref_Pel2RF = CalcPointAcceleration6D(*Robot, Qref_ZeroPelvis, dQref_ZeroPelvis, ddQref_ZeroPelvis, n_link[RAR], rar2EE,false);
        dWref_Pel2RF = Aref_Pel2RF.block(0,0,3,1);
        ddXref_Pel2RF = Aref_Pel2RF.block(3,0,3,1);

        // Pelvis > Left Foot
        Rref_Pel2LF = CalcWorld2BodyOrientation(*Robot, Qref_ZeroPelvis, n_link[LAR],false)*R_lar2EE;
        Xref_Pel2LF = CalcBodyToBaseCoordinates(*Robot, Qref_ZeroPelvis, n_link[LAR], lar2EE,false);
        MatrixNd Vref_Pel2LF = CalcPointVelocity6D(*Robot, Qref_ZeroPelvis, dQref_ZeroPelvis, n_link[LAR], lar2EE,false);
        Wref_Pel2LF = Vref_Pel2LF.block(0,0,3,1);
        dXref_Pel2LF = Vref_Pel2LF.block(3,0,3,1);
        MatrixNd Aref_Pel2LF = CalcPointAcceleration6D(*Robot, Qref_ZeroPelvis, dQref_ZeroPelvis, ddQref_ZeroPelvis, n_link[LAR], lar2EE,false);
        dWref_Pel2LF = Aref_Pel2LF.block(0,0,3,1);
        ddXref_Pel2LF = Aref_Pel2LF.block(3,0,3,1);

    } else {
//        FILE_LOG(logWARNING) << " Reference Update is disabled. " << endl;
    }
}



void LIGHTWholeBody::ChangeRefFrame_ReferenceStates(int _PreviousFrame, int _CurrentFrame)
{
    Matrix3d _R;
    Vector3d _r;
    if(_PreviousFrame == REFSTATE_FLOAT||_PreviousFrame == REFSTATE_NOACT) {
        if(_CurrentFrame == REFSTATE_RDSP||_CurrentFrame == REFSTATE_RSSP) {
            _R = ExtractRotZ(Rref_RF).transpose()*ExtractRotZ(Rref_Pel);
            _r = Xref_RF-Xref_Pel;
        } else if(_CurrentFrame == REFSTATE_LDSP||_CurrentFrame == REFSTATE_LSSP) {
            _R = ExtractRotZ(Rref_LF).transpose()*ExtractRotZ(Rref_Pel);
            _r = Xref_LF-Xref_Pel;
        } else {
            _R = I3;
            _r = zv;
        }
    } else if(_PreviousFrame == REFSTATE_RDSP||_PreviousFrame == REFSTATE_RSSP) {
        if(_CurrentFrame == REFSTATE_FLOAT||_CurrentFrame == REFSTATE_NOACT) {
            _R = ExtractRotZ(Rref_Pel).transpose()*ExtractRotZ(Rref_RF);
            _r = Xref_Pel-Xref_RF;
        } else if(_CurrentFrame == REFSTATE_LDSP||_CurrentFrame == REFSTATE_LSSP) {
            _R = ExtractRotZ(Rref_LF).transpose()*ExtractRotZ(Rref_RF);
            _r = Xref_LF-Xref_RF;
//            FILE_LOG(logINFO) << "R_RF2LF : " << _R;
//            FILE_LOG(logINFO) << "r_RF2LF : " << _r;
        } else {
            _R = I3;
            _r = zv;
        }
    } else if(_PreviousFrame == REFSTATE_LDSP||_PreviousFrame == REFSTATE_LSSP) {
        if(_CurrentFrame == REFSTATE_FLOAT||_CurrentFrame == REFSTATE_NOACT) {
            _R = ExtractRotZ(Rref_Pel).transpose()*ExtractRotZ(Rref_LF);
            _r = Xref_Pel-Xref_LF;
        } else if(_CurrentFrame == REFSTATE_RDSP||_CurrentFrame == REFSTATE_RSSP) {
            _R = ExtractRotZ(Rref_RF).transpose()*ExtractRotZ(Rref_LF);
            _r = Xref_RF-Xref_LF;
//            FILE_LOG(logINFO) << "R_LF2RF : " << _R;
//            FILE_LOG(logINFO) << "r_LF2RF : " << _r;
        } else {
            _R = I3;
            _r = zv;
        }
    }

    Rref_Pel        = _R*Rref_Pel;
    Wref_Pel        = _R*Wref_Pel;
    dWref_Pel       = _R*dWref_Pel;
    Xref_Pel        = _R*(Xref_Pel-_r);
    dXref_Pel       = _R*dXref_Pel;
    ddXref_Pel      = _R*ddXref_Pel;

    Xref_CoM        = _R*(Xref_CoM-_r);
    dXref_CoM       = _R*dXref_CoM;
    ddXref_CoM      = _R*ddXref_CoM;

    Rref_RF         = _R*Rref_RF;
    Wref_RF         = _R*Wref_RF;
    dWref_RF        = _R*dWref_RF;
    Xref_RF         = _R*(Xref_RF-_r);
    dXref_RF        = _R*dXref_RF;
    ddXref_RF       = _R*ddXref_RF;

    Rref_LF         = _R*Rref_LF;
    Wref_LF         = _R*Wref_LF;
    dWref_LF        = _R*dWref_LF;
    Xref_LF         = _R*(Xref_LF-_r);
    dXref_LF        = _R*dXref_LF;
    ddXref_LF       = _R*ddXref_LF;

    Xref_RF2LF      = _R*Xref_RF2LF;
    dXref_RF2LF     = _R*dXref_RF2LF;
    ddXref_RF2LF    = _R*ddXref_RF2LF;

    Xref_RF2CoM     = _R*Xref_RF2CoM;
    dXref_RF2CoM    = _R*dXref_RF2CoM;
    ddXref_RF2CoM   = _R*ddXref_RF2CoM;
//    Xref_RF2CoM(2)   = Xref_Pel(2)-Xref_RF(2);
//    dXref_RF2CoM(2)  = dXref_Pel(2)-dXref_RF(2);
//    ddXref_RF2CoM(2) = ddXref_Pel(2)-ddXref_RF(2);

    Xref_LF2RF      = _R*Xref_LF2RF;
    dXref_LF2RF     = _R*dXref_LF2RF;
    ddXref_LF2RF    = _R*ddXref_LF2RF;

    Xref_LF2CoM     = _R*Xref_LF2CoM;
    dXref_LF2CoM    = _R*dXref_LF2CoM;
    ddXref_LF2CoM   = _R*ddXref_LF2CoM;
//    Xref_LF2CoM(2)   = Xref_Pel(2)-Xref_LF(2);
//    dXref_LF2CoM(2)  = dXref_Pel(2)-dXref_LF(2);
//    ddXref_LF2CoM(2) = ddXref_Pel(2)-ddXref_LF(2);

    Qref(QNUM_END) = sqrt(1.0+Rref_Pel(0,0)+Rref_Pel(1,1)+Rref_Pel(2,2))/2.0;
    Qref(PELVIS_ORI_QNUMSTART+0) = (Rref_Pel(2,1)-Rref_Pel(1,2))/4.0/Qref(QNUM_END);
    Qref(PELVIS_ORI_QNUMSTART+1) = (Rref_Pel(0,2)-Rref_Pel(2,0))/4.0/Qref(QNUM_END);
    Qref(PELVIS_ORI_QNUMSTART+2) = (Rref_Pel(1,0)-Rref_Pel(0,1))/4.0/Qref(QNUM_END);
    Qref(PELVIS_POS_QNUMSTART+0) = Xref_Pel(0);
    Qref(PELVIS_POS_QNUMSTART+1) = Xref_Pel(1);
    Qref(PELVIS_POS_QNUMSTART+2) = Xref_Pel(2);

    dQref(PELVIS_ORI_QNUMSTART+0) = Wref_Pel(0);
    dQref(PELVIS_ORI_QNUMSTART+1) = Wref_Pel(1);
    dQref(PELVIS_ORI_QNUMSTART+2) = Wref_Pel(2);
    dQref(PELVIS_POS_QNUMSTART+0) = dXref_Pel(0);
    dQref(PELVIS_POS_QNUMSTART+1) = dXref_Pel(1);
    dQref(PELVIS_POS_QNUMSTART+2) = dXref_Pel(2);

    UpdateKinematics(*Robot, Qref, dQref, VectorNd::Zero(LIGHT_DOF));

    R_Transit = _R;
    r_Transit = _r;
}


void LIGHTWholeBody::CheckStates() {
    // return 0 : No Problem, 1 : Unnormal States
    if (Flag_StateCheck) {
        double Angle_Roll = sharedSEN->IMU[0].Roll;
        double Angle_Pitch = sharedSEN->IMU[0].Pitch;

        if(fabs(Angle_Roll)>17.0*D2R) {
            Flag_UnnormalStates = true; return;
        }

        if(fabs(Angle_Pitch)>17.0*D2R) {
            Flag_UnnormalStates = true; return;
        }

        Flag_UnnormalStates = false;
        return;
    }
}

//==============================//
// Set Present Reference
//==============================//
VectorNd dQref_old = VectorNd::Zero(LIGHT_ACT_DOF);
VectorNd dQref_oold = VectorNd::Zero(LIGHT_ACT_DOF);
VectorNd Tref_old = VectorNd::Zero(LIGHT_ACT_DOF);
VectorNd Tref_oold = VectorNd::Zero(LIGHT_ACT_DOF);

void LIGHTWholeBody::SetReference_Joint(bool UpdateEnable, bool PelvisCompensation)
{
    if(UpdateEnable) {

        ///// ================== Joint Reference Update =================== /////

        // Joint Position Reference Update
        double _REF_ANGLE = 0.0;
        for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
            _REF_ANGLE = Qref(FLOATING_BASE_DOF+idx, 0);
            jCon->SetJointRefAngle(idx, _REF_ANGLE*R2D);
        }

        // Joint Velocity Reference
        VectorNd dQref_new = VectorNd::Zero(LIGHT_ACT_DOF);
        VectorNd dQref_fil = VectorNd::Zero(LIGHT_ACT_DOF);
        for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
            double f_cut = 15.0; // Ankle torque reference
            if(sharedREF->Simulation_DataEnable==0) { // Robot Mode
                dQref_new(idx) = dQref(FLOATING_BASE_DOF+idx, 0) + dQref_Comp(idx, 0);
            } else { // Simulation Mode
                dQref_new(idx) = dQref(FLOATING_BASE_DOF+idx, 0);
            }
//            dQref_new(idx) = dQref(FLOATING_BASE_DOF+idx, 0);
            dQref_fil(idx) = LPF_2nd(dQref_oold(idx),dQref_old(idx),dQref_new(idx),f_cut);
        }
        dQref_oold = dQref_old;
        dQref_old = dQref_fil;
        double _REF_ANGVEL = 0.0;
        for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
            _REF_ANGVEL = dQref_fil(idx, 0);
            jCon->SetJointRefAngVel(idx, _REF_ANGVEL*R2D);
        }

        // Joint Torque Reference
        VectorNd Tref_new = VectorNd::Zero(LIGHT_ACT_DOF);
        VectorNd Tref_fil = VectorNd::Zero(LIGHT_ACT_DOF);
        for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
            double f_cut = 15.0; // Ankle torque reference
            if(sharedREF->Simulation_DataEnable==0) { // Robot Mode
                Tref_new(idx) = Tref(idx, 0) + Tref_Comp(idx, 0);
            } else { // Simulation Mode
                Tref_new(idx) = Tref(idx, 0);
            }
//            Tref_new(idx) = Tref(idx, 0);
            Tref_fil(idx) = LPF_2nd(Tref_oold(idx),Tref_old(idx),Tref_new(idx),f_cut);
        }
        Tref_oold = Tref_old;
        Tref_old = Tref_fil;
        double _REF_TORQUE = 0.0;
        for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
            _REF_TORQUE = Tref_fil(idx, 0);
            jCon->SetJointRefTorque(idx, _REF_TORQUE);
        }

    } else {
//        FILE_LOG(logWARNING) << " Reference Update is disabled. " << endl;
    }

}


void LIGHTWholeBody::SetReference_Actuator(bool UpdateEnable)
{
    int _JOINT, _JOINT1, _JOINT2;
    double _Qref, _Qnow, _dQref, _Tref;
    double _Xref, _dXref, _Fref;
    double _Q1ref, _Q2ref, _Q1now, _Q2now, _dQ1ref, _dQ2ref, _T1ref, _T2ref;
    VectorNd _XXref(2);
    VectorNd _dXXref(2);
    VectorNd _FFref(2);

    if(UpdateEnable) {

        // 1. Hip Roll Joint /////////////////////////////////////////////
        _JOINT = RHR;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipRoll(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

        _JOINT = LHR;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipRoll(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

        // 2. Hip Yaw Joint /////////////////////////////////////////////
        _JOINT = RHY;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipYaw(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

        _JOINT = LHY;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipYaw(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

        // 3. Hip Pitch Joint /////////////////////////////////////////////
        _JOINT = RHP;
        _Qnow = Qnow(ACT_QNUMSTART+_JOINT); // joint 'current' angle
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipPitch(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]

        _JOINT = LHP;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _Qnow = Qnow(ACT_QNUMSTART+_JOINT); // joint 'current' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipPitch(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]

        // 4. Knee Pitch Linkage /////////////////////////////////////////////
        _JOINT = RKN;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _Qnow = Qnow(ACT_QNUMSTART+_JOINT); // joint 'current' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_Knee(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]

        _JOINT = LKN;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _Qnow = Qnow(ACT_QNUMSTART+_JOINT); // joint 'current' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_Knee(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]

        // 5. Ankle Linkage /////////////////////////////////////////////////
        _JOINT1 = RAP; _JOINT2 = RAR;
        _Q1ref = Qref(ACT_QNUMSTART+_JOINT1); // Pitch joint angle
        _Q2ref = Qref(ACT_QNUMSTART+_JOINT2); // Roll joint angle
        _Q1now = Qnow(ACT_QNUMSTART+_JOINT1);
        _Q2now = Qnow(ACT_QNUMSTART+_JOINT2);
        _dQ1ref = dQref(ACT_QNUMSTART+_JOINT1); // Pitch joint angular velocity
        _dQ2ref = dQref(ACT_QNUMSTART+_JOINT2); // Roll joint angular velocity
        _T1ref = jCon->GetJointRefTorque(_JOINT1); // Pitch joint torque
        _T2ref = jCon->GetJointRefTorque(_JOINT2); // Roll joint torque
        Joint2Actuator_Ankle(_Q1now, _Q1ref, _dQ1ref, _T1ref,
                             _Q2now, _Q2ref, _dQ2ref, _T2ref,
                             _XXref, _dXXref, _FFref);

        Sref(_JOINT1) = _XXref(0);
        dSref(_JOINT1) = _dXXref(0);
        Fref(_JOINT1) = _FFref(0);
        Sref(_JOINT2) = _XXref(1);
        dSref(_JOINT2) = _dXXref(1);
        Fref(_JOINT2) = _FFref(1);
        jCon->SetJointRefActPos(_JOINT1, _XXref(0)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActPos(_JOINT2, _XXref(1)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT1, _dXXref(0)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActVel(_JOINT2, _dXXref(1)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT1, _FFref(0)); // [F]
        jCon->SetJointRefActForce(_JOINT2, _FFref(1)); // [F]

        _JOINT1 = LAP; _JOINT2 = LAR;
        _Q1ref = Qref(ACT_QNUMSTART+_JOINT1); // Pitch joint angle
        _Q2ref = Qref(ACT_QNUMSTART+_JOINT2); // Roll joint angle
        _Q1now = Qnow(ACT_QNUMSTART+_JOINT1);
        _Q2now = Qnow(ACT_QNUMSTART+_JOINT2);
        _dQ1ref = dQref(ACT_QNUMSTART+_JOINT1); // Pitch joint angular velocity
        _dQ2ref = dQref(ACT_QNUMSTART+_JOINT2); // Roll joint angular velocity
        _T1ref = jCon->GetJointRefTorque(_JOINT1); // Pitch joint torque
        _T2ref = jCon->GetJointRefTorque(_JOINT2); // Roll joint torque
        Joint2Actuator_Ankle(_Q1now, _Q1ref, _dQ1ref, _T1ref,
                             _Q2now, _Q2ref, _dQ2ref, _T2ref,
                             _XXref, _dXXref, _FFref);

        Sref(_JOINT1) = _XXref(0);
        dSref(_JOINT1) = _dXXref(0);
        Fref(_JOINT1) = _FFref(0);
        Sref(_JOINT2) = _XXref(1);
        dSref(_JOINT2) = _dXXref(1);
        Fref(_JOINT2) = _FFref(1);
        jCon->SetJointRefActPos(_JOINT1, _XXref(0)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActPos(_JOINT2, _XXref(1)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT1, _dXXref(0)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActVel(_JOINT2, _dXXref(1)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT1, _FFref(0)); // [F]
        jCon->SetJointRefActForce(_JOINT2, _FFref(1)); // [F]

        // 6. Waist Yaw Joint /////////////////////////////////////////////
        _JOINT = WST;
        _Qref = Qref(ACT_QNUMSTART+_JOINT); // joint 'reference' angle
        _dQref = dQref(ACT_QNUMSTART+_JOINT); // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_WaistYaw(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        Sref(_JOINT) = _Xref;
        dSref(_JOINT) = _dXref;
        Fref(_JOINT) = _Fref;
        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

    } else {
        FILE_LOG(logWARNING) << " Reference Update is disabled. " << endl;
    }
}

void LIGHTWholeBody::GenerateSupplyPressureReference()
{
    for(int i=0;i<LIGHT_ACT_DOF;i++)
    {
        double Ps_forWalking = 0.0;

        if(IsWalking) { Ps_forWalking = 25.0; }

        double Ps_margin = 0.0;
        double alpha3 = pow(LIGHT_Info->Sa(i)/LIGHT_Info->Sb(i),3.0);
        double MM2MpStoLPM = 0.06;
        double Kv_max = 7.0/sqrt(100.0/2.0); // LPM/sqrt(bar)
        if(dSref(i)>1e-3) {
            sharedREF->ActFlowrateReference_Future[0][i] = LIGHT.LIGHT_Info->Sa(i)*dSref(i)*6.0/100.0;
            Ps_margin = (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(i)*dSref(i)*MM2MpStoLPM)/Kv_max + Ps_forWalking + 5.0;
        } else if(dSref(i)<-1e-3) {
            sharedREF->ActFlowrateReference_Future[0][i] = -LIGHT.LIGHT_Info->Sb(i)*dSref(i)*6.0/100.0;
            Ps_margin = -(alpha3+1.0)*(LIGHT_Info->Sb(i)*dSref(i)*MM2MpStoLPM)/Kv_max + Ps_forWalking + 5.0;
        } else {
            sharedREF->ActFlowrateReference_Future[0][i] = 0.0;
            Ps_margin = Ps_forWalking + 5.0;
        }

        if(i==RAP||i==RAR||i==LAP||i==LAR) {
            sharedREF->LoadPressureReference_Future[0][i] = 0.0; // + (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(j)*dS_prev(j)*MM2MpStoLPM)/Kv_max;
        } else {
            if(Fref(i)>=0.0) {
                sharedREF->LoadPressureReference_Future[0][i] = Fref(i)/LIGHT_Info->Sa(i)*10.0 + Ps_margin; // + (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(j)*dS_prev(j)*MM2MpStoLPM)/Kv_max;
            } else {
                sharedREF->LoadPressureReference_Future[0][i] = -Fref(i)/LIGHT_Info->Sb(i)*10.0 + Ps_margin; // + (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(j)*dS_prev(j)*MM2MpStoLPM)/Kv_max;
            }
        }

//        double alpha3 = pow(LIGHT_Info->Sa(i)/LIGHT_Info->Sb(i),3.0);
//        double MM2MpStoLPM = 0.06;
//        double Kv_max = 7.0/sqrt(100.0/2.0); // LPM/sqrt(bar)
//        if(dSref(i)>=1e-3) {
//            sharedREF->LoadPressureReference_Future[0][i] = Fref(i)/LIGHT_Info->Sa(i)*10.0 + (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(i)*dSref(i)*MM2MpStoLPM)/Kv_max;
//            sharedREF->ActFlowrateReference_Future[0][i] = fabs(LIGHT_Info->Sa(i)*dSref(i)*MM2MpStoLPM);
//        } else if(dSref(i)<=-1e-3) {
//            sharedREF->LoadPressureReference_Future[0][i] = -Fref(i)/LIGHT_Info->Sb(i)*10.0 - (alpha3+1.0)*(LIGHT_Info->Sb(i)*dSref(i)*MM2MpStoLPM)/Kv_max;
//            sharedREF->ActFlowrateReference_Future[0][i] = fabs(LIGHT_Info->Sb(i)*dSref(i)*MM2MpStoLPM);
//        } else {
//            if(Fref(i)>=0.0) {
//                sharedREF->LoadPressureReference_Future[0][i] = Fref(i)/LIGHT_Info->Sa(i)*10.0;
//            } else {
//                sharedREF->LoadPressureReference_Future[0][i] = -Fref(i)/LIGHT_Info->Sb(i)*10.0;
//            }
//            sharedREF->ActFlowrateReference_Future[0][i] = 0.0;
//        }
    }
}


void LIGHTWholeBody::SetFootStiffnDamp_RF(VectorNd _StiffnDamp)
{
    K_RF_ori_OnContact(0) = _StiffnDamp(0);
    K_RF_ori_OnContact(1) = _StiffnDamp(1);
    K_RF_ori_OnContact(2) = _StiffnDamp(2);
    K_RF_pos_OnContact(0) = _StiffnDamp(3);
    K_RF_pos_OnContact(1) = _StiffnDamp(4);
    K_RF_pos_OnContact(2) = _StiffnDamp(5);
    D_RF_ori_OnContact(0) = _StiffnDamp(6);
    D_RF_ori_OnContact(1) = _StiffnDamp(7);
    D_RF_ori_OnContact(2) = _StiffnDamp(8);
    D_RF_pos_OnContact(0) = _StiffnDamp(9);
    D_RF_pos_OnContact(1) = _StiffnDamp(10);
    D_RF_pos_OnContact(2) = _StiffnDamp(11);

    K_RF_ori_OffContact(0) = _StiffnDamp(12);
    K_RF_ori_OffContact(1) = _StiffnDamp(13);
    K_RF_ori_OffContact(2) = _StiffnDamp(14);
    K_RF_pos_OffContact(0) = _StiffnDamp(15);
    K_RF_pos_OffContact(1) = _StiffnDamp(16);
    K_RF_pos_OffContact(2) = _StiffnDamp(17);
    D_RF_ori_OffContact(0) = _StiffnDamp(18);
    D_RF_ori_OffContact(1) = _StiffnDamp(19);
    D_RF_ori_OffContact(2) = _StiffnDamp(20);
    D_RF_pos_OffContact(0) = _StiffnDamp(21);
    D_RF_pos_OffContact(1) = _StiffnDamp(22);
    D_RF_pos_OffContact(2) = _StiffnDamp(23);
}

void LIGHTWholeBody::SetFootStiffnDamp_LF(VectorNd _StiffnDamp)
{
    K_LF_ori_OnContact(0) = _StiffnDamp(0);
    K_LF_ori_OnContact(1) = _StiffnDamp(1);
    K_LF_ori_OnContact(2) = _StiffnDamp(2);
    K_LF_pos_OnContact(0) = _StiffnDamp(3);
    K_LF_pos_OnContact(1) = _StiffnDamp(4);
    K_LF_pos_OnContact(2) = _StiffnDamp(5);
    D_LF_ori_OnContact(0) = _StiffnDamp(6);
    D_LF_ori_OnContact(1) = _StiffnDamp(7);
    D_LF_ori_OnContact(2) = _StiffnDamp(8);
    D_LF_pos_OnContact(0) = _StiffnDamp(9);
    D_LF_pos_OnContact(1) = _StiffnDamp(10);
    D_LF_pos_OnContact(2) = _StiffnDamp(11);

    K_LF_ori_OffContact(0) = _StiffnDamp(12);
    K_LF_ori_OffContact(1) = _StiffnDamp(13);
    K_LF_ori_OffContact(2) = _StiffnDamp(14);
    K_LF_pos_OffContact(0) = _StiffnDamp(15);
    K_LF_pos_OffContact(1) = _StiffnDamp(16);
    K_LF_pos_OffContact(2) = _StiffnDamp(17);
    D_LF_ori_OffContact(0) = _StiffnDamp(18);
    D_LF_ori_OffContact(1) = _StiffnDamp(19);
    D_LF_ori_OffContact(2) = _StiffnDamp(20);
    D_LF_pos_OffContact(0) = _StiffnDamp(21);
    D_LF_pos_OffContact(1) = _StiffnDamp(22);
    D_LF_pos_OffContact(2) = _StiffnDamp(23);
}

void LIGHTWholeBody::UpdateFootStiffnDamp(bool UpdateEnable) {

   if(IsContacted_RF()) {
        bool OnOff = true;
        if(OnOff) {
            K_RF_ori = K_RF_ori_OnContact;
            K_RF_pos = K_RF_pos_OnContact;
            D_RF_ori = D_RF_ori_OnContact;
            D_RF_pos = D_RF_pos_OnContact;
        } else {
            K_RF_ori = zv;
            K_RF_pos = zv;
            D_RF_ori = zv;
            D_RF_pos = zv;
        }
    } else {
        bool OnOff = true;
        if(OnOff) {
            K_RF_ori = K_RF_ori_OffContact;
            K_RF_pos = K_RF_pos_OffContact;
            D_RF_ori = D_RF_ori_OffContact;
            D_RF_pos = D_RF_pos_OffContact;
        } else {
            K_RF_ori = zv;
            K_RF_pos = zv;
            D_RF_ori = zv;
            D_RF_pos = zv;
        }
    }

    if(IsContacted_LF()) {
        bool OnOff = true;
        if(OnOff) {
            K_LF_ori = K_LF_ori_OnContact;
            K_LF_pos = K_LF_pos_OnContact;
            D_LF_ori = D_LF_ori_OnContact;
            D_LF_pos = D_LF_pos_OnContact;
        } else {
            K_LF_ori = zv;
            K_LF_pos = zv;
            D_LF_ori = zv;
            D_LF_pos = zv;
        }
    } else {
        bool OnOff = true;
        if(OnOff) {
            K_LF_ori = K_LF_ori_OffContact;
            K_LF_pos = K_LF_pos_OffContact;
            D_LF_ori = D_LF_ori_OffContact;
            D_LF_pos = D_LF_pos_OffContact;
        } else {
            K_LF_ori = zv;
            K_LF_pos = zv;
            D_LF_ori = zv;
            D_LF_pos = zv;
        }
    }

}

void LIGHTWholeBody::SetActuatorStiffnDamp(bool UpdateEnable)
{
    bool PosCtrlRelaxation = false;
    //// Cartesian Space >> Joint Space Stiffness & Dapming ///////////////////////////////
    MatrixNd K_joint_RF = MatrixNd::Identity(6,6);
    MatrixNd D_joint_RF = MatrixNd::Identity(6,6);
    MatrixNd K_joint_LF = MatrixNd::Identity(6,6);
    MatrixNd D_joint_LF = MatrixNd::Identity(6,6);
    MatrixNd M_temp = MatrixNd::Identity(6,6);

    VectorNd K_joint_RF_new = VectorNd::Zero(6);
    VectorNd K_joint_LF_new = VectorNd::Zero(6);
    VectorNd D_joint_RF_new = VectorNd::Zero(6);
    VectorNd D_joint_LF_new = VectorNd::Zero(6);
    VectorNd K_joint_RF_update = VectorNd::Zero(6);
    VectorNd K_joint_LF_update = VectorNd::Zero(6);
    VectorNd D_joint_RF_update = VectorNd::Zero(6);
    VectorNd D_joint_LF_update = VectorNd::Zero(6);
    double f_cut_update = 50.0;

    static VectorNd K_joint_RF_old = VectorNd::Zero(6);
    static VectorNd K_joint_LF_old = VectorNd::Zero(6);
    static VectorNd D_joint_RF_old = VectorNd::Zero(6);
    static VectorNd D_joint_LF_old = VectorNd::Zero(6);
    static VectorNd K_joint_RF_oold = VectorNd::Zero(6);
    static VectorNd K_joint_LF_oold = VectorNd::Zero(6);
    static VectorNd D_joint_RF_oold = VectorNd::Zero(6);
    static VectorNd D_joint_LF_oold = VectorNd::Zero(6);

    VectorNd Qref_F = Qref.segment(6,LIGHT_ACT_DOF);
    VectorNd dQref_F = dQref.segment(6,LIGHT_ACT_DOF);
    UpdateKinematics(*Robot_FixedBase, Qref_F, dQref_F, VectorNd::Zero(LIGHT_ACT_DOF));
    MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_ACT_DOF);
    CalcPointJacobian6D(*Robot_FixedBase, Qref_F, n_link_F[RAR], rar2EE, J_RF, false);
    MatrixNd J_LF = MatrixNd::Zero(6,LIGHT_ACT_DOF);
    CalcPointJacobian6D(*Robot_FixedBase, Qref_F, n_link_F[LAR], lar2EE, J_LF, false);

    M_temp(0,0) = K_RF_ori(0);
    M_temp(1,1) = K_RF_ori(1);
    M_temp(2,2) = K_RF_ori(2);
    M_temp(3,3) = K_RF_pos(0);
    M_temp(4,4) = K_RF_pos(1);
    M_temp(5,5) = K_RF_pos(2);
    K_joint_RF = J_RF.block(0,0,6,6).transpose()*M_temp*J_RF.block(0,0,6,6);
    K_joint_RF_new(0) = K_joint_RF(0,0);
    K_joint_RF_new(1) = K_joint_RF(1,1);
    K_joint_RF_new(2) = K_joint_RF(2,2);
    if(IsContacted_RF()&&PosCtrlRelaxation) {
        K_joint_RF_new(3) = 0.25*K_joint_RF(3,3);
        K_joint_RF_new(4) = 0.10*K_joint_RF(4,4);
        K_joint_RF_new(5) = 0.10*K_joint_RF(5,5);
    } else {
        K_joint_RF_new(3) = K_joint_RF(3,3);
        K_joint_RF_new(4) = K_joint_RF(4,4);
        K_joint_RF_new(5) = K_joint_RF(5,5);
    }
    K_joint_RF_update = Vector_LPF_2nd(K_joint_RF_oold, K_joint_RF_old, K_joint_RF_new, f_cut_update);
    K_joint_RF_oold = K_joint_RF_old;
    K_joint_RF_old = K_joint_RF_update;

    M_temp(0,0) = D_RF_ori(0);
    M_temp(1,1) = D_RF_ori(1);
    M_temp(2,2) = D_RF_ori(2);
    M_temp(3,3) = D_RF_pos(0);
    M_temp(4,4) = D_RF_pos(1);
    M_temp(5,5) = D_RF_pos(2);
    D_joint_RF = J_RF.block(0,0,6,6).transpose()*M_temp*J_RF.block(0,0,6,6);
    D_joint_RF_new(0) = D_joint_RF(0,0);
    D_joint_RF_new(1) = D_joint_RF(1,1);
    D_joint_RF_new(2) = D_joint_RF(2,2);
    if(IsContacted_RF()&&PosCtrlRelaxation) {
        D_joint_RF_new(3) = 0.50*D_joint_RF(3,3);
        D_joint_RF_new(4) = 0.30*D_joint_RF(4,4);
        D_joint_RF_new(5) = 0.30*D_joint_RF(5,5);
    } else {
        D_joint_RF_new(3) = D_joint_RF(3,3);
        D_joint_RF_new(4) = D_joint_RF(4,4);
        D_joint_RF_new(5) = D_joint_RF(5,5);
    }
    D_joint_RF_update = Vector_LPF_2nd(D_joint_RF_oold, D_joint_RF_old, D_joint_RF_new, f_cut_update);
    D_joint_RF_oold = D_joint_RF_old;
    D_joint_RF_old = D_joint_RF_update;

    M_temp(0,0) = K_LF_ori(0);
    M_temp(1,1) = K_LF_ori(1);
    M_temp(2,2) = K_LF_ori(2);
    M_temp(3,3) = K_LF_pos(0);
    M_temp(4,4) = K_LF_pos(1);
    M_temp(5,5) = K_LF_pos(2);
    K_joint_LF = J_LF.block(0,6,6,6).transpose()*M_temp*J_LF.block(0,6,6,6);
    K_joint_LF_new(0) = K_joint_LF(0,0);
    K_joint_LF_new(1) = K_joint_LF(1,1);
    K_joint_LF_new(2) = K_joint_LF(2,2);
    if(IsContacted_LF()&&PosCtrlRelaxation) {
        K_joint_LF_new(3) = 0.25*K_joint_LF(3,3);
        K_joint_LF_new(4) = 0.10*K_joint_LF(4,4);
        K_joint_LF_new(5) = 0.10*K_joint_LF(5,5);
    } else {
        K_joint_LF_new(3) = K_joint_LF(3,3);
        K_joint_LF_new(4) = K_joint_LF(4,4);
        K_joint_LF_new(5) = K_joint_LF(5,5);
    }
    K_joint_LF_update = Vector_LPF_2nd(K_joint_LF_oold, K_joint_LF_old, K_joint_LF_new, f_cut_update);
    K_joint_LF_oold = K_joint_LF_old;
    K_joint_LF_old = K_joint_LF_update;

    M_temp(0,0) = D_LF_ori(0);
    M_temp(1,1) = D_LF_ori(1);
    M_temp(2,2) = D_LF_ori(2);
    M_temp(3,3) = D_LF_pos(0);
    M_temp(4,4) = D_LF_pos(1);
    M_temp(5,5) = D_LF_pos(2);
    D_joint_LF = J_LF.block(0,6,6,6).transpose()*M_temp*J_LF.block(0,6,6,6);
    D_joint_LF_new(0) = D_joint_LF(0,0);
    D_joint_LF_new(1) = D_joint_LF(1,1);
    D_joint_LF_new(2) = D_joint_LF(2,2);
    if(IsContacted_LF()&&PosCtrlRelaxation) {
        D_joint_LF_new(3) = 0.50*D_joint_LF(3,3);
        D_joint_LF_new(4) = 0.30*D_joint_LF(4,4);
        D_joint_LF_new(5) = 0.30*D_joint_LF(5,5);
    } else {
        D_joint_LF_new(3) = D_joint_LF(3,3);
        D_joint_LF_new(4) = D_joint_LF(4,4);
        D_joint_LF_new(5) = D_joint_LF(5,5);
    }
    D_joint_LF_update = Vector_LPF_2nd(D_joint_LF_oold, D_joint_LF_old, D_joint_LF_new, f_cut_update);
    D_joint_LF_oold = D_joint_LF_old;
    D_joint_LF_old = D_joint_LF_update;

    sharedREF->JointStiffness[RHR] = K_joint_RF_update(0); // Nm/rad
    sharedREF->JointStiffness[RHY] = K_joint_RF_update(1); // Nm/rad
    sharedREF->JointStiffness[RHP] = K_joint_RF_update(2); // Nm/rad
    sharedREF->JointStiffness[RKN] = K_joint_RF_update(3); // Nm/rad
    sharedREF->JointStiffness[RAP] = K_joint_RF_update(4); // Nm/rad
    sharedREF->JointStiffness[RAR] = K_joint_RF_update(5); // Nm/rad
    sharedREF->JointStiffness[LHR] = K_joint_LF_update(0); // Nm/rad
    sharedREF->JointStiffness[LHY] = K_joint_LF_update(1); // Nm/rad
    sharedREF->JointStiffness[LHP] = K_joint_LF_update(2); // Nm/rad
    sharedREF->JointStiffness[LKN] = K_joint_LF_update(3); // Nm/rad
    sharedREF->JointStiffness[LAP] = K_joint_LF_update(4); // Nm/rad
    sharedREF->JointStiffness[LAR] = K_joint_LF_update(5); // Nm/rad

    sharedREF->JointDamping[RHR] = D_joint_RF_update(0); // Nm/(rad/s)
    sharedREF->JointDamping[RHY] = D_joint_RF_update(1); // Nm/(rad/s)
    sharedREF->JointDamping[RHP] = D_joint_RF_update(2); // Nm/(rad/s)
    sharedREF->JointDamping[RKN] = D_joint_RF_update(3); // Nm/(rad/s)
    sharedREF->JointDamping[RAP] = D_joint_RF_update(4); // Nm/(rad/s)
    sharedREF->JointDamping[RAR] = D_joint_RF_update(5); // Nm/(rad/s)
    sharedREF->JointDamping[LHR] = D_joint_LF_update(0); // Nm/(rad/s)
    sharedREF->JointDamping[LHY] = D_joint_LF_update(1); // Nm/(rad/s)
    sharedREF->JointDamping[LHP] = D_joint_LF_update(2); // Nm/(rad/s)
    sharedREF->JointDamping[LKN] = D_joint_LF_update(3); // Nm/(rad/s)
    sharedREF->JointDamping[LAP] = D_joint_LF_update(4); // Nm/(rad/s)
    sharedREF->JointDamping[LAR] = D_joint_LF_update(5); // Nm/(rad/s)

    //// Joint Space >> Actuating Space Stiffness & Dapming /////////////////////////////////////////
    VectorNd K_act_RF = VectorNd::Zero(6);
    VectorNd D_act_RF = VectorNd::Zero(6);
    VectorNd K_act_LF = VectorNd::Zero(6);
    VectorNd D_act_LF = VectorNd::Zero(6);
    double J_temp = 0.0;
    MatrixNd J2_temp(2,2);

    // (Joint 0,1) Hip Yaw and Roll Joint (Rotary)
    K_act_RF(0) = K_joint_RF_update(0)/R2D; // N/rad >> N/deg
    K_act_RF(1) = K_joint_RF_update(1)/R2D; // N/rad >> N/deg
    K_act_LF(0) = K_joint_LF_update(0)/R2D; // N/rad >> N/deg
    K_act_LF(1) = K_joint_LF_update(1)/R2D; // N/rad >> N/deg
    D_act_RF(0) = D_joint_RF_update(0)/R2D; // N/(rad/s) >> N/(deg/s)
    D_act_RF(1) = D_joint_RF_update(1)/R2D; // N/(rad/s) >> N/(deg/s)
    D_act_LF(0) = D_joint_LF_update(0)/R2D; // N/(rad/s) >> N/(deg/s)
    D_act_LF(1) = D_joint_LF_update(1)/R2D; // N/(rad/s) >> N/(deg/s)

    // (Joint 2) Hip Pitch Joint (Linear)
    J_temp = AngularVel2CylinderVel_Hip(Qnow(FLOATING_BASE_DOF+RHP), 1.0); // joint error(rad) >> cylinder error(mm)
    K_act_RF(2) = K_joint_RF_update(2)/J_temp/J_temp*0.001; // N/m >> N/mm
    D_act_RF(2) = D_joint_RF_update(2)/J_temp/J_temp*0.001; // N/(m/s) >> N/(mm/s)
    J_temp = AngularVel2CylinderVel_Hip(Qnow(FLOATING_BASE_DOF+LHP), 1.0); // joint error(rad) >> cylinder error(mm)
    K_act_LF(2) = K_joint_LF_update(2)/J_temp/J_temp*0.001; // N/m >> N/mm
    D_act_LF(2) = D_joint_LF_update(2)/J_temp/J_temp*0.001; // N/(m/s) >> N/(mm/s)

    // (Joint 3) Knee Pitch Joint (Linear)
    J_temp = AngularVel2CylinderVel_Knee(Qnow(FLOATING_BASE_DOF+RKN), 1.0); // joint error(rad) >> cylinder error(mm)
    K_act_RF(3) = K_joint_RF_update(3)/J_temp/J_temp*0.001; // N/m >> N/mm
    D_act_RF(3) = D_joint_RF_update(3)/J_temp/J_temp*0.001; // N/(m/s) >> N/(mm/s)
    J_temp = AngularVel2CylinderVel_Knee(Qnow(FLOATING_BASE_DOF+LKN), 1.0); // joint error(rad) >> cylinder error(mm)
    K_act_LF(3) = K_joint_LF_update(3)/J_temp/J_temp*0.001; // N/m >> N/mm
    D_act_LF(3) = D_joint_LF_update(3)/J_temp/J_temp*0.001; // N/(m/s) >> N/(mm/s)

    // (Joint 4,5) Ankle Joint (Linear)
    MatrixNd MJ_temp = MatrixNd::Zero(2,2);
    MatrixNd M2_temp(2,2);
    J2_temp.block(0,0,2,1) = AngularVel2CylinderVel_Ankle(Qnow(FLOATING_BASE_DOF+RAP),Qnow(FLOATING_BASE_DOF+RAR),1.0,0.0);
    J2_temp.block(0,1,2,1) = AngularVel2CylinderVel_Ankle(Qnow(FLOATING_BASE_DOF+RAP),Qnow(FLOATING_BASE_DOF+RAR),0.0,1.0);
    MJ_temp(0,0) = K_joint_RF_update(4);
    MJ_temp(1,1) = K_joint_RF_update(5);
    M2_temp = J2_temp.transpose().inverse()*MJ_temp*J2_temp.inverse();
    K_act_RF(4) = M2_temp(0,0)*0.001;  // N/m >> N/mm
    K_act_RF(5) = M2_temp(1,1)*0.001;  // N/m >> N/mm
    MJ_temp(0,0) = D_joint_RF_update(4);
    MJ_temp(1,1) = D_joint_RF_update(5);
    M2_temp = J2_temp.transpose().inverse()*MJ_temp*J2_temp.inverse();
    D_act_RF(4) = M2_temp(0,0)*0.001;  // N/m >> N/mm
    D_act_RF(5) = M2_temp(1,1)*0.001;  // N/m >> N/mm

    J2_temp.block(0,0,2,1) = AngularVel2CylinderVel_Ankle(Qnow(FLOATING_BASE_DOF+LAP),Qnow(FLOATING_BASE_DOF+LAR),1.0,0.0);
    J2_temp.block(0,1,2,1) = AngularVel2CylinderVel_Ankle(Qnow(FLOATING_BASE_DOF+LAP),Qnow(FLOATING_BASE_DOF+LAR),0.0,1.0);
    MJ_temp(0,0) = K_joint_LF_update(4);
    MJ_temp(1,1) = K_joint_LF_update(5);
    M2_temp = J2_temp.transpose().inverse()*MJ_temp*J2_temp.inverse();
    K_act_LF(4) = M2_temp(0,0)*0.001;  // N/m >> N/mm
    K_act_LF(5) = M2_temp(1,1)*0.001;  // N/m >> N/mm
    MJ_temp(0,0) = D_joint_LF_update(4);
    MJ_temp(1,1) = D_joint_LF_update(5);
    M2_temp = J2_temp.transpose().inverse()*MJ_temp*J2_temp.inverse();
    D_act_LF(4) = M2_temp(0,0)*0.001;  // N/m >> N/mm
    D_act_LF(5) = M2_temp(1,1)*0.001;  // N/m >> N/mm

    sharedREF->ActuatorStiffness[RHR] = K_act_RF(0);
    sharedREF->ActuatorDamping[RHR] = D_act_RF(0);
    sharedREF->ActuatorStiffness[RHY] = K_act_RF(1);
    sharedREF->ActuatorDamping[RHY] = D_act_RF(1);
    sharedREF->ActuatorStiffness[RHP] = K_act_RF(2);
    sharedREF->ActuatorDamping[RHP] = D_act_RF(2);
    sharedREF->ActuatorStiffness[RKN] = K_act_RF(3);
    sharedREF->ActuatorDamping[RKN] = D_act_RF(3);
    sharedREF->ActuatorStiffness[RAP] = K_act_RF(4);
    sharedREF->ActuatorDamping[RAP] = D_act_RF(4);
    sharedREF->ActuatorStiffness[RAR] = K_act_RF(5);
    sharedREF->ActuatorDamping[RAR] = D_act_RF(5);

    sharedREF->ActuatorStiffness[LHR] = K_act_LF(0);
    sharedREF->ActuatorDamping[LHR] = D_act_LF(0);
    sharedREF->ActuatorStiffness[LHY] = K_act_LF(1);
    sharedREF->ActuatorDamping[LHY] = D_act_LF(1);
    sharedREF->ActuatorStiffness[LHP] = K_act_LF(2);
    sharedREF->ActuatorDamping[LHP] = D_act_LF(2);
    sharedREF->ActuatorStiffness[LKN] = K_act_LF(3);
    sharedREF->ActuatorDamping[LKN] = D_act_LF(3);
    sharedREF->ActuatorStiffness[LAP] = K_act_LF(4);
    sharedREF->ActuatorDamping[LAP] = D_act_LF(4);
    sharedREF->ActuatorStiffness[LAR] = K_act_LF(5);
    sharedREF->ActuatorDamping[LAR] = D_act_LF(5);

    if(sharedREF->PosOrFor_Selection[RHR] == JointControlMode_Torque) { sharedREF->StiffnDampChange[RHR] = true; }
    if(sharedREF->PosOrFor_Selection[RHY] == JointControlMode_Torque) { sharedREF->StiffnDampChange[RHY] = true; }
    if(sharedREF->PosOrFor_Selection[RHP] == JointControlMode_Torque) { sharedREF->StiffnDampChange[RHP] = true; }
    if(sharedREF->PosOrFor_Selection[RKN] == JointControlMode_Torque) { sharedREF->StiffnDampChange[RKN] = true; }
    if(sharedREF->PosOrFor_Selection[RAP] == JointControlMode_Torque) { sharedREF->StiffnDampChange[RAP] = true; }
    if(sharedREF->PosOrFor_Selection[RAR] == JointControlMode_Torque) { sharedREF->StiffnDampChange[RAR] = true; }

    if(sharedREF->PosOrFor_Selection[LHR] == JointControlMode_Torque) { sharedREF->StiffnDampChange[LHR] = true; }
    if(sharedREF->PosOrFor_Selection[LHY] == JointControlMode_Torque) { sharedREF->StiffnDampChange[LHY] = true; }
    if(sharedREF->PosOrFor_Selection[LHP] == JointControlMode_Torque) { sharedREF->StiffnDampChange[LHP] = true; }
    if(sharedREF->PosOrFor_Selection[LKN] == JointControlMode_Torque) { sharedREF->StiffnDampChange[LKN] = true; }
    if(sharedREF->PosOrFor_Selection[LAP] == JointControlMode_Torque) { sharedREF->StiffnDampChange[LAP] = true; }
    if(sharedREF->PosOrFor_Selection[LAR] == JointControlMode_Torque) { sharedREF->StiffnDampChange[LAR] = true; }

}


//==============================//
// Set Joints limitation
//==============================//

void LIGHTWholeBody::SetJointLimit_dQ()
{
    MatrixNd tempM = MatrixNd::Zero(1,1);
    double dt = SYS_DT_WALKING;
    double temp_ub, temp_lb;

    for(int i=0;i<LIGHT_DOF;i++)
    {
        tempM(0,0) = (Qmax(i,0) - Qref(i,0))/dt;
//        tempM(1,0) = dQmax(i,0);
//        tempM(2,0) = sqrt(2*ddQmax(i,0)*(Qmax(i,0) - Qref(i,0)));

        temp_ub = tempM.minCoeff(); // upperbound

        tempM(0,0) = (Qmin(i,0) - Qref(i,0))/dt;
//        tempM(1,0) = dQmin(i,0);
//        tempM(2,0) = -sqrt(2*ddQmin(i,0)*(Qref(i,0) - Qmin(i,0)));

        temp_lb = tempM.maxCoeff();

        dQCon_ub(i,0) = temp_ub;
        dQCon_lb(i,0) = temp_lb;
    }
}

void LIGHTWholeBody::SetJointLimit_ddQ()
{
    MatrixNd tempM = MatrixNd::Zero(3,1);
    double dt = SYS_DT_WALKING;
    double temp_ub, temp_lb;

    for(int i=0;i<LIGHT_DOF;i++)
    {
        tempM(0,0) = ddQmax(i,0);
        tempM(1,0) = (dQmax(i,0) - dQref(i,0))/dt;
        tempM(2,0) = ((Qmax(i,0)-Qref(i,0))/dt-2.0*dQref(i,0))/dt;

        temp_ub = tempM.minCoeff();

        tempM(0,0) = dQmin(i,0);
        tempM(1,0) = (Qmin(i,0) - Qref(i,0))/dt;
        tempM(2,0) = ((Qmin(i,0)-Qref(i,0))/dt-2.0*dQref(i,0))/dt;

        temp_lb = tempM.maxCoeff();

        ddQCon_ub(i,0) = temp_ub;
        ddQCon_lb(i,0) = temp_lb;
    }
}

//==============================//
// CoM Jacobian
//==============================//

MatrixNd LIGHTWholeBody::CalcCoMJacobian(MatrixNd _Q)
{
    MatrixNd jacob_base2pel = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2torso = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_base2rhr = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2rhy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2rhp = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2rkn = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2rap = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2rar = MatrixNd::Zero(3,n_dof);

    MatrixNd jacob_base2lhr = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2lhy = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2lhp = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2lkn = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2lap = MatrixNd::Zero(3,n_dof);
    MatrixNd jacob_base2lar = MatrixNd::Zero(3,n_dof);

    //get CoM Position jacobian
    MatrixNd JCoM = MatrixNd::Zero(3,n_dof);

    // right leg
    CalcPointJacobian(*Robot,_Q,n_link[RHR],c_link[RHR],jacob_base2rhr);
    CalcPointJacobian(*Robot,_Q,n_link[RHY],c_link[RHY],jacob_base2rhy);
    CalcPointJacobian(*Robot,_Q,n_link[RHP],c_link[RHP],jacob_base2rhp);
    CalcPointJacobian(*Robot,_Q,n_link[RKN],c_link[RKN],jacob_base2rkn);
    CalcPointJacobian(*Robot,_Q,n_link[RAP],c_link[RAP],jacob_base2rap);
    CalcPointJacobian(*Robot,_Q,n_link[RAR],c_link[RAR],jacob_base2rar);
    JCoM += jacob_base2rhr*m_link[RHR]/m_robot;
    JCoM += jacob_base2rhy*m_link[RHY]/m_robot;
    JCoM += jacob_base2rhp*m_link[RHP]/m_robot;
    JCoM += jacob_base2rkn*m_link[RKN]/m_robot;
    JCoM += jacob_base2rap*m_link[RAP]/m_robot;
    JCoM += jacob_base2rar*m_link[RAR]/m_robot;

    // left leg
    CalcPointJacobian(*Robot,_Q,n_link[LHR],c_link[LHR],jacob_base2lhr);
    CalcPointJacobian(*Robot,_Q,n_link[LHY],c_link[LHY],jacob_base2lhy);
    CalcPointJacobian(*Robot,_Q,n_link[LHP],c_link[LHP],jacob_base2lhp);
    CalcPointJacobian(*Robot,_Q,n_link[LKN],c_link[LKN],jacob_base2lkn);
    CalcPointJacobian(*Robot,_Q,n_link[LAP],c_link[LAP],jacob_base2lap);
    CalcPointJacobian(*Robot,_Q,n_link[LAR],c_link[LAR],jacob_base2lar);
    JCoM += jacob_base2lhr*m_link[LHR]/m_robot;
    JCoM += jacob_base2lhy*m_link[LHY]/m_robot;
    JCoM += jacob_base2lhp*m_link[LHP]/m_robot;
    JCoM += jacob_base2lkn*m_link[LKN]/m_robot;
    JCoM += jacob_base2lap*m_link[LAP]/m_robot;
    JCoM += jacob_base2lar*m_link[LAR]/m_robot;

    // torso
    CalcPointJacobian(*Robot,_Q,n_link[PEL],c_link[PEL],jacob_base2pel);
    CalcPointJacobian(*Robot,_Q,n_link[WST],c_link[WST],jacob_base2torso);
    JCoM += jacob_base2pel*m_link[PEL]/m_robot;
    JCoM += jacob_base2torso*m_link[WST]/m_robot;

    return JCoM;
}

MatrixNd LIGHTWholeBody::CalcCoMJacobian6D(MatrixNd _Q)
{
    MatrixNd jacob_base2pel = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2torso = MatrixNd::Zero(6,n_dof);

    MatrixNd jacob_base2rhr = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2rhy = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2rhp = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2rkn = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2rap = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2rar = MatrixNd::Zero(6,n_dof);

    MatrixNd jacob_base2lhr = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2lhy = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2lhp = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2lkn = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2lap = MatrixNd::Zero(6,n_dof);
    MatrixNd jacob_base2lar = MatrixNd::Zero(6,n_dof);

    //get CoM Position jacobian
    MatrixNd JCoM = MatrixNd::Zero(6,n_dof);

    // right leg
    CalcPointJacobian6D(*Robot,_Q,n_link[RHR],c_link[RHR],jacob_base2rhr);
    CalcPointJacobian6D(*Robot,_Q,n_link[RHY],c_link[RHY],jacob_base2rhy);
    CalcPointJacobian6D(*Robot,_Q,n_link[RHP],c_link[RHP],jacob_base2rhp);
    CalcPointJacobian6D(*Robot,_Q,n_link[RKN],c_link[RKN],jacob_base2rkn);
    CalcPointJacobian6D(*Robot,_Q,n_link[RAP],c_link[RAP],jacob_base2rap);
    CalcPointJacobian6D(*Robot,_Q,n_link[RAR],c_link[RAR],jacob_base2rar);
    JCoM += jacob_base2rhr*m_link[RHR]/m_robot;
    JCoM += jacob_base2rhy*m_link[RHY]/m_robot;
    JCoM += jacob_base2rhp*m_link[RHP]/m_robot;
    JCoM += jacob_base2rkn*m_link[RKN]/m_robot;
    JCoM += jacob_base2rap*m_link[RAP]/m_robot;
    JCoM += jacob_base2rar*m_link[RAR]/m_robot;

    // left leg
    CalcPointJacobian6D(*Robot,_Q,n_link[LHR],c_link[LHR],jacob_base2lhr);
    CalcPointJacobian6D(*Robot,_Q,n_link[LHY],c_link[LHY],jacob_base2lhy);
    CalcPointJacobian6D(*Robot,_Q,n_link[LHP],c_link[LHP],jacob_base2lhp);
    CalcPointJacobian6D(*Robot,_Q,n_link[LKN],c_link[LKN],jacob_base2lkn);
    CalcPointJacobian6D(*Robot,_Q,n_link[LAP],c_link[LAP],jacob_base2lap);
    CalcPointJacobian6D(*Robot,_Q,n_link[LAR],c_link[LAR],jacob_base2lar);
    JCoM += jacob_base2lhr*m_link[LHR]/m_robot;
    JCoM += jacob_base2lhy*m_link[LHY]/m_robot;
    JCoM += jacob_base2lhp*m_link[LHP]/m_robot;
    JCoM += jacob_base2lkn*m_link[LKN]/m_robot;
    JCoM += jacob_base2lap*m_link[LAP]/m_robot;
    JCoM += jacob_base2lar*m_link[LAR]/m_robot;

    // torso
    CalcPointJacobian6D(*Robot,_Q,n_link[PEL],c_link[PEL],jacob_base2pel);
    CalcPointJacobian6D(*Robot,_Q,n_link[WST],c_link[WST],jacob_base2torso);
    JCoM += jacob_base2pel*m_link[PEL]/m_robot;
    JCoM += jacob_base2torso*m_link[WST]/m_robot;

    return JCoM;
}

Vector3d LIGHTWholeBody::CalcCoMPosition(MatrixNd _Q){

    Vector3d COM = Vector3d::Zero();
    Vector3d temp_CoM_X = Vector3d::Zero();

    // Right Leg
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[RHR], c_link[RHR], false);
    COM += temp_CoM_X*(m_link[RHR]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[RHY], c_link[RHY], false);
    COM += temp_CoM_X*(m_link[RHY]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[RHP], c_link[RHP], false);
    COM += temp_CoM_X*(m_link[RHP]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[RKN], c_link[RKN], false);
    COM += temp_CoM_X*(m_link[RKN]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[RAP], c_link[RAP], false);
    COM += temp_CoM_X*(m_link[RAP]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[RAR], c_link[RAR], false);
    COM += temp_CoM_X*(m_link[RAR]/m_robot);

    // Left Leg
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[LHR], c_link[LHR], false);
    COM += temp_CoM_X*(m_link[LHR]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[LHY], c_link[LHY], false);
    COM += temp_CoM_X*(m_link[LHY]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[LHP], c_link[LHP], false);
    COM += temp_CoM_X*(m_link[LHP]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[LKN], c_link[LKN], false);
    COM += temp_CoM_X*(m_link[LKN]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[LAP], c_link[LAP], false);
    COM += temp_CoM_X*(m_link[LAP]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[LAR], c_link[LAR], false);
    COM += temp_CoM_X*(m_link[LAR]/m_robot);

    // Torso and body
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[WST], c_link[WST], false);
    COM += temp_CoM_X*(m_link[WST]/m_robot);
    temp_CoM_X = CalcBodyToBaseCoordinates(*Robot, _Q, n_link[PEL], c_link[PEL], false);
    COM += temp_CoM_X*(m_link[PEL]/m_robot);

    return COM;
}

Vector3d LIGHTWholeBody::CalcCoMVelocity(MatrixNd _Q,MatrixNd _dQ){

    Vector3d dCOM = Vector3d::Zero();

    // Right Leg
    dCOM += (m_link[RHR]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[RHR], c_link[RHR], false);
    dCOM += (m_link[RHY]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[RHY], c_link[RHY], false);
    dCOM += (m_link[RHY]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[RHY], c_link[RHY], false);
    dCOM += (m_link[RKN]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[RKN], c_link[RKN], false);
    dCOM += (m_link[RAP]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[RAP], c_link[RAP], false);
    dCOM += (m_link[RAR]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[RAR], c_link[RAR], false);

    // Left Leg
    dCOM += (m_link[LHR]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[LHR], c_link[LHR], false);
    dCOM += (m_link[LHY]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[LHY], c_link[LHY], false);
    dCOM += (m_link[LHY]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[LHY], c_link[LHY], false);
    dCOM += (m_link[LKN]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[LKN], c_link[LKN], false);
    dCOM += (m_link[LAP]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[LAP], c_link[LAP], false);
    dCOM += (m_link[LAR]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[LAR], c_link[LAR], false);

    // Torso and body
    dCOM += (m_link[WST]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[WST], c_link[WST], false);
    dCOM += (m_link[PEL]/m_robot)*CalcPointVelocity(*Robot, _Q, _dQ, n_link[PEL], c_link[PEL], false);

    return dCOM;
}

Vector3d LIGHTWholeBody::CalcCoMAcceleration(MatrixNd _Q,MatrixNd _dQ,MatrixNd _ddQ){

    Vector3d ddCOM = Vector3d::Zero();

    // Right Leg
    ddCOM += (m_link[RHR]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[RHR], c_link[RHR], false);
    ddCOM += (m_link[RHY]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[RHY], c_link[RHY], false);
    ddCOM += (m_link[RHY]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[RHY], c_link[RHY], false);
    ddCOM += (m_link[RKN]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[RKN], c_link[RKN], false);
    ddCOM += (m_link[RAP]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[RAP], c_link[RAP], false);
    ddCOM += (m_link[RAR]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[RAR], c_link[RAR], false);

    // Left Leg
    ddCOM += (m_link[LHR]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[LHR], c_link[LHR], false);
    ddCOM += (m_link[LHY]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[LHY], c_link[LHY], false);
    ddCOM += (m_link[LHY]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[LHY], c_link[LHY], false);
    ddCOM += (m_link[LKN]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[LKN], c_link[LKN], false);
    ddCOM += (m_link[LAP]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[LAP], c_link[LAP], false);
    ddCOM += (m_link[LAR]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[LAR], c_link[LAR], false);

    // Torso and body
    ddCOM += (m_link[WST]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[WST], c_link[WST], false);
    ddCOM += (m_link[PEL]/m_robot)*CalcPointAcceleration(*Robot, _Q, _dQ, _ddQ, n_link[PEL], c_link[PEL], false);

    return ddCOM;
}

Vector3d LIGHTWholeBody::CalcCoMdJdQ(MatrixNd _Q, MatrixNd _dQ) {
    //get CoM Position jacobian
    Vector3d dJdQ = Vector3d::Zero();
    VectorNd _ddQ_Zero = VectorNd::Zero(LIGHT_DOF);

    // right leg
    dJdQ += (m_link[RHR]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[RHR],c_link[RHR],false);
    dJdQ += (m_link[RHY]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[RHY],c_link[RHY],false);
    dJdQ += (m_link[RHY]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[RHY],c_link[RHY],false);
    dJdQ += (m_link[RKN]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[RKN],c_link[RKN],false);
    dJdQ += (m_link[RAP]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[RAP],c_link[RAP],false);
    dJdQ += (m_link[RAR]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[RAR],c_link[RAR],false);

    // left leg
    dJdQ += (m_link[LHR]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[LHR],c_link[LHR],false);
    dJdQ += (m_link[LHY]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[LHY],c_link[LHY],false);
    dJdQ += (m_link[LHY]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[LHY],c_link[LHY],false);
    dJdQ += (m_link[LKN]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[LKN],c_link[LKN],false);
    dJdQ += (m_link[LAP]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[LAP],c_link[LAP],false);
    dJdQ += (m_link[LAR]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[LAR],c_link[LAR],false);

    // pelvis and torso
    dJdQ += (m_link[WST]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[WST],c_link[WST],false);
    dJdQ += (m_link[PEL]/m_robot)*CalcPointAcceleration(*Robot,_Q,_dQ,_ddQ_Zero,n_link[PEL],c_link[PEL],false);

    return dJdQ;
}


//==============================//
// Foot Jacobian
//==============================//

void LIGHTWholeBody::UpdateJacobFoot()
{
//    CalcPointJacobian6D(*Robot,Qref,n_link[RAR],rar2EE,jacob_base2RF,1); // right foot jacobian update
//    jacob_base2RF_ori = jacob_base2RF.block(0,0,3,n_dof);
//    jacob_base2RF_pos = jacob_base2RF.block(3,0,3,n_dof);

//    CalcPointJacobian6D(*Robot,Qref,n_link[LAR],lar2EE,jacob_base2LF,1); // left foot jacobian update
//    jacob_base2LF_ori = jacob_base2LF.block(0,0,3,n_dof);
//    jacob_base2LF_pos = jacob_base2LF.block(3,0,3,n_dof);

//    MatrixNd Qref_pel = Qref;
//    Qref_pel.block(0,0,6,1) = MatrixNd::Zero(6,1);
//    CalcPointJacobian6D(*Robot,Qref_pel,n_link[RAR],rar2EE,jacob_pel2RF,1); // right foot jacobian with respect to pelvis update
//    jacob_pel2RF_ori = jacob_pel2RF.block(0,0,3,n_dof);
//    jacob_pel2RF_pos = jacob_pel2RF.block(3,0,3,n_dof);

//    CalcPointJacobian6D(*Robot,Qref_pel,n_link[LAR],lar2EE,jacob_pel2LF,1); // left foot jacobian with respect to pelvis update
//    jacob_pel2LF_ori = jacob_pel2LF.block(0,0,3,n_dof);
//    jacob_pel2LF_pos = jacob_pel2LF.block(3,0,3,n_dof);
}

//==============================//
// Foot External Force Estimation
//==============================//
void LIGHTWholeBody::CalcFextwithTorque() {

    MatrixNd tempM = MatrixNd::Zero(LIGHT_DOF,LIGHT_DOF);
    CompositeRigidBodyAlgorithm(*Robot,Qnow,tempM,false);
    VectorNd tempN = VectorNd::Zero(LIGHT_DOF);
    NonlinearEffects(*Robot,Qnow,dQnow,tempN);
    VectorNd T_inertia = tempM.block(FLOATING_BASE_DOF,FLOATING_BASE_DOF,LIGHT_ACT_DOF,LIGHT_ACT_DOF)*ddQref.segment(FLOATING_BASE_DOF,LIGHT_ACT_DOF);
    VectorNd T_nonlin = tempN.segment(FLOATING_BASE_DOF,LIGHT_ACT_DOF);

    // Right Foot ZMP estimation with 4 load cell(HP,KN,ANK(2))
    MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_DOF);
    CalcPointJacobian6D(*Robot, Qnow, n_link[RAR], rar2EE, J_RF,1); // Right Foot Jacobian
    MatrixNd J = J_RF.block(0,RL_QNUMSTART+2,6,4).transpose();

    VectorNd T_act_RF = Tnow.segment(RL_JNUMSTART+2,4) - T_nonlin.segment(RL_JNUMSTART+2,4);// - T_inertia.segment(RL_JNUMSTART+2,4);

    MatrixNd JJT = J*J.transpose();
    MatrixNd JJT_inv = JJT.inverse();
    VectorNd F_ext_RF = -J.transpose()*JJT_inv*T_act_RF;
    //         L Estimated Force exerted by external environment
    //           @ RIGHT FOOT (w.r.t absolute frame)
    Textnow_RF_byJTS = F_ext_RF.segment(0,3);
    Fextnow_RF_byJTS = F_ext_RF.segment(3,3);

    // Left Foot ZMP estimation with 4 load cell(HP,KN,ANK(2))
    MatrixNd J_LF = MatrixNd::Zero(6,LIGHT_DOF);
    CalcPointJacobian6D(*Robot, Qnow, n_link[LAR], lar2EE, J_LF,1); // Right Foot Jacobian
    J = J_LF.block(0,LL_QNUMSTART+2,6,4).transpose();

    VectorNd T_act_LF = Tnow.segment(LL_JNUMSTART+2,4) - T_nonlin.segment(LL_JNUMSTART+2,4);// - T_inertia.segment(LL_JNUMSTART+2,4);

    JJT = J*J.transpose();
    JJT_inv = JJT.inverse();
    VectorNd F_ext_LF = -J.transpose()*JJT_inv*T_act_LF;
    //         L Estimated Force exerted by external environment
    //           @ LEFT FOOT (w.r.t absolute frame)
    Textnow_LF_byJTS = F_ext_LF.segment(0,3);
    Fextnow_LF_byJTS = F_ext_LF.segment(3,3);
}


//==============================//
// ZMP
//==============================//
// Get ZMP Position with respect to foot origin by using 6axis FT sensor
void LIGHTWholeBody::CalcFoot2ZMPwith6FT() {

//    ////// Contact Detection //////////////////////////////////////////////////////////////////////////
//    const double Fz_min_contactON = 50.0;
//    if(Fextnow_RF_byFT(2) > Fz_min_contactON) {
//        ContactOn_RF();
//    } else {
//        ContactOff_RF();
//    }

//    if(Fextnow_LF_byFT(2) > Fz_min_contactON) {
//        ContactOn_LF();
//    } else {
//        ContactOff_LF();
//    }

    ////// ZMP Calculation //////////////////////////////////////////////////////////////////////////

    const double Fz_min_ZMPcalc = 10.0;

    // RightFoot ZMP  =============================================================
    Vector3d M_foot = Textnow_RF_byFT;
    Vector3d F_foot = Fextnow_RF_byFT;
    if(F_foot(2) > Fz_min_ZMPcalc) {
        
        //// ZMP equation (to get ZMP position p) ///////////////////////////////////////////////////
        // Mo : Moment at FT sensor
        // Mp : Moment at ZMP (=0)
        // F_foot : Force at FT sensor
        // d : FT sensor height
        // Mp = Mo + r_p2o X (F_foot) = 0
        // Mo = p X (F_RF)
        // Mo = -(F_RF) X p    >> [-(F_RF)X] is not full rank, so we have to add a constraint
        //  [Mo_x - d*Fy]   [    0      Fz  ] [px]
        //  [Mo_y + d*Fx] = [   -Fz      0  ]*[py] = tempA*p = tempB
        //  [Mo_z       ]   [    Fy    -Fx  ]
        //////////////////////////////////////////////////////////////////////////////////////////
        
        double d = 0.0300;//sqrt(rar2EE.dot(rar2EE));
        MatrixNd tempA = MatrixNd::Zero(3,2);
        tempA << 0.0, F_foot(2),
                -F_foot(2), 0.0,
                F_foot(1), -F_foot(0);
        MatrixNd tempB = MatrixNd::Zero(3,1);
        tempB << M_foot(0)-d*F_foot(1),
                M_foot(1)+d*F_foot(0),
                M_foot(2);
        MatrixNd tempATA = tempA.transpose()*tempA;
        MatrixNd tempZMP = tempATA.inverse()*tempA.transpose()*tempB;
        Znow_SSP_RF(0) = tempZMP(0,0);
        Znow_SSP_RF(1) = tempZMP(1,0);
        Znow_SSP_RF(2) = 0.0;
    } else {
        Znow_SSP_RF = zv;
    }
    
    // LeftFoot ZMP  =============================================================
    M_foot = Textnow_LF_byFT;
    F_foot = Fextnow_LF_byFT;
    if(F_foot(2) > Fz_min_ZMPcalc) {
        
        //// ZMP equation (to get ZMP position p) ///////////////////////////////////////////////////
        // Mo : Moment at FT sensor
        // Mp : Moment at ZMP (=0)
        // F_foot : Force at FT sensor
        // d : FT sensor height
        // Mp = Mo + r_p2o X (F_foot) = 0
        // Mo = p X (F_RF)
        // Mo = -(F_RF) X p    >> [-(F_RF)X] is not full rank, so we have to add a constraint
        //  [Mo_x - d*Fy]   [    0      Fz  ] [px]
        //  [Mo_y + d*Fx] = [   -Fz      0  ]*[py] = tempA*p = tempB
        //  [Mo_z       ]   [    Fy    -Fx  ]
        //////////////////////////////////////////////////////////////////////////////////////////
        
        double d = 0.0300;// = sqrt(lar2EE.dot(lar2EE));
        MatrixNd tempA = MatrixNd::Zero(3,2);
        tempA << 0, F_foot(2),
                 -F_foot(2), 0,
                 F_foot(1), -F_foot(0);
        MatrixNd tempB = MatrixNd::Zero(3,1);
        tempB << M_foot(0)-d*F_foot(1),
                 M_foot(1)+d*F_foot(0),
                 M_foot(2);
        MatrixNd tempATA = tempA.transpose()*tempA;
        MatrixNd tempZMP = tempATA.inverse()*tempA.transpose()*tempB;
        Znow_SSP_LF(0) = tempZMP(0,0);
        Znow_SSP_LF(1) = tempZMP(1,0);
        Znow_SSP_LF(2) = 0.0;
    } else {
        Znow_SSP_LF = zv;
    }
        
    // Double Support ZMP  =============================================================
    if( !IsContacted_RF() && !IsContacted_LF() ) {
        Znow_DSP_RF = zv;
        Znow_DSP_LF = zv;
    } else {
        // with respect to RF ////////////////////////////
        {
            Vector3d pRF2LF_wrtRF = Rnow_RF.transpose()*Xnow_RF2LF; // Reference Frame : global > RF
    
            Vector3d F_RF_wrtRF = Fextnow_RF_byFT;
            Vector3d M_RF_wrtRF = Textnow_RF_byFT;

            Vector3d F_LF_wrtRF = Rnow_RF.transpose()*Rnow_LF*Fextnow_LF_byFT; // Reference Frame : LF > RF
            Vector3d M_LF_wrtRF = Rnow_RF.transpose()*Rnow_LF*Textnow_LF_byFT; // Reference Frame : LF > RF

            Vector3d F_total_wrtRF = F_RF_wrtRF + F_LF_wrtRF;
            Vector3d M_total_wrtRF = M_RF_wrtRF + M_LF_wrtRF + pRF2LF_wrtRF.cross(F_LF_wrtRF);

            double d = 0.030;// = sqrt(lar2EE.dot(lar2EE));
            MatrixNd tempA = MatrixNd::Zero(3,2);
            tempA << 0, F_total_wrtRF(2),
                     -F_total_wrtRF(2), 0,
                     F_total_wrtRF(1), -F_total_wrtRF(0);
            MatrixNd tempB = MatrixNd::Zero(3,1);
            tempB << M_total_wrtRF(0) - d*F_total_wrtRF(1),
                     M_total_wrtRF(1) + d*F_total_wrtRF(0),
                     M_total_wrtRF(2);
            MatrixNd tempATA = tempA.transpose()*tempA;
            MatrixNd tempZMP = tempATA.inverse()*tempA.transpose()*tempB;
            Znow_DSP_RF(0) = tempZMP(0,0);
            Znow_DSP_RF(1) = tempZMP(1,0);
            Znow_DSP_RF(2) = 0.0;
        }

        // with respect to LF ////////////////////////////
        {
            Vector3d pLF2RF_wrtLF = Rnow_LF.transpose()*Xnow_LF2RF; // Reference Frame : global > LF

            Vector3d F_RF_wrtLF = Rnow_LF.transpose()*Rnow_RF*Fextnow_RF_byFT;  // Reference Frame : RF > LF
            Vector3d M_RF_wrtLF = Rnow_LF.transpose()*Rnow_RF*Textnow_RF_byFT;  // Reference Frame : RF > LF

            Vector3d F_LF_wrtLF = Fextnow_LF_byFT;
            Vector3d M_LF_wrtLF = Textnow_LF_byFT;

            Vector3d F_total_wrtLF = F_RF_wrtLF + F_LF_wrtLF;
            Vector3d M_total_wrtLF = M_RF_wrtLF + M_LF_wrtLF + pLF2RF_wrtLF.cross(F_RF_wrtLF);

            double d = 0.030;// = sqrt(lar2EE.dot(lar2EE));
            MatrixNd tempA = MatrixNd::Zero(3,2);
            tempA << 0, F_total_wrtLF(2),
                     -F_total_wrtLF(2), 0,
                     F_total_wrtLF(1), -F_total_wrtLF(0);
            MatrixNd tempB = MatrixNd::Zero(3,1);
            tempB << M_total_wrtLF(0) - d*F_total_wrtLF(1),
                     M_total_wrtLF(1) + d*F_total_wrtLF(0),
                     M_total_wrtLF(2);
            MatrixNd tempATA = tempA.transpose()*tempA;
            MatrixNd tempZMP = tempATA.inverse()*tempA.transpose()*tempB;
            Znow_DSP_LF(0) = tempZMP(0,0);
            Znow_DSP_LF(1) = tempZMP(1,0);
            Znow_DSP_LF(2) = 0.0;
        }

        //CalcZMP_DSP_RF();
        //CalcZMP_DSP_LF();
    } 
}

void LIGHTWholeBody::CalcFoot2ZMPwithTorque() {

    // Right Foot ZMP estimation with full torque sensor
    MatrixNd J_RF = MatrixNd::Zero(6,LIGHT_DOF);
    CalcPointJacobian6D(*Robot, Qnow, n_link[RAR], rar2EE, J_RF,1); // Right Foot Jacobian
    MatrixNd JT = J_RF.block(0,RL_QNUMSTART,6,6).transpose();

    MatrixNd R = MatrixNd::Zero(6,6);
    R.block(0,0,3,3) = Rnow_RF.transpose();
    R.block(3,3,3,3) = Rnow_RF.transpose();

    VectorNd Tgrav = VectorNd::Zero(LIGHT_DOF,1);
    VectorNd dQ_zero = VectorNd::Zero(LIGHT_DOF,1);
    NonlinearEffects(*Robot, Qnow, dQ_zero, Tgrav);
    VectorNd Tau = Tnow.block(RL_QNUMSTART,0,6,1)-Tgrav.block(RL_QNUMSTART,0,6,1);

    VectorNd F_ext_RF = -R*JT.inverse()*Tau; // RF's external force w.r.t RF

    double temp_Znow_RF_x = 0.0;
    double temp_Znow_RF_y = 0.0;

    if(F_ext_RF(5)<30.0) { // under 30N = 3kg
        temp_Znow_RF_x = 0.0;
        temp_Znow_RF_y = 0.0;
    } else if (std::isnan(F_ext_RF(5))) {
        temp_Znow_RF_x = 0.0;
        temp_Znow_RF_y = 0.0;
    } else {
        temp_Znow_RF_x = -F_ext_RF(1)/F_ext_RF(5);
        temp_Znow_RF_y = F_ext_RF(0)/F_ext_RF(5);
    }

//    FILE_LOG(logINFO) << "Ext Force Est. with Full Torque : " << F_ext_RF.transpose();
//    FILE_LOG(logINFO) << "RF ZMP Pos Est. with TorqueSensor : " << temp_Znow_RF_x;

}
