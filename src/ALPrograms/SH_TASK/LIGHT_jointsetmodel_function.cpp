//#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicJoint.h"
#include "LIGHT_jointsetmodel.h"
#include <iostream>

extern LIGHTJointSet            LIGHTJoints;
extern JointControlClass       *jCon;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

extern INFO_LIGHT LIGHT_Info;

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern int                     PODO_NO;

//==============================//
// Update Variables
//==============================//

void LIGHTJointSet::UpdateJointPosition()
{
    // Right Leg
    for(int idx=0; idx<LEG_DOF; idx++){
        this->Qnow(RL_JNUMSTART+idx, 0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[RL_JNUMSTART+idx].id][MC_ID_CH_Pairs[RL_JNUMSTART+idx].ch].CurrentAngle*D2R;
        this->dQnow(RL_JNUMSTART+idx, 0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[RL_JNUMSTART+idx].id][MC_ID_CH_Pairs[RL_JNUMSTART+idx].ch].CurrentAngVel*D2R;
        this->Tnow(RL_JNUMSTART+idx, 0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[RL_JNUMSTART+idx].id][MC_ID_CH_Pairs[RL_JNUMSTART+idx].ch].CurrentTorque;
    }

    // Left Leg
    for(int idx=0; idx<LEG_DOF; idx++){
        this->Qnow(LL_JNUMSTART+idx, 0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[LL_JNUMSTART+idx].id][MC_ID_CH_Pairs[LL_JNUMSTART+idx].ch].CurrentAngle*D2R;
        this->dQnow(LL_JNUMSTART+idx, 0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[LL_JNUMSTART+idx].id][MC_ID_CH_Pairs[LL_JNUMSTART+idx].ch].CurrentAngVel*D2R;
        this->Tnow(LL_JNUMSTART+idx, 0) = sharedSEN->ENCODER[MC_ID_CH_Pairs[LL_JNUMSTART+idx].id][MC_ID_CH_Pairs[LL_JNUMSTART+idx].ch].CurrentTorque;
    }
}

//==============================//
// Set Present Reference
//==============================//

void LIGHTJointSet::SetReference_Joint(bool UpdateEnable)
{
    if(UpdateEnable) {
        ///// ================== Joint Reference Update =================== /////
        // Joint Position Reference Update
        double _REF_ANGLE = 0.0;
        for(int idx=0; idx<n_dof; idx++){
            _REF_ANGLE = Qref(idx);
            jCon->SetJointRefAngle(idx, _REF_ANGLE*R2D);
        }

        // Joint Velocity Reference
        double _REF_ANGVEL = 0.0;
        for(int idx=0; idx<n_dof; idx++){
            _REF_ANGVEL = dQref(idx);
            if(idx==RAP) _REF_ANGVEL += dQref_CompMy_RANK;
            if(idx==RAR) _REF_ANGVEL += dQref_CompMx_RANK;
            if(idx==LAP) _REF_ANGVEL += dQref_CompMy_LANK;
            if(idx==LAR) _REF_ANGVEL += dQref_CompMx_LANK;
            jCon->SetJointRefAngVel(idx, _REF_ANGVEL*R2D);
        }

        // Joint Torque Reference
        double _REF_TORQUE = 0.0;
        for(int idx=0; idx<n_dof; idx++){
            _REF_TORQUE = Tref(idx);
            if(idx==RAP) _REF_TORQUE += Tref_CompMy_RANK;
            if(idx==RAR) _REF_TORQUE += Tref_CompMx_RANK;
            if(idx==LAP) _REF_TORQUE += Tref_CompMy_LANK;
            if(idx==LAR) _REF_TORQUE += Tref_CompMx_LANK;
            jCon->SetJointRefTorque(idx, _REF_TORQUE);
        }

    } else {
        FILE_LOG(logWARNING) << " Reference Update is disabled. " << endl;
    }

}

void LIGHTJointSet::SetReference_Actuator(bool UpdateEnable)
{
    int _JOINT, _JOINT1, _JOINT2;
    double _Qref, _Qnow, _dQref, _Tref;
    double _Xref, _dXref, _Fref;
    double _Q1ref, _Q2ref, _Q1now, _Q2now, _dQ1ref, _dQ2ref, _T1ref, _T2ref;
    VectorNd _XXref(2);
    VectorNd _dXXref(2);
    VectorNd _FFref(2);
    if(UpdateEnable) {
        // 1. Hip Roll /////////////////////////////////////////////
        _JOINT = RHR;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
//        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipRoll(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

        _JOINT = LHR;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
//        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipRoll(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

        // 2. Hip Yaw /////////////////////////////////////////////
        _JOINT = RHY;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
//        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipYaw(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

        _JOINT = LHY;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
//        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipYaw(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]


        // 3. Hip Pitch Linkage /////////////////////////////////////////////
        _JOINT = RHP;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipPitch(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]

        _JOINT = LHP;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_HipPitch(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]


        // 4. Knee Pitch Linkage /////////////////////////////////////////////
        _JOINT = RKN;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_Knee(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]

        _JOINT = LKN;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
        _Qnow = sharedSEN->ENCODER[_JOINT][0].CurrentAngle*D2R; // joint 'current' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_Knee(_Qnow,_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT, _dXref*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [F]

        // 5. Ankle Linkage /////////////////////////////////////////////////
        _JOINT1 = RAP; _JOINT2 = RAR;
        _Q1ref = jCon->GetJointRefAngle(_JOINT1)*D2R; // joint angle
        _Q2ref = jCon->GetJointRefAngle(_JOINT2)*D2R; // joint angle
        _Q1now = sharedSEN->ENCODER[_JOINT1][0].CurrentAngle*D2R;
        _Q2now = sharedSEN->ENCODER[_JOINT2][0].CurrentAngle*D2R;
        _dQ1ref = jCon->GetJointRefAngVel(_JOINT1)*D2R; // joint angular velocity
        _dQ2ref = jCon->GetJointRefAngVel(_JOINT2)*D2R; // joint angular velocity
        _T1ref = jCon->GetJointRefTorque(_JOINT1); // joint torque
        _T2ref = jCon->GetJointRefTorque(_JOINT2); // joint torque
        Joint2Actuator_Ankle(_Q1now, _Q1ref, _dQ1ref, _T1ref,
                             _Q2now, _Q2ref, _dQ2ref, _T2ref,
                             _XXref, _dXXref, _FFref);

        jCon->SetJointRefActPos(_JOINT1, _XXref(0)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActPos(_JOINT2, _XXref(1)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT1, _dXXref(0)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActVel(_JOINT2, _dXXref(1)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT1, _FFref(0)); // [F]
        jCon->SetJointRefActForce(_JOINT2, _FFref(1)); // [F]

        _JOINT1 = LAP; _JOINT2 = LAR;
        _Q1ref = jCon->GetJointRefAngle(_JOINT1)*D2R; // joint angle
        _Q2ref = jCon->GetJointRefAngle(_JOINT2)*D2R; // joint angle
        _Q1now = sharedSEN->ENCODER[_JOINT1][0].CurrentAngle*D2R;
        _Q2now = sharedSEN->ENCODER[_JOINT2][0].CurrentAngle*D2R;
        _dQ1ref = jCon->GetJointRefAngVel(_JOINT1)*D2R; // joint angular velocity
        _dQ2ref = jCon->GetJointRefAngVel(_JOINT2)*D2R; // joint angular velocity
        _T1ref = jCon->GetJointRefTorque(_JOINT1); // joint torque
        _T2ref = jCon->GetJointRefTorque(_JOINT2); // joint torque
        Joint2Actuator_Ankle(_Q1now, _Q1ref, _dQ1ref, _T1ref,
                             _Q2now, _Q2ref, _dQ2ref, _T2ref,
                             _XXref, _dXXref, _FFref);

        jCon->SetJointRefActPos(_JOINT1, _XXref(0)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActPos(_JOINT2, _XXref(1)*1000.0); // [m] >> [mm]
        jCon->SetJointRefActVel(_JOINT1, _dXXref(0)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActVel(_JOINT2, _dXXref(1)*1000.0); // [m/s] >> [mm/s]
        jCon->SetJointRefActForce(_JOINT1, _FFref(0)); // [F]
        jCon->SetJointRefActForce(_JOINT2, _FFref(1)); // [F]

        // 6. Waist Yaw /////////////////////////////////////////////
        _JOINT = WST;
        _Qref = jCon->GetJointRefAngle(_JOINT)*D2R; // joint 'reference' angle
        _dQref = jCon->GetJointRefAngVel(_JOINT)*D2R; // joint angular velocity
        _Tref = jCon->GetJointRefTorque(_JOINT); // joint torque
        Joint2Actuator_WaistYaw(_Qref,_dQref,_Tref,_Xref,_dXref,_Fref);

        jCon->SetJointRefActPos(_JOINT, _Xref*R2D); // [rad] >> [deg]
        jCon->SetJointRefActVel(_JOINT, _dXref*R2D); // [rad/s] >> [deg/s]
        jCon->SetJointRefActForce(_JOINT, _Fref); // [Nm]

    } else {
        FILE_LOG(logWARNING) << " Reference Update is disabled. " << endl;
    }
}

///////////////////////////////////////////////
///   [ Joint Info >> Actuator Info ]
/// ///////////////////////////////////////////

// 1. Hip Yaw ////////////////////////////////////////////////////////////
void LIGHTJointSet::Joint2Actuator_HipYaw(double theta, double dtheta, double T,
                                           double &S, double &dS, double &F)
{
    S = theta;
    dS = dtheta;
    F = T;
}

// 2. Hip Roll ////////////////////////////////////////////////////////////
void LIGHTJointSet::Joint2Actuator_HipRoll(double theta, double dtheta, double T,
                                           double &S, double &dS, double &F)
{
    S = theta;
    dS = dtheta;
    F = T;
}

// 3. Hip Pitch ////////////////////////////////////////////////////////////
void LIGHTJointSet::Joint2Actuator_HipPitch(double theta_m, double theta, double dtheta, double T,
                                            double &S, double &dS, double &F)
{
    if(theta_m < LIGHT_Info->Qmin(LHP)) theta_m = LIGHT_Info->Qmin(LHP);
    else if (theta_m > LIGHT_Info->Qmax(LHP)) theta_m = LIGHT_Info->Qmax(LHP);
    if(theta < LIGHT_Info->Qmin(LHP)) theta = LIGHT_Info->Qmin(LHP);
    else if (theta > LIGHT_Info->Qmax(LHP)) theta = LIGHT_Info->Qmax(LHP);

    double l0 = LIGHT_Info->HipLink.l0;
    double l1 = LIGHT_Info->HipLink.l1;
    double alpha = LIGHT_Info->HipLink.alpha;
    double S_ini = LIGHT_Info->HipLink.x_off; // initial actuator position offset (when hip joint is fully contracted)

    // cylinder position (absolute)
    double x = sqrt(l0*l0+l1*l1-2*l0*l1*cos(alpha-theta));
    S = x-S_ini;

    // Jacobian for joint >> actuator
    double temp_dth = 1.0;
    double J = -l0*l1*sin(alpha-theta_m)*temp_dth/x; // when dth1 = 1.0, dS = J

    dS = J*dtheta;
    F = T/J;
}

// 4. Knee Pitch ////////////////////////////////////////////////////////////


void LIGHTJointSet::Joint2Actuator_Knee(double theta_m, double theta, double dtheta, double T,
                                        double &S, double &dS, double &F)
{
    if(theta_m < LIGHT_Info->Qmin(LKN)) theta_m = LIGHT_Info->Qmin(LKN);
    else if (theta_m > LIGHT_Info->Qmax(LKN)) theta_m = LIGHT_Info->Qmax(LKN);
    if(theta < LIGHT_Info->Qmin(LKN)) theta = LIGHT_Info->Qmin(LKN);
    else if (theta > LIGHT_Info->Qmax(LKN)) theta = LIGHT_Info->Qmax(LKN);

    theta_m = -theta_m; // knee joint value has negative direction
    theta = -theta;
    dtheta = -dtheta;
    T = -T;

    double l0 = LIGHT_Info->KneeLink.l0;
    double beta = LIGHT_Info->KneeLink.beta;
    double d1 = LIGHT_Info->KneeLink.d1;
    double d2 = LIGHT_Info->KneeLink.d2;
    double d3 = LIGHT_Info->KneeLink.d3;
    double d4 = LIGHT_Info->KneeLink.d4;
    double delta = LIGHT_Info->KneeLink.delta;
    double S_ini = LIGHT_Info->KneeLink.x_off; // initial actuator position offset (when knee joint is fully contracted)

    double th2 = theta+delta;
    double l2 = sqrt(d1*d1+d3*d3-2.0*d1*d3*cos(th2));
    double th13 = acos((d1*d1+l2*l2-d3*d3)/(2.0*d1*l2));
    double th14 = acos((d2*d2+l2*l2-d4*d4)/(2.0*d2*l2));
    double th1 = th13+th14;

    // cylinder position
    double x = sqrt(l0*l0+d2*d2-2*l0*d2*cos(PI-th1+beta));
    S = x-S_ini;

    // Jacobian for joint >> actuator
    double temp_dth = 1.0;
    th2 = theta_m+delta;
    l2 = sqrt(d1*d1+d3*d3-2.0*d1*d3*cos(th2));
    th13 = acos((d1*d1+l2*l2-d3*d3)/(2.0*d1*l2));
    th14 = acos((d2*d2+l2*l2-d4*d4)/(2.0*d2*l2));
    th1 = th13+th14;
    double dl2 = (d1*d3*sin(th2)/l2)*temp_dth;
    double dth1 = ((d2*cos(th14)-l2)/(d2*sin(th14)*l2)+(d1*cos(th13)-l2)/(d1*sin(th13)*l2))*dl2;
    double J = -l0*d2*sin(PI-th1+beta)/x*dth1;

    dS = J*dtheta;
    F = T/J;
}


// 5. Ankle Pitch and Roll ////////////////////////////////////////////////////////////
void LIGHTJointSet::Joint2Actuator_Ankle(double theta_m, double theta, double dtheta, double Tp,
                                         double phi_m, double phi, double dphi, double Tr,
                                         VectorNd &S, VectorNd &dS, VectorNd &F)
{
    if(theta_m < LIGHT_Info->Qmin(LAP)) theta_m = LIGHT_Info->Qmin(LAP);
    else if (theta_m > LIGHT_Info->Qmax(LAP)) theta_m = LIGHT_Info->Qmax(LAP);
    if(phi_m < LIGHT_Info->Qmin(LAR)) phi_m = LIGHT_Info->Qmin(LAR);
    else if (phi_m > LIGHT_Info->Qmax(LAR)) phi_m = LIGHT_Info->Qmax(LAR);

    if(theta < LIGHT_Info->Qmin(LAP)) theta = LIGHT_Info->Qmin(LAP);
    else if (theta > LIGHT_Info->Qmax(LAP)) theta = LIGHT_Info->Qmax(LAP);
    if(phi < LIGHT_Info->Qmin(LAR)) phi = LIGHT_Info->Qmin(LAR);
    else if (phi > LIGHT_Info->Qmax(LAR)) phi = LIGHT_Info->Qmax(LAR);

    theta_m = theta_m - LIGHT_Info->AnkleLink.theta_off;
    theta = theta - LIGHT_Info->AnkleLink.theta_off;

    // Point O : Ankle Joint Center
    // Point P : Joint Link(Universal Joint) Center
    // Point Q1 : Right Link(connected to cylinder) Rodend Center
    // Point Q2 : Left Link(connected to cylinder) Rodend Center
    // Point C1 : Right Cylinder base
    // Point C2 : Left Cylinder base
    Vector3d OP = LIGHT_Info->AnkleLink.OP;
    Vector3d PQ1 = LIGHT_Info->AnkleLink.PQ_R; // right : 1
    Vector3d PQ2 = LIGHT_Info->AnkleLink.PQ_L; // left : 2
    Vector3d OC1 = LIGHT_Info->AnkleLink.OC_R;
    Vector3d OC2 = LIGHT_Info->AnkleLink.OC_L;
    double alpha = LIGHT_Info->AnkleLink.alpha; // inclined cylinder angle
    double l = LIGHT_Info->AnkleLink.l; // intermedium link length
    double Sr_off = LIGHT_Info->AnkleLink.x_off_R;
    double Sl_off = LIGHT_Info->AnkleLink.x_off_L;

    // cylinder position
    Matrix3d R_p = Matrix3d(cos(theta),      0.0, sin(theta),
                                   0.0,      1.0,        0.0,
                           -sin(theta),      0.0, cos(theta));
    Matrix3d R_r = Matrix3d(       1.0,      0.0,        0.0,
                                   0.0, cos(phi),   -sin(phi),
                                   0.0, sin(phi),    cos(phi));
    Vector3d p = Vector3d(sin(alpha), 0, -cos(alpha));

    Vector3d OQ1 = R_p*(R_r*PQ1+OP); // Reference frame : kalf link
    Vector3d OQ2 = R_p*(R_r*PQ2+OP); // Reference frame : kalf link

    Vector3d C1Q1 = OQ1-OC1;
    double tempA = p.dot(C1Q1);
    double tempB = C1Q1.dot(C1Q1)-tempA*tempA;
    double x1 = tempA-sqrt(l*l-tempB);

    Vector3d C2Q2 = OQ2-OC2;
    tempA = p.dot(C2Q2);
    tempB = C2Q2.dot(C2Q2)-tempA*tempA;
    double x2 = tempA-sqrt(l*l-tempB);

    S << (x1-Sr_off),(x2-Sl_off);


    // Jacobian for joint >> actuator

    R_p = Matrix3d(cos(theta_m),      0.0, sin(theta_m),
                          0.0,      1.0,        0.0,
                  -sin(theta_m),      0.0, cos(theta_m));
    R_r = Matrix3d(       1.0,      0.0,        0.0,
                          0.0, cos(phi_m),   -sin(phi_m),
                          0.0, sin(phi_m),    cos(phi_m));
    p = Vector3d(sin(alpha), 0, -cos(alpha));

    OQ1 = R_p*(R_r*PQ1+OP); // Reference frame : kalf link
    OQ2 = R_p*(R_r*PQ2+OP); // Reference frame : kalf link
    C1Q1 = OQ1-OC1;
    tempA = p.dot(C1Q1);
    tempB = C1Q1.dot(C1Q1)-tempA*tempA;
    x1 = tempA-sqrt(l*l-tempB);
    C2Q2 = OQ2-OC2;
    tempA = p.dot(C2Q2);
    tempB = C2Q2.dot(C2Q2)-tempA*tempA;
    x2 = tempA-sqrt(l*l-tempB);

    Matrix3d dR_p = Matrix3d(-sin(theta_m),     0.0, cos(theta_m),
                                     0.0,     0.0,        0.0,
                             -cos(theta_m),     0.0,-sin(theta_m));
    Matrix3d dR_r = Matrix3d(        0.0,       0.0,       0.0,
                                     0.0, -sin(phi_m), -cos(phi_m),
                                     0.0,  cos(phi_m), -sin(phi_m));

    Vector3d dC1Q1_1 = dR_p*(R_r*PQ1+OP);
    Vector3d dC1Q1_2 = R_p*(dR_r*PQ1);
    MatrixNd J_dC1Q1(3,2);
    J_dC1Q1.block(0,0,3,1) = dC1Q1_1;
    J_dC1Q1.block(0,1,3,1) = dC1Q1_2;
    MatrixNd J_x1 = 1.0/(x1-p.dot(C1Q1))*(x1*p.transpose()-C1Q1.transpose())*J_dC1Q1;

    Vector3d dC2Q2_1 = dR_p*(R_r*PQ2+OP);
    Vector3d dC2Q2_2 = R_p*(dR_r*PQ2);
    MatrixNd J_dC2Q2(3,2);
    J_dC2Q2.block(0,0,3,1) = dC2Q2_1;
    J_dC2Q2.block(0,1,3,1) = dC2Q2_2;
    MatrixNd J_x2 = 1.0/(x2-p.dot(C2Q2))*(x2*p.transpose()-C2Q2.transpose())*J_dC2Q2;

    // Jacobian for joint angular velocity >> cylinder velocity (v = J*thetadot)
    MatrixNd J(2,2);
    J.block(0,0,1,2) = J_x1; // for pitch joint velocity
    J.block(1,0,1,2) = J_x2; // for roll joint velocity

    VectorNd w(2);
    w << dtheta,dphi;
    dS = J*w;

    VectorNd T(2);
    T << Tp,Tr;
    F = J.transpose().inverse()*T;
}

// 6. Waist Yaw ////////////////////////////////////////////////////////////
void LIGHTJointSet::Joint2Actuator_WaistYaw(double theta, double dtheta, double T,
                                           double &S, double &dS, double &F)
{
    S = theta;
    dS = dtheta;
    F = T;
}
