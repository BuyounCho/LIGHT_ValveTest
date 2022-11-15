/*============================================================================
 *
 *  <light hopping joint-level functions>
 *
 *
 *                -  Buyoun,Cho 2018.05.16
 *
=============================================================================*/

#include "LIGHT_robotmodel.h"
#include "LIGHT_dynamics.h"
#include "LIGHT_kinematics.h"

#include "ManualCAN.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern int     PODO_NO;

extern INFO_LIGHT LIGHT_Info;

///////////////////////////////////////////////
///   [ Joint Info >> Actuator Info ]
/// ///////////////////////////////////////////

// 1. Hip Roll ////////////////////////////////////////////////////////////
void LIGHTWholeBody::Joint2Actuator_HipRoll(double theta, double dtheta, double T,
                                           double &S, double &dS, double &F)
{
    S = theta;
    dS = dtheta;
    F = T;
}

// 2. Hip Yaw ////////////////////////////////////////////////////////////
void LIGHTWholeBody::Joint2Actuator_HipYaw(double theta, double dtheta, double T,
                                           double &S, double &dS, double &F)
{
    S = theta;
    dS = dtheta;
    F = T;
}

// 3. Hip Pitch ////////////////////////////////////////////////////////////
void LIGHTWholeBody::Joint2Actuator_HipPitch(double theta_m, double theta, double dtheta, double T,
                                             double &S, double &dS, double &F)
{
    if(theta < LIGHT_Info->Qmin(LHP)) theta = LIGHT_Info->Qmin(LHP);
    else if (theta > LIGHT_Info->Qmax(LHP)) theta = LIGHT_Info->Qmax(LHP);
    if(theta_m < LIGHT_Info->Qmin(LHP)) theta_m = LIGHT_Info->Qmin(LHP);
    else if (theta_m > LIGHT_Info->Qmax(LHP)) theta_m = LIGHT_Info->Qmax(LHP);

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

double  LIGHTWholeBody::Angle2CylinderPos_Hip(double theta){
    // input : relative hip angle (0~fullangle) [rad]
    // output : relative cylinder length (0~stroke) [m]

    double l0 = LIGHT_Info->HipLink.l0;
    double l1 = LIGHT_Info->HipLink.l1;
    double alpha = LIGHT_Info->HipLink.alpha;

    double th1 = theta;
    if(th1 < -1.0*D2R) th1 = -1.0*D2R;
    else if (th1 > 100.0*D2R) th1 = 100.0*D2R;

    // cylinder position
    double x_real = sqrt(l0*l0+l1*l1-2*l0*l1*cos(alpha-th1));
    double x_off = LIGHT_Info->HipLink.x_off;

    return (x_real-x_off);
}

double  LIGHTWholeBody::AngularVel2CylinderVel_Hip(double theta, double dtheta){
    double l0 = LIGHT_Info->HipLink.l0;
    double l1 = LIGHT_Info->HipLink.l1;
    double alpha = LIGHT_Info->HipLink.alpha;

    double th1 = theta;
    if(th1 < -1.0*D2R) th1 = -1.0*D2R;
    else if (th1 > 100.0*D2R) th1 = 100.0*D2R;

    // cylinder position
    double x = sqrt(l0*l0+l1*l1-2*l0*l1*cos(alpha-th1));

    // cylinder velocity
    double dth1 = dtheta;
    double dx = -l0*l1*sin(alpha-th1)/x*dth1;

    return dx;
}

double  LIGHTWholeBody::Torque2CylinderForce_Hip(double theta, double T){
    double l0 = LIGHT_Info->HipLink.l0;
    double l1 = LIGHT_Info->HipLink.l1;
    double alpha = LIGHT_Info->HipLink.alpha;

    double th1 = theta;
    if(th1 < -1.0*D2R) th1 = -1.0*D2R;
    else if (th1 > 100.0*D2R) th1 = 100.0*D2R;

    double x = sqrt(l0*l0+l1*l1-2*l0*l1*cos(alpha-th1));

    double F = -x/(l0*l1*sin(alpha-th1))*T;

    return F;
}

// 4. Knee Pitch ////////////////////////////////////////////////////////////

void LIGHTWholeBody::Joint2Actuator_Knee(double theta_m, double theta, double dtheta, double T,
                                         double &S, double &dS, double &F)
{
    if(theta < LIGHT_Info->Qmin(LKN)) theta = LIGHT_Info->Qmin(LKN);
    else if (theta > LIGHT_Info->Qmax(LKN)) theta = LIGHT_Info->Qmax(LKN);
    if(theta_m < LIGHT_Info->Qmin(LKN)) theta_m = LIGHT_Info->Qmin(LKN);
    else if (theta_m > LIGHT_Info->Qmax(LKN)) theta_m = LIGHT_Info->Qmax(LKN);

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

double  LIGHTWholeBody::Angle2CylinderPos_Knee(double theta){
    // input : relative knee angle (0~fullangle) [rad]
    // output : relative cylinder length (0~stroke) [m]

    double l0 = LIGHT_Info->KneeLink.l0;
    double beta = LIGHT_Info->KneeLink.beta;

    double d1 = LIGHT_Info->KneeLink.d1;
    double d2 = LIGHT_Info->KneeLink.d2;
    double d3 = LIGHT_Info->KneeLink.d3;
    double d4 = LIGHT_Info->KneeLink.d4;
    double delta = LIGHT_Info->KneeLink.delta;

    double temp = -theta; // theta is negative direction
    if(temp < -1.0*D2R) temp = -1.0*D2R;
    else if (temp > 120.0*D2R) temp = 120.0*D2R;

    double th2 = temp+delta;
    double l2 = sqrt(d1*d1+d3*d3-2.0*d1*d3*cos(th2));
    double th13 = acos((d1*d1+l2*l2-d3*d3)/(2.0*d1*l2));
    double th14 = acos((d2*d2+l2*l2-d4*d4)/(2.0*d2*l2));
    double th1 = th13+th14;

    // cylinder position
    double x = sqrt(l0*l0+d2*d2-2*l0*d2*cos(PI-th1+beta));
    double x_off = LIGHT_Info->KneeLink.x_off;

    return (x-x_off);
}

double  LIGHTWholeBody::AngularVel2CylinderVel_Knee(double theta, double dtheta){
    double l0 = LIGHT_Info->KneeLink.l0;
    double beta = LIGHT_Info->KneeLink.beta;

    double d1 = LIGHT_Info->KneeLink.d1;
    double d2 = LIGHT_Info->KneeLink.d2;
    double d3 = LIGHT_Info->KneeLink.d3;
    double d4 = LIGHT_Info->KneeLink.d4;
    double delta = LIGHT_Info->KneeLink.delta;

    double temp = -theta; // theta is negative direction
    if(temp < -1.0*D2R) temp = -1.0*D2R;
    else if (temp > 120.0*D2R) temp = 120.0*D2R;

    double th2 = temp+delta;
    double l2 = sqrt(d1*d1+d3*d3-2.0*d1*d3*cos(th2));
    double th13 = acos((d1*d1+l2*l2-d3*d3)/(2.0*d1*l2));
    double th14 = acos((d2*d2+l2*l2-d4*d4)/(2.0*d2*l2));
    double th1 = th13+th14;
    double x = sqrt(l0*l0+d2*d2-2*l0*d2*cos(PI-th1+beta));

    double dth2 = -dtheta;
    double dl2 = (d1*d3*sin(th2)/l2)*dth2;
    double dth1 = ((d2*cos(th14)-l2)/(d2*sin(th14)*l2)+(d1*cos(th13)-l2)/(d1*sin(th13)*l2))*dl2;

    // cylinder velocity
    double dx = -l0*d2*sin(PI-th1+beta)/x*dth1;

    return dx;
}

double  LIGHTWholeBody::Torque2CylinderForce_Knee(double theta, double T){
    double l0 = LIGHT_Info->KneeLink.l0;
    double beta = LIGHT_Info->KneeLink.beta;

    double d1 = LIGHT_Info->KneeLink.d1;
    double d2 = LIGHT_Info->KneeLink.d2;
    double d3 = LIGHT_Info->KneeLink.d3;
    double d4 = LIGHT_Info->KneeLink.d4;
    double delta = LIGHT_Info->KneeLink.delta;

    double temp = -theta; // theta is negative direction
    if(temp < -1.0*D2R) temp = -1.0*D2R;
    else if (temp > 120.0*D2R) temp = 120.0*D2R;

    double th2 = temp+delta;
    double l2 = sqrt(d1*d1+d3*d3-2.0*d1*d3*cos(th2));
    double th13 = acos((d1*d1+l2*l2-d3*d3)/(2.0*d1*l2));
    double th14 = acos((d2*d2+l2*l2-d4*d4)/(2.0*d2*l2));
    double th1 = th13+th14;
    double x = sqrt(l0*l0+d2*d2-2*l0*d2*cos(PI-th1+beta));

    double T2 = -T;
    double F2 = (l2/(d1*d3*sin(th2)))*T2;
    double T1 = 1.0/((d2*cos(th14)-l2)/(d2*sin(th14)*l2)+(d1*cos(th13)-l2)/(d1*sin(th13)*l2))*F2;

    // cylinder force
    double F = -x/(l0*d2*sin(PI-th1+beta))*T1;

    return F;
}

// 5. Ankle Pitch and Roll ////////////////////////////////////////////////////////////
void LIGHTWholeBody::Joint2Actuator_Ankle(double theta_m, double theta, double dtheta, double Tp,
                                         double phi_m, double phi, double dphi, double Tr,
                                         VectorNd &S, VectorNd &dS, VectorNd &F)
{
    if(theta < LIGHT_Info->Qmin(LAP)) theta = LIGHT_Info->Qmin(LAP);
    else if (theta > LIGHT_Info->Qmax(LAP)) theta = LIGHT_Info->Qmax(LAP);
    if(phi < LIGHT_Info->Qmin(LAR)) phi = LIGHT_Info->Qmin(LAR);
    else if (phi > LIGHT_Info->Qmax(LAR)) phi = LIGHT_Info->Qmax(LAR);

    if(theta_m < LIGHT_Info->Qmin(LAP)) theta_m = LIGHT_Info->Qmin(LAP);
    else if (theta_m > LIGHT_Info->Qmax(LAP)) theta_m = LIGHT_Info->Qmax(LAP);
    if(phi_m < LIGHT_Info->Qmin(LAR)) phi_m = LIGHT_Info->Qmin(LAR);
    else if (phi_m > LIGHT_Info->Qmax(LAR)) phi_m = LIGHT_Info->Qmax(LAR);

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
void LIGHTWholeBody::Joint2Actuator_WaistYaw(double theta, double dtheta, double T,
                                           double &S, double &dS, double &F)
{
    S = theta;
    dS = dtheta;
    F = T;
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

VectorNd  LIGHTWholeBody::Angle2CylinderPos_Ankle(double theta, double phi)
{
    // [Input]
    // theta : pitch angle [rad]
    // phi : roll angle [rad]

    // [Output]
    // x(0) : right cylinder length [m]
    // x(1) : left cylinder length [m]

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
    double l = LIGHT_Info->AnkleLink.l;
    double theta_off = LIGHT_Info->AnkleLink.theta_off;
    double x1_off = LIGHT_Info->AnkleLink.x_off_R;
    double x2_off = LIGHT_Info->AnkleLink.x_off_L;

    theta = theta - theta_off;
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

    VectorNd x(2);
    x << (x1-x1_off),(x2-x2_off);
    return x;
}

VectorNd  LIGHTWholeBody::AngularVel2CylinderVel_Ankle(double theta, double phi, double dtheta, double dphi)
{
    // [Input]
    // theta : pitch angle [rad]
    // phi : roll angle [rad]
    // dtheta : pitch angular velocity [rad/s]
    // dphi : roll angular velocity [rad/s]

    // [Output]
    // x(0) : right cylinder velocity [m/s]
    // x(1) : left cylinder velocity [m/s]

    Vector3d OP = LIGHT_Info->AnkleLink.OP;
    Vector3d PQ1 = LIGHT_Info->AnkleLink.PQ_R; // right : 1
    Vector3d PQ2 = LIGHT_Info->AnkleLink.PQ_L; // left : 2
    Vector3d OC1 = LIGHT_Info->AnkleLink.OC_R;
    Vector3d OC2 = LIGHT_Info->AnkleLink.OC_L;
    double alpha = LIGHT_Info->AnkleLink.alpha; // inclined cylinder angle
    double l = LIGHT_Info->AnkleLink.l;
    double theta_off = LIGHT_Info->AnkleLink.theta_off;

    theta = theta - theta_off;
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

    Matrix3d dR_p = Matrix3d(-sin(theta),     0.0, cos(theta),
                                     0.0,     0.0,        0.0,
                             -cos(theta),     0.0,-sin(theta));
    Matrix3d dR_r = Matrix3d(        0.0,       0.0,       0.0,
                                     0.0, -sin(phi), -cos(phi),
                                     0.0,  cos(phi), -sin(phi));

    Vector3d dC1Q1_1 = dR_p*(R_r*PQ1+OP);
    Vector3d dC1Q1_2 = R_p*(dR_r*PQ1);
    MatrixNd J_dC1Q1(3,2);
    J_dC1Q1.block(0,0,3,1) = dC1Q1_1;
    J_dC1Q1.block(0,1,3,1) = dC1Q1_2;
    MatrixNd J_x1 = 1/(x1-p.dot(C1Q1))*(x1*p.transpose()-C1Q1.transpose())*J_dC1Q1;

    Vector3d dC2Q2_1 = dR_p*(R_r*PQ2+OP);
    Vector3d dC2Q2_2 = R_p*(dR_r*PQ2);
    MatrixNd J_dC2Q2(3,2);
    J_dC2Q2.block(0,0,3,1) = dC2Q2_1;
    J_dC2Q2.block(0,1,3,1) = dC2Q2_2;
    MatrixNd J_x2 = 1/(x2-p.dot(C2Q2))*(x2*p.transpose()-C2Q2.transpose())*J_dC2Q2;

    // Jacobian for joint angular velocity >> cylinder velocity (v = J*thetadot)
    MatrixNd J(2,2);
    J.block(0,0,1,2) = J_x1; // for pitch joint velocity
    J.block(1,0,1,2) = J_x2; // for roll joint velocity

    VectorNd w(2);
    w << dtheta,dphi;
    VectorNd dx = J*w;

    return dx;
}

VectorNd  LIGHTWholeBody::Torque2CylinderForce_Ankle(double theta, double phi, double T_p, double T_r)
{
    // [Input]
    // theta : pitch angle [rad]
    // phi : roll angle [rad]
    // T_p : pitch torque [Nm]
    // dphi : roll torque [Nm]

    // [Output]
    // x(0) : right cylinder force [N]
    // x(1) : left cylinder force [N]

    Vector3d OP = LIGHT_Info->AnkleLink.OP;
    Vector3d PQ1 = LIGHT_Info->AnkleLink.PQ_R; // right : 1
    Vector3d PQ2 = LIGHT_Info->AnkleLink.PQ_L; // left : 2
    Vector3d OC1 = LIGHT_Info->AnkleLink.OC_R;
    Vector3d OC2 = LIGHT_Info->AnkleLink.OC_L;
    double alpha = LIGHT_Info->AnkleLink.alpha; // inclined cylinder angle
    double l = LIGHT_Info->AnkleLink.l;
    double theta_off = LIGHT_Info->AnkleLink.theta_off;

    theta = theta - theta_off;
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

    Matrix3d dR_p = Matrix3d(-sin(theta),     0.0, cos(theta),
                                     0.0,     0.0,        0.0,
                             -cos(theta),     0.0,-sin(theta));
    Matrix3d dR_r = Matrix3d(        0.0,       0.0,       0.0,
                                     0.0, -sin(phi), -cos(phi),
                                     0.0,  cos(phi), -sin(phi));

    Vector3d dC1Q1_1 = dR_p*(R_r*PQ1+OP);
    Vector3d dC1Q1_2 = R_p*(dR_r*PQ1);
    MatrixNd J_dC1Q1(3,2);
    J_dC1Q1.block(0,0,3,1) = dC1Q1_1;
    J_dC1Q1.block(0,1,3,1) = dC1Q1_2;
    MatrixNd J_x1 = 1/(x1-p.dot(C1Q1))*(x1*p.transpose()-C1Q1.transpose())*J_dC1Q1;

    Vector3d dC2Q2_1 = dR_p*(R_r*PQ2+OP);
    Vector3d dC2Q2_2 = R_p*(dR_r*PQ2);
    MatrixNd J_dC2Q2(3,2);
    J_dC2Q2.block(0,0,3,1) = dC2Q2_1;
    J_dC2Q2.block(0,1,3,1) = dC2Q2_2;
    MatrixNd J_x2 = 1/(x2-p.dot(C2Q2))*(x2*p.transpose()-C2Q2.transpose())*J_dC2Q2;

    // Jacobian for joint angular velocity >> cylinder velocity (v = J*thetadot)
    MatrixNd J(2,2);
    J.block(0,0,1,2) = J_x1; // for pitch joint velocity
    J.block(1,0,1,2) = J_x2; // for roll joint velocity

    VectorNd T(2);
    T << T_p,T_r;
    VectorNd F = J.transpose().inverse()*T;

    return F;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  LIGHTWholeBody::CalcTorqueLimit_Hip(int RL, double& Tmax, double& Tmin)
{
    double l0 = LIGHT_Info->HipLink.l0;
    double l1 = LIGHT_Info->HipLink.l1;
    double alpha = LIGHT_Info->HipLink.alpha;

    double theta;
    if(RL==0) {
        theta = Qnow(ACT_QNUMSTART+RHP);
    } else {
        theta = Qnow(ACT_QNUMSTART+LHP);
    }

    double th1 = theta;
    if(th1 < -1.0*D2R) th1 = -1.0*D2R;
    else if (th1 > 100.0*D2R) th1 = 100.0*D2R;

    double x = sqrt(l0*l0+l1*l1-2*l0*l1*cos(alpha-th1));

    Tmin = -LIGHT_Info->HipLink.Fmax*(l0*l1*sin(alpha-th1))/x;
    Tmax = -LIGHT_Info->HipLink.Fmin*(l0*l1*sin(alpha-th1))/x;
}

void  LIGHTWholeBody::CalcTorqueLimit_Knee(int RL, double& Tmax, double& Tmin)
{
    double l0 = LIGHT_Info->KneeLink.l0;
    double beta = LIGHT_Info->KneeLink.beta;

    double d1 = LIGHT_Info->KneeLink.d1;
    double d2 = LIGHT_Info->KneeLink.d2;
    double d3 = LIGHT_Info->KneeLink.d3;
    double d4 = LIGHT_Info->KneeLink.d4;
    double delta = LIGHT_Info->KneeLink.delta;

    double theta;
    if(RL==0) {
        theta = Qnow(ACT_QNUMSTART+RKN);
    } else {
        theta = Qnow(ACT_QNUMSTART+LKN);
    }

    double temp = -theta; // theta is negative direction
    if(temp < -1.0*D2R) temp = -1.0*D2R;
    else if (temp > 120.0*D2R) temp = 120.0*D2R;

    double th2 = temp+delta;
    double l2 = sqrt(d1*d1+d3*d3-2.0*d1*d3*cos(th2));
    double th13 = acos((d1*d1+l2*l2-d3*d3)/(2.0*d1*l2));
    double th14 = acos((d2*d2+l2*l2-d4*d4)/(2.0*d2*l2));
    double th1 = th13+th14;
    double x = sqrt(l0*l0+d2*d2-2*l0*d2*cos(PI-th1+beta));

    double T1,F2,T2;

    T1 = -LIGHT_Info->KneeLink.Fmin/x*(l0*d2*sin(PI-th1+beta));
    F2 = T1*((d2*cos(th14)-l2)/(d2*sin(th14)*l2)+(d1*cos(th13)-l2)/(d1*sin(th13)*l2));
    T2 = F2/(l2/(d1*d3*sin(th2)));
    Tmax = -T2;

    T1 = -LIGHT_Info->KneeLink.Fmax/x*(l0*d2*sin(PI-th1+beta));
    F2 = T1*((d2*cos(th14)-l2)/(d2*sin(th14)*l2)+(d1*cos(th13)-l2)/(d1*sin(th13)*l2));
    T2 = F2/(l2/(d1*d3*sin(th2)));
    Tmin = -T2;
}

void  LIGHTWholeBody::CalcTorqueLimit_Ankle(int RL, double& Tpitchmax, double& Tpitchmin, double& Trollmax, double& Trollmin)
{
    // [Input]
    // theta : pitch angle [rad]
    // phi : roll angle [rad]
    // T_p : pitch torque [Nm]
    // dphi : roll torque [Nm]

    // [Output]
    // x(0) : right cylinder force [N]
    // x(1) : left cylinder force [N]

    Vector3d OP = LIGHT_Info->AnkleLink.OP;
    Vector3d PQ1 = LIGHT_Info->AnkleLink.PQ_R; // right : 1
    Vector3d PQ2 = LIGHT_Info->AnkleLink.PQ_L; // left : 2
    Vector3d OC1 = LIGHT_Info->AnkleLink.OC_R;
    Vector3d OC2 = LIGHT_Info->AnkleLink.OC_L;
    double alpha = LIGHT_Info->AnkleLink.alpha; // inclined cylinder angle
    double l = LIGHT_Info->AnkleLink.l;
    double theta_off = LIGHT_Info->AnkleLink.theta_off;

    double theta,phi;
    if(RL==0) {
        theta = Qnow(ACT_QNUMSTART+RAP);
        phi = Qnow(ACT_QNUMSTART+RAR);
    } else {
        theta = Qnow(ACT_QNUMSTART+LAP);
        phi = Qnow(ACT_QNUMSTART+LAR);
    }

    theta = theta - theta_off;
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

    Matrix3d dR_p = Matrix3d(-sin(theta),     0.0, cos(theta),
                                     0.0,     0.0,        0.0,
                             -cos(theta),     0.0,-sin(theta));
    Matrix3d dR_r = Matrix3d(        0.0,       0.0,       0.0,
                                     0.0, -sin(phi), -cos(phi),
                                     0.0,  cos(phi), -sin(phi));

    Vector3d dC1Q1_1 = dR_p*(R_r*PQ1+OP);
    Vector3d dC1Q1_2 = R_p*(dR_r*PQ1);
    MatrixNd J_dC1Q1(3,2);
    J_dC1Q1.block(0,0,3,1) = dC1Q1_1;
    J_dC1Q1.block(0,1,3,1) = dC1Q1_2;
    MatrixNd J_x1 = 1/(x1-p.dot(C1Q1))*(x1*p.transpose()-C1Q1.transpose())*J_dC1Q1;

    Vector3d dC2Q2_1 = dR_p*(R_r*PQ2+OP);
    Vector3d dC2Q2_2 = R_p*(dR_r*PQ2);
    MatrixNd J_dC2Q2(3,2);
    J_dC2Q2.block(0,0,3,1) = dC2Q2_1;
    J_dC2Q2.block(0,1,3,1) = dC2Q2_2;
    MatrixNd J_x2 = 1/(x2-p.dot(C2Q2))*(x2*p.transpose()-C2Q2.transpose())*J_dC2Q2;

    // Jacobian for joint angular velocity >> cylinder velocity (v = J*thetadot)
    MatrixNd J(2,2);
    J.block(0,0,1,2) = J_x1; // for pitch joint velocity
    J.block(1,0,1,2) = J_x2; // for roll joint velocity

    VectorNd F(2), T(2);

    F << LIGHT_Info->AnkleLink.Fmax,LIGHT_Info->AnkleLink.Fmax;
    T = J.transpose()*F;
    Tpitchmax = T(0);

    F << LIGHT_Info->AnkleLink.Fmin,LIGHT_Info->AnkleLink.Fmin;
    T = J.transpose()*F;
    Tpitchmin = T(0);

    F << LIGHT_Info->AnkleLink.Fmax,LIGHT_Info->AnkleLink.Fmin;
    T = J.transpose()*F;
    Trollmax = T(1);

    F << LIGHT_Info->AnkleLink.Fmin,LIGHT_Info->AnkleLink.Fmax;
    T = J.transpose()*F;
    Trollmin = T(1);

//    FILE_LOG(logDEBUG) << "T_roll_max: " << Trollmax;
//    FILE_LOG(logDEBUG) << "T_roll_min: " << Trollmin;
}


