#ifndef LIGHT_INFO
#define LIGHT_INFO

#include "rbdl/rbdl.h"
#include "LIGHT_var_and_func.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

inline Matrix3d RotX (const double &xrot) {
  double s, c;
  s = sin (xrot);
  c = cos (xrot);
  return Matrix3d (
      1., 0., 0.,
      0., c,  -s,
      0., s,  c
      );
}

inline Matrix3d RotY (const double &yrot) {
  double s, c;
  s = sin (yrot);
  c = cos (yrot);
  return Matrix3d (
      c,  0., s,
      0., 1., 0.,
      -s,  0., c
      );
}

inline Matrix3d RotZ (const double &zrot) {
  double s, c;
  s = sin (zrot);
  c = cos (zrot);
  return Matrix3d (
      c,  -s, 0.,
      s,  c, 0.,
      0., 0., 1.
      );
}

inline Matrix3d dRotX (const double &xrot, const double &dxrot) {
  double s, c;
  s = sin (xrot);
  c = cos (xrot);
  return Matrix3d (
      0., 0., 0.,
      0., -s*dxrot,  -c*dxrot,
      0., c*dxrot,  -s*dxrot
      );
}

inline Matrix3d dRotY (const double &yrot, const double &dyrot) {
  double s, c;
  s = sin (yrot);
  c = cos (yrot);
  return Matrix3d (
      -s*dyrot,  0., c*dyrot,
      0., 0., 0.,
      -c*dyrot,  0., -s*dyrot
      );
}

inline Matrix3d dRotZ (const double &zrot, const double &dzrot) {
  double s, c;
  s = sin (zrot);
  c = cos (zrot);
  return Matrix3d (
      -s*dzrot,  -c*dzrot, 0.,
      c*dzrot,  -s*dzrot, 0.,
      0., 0., 0.
      );
}

// LIGHT's Joint Number (or DoF) ///////////////////////////////////////////////////
enum LIGHT_DOFINFO
{
    // Floating Base : 6DoF
    // Leg : 6DoF
    // Waist : 1DoF
    FLOATING_BASE_DOF = 6,
    LEG_DOF = 6,
    WST_DOF = 1,
    ARM_DOF = 0,

    LIGHT_RL_DOF = LEG_DOF,
    LIGHT_LL_DOF = LEG_DOF,
    LIGHT_RA_DOF = ARM_DOF,
    LIGHT_LA_DOF = ARM_DOF,
    LIGHT_WST_DOF = WST_DOF,

    LIGHT_DOF = FLOATING_BASE_DOF+LIGHT_RL_DOF+LIGHT_LL_DOF+LIGHT_WST_DOF+LIGHT_RA_DOF+LIGHT_LA_DOF,
    LIGHT_ACT_DOF = LIGHT_RL_DOF+LIGHT_LL_DOF+LIGHT_WST_DOF+LIGHT_RA_DOF+LIGHT_LA_DOF,
    LIGHT_NUM_CON = 3 // two point feet and x,y,z axis
};

// LIGHT's Joint Number Index ///////////////////////////////////////////////////////
enum QNUM_START // Q vector index starting point
{
    PELVIS_POS_QNUMSTART = 0,
    PELVIS_ORI_QNUMSTART = 3,
    ACT_QNUMSTART = FLOATING_BASE_DOF,
    RL_QNUMSTART = FLOATING_BASE_DOF,
    LL_QNUMSTART = FLOATING_BASE_DOF+LIGHT_RL_DOF,
    WST_QNUMSTART = FLOATING_BASE_DOF+LIGHT_RL_DOF+LIGHT_LL_DOF,
    QNUM_END = FLOATING_BASE_DOF+LIGHT_RL_DOF+LIGHT_LL_DOF+LIGHT_WST_DOF
};

enum JNUM_START // Joint number index starting point
{
    RL_JNUMSTART = 0,
    LL_JNUMSTART = LIGHT_RL_DOF,
    WST_JNUMSTART = LIGHT_RL_DOF+LIGHT_LL_DOF,
};

// LIGHT's Control Module ID ///////////////////////////////////////////////////////

enum LIGHTMODULE_ID{
    RIGHTLEG = -1,
    LEFTLEG = 1,
    WAIST = 0
};

// LIGHT's actuator level info. ///////////////////////////////////////////////////////
struct LightHipLink {
    double l0,l1;
    double alpha,beta;
    double x_off;

    double Fmin,Fmax;

    // Updated to LIGHT_ver2.0 (210430)
    LightHipLink(){
        l0 = 170.0*MM2M;
        l1 = 50.0*MM2M;
        alpha = 139.8520*D2R; // in case of fully closed position

        x_off = 210.70*MM2M;
        // x_hip : 125.90(116.01deg, fully Stretched) ~ 210.70(0deg, fully bended)
        // stroke : 84.80mm

        Fmin = -210.0*(25.0*25.0-18.0*18.0)*3.141592/4.0*1e-1;
        Fmax = 210.0*(25.0*25.0-18.0*18.0)*3.141592/4.0*1e-1;
    }
};

struct LightKneeLink {
    double l0;
    double beta;
    double d1,d2,d3,d4;
    double delta;
    double x_off;

    double Fmin,Fmax;

    // Updated to LIGHT_ver2.0 (210430)
    LightKneeLink(){
        l0 = 194.20*MM2M;
        beta = 16.48*D2R;

        d1 = 60.0*MM2M;
        d2 = 44.0*MM2M;
        d3 = 40.0*MM2M;
        d4 = 70.0*MM2M;

        delta = (31.06)*D2R;// actually, delta : theta2_ini

        x_off = 156.50*MM2M;
        // x_knee : 156.50(0deg, fully bended) ~ 231.73(122.02deg, fully Stretched)
        // stroke : 75.23mm

        Fmin = -210.0*(25.0*25.0-18.0*18.0)*3.141592/4.0*1e-1;
        Fmax = 210.0*(25.0*25.0)*3.141592/4.0*1e-1;
    }
};

struct LightAnkleLink {

    // Point O : Ankle Joint Center
    // Point P : Joint Link(Universal Joint) Center
    // Point Q1 : Right Link(connected to cylinder) Rodend Center
    // Point Q2 : Left Link(connected to cylinder) Rodend Center
    // Point C1 : Right Cylinder base
    // Point C2 : Left Cylinder base
    Vector3d OP; // ankle origin to ankle link center  (Reference Frame : Ankle link)
    Vector3d PQ_R; // ankle link center to right rodend joint (Reference Frame : Ankle link)
    Vector3d PQ_L; // ankle link center to left rodend joint (Reference Frame : Ankle link)
    Vector3d OC_R;  // ankle origin to right cylinder base (Reference Frame : kalf link)
    Vector3d OC_L;  // ankle origin to left cylinder base (Reference Frame : kalf link)

    double l; // length from cylinder balljoint and ankle rodend
    double alpha; // inclined cylinder angle

    double theta_off;
    double x_off_R;
    double x_off_L;

    double Fmin,Fmax;

    LightAnkleLink(){

        OP = Vector3d(0.0, 0.0, -28.00)*MM2M;
        PQ_R = Vector3d(45.25, -29.0, -3.00)*MM2M;
        PQ_L = Vector3d(45.25, 29.0, -3.00)*MM2M;
        OC_R = Vector3d(36.33, -29.0, 291.48)*MM2M;
        OC_L = Vector3d(36.33, 29.0, 291.48)*MM2M;
        alpha = 1.51*D2R;
        l = 91.20*MM2M;

        theta_off = (90.00-19.37)*D2R;
        x_off_R = 168.0*MM2M;
        x_off_L = 168.0*MM2M;

        Fmin = -210.0*(15.0*15.0-10.0*10.0)*3.141592/4.0*1e-1;
        Fmax = 210.0*(15.0*15.0)*3.141592/4.0*1e-1;
    }
};

// LIGHT's Basic information (mass parameter, link length, etc...) /////////////////

class INFO_LIGHT
{
public :
    //mass information
    double m_robot;
    double m_pel;
    double m_torso;
    double m_rhr,m_rhy,m_rhp,m_rkn,m_rap,m_rar;
    double m_lhr,m_lhy,m_lhp,m_lkn,m_lap,m_lar;

    // Rotation inertia info.
    Matrix3d I_base;  // base : pelvis+torso
    Matrix3d I_torso, I_pel;

    Matrix3d I_rleg,I_lleg;
    Matrix3d I_rhr, I_rhy, I_rhp, I_rkn, I_rap, I_rar;
    Matrix3d I_lhr, I_lhy, I_lhp, I_lkn, I_lap, I_lar;

    //Center of Mass information
    Vector3d c_base;
    Vector3d c_torso, c_pel;

    Vector3d c_rleg,c_lleg;
    Vector3d c_rhr, c_rhy, c_rhp, c_rkn, c_rap, c_rar;
    Vector3d c_lhr, c_lhy, c_lhp, c_lkn, c_lap, c_lar;

    // Coordinate offset (Link length)
    Vector3d offset_pel2torso;
    Vector3d offset_pel2rhr, offset_rhr2rhy, offset_rhy2rhp, offset_rhp2rkn, offset_rkn2rap, offset_rap2rar, offset_rar2EE;
    Vector3d offset_pel2lhr, offset_lhr2lhy, offset_lhy2lhp, offset_lhp2lkn, offset_lkn2lap, offset_lap2lar, offset_lar2EE;
    Matrix3d offset_R_rar2EE, offset_R_lar2EE;

    // Foot size
    double footsize_inside, footsize_outside, footsize_toe, footsize_heel;
    double footlimit_inside, footlimit_outside, footlimit_toe, footlimit_heel;
    double ZMPlimit_inside, ZMPlimit_outside, ZMPlimit_toe, ZMPlimit_heel;

    // Step Constraints
    double XStep_Saturation;
    double YStepOutside_Saturation, YStepInside_Saturation;
    double YawStepInside_Saturation;

    // Linkage Info
    LightHipLink HipLink;
    LightKneeLink KneeLink;
    LightAnkleLink AnkleLink;

    // Actuator Info
    VectorNd Sa, Sb;

    // Joint Limits (Angle, Velocity, Acceleration)
    VectorNd Qmax, Qmin;
    VectorNd dQmax, dQmin;
    VectorNd ddQmax, ddQmin;

    //Initialize
    INFO_LIGHT()  //set initial value.
    {

        ////// Mass Parameters ///////////////////////////

//        m_torso = 32.0; // (with Full-HPU and Battery)
//        m_torso = 35.0; // (with HPU)
        m_torso = 34.0; // 30.0; (without HPU)
        m_pel = 5.498; // 4.498;

//        //mass information / Unit : [kg]
        m_rhr = 1.214; // 0.914;
        m_rhy = 0.661; // 0.661;
        m_rhp = 6.250; // 7.750;
        m_rkn = 3.247; // 3.247;
        m_rap = 0.237; // 0.137;
        m_rar = 0.888; // 0.788;

        m_lhr = m_rhr;
        m_lhy = m_rhy;
        m_lhp = m_rhp;
        m_lkn = m_rkn;
        m_lap = m_rap;
        m_lar = m_rar;

        m_robot = m_torso + m_pel
                + m_rhr+m_rhy+m_rhp+m_rkn+m_rap+m_rar
                + m_lhr+m_lhy+m_lhp+m_lkn+m_lap+m_lar;

        //Inertia information
        //inertia with respect to CoM of link (absolute frame) / Unit : [kg*m^2]
        I_torso = Matrix3d(810.9, 0.0, 40.0, 0.0, 628.0, -13.0, 40.0, -13.0, 742.1)*0.001*1.0;
        I_pel = Matrix3d(19.46, 0.0, -2.155, 0.0, 40.90, 0.0, -2.155, 0.0, 38.76)*0.001*1.0;

//        I_rhr = Matrix3d(0.463, 0.0, 0.0, 0.0, 1.677, 0.0, 0.0, 0.0, 1.665)*0.001*1.0;
//        I_rhy = Matrix3d(0.789, 0.0, 0.0, 0.0, 0.496, 0.0, 0.0, 0.0, 0.433)*0.001*1.0;
//        I_rhp = Matrix3d(30.69, 0.021, 14.13, 0.021, 111.743, 0.136, 14.13, 0.136, 94.59)*0.001*1.0;
//        I_rkn = Matrix3d(34.27, 0.266, 13.06, 0.266, 40.03, 0.534, 13.06, 0.534, 11.38)*0.001*1.0;
//        I_rap = Matrix3d(0.035, 0.0, 0.0, 0.0, 0.0535, 0.0, 0.0, 0.0, 0.0437)*0.001*1.0;
//        I_rar = Matrix3d(1.560, 0.012, 0.639, 0.012, 1.750, 0.0075, 0.639, 0.0075, 1.3775.0)*0.001*30.0;

//        I_lhr = Matrix3d(0.463, 0.0, 0.0, 0.0, 1.677, 0.0, 0.0, 0.0, 1.665)*0.001*1.0;
//        I_lhy = Matrix3d(0.789, 0.0, 0.0, 0.0, 0.496, 0.0, 0.0, 0.0, 0.433)*0.001*1.0;
//        I_lhp = Matrix3d(30.69, -0.021, 14.13, -0.021, 111.743, -0.136, 14.13, -0.136, 94.59)*0.001*1.0;
//        I_lkn = Matrix3d(34.27, -0.266, 13.06, -0.266, 40.03, -0.534, 13.06, -0.534, 11.38)*0.001*1.0;
//        I_lap = Matrix3d(0.035, 0.0, 0.0, 0.0, -0.0535, 0.0, 0.0, 0.0, 0.0437)*0.001*1.0;
//        I_lar = Matrix3d(1.560, -0.012, 0.639, -0.012, 1.750, -0.0075, 0.639, -0.0075, 1.377)*0.001*30.0;

        I_rhr = Matrix3d(0.463, 0.0, 0.0, 0.0, 1.677, 0.0, 0.0, 0.0, 1.665)*0.001*1.0;
        I_rhy = Matrix3d(0.789, 0.0, 0.0, 0.0, 0.496, 0.0, 0.0, 0.0, 0.433)*0.001*1.0;
        I_rhp = Matrix3d(30.69, 0.0, 14.13, 0.0, 111.743, 0.0, 14.13, 0.0, 94.59)*0.001*1.0;
        I_rkn = Matrix3d(34.27, 0.0, 13.06, 0.0, 40.03, 0.0, 13.06, 0.0, 11.38)*0.001*1.0;
        I_rap = Matrix3d(0.035, 0.0, 0.0, 0.0, 0.0535, 0.0, 0.0, 0.0, 0.0437)*0.001*1.0;
        I_rar = Matrix3d(1.560, 0.0, 0.639, 0.0, 1.750, 0.0, 0.639, 0.0, 1.377)*0.001*10.0;

        I_lhr = Matrix3d(0.463, 0.0, 0.0, 0.0, 1.677, 0.0, 0.0, 0.0, 1.665)*0.001*1.0;
        I_lhy = Matrix3d(0.789, 0.0, 0.0, 0.0, 0.496, 0.0, 0.0, 0.0, 0.433)*0.001*1.0;
        I_lhp = Matrix3d(30.69, 0.0, 14.13, 0.0, 111.743, 0.0, 14.13, 0.0, 94.59)*0.001*0.0;
        I_lkn = Matrix3d(34.27, 0.0, 13.06, 0.0, 40.03, 0.0, 13.06, 0.0, 11.38)*0.001*1.0;
        I_lap = Matrix3d(0.035, 0.0, 0.0, 0.0, 0.0535, 0.0, 0.0, 0.0, 0.0437)*0.001*1.0;
        I_lar = Matrix3d(1.560, 0.0, 0.639, 0.0, 1.750, 0.0, 0.639, 0.0, 1.377)*0.001*10.0;


        //center of mass_lb (absolute frame) / Unit : [m]
//        c_torso = Vector3d(0.0, 0.0, 330.00) * MM2M; // (with Full-HPU, Battery, and Arm)
//        c_torso = Vector3d(-60.0, 5.0, 320.00) * MM2M; // (with Full-HPU and Battery)
//        c_torso = Vector3d(-60.0, 0.0, 320.00) * MM2M; // (with HPU)
        c_torso = Vector3d(-55.0, 0.0, 330.00) * MM2M; // (with Arm and Battery)
//        c_torso = Vector3d(-20.0, 0.0, 340.00) * MM2M; // (without HPU)
        c_pel = Vector3d(-11.65, 0.0, 21.58) * MM2M;

        c_rhr = Vector3d(16.79, 0.0, 0.0) * MM2M;
        c_rhy = Vector3d(0.0, -15.16, 0.0) * MM2M;
        c_rhp = Vector3d(170.96, -0.0, -14.87) * MM2M; // y-direction tuning
        c_rkn = Vector3d(-34.73, -0.0, -177.44) * MM2M;
        c_rap = Vector3d(8.12, 0.0, -10.09) * MM2M;
        c_rar = Vector3d(21.57, 0.0, -5.78) * MM2M;

        c_lhr = Vector3d(16.79, 0.0, 0.0) * MM2M;
        c_lhy = Vector3d(0.0, 15.16, 0.0) * MM2M;
        c_lhp = Vector3d(170.96, 0.0, -14.87) * MM2M; // y-direction tuning
        c_lkn = Vector3d(-34.73, 0.0, -177.44) * MM2M;
        c_lap = Vector3d(8.12, 0.0, -10.09) * MM2M;
        c_lar = Vector3d(21.57, 0.0, -5.78) * MM2M;

        ////// Kinematic Parameters ///////////////////////////

        // link offset (absolute frame) >> Updated to LIGHT_ver2.0 (210430)
        offset_pel2torso = Vector3d(0.0, 0.0, 78.0) * MM2M;

        offset_pel2rhr = Vector3d(-2.5, -60.0, 0.0) * MM2M;
        offset_rhr2rhy = Vector3d(0.0, 0.0, 0.0) * MM2M;
        offset_rhy2rhp = Vector3d(0.0, -122.0, 0.0) * MM2M;
        offset_rhp2rkn = Vector3d(346.72, 0.0, 47.82) * MM2M;
        offset_rkn2rap = Vector3d(-136.62, 0.0, -354.59) * MM2M;
        offset_rap2rar = Vector3d(21.31, 0.0, -18.16) * MM2M;
        offset_rar2EE = Vector3d(37.98, 0.0, -24.82) * MM2M; // Projection of the ankle joint to foot
        offset_R_rar2EE = RotY(-49.56*D2R);

        offset_pel2lhr = Vector3d(-2.5, 60.0, 0.0) * MM2M;
        offset_lhr2lhy = Vector3d(0.0, 0.0, 0.0) * MM2M;
        offset_lhy2lhp = Vector3d(0.0, 122.0, 0.0) * MM2M;
        offset_lhp2lkn = Vector3d(346.72, 0.0, 47.82) * MM2M;
        offset_lkn2lap = Vector3d(-136.62, 0.0, -354.59) * MM2M;
        offset_lap2lar = Vector3d(21.31, 0.0, -18.16) * MM2M;
        offset_lar2EE = Vector3d(37.98, 0.0, -24.82) * MM2M;  // Projection of the ankle joint to foot
        offset_R_lar2EE = RotY(-49.56*D2R);

        footsize_inside = 0.050;
        footsize_outside = 0.075;
        footsize_toe = 0.130;
        footsize_heel = 0.130;
        footlimit_inside = footsize_inside; // Constraint for CP Pattern Generation
        footlimit_outside = footsize_outside;
        footlimit_toe = footsize_toe - 0.01;
        footlimit_heel = footsize_heel - 0.01;
        ZMPlimit_inside = footsize_inside + 0.00;
        ZMPlimit_outside = footsize_outside - 0.01;
        ZMPlimit_toe = footsize_toe - 0.07;
        ZMPlimit_heel = footsize_heel - 0.07;

        XStep_Saturation = 0.42;
        YStepOutside_Saturation = 0.40;
        YStepInside_Saturation = footsize_inside*2.0 + 0.04;
        YawStepInside_Saturation = 8.0*D2R;

        // Actuator Info >> Updated to LIGHT_ver2.0 (210430)
        // Unit : [mm^2*m] Rotary type
        //        [mm^2] Linear type
        Sa = VectorNd::Zero(LIGHT_ACT_DOF);
        Sb = VectorNd::Zero(LIGHT_ACT_DOF);
        Sa << 9595.0*1e-3, 5376.0*1e-3, 236.40, 490.87, 176.71, 176.71,  // Right Leg
              9595.0*1e-3, 5376.0*1e-3, 236.40, 490.87, 176.71, 176.71,  // Left Leg
              10000.0*1e-3; // Waist
        Sb << 9595.0*1e-3, 5376.0*1e-3, 236.40, 236.40, 98.17, 98.17,  // Right Leg
              9595.0*1e-3, 5376.0*1e-3, 236.40, 236.40, 98.17, 98.17,  // Left Leg
              10000.0*1e-3; // Waist

        // Joint Limits (range of motion) >> Updated to LIGHT_ver2.0 (210430)
        // 0~5 : Pelvis coordinate
        // 6~11 : Right leg
        // 12~17 : Left leg
        // 18 : Waist

        Qmax = VectorNd::Zero(LIGHT_DOF+1);
        Qmin = VectorNd::Zero(LIGHT_DOF+1);
        dQmax = VectorNd::Zero(LIGHT_DOF);
        dQmin = VectorNd::Zero(LIGHT_DOF);
        ddQmax = VectorNd::Zero(LIGHT_DOF);
        ddQmin = VectorNd::Zero(LIGHT_DOF);

        Qmax <<  10000.0, 10000.0,  10000.0, 10000.0, 10000.0, 10000.0, // floating base
                25.0, 39.0, 116.01, 1.0, 109.12, 23.0,  // Right Leg
                50.0, 40.0, 116.01, 1.0, 109.12, 23.0,  // Left Leg
                10000.0, // Waist
                10000.0; // base w element

        Qmin << -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, // floating base
                -50.0, -40.0, -1.0, -122.03, -1.0, -23.0,  // Right Leg
                -25.0, -39.0, -1.0, -122.03, -1.0, -23.0,  // Left Leg
                -10000.0, // Waist
                -10000.0; // base w element

        dQmax << 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, // floating base
                10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,  // Right Leg
                10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,  // Left Leg
                10000.0; // Waist

        dQmin << -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, // floating base
                -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0,  // Right Leg
                -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0,  // Left Leg
                -10000.0; // Waist

        ddQmax <<  10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0, // floating base
                10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,    // Right Leg
                10000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,    // Left Leg
                10000.0; // Waist

        ddQmin << -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, // floating base
                -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0,   // Right Leg
                -10000.0, -10000.0, -10000.0, -10000.0, -10000.0, -10000.0,   // Left Leg
                -10000.0; // Waist

        Qmax *= D2R;
        Qmin *= D2R;
        dQmax *= D2R;
        dQmin *= D2R;
        ddQmax *= D2R;
        ddQmin *= D2R;

    }

};


#endif // LIGHT_INFO

