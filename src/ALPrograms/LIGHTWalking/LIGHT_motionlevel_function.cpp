/*============================================================================
 *
 *  <light hopping Motion-level functions>
 *
 *
 *                -  Buyoun,Cho 2018.05.16
 *
=============================================================================*/
#include "LIGHT_motion.h"

extern LIGHTWholeBody LIGHT;

extern LIGHT_InvKinematics_INFO INFO_InvKin;

extern Vector3d OrientationError(Matrix3d Rdes, Matrix3d R);
extern bool PelvisCompensation;

double Pelvis_SquatStartHeight = Pelvis_BaseHeight;

void LIGHTWholeMotions::ClearDesiredMotions()
{
    INFO_InvKin.Reset();
}


extern void SetMotionBase_RF(bool IsDSP);
extern void SetMotionBase_LF(bool IsDSP);


Vector3d _Xini_temp, _Xini_temp2, _Xini_temp3;
Matrix3d _Rini_temp, _Rini_temp2, _Rini_temp3;

///////////////////////////////////////////////////////////////////////////////////////////////////
////////// Submotion Functions ////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

//// Orientation Sub-motion ///////////////////////////////////////////////////////////////////////

void LIGHTWholeMotions::Orientation_Trajectory(double t, double T, Matrix3d Rini, Matrix3d Rdes, Matrix3d &Rnext)
{
    Quaternion First_Des_Ori_quat = Quaternion::fromMatrix(Rini);   // No velocity
    Quaternion Final_Des_Ori_quat = Quaternion::fromMatrix(Rdes);   // No velocity
    Quaternion Next_Des_Ori_q;

    double r = t/T;
    double s;
    if (T <= SYS_DT_WALKING) {
        s = 1.0;
    } else if (t < T) {
        // 5th order polynomial
//        s = 10.0*r*r*r - 15.0*r*r*r*r + 6.0*r*r*r*r*r;
        // 3rd order polynomial
        s = 3.0*r*r - 2.0*r*r*r;
        // 1st order polynomial
//        s = r;
    } else {
        s = 0.0;
    }

    slerp_trajectory(s, First_Des_Ori_quat, Final_Des_Ori_quat, Next_Des_Ori_q);


//    Quaternion2AngularInfo(Next_Des_Ori_q, Next_Des_Ori_dq, Next_Des_Ori_ddq,
//                           Rnext, Wnext, dWnext);
    Quaternion2AngularInfo(Next_Des_Ori_q, Rnext);

}

void LIGHTWholeMotions::Submotion_Global2Pel_Ori(double t, double T, Matrix3d Rdes)
{
    static Matrix3d Rini_Pel;
    if (TimeIsZero()) {
        Rini_Pel = LIGHT.Rref_Pel;
    }

    Matrix3d Rnext;
    Orientation_Trajectory(t, T, Rini_Pel, Rdes, Rnext);

    LIGHT.dWdes_Pel = Vector3d::Zero();
    LIGHT.Wdes_Pel = Vector3d::Zero();
    LIGHT.Rdes_Pel = Rnext;
}

void LIGHTWholeMotions::Submotion_Global2RF_Ori(double t, double T, Matrix3d Rdes)
{
    static Matrix3d Rini_RF;
    if (TimeIsZero()) {
        Rini_RF = LIGHT.Rref_RF;
    }

    Matrix3d Rnext;
    Orientation_Trajectory(t, T, Rini_RF, Rdes, Rnext);

    LIGHT.dWdes_RF = Vector3d::Zero();
    LIGHT.Wdes_RF = Vector3d::Zero();
    LIGHT.Rdes_RF = Rnext;
}

void LIGHTWholeMotions::Submotion_Global2LF_Ori(double t, double T, Matrix3d Rdes)
{
    static Matrix3d Rini_LF;
    if (TimeIsZero()) {
        Rini_LF = LIGHT.Rref_LF;
    }

    Matrix3d Rnext;
    Orientation_Trajectory(t, T, Rini_LF, Rdes, Rnext);

    LIGHT.dWdes_LF = Vector3d::Zero();
    LIGHT.Wdes_LF = Vector3d::Zero();
    LIGHT.Rdes_LF = Rnext;
}

void LIGHTWholeMotions::Submotion_Global2RFswing_forWalking(double t_swing, double RotZdes)
{
    static double RotZini, RotYini;
    if (TimeIsZero()) {
        RotZini = atan2(LIGHT.Rref_RF(1,0), LIGHT.Rref_RF(0,0));
        RotYini = -LIGHT.Xref_LF2RF(0)/0.35*(15.0)*D2R;
    }

    double RotYdes = -(SDB[0].X_STEP(0) + SDB[0].X_STEP_adj(0))/0.35*(20.0)*D2R;

    double s = TimeNow()/t_swing;
    double RotZ_interpole = (1.0-s)*RotZini + s*RotZdes;
    double RotY_interpole = 3.0*(1.0-s)*(1.0-s)*s*RotYini + 3.0*(1.0-s)*s*s*RotYdes;

    LIGHT.dWdes_RF = Vector3d::Zero();
    LIGHT.Wdes_RF = Vector3d::Zero();
    LIGHT.Rdes_RF = RotZ(RotZ_interpole)*RotY(RotY_interpole);
}

void LIGHTWholeMotions::Submotion_Global2LFswing_forWalking(double t_swing, double RotZdes)
{
    static double RotZini, RotYini;
    if (TimeIsZero()) {
        RotZini = atan2(LIGHT.Rref_LF(1,0), LIGHT.Rref_LF(0,0));
        RotYini = -LIGHT.Xref_RF2LF(0)/0.35*(15.0)*D2R;
    }

    double RotYdes = -(SDB[0].X_STEP(0) + SDB[0].X_STEP_adj(0))/0.35*(20.0)*D2R;

    double s = TimeNow()/t_swing;
    double RotZ_interpole = (1.0-s)*RotZini + s*RotZdes;
    double RotY_interpole = 3.0*(1.0-s)*(1.0-s)*s*RotYini + 3.0*(1.0-s)*s*s*RotYdes;

    LIGHT.dWdes_LF = Vector3d::Zero();
    LIGHT.Wdes_LF = Vector3d::Zero();
    LIGHT.Rdes_LF = RotZ(RotZ_interpole)*RotY(RotY_interpole);
}


void LIGHTWholeMotions::Submotion_Pelvis2RF_Ori(double t, double T, Matrix3d Rdes)
{
    static Matrix3d Rini_Pel2RF;
    if (TimeIsZero()) {
        Rini_Pel2RF = LIGHT.Rref_Pel2RF;
    }

    Matrix3d Rnext;
    Orientation_Trajectory(t, T, Rini_Pel2RF, Rdes, Rnext);

    LIGHT.dWdes_Pel2RF = Vector3d::Zero();
    LIGHT.Wdes_Pel2RF = Vector3d::Zero();
    LIGHT.Rdes_Pel2RF = Rnext;
}

void LIGHTWholeMotions::Submotion_Pelvis2LF_Ori(double t, double T, Matrix3d Rdes)
{
    static Matrix3d Rini_Pel2LF;
    if (TimeIsZero()) {
        Rini_Pel2LF = LIGHT.Rref_Pel2LF;
    }

    Matrix3d Rnext;
    Orientation_Trajectory(t, T, Rini_Pel2LF, Rdes, Rnext);

    LIGHT.dWdes_Pel2LF = Vector3d::Zero();
    LIGHT.Wdes_Pel2LF = Vector3d::Zero();
    LIGHT.Rdes_Pel2LF = Rnext;
}


//// Position Sub-motion ///////////////////////////////////////////////////////////////////////

void LIGHTWholeMotions::Position_5th_Trajectory(double Tmove, Vector3d Xini, Vector3d dXini, Vector3d ddXini, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes, Vector3d &Xnext, Vector3d &dXnext, Vector3d &ddXnext)
{
    // Tmove : moving time
    // Xini, dXini, ddXini : current position, velocity, acceleration (3-dimension)
    // Xdes, dXdes, ddXdes : Desired position, velocity, acceleration (3-dimension)
    // Xnext, dXnext, ddXnext : Next step position, velocity, acceleration (3-dimension)

    for(int i=0;i<3;i++){
        double p = Xini(i);
        double v = dXini(i);
        double a = ddXini(i);
        double pf = Xdes(i);
        double vf = dXdes(i);
        double af = ddXdes(i);

        double pn,vn,an;
        fifth_trajectory_oneaxis(Tmove,p,v,a,pf,vf,af,pn,vn,an);

        Xnext(i) = pn;
        dXnext(i) = vn;
        ddXnext(i) = an;
    }
}

void LIGHTWholeMotions::Position_3rd_Trajectory(double Tmove, Vector3d Xini, Vector3d dXini , Vector3d Xdes, Vector3d dXdes, Vector3d &Xnext, Vector3d &dXnext)
{
    // Tmove : moving time
    // Xini, dXini, ddXini : current position, velocity (3-dimension)
    // Xdes, dXdes, ddXdes : Desired position, velocity (3-dimension)
    // Xnext, dXnext, ddXnext : Next step position, velocity (3-dimension)

    for(int i=0;i<3;i++){
        double p = Xini(i);
        double v = dXini(i);
        double pf = Xdes(i);
        double vf = dXdes(i);

        double pn,vn;
        third_trajectory_oneaxis(Tmove,p,v,pf,vf,pn,vn);

        Xnext(i) = pn;
        dXnext(i) = vn;
    }
}

void LIGHTWholeMotions::Submotion_Global2RF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_RF = LIGHT.Xref_RF;
        LIGHT.dXdes_RF = LIGHT.dXref_RF;
        LIGHT.ddXdes_RF = LIGHT.ddXref_RF;
    }

    Vector3d Xini = LIGHT.Xdes_RF;
    Vector3d dXini = LIGHT.dXdes_RF;
    Vector3d ddXini = LIGHT.ddXdes_RF;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
    //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_RF = Xnext;
    LIGHT.dXdes_RF = dXnext;
    LIGHT.ddXdes_RF = ddXnext;
}

void LIGHTWholeMotions::Submotion_Global2LF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_LF = LIGHT.Xref_LF;
        LIGHT.dXdes_LF = LIGHT.dXref_LF;
        LIGHT.ddXdes_LF = LIGHT.ddXref_LF;
    }

    Vector3d Xini = LIGHT.Xdes_LF;
    Vector3d dXini = LIGHT.dXdes_LF;
    Vector3d ddXini = LIGHT.ddXdes_LF;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
    //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_LF = Xnext;
    LIGHT.dXdes_LF = dXnext;
    LIGHT.ddXdes_LF = ddXnext;
}

void LIGHTWholeMotions::Submotion_Pelvis2RF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_Pel2RF = LIGHT.Xref_Pel2RF;
        LIGHT.dXdes_Pel2RF = LIGHT.dXref_Pel2RF;
        LIGHT.ddXdes_Pel2RF = LIGHT.ddXref_Pel2RF;
    }

    Vector3d Xini = LIGHT.Xdes_Pel2RF;
    Vector3d dXini = LIGHT.dXdes_Pel2RF;
    Vector3d ddXini = LIGHT.ddXdes_Pel2RF;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
    //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_Pel2RF = Xnext;
    LIGHT.dXdes_Pel2RF = dXnext;
    LIGHT.ddXdes_Pel2RF = ddXnext;
}

void LIGHTWholeMotions::Submotion_Pelvis2LF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_Pel2LF = LIGHT.Xref_Pel2LF;
        LIGHT.dXdes_Pel2LF = LIGHT.dXref_Pel2LF;
        LIGHT.ddXdes_Pel2LF = LIGHT.ddXref_Pel2LF;
    }

    Vector3d Xini = LIGHT.Xdes_Pel2LF;
    Vector3d dXini = LIGHT.dXdes_Pel2LF;
    Vector3d ddXini = LIGHT.ddXdes_Pel2LF;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
    //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_Pel2LF = Xnext;
    LIGHT.dXdes_Pel2LF = dXnext;
    LIGHT.ddXdes_Pel2LF = ddXnext;
}

void LIGHTWholeMotions::Submotion_RF2CoM_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_RF2CoM = LIGHT.Xref_RF2CoM;
        LIGHT.dXdes_RF2CoM = LIGHT.dXref_RF2CoM;
        LIGHT.ddXdes_RF2CoM = LIGHT.ddXref_RF2CoM;
    }

    Vector3d Xini = LIGHT.Xdes_RF2CoM;
    Vector3d dXini = LIGHT.dXdes_RF2CoM;
    Vector3d ddXini = LIGHT.ddXdes_RF2CoM;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
    //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_RF2CoM = Xnext;
    LIGHT.dXdes_RF2CoM = dXnext;
    LIGHT.ddXdes_RF2CoM = ddXnext;
}

void LIGHTWholeMotions::Submotion_RF2LF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_RF2LF = LIGHT.Xref_RF2LF;
        LIGHT.dXdes_RF2LF = LIGHT.dXref_RF2LF;
        LIGHT.ddXdes_RF2LF = LIGHT.ddXref_RF2LF;
    }

    Vector3d Xini = LIGHT.Xdes_RF2LF;
    Vector3d dXini = LIGHT.dXdes_RF2LF;
    Vector3d ddXini = LIGHT.ddXdes_RF2LF;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
        //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_RF2LF = Xnext;
    LIGHT.dXdes_RF2LF = dXnext;
    LIGHT.ddXdes_RF2LF = ddXnext;
}

void LIGHTWholeMotions::Submotion_LF2CoM_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_LF2CoM = LIGHT.Xref_LF2CoM;
        LIGHT.dXdes_LF2CoM = LIGHT.dXref_LF2CoM;
        LIGHT.ddXdes_LF2CoM = LIGHT.ddXref_LF2CoM;
    }

    Vector3d Xini = LIGHT.Xdes_LF2CoM;
    Vector3d dXini = LIGHT.dXdes_LF2CoM;
    Vector3d ddXini = LIGHT.ddXdes_LF2CoM;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
    //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_LF2CoM = Xnext;
    LIGHT.dXdes_LF2CoM = dXnext;
    LIGHT.ddXdes_LF2CoM = ddXnext;
}

void LIGHTWholeMotions::Submotion_LF2RF_Pos(double TIME, Vector3d Xdes, Vector3d dXdes, Vector3d ddXdes)
{
    if (TimeIsZero()) {
        LIGHT.Xdes_LF2RF = LIGHT.Xref_LF2RF;
        LIGHT.dXdes_LF2RF = LIGHT.dXref_LF2RF;
        LIGHT.ddXdes_LF2RF = LIGHT.ddXref_LF2RF;
    }

    Vector3d Xini = LIGHT.Xdes_LF2RF;
    Vector3d dXini = LIGHT.dXdes_LF2RF;
    Vector3d ddXini = LIGHT.ddXdes_LF2RF;

    Vector3d Xnext, dXnext, ddXnext;
    if(TIME<=SYS_DT_WALKING) {
        Xnext = Xdes;
        dXnext = dXdes;
        ddXnext = ddXdes;
    } else {
        Position_5th_Trajectory(TIME, Xini, dXini, ddXini, Xdes, dXdes, ddXdes, Xnext, dXnext, ddXnext);
    //    Position_3rd_Trajectory(TIME, Xini, dXini, Xdes, dXdes, Xnext, dXnext);
    }

    LIGHT.Xdes_LF2RF = Xnext;
    LIGHT.dXdes_LF2RF = dXnext;
    LIGHT.ddXdes_LF2RF = ddXnext;
}

double t2s(double _t_now, double _t_end)
{
    if(_t_end<0.0) {
        FILE_LOG(logERROR) << "[t2s function Error] Time Setting Error!";
        return 0.0;
    }
    double r = _t_now/_t_end;
    double s;

    if (_t_now<_t_end) {
        // 5th order polynomial
    //    s = 10.0*r*r*r - 15.0*r*r*r*r + 6.0*r*r*r*r*r;
        // 3rd order polynomial
//        s = 3.0*r*r - 2.0*r*r*r;
        // 1st order polynomial
        s = r;
    } else {
        s = 1;
    }
    return s;
}

double dt2ds(double _t_now, double _t_end)
{
    if((_t_end<0.0)||(_t_end<_t_now)) {
        FILE_LOG(logERROR) << "[dt2ds function Error] Time Setting Error!";
        return 0.0;
    }
    double r = _t_now/_t_end;
    double dr = 1.0/_t_end;
    double ds;

    if (_t_now<_t_end) {
        // 5th order polynomial
    //    ds = (30.0*r*r -60.0*r*r*r + 30.0*r*r*r*r)*dr;
        // 3rd order polynomial
//        ds = (6.0*r - 6.0*r*r)*dr;
        // 1st order polynomial
        ds = dr;
    } else {
        ds = 0;
    }
    return ds;
}

double ddt2dds(double _t_now, double _t_end)
{
    if((_t_end<0.0)||(_t_end<_t_now)) {
        FILE_LOG(logERROR) << "[ddt2dds function Error] Time Setting Error!";
        return 0.0;
    }
    double r = _t_now/_t_end;
    double dr = 1.0/_t_end;
    double ddr = 0.0;
    double dds;

    if (_t_now<_t_end) {
        // 5th order polynomial
    //    dds = (60.0*r -180.0*r*r + 120.0*r*r*r)*dr*dr+(30.0*r*r -60.0*r*r*r + 30.0*r*r*r*r)*ddr;
        // 3rd order polynomial
//        dds = (6.0 - 12.0*r)*dr*dr + (6.0*r - 6.0*r*r)*ddr;
        // 1st order polynomial
        dds = ddr;
    } else {
        dds = 0;
    }
    return dds;
}

void LIGHTWholeMotions::Submotion_LF2RFSwing_forWalking(double t_Swing, Vector3d _Xdes_LF2RF, double z_SwingUp)
{
    static Vector3d _Xini_LF2RF;
    if (TimeIsZero()) {
        _Xini_LF2RF = LIGHT.Xref_LF2RF;
    }

    // 3rd-order Bezier Curve
    double s = t2s(TimeNow(), t_Swing);
    double ds = dt2ds(TimeNow(), t_Swing);
    double dds = ddt2dds(TimeNow(), t_Swing);

    double alpha = 0.18;
    Vector3d P0 = _Xini_LF2RF;
    Vector3d P3 = _Xdes_LF2RF;
    Vector3d P1 = (1.0-alpha)*P0 + alpha*P3; P1(2) += z_SwingUp*4.0/3.0;
    Vector3d P2 = (1.0-alpha)*P3 + alpha*P0; P2(2) += z_SwingUp*4.0/3.0;
    Vector3d Ps = (1.0-s)*(1.0-s)*(1.0-s)*P0 + 3.0*s*(1.0-s)*(1.0-s)*P1 + 3.0*s*s*(1.0-s)*P2 + s*s*s*P3;
    Vector3d dPs = ds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3);
    Vector3d ddPs = dds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3)
            + ds*ds*(6.0*(1.0-s)*P0 + (-6.0*(1.0-s)-6.0*(1.0-s)+6.0*s)*P1 + (6.0*(1.0-s)-6.0*s-6.0*s)*P2 + 6.0*s*P3);

    LIGHT.Xdes_LF2RF = Ps;
    LIGHT.dXdes_LF2RF = dPs;
    LIGHT.ddXdes_LF2RF = ddPs;
}

void LIGHTWholeMotions::Submotion_RF2LFSwing_forWalking(double t_Swing, Vector3d _Xdes_RF2LF, double z_SwingUp)
{
    static Vector3d _Xini_RF2LF;
    if (TimeIsZero()) {
        _Xini_RF2LF = LIGHT.Xref_RF2LF;
    }

    // 3rd-order Bezier Curve
    double s = t2s(TimeNow(), t_Swing);
    double ds = dt2ds(TimeNow(), t_Swing);
    double dds = ddt2dds(TimeNow(), t_Swing);

    double alpha = 0.18;
    Vector3d P0 = _Xini_RF2LF;
    Vector3d P3 = _Xdes_RF2LF;
    Vector3d P1 = (1.0-alpha)*P0 + alpha*P3; P1(2) += z_SwingUp*4.0/3.0;
    Vector3d P2 = (1.0-alpha)*P3 + alpha*P0; P2(2) += z_SwingUp*4.0/3.0;
    Vector3d Ps = (1.0-s)*(1.0-s)*(1.0-s)*P0 + 3.0*s*(1.0-s)*(1.0-s)*P1 + 3.0*s*s*(1.0-s)*P2 + s*s*s*P3;
    Vector3d dPs = ds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3);
    Vector3d ddPs = dds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3)
            + ds*ds*(6.0*(1.0-s)*P0 + (-6.0*(1.0-s)-6.0*(1.0-s)+6.0*s)*P1 + (6.0*(1.0-s)-6.0*s-6.0*s)*P2 + 6.0*s*P3);

    LIGHT.Xdes_RF2LF = Ps;
    LIGHT.dXdes_RF2LF = dPs;
    LIGHT.ddXdes_RF2LF = ddPs;

}


//void LIGHTWholeMotions::Submotion_LF2RFSwing_forWalking_withHeelToe(double t_Swing, Vector3d Xdes_LF2RF, double z_SwingUp)
//{
//    static Vector3d Xini_LF2RF;
//    static Vector3d Xini_LF2RFend, Xfin_LF2RFend;
//    Vector3d Center2toe(LIGHT.LIGHT_Info->footsize_toe,0.0,0.0);
//    Vector3d Center2heel(-LIGHT.LIGHT_Info->footsize_heel,0.0,0.0);

//    if (TimeIsZero()) {
//        Xini_LF2RF = LIGHT.Xref_LF2RF;
//    }


//    if(Xini_LF2RF(0)>=0.0) {
//        Xini_LF2RFend = LIGHT.Xref_LF2RF + LIGHT.Rref_RF*Center2heel;
//    } else {
//        Xini_LF2RFend = LIGHT.Xref_LF2RF + LIGHT.Rref_RF*Center2toe;
//    }

//    if(Xdes_LF2RF(0)>=0.0) {
//        Xfin_LF2RFend = _Xdes_LF2RF + LIGHT.Rref_RF*Center2heel;
//    } else {
//        Xfin_LF2RFend = _Xdes_LF2RF + LIGHT.Rref_RF*Center2toe;
//    }


//    LIGHT.Xdes_LF2RF = Ps;
//    LIGHT.dXdes_LF2RF = dPs;
//    LIGHT.ddXdes_LF2RF = ddPs;
//}

//void LIGHTWholeMotions::Submotion_RF2LFSwing_forWalking_withHeelToe(double t_Swing, Vector3d _Xdes_RF2LF, double z_SwingUp)
//{
//    static Vector3d _Xini_RF2LF;
//    if (TimeIsZero()) {
//        _Xini_RF2LF = LIGHT.Xref_RF2LF;
//    }

//    // 3rd-order Bezier Curve
//    double s = t2s(TimeNow(), t_Swing);
//    double ds = dt2ds(TimeNow(), t_Swing);
//    double dds = ddt2dds(TimeNow(), t_Swing);

//    double alpha = 0.25;
//    Vector3d P0 = _Xini_RF2LF;
//    Vector3d P3 = _Xdes_RF2LF;
//    Vector3d P1 = (1.0-alpha)*P0 + alpha*P3; P1(2) += z_SwingUp*4.0/3.0;
//    Vector3d P2 = (1.0-alpha)*P3 + alpha*P0; P2(2) += z_SwingUp*4.0/3.0;
//    Vector3d Ps = (1.0-s)*(1.0-s)*(1.0-s)*P0 + 3.0*s*(1.0-s)*(1.0-s)*P1 + 3.0*s*s*(1.0-s)*P2 + s*s*s*P3;
//    Vector3d dPs = ds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3);
//    Vector3d ddPs = dds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3)
//            + ds*ds*(6.0*(1.0-s)*P0 + (-6.0*(1.0-s)-6.0*(1.0-s)+6.0*s)*P1 + (6.0*(1.0-s)-6.0*s-6.0*s)*P2 + 6.0*s*P3);

//    LIGHT.Xdes_RF2LF = Ps;
//    LIGHT.dXdes_RF2LF = dPs;
//    LIGHT.ddXdes_RF2LF = ddPs;

//}



void LIGHTWholeMotions::Submotion_LF2RFSwingUp(double t_Swing, Vector3d _Xdes_LF2RF)
{
    static Vector3d _Xini_LF2RF;
    if (TimeIsZero()) {
        _Xini_LF2RF = LIGHT.Xref_LF2RF;
    }

    // 3rd-order Bezier Curve
    double s = t2s(TimeNow(), t_Swing);
    double ds = dt2ds(TimeNow(), t_Swing);
    double dds = ddt2dds(TimeNow(), t_Swing);

    Vector3d P0 = _Xini_LF2RF;
    Vector3d P3 = _Xdes_LF2RF;
    Vector3d P1 = P0; P1(2) = P3(2);
    Vector3d P2 = (P3+P0)/2.0; P2(2) = P3(2);

    Vector3d Ps = (1.0-s)*(1.0-s)*(1.0-s)*P0 + 3.0*s*(1.0-s)*(1.0-s)*P1 + 3.0*s*s*(1.0-s)*P2 + s*s*s*P3;
    Vector3d dPs = ds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3);
    Vector3d ddPs = dds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3)
            + ds*ds*(6.0*(1.0-s)*P0 + (-6.0*(1.0-s)-6.0*(1.0-s)+6.0*s)*P1 + (6.0*(1.0-s)-6.0*s-6.0*s)*P2 + 6.0*s*P3);

    LIGHT.Xdes_LF2RF = Ps;
    LIGHT.dXdes_LF2RF = dPs;
    LIGHT.ddXdes_LF2RF = ddPs;
}

void LIGHTWholeMotions::Submotion_RF2LFSwingUp(double t_Swing, Vector3d _Xdes_RF2LF)
{
    static Vector3d _Xini_LF2RF;
    if (TimeIsZero()) {
        _Xini_LF2RF = LIGHT.Xref_RF2LF;
    }

    // 3rd-order Bezier Curve
    double s = t2s(TimeNow(), t_Swing);
    double ds = dt2ds(TimeNow(), t_Swing);
    double dds = ddt2dds(TimeNow(), t_Swing);

    Vector3d P0 = _Xini_LF2RF;
    Vector3d P3 = _Xdes_RF2LF;
    Vector3d P1 = P0; P1(2) = P3(2);
    Vector3d P2 = (P3+P0)/2.0; P2(2) = P3(2);

    Vector3d Ps = (1.0-s)*(1.0-s)*(1.0-s)*P0 + 3.0*s*(1.0-s)*(1.0-s)*P1 + 3.0*s*s*(1.0-s)*P2 + s*s*s*P3;
    Vector3d dPs = ds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3);
    Vector3d ddPs = dds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3)
            + ds*ds*(6.0*(1.0-s)*P0 + (-6.0*(1.0-s)-6.0*(1.0-s)+6.0*s)*P1 + (6.0*(1.0-s)-6.0*s-6.0*s)*P2 + 6.0*s*P3);

    LIGHT.Xdes_RF2LF = Ps;
    LIGHT.dXdes_RF2LF = dPs;
    LIGHT.ddXdes_RF2LF = ddPs;
}

void LIGHTWholeMotions::Submotion_LF2RFSwingDown(double t_Swing, Vector3d _Xdes_LF2RF)
{
    static Vector3d _Xini_LF2RF;
    if (TimeIsZero()) {
        _Xini_LF2RF = LIGHT.Xref_LF2RF;
    }

    // 3rd-order Bezier Curve
    double s = t2s(TimeNow(), t_Swing);
    double ds = dt2ds(TimeNow(), t_Swing);
    double dds = ddt2dds(TimeNow(), t_Swing);

    Vector3d P0 = _Xini_LF2RF;
    Vector3d P3 = _Xdes_LF2RF;
//    Vector3d P1 = P0;
//    Vector3d P2 = (P3+P0)/2.0; P2(2) = P0(2);
    Vector3d P1 = (P3+P0)/2.0; P1(2) = P0(2);
    Vector3d P2 = P3; P2(2) = P0(2);
    Vector3d Ps = (1.0-s)*(1.0-s)*(1.0-s)*P0 + 3.0*s*(1.0-s)*(1.0-s)*P1 + 3.0*s*s*(1.0-s)*P2 + s*s*s*P3;
    Vector3d dPs = ds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3);
    Vector3d ddPs = dds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3)
            + ds*ds*(6.0*(1.0-s)*P0 + (-6.0*(1.0-s)-6.0*(1.0-s)+6.0*s)*P1 + (6.0*(1.0-s)-6.0*s-6.0*s)*P2 + 6.0*s*P3);

    LIGHT.Xdes_LF2RF = Ps;
    LIGHT.dXdes_LF2RF = dPs;
    LIGHT.ddXdes_LF2RF = ddPs;
}

void LIGHTWholeMotions::Submotion_RF2LFSwingDown(double t_Swing, Vector3d _Xdes_RF2LF)
{
    static Vector3d _Xini_RF2LF;
    if (TimeIsZero()) {
        _Xini_RF2LF = LIGHT.Xref_RF2LF;
    }

    // 3rd-order Bezier Curve
    double s = t2s(TimeNow(), t_Swing);
    double ds = dt2ds(TimeNow(), t_Swing);
    double dds = ddt2dds(TimeNow(), t_Swing);

    Vector3d P0 = _Xini_RF2LF;
    Vector3d P3 = _Xdes_RF2LF;
//    Vector3d P1 = P0;
//    Vector3d P2 = P3; P2(2) = P0(2);
    Vector3d P1 = (P3+P0)/2.0; P1(2) = P0(2);
    Vector3d P2 = P3; P2(2) = P0(2);
    Vector3d Ps = (1.0-s)*(1.0-s)*(1.0-s)*P0 + 3.0*s*(1.0-s)*(1.0-s)*P1 + 3.0*s*s*(1.0-s)*P2 + s*s*s*P3;
    Vector3d dPs = ds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3);
    Vector3d ddPs = dds*(-3.0*(1.0-s)*(1.0-s)*P0 + (3.0*(1.0-s)*(1.0-s)-6.0*s*(1.0-s))*P1 + (6.0*s*(1.0-s)-3.0*s*s)*P2 + 3.0*s*s*P3)
            + ds*ds*(6.0*(1.0-s)*P0 + (-6.0*(1.0-s)-6.0*(1.0-s)+6.0*s)*P1 + (6.0*(1.0-s)-6.0*s-6.0*s)*P2 + 6.0*s*P3);

    LIGHT.Xdes_RF2LF = Ps;
    LIGHT.dXdes_RF2LF = dPs;
    LIGHT.ddXdes_RF2LF = ddPs;
}

//// Joint Sub-motion ///////////////////////////////////////////////////////////////////////
void LIGHTWholeMotions::Joint_5th_Trajectory(double Tmove, double th_ini, double dth_ini, double ddth_ini, double th_des, double dth_des, double ddth_des, double &th_next, double &dth_next, double &ddth_next)
{
    // Tmove : moving time
    // th_ini, dth_ini, ddth_ini : current position, velocity, acceleration (joint angle)
    // th_des, dth_des, ddth_des : Desired position, velocity, acceleration (joint angle)
    // th_next, dth_next, ddth_next : Next step position, velocity, acceleration (joint angle)

    double p = th_ini;
    double v = dth_ini;
    double a = ddth_ini;
    double pf = th_des;
    double vf = dth_des;
    double af = ddth_des;

    double pn,vn,an;
    fifth_trajectory_oneaxis(Tmove,p,v,a,pf,vf,af,pn,vn,an);

    th_next = pn;
    dth_next = vn;
    ddth_next = an;
}

void LIGHTWholeMotions::Joint_3rd_Trajectory(double Tmove, double th_ini, double dth_ini, double th_des, double dth_des, double &th_next, double &dth_next)
{
    // Tmove : moving time
    // th_ini, dth_ini : current position, velocity (joint angle)
    // th_des, dth_des : Desired position, velocity (joint angle)
    // th_next, dth_next : Next step position, velocity (joint angle)

    double p = th_ini;
    double v = dth_ini;
    double pf = th_des;
    double vf = dth_des;

    double pn,vn;
    third_trajectory_oneaxis(Tmove,p,v,pf,vf,pn,vn);

    th_next = pn;
    dth_next = vn;
}

void LIGHTWholeMotions::Submotion_RightLeg_Q(double TIME, VectorNd Qdes, VectorNd dQdes, VectorNd ddQdes)
{
    if (TimeIsZero()) {
        for(int idx = 0; idx<LEG_DOF; idx++) {
            LIGHT.Qdes(RL_QNUMSTART+idx) = LIGHT.Qref(RL_QNUMSTART+idx);
            LIGHT.dQdes(RL_QNUMSTART+idx) = 0.0;//LIGHT.dQref(RL_QNUMSTART+idx);
            LIGHT.ddQdes(RL_QNUMSTART+idx) = 0.0;//LIGHT.ddQref(RL_QNUMSTART+idx);
        }
    }

    for(int idx = 0; idx<LEG_DOF; idx++) {
        double th_ini = LIGHT.Qdes(RL_QNUMSTART+idx);
        double dth_ini = LIGHT.dQdes(RL_QNUMSTART+idx);
        double ddth_ini = LIGHT.ddQdes(RL_QNUMSTART+idx);
        double th_des = Qdes(idx);
        double dth_des = dQdes(idx);
        double ddth_des = ddQdes(idx);

        double th_next, dth_next, ddth_next;
        Joint_5th_Trajectory(TIME, th_ini, dth_ini, ddth_ini, th_des, dth_des, ddth_des, th_next, dth_next, ddth_next);
        //        Joint_3rd_Trajectory(TIME, th_ini, dth_ini, th_des, dth_des, th_next, dth_next);

        LIGHT.Qdes(RL_QNUMSTART+idx) = th_next;
        LIGHT.dQdes(RL_QNUMSTART+idx) = dth_next;
        LIGHT.ddQdes(RL_QNUMSTART+idx) = ddth_next;
    }
}

void LIGHTWholeMotions::Submotion_LeftLeg_Q(double TIME, VectorNd Qdes, VectorNd dQdes, VectorNd ddQdes)
{
    if (TimeIsZero()) {
        for(int idx = 0; idx<LEG_DOF; idx++) {
            LIGHT.Qdes(LL_QNUMSTART+idx) = LIGHT.Qref(LL_QNUMSTART+idx);
            LIGHT.dQdes(LL_QNUMSTART+idx) = 0.0;//LIGHT.dQref(LL_QNUMSTART+idx);
            LIGHT.ddQdes(LL_QNUMSTART+idx) = 0.0;//LIGHT.ddQref(LL_QNUMSTART+idx);
        }
    }
    for(int idx = 0; idx<LEG_DOF; idx++) {
        double th_ini = LIGHT.Qdes(LL_QNUMSTART+idx);
        double dth_ini = LIGHT.dQdes(LL_QNUMSTART+idx);
        double ddth_ini = LIGHT.ddQdes(LL_QNUMSTART+idx);
        double th_des = Qdes(idx);
        double dth_des = dQdes(idx);
        double ddth_des = ddQdes(idx);

        double th_next, dth_next, ddth_next;
        Joint_5th_Trajectory(TIME, th_ini, dth_ini, ddth_ini, th_des, dth_des, ddth_des, th_next, dth_next, ddth_next);
        //        Joint_3rd_Trajectory(TIME, th_ini, dth_ini, th_des, dth_des, th_next, dth_next);

        LIGHT.Qdes(LL_QNUMSTART+idx) = th_next;
        LIGHT.dQdes(LL_QNUMSTART+idx) = dth_next;
        LIGHT.ddQdes(LL_QNUMSTART+idx) = ddth_next;
    }
}

void LIGHTWholeMotions::Submotion_Waist_Q(double TIME, VectorNd Qdes, VectorNd dQdes, VectorNd ddQdes)
{
    if (TimeIsZero()) {
        for(int idx = 0; idx<WST_DOF; idx++) {
            LIGHT.Qdes(WST_QNUMSTART+idx) = LIGHT.Qref(WST_QNUMSTART+idx);
            LIGHT.dQdes(WST_QNUMSTART+idx) = 0.0;//LIGHT.dQref(WST_QNUMSTART+idx);
            LIGHT.ddQdes(WST_QNUMSTART+idx) = 0.0;//LIGHT.ddQref(WST_QNUMSTART+idx);
        }
    }

    for(int idx = 0; idx<WST_DOF; idx++) {
        double th_ini = LIGHT.Qdes(WST_QNUMSTART+idx);
        double dth_ini = LIGHT.dQdes(WST_QNUMSTART+idx);
        double ddth_ini = LIGHT.ddQdes(WST_QNUMSTART+idx);
        double th_des = Qdes(idx);
        double dth_des = dQdes(idx);
        double ddth_des = ddQdes(idx);

        double th_next, dth_next, ddth_next;
        Joint_5th_Trajectory(TIME, th_ini, dth_ini, ddth_ini, th_des, dth_des, ddth_des, th_next, dth_next, ddth_next);
        //        Joint_3rd_Trajectory(TIME, th_ini, dth_ini, th_des, dth_des, th_next, dth_next);

        LIGHT.Qdes(WST_QNUMSTART+idx) = th_next;
        LIGHT.dQdes(WST_QNUMSTART+idx) = dth_next;
        LIGHT.ddQdes(WST_QNUMSTART+idx) = ddth_next;
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
////////// Part Motion Module Functions /////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

bool LIGHTWholeMotions::JointSpaceMove_ALL(double _TIME, MatrixNd _Qdes){
    if (TimeIsZero()) {
        ClearDesiredMotions();
        LIGHT.CurRefStateIs_NOACT();

        // Set Cost Weight for Kinematics Problem
        INFO_InvKin.WeightSet_JointMotion();

        TimeLimitSet(_TIME);
    }

    Calc_FuturePressureReference(TimeNow(), _TIME, PumpPressure_FLOAT);

    // =============== Global >> Pelvis Orientation and Position Trajectory ===============
    Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),I3);
    // =============== RightLeg Joint Trajectory ===============
    MatrixNd Qdes_RightLeg = _Qdes.block(RL_JNUMSTART,0,LEG_DOF,1);
    MatrixNd dQdes_RightLeg = MatrixNd::Zero(LEG_DOF,1);
    MatrixNd ddQdes_RightLeg = MatrixNd::Zero(LEG_DOF,1);
    Submotion_RightLeg_Q(TimeLeft(), Qdes_RightLeg, dQdes_RightLeg, ddQdes_RightLeg);
    // =============== LeftLeg Joint Trajectory ===============
    MatrixNd Qdes_LeftLeg = _Qdes.block(LL_JNUMSTART,0,LEG_DOF,1);
    MatrixNd dQdes_LeftLeg = MatrixNd::Zero(LEG_DOF,1);
    MatrixNd ddQdes_LeftLeg = MatrixNd::Zero(LEG_DOF,1);
    Submotion_LeftLeg_Q(TimeLeft(), Qdes_LeftLeg, dQdes_LeftLeg, ddQdes_LeftLeg);

    SupportControl_Noact();

    TimeUpdate();
    if(TimeCheck()) {
        TimeReset();
        return true;
    } else { return false; }

}

bool LIGHTWholeMotions::WorkSpaceMove_ALL(double _TIME, Matrix3d _Rdes_RF ,Vector3d _Xdes_RF, Matrix3d _Rdes_LF, Vector3d _Xdes_LF){
    if (TimeIsZero()) {
        ClearDesiredMotions();
        LIGHT.CurRefStateIs_FLOAT();
//        SetSupportControlGain_Float();

        // Set Cost Weight for Kinematics Problem
        INFO_InvKin.WeightSet_PelBaseMotion();

        TimeLimitSet(_TIME);
    }

    Calc_FuturePressureReference(TimeNow(), _TIME, PumpPressure_FLOAT);

    // =============== Global >> Pelvis Orientation and Position Trajectory ===============
    Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),I3);
    // =============== Pelvis >> RightFoot Orientation and Position Trajectory ===============
    Submotion_Pelvis2RF_Ori(TimeNow(), TimeLimit(), _Rdes_RF);
    Submotion_Pelvis2RF_Pos(TimeLeft(), _Xdes_RF, Vector3d::Zero(), Vector3d::Zero());
    // =============== Pelvis >> LeftFoot Orientation and Position Trajectory ===============
    Submotion_Pelvis2LF_Ori(TimeNow(), TimeLimit(), _Rdes_LF);
    Submotion_Pelvis2LF_Pos(TimeLeft(), _Xdes_LF, Vector3d::Zero(), Vector3d::Zero());

    TimeUpdate();
    if(TimeCheck()) {
        TimeReset();
        return true;
    } else { return false; }
}


bool LIGHTWholeMotions::CoM_Move_RDSP(double _TIME,
                                      Matrix3d _Rdes_Global2Pel,
                                      Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                                      Vector3d _Xdes_RF2CoM, Vector3d _dXdes_RF2CoM, Vector3d _ddXdes_RF2CoM)
{
    if (TimeIsZero()) {
        SetMotionBase_RF(true);

        _Xini_temp = LIGHT.Xref_RF2LF;
        TimeLimitSet(_TIME);
    }

    Calc_FuturePressureReference(TimeNow(), _TIME, PumpPressure_DSP);

    // =============== Global >> Pelvis Orientation and Position Trajectory ===============
    Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),_Rdes_Global2Pel);
    // =============== Global >> RF Orientation Trajectory ===============
    Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2RF);
    // =============== Global >> Pelvis Orientation and Position Trajectory ===============
    Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2LF);
    // =============== RF >> CoM Position Trajectory ===============
    Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, _dXdes_RF2CoM, _ddXdes_RF2CoM);
    // =============== RF >> LF Position Trajectory ===============
    Submotion_RF2LF_Pos(TimeLeft(), _Xini_temp, zv, zv);

    TimeUpdate();
    if(TimeCheck()) {
        TimeReset();
        FILE_LOG(logSUCCESS) << " CoM Moving Motion is done!! ";
        return true;
    } else { return false; }
}

bool LIGHTWholeMotions::CoM_Move_LDSP(double _TIME,
                                      Matrix3d _Rdes_Global2Pel,
                                      Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                                      Vector3d _Xdes_LF2CoM, Vector3d _dXdes_LF2CoM, Vector3d _ddXdes_LF2CoM)
{
    if (TimeIsZero()) {
        SetMotionBase_LF(true);

        _Xini_temp = LIGHT.Xref_LF2RF;
        TimeLimitSet(_TIME);
    }

    Calc_FuturePressureReference(TimeNow(), _TIME, PumpPressure_DSP);

    // =============== Global >> Pelvis Orientation and Position Trajectory ===============
    Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),_Rdes_Global2Pel);
    // =============== Global >> LF Orientation Trajectory ===============
    Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2LF);
    // =============== Global >> RF Orientation Trajectory ===============
    Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2RF);
    // =============== LF >> CoM Position Trajectory ===============
    Submotion_LF2CoM_Pos(TimeLeft(), _Xdes_LF2CoM, _dXdes_LF2CoM, _ddXdes_LF2CoM);
    // =============== RF >> LF Position Trajectory ===============
    Submotion_LF2RF_Pos(TimeLeft(), _Xini_temp, zv, zv);

    TimeUpdate();
    if(TimeCheck()) {
        TimeReset();
        FILE_LOG(logSUCCESS) << " CoM Moving Motion is done!! ";
        return true;
    } else { return false; }
}

bool LIGHTWholeMotions::CoM_Move_RSSP(double _TIME,
                                      Matrix3d _Rdes_Global2Pel,
                                      Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                                      Vector3d _Xdes_RF2CoM, Vector3d _dXdes_RF2CoM, Vector3d _ddXdes_RF2CoM,
                                      Vector3d _Xdes_RF2LF, Vector3d _dXdes_RF2LF, Vector3d _ddXdes_RF2LF)
{
    if (TimeIsZero()) {
        SetMotionBase_RF(false);

        TimeLimitSet(_TIME);
    }

    Calc_FuturePressureReference(TimeNow(), _TIME, PumpPressure_SSP);

    // =============== Global >> RF Orientation Trajectory ===============
    Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),_Rdes_Global2Pel);
    // =============== Global >> RF Orientation Trajectory ===============
    Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2RF);
    // =============== Global >> LF Orientation Trajectory ===============
    Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2LF);
    // =============== RF >> CoM Position Trajectory ===============
    Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, _dXdes_RF2CoM, _ddXdes_RF2CoM);
    // =============== RF >> LF Position Trajectory ===============
    Submotion_RF2LF_Pos(TimeLeft(), _Xdes_RF2LF, _dXdes_RF2LF, _ddXdes_RF2LF);

    TimeUpdate();
    if(TimeCheck()) {
        TimeReset();
        FILE_LOG(logSUCCESS) << " CoM Moving Motion is done!! ";
        return true;
    } else { return false; }
}

bool LIGHTWholeMotions::CoM_Move_LSSP(double _TIME,
                                      Matrix3d _Rdes_Global2Pel,
                                      Matrix3d _Rdes_Global2RF, Matrix3d _Rdes_Global2LF,
                                      Vector3d _Xdes_LF2CoM, Vector3d _dXdes_LF2CoM, Vector3d _ddXdes_LF2CoM,
                                      Vector3d _Xdes_LF2RF, Vector3d _dXdes_LF2RF, Vector3d _ddXdes_LF2RF)
{
    if (TimeIsZero()) {
        SetMotionBase_LF(false);

        TimeLimitSet(_TIME);
    }

    Calc_FuturePressureReference(TimeNow(), _TIME, PumpPressure_SSP);

    // =============== Global >> Pelvis Orientation and Position Trajectory ===============
    Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),_Rdes_Global2Pel);
    // =============== Global >> LF Orientation Trajectory ===============
    Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2LF);
    // =============== Global >> RF Orientation Trajectory ===============
    Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),_Rdes_Global2RF);
    // =============== LF >> CoM Position Trajectory ===============
    Submotion_LF2CoM_Pos(TimeLeft(), _Xdes_LF2CoM, _dXdes_LF2CoM, _ddXdes_LF2CoM);
    // =============== LF >> RF Position Trajectory ===============
    Submotion_LF2RF_Pos(TimeLeft(), _Xdes_LF2RF, _dXdes_LF2RF, _ddXdes_LF2RF);

    TimeUpdate();
    if(TimeCheck()) {
        TimeReset();
        FILE_LOG(logSUCCESS) << " CoM Moving Motion is done!! ";
        return true;
    } else { return false; }
}

int _CurTransStage = 0;
bool _IsChecked_CtrlMode = false;
int LIGHTWholeMotions::SupportPhase_Transition(int _TR_MODE) {
    double OffsetY = 0.0;
    double v_swingdown = -0.5;

    switch(_TR_MODE) {
    case SUPPORTPHASE_TRANSITION_RDSP_RSSP:
    {
        if(LIGHT.IsCurRefState_DSP()||_IsChecked_CtrlMode) {
            _IsChecked_CtrlMode = true;
            switch(_CurTransStage) {
            case 0:
            {
                if (TimeIsZero()) {
                    LIGHT.CurRefStateIs_RDSP();
//                    SetSupportControlGain_DSP();

                    // Set Cost Weight for Kinematics Problem
                    INFO_InvKin.WeightSet_RFBaseMotion();

                    FILE_LOG(logWARNING) << " Support Phase Transition : RDSP > RSSP ";
                    TimeLimitSet(4.0);
                }

                Calc_FuturePressureReference(TimeNow(), 4.0, PumpPressure_SSP);

                // =============== RF >> CoM Position Trajectory ===============
                Vector3d _Xdes_RF2CoM(0.0, OffsetY, Pelvis_BaseHeight);
                Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, zv, zv);
                // =============== RF >> LF Position Trajectory ===============
                Vector3d _Xdes_RF2LF = LIGHT.Xref_RF2LF;
                Submotion_RF2LF_Pos(TimeLeft(), _Xdes_RF2LF, zv, zv);

                TimeUpdate();
                if(TimeCheck()) {
                    TimeReset();
                    _CurTransStage++;
                } return false;
            }
            case 1:
            {
                if (TimeIsZero()) {
                    LIGHT.CurRefStateIs_RSSP();
//                    SetSupportControlGain_RSSP();

                    _Xini_temp = LIGHT.Xref_RF2LF;
                    TimeLimitSet(3.0);
                }
                // =============== RF >> CoM Position Trajectory ===============
                Vector3d _Xdes_RF2CoM(0.0, OffsetY, Pelvis_BaseHeight);
                Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, zv, zv);
                // =============== RF >> LF Position Trajectory ===============
                Vector3d _Xdes_RF2LF(_Xini_temp(0), _Xini_temp(1), 0.10);
                Submotion_RF2LF_Pos(TimeLeft(), _Xdes_RF2LF, zv, zv);

                TimeUpdate();
                if(TimeCheck()) {
                    TimeReset();
                    _CurTransStage=0;
                    _IsChecked_CtrlMode = false;
                    FILE_LOG(logSUCCESS) << " Transition Complete! ";
                    return true;
                } else {
                    return false;
                }
            }
            }
        } else {
            FILE_LOG(logERROR) << " [Error] Current support phase is not RDSP.";
            return -1;
        }
    }
    case SUPPORTPHASE_TRANSITION_RSSP_RDSP:
    {
        if(LIGHT.IsCurRefState_RSSP()||_IsChecked_CtrlMode) {
            _IsChecked_CtrlMode = true;
            switch(_CurTransStage) {
            case 0: // Swing Leg Down
            {
                if (TimeIsZero()) {
                    FILE_LOG(logWARNING) << " Support Phase Transition : RSSP > RDSP ";
                    _Xini_temp = LIGHT.Xref_RF2LF;
                    TimeLimitSet(1.0);
                }
                // =============== RF >> CoM Position Trajectory ===============
                Vector3d _Xdes_RF2CoM(0.0, OffsetY, Pelvis_BaseHeight);
                Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, zv, zv);
                // =============== RF >> LF Position Trajectory ===============
                Vector3d _Xdes_RF2LF(_Xini_temp(0), _Xini_temp(1), 0.0);
                Vector3d _dXdes_RF2LF(0.0, 0.0, v_swingdown);
                Submotion_RF2LF_Pos(TimeLeft(), _Xdes_RF2LF, _dXdes_RF2LF, zv);

                if (LIGHT.IsContacted_LF()) {
                    TimeReset();
                    Submotion_RF2LF_Pos(TimeLeft(), _Xdes_RF2LF, zv, zv);
                    FILE_LOG(logERROR) << " Early Contact Detection! ";
                    _CurTransStage = 2;
                } else {
                    TimeUpdate();
                    if(TimeCheck()) {
                        TimeReset();
                        _CurTransStage = 1;
                    }
                } return false;
            }
            case 1: // Ground Contact
            {
                // =============== RF >> LF Position Trajectory ===============
                Vector3d _Xdes_RF2LF = LIGHT.Xref_RF2LF;
                Vector3d _dXdes_RF2LF(0.0, 0.0, v_swingdown);
//                _Xdes_RF2LF(2) = _Xdes_RF2LF(2) + v_swingdown*SYS_DT; // swing leg down speed : 0.7m/s
                Submotion_RF2LF_Pos(SYS_DT_WALKING, _Xdes_RF2LF, _dXdes_RF2LF, zv);
                FILE_LOG(logERROR) << " Contact Detection is not occured! ";
                FILE_LOG(logERROR) << " Left Fz Force : " << LIGHT.Fextnow_LF_byJTS(2);

                if (LIGHT.IsContacted_LF()) {
                    _CurTransStage = 2;
                } return false;
            }
            case 2: // Contact post-processing
            {
                if (TimeIsZero()) {
                    _Xini_temp2 = LIGHT.Xref_RF2LF;
                    TimeLimitSet(0.1);
                }

                Calc_FuturePressureReference(TimeNow(), 0.1, PumpPressure_DSP);

                // =============== RF >> CoM Position Trajectory ===============
                Vector3d _Xdes_RF2CoM(0.0, 0.0, Pelvis_BaseHeight);
                Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, zv, zv);

                LIGHT.Xdes_RF2LF = _Xini_temp2;
                LIGHT.dXdes_RF2LF = zv;

                TimeUpdate();
                if(TimeCheck()) {
                    TimeReset();
                    _CurTransStage = 3;
                } return false;
            }
            case 3: // Support Control Mode Change
            {
                LIGHT.CurRefStateIs_RDSP();
//                SetSupportControlGain_DSP();

                _CurTransStage=0;
                _IsChecked_CtrlMode = false;
                FILE_LOG(logSUCCESS) << " Transition Complete! ";
                return true;
            }
            }
        } else {
            FILE_LOG(logERROR) << " [Error] Current support phase is not RSSP.";
            return -1;
        }
    }
    case SUPPORTPHASE_TRANSITION_RDSP_LDSP:
    {
        if(LIGHT.IsCurRefState_RDSP()) {
            LIGHT.CurRefStateIs_LDSP();

            // Set Cost Weight for Kinematics Problem
            INFO_InvKin.WeightSet_LFBaseMotion();

            _Xini_temp = LIGHT.Xref_RF2CoM-LIGHT.Xref_RF2LF;
            _Xini_temp2 = -LIGHT.Xref_RF2LF;
            // =============== LF >> CoM Position Trajectory ===============
            Vector3d _Xdes_LF2CoM = _Xini_temp;
            Submotion_LF2CoM_Pos(SYS_DT_WALKING, _Xdes_LF2CoM, zv, zv);
            // =============== LF >> RF Position Trajectory ===============
            Vector3d _Xdes_LF2RF = _Xini_temp2;
            Submotion_LF2RF_Pos(SYS_DT_WALKING, _Xdes_LF2RF, zv, zv);
            return true;
        } else {
            FILE_LOG(logERROR) << " [Error] Current support phase is not RDSP.";
            return -1;
        }
    }
    case SUPPORTPHASE_TRANSITION_LDSP_RDSP:
    {
        if(LIGHT.IsCurRefState_LDSP()) {
            LIGHT.CurRefStateIs_RDSP();

            // Set Cost Weight for Kinematics Problem
            INFO_InvKin.WeightSet_RFBaseMotion();

            _Xini_temp = LIGHT.Xref_LF2CoM-LIGHT.Xref_LF2RF;
            _Xini_temp2 = -LIGHT.Xref_LF2RF;
            // =============== LF >> CoM Position Trajectory ===============
            Vector3d _Xdes_RF2CoM = _Xini_temp;
            Submotion_RF2CoM_Pos(TimeLeft(), _Xdes_RF2CoM, zv, zv);
            // =============== LF >> RF Position Trajectory ===============
            Vector3d _Xdes_RF2LF = _Xini_temp2;
            Submotion_RF2LF_Pos(TimeLeft(), _Xdes_RF2LF, zv, zv);
            return true;
        } else {
            FILE_LOG(logERROR) << " [Error] Current support phase is not LDSP.";
            return -1;
        }
    }
    case SUPPORTPHASE_TRANSITION_LDSP_LSSP:
    {
        if(LIGHT.IsCurRefState_DSP()||_IsChecked_CtrlMode) {
            _IsChecked_CtrlMode = true;
            switch(_CurTransStage) {
            case 0:
            {
                if (TimeIsZero()) {
                    LIGHT.CurRefStateIs_LDSP();

                    // Set Cost Weight for Kinematics Problem
                    INFO_InvKin.WeightSet_LFBaseMotion();

                    FILE_LOG(logWARNING) << " Support Phase Transition : LDSP > LSSP ";
                    TimeLimitSet(4.0);
                }

                Calc_FuturePressureReference(TimeNow(), 4.0, PumpPressure_SSP);

                // =============== LF >> CoM Position Trajectory ===============
                Vector3d _Xdes_LF2CoM(0.0, -OffsetY, Pelvis_BaseHeight);
                Submotion_LF2CoM_Pos(TimeLeft(), _Xdes_LF2CoM, zv, zv);
                // =============== LF >> RF Position Trajectory ===============
                Vector3d _Xdes_LF2RF = LIGHT.Xref_LF2RF;
                Submotion_LF2RF_Pos(TimeLeft(), _Xdes_LF2RF, zv, zv);

                TimeUpdate();
                if(TimeCheck()) {
                    TimeReset();
                    _CurTransStage++;
                } return false;
            }
            case 1:
            {
                if (TimeIsZero()) {
                    LIGHT.CurRefStateIs_LSSP();
//                    SetSupportControlGain_LSSP();
                    _Xini_temp = LIGHT.Xref_LF2RF;
                    TimeLimitSet(3.0);
                }
                // =============== LF >> CoM Position Trajectory ===============
                Vector3d _Xdes_LF2CoM(0.0, -OffsetY, Pelvis_BaseHeight);
                Submotion_LF2CoM_Pos(TimeLeft(), _Xdes_LF2CoM, zv, zv);
                // =============== LF >> RF Position Trajectory ===============
                Vector3d _Xdes_LF2RF(_Xini_temp(0), _Xini_temp(1), 0.10);
                Submotion_LF2RF_Pos(TimeLeft(), _Xdes_LF2RF, zv, zv);

                TimeUpdate();
                if(TimeCheck()) {
                    TimeReset();
                    _CurTransStage=0;
                    _IsChecked_CtrlMode = false;
                    FILE_LOG(logSUCCESS) << " Transition Complete! ";
                    return true;
                } else {
                    return false;
                }
            }
            }
        } else {
            FILE_LOG(logERROR) << " [Error] Current support phase is not LDSP.";
            return -1;
        }    }
    case SUPPORTPHASE_TRANSITION_LSSP_LDSP:
    {
        if(LIGHT.IsCurRefState_LSSP()||_IsChecked_CtrlMode) {
            _IsChecked_CtrlMode = true;
            switch(_CurTransStage) {
            case 0: // Swing Leg Down
            {
                if (TimeIsZero()) {
                    FILE_LOG(logWARNING) << " Support Phase Transition : LSSP > LDSP ";
                    _Xini_temp = LIGHT.Xref_LF2RF;
                    TimeLimitSet(1.0);
                }
                // =============== LF >> CoM Position Trajectory ===============
                Vector3d _Xdes_LF2CoM(0.0, -OffsetY, Pelvis_BaseHeight);
                Submotion_LF2CoM_Pos(TimeLeft(), _Xdes_LF2CoM, zv, zv);
                // =============== LF >> RF Position Trajectory ===============
                Vector3d _Xdes_LF2RF(_Xini_temp(0), _Xini_temp(1), 0.0);
                Submotion_LF2RF_Pos(TimeLeft(), _Xdes_LF2RF, zv, zv);

                if (LIGHT.IsContacted_RF()) {
                    TimeReset();
                    FILE_LOG(logERROR) << " Early Contact Detection! ";
                    _CurTransStage = 2;
                } else {
                    TimeUpdate();
                    if(TimeCheck()) {
                        TimeReset();
                        _CurTransStage = 1;
                    }
                } return false;
            }
            case 1: // Late contact
            {
                // =============== RF >> LF Position Trajectory ===============
                Vector3d _Xdes_LF2RF = LIGHT.Xref_LF2RF;
                _Xdes_LF2RF(2) = _Xdes_LF2RF(2) - 0.7*SYS_DT_WALKING; // swing leg down speed : 0.7m/s
                Submotion_LF2RF_Pos(SYS_DT_WALKING, _Xdes_LF2RF, zv, zv);
                FILE_LOG(logERROR) << " Contact Detection is not occured! ";
                FILE_LOG(logERROR) << " Right Fz Force : " << LIGHT.Fextnow_RF_byJTS(2);

                if (LIGHT.IsContacted_RF()) {
                    _CurTransStage = 2;
                } return false;
            }
            case 2: // Contact post-processing
            {
                if (TimeIsZero()) {
                    _Xini_temp2 = LIGHT.Xref_LF2RF;
                    TimeLimitSet(0.1);
                }

                Calc_FuturePressureReference(TimeNow(), 0.1, PumpPressure_DSP);

                // =============== LF >> CoM Position Trajectory ===============
                Vector3d _Xdes_LF2CoM(0.0, 0.0, Pelvis_BaseHeight);
                Submotion_LF2CoM_Pos(TimeLeft(), _Xdes_LF2CoM, zv, zv);

                LIGHT.Xdes_LF2RF = _Xini_temp2;
                LIGHT.dXdes_LF2RF = zv;

                TimeUpdate();
                if(TimeCheck()) {
                    TimeReset();
                    _CurTransStage = 3;
                } return false;
            }
            case 3: // Support Control Mode Change
            {
                LIGHT.CurRefStateIs_LDSP();
//                SetSupportControlGain_DSP();
                _CurTransStage=0;
                _IsChecked_CtrlMode = false;
                FILE_LOG(logSUCCESS) << " Transition Complete! ";
                return true;
            }
            }
        } else {
            FILE_LOG(logERROR) << " [Error] Current support phase is not LSSP.";
            return -1;
        }    }
    case SUPPORTPHASE_TRANSITION_NOACT:
        break;
    default:
        break;
    }
    return false;
}

enum Squat_STAGE {
    Squat_READY = 0,
    Squat_MOVE,
    Squat_TERMINATE,
    Squat_FINISH
};
unsigned int Squat_CurrentStage = Squat_READY;

bool LIGHTWholeMotions::Squat_DSP(double _HEIGHT, double _TIME, double _NUM) {

    Pelvis_SquatStartHeight = Pelvis_BaseHeight + 0.03;
    Vector3d _Xdes_RF2CoM = Vector3d(0.0, LIGHT.Xref_RF2LF(1)/2.0, Pelvis_SquatStartHeight);

    switch(Squat_CurrentStage) {
    case Squat_READY:
    {
        double t_ready = 5.0;
        if (TimeIsZero()) {
            _Xini_temp2 = LIGHT.Xref_RF2CoM; // Original Posture
            FILE_LOG(logWARNING) << " Squat Motion Ready... ";
        }

        bool FINISH = CoM_Move_RDSP(t_ready,
                                    I3,I3,I3,
                                    _Xdes_RF2CoM,zv,zv);
        if(FINISH) {
            Squat_CurrentStage = Squat_MOVE;
        }  return false; // not finished!
        break;
    }
    case Squat_MOVE:
    {
        if (TimeIsZero()) {
            _Xini_temp = LIGHT.Xref_RF2CoM;
            TimeLimitSet(_TIME*_NUM - 0.4*_TIME);
            FILE_LOG(logWARNING) << " Squat Motion Start!";
        }

        double _qdes_BodyPitch = (0.0*D2R)/2.0*(1.0-cos(2.0*PI*TimeNow()/_TIME));
//        double _dqdes_BodyPitch = (3.0*D2R)/2.0*(2.0*PI/_TIME)*sin(2.0*PI*TimeNow()/_TIME);
//        double _ddqdes_BodyPitch = (3.0*D2R)/2.0*(2.0*PI/_TIME)*(2.0*PI/_TIME)*cos(2.0*PI*TimeNow()/_TIME);
        Matrix3d _Rdes_BodyPitch = RotY(_qdes_BodyPitch);
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_Global2Pel_Ori(0.0, 0.0, _Rdes_BodyPitch);

        Vector3d _Xdes_RF2CoM = _Xini_temp;
        Vector3d _dXdes_RF2CoM = Vector3d(0.0, 0.0, 0.0);
        Vector3d _ddXdes_RF2CoM = Vector3d(0.0, 0.0, 0.0);
        _Xdes_RF2CoM(2) = _Xini_temp(2) - _HEIGHT/2.0*(1.0-cos(2.0*PI*TimeNow()/_TIME));
        _dXdes_RF2CoM(2) = -_HEIGHT/2.0*(2.0*PI/_TIME)*sin(2.0*PI*TimeNow()/_TIME);
        _ddXdes_RF2CoM(2) =  -_HEIGHT/2.0*(2.0*PI/_TIME)*(2.0*PI/_TIME)*cos(2.0*PI*TimeNow()/_TIME);
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT_WALKING, _Xdes_RF2CoM, _dXdes_RF2CoM, _ddXdes_RF2CoM);

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            Squat_CurrentStage = Squat_TERMINATE;
        } return false; // not finished!
        break;
    }
    case Squat_TERMINATE:
    {
        bool FINISH = CoM_Move_RDSP(1.0*_TIME,
                                    I3,I3,I3,
                                    _Xini_temp2,zv,zv);
        if(FINISH) {
            Squat_CurrentStage = Squat_READY;
            return true;
        } else {
            return false;
        }
        break;
    }
    }
}

bool LIGHTWholeMotions::Squat_SSP(double _HEIGHT, double _TIME, double _NUM)
{

    bool RorL = true; // true : RSSP, false : LSSP
    if(LIGHT.IsCurRefState_RSSP()) {
        RorL = true;
    } else if (LIGHT.IsCurRefState_LSSP()) {
        RorL = false;
    }

    static Vector3d _Xdes_CoM,_Xdes_RF2LF,_Xdes_LF2RF;
    switch(Squat_CurrentStage) {
    case Squat_READY:
    {
        double t_ready = 5.0;
        if (TimeIsZero()) {
            Pelvis_SquatStartHeight = Pelvis_BaseHeight + 0.03;
            if(RorL) {
                _Xdes_CoM = Vector3d(0.0,LIGHT.Xref_RF2CoM(1),Pelvis_SquatStartHeight);
                _Xdes_RF2LF = Vector3d(-0.10,0.20,0.15);
                _Xini_temp2 = LIGHT.Xref_RF2CoM; // Original Posture
                _Xini_temp3 = LIGHT.Xref_RF2LF;
            } else {
                _Xdes_CoM = Vector3d(0.0,LIGHT.Xref_LF2CoM(1),Pelvis_SquatStartHeight);
                _Xdes_LF2RF = Vector3d(-0.10,-0.20,0.15);
                _Xini_temp2 = LIGHT.Xref_LF2CoM; // Original Posture
                _Xini_temp3 = LIGHT.Xref_LF2RF;
            }
            FILE_LOG(logWARNING) << " Squat Motion Ready... ";
        }

        bool FINISH;
        if(RorL) {
            FINISH = CoM_Move_RSSP(t_ready,
                                   I3,I3,RotY(20.0*D2R),
                                   _Xdes_CoM,zv,zv,
                                   _Xdes_RF2LF,zv,zv);
        } else {
            FINISH = CoM_Move_LSSP(t_ready,
                                   I3,RotY(20.0*D2R),I3,
                                   _Xdes_CoM,zv,zv,
                                   _Xdes_LF2RF,zv,zv);
        }

        if(FINISH) {
            Squat_CurrentStage = Squat_MOVE;
        }  return false; // not finished!
        break;
    }
    case Squat_MOVE:
    {
        if (TimeIsZero()) {
            if(RorL) {
                _Xini_temp = LIGHT.Xref_RF2CoM;
            } else {
                _Xini_temp = LIGHT.Xref_LF2CoM;
            }
            TimeLimitSet(_TIME*_NUM - 0.4*_TIME);
            FILE_LOG(logWARNING) << " Squat Motion Start!";
        }

        Vector3d _Xdes_CoM = _Xini_temp;
        Vector3d _dXdes_CoM = Vector3d(0.0, 0.0, 0.0);
        Vector3d _ddXdes_CoM = Vector3d(0.0, 0.0, 0.0);
        _Xdes_CoM(2) = _Xini_temp(2) - _HEIGHT/2.0*(1.0-cos(2.0*PI*TimeNow()/_TIME));
        _dXdes_CoM(2) = -_HEIGHT/2.0*(2.0*PI/_TIME)*sin(2.0*PI*TimeNow()/_TIME);
        _ddXdes_CoM(2) =  -_HEIGHT/2.0*(2.0*PI/_TIME)*(2.0*PI/_TIME)*cos(2.0*PI*TimeNow()/_TIME);

        if(RorL) {
            // =============== RF >> CoM Position Trajectory ===============
            Submotion_RF2CoM_Pos(SYS_DT_WALKING, _Xdes_CoM, _dXdes_CoM, _ddXdes_CoM);
        } else {
            // =============== LF >> CoM Position Trajectory ===============
            Submotion_LF2CoM_Pos(SYS_DT_WALKING, _Xdes_CoM, _dXdes_CoM, _ddXdes_CoM);
        }

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            Squat_CurrentStage = Squat_TERMINATE;
        } return false; // not finished!
        break;
    }
    case Squat_TERMINATE:
    {
        bool FINISH;
        if(RorL) {
            FINISH = CoM_Move_RSSP(_TIME,
                                   I3,I3,I3,
                                   _Xini_temp2,zv,zv,
                                   _Xini_temp3,zv,zv);
        } else {
            FINISH = CoM_Move_LSSP(_TIME,
                                   I3,I3,I3,
                                   _Xini_temp2,zv,zv,
                                   _Xini_temp3,zv,zv);
        }

        if(FINISH) {
            Squat_CurrentStage = Squat_READY;
            return true;
        } else {
            return false;
        }
        break;
    }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
////////// Deflected Pelvis Compensation + Swing Foot Compensation /////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


//Vector3d    Wfil_FootComp_Float = Vector3d::Zero();
//bool LIGHTWholeMotions::SwingFootComp_Float()
//{
//    Matrix3d _Rnow_Pel = LIGHT.Rnow_Pel;

//    double Kcomp = 1.0;
//    double wn = 2.0*PI*0.2;
//    double alpha_FootComp = 1.0/(1.0+SYS_FREQ/(2.0*PI*wn));
//    Vector3d Wnew_FootComp_Float = Kcomp*OrientationError(I3,_Rnow_Pel);
//    Wfil_FootComp_Float = alpha_FootComp*Wnew_FootComp_Float + (1.0-alpha_FootComp)*Wfil_FootComp_Float;

//    LIGHT.Wdes_Pel = -wn*Wfil_FootComp_Float;
//    LIGHT.Rdes_Pel = expMatrix(VectorCross2Matrix(Wfil_FootComp_Float));
//}

//Vector3d    Wfil_FootComp_RF = Vector3d::Zero();
//bool LIGHTWholeMotions::SwingFootComp_RF() {
//    Matrix3d _Rnow_Pel = LIGHT.Rnow_Pel;

//    double Kcomp = 0.7;
//    double wn = 2.0*PI*0.2;
//    double alpha_FootComp = 1.0/(1.0+SYS_FREQ/(2.0*PI*wn));
//    Vector3d Wnew_FootComp_RF = Kcomp*OrientationError(I3,_Rnow_Pel);
//    Wfil_FootComp_RF = alpha_FootComp*Wnew_FootComp_RF + (1.0-alpha_FootComp)*Wfil_FootComp_RF;

//    LIGHT.Wdes_Pel = wn*Wfil_FootComp_RF;
//    LIGHT.Rdes_Pel = expMatrix(VectorCross2Matrix(-Wfil_FootComp_RF));
//}

//bool LIGHTWholeMotions::SwingFootComp_LF() {

//}

double LIGHTWholeMotions::CoMHeightControl(double Damping, double Spring, double Integral)
{
    // Damping : Ns/m
    // Spring : N/m
    // Integral : N/ms
    static double e_sum = 0.0;
    if(Flag_CoMHeightCtrl) {
        double z,dz;
        double zd,dzd;
        if (LIGHT.Cur_RefFrame == LIGHT.REFFRAME_RF) {
            z = LIGHT.Xnow_Pel(2);
            dz = LIGHT.dXnow_Pel(2);
//            z = LIGHT.Xnow_RF2CoM(2);
//            dz = LIGHT.dXnow_RF2CoM(2);
            zd = LIGHT.Xdes_RF2CoM(2);
            dzd = LIGHT.dXdes_RF2CoM(2);
        }
        else if (LIGHT.Cur_RefFrame == LIGHT.REFFRAME_LF) {
            z = LIGHT.Xnow_Pel(2);
            dz = LIGHT.dXnow_Pel(2);
//            z = LIGHT.Xnow_LF2CoM(2);
//            dz = LIGHT.dXnow_LF2CoM(2);
            zd = LIGHT.Xdes_LF2CoM(2);
            dzd = LIGHT.dXdes_LF2CoM(2);
        }
        else {
            return 0.0;
        }

        double e = zd - z;
        double de = dzd - dz;
        e_sum += e*SYS_DT_WALKING;

        double Fz = Damping*de + Spring*e + Integral*e_sum;

        double Ka = 1.0/(Integral+1);
        if(Fz > 200.0) {
            double e_sum_adj = Ka*(Fz - 200.0);
            e_sum -= e_sum_adj;
            Fz = 200.0;
        } else if (Fz < -200.0) {
            double e_sum_adj = Ka*(Fz + 200.0);
            e_sum -= e_sum_adj;
            Fz = -200.0;
        }

        double t_decay = 5.0;
        e_sum = (1.0 - SYS_DT_WALKING/t_decay)*e_sum;
        return Fz;
    } else {
        return 0.0;
    }
}

