#include "LIGHT_motion.h"
#include "LIGHT_savedata.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;

extern LIGHTWholeBody LIGHT;
extern LIGHTWholeMotions LIGHT_WholeMotions;
extern INFO_LIGHT LIGHT_Info;

void Convert_CoMandCoP2ActuatorStates(bool RorL, Vector3d X_CoM, Vector3d dX_CoM, Vector3d X_ZMP, Vector3d F_push,
                                      VectorNd &dS_act, VectorNd &F_act)
{
    double x = X_CoM(0);
    double y = X_CoM(1);
    double z = X_CoM(2);
    double dx = dX_CoM(0);
    double dy = dX_CoM(1);
    double dz = dX_CoM(2);
    double px = X_ZMP(0);
    double py = X_ZMP(1);

    double S_hipr, dS_hipr, F_hipr;
    double S_hipp, dS_hipp, F_hipp;
    double S_knee, dS_knee, F_knee;
    VectorNd S_ank = VectorNd::Zero(2);
    VectorNd dS_ank = VectorNd::Zero(2);
    VectorNd F_ank = VectorNd::Zero(2);

    double x_ank = -0.00575;
    double z_ank = 0.073;

    double Lup = LIGHT_Info.offset_rhp2rkn.norm();
    double Llo = LIGHT_Info.offset_rkn2rap.norm();

    double L_pel,L_arm;
    if(RorL) { // Reference Foot : Right
        L_pel = -LIGHT_Info.offset_pel2rhr(1);
        L_arm = -LIGHT_Info.offset_rhy2rhp(1);
    } else {
        L_pel = -LIGHT_Info.offset_pel2lhr(1);
        L_arm = -LIGHT_Info.offset_lhy2lhp(1);
    }

    double L_leg = sqrt((x-x_ank)*(x-x_ank)+(z-z_ank)*(z-z_ank));

    // 1. CoM Position >> Leg length >> knee angle
    double theta_knee = acos((Lup*Lup+Llo*Llo-L_leg*L_leg)/(2.0*Lup*Llo));

    // 2. CoM Position >> ankle pitch and roll angles
    double theta_ankp = atan2(z-z_ank,x-x_ank)-acos((L_leg*L_leg+Llo*Llo-Lup*Lup)/(2.0*L_leg*Llo));
    double theta_ankr = atan2(y-L_pel,z-z_ank)-atan2(L_arm,L_leg);
    double theta_hipr = -theta_ankr;
    double theta_hipp = theta_knee - theta_ankp;

    // 3. CoM x,z Velocity >> hip pitch, knee and ankle pitch angular velocities
    double dtheta_hipp,dtheta_knee,dtheta_ankp;
    VectorNd temp_dtheta = VectorNd::Zero(2);
    VectorNd temp_dX = VectorNd::Zero(2);
    temp_dX << dx, dz;
    MatrixNd temp_J = MatrixNd::Zero(2,2);
    temp_J << Lup*sin(theta_knee-theta_ankp), -Llo*sin(theta_ankp)-Lup*sin(theta_knee-theta_ankp),
            Lup*cos(theta_knee-theta_ankp),  Llo*cos(theta_ankp)-Lup*cos(theta_knee-theta_ankp);
    temp_dtheta = temp_J.inverse()*temp_dX;

    dtheta_knee = temp_dtheta(0);
    dtheta_ankp = temp_dtheta(1);
    dtheta_hipp = dtheta_knee - dtheta_ankp;

    // 4. CoM y Velocity >> hip roll, ankle roll angular velocities
    double dtheta_ankr = dy/(L_leg*cos(theta_ankr)-L_arm*sin(theta_ankr));
    double dtheta_hipr = -dtheta_ankr;

    // 5. CoP Position >> ankle torques
    double T_ankp = F_push(2)*(px-x_ank);
    double T_ankr = -F_push(2)*py;

    // 6. CoP(ZMP) Position >> push force >> knee torque
    Vector3d r_loleg(Llo,0.0,0.0);
    r_loleg = RotX(theta_ankr).transpose()*RotY(theta_ankp).transpose()*r_loleg;
    Vector3d e_knee(0.0,cos(theta_ankr),-sin(theta_ankr));
    Vector3d F_push_proj = F_push-e_knee.dot(F_push)*e_knee;
    Vector3d rxF_knee = r_loleg.cross(F_push_proj);

    double T_knee = rxF_knee.norm()-T_ankp;

    // 7. hip roll angular velocity and torque
    LIGHT.Joint2Actuator_HipRoll(theta_hipr, dtheta_hipr, 0.0, S_hipr, dS_hipr, F_hipr);

    // 8. hip pitch actuator velocity and force
    double theta_hipp_offset = acos(LIGHT_Info.offset_rhp2rkn(0)/LIGHT_Info.offset_rhp2rkn.norm());
    LIGHT.Joint2Actuator_HipPitch(theta_hipp-theta_hipp_offset, theta_hipp-theta_hipp_offset, dtheta_hipp, 0.0, S_hipp, dS_hipp, F_hipp);

    // 9. knee actuator velocity and force
    double theta_knee_offset = acos(-LIGHT_Info.offset_rhp2rkn.dot(LIGHT_Info.offset_rkn2rap)/(Lup*Llo));
    LIGHT.Joint2Actuator_Knee(-(theta_knee-theta_knee_offset),-(theta_knee-theta_knee_offset),-dtheta_knee,-T_knee,
                              S_knee,dS_knee,F_knee);

    // 10. Ankle actuator velocity and force
    double theta_ankp_offset = 19.37*D2R;
    LIGHT.Joint2Actuator_Ankle(theta_ankp-theta_ankp_offset,theta_ankp-theta_ankp_offset,dtheta_ankp,T_ankp,
                               theta_ankr,theta_ankr,dtheta_ankr,T_ankr,
                               S_ank,dS_ank,F_ank);

//    if(RorL) { cout << " [Right Foot Reference] " << endl; }
//    else { cout << " [Left Foot Reference] " << endl; }

//    cout << "CoM : " << x << " " << y << endl;
//    cout << "CoP : " << px << " " << py << endl;
//    cout << "F_push : " << F_push.transpose() << endl;

//    cout << "theta_knee : " << -(theta_knee-theta_knee_offset)*R2D << endl;
//    cout << "theta_ankp : " << (theta_ankp-theta_ankp_offset)*R2D << endl;
//    cout << "theta_ankr : " << theta_ankr*R2D << endl;

//    cout << "T_knee : " << -T_knee << endl;
//    cout << "T_ankp : " << T_ankp << endl;
//    cout << "T_ankr : " << T_ankr << endl;

//    cout << "dS_hipr : " << dS_hipr << endl;
//    cout << "dS_hipp : " << dS_hipp << endl;
//    cout << "dS_knee : " << dS_knee << endl;
//    cout << "dS_ank(0) : " << dS_ank(0) << endl;
//    cout << "dS_ank(1) : " << dS_ank(1) << endl;

//    cout << "F_hipr : " << F_hipr << endl;
//    cout << "F_hipp : " << F_hipp << endl;
//    cout << "F_knee : " << F_knee << endl;
//    cout << "F_ank(0) : " << F_ank(0) << endl;
//    cout << "F_ank(1) : " << F_ank(1) << endl;

    dS_act(0) = dS_hipr;
    dS_act(1) = dS_hipp;
    dS_act(2) = dS_knee;
    dS_act(3) = dS_ank(0);
    dS_act(4) = dS_ank(1);
    F_act(0) = F_hipr;
    F_act(1) = F_hipp;
    F_act(2) = F_knee;
    F_act(3) = F_ank(0);
    F_act(4) = F_ank(1);
}

void LIGHTWholeMotions::Calc_FuturePressureReference(double _t_now, double _T_set, double _Ps_des)
{
    int _N_Prev = sharedREF->N_PrevPump;
    double _dT_Prev = sharedREF->dT_PrevPump;

    static double Ps_des_ini;
    if(_t_now <= SYS_DT_WALKING) {
        Ps_des_ini = sharedREF->RequiredPressureReference_Future[0];
    }

    double t_Prev = _t_now;
    for(int i=0;i<=_N_Prev;i++) {
        if(t_Prev<=_T_set) {
            if(Ps_des_ini <= _Ps_des) {
                sharedREF->RequiredPressureReference_Future[i] = (_Ps_des-Ps_des_ini)*(1.0-cos(t_Prev/_T_set*PI/2.0))+Ps_des_ini;
            } else {
                sharedREF->RequiredPressureReference_Future[i] = (_Ps_des-Ps_des_ini)*sin(t_Prev/_T_set*PI/2.0)+Ps_des_ini;
            }
        } else {
            sharedREF->RequiredPressureReference_Future[i] = _Ps_des;
        }
        t_Prev = t_Prev + _dT_Prev;
    }
}

bool LIGHTWholeMotions::Calc_FuturePressureReference_WithWalkPattern(int _N_Prev, double _dT_Prev,
                                                                     Vector3d *_CPref_horizon, Vector3d *_ZMPref_horizon, Vector3d *_CoMref_horizon)
{
    int n = GetFinalStep();
    int n_Prev = 0;

    int RefFrame_Prev = CurStepSupportPhase;

    double t_Prev = t_CurSTEP;
    for(int i=0;i<_N_Prev;i++) {
//        cout << "========= ("<< i <<")-th Preview Step ========"<< endl;
//        cout << "RefFrame_Prev : " << RefFrame_Prev << endl;
//        cout << "t_Prev : " << t_Prev << endl;

        VectorNd dS_act_R = VectorNd::Zero(5);
        VectorNd dS_act_L = VectorNd::Zero(5);
        VectorNd F_act_R = VectorNd::Zero(5);
        VectorNd F_act_L = VectorNd::Zero(5);

        if(((t_Prev <= SDB[n_Prev].t_STEP-SDB[n_Prev].t_DSP)&&(abs(RefFrame_Prev)==1))&&(n_Prev<=n)) { // Previewed step is SSP.
            Vector3d Step = SDB[n_Prev].X_STEP + SDB[n_Prev].X_STEP_adj;

            double x = _CoMref_horizon[i](0); // Stance Foot to CoM Position
            double y = _CoMref_horizon[i](1);
            double dx = wn_LIPM*(_CPref_horizon[i](0)-_CoMref_horizon[i](0));  // Stance Foot to CoM Velocity
            double dy = wn_LIPM*(_CPref_horizon[i](1)-_CoMref_horizon[i](1));
            double sx = Step(0)-(1.0-t_Prev/(SDB[n_Prev].t_STEP-SDB[n_Prev].t_DSP))*(SDB[n_Prev].V_DES(0)*(SDB[n_Prev].t_STEP-SDB[n_Prev].t_DSP)); // Stance Foot to Swing Foot Position
            double sy = Step(1)-(1.0-t_Prev/(SDB[n_Prev].t_STEP-SDB[n_Prev].t_DSP))*(SDB[n_Prev].V_DES(1)*(SDB[n_Prev].t_STEP-SDB[n_Prev].t_DSP));
            double sdx = SDB[n_Prev].V_DES(0)*2.0;  // Stance Foot to Swing Foot Velocity
            double sdy = SDB[n_Prev].V_DES(1)*2.0;

            double px = _ZMPref_horizon[i](0);
            double py = _ZMPref_horizon[i](1);

            double m_push = LIGHT_Info.m_robot-LIGHT_Info.m_rkn-LIGHT_Info.m_rap-LIGHT_Info.m_rar;
            Vector3d F_push(m_push*wn_LIPM*wn_LIPM*(x-px),
                            m_push*wn_LIPM*wn_LIPM*(y-py),
                            m_push*g_const);
            Vector3d F_swing(0.0,0.0,1.0);

            if(RefFrame_Prev>0) {
                // Right : Swing foot
                Vector3d X_CoM_R(x-sx, y-sy, Pelvis_BaseHeight);
                Vector3d dX_CoM_R(dx-sdx, dy-sdy, 0.0);
                Vector3d X_ZMP_R = zv;
                Convert_CoMandCoP2ActuatorStates(true, X_CoM_R, dX_CoM_R, X_ZMP_R, F_swing, dS_act_R, F_act_R);

                // Left : Stance foot
                Vector3d X_CoM_L(x, y, Pelvis_BaseHeight);
                Vector3d dX_CoM_L(dx, dy, 0.0);
                Vector3d X_ZMP_L(px, py, 0.0);
                Convert_CoMandCoP2ActuatorStates(false, X_CoM_L, dX_CoM_L, X_ZMP_L, F_push, dS_act_L, F_act_L);

            } else {
                // Right : Stance foot
                Vector3d X_CoM_R(x, y, Pelvis_BaseHeight);
                Vector3d dX_CoM_R(dx, dy, 0.0);
                Vector3d X_ZMP_R(px, py, 0.0);
                Convert_CoMandCoP2ActuatorStates(true, X_CoM_R, dX_CoM_R, X_ZMP_R, F_push, dS_act_R, F_act_R);

                // Left : Swing foot
                Vector3d X_CoM_L(x-sx, y-sy, Pelvis_BaseHeight);
                Vector3d dX_CoM_L(dx-sdx, dy-sdy, 0.0);
                Vector3d X_ZMP_L = zv;
                Convert_CoMandCoP2ActuatorStates(false, X_CoM_L, dX_CoM_L, X_ZMP_L, F_swing, dS_act_L, F_act_L);
            }

            t_Prev = t_Prev + _dT_Prev;
            if(t_Prev > SDB[n_Prev].t_STEP) {
                t_Prev = t_Prev - SDB[n_Prev].t_STEP;
                RefFrame_Prev = SDB[n_Prev].NextSupportPhase;
                for(int k=0;k<_N_Prev;k++) {
                    _CoMref_horizon[k] = RotZ(SDB[n_Prev].Yaw_STEP).transpose()*(_CoMref_horizon[k]-(SDB[n_Prev].X_STEP+SDB[n_Prev].X_STEP_adj));
                    _CPref_horizon[k] = RotZ(SDB[n_Prev].Yaw_STEP).transpose()*(_CPref_horizon[k]-(SDB[n_Prev].X_STEP+SDB[n_Prev].X_STEP_adj));
                    _ZMPref_horizon[k] = RotZ(SDB[n_Prev].Yaw_STEP).transpose()*(_ZMPref_horizon[k]-(SDB[n_Prev].X_STEP+SDB[n_Prev].X_STEP_adj));
                }
                n_Prev++;
            }
        } else if (((t_Prev <= SDB[n_Prev].t_STEP)||(abs(RefFrame_Prev)==2))&&(n_Prev<=n)) { // Previewed step is DSP.
            Vector3d Step = SDB[n_Prev].X_STEP + SDB[n_Prev].X_STEP_adj;

            if(RefFrame_Prev<0) {

                double x_R = _CoMref_horizon[i](0); // Stance Foot to CoM Position
                double y_R = _CoMref_horizon[i](1);
                double dx_R = wn_LIPM*(_CPref_horizon[i](0)-_CoMref_horizon[i](0)); // Stance Foot to CoM Velocity
                double dy_R = wn_LIPM*(_CPref_horizon[i](1)-_CoMref_horizon[i](1));

                double x_L = x_R - Step(0); // Swing Foot to CoM Position
                double y_L = y_R - Step(1);
                double dx_L = dx_R; // Swing Foot to CoM Velocity
                double dy_L = dy_R;

                double px = _ZMPref_horizon[i](0);
                double py = _ZMPref_horizon[i](1);

                double m_push = LIGHT_Info.m_robot-LIGHT_Info.m_rkn-LIGHT_Info.m_rap-LIGHT_Info.m_rar
                        -LIGHT_Info.m_lkn-LIGHT_Info.m_lap-LIGHT_Info.m_lar;
                Vector3d F_push(m_push*wn_LIPM*wn_LIPM*(x_R-px),
                                m_push*wn_LIPM*wn_LIPM*(y_R-py),
                                m_push*g_const);

                Vector3d ZMP(px,py,0.0);
                double alpha = Step.dot(ZMP)/Step.dot(Step);

                double px_R, px_L, py_R, py_L;
                Vector3d F_push_R, F_push_L;
                if(alpha>=0.0 && alpha<=1.0) {
                    Vector3d ZMP_Offset = ZMP - alpha*Step;
                    px_R = ZMP_Offset(0);
                    py_R = ZMP_Offset(1);
                    px_L = px_R + Step(0);
                    py_L = py_R + Step(1);
                    F_push_R = (1.0-alpha)*F_push;
                    F_push_L = alpha*F_push;
                } else if(alpha<0.0) {
                    px_R = px;
                    py_R = py;
                    px_L = px_R + Step(0);
                    py_L = py_R + Step(1);
                    F_push_R = F_push;
                    F_push_L = zv;
                } else {
                    px_L = px;
                    py_L = py;
                    px_R = px_L - Step(0);
                    py_R = py_L - Step(1);
                    F_push_L = F_push;
                    F_push_R = zv;
                }

                Vector3d X_CoM_R(x_R, y_R, Pelvis_BaseHeight);
                Vector3d dX_CoM_R(dx_R, dy_R, 0.0);
                Vector3d X_ZMP_R(px_R, py_R, 0.0);
                Convert_CoMandCoP2ActuatorStates(true, X_CoM_R, dX_CoM_R, X_ZMP_R, F_push_R, dS_act_R, F_act_R);

                Vector3d X_CoM_L(x_L, y_L, Pelvis_BaseHeight);
                Vector3d dX_CoM_L(dx_L, dy_L, 0.0);
                Vector3d X_ZMP_L(px_L, py_L, 0.0);
                Convert_CoMandCoP2ActuatorStates(false, X_CoM_L, dX_CoM_L, X_ZMP_L-Step, F_push_L, dS_act_L, F_act_L);

            } else if(RefFrame_Prev>0) {

                double x_L = _CoMref_horizon[i](0); // Stance Foot to CoM Position
                double y_L = _CoMref_horizon[i](1);
                double dx_L = wn_LIPM*(_CPref_horizon[i](0)-_CoMref_horizon[i](0)); // Stance Foot to CoM Velocity
                double dy_L = wn_LIPM*(_CPref_horizon[i](1)-_CoMref_horizon[i](1));

                double x_R = x_L - Step(0); // Swing Foot to CoM Position
                double y_R = y_L - Step(1);
                double dx_R = dx_L; // Swing Foot to CoM Velocity
                double dy_R = dy_L;

                double px = _ZMPref_horizon[i](0);
                double py = _ZMPref_horizon[i](1);

                double m_push = LIGHT_Info.m_robot-LIGHT_Info.m_rkn-LIGHT_Info.m_rap-LIGHT_Info.m_rar
                        -LIGHT_Info.m_lkn-LIGHT_Info.m_lap-LIGHT_Info.m_lar;
                Vector3d F_push(m_push*wn_LIPM*wn_LIPM*(x_L-px),
                                m_push*wn_LIPM*wn_LIPM*(y_L-py),
                                m_push*g_const);

                Vector3d ZMP(px,py,0.0);
                double alpha = Step.dot(ZMP)/Step.dot(Step);

                double px_R, px_L, py_R, py_L;
                Vector3d F_push_R, F_push_L;
                if(alpha>=0.0 && alpha<=1.0) {
                    Vector3d ZMP_Offset = ZMP - alpha*Step;
                    px_L = ZMP_Offset(0);
                    py_L = ZMP_Offset(1);
                    px_R = px_L + Step(0);
                    py_R = py_L + Step(1);
                    F_push_L = (1.0-alpha)*F_push;
                    F_push_R = alpha*F_push;
                } else if(alpha<0.0) {
                    px_L = px;
                    py_L = py;
                    px_R = px_L + Step(0);
                    py_R = py_L + Step(1);
                    F_push_L = F_push;
                    F_push_R = zv;
                } else {
                    px_R = px;
                    py_R = py;
                    px_L = px_R - Step(0);
                    py_L = py_R - Step(1);
                    F_push_R = F_push;
                    F_push_L = zv;
                }

                Vector3d X_CoM_R(x_R, y_R, Pelvis_BaseHeight);
                Vector3d dX_CoM_R(dx_R, dy_R, 0.0);
                Vector3d X_ZMP_R(px_R, py_R, 0.0);
                Convert_CoMandCoP2ActuatorStates(true, X_CoM_R, dX_CoM_R, X_ZMP_R-Step, F_push_R, dS_act_R, F_act_R);

                Vector3d X_CoM_L(x_L, y_L, Pelvis_BaseHeight);
                Vector3d dX_CoM_L(dx_L, dy_L, 0.0);
                Vector3d X_ZMP_L(px_L, py_L, 0.0);
                Convert_CoMandCoP2ActuatorStates(false, X_CoM_L, dX_CoM_L, X_ZMP_L, F_push_L, dS_act_L, F_act_L);

            } else {
                FILE_LOG(logERROR) << "Stance Type is not defined!!";
                FILE_LOG(logERROR) << "Stance Type is not defined!!";
                FILE_LOG(logERROR) << "Stance Type is not defined!!";
            }

            t_Prev = t_Prev + _dT_Prev;
            if(t_Prev > SDB[n_Prev].t_STEP) {
                t_Prev = t_Prev - SDB[n_Prev].t_STEP;
                RefFrame_Prev = SDB[n_Prev].NextSupportPhase;
                for(int k=0;k<_N_Prev;k++) {
                    _CoMref_horizon[k] = RotZ(SDB[n_Prev].Yaw_STEP).transpose()*(_CoMref_horizon[k]-(SDB[n_Prev].X_STEP+SDB[n_Prev].X_STEP_adj));
                    _CPref_horizon[k] = RotZ(SDB[n_Prev].Yaw_STEP).transpose()*(_CPref_horizon[k]-(SDB[n_Prev].X_STEP+SDB[n_Prev].X_STEP_adj));
                    _ZMPref_horizon[k] = RotZ(SDB[n_Prev].Yaw_STEP).transpose()*(_ZMPref_horizon[k]-(SDB[n_Prev].X_STEP+SDB[n_Prev].X_STEP_adj));
                }
                n_Prev++;
            }
        } else {
            Vector3d Step = -RotZ(SDB[n].Yaw_STEP)*(SDB[n].X_STEP + SDB[n].X_STEP_adj);
            if(RefFrame_Prev<0) {

                double x_R = _CoMref_horizon[i](0); // Stance Foot to CoM Position
                double y_R = _CoMref_horizon[i](1);
                double dx_R = wn_LIPM*(_CPref_horizon[i](0)-_CoMref_horizon[i](0)); // Stance Foot to CoM Velocity
                double dy_R = wn_LIPM*(_CPref_horizon[i](1)-_CoMref_horizon[i](1));

                double x_L = x_R - Step(0); // Swing Foot to CoM Position
                double y_L = y_R - Step(1);
                double dx_L = dx_R; // Swing Foot to CoM Velocity
                double dy_L = dy_R;

                double px = _ZMPref_horizon[i](0);
                double py = _ZMPref_horizon[i](1);

                double m_push = LIGHT_Info.m_robot-LIGHT_Info.m_rkn-LIGHT_Info.m_rap-LIGHT_Info.m_rar
                        -LIGHT_Info.m_lkn-LIGHT_Info.m_lap-LIGHT_Info.m_lar;
                Vector3d F_push(m_push*wn_LIPM*wn_LIPM*(x_R-px),
                                m_push*wn_LIPM*wn_LIPM*(y_R-py),
                                m_push*g_const);

                Vector3d ZMP(px,py,0.0);
                double alpha = Step.dot(ZMP)/Step.dot(Step);

                double px_R, px_L, py_R, py_L;
                Vector3d F_push_R, F_push_L;
                if(alpha>=0.0 && alpha<=1.0) {
                    Vector3d ZMP_Offset = ZMP - alpha*Step;
                    px_R = ZMP_Offset(0);
                    py_R = ZMP_Offset(1);
                    px_L = px_R + Step(0);
                    py_L = py_R + Step(1);
                    F_push_R = (1.0-alpha)*F_push;
                    F_push_L = alpha*F_push;
                } else if(alpha<0.0) {
                    px_R = px;
                    py_R = py;
                    px_L = px_R + Step(0);
                    py_L = py_R + Step(1);
                    F_push_R = F_push;
                    F_push_L = zv;
                } else {
                    px_L = px;
                    py_L = py;
                    px_R = px_L - Step(0);
                    py_R = py_L - Step(1);
                    F_push_L = F_push;
                    F_push_R = zv;
                }

                Vector3d X_CoM_R(x_R, y_R, Pelvis_BaseHeight);
                Vector3d dX_CoM_R(dx_R, dy_R, 0.0);
                Vector3d X_ZMP_R(px_R, py_R, 0.0);
                Convert_CoMandCoP2ActuatorStates(true, X_CoM_R, dX_CoM_R, X_ZMP_R, F_push_R, dS_act_R, F_act_R);

                Vector3d X_CoM_L(x_L, y_L, Pelvis_BaseHeight);
                Vector3d dX_CoM_L(dx_L, dy_L, 0.0);
                Vector3d X_ZMP_L(px_L, py_L, 0.0);
                Convert_CoMandCoP2ActuatorStates(false, X_CoM_L, dX_CoM_L, X_ZMP_L-Step, F_push_L, dS_act_L, F_act_L);

            } else if(RefFrame_Prev>0) {

                double x_L = _CoMref_horizon[i](0); // Stance Foot to CoM Position
                double y_L = _CoMref_horizon[i](1);
                double dx_L = wn_LIPM*(_CPref_horizon[i](0)-_CoMref_horizon[i](0)); // Stance Foot to CoM Velocity
                double dy_L = wn_LIPM*(_CPref_horizon[i](1)-_CoMref_horizon[i](1));

                double x_R = x_L - Step(0); // Swing Foot to CoM Position
                double y_R = y_L - Step(1);
                double dx_R = dx_L; // Swing Foot to CoM Velocity
                double dy_R = dy_L;

                double px = _ZMPref_horizon[i](0);
                double py = _ZMPref_horizon[i](1);

                double m_push = LIGHT_Info.m_robot-LIGHT_Info.m_rkn-LIGHT_Info.m_rap-LIGHT_Info.m_rar
                        -LIGHT_Info.m_lkn-LIGHT_Info.m_lap-LIGHT_Info.m_lar;
                Vector3d F_push(m_push*wn_LIPM*wn_LIPM*(x_L-px),
                                m_push*wn_LIPM*wn_LIPM*(y_L-py),
                                m_push*g_const);

                Vector3d ZMP(px,py,0.0);
                double alpha = Step.dot(ZMP)/Step.dot(Step);

                double px_R, px_L, py_R, py_L;
                Vector3d F_push_R, F_push_L;
                if(alpha>=0.0 && alpha<=1.0) {
                    Vector3d ZMP_Offset = ZMP - alpha*Step;
                    px_L = ZMP_Offset(0);
                    py_L = ZMP_Offset(1);
                    px_R = px_L + Step(0);
                    py_R = py_L + Step(1);
                    F_push_L = (1.0-alpha)*F_push;
                    F_push_R = alpha*F_push;
                } else if(alpha<0.0) {
                    px_L = px;
                    py_L = py;
                    px_R = px_L + Step(0);
                    py_R = py_L + Step(1);
                    F_push_L = F_push;
                    F_push_R = zv;
                } else {
                    px_R = px;
                    py_R = py;
                    px_L = px_R - Step(0);
                    py_L = py_R - Step(1);
                    F_push_R = F_push;
                    F_push_L = zv;
                }

                Vector3d X_CoM_R(x_R, y_R, Pelvis_BaseHeight);
                Vector3d dX_CoM_R(dx_R, dy_R, 0.0);
                Vector3d X_ZMP_R(px_R, py_R, 0.0);
                Convert_CoMandCoP2ActuatorStates(true, X_CoM_R, dX_CoM_R, X_ZMP_R-Step, F_push_R, dS_act_R, F_act_R);

                Vector3d X_CoM_L(x_L, y_L, Pelvis_BaseHeight);
                Vector3d dX_CoM_L(dx_L, dy_L, 0.0);
                Vector3d X_ZMP_L(px_L, py_L, 0.0);
                Convert_CoMandCoP2ActuatorStates(false, X_CoM_L, dX_CoM_L, X_ZMP_L, F_push_L, dS_act_L, F_act_L);

            } else {
                FILE_LOG(logDEBUG) << "Stance Type is not defined!!";
            }
        }

        // Actuator States >> Supply Pressure and Required Flowrate
        VectorNd dS_prev = VectorNd::Zero(12);
        VectorNd F_prev = VectorNd::Zero(12);
        dS_prev.segment(1,5) = dS_act_R;
        dS_prev.segment(7,5) = dS_act_L;
        F_prev.segment(1,5) = F_act_R;
        F_prev.segment(7,5) = F_act_L;

        for(int j=0;j<12;j++)
        {
            double alpha3 = pow(LIGHT.LIGHT_Info->Sa(j)/LIGHT.LIGHT_Info->Sb(j),3.0);
            double MM2MpStoLPM = 0.06;
            double Kv_max = 7.0/sqrt(100.0/2.0); // LPM/sqrt(bar)
            if(dS_prev(j)>1e-3) {
                sharedREF->LoadPressureReference_Future[i][j] = F_prev(j)/LIGHT.LIGHT_Info->Sa(j)*10.0;// + (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(j)*dS_prev(j)*MM2MpStoLPM)/Kv_max;
                sharedREF->ActFlowrateReference_Future[i][j] = LIGHT.LIGHT_Info->Sa(j)*dS_prev(j)*6.0/100.0;// + (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(j)*dS_prev(j)*MM2MpStoLPM)/Kv_max;
            } else if(dS_prev(j)<-1e-3) {
                sharedREF->LoadPressureReference_Future[i][j] = -F_prev(j)/LIGHT.LIGHT_Info->Sb(j)*10.0;// + (alpha3+1.0)*(LIGHT_Info->Sb(j)*dS_prev(j)*MM2MpStoLPM)/Kv_max;
                sharedREF->ActFlowrateReference_Future[i][j] = -LIGHT.LIGHT_Info->Sb(j)*dS_prev(j)*6.0/100.0;// + (alpha3+1.0)/alpha3*(LIGHT_Info->Sa(j)*dS_prev(j)*MM2MpStoLPM)/Kv_max;
            } else {
                if(F_prev(j)>=0.0) {
                    sharedREF->LoadPressureReference_Future[i][j] = F_prev(j)/LIGHT.LIGHT_Info->Sa(j)*10.0;
                    sharedREF->ActFlowrateReference_Future[i][j] = LIGHT.LIGHT_Info->Sa(j)*dS_prev(j)*6.0/100.0;
                } else {
                    sharedREF->LoadPressureReference_Future[i][j] = -F_prev(j)/LIGHT.LIGHT_Info->Sb(j)*10.0;
                    sharedREF->ActFlowrateReference_Future[i][j] = -LIGHT.LIGHT_Info->Sb(j)*dS_prev(j)*6.0/100.0;
                }
            }
        }
    }
}

void LIGHTWholeMotions::Load_FuturePumpReference_FullTaskScenario()
{
    FILE* pFile = fopen ( "DataLog/PumpOperationReference_FullScenarioTask.txt" , "r" );
    if (pFile==NULL) {fputs ("File error",stderr); exit (1);}

    // obtain file size:
    int ch;
    unsigned int number_of_lines = 0;
    while (EOF != (ch=getc(pFile)))
        if ('\n' == ch) ++number_of_lines;
    fseek(pFile, 0, SEEK_SET);

    Ps_Future_FullTaskScenario.resize(number_of_lines);
    Qp_Future_FullTaskScenario.resize(number_of_lines);

    // copy the file into the buffer:
    for(int i=0;i<number_of_lines;i++) {
        float temp1,temp2,temp3;
        fscanf(pFile, "%f %f %f \n", &temp1, &temp2, &temp3);

        double Ps_min = 30.0;
        if(temp2>Ps_min) Ps_Future_FullTaskScenario(i) = temp2;
        else Ps_Future_FullTaskScenario(i) = Ps_min;

        Qp_Future_FullTaskScenario(i) = temp3;
    }

    // terminate
    fclose (pFile);

    IsLoaded_FuturePumpReference_FullTaskScenario = true;
    return;
}
