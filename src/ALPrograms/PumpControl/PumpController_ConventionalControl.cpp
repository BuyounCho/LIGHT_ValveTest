#include "PumpController_BasicFunctions.h"
#include "PumpControl_ControlFunctions.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
//extern pUSER_SHM                userData;

extern double wp_ref_max;
extern double wp_ref_min;
extern double Ps_des[MAX_PREVIEW+1];

double Qflow_ref = 0.0;
double Qflow_ref_JointVel = 0.0;
double Qflow_ref_Leakage = 0.0;
double Qflow_ref_Accumulator = 0.0;
double Qflow_ref_Feedback = 0.0;

double wp_ref_LPF = 0.0;

double P_error = 0.0;
double P_error_sum = 0.0;


void PumpFlowControl_Active_LIGHT2(double Ps_now, double Ps_ref, double dPs_ref, double &u_out)
{
    Qflow_ref = 0.0;

    // Get Estimated total flowrate with reference joint velocity =======================//

    const double M3pStoLPM = 60000.0;

    // LIGHT2 Model
    double Area_A[NO_OF_JOINTS];
    Area_A[RHR] = 9595.0*1e-9; //rotary, rA = m^3
    Area_A[RHY] = 5376.0*1e-9; //rotary, rA = m^3
    Area_A[RHP] = 236.0*1e-6; //linear, A = m^2
    Area_A[RKN] = 491.0*1e-6; //linear, A = m^2
    Area_A[RAP] = 177.0*1e-6; //linear, A = m^2
    Area_A[RAR] = 177.0*1e-6; //linear, A = m^2
    Area_A[LHR] = 9595.0*1e-9; //rotary, rA = m^3
    Area_A[LHY] = 5376.0*1e-9; //rotary, rA = m^3
    Area_A[LHP] = 236.0*1e-6; //linear, A = m^2
    Area_A[LKN] = 491.0*1e-6; //linear, A = m^2
    Area_A[LAP] = 177.0*1e-6; //linear, A = m^2
    Area_A[LAR] = 177.0*1e-6; //linear, A = m^2
    Area_A[WST] = 5376.0*1e-9; //rotary, rA = m^3

    double Area_B[NO_OF_JOINTS];
    Area_B[RHR] = 9595.0*1e-9; //rotary, rA = m^3
    Area_B[RHY] = 5376.0*1e-9; //rotary, rA = m^3
    Area_B[RHP] = 236.0*1e-6; //linear, A = m^2
    Area_B[RKN] = 236.0*1e-6; //linear, A = m^2
    Area_B[RAP] = 98.0*1e-6; //linear, A = m^2
    Area_B[RAR] = 98.0*1e-6; //linear, A = m^2
    Area_B[LHR] = 9595.0*1e-9; //rotary, rA = m^3
    Area_B[LHY] = 5376.0*1e-9; //rotary, rA = m^3
    Area_B[LHP] = 236.0*1e-6; //linear, A = m^2
    Area_B[LKN] = 236.0*1e-6; //linear, A = m^2
    Area_B[LAP] = 98.0*1e-6; //linear, A = m^2
    Area_B[LAR] = 98.0*1e-6; //linear, A = m^2
    Area_B[WST] = 5376.0*1e-9; //rotary, rA = m^3

    double JointVel[NO_OF_JOINTS];
    JointVel[RHR] = sharedSEN->ENCODER[RHR][0].CurrentRefActVel * 3.1415/180; // rad/s
    JointVel[RHY] = sharedSEN->ENCODER[RHY][0].CurrentRefActVel * 3.1415/180; // rad/s
    JointVel[RHP] = sharedSEN->ENCODER[RHP][0].CurrentRefActVel * 0.001; // m/s
    JointVel[RKN] = sharedSEN->ENCODER[RKN][0].CurrentRefActVel * 0.001; // m/s
    JointVel[RAP] = sharedSEN->ENCODER[RAP][0].CurrentRefActVel * 0.001; // m/s
    JointVel[RAR] = sharedSEN->ENCODER[RAR][0].CurrentRefActVel * 0.001; // m/s
    JointVel[LHR] = sharedSEN->ENCODER[LHR][0].CurrentRefActVel * 3.1415/180; // rad/s
    JointVel[LHY] = sharedSEN->ENCODER[LHY][0].CurrentRefActVel * 3.1415/180; // rad/s
    JointVel[LHP] = sharedSEN->ENCODER[LHP][0].CurrentRefActVel * 0.001; // m/s
    JointVel[LKN] = sharedSEN->ENCODER[LKN][0].CurrentRefActVel * 0.001; // m/s
    JointVel[LAP] = sharedSEN->ENCODER[LAP][0].CurrentRefActVel * 0.001; // m/s
    JointVel[LAR] = sharedSEN->ENCODER[LAR][0].CurrentRefActVel * 0.001; // m/s
    JointVel[WST] = sharedSEN->ENCODER[WST][0].CurrentRefActVel * 3.1415/180; // rad/s

    Qflow_ref_JointVel = 0.0;
    static double JointVel_LPF[12] = {0.0,};
    for(int i=0;i<NO_OF_JOINTS;i++) {
        double alpha_update_vel = 1.0/(1.0+SYS_FREQ_PUMPING/(2.0*3.1415*20.0));
        JointVel_LPF[i] = (1.0-alpha_update_vel)*JointVel_LPF[i] + alpha_update_vel*JointVel[i];
        if(JointVel_LPF[i]>0.0) {
            Qflow_ref_JointVel += fabs(Area_A[i] * M3pStoLPM * JointVel_LPF[i]);
        } else {
            Qflow_ref_JointVel += fabs(Area_B[i] * M3pStoLPM * JointVel_LPF[i]);
        }
    }

    // Leakage Compensation ==============================================================//
    double n_act = 13.0;
    Qflow_ref_Leakage = n_act*K_leak*(Ps_ref); // [LPM], Moog Valve : 13EA

    // Supply pressure feedback control + Accumulator Model ==================================================//
    Qflow_ref_Accumulator = 0.0; // [LPM]
    Qflow_ref_Feedback = 0.0; // [LPM]

    P_error = Ps_ref-Ps_now;
    P_error_sum = P_error_sum + P_error*SYS_DT_PUMPING;

    double Kp = 0.05, Ki = 0.05;
    if(Ps_des[0] > P_ModeChange) {
        double dP_star = dPs_ref + (2.0*3.1415*5.0)*P_error;
        Qflow_ref_Accumulator = dP_star*n_gas*pow((P_pre/(Ps_ref+1.0)),1/n_gas)*(V_pre/(Ps_ref+1.0));
        Qflow_ref_Feedback = Kp*P_error + Ki*P_error_sum;
    } else {
        Qflow_ref_Accumulator = 0.0;
        Qflow_ref_Feedback = Kp*P_error + Ki*P_error_sum;
    }

    // Reference Duty Summation ==================================================//
    Qflow_ref = Qflow_ref_JointVel
                + Qflow_ref_Leakage
                + Qflow_ref_Feedback
                + Qflow_ref_Accumulator;
//    Qflow_ref = Qflow_ref_JointVel + Qflow_ref_Leakage;

    double K_comp = 1.1;
    double wp_des = K_comp*Qflow_ref/OutputFlowPerRev;

    double alpha_PumpSpeed = 1.0/(1.0+SYS_FREQ_PUMPING/(2.0*3.141592*5.0));
    wp_ref_LPF=(1.0-alpha_PumpSpeed)*wp_ref_LPF+alpha_PumpSpeed*wp_des;


    // Duty Saturation & Anti-windup for feedback =================================//
    if(fabs(Ki)>1e-5) {
        double Ka = 1.0/(Ki+0.001);
        if(wp_ref_LPF > wp_ref_max) {
            P_error_sum = P_error_sum - Ka*(wp_ref_LPF-wp_ref_max)*OutputFlowPerRev*SYS_DT_PUMPING;
            wp_ref_LPF = wp_ref_max;
        } else if(wp_ref_LPF < wp_ref_min) {
            P_error_sum = P_error_sum - Ka*(wp_ref_LPF-wp_ref_min)*OutputFlowPerRev*SYS_DT_PUMPING;
            wp_ref_LPF = wp_ref_min;
        }
        double alpha_decay = 1.0/(1.0+SYS_FREQ_PUMPING/(2.0*3.141592*0.3));
        P_error_sum =(1.0-alpha_decay)*P_error_sum+alpha_decay*0.0;
    } else {
        if (wp_ref_LPF > wp_ref_max) wp_ref_LPF = wp_ref_max;
        if (wp_ref_LPF < wp_ref_min) wp_ref_LPF = wp_ref_min;
    }

    u_out = wp_ref_LPF;

//    FILE_LOG(logDEBUG) << "Qflow_ref_JointVel : " << Qflow_ref_JointVel;
//    FILE_LOG(logDEBUG) << "Qflow_ref_Leakage : " << Qflow_ref_Leakage;
//    FILE_LOG(logDEBUG) << "Qflow_ref_Accumulator : " << Qflow_ref_Accumulator;
//    FILE_LOG(logDEBUG) << "Qflow_ref_Feedback : " << Qflow_ref_Feedback;
//    FILE_LOG(logDEBUG) << "wp_des : " << wp_des;

//    FILE_LOG(logDEBUG) << "===================================";
}

void PumpFlowControl_Active_QuadTest(double Ps_now, char PsRef_type, double &u_out)
{

    // future supply pressure reference =============================================//
    double Ps_ref,dPs_ref;
    if(PsRef_type == 0) { // Ps_ref : From This AL
        Ps_ref = Ps_des[0];
        if(Ps_ref < Ps_min) {
            Ps_ref = Ps_min;
        } else if (Ps_ref > Ps_max) {
            Ps_ref = Ps_max;
        }

        double Ps_next = Ps_des[1];
        if(Ps_next < Ps_min) {
            Ps_next = Ps_min;
        } else if (Ps_next > Ps_max) {
            Ps_next = Ps_max;
        }
        dPs_ref = (Ps_next-Ps_ref)/sharedREF->dT_PrevPump;
    } else if (PsRef_type == 1) { // Ps_ref : From Shared Memory
        double Ps_temp = 1000.0; // [bar]
        for(int j=0;j<MAX_VC;j++) {
            double Pl = sharedREF->LoadPressureReference_Future[0][j];
            if(Ps_temp > Pl) {
                Ps_temp = Pl;
            }
        }
        if(Ps_temp < Ps_min) {
            Ps_temp = Ps_min;
        } else if (Ps_temp > Ps_max) {
            Ps_temp = Ps_max;
        }
        Ps_ref = Ps_temp;

        Ps_temp = 1000.0; // [bar]
        for(int j=0;j<MAX_VC;j++) {
            double Pl = sharedREF->LoadPressureReference_Future[1][j];
            if(Ps_temp > Pl) {
                Ps_temp = Pl;
            }
        }
        if(Ps_temp < Ps_min) {
            Ps_temp = Ps_min;
        } else if (Ps_temp > Ps_max) {
            Ps_temp = Ps_max;
        }
        dPs_ref = (Ps_temp-Ps_ref)/sharedREF->dT_PrevPump;
    }

    Qflow_ref = 0.0;

    // Get Estimated total flowrate with reference joint velocity =======================//
    Qflow_ref_JointVel = 0.0;
    const double M3pStoLPM = 60000.0;

    // Quad Model
    double Area_HipYaw      = 0.01875*0.0085*0.016*2; //rotary, rA = m^3
    double Area_HipRoll     = 0.02525*0.0095*0.020*2; //rotary, rA = m^3
    double Area_HipPitch    = 0.02525*0.0095*0.020*2; //linear, A = m^2
    double Area_Knee_A      = 0.0;
    double Area_Knee_B      = 0.0;
    double Area_Ankle_A     = 0.0;
    double Area_Ankle_B     = 0.0;

    double JointVel[12];
    JointVel[RHY] = sharedSEN->ENCODER[RHY][0].CurrentActVel * 3.1415/180; // rad/s
    JointVel[RHR] = sharedSEN->ENCODER[RHR][0].CurrentActVel * 3.1415/180; // rad/s
    JointVel[RHP] = sharedSEN->ENCODER[RHP][0].CurrentActVel * 3.1415/180; // rad/s
    JointVel[RKN] = 0.0;
    JointVel[RAP] = 0.0;
    JointVel[RAR] = 0.0;
    JointVel[LHY] = 0.0;
    JointVel[LHR] = 0.0;
    JointVel[LHP] = 0.0;
    JointVel[LKN] = 0.0;
    JointVel[LAP] = 0.0;
    JointVel[LAR] = 0.0;

    static double JointVel_LPF[12] = {0.0,};
    for(int i=0;i<12;i++) {
        double alpha_update_vel = 1.0/(1.0+SYS_FREQ_PUMPING/(2.0*3.1415*20.0));
        JointVel_LPF[i] = (1.0-alpha_update_vel)*JointVel_LPF[i] + alpha_update_vel*JointVel[i];
    }

    Qflow_ref_JointVel += fabs(Area_HipYaw  * M3pStoLPM * JointVel_LPF[RHY]);
    Qflow_ref_JointVel += fabs(Area_HipRoll  * M3pStoLPM * JointVel_LPF[RHR]);
    Qflow_ref_JointVel += fabs(Area_HipPitch * M3pStoLPM * JointVel_LPF[RHP]);

    if(JointVel_LPF[RKN]>0.0) Qflow_ref_JointVel += fabs(Area_Knee_A * M3pStoLPM * JointVel_LPF[RKN]);
    else  Qflow_ref_JointVel += fabs(Area_Knee_B * M3pStoLPM * JointVel_LPF[RKN]);
    if(JointVel_LPF[RAP]>0.0) Qflow_ref_JointVel += fabs(Area_Ankle_A * M3pStoLPM * JointVel_LPF[RAP]);
    else  Qflow_ref_JointVel += fabs(Area_Ankle_B * M3pStoLPM * JointVel_LPF[RAP]);
    if(JointVel_LPF[RAR]>0.0) Qflow_ref_JointVel += fabs(Area_Ankle_A * M3pStoLPM * JointVel_LPF[RAR]);
    else  Qflow_ref_JointVel += fabs(Area_Ankle_B * M3pStoLPM * JointVel_LPF[RAR]);

    Qflow_ref_JointVel += fabs(Area_HipYaw * M3pStoLPM * JointVel_LPF[LHY]);
    Qflow_ref_JointVel += fabs(Area_HipRoll * M3pStoLPM * JointVel_LPF[LHR]);
    Qflow_ref_JointVel += fabs(Area_HipPitch * M3pStoLPM * JointVel_LPF[LHP]);

    if(JointVel_LPF[LKN]>0.0) Qflow_ref_JointVel += fabs(Area_Knee_A * M3pStoLPM * JointVel_LPF[LKN]);
    else  Qflow_ref_JointVel += fabs(Area_Knee_B * M3pStoLPM * JointVel_LPF[LKN]);
    if(JointVel_LPF[LAP]>0.0) Qflow_ref_JointVel += fabs(Area_Ankle_A * M3pStoLPM * JointVel_LPF[LAP]);
    else  Qflow_ref_JointVel += fabs(Area_Ankle_B * M3pStoLPM * JointVel_LPF[LAP]);
    if(JointVel_LPF[LAR]>0.0) Qflow_ref_JointVel += fabs(Area_Ankle_A * M3pStoLPM * JointVel_LPF[LAR]);
    else  Qflow_ref_JointVel += fabs(Area_Ankle_B * M3pStoLPM * JointVel_LPF[LAR]);

    if(Ps_des[0] > P_ModeChange) {
        Qflow_ref_JointVel = 1.0*Qflow_ref_JointVel;
    } else {
        Qflow_ref_JointVel = 0.7*Qflow_ref_JointVel;
    }

    // Leakage Compensation ==============================================================//
    double n_valve = 3.0;
    Qflow_ref_Leakage = n_valve*K_leak*(Ps_ref); // [LPM], Moog Valve : 1EA
                                                 // 100bar >> 0.20LPM

    // Supply pressure feedback control + Accumulator Model ==================================================//
    Qflow_ref_Accumulator = 0.0; // [LPM]
    Qflow_ref_Feedback = 0.0; // [LPM]

    P_error = Ps_ref-Ps_now;
    P_error_sum = P_error_sum + P_error*SYS_DT_PUMPING;

    double Kp = 0.40, Ki = 0.0;
    if(Ps_des[0] > P_ModeChange) {
        double dP_star = dPs_ref + 1.0*P_error;
        Qflow_ref_Accumulator = dP_star*pow((P_pre/(Ps_ref+1.0)),1/n_gas)*(V_pre/n_gas/(Ps_ref+1.0))*60.0; // 60 : sec/min

        Kp = 0.40; // Pressure error gain, 40bar error > 10.0LPM
        Ki = 0.0;
        Qflow_ref_Feedback = Kp*P_error + Ki*P_error_sum;
    } else {
        Qflow_ref_Accumulator = 0.0;

        Kp = 0.40; // Pressure error gain, 200bar error > 10.0LPM
//        Ki = 0.10;
        Qflow_ref_Feedback = Kp*P_error + Ki*P_error_sum;
    }

    // Reference Duty Summation ==================================================//
    Qflow_ref = Qflow_ref_JointVel
                + Qflow_ref_Leakage
                + Qflow_ref_Feedback
                + Qflow_ref_Accumulator;

    double wp_des = Qflow_ref/OutputFlowPerRev;

    double alpha_PumpSpeed = 1.0/(1.0+SYS_FREQ_PUMPING/(2.0*3.141592*30.0));
    wp_ref_LPF=(1.0-alpha_PumpSpeed)*wp_ref_LPF+alpha_PumpSpeed*wp_des;

    // Duty Saturation & Anti-windup for feedback =================================//
    if(Ki!=0.0) {
        double Ka = 2.0/(Ki+0.001);
        if(wp_ref_LPF > wp_ref_max) {
            P_error_sum = P_error_sum - Ka*(wp_ref_LPF-wp_ref_max)*OutputFlowPerRev;
//            wp_ref_LPF = wp_ref_max;
        } else if(wp_ref_LPF < wp_ref_min) {
            P_error_sum = P_error_sum - Ka*(wp_ref_LPF-wp_ref_min)*OutputFlowPerRev;
//            wp_ref_LPF = wp_ref_min;
        }
        double alpha_decay = 1.0/(1.0+SYS_FREQ_PUMPING/(2.0*3.141592*0.3));
        P_error_sum =(1.0-alpha_decay)*P_error_sum+alpha_decay*0.0;
    } else {
//        if (wp_ref_LPF > wp_ref_max) wp_ref_LPF = wp_ref_max;
//        if (wp_ref_LPF < wp_ref_min) wp_ref_LPF = wp_ref_min;
    }

    u_out = wp_ref_LPF;
}
