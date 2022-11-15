#ifndef OTYPES_H
#define OTYPES_H
#include "BasicMatrix.h"
enum QUADGAIT
{
    TrotWalk,
    PaceWalk,
    GallopWalk,
};
struct WalkParams
{
    vec3 step_L;
    double step_T;
    double step_Rot;
    double FB_L, LR_L;
    int Gait;
};
struct QuadJointVels
{
    double RHR,RHP,RKN;
    double LHR,LHP,LKN;
    double RSR,RSP,REB;
    double LSR,LSP,LEB;
    vec3 dpPel;
    vec3 dqPel;
};
struct QuadJoints
{
    double RHR,RHP,RKN;
    double LHR,LHP,LKN;
    double RSR,RSP,REB;
    double LSR,LSP,LEB;
    vec3 pPel;
    quat qPel;

};
struct WalkSensors
{
    QuadJoints JointPos;
    QuadJoints JointPosEnc;
    QuadJointVels JointVel;
    vec3 IMUangle;
    quat IMUquat;
    vec3 IMUomega;
    double RF_Fz, LF_Fz, RH_Fz, LH_Fz;
    double f_RF_Fz, f_LF_Fz, f_RH_Fz, f_LH_Fz;
    double Fs, f0,Q,w0,alpha,b0,b1,b2,a0,a1,a2;

    double x0[4],x1[4],x2[4];
    double y0[4],y1[4],y2[4];
    vec3 F_RF,F_LF,F_RH,F_LH,M_RF,M_LF,M_LH,M_RH;
    WalkSensors()
    {
        f0 = 30;
        Fs = 200;
        Q = 1;
        w0 = 2*M_PI*f0/Fs;
        alpha = sin(w0)/(2*Q);
        b0 = (1-cos(w0))/2;
        b1 = (1-cos(w0));
        b2 = (1-cos(w0))/2;
        a0 = 1+alpha;
        a1 = -2*cos(w0);
        a2 = 1-alpha;
        for(int i=0;i<4;i++)
        {
            x0[i] = x1[i] = x2[i] = y0[i] = y1[i] = y2[i] = 0;
        }
    }

    void do_filtering()
    {
        x0[0] = RF_Fz;
        x0[1] = LF_Fz;
        x0[2] = RH_Fz;
        x0[3] = LH_Fz;
        for(int i=0;i<4;i++)
        {
            y0[i] = (b0/a0)*x0[i] + (b1/a0)*x1[i] + (b2/a0)*x2[i]
                     - (a1/a0)*y1[i] - (a2/a0)*y2[i];
        }
        f_RF_Fz = y0[0];
        f_LF_Fz = y0[1];
        f_RH_Fz = y0[2];
        f_LH_Fz = y0[3];
        for(int i=0;i<4;i++)
        {
            y2[i] = y1[i];
            y1[i] = y0[i];
            x2[i] = x1[i];
            x1[i] = x0[i];
        }


    }
};


struct Moments
{
    vec3 Linear;
    vec3 Angular;
};
struct QuadPos
{
    vec3 pCOM, dCOM, ddCOM;
    vec3 CP;
    vec3 pPel;
    quat qPel;
    vec3 pRF,pLF,pRH,pLH;
    quat qRF,qLF,qRH,qLH;//read only
};
enum WALKING_STATE
{
    DSP = 1,
    UPR = 2,//FOOT up
    DWR = 4,//FOOT down
    BFU = 8,//FOOT BEFORESWING
    SWR = 16,//FOOT SWING
    AFD = 32//FOOT AFTERSWING
    //upswing, downsing can be possible
};


#endif // OTYPES_H
