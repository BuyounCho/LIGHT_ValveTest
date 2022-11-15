#ifndef OTYPES_H
#define OTYPES_H
#include "BasicMatrix.h"
#include <deque>
enum QUADGAIT
{
    Standing,
    Trot,
    Wave,
    Wave2,
    Pronk,
    Flytrot,
    Demo
};
struct WalkParams
{
    vec3 step_L;
    double step_T;
    double step_Rot;
    double FB_L, LR_L;    
    double delZ;
    int Gait;
    double dsp_ratio;
    double landing_depth;
    double overlap;//0to 1 ->0 complete wave 1 complete trot
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
struct JG
{
    double Kp_swing;
    double Kd_swing;
    double Kp_stand;
    double Kd_stand;
    JG()
    {
        Kp_swing = 20;
        Kd_swing = 0.6;
        Kp_stand = 10;
        Kd_stand = 0.3;
    }
};
struct JointGains
{
  JG HRG;
  JG HPG;
  JG KNG;
  JG KNGland;
  JointGains()
  {
      HRG.Kp_stand = 40;
      HRG.Kd_stand = 1.5;

      HPG.Kp_swing = 20;
      HPG.Kd_swing = 0.6;
      KNG.Kp_swing = 20;
      KNG.Kd_swing = 0.6;
      KNG.Kp_stand = 20;
      KNG.Kd_stand = 0.6;

      KNGland.Kp_swing = 10;
      KNGland.Kd_swing = 0.3;
  }
};
struct HPRL
{
    double alpha, oldLFy, oldHFy, RL,dt;
    bool first;
    void init(double _alpha, double _RL, double _dt)
    {
        first = true;
        alpha = _alpha;
        RL = _RL;
        dt = _dt;
    }
    double do_filt(double in)
    {
        double out;
        if(first)
        {
            oldLFy = in;
            oldHFy = 0.0;
            first = false;
        }
        double LFy = (1-alpha)*in + alpha*oldLFy;
        double HFy = in-LFy;
        if(HFy-oldHFy >  RL*dt){HFy = oldHFy+RL*dt;}
        if(HFy-oldHFy < -RL*dt){HFy = oldHFy-RL*dt;}
        out = LFy+HFy;
        oldLFy = LFy;
        oldHFy = HFy;
        return out;
    }
};

struct filt
{
    double Fs, f0,Q,w0,alpha,b0,b1,b2,a0,a1,a2;
    double x0,x1,x2;
    double y0,y1,y2;
    void set_lpf(double f,double _Q)
    {
        f0 = f;
        Fs = 500;
        Q = _Q;
        w0 = 2*M_PI*f0/Fs;
        alpha = sin(w0)/(2*Q);
        b0 = (1-cos(w0))/2;
        b1 = (1-cos(w0));
        b2 = (1-cos(w0))/2;
        a0 = 1+alpha;
        a1 = -2*cos(w0);
        a2 = 1-alpha;
        reset_filt(0,0);
    }
    void set_notch(double f, double _Q)
    {
        f0 = f;
        Fs = 500;
        Q = _Q;
        w0 = 2*M_PI*f0/Fs;
        alpha = sin(w0)/(2*Q);
        b0 = 1;
        b1 = -2*cos(w0);
        b2 = 1;
        a0 = 1+alpha;
        a1 = -2*cos(w0);
        a2 = 1-alpha;
        reset_filt(0,0);
    }
    filt()
    {
        f0 = 15;
        Fs = 500;
        Q = 1*3;
        w0 = 2*M_PI*f0/Fs;
        alpha = sin(w0)/(2*Q);
        b0 = (1-cos(w0))/2;
        b1 = (1-cos(w0));
        b2 = (1-cos(w0))/2;
        a0 = 1+alpha;
        a1 = -2*cos(w0);
        a2 = 1-alpha;
        reset_filt(0,0);
    }
    void reset_filt(double x, double y)
    {
        x1 = x2 = x0 = x;
        y1 = y2 = y0 = y;
    }
    double do_filt(double in)
    {
        x0 = in;
        y0 = (b0/a0)*x0 + (b1/a0)*x1 + (b2/a0)*x2
                 - (a1/a0)*y1 - (a2/a0)*y2;
        y2 = y1;
        y1 = y0;
        x2 = x1;
        x1 = x0;
        return y0;
    }
};
struct WalkSensors
{
    QuadJoints JointPos;    
    QuadJoints JointPosEnc;
    QuadJointVels JointVel;
    vec3 IMUangle;
    quat IMUquat;
    vec3 IMUomega, IMUomega_raw;
    double RF_Fz, LF_Fz, RH_Fz, LH_Fz;
    vec3 F_RF,F_LF,F_RH,F_LH,M_RF,M_LF,M_LH,M_RH;
    QuadJoints JointCurrent;
    QuadJoints JointPWM;
    bool BIGERR[12];
    bool ENCERR[12];
    bool CANERR[12];
    double looptime;
    vec3 IMUacc;
    int NO_RESPONSE_CNT;
    int NO_RESPONSE_CNT_COCOA[12];
    int QPcnt;
    WalkSensors()
    {       

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
    vec3 dqPel;
    vec3 dRF,dLF,dRH,dLH;
    vec3 ddqPel;
    vec3 ddRF,ddLF,ddRH,ddLH;
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

struct XYZxdxddx
{
  vec3 xdxddx;
  vec3 ydyddy;
  vec3 zdzddz;
  bool isContact;
  vec3 X()
  {
      return vec3(xdxddx[0],ydyddy[0],zdzddz[0]);
  }
  vec3 dX()
  {
      return vec3(xdxddx[1],ydyddy[1],zdzddz[1]);
  }
  vec3 ddX()
  {
      return vec3(xdxddx[2],ydyddy[2],zdzddz[2]);
  }
  void setX(vec3 X)
  {
      xdxddx[0] = X.x;
      ydyddy[0] = X.y;
      zdzddz[0] = X.z;
  }
  void setdX(vec3 dX)
  {
      xdxddx[1] = dX.x;
      ydyddy[1] = dX.y;
      zdzddz[1] = dX.z;
  }
  void setddX(vec3 ddX)
  {
      xdxddx[2] = ddX.x;
      ydyddy[2] = ddX.y;
      zdzddz[2] = ddX.z;
  }
  void dXddX0()
  {
      xdxddx[1] = xdxddx[2] = 0;
      ydyddy[1] = ydyddy[2] = 0;
      zdzddz[1] = zdzddz[2] = 0;
  }
};
struct FootContact
{
    int cnt;//cnt left. when 0, poped
    int upcnt;//if>0, goes up
    bool isStance;//if true, Stance, if false, Flying;
    double up_L;//up_L
};
struct GeneralWalkParams//jump?
{
    std::deque<WalkParams> WPs;//do not see Gait, Skipnum, up_L
    std::deque<int> WP_cnts;//cnt for WPs//if 0, pop WPs and WP_cnts
    std::deque<FootContact> CRF,CLF,CRH,CLH;
    bool finishing;

    double StanceRate;
    int Periodcnt;
    double phase_off[4];
    double upratio = 0.5;


    void pro_Cs(std::deque<FootContact> &Cin)
    {//call by reference must be better...
        if(!Cin.empty())
        {
            Cin.front().cnt--;
            Cin.front().upcnt--;
            if(Cin.front().cnt<=0)
            {
                Cin.pop_front();
            }
        }

    }
    void proceed()
    {
        if(!WP_cnts.empty())
        {
            WP_cnts.front()--;
            if(WP_cnts.front()<=0)
            {
                WP_cnts.pop_front();
                WPs.pop_front();
            }
        }
        pro_Cs(CRF);
        pro_Cs(CLF);
        pro_Cs(CRH);
        pro_Cs(CLH);
        finishing = CRF.empty()||CLF.empty()||CRH.empty()||CLH.empty();
        //if finishing, stop body and go to standing when foot is in contact

    }

};
#endif // OTYPES_H
