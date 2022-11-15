#ifndef _OW_QUAD
#define _OW_QUAD
#include "Oinverse.h"
#include "OW_RBDL.h"
#include "BasicFiles/BasicSetting.h"
//#include "joint_inverse.h"
#include "joint_inverse_SW.h"
#include "Gaits/ow_pvddx.h"
#include "Otypes.h"
#include "ow_onedofmpc.h"
extern pRBCORE_SHM_COMMAND     sharedCMD;

class OW_Quad
{

    Oinverse Oi;
    OW_PVDDX OP;
public:
    QuadPos QP,QP_QP;
    QuadPos QP_controled;
    //QuadPos conQP;
    QuadPos QP_real;
    QuadPos QP_ENC_PLANE;
    QuadJoints QJ_CRef,QJ_CFF, QJ_CFILTERED;
    WalkParams GetWP(){return WP;}
    GeneralWalkParams GWP;
private:
    ONELEGFKIK Oii;
    QuadJoints QJ;
    WalkParams WP;
    WalkParams WP_next;
    bool walkchanged;
    bool Hcontact,Fcontact;

    WalkSensors WS;
    int Qcnt,Rcnt;
    double t_now;
    bool isRHLFmove,oisRHLFmove;
    bool isRmove,oisRmove;//for Pace
    int SWphase;//for slowwave
    double upL;
    double before_up_ratio;
    double after_down_ratio;
    double upRatio, flyRatio;
    vec3 oldpRFx, oldpLFx, oldpRFy, oldpLFy, oldpFzH,oldpFzF, oldRotZ, oldRotX,oldRotY;
    vec3 oldpRHx, oldpLHx, oldpRHy, oldpLHy;
    //added in flytrot
    vec3 oldpRHz, oldpLHz, oldpRFz, oldpLFz;

    vec3 FootTogoRF,FootTogoLF,FootTogoRH, FootTogoLH;
    double FootTogoAngle;
    bool isFirststep,isStopping, isFinishing;
    bool calc3th = false;
    VectorNd F3thx,F3thy;
    vec3 sumCOMerr;
public:
    double dt,g,e,w;
    int stopcnt;
    vec3 COM_old;
    double FC_FB, FC_LR;
    vec3 desCOM, estCOM;
    vec3 stepLFilter[5];
    vec3 COMr,dCOMr,ddCOMr;
    double ff,ll;
    int cTT;
    double Hlandoff;
    double Flandoff;

private:
    vec3 CP_nn,CP, COM_nn, CP_nn_nocon;
    vec3 CP_est, CP_est_nn;
    vec3 ZMP, ZMPr;
    vec3 Next_ZMP_offset;
    quat qPel_stepstart;
    bool RFlanded, LFlanded, RHlanded,LHlanded;

    double landing_fthres;
    double noncontact_fthres;
    double FootZctFT[4],FootZctRef[4],dFootZctRef[4],FootZctFT2[4];
public:
    bool FootonAir[4];
private:
    int FootLandCnt[4];
    int LandCnt;
    vec3 ct,ct_ff;
    OW_DYNAMICS OD,ODQP,ODJUMP;

    bool RFcontact,LFcontact,RHcontact,LHcontact,firstCon;
    bool RFlateland,LFlateland,RHlateland,LHlateland;
    double foot_xlim, foot_ylim;
    double RFx_ref, LFx_ref, RHx_ref, LHx_ref;
    double RFy_ref, LFy_ref, RHy_ref, LHy_ref;
    double RFz_ref, LFz_ref, RHz_ref, LHz_ref;

    double trackingDelayTime;
    MatrixNd Jqref;
    VectorNd dXref;

    double dmcol_RHP,dmcol_RKN;
    bool conRHP,conRKN;
    double alphamref[NO_OF_JOINTS];
    double mrefs0[NO_OF_JOINTS];
    double mrefs[NO_OF_JOINTS];
    double dmrefs[NO_OF_JOINTS];
    double ddmrefs[NO_OF_JOINTS];
    double ms[NO_OF_JOINTS], dms[NO_OF_JOINTS], Ains[NO_OF_JOINTS], Ains_xyz[NO_OF_JOINTS];
    double fdms[NO_OF_JOINTS];
    double ms_old[NO_OF_JOINTS], dms_old[NO_OF_JOINTS];
    double conFx,conFy;


    vec3 cpe, dcpe;

    vec3 last_pCOM, oldpPel, last_dCOM, last_dpPel, dpCOM;
    int cnt_fly;
    bool firstCon2;
    VectorNd TTF, oTTF;
    vec3 sL;
    HPRL cpef[3];
    HPRL dcpef[3];
    HPRL dmf[12];
    vec3 fcpe;
    vec3 fdcpe;


public:
    VectorNd Qnow, dQnow, ddQref, Qref, dQref, ddQnow, Tauout,Fcon;
    VectorNd QP_Qnow, QP_dQnow, QP_ddQref;
    VectorNd QP_Qnow_save, QP_dQnow_save;
    VectorNd QP_Qref, QP_dQref, QP_ddQnow, QP_Tauout,QP_Fcon;
    bool changetoPcon[4];
    bool changetoCcon[4];
    bool changetoPconKnee[4];
    bool changetoCconKnee[4];
    bool isPcon[4];
    bool Mchanged;
    bool Mch[4];
    vec3 COMref,dCOMref,ddCOMref;
    int ffcnt,ffcntMax;
    double Fzdes, colV;
    double offkv;
    vec3 oldCOMx,oldCOMy,oldCOMz;
    vec3 ddCOMLIPM,ddCOMFBDY;
    double contact_L;
    std::deque<vec3> ZMPrs;
public:
    bool isWalking;
    bool isStandingControl;
    bool isCOMadjusting;
    bool isCOMadjusting_finish;

    double delZ,des_delZ;
    QuadPos QP_est;
    vec3 decomp_A, decomp_W, decomp_A_ref, decomp_I, rotE_I;
   // vec3 drift_in_step_est,drift_in_step_est_f;
    vec3 decomp_W_filtered,IMUomega_filtered;
    vec3 tx,ty,tz;
    bool saveflag;
    int savenum;
    double savestepT;
    bool oldRHLFmove,oldisRmove;
    filt LPFs[6];
    bool isfalldown;
    double dYawcon, ddYawcon;
    bool NO_ROBOT_TEST;
    bool DO_ADJUST_SLOPE;
    bool DO_FOOTZ_DIRECTION;
    bool DO_DDCOMCON_WHILE_PREVIEW;
    QuadJoints adjust_FootZs(QuadJoints QJin, bool fric = true, double CC = 8);

    void set_des_delZ_slope();
    //variables for adjust_FootZs_direction

    int cnt_FZ[4];
    void fallcheck();
    void calc_decomp();
    void calc_decomp_pace();



    double fComp(double dmnow, double dmMaxF, double fricA);

    void init_params();
    OW_Quad()    
    {
        init_params();
    }
    void COM_ZERO();
    void init_Quad(QuadJoints _QJ, WalkParams _WP);

    void start_standing_control(int _cTT = 0);
    void stop_standing_control();
    void start_Walking();
    void stop_Walking();
    void changeWalk(WalkParams _WP);

    QuadPos state_est_real(QuadPos QP_before);
    QuadPos state_est2();
    QuadPos fk_dQ(QuadJoints _QJ, QuadJointVels _dQJ);
    void calc_ddQref(vec3 _ddpCOMref, vec3 ddqPELref, vec3 ddpRFref, vec3 ddpLFref, vec3 ddpRHref, vec3 ddpLHref);
    void calc_dQref(vec3 _dpCOMref, vec3 dqPELref, vec3 dpRFref, vec3 dpLFref, vec3 dpRHref, vec3 dpLHref);
    void calc_QPddQref(vec3 _ddpCOMref, vec3 ddqPELref, vec3 ddpRFref, vec3 ddpLFref, vec3 ddpRHref, vec3 ddpLHref);
    void calc_QPdQref(vec3 _dpCOMref, vec3 dqPELref, vec3 dpRFref, vec3 dpLFref, vec3 dpRHref, vec3 dpLHref);
    QuadPos state_est();

    MatrixNd pinv(MatrixNd in);
    double FFzref_alpha = 0.2;//0.995;
    void FFz_ref_ssp();
    void FFz_ref();
    void FFz_ref_3con(int noncon);
    vec3 dCOM_nn, oCOM_nn;
    //void control_joints(double Kp_swing = 20, double Kd_swing = 1, double Kp_stand= 10, double Kd_stand = 1);
    void control_joints(JointGains Gs);
    void control_joints_jump(JointGains Gs);
    void test_function();
    QuadJoints StandingControl_onestep(WalkSensors _WS);



    vec3 oldF1,oldF2,F1,F2;//foot fosition for plane estimation;
    vec3 abd,abd_f, plane_rpy, plane_rpy_f;
    vec3 rpyQP;
    void estimate_plane();
    void calc_rpy();
    double Zfromplane(double x, double y);

    void DSP_control();

    void COMADJUST_control();
    void SSP_control();
    void WAVE_control(int noncon);

    void STAND_control();
    void PRONK_control();
    void FLYTROT_control();
    void AIR_control();
    void DEMO_control();



    void MOVE_LEGS_TROT();
    void MOVE_LEGS_WAVE();
    void MOVE_LEGS_WAVE2();
    void MOVE_LEGS_FLYTROT();
    //fpr slope adjust
    void calc_decomp_slope();
    void MOVE_LEGS_TROT_SLOPE();
    void DSP_control_slope();
    void SSP_control_slope();



    void MOVE_LEGS_WAVE_SLOPE();
    void MOVE_LEGS_WAVE2_SLOPE();
    void WAVE_control_slope(int noncon);
    void SLOWWAVE_control_slope(int noncon);
    //later tbi
    void DSP_control_slow_slope();
    void SSP_control_slow_slope();


    QuadJoints COMadjust_onestep(WalkSensors _WS);

    QuadJoints Trot_onestep(WalkSensors _WS);
    QuadJoints Standing_onestep(WalkSensors _WS);
    QuadJoints Wave_onestep(WalkSensors _WS);
    QuadJoints Wave2_onestep(WalkSensors _WS);
    QuadJoints Pronk_onestep(WalkSensors _WS);
    QuadJoints Flytrot_onestep(WalkSensors _WS);
    QuadJoints Demo_onestep(WalkSensors _WS);
    double calcF(double Ft,double t, double T);

    int HOPPHASE,lastHOPPHASE;
    XYZxdxddx RFref, LFref, RHref, LHref, COMrefXdXddX, Angleref;
    double dcomz,ddcomz,ddcomzland;
    double dcomx,ddcomx;
    double dcomy,ddcomy;
    int lastWave;


    int time2Wphase(double _tnow);
    int time2Wphase_slowwave(double _tnow);
    int time2Wphase_wave(double _tnow);
    double step_T_each,t_uf,t_f2h,t_uh, t_wait;//t_wait on front and back of the swing
    int WS_NOW;
    const static int SAVEMAX = 750;
    const static int SAVEMAXCNT = 100000;
    double SAVE[SAVEMAX][SAVEMAXCNT];
    bool overcnt;
    void save_onestep(int cnt);
    inline double stept2off(double stept, double _w, double stepw)//is it right?
    {
        return stepw/(2+pow(e,_w*stept));
    }
    bool NOSAVE;
    void do_save_all();
    void save_all(int fnum = 0);
    VectorNd calc_3th(double t_0, double t_e,vec3 h_0,vec3 h_e);
    VectorNd calc_5th(double t_0, double t_e,vec3 h_0,vec3 h_e);
    VectorNd calc_5th_ZMP(double t_0, double t_e,vec3 h_0,vec3 h_e);
    VectorNd calc_5th_acc_free(double t_0, double t_e,vec3 h_0,vec3 h_e);
    XYZxdxddx calc_nextRef(double t_now,double t_e, XYZxdxddx ref_now, vec3 ref_end);

    void calc_ff_tau_ddq_QP();
    bool QPsolved;
    vec3 QP_ddCOM;
    vec3 opPel;
    bool QPFcon[4];
    int QPcnt;
    double RollQP[4];
    double dRollQP[4];
    filt ddCOMnotch[2];


    double upV, flyT, swT;
    void test_MPC();
};

#endif // OW_QUAD
