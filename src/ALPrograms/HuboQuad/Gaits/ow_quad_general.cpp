#include "ow_quad.h"
#include <ctime>
void OW_Quad::init_Quad(QuadJoints _QJ, WalkParams _WP)
{
    QJ = _QJ;
    // QJ.qPel = quat(vec3(0,0,1),0.3);
    oldRotZ = vec3(0,0,0);
    oldRotX = vec3(0,0,0);
    oldRotY = vec3(0,0,0);
    qPel_stepstart = QJ.qPel;
    QP = Oi.FK(QJ);
    QP_controled = QP;
    QP_real = QP;
    COM_ZERO();
    QP_est = QP;
    COM_old = QP.pCOM;
    desCOM = QP.pCOM;
    COMr = QP.pCOM;
    dCOMr = vec3(0,0,0);
    FC_FB = FC_LR = 0;

    WP = _WP;
    Qcnt = 0;
    Rcnt = 0;
    t_now = 0;
    des_delZ = WP.delZ;
    delZ = QP.pCOM.z-QP.pRF.z;
    w = sqrtp(g/delZ);
    isRHLFmove = true;
    isRmove = true;
    isFirststep = true;
    isFinishing = false;
    isStandingControl = false;
    for(int i=0;i<4;i++)
    {
        FootZctFT[i] = 0.0;
        FootZctRef[i] = 0.0;
        dFootZctRef[i] = 0.0;
        FootonAir[i] = false;
        FootLandCnt[i] = 0;
        changetoPcon[i] = false;
        changetoCcon[i] = false;
        changetoPconKnee[i] = false;
        changetoCconKnee[i] = false;
        isPcon[i] = true;
    }
    LFlanded = RFlanded = LHlanded = RHlanded = false;

    Qnow = VectorNd::Zero(OD.Robot->q_size);
    dQnow = VectorNd::Zero(OD.Robot->dof_count);
    dQref = VectorNd::Zero(OD.Robot->dof_count);
    Qref = VectorNd::Zero(OD.Robot->q_size);
    QP_Qnow = VectorNd::Zero(OD.Robot->q_size);
    QP_dQnow = VectorNd::Zero(OD.Robot->dof_count);
    QP_Qnow_save = VectorNd::Zero(OD.Robot->q_size);
    QP_dQnow_save = VectorNd::Zero(OD.Robot->dof_count);
    QP_dQref = VectorNd::Zero(OD.Robot->dof_count);
    QP_Qref = VectorNd::Zero(OD.Robot->q_size);
    TTF = dQnow;
    oTTF = TTF;
    ddQref = VectorNd::Zero(OD.Robot->dof_count);
    ddQnow = VectorNd::Zero(OD.Robot->dof_count);
    Tauout = VectorNd::Zero(12);
    QP_ddQref = VectorNd::Zero(OD.Robot->dof_count);
    QP_ddQnow = VectorNd::Zero(OD.Robot->dof_count);
    QP_Tauout = VectorNd::Zero(12);
    RFcontact=LFcontact=RHcontact=LHcontact=firstCon = firstCon2 = true;
    RFlateland=LFlateland=RHlateland=LHlateland =false;

    RFx_ref = LFx_ref = RHx_ref = LHx_ref =0;
    RFy_ref = LFy_ref = RHy_ref = LHy_ref =0;
    RFz_ref = LFz_ref = RHz_ref = LHz_ref =0;
    for(int i=0;i<NO_OF_JOINTS;i++)
    {

        ms[i] = -1000;
        dms[i] = -1000;
        ms_old[i] = -1000;
        dms_old[i] = -1000;
        alphamref[i] = 1;
    }
    for(int i=0;i<4;i++)
    {
        Mch[i] = true;
    }
    Mchanged = true;
    ff = ll = 0;
    HOPPHASE = 0;
    lastHOPPHASE = 2;
    RFref.setX(QP.pRF);
    RFref.dXddX0();
    LFref.setX(QP.pLF);
    LFref.dXddX0();
    RHref.setX(QP.pRH);
    RHref.dXddX0();
    LHref.setX(QP.pLH);
    LHref.dXddX0();
    dcomz = ddcomz = ddcomzland = 0;
    calc3th = false;
    ddYawcon = dYawcon = 0;
    lastWave = 100;
    ZMPrs.clear();
    overcnt = false;
    sumCOMerr = vec3();
    for(int i=0;i<5;i++)
    {        
        stepLFilter[i] = vec3();
    }
    SWphase = 0;//anyway start from 0
    DO_DDCOMCON_WHILE_PREVIEW = true;

    rotE_I = vec3();
    estCOM = QP.pCOM;
    for(int i=0;i<3;i++)
    {
        cpef[i].init(0.8,0.02,0.002);
        dcpef[i].init(0.8,20,0.002);
    }
    for(int i=0;i<12;i++)
    {
        dmf[i].init(0.8,1,0.002);
    }
    QPsolved = false;
    QPcnt = 0;
    ddCOMnotch[0].set_notch(46.8,0.3);
    ddCOMnotch[1].set_notch(46.8,0.3);
}
void OW_Quad::fallcheck()
{
    vec3 rpyQP = Oi.Decomp_Rmat(QP.qPel,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));
    if(fabs(WS.IMUangle.x-rpyQP[2])*R2Df>15){printf("Roll fall!n\n"); isWalking = false; save_all(2);}
    if(fabs(WS.IMUangle.y-rpyQP[1])*R2Df>15){printf("Pitch fall!n\n"); isWalking = false; save_all(2);}
    //for(int i=0;i<12;i++)
    for(int i=3;i<12;i++)//0,1,2 too many times
    {
        if(WS.BIGERR[i]==true){printf("BIGERR! %d\n",i); isWalking = false; save_all(2);}
        if(WS.ENCERR[i]==true){printf("ENCERR! %d\n",i); isWalking = false; save_all(2);}
        if(WS.CANERR[i]==true){printf("CANERR! %d\n",i); isWalking = false; save_all(2);}
    }
    if(WS.NO_RESPONSE_CNT>200&&stopcnt>2)
    {
        printf("NO_RESPONSE!\n");
        stop_Walking();
    }
    for(int i=0;i<12;i++)
    {
        if(WS.NO_RESPONSE_CNT_COCOA[i]>20)
        {
            printf("NO_RESPONSE_ENC!\n");
            isWalking = false; save_all(2);
        }
    }
    if(WS.QPcnt>10)
    {
        printf("NO_RESPONSE_QP!\n");
        isWalking = false; save_all(2);
    }
}

QuadJoints OW_Quad::adjust_FootZs(QuadJoints QJin,bool fric, double CC)
//have to change. just fit y only to change roll
{
    if(NO_ROBOT_TEST){return QJin;}//do_nothing

     QuadJoints QJt = QJin;
     QuadJoints QJE = WS.JointPosEnc;


    double Cthres = CC;//2.2222  is ratio
    double FC = 2.0;
    double mdiffthres = 0.02;
    if(fric)
    {
        if(QJ_CRef.RKN+QJ_CFF.RKN>Cthres&&FootonAir[0]==false)//standing
        {
            if(fComp(mrefs[RKN]-ms[RKN],mdiffthres,FC)>0)
            {
                QJ_CRef.RKN+=fComp(mrefs[RKN]-ms[RKN],mdiffthres,FC);
            }
        }
        if(QJ_CRef.LKN+QJ_CFF.LKN>Cthres&&FootonAir[1]==false)//standing
        {
            if(fComp(mrefs[LKN]-ms[LKN],mdiffthres,FC)>0)
            {
                QJ_CRef.LKN+=fComp(mrefs[LKN]-ms[LKN],mdiffthres,FC);
            }
        }
//        if(QJ_CRef.REB+QJ_CFF.REB>Cthres&&FootonAir[2]==false)//standing
//        {
//            //if(fComp(mrefs[REB]-ms[REB],mdiffthres,FC)>0)//front//////////
//            {
//                QJ_CRef.REB+=fComp(mrefs[REB]-ms[REB],mdiffthres,FC);
//            }
//        }
//        if(QJ_CRef.LEB+QJ_CFF.LEB>Cthres&&FootonAir[3]==false)//standing
//        {
//            //if(fComp(mrefs[LEB]-ms[LEB],mdiffthres,FC)>0)//front//////////
//            {
//                QJ_CRef.LEB+=fComp(mrefs[LEB]-ms[LEB],mdiffthres,FC);
//            }
//        }
    }

    return QJt;
    //return QJin;//just return.

}
void OW_Quad::init_params()
{
    isStandingControl = false;
    isWalking = false;
    walkchanged = false;
    upL = 0.08;//later 10cm up    
    //upL = 0.1;
    before_up_ratio = 0.05;
    after_down_ratio = 0.05;
    upRatio = 0.4;
    flyRatio = 0.3;//0.3;
    dt = 0.002;
    g = 9.81;
    e = M_E;
    landing_fthres = 100*60;//300//no detection
    noncontact_fthres = 50;
    foot_xlim = foot_ylim = 0.23;//
    trackingDelayTime = 0.024;
    Jqref = MatrixNd::Zero(18,18);
    dXref = VectorNd::Zero(18);
    saveflag = false;
    isfalldown = false;
    dYawcon = 0;
    LPFs[4].set_lpf(50,3);
    DO_DDCOMCON_WHILE_PREVIEW = true;//alwayse do
    LandCnt = 0.1/dt;
}
void OW_Quad::FFz_ref_ssp()//2foot standing
{
    double Mg =(Oi.M_total)*g;//to be always early landing
    if(DO_ADJUST_SLOPE)
    {
        Mg = (Oi.M_total)*(g-QP.ddCOM.x*tan(plane_rpy_f.y)+QP.ddCOM.y*tan(plane_rpy_f.x));
    }
    if(isRHLFmove)
    {
        double of = (ZMP-QP.pLH).norm();
        double ob = (ZMP-QP.pRF).norm();

        LHz_ref = FFzref_alpha*LHz_ref + (1-FFzref_alpha)*Mg*(of)/(of+ob);
        RFz_ref = FFzref_alpha*RFz_ref + (1-FFzref_alpha)*Mg*(ob)/(of+ob);

        RHz_ref = 0;
        LFz_ref = 0;
    }
    else
    {
        double of = (ZMP-QP.pRH).norm();
        double ob = (ZMP-QP.pLF).norm();

        RHz_ref = FFzref_alpha*RHz_ref + (1-FFzref_alpha)*Mg*(of)/(of+ob);
        LFz_ref = FFzref_alpha*LFz_ref + (1-FFzref_alpha)*Mg*(ob)/(of+ob);

        LHz_ref = 0;
        RFz_ref = 0;
    }
}
void OW_Quad::FFz_ref_3con(int noncon)
{
    MatrixNd A;
    A = MatrixNd::Zero(3,3);
    MatrixNd B;
    B = MatrixNd::Zero(3,1);
    double Mg =(Oi.M_total)*g;
    if(DO_ADJUST_SLOPE)
    {
        Mg = (Oi.M_total)*(g-QP.ddCOM.x*tan(plane_rpy_f.y)+QP.ddCOM.y*tan(plane_rpy_f.x));
    }
    double x1,x2,x3;
    double y1,y2,y3;
    double f1,f2,f3;
    if(noncon==0)
    {
        x1 = QP.pLF.x; x2 = QP.pRH.x; x3 = QP.pLH.x;
        y1 = QP.pLF.y; y2 = QP.pRH.y; y3 = QP.pLH.y;
    }
    if(noncon==1)
    {
        x1 = QP.pRF.x; x2 = QP.pRH.x; x3 = QP.pLH.x;
        y1 = QP.pRF.y; y2 = QP.pRH.y; y3 = QP.pLH.y;
    }
    if(noncon==2)
    {
        x1 = QP.pRF.x; x2 = QP.pLF.x; x3 = QP.pLH.x;
        y1 = QP.pRF.y; y2 = QP.pLF.y; y3 = QP.pLH.y;
    }
    if(noncon==3)
    {
        x1 = QP.pRF.x; x2 = QP.pLF.x; x3 = QP.pRH.x;
        y1 = QP.pRF.y; y2 = QP.pLF.y; y3 = QP.pRH.y;
    }


    A(0,0) = 1;
    A(0,1) = 1;
    A(0,2) = 1;
    A(1,0) = x1;
    A(1,1) = x2;
    A(1,2) = x3;
    A(2,0) = y1;
    A(2,1) = y2;
    A(2,2) = y3;
    B(0,0) = Mg;
    B(1,0) = Mg*ZMP.x;
    B(2,0) = Mg*ZMP.y;

    MatrixNd C = (A.inverse())*B;//pseudoinverse


    f1 = C(0,0);
    f2 = C(1,0);
    f3 = C(2,0);

    if(noncon==0){RFz_ref = 0;  LFz_ref = f1; RHz_ref = f2; LHz_ref = f3;}
    if(noncon==1){RFz_ref = f1; LFz_ref = 0;  RHz_ref = f2; LHz_ref = f3;}
    if(noncon==2){RFz_ref = f1; LFz_ref = f2; RHz_ref = 0;  LHz_ref = f3;}
    if(noncon==3){RFz_ref = f1; LFz_ref = f2; RHz_ref = f3; LHz_ref = 0; }


}
void OW_Quad::FFz_ref()//3~4foot standing//3foot?
{
    MatrixNd A;
    A = MatrixNd::Zero(3,4);
    MatrixNd B;
    B = MatrixNd::Zero(3,1);
    double Mg =(Oi.M_total)*g;
    if(DO_ADJUST_SLOPE)
    {
        Mg = (Oi.M_total)*(g-QP.ddCOM.x*tan(plane_rpy_f.y)+QP.ddCOM.y*tan(plane_rpy_f.x));
    }

    A(0,0) = 1;
    A(0,1) = 1;
    A(0,2) = 1;
    A(0,3) = 1;
    A(1,0) = QP.pRF.x;
    A(1,1) = QP.pLF.x;
    A(1,2) = QP.pRH.x;
    A(1,3) = QP.pLH.x;
    A(2,0) = QP.pRF.y;
    A(2,1) = QP.pLF.y;
    A(2,2) = QP.pRH.y;
    A(2,3) = QP.pLH.y;
    B(0,0) = Mg;
    B(1,0) = Mg*ZMP.x;
    B(2,0) = Mg*ZMP.y;

    MatrixNd C = pinv(A)*B;//pseudoinverse
    RFz_ref = C(0,0);
    LFz_ref = C(1,0);
    RHz_ref = C(2,0);
    LHz_ref = C(3,0);

//        RFz_ref = 0.25*Mg;
//        LFz_ref = 0.25*Mg;
//        RHz_ref = 0.25*Mg;
//        LHz_ref = 0.25*Mg;
}

//0.5A/pulse -> 0.5*0.2N/(2*pi/20000*12) = 0.1N/0.0037rad = 27;
//lets make current position control test

//20 0.3 10 0.3
void OW_Quad::control_joints(JointGains Gs)
{
    Qref = OD.Joints2Q(Oi.IK_COM(QP_controled));
    vec3 dCOM, dqPel;
    vec3 dRF, dLF, dRH, dLH;
    vec3 ddCOM, ddqPel;
    vec3 ddRF, ddLF, ddRH, ddLH;
    {
        dCOM = QP.dCOM;
        dqPel = QP.dqPel;
        dRF = QP.dRF+dFootZctRef[0];
        dLF = QP.dLF+dFootZctRef[1];
        dRH = QP.dRH+dFootZctRef[2];
        dLH = QP.dLH+dFootZctRef[3];
        ddCOM = QP.ddCOM;
        ddqPel = QP.ddqPel;
        ddRF = QP.ddRF;
        ddLF = QP.ddLF;
        ddRH = QP.ddRH;
        ddLH = QP.ddLH;
    }
    calc_dQref(dCOM,dqPel,
                dRF,dLF,dRH,dLH);//dQref
    calc_ddQref(ddCOM,ddqPel,
                ddRF,ddLF,ddRH,ddLH);//ddQref
    //Current control here        //Q,dQ,ddQref to m,dm,ddmRef


//    double effM = 0.3321*0.25;//GGGGGG//about half of fitted value?
//    double effMP = 0.3321*0.25;//have to tune
    ///////////////////have to change
    ///
    ///
    ///
    double effMR = 0.0035;//fitted value
    double effMK = 0.0035;//fitted value
    double effMP = 0.0035;//fitted value
    double effDR = 0;//almost zero
    double effDK = 0;//almost zero
    double effDP = 0;//almost zero
    double effFR = 0;//almost zero
    double effFK = 0;//almost zero
    double effFP = 0;//almost zero

    double MaxA = 10;



    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        ms_old[i] = ms[i];
        dms_old[i] = dms[i];
    }


    ms[RHR]= Oii.q2m_rhr(WS.JointPosEnc.RHR);
    dms[RHR] = Oii.dq2dm_rhr(WS.JointPosEnc.RHR,WS.JointVel.RHR);
    ms[LHR]= Oii.q2m_lhr(WS.JointPosEnc.LHR);
    dms[LHR] = Oii.dq2dm_lhr(WS.JointPosEnc.LHR,WS.JointVel.LHR);
    //ms[RSR]= Oii.q2m_rhr(WS.JointPosEnc.RSR);
    //dms[RSR] = Oii.dq2dm_rhr(WS.JointPosEnc.RSR,WS.JointVel.RSR);
    //ms[LSR]= Oii.q2m_lhr(WS.JointPosEnc.LSR);
   // dms[LSR] = Oii.dq2dm_lhr(WS.JointPosEnc.LSR,WS.JointVel.LSR);


    ms[RHP]= Oii.q2m_hp(WS.JointPosEnc.RHP);
    dms[RHP] = Oii.dq2dm_hp(WS.JointPosEnc.RHP,WS.JointVel.RHP);
    ms[LHP]= Oii.q2m_hp(WS.JointPosEnc.LHP);
    dms[LHP] = Oii.dq2dm_hp(WS.JointPosEnc.LHP,WS.JointVel.LHP);
//    ms[RSP]= Oii.q2m_hp(WS.JointPosEnc.RSP);
//    dms[RSP] = Oii.dq2dm_hp(WS.JointPosEnc.RSP,WS.JointVel.RSP);
//    ms[LSP]= Oii.q2m_hp(WS.JointPosEnc.LSP);
//    dms[LSP] = Oii.dq2dm_hp(WS.JointPosEnc.LSP,WS.JointVel.LSP);


    ms[RKN]= Oii.q2m_kn(WS.JointPosEnc.RKN);
    dms[RKN] = Oii.dq2dm_kn(WS.JointPosEnc.RKN,WS.JointVel.RKN);
    ms[LKN]= Oii.q2m_kn(WS.JointPosEnc.LKN);
    dms[LKN] = Oii.dq2dm_kn(WS.JointPosEnc.LKN,WS.JointVel.LKN);
//    ms[REB]= Oii.q2m_kn(WS.JointPosEnc.REB);
//    dms[REB] = Oii.dq2dm_kn(WS.JointPosEnc.REB,WS.JointVel.REB);
//    ms[LEB]= Oii.q2m_kn(WS.JointPosEnc.LEB);
//    dms[LEB] = Oii.dq2dm_kn(WS.JointPosEnc.LEB,WS.JointVel.LEB);


    //cartesian space tracking
    //later....


    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        Ains_xyz[i] +=Ains_xyz[i]*effMR*ddmrefs[i];//acc compensation
    }


    //joint space tracking
    int maxAlpha = 5;//0.01s

//    for(int i=0;i<NO_OF_JOINTS;i++)
//    {
//        fdms[i] = dmf[i].do_filt(dms[i]);
//        if(ms_old[i]<-900)
//        {
//            ms_old[i] = ms[i];
//        }
//        if(dms_old[i]<-900)
//        {
//            dms_old[i] = dms[i];
//        }
//        if(i==RHR||i==RSR)
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==RHR){Footnum = 0;}
//            if(i==RSR){Footnum = 2;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.HRG.Kp_swing; Kd = Gs.HRG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                //Kp = Kp_stand; Kd = Kd_stand;
//                Kp = Gs.HRG.Kp_stand; Kd = Gs.HRG.Kd_stand;
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;
//                if(GetWP().Gait==Trot)
//                {
//                    aa = 0;
//                }
//                else
//                {
//                    aa = alphamref[i]/maxAlpha;
//                }
//            }//GGGGGG

//            mrefs[i] = Oii.q2m_rhr(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_rhr(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_rhr(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMR*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMP*ddmrefs[i];
//            Ains[i]+=effDR*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFR);
//        }
//        else if(i==LHR||i==LSR)
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==LHR){Footnum = 1;}
//            if(i==LSR){Footnum = 3;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.HRG.Kp_swing; Kd = Gs.HRG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                //Kp = Kp_stand; Kd = Kd_stand;
//                Kp = Gs.HRG.Kp_stand; Kd = Gs.HRG.Kd_stand;
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;
//                if(GetWP().Gait==Trot)
//                {
//                    aa = 0;
//                }
//                else
//                {
//                    aa = alphamref[i]/maxAlpha;
//                }
//            }//GGGGGG

//            mrefs[i] = Oii.q2m_lhr(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_lhr(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_lhr(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMR*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMP*ddmrefs[i];
//            Ains[i]+=effDR*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFR);
//        }
//        else if(i==RHP||i==LHP||i==RSP||i==LSP)
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==RHP){Footnum = 0;}
//            if(i==LHP){Footnum = 1;}
//            if(i==RSP){Footnum = 2;}
//            if(i==LSP){Footnum = 3;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.HPG.Kp_swing; Kd = Gs.HPG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                //Kp = Kp_stand; Kd = Kd_stand;
//                Kp = Gs.HPG.Kp_swing; Kd = Gs.HPG.Kd_swing;
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;
//                if(GetWP().Gait==Trot)
//                {
//                    aa = 0;
//                }
//                else
//                {
//                    aa = alphamref[i]/maxAlpha;
//                }
//            }//GGGGGG


//            mrefs[i] = Oii.q2m_hp(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_hp(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_hp(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMP*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMP*ddmrefs[i];
//            Ains[i]+=effDP*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFP);
//        }
//        else
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==RKN){Footnum = 0;}
//            if(i==LKN){Footnum = 1;}
//            if(i==REB){Footnum = 2;}
//            if(i==LEB){Footnum = 3;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.KNG.Kp_swing; Kd = Gs.KNG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                Kp = Gs.KNG.Kp_stand; Kd = Gs.KNG.Kd_stand;//small gain for knee only
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;
//                for(int fn = 0;fn<4;fn++)
//                {
//                    if(Footnum==fn&&FootLandCnt[fn]>0)//early landing
//                    {
//                        //std::cout<<"ttat"<<std::endl;
//                        FootLandCnt[fn]--;
//                        Kp = Gs.KNGland.Kp_swing;Kd = Gs.KNGland.Kd_swing;
//                    }
//                }
//                if(GetWP().Gait==Trot)
//                {
//                    aa = 0;
//                }
//                else
//                {
//                    aa = alphamref[i]/maxAlpha;
//                }
//            }//GGGGGG

//            mrefs[i] = Oii.q2m_kn(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_kn(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_kn(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMK*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMK*ddmrefs[i];
//            Ains[i]+=effDK*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFK);//front/////
//        }
//        if(Ains[i]>MaxA){Ains[i] = MaxA;}
//        if(Ains[i]<-MaxA){Ains[i] = -MaxA;}

//        if(i==RKN||i==LKN||i==REB||i==LEB||i==RHP||i==LHP||i==RSP||i==LSP)
//        {
//            if(fabs(ms[i]-ms_old[i])>0.01*100)
//            {
//                cout<<"encoder pos jump "<<ms[i]<<" "<<ms_old[i]<<" "<<i<<endl;
//                isWalking = false; save_all(2);
//                Ains[i] = 0;
//            }
//            if(fabs(dms[i]-dms_old[i])>1.0*100)
//            {
//                cout<<"encoder vel jump "<<dms[i]<<" "<<dms_old[i]<<" "<<i<<endl;
//                isWalking = false; save_all(2);
//                Ains[i] = 0;
//            }
//        }
//    }
//    QJ_CRef.RHR = Ains[RHR];
//    QJ_CRef.LHR = Ains[LHR];
//    QJ_CRef.RSR = Ains[RSR];
//    QJ_CRef.LSR = Ains[LSR];

//        QJ_CRef.RHP = Ains[RHP];
//        QJ_CRef.LHP = Ains[LHP];
//        QJ_CRef.RSP = Ains[RSP];
//        QJ_CRef.LSP = Ains[LSP];

//    QJ_CRef.RKN = Ains[RKN];
//    QJ_CRef.LKN = Ains[LKN];
//    QJ_CRef.REB = Ains[REB];
//    QJ_CRef.LEB = Ains[LEB];

//    //calc JT-F from foot
//    //maybe have to change from Qref to Qnow

//    OD.CalcEndeffectorJacobian3D(Qnow);
//    VectorNd FFoot = VectorNd::Zero(3);
//    FFoot[0] = -RFx_ref;
//    FFoot[1] = -RFy_ref;
//    FFoot[2] = -RFz_ref;
//    VectorNd TF = OD.JacobianRF3D.transpose()*FFoot;
//    TTF = TF;
//    TTF[6+RHR] = TF[6+RHR];
//    TTF[6+RHP] = TF[6+RHP];
//    TTF[6+RKN] = TF[6+RKN];

//    FFoot[0] = -LFx_ref;
//    FFoot[1] = -LFy_ref;
//    FFoot[2] = -LFz_ref;
//    TF = OD.JacobianLF3D.transpose()*FFoot;
//    TTF[6+LHR] = TF[6+LHR];
//    TTF[6+LHP] = TF[6+LHP];
//    TTF[6+LKN] = TF[6+LKN];

//    FFoot[0] = -RHx_ref;
//    FFoot[1] = -RHy_ref;
//    FFoot[2] = -RHz_ref;
//    TF = OD.JacobianRH3D.transpose()*FFoot;
//    TTF[6+RSR] = TF[6+RSR];
//    TTF[6+RSP] = TF[6+RSP];
//    TTF[6+REB] = TF[6+REB];

//    FFoot[0] = -LHx_ref;
//    FFoot[1] = -LHy_ref;
//    FFoot[2] = -LHz_ref;
//    TF = OD.JacobianLH3D.transpose()*FFoot;
//    TTF[6+LSR] = TF[6+LSR];
//    TTF[6+LSP] = TF[6+LSP];
//    TTF[6+LEB] = TF[6+LEB];

//    //put result of QP here into TTF
//    //how about ddq?
//    if(Qcnt>10)//hmmmmmm
//    {
//        int ncon = 0;
//        for(int i=0;i<4;i++){if(FootonAir[i]==false){ncon++;}}
//        if(Fcon.size()==ncon*3)
//        {
//            for(int i=0;i<4;i++)
//            {
////                if(FootonAir[i]==false)
////                {
//                    TTF[6+i*3 +0] = QP_Tauout[i*3 +0];
//                    TTF[6+i*3 +1] = QP_Tauout[i*3 +1];
//                    TTF[6+i*3 +2] = QP_Tauout[i*3 +2];
////                }
////                else
////                {
////                    TTF[6+i*3 +0] = 0;
////                    TTF[6+i*3 +1] = 0;
////                    TTF[6+i*3 +2] = 0;
////                }
//            }
//        }
//        else
//        {
//            TTF = oTTF;
//        }
////        double MRoll = 45*M_PI/180;
////        for(int i=0;i<4;i++)
////        {
////            RollQP[i] = RollQP[i] + dRollQP[i]*dt + QP_ddQnow[i*3]*dt*dt*0.5;
////            dRollQP[i] = dRollQP[i] +  QP_ddQnow[i*3]*dt;
////            if(RollQP[i] > MRoll){RollQP[i] = MRoll;}
////            if(RollQP[i] < -MRoll){RollQP[i] = -MRoll;}
////        }
//    }
//    else
//    {
////        QuadJoints QJrr = Oi.IK_COM(QP_controled);
////        RollQP[0] = QJrr.RHR;
////        RollQP[1] = QJrr.LHR;
////        RollQP[2] = QJrr.RSR;
////        RollQP[3] = QJrr.LSR;
////        dRollQP[0] = 0;
////        dRollQP[1] = 0;
////        dRollQP[2] = 0;
////        dRollQP[3] = 0;
//    }
//    oTTF = TTF;

//    double tu = 1.0;
//    //lets tune this using cpe vlaue
//    QJ_CFF.RHR = tu*Oii.T2CR*(TTF[6+RHR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+RHR])));
//    QJ_CFF.RHP = tu*Oii.T2CP*(TTF[6+RHP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+RHP])));
//    QJ_CFF.RKN = tu*Oii.T2CK*(TTF[6+RKN]*Oii.J_kn(Oii.q2m_kn(Qnow[6+RKN])));

//    QJ_CFF.LHR = tu*Oii.T2CR*(TTF[6+LHR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+LHR])));
//    QJ_CFF.LHP = tu*Oii.T2CP*(TTF[6+LHP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+LHP])));
//    QJ_CFF.LKN = tu*Oii.T2CK*(TTF[6+LKN]*Oii.J_kn(Oii.q2m_kn(Qnow[6+LKN])));

//    QJ_CFF.RSR = tu*Oii.T2CR*(TTF[6+RSR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+RSR])));
//    QJ_CFF.RSP = tu*Oii.T2CP*(TTF[6+RSP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+RSP])));
//    QJ_CFF.REB = tu*Oii.T2CK*(TTF[6+REB]*Oii.J_kn(Oii.q2m_kn(Qnow[6+REB])));

//    QJ_CFF.LSR = tu*Oii.T2CR*(TTF[6+LSR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+LSR])));
//    QJ_CFF.LSP = tu*Oii.T2CP*(TTF[6+LSP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+LSP])));
//    QJ_CFF.LEB = tu*Oii.T2CK*(TTF[6+LEB]*Oii.J_kn(Oii.q2m_kn(Qnow[6+LEB])));




}

QuadJoints OW_Quad::COMadjust_onestep(WalkSensors _WS)
{
    double COMadjustT = 2.0;
    if(Qcnt==0)
    {
        oldCOMx = vec3(QP.pCOM.x,0,0);
        oldCOMy = vec3(QP.pCOM.y,0,0);
        oldCOMz = vec3(QP.pCOM.z,0,0);
        //DSP start test
        if(NO_ROBOT_TEST==false)
        {
            changetoPcon[0] = true;//////////////
            changetoPcon[1] = true;
            changetoPcon[2] = true;
            changetoPcon[3] = true;
        }
    }
    WS = _WS;

    RFcontact = LFcontact = RHcontact = LHcontact = true;
    QP_est = state_est();


    //COMx,COMy,COMz interpolation

    vec3 des_COM = 0.25*(QP.pRF+QP.pLF+QP.pRH+QP.pLH);
    if(isCOMadjusting==true)
    {
        des_COM.z = des_delZ;
    }
    if(isCOMadjusting_finish==true)
    {
        des_COM.z = min(des_delZ+0.1,0.5);
    }
    VectorNd Fx, Fy, Fz;
    Fx = calc_5th((t_now-dt),COMadjustT,oldCOMx,vec3(des_COM.x,0,0));
    Fy = calc_5th((t_now-dt),COMadjustT,oldCOMy,vec3(des_COM.y,0,0));
    Fz = calc_5th((t_now-dt),COMadjustT,oldCOMz,vec3(des_COM.z,0,0));
    double st_5,st_4,st_3,st_2,st_1;
    st_1 = t_now;
    st_2 = st_1*st_1;
    st_3 = st_2*st_1;
    st_4 = st_3*st_1;
    st_5 = st_4*st_1;
    QP.pCOM.x =Fx[0]*st_5 + Fx[1]*st_4 + Fx[2]*st_3 + Fx[3]*st_2 + Fx[4]*st_1 + Fx[5];
    oldCOMx[0] =QP.pCOM.x;
    oldCOMx[1] = 5*Fx[0]*st_4 + 4*Fx[1]*st_3 + 3*Fx[2]*st_2 + 2*Fx[3]*st_1 + Fx[4];
    oldCOMx[2] = 20*Fx[0]*st_3 + 12*Fx[1]*st_2 + 6*Fx[2]*st_1 + 2*Fx[3];
    QP.dCOM.x = oldCOMx[1];
    QP.ddCOM.x = oldCOMx[2];
    QP.pCOM.y =Fy[0]*st_5 + Fy[1]*st_4 + Fy[2]*st_3 + Fy[3]*st_2 + Fy[4]*st_1 + Fy[5];
    oldCOMy[0] =QP.pCOM.y;
    oldCOMy[1] = 5*Fy[0]*st_4 + 4*Fy[1]*st_3 + 3*Fy[2]*st_2 + 2*Fy[3]*st_1 + Fy[4];
    oldCOMy[2] = 20*Fy[0]*st_3 + 12*Fy[1]*st_2 + 6*Fy[2]*st_1 + 2*Fy[3];
    QP.dCOM.y = oldCOMy[1];
    QP.ddCOM.y = oldCOMy[2];
    QP.pCOM.z =Fz[0]*st_5 + Fz[1]*st_4 + Fz[2]*st_3 + Fz[3]*st_2 + Fz[4]*st_1 + Fz[5];
    oldCOMz[0] =QP.pCOM.z;
    oldCOMz[1] = 5*Fz[0]*st_4 + 4*Fz[1]*st_3 + 3*Fz[2]*st_2 + 2*Fz[3]*st_1 + Fz[4];
    oldCOMz[2] = 20*Fz[0]*st_3 + 12*Fz[1]*st_2 + 6*Fz[2]*st_1 + 2*Fz[3];
    QP.dCOM.z = oldCOMz[1];
    QP.ddCOM.z = oldCOMz[2];


    QP_controled = QP;
    QP_controled.pRF.z =QP.pRF.z + FootZctFT[0];
    QP_controled.pLF.z =QP.pLF.z + FootZctFT[1];
    QP_controled.pRH.z =QP.pRH.z + FootZctFT[2];
    QP_controled.pLH.z =QP.pLH.z + FootZctFT[3];

    oldF1 = QP_controled.pRF;
    oldF2 = QP_controled.pLH;
    F1 = QP_controled.pRH;
    F2 = QP_controled.pLH;

    if(Qcnt*dt>COMadjustT&&isCOMadjusting==true)
    {
        isCOMadjusting = false;
        Qcnt = 0;
        t_now = 0;
        estimate_plane();
    }
    if(Qcnt*dt>COMadjustT&&isCOMadjusting_finish==true)
    {
        isCOMadjusting_finish = false;
        Qcnt = 0;
        t_now = 0;
        QuadJoints QJenc = WS.JointPosEnc;
        if(NO_ROBOT_TEST)
        {
            QJenc = WS.JointPos;
        }
        QuadJoints QJref = Oi.IK_COM(QP_controled);

        vec3 jRF(QJref.RHR,QJref.RHP,QJenc.RKN);
        QP = Oi.Update_RF(QP_controled,jRF);
        vec3 jLH(QJref.LSR,QJref.LSP,QJenc.LEB);
        QP = Oi.Update_LH(QP,jLH);
        vec3 jLF(QJref.LHR,QJref.LHP,QJenc.LKN);
        QP = Oi.Update_LF(QP,jLF);
        vec3 jRH(QJref.RSR,QJref.RSP,QJenc.REB);
        QP = Oi.Update_RH(QP,jRH);
        QP_controled = QP;
        for(int i=0;i<4;i++)
        {
            FootZctFT[i] = 0.0;
            FootZctRef[i] = 0.0;
            dFootZctRef[i] = 0.0;
        }
        isWalking = false;
        return QJenc;
    }
    ZMP.x = QP.pCOM.x;//no ddCOM now
    ZMP.y = QP.pCOM.y;
    ZMP.z = 0;
    FFz_ref();
    COMADJUST_control();
    Mchanged = true;
    oldpRFx = vec3(QP.pRF.x,0,0);
    oldpRFy = vec3(QP.pRF.y,0,0);
    oldpLFx = vec3(QP.pLF.x,0,0);
    oldpLFy = vec3(QP.pLF.y,0,0);
    oldpRHx = vec3(QP.pRH.x,0,0);
    oldpRHy = vec3(QP.pRH.y,0,0);
    oldpLHx = vec3(QP.pLH.x,0,0);
    oldpLHy = vec3(QP.pLH.y,0,0);
    Qcnt++;
    t_now += dt;
    QuadJoints QJout = Oi.IK_COM(QP_controled);
    return QJout;
}




void OW_Quad::save_onestep(int cnt)
{
    Rcnt++;
    if(cnt<SAVEMAXCNT)
    {
        SAVE[0][cnt] = QP.pCOM.x;
        SAVE[1][cnt] = QP.pCOM.y;
        SAVE[2][cnt] = QP.pCOM.z;

        SAVE[3][cnt] = QP.dCOM.x;
        SAVE[4][cnt] = QP.dCOM.y;
        SAVE[5][cnt] = QP.dCOM.z;

        SAVE[6][cnt] = QP.ddCOM.x;
        SAVE[7][cnt] = QP.ddCOM.y;
        SAVE[8][cnt] = QP.ddCOM.z;

        SAVE[9][cnt] = QP.pRF.x;
        SAVE[10][cnt] = QP.pRF.y;
        SAVE[11][cnt] = QP.pRF.z;

        SAVE[12][cnt] = QP.pLF.x;
        SAVE[13][cnt] = QP.pLF.y;
        SAVE[14][cnt] = QP.pLF.z;

        SAVE[15][cnt] = QP.pRH.x;
        SAVE[16][cnt] = QP.pRH.y;
        SAVE[17][cnt] = QP.pRH.z;

        SAVE[18][cnt] = QP.pLH.x;
        SAVE[19][cnt] = QP.pLH.y;
        SAVE[20][cnt] = QP.pLH.z;

        SAVE[21][cnt] = QP.qPel.x;
        SAVE[22][cnt] = QP.qPel.y;
        SAVE[23][cnt] = QP.qPel.z;
        SAVE[24][cnt] = QP.qPel.w;
        SAVE[25][cnt] = 0;
        SAVE[26][cnt] = QPcnt;

        SAVE[27][cnt] = QP_est.pCOM.x;
        SAVE[28][cnt] = QP_est.pCOM.y;
        SAVE[29][cnt] = QP_est.pCOM.z;

        SAVE[30][cnt] = WS.IMUangle.x;
        SAVE[31][cnt] = WS.IMUangle.y;
        SAVE[32][cnt] = WS.IMUangle.z;

        SAVE[33][cnt] = WS.IMUomega_raw.x;
        SAVE[34][cnt] = WS.IMUomega_raw.y;
        SAVE[35][cnt] = WS.IMUomega_raw.z;

        SAVE[36][cnt] = CP.x;
        SAVE[37][cnt] = CP.y;
        SAVE[38][cnt] = CP.z;

        SAVE[39][cnt] = CP_nn.x;
        SAVE[40][cnt] = CP_nn.y;
        SAVE[41][cnt] = CP_nn.z;

        SAVE[42][cnt] = WS.RF_Fz;
        SAVE[43][cnt] = WS.LF_Fz;
        SAVE[44][cnt] = WS.RH_Fz;

        SAVE[45][cnt] = WS.LH_Fz;
        SAVE[46][cnt] = 0;
        SAVE[47][cnt] = Rcnt*dt;

        SAVE[48][cnt] = QP_est.dCOM.x;
        SAVE[49][cnt] = QP_est.dCOM.y;
        SAVE[50][cnt] = QP_est.dCOM.z;

        SAVE[51][cnt] = COM_nn.x;
        SAVE[52][cnt] = COM_nn.y;
        SAVE[53][cnt] = COM_nn.z;

        SAVE[54][cnt] = ZMP.x;
        SAVE[55][cnt] = ZMP.y;
        SAVE[56][cnt] = ZMP.z;

        SAVE[57][cnt] = lastWave;
        SAVE[58][cnt] = 0;
        SAVE[59][cnt] = 0;

        SAVE[60][cnt] = 0;
        SAVE[61][cnt] = WS_NOW;
        SAVE[62][cnt] = 0;

        SAVE[63][cnt] = FootZctFT[0];
        SAVE[64][cnt] = FootZctFT[1];
        SAVE[65][cnt] = FootZctFT[2];

        SAVE[66][cnt] = FootZctFT[3];
        SAVE[67][cnt] = FC_FB;
        SAVE[68][cnt] = FC_LR;

        SAVE[69][cnt] = RFz_ref;
        SAVE[70][cnt] = LFz_ref;
        SAVE[71][cnt] = RHz_ref;

        SAVE[72][cnt] = LHz_ref;
        SAVE[73][cnt] = 0;
        SAVE[74][cnt] = FootZctFT2[0];

        SAVE[75][cnt] = FootZctFT2[1];
        SAVE[76][cnt] = FootZctFT2[2]            ;
        SAVE[77][cnt] = FootZctFT2[3];


        SAVE[78][cnt] = QP.qRF[0];
        SAVE[79][cnt] = QP.qRF[1];
        SAVE[80][cnt] = QP.qRF[2];


        SAVE[81][cnt] = QP.qRF[3];
        SAVE[82][cnt] = 0;
        SAVE[83][cnt] = 0;

        SAVE[84][cnt] = WS.F_RF.x;
        SAVE[85][cnt] = WS.F_RF.y;
        SAVE[86][cnt] = WS.F_RF.z;

        SAVE[87][cnt] = ct.x;
        SAVE[88][cnt] = ct.y;
        SAVE[89][cnt] = ct.z;

        SAVE[90][cnt] = ct_ff.x;
        SAVE[91][cnt] = ct_ff.y;
        SAVE[92][cnt] = ct_ff.z;

        SAVE[93][cnt] = dFootZctRef[0];
        SAVE[94][cnt] = dFootZctRef[1];
        SAVE[95][cnt] = dFootZctRef[2];

        SAVE[96][cnt] = dFootZctRef[3];
        SAVE[97][cnt] = 0;
        SAVE[98][cnt] = 0;

        SAVE[99][cnt] = OP.zmp_refs[0].x;
        SAVE[100][cnt] =OP.zmp_refs[0].y;
        SAVE[101][cnt] =OP.zmp_refs[0].z;

        SAVE[102][cnt] = dYawcon;
        SAVE[103][cnt] = ff;
        SAVE[104][cnt] = ll;

        SAVE[105][cnt] = conFx;
        SAVE[106][cnt] = conFy;
        SAVE[107][cnt] = WS.looptime;

        SAVE[108][cnt] = cpe.x;
        SAVE[109][cnt] = cpe.y;
        SAVE[110][cnt] = cpe.z;

        SAVE[111][cnt] = COMr.x;
        SAVE[112][cnt] = COMr.y;
        SAVE[113][cnt] = COMr.z;

        SAVE[114][cnt] = Hlandoff;
        SAVE[115][cnt] = Flandoff;
        SAVE[116][cnt] = t_now;

        SAVE[117][cnt] = CP_est.x;
        SAVE[118][cnt] = CP_est.y;
        SAVE[119][cnt] = CP_est.z;


        SAVE[120][cnt] = CP_est_nn.x;
        SAVE[121][cnt] = CP_est_nn.y;
        SAVE[122][cnt] = CP_est_nn.z;

        SAVE[123][cnt] = HOPPHASE;
        SAVE[124][cnt] = 0;
        SAVE[125][cnt] = 0;
        SAVE[126][cnt] = 0;

        SAVE[127][cnt] = decomp_A.x;
        SAVE[128][cnt] = decomp_A.y;
        SAVE[129][cnt] = decomp_A.z;

        SAVE[130][cnt] = decomp_W.x;
        SAVE[131][cnt] = decomp_W.y;
        SAVE[132][cnt] = decomp_W.z;

        SAVE[133][cnt] = desCOM.x;
        SAVE[134][cnt] = desCOM.y;
        SAVE[135][cnt] = desCOM.z;

        SAVE[136][cnt] = QP_real.pCOM.x;
        SAVE[137][cnt] = QP_real.pCOM.y;
        SAVE[138][cnt] = QP_real.pCOM.z;


        //from 140

        for(int i=0;i<18;i++)
        {
            //140+19+19
            SAVE[i+140][cnt] = dQref[i];
        }
        for(int i=0;i<18;i++)
        {
            //140+19+19+18
            SAVE[i+160][cnt] = ddQref[i];
        }

        SAVE[180][cnt] = QJ_CRef.RHR;//Current reference
        SAVE[181][cnt] = QJ_CRef.RHP;
        SAVE[182][cnt] = QJ_CRef.RKN;
        SAVE[183][cnt] = QJ_CRef.LHR;
        SAVE[184][cnt] = QJ_CRef.LHP;
        SAVE[185][cnt] = QJ_CRef.LKN;

        SAVE[186][cnt] = QJ_CRef.RSR;
        SAVE[187][cnt] = QJ_CRef.RSP;
        SAVE[188][cnt] = QJ_CRef.REB;
        SAVE[189][cnt] = QJ_CRef.LSR;
        SAVE[190][cnt] = QJ_CRef.LSP;
        SAVE[191][cnt] = QJ_CRef.LEB;


        //from 200
        SAVE[200][cnt] = WS.JointPos.RHR;//one step delayed reference
        SAVE[201][cnt] = WS.JointPos.RHP;
        SAVE[202][cnt] = WS.JointPos.RKN;
        SAVE[203][cnt] = WS.JointPos.LHR;
        SAVE[204][cnt] = WS.JointPos.LHP;
        SAVE[205][cnt] = WS.JointPos.LKN;

        SAVE[206][cnt] = WS.JointPos.RSR;
        SAVE[207][cnt] = WS.JointPos.RSP;
        SAVE[208][cnt] = WS.JointPos.REB;
        SAVE[209][cnt] = WS.JointPos.LSR;
        SAVE[210][cnt] = WS.JointPos.LSP;
        SAVE[211][cnt] = WS.JointPos.LEB;
        //from 220
        SAVE[220][cnt] = WS.JointPosEnc.RHR;
        SAVE[221][cnt] = WS.JointPosEnc.RHP;
        SAVE[222][cnt] = WS.JointPosEnc.RKN;
        SAVE[223][cnt] = WS.JointPosEnc.LHR;
        SAVE[224][cnt] = WS.JointPosEnc.LHP;
        SAVE[225][cnt] = WS.JointPosEnc.LKN;

        SAVE[226][cnt] = WS.JointPosEnc.RSR;
        SAVE[227][cnt] = WS.JointPosEnc.RSP;
        SAVE[228][cnt] = WS.JointPosEnc.REB;
        SAVE[229][cnt] = WS.JointPosEnc.LSR;
        SAVE[230][cnt] = WS.JointPosEnc.LSP;
        SAVE[231][cnt] = WS.JointPosEnc.LEB;

        //from 240
        SAVE[240][cnt] = WS.JointVel.RHR;
        SAVE[241][cnt] = WS.JointVel.RHP;
        SAVE[242][cnt] = WS.JointVel.RKN;
        SAVE[243][cnt] = WS.JointVel.LHR;
        SAVE[244][cnt] = WS.JointVel.LHP;
        SAVE[245][cnt] = WS.JointVel.LKN;
        SAVE[246][cnt] = WS.JointVel.RSR;
        SAVE[247][cnt] = WS.JointVel.RSP;
        SAVE[248][cnt] = WS.JointVel.REB;
        SAVE[249][cnt] = WS.JointVel.LSR;
        SAVE[250][cnt] = WS.JointVel.LSP;
        SAVE[251][cnt] = WS.JointVel.LEB;

        //from 260
        SAVE[260][cnt] = WS.JointCurrent.RHR;
        SAVE[261][cnt] = WS.JointCurrent.RHP;
        SAVE[262][cnt] = WS.JointCurrent.RKN;
        SAVE[263][cnt] = WS.JointCurrent.LHR;
        SAVE[264][cnt] = WS.JointCurrent.LHP;
        SAVE[265][cnt] = WS.JointCurrent.LKN;
        SAVE[266][cnt] = WS.JointCurrent.RSR;
        SAVE[267][cnt] = WS.JointCurrent.RSP;
        SAVE[268][cnt] = WS.JointCurrent.REB;
        SAVE[269][cnt] = WS.JointCurrent.LSR;
        SAVE[270][cnt] = WS.JointCurrent.LSP;
        SAVE[271][cnt] = WS.JointCurrent.LEB;

        //from 280
        SAVE[280][cnt] = QJ_CFF.RHR;
        SAVE[281][cnt] = QJ_CFF.RHP;
        SAVE[282][cnt] = QJ_CFF.RKN;
        SAVE[283][cnt] = QJ_CFF.LHR;
        SAVE[284][cnt] = QJ_CFF.LHP;
        SAVE[285][cnt] = QJ_CFF.LKN;
        SAVE[286][cnt] = QJ_CFF.RSR;
        SAVE[287][cnt] = QJ_CFF.RSP;
        SAVE[288][cnt] = QJ_CFF.REB;
        SAVE[289][cnt] = QJ_CFF.LSR;
        SAVE[290][cnt] = QJ_CFF.LSP;
        SAVE[291][cnt] = QJ_CFF.LEB;

        //from 300
        SAVE[300][cnt] = WS.JointPWM.RHR;
        SAVE[301][cnt] = WS.JointPWM.RHP;
        SAVE[302][cnt] = WS.JointPWM.RKN;
        SAVE[303][cnt] = WS.JointPWM.LHR;
        SAVE[304][cnt] = WS.JointPWM.LHP;
        SAVE[305][cnt] = WS.JointPWM.LKN;
        SAVE[306][cnt] = WS.JointPWM.RSR;
        SAVE[307][cnt] = WS.JointPWM.RSP;
        SAVE[308][cnt] = WS.JointPWM.REB;
        SAVE[309][cnt] = WS.JointPWM.LSR;
        SAVE[310][cnt] = WS.JointPWM.LSP;
        SAVE[311][cnt] = WS.JointPWM.LEB;

        //from 320
        SAVE[320][cnt] = ms[RHR];
        SAVE[321][cnt] = ms[RHP];
        SAVE[322][cnt] = ms[RKN];
        SAVE[323][cnt] = ms[LHR];
        SAVE[324][cnt] = ms[LHP];
        SAVE[325][cnt] = ms[LKN];
//        SAVE[326][cnt] = ms[RSR];
//        SAVE[327][cnt] = ms[RSP];
//        SAVE[328][cnt] = ms[REB];
//        SAVE[329][cnt] = ms[LSR];
//        SAVE[330][cnt] = ms[LSP];
//        SAVE[331][cnt] = ms[LEB];

        //from 340
        SAVE[340][cnt] = dms[RHR];
        SAVE[341][cnt] = dms[RHP];
        SAVE[342][cnt] = dms[RKN];
        SAVE[343][cnt] = dms[LHR];
        SAVE[344][cnt] = dms[LHP];
        SAVE[345][cnt] = dms[LKN];
//        SAVE[346][cnt] = dms[RSR];
//        SAVE[347][cnt] = dms[RSP];
//        SAVE[348][cnt] = dms[REB];
//        SAVE[349][cnt] = dms[LSR];
//        SAVE[350][cnt] = dms[LSP];
//        SAVE[351][cnt] = dms[LEB];


        //from 360
        SAVE[360][cnt] = mrefs[RHR];
        SAVE[361][cnt] = mrefs[RHP];
        SAVE[362][cnt] = mrefs[RKN];
        SAVE[363][cnt] = mrefs[LHR];
        SAVE[364][cnt] = mrefs[LHP];
        SAVE[365][cnt] = mrefs[LKN];
//        SAVE[366][cnt] = mrefs[RSR];
//        SAVE[367][cnt] = mrefs[RSP];
//        SAVE[368][cnt] = mrefs[REB];
//        SAVE[369][cnt] = mrefs[LSR];
//        SAVE[370][cnt] = mrefs[LSP];
//        SAVE[371][cnt] = mrefs[LEB];

        //from 380
        SAVE[380][cnt] = dmrefs[RHR];
        SAVE[381][cnt] = dmrefs[RHP];
        SAVE[382][cnt] = dmrefs[RKN];
        SAVE[383][cnt] = dmrefs[LHR];
        SAVE[384][cnt] = dmrefs[LHP];
        SAVE[385][cnt] = dmrefs[LKN];
//        SAVE[386][cnt] = dmrefs[RSR];
//        SAVE[387][cnt] = dmrefs[RSP];
//        SAVE[388][cnt] = dmrefs[REB];
//        SAVE[389][cnt] = dmrefs[LSR];
//        SAVE[390][cnt] = dmrefs[LSP];
//        SAVE[391][cnt] = dmrefs[LEB];

        //from 400
        SAVE[400][cnt] = WS.BIGERR[RHR];
        SAVE[401][cnt] = WS.BIGERR[RHP];
        SAVE[402][cnt] = WS.BIGERR[RKN];
        SAVE[403][cnt] = WS.BIGERR[LHR];
        SAVE[404][cnt] = WS.BIGERR[LHP];
        SAVE[405][cnt] = WS.BIGERR[LKN];
//        SAVE[406][cnt] = WS.BIGERR[RSR];
//        SAVE[407][cnt] = WS.BIGERR[RSP];
//        SAVE[408][cnt] = WS.BIGERR[REB];
//        SAVE[409][cnt] = WS.BIGERR[LSR];
//        SAVE[410][cnt] = WS.BIGERR[LSP];
//        SAVE[411][cnt] = WS.BIGERR[LEB];

        //from 420
        SAVE[420][cnt] = WS.ENCERR[RHR];
        SAVE[421][cnt] = WS.ENCERR[RHP];
        SAVE[422][cnt] = WS.ENCERR[RKN];
        SAVE[423][cnt] = WS.ENCERR[LHR];
        SAVE[424][cnt] = WS.ENCERR[LHP];
        SAVE[425][cnt] = WS.ENCERR[LKN];
//        SAVE[426][cnt] = WS.ENCERR[RSR];
//        SAVE[427][cnt] = WS.ENCERR[RSP];
//        SAVE[428][cnt] = WS.ENCERR[REB];
//        SAVE[429][cnt] = WS.ENCERR[LSR];
//        SAVE[430][cnt] = WS.ENCERR[LSP];
//        SAVE[431][cnt] = WS.ENCERR[LEB];

        //from 440
        SAVE[440][cnt] = WS.CANERR[RHR];
        SAVE[441][cnt] = WS.CANERR[RHP];
        SAVE[442][cnt] = WS.CANERR[RKN];
        SAVE[443][cnt] = WS.CANERR[LHR];
        SAVE[444][cnt] = WS.CANERR[LHP];
        SAVE[445][cnt] = WS.CANERR[LKN];
//        SAVE[446][cnt] = WS.CANERR[RSR];
//        SAVE[447][cnt] = WS.CANERR[RSP];
//        SAVE[448][cnt] = WS.CANERR[REB];
//        SAVE[449][cnt] = WS.CANERR[LSR];
//        SAVE[450][cnt] = WS.CANERR[LSP];
//        SAVE[451][cnt] = WS.CANERR[LEB];


        SAVE[452][cnt] = COMref.x;
        SAVE[453][cnt] = COMref.y;
        SAVE[454][cnt] = COMref.z;

        SAVE[455][cnt] = dCOMref.x;
        SAVE[456][cnt] = dCOMref.y;
        SAVE[457][cnt] = dCOMref.z;

        SAVE[458][cnt] = IMUomega_filtered.x;
        SAVE[459][cnt] = IMUomega_filtered.y;
        SAVE[460][cnt] = IMUomega_filtered.z;

        SAVE[461][cnt] = decomp_W_filtered.x;
        SAVE[462][cnt] = decomp_W_filtered.y;
        SAVE[463][cnt] = decomp_W_filtered.z;

        SAVE[464][cnt] = RFx_ref;
        SAVE[465][cnt] = LFx_ref;
        SAVE[466][cnt] = RHx_ref;
        SAVE[467][cnt] = LHx_ref;

        SAVE[468][cnt] = RFy_ref;
        SAVE[469][cnt] = LFy_ref;
        SAVE[470][cnt] = RHy_ref;
        SAVE[471][cnt] = LHy_ref;

        SAVE[472][cnt] = ddCOMLIPM.x;
        SAVE[473][cnt] = ddCOMLIPM.y;
        SAVE[474][cnt] = ddCOMLIPM.z;

        SAVE[475][cnt] = ddCOMFBDY.x;
        SAVE[476][cnt] = ddCOMFBDY.y;
        SAVE[477][cnt] = ddCOMFBDY.z;

        SAVE[478][cnt] = WS.IMUacc.x;
        SAVE[479][cnt] = WS.IMUacc.y;
        SAVE[480][cnt] = WS.IMUacc.z;

        SAVE[481][cnt] = sumCOMerr.x;
        SAVE[482][cnt] = sumCOMerr.y;
        SAVE[483][cnt] = sumCOMerr.z;



        for(int i=0;i<12;i++)
        {
            SAVE[484+i][cnt] = TTF[6+i];
        }
        //500to 550
        for(int i=0;i<OD.err.rows();i++)
        {
            SAVE[500+i][cnt] = OD.err(i,0);
        }
        SAVE[500+OD.err.rows()][cnt] = 1;
        //from 550
        SAVE[550][cnt] = abd[0];
        SAVE[551][cnt] = abd[1];
        SAVE[552][cnt] = abd[2];

        SAVE[553][cnt] = abd_f[0];
        SAVE[554][cnt] = abd_f[1];
        SAVE[555][cnt] = abd_f[2];

        SAVE[556][cnt] = QP_ENC_PLANE.pRF[0];
        SAVE[557][cnt] = QP_ENC_PLANE.pRF[1];
        SAVE[558][cnt] = QP_ENC_PLANE.pRF[2];

        SAVE[559][cnt] = QP_ENC_PLANE.pLF[0];
        SAVE[560][cnt] = QP_ENC_PLANE.pLF[1];
        SAVE[561][cnt] = QP_ENC_PLANE.pLF[2];

        SAVE[562][cnt] = QP_ENC_PLANE.pRH[0];
        SAVE[563][cnt] = QP_ENC_PLANE.pRH[1];
        SAVE[564][cnt] = QP_ENC_PLANE.pRH[2];

        SAVE[565][cnt] = QP_ENC_PLANE.pLH[0];
        SAVE[566][cnt] = QP_ENC_PLANE.pLH[1];
        SAVE[567][cnt] = QP_ENC_PLANE.pLH[2];

        SAVE[568][cnt] = plane_rpy.x;
        SAVE[569][cnt] = plane_rpy.y;
        SAVE[570][cnt] = plane_rpy.z;

        SAVE[571][cnt] = plane_rpy_f.x;
        SAVE[572][cnt] = plane_rpy_f.y;
        SAVE[573][cnt] = plane_rpy_f.z;

        SAVE[574][cnt] = rpyQP.x;
        SAVE[575][cnt] = rpyQP.y;
        SAVE[576][cnt] = rpyQP.z;

        SAVE[577][cnt] = cnt_FZ[0];
        SAVE[578][cnt] = cnt_FZ[1];
        SAVE[579][cnt] = cnt_FZ[2];
        SAVE[580][cnt] = cnt_FZ[3];

        SAVE[581][cnt] = 0;
        SAVE[582][cnt] = 0;
        SAVE[583][cnt] = 0;

        SAVE[584][cnt] = 0;
        SAVE[585][cnt] = 0;
        SAVE[586][cnt] = 0;

        SAVE[587][cnt] = 0;
        SAVE[588][cnt] = 0;
        SAVE[589][cnt] = 0;

        SAVE[590][cnt] = rotE_I.x;
        SAVE[591][cnt] = rotE_I.y;
        SAVE[592][cnt] = rotE_I.z;

        SAVE[593][cnt] = decomp_I.x;
        SAVE[594][cnt] = decomp_I.y;
        SAVE[595][cnt] = decomp_I.z;

        SAVE[596][cnt] = sL.x;
        SAVE[597][cnt] = sL.y;
        SAVE[598][cnt] = sL.z;

        SAVE[599][cnt] = 0;
        SAVE[600][cnt] = 0;
        SAVE[601][cnt] = 0;

        SAVE[602][cnt] = 0;
        SAVE[603][cnt] = 0;
        SAVE[604][cnt] = 0;

        SAVE[605][cnt] = estCOM.x;
        SAVE[606][cnt] = estCOM.y;
        SAVE[607][cnt] = estCOM.z;

        SAVE[608][cnt] = WS.IMUomega.x;
        SAVE[609][cnt] = WS.IMUomega.y;
        SAVE[610][cnt] = WS.IMUomega.z;


        SAVE[611][cnt] = QP_est.pRF[0];
        SAVE[612][cnt] = QP_est.pRF[1];
        SAVE[613][cnt] = QP_est.pRF[2];

        SAVE[614][cnt] = QP_est.pLF[0];
        SAVE[615][cnt] = QP_est.pLF[1];
        SAVE[616][cnt] = QP_est.pLF[2];

        SAVE[617][cnt] = QP_est.pRH[0];
        SAVE[618][cnt] = QP_est.pRH[1];
        SAVE[619][cnt] = QP_est.pRH[2];

        SAVE[620][cnt] = QP_est.pLH[0];
        SAVE[621][cnt] = QP_est.pLH[1];
        SAVE[622][cnt] = QP_est.pLH[2];

        SAVE[623][cnt] = fcpe[0];
        SAVE[624][cnt] = fcpe[1];
        SAVE[625][cnt] = fcpe[2];

        SAVE[626][cnt] = fdcpe[0];
        SAVE[627][cnt] = fdcpe[1];
        SAVE[628][cnt] = fdcpe[2];

        for(int i=0;i<12;i++)
        {
            SAVE[630+i][cnt] = QP_ddQnow[6+i];
        }
        //500to 550
        for(int i=0;i<12;i++)
        {
            SAVE[642+i][cnt] = QP_Tauout[i];
        }
        int foff = 0;
        int ncon=0;
        for(int i=0;i<12;i++)
        {
            SAVE[654 + i][cnt] = 0;
            SAVE[655 + i][cnt] = 0;
            SAVE[656 + i][cnt] = 0;
        }
        for(int i=0;i<4;i++){if(FootonAir[i]==false){ncon++;}}
        if(Fcon.size()==ncon*3)
        {
            for(int i=0;i<4;i++)
            {
                if(!FootonAir[i])
                {
                    SAVE[654 + i*3][cnt] = Fcon[foff];
                    SAVE[655 + i*3][cnt] = Fcon[foff + 1];
                    SAVE[656 + i*3][cnt] = Fcon[foff + 2];
                    foff += 3;
                }
            }
        }
        SAVE[670][cnt] = QP_Qnow_save[0];
        SAVE[671][cnt] = QP_Qnow_save[1];
        SAVE[672][cnt] = QP_Qnow_save[2];

        SAVE[673][cnt] = QP_Qref[0];
        SAVE[674][cnt] = QP_Qref[1];
        SAVE[675][cnt] = QP_Qref[2];

        SAVE[676][cnt] = QP_dQnow_save[0];
        SAVE[677][cnt] = QP_dQnow_save[1];
        SAVE[678][cnt] = QP_dQnow_save[2];

        SAVE[679][cnt] = QP_dQref[0];
        SAVE[680][cnt] = QP_dQref[1];
        SAVE[681][cnt] = QP_dQref[2];

        SAVE[682][cnt] = QP_dQnow[3];
        SAVE[683][cnt] = QP_dQnow[4];
        SAVE[684][cnt] = QP_dQnow[5];

        SAVE[685][cnt] = QP_ddQnow[3];
        SAVE[686][cnt] = QP_ddQnow[4];
        SAVE[687][cnt] = QP_ddQnow[5];

        SAVE[688][cnt] = RollQP[0];
        SAVE[689][cnt] = RollQP[1];
        SAVE[690][cnt] = RollQP[2];
        SAVE[691][cnt] = RollQP[3];

        SAVE[692][cnt] = QP_ddQnow[0];
        SAVE[693][cnt] = QP_ddQnow[1];
        SAVE[694][cnt] = QP_ddQnow[2];

        SAVE[695][cnt] = Angleref.X().x;
        SAVE[696][cnt] = Angleref.X().y;
        SAVE[697][cnt] = Angleref.X().z;


        //from 700
        SAVE[700][cnt] = QJ_CFILTERED.RHR;
        SAVE[701][cnt] = QJ_CFILTERED.RHP;
        SAVE[702][cnt] = QJ_CFILTERED.RKN;
        SAVE[703][cnt] = QJ_CFILTERED.LHR;
        SAVE[704][cnt] = QJ_CFILTERED.LHP;
        SAVE[705][cnt] = QJ_CFILTERED.LKN;
        SAVE[706][cnt] = QJ_CFILTERED.RSR;
        SAVE[707][cnt] = QJ_CFILTERED.RSP;
        SAVE[708][cnt] = QJ_CFILTERED.REB;
        SAVE[709][cnt] = QJ_CFILTERED.LSR;
        SAVE[710][cnt] = QJ_CFILTERED.LSP;
        SAVE[711][cnt] = QJ_CFILTERED.LEB;


    }
    else
    {
        Qcnt = min(500*10,SAVEMAXCNT);
        overcnt = true;
        cout<<"over MaxQcnt!! start over from 5000"<<endl;
    }
}

void OW_Quad::do_save_all()
{
    if(NOSAVE)
    {
        saveflag = false;
        cout<<"NOSAVE"<<endl;
        return;
    }
    saveflag = false;
    int fnum = savenum;
    FILE* ffp = NULL;
    std::string str("OW_Quad");
    std:;time_t rawtime;
    std::tm* timeinfo;
    char buffer[80];
    std::time(&rawtime);
    timeinfo = std::localtime(&rawtime);
    std::strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
    std::string timestr(buffer);

    if(fnum==0)
    {
        std::ostringstream strs,strs2;
        strs << savestepT;
        str = str + strs.str();
        str = str + "_GAITNUM";
        strs2 <<WP.Gait;
        str = str + strs2.str();
        str = str + "_";
        str = str+timestr;
        str = str + ".txt";
        ffp= fopen(str.c_str(),"w");
    }
    else if(fnum==1)
    {ffp= fopen("OW_Quad_controltest.txt","w");}
    else if(fnum==2)
    {        
        str = str+"_falling_";
        str = str + "_";
        str = str+timestr;
        str = str+".txt";
        ffp= fopen(str.c_str(),"w");
    }
    int savenum = min(Qcnt,SAVEMAXCNT);
    if(overcnt){savenum  = SAVEMAXCNT;}
    for(int i=0;i<savenum;i++)
    {
        for(int j=0;j<SAVEMAX;j++)
        {
            fprintf(ffp,"%f\t",SAVE[j][i]);
        }
        fprintf(ffp,"\n");
        if(i%10000==0)
        {
            printf("saving... %d / %d\n",i,Qcnt);
        }
    }

    fclose(ffp);
    printf("%s\n",str.c_str());
    printf("save done\n");

}
void OW_Quad::save_all(int fnum)
{
    savenum = fnum;
    if(fnum==2)
    {
        isfalldown = true;
        printf("falldown!!\n");
    }
    saveflag = true;
    savestepT = WP.step_T;
    printf("walk finished %d type: %d\n",Qcnt,fnum);

}

void OW_Quad:: COMADJUST_control()
{
    double Kpy = 0.02*R2Df*0.5;
    double Kdy = 0.001*R2Df*0.5;
    double Kpx = 0.01*R2Df*0.5;
    double Kdx = 0.0005*R2Df*0.5;

    //modify RFz_ref, LFz_ref, RHz_ref, LHz_ref
    //and Leg length at once???
    //lets try just leg length

    FC_FB = (+ Kpy*WS.IMUangle.y  + Kdy*WS.IMUomega.y);
    FC_LR = -(- Kpx*WS.IMUangle.x - Kdx*WS.IMUomega.x);
    double MaxFC = 0.07;
    if(FC_FB> MaxFC){FC_FB = MaxFC;}
    if(FC_FB<-MaxFC){FC_FB =-MaxFC;}
    if(FC_LR> MaxFC){FC_LR = MaxFC;}
    if(FC_LR<-MaxFC){FC_LR =-MaxFC;}

    dFootZctRef[0] = (FC_FB-FC_LR);
    dFootZctRef[1] = (FC_FB+FC_LR);
    dFootZctRef[2] = (-FC_FB-FC_LR);
    dFootZctRef[3] = (-FC_FB+FC_LR);

    for(int i=0;i<4;i++)
    {
        FootZctRef[i] = FootZctRef[i]+dFootZctRef[i]*dt;
        double MaxCon = 0.05;
        if(FootZctRef[i]>MaxCon){FootZctRef[i] = MaxCon; if(dFootZctRef[i]>0){dFootZctRef[i] = 0;}}
        if(FootZctRef[i]<-MaxCon){FootZctRef[i] = -MaxCon; if(dFootZctRef[i]<0){dFootZctRef[i] = 0;}}
        FootZctFT[i] = FootZctRef[i];
    }
    //rotation control
    mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    mat3 IMUrotz = mat3(QP.qPel);
    mat3 IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x
    WS.JointPosEnc.qPel = quat(IMUrot);
    Qnow = OD.Joints2Q(WS.JointPosEnc);
    vec3 dCOM_est = QP_est.dCOM;
    QP_est = Oi.FK(WS.JointPosEnc);
    double FB_L = WP.FB_L;//((QP_est.pRH.x+QP_est.pLH.x)-(QP_est.pRF.x+QP_est.pLF.x))*0.5;
    double LR_L = WP.LR_L;//((QP_est.pLH.y+QP_est.pLF.y)-(QP_est.pRF.x+QP_est.pRF.x))*0.5;
    double wn = 10;
    double z = 1;
    double Iy = Oi.I_torso[1][1];
    double Ix = Oi.I_torso[0][0];
    double Kpyf = wn*wn*Iy/FB_L*40;//20 4000 * I / somevalue
    double Kdyf = 2*wn*z*Iy/FB_L*15;//15 300 * I /somevalue //realwn ~26?
    double Kpxf = w*wn*Ix/LR_L*40;//20
    double Kdxf = 2*wn*z*Ix/LR_L*15;//15
    conFy = -Kpyf*WS.IMUangle.y - Kdyf*WS.IMUomega.y;
    conFx = -Kpxf*WS.IMUangle.x - Kdxf*WS.IMUomega.x;
    double MG = Oi.M_total*g;

    if(conFy>0.3*MG){ conFy = 0.3*MG;}
    if(conFy<-0.3*MG){ conFy = -0.3*MG;}
    if(conFx>0.3*MG){ conFx = 0.3*MG;}
    if(conFx<-0.3*MG){ conFx = -0.3*MG;}
    RFz_ref += (conFy-conFx);
    LFz_ref += (conFy+conFx);
    RHz_ref += (-conFy-conFx);
    LHz_ref += (-conFy+conFx);

    QP.dqPel =vec3(0,0,0);// vec3(-Kdxf*WS.IMUangle.x,-Kdyf*WS.IMUangle.y,0);//????

    //COM position control
    vec3 mf_ref = (QP.pRF+QP.pLF+QP.pRH+QP.pLH)*0.25;
    vec3 mf_est = (QP_est.pRF+QP_est.pLF+QP_est.pRH+QP_est.pLH)*0.25;

    vec3 com2mf_ref = QP.pCOM-mf_ref;
    vec3 com2mf_est = QP_est.pCOM-mf_est;
    if(Qcnt>0)
    {
        dCOM_est = (com2mf_est-oCOM_nn)/dt;
    }
    oCOM_nn = com2mf_est;
    QP_est.dCOM = dCOM_est;

    cpe = com2mf_ref-com2mf_est;//COM position error
    //cout<<"COM POSITION ERROR "<<cpe.x<<" "<<cpe.y<<" "<<cpe.z<<endl;
    double M = Oi.M_total;
    double Kpcom, Kdcom;
    wn = 10;
    z = 1*0.6;
    Kpcom = wn*wn*M;
    Kdcom = 2*wn*z*M;
    vec3 COMconF = Kpcom*cpe + Kdcom*(-dCOM_est);//if not working, delete dCOM_est
    for(int i=0;i<3;i++)
    {
        double maxF = 0.25*0.5*MG;
        if(COMconF[i]>maxF){COMconF = maxF;}
        if(COMconF[i]<-maxF){COMconF = -maxF;}
    }
    RFx_ref = RFy_ref = 0;
    LFx_ref = LFy_ref = 0;
    RHx_ref = RHy_ref = 0;
    LHx_ref = LHy_ref = 0;

    //testtest
    RFx_ref = M*QP.ddCOM.x*0.25;
    LFx_ref = M*QP.ddCOM.x*0.25;
    RHx_ref = M*QP.ddCOM.x*0.25;
    LHx_ref = M*QP.ddCOM.x*0.25;

    RFy_ref = M*QP.ddCOM.y*0.25;
    LFy_ref = M*QP.ddCOM.y*0.25;
    RHy_ref = M*QP.ddCOM.y*0.25;
    LHy_ref = M*QP.ddCOM.y*0.25;



//    RFx_ref+=COMconF.x*0.25;
//    LFx_ref+=COMconF.x*0.25;
//    RHx_ref+=COMconF.x*0.25;
//    LHx_ref+=COMconF.x*0.25;
//    RFy_ref+=COMconF.y*0.25;
//    LFy_ref+=COMconF.y*0.25;
//    RHy_ref+=COMconF.y*0.25;
//    LHy_ref+=COMconF.y*0.25;

    RFz_ref+=COMconF.z*0.25;
    LFz_ref+=COMconF.z*0.25;
    RHz_ref+=COMconF.z*0.25;
    LHz_ref+=COMconF.z*0.25;

    if(RFz_ref<10){ RFz_ref = 10;}
    if(LFz_ref<10){ LFz_ref = 10;}
    if(RHz_ref<10){ RHz_ref = 10;}
    if(LHz_ref<10){ LHz_ref = 10;}



    QP_controled = QP;
    QP_controled.pRF.z =QP.pRF.z + FootZctFT[0];
    QP_controled.pLF.z =QP.pLF.z + FootZctFT[1];
    QP_controled.pRH.z =QP.pRH.z + FootZctFT[2];
    QP_controled.pLH.z =QP.pLH.z + FootZctFT[3];

    JointGains Gains;
    Gains.HRG.Kp_stand = 0;
    Gains.HRG.Kd_stand = 0;
    Gains.HPG.Kp_stand = 0;
    Gains.HPG.Kd_stand = 0;
    Gains.KNG.Kp_stand = 0;
    Gains.KNG.Kd_stand = 0;
    control_joints(Gains);//stance leg damping only?
}
void OW_Quad::set_des_delZ_slope()
{
    vec3 rpyQP = Oi.Decomp_Rmat(QP.qPel,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));
    double rate = 1/sqrt(1+tan(rpyQP[2])*tan(rpyQP[2])+tan(rpyQP[1])*tan(rpyQP[1]));
    //rate = rate*rate*rate;
    if(rate<0.5){ rate = 0.5;}
    des_delZ = WP.delZ*rate;
}


void OW_Quad::control_joints_jump(JointGains Gs)
{
    Qref = OD.Joints2Q(Oi.IK_COM(QP_controled));
    vec3 dCOM, dqPel;
    vec3 dRF, dLF, dRH, dLH;
    vec3 ddCOM, ddqPel;
    vec3 ddRF, ddLF, ddRH, ddLH;
    {
        dCOM = QP.dCOM;
        dqPel = QP.dqPel;
        dRF = QP.dRF+dFootZctRef[0];
        dLF = QP.dLF+dFootZctRef[1];
        dRH = QP.dRH+dFootZctRef[2];
        dLH = QP.dLH+dFootZctRef[3];
        ddCOM = QP.ddCOM;
        ddqPel = QP.ddqPel;
        ddRF = QP.ddRF;
        ddLF = QP.ddLF;
        ddRH = QP.ddRH;
        ddLH = QP.ddLH;
    }
    calc_dQref(dCOM,dqPel,
                dRF,dLF,dRH,dLH);//dQref
    calc_ddQref(ddCOM,ddqPel,
                ddRF,ddLF,ddRH,ddLH);//ddQref
    //Current control here        //Q,dQ,ddQref to m,dm,ddmRef


//    double effM = 0.3321*0.25;//GGGGGG//about half of fitted value?
//    double effMP = 0.3321*0.25;//have to tune
    ///////////////////have to change
    ///
    ///
    ///
    double effMR = 0.0035;//fitted value
    double effMK = 0.0035;//fitted value
    double effMP = 0.0035;//fitted value
    double effDR = 0;//almost zero
    double effDK = 0;//almost zero
    double effDP = 0;//almost zero
    double effFR = 0;//almost zero
    double effFK = 0;//almost zero
    double effFP = 0;//almost zero

    double MaxA = 10;



    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        ms_old[i] = ms[i];
        dms_old[i] = dms[i];
    }


    ms[RHR]= Oii.q2m_rhr(WS.JointPosEnc.RHR);
    dms[RHR] = Oii.dq2dm_rhr(WS.JointPosEnc.RHR,WS.JointVel.RHR);
    ms[LHR]= Oii.q2m_lhr(WS.JointPosEnc.LHR);
    dms[LHR] = Oii.dq2dm_lhr(WS.JointPosEnc.LHR,WS.JointVel.LHR);
//    ms[RSR]= Oii.q2m_rhr(WS.JointPosEnc.RSR);
//    dms[RSR] = Oii.dq2dm_rhr(WS.JointPosEnc.RSR,WS.JointVel.RSR);
//    ms[LSR]= Oii.q2m_lhr(WS.JointPosEnc.LSR);
//    dms[LSR] = Oii.dq2dm_lhr(WS.JointPosEnc.LSR,WS.JointVel.LSR);


    ms[RHP]= Oii.q2m_hp(WS.JointPosEnc.RHP);
    dms[RHP] = Oii.dq2dm_hp(WS.JointPosEnc.RHP,WS.JointVel.RHP);
    ms[LHP]= Oii.q2m_hp(WS.JointPosEnc.LHP);
    dms[LHP] = Oii.dq2dm_hp(WS.JointPosEnc.LHP,WS.JointVel.LHP);
//    ms[RSP]= Oii.q2m_hp(WS.JointPosEnc.RSP);
//    dms[RSP] = Oii.dq2dm_hp(WS.JointPosEnc.RSP,WS.JointVel.RSP);
//    ms[LSP]= Oii.q2m_hp(WS.JointPosEnc.LSP);
//    dms[LSP] = Oii.dq2dm_hp(WS.JointPosEnc.LSP,WS.JointVel.LSP);


    ms[RKN]= Oii.q2m_kn(WS.JointPosEnc.RKN);
    dms[RKN] = Oii.dq2dm_kn(WS.JointPosEnc.RKN,WS.JointVel.RKN);
    ms[LKN]= Oii.q2m_kn(WS.JointPosEnc.LKN);
    dms[LKN] = Oii.dq2dm_kn(WS.JointPosEnc.LKN,WS.JointVel.LKN);
//    ms[REB]= Oii.q2m_kn(WS.JointPosEnc.REB);
//    dms[REB] = Oii.dq2dm_kn(WS.JointPosEnc.REB,WS.JointVel.REB);
//    ms[LEB]= Oii.q2m_kn(WS.JointPosEnc.LEB);
//    dms[LEB] = Oii.dq2dm_kn(WS.JointPosEnc.LEB,WS.JointVel.LEB);


    //cartesian space tracking
    //later....


    for(int i=0;i<NO_OF_JOINTS;i++)
    {
        Ains_xyz[i] +=Ains_xyz[i]*effMR*ddmrefs[i];//acc compensation
    }


    //joint space tracking
    int maxAlpha = 5;//0.01s

//    for(int i=0;i<NO_OF_JOINTS;i++)
//    {
//        fdms[i] = dmf[i].do_filt(dms[i]);
//        if(ms_old[i]<-900)
//        {
//            ms_old[i] = ms[i];
//        }
//        if(dms_old[i]<-900)
//        {
//            dms_old[i] = dms[i];
//        }
//        if(i==RHR||i==RSR)
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==RHR){Footnum = 0;}
//            if(i==RSR){Footnum = 2;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.HRG.Kp_swing; Kd = Gs.HRG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                //Kp = Kp_stand; Kd = Kd_stand;
//                Kp = Gs.HRG.Kp_stand; Kd = Gs.HRG.Kd_stand;
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;

//                aa = 0;

//            }//GGGGGG

//            mrefs[i] = Oii.q2m_rhr(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_rhr(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_rhr(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMR*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMP*ddmrefs[i];
//            Ains[i]+=effDR*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFR);
//        }
//        else if(i==LHR||i==LSR)
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==LHR){Footnum = 1;}
//            if(i==LSR){Footnum = 3;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.HRG.Kp_swing; Kd = Gs.HRG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                //Kp = Kp_stand; Kd = Kd_stand;
//                Kp = Gs.HRG.Kp_stand; Kd = Gs.HRG.Kd_stand;
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;
//                aa = 0;
//            }//GGGGGG

//            mrefs[i] = Oii.q2m_lhr(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_lhr(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_lhr(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMR*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMP*ddmrefs[i];
//            Ains[i]+=effDR*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFR);
//        }
//        else if(i==RHP||i==LHP||i==RSP||i==LSP)
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==RHP){Footnum = 0;}
//            if(i==LHP){Footnum = 1;}
//            if(i==RSP){Footnum = 2;}
//            if(i==LSP){Footnum = 3;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.HPG.Kp_swing; Kd = Gs.HPG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                //Kp = Kp_stand; Kd = Kd_stand;
//                Kp = Gs.HPG.Kp_swing; Kd = Gs.HPG.Kd_swing;
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;
//                aa = 0;
//            }//GGGGGG


//            mrefs[i] = Oii.q2m_hp(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_hp(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_hp(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMP*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMP*ddmrefs[i];
//            Ains[i]+=effDP*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFP);
//        }
//        else
//        {
//            double Kp, Kd;
//            int Footnum = 0;
//            if(i==RKN){Footnum = 0;}
//            if(i==LKN){Footnum = 1;}
//            if(i==REB){Footnum = 2;}
//            if(i==LEB){Footnum = 3;}
//            double aa;
//            if(FootonAir[Footnum])
//            {
//                Kp = Gs.KNG.Kp_swing; Kd = Gs.KNG.Kd_swing;
//                alphamref[i] = alphamref[i]-1;
//                if(alphamref[i]<0){alphamref[i] = 0;}
//                aa = alphamref[i]/maxAlpha;
//            }
//            else
//            {
//                Kp = Gs.KNG.Kp_stand; Kd = Gs.KNG.Kd_stand;//small gain for knee only
//                mrefs0[i] = ms[i];
//                alphamref[i] = maxAlpha;
//                for(int fn = 0;fn<4;fn++)
//                {
//                    if(Footnum==fn&&FootLandCnt[fn]>0)//early landing
//                    {
//                        //std::cout<<"ttat"<<std::endl;
//                        FootLandCnt[fn]--;
//                        Kp = Gs.KNGland.Kp_swing;Kd = Gs.KNGland.Kd_swing;
//                    }
//                }
//                aa = 0;
//            }//GGGGGG

//            mrefs[i] = Oii.q2m_kn(Qref[6+i])*(1-aa)+mrefs0[i]*aa;
//            dmrefs[i] = Oii.dq2dm_kn(Qref[6+i],dQref[6+i]);
//            ddmrefs[i] = Oii.ddq2ddm_kn(Qref[6+i],dQref[6+i],ddQref[6+i]);
//            Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-dms[i])+effMK*ddmrefs[i];
//            //Ains[i] = Kp*(mrefs[i]-ms[i])+Kd*(dmrefs[i]-fdms[i])+effMK*ddmrefs[i];
//            Ains[i]+=effDK*dmrefs[i];
//            Ains[i]+=fComp(dms[i],0.003,effFK);//front/////
//        }
//        if(Ains[i]>MaxA){Ains[i] = MaxA;}
//        if(Ains[i]<-MaxA){Ains[i] = -MaxA;}

//        if(i==RKN||i==LKN||i==REB||i==LEB||i==RHP||i==LHP||i==RSP||i==LSP)
//        {
//            if(fabs(ms[i]-ms_old[i])>0.01*100)
//            {
//                cout<<"encoder pos jump "<<ms[i]<<" "<<ms_old[i]<<" "<<i<<endl;
//                isWalking = false; save_all(2);
//                Ains[i] = 0;
//            }
//            if(fabs(dms[i]-dms_old[i])>1.0*100)
//            {
//                cout<<"encoder vel jump "<<dms[i]<<" "<<dms_old[i]<<" "<<i<<endl;
//                isWalking = false; save_all(2);
//                Ains[i] = 0;
//            }
//        }
//    }
//    QJ_CRef.RHR = Ains[RHR];
//    QJ_CRef.LHR = Ains[LHR];
//    QJ_CRef.RSR = Ains[RSR];
//    QJ_CRef.LSR = Ains[LSR];

//        QJ_CRef.RHP = Ains[RHP];
//        QJ_CRef.LHP = Ains[LHP];
//        QJ_CRef.RSP = Ains[RSP];
//        QJ_CRef.LSP = Ains[LSP];

//    QJ_CRef.RKN = Ains[RKN];
//    QJ_CRef.LKN = Ains[LKN];
//    QJ_CRef.REB = Ains[REB];
//    QJ_CRef.LEB = Ains[LEB];

    //calc JT-F from foot
    //maybe have to change from Qref to Qnow

//    OD.CalcEndeffectorJacobian3D(Qnow);
//    VectorNd FFoot = VectorNd::Zero(3);
//    FFoot[0] = -RFx_ref;
//    FFoot[1] = -RFy_ref;
//    FFoot[2] = -RFz_ref;
//    VectorNd TF = OD.JacobianRF3D.transpose()*FFoot;
//    TTF = TF;
//    TTF[6+RHR] = TF[6+RHR];
//    TTF[6+RHP] = TF[6+RHP];
//    TTF[6+RKN] = TF[6+RKN];

//    FFoot[0] = -LFx_ref;
//    FFoot[1] = -LFy_ref;
//    FFoot[2] = -LFz_ref;
//    TF = OD.JacobianLF3D.transpose()*FFoot;
//    TTF[6+LHR] = TF[6+LHR];
//    TTF[6+LHP] = TF[6+LHP];
//    TTF[6+LKN] = TF[6+LKN];

//    FFoot[0] = -RHx_ref;
//    FFoot[1] = -RHy_ref;
//    FFoot[2] = -RHz_ref;
//    TF = OD.JacobianRH3D.transpose()*FFoot;
//    TTF[6+RSR] = TF[6+RSR];
//    TTF[6+RSP] = TF[6+RSP];
//    TTF[6+REB] = TF[6+REB];

//    FFoot[0] = -LHx_ref;
//    FFoot[1] = -LHy_ref;
//    FFoot[2] = -LHz_ref;
//    TF = OD.JacobianLH3D.transpose()*FFoot;
//    TTF[6+LSR] = TF[6+LSR];
//    TTF[6+LSP] = TF[6+LSP];
//    TTF[6+LEB] = TF[6+LEB];

    //put result of QP here into TTF
    //how about ddq?
    if(Qcnt>10)//hmmmmmm
    {
        int ncon = 0;
        for(int i=0;i<4;i++){if(FootonAir[i]==false){ncon++;}}
        if(Fcon.size()==ncon*3)
        {
            for(int i=0;i<4;i++)
            {
//                if(FootonAir[i]==false)
//                {
                    TTF[6+i*3 +0] = QP_Tauout[i*3 +0];
                    TTF[6+i*3 +1] = QP_Tauout[i*3 +1];
                    TTF[6+i*3 +2] = QP_Tauout[i*3 +2];
//                }
//                else
//                {
//                    TTF[6+i*3 +0] = 0;
//                    TTF[6+i*3 +1] = 0;
//                    TTF[6+i*3 +2] = 0;
//                }
            }
        }
        else
        {
            TTF = oTTF;
        }
//        double MRoll = 45*M_PI/180;
//        for(int i=0;i<4;i++)
//        {
//            RollQP[i] = RollQP[i] + dRollQP[i]*dt + QP_ddQnow[i*3]*dt*dt*0.5;
//            dRollQP[i] = dRollQP[i] +  QP_ddQnow[i*3]*dt;
//            if(RollQP[i] > MRoll){RollQP[i] = MRoll;}
//            if(RollQP[i] < -MRoll){RollQP[i] = -MRoll;}
//        }
    }
    else
    {
//        QuadJoints QJrr = Oi.IK_COM(QP_controled);
//        RollQP[0] = QJrr.RHR;
//        RollQP[1] = QJrr.LHR;
//        RollQP[2] = QJrr.RSR;
//        RollQP[3] = QJrr.LSR;
//        dRollQP[0] = 0;
//        dRollQP[1] = 0;
//        dRollQP[2] = 0;
//        dRollQP[3] = 0;
    }
    oTTF = TTF;

    double tu = 1.0;
    //lets tune this using cpe vlaue
    QJ_CFF.RHR = tu*Oii.T2CR*(TTF[6+RHR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+RHR])));
    QJ_CFF.RHP = tu*Oii.T2CP*(TTF[6+RHP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+RHP])));
    QJ_CFF.RKN = tu*Oii.T2CK*(TTF[6+RKN]*Oii.J_kn(Oii.q2m_kn(Qnow[6+RKN])));

    QJ_CFF.LHR = tu*Oii.T2CR*(TTF[6+LHR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+LHR])));
    QJ_CFF.LHP = tu*Oii.T2CP*(TTF[6+LHP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+LHP])));
    QJ_CFF.LKN = tu*Oii.T2CK*(TTF[6+LKN]*Oii.J_kn(Oii.q2m_kn(Qnow[6+LKN])));

//    QJ_CFF.RSR = tu*Oii.T2CR*(TTF[6+RSR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+RSR])));
//    QJ_CFF.RSP = tu*Oii.T2CP*(TTF[6+RSP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+RSP])));
//    QJ_CFF.REB = tu*Oii.T2CK*(TTF[6+REB]*Oii.J_kn(Oii.q2m_kn(Qnow[6+REB])));

//    QJ_CFF.LSR = tu*Oii.T2CR*(TTF[6+LSR]*Oii.J_hp(Oii.q2m_rhr(Qnow[6+LSR])));
//    QJ_CFF.LSP = tu*Oii.T2CP*(TTF[6+LSP]*Oii.J_hp(Oii.q2m_hp(Qnow[6+LSP])));
//    QJ_CFF.LEB = tu*Oii.T2CK*(TTF[6+LEB]*Oii.J_kn(Oii.q2m_kn(Qnow[6+LEB])));




}
