
#include "ow_quad.h"

QuadJoints OW_Quad::Standing_onestep(WalkSensors _WS)
{
    WS = _WS;
    if(Qcnt<2)
    {
        cout<<"changetoCcon"<<endl;
        if(NO_ROBOT_TEST==false)
        {
            changetoCcon[0] = true;
            changetoCcon[1] = true;
            changetoCcon[2] = true;
            changetoCcon[3] = true;
        }
    }
    delZ = QP.pCOM.z;
    w = sqrtp(g/delZ);
    if(delZ>des_delZ+0.001)////0.01m/s
    {
        QP.pCOM.z = QP.pCOM.z - 0.01*dt;
        if(Qcnt%250==0)
        {cout<<"COMdown"<<endl;}
    }
    if(delZ<des_delZ-0.001)
    {
        QP.pCOM.z = QP.pCOM.z + 0.01*dt;
        if(Qcnt%250==0)
        {cout<<"COMup"<<endl;}
    }

    QP.pRF.z = 0.99*QP.pRF.z;
    QP.pLF.z = 0.99*QP.pLF.z;
    QP.pRH.z = 0.99*QP.pRH.z;
    QP.pLH.z = 0.99*QP.pLH.z;//may not enough. how to prevent slip?

    t_now = 0;//always zero
    ddYawcon = dYawcon = 0;

    if(isFinishing)
    {
        //later finishcnt should be added for additional control
        if(NO_ROBOT_TEST==false)
        {
            changetoPcon[0] = true;
            changetoPcon[1] = true;
            changetoPcon[2] = true;
            changetoPcon[3] = true;
        }
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
//        isCOMadjusting_finish = true;
//        Qcnt = 0;
//        t_now = 0;
        save_all();
        return QJenc;
    }
    bool oknextGait = (WP_next.Gait==Standing)||(WP_next.Gait==Trot)||(WP_next.Gait==Wave)||(WP_next.Gait==Wave2);
    if(walkchanged&&oknextGait)
    {
        if(WP.Gait!=WP_next.Gait)
        {
            desCOM = QP.pCOM;
            sumCOMerr = vec3();
            for(int i=0;i<5;i++)
            {
                stepLFilter[i] = vec3();
            }
        }
        if(WP_next.Gait==Wave){isFirststep = true;}//need check111111111111111111111}
        WP = WP_next;
        des_delZ = WP.delZ;
        walkchanged  = false;
    }
    fallcheck();
    FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
    FFz_ref();
    STAND_control();

//    if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
//    {

//        DSP_control_slow();
//    }
//    else
//    {
//        FFz_ref_ssp();
//        SSP_control_slow();
//    }

    save_onestep(Qcnt);
    Qcnt++;

    QuadJoints QJout = Oi.IK_COM(QP_controled);

    QJout = adjust_FootZs(QJout);//lets try more
    return QJout;

}
void OW_Quad::STAND_control()
{
    //first, same with DSP control
    double Kpy = 0.02*R2Df*2;
    double Kdy = 0.001*R2Df;
    double Kpx = 0.01*R2Df*2;
    double Kdx = 0.0005*R2Df;

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
    double Kpyf = wn*wn*Iy/FB_L*40;//20
    double Kdyf = 2*wn*z*Iy/FB_L*15*0.5;//15
    double Kpxf = w*wn*Ix/LR_L*40;//20
    double Kdxf = 2*wn*z*Ix/LR_L*15*0.5;//15
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


    if(NO_ROBOT_TEST)
    {
        COMconF = vec3();
    }


    RFx_ref+=COMconF.x*0.25;
    LFx_ref+=COMconF.x*0.25;
    RHx_ref+=COMconF.x*0.25;
    LHx_ref+=COMconF.x*0.25;
    RFy_ref+=COMconF.y*0.25;
    LFy_ref+=COMconF.y*0.25;
    RHy_ref+=COMconF.y*0.25;
    LHy_ref+=COMconF.y*0.25;

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
