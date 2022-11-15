#include "ow_quad.h"

QuadJoints OW_Quad::Demo_onestep(WalkSensors _WS)
{
    WS = _WS;
//    if(Qcnt<2)
//    {
//        cout<<"changetoCcon"<<endl;
//        if(NO_ROBOT_TEST==false)
//        {
//            changetoCcon[0] = true;
//            changetoCcon[1] = true;
//            changetoCcon[2] = true;
//            changetoCcon[3] = true;
//        }
//    }
    //POSITION_CONTROL_ONLY

    if(!NO_ROBOT_TEST)
    {
        fallcheck();
    }
    if(Qcnt<2)
    {
        RFref.setX(QP.pRF);
        RFref.dXddX0();
        LFref.setX(QP.pLF);
        LFref.dXddX0();
        RHref.setX(QP.pRH);
        RHref.dXddX0();
        LHref.setX(QP.pLH);
        LHref.dXddX0();
        COMrefXdXddX.setX(QP.pCOM);
        COMrefXdXddX.dXddX0();
        Angleref.setX(vec3());
        Angleref.dXddX0();
    }
    int demo_max = 50;
    int demo_cnt[demo_max];
    demo_cnt[0] = 100;
    int cntperphase = 300;//700;
    for(int i=1;i<demo_max;i++)
    {
        demo_cnt[i]= demo_cnt[i-1]+cntperphase;
    }
    double LD = 0.2;
    double LH = 0.45;
    double L0 = 0.36;
    double PA = 30*D2Rf;
    double PR = 30*D2Rf;
    double PY = 30*D2Rf;
    vec3 midpoint = 0.25*(QP.pRF+QP.pLF+QP.pRH+QP.pLH);
    if(Qcnt<demo_cnt[0])
    {
        t_now = 0;
        //do_nothing
    }
    else if(Qcnt<demo_cnt[1])
    {//down
        double tt = (demo_cnt[1])*dt;
        vec3 rr = vec3(midpoint.x,midpoint.y,LD);
        COMrefXdXddX  = calc_nextRef(Qcnt*dt,tt,COMrefXdXddX,rr);
    }
    else if(Qcnt<demo_cnt[2])
    {//up
        double tt = (demo_cnt[2])*dt;
        vec3 rr = vec3(midpoint.x,midpoint.y,LH);
        COMrefXdXddX  = calc_nextRef(Qcnt*dt,tt,COMrefXdXddX,rr);
    }
    else if(Qcnt<demo_cnt[3])
    {//down
        double tt = (demo_cnt[3])*dt;
        vec3 rr = vec3(midpoint.x,midpoint.y,LD);
        COMrefXdXddX  = calc_nextRef(Qcnt*dt,tt,COMrefXdXddX,rr);
    }
    else if(Qcnt<demo_cnt[4])
    {//up
        double tt = (demo_cnt[4])*dt;
        vec3 rr = vec3(midpoint.x,midpoint.y,LH);
        COMrefXdXddX  = calc_nextRef(Qcnt*dt,tt,COMrefXdXddX,rr);
    }
    else if(Qcnt<demo_cnt[5])
    {//nominal
        double tt = (demo_cnt[5])*dt;
        vec3 rr = vec3(midpoint.x,midpoint.y,L0);
        COMrefXdXddX  = calc_nextRef(Qcnt*dt,tt,COMrefXdXddX,rr);
    }
    else if(Qcnt<demo_cnt[6])
    {//up
        double tt = (demo_cnt[6])*dt;
        vec3 rr = vec3(0,PA,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[7])
    {//down
        double tt = (demo_cnt[7])*dt;
        vec3 rr = vec3(0,-PA,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[8])
    {//up
        double tt = (demo_cnt[8])*dt;
        vec3 rr = vec3(0,PA,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[9])
    {//down
        double tt = (demo_cnt[9])*dt;
        vec3 rr = vec3(0,-PA,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[10])
    {//nominal
        double tt = (demo_cnt[10])*dt;
        vec3 rr = vec3(0,0,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[11])
    {//up
        double tt = (demo_cnt[11])*dt;
        vec3 rr = vec3(PR,0,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[12])
    {//down
        double tt = (demo_cnt[12])*dt;
        vec3 rr = vec3(-PR,0,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[13])
    {//up
        double tt = (demo_cnt[13])*dt;
        vec3 rr = vec3(PR,0,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[14])
    {//down
        double tt = (demo_cnt[14])*dt;
        vec3 rr = vec3(-PR,0,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[15])
    {//nominal
        double tt = (demo_cnt[15])*dt;
        vec3 rr = vec3(0,0,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[16])
    {//up
        double tt = (demo_cnt[16])*dt;
        vec3 rr = vec3(0,0,PY);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[17])
    {//down
        double tt = (demo_cnt[17])*dt;
        vec3 rr = vec3(0,0,-PY);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[18])
    {//up
        double tt = (demo_cnt[18])*dt;
        vec3 rr = vec3(0,0,PY);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[19])
    {//down
        double tt = (demo_cnt[19])*dt;
        vec3 rr = vec3(0,0,-PY);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else if(Qcnt<demo_cnt[20])
    {//nominal
        double tt = (demo_cnt[20])*dt;
        vec3 rr = vec3(0,0,0);
        Angleref  = calc_nextRef(Qcnt*dt,tt,Angleref,rr);
    }
    else
    {//finish
        QuadJoints QJenc = WS.JointPosEnc;
        isWalking = false;
        NOSAVE = true;
        save_all();
        return QJenc;
    }

    QP.qPel = mat3(quat(mat3(vec3(0,0,-1),Angleref.X().z))
            *quat(mat3(vec3(0,-1,0),Angleref.X().y))
            *quat(mat3(vec3(-1,0,0),Angleref.X().x)));
    QP.dqPel = Angleref.dX();//not precise
    QP.ddqPel = Angleref.ddX();//not precise


    QP.pCOM = COMrefXdXddX.X();
    QP.dCOM = COMrefXdXddX.dX();
    QP.ddCOM = COMrefXdXddX.ddX();

    QP.pRF = RFref.X();
    QP.pLF = LFref.X();
    QP.pRH = RHref.X();
    QP.pLH = LHref.X();

    QP.dRF = RFref.dX();
    QP.dLF = LFref.dX();
    QP.dRH = RHref.dX();
    QP.dLH = LHref.dX();

    QP.ddRF = RFref.ddX();
    QP.ddLF = LFref.ddX();
    QP.ddRH = RHref.ddX();
    QP.ddLH = LHref.ddX();



//make-motion to here



     DEMO_control();


    save_onestep(Qcnt);
    Qcnt++;
    t_now=0;

    QuadJoints QJout = Oi.IK_COM(QP_controled);


    QJout = adjust_FootZs(QJout);//lets try more

//    QJout.RHR = RollQP[0];
//    QJout.LHR = RollQP[1];
//    QJout.RSR = RollQP[2];
//    QJout.LSR = RollQP[3];

    return QJout;

}


void OW_Quad::DEMO_control()
{
    //NO_CONTROL


    dFootZctRef[0] = 0;
    dFootZctRef[1] = 0;
    dFootZctRef[2] = 0;
    dFootZctRef[3] = 0;

    for(int i=0;i<4;i++)
    {
        FootZctRef[i] = FootZctRef[i]+dFootZctRef[i]*dt;
        double MaxCon = 0.05;
        if(FootZctRef[i]>MaxCon){FootZctRef[i] = MaxCon; if(dFootZctRef[i]>0){dFootZctRef[i] = 0;}}
        if(FootZctRef[i]<-MaxCon){FootZctRef[i] = -MaxCon; if(dFootZctRef[i]<0){dFootZctRef[i] = 0;}}
        FootZctFT[i] = FootZctRef[i];
    }



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
