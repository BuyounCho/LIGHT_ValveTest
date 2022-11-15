#include "ow_quad.h"
#include "OW_RBDL.h"
//ddcom_qp : not controller, feedforward
//input ->ddRF,ddLF,ddRH,ddLH,ddqpel(maybe?),contacts
//output ->ddcom

//X = [q; tau; F]; (18+12+6or12) ->36or 42
//constraint = 18<-FB dynamics
//high weight -> swing leg, stance leg, body(ddRF,ddLF,ddRH,ddLH,ddqpel)
//3+3+3+3+3 ->15

//if trot, 2 leg stance,
//18+15->32 is fixed
//3 left
//ddcomz is fixed to be zero
//2left
// ddcomx, ddcomy is not fixed, should be optimized? or solution space exist.
//so, qp or just matrix nullspace is enough?

//lets choose faster one.

//if 4 leg stance
//just pattern is enough, just tripping test is enough

void OW_Quad::calc_ff_tau_ddq_QP()
{
    QP_QP = QP;
    for(int i=0;i<4;i++)
    {
        QPFcon[i] = !FootonAir[i];
    }

    QP_Qref = ODQP.Joints2Q(Oi.IK_COM(QP_QP));

    if(QPFcon[0]){QP_QP.dRF = QP_QP.ddRF = vec3(0,0,0);}
    if(QPFcon[1]){QP_QP.dLF = QP_QP.ddLF = vec3(0,0,0);}
    if(QPFcon[2]){QP_QP.dRH = QP_QP.ddRH = vec3(0,0,0);}
    if(QPFcon[3]){QP_QP.dLH = QP_QP.ddLH = vec3(0,0,0);}

    calc_QPdQref(QP_QP.dCOM,QP_QP.dqPel,QP_QP.dRF,QP_QP.dLF,QP_QP.dRH,QP_QP.dLH);
    calc_QPddQref(QP_QP.ddCOM,QP_QP.ddqPel,QP_QP.ddRF,QP_QP.ddLF,QP_QP.ddRH,QP_QP.ddLH);


    int mode = XYPOS;
    QP_wnzW0s ws;
    if(GetWP().Gait==Trot)
    {
        mode = XYVEL;
        ws.rotx.wn = 30;
        ws.rotx.z = 1.5;
        ws.roty.wn = 30;
        ws.roty.z = 1.5;
        ws.rotz.wn = 30;
        ws.rotz.z = 1.0;

        ws.pelz.wn = 18;
        ws.pelz.z = 0.2;
        ws.pelxy.wn = 6;
        ws.pelxy.z = 0.2;

        //ws.dpelxy.wn = 0.6;//have to tune with ff, ll, and ZMPoff.....
        ws.dpelxy.wn = 2.0;//have to tune with ff, ll, and ZMPoff.....
        ws.dpelxy.z = 1;
        ws.dpelxy.W0 = 0.3;

    }//this works
    if(GetWP().Gait==Standing)//hmm.... need tuning
    {
        mode = XYPOS;

        ws.rotx.wn = 30;
        ws.rotx.z = 0.7;
        ws.roty.wn = 30;
        ws.roty.z = 0.7;
        ws.rotz.wn = 30;
        ws.rotz.z = 0.7;

        ws.pelz.wn = 18;
        ws.pelz.z = 0.2;
        ws.pelxy.wn = 6;
        ws.pelxy.z = 0.2;

//        ws.rotx.wn = 0.5;
//        ws.rotx.z = 1;
//        ws.roty.wn = 0.5;
//        ws.roty.z = 1;
//        ws.rotz.wn = 0.5;
//        ws.rotz.z = 1;

//        ws.pelz.wn = 0.7;
//        ws.pelz.z = 1;
//        ws.pelxy.wn = 0.7;
//        ws.pelxy.z = 1;
    }
    if(GetWP().Gait==Pronk)
    {
        int ncon = 0;
        for(int i=0;i<4;i++)
        {
            if(QPFcon[i]==true)
            {
                ncon++;
            }
        }
        if(ncon==4)
        {
            mode = XYPOS;
            ws.rotx.wn = 30;
            ws.rotx.z = 1;
            ws.roty.wn = 30;
            ws.roty.z = 1;
            ws.rotz.wn = 30;
            ws.rotz.z = 1;

            ws.pelz.wn = 20;
            ws.pelz.z = 0.7;
            ws.pelxy.wn = 10;//hard to increase
            ws.pelxy.z = 0.2;//hard to increase
        }
    }
    if(GetWP().Gait==Flytrot)
    {
        int ncon = 0;
        for(int i=0;i<4;i++)
        {
            if(QPFcon[i]==true)
            {
                ncon++;
            }
        }
        if(ncon==4)
        {
            mode = XYPOS;
            ws.rotx.wn = 30;
            ws.rotx.z = 1;
            ws.roty.wn = 30;
            ws.roty.z = 1;
            ws.rotz.wn = 30;
            ws.rotz.z = 1;

            ws.pelz.wn = 20;
            ws.pelz.z = 0.7;
            ws.pelxy.wn = 10;//hard to increase
            ws.pelxy.z = 0.2;//hard to increase
        }
        if(ncon==2)
        {
            mode = XYVEL;
            ws.rotx.wn = 13;
            ws.rotx.z = 1;
            ws.roty.wn = 13;
            ws.roty.z = 1;
            ws.rotz.wn = 13;
            ws.rotz.z = 1;

            ws.pelz.wn = 20;
            ws.pelz.z = 0.7;
            ws.dpelxy.wn = 2.0;//2.0;//have to tune with ff, ll, and ZMPoff.....
            ws.dpelxy.z = 1;
            ws.dpelxy.W0 = 0.3;

            mode = XYVEL_MPC;
            double stanceT = (1-flyRatio)*WP.step_T;

            double flyT = WP.step_T-stanceT;
            ODQP.flyT = flyT;
            ODQP.MPC_R.x = 1*0.7;//0.7increase roll gain
            ODQP.MPC_R.y = 1;//1.0
            ODQP.MPC_R.z = 1;

            ODQP.MPC_Qx.x = 1e3;
            ODQP.MPC_Qv.x = 1e2;//hard to tune.....

            ODQP.MPC_Qx.y = 1e3;
            ODQP.MPC_Qv.y = 1e2;//hard to tune.....

            ODQP.MPC_Qx.z = 1e3;
            ODQP.MPC_Qv.z = 0.5*1e2;//hard to tune.....
            ODQP.stanceN = (stanceT-t_now)/0.02;//0.02 is now fixed in MPC

        }
    }

    if(GetWP().Gait==Wave||GetWP().Gait==Wave2)
    {
        int ncon = 0;
        for(int i=0;i<4;i++)
        {
            if(QPFcon[i]==true)
            {
                ncon++;
            }
        }
       if(ncon==2)
       {
           mode = XYVEL;

           ws.rotx.wn = 30;
           ws.rotx.z = 1.0;
           ws.roty.wn = 30;
           ws.roty.z = 1.0;
           ws.rotz.wn = 30;
           ws.rotz.z = 1.0;

           ws.pelz.wn = 18;
           ws.pelz.z = 0.2;
           ws.pelxy.wn = 6;
           ws.pelxy.z = 0.2;

           //ws.dpelxy.wn = 0.6;//have to tune with ff, ll, and ZMPoff.....
           ws.dpelxy.wn = 2.0;//have to tune with ff, ll, and ZMPoff.....
           ws.dpelxy.z = 1;
           ws.dpelxy.W0 = 0.3;

       }
       else
       {
           mode = XYPOS;
           ws.rotx.wn = 30;
           ws.rotx.z = 0.7;
           ws.roty.wn = 30;
           ws.roty.z = 0.7;
           ws.rotz.wn = 30;
           ws.rotz.z = 0.7;

           ws.pelz.wn = 18;
           ws.pelz.z = 0.2;
           ws.pelxy.wn = 6;
           ws.pelxy.z = 0.2;
       }
    }//this works


    if(NO_ROBOT_TEST)
    {
        QP_Qnow = QP_Qref;
        QP_dQnow = QP_dQref;
    }
    else
    {
        QP_Qnow = ODQP.Joints2Q(WS.JointPosEnc);
        QP_dQnow = ODQP.Joints2dQ(WS.JointVel);
    }
    //contact foot pelvis est
    int ncon = 0;
    for(int i=0;i<4;i++){if(QPFcon[i]){ncon++;}}
    if(ncon==0)//flying
    {
        QP_Qnow[0] = QP_Qref[0];
        QP_Qnow[1] = QP_Qref[1];
        QP_Qnow[2] = QP_Qref[2];
        QP_dQnow[0] = QP_dQref[0];
        QP_dQnow[1] = QP_dQref[1];//hmm
        QP_dQnow[2] = QP_dQref[2];
    }
    else
    {
        QuadPos QRP = QP_QP;
        QuadPos QEP = Oi.FK(WS.JointPosEnc);
        if(NO_ROBOT_TEST)
        {
            QEP = QRP;
        }
        vec3 mR(0,0,0);
        vec3 mE(0,0,0);
        if(QPFcon[0]) {mR = mR + QRP.pRF/ncon; mE = mE + QEP.pRF/ncon;}
        if(QPFcon[1]) {mR = mR + QRP.pLF/ncon; mE = mE + QEP.pLF/ncon;}
        if(QPFcon[2]) {mR = mR + QRP.pRH/ncon; mE = mE + QEP.pRH/ncon;}
        if(QPFcon[3]) {mR = mR + QRP.pLH/ncon; mE = mE + QEP.pLH/ncon;}
        vec3 pelE(QP_Qnow[0],QP_Qnow[1],QP_Qnow[2]);
        //mR = mE + trans;
        //realPel = pelE + trans;
        vec3 trans = mR-mE;
        QP_Qnow[0] = pelE[0] + trans[0];
        QP_Qnow[1] = pelE[1] + trans[1];
        QP_Qnow[2] = pelE[2] + trans[2];

        ODQP.CalcEndeffectorJacobian3D(QP_Qnow);

        vec3 dmR(0,0,0);//should be always zero;
        vec3 dmE(0,0,0);
        VectorNd vRF = ODQP.JacobianRF3D*QP_dQnow;
        VectorNd vLF = ODQP.JacobianLF3D*QP_dQnow;
        VectorNd vRH = ODQP.JacobianRH3D*QP_dQnow;
        VectorNd vLH = ODQP.JacobianLH3D*QP_dQnow;
        if(QPFcon[0])  {dmE = dmE + vec3(vRF[0],vRF[1],vRF[2])/ncon;}
        if(QPFcon[1])  {dmE = dmE + vec3(vLF[0],vLF[1],vLF[2])/ncon;}
        if(QPFcon[2])  {dmE = dmE + vec3(vRH[0],vRH[1],vRH[2])/ncon;}
        if(QPFcon[3])  {dmE = dmE + vec3(vLH[0],vLH[1],vLH[2])/ncon;}

        vec3 dpelE(QP_dQnow[0],QP_dQnow[1],QP_dQnow[2]);
        //dmR = dmE + dtrans;
        //realPel = dpelE + dtrans;
        vec3 dtrans = dmR-dmE;
        QP_dQnow[0] = dpelE[0] + dtrans[0];
        QP_dQnow[1] = dpelE[1] + dtrans[1];
        QP_dQnow[2] = dpelE[2] + dtrans[2];

        if(QPsolved==false)
        {
            QP_dQref[0] = 0;
            QP_dQref[1] = 0;
            QP_dQref[2] = 0;
        }
        else
        {
            QP_dQref[0] = (QP_Qref[0]-opPel[0])*250;
            QP_dQref[1] = (QP_Qref[1]-opPel[1])*250;
            QP_dQref[2] = (QP_Qref[2]-opPel[2])*250;
        }
        opPel = vec3(QP_Qref[0],QP_Qref[1],QP_Qref[2]);
        QPsolved = true;

    }

    vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
    FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
    vec3 FrontVector = FrontVt.normalize();//not really...
    vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();
    vec3 sL = FrontVector*(WP.step_L.x+ff*0.2)+LeftVector*(WP.step_L.y+ll*0.2);
    //hmmm....
    //vec3 sL = FrontVector*(WP.step_L.x)+LeftVector*(WP.step_L.y);
    vec3 sV = sL/WP.step_T;
    //gains should be tuned inside ODQP.calc_QP
    VectorNd ddQ_Tau_F = ODQP.calc_QP(QP_Qnow,QP_Qref,QP_dQnow,QP_dQref,QP_ddQref,
                                    QPFcon[0],QPFcon[1],QPFcon[2],QPFcon[3],mode,sV,ws);
    VectorNd TQPt = ddQ_Tau_F.segment(QP_dQnow.size(),12);
    bool QPok = true;
    for(int i=0;i<12;i++)
    {
        if(TQPt[i]>200||TQPt[i]<-200)//near limit
        {
            cout<<"QPErr "<<i<<endl;
            cout<<"cnt "<<Qcnt<<endl;
            cout<<"time "<<Qcnt*dt<<endl;
            cout<<"TQPt[i] "<<TQPt[i]<<endl;
//            cout<<"Qnow "<<endl<<QP_Qnow<<endl;
//            cout<<"Qref "<<endl<<QP_Qref<<endl;
//            cout<<"dQnow "<<endl<<QP_dQnow<<endl;
//            cout<<"dQref "<<endl<<QP_dQref<<endl;
            QPok = false;
            break;
        }
    }
    if(QPok==true)
    {
        QP_ddQnow = ddQ_Tau_F.segment(0,QP_dQnow.size());
        QP_Tauout = ddQ_Tau_F.segment(QP_dQnow.size(),12);
        Fcon = ddQ_Tau_F.segment(30,ddQ_Tau_F.size()-30);
    }
    vec3 tddCOM(0,0,0);
    for(int i=0;i<Fcon.size();i++)
    {
        tddCOM[i%3] = tddCOM[i%3] + Fcon[i]/Oi.M_total;
    }
    tddCOM.z = 0;

    QP_ddCOM = tddCOM;//+ddQpel + COMz?
    QPcnt++;

    QP_Qnow_save = QP_Qnow;
    QP_dQnow_save = QP_dQnow;

}
void OW_Quad::calc_QPddQref(vec3 _ddpCOMref, vec3 ddqPELref, vec3 ddpRFref, vec3 ddpLFref, vec3 ddpRHref, vec3 ddpLHref)
{
    //            ODQP.CalcEndeffectorJacobian3D(Qref);
    //            ODQP.CalcCOMJacobian3D(Qref);

    MatrixNd Jqref = MatrixNd::Zero(18,18);
    Jqref.block(0,0,3,18) = ODQP.JacobianCOMall3D;
    Jqref.block(3,3,3,3) = MatrixNd::Identity(3,3);
    Jqref.block(6,0,3,18) = ODQP.JacobianRF3D;
    Jqref.block(9,0,3,18) = ODQP.JacobianLF3D;
    Jqref.block(12,0,3,18) = ODQP.JacobianRH3D;
    Jqref.block(15,0,3,18) = ODQP.JacobianLH3D;

    VectorNd ddXref = VectorNd::Zero(18);
    ddXref.segment(0,3) = Vector3d(_ddpCOMref.x,_ddpCOMref.y,_ddpCOMref.z);
    ddXref.segment(3,3) = Vector3d(ddqPELref.x,ddqPELref.y,ddqPELref.z);
    ddXref.segment(6,3) = Vector3d(ddpRFref.x,ddpRFref.y,ddpRFref.z);
    ddXref.segment(9,3) = Vector3d(ddpLFref.x,ddpLFref.y,ddpLFref.z);
    ddXref.segment(12,3) = Vector3d(ddpRHref.x,ddpRHref.y,ddpRHref.z);
    ddXref.segment(15,3) = Vector3d(ddpLHref.x,ddpLHref.y,ddpLHref.z);

    VectorNd ddqZero = VectorNd::Zero(18);
    Vector3d dJdqpCOM = (Oi.m_torso * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_torso,Vector3d(Oi.c_torso.x,Oi.c_torso.y,Oi.c_torso.z))
                         + Oi.m_rhr * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_rhr,Vector3d(Oi.c_rhr.x,Oi.c_rhr.y,Oi.c_rhr.z))
                         + Oi.m_rhp * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_rhp,Vector3d(Oi.c_rhp.x,Oi.c_rhp.y,Oi.c_rhp.z))
                         + Oi.m_rkn * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_rkn,Vector3d(Oi.c_rkn.x,Oi.c_rkn.y,Oi.c_rkn.z))

                         + Oi.m_lhr * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_lhr,Vector3d(Oi.c_lhr.x,Oi.c_lhr.y,Oi.c_lhr.z))
                         + Oi.m_lhp * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_lhp,Vector3d(Oi.c_lhp.x,Oi.c_lhp.y,Oi.c_lhp.z))
                         + Oi.m_lkn * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_lkn,Vector3d(Oi.c_lkn.x,Oi.c_lkn.y,Oi.c_lkn.z))

                         + Oi.m_rsr * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_rsr,Vector3d(Oi.c_rsr.x,Oi.c_rsr.y,Oi.c_rsr.z))
                         + Oi.m_rsp * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_rsp,Vector3d(Oi.c_rsp.x,Oi.c_rsp.y,Oi.c_rsp.z))
                         + Oi.m_reb * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_reb,Vector3d(Oi.c_reb.x,Oi.c_reb.y,Oi.c_reb.z))

                         + Oi.m_lsr * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_lsr,Vector3d(Oi.c_lsr.x,Oi.c_lsr.y,Oi.c_lsr.z))
                         + Oi.m_lsp * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_lsp,Vector3d(Oi.c_lsp.x,Oi.c_lsp.y,Oi.c_lsp.z))
                         + Oi.m_leb * CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_leb,Vector3d(Oi.c_leb.x,Oi.c_leb.y,Oi.c_leb.z)))/Oi.M_total;
    Vector3d dJdqqPel = Vector3d::Zero(3); //dJ/dt = 0
    Vector3d dJdqRF = CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_rkn,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));
    Vector3d dJdqLF = CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_lkn,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));
    Vector3d dJdqRH = CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_reb,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));
    Vector3d dJdqLH = CalcPointAcceleration(*(ODQP.Robot),QP_Qref,QP_dQref,ddqZero,ODQP.n_leb,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));

    VectorNd dJdq = VectorNd::Zero(18);
    dJdq.segment(0,3) = dJdqpCOM;
    dJdq.segment(3,3) = dJdqqPel;
    dJdq.segment(6,3) = dJdqRF;
    dJdq.segment(9,3) = dJdqLF;
    dJdq.segment(12,3) = dJdqRH;
    dJdq.segment(15,3) = dJdqLH;

    QP_ddQref = Jqref.inverse()*ddXref - dJdq;
}
void OW_Quad::calc_QPdQref(vec3 _dpCOMref, vec3 dqPELref, vec3 dpRFref, vec3 dpLFref, vec3 dpRHref, vec3 dpLHref)
{
    ODQP.CalcEndeffectorJacobian3D(QP_Qref);
    ODQP.CalcCOMJacobian3D(QP_Qref);

    MatrixNd Jqref = MatrixNd::Zero(18,18);
    Jqref.block(0,0,3,18) = ODQP.JacobianCOMall3D;
    Jqref.block(3,3,3,3) = MatrixNd::Identity(3,3);
    Jqref.block(6,0,3,18) = ODQP.JacobianRF3D;
    Jqref.block(9,0,3,18) = ODQP.JacobianLF3D;
    Jqref.block(12,0,3,18) = ODQP.JacobianRH3D;
    Jqref.block(15,0,3,18) = ODQP.JacobianLH3D;

    VectorNd dXref = VectorNd::Zero(18);
    dXref.segment(0,3) = Vector3d(_dpCOMref.x,_dpCOMref.y,_dpCOMref.z);
    dXref.segment(3,3) = Vector3d(dqPELref.x,dqPELref.y,dqPELref.z);
    dXref.segment(6,3) = Vector3d(dpRFref.x,dpRFref.y,dpRFref.z);
    dXref.segment(9,3) = Vector3d(dpLFref.x,dpLFref.y,dpLFref.z);
    dXref.segment(12,3) = Vector3d(dpRHref.x,dpRHref.y,dpRHref.z);
    dXref.segment(15,3) = Vector3d(dpLHref.x,dpLHref.y,dpLHref.z);

    QP_dQref = Jqref.inverse()*dXref;

}

void OW_Quad::test_MPC()
{
    int N = 20;//okmaybe
    double _dt = 0.015;
    double maxu = 3;

    ow_onedofMPC MM(N,_dt,maxu);

    vec3 x0(0.3,0,0);
    std::vector<vec3> ref;
    ref.push_back(vec3());
    vec3 x = x0;
    int stanceN = 10;
    double flyT = 0.4;
    double R = 1;
    double Qx = 1e3;
    double Qv = 1e2;
    RTIME temptime = rt_timer_read();

    for(int i=0;i<1000;i++)
    {
        x[2] = MM.getddx(x,ref,stanceN,flyT,R,Qx,Qv,maxu);
        x[0] = x[0]+x[1]*_dt+x[2]*0.5*_dt*_dt;
        x[1] = x[1]+x[2]*_dt;
        stanceN--;
        if(stanceN==0)
        {
            stanceN = 10;
            x[2] = 0;
            x[0] = x[0]+x[1]*flyT;
        }
        if(i%13==0)
        {
           // std::cout<<"x "<<x[0]<<" "<<x[1]<<" "<<x[2]<<std::endl;
        }
        if(i==552)
        {x[1] = 0.7;}
    }
    RTIME temptime2 = rt_timer_read();
    double calcms = (temptime2-temptime)*0.001*0.001;
    printf("1000MPC %f 1MPC %f\n",calcms,calcms/1000);

}
