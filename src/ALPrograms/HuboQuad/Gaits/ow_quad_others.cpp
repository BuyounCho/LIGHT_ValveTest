#include "ow_quad.h"
void OW_Quad::COM_ZERO()
{
    QP.pPel.z -=QP.pRF.z;
    QP.pCOM.z -=QP.pRF.z;
    QP.pLF.z -=QP.pRF.z;
    QP.pRH.z -=QP.pRF.z;
    QP.pLH.z -=QP.pRF.z;
    QP.pRF.z = 0;

    QP.pPel.x -=QP.pCOM.x;
    QP.pPel.y -=QP.pCOM.y;
    QP.pRF.x -=QP.pCOM.x;
    QP.pRF.y -=QP.pCOM.y;
    QP.pLF.x -=QP.pCOM.x;
    QP.pLF.y -=QP.pCOM.y;
    QP.pRH.x -=QP.pCOM.x;
    QP.pRH.y -=QP.pCOM.y;
    QP.pLH.x -=QP.pCOM.x;
    QP.pLH.y -=QP.pCOM.y;
    QP.pCOM.x = 0;
    QP.pCOM.y = 0;
}
QuadPos OW_Quad::state_est_real(QuadPos QP_before)
{
    QuadPos QP_ss;

    if(RFcontact)
    {
        if(firstCon2){QP_ss.pRF.z = 0;firstCon2 = false;}
        QP_ss = Oi.FK_RF(WS.JointPosEnc,QP_before.pRF);
        WS.JointPosEnc.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else if(LFcontact)
    {
        if(firstCon2){QP_ss.pLF.z = 0;firstCon2 = false;}
        QP_ss = Oi.FK_LF(WS.JointPosEnc,QP_before.pLF);
        WS.JointPosEnc.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else if(RHcontact)
    {
        if(firstCon2){QP_ss.pRH.z = 0;firstCon2 = false;}
        QP_ss = Oi.FK_RH(WS.JointPos,QP_before.pRH);
        //QP_ss = Oi.FK_RH(WS.JointPos,QP_ss.pRH);
        WS.JointPosEnc.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else if(LHcontact)
    {
        if(firstCon2){QP_ss.pLH.z = 0;firstCon2 = false;}
        QP_ss = Oi.FK_LH(WS.JointPos,QP_before.pLH);
        //QP_ss = Oi.FK_LH(WS.JointPos,QP_ss.pLH);
        WS.JointPosEnc.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else
    {
        cnt_fly++;
        QP_ss = Oi.FK(WS.JointPosEnc);

        vec3 temp_mod = last_pCOM + last_dCOM*(cnt_fly*dt) + 0.5*vec3(0,0,-9.81)*(cnt_fly*dt)*(cnt_fly*dt) - QP_ss.pCOM;
        dpCOM = last_dCOM + vec3(0,0,-9.81)*(cnt_fly*dt);
        QP_ss.pCOM = QP_ss.pCOM + temp_mod;
        QP_ss.pPel = QP_ss.pPel + temp_mod;
        QP_ss.pRF = QP_ss.pRF + temp_mod;
        QP_ss.pLF = QP_ss.pLF + temp_mod;
        QP_ss.pRH = QP_ss.pRH + temp_mod;
        QP_ss.pLH = QP_ss.pLH + temp_mod;

        WS.JointPosEnc.pPel = QP_ss.pPel;
        firstCon2 = true;
    }

}
QuadPos OW_Quad::state_est2()
{
    QuadPos QP_ss;
//    if(isnan(WS.JointPos.pPel.x)||isnan(WS.JointPos.pPel.y)||isnan(WS.JointPos.pPel.z))
    if(0)
    {
        WS.JointPos.pPel = vec3();
    }
//    if(isnan(WS.JointPos.qPel[0])||isnan(WS.JointPos.qPel[1])||isnan(WS.JointPos.qPel[2])||isnan(WS.JointPos.qPel[3]))
    if(0)
    {
        WS.JointPos.qPel = quat();
    }
    if(RFcontact)
    {
        if(firstCon){QP_ss.pRF.z = 0;firstCon = false;}
        QP_ss = Oi.FK_RF(WS.JointPos,QP.pRF);
        //QP_ss = Oi.FK_RF(WS.JointPos,QP_ss.pRF);
        WS.JointPos.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else if(LFcontact)
    {
        if(firstCon){QP_ss.pLF.z = 0;firstCon = false;}
        QP_ss = Oi.FK_LF(WS.JointPos,QP.pLF);
        //QP_ss = Oi.FK_LF(WS.JointPos,QP_ss.pLF);
        WS.JointPos.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else if(RHcontact)
    {
        if(firstCon){QP_ss.pRH.z = 0;firstCon = false;}
        QP_ss = Oi.FK_RH(WS.JointPos,QP.pRH);
        //QP_ss = Oi.FK_RH(WS.JointPos,QP_ss.pRH);
        WS.JointPos.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else if(LHcontact)
    {
        if(firstCon){QP_ss.pLH.z = 0;firstCon = false;}
        QP_ss = Oi.FK_LH(WS.JointPos,QP.pLH);
        //QP_ss = Oi.FK_LH(WS.JointPos,QP_ss.pLH);
        WS.JointPos.pPel = QP_ss.pPel;

        last_pCOM = QP_ss.pCOM;
        cnt_fly = 0;
    }
    else
    {
        cnt_fly++;
        QP_ss = Oi.FK(WS.JointPos);

        vec3 temp_mod = last_pCOM + last_dCOM*(cnt_fly*dt) + 0.5*vec3(0,0,-9.81)*(cnt_fly*dt)*(cnt_fly*dt) - QP_ss.pCOM;
        dpCOM = last_dCOM + vec3(0,0,-9.81)*(cnt_fly*dt);
        QP_ss.pCOM = QP_ss.pCOM + temp_mod;
        QP_ss.pPel = QP_ss.pPel + temp_mod;
        QP_ss.pRF = QP_ss.pRF + temp_mod;
        QP_ss.pLF = QP_ss.pLF + temp_mod;
        QP_ss.pRH = QP_ss.pRH + temp_mod;
        QP_ss.pLH = QP_ss.pLH + temp_mod;

        WS.JointPos.pPel = QP_ss.pPel;
        firstCon = true;
    }


    // WS.JointVel.dpPel = (WS.JointPos.pPel-oldpPel)/dt;



    //should be global, not local
    //t2 = mat3(WS.IMUquat)*WS.IMULocalW;//why.....


    WS.JointVel.dqPel =WS.IMUomega;//local, but should be global?????? ask kangkyu

    Qnow = OD.Joints2Q(WS.JointPos);
    dQnow = OD.Joints2dQ(WS.JointVel);

    //Calculation dpPel
    //copythis------------------------------------
    if(Qcnt ==0) oldpPel = WS.JointPos.pPel;
    vec3 dpPelNumeric = (WS.JointPos.pPel-oldpPel)/dt;
    oldpPel = WS.JointPos.pPel;
    VectorNd Qtemp = Qnow;
    VectorNd dQrdot = dQnow;
    vec3 Fomega;
    Vector3d LFVel;
    vec3 drdt;

    Qtemp[0] = 0.0; Qtemp[1] = 0.0; Qtemp[2] = 0.0; Qtemp[3] = 0.0; Qtemp[4] = 0.0; Qtemp[5] = 0.0; Qtemp[18] =1.0;
    dQrdot[0] = 0.0; dQrdot[1] = 0.0; dQrdot[2] = 0.0; dQrdot[3] = 0.0; dQrdot[4] = 0.0; dQrdot[5] = 0.0;

    OD.CalcEndeffectorJacobian3D(Qtemp);//local frame jacobian


    mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    mat3 IMUrotz = mat3(QP.qPel);
    mat3 IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x

    vec3 IMUomega2 = mat3(IMUrot)*WS.IMUomega;//inverse???/
    WS.JointPos.qPel = quat(IMUrot);
    if(RFcontact)
    {
        Fomega = cross(IMUomega2,(QP_ss.pPel-QP_ss.pRF));
        LFVel =  -(OD.JacobianRF3D * dQrdot);

        vec3 EGHA = vec3(LFVel[0],LFVel[1],LFVel[2]);
        EGHA = WS.JointPos.qPel*EGHA;
        drdt = Fomega + EGHA;
        WS.JointVel.dpPel = drdt;//if want to use Calculation velocity
    }
    else if(LFcontact)
    {
        Fomega = cross(IMUomega2,(QP_ss.pPel-QP_ss.pLF));
        LFVel =  -(OD.JacobianLF3D * dQrdot);

        vec3 EGHA = vec3(LFVel[0],LFVel[1],LFVel[2]);
        EGHA = WS.JointPos.qPel*EGHA;
        drdt = Fomega + EGHA;
        WS.JointVel.dpPel = drdt;//if want to use Calculation velocity
    }
    else if(RHcontact)
    {
        Fomega = cross(IMUomega2,(QP_ss.pPel-QP_ss.pRH));
        LFVel =  -(OD.JacobianRH3D * dQrdot);

        vec3 EGHA = vec3(LFVel[0],LFVel[1],LFVel[2]);
        EGHA = WS.JointPos.qPel*EGHA;
        drdt = Fomega + EGHA;
        WS.JointVel.dpPel = drdt;//if want to use Calculation velocity
    }
    else if(LHcontact)
    {
        Fomega = cross(IMUomega2,(QP_ss.pPel-QP_ss.pLH));
        LFVel =  -(OD.JacobianLH3D * dQrdot);

        vec3 EGHA = vec3(LFVel[0],LFVel[1],LFVel[2]);
        EGHA = WS.JointPos.qPel*EGHA;
        drdt = Fomega + EGHA;
        WS.JointVel.dpPel = drdt;//if want to use Calculation velocity
    }
    else{

        Vector3d PELCOMVEL = OD.JacobianCOMall3D*dQrdot;
        WS.JointVel.dpPel = dpCOM + cross(IMUomega2,QP_ss.pPel-QP_ss.pCOM) + vec3(PELCOMVEL[0],PELCOMVEL[1],PELCOMVEL[2]);

    }
    //WS.JointVel.dpPel = dpPelNumeric;//if want to use numeric
    last_dpPel = WS.JointVel.dpPel;
    QP_ss.dCOM = vec3(OD.COMdot[0],OD.COMdot[1],OD.COMdot[2]);

    Qnow = OD.Joints2Q(WS.JointPos);
    dQnow = OD.Joints2dQ(WS.JointVel);
    OD.CalcCOMMomentum(Qnow,dQnow);
    if (cnt_fly==0)  last_dCOM = vec3(OD.COMdot[0],OD.COMdot[1],OD.COMdot[2]);

    return QP_ss;
}
void OW_Quad::calc_ddQref(vec3 _ddpCOMref, vec3 ddqPELref, vec3 ddpRFref, vec3 ddpLFref, vec3 ddpRHref, vec3 ddpLHref)
{
    //            OD.CalcEndeffectorJacobian3D(Qref);
    //            OD.CalcCOMJacobian3D(Qref);

    MatrixNd Jqref = MatrixNd::Zero(18,18);
    Jqref.block(0,0,3,18) = OD.JacobianCOMall3D;
    Jqref.block(3,3,3,3) = MatrixNd::Identity(3,3);
    Jqref.block(6,0,3,18) = OD.JacobianRF3D;
    Jqref.block(9,0,3,18) = OD.JacobianLF3D;
    Jqref.block(12,0,3,18) = OD.JacobianRH3D;
    Jqref.block(15,0,3,18) = OD.JacobianLH3D;

    VectorNd ddXref = VectorNd::Zero(18);
    ddXref.segment(0,3) = Vector3d(_ddpCOMref.x,_ddpCOMref.y,_ddpCOMref.z);
    ddXref.segment(3,3) = Vector3d(ddqPELref.x,ddqPELref.y,ddqPELref.z);
    ddXref.segment(6,3) = Vector3d(ddpRFref.x,ddpRFref.y,ddpRFref.z);
    ddXref.segment(9,3) = Vector3d(ddpLFref.x,ddpLFref.y,ddpLFref.z);
    ddXref.segment(12,3) = Vector3d(ddpRHref.x,ddpRHref.y,ddpRHref.z);
    ddXref.segment(15,3) = Vector3d(ddpLHref.x,ddpLHref.y,ddpLHref.z);

    VectorNd ddqZero = VectorNd::Zero(18);
    Vector3d dJdqpCOM = (Oi.m_torso * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_torso,Vector3d(Oi.c_torso.x,Oi.c_torso.y,Oi.c_torso.z))
                         + Oi.m_rhr * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_rhr,Vector3d(Oi.c_rhr.x,Oi.c_rhr.y,Oi.c_rhr.z))
                         + Oi.m_rhp * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_rhp,Vector3d(Oi.c_rhp.x,Oi.c_rhp.y,Oi.c_rhp.z))
                         + Oi.m_rkn * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_rkn,Vector3d(Oi.c_rkn.x,Oi.c_rkn.y,Oi.c_rkn.z))

                         + Oi.m_lhr * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_lhr,Vector3d(Oi.c_lhr.x,Oi.c_lhr.y,Oi.c_lhr.z))
                         + Oi.m_lhp * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_lhp,Vector3d(Oi.c_lhp.x,Oi.c_lhp.y,Oi.c_lhp.z))
                         + Oi.m_lkn * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_lkn,Vector3d(Oi.c_lkn.x,Oi.c_lkn.y,Oi.c_lkn.z))

                         + Oi.m_rsr * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_rsr,Vector3d(Oi.c_rsr.x,Oi.c_rsr.y,Oi.c_rsr.z))
                         + Oi.m_rsp * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_rsp,Vector3d(Oi.c_rsp.x,Oi.c_rsp.y,Oi.c_rsp.z))
                         + Oi.m_reb * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_reb,Vector3d(Oi.c_reb.x,Oi.c_reb.y,Oi.c_reb.z))

                         + Oi.m_lsr * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_lsr,Vector3d(Oi.c_lsr.x,Oi.c_lsr.y,Oi.c_lsr.z))
                         + Oi.m_lsp * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_lsp,Vector3d(Oi.c_lsp.x,Oi.c_lsp.y,Oi.c_lsp.z))
                         + Oi.m_leb * CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_leb,Vector3d(Oi.c_leb.x,Oi.c_leb.y,Oi.c_leb.z)))/Oi.M_total;
    Vector3d dJdqqPel = Vector3d::Zero(3); //dJ/dt = 0
    Vector3d dJdqRF = CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_rkn,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));
    Vector3d dJdqLF = CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_lkn,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));
    Vector3d dJdqRH = CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_reb,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));
    Vector3d dJdqLH = CalcPointAcceleration(*(OD.Robot),Qref,dQref,ddqZero,OD.n_leb,Vector3d(Oi.offset_lleg.x,Oi.offset_lleg.y,Oi.offset_lleg.z));

    VectorNd dJdq = VectorNd::Zero(18);
    dJdq.segment(0,3) = dJdqpCOM;
    dJdq.segment(3,3) = dJdqqPel;
    dJdq.segment(6,3) = dJdqRF;
    dJdq.segment(9,3) = dJdqLF;
    dJdq.segment(12,3) = dJdqRH;
    dJdq.segment(15,3) = dJdqLH;

    ddQref = Jqref.inverse()*ddXref - dJdq;
}
void OW_Quad::calc_dQref(vec3 _dpCOMref, vec3 dqPELref, vec3 dpRFref, vec3 dpLFref, vec3 dpRHref, vec3 dpLHref)
{
    OD.CalcEndeffectorJacobian3D(Qref);
    OD.CalcCOMJacobian3D(Qref);


    Jqref.block(0,0,3,18) = OD.JacobianCOMall3D;
    Jqref.block(3,3,3,3) = MatrixNd::Identity(3,3);
    Jqref.block(6,0,3,18) = OD.JacobianRF3D;
    Jqref.block(9,0,3,18) = OD.JacobianLF3D;
    Jqref.block(12,0,3,18) = OD.JacobianRH3D;
    Jqref.block(15,0,3,18) = OD.JacobianLH3D;


    dXref.segment(0,3) = Vector3d(_dpCOMref.x,_dpCOMref.y,_dpCOMref.z);
    dXref.segment(3,3) = Vector3d(dqPELref.x,dqPELref.y,dqPELref.z);
    dXref.segment(6,3) = Vector3d(dpRFref.x,dpRFref.y,dpRFref.z);
    dXref.segment(9,3) = Vector3d(dpLFref.x,dpLFref.y,dpLFref.z);
    dXref.segment(12,3) = Vector3d(dpRHref.x,dpRHref.y,dpRHref.z);
    dXref.segment(15,3) = Vector3d(dpLHref.x,dpLHref.y,dpLHref.z);

    dQref = Jqref.inverse()*dXref;

}
QuadPos OW_Quad::fk_dQ(QuadJoints _QJ, QuadJointVels _dQJ)
{
    QuadPos Qout = Oi.FK(_QJ);
    ODJUMP.CalcEndeffectorJacobian3D(ODJUMP.Joints2Q(_QJ));
    ODJUMP.CalcCOMJacobian3D(ODJUMP.Joints2Q(_QJ));

    MatrixNd Jqnow = MatrixNd::Zero(18,18);
    Jqnow.block(0,0,3,18) = ODJUMP.JacobianCOMall3D;
    Jqnow.block(3,3,3,3) = MatrixNd::Identity(3,3);
    Jqnow.block(6,0,3,18) = ODJUMP.JacobianRF3D;
    Jqnow.block(9,0,3,18) = ODJUMP.JacobianLF3D;
    Jqnow.block(12,0,3,18) = ODJUMP.JacobianRH3D;
    Jqnow.block(15,0,3,18) = ODJUMP.JacobianLH3D;

    VectorNd dXnow = Jqnow*ODJUMP.Joints2dQ(_dQJ);

    Qout.dCOM = vec3(dXnow(0),dXnow(1),dXnow(2));
    Qout.dqPel = vec3(dXnow(3),dXnow(4),dXnow(5));
    Qout.dLF = vec3(dXnow(6),dXnow(7),dXnow(8));
    Qout.dRF = vec3(dXnow(9),dXnow(10),dXnow(11));
    Qout.dRH = vec3(dXnow(12),dXnow(13),dXnow(14));
    Qout.dLH = vec3(dXnow(15),dXnow(16),dXnow(17));
    return Qout;
}
QuadPos OW_Quad::state_est()
{
    QuadPos QP_ee = QP;
    if(Qcnt==0)
    {
        return QP_ee;
    }
    //        mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    //        mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    //        mat3 IMUrot = IMUrotx*IMUroty;

    //        QP_ee.pCOM = ZMP + IMUrot*(QP.pCOM-ZMP);
    //        vec3 Rotvel = cross(WS.IMUomega,QP.pCOM-ZMP);
    //        QP_ee.dCOM = IMUrot*(QP.dCOM) + Rotvel;//wrong here!!!!!

    RFcontact = !FootonAir[0];
    LFcontact = !FootonAir[1];
    RHcontact = !FootonAir[2];
    LHcontact = !FootonAir[3];
    vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
    FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
    vec3 FrontVector = FrontVt.normalize();//not really...
    vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();

    QuadPos QP_ss = state_est2();
    mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    mat3 IMUrotz = mat3(QP.qPel);
    mat3 IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x

    double LeftOff = dot(LeftVector,QP.pCOM-ZMP);
    double FrontOff = dot(FrontVector,QP.pCOM-ZMP);
    double ZOff = dot(vec3(0,0,1),QP.pCOM-ZMP);

    vec3 FOff = FrontVector*(FrontOff*cos(WS.IMUangle.y)+ZOff*sin(WS.IMUangle.y));
    vec3 LOff = LeftVector*(LeftOff*cos(-WS.IMUangle.x)+ZOff*sin(-WS.IMUangle.x));



    //QP_ee.pCOM = ZMP + IMUrot*(QP.pCOM-ZMP);
    QP_ee.pCOM = ZMP + FOff+LOff+vec3(0,0,1)*ZOff;
    //QP_ee.pCOM = ZMP + IMUrot*(QP.pCOM-ZMP);
    QP_ss.pCOM = QP_ee.pCOM;//hmm....

    QP_real = state_est_real(QP_real);

    return QP_ss;//
}


MatrixNd OW_Quad::pinv(MatrixNd in)
{
    Eigen::JacobiSVD<MatrixNd> _svd(in,Eigen::ComputeThinU|Eigen::ComputeThinV);
    double tol = 1e-4;
    return _svd.matrixV()*(_svd.singularValues().array().abs()>tol).select(_svd.singularValues().array().inverse(),0).matrix().asDiagonal()*_svd.matrixU().adjoint();
}



VectorNd OW_Quad::calc_3th(double t_0, double t_e,vec3 h_0,vec3 h_e)
{
    if(t_e-t_0<0.05){t_e = t_0+0.05;}
    MatrixNd AAA(4,4);
    AAA(0,0) = pow(t_0,3);
    AAA(0,1) = pow(t_0,2);
    AAA(0,2) = pow(t_0,1);
    AAA(0,3) = 1;

    AAA(1,0) = 3*pow(t_0,2);
    AAA(1,1) = 2*pow(t_0,1);
    AAA(1,2) = 1;
    AAA(1,3) = 0;

    AAA(2,0) = pow(t_e,3);
    AAA(2,1) = pow(t_e,2);
    AAA(2,2) = pow(t_e,1);
    AAA(2,3) = 1;

    AAA(3,0) = 3*pow(t_e,2);
    AAA(3,1) = 2*pow(t_e,1);
    AAA(3,2) = 1;
    AAA(3,3) = 0;

    VectorNd BBB(4);
    BBB[0] = h_0[0];//position
    BBB[1] = h_0[1];//velocity
    BBB[2] = h_e[0];//position
    BBB[3] = h_e[1];//velocity

    return (AAA.inverse())*BBB;
}
VectorNd OW_Quad::calc_5th_ZMP(double t_0, double t_e,vec3 h_0,vec3 h_e)
{
    vec3 th_0 = h_0;
    vec3 th_e = h_e;
    th_0.z = (h_0[2]-h_0[0])*g/delZ;
    th_e.z = (h_e[2]-h_e[0])*g/delZ;
    return calc_5th(t_0,t_e,th_0,th_e);
}
VectorNd OW_Quad::calc_5th_acc_free(double t_0, double t_e,vec3 h_0,vec3 h_e)
{
    vec3 tempV = h_e;
    tempV[0] = 0;
    if((t_e-t_0<0.05)&&tempV.norm()<1e-8){t_e = t_0+0.05;}
    MatrixNd AAA(4,6);
    AAA(0,0) = pow(t_0,5);
    AAA(0,1) = pow(t_0,4);
    AAA(0,2) = pow(t_0,3);
    AAA(0,3) = pow(t_0,2);
    AAA(0,4) = pow(t_0,1);
    AAA(0,5) = 1;

    AAA(1,0) = 5*pow(t_0,4);
    AAA(1,1) = 4*pow(t_0,3);
    AAA(1,2) = 3*pow(t_0,2);
    AAA(1,3) = 2*pow(t_0,1);
    AAA(1,4) = 1;
    AAA(1,5) = 0;

    AAA(2,0) = pow(t_e,5);
    AAA(2,1) = pow(t_e,4);
    AAA(2,2) = pow(t_e,3);
    AAA(2,3) = pow(t_e,2);
    AAA(2,4) = pow(t_e,1);
    AAA(2,5) = 1;

    AAA(3,0) = 5*pow(t_e,4);
    AAA(3,1) = 4*pow(t_e,3);
    AAA(3,2) = 3*pow(t_e,2);
    AAA(3,3) = 2*pow(t_e,1);
    AAA(3,4) = 1;
    AAA(3,5) = 0;



    VectorNd BBB(4);
    BBB[0] = h_0[0];//position
    BBB[1] = h_0[1];//velocity
    BBB[2] = h_e[0];//position
    BBB[3] = h_e[1];//velocity

    return pinv(AAA)*BBB;
}
VectorNd OW_Quad::calc_5th(double t_0, double t_e,vec3 h_0,vec3 h_e)
{
    vec3 tempV = h_e;
    tempV[0] = 0;
    if((t_e-t_0<0.05)&&tempV.norm()<1e-8){t_e = t_0+0.05;}



    MatrixNd AAA(6,6);
    AAA(0,0) = pow(t_0,5);
    AAA(0,1) = pow(t_0,4);
    AAA(0,2) = pow(t_0,3);
    AAA(0,3) = pow(t_0,2);
    AAA(0,4) = pow(t_0,1);
    AAA(0,5) = 1;

    AAA(1,0) = 5*pow(t_0,4);
    AAA(1,1) = 4*pow(t_0,3);
    AAA(1,2) = 3*pow(t_0,2);
    AAA(1,3) = 2*pow(t_0,1);
    AAA(1,4) = 1;
    AAA(1,5) = 0;

    AAA(2,0) = 20*pow(t_0,3);
    AAA(2,1) = 12*pow(t_0,2);
    AAA(2,2) = 6*pow(t_0,1);
    AAA(2,3) = 2*1;
    AAA(2,4) = 0;
    AAA(2,5) = 0;

    AAA(3,0) = pow(t_e,5);
    AAA(3,1) = pow(t_e,4);
    AAA(3,2) = pow(t_e,3);
    AAA(3,3) = pow(t_e,2);
    AAA(3,4) = pow(t_e,1);
    AAA(3,5) = 1;

    AAA(4,0) = 5*pow(t_e,4);
    AAA(4,1) = 4*pow(t_e,3);
    AAA(4,2) = 3*pow(t_e,2);
    AAA(4,3) = 2*pow(t_e,1);
    AAA(4,4) = 1;
    AAA(4,5) = 0;

    AAA(5,0) = 20*pow(t_e,3);
    AAA(5,1) = 12*pow(t_e,2);
    AAA(5,2) = 6*pow(t_e,1);
    AAA(5,3) = 2*1;
    AAA(5,4) = 0;
    AAA(5,5) = 0;

    VectorNd BBB(6);
    BBB[0] = h_0[0];//position
    BBB[1] = h_0[1];//velocity
    BBB[2] = h_0[2];//acc
    BBB[3] = h_e[0];//position
    BBB[4] = h_e[1];//velocity
    BBB[5] = h_e[2];//acc

    VectorNd CCC = (AAA.inverse())*BBB;
    for(int i=0;i<6;i++)
    {
//        if(isnan(CCC[i]))
        if(0)
        {
            CCC[0] = CCC[1] = CCC[2] = CCC[3] = CCC[4]  = 0;
            CCC[5] = h_e[0];
            //printf("nan detected 5th %f",Qcnt*dt);
            return CCC;
        }
    }
    return CCC;
}

XYZxdxddx OW_Quad::calc_nextRef(double t_now,double t_e, XYZxdxddx ref_now, vec3 ref_end)
{
    double st_5,st_4,st_3,st_2,st_1;
    XYZxdxddx rt;
    VectorNd FPR;
    double t_now2 = 0;
    double t_e2 = t_e-t_now;
    st_1 = t_now2;
    st_2 = st_1*st_1;
    st_3 = st_2*st_1;
    st_4 = st_3*st_1;
    st_5 = st_4*st_1;
    FPR = calc_5th(t_now2-dt,t_e2,ref_now.xdxddx,vec3(ref_end.x,0,0));
    rt.xdxddx[0] = FPR[0]*st_5 + FPR[1]*st_4 + FPR[2]*st_3 + FPR[3]*st_2 + FPR[4]*st_1 + FPR[5];
    rt.xdxddx[1] = 5*FPR[0]*st_4 + 4*FPR[1]*st_3 + 3*FPR[2]*st_2 + 2*FPR[3]*st_1 + FPR[4];
    rt.xdxddx[2] = 20*FPR[0]*st_3 + 12*FPR[1]*st_2 + 6*FPR[2]*st_1 + 2*FPR[3];

    FPR = calc_5th(t_now2-dt,t_e2,ref_now.ydyddy,vec3(ref_end.y,0,0));
    rt.ydyddy[0] = FPR[0]*st_5 + FPR[1]*st_4 + FPR[2]*st_3 + FPR[3]*st_2 + FPR[4]*st_1 + FPR[5];
    rt.ydyddy[1] = 5*FPR[0]*st_4 + 4*FPR[1]*st_3 + 3*FPR[2]*st_2 + 2*FPR[3]*st_1 + FPR[4];
    rt.ydyddy[2] = 20*FPR[0]*st_3 + 12*FPR[1]*st_2 + 6*FPR[2]*st_1 + 2*FPR[3];

    FPR =calc_5th(t_now2-dt,t_e2,ref_now.zdzddz,vec3(ref_end.z,0,0));
    rt.zdzddz[0] = FPR[0]*st_5 + FPR[1]*st_4 + FPR[2]*st_3 + FPR[3]*st_2 + FPR[4]*st_1 + FPR[5];
    rt.zdzddz[1] = 5*FPR[0]*st_4 + 4*FPR[1]*st_3 + 3*FPR[2]*st_2 + 2*FPR[3]*st_1 + FPR[4];
    rt.zdzddz[2] = 20*FPR[0]*st_3 + 12*FPR[1]*st_2 + 6*FPR[2]*st_1 + 2*FPR[3];

    return rt;
}
void OW_Quad::calc_decomp()
{
    mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    mat3 IMUrotz = mat3(QP.qPel);
    //mat3 IMUrotz = mat3(quat());
    mat3 IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x
    IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x
    tz = vec3(0,0,1);
    vec3 nV;
    if(isRHLFmove){nV = QP.pLH-QP.pRF;}
    else{nV = QP.pRH-QP.pLF;}
    vec3 td = cross(tz,nV);
    ty = td.normalize();//rotation to be controled now
    tx = cross(ty,tz);
//        decomp_A = Oi.Decomp_Rmat(IMUrot,tz,ty,tx);//z-y-x???

//        decomp_A = Oi.Decomp_Rmat(IMUrot,tz,tx,ty);//z-x-y???
//        double tt = decomp_A.y;
//        decomp_A.y= decomp_A.z;
//        decomp_A.z = tt;

    decomp_A = Oi.Decomp_Rmat(IMUrot,tx,ty,tz);//x-y-z???
    double tt = decomp_A.x;
    decomp_A.x = decomp_A.z;
    decomp_A.z = tt;

    vec3 IMUomega_Global = IMUrot*WS.IMUomega;
    decomp_W.x = dot(tz,IMUomega_Global);
    decomp_W.y = dot(ty,IMUomega_Global);
    decomp_W.z = dot(tx,IMUomega_Global);

    IMUomega_filtered.x = LPFs[0].do_filt(WS.IMUomega.x);
    IMUomega_filtered.y = LPFs[1].do_filt(WS.IMUomega.y);
    IMUomega_filtered.z = LPFs[2].do_filt(WS.IMUomega.z);

    if(oldRHLFmove!=isRHLFmove)
    {
        LPFs[3].reset_filt(decomp_W.x,decomp_W.x);
        LPFs[4].reset_filt(decomp_W.y,decomp_W.y);
        LPFs[5].reset_filt(decomp_W.z,decomp_W.z);
    }
    rotE_I.x = rotE_I.x+WS.IMUangle.x*dt;
    rotE_I.y = rotE_I.y+WS.IMUangle.y*dt;
    double maxrotE = 4*D2Rf;
    if(rotE_I.x>maxrotE){rotE_I.x = maxrotE;}
    if(rotE_I.x<-maxrotE){rotE_I.x = -maxrotE;}
    if(rotE_I.y>maxrotE){rotE_I.y = maxrotE;}
    if(rotE_I.y<-maxrotE){rotE_I.y = -maxrotE;}
    rotE_I.z = 0;
    rotE_I.y = 0;//x only....
    //rotE_I.x = 0;///////////////////////////////////ttttttttttt
    vec3 rotE_I_Global = IMUrot*rotE_I;
    decomp_I.x = 0;
    decomp_I.y = dot(ty,rotE_I_Global);
    decomp_I.z = dot(tx,rotE_I_Global);

    decomp_W_filtered.x = LPFs[3].do_filt(decomp_W.x);
    decomp_W_filtered.y = LPFs[4].do_filt(decomp_W.y);
    decomp_W_filtered.z = LPFs[5].do_filt(decomp_W.z);

    oldRHLFmove = isRHLFmove;

}
void OW_Quad::calc_decomp_pace()
{
    mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    mat3 IMUrotz = mat3(QP.qPel);
    //mat3 IMUrotz = mat3(quat());
    mat3 IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x
    IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x
    tz = vec3(0,0,1);
    vec3 nV;
    if(isRmove){nV = QP.pLH-QP.pLF;}
    else{nV = QP.pRH-QP.pRF;}
    vec3 td = cross(tz,nV);
    ty = td.normalize();//rotation to be controled now
    tx = cross(ty,tz);
//        decomp_A = Oi.Decomp_Rmat(IMUrot,tz,ty,tx);//z-y-x???

//        decomp_A = Oi.Decomp_Rmat(IMUrot,tz,tx,ty);//z-x-y???
//        double tt = decomp_A.y;
//        decomp_A.y= decomp_A.z;
//        decomp_A.z = tt;

    decomp_A = Oi.Decomp_Rmat(IMUrot,tx,ty,tz);//x-y-z???
    double tt = decomp_A.x;
    decomp_A.x = decomp_A.z;
    decomp_A.z = tt;

    vec3 IMUomega_Global = IMUrot*WS.IMUomega;
    decomp_W.x = dot(tz,IMUomega_Global);
    decomp_W.y = dot(ty,IMUomega_Global);
    decomp_W.z = dot(tx,IMUomega_Global);

    IMUomega_filtered.x = LPFs[0].do_filt(WS.IMUomega.x);
    IMUomega_filtered.y = LPFs[1].do_filt(WS.IMUomega.y);
    IMUomega_filtered.z = LPFs[2].do_filt(WS.IMUomega.z);

    if(oldisRmove!=isRmove)
    {
        LPFs[3].reset_filt(decomp_W.x,decomp_W.x);
        LPFs[4].reset_filt(decomp_W.y,decomp_W.y);
        LPFs[5].reset_filt(decomp_W.z,decomp_W.z);


    }
    rotE_I.x = rotE_I.x+WS.IMUangle.x*dt;
    rotE_I.y = rotE_I.y+WS.IMUangle.y*dt;
    double maxrotE = 4*D2Rf;
    if(rotE_I.x>maxrotE){rotE_I.x = maxrotE;}
    if(rotE_I.x<-maxrotE){rotE_I.x = -maxrotE;}
    if(rotE_I.y>maxrotE){rotE_I.y = maxrotE;}
    if(rotE_I.y<-maxrotE){rotE_I.y = -maxrotE;}
    rotE_I.z = 0;
    rotE_I.y = 0;//x only....
    decomp_I.x = 0;
    decomp_I.y = dot(ty,rotE_I);
    decomp_I.z = dot(tx,rotE_I);

    decomp_W_filtered.x = LPFs[3].do_filt(decomp_W.x);
    decomp_W_filtered.y = LPFs[4].do_filt(decomp_W.y);
    decomp_W_filtered.z = LPFs[5].do_filt(decomp_W.z);

    oldisRmove = isRmove;

}
void OW_Quad::estimate_plane()
{
    //plane estimation
    vec3 rpyQP = Oi.Decomp_Rmat(QP.qPel,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));
    mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    mat3 IMUrotz = mat3(vec3(0,0,-1),rpyQP[0]);
    mat3 IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x
    WS.JointPosEnc.qPel = quat(IMUrot);
    QuadJoints QJJ = Oi.IK_COM(QP_controled);
    WS.JointPosEnc.pPel = QJJ.pPel;//hmmmmm....
    //WS.JointPosEnc.pPel = QP.pPel;//hmmmmm....//wrong here
    QP_ENC_PLANE = Oi.FK(WS.JointPosEnc);
    if(isRHLFmove)
    {
        oldF1 = QP_ENC_PLANE.pRH;
        oldF2 = QP_ENC_PLANE.pLF;
        F1 = QP_ENC_PLANE.pLH;
        F2 = QP_ENC_PLANE.pRF;
    }
    else
    {
        oldF1 = QP_ENC_PLANE.pLH;
        oldF2 = QP_ENC_PLANE.pRF;
        F1 = QP_ENC_PLANE.pRH;
        F2 = QP_ENC_PLANE.pLF;
    }


    double alpha = 0.2;
    //ax + by + cz + d = 0;
    //let c = 1 always
    vec3 V1 = oldF1-oldF2;
    vec3 V2 = F1-F2;
    vec3 normal = cross(V1,V2).normalize();
    if(fabs(normal.z)<1e-6)
    {
        abd[0] = 0;
        abd[1] = 0;
    }
    else
    {
        abd[0] = normal.x/normal.z;
        abd[1] = normal.y/normal.z;
    }
    abd[2] = -1*(F1.z+F2.z)*0.5;

    if(isFirststep){abd_f = abd;}
    abd_f = alpha*abd + (1-alpha)*abd_f;

    //footz from plane eq
    //z = -(ax+by+d)/c
    //Fooz = -(abd[0]*Footx+abd[1]*Footy+abd[2]);
    calc_rpy();
}
void OW_Quad::calc_rpy()
{
    if(NO_ROBOT_TEST)
    {
        abd[0] = abd_f[0] =  0;//-0.2
        abd[1] = abd_f[1] =   0;
        abd[2] = abd_f[2] =   0;
    }

    vec3 ttx,tty,ttz;
    vec3 x_R,y_R,z_R,tV;
    mat3 planeR;
    rpyQP = Oi.Decomp_Rmat(QP.qPel,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));

    x_R = mat3(vec3(0,0,-1),rpyQP[0])*vec3(1,0,0);
    y_R = mat3(vec3(0,0,-1),rpyQP[0])*vec3(0,1,0);
    z_R = vec3(0,0,1);

    ttz = vec3(abd[0],abd[1],1);
    ttz.normalize();
    ttx = x_R-dot(x_R,ttz)*ttz;
    ttx.normalize();
    tty = cross(ttz,ttx);
    tty.normalize();


    planeR.x = ttx;
    planeR.y = tty;
    planeR.z = ttz;
    planeR.inverse();

    tV= Oi.Decomp_Rmat(planeR,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));//x-y-z
    plane_rpy.x = tV.z;
    plane_rpy.y = tV.y;//why???
    plane_rpy.z = tV.x;

    ttz = vec3(abd_f[0],abd_f[1],1);
    ttz.normalize();
    ttx = x_R-dot(x_R,ttz)*ttz;
    ttx.normalize();
    tty = cross(ttz,ttx);
    tty.normalize();


    planeR.x = ttx;
    planeR.y = tty;
    planeR.z = ttz;
    planeR.inverse();

    tV= Oi.Decomp_Rmat(planeR,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));//x-y-z
    plane_rpy_f.x = tV.z;
    plane_rpy_f.y = tV.y;//why???
    plane_rpy_f.z = tV.x;


}
double OW_Quad::Zfromplane(double x, double y)
{
    //Fooz = -(abd[0]*Footx+abd[1]*Footy+abd[2]);
    return -(abd_f[0]*x+abd_f[1]*y+abd_f[2]);
}

