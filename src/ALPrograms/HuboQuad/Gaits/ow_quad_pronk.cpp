#include "ow_quad.h"

QuadJoints OW_Quad::Pronk_onestep(WalkSensors _WS)
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
    vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
    FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
    vec3 FrontVector = FrontVt.normalize();//not really...
    vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();

    if(NO_ROBOT_TEST==false)
    {
        fallcheck();
    }


//make-motion from here
    double conT = 0.15;
//    if(isFirststep)//bug? why com goes down????
//    {
//        conT = 0.6;
//    }
    double pushT = 0.15;

    double upL = 0.08;
    double upV0 = 0.8;
//    double flyT = (upV*2)/g;
//    double swT = flyT;

    double hopT = conT+pushT+flyT;

    vec3 des_V = FrontVector*WP.step_L.x/hopT + LeftVector*WP.step_L.y/hopT;
    double L0 = WP.delZ+0.05;//+0.02;


    if(HOPPHASE==0)//Jumping
    {
        if(isFirststep&&lastHOPPHASE==2)//very first step
        {
            upV  = upV0;
            flyT = (upV*2)/g;
            swT = flyT;
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
            dcomz = 0;//hmm....
            dcomx = dcomy = 0;
            ddcomzland = 0;
            ddcomz = (upV-dcomz)/pushT;//fixed ddcomz
            printf("firsttep! %d \n",stopcnt);
            oldRotZ = vec3(0,0,0);
            lastHOPPHASE = 0;
            double xoff = des_V.x*(conT+pushT)*0.5;//kvx*(dQnow[0]-refVx);// + ;
            double yoff = des_V.y*(conT+pushT)*0.5;//kvy*(dQnow[1]-refVy);// + ;
            vec3 midpoint = 0.25*(QP.pRF+QP.pLF+QP.pRH+QP.pLH);
            vec3 x0(QP.pCOM.x,QP.dCOM.x,0);
            vec3 xe(midpoint.x+xoff,des_V.x,0);
            F3thx = calc_3th(t_now,conT+pushT,x0,xe);
            vec3 y0(QP.pCOM.y,QP.dCOM.y,0);
            vec3 ye(midpoint.y+yoff,des_V.y,0);
            F3thy = calc_3th(t_now,conT+pushT,y0,ye);
            //WP.delZ = WP.delZ+0.03;//
        }
        if(lastHOPPHASE==1)//just landed
        {//how to measure? or just start frum v=0?
            isFirststep = false;
            if(isStopping)
            {
                stopcnt--;
                if(stopcnt<0)
                {
                    isStopping = false;
                    isFinishing = true;
                }
            }
            if(walkchanged)
            {
                WP = WP_next;
                des_delZ = WP.delZ;
                walkchanged  = false;
            }
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
            //dcomz = 0;//hmm....
            ddcomzland = (0-dcomz)/conT;//fixed ddcomzland
            ddcomz = (upV0-0)/pushT;//fixed ddcomz
            printf("nextstep! %d \n",stopcnt);
            oldRotZ = vec3(0,0,0);
            t_now =0;
            lastHOPPHASE = 0;
            double xoff = des_V.x*(conT+pushT)*0.5;//kvx*(dQnow[0]-refVx);// + ;
            double yoff = des_V.y*(conT+pushT)*0.5;//kvy*(dQnow[1]-refVy);// + ;
            vec3 midpoint = 0.25*(QP.pRF+QP.pLF+QP.pRH+QP.pLH);
            vec3 x0(QP.pCOM.x,QP.dCOM.x,0);
            vec3 xe(midpoint.x+xoff,des_V.x,0);
            F3thx = calc_3th(t_now,conT+pushT,x0,xe);
            vec3 y0(QP.pCOM.y,QP.dCOM.y,0);
            vec3 ye(midpoint.y+yoff,des_V.y,0);
            F3thy = calc_3th(t_now,conT+pushT,y0,ye);
        }


        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        qPel_stepstart = QP.qPel;



        if(isFirststep&&t_now<conT-0.5*dt)//landing
        {

        }
        else if(t_now<conT-0.5*dt)
        {
            dcomz +=ddcomzland*dt;
            COMrefXdXddX.setddX(vec3(0,0,ddcomzland));
        }
        else//jumping
        {
            dcomz +=ddcomz*dt;
            COMrefXdXddX.setddX(vec3(0,0,ddcomz));
        }

        double st1 = t_now;
        double st2 = st1*st1;
        double st3 = st2*st1;

        double comx, comy;
        comx = F3thx[0]*st3 + F3thx[1]*st2 + F3thx[2]*st1 + F3thx[3];
        comy = F3thy[0]*st3 + F3thy[1]*st2 + F3thy[2]*st1 + F3thy[3];
        dcomx = 3*F3thx[0]*st2 + 2*F3thx[1]*st1 + F3thx[2];
        dcomy = 3*F3thy[0]*st2 + 2*F3thy[1]*st1 + F3thy[2];

        if(isFirststep&&(t_now<conT-0.5*dt))
        {
            double aa = upV/pushT;
            double LL = 0.5*aa*pushT*pushT;
            vec3 midpoint = 0.25*(QP.pRF+QP.pLF+QP.pRH+QP.pLH);
            vec3 rr = QP.pCOM;
            rr.z = L0-LL+midpoint.z;
            COMrefXdXddX = calc_nextRef(t_now,conT,COMrefXdXddX,rr);
        }
        else
        {
            COMrefXdXddX.setdX(vec3(dcomx,dcomy,dcomz));
            COMrefXdXddX.setX(COMrefXdXddX.X()+COMrefXdXddX.dX()*dt);
        }



        if(t_now>pushT+conT)//jump done
        {
            HOPPHASE = 1;
            //dcomz = 0;
            t_now = 0;
            //gait change maybe
            lastHOPPHASE = 0;
            //update kinematics-reference here

            QuadPos QPnew;
            if(NO_ROBOT_TEST==false)
            {
                QuadJoints QJJ = WS.JointPos;
                QuadJointVels dQJJ = WS.JointVel;
                QJJ.qPel = QP.qPel;
                dQJJ.dqPel = QP.dqPel;
                //hmm.........

                QPnew = fk_dQ(QJJ,dQJJ);

            }
            else
            {
                QPnew = QP;
                QPnew.dCOM.z = QPnew.dCOM.z*0.8;
                QPnew.dCOM.x = 0.1;
            }
            vec3 mcref = (QP.pRF+QP.pLF+QP.pRH+QP.pLH)*0.25;
            vec3 dmcref = (QP.dRF+QP.dLF+QP.dRH+QP.dLH)*0.25;
            vec3 mcnow = (QPnew.pRF+QPnew.pLF+QPnew.pRH+QPnew.pLH)*0.25;
            vec3 dmcnow = (QPnew.dRF+QPnew.dLF+QPnew.dRH+QPnew.dLH)*0.25;
            vec3 mn2r = mcref-mcnow;
            vec3 dmn2r = dmcref-dmcnow;
            QPnew.pCOM = QPnew.pCOM+mn2r;
            QPnew.pRF = QPnew.pRF+mn2r;
            QPnew.pLF = QPnew.pLF+mn2r;
            QPnew.pRH = QPnew.pRH+mn2r;
            QPnew.pLH = QPnew.pLH+mn2r;

            QPnew.dCOM = QPnew.dCOM+dmn2r;
            QPnew.dRF = QPnew.dRF+dmn2r;
            QPnew.dLF = QPnew.dLF+dmn2r;
            QPnew.dRH = QPnew.dRH+dmn2r;
            QPnew.dLH = QPnew.dLH+dmn2r;



            QP.pCOM = QPnew.pCOM + QPnew.dCOM*dt;
            QP.pRF = QPnew.pRF + QPnew.dRF*dt;
            QP.pLF = QPnew.pLF + QPnew.dLF*dt;
            QP.pRH = QPnew.pRH + QPnew.dRH*dt;
            QP.pLH = QPnew.pLH + QPnew.dLH*dt;

            QP.dCOM = QPnew.dCOM;
            QP.dRF = QPnew.dRF;
            QP.dLF = QPnew.dLF;
            QP.dRH = QPnew.dRH;
            QP.dLH = QPnew.dLH;

            upV  = QP.dCOM.z;
            flyT = (upV*2)/g;
            swT = flyT;
            dcomz = QP.dCOM.z;
            dcomx = QP.dCOM.x;
            dcomy = QP.dCOM.y;
            RFref.setX(QP.pRF);
            RFref.dXddX0();
            LFref.setX(QP.pLF);
            LFref.dXddX0();
            RHref.setX(QP.pRH);
            RHref.dXddX0();
            LHref.setX(QP.pLH);
            LHref.dXddX0();
            COMrefXdXddX.setX(QP.pCOM);
            COMrefXdXddX.setdX(QP.dCOM);

        }

        if(isFinishing&&t_now>conT)
        {
            //later finishcnt should be added for additional control
            changetoPcon[0] = true;
            changetoPcon[1] = true;
            changetoPcon[2] = true;
            changetoPcon[3] = true;
            QuadJoints QJenc = WS.JointPosEnc;
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
            save_all();
            return QJenc;
        }




    }
    else//flying/swing
    {
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=true;
        QuadPos QP_eop = QP;
        double refVx = 0;
        double refVy = 0;//how to get dqnow?
        double kvx = 0.0;
        double kvy = 0.0;
        double xoff = des_V.x*(conT+pushT)*0.5 + kvx*(QP.dCOM.x-refVx);// + ;
        double yoff = des_V.y*(conT+pushT)*0.5 + kvy*(QP.dCOM.y-refVy);// + ;

        vec3 RHoff =   WP.FB_L/2.0*FrontVector - WP.LR_L/2.0*LeftVector;
        vec3 LFoff = - WP.FB_L/2.0*FrontVector + WP.LR_L/2.0*LeftVector;
        vec3 LHoff =   WP.FB_L/2.0*FrontVector + WP.LR_L/2.0*LeftVector;
        vec3 RFoff = - WP.FB_L/2.0*FrontVector - WP.LR_L/2.0*LeftVector;

        QP_eop.pRF = vec3(xoff,yoff,0) + QP.pCOM + RFoff + cross(vec3(0,0,WP.step_Rot/hopT),RFoff)*(conT+pushT)*0.5;
        QP_eop.pLF = vec3(xoff,yoff,0) + QP.pCOM + LFoff + cross(vec3(0,0,WP.step_Rot/hopT),LFoff)*(conT+pushT)*0.5;
        QP_eop.pRH = vec3(xoff,yoff,0) + QP.pCOM + RHoff + cross(vec3(0,0,WP.step_Rot/hopT),RHoff)*(conT+pushT)*0.5;
        QP_eop.pLH = vec3(xoff,yoff,0) + QP.pCOM + LHoff + cross(vec3(0,0,WP.step_Rot/hopT),LHoff)*(conT+pushT)*0.5;

        QP_eop.pRF.z = QP_eop.pLF.z = QP_eop.pRH.z = QP_eop.pLH.z = 0;

        if(t_now<swT*0.5)
        {
            RFref = calc_nextRef(t_now,0.5*swT,RFref,QP_eop.pRF+vec3(0,0,upL));
            LFref = calc_nextRef(t_now,0.5*swT,LFref,QP_eop.pLF+vec3(0,0,upL));
            RHref = calc_nextRef(t_now,0.5*swT,RHref,QP_eop.pRH+vec3(0,0,upL));
            LHref = calc_nextRef(t_now,0.5*swT,LHref,QP_eop.pLH+vec3(0,0,upL));
        }
        else if(t_now<swT-0.5*dt)
        {
            RFref = calc_nextRef(t_now,swT,RFref,QP_eop.pRF);
            LFref = calc_nextRef(t_now,swT,LFref,QP_eop.pLF);
            RHref = calc_nextRef(t_now,swT,RHref,QP_eop.pRH);
            LHref = calc_nextRef(t_now,swT,LHref,QP_eop.pLH);
        }
        else
        {
            RFref.setX(QP_eop.pRF);
            RFref.dXddX0();
            LFref.setX(QP_eop.pLF);
            LFref.dXddX0();
            RHref.setX(QP_eop.pRH);
            RHref.dXddX0();
            LHref.setX(QP_eop.pLH);
            LHref.dXddX0();
        }
        //landing detection here later

        QuadJoints QJenc = WS.JointPosEnc;
        QJenc.qPel = quat();
        QuadPos QPee = Oi.FK(QJenc);
        double midcomp;
        double midcnt = 0;
        vec3 midpoint = vec3(0,0,0);
        {midpoint = midpoint+QPee.pRF;midcnt++;}
        {midpoint = midpoint+QPee.pLF;midcnt++;}
        {midpoint = midpoint+QPee.pRH;midcnt++;}
        {midpoint = midpoint+QPee.pLH;midcnt++;}
        if(midcnt>0.5){midpoint = midpoint/midcnt;}
        midcomp = L0-(QPee.pCOM.z-midpoint.z);

        dcomz = upV-t_now*g;

        if(NO_ROBOT_TEST)
        {midcomp = 0.03;}
        if(midcomp>0.02&&t_now>(flyT))//v must be <-des_V.z/2
        {
            HOPPHASE= 0;
            RFcontact = LFcontact = RHcontact = LHcontact = true;
            cout<<"landed " <<L0<<" "<<midcomp<<" "<<t_now<<endl;
            cout<<"dCOM "<<QP.dCOM.x<<" "<<QP.dCOM.y<<endl;
            cout<<"jumpV "<<upV<<" flyT "<<flyT<<endl;

            FootLandCnt[0] = LandCnt;
            FootLandCnt[1] = LandCnt;
            FootLandCnt[2] = LandCnt;
            FootLandCnt[3] = LandCnt;
            t_now = 0;
        }


        lastHOPPHASE = 1;
        COMrefXdXddX.setddX(vec3(0,0,-g));
        COMrefXdXddX.setdX(vec3(dcomx,dcomy,dcomz));
        COMrefXdXddX.setX(COMrefXdXddX.X()+COMrefXdXddX.dX()*dt);

    }

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


    if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
    {
        FFz_ref();
        PRONK_control();
    }
    else
    {

        AIR_control();
    }


    save_onestep(Qcnt);
    Qcnt++;
    t_now+=dt;

    QuadJoints QJout = Oi.IK_COM(QP_controled);


    QJout = adjust_FootZs(QJout);//lets try more

//    QJout.RHR = RollQP[0];
//    QJout.LHR = RollQP[1];
//    QJout.RSR = RollQP[2];
//    QJout.LSR = RollQP[3];

    return QJout;

}

void OW_Quad::AIR_control()
{

    for(int i=0;i<4;i++)
    {
        FootZctRef[i] =0.8*FootZctRef[i];
        dFootZctRef[i] = 0.0;
        FootZctFT[i] = FootZctRef[i];
    }
    RFz_ref = 0.0;
    LFz_ref = 0.0;
    RHz_ref = 0.0;
    LHz_ref = 0.0;

    QP.dqPel =vec3(0,0,0);// vec3(-Kdxf*WS.IMUangle.x,-Kdyf*WS.IMUangle.y,0);//????


    RFx_ref=0.0;
    LFx_ref=0.0;
    RHx_ref=0.0;
    LHx_ref=0.0;
    RFy_ref=0.0;
    LFy_ref=0.0;
    RHy_ref=0.0;
    LHy_ref=0.0;

    //here, control for leg vertical movement


    QP_controled = QP;
    QP_controled.pRF.z =QP.pRF.z + FootZctFT[0];
    QP_controled.pLF.z =QP.pLF.z + FootZctFT[1];
    QP_controled.pRH.z =QP.pRH.z + FootZctFT[2];
    QP_controled.pLH.z =QP.pLH.z + FootZctFT[3];

    JointGains Gains;

    control_joints_jump(Gains);//stance leg damping only?


}


void OW_Quad::PRONK_control()
{
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

    QP_controled = QP;
    QP_controled.pRF.z =QP.pRF.z + FootZctFT[0];
    QP_controled.pLF.z =QP.pLF.z + FootZctFT[1];
    QP_controled.pRH.z =QP.pRH.z + FootZctFT[2];
    QP_controled.pLH.z =QP.pLH.z + FootZctFT[3];

    JointGains Gains;
    Gains.HRG.Kp_stand = 50;
    Gains.HRG.Kd_stand = 1.5;
    Gains.KNG.Kp_stand = 30;
    Gains.KNG.Kd_stand = 1.0;

    Gains.KNGland.Kp_swing = 10;
    Gains.KNGland.Kd_swing = 0.3;

    control_joints_jump(Gains);//stance leg damping only?


}
