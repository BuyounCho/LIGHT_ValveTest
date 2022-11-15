#include "ow_quad.h"
void OW_Quad::MOVE_LEGS_TROT()
{
    //printf("LF %f RF %f LH %f RH %f\n",WS.LF_Fz, WS.RF_Fz, WS.LH_Fz, WS.RH_Fz);
    //  printf("LF %d RF %d LH %d RH %d\n",LFlanded, RFlanded, LHlanded, RHlanded);
    VectorNd FPR;
    double st_5,st_4,st_3,st_2,st_1;
    st_1 = t_now;
    st_2 = st_1*st_1;
    st_3 = st_2*st_1;
    st_4 = st_3*st_1;
    st_5 = st_4*st_1;
    FPR = calc_5th((t_now-dt),WP.step_T,oldRotZ,vec3(WP.step_Rot,WP.step_Rot/WP.step_T,0));
    double PelRotAngle = FPR[0]*st_5 + FPR[1]*st_4 + FPR[2]*st_3 + FPR[3]*st_2 + FPR[4]*st_1 + FPR[5];
    oldRotZ[0] = PelRotAngle;
    oldRotZ[1] = 5*FPR[0]*st_4 + 4*FPR[1]*st_3 + 3*FPR[2]*st_2 + 2*FPR[3]*st_1 + FPR[4];
    oldRotZ[2] = 20*FPR[0]*st_3 + 12*FPR[1]*st_2 + 6*FPR[2]*st_1 + 2*FPR[3];

    QP.qPel = quat(mat3(vec3(0,0,-1),PelRotAngle))*qPel_stepstart;//post? pre? not sure
    QP.dqPel = vec3(0,0,oldRotZ[1]);
    QP.ddqPel = vec3(0,0,oldRotZ[2]);




    VectorNd Fps, FpszH, FpszF;
    double t_u = WP.step_T*(1.0-WP.dsp_ratio)*upRatio;
    st_1 = t_now;
    st_2 = st_1*st_1;
    st_3 = st_2*st_1;
    st_4 = st_3*st_1;
    st_5 = st_4*st_1;
    if(t_now<0.5*dt)
    {
        if(isRHLFmove)
        {
            oldpFzF = vec3(QP.pLF.z,0,0);
            oldpFzH = vec3(QP.pRH.z,0,0);
            contact_L = (QP_controled.pRF-QP_controled.pLH).norm();
            //dYawcon = 0;
        }
        else
        {
            oldpFzF = vec3(QP.pRF.z,0,0);
            oldpFzH = vec3(QP.pLH.z,0,0);
            contact_L = (QP_controled.pLF-QP_controled.pRH).norm();
            //dYawcon = 0;
        }
        cout<<"contact_L "<<contact_L<<"ori "<<sqrt(WP.LR_L*WP.LR_L+WP.FB_L*WP.FB_L)<<endl;
    }
    double wn = 1.0;
    double z = 1.0*1.0;//for test
    double Aerr = decomp_A[0]-WS.IMUangle.z;
    if(Aerr> M_PI){Aerr =Aerr-2*M_PI;}
    if(Aerr<-M_PI){Aerr =Aerr+2*M_PI;}
    //ddYawcon = -wn*wn*(Aerr)-2*wn*z*(oldRotZ[1]-WS.IMUomega.z);//rotvelref
    ddYawcon = -QP_ddQnow[5];
    dYawcon = dYawcon + ddYawcon*dt;
    double MaxdYawcon = 15*D2Rf;
    if(dYawcon> MaxdYawcon){dYawcon = MaxdYawcon;}
    if(dYawcon<-MaxdYawcon){dYawcon =-MaxdYawcon;}

    if(WS_NOW==DSP)//now WP.dsp_ratio is zero
    {
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        if(isFirststep)
        {

        }
        else if(isFinishing)
        {

        }
        else
        {
            vec3 Fcenter = (QP_controled.pRF+QP_controled.pLF+QP_controled.pRH+QP_controled.pLH)/4.0;

            vec3 Foff,Hoff;

            Fcenter = ZMP;//QP.pCOM;

            Foff = (QP_controled.pRF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pRF.x = Fcenter.x+Foff.x;
            QP.pRF.y = Fcenter.y+Foff.y;

            Foff = (QP_controled.pLF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pLF.x = Fcenter.x+Foff.x;
            QP.pLF.y = Fcenter.y+Foff.y;


            Hoff = (QP_controled.pRH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pRH.x = Fcenter.x+Hoff.x;
            QP.pRH.y = Fcenter.y+Hoff.y;

            Hoff = (QP_controled.pLH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pLH.x = Fcenter.x+Hoff.x;
            QP.pLH.y = Fcenter.y+Hoff.y;



            QP.pRH.z = 0.9*QP.pRH.z;
            QP.pLF.z = 0.9*QP.pLF.z;
            QP.pLH.z = 0.9*QP.pLH.z;
            QP.pRF.z = 0.9*QP.pRF.z;
        }
        //if(WP.Gait==SlowTrot){dYawcon = 0;}
    }
    else
    {
        if(WS_NOW&UPR)
        {
            if(isRHLFmove)
            {
                if(Mchanged)
                {
                    if(!isPcon[0])
                    {
                        QuadJoints QJenc = WS.JointPosEnc;
                        QuadJoints QJref = Oi.IK_COM(QP_controled);
                    }
                    if(!isPcon[3])
                    {
                        QuadJoints QJenc = WS.JointPosEnc;
                        QuadJoints QJref = Oi.IK_COM(QP_controled);
                    }
                    Mchanged = false;
                    oldpFzH = vec3(QP.pRH.z,0,0);
                    oldpFzF = vec3(QP.pLF.z,0,0);
                    oldpRFx = vec3(QP.pRF.x,0,0);
                    oldpRFy = vec3(QP.pRF.y,0,0);
                    oldpLFx = vec3(QP.pLF.x,0,0);
                    oldpLFy = vec3(QP.pLF.y,0,0);
                    oldpRHx = vec3(QP.pRH.x,0,0);
                    oldpRHy = vec3(QP.pRH.y,0,0);
                    oldpLHx = vec3(QP.pLH.x,0,0);
                    oldpLHy = vec3(QP.pLH.y,0,0);

//                    todmZero[RHP] = todmZero[RKN] = true;
//                    todmZero[LSP] = todmZero[LEB] = true;
//                    printf("RFLHL %f\n",Qcnt*dt);
                }

            }
            else
            {
                if(Mchanged)
                {
                    if(!isPcon[1])
                    {
                        QuadJoints QJenc = WS.JointPosEnc;
                        QuadJoints QJref = Oi.IK_COM(QP_controled);
                    }
                    if(!isPcon[2])
                    {
                        QuadJoints QJenc = WS.JointPosEnc;
                        QuadJoints QJref = Oi.IK_COM(QP_controled);
                    }
                    Mchanged = false;
                    oldpFzH = vec3(QP.pLH.z,0,0);
                    oldpFzF = vec3(QP.pRF.z,0,0);
                    oldpRFx = vec3(QP.pRF.x,0,0);
                    oldpRFy = vec3(QP.pRF.y,0,0);
                    oldpLFx = vec3(QP.pLF.x,0,0);
                    oldpLFy = vec3(QP.pLF.y,0,0);
                    oldpRHx = vec3(QP.pRH.x,0,0);
                    oldpRHy = vec3(QP.pRH.y,0,0);
                    oldpLHx = vec3(QP.pLH.x,0,0);
                    oldpLHy = vec3(QP.pLH.y,0,0);

//                    todmZero[LHP] = todmZero[LKN] = true;
//                    todmZero[RSP] = todmZero[REB] = true;
//                    printf("LFRHL %f\n",Qcnt*dt);
                }

            }

            FpszH = calc_5th((t_now-dt),t_u,oldpFzH,vec3(upL,0,0));
            FpszF = calc_5th((t_now-dt),t_u,oldpFzF,vec3(upL,0,0));


        }
        else
        {
            Hlandoff = 0;
            Flandoff = 0;
            if(isRHLFmove)
            {
                vec3 offH = FootTogoRH-COM_nn;
                offH.z = 0;
                vec3 offF = FootTogoLF-COM_nn;
                offF.z = 0;

                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1]);
                Flandoff = -(dot(offF,ty))*tan(decomp_A[2])-(dot(offF,tx))*tan(decomp_A[1]);
            }
            else
            {
                vec3 offH = FootTogoLH-COM_nn;
                offH.z = 0;
                vec3 offF = FootTogoRF-COM_nn;
                offF.z = 0;
                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1]);
                Flandoff = -(dot(offF,ty))*tan(decomp_A[2])-(dot(offF,tx))*tan(decomp_A[1]);
          }
            FpszH = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio),oldpFzH,vec3(WP.landing_depth+Hlandoff,0,0));
            FpszF = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio),oldpFzF,vec3(WP.landing_depth+Flandoff,0,0));
       }

        if(isFirststep)
        {
            oldpRFx = vec3(QP.pRF.x,0,0);
            oldpRFy = vec3(QP.pRF.y,0,0);
            oldpRHx = vec3(QP.pRH.x,0,0);
            oldpRHy = vec3(QP.pRH.y,0,0);
            oldpLFx = vec3(QP.pLF.x,0,0);
            oldpLFy = vec3(QP.pLF.y,0,0);
            oldpLHx = vec3(QP.pLH.x,0,0);
            oldpLHy = vec3(QP.pLH.y,0,0);
        }
        else if(isFinishing)
        {
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        }
        else if(isRHLFmove==true)// RH,LF swing
        {

            LHlanded = false;
            RFlanded = false;            
            FootonAir[0] = false;
            FootonAir[3] = false;

            if(WS_NOW&UPR)
            {
                FootonAir[1] = true;
                FootonAir[2] = true;

            }
            if(WS_NOW&AFD)
            {
                FootonAir[1] = false;
                FootonAir[2] = false;
                if(RHlanded)
                {
                    QP.pRH.z = 0.9*QP.pRH.z;
                }
                if(LFlanded)
                {
                    QP.pLF.z = 0.9*QP.pLF.z;
                }
            }

            QP.pLH.z = 0.9*QP.pLH.z;
            QP.pRF.z = 0.9*QP.pRF.z;


            if(WS_NOW!=DSP)
            {
                vec3 Fcenter = (QP_controled.pLH+QP_controled.pRF)/2.0;

                Fcenter = QP.pCOM;

                vec3 Hoff = (QP_controled.pLH-Fcenter).normalize()*contact_L*0.5;

                Hoff = (QP_controled.pLH-Fcenter);

                Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
                QP.pLH.x = Fcenter.x+Hoff.x;
                QP.pLH.y = Fcenter.y+Hoff.y;
                vec3 Foff = (QP_controled.pRF-Fcenter).normalize()*contact_L*0.5;

                Foff = (QP_controled.pRF-Fcenter);

                Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
                QP.pRF.x = Fcenter.x+Foff.x;
                QP.pRF.y = Fcenter.y+Foff.y;

            }

            if(RHlanded==false)
            {
                QP.pRH.z =FpszH[0]*st_5 + FpszH[1]*st_4 + FpszH[2]*st_3 + FpszH[3]*st_2 + FpszH[4]*st_1 + FpszH[5];
                oldpFzH[0] = QP.pRH.z;
                oldpFzH[1] = 5*FpszH[0]*st_4 + 4*FpszH[1]*st_3 + 3*FpszH[2]*st_2 + 2*FpszH[3]*st_1 + FpszH[4];
                oldpFzH[2] = 20*FpszH[0]*st_3 + 12*FpszH[1]*st_2 + 6*FpszH[2]*st_1 + 2*FpszH[3];
                QP.dRH.z = oldpFzH[1];
                QP.ddRH.z = oldpFzH[2];
            }
            else
            {
                QP.pRH.z = 0.98*QP.pRH.z;
                QP.dRH.z = 0;
                QP.ddRH.z = 0;
            }

            if(WS_NOW&BFU)
            {
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
            }
            else if((WS_NOW&SWR))//&&RHlanded==false)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRHx,vec3(FootTogoRH.x,0,0));

                QP.pRH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRHx[0] = QP.pRH.x;
                oldpRHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRH.x = oldpRHx[1];
                QP.ddRH.x = oldpRHx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRHy,vec3(FootTogoRH.y,0,0));

                QP.pRH.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRHy[0] = QP.pRH.y;
                oldpRHy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRHy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRH.y = oldpRHy[1];
                QP.ddRH.y = oldpRHy[2];
            }
            else//after down
            {
                QP.dRH.x = QP.ddRH.x = 0;
                QP.dRH.y = QP.ddRH.y = 0;
            }

            if(LFlanded==false)
            {
                QP.pLF.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
                oldpFzF[0] = QP.pLF.z;
                oldpFzF[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
                oldpFzF[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
                QP.dLF.z = oldpFzF[1];
                QP.ddLF.z = oldpFzF[2];
            }
            else
            {
                QP.pLF.z = 0.98*QP.pLF.z;
                QP.dLF.z = 0;
                QP.ddLF.z = 0;
            }

            if(WS_NOW&BFU)
            {
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
            }
            else if((WS_NOW&SWR))//&&LFlanded==false)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLFx,vec3(FootTogoLF.x,0,0));

                QP.pLF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLFx[0] = QP.pLF.x;
                oldpLFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLF.x = oldpLFx[1];
                QP.ddLF.x = oldpLFx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLFy,vec3(FootTogoLF.y,0,0));

                QP.pLF.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLFy[0] = QP.pLF.y;
                oldpLFy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLFy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLF.y = oldpLFy[1];
                QP.ddLF.y = oldpLFy[2];

            }
            else//after down
            {
                QP.dLF.x = QP.ddLF.x = 0;
                QP.dLF.y = QP.ddLF.y = 0;
            }
            if(t_now>t_u + (WP.step_T*(1.0-WP.dsp_ratio)-t_u)*0.3)
            {
                if(Mchanged==false)
                {
                    if(NO_ROBOT_TEST==false)
                    {
                        changetoCcon[1] = true;
                        changetoCcon[2] = true;
                    }
                    isPcon[1] = isPcon[2] = false;
                    Mchanged = true;
                }
                if(WS_NOW&DWR&&LFlanded==false)
                {
                    if(dms[LKN]-dms_old[LKN]<-3)
                    //if(WS.LF_Fz>landing_fthres)
                    {
                        oldpFzF[1] = 0;oldpFzF[2] = 0;LFlanded = true; FootonAir[1] = false;
                        FootLandCnt[1] = LandCnt;
                        printf("LFlanded\n");
                     // todmZero[LHP] = todmZero[LKN] = true;
                       // todmZero[LKN] = true;

                    }
                }
                if(WS_NOW&DWR&&RHlanded==false)
                {
                    //if(dms[REB]-dms_old[REB]<-3)
                    //if(WS.RH_Fz>landing_fthres)
                    {
                        oldpFzH[1] = 0;oldpFzH[2] = 0;RHlanded = true; FootonAir[2] = false;
                        FootLandCnt[2] = LandCnt;
                        printf("RHlanded\n");
                      // todmZero[RSP] = todmZero[REB] = true;
                       // todmZero[REB] = true;
                    }
                }
            }


        }
        else// LH,RF swing
        {
            RHlanded = false;
            LFlanded = false;            
            FootonAir[1] = false;
            FootonAir[2] = false;

            if(WS_NOW&UPR)
            {
                FootonAir[0] = true;
                FootonAir[3] = true;

            }
            if(WS_NOW&AFD)
            {
                FootonAir[0] = false;
                FootonAir[3] = false;
                if(LHlanded)
                {
                    QP.pLH.z = 0.9*QP.pLH.z;
                }
                if(RFlanded)
                {
                    QP.pRF.z = 0.9*QP.pRF.z;
                }
            }

            QP.pRH.z = 0.9*QP.pRH.z;
            QP.pLF.z = 0.9*QP.pLF.z;
            if(WS_NOW!=DSP)
            {

                vec3 Fcenter = (QP_controled.pRH+QP_controled.pLF)/2.0;

                Fcenter = QP.pCOM;

                vec3 Hoff = (QP_controled.pRH-Fcenter).normalize()*contact_L*0.5;

                Hoff = (QP_controled.pRH-Fcenter);

                Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
                QP.pRH.x = Fcenter.x+Hoff.x;
                QP.pRH.y = Fcenter.y+Hoff.y;
                vec3 Foff = (QP_controled.pLF-Fcenter).normalize()*contact_L*0.5;

                Foff = (QP_controled.pLF-Fcenter);

                Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
                QP.pLF.x = Fcenter.x+Foff.x;
                QP.pLF.y = Fcenter.y+Foff.y;
            }

            if(LHlanded==false)
            {
                QP.pLH.z =FpszH[0]*st_5 + FpszH[1]*st_4 + FpszH[2]*st_3 + FpszH[3]*st_2 + FpszH[4]*st_1 + FpszH[5];
                oldpFzH[0] = QP.pLH.z;
                oldpFzH[1] = 5*FpszH[0]*st_4 + 4*FpszH[1]*st_3 + 3*FpszH[2]*st_2 + 2*FpszH[3]*st_1 + FpszH[4];
                oldpFzH[2] = 20*FpszH[0]*st_3 + 12*FpszH[1]*st_2 + 6*FpszH[2]*st_1 + 2*FpszH[3];
                QP.dLH.z = oldpFzH[1];
                QP.ddLH.z = oldpFzH[2];
            }
            else
            {
                QP.pLH.z = 0.98*QP.pLH.z;
                QP.dLH.z = 0;
                QP.ddLH.z = 0;
            }


            if(WS_NOW&BFU)
            {
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            else if((WS_NOW&SWR))//&&LHlanded==false)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLHx,vec3(FootTogoLH.x,0,0));

                QP.pLH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLHx[0] = QP.pLH.x;
                oldpLHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLH.x = oldpLHx[1];
                QP.ddLH.x = oldpLHx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLHy,vec3(FootTogoLH.y,0,0));

                QP.pLH.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLHy[0] = QP.pLH.y;
                oldpLHy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLHy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLH.y = oldpLHy[1];
                QP.ddLH.y = oldpLHy[2];
            }
            else//after down
            {
                QP.dLH.x = QP.ddLH.x = 0;
                QP.dLH.y = QP.ddLH.y = 0;
            }
            if(RFlanded==false)
            {
                QP.pRF.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
                oldpFzF[0] =QP.pRF.z;
                oldpFzF[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
                oldpFzF[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
                QP.dRF.z = oldpFzF[1];
                QP.ddRF.z = oldpFzF[2];
            }
            else
            {
                QP.pRF.z = 0.98*QP.pRF.z;
                QP.dRF.z = 0;
                QP.ddRF.z = 0;
            }

            if(WS_NOW&BFU)
            {
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
            }
            else if((WS_NOW&SWR))//&&RFlanded==false)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRFx,vec3(FootTogoRF.x,0,0));

                QP.pRF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRFx[0] = QP.pRF.x;
                oldpRFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRF.x = oldpRFx[1];
                QP.ddRF.x = oldpRFx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRFy,vec3(FootTogoRF.y,0,0));

                QP.pRF.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRFy[0] = QP.pRF.y;
                oldpRFy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRFy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRF.y = oldpRFy[1];
                QP.ddRF.y = oldpRFy[2];
            }
            else
            {
                QP.dRF.x = QP.ddRF.x = 0;
                QP.dRF.y = QP.ddRF.y = 0;
            }

            if(t_now>t_u + (WP.step_T*(1.0-WP.dsp_ratio)-t_u)*0.3)
            {
                if(Mchanged==false)
                {
                    if(NO_ROBOT_TEST==false)
                    {
                        changetoCcon[0] = true;
                        changetoCcon[3] = true;
                    }
                    isPcon[0] = isPcon[3] = false;
                    Mchanged = true;
                }
                if(WS_NOW&DWR&&RFlanded==false)
                {
                    if(dms[RKN]-dms_old[RKN]<-3)
                    //if(WS.RF_Fz>landing_fthres)
                    {
                        RFlanded = true; oldpFzF[1] = 0; oldpFzF[2] = 0;FootonAir[0] = false;
                        FootLandCnt[0] = LandCnt;
                        printf("RFlanded\n");
                        //todmZero[RHP] = todmZero[RKN] = true;
                        //todmZero[RKN] = true;
                    }
                }
                if(WS_NOW&DWR&&LHlanded==false)
                {
                    //if(dms[LEB]-dms_old[LEB]<-3)
                    //if(WS.LH_Fz>landing_fthres)
                    {
                        LHlanded = true; oldpFzH[1] = 0; oldpFzH[2] = 0;FootonAir[3] = false;
                        FootLandCnt[3] = LandCnt;
                        printf("LHlanded\n");
                         // todmZero[LSP] = todmZero[LEB] = true;
                        //todmZero[LEB] = true;
                    }
                }
            }


        }
    }
}
QuadJoints OW_Quad::Trot_onestep(WalkSensors _WS)
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
    vec3 realStepL;
    if(DO_ADJUST_SLOPE){calc_decomp_slope();}
    else{calc_decomp();}
    //LIPM dynamics
    delZ = QP.pCOM.z;
    w = sqrtp(g/delZ);///////////////////////////////////////////////changed
    if(QP.pCOM.z>des_delZ+0.001)////0.01m/s
    {
        QP.pCOM.z = QP.pCOM.z - 0.01*dt;
        if(Qcnt%250==0)
        {cout<<"COMdown"<<endl;}
    }
    if(QP.pCOM.z<des_delZ-0.001)
    {
        QP.pCOM.z = QP.pCOM.z + 0.01*dt;
        if(Qcnt%250==0)
        {cout<<"COMup"<<endl;}
    }

    if(t_now>WP.step_T+0.5*dt)
    {
        //////////////always 4 foot contact here?
        estimate_plane();
        ////////////////
        QP_est = Oi.FK(WS.JointPosEnc);

        if(isRHLFmove)
        {
            //realStepL = (QP.pRH+QP.pLF)/2-(QP.pLH+QP.pRF)/2;
            realStepL = (QP_est.pRH+QP_est.pLF)/2-(QP_est.pLH+QP_est.pRF)/2;
        }
        else
        {
            //realStepL = (QP.pLH+QP.pRF)/2-(QP.pRH+QP.pLF)/2;
            realStepL = (QP_est.pLH+QP_est.pRF)/2-(QP_est.pRH+QP_est.pLF)/2;
        }

        realStepL.z = 0;
        estCOM = estCOM+realStepL;

        vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
        FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
        vec3 FrontVector = FrontVt.normalize();//not really...
        vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();
        vec3 ss = vec3();
        for(int i=4;i>=0;i--)
        {
            stepLFilter[i+1] = stepLFilter[i];
        }
        stepLFilter[0] = FrontVector*(WP.step_L.x) +LeftVector*(WP.step_L.y);
        if(WP.step_L.norm()>1e-6)
        {
            for(int i=0;i<5;i++)
            {
                stepLFilter[i] = stepLFilter[0];//hmmmmm
            }
        }
        for(int i=0;i<5;i++)
        {
            ss = ss+stepLFilter[i]*0.2;
        }
        desCOM = desCOM + ss;

  //      desCOM = desCOM + FrontVector*(WP.step_L.x) +LeftVector*(WP.step_L.y);
        double maxff = 0.1;
        double maxll = 0.1*1.5;
//        double kkfb = 0.05*4;
//        double kklr = 0.05*4;
        double kkfb = 0.2;// 0.3*0.5;//hmm.......
        double kklr = 0.2;//0.3*0.5;//hmm.......
        if(NO_ROBOT_TEST)
        {
            estCOM = QP.pCOM;
        }


//        ff = kkfb*dot(FrontVector,(desCOM-QP.pCOM));
//        ll = kklr*dot(LeftVector,(desCOM-QP.pCOM));
        double ffoff = 0.03;//0.06;//I want to make this to be zero
        ff = kkfb*dot(FrontVector,(desCOM-estCOM))+ffoff;//ff feedforward
        ll = kklr*dot(LeftVector,(desCOM-estCOM));


        if(ff>maxff+ffoff){ff = maxff+ffoff;}
        if(ff<-maxff+ffoff){ff = -maxff+ffoff;}
        if(ll>maxll){ll = maxll;}
        if(ll<-maxll){ll = -maxll;}
        if(((desCOM-estCOM).norm())>0.3)
        {
            desCOM = estCOM+((desCOM-estCOM))/((desCOM-estCOM).norm())*0.3;
        }

        cout<<"ff "<<ff<<" ll "<<ll<<endl;//lets see this reduce/increase
        isRHLFmove = !isRHLFmove;
        isFirststep = false;
        printf("nextstep! %d realStepL%f %f %f\n",stopcnt, realStepL.x, realStepL.y,realStepL.z);
        printf("estCOM %f %f %f pCOM %f %f %f\n",estCOM.x,estCOM.y,estCOM.z,QP.pCOM.x, QP.pCOM.y, QP.pCOM.z);
        //oldRotZ = vec3(0,0,0);
        oldRotZ[0] = 0;
        oldRotX[0] = 0;
        oldRotY[0] = 0;
        t_now =0;
        qPel_stepstart = QP.qPel;
        if(isRHLFmove)//late_landing
        {
            if(WS.RF_Fz<noncontact_fthres){RFlateland = true;}
            if(WS.LH_Fz<noncontact_fthres){LHlateland = true;}            
        }
        else
        {
            if(WS.LF_Fz<noncontact_fthres){LFlateland = true;}
            if(WS.RH_Fz<noncontact_fthres){RHlateland = true;}            
        }
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
//            isCOMadjusting_finish = true;
//            Qcnt = 0;
//            t_now = 0;
            save_all();

            return QJenc;
        }
        if(isStopping)
        {
            stopcnt--;
            if(stopcnt<0)
            {
                isStopping = false;
                isFinishing = true;
            }
        }
        bool oknextGait = (WP_next.Gait==Standing)||(WP_next.Gait==Trot)
                ||(WP_next.Gait==Wave)||(WP_next.Gait==Wave2);
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
            if(WP_next.Gait==Wave){calc3th = false;}
            if(WP_next.Gait==Trot)
            {
                double DCgain = -60.0;
            }
            WP = WP_next;


            des_delZ = WP.delZ;
            walkchanged  = false;
        }

    }

    fallcheck();



    //COM trajectory generation

    vec3 RF2LH = QP.pLF-QP.pRF;
    vec3 LF2RH = QP.pRF-QP.pLF;

    vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
    FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
    vec3 FrontVector = FrontVt.normalize();//not really...
    vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();

    vec3 sL = FrontVector*(WP.step_L.x+ff)+LeftVector*(WP.step_L.y+ll);
    Next_ZMP_offset = sL/(pow(e,w*WP.step_T)-1)*1.6;//1.6 is almost best value

    //it's feedforward tuning, so it shouldn;t affet feedback
    if(isFirststep)
    {
        ZMPr = -Next_ZMP_offset/((pow(e,w*WP.step_T)-1));
        ZMP = ZMPr;
    }
    else if(isRHLFmove==true)// RH,LF swing
    {
        ZMPr = (QP.pLH+QP.pRF)*0.5;//later controllable factor
        vec3 F2H = (QP.pLH-QP.pRF).normalize();
        //ZMP = QP.pRF + dot((QP.pCOM-QP.pRF),F2H)*F2H + dot(Kp*(QP.pCOM-COMr)+Kd*(QP.dCOM-dCOMr),F2H)*F2H;
        ZMP = ZMPr;// + dot(Kp*(QP.pCOM-COMr)+Kd*(QP.dCOM-dCOMr),F2H)*F2H;


        RFcontact =  LHcontact = true;
        LFcontact =  RHcontact = false;
    }
    else
    {
        ZMPr = (QP.pRH+QP.pLF)*0.5;//later controllable factor
        vec3 F2H = (QP.pRH-QP.pLF).normalize();
        //ZMP = QP.pLF + dot((QP.pCOM-QP.pLF),F2H)*F2H + dot(Kp*(QP.pCOM-COMr)+Kd*(QP.dCOM-dCOMr),F2H)*F2H;
        ZMP = ZMPr;// + dot(Kp*(QP.pCOM-COMr)+Kd*(QP.dCOM-dCOMr),F2H)*F2H;

        RFcontact =  LHcontact = false;
        LFcontact =  RHcontact = true;
    }


    QP_est = state_est();



    if(isFirststep)
    {
        //QP.ddCOM = g*(QP.pCOM-ZMP)/delZ;
        vec3 des_COM = 0.25*(QP.pRF+QP.pLF+QP.pRH+QP.pLH)+Next_ZMP_offset;
        VectorNd Fx, Fy;
        Fx = calc_5th((t_now-dt),WP.step_T,oldCOMx,vec3(des_COM.x,0,0));
        Fy = calc_5th((t_now-dt),WP.step_T,oldCOMy,vec3(des_COM.y,0,0));
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
        vec3 ddCref = vec3(oldCOMx[2],oldCOMy[2],0);
        ZMP = QP.pCOM-delZ/g*ddCref;
        ZMP.z = 0;
    }
    else if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
    {
        QP.ddCOM = g*(QP.pCOM-ZMP)/delZ;
    }
    else
    {
        QP.ddCOM = g*(QP.pCOM-ZMP)/delZ;//nocon//nnnnnnnnnnnnn
    }

//    vec3 tt = calc_ddcom_qp(QP,FootonAir);

    ddCOMLIPM = QP.ddCOM;

    QP.ddCOM = ddCOMLIPM;
//    QP.ddCOM.x = ddCOMnotch[0].do_filt((ddCOMLIPM.x));
//    QP.ddCOM.y = ddCOMnotch[1].do_filt((ddCOMLIPM.y));


    double friccoef = 0.8;
    double MaxA = friccoef*g;
    QP.ddCOM.z = 0;
    if(QP.ddCOM.norm()>MaxA)
    {
        QP.ddCOM = QP.ddCOM/QP.ddCOM.norm()*MaxA;//friction cone
    }



    if(isFinishing)
    {
        QP.ddCOM = vec3(0,0,0);//later zmp control
        QP.dCOM = vec3(0,0,0);//later zmp control
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
    }




    CP = QP.pCOM + QP.dCOM/w;
    CP_est = QP_est.pCOM + QP_est.dCOM/w;
    if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
    {//all contact
    }
    else
    {
        CP_nn = pow(e,w*(WP.step_T-t_now))*(CP-ZMP)+ZMP;
        CP_est_nn = pow(e,w*(WP.step_T-t_now))*(CP_est-ZMP)+ZMP;
        vec3 A1 = ((QP.pCOM-ZMP)+QP.dCOM/w)*0.5;
        vec3 A2 = ((QP.pCOM-ZMP)-QP.dCOM/w)*0.5;

        COM_nn = A1*pow(e,w*(WP.step_T-t_now)) +A2*pow(e,-w*(WP.step_T-t_now)) + ZMP;


        double k_footstep_front = 0.6;//1.0//bigger, more aggressive
          double k_footstep_left = 0.6;//1.0//bigger, more aggressive
        vec3 offFF = 0.5*sL;//


        vec3 offF = dot((CP_nn-(COM_nn+offFF)),FrontVector)*FrontVector;
        vec3 offL = dot((CP_nn-(COM_nn+offFF)),LeftVector)*LeftVector;
        //maybe change to 1.0 after change ddCOM
        CP_nn = (COM_nn+offFF) + k_footstep_front*offF + k_footstep_left*offL;
        if(CP_nn.x-COM_nn.x>foot_xlim){CP_nn.x = COM_nn.x + foot_xlim;}
        if(CP_nn.x-COM_nn.x<-foot_xlim){CP_nn.x = COM_nn.x - foot_xlim;}
        if(CP_nn.y-COM_nn.y>foot_ylim){CP_nn.y = COM_nn.y + foot_ylim;}
        if(CP_nn.y-COM_nn.y<-foot_ylim){CP_nn.y = COM_nn.y - foot_ylim;}
        CP_nn.z = 0;
        ZMP.z = 0;
        vec3 stepNN = (ZMP-CP_nn);
        vec3 ZMPest;


    }    
    if(!isFirststep)
    {
        QP.ddCOM = QP_ddCOM;//follow last input?
        QP.pCOM = QP.pCOM + QP.dCOM*dt + QP_ddCOM*dt*dt/2;
        QP.dCOM = QP.dCOM + QP_ddCOM*dt;
    }


    //lets generate....
    if(isFirststep)
    {
        FootTogoRF = QP.pRF;
        FootTogoLF = QP.pLF;
        FootTogoRH = QP.pRH;
        FootTogoLH = QP.pLH;
    }
    else if(isRHLFmove==true)// RH,LF swing
    {
        double FR = 0.5;
        vec3 LFV = - FrontVector*WP.FB_L*FR + LeftVector*WP.LR_L*FR;
        vec3 RHV =   FrontVector*WP.FB_L*(1-FR) - LeftVector*WP.LR_L*(1-FR);
        mat3 RotZ = mat3(vec3(0,0,-1),WP.step_Rot);
        LFV = RotZ*LFV;
        RHV = RotZ*RHV;

        vec3 Center = CP_nn - Next_ZMP_offset;

        FootTogoLF = Center + LFV;
        FootTogoRH = Center + RHV;


        FootTogoLH = QP.pLH;//should be modefied
        FootTogoRF = QP.pRF;//should be modefied
    }
    else
    {
        double FR = 0.5;
        vec3 RFV = - FrontVector*WP.FB_L*FR - LeftVector*WP.LR_L*FR;
        vec3 LHV =   FrontVector*WP.FB_L*(1-FR) + LeftVector*WP.LR_L*(1-FR);
        mat3 RotZ = mat3(vec3(0,0,-1),WP.step_Rot);
        RFV = RotZ*RFV;
        LHV = RotZ*LHV;

        vec3 Center = CP_nn - Next_ZMP_offset;


        FootTogoRF = Center + RFV;
        FootTogoLH = Center + LHV;

        FootTogoLF = QP.pLF;//should be modefied
        FootTogoRH = QP.pRH;//should be modefied
    }
    if(Oi.NANNAN)
    {
        printf("inverse nan\n");
        printf("time %f %d\n",Qcnt*dt,Qcnt);
        isFinishing = true;
    }
    for(int i=0;i<3;i++)
    {
//        if(isnan(CP_nn[i]))
        if(0)
        {
            printf("CP_nn nan\n");
            isFinishing = true;
        }
    }

    //calculate foot trajectory
    WS_NOW = time2Wphase(t_now);
    if(DO_ADJUST_SLOPE){MOVE_LEGS_TROT_SLOPE();}
    else{MOVE_LEGS_TROT();}
    bool oktomove = t_now<(WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt-0.2);
    //at least 0.2 sec to move
    //oktomove = false;//not use///////////timechange~~~~
    if(!isFirststep)
    {
        vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
        FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
        vec3 FrontVector = FrontVt.normalize();//not really...
        vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();
        vec3 nominalstep = FrontVector*(WP.step_L.x) +LeftVector*(WP.step_L.y);

        vec3 serr = ((CP_nn - Next_ZMP_offset)-COM_nn)-nominalstep*0.5;
        serr.z = 0;
        if(serr.norm()>0.13&&WP.step_T>0.3&&oktomove)
        {
            WP_next = WP;
            WP_next.step_T = 0.25;//just fastest
            std::cout<<"timechange!!"<<Rcnt*dt<<std::endl;
            std::cout<<"serr"<<serr.x<<" "<<serr.y<<std::endl;
            desCOM = estCOM;
            walkchanged = true;
        }
    }
    if(WP.step_T<0.28){oktomove = false;}//
    //just testtest
//    if(fabs(WS.IMUangle.x)>4.0*D2Rf||fabs(WS.IMUangle.y)>4.0*D2Rf)
//    {
//        std::cout<<std::endl;
//        std::cout<<"resetCOM!!"<<Rcnt*dt<<std::endl;
//        desCOM = estCOM;
//    }

    if((WS_NOW&BFU||((WS_NOW&SWR)&&oktomove))&&walkchanged&&WP_next.Gait==Trot)
    {
        WP = WP_next;
        double t_u = WP.step_T*(1.0-WP.dsp_ratio)*upRatio;
        if(t_now>t_u)
        {t_now = t_u;}
        des_delZ = WP.delZ;
        walkchanged  = false;
    }

    if(DO_ADJUST_SLOPE)
    {
        set_des_delZ_slope();
    }

    ///////////control here

    if(DO_ADJUST_SLOPE)
    {
        if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
        {
            FFz_ref();
            DSP_control_slope();
        }
        else
        {
            FFz_ref_ssp();
            SSP_control_slope();
        }
    }
    else
    {
        if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
        {
            FFz_ref();
            DSP_control();
        }
        else
        {
            FFz_ref_ssp();
            SSP_control();
        }
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

void OW_Quad::SSP_control()
{

    double Kpy = 0.02*R2Df*2.0;//*0.3;
    double Kdy = 0.001*R2Df*1.5;//*0.3;
    double KpA2 = 0.3*1.5*0.3;//hmmmm
    double decomp_A1ref;
    if(isRHLFmove)
    {
        decomp_A1ref = KpA2*decomp_A[2];
    }
    else
    {
        decomp_A1ref = -KpA2*decomp_A[2];
    }

    FC_FB =  + Kpy*(decomp_A[1]-decomp_A1ref) + Kdy*(decomp_W[1]);
    FC_LR = 0;//-(- Kpx*WS.IMUangle.x - Kdx*WS.IMUomega.x);
    double MaxFC = 0.08;
    if(FC_FB> MaxFC){FC_FB = MaxFC;}
    if(FC_FB<-MaxFC){FC_FB =-MaxFC;}
    if(FC_LR> MaxFC){FC_LR = MaxFC;}
    if(FC_LR<-MaxFC){FC_LR =-MaxFC;}

    dFootZctRef[0] = (FC_FB-FC_LR);
    dFootZctRef[1] = (FC_FB+FC_LR);
    dFootZctRef[2] = (-FC_FB-FC_LR);
    dFootZctRef[3] = (-FC_FB+FC_LR);

    bool do_control[4] = {true,true,true,true};
    if(isRHLFmove)
    {
        do_control[0] = do_control[3] = true;
        do_control[1] = do_control[2] = false;
        LFlateland = false;
        RHlateland = false;
    }
    else
    {
        do_control[0] = do_control[3] = false;
        do_control[1] = do_control[2] = true;
        RFlateland = false;
        LHlateland = false;
    }
    double conRate[4];
    for(int i=0;i<4;i++)
    {
        if(do_control[i])
        {
            FootZctRef[i] = FootZctRef[i]+dFootZctRef[i]*dt;
            double MaxCon = 0.05;
            if(FootZctRef[i]>MaxCon){FootZctRef[i] = MaxCon; if(dFootZctRef[i]>0){dFootZctRef[i] = 0;}}
            if(FootZctRef[i]<-MaxCon){FootZctRef[i] = -MaxCon; if(dFootZctRef[i]<0){dFootZctRef[i] = 0;}}
        }
        else
        {
            FootZctRef[i] =0.8*FootZctRef[i];
            dFootZctRef[i] = 0.0;
        }
        FootZctFT[i] = FootZctRef[i];
        if(do_control[i]){conRate[i] = 0.5;}
        else{conRate[i] = 0.0;}
    }




    QP_controled = QP;
    QP_controled.pRF.z =QP.pRF.z + FootZctFT[0];
    QP_controled.pLF.z =QP.pLF.z + FootZctFT[1];
    QP_controled.pRH.z =QP.pRH.z + FootZctFT[2];
    QP_controled.pLH.z =QP.pLH.z + FootZctFT[3];

    //control_joints(240,15,360,30);
    JointGains Gains;



    control_joints(Gains);//stance leg damping only?


}

void OW_Quad::DSP_control()
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

    control_joints(Gains);//stance leg damping only?


}
int OW_Quad::time2Wphase(double _tnow)
{
    int Wphase;
    double tref =_tnow+0.5*dt;

    if(tref<(1.0-WP.dsp_ratio)*WP.step_T)
    {
        double t_u = WP.step_T*(1.0-WP.dsp_ratio)*upRatio;
        if(tref<t_u)
        {
            Wphase = UPR;
        }
        else
        {
            Wphase = DWR;
        }        
        if(tref<WP.step_T*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
        {
            Wphase = Wphase+BFU;
        }
        else if(tref<WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)
        {
            Wphase = Wphase+SWR;
        }
        else
        {
            Wphase = Wphase+AFD;
        }
    }
    else
    {
        Wphase = DSP;
    }

    return Wphase;
}
