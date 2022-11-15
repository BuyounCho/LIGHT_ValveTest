
#include "ow_quad.h"
int SW_seq[4] = {2,0,3,1};//wave2
//int SW_seq[4] = {0,2,1,3};//wave
//general convention, RH->RF->LH->LF

void OW_Quad::MOVE_LEGS_SLOWWAVE()
{
    VectorNd FPR;
    double t_u = WP.step_T*(1.0-WP.dsp_ratio)*upRatio;
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

    //always move one by one
    VectorNd Fps,FpszF;
//    step_T_each = WP.step_T/(2.0-WP.overlap)*(1.0-WP.dsp_ratio);
//    t_uf = step_T_each*upRatio;
//    t_f2h =  step_T_each*(1.0-WP.overlap);
//    t_uh = t_uf + t_f2h;

    //double st_5,st_4,st_3,st_2,st_1;
    st_1 = t_now;
    st_2 = st_1*st_1;
    st_3 = st_2*st_1;
    st_4 = st_3*st_1;
    st_5 = st_4*st_1;
    if(t_now<0.5*dt)
    {
        if(SW_seq[SWphase]==0)
        {
            oldpFzF = vec3(QP.pRF.z,0,0);
        }
        if(SW_seq[SWphase]==1)
        {
            oldpFzF = vec3(QP.pLF.z,0,0);
        }
        if(SW_seq[SWphase]==2)
        {
            oldpFzF = vec3(QP.pRH.z,0,0);
        }
        if(SW_seq[SWphase]==3)
        {
            oldpFzF = vec3(QP.pLH.z,0,0);
        }
    }
    double wn = 1.0;
    double z = 1.0;//for test
    double Aerr = decomp_A[0]-WS.IMUangle.z;
    if(Aerr> M_PI){Aerr =Aerr-2*M_PI;}
    if(Aerr<-M_PI){Aerr =Aerr+2*M_PI;}
    double ddYawcon = -wn*wn*(Aerr)-2*wn*z*(oldRotZ[1]-WS.IMUomega.z);//rotvelref
    dYawcon = dYawcon + ddYawcon*dt;
    double MaxdYawcon = 15*D2Rf;
    if(dYawcon> MaxdYawcon){dYawcon = MaxdYawcon;}
    if(dYawcon<-MaxdYawcon){dYawcon =-MaxdYawcon;}

    if(WS_NOW==DSP)//now WP.dsp_ratio is zero
    {
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
    }
    else
    {
        if(WS_NOW&UPR&&WS_NOW&SWR)
        {
            if(Mch[0])
            {
                Mch[0] = false;
                oldpFzF = vec3(QP.pRF.z,0,0);
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            if(Mch[1])
            {
                Mch[1] = false;
                oldpFzF = vec3(QP.pLF.z,0,0);
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            if(Mch[2])
            {
                Mch[2] = false;
                oldpFzF = vec3(QP.pRH.z,0,0);
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            if(Mch[3])
            {
                Mch[3] = false;
                oldpFzF = vec3(QP.pLH.z,0,0);
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            FpszF = calc_5th((t_now-dt),t_u,oldpFzF,vec3(upL,0,0));
        }
        else
        {
//            if(isRHLFmove)
//            {
//                vec3 offF = FootTogoLF-COM_nn;
//                offF.z = 0;
//                Flandoff = -(dot(offF,ty))*tan(decomp_A[2])-(dot(offF,tx))*tan(decomp_A[1]);
//            }
//            else
//            {
//                vec3 offF = FootTogoRF-COM_nn;
//                offF.z = 0;
//                Flandoff = -(dot(offF,ty))*tan(decomp_A[2])-(dot(offF,tx))*tan(decomp_A[1]);
//          }
            Flandoff = 0;
            FpszF = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio)*(1-after_down_ratio),oldpFzF,vec3(WP.landing_depth+Flandoff,0,0));
        }


        if(isFirststep)
        {
            FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        }
        else if(isFinishing)
        {
            FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        }
        else if(SW_seq[SWphase]==0)// RFswing
        {
            LFlanded = false;
            RHlanded = false;
            LHlanded = false;
            if((WS_NOW&SWR))
            {
                FootonAir[0] = true;
            }
            if(WS_NOW&AFD)
            {
                FootonAir[0] = false;
                QP.pRF.z = 0.9*QP.pRF.z;
            }

            QP.pLF.z = 0.9*QP.pLF.z;
            QP.pRH.z = 0.9*QP.pRH.z;
            QP.pLH.z = 0.9*QP.pLH.z;

            if(RFlanded==false&&(WS_NOW&SWR))
            {
                QP.pRF.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
                oldpFzF[0] = QP.pRF.z;
                oldpFzF[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
                oldpFzF[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
                QP.dRF.z = oldpFzF[1];
                QP.ddRF.z = oldpFzF[2];
            }
            else
            {
                QP.pRF.z = 0.98*QP.pRF.z;
                QP.pRF.z = 0;
                QP.ddRF.z = 0;
            }

            if(WS_NOW&BFU)
            {
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
            }
            else if(WS_NOW&SWR)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpRFx,vec3(FootTogoRF.x,0,0));

                QP.pRF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRFx[0] = QP.pRF.x;
                oldpRFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRF.x = oldpRFx[1];
                QP.ddRF.x = oldpRFx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpRFy,vec3(FootTogoRF.y,0,0));

                QP.pRF.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRFy[0] = QP.pRF.y;
                oldpRFy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRFy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRF.y = oldpRFy[1];
                QP.ddRF.y = oldpRFy[2];

            }
            else//after down
            {
                QP.dRF.x = QP.ddRF.x = 0;
                QP.dRF.y = QP.ddRF.y = 0;
            }
            if(t_now>t_u&&RFlanded==false)
            {
                if(Mch[0]==false)
                {
                    changetoCcon[0] = true;
                    isPcon[0] = false;
                    Mch[0] = true;
                }
                if(WS.RF_Fz>landing_fthres)
                {
                    oldpFzF[0] = 0;oldpFzF[0] = 0;RFlanded = true; FootonAir[0] = false;
                    printf("RFlanded\n");
                  todmZero[RHP] = todmZero[RKN] = true;

                }
            }
        }
        else if(SW_seq[SWphase]==1)// LFswing
        {
            RFlanded = false;
            RHlanded = false;
            LHlanded = false;
            if((WS_NOW&SWR))
            {
                FootonAir[1] = true;
            }
            if(WS_NOW&AFD)
            {
                FootonAir[1] = false;
                QP.pLF.z = 0.9*QP.pLF.z;
            }

            QP.pRF.z = 0.9*QP.pRF.z;
            QP.pRH.z = 0.9*QP.pRH.z;
            QP.pLH.z = 0.9*QP.pLH.z;

            if(LFlanded==false&&(WS_NOW&SWR))
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
            else if(WS_NOW&SWR)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpLFx,vec3(FootTogoLF.x,0,0));

                QP.pLF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLFx[0] = QP.pLF.x;
                oldpLFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLF.x = oldpLFx[1];
                QP.ddLF.x = oldpLFx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpLFy,vec3(FootTogoLF.y,0,0));

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
            if(t_now>t_u&&LFlanded==false)
            {
                if(Mch[1]==false)
                {
                    changetoCcon[1] = true;
                    isPcon[1] = false;
                    Mch[1] = true;
                }
                if(WS.LF_Fz>landing_fthres)
                {
                    oldpFzF[1] = 0;oldpFzF[1] = 0;LFlanded = true; FootonAir[1] = false;
                    printf("LFlanded\n");
                  todmZero[LHP] = todmZero[LKN] = true;

                }
            }
        }
        else if(SW_seq[SWphase]==2)// RHswing
        {
            LHlanded = false;
            RFlanded = false;
            LFlanded = false;
            if((WS_NOW&SWR))
            {
                FootonAir[2] = true;
            }
            if(WS_NOW&AFD)
            {
                FootonAir[2] = false;
                QP.pRH.z = 0.9*QP.pRH.z;
            }

            QP.pLH.z = 0.9*QP.pLH.z;
            QP.pRF.z = 0.9*QP.pRF.z;
            QP.pLF.z = 0.9*QP.pLF.z;

            if(RHlanded==false&&(WS_NOW&SWR))
            {
                QP.pRH.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
                oldpFzF[0] = QP.pRH.z;
                oldpFzF[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
                oldpFzF[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
                QP.dRH.z = oldpFzF[1];
                QP.ddRH.z = oldpFzF[2];
            }
            else
            {
                QP.pRH.z = 0.98*QP.pRH.z;
                QP.pRH.z = 0;
                QP.ddRH.z = 0;
            }

            if(WS_NOW&BFU)
            {
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
            }
            else if(WS_NOW&SWR)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpRHx,vec3(FootTogoRH.x,0,0));

                QP.pRH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRHx[0] = QP.pRH.x;
                oldpRHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRH.x = oldpRHx[1];
                QP.ddRH.x = oldpRHx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpRHy,vec3(FootTogoRH.y,0,0));

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
            if(t_now>t_u&&RHlanded==false)
            {
                if(Mch[2]==false)
                {
                    changetoCcon[2] = true;
                    isPcon[2] = false;
                    Mch[2] = true;
                }
                if(WS.RH_Fz>landing_fthres)
                {
                    oldpFzF[2] = 0;oldpFzF[2] = 0;RHlanded = true; FootonAir[2] = false;
                    printf("RHlanded\n");
                  todmZero[RSP] = todmZero[REB] = true;

                }
            }
        }
        else if(SW_seq[SWphase]==3)// LHswing
        {
            RHlanded = false;
            RFlanded = false;
            LFlanded = false;
            if((WS_NOW&SWR))
            {
                FootonAir[3] = true;
            }
            if(WS_NOW&AFD)
            {
                FootonAir[3] = false;
                QP.pLH.z = 0.9*QP.pLH.z;
            }

            QP.pRH.z = 0.9*QP.pRH.z;
            QP.pRF.z = 0.9*QP.pRF.z;
            QP.pLF.z = 0.9*QP.pLF.z;

            if(LHlanded==false&&(WS_NOW&SWR))
            {
                QP.pLH.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
                oldpFzF[0] = QP.pLH.z;
                oldpFzF[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
                oldpFzF[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
                QP.dLH.z = oldpFzF[1];
                QP.ddLH.z = oldpFzF[2];
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
            else if(WS_NOW&SWR)
            {
                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpLHx,vec3(FootTogoLH.x,0,0));

                QP.pLH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLHx[0] = QP.pLH.x;
                oldpLHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLH.x = oldpLHx[1];
                QP.ddLH.x = oldpLHx[2];

                Fps = calc_5th((t_now-dt),WP.step_T*(1-after_down_ratio)*(1-WP.dsp_ratio)
                               ,oldpLHy,vec3(FootTogoLH.y,0,0));

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
            if(t_now>t_u&&LHlanded==false)
            {
                if(Mch[3]==false)
                {
                    changetoCcon[3] = true;
                    isPcon[3] = false;
                    Mch[3] = true;
                }
                if(WS.LH_Fz>landing_fthres)
                {
                    oldpFzF[3] = 0;oldpFzF[3] = 0;LHlanded = true; FootonAir[3] = false;
                    printf("LHlanded\n");
                  todmZero[LSP] = todmZero[LEB] = true;

                }
            }
        }

    }
    if(isFirststep)
    {

    }
    else if(isFinishing)
    {

    }
    else
    {
        vec3 Fcenter = ZMP;

        vec3 Foff,Hoff;
        if(FootonAir[0]==false)
        {
            Foff = (QP_controled.pRF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pRF.x = Fcenter.x+Foff.x;
            QP.pRF.y = Fcenter.y+Foff.y;
        }
        if(FootonAir[1]==false)
        {
            Foff = (QP_controled.pLF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pLF.x = Fcenter.x+Foff.x;
            QP.pLF.y = Fcenter.y+Foff.y;
        }
        if(FootonAir[2]==false)
        {
            Hoff = (QP_controled.pRH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pRH.x = Fcenter.x+Hoff.x;
            QP.pRH.y = Fcenter.y+Hoff.y;
        }
        if(FootonAir[3]==false)
        {
            Hoff = (QP_controled.pLH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pLH.x = Fcenter.x+Hoff.x;
            QP.pLH.y = Fcenter.y+Hoff.y;
        }
    }
}
QuadJoints OW_Quad::SlowWave_onestep(WalkSensors _WS)
{
    WS = _WS;
    if(Qcnt<2)
    {
        cout<<"changetoCcon"<<endl;
        changetoCcon[0] = true;
        changetoCcon[1] = true;
        changetoCcon[2] = true;
        changetoCcon[3] = true;
    }
    vec3 realStepL;
    calc_decomp();
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


    if(t_now>WP.step_T-0.5*dt)
    {

        estimate_plane();

        realStepL.z = 0;

        vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
        FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
        vec3 FrontVector = FrontVt.normalize();//not really...
        vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();

        sL = FrontVector*(WP.step_L.x) +LeftVector*(WP.step_L.y);
        desCOM = desCOM + sL;


        SWphase++;
        SWphase = SWphase%4;
        isFirststep = false;//hmm....
        printf("nextstep! %d %d desCOM%f %f %f\n",stopcnt, SWphase,desCOM.x,desCOM.y,desCOM.z);
        //oldRotZ = vec3(0,0,0);
        oldRotZ[0] = 0;
        t_now =0;
        qPel_stepstart = QP.qPel;

        if(isFinishing)
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
                ||(WP_next.Gait==SlowTrot||(WP_next.Gait==SlowWave));
        if(walkchanged&&oknextGait)
        {
            if(WP.Gait!=WP_next.Gait)
            {
                desCOM = QP.pCOM;
                sumCOMerr = vec3();
                for(int i=0;i<5;i++)
                {
                    stepLFilter[i] = (FrontVector*(WP.step_L.x) +LeftVector*(WP.step_L.y))/WP.step_T*WP_next.step_T;
                }
            }
            WP = WP_next;
            if(WP_next.Gait==SlowWave){calc3th = false;}

            des_delZ = WP.delZ;
            walkchanged  = false;
        }
        calc3th = false;//always? maybe...

    }

    //if(realStepL.norm()>0.25){printf("too much step! %f %f %f\n",realStepL.x,realStepL.y,realStepL.z); isWalking = false; save_all(2);}
//    if(fabs(WS.IMUangle.x*R2Df)>8){printf("x direction near-fall!n\n"); WalkParams tWP = WP; tWP.Gait = SlowTrot; tWP.step_T = 1.0; changeWalk(tWP);}
//    if(fabs(WS.IMUangle.y*R2Df)>8){printf("y direction near-fall!n\n"); WalkParams tWP = WP; tWP.Gait = SlowTrot; tWP.step_T = 1.0; changeWalk(tWP);}
    fallcheck();



    //COM trajectory generation

    vec3 FrontVt = mat3(qPel_stepstart)*vec3(1,0,0);
    FrontVt = FrontVt-dot(FrontVt,vec3(0,0,1))*vec3(0,0,1);
    vec3 FrontVector = FrontVt.normalize();//not really...
    vec3 LeftVector = cross(vec3(0,0,1),FrontVector).normalize();

    if(isFirststep)
    {
        RFcontact =  LHcontact = true;
        LFcontact =  RHcontact = true;
       // calc3th = true;
    }
    else if(SW_seq[SWphase]==0)// RF swing
    {
        LFcontact = RHcontact = LHcontact = true;
        RFcontact =  false;
    }
    else if(SW_seq[SWphase]==1)// LF swing
    {
        RFcontact = RHcontact = LHcontact = true;
        LFcontact =  false;
    }
    else if(SW_seq[SWphase]==2)// RH swing
    {
        RFcontact = LFcontact = LHcontact = true;
        RHcontact =  false;
    }
    else if(SW_seq[SWphase]==3)// LH swing
    {
        RFcontact = LFcontact = RHcontact = true;
        LHcontact =  false;
    }


    QP_est = state_est();


    bool stable = true;
    if(stable)
    {
        //lets do preview control here
        //
        if(calc3th==false)
        {
           ZMPrs.clear();
           ZMPrs.resize(OP.NL2ms*4);
           cout<<"tnow "<<t_now<<endl;
           for(int i=0;i<OP.NL2ms*4;i++)
           {
               double tt = t_now+dt*i;
               int stepN = tt/WP.step_T;
               vec3 Fcenter;
               vec3 RF,LF,RH,LH;
               RF = QP.pRF; LF = QP.pLF; RH = QP.pRH; LH = QP.pLH;
               if(SW_seq[SWphase]==0){RF = LH+2.0*((RH+LF)*0.5-LH);}
               if(SW_seq[SWphase]==1){LF = RH+2.0*((LH+RF)*0.5-RH);}
               if(SW_seq[SWphase]==2){RH = LF+2.0*((LH+RF)*0.5-LF);}
               if(SW_seq[SWphase]==3){LH = RF+2.0*((RH+LF)*0.5-RF);}



               int sP = SWphase+stepN;
               sP = sP%4;

               double rd = 3.0;//diagonal
               double rc = 1.0;//corner
               double rsum = rd+rd+rc;

               if(SW_seq[sP]==0)//RFswing
               {
                   Fcenter = (rd*LF+rd*RH+rc*LH)/rsum;

               }
               else if(SW_seq[sP]==1)//LFswing
               {
                   Fcenter = (rd*RF+rc*RH+rd*LH)/rsum;
               }
               else if(SW_seq[sP]==2)//RHswing
               {
                  Fcenter = (rd*RF+rc*LF+rd*LH)/rsum;
               }
               else//LHswing
               {
                   Fcenter = (rc*RF+rd*LF+rd*RH)/rsum;
               }
//               if(isFirststep&&stepN==0)
//               {
//                   if(!LR){ Fcenter = ((RF+LF+RH+LH)/4.0)*0.7+(rd*RF+rc*RH+rd*LH)/rsum*0.3;}
//                   else{ Fcenter = ((RF+LF+RH+LH)/4.0)*0.7+(rd*LF+rd*RH+rc*LH)/rsum*0.3;}
//               }
               if(isFinishing||(stopcnt-stepN<0))
               {
                   Fcenter = (RF+LF+RH+LH)/4.0;
               }

               ZMPrs[i] = Fcenter+sL*stepN*0.25;
           }
//           cout<<endl;
//           exit(0);

            calc3th = true;
            OP.preview_init(delZ,QP.pCOM,QP.dCOM,ZMPrs);
        }
        OP.calc_ddCOM_pvddx();
        ZMP = OP.ZMP;

    }


    QP.ddCOM = g*(QP.pCOM-ZMP)/delZ;
    QP.ddCOM.z = 0;



    if(isFinishing)
    {
        QP.ddCOM = vec3(0,0,0);//later zmp control
        QP.dCOM = vec3(0,0,0);//later zmp control
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
    }

    QP.pCOM = QP.pCOM + QP.dCOM*dt + QP.ddCOM*dt*dt/2;
    QP.dCOM = QP.dCOM + QP.ddCOM*dt;

    if(stable)
    {
        QP.pCOM = OP.COMr ;
        QP.dCOM = OP.dCOMr;
    }

    //lets generate....
    if(isFirststep)
    {

    }
    else
    {
        double FR = 0.5;
        vec3 LFV = - FrontVector*WP.FB_L*FR + LeftVector*WP.LR_L*FR;
        vec3 RHV =   FrontVector*WP.FB_L*(1-FR) - LeftVector*WP.LR_L*(1-FR);
        vec3 RFV = - FrontVector*WP.FB_L*FR - LeftVector*WP.LR_L*FR;
        vec3 LHV =   FrontVector*WP.FB_L*(1-FR) + LeftVector*WP.LR_L*(1-FR);
        mat3 RotZ = mat3(vec3(0,0,-1),WP.step_Rot);
        LFV = RotZ*LFV;
        RHV = RotZ*RHV;
        RFV = RotZ*RFV;
        LHV = RotZ*LHV;

        vec3 Fcenter;
        vec3 RF,LF,RH,LH;
        RF = QP.pRF; LF = QP.pLF; RH = QP.pRH; LH = QP.pLH;
        FootTogoRF = QP.pRF;
        FootTogoLH = QP.pLH;
        FootTogoLF = QP.pLF;//should be modefied
        FootTogoRH = QP.pRH;//should be modefied
        if(SW_seq[SWphase]==0&&t_now<0.5*dt)
        {
            printf("FootTogoRF %f %f %f \n",FootTogoRF.x,FootTogoRF.y, FootTogoRF.z);
        }
        if(SW_seq[SWphase]==0){RF = LH+2.0*((RH+LF)*0.5-LH);}
        if(SW_seq[SWphase]==1){LF = RH+2.0*((LH+RF)*0.5-RH);}
        if(SW_seq[SWphase]==2){RH = LF+2.0*((LH+RF)*0.5-LF);}
        if(SW_seq[SWphase]==3){LH = RF+2.0*((RH+LF)*0.5-RF);}
        Fcenter = (RF+LF+RH+LH)*0.25;
        if(SW_seq[SWphase]==0){FootTogoRF = Fcenter + RFV + sL;}
        if(SW_seq[SWphase]==1){FootTogoLF = Fcenter + LFV + sL;}
        if(SW_seq[SWphase]==2){FootTogoRH = Fcenter + RHV + sL;}
        if(SW_seq[SWphase]==3){FootTogoLH = Fcenter + LHV + sL;}

        if(SW_seq[SWphase]==0&&t_now<0.5*dt)
        {
            printf("FootTogoRF %f %f %f \n",FootTogoRF.x,FootTogoRF.y, FootTogoRF.z);
        }
    }

    WS_NOW = time2Wphase_slowwave(t_now);
    //calculate foot trajectory
    MOVE_LEGS_SLOWWAVE();


    ///////////control here

    //wave_control needed
    if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
    {
        FFz_ref();
        DSP_control();
    }
    else
    {
        int flynum = 0;
        for(int i=0;i<4;i++)
        {
            if(FootonAir[i]==true)
            {
                flynum = i;
            }
        }

        FFz_ref_3con(flynum);
        SLOWWAVE_control(flynum);
        //WAVE_CONTROL NEEDED
    }

    save_onestep(Qcnt);
    Qcnt++;
    t_now+=dt;

    QuadJoints QJout = Oi.IK_COM(QP_controled);

    QJout = adjust_FootZs(QJout);

    return QJout;

}
void OW_Quad::SLOWWAVE_control(int noncon)
{

    double Kpy = 0.02*R2Df*2*0.5;
    double Kdy = 0.001*R2Df*0.5;
    double Kpx = 0.01*R2Df*2*0.5;
    double Kdx = 0.0005*R2Df*0.5;

    vec3 IMUref = vec3(0,0,0);
    FC_FB = (+ Kpy*(WS.IMUangle.y-IMUref.y)  + Kdy*WS.IMUomega.y);
    FC_LR = -(- Kpx*(WS.IMUangle.x-IMUref.x) - Kdx*WS.IMUomega.x);
    double MaxFC = 0.08;
    if(FC_FB> MaxFC){FC_FB = MaxFC;}
    if(FC_FB<-MaxFC){FC_FB =-MaxFC;}
    if(FC_LR> MaxFC){FC_LR = MaxFC;}
    if(FC_LR<-MaxFC){FC_LR =-MaxFC;}




    dFootZctRef[0] = (FC_FB-FC_LR);
    dFootZctRef[1] = (FC_FB+FC_LR);
    dFootZctRef[2] = (-FC_FB-FC_LR);
    dFootZctRef[3] = (-FC_FB+FC_LR);


    if(noncon==0)//RF flying
    {
        dFootZctRef[0] = (FC_FB-FC_LR)*0.0;
        dFootZctRef[1] = (FC_FB+0.5*FC_LR);
        dFootZctRef[2] = (-0.5*FC_FB-FC_LR);
        dFootZctRef[3] = (-0.5*FC_FB+0.5*FC_LR);
    }
    if(noncon==1)
    {
        dFootZctRef[0] = (FC_FB-0.5*FC_LR);
        dFootZctRef[1] = (FC_FB+FC_LR)*0.0;
        dFootZctRef[2] = (-0.5*FC_FB-0.5*FC_LR);
        dFootZctRef[3] = (-0.5*FC_FB+FC_LR);
    }
    if(noncon==2)
    {
        dFootZctRef[0] = (0.5*FC_FB-FC_LR);
        dFootZctRef[1] = (0.5*FC_FB+0.5*FC_LR);
        dFootZctRef[2] = (-FC_FB-FC_LR)*0.0;
        dFootZctRef[3] = (-FC_FB+0.5*FC_LR);
    }
    if(noncon==3)
    {
        dFootZctRef[0] = (0.5*FC_FB-0.5*FC_LR);
        dFootZctRef[1] = (0.5*FC_FB+FC_LR);
        dFootZctRef[2] = (-FC_FB-0.5*FC_LR);
        dFootZctRef[3] = (-FC_FB+FC_LR)*0.0;
    }

    bool do_control[4] = {true,true,true,true};
    for(int i=0;i<4;i++)
    {
        do_control[i] = true;
    }
    do_control[noncon] = false;
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
    double Kpyf = wn*wn*Iy/FB_L*40;
    double Kdyf = 2*wn*z*Iy/FB_L*15*0.8;
    double Kpxf = w*wn*Ix/LR_L*40;
    double Kdxf = 2*wn*z*Ix/LR_L*15*0.8;
    conFy = -Kpyf*(WS.IMUangle.y-IMUref.y) - Kdyf*WS.IMUomega.y;
    conFx = -Kpxf*(WS.IMUangle.x-IMUref.x) - Kdxf*WS.IMUomega.x;
    double MG = Oi.M_total*g;

    double rRF = (RFz_ref)/(MG);
    double rLF = (LFz_ref)/(MG);
    double rRH = (RHz_ref)/(MG);
    double rLH = (LHz_ref)/(MG);

    double Mc = 0.6*MG;
    if(conFy>Mc){ conFy = Mc;}
    if(conFy<-Mc){ conFy = -Mc;}
    if(conFx>Mc){ conFx = Mc;}
    if(conFx<-Mc){ conFx = -Mc;}
    double rfc,lfc,rhc,lhc;
    if(noncon==0)//RF flying
    {
        rfc = (conFy-conFx)*rRF*3*0.0;
        lfc = (conFy+conFx*0.5)*rLF*3;
        rhc = (-conFy*0.5-conFx)*rRH*3;
        lhc = (-conFy*0.5+conFx*0.5)*rLH*3;
        double fsum = lfc+rhc+lhc;
        rfc = 0;
        lfc = lfc-fsum/3.0;
        rhc = rhc-fsum/3.0;
        lhc = lhc-fsum/3.0;
    }
    if(noncon==1)
    {
        rfc =  (conFy-conFx*0.5)*rRF*3;
        lfc =  (conFy+conFx)*rLF*3*0.0;
        rhc =  (-conFy*0.5-conFx*0.5)*rRH*3;
        lhc =  (-conFy*0.5+conFx)*rLH*3;
        double fsum = rfc+rhc+lhc;
        rfc = rfc-fsum/3.0;
        lfc = 0;
        rhc = rhc-fsum/3.0;
        lhc = lhc-fsum/3.0;
    }
    if(noncon==2)
    {
        rfc =  (conFy*0.5-conFx)*rRF*3;
        lfc =  (conFy*0.5+conFx*0.5)*rLF*3;
        rhc =  (-conFy-conFx)*rRH*3*0.0;
        lhc =  (-conFy+conFx*0.5)*rLH*3;
        double fsum = rfc+lfc+lhc;
        rfc = rfc-fsum/3.0;
        lfc = lfc-fsum/3.0;
        rhc = 0;
        lhc = lhc-fsum/3.0;
    }
    if(noncon==3)
    {
        rfc =  (conFy*0.5-conFx*0.5)*rRF*3;
        lfc =  (conFy*0.5+conFx)*rLF*3;
        rhc =  (-conFy-conFx*0.5)*rRH*3;
        lhc =  (-conFy+conFx)*rLH*3*0.0;
        double fsum = rfc+lfc+rhc;
        rfc = rfc-fsum/3.0;
        lfc = lfc-fsum/3.0;
        rhc = rhc-fsum/3.0;
        lhc = 0;
    }

    RFz_ref += rfc;
    LFz_ref += lfc;
    RHz_ref += rhc;
    LHz_ref += lhc;

    QP.dqPel =vec3(0,0,0);// vec3(-Kdxf*WS.IMUangle.x,-Kdyf*WS.IMUangle.y,0);//????



    //COM position control
    vec3 mf_ref = (QP.pRF+QP.pLF+QP.pRH+QP.pLH)*0.25;
    vec3 mf_est = (QP_est.pRF+QP_est.pLF+QP_est.pRH+QP_est.pLH)*0.25;
//    mf_ref.z = QP.pRF.z*rRF+QP.pLF.z*rLF
//            +QP.pRH.z*rRH*QP.pLH.z*rLH;
//    mf_est.z = QP_est.pRF.z*rRF+QP_est.pLF.z*rLF
//            +QP_est.pRH.z*rRH*QP_est.pLH.z*rLH;


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

    RFx_ref = M*QP.ddCOM.x*rRF;
    LFx_ref = M*QP.ddCOM.x*rLF;
    RHx_ref = M*QP.ddCOM.x*rRH;
    LHx_ref = M*QP.ddCOM.x*rLH;

    RFy_ref = M*QP.ddCOM.y*rRF;
    LFy_ref = M*QP.ddCOM.y*rLF;
    RHy_ref = M*QP.ddCOM.y*rRH;
    LHy_ref = M*QP.ddCOM.y*rLH;


    RFx_ref+=COMconF.x*rRF;
    LFx_ref+=COMconF.x*rLF;
    RHx_ref+=COMconF.x*rRH;
    LHx_ref+=COMconF.x*rLH;
    RFy_ref+=COMconF.y*rRF;
    LFy_ref+=COMconF.y*rLF;
    RHy_ref+=COMconF.y*rRH;
    LHy_ref+=COMconF.y*rLH;

    RFz_ref+=COMconF.z*rRF;
    LFz_ref+=COMconF.z*rLF;
    RHz_ref+=COMconF.z*rRH;
    LHz_ref+=COMconF.z*rLH;

    double minF = 30;
    double tF;
    if(RFz_ref<minF&&noncon!=0)
    {
        tF = RFz_ref; RFz_ref = minF;
        LFz_ref= LFz_ref-(minF-tF)/2.0;
        RHz_ref= RHz_ref-(minF-tF)/2.0;
        LHz_ref= LHz_ref-(minF-tF)/2.0;
    }
    if(LFz_ref<minF&&noncon!=1)
    {
        tF = LFz_ref; LFz_ref = minF;
        RFz_ref= RFz_ref-(minF-tF)/2.0;
        RHz_ref= RHz_ref-(minF-tF)/2.0;
        LHz_ref= LHz_ref-(minF-tF)/2.0;
    }
    if(RHz_ref<minF&&noncon!=2)
    {
        tF = RHz_ref; RHz_ref = minF;
        RFz_ref= RFz_ref-(minF-tF)/2.0;
        LFz_ref= LFz_ref-(minF-tF)/2.0;
        LHz_ref= LHz_ref-(minF-tF)/2.0;
    }
    if(LHz_ref<minF&&noncon!=3)
    {
        tF = LHz_ref; LHz_ref = minF;
        RFz_ref= RFz_ref-(minF-tF)/2.0;
        LFz_ref= LFz_ref-(minF-tF)/2.0;
        RHz_ref= RHz_ref-(minF-tF)/2.0;
    }

    dFootZctRef[0] = 0;
    dFootZctRef[1] = 0;
    dFootZctRef[2] = 0;
    dFootZctRef[3] = 0;

    if(noncon==0){RFx_ref = RFy_ref = RFz_ref = 0;}
    if(noncon==1){LFx_ref = LFy_ref = LFz_ref = 0;}
    if(noncon==2){RHx_ref = RHy_ref = RHz_ref = 0;}
    if(noncon==3){LHx_ref = LHy_ref = LHz_ref = 0;}

    QP_controled = QP;
    QP_controled.pRF.z =QP.pRF.z + FootZctFT[0];
    QP_controled.pLF.z =QP.pLF.z + FootZctFT[1];
    QP_controled.pRH.z =QP.pRH.z + FootZctFT[2];
    QP_controled.pLH.z =QP.pLH.z + FootZctFT[3];

    control_joints(480,30,120,30);

}
int OW_Quad::time2Wphase_slowwave(double _tnow)
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
