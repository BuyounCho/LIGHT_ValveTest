#include "ow_quad.h"

int OW_Quad::time2Wphase_wave(double _tnow)
{
    int Wphase = 0;
    double tref =fmod(_tnow,WP.step_T);
    int stepN = _tnow/WP.step_T;
    if(tref<t_uf)//upF
    {
        Wphase +=0;
    }
    else if(tref<step_T_each)//downF
    {
        Wphase +=1;
    }
    else
    {
        Wphase +=2;//groundF
    }
    if(tref<t_f2h)//groundH
    {
        Wphase +=20;
    }
    else if(tref<t_uh)//upH
    {
        Wphase +=0;
    }
    else if(tref<t_f2h+step_T_each)//downH
    {
        Wphase +=10;
    }
    else
    {
        Wphase +=20;
    }
    if(stepN%2==1){Wphase+=100;}//LRchanged


    return Wphase;
}
void OW_Quad::MOVE_LEGS_WAVE()
{
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

    //sequence should be foot->arm
    VectorNd Fps, FpszH, FpszF;
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
        cout<<"contact_L "<<contact_L<<"contact_L_des "<<sqrt(WP.LR_L*WP.LR_L+WP.FB_L*WP.FB_L)<<endl;
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

    if(t_now>WP.step_T*(1-WP.dsp_ratio))//now WP.dsp_ratio is zero
    {
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
    }
    else
    {
        if(t_now<t_uf)
        {
            if(Mch[1]&&Mch[2])
            {
                Mch[1]=Mch[2] = false;
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

            }
            if(Mch[0]&&Mch[3])
            {
                Mch[0]=Mch[3] = false;
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
            }
            if(t_now<t_wait)
            {
                FpszF = calc_5th((t_now-dt),t_wait,oldpFzF,vec3(0,0,0));
            }
            else
            {
                FpszF = calc_5th((t_now-dt),t_uf,oldpFzF,vec3(upL,0,0));
            }
        }
        else if(t_now<step_T_each)
        {
            Flandoff = 0;
            if(t_now<step_T_each-t_wait)
            {
                FpszF = calc_5th((t_now-dt),step_T_each-t_wait,oldpFzF,vec3(WP.landing_depth+Flandoff,0,0));
            }
            else
            {

                FpszF = calc_5th((t_now-dt),step_T_each,oldpFzF,vec3(WP.landing_depth+Flandoff,0,0));
            }
        }
        else//stop
        {
            FpszF = calc_5th((t_now-dt),WP.step_T,oldpFzF,vec3(WP.landing_depth,0,0));
        }
        if(t_now<t_f2h)//stop
        {
            FpszH = calc_5th((t_now-dt),t_f2h,oldpFzH,vec3(WP.landing_depth,0,0));
        }
        else if(t_now<t_uh)
        {
            if(t_now<t_f2h+t_wait)
            {
               FpszH = calc_5th((t_now-dt),t_f2h+t_wait,oldpFzH,vec3(0,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),t_uh,oldpFzH,vec3(upL,0,0));
            }
        }
        else
        {
            Hlandoff = 0;
            if(t_now<WP.step_T*(1.0-WP.dsp_ratio)-t_wait)
            {
                FpszH = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio)-t_wait,oldpFzH,vec3(WP.landing_depth+Hlandoff,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio),oldpFzH,vec3(WP.landing_depth+Hlandoff,0,0));
            }
        }


        if(isFirststep)
        {
            FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        }
        else if(isFinishing)
        {
            FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        }
        else if(isRHLFmove==true)// RH,LF swing
        {
            LHlanded = false;
            RFlanded = false;
            FootonAir[0] = FootonAir[3] = false;
            if(t_now<t_uf)
            {                
                if(t_now<t_wait)
                {
                    FootonAir[1] = false;
                }
                else
                {
                    FootonAir[1] = true;
                }
            }
            else if(t_now<step_T_each)
            {
                //FootonAir[1] = true;
            }
            else//stop
            {
                FootonAir[1] = false;
            }
            if(t_now<t_f2h)//stop
            {
                FootonAir[2] = false;                
            }
            else if(t_now<t_uh)
            {                
                if(t_now<t_f2h+t_wait)
                {
                    FootonAir[2] = false;
                }
                else
                {
                    FootonAir[2] = true;
                }
            }
            else if(t_now<t_f2h+step_T_each)
            {
                //FootonAir[2] = true;
            }
            else
            {
                FootonAir[2] = false;
            }

            QP.pLH.z = 0.9*QP.pLH.z;
            QP.pRF.z = 0.9*QP.pRF.z;
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

            if(t_now<t_wait+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
            }
            else if(t_now<step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt-t_wait)//&&LFlanded==false)
            {
                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpLFx,vec3(FootTogoLF.x,0,0));

                QP.pLF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLFx[0] = QP.pLF.x;
                oldpLFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLF.x = oldpLFx[1];
                QP.ddLF.x = oldpLFx[2];

                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpLFy,vec3(FootTogoLF.y,0,0));

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

            if(t_now<t_wait+t_f2h+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
            }
            else if(t_now<t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait-0.5*dt)//&&RHlanded==false)
            {
                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpRHx,vec3(FootTogoRH.x,0,0));

                QP.pRH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRHx[0] = QP.pRH.x;
                oldpRHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRH.x = oldpRHx[1];
                QP.ddRH.x = oldpRHx[2];

                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpRHy,vec3(FootTogoRH.y,0,0));

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
            if(t_now>t_uf&&LFlanded==false)
            {
                if(Mch[1]==false)
                {
                    //changetoCconKnee[1] = true;
                    if(NO_ROBOT_TEST==false)
                    {
                        changetoCcon[1] = true;
                    }
                    isPcon[1] = false;
                    Mch[1] = true;
                }
                if(dms[LKN]-dms_old[LKN]<-3)
                //if(WS.LF_Fz>landing_fthres)
                {
                    oldpFzF[1] = 0;oldpFzF[2] = 0;LFlanded = true; FootonAir[1] = false;
                    printf("LFlanded\n");
                 // todmZero[LHP] = todmZero[LKN] = true;

                }
            }
            if(t_now>t_uh&&RHlanded==false)
            {
                if(Mch[2]==false)
                {
                    //changetoCconKnee[2] = true;
                    if(NO_ROBOT_TEST==false)
                    {
                        changetoCcon[2] = true;
                    }
                    isPcon[2] = false;
                    Mch[2] = true;
                }
                //if(dms[REB]-dms_old[REB]<-3)
                //if(WS.RH_Fz>landing_fthres)
                {
                    oldpFzH[1] = 0;oldpFzH[2] = 0;RHlanded = true; FootonAir[2] = false;
                    printf("RHlanded\n");
                  // todmZero[RSP] = todmZero[REB] = true;
                }
            }


        }
        else// LH,RF swing
        {
            RHlanded = false;
            LFlanded = false;
            FootonAir[1] = FootonAir[2] = false;
            if(t_now<t_uf)
            {                
                if(t_now<t_wait)
                {
                    FootonAir[0] = false;
                }
                else
                {
                    FootonAir[0] = true;
                }
            }
            else if(t_now<step_T_each)
            {
                //FootonAir[0] = true;
            }
            else//stop
            {
                FootonAir[0] = false;
            }
            if(t_now<t_f2h)//stop
            {
                FootonAir[3] = false;
            }
            else if(t_now<t_uh)
            {                
                if(t_now<t_f2h+t_wait)
                {
                    FootonAir[3] = false;
                }
                else
                {
                    FootonAir[3] = true;
                }
            }
            else if(t_now<t_f2h+step_T_each)
            {
                //FootonAir[3] = true;
            }
            else
            {
                FootonAir[3] = false;
            }

            QP.pRH.z = 0.9*QP.pRH.z;
            QP.pLF.z = 0.9*QP.pLF.z;

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

            if(t_now<t_wait+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
            }
            else if(t_now<-t_wait+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&RFlanded==false)
            {
                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpRFx,vec3(FootTogoRF.x,0,0));

                QP.pRF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRFx[0] = QP.pRF.x;
                oldpRFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRF.x = oldpRFx[1];
                QP.ddRF.x = oldpRFx[2];

                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpRFy,vec3(FootTogoRF.y,0,0));

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


            if(t_now<t_wait+t_f2h+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            else if(t_now<t_f2h-t_wait+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&RHlanded==false)
            {
                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpLHx,vec3(FootTogoLH.x,0,0));

                QP.pLH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLHx[0] = QP.pLH.x;
                oldpLHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLH.x = oldpLHx[1];
                QP.ddLH.x = oldpLHx[2];

                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-t_wait,oldpLHy,vec3(FootTogoLH.y,0,0));

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




            if(t_now>t_uf&&RFlanded==false)
            {
                if(Mch[0]==false)
                {
                    //changetoCconKnee[0] = true;
                    if(NO_ROBOT_TEST==false)
                    {
                        changetoCcon[0] = true;
                    }
                    isPcon[0] = false;
                    Mch[0] = true;
                }
                if(dms[RKN]-dms_old[RKN]<-3)
                //if(WS.RF_Fz>landing_fthres)
                {
                    RFlanded = true; oldpFzF[1] = 0; oldpFzF[2] = 0;FootonAir[0] = false;
                    printf("RFlanded\n");
                    //todmZero[RHP] = todmZero[RKN] = true;
                }
            }
            if(t_now>t_uh&&LHlanded==false)
            {
                if(Mch[3]==false)
                {
                    //changetoCconKnee[3] = true;
                    if(NO_ROBOT_TEST==false)
                    {
                        changetoCcon[3] = true;
                    }
                    isPcon[3] = false;
                    Mch[3] = true;
                }
                //if(dms[LEB]-dms_old[LEB]<-3)
                //if(WS.LH_Fz>landing_fthres)
                {
                    LHlanded = true; oldpFzH[1] = 0; oldpFzH[2] = 0;FootonAir[3] = false;
                    printf("LHlanded\n");
                     // todmZero[LSP] = todmZero[LEB] = true;
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
QuadJoints OW_Quad::Wave_onestep(WalkSensors _WS)
{
    WS = _WS;
    if(Qcnt<2)
    {
        cout<<"changetoCcon"<<endl;
//        changetoCconKnee[0] = true;
//        changetoCconKnee[1] = true;
//        changetoCconKnee[2] = true;
//        changetoCconKnee[3] = true;
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

    if(t_now>WP.step_T-0.5*dt)
    {

        estimate_plane();

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

        desCOM = desCOM + FrontVector*(WP.step_L.x) +LeftVector*(WP.step_L.y);
        double kkfb = 0.1;/////////////////////////////////////
        double kklr = 0.1;///////////////////////////////////////////////////////
        if(NO_ROBOT_TEST)
        {
            estCOM = QP.pCOM;
        }

//        ff = kkfb*dot(FrontVector,(desCOM-QP.pCOM))-dot(FrontVector,drift_in_step_est_f);
//        ll = kklr*dot(LeftVector,(desCOM-QP.pCOM))-dot(LeftVector,drift_in_step_est_f);
        ff = kkfb*dot(FrontVector,(desCOM-estCOM));//-dot(FrontVector,drift_in_step_est_f);
        ll = kklr*dot(LeftVector,(desCOM-estCOM));//-dot(LeftVector,drift_in_step_est_f);
        double maxff = 0.02;
        double maxll = 0.02;
        if(ff>maxff){ff = maxff;}
        if(ff<-maxff){ff = -maxff;}
        if(ll>maxll){ll = maxll;}
        if(ll<-maxll){ll = -maxll;}


        cout<<"ff "<<ff<<" ll "<<ll<<endl;
        isRHLFmove = !isRHLFmove;
        isFirststep = false;
        printf("nextstep! %d realStepL%f %f %f\n",stopcnt, realStepL.x, realStepL.y,realStepL.z);
        //oldRotZ = vec3(0,0,0);
        oldRotZ[0] = 0;
        oldRotX[0] = 0;
        oldRotY[0] = 0;
        t_now =0;
        qPel_stepstart = QP.qPel;

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
                ||(WP_next.Gait==Wave);
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
            if(WP_next.Gait==Wave){calc3th = false;}

            des_delZ = WP.delZ;
            walkchanged  = false;
        }

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
    double sstepT = WP.step_T*(1-WP.dsp_ratio);
    double dstepT = WP.step_T*(WP.dsp_ratio);

    sL = FrontVector*(WP.step_L.x+ff)+LeftVector*(WP.step_L.y+ll);


    Next_ZMP_offset = sL/(pow(e,w*sstepT)-1);
    double wn = 15;
    double z = 1;
    double kp = wn*wn;
    double kd = 2*wn*z;
    if(isFirststep)
    {
        if(t_now<1.5*dt)
        {
            COMr = QP.pCOM;
            cout<<"tttt111"<<endl;
        }
        vec3 dCref = vec3(0,0,0);
        vec3 Cref = Next_ZMP_offset+COMr;
        vec3 ddCref = kp*(Cref-QP.pCOM)+kd*(dCref-QP.dCOM);
        ZMP = QP.pCOM-delZ/g*ddCref;
        ZMP.z = 0;

        RFcontact =  LHcontact = true;
        LFcontact =  RHcontact = true;
       // calc3th = true;
    }
    else if(isRHLFmove==true)// RH,LF swing
    {
        ZMPr = (QP.pLH+QP.pRF)*0.5;//later controllable factor
        vec3 F2H = (QP.pLH-QP.pRF).normalize();
        ZMP = ZMPr;// + dot(Kp*(QP.pCOM-COMr)+Kd*(QP.dCOM-dCOMr),F2H)*F2H;

        RFcontact =  LHcontact = true;
        LFcontact =  RHcontact = false;
    }
    else
    {
        ZMPr = (QP.pRH+QP.pLF)*0.5;//later controllable factor
        vec3 F2H = (QP.pRH-QP.pLF).normalize();
        ZMP = ZMPr;// + dot(Kp*(QP.pCOM-COMr)+Kd*(QP.dCOM-dCOMr),F2H)*F2H;
        RFcontact =  LHcontact = false;
        LFcontact =  RHcontact = true;
    }


    QP_est = state_est();


    bool stable = false;
    step_T_each = WP.step_T/(2.0-WP.overlap)*(1.0-WP.dsp_ratio);
    t_uf = step_T_each*upRatio;
    t_f2h =  step_T_each*(1.0-WP.overlap);
    t_uh = t_uf + t_f2h;
    t_wait = step_T_each*0.2;//0.2wait 0.6swing 0.2wait
    t_wait = 0;///////////////////////////


    if(t_now>sstepT-0.5*dt)
    {//all contact
        if(lastWave!=5)
        {
            lastWave = 5;
            calc3th = false;
        }
        stable = true;

    }
    else if(!isRHLFmove&&t_now<t_f2h)
    {//RF air only
        if(lastWave!=0)
        {
            lastWave = 0;
            calc3th = false;
        }
        stable = true;

    }
    else if(isRHLFmove&&t_now<t_f2h)
    {//LF air only
        if(lastWave!=1)
        {
            lastWave = 1;
            calc3th = false;
        }
        stable = true;

    }
    else if(isRHLFmove&&t_now<sstepT&&t_now>step_T_each-0.5*dt)
    {//RH air only        
        if(lastWave!=2)
        {
            calc3th = false; lastWave = 2;
        }
        stable = true;


    }
    else if(!isRHLFmove&&t_now<sstepT&&t_now>step_T_each-0.5*dt)
    {//LH air only
        if(lastWave!=3)
        {
            calc3th = false; lastWave = 3;
        }
        stable = true;

    }
    else
    {
        lastWave = 4;
       // calc3th = false;
    }
    if(isFirststep)
    {
        stable = true;
    }
    //if(stable&&!isFirststep)
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
               if(isRHLFmove){RH = LF+2.0*((RF+LH)*0.5-LF);}//F contacted
               else{LH = RF+2.0*((RH+LF)*0.5-RF);}


               int WW = time2Wphase_wave(tt);
               bool LR = isRHLFmove;
               if(WW>99)
               {
                   LR = !LR;
                   WW-=100;
               }
               bool Fc = (WW%10==2);
               bool Hc = (WW/10==2);
               double rd = 3.0;//diagonal
               double rc = 2.0;//corner
               double rsum = rd+rd+rc;

               if(Fc&&Hc)
               {
                  // cout<<"t1";
                   Fcenter = (RF+LF+RH+LH)/4.0;
               }
               else if((!Fc)&&Hc)//foot flying
               {
                   //cout<<"t2";
                   if(LR){ Fcenter = (rd*RF+rc*RH+rd*LH)/rsum;}
                   else{ Fcenter = (rd*LF+rd*RH+rc*LH)/rsum;}
               }
               else if(Fc&&(!Hc))//Hand flyning
               {
                  // cout<<"t3";
                   if(LR){ Fcenter = (rd*RF+rc*LF+rd*LH)/rsum;}
                   else{ Fcenter = (rc*RF+rd*LF+rd*RH)/rsum;}
               }
               else//trot
               {
                  // cout<<"t4";
                   if(LR){ Fcenter = (RF+LH)/2.0;}
                   else{ Fcenter = (LF+RH)/2.0;}
               }
               if(isFirststep&&stepN==0)
               {
                   if(!LR){ Fcenter = ((RF+LF+RH+LH)/4.0)*0.7+(rd*RF+rc*RH+rd*LH)/rsum*0.3;}
                   else{ Fcenter = ((RF+LF+RH+LH)/4.0)*0.7+(rd*LF+rd*RH+rc*LH)/rsum*0.3;}
               }
               if(isFinishing||(stopcnt-stepN<0))
               {
                   Fcenter = (RF+LF+RH+LH)/4.0;
               }

               ZMPrs[i] = Fcenter+sL*stepN;
           }


            calc3th = true;
            OP.preview_init(delZ,QP.pCOM,QP.dCOM,ZMPrs);
            if(DO_DDCOMCON_WHILE_PREVIEW)
            {

            }
        }
        OP.calc_ddCOM_pvddx();
        ZMP = OP.ZMP;
    }
    else
    {
        OP.calc_ddCOM_pvddx();//just calc
    }


    QP.ddCOM = g*(QP.pCOM-ZMP)/delZ;
    double friccoef = 0.8;
    double MaxA = friccoef*g;
    QP.ddCOM.z = 0;
    double Kplr, Kdlr, Kilr;
    Kplr = 1.0/delZ*4.0;//lets increase gain after check ft sensor
    Kdlr = Kplr*0.2;
    Kilr = Kplr*2.0;
    //Kplr = 0;//no control at all//nnnnnnnnnnnnn
    {
        ct = ty*(-Kplr*decomp_A[2]-Kdlr*decomp_W[2]);
        ct = ct+ty*(-Kilr*decomp_I[2]);
        if(stable)
        {//all contact
           // QP.ddCOM = QP_ddCOM;
        }
        else
        {
            QP.ddCOM = QP_ddCOM;
        }

    }



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
    double KFP = 1.0;//teoratically 3 or four is ideal maybe.
    double KFV = 1/w*1.0;//about 0.18 in CP scheme (1/w);

    if((lastWave==2||lastWave==3)&&!isFirststep)
    {
        vec3 conFs;
        if(isRHLFmove)
        {conFs = (QP.pLH+QP.pRF)*0.5;}
        else
        {conFs = (QP.pRH+QP.pLF)*0.5;}
        COM_nn = conFs+0.5*sL;

        CP_nn =conFs+sL;
        CP_nn_nocon = conFs+sL;
        if(DO_DDCOMCON_WHILE_PREVIEW)
        {
            CP_nn =conFs+sL;
        }        
        CP_nn.z = 0;
        ZMP.z = 0;
    }
    else if((lastWave==0||lastWave==1)&&!isFirststep)
    {

        vec3 conFs;
        if(isRHLFmove)
        {conFs = (QP.pLH+QP.pRF)*0.5;}
        else
        {conFs = (QP.pRH+QP.pLF)*0.5;}
        COM_nn = conFs+0.5*sL;
        CP_nn =conFs+sL;
        CP_nn_nocon = conFs+sL;
        if(DO_DDCOMCON_WHILE_PREVIEW)
        {
            CP_nn =conFs+sL;
        }

        CP_nn.z = 0;
        ZMP.z = 0;
    }
    else if(lastWave==5&&!isFirststep)
    {
        vec3 conFs = (QP.pLH+QP.pRF+QP.pRF+QP.pRH)*0.25;
        COM_nn = conFs+0.5*sL;        

    }
    else
    {

        vec3 conFs;
        if(isRHLFmove)
        {conFs = (QP.pLH+QP.pRF)*0.5;}
        else
        {conFs = (QP.pRH+QP.pLF)*0.5;}
        COM_nn = conFs+0.5*sL;
        //CP_nn =COM_nn+0.5*sL+KF2*(QP.dCOM-sL/WP.step_T);//stable, not sure effective
        CP_nn = conFs+sL + KFP*(QP.pCOM-OP.COMr) + KFV*(QP.dCOM-OP.dCOMr);
        CP_nn_nocon = conFs+sL;
        //lets fix this.
        //have to calc nominal CP and offsets


        CP_nn.z = 0;

    }

        QP.dCOM.z=0;
        QP.ddCOM.z = 0;
        QP.pCOM = QP.pCOM + QP.dCOM*dt + QP.ddCOM*dt*dt/2;
        QP.dCOM = QP.dCOM + QP.ddCOM*dt;




    Next_ZMP_offset = vec3();

    if(stable)//&&!isFirststep)
    {
        if(DO_DDCOMCON_WHILE_PREVIEW)
        {
            QP.pCOM.x = OP.COMr.x ;
            QP.pCOM.y = OP.COMr.y ;//not z.
            QP.dCOM = OP.dCOMr ;
        }
        else
        {
            QP.pCOM.x = OP.COMr.x ;
            QP.pCOM.y = OP.COMr.y ;//not z.
            QP.dCOM = OP.dCOMr;
        }
    }

    //lets generate....
    if(isFirststep)
    {

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

        if(FootonAir[1]==false)//LF on ground
        {
            RHV = RHV+((QP.pLF-Center)-LFV);//move more
        }

        FootTogoLF = Center + LFV;
        FootTogoRH = Center + RHV;
        if(lastWave==4&&!isFirststep)//foot goto CP
        {
            FootTogoRH = CP_nn_nocon + RHV;
        }


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
        if(FootonAir[0]==false)//RF on ground
        {
            LHV = LHV+((QP.pRF-Center)-RFV);//move more
        }


        FootTogoRF = Center + RFV;
        FootTogoLH = Center + LHV;
        if(lastWave==4&&!isFirststep)//foot goto CP
        {
            FootTogoLH = CP_nn_nocon + LHV;
        }

        FootTogoLF = QP.pLF;//should be modefied
        FootTogoRH = QP.pRH;//should be modefied
    }


    //calculate foot trajectory
    if(DO_ADJUST_SLOPE){MOVE_LEGS_WAVE_SLOPE();}
    else{MOVE_LEGS_WAVE();}

    if(DO_ADJUST_SLOPE)
    {
        set_des_delZ_slope();
    }

    ///////////control here

    //wave_control needed
    if(DO_ADJUST_SLOPE)
    {
        if(FootonAir[0]==false&&FootonAir[1]==false&&FootonAir[2]==false&&FootonAir[3]==false)
        {
            FFz_ref();
            DSP_control_slope();
        }
        else if(stable)
        {
            int flynum = 0;
            for(int i=0;i<4;i++)
            {
                if(FootonAir[i]==true)
                {
                    flynum = i;
                }
            }
            //why flynum!=lastWave???
            if(flynum!=lastWave)
            {
                //cout<<"??? "<<FootonAir[0]<<FootonAir[1]<<FootonAir[2]<<FootonAir[3]<<" "<<t_now<<" "<<lastWave<<endl;
            }
          flynum = lastWave;
            FFz_ref_3con(flynum);
            WAVE_control_slope(flynum);
            //WAVE_CONTROL NEEDED
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
        else if(stable)
        {
            int flynum = 0;
            for(int i=0;i<4;i++)
            {
                if(FootonAir[i]==true)
                {
                    flynum = i;
                }
            }
            //why flynum!=lastWave???
            if(flynum!=lastWave)
            {
                //cout<<"??? "<<FootonAir[0]<<FootonAir[1]<<FootonAir[2]<<FootonAir[3]<<" "<<t_now<<" "<<lastWave<<endl;
            }
          flynum = lastWave;
            FFz_ref_3con(flynum);
            WAVE_control(flynum);
            //WAVE_CONTROL NEEDED
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

    QJout = adjust_FootZs(QJout);

    return QJout;

}
void OW_Quad::WAVE_control(int noncon)
{

    double Kpy = 0.02*R2Df*2;
    double Kdy = 0.001*R2Df;
    double Kpx = 0.01*R2Df*2;
    double Kdx = 0.0005*R2Df;

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


    QP_controled = QP;
    QP_controled.pRF.z =QP.pRF.z + FootZctFT[0];
    QP_controled.pLF.z =QP.pLF.z + FootZctFT[1];
    QP_controled.pRH.z =QP.pRH.z + FootZctFT[2];
    QP_controled.pLH.z =QP.pLH.z + FootZctFT[3];


    JointGains Gains;
    control_joints(Gains);//stance leg damping only?

}
