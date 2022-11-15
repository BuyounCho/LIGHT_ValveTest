
#include "ow_quad.h"

void OW_Quad::MOVE_LEGS_WAVE_SLOPE()
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

    double wn = 4.0;
    double z = 1.0;
    rpyQP = Oi.Decomp_Rmat(QP.qPel,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));

    oldRotX[2] =wn*wn*(plane_rpy_f[0]-rpyQP[2])+2*z*wn*(-oldRotX[1]);
    oldRotY[2] =wn*wn*(plane_rpy_f[1]-rpyQP[1])+2*z*wn*(-oldRotY[1]);

    oldRotX[1] =oldRotX[1] + oldRotX[2]*dt;
    oldRotX[0] =oldRotX[0] + oldRotX[1]*dt;
    oldRotY[1] =oldRotY[1] + oldRotY[2]*dt;
    oldRotY[0] =oldRotY[0] + oldRotY[1]*dt;



    QP.qPel = qPel_stepstart*quat(mat3(vec3(0,0,-1),PelRotAngle))*quat(mat3(vec3(0,-1,0),oldRotY[0]))*quat(mat3(vec3(-1,0,0),oldRotX[0]));//post? pre? not sure

    QP.dqPel = vec3(oldRotX[1],oldRotY[1],oldRotZ[1]);
    QP.ddqPel = vec3(oldRotX[2],oldRotY[2],oldRotZ[2]);

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
    wn = 1.0;
    z = 1.0*2.0;//for test
    double Aerr = decomp_A[0]-WS.IMUangle.z;
    if(Aerr> M_PI){Aerr =Aerr-2*M_PI;}
    if(Aerr<-M_PI){Aerr =Aerr+2*M_PI;}
    double ddYawcon = -wn*wn*(Aerr)-2*wn*z*(oldRotZ[1]-WS.IMUomega.z);//rotvelref
    dYawcon = dYawcon + ddYawcon*dt;
    double MaxdYawcon = 15*D2Rf;
    if(dYawcon> MaxdYawcon){dYawcon = MaxdYawcon;}
    if(dYawcon<-MaxdYawcon){dYawcon =-MaxdYawcon;}
    //////////////////////////////set z of FootTogo
    FootTogoRF.z = Zfromplane(FootTogoRF.x, FootTogoRF.y);
    FootTogoLF.z = Zfromplane(FootTogoLF.x, FootTogoLF.y);
    FootTogoRH.z = Zfromplane(FootTogoRH.x, FootTogoRH.y);
    FootTogoLH.z = Zfromplane(FootTogoLH.x, FootTogoLH.y);

    double midZ = 0.25*(FootTogoRF.z+FootTogoLF.z+FootTogoRH.z+FootTogoLH.z);
    FootTogoRF.z =FootTogoRF.z -midZ;
    FootTogoLF.z =FootTogoLF.z -midZ;
    FootTogoRH.z =FootTogoRH.z -midZ;
    FootTogoLH.z =FootTogoLH.z -midZ;//make it 0 mean to reamin COM heigth constant



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
            if(isRHLFmove)
            {
                FpszF = calc_5th((t_now-dt),t_uf,oldpFzF,vec3(upL+FootTogoLF.z,0,0));
            }
            else
            {
                FpszF = calc_5th((t_now-dt),t_uf,oldpFzF,vec3(upL+FootTogoRF.z,0,0));
            }


        }
        else if(t_now<step_T_each)
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
            if(isRHLFmove)
            {
                FpszF = calc_5th((t_now-dt),step_T_each,oldpFzF,vec3(WP.landing_depth+Flandoff+FootTogoLF.z,0,0));
            }
            else
            {
                FpszF = calc_5th((t_now-dt),step_T_each,oldpFzF,vec3(WP.landing_depth+Flandoff+FootTogoRF.z,0,0));
            }
        }
        else//stop
        {
            if(isRHLFmove)
            {
                FpszF = calc_5th((t_now-dt),WP.step_T,oldpFzF,vec3(WP.landing_depth+FootTogoLF.z,0,0));
            }
            else
            {
                FpszF = calc_5th((t_now-dt),WP.step_T,oldpFzF,vec3(WP.landing_depth+FootTogoRF.z,0,0));
            }
        }
        if(t_now<t_f2h)//stop
        {
            if(isRHLFmove)
            {
                //FpszH = calc_5th((t_now-dt),t_f2h,oldpFzH,vec3(WP.landing_depth+FootTogoRH.z,0,0));
                //FpszH = calc_5th((t_now-dt),t_f2h,oldpFzH,vec3(oldpFzH.z,0,0));
            }
            else
            {
                //FpszH = calc_5th((t_now-dt),t_f2h,oldpFzH,vec3(WP.landing_depth+FootTogoLH.z,0,0));
                //FpszH = calc_5th((t_now-dt),t_f2h,oldpFzH,vec3(oldpFzH.z,0,0));
            }
        }
        else if(t_now<t_uh)
        {
            if(isRHLFmove)
            {
                FpszH = calc_5th((t_now-dt),t_uh,oldpFzH,vec3(upL+FootTogoRH.z,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),t_uh,oldpFzH,vec3(upL+FootTogoLH.z,0,0));
            }
        }
        else
        {
//            if(isRHLFmove)
//            {
//                vec3 offH = FootTogoRH-COM_nn;
//                offH.z = 0;
//                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1]);
//            }
//            else
//            {
//                vec3 offH = FootTogoLH-COM_nn;
//                offH.z = 0;
//                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1]);
//          }
            Hlandoff = 0;
            if(isRHLFmove)
            {
                FpszH = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio),oldpFzH,vec3(WP.landing_depth+Hlandoff+FootTogoRH.z,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio),oldpFzH,vec3(WP.landing_depth+Hlandoff+FootTogoLH.z,0,0));
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
            if(t_now<t_uf)
            {
                FootonAir[1] = true;
            }
            else if(t_now<step_T_each)
            {
                FootonAir[1] = true;
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
                FootonAir[2] = true;
            }
            else if(t_now<t_f2h+step_T_each)
            {
                FootonAir[2] = true;
            }
            else
            {
                FootonAir[2] = false;
            }

//            QP.pLH.z = 0.9*QP.pLH.z;
//            QP.pRF.z = 0.9*QP.pRF.z;
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
                QP.pLF.z = 0.98*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
                QP.dLF.z = 0;
                QP.ddLF.z = 0;
            }

            if(t_now<step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
            }
            else if(t_now<step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&LFlanded==false)
            {
                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLFx,vec3(FootTogoLF.x,0,0));

                QP.pLF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLFx[0] = QP.pLF.x;
                oldpLFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLF.x = oldpLFx[1];
                QP.ddLF.x = oldpLFx[2];

                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLFy,vec3(FootTogoLF.y,0,0));

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
                if(t_now<t_f2h){}
                else
                {
                    QP.pRH.z =FpszH[0]*st_5 + FpszH[1]*st_4 + FpszH[2]*st_3 + FpszH[3]*st_2 + FpszH[4]*st_1 + FpszH[5];
                    oldpFzH[0] = QP.pRH.z;
                    oldpFzH[1] = 5*FpszH[0]*st_4 + 4*FpszH[1]*st_3 + 3*FpszH[2]*st_2 + 2*FpszH[3]*st_1 + FpszH[4];
                    oldpFzH[2] = 20*FpszH[0]*st_3 + 12*FpszH[1]*st_2 + 6*FpszH[2]*st_1 + 2*FpszH[3];
                    QP.dRH.z = oldpFzH[1];
                    QP.ddRH.z = oldpFzH[2];
                }
            }
            else
            {
                QP.pRH.z = 0.98*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
                QP.dRH.z = 0;
                QP.ddRH.z = 0;
            }

            if(t_now<t_f2h+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
            }
            else if(t_now<t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&RHlanded==false)
            {
                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRHx,vec3(FootTogoRH.x,0,0));

                QP.pRH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRHx[0] = QP.pRH.x;
                oldpRHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRH.x = oldpRHx[1];
                QP.ddRH.x = oldpRHx[2];

                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRHy,vec3(FootTogoRH.y,0,0));

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

            if(t_now<t_uf)
            {
                FootonAir[0] = true;
            }
            else if(t_now<step_T_each)
            {
                FootonAir[0] = true;
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
                FootonAir[3] = true;
            }
            else if(t_now<t_f2h+step_T_each)
            {
                FootonAir[3] = true;
            }
            else
            {
                FootonAir[3] = false;
            }

//            QP.pRH.z = 0.9*QP.pRH.z;
//            QP.pLF.z = 0.9*QP.pLF.z;

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
                QP.pRF.z = 0.98*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
                QP.dRF.z = 0;
                QP.ddRF.z = 0;
            }

            if(t_now<step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
            }
            else if(t_now<step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&RFlanded==false)
            {
                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRFx,vec3(FootTogoRF.x,0,0));

                QP.pRF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRFx[0] = QP.pRF.x;
                oldpRFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRF.x = oldpRFx[1];
                QP.ddRF.x = oldpRFx[2];

                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRFy,vec3(FootTogoRF.y,0,0));

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
                if(t_now<t_f2h){}
                else
                {
                    QP.pLH.z =FpszH[0]*st_5 + FpszH[1]*st_4 + FpszH[2]*st_3 + FpszH[3]*st_2 + FpszH[4]*st_1 + FpszH[5];
                    oldpFzH[0] = QP.pLH.z;
                    oldpFzH[1] = 5*FpszH[0]*st_4 + 4*FpszH[1]*st_3 + 3*FpszH[2]*st_2 + 2*FpszH[3]*st_1 + FpszH[4];
                    oldpFzH[2] = 20*FpszH[0]*st_3 + 12*FpszH[1]*st_2 + 6*FpszH[2]*st_1 + 2*FpszH[3];
                    QP.dLH.z = oldpFzH[1];
                    QP.ddLH.z = oldpFzH[2];
                }
            }
            else
            {
                QP.pLH.z = 0.98*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
                QP.dLH.z = 0;
                QP.ddLH.z = 0;
            }


            if(t_now<t_f2h+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            else if(t_now<t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&RHlanded==false)
            {
                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLHx,vec3(FootTogoLH.x,0,0));

                QP.pLH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLHx[0] = QP.pLH.x;
                oldpLHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLH.x = oldpLHx[1];
                QP.ddLH.x = oldpLHx[2];

                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLHy,vec3(FootTogoLH.y,0,0));

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
        vec3 Fcenter = ZMP;

        vec3 Foff,Hoff;
        if(FootonAir[0]==false)
        {
            Foff = (QP_controled.pRF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pRF.x = Fcenter.x+Foff.x;
            QP.pRF.y = Fcenter.y+Foff.y;
            QP.pRF.z = 0.9*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
        }
        if(FootonAir[1]==false)
        {
            Foff = (QP_controled.pLF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pLF.x = Fcenter.x+Foff.x;
            QP.pLF.y = Fcenter.y+Foff.y;
            QP.pLF.z = 0.9*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
        }
        if(FootonAir[2]==false)
        {
            Hoff = (QP_controled.pRH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pRH.x = Fcenter.x+Hoff.x;
            QP.pRH.y = Fcenter.y+Hoff.y;
            QP.pRH.z = 0.9*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
        }
        if(FootonAir[3]==false)
        {
            Hoff = (QP_controled.pLH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pLH.x = Fcenter.x+Hoff.x;
            QP.pLH.y = Fcenter.y+Hoff.y;
            QP.pLH.z = 0.9*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
        }
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
            QP.pRF.z = 0.9*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
        }
        if(FootonAir[1]==false)
        {
            Foff = (QP_controled.pLF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pLF.x = Fcenter.x+Foff.x;
            QP.pLF.y = Fcenter.y+Foff.y;
            QP.pLF.z = 0.9*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
        }
        if(FootonAir[2]==false)
        {
            Hoff = (QP_controled.pRH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pRH.x = Fcenter.x+Hoff.x;
            QP.pRH.y = Fcenter.y+Hoff.y;
            if(t_now<t_f2h){}
            else
            {
            QP.pRH.z = 0.9*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
            }
        }
        if(FootonAir[3]==false)
        {
            Hoff = (QP_controled.pLH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pLH.x = Fcenter.x+Hoff.x;
            QP.pLH.y = Fcenter.y+Hoff.y;
            if(t_now<t_f2h){}
            else
            {
            QP.pLH.z = 0.9*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
            }
        }
    }
}

void OW_Quad::MOVE_LEGS_WAVE2_SLOPE()//change RH<->LF change LH<->RF
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


    double wn = 2.0;
    double z = 1.0;
    rpyQP = Oi.Decomp_Rmat(QP.qPel,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));

    oldRotX[2] =wn*wn*(plane_rpy_f[0]-rpyQP[2])+2*z*wn*(-oldRotX[1]);
    oldRotY[2] =wn*wn*(plane_rpy_f[1]-rpyQP[1])+2*z*wn*(-oldRotY[1]);

    oldRotX[1] =oldRotX[1] + oldRotX[2]*dt;
    oldRotX[0] =oldRotX[0] + oldRotX[1]*dt;
    oldRotY[1] =oldRotY[1] + oldRotY[2]*dt;
    oldRotY[0] =oldRotY[0] + oldRotY[1]*dt;



    QP.qPel = qPel_stepstart*quat(mat3(vec3(0,0,-1),PelRotAngle))*quat(mat3(vec3(0,-1,0),oldRotY[0]))*quat(mat3(vec3(-1,0,0),oldRotX[0]));//post? pre? not sure

    QP.dqPel = vec3(oldRotX[1],oldRotY[1],oldRotZ[1]);
    QP.ddqPel = vec3(oldRotX[2],oldRotY[2],oldRotZ[2]);

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
    wn = 1.0;
    z = 1.0*2.0;//for test
    double Aerr = decomp_A[0]-WS.IMUangle.z;
    if(Aerr> M_PI){Aerr =Aerr-2*M_PI;}
    if(Aerr<-M_PI){Aerr =Aerr+2*M_PI;}
    double ddYawcon = -wn*wn*(Aerr)-2*wn*z*(oldRotZ[1]-WS.IMUomega.z);//rotvelref
    dYawcon = dYawcon + ddYawcon*dt;
    double MaxdYawcon = 15*D2Rf;
    if(dYawcon> MaxdYawcon){dYawcon = MaxdYawcon;}
    if(dYawcon<-MaxdYawcon){dYawcon =-MaxdYawcon;}

    //////////////////////////////set z of FootTogo
    FootTogoRF.z = Zfromplane(FootTogoRF.x, FootTogoRF.y);
    FootTogoLF.z = Zfromplane(FootTogoLF.x, FootTogoLF.y);
    FootTogoRH.z = Zfromplane(FootTogoRH.x, FootTogoRH.y);
    FootTogoLH.z = Zfromplane(FootTogoLH.x, FootTogoLH.y);

    double midZ = 0.25*(FootTogoRF.z+FootTogoLF.z+FootTogoRH.z+FootTogoLH.z);
    FootTogoRF.z =FootTogoRF.z -midZ;
    FootTogoLF.z =FootTogoLF.z -midZ;
    FootTogoRH.z =FootTogoRH.z -midZ;
    FootTogoLH.z =FootTogoLH.z -midZ;//make it 0 mean to reamin COM heigth constant



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
            if(isRHLFmove)
            {
                FpszH = calc_5th((t_now-dt),t_uf,oldpFzH,vec3(upL+FootTogoRH.z,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),t_uf,oldpFzH,vec3(upL+FootTogoLH.z,0,0));
            }
        }
        else if(t_now<step_T_each)
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
            Hlandoff = 0;
            if(isRHLFmove)
            {
                FpszH = calc_5th((t_now-dt),step_T_each,oldpFzH,vec3(WP.landing_depth+Hlandoff+FootTogoRH.z,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),step_T_each,oldpFzH,vec3(WP.landing_depth+Hlandoff+FootTogoLH.z,0,0));
            }
        }
        else//stop
        {
            if(isRHLFmove)
            {
                FpszH = calc_5th((t_now-dt),WP.step_T,oldpFzH,vec3(WP.landing_depth+FootTogoRH.z,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),WP.step_T,oldpFzH,vec3(WP.landing_depth+FootTogoLH.z,0,0));
            }
        }
        if(t_now<t_f2h)//stop
        {
            //FpszF = calc_5th((t_now-dt),t_f2h,oldpFzF,vec3(WP.landing_depth,0,0));
            //FpszF = calc_5th((t_now-dt),t_f2h,oldpFzF,vec3(oldpFzF.z,0,0));
        }
        else if(t_now<t_uh)
        {
            if(isRHLFmove)
            {
                FpszF = calc_5th((t_now-dt),t_uh,oldpFzF,vec3(upL+FootTogoLF.z,0,0));
            }
            else
            {
                FpszF = calc_5th((t_now-dt),t_uh,oldpFzF,vec3(upL+FootTogoRF.z,0,0));
            }
        }
        else
        {
//            if(isRHLFmove)
//            {
//                vec3 offH = FootTogoRH-COM_nn;
//                offH.z = 0;
//                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1]);
//            }
//            else
//            {
//                vec3 offH = FootTogoLH-COM_nn;
//                offH.z = 0;
//                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1]);
//          }
            Flandoff = 0;
            if(isRHLFmove)
            {
                FpszF = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio),oldpFzF,vec3(WP.landing_depth+Flandoff+FootTogoLF.z,0,0));
            }
            else
            {
                FpszF = calc_5th((t_now-dt),WP.step_T*(1.0-WP.dsp_ratio),oldpFzF,vec3(WP.landing_depth+Flandoff+FootTogoRF.z,0,0));
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
            if(t_now<t_uf)
            {
                FootonAir[2] = true;
            }
            else if(t_now<step_T_each)
            {
               // FootonAir[2] = true;
            }
            else//stop
            {
                FootonAir[2] = false;
            }
            if(t_now<t_f2h)//stop
            {
                FootonAir[1] = false;
            }
            else if(t_now<t_uh)
            {
                FootonAir[1] = true;
            }
            else if(t_now<t_f2h+step_T_each)
            {
              //  FootonAir[1] = true;
            }
            else
            {
                FootonAir[1] = false;
            }

//            QP.pLH.z = 0.9*QP.pLH.z;
//            QP.pRF.z = 0.9*QP.pRF.z;
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
                QP.pRH.z = 0.98*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
                QP.dRH.z = 0;
                QP.ddRH.z = 0;
            }

            if(t_now<step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);
            }
            else if(t_now<step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&RHlanded==false)
            {
                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRHx,vec3(FootTogoRH.x,0,0));

                QP.pRH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRHx[0] = QP.pRH.x;
                oldpRHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRH.x = oldpRHx[1];
                QP.ddRH.x = oldpRHx[2];

                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRHy,vec3(FootTogoRH.y,0,0));

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
                RHlanded = true;//HHHHHHHHHHHHHHHHHHHHH
            }

            if(LFlanded==false)
            {
                if(t_now<t_f2h){}
                else
                {
                QP.pLF.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
                oldpFzF[0] = QP.pLF.z;
                oldpFzF[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
                oldpFzF[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
                QP.dLF.z = oldpFzF[1];
                QP.ddLF.z = oldpFzF[2];
                }
            }
            else
            {
                QP.pLF.z = 0.98*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
                QP.dLF.z = 0;
                QP.ddLF.z = 0;
            }

            if(t_now<t_f2h+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
            }
            else if(t_now<t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&LFlanded==false)
            {
                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLFx,vec3(FootTogoLF.x,0,0));

                QP.pLF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLFx[0] = QP.pLF.x;
                oldpLFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLF.x = oldpLFx[1];
                QP.ddLF.x = oldpLFx[2];

                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLFy,vec3(FootTogoLF.y,0,0));

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
                LFlanded = true;//HHHHHHHHHHHHHHHHHHHHH
            }
            if(t_now>t_uf&&RHlanded==false)
            {
                if(Mch[2]==false)
                {
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
            if(t_now>t_uh&&LFlanded==false)
            {
                if(Mch[1]==false)
                {
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


        }
        else// LH,RF swing
        {
            RHlanded = false;
            LFlanded = false;

            if(t_now<t_uf)
            {
                FootonAir[3] = true;
            }
            else if(t_now<step_T_each)
            {
                //FootonAir[3] = true;
            }
            else//stop
            {
                FootonAir[3] = false;
            }
            if(t_now<t_f2h)//stop
            {
                FootonAir[0] = false;
            }
            else if(t_now<t_uh)
            {
                FootonAir[0] = true;
            }
            else if(t_now<t_f2h+step_T_each)
            {
                //FootonAir[0] = true;
            }
            else
            {
                FootonAir[0] = false;
            }

//            QP.pRH.z = 0.9*QP.pRH.z;
//            QP.pLF.z = 0.9*QP.pLF.z;

            if(LHlanded==false)
            {
                QP.pLH.z =FpszH[0]*st_5 + FpszH[1]*st_4 + FpszH[2]*st_3 + FpszH[3]*st_2 + FpszH[4]*st_1 + FpszH[5];
                oldpFzH[0] =QP.pLH.z;
                oldpFzH[1] = 5*FpszH[0]*st_4 + 4*FpszH[1]*st_3 + 3*FpszH[2]*st_2 + 2*FpszH[3]*st_1 + FpszH[4];
                oldpFzH[2] = 20*FpszH[0]*st_3 + 12*FpszH[1]*st_2 + 6*FpszH[2]*st_1 + 2*FpszH[3];
                QP.dLH.z = oldpFzH[1];
                QP.ddLH.z = oldpFzH[2];
            }
            else
            {
                QP.pLH.z = 0.98*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
                QP.dLH.z = 0;
                QP.ddLH.z = 0;
            }

            if(t_now<step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);
            }
            else if(t_now<step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&LHlanded==false)
            {
                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLHx,vec3(FootTogoLH.x,0,0));

                QP.pLH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLHx[0] = QP.pLH.x;
                oldpLHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLH.x = oldpLHx[1];
                QP.ddLH.x = oldpLHx[2];

                Fps = calc_5th((t_now-dt),step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpLHy,vec3(FootTogoLH.y,0,0));

                QP.pLH.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpLHy[0] = QP.pLH.y;
                oldpLHy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpLHy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dLH.y = oldpLHy[1];
                QP.ddLH.y = oldpLHy[2];
            }
            else
            {
                QP.dLH.x = QP.ddLH.x = 0;
                QP.dLH.y = QP.ddLH.y = 0;
                LHlanded = true;//HHHHHHHHHHHHHHHHHHHHH
            }

            if(RFlanded==false)
            {
                if(t_now<t_f2h){}
                else
                {
                QP.pRF.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
                oldpFzF[0] = QP.pRF.z;
                oldpFzF[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
                oldpFzF[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
                QP.dRF.z = oldpFzF[1];
                QP.ddRF.z = oldpFzF[2];
                }
            }
            else
            {
                QP.pRF.z = 0.98*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
                QP.dRF.z = 0;
                QP.ddRF.z = 0;
            }


            if(t_now<t_f2h+step_T_each*(before_up_ratio)*(1-WP.dsp_ratio)-0.5*dt)
            {
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
            }
            else if(t_now<t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio)-0.5*dt)//&&RHlanded==false)
            {
                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRFx,vec3(FootTogoRF.x,0,0));

                QP.pRF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
                oldpRFx[0] = QP.pRF.x;
                oldpRFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
                oldpRFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
                QP.dRF.x = oldpRFx[1];
                QP.ddRF.x = oldpRFx[2];

                Fps = calc_5th((t_now-dt),t_f2h+step_T_each*(1-after_down_ratio)*(1-WP.dsp_ratio),oldpRFy,vec3(FootTogoRF.y,0,0));

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
                RFlanded = true;//HHHHHHHHHHHHHHHHHHHHH
            }




            if(t_now>t_uf&&LHlanded==false)
            {
                if(Mch[3]==false)
                {
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
            if(t_now>t_uh&&RFlanded==false)
            {
                if(Mch[0]==false)
                {
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
            if(t_now<t_f2h){}
            else
            {
            QP.pRF.z = 0.9*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
            }
        }
        if(FootonAir[1]==false)
        {
            Foff = (QP_controled.pLF-Fcenter);
            Foff = (mat3(vec3(0,0,-1),dYawcon*dt))*Foff;
            QP.pLF.x = Fcenter.x+Foff.x;
            QP.pLF.y = Fcenter.y+Foff.y;
            if(t_now<t_f2h){}
            else
            {
            QP.pLF.z = 0.9*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
            }
        }
        if(FootonAir[2]==false)
        {
            Hoff = (QP_controled.pRH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pRH.x = Fcenter.x+Hoff.x;
            QP.pRH.y = Fcenter.y+Hoff.y;
            QP.pRH.z = 0.9*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
        }
        if(FootonAir[3]==false)
        {
            Hoff = (QP_controled.pLH-Fcenter);
            Hoff = (mat3(vec3(0,0,-1),dYawcon*dt))*Hoff;
            QP.pLH.x = Fcenter.x+Hoff.x;
            QP.pLH.y = Fcenter.y+Hoff.y;
            QP.pLH.z = 0.9*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
        }
    }
}
void OW_Quad::WAVE_control_slope(int noncon)
{

    double Kpy = 0.02*R2Df*2;
    double Kdy = 0.001*R2Df;
    double Kpx = 0.01*R2Df*2;
    double Kdx = 0.0005*R2Df;

    vec3 IMUref = plane_rpy_f;
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
