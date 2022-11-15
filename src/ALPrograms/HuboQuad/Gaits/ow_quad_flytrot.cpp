#include "ow_quad.h"
void OW_Quad::MOVE_LEGS_FLYTROT()
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
    if(std::isnan(PelRotAngle))
    {
        std::cout<<"FPR "<<FPR<<std::endl;
        std::cout<<" "<<t_now<<std::endl;
    }

//    QP.qPel = quat();
//    QP.dqPel  = vec3();
//    QP.ddqPel  = vec3();


    VectorNd Fps, FpszH, FpszF, FpszH2, FpszF2;
    double t_u = WP.step_T*upRatio;
    double t_jump = WP.step_T*(1-flyRatio);
    st_1 = t_now;
    st_2 = st_1*st_1;
    st_3 = st_2*st_1;
    st_4 = st_3*st_1;
    st_5 = st_4*st_1;
    if(t_now<0.5*dt)
    {
        if(isRHLFmove)
        {
            oldpLFz = vec3(QP.pLF.z,0,0);
            oldpRHz = vec3(QP.pRH.z,0,0);
            contact_L = (QP_controled.pRF-QP_controled.pLH).norm();
            //dYawcon = 0;
        }
        else
        {
            oldpRFz = vec3(QP.pRF.z,0,0);
            oldpLHz = vec3(QP.pLH.z,0,0);
            contact_L = (QP_controled.pLF-QP_controled.pRH).norm();
            //dYawcon = 0;
        }
        cout<<"contact_L "<<contact_L<<"ori "<<sqrt(WP.LR_L*WP.LR_L+WP.FB_L*WP.FB_L)<<endl;
    }

    ddYawcon = -QP_ddQnow[5];
    dYawcon = dYawcon + ddYawcon*dt;
    double MaxdYawcon = 15*D2Rf;
    if(dYawcon> MaxdYawcon){dYawcon = MaxdYawcon;}
    if(dYawcon<-MaxdYawcon){dYawcon =-MaxdYawcon;}



        if(t_now<t_u)
        {
            if(isRHLFmove)
            {
                if(Mchanged)
                {
                    Mchanged = false;
                    oldpRHz = vec3(QP.pRH.z,0,0);
                    oldpLFz = vec3(QP.pLF.z,0,0);

                    oldpRFx = vec3(QP.pRF.x,0,0);
                    oldpRFy = vec3(QP.pRF.y,0,0);
                    oldpLFx = vec3(QP.pLF.x,0,0);
                    oldpLFy = vec3(QP.pLF.y,0,0);
                    oldpRHx = vec3(QP.pRH.x,0,0);
                    oldpRHy = vec3(QP.pRH.y,0,0);
                    oldpLHx = vec3(QP.pLH.x,0,0);
                    oldpLHy = vec3(QP.pLH.y,0,0);

                }
                FpszH = calc_5th((t_now-dt),t_u,oldpRHz,vec3(upL,0,0));
                FpszF = calc_5th((t_now-dt),t_u,oldpLFz,vec3(upL,0,0));

            }
            else
            {
                if(Mchanged)
                {
                    Mchanged = false;
                    oldpLHz = vec3(QP.pLH.z,0,0);
                    oldpRFz = vec3(QP.pRF.z,0,0);

                    oldpRFx = vec3(QP.pRF.x,0,0);
                    oldpRFy = vec3(QP.pRF.y,0,0);
                    oldpLFx = vec3(QP.pLF.x,0,0);
                    oldpLFy = vec3(QP.pLF.y,0,0);
                    oldpRHx = vec3(QP.pRH.x,0,0);
                    oldpRHy = vec3(QP.pRH.y,0,0);
                    oldpLHx = vec3(QP.pLH.x,0,0);
                    oldpLHy = vec3(QP.pLH.y,0,0);
                }
                FpszH = calc_5th((t_now-dt),t_u,oldpLHz,vec3(upL,0,0));
                FpszF = calc_5th((t_now-dt),t_u,oldpRFz,vec3(upL,0,0));

            }


        }
        else
        {
            if(isRHLFmove)
            {
                FpszH = calc_5th((t_now-dt),WP.step_T,oldpRHz,vec3(WP.landing_depth,0,0));
                FpszF = calc_5th((t_now-dt),WP.step_T,oldpLFz,vec3(WP.landing_depth,0,0));
            }
            else
            {
                FpszH = calc_5th((t_now-dt),WP.step_T,oldpLHz,vec3(WP.landing_depth,0,0));
                FpszF = calc_5th((t_now-dt),WP.step_T,oldpRFz,vec3(WP.landing_depth,0,0));
            }
        }

        if(isFirststep)
        {
            if(t_now<t_jump)
            {
                FootonAir[0] = FootonAir[1] = FootonAir[2] = FootonAir[3] = false;
            }
            else
            {
                FootonAir[0] = FootonAir[1] = FootonAir[2] = FootonAir[3] = true;
            }
            oldpRFx = vec3(QP.pRF.x,0,0);
            oldpRFy = vec3(QP.pRF.y,0,0);
            oldpRFz = vec3(QP.pRF.z,0,0);
            oldpRHx = vec3(QP.pRH.x,0,0);
            oldpRHy = vec3(QP.pRH.y,0,0);
            oldpRHz = vec3(QP.pRH.z,0,0);
            oldpLFx = vec3(QP.pLF.x,0,0);
            oldpLFy = vec3(QP.pLF.y,0,0);
            oldpLFz = vec3(QP.pLF.z,0,0);
            oldpLHx = vec3(QP.pLH.x,0,0);
            oldpLHy = vec3(QP.pLH.y,0,0);
            oldpLHz = vec3(QP.pLH.z,0,0);
        }
        else if(isFinishing)
        {
        FootonAir[0]=FootonAir[1]=FootonAir[2]=FootonAir[3]=false;
        }
        else if(isRHLFmove==true)// RH,LF swing
        {

            LHlanded = false;
            RFlanded = false;
            if(t_now<t_jump)
            {
                FootonAir[0] = false;
                FootonAir[3] = false;
                QP.pLH.z = 0.9*QP.pLH.z;
                QP.pRF.z = 0.9*QP.pRF.z;
                oldpLHz = vec3(QP.pLH.z,0,0);
                oldpRFz = vec3(QP.pRF.z,0,0);
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
            else
            {
                FootonAir[0] = true;
                FootonAir[3] = true;
                oldpRFx = vec3(QP.pRF.x,0,0);
                oldpRFy = vec3(QP.pRF.y,0,0);
                oldpLHx = vec3(QP.pLH.x,0,0);
                oldpLHy = vec3(QP.pLH.y,0,0);

                if(stopcnt>0)
                {
                    FpszF2 = calc_5th((t_now-dt),WP.step_T,oldpRFz,vec3(upL,0,0));
                    QP.pRF.z =FpszF2[0]*st_5 + FpszF2[1]*st_4 + FpszF2[2]*st_3 + FpszF2[3]*st_2 + FpszF2[4]*st_1 + FpszF2[5];
                    oldpRFz[0] =QP.pRF.z;
                    oldpRFz[1] = 5*FpszF2[0]*st_4 + 4*FpszF2[1]*st_3 + 3*FpszF2[2]*st_2 + 2*FpszF2[3]*st_1 + FpszF2[4];
                    oldpRFz[2] = 20*FpszF2[0]*st_3 + 12*FpszF2[1]*st_2 + 6*FpszF2[2]*st_1 + 2*FpszF2[3];
                    QP.dRF.z = oldpRFz[1];
                    QP.ddRF.z = oldpRFz[2];
                    FpszH2 = calc_5th((t_now-dt),WP.step_T,oldpLHz,vec3(upL,0,0));
                    QP.pLH.z =FpszH2[0]*st_5 + FpszH2[1]*st_4 + FpszH2[2]*st_3 + FpszH2[3]*st_2 + FpszH2[4]*st_1 + FpszH2[5];
                    oldpLHz[0] = QP.pLH.z;
                    oldpLHz[1] = 5*FpszH2[0]*st_4 + 4*FpszH2[1]*st_3 + 3*FpszH2[2]*st_2 + 2*FpszH2[3]*st_1 + FpszH2[4];
                    oldpLHz[2] = 20*FpszH2[0]*st_3 + 12*FpszH2[1]*st_2 + 6*FpszH2[2]*st_1 + 2*FpszH2[3];
                    QP.dLH.z = oldpLHz[1];
                    QP.ddLH.z = oldpLHz[2];
                }

            }

                FootonAir[1] = true;
                FootonAir[2] = true;


            QP.pRH.z =FpszH[0]*st_5 + FpszH[1]*st_4 + FpszH[2]*st_3 + FpszH[3]*st_2 + FpszH[4]*st_1 + FpszH[5];
            oldpRHz[0] = QP.pRH.z;
            oldpRHz[1] = 5*FpszH[0]*st_4 + 4*FpszH[1]*st_3 + 3*FpszH[2]*st_2 + 2*FpszH[3]*st_1 + FpszH[4];
            oldpRHz[2] = 20*FpszH[0]*st_3 + 12*FpszH[1]*st_2 + 6*FpszH[2]*st_1 + 2*FpszH[3];
            QP.dRH.z = oldpRHz[1];
            QP.ddRH.z = oldpRHz[2];



            Fps = calc_5th((t_now-dt),WP.step_T,oldpRHx,vec3(FootTogoRH.x,0,0));

            QP.pRH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpRHx[0] = QP.pRH.x;
            oldpRHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpRHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dRH.x = oldpRHx[1];
            QP.ddRH.x = oldpRHx[2];

            Fps = calc_5th((t_now-dt),WP.step_T,oldpRHy,vec3(FootTogoRH.y,0,0));

            QP.pRH.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpRHy[0] = QP.pRH.y;
            oldpRHy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpRHy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dRH.y = oldpRHy[1];
            QP.ddRH.y = oldpRHy[2];


            QP.pLF.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
            oldpLFz[0] = QP.pLF.z;
            oldpLFz[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
            oldpLFz[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
            QP.dLF.z = oldpLFz[1];
            QP.ddLF.z = oldpLFz[2];



            Fps = calc_5th((t_now-dt),WP.step_T,oldpLFx,vec3(FootTogoLF.x,0,0));

            QP.pLF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpLFx[0] = QP.pLF.x;
            oldpLFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpLFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dLF.x = oldpLFx[1];
            QP.ddLF.x = oldpLFx[2];

            Fps = calc_5th((t_now-dt),WP.step_T,oldpLFy,vec3(FootTogoLF.y,0,0));

            QP.pLF.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpLFy[0] = QP.pLF.y;
            oldpLFy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpLFy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dLF.y = oldpLFy[1];
            QP.ddLF.y = oldpLFy[2];




        }
        else// LH,RF swing
        {
            RHlanded = false;
            LFlanded = false;
            if(t_now<t_jump)
            {
                FootonAir[1] = false;
                FootonAir[2] = false;
                QP.pRH.z = 0.9*QP.pRH.z;
                QP.pLF.z = 0.9*QP.pLF.z;
                oldpRHz = vec3(QP.pRH.z,0,0);
                oldpLFz = vec3(QP.pLF.z,0,0);
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
            else
            {
                FootonAir[1] = true;
                FootonAir[2] = true;
                oldpLFx = vec3(QP.pLF.x,0,0);
                oldpLFy = vec3(QP.pLF.y,0,0);
                oldpRHx = vec3(QP.pRH.x,0,0);
                oldpRHy = vec3(QP.pRH.y,0,0);

                if(stopcnt>0)
                {
                    FpszF2 = calc_5th((t_now-dt),WP.step_T,oldpLFz,vec3(upL,0,0));
                    QP.pLF.z =FpszF2[0]*st_5 + FpszF2[1]*st_4 + FpszF2[2]*st_3 + FpszF2[3]*st_2 + FpszF2[4]*st_1 + FpszF2[5];
                    oldpLFz[0] =QP.pLF.z;
                    oldpLFz[1] = 5*FpszF2[0]*st_4 + 4*FpszF2[1]*st_3 + 3*FpszF2[2]*st_2 + 2*FpszF2[3]*st_1 + FpszF2[4];
                    oldpLFz[2] = 20*FpszF2[0]*st_3 + 12*FpszF2[1]*st_2 + 6*FpszF2[2]*st_1 + 2*FpszF2[3];
                    QP.dLF.z = oldpLFz[1];
                    QP.ddLF.z = oldpLFz[2];
                    FpszH2 = calc_5th((t_now-dt),WP.step_T,oldpRHz,vec3(upL,0,0));
                    QP.pRH.z =FpszH2[0]*st_5 + FpszH2[1]*st_4 + FpszH2[2]*st_3 + FpszH2[3]*st_2 + FpszH2[4]*st_1 + FpszH2[5];
                    oldpRHz[0] = QP.pRH.z;
                    oldpRHz[1] = 5*FpszH2[0]*st_4 + 4*FpszH2[1]*st_3 + 3*FpszH2[2]*st_2 + 2*FpszH2[3]*st_1 + FpszH2[4];
                    oldpRHz[2] = 20*FpszH2[0]*st_3 + 12*FpszH2[1]*st_2 + 6*FpszH2[2]*st_1 + 2*FpszH2[3];
                    QP.dRH.z = oldpRHz[1];
                    QP.ddRH.z = oldpRHz[2];
                }
            }


            FootonAir[0] = true;
            FootonAir[3] = true;



            QP.pLH.z =FpszH[0]*st_5 + FpszH[1]*st_4 + FpszH[2]*st_3 + FpszH[3]*st_2 + FpszH[4]*st_1 + FpszH[5];
            oldpLHz[0] = QP.pLH.z;
            oldpLHz[1] = 5*FpszH[0]*st_4 + 4*FpszH[1]*st_3 + 3*FpszH[2]*st_2 + 2*FpszH[3]*st_1 + FpszH[4];
            oldpLHz[2] = 20*FpszH[0]*st_3 + 12*FpszH[1]*st_2 + 6*FpszH[2]*st_1 + 2*FpszH[3];
            QP.dLH.z = oldpLHz[1];
            QP.ddLH.z = oldpLHz[2];


            Fps = calc_5th((t_now-dt),WP.step_T,oldpLHx,vec3(FootTogoLH.x,0,0));

            QP.pLH.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpLHx[0] = QP.pLH.x;
            oldpLHx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpLHx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dLH.x = oldpLHx[1];
            QP.ddLH.x = oldpLHx[2];

            Fps = calc_5th((t_now-dt),WP.step_T,oldpLHy,vec3(FootTogoLH.y,0,0));

            QP.pLH.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpLHy[0] = QP.pLH.y;
            oldpLHy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpLHy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dLH.y = oldpLHy[1];
            QP.ddLH.y = oldpLHy[2];


            QP.pRF.z =FpszF[0]*st_5 + FpszF[1]*st_4 + FpszF[2]*st_3 + FpszF[3]*st_2 + FpszF[4]*st_1 + FpszF[5];
            oldpRFz[0] =QP.pRF.z;
            oldpRFz[1] = 5*FpszF[0]*st_4 + 4*FpszF[1]*st_3 + 3*FpszF[2]*st_2 + 2*FpszF[3]*st_1 + FpszF[4];
            oldpRFz[2] = 20*FpszF[0]*st_3 + 12*FpszF[1]*st_2 + 6*FpszF[2]*st_1 + 2*FpszF[3];
            QP.dRF.z = oldpRFz[1];
            QP.ddRF.z = oldpRFz[2];

            Fps = calc_5th((t_now-dt),WP.step_T,oldpRFx,vec3(FootTogoRF.x,0,0));

            QP.pRF.x =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpRFx[0] = QP.pRF.x;
            oldpRFx[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpRFx[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dRF.x = oldpRFx[1];
            QP.ddRF.x = oldpRFx[2];

            Fps = calc_5th((t_now-dt),WP.step_T,oldpRFy,vec3(FootTogoRF.y,0,0));

            QP.pRF.y =Fps[0]*st_5 + Fps[1]*st_4 + Fps[2]*st_3 + Fps[3]*st_2 + Fps[4]*st_1 + Fps[5];
            oldpRFy[0] = QP.pRF.y;
            oldpRFy[1] = 5*Fps[0]*st_4 + 4*Fps[1]*st_3 + 3*Fps[2]*st_2 + 2*Fps[3]*st_1 + Fps[4];
            oldpRFy[2] = 20*Fps[0]*st_3 + 12*Fps[1]*st_2 + 6*Fps[2]*st_1 + 2*Fps[3];
            QP.dRF.y = oldpRFy[1];
            QP.ddRF.y = oldpRFy[2];

        }

}
QuadJoints OW_Quad::Flytrot_onestep(WalkSensors _WS)
{
    WS = _WS;
    if(Qcnt<2)
    {
        //upL = 0.13;//13cm up
        //too much voltage from here
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

    fallcheck();


//make-motion from here

    double stanceT = (1-flyRatio)*WP.step_T;

    double flyT = WP.step_T-stanceT;
    double upV = flyT*g*0.5;

    //vec3 des_V = FrontVector*WP.step_L.x/WP.step_T + LeftVector*WP.step_L.y/WP.step_T;
    vec3 des_V = FrontVector*(WP.step_L.x+ff)/WP.step_T
            +LeftVector*(WP.step_L.y+ll)/WP.step_T;
    double L0 = WP.delZ;//+0.02;

    QuadJoints QJenc = WS.JointPosEnc;
    QJenc.qPel = QP.qPel;
    QuadPos QPee = Oi.FK(QJenc);
    double midcnt = 0;
    vec3 midpoint = vec3(0,0,0);
    if(isRHLFmove==true)//have to check
    {
        {midpoint = midpoint+QPee.pRH;midcnt++;}
        {midpoint = midpoint+QPee.pLF;midcnt++;}
    }
    else
    {//check midpoint
        {midpoint = midpoint+QPee.pLH;midcnt++;}
        {midpoint = midpoint+QPee.pRF;midcnt++;}
    }

    if(midcnt>0.5){midpoint = midpoint/midcnt;}
    double midcomp = L0-(QPee.pCOM.z-midpoint.z);
    //trot-like code
    if(NO_ROBOT_TEST)
    {midcomp = 0.03;}
    if(t_now>WP.step_T-0.5*dt)//&&midcomp>0.02)//if(landed)
    {
        cout<<"landed " <<L0<<" "<<midcomp<<" "<<t_now<<endl;
        double maxff = 0.2;
        double maxll = 0.2;
        double kkfb = 0.1*2;// 0.3*0.5;//hmm.......
        double kklr = 0.1;//0.3*0.5;//hmm.......
        double ffoff = 0.00;//0.06;//I want to make this to be zero
        sL = FrontVector*(WP.step_L.x)+LeftVector*(WP.step_L.y);
        desCOM = desCOM+sL;
        estCOM = QP.pCOM;
//        if(((desCOM-estCOM).norm())>1.0)
//        {
//            desCOM = estCOM+((desCOM-estCOM))/((desCOM-estCOM).norm())*1.0;
//        }//no_limitation so far
        ff = kkfb*dot(FrontVector,(desCOM-estCOM))+ffoff;//ff feedforward
        ll = kklr*dot(LeftVector,(desCOM-estCOM));
        if(ff>maxff+ffoff){ff = maxff+ffoff;}
        if(ff<-maxff+ffoff){ff = -maxff+ffoff;}
        if(ll>maxll){ll = maxll;}
        if(ll<-maxll){ll = -maxll;}

        des_V = FrontVector*(WP.step_L.x+ff)/WP.step_T
                   +LeftVector*(WP.step_L.y+ll)/WP.step_T;


        cout<<"ff "<<ff<<" ll "<<ll<<endl;//lets see this reduce/increase

        cout<<"des_V "<<des_V.x<<" "<<des_V.y<<" "<<des_V.z<<endl;//lets see this reduce/increase
        cout<<"sL "<<sL.x<<" "<<sL.y<<" "<<sL.z<<endl;//lets see this reduce/increase
        //////////////////////have to enable here after fix robot

        //not tested yet


//        sL = FrontVector*(WP.step_L.x)+LeftVector*(WP.step_L.y);
//        des_V = sL/WP.step_T;//no_high_level_feedback

        isRHLFmove = !isRHLFmove;
        isFirststep = false;

        oldRotZ[0] = 0;
        oldRotX[0] = 0;
        oldRotY[0] = 0;
        t_now =0;
        qPel_stepstart = QP.qPel;

        if(isStopping)
        {
            stopcnt--;
            if(stopcnt<0)
            {
//                desCOM = QP.pCOM;
//                QP.dCOM.z = 0;
//                sumCOMerr = vec3();
//                for(int i=0;i<5;i++)
//                {
//                    stepLFilter[i] = vec3();
//                }
//                WP_next = WP;
//                WP_next.step_T = 0.25;
//                WP_next.Gait = Trot;
//                WP = WP_next;
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
        }//change to walkingtrot
        bool oknextGait = (WP_next.Gait==Flytrot);
        if(walkchanged&&oknextGait)
        {
            WP = WP_next;
            des_delZ = WP.delZ;
            walkchanged  = false;
        }


        double kddcomz = 10;
        ddcomz = (upV-QP.dCOM.z)/stanceT + kddcomz*(L0-QP.pCOM.z+0.02);

        //cout<<"landV " <<QP.dCOM.z<<" "<<ddcomz<<" "<<stanceT<<endl;

    }
    //CoM trajectory
    if(isFirststep)//pronk
    {
        ddcomz = upV/stanceT;
        QP.ddCOM = vec3();

    }
    else
    {
        QP.ddCOM = QP_ddCOM;//follow last input?
    }
    if(t_now<stanceT-0.5*dt)//jump
    {
        QP.ddCOM.z = ddcomz;//have to do here
        calc3th = false;
    }
    else
    {
        if(calc3th==false)
        {
            //update QP.dCOM as estimated QP.dpel
            QP.dCOM.x = QP_dQnow_save[0];
            QP.dCOM.y = QP_dQnow_save[1];//hmm....


            calc3th = true;
        }
        QP.ddCOM.z = -g;
    }
    QP.pCOM = QP.pCOM + QP.dCOM*dt + QP.ddCOM*dt*dt/2;
    QP.dCOM = QP.dCOM + QP.ddCOM*dt;

    if(isFirststep)
    {
        ZMP = (QP.pRH+QP.pLF+QP.pLH+QP.pRF)*0.25;
    }
    else if(isRHLFmove==true)// RH,LF swing
    {
        ZMP= (QP.pLH+QP.pRF)*0.5;//later controllable factor
    }
    else
    {
        ZMP = (QP.pRH+QP.pLF)*0.5;//later controllable factor
    }


    COM_nn = QP.pCOM + QP.dCOM*(WP.step_T-t_now);//super-simple-scheme

    double k_footstep = 0.5;//gain~~
    //delx ~ -WP.FB_L*pitch
    double k_angle_y = WP.FB_L;
    double k_angle_x = -WP.LR_L;
    vec3 angle_off = FrontVector*k_angle_y*WS.IMUangle.y
                    +LeftVector*k_angle_x*WS.IMUangle.x;

    double k_com =0.03;//sign was reversed
    vec3 offFF = 0.5*sL + k_com*(estCOM-desCOM);//sL = des_V*WP.step_T;

    CP_nn = (COM_nn+offFF) + WP.step_T*k_footstep*(QP.dCOM-des_V);//+angle_off;
    if(CP_nn.x-COM_nn.x>foot_xlim){CP_nn.x = COM_nn.x + foot_xlim;}
    if(CP_nn.x-COM_nn.x<-foot_xlim){CP_nn.x = COM_nn.x - foot_xlim;}
    if(CP_nn.y-COM_nn.y>foot_ylim){CP_nn.y = COM_nn.y + foot_ylim;}
    if(CP_nn.y-COM_nn.y<-foot_ylim){CP_nn.y = COM_nn.y - foot_ylim;}
    CP_nn.z = 0;
    ZMP.z = 0;
    //foot placement control

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

        vec3 Center = CP_nn;

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

        vec3 Center = CP_nn;


        FootTogoRF = Center + RFV;
        FootTogoLH = Center + LHV;

        FootTogoLF = QP.pLF;//should be modefied
        FootTogoRH = QP.pRH;//should be modefied
    }


    //move_leg_here~~
    MOVE_LEGS_FLYTROT();



//make-motion to here

    int ncon = 0;
    for(int i=0;i<4;i++)
    {
        if(FootonAir[i]==false){ncon++;}
    }

    if(ncon==4)
    {
        FFz_ref();
        PRONK_control();
    }
    else if(ncon==2)
    {
        FFz_ref_ssp();
        FLYTROT_control();
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

void OW_Quad::FLYTROT_control()
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



    control_joints_jump(Gains);//stance leg damping only?


}
