#include "ow_quad.h"
void OW_Quad::calc_decomp_slope()//ok maybe
{
    vec3 rpyQP = Oi.Decomp_Rmat(QP.qPel,vec3(0,0,1),vec3(0,1,0),vec3(1,0,0));
    mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
    mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
    mat3 IMUrotz = mat3(vec3(0,0,-1),rpyQP[0]);
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

    //decomp_A = Oi.Decomp_Rmat(IMUrot,tx,ty,tz);//x-y-z???
    ///////////////////
    mat3 IMUrotxp = mat3(vec3(-1,0,0),plane_rpy_f[0]);
    mat3 IMUrotyp = mat3(vec3(0,-1,0),plane_rpy_f[1]);
    mat3 IMUrotzp = mat3(vec3(0,0,-1),rpyQP[0]);
    mat3 IMUrotp = IMUrotzp*IMUrotyp*IMUrotxp;//z-y-x
    //////////////////
    decomp_A = Oi.Decomp_Rmat(IMUrot,tx,ty,tz);//x-y-z???
    vec3 decomp_Ap = Oi.Decomp_Rmat(IMUrotp,tx,ty,tz);//x-y-z???

    double tt = decomp_A.x-decomp_Ap.x;
    decomp_A.x = decomp_A.z;
    decomp_A.y = decomp_A.y-decomp_Ap.y;
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
    rotE_I.x = rotE_I.x+(WS.IMUangle.x-plane_rpy_f.x)*dt;
    rotE_I.y = rotE_I.y+(WS.IMUangle.y-plane_rpy_f.y)*dt;

    double maxrotE = 4*D2Rf;
    if(rotE_I.x>maxrotE){rotE_I.x = maxrotE;}
    if(rotE_I.x<-maxrotE){rotE_I.x = -maxrotE;}
    if(rotE_I.y>maxrotE){rotE_I.y = maxrotE;}
    if(rotE_I.y<-maxrotE){rotE_I.y = -maxrotE;}
    rotE_I.z = 0;
    rotE_I.y = 0;//x only....

    vec3 rotE_I_Global = IMUrot*rotE_I;
    decomp_I.x = 0;
    decomp_I.y = dot(ty,rotE_I_Global);
    decomp_I.z = dot(tx,rotE_I_Global);


    decomp_W_filtered.x = LPFs[3].do_filt(decomp_W.x);
    decomp_W_filtered.y = LPFs[4].do_filt(decomp_W.y);
    decomp_W_filtered.z = LPFs[5].do_filt(decomp_W.z);

    oldRHLFmove = isRHLFmove;

    if(NO_ROBOT_TEST)
    {
        decomp_A[1] = 0; decomp_A[2] = 0;
        decomp_I = vec3(0,0,0);
        rotE_I.z = 0;
        rotE_I.y = 0;//x only....
        rotE_I.x = 2*D2Rf;//x only....

        vec3 rotE_I_Global = IMUrot*rotE_I;
        decomp_I.x = 0;
        decomp_I.y = dot(ty,rotE_I_Global);
        decomp_I.z = dot(tx,rotE_I_Global);

    }

}
void OW_Quad::MOVE_LEGS_TROT_SLOPE()//ok maybe
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

    VectorNd Fps, FpszH, FpszF;
    double t_u = WP.step_T*(1.0-WP.dsp_ratio)*upRatio;
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
        cout<<"contact_L "<<contact_L<<"ori "<<sqrt(WP.LR_L*WP.LR_L+WP.FB_L*WP.FB_L)<<endl;
    }
    wn = 1.0;
    z = 1.0*1.0;//for test
    double Aerr = decomp_A[0]-WS.IMUangle.z;
    if(Aerr> M_PI){Aerr =Aerr-2*M_PI;}
    if(Aerr<-M_PI){Aerr =Aerr+2*M_PI;}
    //double ddYawcon = -wn*wn*(Aerr)-2*wn*z*(oldRotZ[1]-WS.IMUomega.z);//rotvelref
    ddYawcon = -QP_ddQnow[5];
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

    //////////////////////////////////////////////////////

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

            Fcenter = QP.pCOM;

            vec3 Foff,Hoff;

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



            QP.pRH.z = 0.9*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
            QP.pLF.z = 0.9*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
            QP.pLH.z = 0.9*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
            QP.pRF.z = 0.9*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
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
                }
                FpszH = calc_5th((t_now-dt),t_u,oldpFzH,vec3(upL+FootTogoRH.z*0.5,0,0));
                FpszF = calc_5th((t_now-dt),t_u,oldpFzF,vec3(upL+FootTogoLF.z*0.5,0,0));

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
                }
                FpszH = calc_5th((t_now-dt),t_u,oldpFzH,vec3(upL+FootTogoLH.z*0.5,0,0));
                FpszF = calc_5th((t_now-dt),t_u,oldpFzF,vec3(upL+FootTogoRF.z*0.5,0,0));

            }



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

                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1])+FootTogoRH.z;
                Flandoff = -(dot(offF,ty))*tan(decomp_A[2])-(dot(offF,tx))*tan(decomp_A[1])+FootTogoLF.z;
            }
            else
            {
                vec3 offH = FootTogoLH-COM_nn;
                offH.z = 0;
                vec3 offF = FootTogoRF-COM_nn;
                offF.z = 0;
                Hlandoff = -(dot(offH,ty))*tan(decomp_A[2])-(dot(offH,tx))*tan(decomp_A[1])+FootTogoLH.z;
                Flandoff = -(dot(offF,ty))*tan(decomp_A[2])-(dot(offF,tx))*tan(decomp_A[1])+FootTogoRF.z;
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
                    QP.pRH.z = 0.9*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
                }
                if(LFlanded)
                {
                    QP.pLF.z = 0.9*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
                }
            }

            QP.pLH.z = 0.9*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
            QP.pRF.z = 0.9*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;


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
                QP.pRH.z = 0.98*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
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
                QP.pLF.z = 0.98*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
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
                        printf("LFlanded\n");
                     // todmZero[LHP] = todmZero[LKN] = true;

                    }
                }
                if(WS_NOW&DWR&&RHlanded==false)
                {
                    //if(dms[REB]-dms_old[REB]<-3)
                    //if(WS.RH_Fz>landing_fthres)
                    {
                        oldpFzH[1] = 0;oldpFzH[2] = 0;RHlanded = true; FootonAir[2] = false;
                        printf("RHlanded\n");
                      // todmZero[RSP] = todmZero[REB] = true;
                    }
                }
            }


        }
        else// LH,RF swing
        {
            RHlanded = false;
            LFlanded = false;

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
                    QP.pLH.z = 0.9*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
                }
                if(RFlanded)
                {
                    QP.pRF.z = 0.9*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
                }
            }

            QP.pRH.z = 0.9*(QP.pRH.z-FootTogoRH.z)+FootTogoRH.z;
            QP.pLF.z = 0.9*(QP.pLF.z-FootTogoLF.z)+FootTogoLF.z;
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
                QP.pLH.z = 0.98*(QP.pLH.z-FootTogoLH.z)+FootTogoLH.z;
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
                QP.pRF.z = 0.98*(QP.pRF.z-FootTogoRF.z)+FootTogoRF.z;
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
                        printf("RFlanded\n");
                        //todmZero[RHP] = todmZero[RKN] = true;
                    }
                }
                if(WS_NOW&DWR&&LHlanded==false)
                {
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
    }
}
void OW_Quad::DSP_control_slope()//qPel control not implemented
{
    double Kpy = 0.02*R2Df*2;
    double Kdy = 0.001*R2Df;
    double Kpx = 0.01*R2Df*2;
    double Kdx = 0.0005*R2Df;

    //modify RFz_ref, LFz_ref, RHz_ref, LHz_ref
    //and Leg length at once???
    //lets try just leg length

    FC_FB = (+ Kpy*(WS.IMUangle.y-plane_rpy_f.y)  + Kdy*WS.IMUomega.y);
    FC_LR = -(- Kpx*(WS.IMUangle.x-plane_rpy_f.x) - Kdx*WS.IMUomega.x);
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
void OW_Quad::SSP_control_slope()//qPel control not implemented
{

    double Kpy = 0.02*R2Df*2.0;//*0.3;//not needed anymore
    double Kdy = 0.001*R2Df*1.5;//*0.3;//not needed anymore
    double KpA2 = 0.3*1.5*0.3;//not needed anymore
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

    JointGains Gains;

    control_joints(Gains);//stance leg damping only?
}
