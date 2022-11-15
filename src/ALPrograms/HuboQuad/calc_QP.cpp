#include "OW_RBDL.h"

VectorNd OW_DYNAMICS::calc_QP(VectorNd _Q, VectorNd _Qref, VectorNd _dQ, VectorNd _dQref, VectorNd _ddQref, bool cRF, bool cLF, bool cRH, bool cLH,int feedback_pose, vec3 sV,QP_wnzW0s ws)
{
    double z, wn;
    MatrixNd _M = MatrixNd::Zero (Robot->dof_count, Robot->dof_count);
    CompositeRigidBodyAlgorithm(*Robot,_Q,_M,true);
    VectorNd Nonlin = VectorNd::Zero (Robot->dof_count);
    NonlinearEffects(*Robot,_Q,_dQ,Nonlin);
    CalcEndeffectorJacobian3D(_Q);
    MakeContactJacobian(cRF,cLF,cRH,cLH);
    VectorNd ddqZero = VectorNd::Zero(_dQ.size());
    Vector3d RFdJdQ = CalcPointAcceleration(*Robot,_Q,_dQ,ddqZero,n_rkn,v2V(Oi.offset_lleg));
    Vector3d LFdJdQ = CalcPointAcceleration(*Robot,_Q,_dQ,ddqZero,n_lkn,v2V(Oi.offset_lleg));
    Vector3d RHdJdQ = CalcPointAcceleration(*Robot,_Q,_dQ,ddqZero,n_reb,v2V(Oi.offset_lleg));
    Vector3d LHdJdQ = CalcPointAcceleration(*Robot,_Q,_dQ,ddqZero,n_leb,v2V(Oi.offset_lleg));
    Vector3d BasedJdQ = CalcPointAcceleration(*Robot,_Q,_dQ,ddqZero,n_torso,Vector3d(0.0,0.0,0.0));
    SpatialVector BaseddJdQ6D = CalcPointAcceleration6D(*Robot,_Q,_dQ,ddqZero,n_torso,Vector3d(0.0,0.0,0.0));
    Vector3d BasedJRdQ = Vector3d(BaseddJdQ6D(3),BaseddJdQ6D(4),BaseddJdQ6D(5));


    MatrixNd ddx_djdq = MatrixNd::Zero(ContactJacobian.rows(),1);
    int n_con = 0;
    if(cRF){ddx_djdq.block(n_con*3,0,3,1) = -RFdJdQ; n_con++;}
    if(cLF){ddx_djdq.block(n_con*3,0,3,1) = -LFdJdQ; n_con++;}
    if(cRH){ddx_djdq.block(n_con*3,0,3,1) = -RHdJdQ; n_con++;}
    if(cLH){ddx_djdq.block(n_con*3,0,3,1) = -LHdJdQ; n_con++;}

    std::vector<MatrixNd> As,Bs;
    MatrixNd A0,B0;
    double W0;
    std::vector<double> Ws;
    if(n_con==0)
    {
         OC.setNums(18+12,18,0);//ddq, tau, contactforce
         OC.MakeEqFly(_M,Nonlin);
         OC.MakeIneq();
         ////////////////////swingleg
         double z =ws.swingleg.z;
         double wn = ws.swingleg.wn;
         int cnt4SL = 0;
         A0  = MatrixNd::Zero(18+12,18+12);
         B0  = MatrixNd::Zero(18+12,1);
         double ddQfb[18];
         for(int i=6;i<18;i++)
         {
             ddQfb[i] = -2*z*wn*(_dQ[i]-_dQref[i]) - wn*wn*(_Q[i]-_Qref[i]);
             //joint reference angle based control
         }
         if(!cRF)
         {
             A0(cnt4SL+0,6) = 1; B0(cnt4SL+0,0) = ddQfb[6]+_ddQref[6];
             A0(cnt4SL+1,7) = 1; B0(cnt4SL+1,0) = ddQfb[7]+_ddQref[7];
             A0(cnt4SL+2,8) = 1; B0(cnt4SL+2,0) = ddQfb[8]+_ddQref[8];
             cnt4SL+=3;
         }
         if(!cLF)
         {
             A0(cnt4SL+0,9) = 1; B0(cnt4SL+0,0) = ddQfb[9]+_ddQref[9];
             A0(cnt4SL+1,10) = 1; B0(cnt4SL+1,0) = ddQfb[10]+_ddQref[10];
             A0(cnt4SL+2,11) = 1; B0(cnt4SL+2,0) = ddQfb[11]+_ddQref[11];
             cnt4SL+=3;
         }
         if(!cRH)
         {
             A0(cnt4SL+0,12) = 1; B0(cnt4SL+0,0) = ddQfb[12]+_ddQref[12];
             A0(cnt4SL+1,13) = 1; B0(cnt4SL+1,0) = ddQfb[13]+_ddQref[13];
             A0(cnt4SL+2,14) = 1; B0(cnt4SL+2,0) = ddQfb[14]+_ddQref[14];
             cnt4SL+=3;
         }
         if(!cLH)
         {
             A0(cnt4SL+0,15) = 1; B0(cnt4SL+0,0) = ddQfb[15]+_ddQref[15];
             A0(cnt4SL+1,16) = 1; B0(cnt4SL+1,0) = ddQfb[16]+_ddQref[16];
             A0(cnt4SL+2,17) = 1; B0(cnt4SL+2,0) = ddQfb[17]+_ddQref[17];
             cnt4SL+=3;
         }
         if(cnt4SL>0)
         {
             W0 = ws.swingleg.W0;
             As.push_back(A0);
             Bs.push_back(B0);
             Ws.push_back(W0);
         }
         A0  = MatrixNd::Identity(18+12,18+12);
         B0  = MatrixNd::Zero(18+12,1);

         B0(2,0) = -9.81;//-g
         W0 = 1e-6;
         As.push_back(A0);
         Bs.push_back(B0);
         Ws.push_back(W0);

         OC.MakeHF(As,Bs,Ws);
         return OC.calcX2();

    }



    OC.setNums(18+12+3*n_con,18,5*n_con);//ddq, tau, contactforce

    OC.MakeEq(_M,Nonlin,ContactJacobian);

    OC.MakeIneq();


    //

    //ddq,Tau,F
    ///////////////contactconstraint + reduce leg contact speed
    ///
    z = ws.contact.z;
    wn = ws.contact.wn;
    VectorNd LegSpeed = ContactJacobian*_dQ;
    if(LegSpeed.maxCoeff()>0.1||LegSpeed.minCoeff()<-0.1)
    {
        //cout<<LegSpeed.transpose()<<endl;
    }
    A0  = MatrixNd::Zero(18+12+3*n_con,18+12+3*n_con);
    B0  = MatrixNd::Zero(18+12+3*n_con,1);
    A0.block(0,0,ContactJacobian.rows(),ContactJacobian.cols()) = ContactJacobian;
    B0.block(0,0,ddx_djdq.rows(),ddx_djdq.cols()) = ddx_djdq -wn*LegSpeed;
    W0 = ws.contact.W0;
    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);


    ////////////////////swingleg
    ///
    z =ws.swingleg.z;
    wn = ws.swingleg.wn;
    int cnt4SL = 0;
    A0  = MatrixNd::Zero(18+12+3*n_con,18+12+3*n_con);
    B0  = MatrixNd::Zero(18+12+3*n_con,1);
    double ddQfb[18];
    for(int i=6;i<18;i++)
    {
        ddQfb[i] = -2*z*wn*(_dQ[i]-_dQref[i]) - wn*wn*(_Q[i]-_Qref[i]);
        //joint reference angle based control
    }
    if(!cRF)
    {
        A0(cnt4SL+0,6) = 1; B0(cnt4SL+0,0) = ddQfb[6]+_ddQref[6];
        A0(cnt4SL+1,7) = 1; B0(cnt4SL+1,0) = ddQfb[7]+_ddQref[7];
        A0(cnt4SL+2,8) = 1; B0(cnt4SL+2,0) = ddQfb[8]+_ddQref[8];
        cnt4SL+=3;
    }
    if(!cLF)
    {
        A0(cnt4SL+0,9) = 1; B0(cnt4SL+0,0) = ddQfb[9]+_ddQref[9];
        A0(cnt4SL+1,10) = 1; B0(cnt4SL+1,0) = ddQfb[10]+_ddQref[10];
        A0(cnt4SL+2,11) = 1; B0(cnt4SL+2,0) = ddQfb[11]+_ddQref[11];
        cnt4SL+=3;
    }
    if(!cRH)
    {
        A0(cnt4SL+0,12) = 1; B0(cnt4SL+0,0) = ddQfb[12]+_ddQref[12];
        A0(cnt4SL+1,13) = 1; B0(cnt4SL+1,0) = ddQfb[13]+_ddQref[13];
        A0(cnt4SL+2,14) = 1; B0(cnt4SL+2,0) = ddQfb[14]+_ddQref[14];
        cnt4SL+=3;
    }
    if(!cLH)
    {
        A0(cnt4SL+0,15) = 1; B0(cnt4SL+0,0) = ddQfb[15]+_ddQref[15];
        A0(cnt4SL+1,16) = 1; B0(cnt4SL+1,0) = ddQfb[16]+_ddQref[16];
        A0(cnt4SL+2,17) = 1; B0(cnt4SL+2,0) = ddQfb[17]+_ddQref[17];
        cnt4SL+=3;
    }

    if(cnt4SL>0)
    {
        W0 = ws.swingleg.W0;
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);
    }

    ///////////////ddpelz = k(pel_ref-pel) + kd(peld_ref - pel_d)
    z =ws.pelz.z;
    wn = ws.pelz.wn;//3;//reduced from 15

    A0  = MatrixNd::Zero(18+12+3*n_con,18+12+3*n_con);
    B0  = MatrixNd::Zero(18+12+3*n_con,1);
    double ddpelzB  = wn*wn*(_Qref[2]-_Q[2]) + 2*z*wn*(_dQref[2]-_dQ[2]);
    double Mddpel = 3;//~0.3g
    if(ddpelzB>Mddpel){ ddpelzB = Mddpel;}
    if(ddpelzB<-Mddpel){ ddpelzB = -Mddpel;}
    A0(0,2) = 1;
    B0(0,0) = ddpelzB +_ddQref[2];
    W0 = ws.pelz.W0;
    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);

    /////////////simplified to xyz rotation.
    /// //is in global frame

    double wnx = ws.rotx.wn;
    double zx = ws.rotx.z;
    double wny = ws.roty.wn;
    double zy = ws.roty.z;
    double wnz = ws.rotz.wn;
    double zz = ws.rotz.z;


    A0  = MatrixNd::Zero(18+12+3*n_con,18+12+3*n_con);
    B0  = MatrixNd::Zero(18+12+3*n_con,1);
    quat qPelnow = quat(_Q[18],_Q[3],_Q[4],_Q[5]);
    quat qPelref = quat(_Qref[18],_Qref[3],_Qref[4],_Qref[5]);
    quat qPelnow_minus = quat(_Q[18],-_Q[3],-_Q[4],-_Q[5]);
    quat qPelref_minus = quat(_Qref[18],-_Qref[3],-_Qref[4],-_Qref[5]);
    //quat deltaQ = quat(mat3(qPelref)*(mat3(qPelnow).inverse()));
    quat deltaQ = qPelnow*qPelref_minus;//this is wrong maybe

    if(deltaQ[0]<0)
    {
        deltaQ[0] = -deltaQ[0];
        deltaQ[1] = -deltaQ[1];
        deltaQ[2] = -deltaQ[2];
        deltaQ[3] = -deltaQ[3];
    }
    double angle = 2*acos(deltaQ[0]);
    double sahalf = sin(angle/2);
    vec3 rotxyz = vec3(0,0,0);
    if(angle>1e-6)
    {
        rotxyz = -vec3(deltaQ[1],deltaQ[2],deltaQ[3])/sahalf * angle;

        rotxyz =mat3( qPelnow).inverse()*rotxyz;//local....
        if(angle>20*D2Rf)
        {
//            using namespace std;
//            cout<<"qPelnow "<<_Q[18]<<" "<<_Q[3]<<" "<<_Q[4]<<" "<<_Q[5]<<" "<<endl;
//            cout<<"qPelref "<<_Qref[18]<<" "<<_Qref[3]<<" "<<_Qref[4]<<" "<<_Qref[5]<<" "<<endl;
//            cout<<"rotxyz "<<rotxyz.x<<" "<<rotxyz.y<<" "<<rotxyz.z<<endl;
//            cout<<"-dqrot "<<-_dQ[3]<<" "<<-_dQ[4]<<" "<<-_dQ[5]<<endl;
        }
    }
    rotxyzcon = rotxyz;
    double dwx = wnx*wnx*(rotxyz.x) + 2*zx*wnx*(_dQref[3]-_dQ[3])  + _ddQref[3];// - BasedJRdQ(0);//xrot to be zero  -dJR*dQ??
    double dwy = wny*wny*(rotxyz.y) + 2*zy*wny*(_dQref[4]-_dQ[4])  + _ddQref[4];// - BasedJRdQ(1);//yrot to be zero -dJR*dQ??
    double dwz = wnz*wnz*(rotxyz.z) + 2*zz*wnz*(_dQref[5]-_dQ[5])  + _ddQref[5];// - BasedJRdQ(2);//yrot to be zero -dJR*dQ??

    double maxdw = 300;
    if(dwx>maxdw){dwx = maxdw;}
    if(dwx<-maxdw){dwx = -maxdw;}
    if(dwy>maxdw){dwy = maxdw;}
    if(dwy<-maxdw){dwy = -maxdw;}
    if(dwz>maxdw){dwz = maxdw;}
    if(dwz<-maxdw){dwz = -maxdw;}
    if(feedback_pose==XYVEL_MPC)
    {
        maxdw = 60;
        std::vector<vec3> ref;
        ref.push_back(vec3());
        ref[0] = vec3(0,_dQref[3],0);
        dwx = M1->getddx(vec3(rotxyz.x,_dQ[3],0),ref,stanceN,flyT,MPC_R.x,MPC_Qx.x,MPC_Qv.x,maxdw);
        ref[0] = vec3(0,_dQref[4],0);
        dwy = M1->getddx(vec3(rotxyz.y,_dQ[4],0),ref,stanceN,flyT,MPC_R.y,MPC_Qx.y,MPC_Qv.y,maxdw);
        ref[0] = vec3(0,_dQref[5],0);
        //dwz = M1->getddx(vec3(rotxyz.z,_dQ[5],0),ref,stanceN,flyT,MPC_R.z,MPC_Qx.z,MPC_Qv.z,maxdw);
    }

    A0(0,3) = 1;
    B0(0,0) = dwx;
    A0(1,4) = 1;
    B0(1,0) = dwy;
    A0(2,5) = 1;
    B0(2,0) = dwz;
    W0 = ws.rotx.W0;
    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);


    if(feedback_pose==XYPOS)//////////base position
    {
        double z = ws.pelxy.z;
        double wn = ws.pelxy.wn;
        A0  = MatrixNd::Zero(18+12+3*n_con,18+12+3*n_con);
        B0  = MatrixNd::Zero(18+12+3*n_con,1);
        A0(0,0) = 1;
        B0(0,0) = wn*wn*(_Qref[0]-_Q[0]) + 2*z*wn*(_dQref[0]-_dQ[0]) + _ddQref[0];
        A0(1,1) = 1;
        B0(1,0) = wn*wn*(_Qref[1]-_Q[1]) + 2*z*wn*(_dQref[1]-_dQ[1]) + _ddQref[1];
        W0 = ws.pelxy.W0;
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);
    }
    else if(feedback_pose==XYVEL||feedback_pose==XYVEL_MPC)//base velocity->this makes robot more stable, but contact force too small
    {//effective->

        double z = ws.dpelxy.z;
        double wn = ws.dpelxy.wn;
        A0  = MatrixNd::Zero(18+12+3*n_con,18+12+3*n_con);
        B0  = MatrixNd::Zero(18+12+3*n_con,1);
        A0(0,0) = 1;
        B0(0,0) = 2*z*wn*(sV.x-_dQ[0]);
        A0(1,1) = 1;
        B0(1,0) = 2*z*wn*(sV.y-_dQ[1]);
        W0 = ws.dpelxy.W0;//0.3
        As.push_back(A0);
        Bs.push_back(B0);
        Ws.push_back(W0);

    }
    else//MPCNONE
    {

    }
    A0  = MatrixNd::Identity(18+12+3*n_con,18+12+3*n_con);//make it smaller
    B0  = MatrixNd::Zero(18+12+3*n_con,1);
    W0 = 1e-6;
    As.push_back(A0);
    Bs.push_back(B0);
    Ws.push_back(W0);
    OC.MakeHF(As,Bs,Ws);

    return OC.calcX2();
}
