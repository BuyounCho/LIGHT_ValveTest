#ifndef OW_RBDL_H
#define OW_RBDL_H
#include "Oinverse.h"
#include <vector>
#include "ow_cplex.h"
#include "rbdl/rbdl.h"
#include "Eigen/Geometry"
#include "ow_onedofmpc.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
enum XYMODE
{
    XYPOS,XYVEL,XYNONE,
    XYVEL_MPC
};
class wnzW0
{
public:
    double wn;
    double z;
    double W0;
};
class QP_wnzW0s
{
    public:
    QP_wnzW0s()
    {
//        contact.wn = 2;
//        contact.z = 1;
//        contact.W0 = 1;

//        swingleg.wn = 5;
//        swingleg.z = 1.0;
//        swingleg.W0 = 1;

//        pelz.wn = 3;
//        pelz.z = 1.5;
//        pelz.W0 = 1;

//        rotx.wn = 12;
//        rotx.z = 1.5;
//        rotx.W0 = 1;
//        roty.wn = 12;
//        roty.z = 1.5;
//        roty.W0 = 1;
//        rotz.wn = 1;
//        rotz.z = 1;
//        rotz.W0 = 1;

//        pelxy.wn = 1.0;
//        pelxy.z = 1.0;
//        pelxy.W0 = 1.0;

//        dpelxy.wn = 2;
//        dpelxy.z = 1;
//        dpelxy.W0 = 0.3;

        contact.wn = 2;
        contact.z = 1;
        contact.W0 = 1;

        swingleg.wn = 3;
        swingleg.z = 1.0;
        swingleg.W0 = 1;

        pelz.wn = 0.5;
        pelz.z = 1;
        pelz.W0 = 1;

        rotx.wn = 0.5;
        rotx.z = 1;
        rotx.W0 = 1;
        roty.wn = 0.5;
        roty.z = 1;
        roty.W0 = 1;
        rotz.wn = 0.5;
        rotz.z = 1;
        rotz.W0 = 1;

        pelxy.wn = 0.5;
        pelxy.z = 1.0;
        pelxy.W0 = 1.0;

        dpelxy.wn = 2;//have to tune with ff, ll, and ZMPoff.....
        dpelxy.z = 1;
        dpelxy.W0 = 0.3;
    }
    wnzW0 contact;
    wnzW0 swingleg;
    wnzW0 pelz;
    wnzW0 rotx;
    wnzW0 roty;
    wnzW0 rotz;
    wnzW0 pelxy;
    wnzW0 dpelxy;

};
class OW_DYNAMICS
{
    OW_CPLEX OC;
    public:
    Model* Robot;
    Body b_torso;
    Body b_rhr, b_rhp, b_rkn;
    Body b_lhr, b_lhp, b_lkn;
    Body b_rsr, b_rsp, b_reb;
    Body b_lsr, b_lsp, b_leb;

    Joint j_world;    
    Joint j_rhr, j_rhp, j_rkn;
    Joint j_lhr, j_lhp, j_lkn;
    Joint j_rsr, j_rsp, j_reb;
    Joint j_lsr, j_lsp, j_leb;

    int n_torso;
    int n_rhr, n_rhp, n_rkn;
    int n_lhr, n_lhp, n_lkn;
    int n_rsr, n_rsp, n_reb;
    int n_lsr, n_lsp, n_leb;
    //Upadting Values//----------------------------------------------
    // Q, Qdot, Qddo
    VectorNd Q ;
    VectorNd Qdot;
    VectorNd Qddot;

    //6D Jacobian, 3D Jacobian
    MatrixNd JacobianRF6D;
    MatrixNd JacobianLF6D;
    MatrixNd JacobianRH6D;
    MatrixNd JacobianLH6D;

    MatrixNd JacobianRF3D;
    MatrixNd JacobianLF3D;
    MatrixNd JacobianRH3D;
    MatrixNd JacobianLH3D;


    //COM Jacobian 6D & 3D
    MatrixNd JacobianCOMtorso6D;

    MatrixNd JacobianCOMrhr6D;
    MatrixNd JacobianCOMrhp6D;
    MatrixNd JacobianCOMrkn6D;

    MatrixNd JacobianCOMlhr6D;
    MatrixNd JacobianCOMlhp6D;
    MatrixNd JacobianCOMlkn6D;

    MatrixNd JacobianCOMrsr6D;
    MatrixNd JacobianCOMrsp6D;
    MatrixNd JacobianCOMreb6D;

    MatrixNd JacobianCOMlsr6D;
    MatrixNd JacobianCOMlsp6D;
    MatrixNd JacobianCOMleb6D;

    MatrixNd JacobianCOMtorso3D;

    MatrixNd JacobianCOMrhr3D;
    MatrixNd JacobianCOMrhp3D;
    MatrixNd JacobianCOMrkn3D;

    MatrixNd JacobianCOMlhr3D;
    MatrixNd JacobianCOMlhp3D;
    MatrixNd JacobianCOMlkn3D;

    MatrixNd JacobianCOMrsr3D;
    MatrixNd JacobianCOMrsp3D;
    MatrixNd JacobianCOMreb3D;

    MatrixNd JacobianCOMlsr3D;
    MatrixNd JacobianCOMlsp3D;
    MatrixNd JacobianCOMleb3D;

    MatrixNd JacobianCOMall6D;
    MatrixNd JacobianCOMall3D;


    MatrixNd ContactJacobian;

    MatrixNd Qu,Qc,R;

    ow_onedofMPC *M1,*M2,*M3;
    int stanceN;
    double flyT;
    vec3 MPC_R,MPC_Qx,MPC_Qv;

    //Externel Force from RightFoot, LeftFoot, RightHand, LeftHand
public:
    VectorNd FextRF;
    VectorNd FextLF;
    VectorNd FextRH;
    VectorNd FextLH;

    VectorNd FextRF_est;
    VectorNd FextLF_est;
    VectorNd FextRH_est;
    VectorNd FextLH_est;

    VectorNd FextRF_est6D;
    VectorNd FextLF_est6D;
    VectorNd FextRH_est6D;
    VectorNd FextLH_est6D;

    VectorNd TextRF;
    VectorNd TextLF;
    VectorNd TextRH;
    VectorNd TextLH;
    VectorNd Text;

    VectorNd Tmotor;

    VectorNd Tau;
    VectorNd TauContact;
    //Mass Weight
    double Mass_Total,Weight_Total;

    //COM information
    Vector3d COM,COMdot;

    //Momentum
    Vector3d Momentum;

    //Potential E, Kinetic E
    Vector3d PE,KE;
    vec3 rotxyzcon;
    MatrixNd err;
private:
    Oinverse Oi;

    Vector3d v2V(vec3 in)
    {
        return Vector3d(in.x,in.y,in.z);
    }
    Matrix3d m2M(mat3 in)
    {
        Matrix3d out;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                out(i,j) = in[i][j];
            }
        }
        return out;
    }
public:
    OW_DYNAMICS()
    {
         Robot = new Model();
         Robot->gravity = Vector3d(0,0,-9.81);



         //BODY_body
         b_torso = Body(Oi.m_torso,v2V(Oi.c_torso),m2M(Oi.I_torso));

         //BODY_legs
         b_rhr = Body(Oi.m_rhr,v2V(Oi.c_rhr),m2M(Oi.I_rhr));
         b_rhp = Body(Oi.m_rhp,v2V(Oi.c_rhp),m2M(Oi.I_rhp));
         b_rkn = Body(Oi.m_rkn,v2V(Oi.c_rkn),m2M(Oi.I_rkn));

         b_lhr = Body(Oi.m_lhr,v2V(Oi.c_lhr),m2M(Oi.I_lhr));
         b_lhp = Body(Oi.m_lhp,v2V(Oi.c_lhp),m2M(Oi.I_lhp));
         b_lkn = Body(Oi.m_lkn,v2V(Oi.c_lkn),m2M(Oi.I_lkn));

         //BODY_arms         
         b_rsr = Body(Oi.m_rsr,v2V(Oi.c_rsr),m2M(Oi.I_rsr));         
         b_rsp = Body(Oi.m_rsp,v2V(Oi.c_rsp),m2M(Oi.I_rsp));
         b_reb = Body(Oi.m_reb,v2V(Oi.c_reb),m2M(Oi.I_reb));

         b_lsr = Body(Oi.m_lsr,v2V(Oi.c_lsr),m2M(Oi.I_lsr));
         b_lsp = Body(Oi.m_lsp,v2V(Oi.c_lsp),m2M(Oi.I_lsp));         
         b_leb = Body(Oi.m_leb,v2V(Oi.c_leb),m2M(Oi.I_leb));

         //JOINT world
         j_world = Joint(JointTypeFloatingBase);


         //JOINT_legs

         j_rhr = Joint(JointTypeRevoluteX);
         j_rhp = Joint(JointTypeRevoluteY);
         j_rkn = Joint(JointTypeRevoluteY);


         j_lhr = Joint(JointTypeRevoluteX);
         j_lhp = Joint(JointTypeRevoluteY);
         j_lkn = Joint(JointTypeRevoluteY);

        //JOINT_arms         
         j_rsr = Joint(JointTypeRevoluteX);
         j_rsp = Joint(JointTypeRevoluteY);
         j_reb = Joint(JointTypeRevoluteY);

         j_lsr = Joint(JointTypeRevoluteX);
         j_lsp = Joint(JointTypeRevoluteY);
         j_leb = Joint(JointTypeRevoluteY);


         Vector3d zv = Vector3d(0.0,0.0,0.0);         
         n_torso = Robot->AddBody(0,Xtrans(zv),j_world,b_torso,"TOR");
         //robot_rl         
         n_rhr = Robot->AddBody(n_torso,Xtrans(v2V(Oi.offset_p2rh)),j_rhr,b_rhr,"RHR");
         n_rhp = Robot->AddBody(n_rhr,Xtrans(v2V(Oi.offset_rh2rp)),j_rhp,b_rhp,"RHP");
         n_rkn = Robot->AddBody(n_rhp,Xtrans(v2V(Oi.offset_uleg)),j_rkn,b_rkn,"RKN");

        //robot_ll         
         n_lhr = Robot->AddBody(n_torso,Xtrans(v2V(Oi.offset_p2lh)),j_lhr,b_lhr,"LHR");
         n_lhp = Robot->AddBody(n_lhr,Xtrans(v2V(Oi.offset_lh2lp)),j_lhp,b_lhp,"LHP");
         n_lkn = Robot->AddBody(n_lhp,Xtrans(v2V(Oi.offset_uleg)),j_lkn,b_lkn,"LKN");

         //robot_ra         
         n_rsr = Robot->AddBody(n_torso,Xtrans(v2V(Oi.offset_p2rs)),j_rsr,b_rsr,"RSR");
         n_rsp = Robot->AddBody(n_rsr,Xtrans(v2V(Oi.offset_rs2rp)),j_rsp,b_rsp,"RSP");
         n_reb = Robot->AddBody(n_rsp,Xtrans(v2V(Oi.offset_uleg)),j_reb,b_reb,"REB");
         //robot_la
         n_lsr = Robot->AddBody(n_torso,Xtrans(v2V(Oi.offset_p2ls)),j_lsr,b_lsr,"LSR");
         n_lsp = Robot->AddBody(n_lsr,Xtrans(v2V(Oi.offset_ls2lp)),j_lsp,b_lsp,"LSP");//strange here
         n_leb = Robot->AddBody(n_lsp,Xtrans(v2V(Oi.offset_uleg)),j_leb,b_leb,"LEB");


         Q = VectorNd::Zero(Robot->q_size);
         Qdot = VectorNd::Zero(Robot->dof_count);
         Qddot = VectorNd::Zero(Robot->dof_count);

         JacobianRF6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianLF6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianLH6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianRH6D = MatrixNd::Zero (6, Robot->dof_count);

         JacobianRF3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianLF3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianRH3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianLH3D = MatrixNd::Zero (3, Robot->dof_count);

         JacobianCOMtorso6D = MatrixNd::Zero (6, Robot->dof_count);

         JacobianCOMrhr6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMrhp6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMrkn6D = MatrixNd::Zero (6, Robot->dof_count);

         JacobianCOMlhr6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMlhp6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMlkn6D = MatrixNd::Zero (6, Robot->dof_count);

         JacobianCOMrsr6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMrsp6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMreb6D = MatrixNd::Zero (6, Robot->dof_count);

         JacobianCOMlsr6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMlsp6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMleb6D = MatrixNd::Zero (6, Robot->dof_count);

         JacobianCOMtorso3D = MatrixNd::Zero (3, Robot->dof_count);

         JacobianCOMrhr3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMrhp3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMrkn3D = MatrixNd::Zero (3, Robot->dof_count);

         JacobianCOMlhr3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMlhp3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMlkn3D = MatrixNd::Zero (3, Robot->dof_count);

         JacobianCOMrsr3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMrsp3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMreb3D = MatrixNd::Zero (3, Robot->dof_count);

         JacobianCOMlsr3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMlsp3D = MatrixNd::Zero (3, Robot->dof_count);
         JacobianCOMleb3D = MatrixNd::Zero (3, Robot->dof_count);

         JacobianCOMall6D = MatrixNd::Zero (6, Robot->dof_count);
         JacobianCOMall3D = MatrixNd::Zero (3, Robot->dof_count);


         FextRF = VectorNd::Zero(6);
         FextLF = VectorNd::Zero(6);
         FextRH = VectorNd::Zero(6);
         FextLH = VectorNd::Zero(6);

         FextRF_est6D = VectorNd::Zero(6);
         FextLF_est6D = VectorNd::Zero(6);
         FextRH_est6D = VectorNd::Zero(6);
         FextLH_est6D = VectorNd::Zero(6);

         FextRF_est = VectorNd::Zero(3);
         FextLF_est = VectorNd::Zero(3);
         FextRH_est = VectorNd::Zero(3);
         FextLH_est = VectorNd::Zero(3);

         TextRF = VectorNd::Zero(6);
         TextLF = VectorNd::Zero(6);
         TextRH = VectorNd::Zero(6);
         TextLH = VectorNd::Zero(6);
         Text = VectorNd::Zero(6);

         Tmotor = VectorNd::Zero(6);
         Tau = VectorNd::Zero(Robot->dof_count);
         TauContact = VectorNd::Zero(Robot->dof_count);


         Mass_Total = Oi.M_total;

         COM = Vector3d::Zero(3);
         COMdot = Vector3d::Zero(3);

         Momentum = Vector3d::Zero(3);

         PE = Vector3d::Zero(3);
         KE = Vector3d::Zero(3);

         M1 = new ow_onedofMPC(15,0.02,300);//
         M2 = new ow_onedofMPC(15,0.02,300);//
         M3 = new ow_onedofMPC(15,0.02,300);//
         flyT = 0.25*0.7;
         MPC_R = vec3(1);
         MPC_Qx = vec3(1e3);
         MPC_Qv = vec3(1e2);
         stanceN = 5;//should be dynamically changed
    }
    VectorNd Joints2Q(QuadJoints QJ)
       {
           VectorNd Q = VectorNd::Zero(Robot->q_size);
           Q[0] = QJ.pPel.x;
           Q[1] = QJ.pPel.y;
           Q[2] = QJ.pPel.z;

           Math::Quaternion QQ(QJ.qPel.x,QJ.qPel.y,QJ.qPel.z,QJ.qPel.w);
           Robot->SetQuaternion(n_torso,QQ,Q);

           Q[6] = QJ.RHR;
           Q[7] = QJ.RHP;
           Q[8] = QJ.RKN;

           Q[9] = QJ.LHR;
           Q[10] = QJ.LHP;
           Q[11] = QJ.LKN;

           Q[12] = QJ.RSR;
           Q[13] = QJ.RSP;
           Q[14] = QJ.REB;

           Q[15] = QJ.LSR;
           Q[16] = QJ.LSP;
           Q[17] = QJ.LEB;

           return Q;
       }

       VectorNd Joints2dQ(QuadJointVels dQJ)
       {
           VectorNd Qdot = VectorNd::Zero(Robot->dof_count);

           Qdot[0] = dQJ.dpPel.x;
           Qdot[1] = dQJ.dpPel.y;
           Qdot[2] = dQJ.dpPel.z;

           //angular velocity
           Qdot[3] = dQJ.dqPel.x;
           Qdot[4] = dQJ.dqPel.y;
           Qdot[5] = dQJ.dqPel.z;


           Qdot[6] = dQJ.RHR;
           Qdot[7] = dQJ.RHP;
           Qdot[8] = dQJ.RKN;

           Qdot[9] = dQJ.LHR;
           Qdot[10] = dQJ.LHP;
           Qdot[11] = dQJ.LKN;

           Qdot[12] = dQJ.RSR;
           Qdot[13] = dQJ.RSP;
           Qdot[14] = dQJ.REB;

           Qdot[15] = dQJ.LSR;
           Qdot[16] = dQJ.LSP;
           Qdot[17] = dQJ.LEB;

           return Qdot;
       }
       void CalcCOMMomentum(VectorNd _Q,VectorNd _Qdot)
         {
             Utils::CalcCenterOfMass(*Robot,_Q,_Qdot,Oi.M_total,COM,&COMdot,&Momentum,true);
         }

         //Jacobian Calculation
         void CalcEndeffectorJacobian6D(VectorNd _Q)
         {
             CalcPointJacobian6D(*Robot,_Q,n_rkn,v2V(Oi.offset_lleg),JacobianRF6D);
             CalcPointJacobian6D(*Robot,_Q,n_lkn,v2V(Oi.offset_lleg),JacobianLF6D);
             CalcPointJacobian6D(*Robot,_Q,n_reb,v2V(Oi.offset_lleg),JacobianRH6D);
             CalcPointJacobian6D(*Robot,_Q,n_leb,v2V(Oi.offset_lleg),JacobianLH6D);
     //        std::cout << "JacobianRF6D \n" << JacobianRF6D << std::endl;
         }
         void CalcEndeffectorJacobian3D(VectorNd _Q)
         {
             CalcPointJacobian(*Robot,_Q,n_lkn,v2V(Oi.offset_lleg),JacobianLF3D);
             CalcPointJacobian(*Robot,_Q,n_reb,v2V(Oi.offset_lleg),JacobianRH3D);
             CalcPointJacobian(*Robot,_Q,n_rkn,v2V(Oi.offset_lleg),JacobianRF3D);
             CalcPointJacobian(*Robot,_Q,n_leb,v2V(Oi.offset_lleg),JacobianLH3D);
     //        std::cout << "JacobianRF3D \n" << JacobianRF3D << std::endl;
         }

         void CalcCOMJacobian6D(VectorNd _Q)
         {
             //get COM 6D jacobian
             CalcPointJacobian6D(*Robot,_Q,n_torso,v2V(Oi.c_torso),JacobianCOMtorso6D);

             CalcPointJacobian6D(*Robot,_Q,n_rhr,v2V(Oi.c_rhr),JacobianCOMrhr6D);
             CalcPointJacobian6D(*Robot,_Q,n_rhp,v2V(Oi.c_rhp),JacobianCOMrhp6D);
             CalcPointJacobian6D(*Robot,_Q,n_rkn,v2V(Oi.c_rkn),JacobianCOMrkn6D);

             CalcPointJacobian6D(*Robot,_Q,n_lhr,v2V(Oi.c_lhr),JacobianCOMlhr6D);
             CalcPointJacobian6D(*Robot,_Q,n_lhp,v2V(Oi.c_lhp),JacobianCOMlhp6D);
             CalcPointJacobian6D(*Robot,_Q,n_lkn,v2V(Oi.c_lkn),JacobianCOMlkn6D);

             CalcPointJacobian6D(*Robot,_Q,n_rsr,v2V(Oi.c_rsr),JacobianCOMrsr6D);
             CalcPointJacobian6D(*Robot,_Q,n_rsp,v2V(Oi.c_rsp),JacobianCOMrsp6D);
             CalcPointJacobian6D(*Robot,_Q,n_reb,v2V(Oi.c_reb),JacobianCOMreb6D);

             CalcPointJacobian6D(*Robot,_Q,n_lsr,v2V(Oi.c_lsr),JacobianCOMlsr6D);
             CalcPointJacobian6D(*Robot,_Q,n_lsp,v2V(Oi.c_lsp),JacobianCOMlsp6D);
             CalcPointJacobian6D(*Robot,_Q,n_leb,v2V(Oi.c_leb),JacobianCOMleb6D);

             JacobianCOMall6D = (Oi.m_torso*JacobianCOMtorso6D + Oi.m_rhr*JacobianCOMrhr6D + Oi.m_rhp*JacobianCOMrhp6D + Oi.m_rkn*JacobianCOMrkn6D + Oi.m_lhr*JacobianCOMlhr6D + Oi.m_lhp*JacobianCOMlhp6D + Oi.m_lkn*JacobianCOMlkn6D + Oi.m_rsr*JacobianCOMrsr6D + Oi.m_rsp*JacobianCOMrsp6D + Oi.m_reb*JacobianCOMreb6D + Oi.m_lsr*JacobianCOMlsr6D + Oi.m_lsp*JacobianCOMlsp6D + Oi.m_leb*JacobianCOMleb6D)/Oi.M_total;
         }

         void CalcCOMJacobian3D(VectorNd _Q)
         {
             //get COM 3D jacobian
             CalcPointJacobian(*Robot,_Q,n_torso,v2V(Oi.c_torso),JacobianCOMtorso3D);

             CalcPointJacobian(*Robot,_Q,n_rhr,v2V(Oi.c_rhr),JacobianCOMrhr3D);
             CalcPointJacobian(*Robot,_Q,n_rhp,v2V(Oi.c_rhp),JacobianCOMrhp3D);
             CalcPointJacobian(*Robot,_Q,n_rkn,v2V(Oi.c_rkn),JacobianCOMrkn3D);

             CalcPointJacobian(*Robot,_Q,n_lhr,v2V(Oi.c_lhr),JacobianCOMlhr3D);
             CalcPointJacobian(*Robot,_Q,n_lhp,v2V(Oi.c_lhp),JacobianCOMlhp3D);
             CalcPointJacobian(*Robot,_Q,n_lkn,v2V(Oi.c_lkn),JacobianCOMlkn3D);

             CalcPointJacobian(*Robot,_Q,n_rsr,v2V(Oi.c_rsr),JacobianCOMrsr3D);
             CalcPointJacobian(*Robot,_Q,n_rsp,v2V(Oi.c_rsp),JacobianCOMrsp3D);
             CalcPointJacobian(*Robot,_Q,n_reb,v2V(Oi.c_reb),JacobianCOMreb3D);

             CalcPointJacobian(*Robot,_Q,n_lsr,v2V(Oi.c_lsr),JacobianCOMlsr3D);
             CalcPointJacobian(*Robot,_Q,n_lsp,v2V(Oi.c_lsp),JacobianCOMlsp3D);
             CalcPointJacobian(*Robot,_Q,n_leb,v2V(Oi.c_leb),JacobianCOMleb3D);

             JacobianCOMall3D = (Oi.m_torso*JacobianCOMtorso3D + Oi.m_rhr*JacobianCOMrhr3D + Oi.m_rhp*JacobianCOMrhp3D + Oi.m_rkn*JacobianCOMrkn3D + Oi.m_lhr*JacobianCOMlhr3D + Oi.m_lhp*JacobianCOMlhp3D + Oi.m_lkn*JacobianCOMlkn3D + Oi.m_rsr*JacobianCOMrsr3D + Oi.m_rsp*JacobianCOMrsp3D + Oi.m_reb*JacobianCOMreb3D + Oi.m_lsr*JacobianCOMlsr3D + Oi.m_lsp*JacobianCOMlsp3D + Oi.m_leb*JacobianCOMleb3D)/Oi.M_total;

         }
         //Dynamics
         void CalcForwardDynamics(VectorNd _Q,VectorNd _Qdot,VectorNd _Tau)
         {
             VectorNd _Qddot = VectorNd::Zero(Robot->dof_count);
             ForwardDynamics(*Robot,_Q,_Qdot,_Tau,_Qddot);
             Qddot = _Qddot;
         }
         void CalcInverseDynamics(VectorNd _Q,VectorNd _Qdot,VectorNd _Qddot)
         {
             VectorNd _Tau = VectorNd::Zero(Robot->dof_count);
             InverseDynamics(*Robot,_Q,_Qdot,_Qddot,_Tau);
             Tau = _Tau;
         }
         void GetFext2Text(VectorNd _Q,VectorNd _FextRF,VectorNd _FextLF,VectorNd _FextRH,VectorNd _FextLH)
         {
             CalcEndeffectorJacobian6D(_Q);
             TextRF = JacobianRF6D.transpose()*_FextRF;
             TextLF = JacobianLF6D.transpose()*_FextLF;
             TextRH = JacobianRH6D.transpose()*_FextRH;
             TextLH = JacobianLH6D.transpose()*_FextLH;
             Text = TextRF + TextLF + TextRH + TextLH;
         }
         void GetMotorTorque(VectorNd _Q,VectorNd _Qdot,VectorNd _Qddot,VectorNd _Text)
         {
             VectorNd _Tau = VectorNd::Zero(Robot->dof_count);
             InverseDynamics(*Robot,_Q,_Qdot,_Qddot,_Tau);
             Tmotor = _Tau-_Text;
         }
         void MakeContactJacobian(bool Contact_RF,bool Contact_LF,bool Contact_RH,bool Contact_LH)
            {
                int _CN = 0;
                int _order = 0;

                int _RF = 0;
                int _LF = 0;
                int _RH = 0;
                int _LH = 0;

                if (Contact_RF==true){
                    _CN = _CN+3;
                    _order++;
                    _RF = _order;
                }
                if (Contact_LF==true){
                    _CN = _CN+3;
                    _order++;
                    _LF = _order;
                }
                if (Contact_RH==true){
                    _CN = _CN+3;
                    _order++;
                    _RH = _order;
                }
                if (Contact_LH==true){
                    _CN = _CN+3;
                    _order++;
                    _LH = _order;
                }

                ContactJacobian = MatrixNd::Zero (_CN, Robot->dof_count);

                if (_RF == 1) ContactJacobian.block(0,0,3,Robot->dof_count) = JacobianRF3D;
                if (_LF == 1) ContactJacobian.block(0,0,3,Robot->dof_count) = JacobianLF3D;
                if (_RH == 1) ContactJacobian.block(0,0,3,Robot->dof_count) = JacobianRH3D;
                if (_LH == 1) ContactJacobian.block(0,0,3,Robot->dof_count) = JacobianLH3D;

                if (_RF == 2) ContactJacobian.block(3,0,3,Robot->dof_count) = JacobianRF3D;
                if (_LF == 2) ContactJacobian.block(3,0,3,Robot->dof_count) = JacobianLF3D;
                if (_RH == 2) ContactJacobian.block(3,0,3,Robot->dof_count) = JacobianRH3D;
                if (_LH == 2) ContactJacobian.block(3,0,3,Robot->dof_count) = JacobianLH3D;

                if (_RF == 3) ContactJacobian.block(6,0,3,Robot->dof_count) = JacobianRF3D;
                if (_LF == 3) ContactJacobian.block(6,0,3,Robot->dof_count) = JacobianLF3D;
                if (_RH == 3) ContactJacobian.block(6,0,3,Robot->dof_count) = JacobianRH3D;
                if (_LH == 3) ContactJacobian.block(6,0,3,Robot->dof_count) = JacobianLH3D;

                if (_RF == 4) ContactJacobian.block(9,0,3,Robot->dof_count) = JacobianRF3D;
                if (_LF == 4) ContactJacobian.block(9,0,3,Robot->dof_count) = JacobianLF3D;
                if (_RH == 4) ContactJacobian.block(9,0,3,Robot->dof_count) = JacobianRH3D;
                if (_LH == 4) ContactJacobian.block(9,0,3,Robot->dof_count) = JacobianLH3D;

            }

            void CalcEndeffectorForce(VectorNd _Q,VectorNd _Qdot,VectorNd _Tau,bool Contact_RF,bool Contact_LF,bool Contact_RH,bool Contact_LH)
            {
                MatrixNd _M = MatrixNd::Zero (Robot->dof_count, Robot->dof_count);
                MatrixNd _Minv = MatrixNd::Zero (Robot->dof_count, Robot->dof_count);
                CompositeRigidBodyAlgorithm(*Robot,_Q,_M,true);
                _Minv = _M.inverse();

                VectorNd Nonlin = VectorNd::Zero (Robot->dof_count);
                NonlinearEffects(*Robot,_Q,_Qdot,Nonlin);

                VectorNd ContactFs = (ContactJacobian*_Minv*ContactJacobian.transpose()).inverse()*(ContactJacobian*_Minv*(-_Tau+Nonlin));//no dynamic effect

                int _CN =0;

                if (Contact_RF==true)
                {
                    FextRF_est[0] = ContactFs[_CN];
                    FextRF_est[1] = ContactFs[_CN+1];
                    FextRF_est[2] = ContactFs[_CN+2];
                    _CN = _CN+3;
                }
                if (Contact_LF==true){
                    FextLF_est[0] = ContactFs[_CN];
                    FextLF_est[1] = ContactFs[_CN+1];
                    FextLF_est[2] = ContactFs[_CN+2];
                    _CN = _CN+3;
                }
                if (Contact_RH==true){
                    FextRH_est[0] = ContactFs[_CN];
                    FextRH_est[1] = ContactFs[_CN+1];
                    FextRH_est[2] = ContactFs[_CN+2];
                    _CN = _CN+3;
                }
                if (Contact_LH==true){
                    FextLH_est[0] = ContactFs[_CN];
                    FextLH_est[1] = ContactFs[_CN+1];
                    FextLH_est[2] = ContactFs[_CN+2];
                    _CN = _CN+3;
                }

            }
            vec3 calc_ddcom_pinv(VectorNd _Q, VectorNd _dQ, QuadPos QPref, bool contacts[])//QPref for ddXs
            {

                MatrixNd _M = MatrixNd::Zero (Robot->dof_count, Robot->dof_count);
                CompositeRigidBodyAlgorithm(*Robot,_Q,_M,true);
                VectorNd Nonlin = VectorNd::Zero (Robot->dof_count);
                NonlinearEffects(*Robot,_Q,_dQ,Nonlin);
                CalcEndeffectorJacobian3D(_Q);
                bool cRF = contacts[0];
                bool cLF = contacts[1];
                bool cRH = contacts[2];
                bool cLH = contacts[3];
                MakeContactJacobian(cRF,cLF,cRH,cLH);
                VectorNd LegSpeed = ContactJacobian*_dQ;
                _dQ[0] += -LegSpeed[0];
                _dQ[1] += -LegSpeed[1];
                _dQ[2] += -LegSpeed[2];//ok??
                _dQ[3] = QPref.dqPel.x;
                _dQ[4] = QPref.dqPel.y;
                _dQ[5] = QPref.dqPel.z;

                LegSpeed = ContactJacobian*_dQ;

//                cout<<"LEGSPEEDTEST "<<endl;
//                cout<<LegSpeed<<endl;
//                exit(0);


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


                MatrixNd A = MatrixNd::Zero(12+18+n_con*3,12+18+n_con*3);
                MatrixNd B = MatrixNd::Zero(12+18+n_con*3,1);

                MatrixNd Aeq = MatrixNd::Zero(0,12+18+n_con*3);
                MatrixNd Beq = MatrixNd::Zero(0,1);
                //1. Dynamics
                A.block(0,0,_M.rows(),_M.cols()) = _M;
                A.block(6,18,12,12) = -MatrixNd::Identity(12,12);//Tau
                A.block(0,18+12,ContactJacobian.cols(),ContactJacobian.rows())
                        = -ContactJacobian.transpose();//force
                B.block(0,0,Nonlin.rows(),Nonlin.cols()) = -Nonlin;//nonlin

//                MatrixNd Aeq = MatrixNd::Zero(18,12+18+n_con*3);
//                MatrixNd Beq = MatrixNd::Zero(18,1);
//                //1. Dynamics
//                Aeq.block(0,0,_M.rows(),_M.cols()) = _M;
//                Aeq.block(6,18,12,12) = -MatrixNd::Identity(12,12);//Tau
//                Aeq.block(0,18+12,ContactJacobian.cols(),ContactJacobian.rows())
//                        = -ContactJacobian.transpose();//force
//                Beq.block(0,0,Nonlin.rows(),Nonlin.cols()) = -Nonlin;//nonlin

                //2. qPel
                A(18,3) = 1; B(18,0) = 0;//ddqx = 0;
                A(19,4) = 1; B(19,0) = 0;//ddqy = 0;
                A(20,5) = 1; B(20,0) = 0;//ddqz = 0;


                //3. comz
                for(int i=0;i<n_con;i++)
                {
                    A(21,18+12+i*3+2) = 1;
                }
                B(21,0) = Mass_Total*9.81;//+?-?

                //4. swingleg/stanceleg
                MakeContactJacobian(true,true,true,true);
                ddx_djdq = MatrixNd::Zero(ContactJacobian.rows(),1);
                {ddx_djdq.block(0*3,0,3,1) = -RFdJdQ;}
                {ddx_djdq.block(1*3,0,3,1) = -LFdJdQ;}
                {ddx_djdq.block(2*3,0,3,1) = -RHdJdQ;}
                {ddx_djdq.block(3*3,0,3,1) = -LHdJdQ;}
                A.block(22,0,ContactJacobian.rows(),ContactJacobian.cols()) = ContactJacobian;
                B.block(22,0,ddx_djdq.rows(),ddx_djdq.cols()) = ddx_djdq;
                for(int i=0;i<3;i++)
                {
                    B(22+i,0) += QPref.ddRF[i];
                    B(25+i,0) += QPref.ddLF[i];
                    B(28+i,0) += QPref.ddRH[i];
                    B(31+i,0) += QPref.ddLH[i];
                }
                //5. ZMP center                
                int Findex, Hindex;
                Findex = 0; Hindex = 1;//for safety
                if(n_con==2)
                {
                    Findex = 0; Hindex = 1;
                }
                else
                {
                    if(cRF&&cLH)
                    {
                        if(cLF)
                        {
                            Findex = 0; Hindex = 2;
                        }
                        else
                        {
                            Findex = 0; Hindex = 1;
                        }
                    }
                    if(cLF&&cRH)
                    {
                        if(cRF)
                        {
                            Findex = 1; Hindex = 2;
                        }
                        else
                        {
                            Findex = 0; Hindex = 1;
                        }
                    }
                }
                A(33,18+12+Findex*3+2) = 1;
                A(33,18+12+Hindex*3+2) = -1;
                B(33,0) = 0;//fz1 = fz2;
                if(n_con==3)//additional foot contactforce zero
                {
                    B(34,0) = 0;
                    if(!cRF){ A(34,18+12+2*3+2)=1;}
                    if(!cLF){ A(34,18+12+2*3+2)=1;}
                    if(!cRH){ A(34,18+12+1*3+2)=1;}
                    if(!cLH){ A(34,18+12+0*3+2)=1;}
                }

                VectorNd X = OC.calcX3(A,B,Aeq,Beq);
                //VectorNd X = OC.calcX4(A,B);


                err = A*X-B;

                VectorNd Fs = X.segment(18+12,n_con*3);
                vec3 FCOM = vec3();
                for(int i=0;i<n_con;i++)
                {
                    FCOM[0]+=Fs[i*3+0];
                    FCOM[1]+=Fs[i*3+1];
                    FCOM[2]+=Fs[i*3+2];
                }

                vec3 ddcomout = FCOM/Mass_Total;
                ddcomout.z-=9.81;
//                ddcomout[0] = X[0];//ddppel test
//                ddcomout[1] = X[1];
//                ddcomout[2] = X[2];
                return ddcomout;
            }
            VectorNd calc_QP(VectorNd _Q, VectorNd _Qref, VectorNd _dQ, VectorNd _dQref, VectorNd _ddQref, bool cRF, bool cLF, bool cRH, bool cLH,int feedback_pose = XYPOS, vec3 sV = vec3(), QP_wnzW0s ws = QP_wnzW0s());



};

#endif // OW_RBDL_H
