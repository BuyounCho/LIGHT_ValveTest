/*============================================================================
 *
 *  <light hopping operational function>
 *
 * Input : Robot States
 * Output : Object Matrix(A&b in 1/2|Ax-b|^2) or Constant Matrix (A&b in Ax=b)
 *          for several tasks\
 *
 *                -  Buyoun,Cho 2018.05.08
 *
=============================================================================*/

#include "LIGHT_commands.h"
#include "LIGHT_var_and_func.h"
#include "LIGHT_robotmodel.h"
#include "ManualCAN.h"

// New operational function for "LIGHT" - 2018.05.01
void LIGHT_JUMPING();

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
extern pUSER_SHM               userData;
extern JointControlClass       *joint;
extern JointControlClass       *jCon;
extern int                     PODO_NO;


//==================================================//
// Equation of Motion (for jumping with linear guide)
// EoM is Fundamental equation for the system
// This equation is based on physical phenomenom.
// So, the solution must satisfy EoM  >>>  "Equality Constraint"
//
//
//   [M -S J^T]*[ddq tau F_ext]^T = -N
//
//  input : robot model and robot state (q,dq)
//  output : equality constraint matrix (A,B)
//==================================================//
void EOM_LMGuide(LIGHTLeg & _Leg,MatrixNd & _Q, MatrixNd & _dQ, MatrixNd & _A, MatrixNd & _B)
{
    int n_dof_act = _Leg.n_DOF;
    int n_contact = 1;

    // Make mass matrix (M)
    MatrixNd tempM = MatrixNd::Zero (n_dof_act, n_dof_act);
    CompositeRigidBodyAlgorithm(*_Leg.LIGHTLegModel,_Q,tempM,true);

    // Make nonlinear matrix (N)
    VectorNd tempN = VectorNd::Zero(n_dof_act,1);
    NonlinearEffects(*_Leg.LIGHTLegModel, _Q, _dQ, tempN);

    // Make Selection matrix (S)
    MatrixNd tempS = MatrixNd::Identity(n_dof_act,n_dof_act);

    // Make Contact Jacobian and Transpose (J and J^T)
    MatrixNd tempJ = _Leg.update_FootJacobian(_Leg.LIGHTLegModel,_Q);
    MatrixNd tempJT = tempJ.transpose();

    // A = [M -S JT]
    // M : n_dof_act*n_dof_act
    // S : n_dof_act*n_dof_act
    // JT : n_dof_act*(3*contact_point)
    MatrixNd tempA =  MatrixNd::Zero (n_dof_act,2*n_dof_act+3*n_contact);
    tempA.block(0,0,n_dof_act, n_dof_act) = tempM;
    tempA.block(0,n_dof_act,n_dof_act, n_dof_act) = tempS;
    tempA.block(0,2*n_dof_act,n_dof_act, 3*contact_point) = tempJT;
    _A = tempA;
    _B = -tempN;

}

//==============================//
// Landing motion
//==============================//



//==============================//
// Jumping motion
//==============================//
void LIGHT_JUMPING(LIGHTLeg & _LegModel,MatrixNd & _Q, MatrixNd & _dQ,MatrixNd & _A, MatrixNd & _b)
{



}



//==============================//
// Sit & Stand motion
//==============================//


