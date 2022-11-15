#ifndef _LIGHT_JOINTSETMODEL_H
#define _LIGHT_JOINTSETMODEL_H

#include "rbdl/rbdl.h"
#include "../LIGHTWalking/LIGHT_info.h"
#include <string>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class LIGHTJointSet
{
private:
    // All generalized coordinates of whole body [base pos & ori(6)+ joint angle(n)]

public:
    Model* Robot;
    INFO_LIGHT* LIGHT_Info;

    bool Flag_Error;
    int n_dof;

    ///// state and reference ///////////////////////////////////////////////////////////////

    // Joint State
    MatrixNd Qnow;  // position
    MatrixNd dQnow; // velocity
    MatrixNd Tnow;  // torque

    VectorNd Qref;
    VectorNd dQref;
    VectorNd ddQref;
    MatrixNd Tref;

    double dQref_CompMx_RANK;
    double dQref_CompMy_RANK;
    double dQref_CompMx_LANK;
    double dQref_CompMy_LANK;
    double Tref_CompMx_RANK;
    double Tref_CompMy_RANK;
    double Tref_CompMx_LANK;
    double Tref_CompMy_LANK;

    // Actuator State
    MatrixNd Snow;  // position
    MatrixNd dSnow; // velocity
    MatrixNd Fnow;  // force

    MatrixNd Sref;
    MatrixNd dSref;
    MatrixNd Fref;

public:

    LIGHTJointSet()
    {
        LIGHT_Info = new INFO_LIGHT();
        Robot = new Model();
        n_dof = 13;
        Flag_Error = false;

        // //////////////////////////////////////////////////////////
        // Robot states
        // //////////////////////////////////////////////////////////

        Qnow = VectorNd::Zero(n_dof);
        dQnow = VectorNd::Zero(n_dof);
        Tnow = VectorNd::Zero(n_dof);

        Qref = VectorNd::Zero(n_dof);
        dQref = VectorNd::Zero(n_dof);
        ddQref = VectorNd::Zero(n_dof);
        Tref = VectorNd::Zero(n_dof);

    }

    ///// state update
    void UpdateJointPosition();

    void SetReference_Joint(bool UpdateEnable);
    void SetReference_Actuator(bool UpdateEnable);

    ///// Joint State >> Actuator(Cylinder or Rotary) State ////////////////////////////
    ///// For reference generation
    void    Joint2Actuator_HipYaw(double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_HipRoll(double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_HipPitch(double theta_m, double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_Knee(double theta_m, double theta, double dtheta, double T, double &S, double &dS, double &F);
    void    Joint2Actuator_Ankle(double theta_m, double theta, double dtheta, double Tp,
                                 double phi_m, double phi, double dphi, double Tr,
                                 VectorNd &S, VectorNd &dS, VectorNd &F);
    void    Joint2Actuator_WaistYaw(double theta, double dtheta, double T, double &S, double &dS, double &F);

    ///// Joint Control Function ////////////////////////////////////////////////////////
    bool ValveOpenCtrl_SineWave();
    bool ValveOpenCtrl_Jump();

    bool PosCtrl_HomePose();
    bool PosCtrl_WalkReadyPose();
    bool PosCtrl_GOTO_Pose();
    bool PosCtrl_Random_Motion();
    bool PosCtrl_Sine_Motion();

    bool TorCtrl_ConstantTorque(double DesiredTorque, double Kp_Comp, double Ki_Comp, bool _StopFlag);

    bool Variable_SupplyPressure_Test(bool _ONOFF,
                                      double _MAG, double _PER, int _N,
                                      double _LOAD);

    string ValveId_Date;
    int ValveId_Direction;
    int ValveId_OpenMin;
    int ValveId_OpenResol;
    int ValveId_OpenMax;
    double ValveId_PressureMin;
    double ValveId_PressureResol;
    double ValveId_PressureMax;
    void ValveIdentification_ParameterSet(string _DATE, int _DIR, int _OPEN_MIN, int _OPEN_RESOL, int _OPEN_MAX, double _Ps_MIN, double _Ps_RESOL, double _Ps_MAX);
    bool ValveIdentification_VariablePressure(bool _ONOFF);
    bool ValveIdentification_Positive(int Ps, bool _ONOFF);
    bool ValveIdentification_Negative(int Ps, bool _ONOFF);
};

#endif // _LIGHT_JOINTSETMODEL_H

