/*============================================================================
 *
 *  <light hopping Kinematics>
 *
 *  On this file, there are functions to find optimal joint velocity with some constraints.
 *  (Kinematics)
 *
 *  (Inequality constraints)
 *   - Joint angle limit and velocity limit
 *
 *  (Object function)
 *   - Cartesian(Operational) space control
 *   - joint space control (= direct tracking)
 *
 *
 * Input : Robot States (q,dq, Robot model... something like that)
 * Output : Object Matrix(A&b in 1/2|Ax-b|^2) or Constant Matrix (A&b in Ax=b)
 *          for kinematics level (without force term)
 *
 *                -  Buyoun,Cho 2018.05.08
 *
=============================================================================*/

#ifndef LIGHT_HOPPING_KINEMATICS
#define LIGHT_HOPPING_KINEMATICS

#include "LIGHT_robotmodel.h"

extern LIGHTWholeBody LIGHT;

class LIGHT_InvKinematics_INFO {
public:
    // Cost Function Info(weight)
    enum KinematicsWeightIdx {
        W_PEL_Ori = 0,
        W_PEL_Pos,

        W_RF_Ori,
        W_LF_Ori,
        W_RF_Pos,
        W_LF_Pos,

        W_Pel2RF_Ori,
        W_Pel2LF_Ori,
        W_Pel2RF_Pos,
        W_Pel2LF_Pos,

        W_RF2CoM_Pos,     // reference frame : global, not right foot
        W_RF2LF_Pos,  // reference frame : global, not right foot

        W_LF2CoM_Pos,     // reference frame : global, not right foot
        W_LF2RF_Pos,  // reference frame : global, not right foot

        W_RF_dQ,
        W_LF_dQ,
        W_WST_dQ,

        W_YawMomentum_Minimize,
        W_Change_Minimize,

        W_NUM
    };
    double Weight[W_NUM];

public:
    LIGHT_InvKinematics_INFO() {
        for(int i=0;i<W_NUM;i++){
            Weight[i] = 0.0;
        }
    }

    void Reset() {
        for(int i=0;i<W_NUM;i++){
            Weight[i] = 0.0;
            //            std::cout << "Weight " << i << " : " <<  Weight[i] << std::endl;
        }
    }

    bool CheckWeightZero() {
        for(int i=0;i<W_NUM;i++){
            if(Weight[i] != 0) {
                return false;
            }
        } return true;
    }

    void WeightSet_RFBaseMotion() {
        Reset();
//        Weight[W_Change_Minimize] = 0.1;
        Weight[W_PEL_Ori] = 1.0;
        Weight[W_RF_Ori] = 1.0;
        Weight[W_LF_Ori] = 1.0;
        Weight[W_RF_Pos] = 1.0;
        Weight[W_RF2CoM_Pos] = 1.0;
        Weight[W_RF2LF_Pos] = 1.0;
        Weight[W_WST_dQ] = 0.2;
        Weight[W_YawMomentum_Minimize] = 0.1;
    }

    void WeightSet_LFBaseMotion() {
        Reset();
//        Weight[W_Change_Minimize] = 0.1;
        Weight[W_PEL_Ori] = 1.0;
        Weight[W_RF_Ori] = 1.0;
        Weight[W_LF_Ori] = 1.0;
        Weight[W_LF_Pos] = 1.0;
        Weight[W_LF2CoM_Pos] = 1.0;
        Weight[W_LF2RF_Pos] = 1.0;
        Weight[W_WST_dQ] = 0.2;
        Weight[W_YawMomentum_Minimize] = 0.1;
    }

    void WeightSet_PelBaseMotion() {
        Reset();
//        Weight[W_Change_Minimize] = 0.1;
        Weight[W_PEL_Ori] = 1.0;
        Weight[W_PEL_Pos] = 1.0;
        Weight[W_Pel2RF_Ori] = 1.0;
        Weight[W_Pel2LF_Ori] = 1.0;
        Weight[W_Pel2RF_Pos] = 1.0;
        Weight[W_Pel2LF_Pos] = 1.0;
        Weight[W_WST_dQ] = 0.2;
        Weight[W_YawMomentum_Minimize] = 0.1;
    }

    void WeightSet_JointMotion() {
        Reset();
//        Weight[W_Change_Minimize] = 0.1;
        Weight[W_PEL_Ori] = 1.0;
        Weight[W_PEL_Pos] = 1.0;
        Weight[W_RF_dQ] = 1.0;
        Weight[W_LF_dQ] = 1.0;
        Weight[W_WST_dQ] = 0.2;
        Weight[W_YawMomentum_Minimize] = 0.1;
    }
};

class LIGHT_InvKinematics_SUB_INFO  // FixedBase Inverse Kinematics
{
public:
    // Cost Function Info(weight)
    enum KinematicsWeightIdx {
        W_Pel2RF_Ang,
        W_Pel2LF_Ang,
        W_Pel2RF_Pos,
        W_Pel2LF_Pos,

        W_RF2CoM_Pos,     // reference frame : global, not right foot
        W_RF2LF_Pos,  // reference frame : global, not right foot
        W_LF2CoM_Pos,     // reference frame : global, not right foot
        W_LF2RF_Pos,  // reference frame : global, not right foot

        W_Vel_Minimize,

        W_NUM
    };
    double Weight[W_NUM];

public:
    LIGHT_InvKinematics_SUB_INFO() {
        for(int i=0;i<W_NUM;i++){
            Weight[i] = 0.0;
        }
    }

    void Reset() {
        for(int i=0;i<W_NUM;i++){
            Weight[i] = 0.0;
            //            std::cout << "Weight " << i << " : " <<  Weight[i] << std::endl;
        }
    }

    bool CheckWeightZero() {
        for(int i=0;i<W_NUM;i++){
            if(Weight[i] != 0) {
                return false;
            }
        } return true;
    }

    void WeightSet_LFBaseMotion() {
        Reset();
        Weight[W_Vel_Minimize] = 0.00001;
        Weight[W_Pel2RF_Ang] = 1.0;
        Weight[W_Pel2LF_Ang] = 1.0;
        Weight[W_LF2CoM_Pos] = 1.0;
        Weight[W_LF2RF_Pos] = 1.0;
    }

    void WeightSet_RFBaseMotion() {
        Reset();
        Weight[W_Vel_Minimize] = 0.00001;
        Weight[W_Pel2RF_Ang] = 1.0;
        Weight[W_Pel2LF_Ang] = 1.0;
        Weight[W_RF2CoM_Pos] = 1.0;
        Weight[W_RF2LF_Pos] = 1.0;
    }

    void WeightSet_PelBaseMotion() {
        Reset();
        Weight[W_Vel_Minimize] = 1.0;
        Weight[W_Pel2RF_Ang] = 1.0;
        Weight[W_Pel2LF_Ang] = 1.0;
        Weight[W_Pel2RF_Pos] = 1.0;
        Weight[W_Pel2LF_Pos] = 1.0;
    }
};


#endif // LIGHT_HOPPING_KINEMATICS

