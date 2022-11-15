#ifndef BASICJOINT_H
#define BASICJOINT_H

#include <iostream>
#include <math.h>

#include "RBSharedMemory.h"
#include "JointInformation.h"
#include "RBLog.h"

class JointClass;
typedef QVector<JointClass*> JointVector;

// enum variables
enum ErrCode{
    ERR_OK = 0,
    ERR_GOAL_TIME,
    ERR_ALREADY_MOVING,
    ERR_WRONG_MODE,
    ERR_WRONG_SELECTION
};
enum MovingStatus{
    MOVE_DONE = 0,
    STILL_MOVING
};
enum MoveCommandMode{
    MOVE_RELATIVE = 0,
    MOVE_ABSOLUTE
};


class JointClass
{
public:
    int         JNum;
    int         MCId;
    int         MCCh;
    double      RefAngleCurrent;
    double      RefVelocityCurrent;
    double      RefLinPosCurrent;
    double      RefLinVelCurrent;
    double      RefTorqueCurrent;
    double      RefLinForceCurrent;
    double      RefPressureCurrent;
    double      RefPWMCurrent;
    double      RefValveCurrent;

public:
    JointClass(){
        RefAngleCurrent = 0.f;
        RefVelocityCurrent = 0.f;
        RefLinPosCurrent = 0.f;
        RefLinVelCurrent = 0.f;
        RefTorqueCurrent = 0.f;
        RefLinForceCurrent= 0.f;
        RefPressureCurrent= 0.f;
        RefPWMCurrent= 0.f;
        RefValveCurrent= 0.f;
        MoveFlag = false;
    }
    JointClass(const int jnum, const int id, const int ch){
        JNum = jnum; MCId = id; MCCh = ch;
        RefAngleCurrent = 0.f;
        RefVelocityCurrent = 0.f;
        RefLinPosCurrent = 0.f;
        RefLinVelCurrent = 0.f;
        RefTorqueCurrent = 0.f;
        RefLinForceCurrent= 0.f;
        RefPressureCurrent= 0.f;
        RefPWMCurrent= 0.f;
        RefValveCurrent= 0.f;
        MoveFlag = false;
    }

    // For LIGHT (by BUYOUN)
    void    SetRefAngleCurrent(const double ref)	{RefAngleCurrent = ref;}
    double  GetRefAngleCurrent()					{return RefAngleCurrent;}

    void    SetRefVelocityCurrent(const double ref)	{RefVelocityCurrent = ref;}
    double  GetRefVelocityCurrent()					{return RefVelocityCurrent;}

    void    SetRefLinPosCurrent(const double ref)	{RefLinPosCurrent = ref;}
    double  GetRefLinPosCurrent()					{return RefLinPosCurrent;}

    void    SetRefLinVelCurrent(const double ref)	{RefLinVelCurrent = ref;}
    double  GetRefLinVelCurrent()					{return RefLinVelCurrent;}

    void    SetRefTorqueCurrent(const double ref)	{RefTorqueCurrent = ref;}
    double  GetRefTorqueCurrent()					{return RefTorqueCurrent;}

    void    SetRefLinForceCurrent(const double ref)	{RefLinForceCurrent = ref;}
    double  GetRefLinForceCurrent()					{return RefLinForceCurrent;}

    void    SetRefPressureCurrent(const double ref)	{RefPressureCurrent = ref;}
    double  GetRefPressureCurrent()					{return RefPressureCurrent;}

    void    SetRefPWMCurrent(const double ref)	{RefPWMCurrent = ref;}
    double  GetRefPWMCurrent()					{return RefPWMCurrent;}

    void    SetRefValveCurrent(const double ref)	{RefValveCurrent = ref;}
    double  GetRefValveCurrent()					{return RefValveCurrent;}


    void    SetMoveFlag(unsigned char flag)         {MoveFlag = flag;}

    char    SetMoveJoint(const double _angle, const double _msTime, const unsigned int _mode){
        if(_msTime <= 0){
            FILE_LOG(logWARNING) << "Goal time must be greater than zero(SetMoveJoint)[JNum: " << JNum << "]"; return ERR_GOAL_TIME;
        }

        MoveFlag = false;
        switch(_mode)
        {
        case MOVE_RELATIVE:	// relative mode
            RefAngleToGo = RefAngleCurrent + _angle;
            break;
        case MOVE_ABSOLUTE:	// absolute mode
            RefAngleToGo = _angle;
            break;
        default:
            FILE_LOG(logWARNING) << "Wrong reference mode(SetMoveJoint)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefAngleInitial = RefAngleCurrent;
        RefAngleDelta = RefAngleToGo - RefAngleCurrent;
        CurrentTimeCount = 0;

        GoalTimeCount = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
        MoveFlag = true;
        return ERR_OK;
    }
    char    MoveJoint(){
        if(MoveFlag == true){
            CurrentTimeCount++;
            if(GoalTimeCount <= CurrentTimeCount){
                GoalTimeCount = CurrentTimeCount = 0;
                RefAngleCurrent = RefAngleToGo;
                MoveFlag = false;
                return MOVE_DONE;
            }else{
                RefAngleCurrent = RefAngleInitial+RefAngleDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount*(double)CurrentTimeCount));
            }
        }
        return STILL_MOVING;
    }

private:
    double			RefAngleDelta;
    double			RefAngleToGo;
    double			RefAngleInitial;
    unsigned long	GoalTimeCount;
    unsigned long	CurrentTimeCount;
    unsigned char	MoveFlag;
};



class JointControlClass
{
public:

    explicit JointControlClass(pRBCORE_SHM_REFERENCE _shm_ref, pRBCORE_SHM_SENSOR _shm_sen, pRBCORE_SHM_COMMAND _shm_com, int _podoNum){
        Shm_ref = _shm_ref;
        Shm_sen = _shm_sen;
        Shm_com = _shm_com;
        PODONum = _podoNum;
        Joints = JointVector(NO_OF_JOINTS);
        for(int i=0; i<NO_OF_JOINTS; i++)
            Joints[i] = new JointClass(i, MC_GetID(i), MC_GetCH(i));
    }

    JointVector		Joints;

    // For LIGHT (by BUYOUN)
    double  GetJointRefAngle(const int n)						{return Joints[n]->GetRefAngleCurrent();}
    void    SetJointRefAngle(const int n, const double _ref)	{Joints[n]->SetRefAngleCurrent(_ref);}

    double  GetJointRefVelocity(const int n)                    {return Joints[n]->GetRefVelocityCurrent();}
    void    SetJointRefVelocity(const int n, const double _ref)	{Joints[n]->SetRefVelocityCurrent(_ref);}

    double  GetJointRefLinPos(const int n)						{return Joints[n]->GetRefLinPosCurrent();}
    void    SetJointRefLinPos(const int n, const double _ref)	{Joints[n]->SetRefLinPosCurrent(_ref);}

    double  GetJointRefLinVel(const int n)						{return Joints[n]->GetRefLinVelCurrent();}
    void    SetJointRefLinVel(const int n, const double _ref)	{Joints[n]->SetRefLinVelCurrent(_ref);}

    double  GetJointRefTorque(const int n)						{return Joints[n]->GetRefTorqueCurrent();}
    void    SetJointRefTorque(const int n, const double _ref)	{Joints[n]->SetRefTorqueCurrent(_ref);}

    double  GetJointRefLinForce(const int n)					{return Joints[n]->GetRefLinForceCurrent();}
    void    SetJointRefLinForce(const int n, const double _ref)	{Joints[n]->SetRefLinForceCurrent(_ref);}

    double  GetJointRefPressure(const int n)						{return Joints[n]->GetRefPressureCurrent();}
    void    SetJointRefPressure(const int n, const double _ref)	{Joints[n]->SetRefPressureCurrent(_ref);}

    double  GetJointRefPWM(const int n)						{return Joints[n]->GetRefPWMCurrent();}
    void    SetJointRefPWM(const int n, const double _ref)	{Joints[n]->SetRefPWMCurrent(_ref);}

    double  GetJointRefValve(const int n)						{return Joints[n]->GetRefValveCurrent();}
    void    SetJointRefValve(const int n, const double _ref)	{Joints[n]->SetRefValveCurrent(_ref);}


    void    SetMotionOwner(const int _jnum){
        Shm_com->MotionOwner[Joints[_jnum]->MCId][Joints[_jnum]->MCCh] = PODONum;
    }
    void    SetAllMotionOwner(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            SetMotionOwner(i);
        }
    }

    char	SetMoveJoint(const int _jnum, const double _angle, const double _msTime, const unsigned int _mode){
        return Joints[_jnum]->SetMoveJoint(_angle, _msTime, _mode);
    }
    char	MoveJoint(const int _jnum){
        return Joints[_jnum]->MoveJoint();
    }
    void    MoveAllJoint(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            MoveJoint(i);
        }
    }

    void    JointUpdate(){
        int mcId, mcCh;
        for(int i=0; i<NO_OF_JOINTS; i++){
            mcId = Joints[i]->MCId;
            mcCh = Joints[i]->MCCh;
            Shm_ref->AngleReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
            // For LIGHT (by BUYOUN)

//            VelocityReference[MAX_AL][MAX_MC][MOTOR_2CH];
//            ForceReference[MAX_AL][MAX_MC][MOTOR_2CH];
//            PressureReference[MAX_AL][MAX_MC][MOTOR_2CH];
//            PWMReference[MAX_AL][MAX_MC][MOTOR_2CH];
//            ValveReference[MAX_AL][MAX_MC][MOTOR_2CH];
            Shm_ref->AngVelReference[PODONum][mcId][mcCh] = Joints[i]->GetRefVelocityCurrent();
            Shm_ref->TorqueReference[PODONum][mcId][mcCh] = Joints[i]->GetRefTorqueCurrent();
            Shm_ref->PressureReference[PODONum][mcId][mcCh] = Joints[i]->GetRefPressureCurrent();
            Shm_ref->PWMReference[PODONum][mcId][mcCh] = Joints[i]->GetRefPWMCurrent();
            Shm_ref->ValveReference[PODONum][mcId][mcCh] = Joints[i]->GetRefValveCurrent();

            if(Shm_com->MotionOwner[mcId][mcCh] == PODONum)
                Shm_com->ACK_SIGNAL[mcId][mcCh] = true;
        }

        Shm_com->SYNC_SIGNAL[PODONum] = false;
    }
    void    RefreshToCurrentReference(){
        int mcId, mcCh;
        for(int i=0; i<NO_OF_JOINTS; i++){
            Joints[i]->SetMoveFlag(false);
            mcId = Joints[i]->MCId;
            mcCh = Joints[i]->MCCh;

            // For LIGHT (by BUYOUN)
            Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefAngle);
            Shm_ref->AngleReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();

            Joints[i]->SetRefVelocityCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefAngVel);
            Shm_ref->AngVelReference[PODONum][mcId][mcCh] = Joints[i]->GetRefVelocityCurrent();

//            Joints[i]->SetRefLinPosCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefLinPos);
//            Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefLinPosCurrent();

//            Joints[i]->SetRefLinVelCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefLinVel);
//            Shm_ref->VelocityReference[PODONum][mcId][mcCh] = Joints[i]->GetRefLinVelCurrent();

            Joints[i]->SetRefTorqueCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefTorque);
            Shm_ref->TorqueReference[PODONum][mcId][mcCh] = Joints[i]->GetRefTorqueCurrent();

            Joints[i]->SetRefPressureCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefPressure);
            Shm_ref->PressureReference[PODONum][mcId][mcCh] = Joints[i]->GetRefPressureCurrent();

            Joints[i]->SetRefPWMCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefPWM);
            Shm_ref->PWMReference[PODONum][mcId][mcCh] = Joints[i]->GetRefPWMCurrent();

            Joints[i]->SetRefValveCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefValve);
            Shm_ref->ValveReference[PODONum][mcId][mcCh] = Joints[i]->GetRefValveCurrent();

        }
    }

private:
    int                     PODONum;
    pRBCORE_SHM_REFERENCE   Shm_ref;
    pRBCORE_SHM_SENSOR      Shm_sen;
    pRBCORE_SHM_COMMAND     Shm_com;
};

#endif // BASICJOINT_H
