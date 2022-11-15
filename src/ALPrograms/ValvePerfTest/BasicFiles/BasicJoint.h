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
    double      RefAngVelCurrent;     // Joint angular vel., not cylinder vel.
    double      RefActPosCurrent;
    double      RefActVelCurrent;
    double      RefTorqueCurrent;
    double      RefActForceCurrent;
    double      RefValvePosCurrent;
    double      RefPWMCurrent;

public:
    JointClass(){
        RefAngleCurrent = 0.f;
        RefAngVelCurrent = 0.f;
        RefActPosCurrent = 0.f;
        RefActVelCurrent = 0.f;
        RefTorqueCurrent = 0.f;
        RefActForceCurrent= 0.f;
        RefValvePosCurrent= 0.f;
        RefPWMCurrent= 0.f;
        MoveFlag = false;
    }
    JointClass(const int jnum, const int id, const int ch){
        JNum = jnum; MCId = id; MCCh = ch;
        RefAngleCurrent = 0.f;
        RefAngVelCurrent = 0.f;
        RefActPosCurrent = 0.f;
        RefActVelCurrent = 0.f;
        RefTorqueCurrent = 0.f;
        RefActForceCurrent = 0.f;
        RefValvePosCurrent = 0.f;
        RefPWMCurrent = 0.f;
        MoveFlag = false;
    }

    // For LIGHT (by BUYOUN)
    void    SetRefAngleCurrent(const double ref)	{RefAngleCurrent = ref;}
    double  GetRefAngleCurrent()					{return RefAngleCurrent;}

    void    SetRefAngVelCurrent(const double ref)	{RefAngVelCurrent = ref;}
    double  GetRefAngVelCurrent()					{return RefAngVelCurrent;}

    void    SetRefActPosCurrent(const double ref)	{RefActPosCurrent = ref;}
    double  GetRefActPosCurrent()					{return RefActPosCurrent;}

    void    SetRefActVelCurrent(const double ref)	{RefActVelCurrent = ref;}
    double  GetRefActVelCurrent()					{return RefActVelCurrent;}

    void    SetRefTorqueCurrent(const double ref)	{RefTorqueCurrent = ref;}
    double  GetRefTorqueCurrent()					{return RefTorqueCurrent;}

    void    SetRefActForceCurrent(const double ref)	{RefActForceCurrent = ref;}
    double  GetRefActForceCurrent()					{return RefActForceCurrent;}

    void    SetRefValvePosCurrent(const double ref)	{RefValvePosCurrent = ref;}
    double  GetRefValvePosCurrent()					{return RefValvePosCurrent;}

    void    SetRefPWMCurrent(const double ref)	{RefPWMCurrent = ref;}
    double  GetRefPWMCurrent()					{return RefPWMCurrent;}

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

        GoalTimeCount = (unsigned long)(_msTime/(RT_TIMER_PERIOD_US/1000));
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
                RefAngleCurrent = RefAngleInitial+RefAngleDelta*0.5f*(1.0f-cos(PI/(double)GoalTimeCount*(double)CurrentTimeCount));
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

    double  GetJointRefAngVel(const int n)                    {return Joints[n]->GetRefAngVelCurrent();}
    void    SetJointRefAngVel(const int n, const double _ref)	{Joints[n]->SetRefAngVelCurrent(_ref);}

    double  GetJointRefActPos(const int n)						{return Joints[n]->GetRefActPosCurrent();}
    void    SetJointRefActPos(const int n, const double _ref)	{Joints[n]->SetRefActPosCurrent(_ref);}

    double  GetJointRefActVel(const int n)						{return Joints[n]->GetRefActVelCurrent();}
    void    SetJointRefActVel(const int n, const double _ref)	{Joints[n]->SetRefActVelCurrent(_ref);}

    double  GetJointRefTorque(const int n)						{return Joints[n]->GetRefTorqueCurrent();}
    void    SetJointRefTorque(const int n, const double _ref)	{Joints[n]->SetRefTorqueCurrent(_ref);}

    double  GetJointRefActForce(const int n)					{return Joints[n]->GetRefActForceCurrent();}
    void    SetJointRefActForce(const int n, const double _ref)	{Joints[n]->SetRefActForceCurrent(_ref);}

    double  GetRefValvePos(const int n)                         {return Joints[n]->GetRefValvePosCurrent();}
    void    SetRefValvePos(const int n, const double _ref)      {Joints[n]->SetRefValvePosCurrent(_ref);}

    double  GetRefPWM(const int n)                         {return Joints[n]->GetRefPWMCurrent();}
    void    SetRefPWM(const int n, const double _ref)      {Joints[n]->SetRefPWMCurrent(_ref);}

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
            Shm_ref->AngVelReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngVelCurrent();
            Shm_ref->ActPosReference[PODONum][mcId][mcCh] = Joints[i]->GetRefActPosCurrent();
            Shm_ref->ActVelReference[PODONum][mcId][mcCh] = Joints[i]->GetRefActVelCurrent();
            Shm_ref->TorqueReference[PODONum][mcId][mcCh] = Joints[i]->GetRefTorqueCurrent();
            Shm_ref->ActForceReference[PODONum][mcId][mcCh] = Joints[i]->GetRefActForceCurrent();
            Shm_ref->ValvePosReference[PODONum][mcId][mcCh] = Joints[i]->GetRefValvePosCurrent();
            Shm_ref->PWMReference[PODONum][mcId][mcCh] = Joints[i]->GetRefPWMCurrent();
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

            Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefAngle);
            Shm_ref->AngleReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
            Joints[i]->SetRefAngVelCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefAngVel);
            Shm_ref->AngVelReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngVelCurrent();
            Joints[i]->SetRefTorqueCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefTorque);
            Shm_ref->TorqueReference[PODONum][mcId][mcCh] = Joints[i]->GetRefTorqueCurrent();
            Joints[i]->SetRefActPosCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefActPos);
            Shm_ref->ActPosReference[PODONum][mcId][mcCh] = Joints[i]->GetRefActPosCurrent();
            Joints[i]->SetRefActVelCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefActVel);
            Shm_ref->ActVelReference[PODONum][mcId][mcCh] = Joints[i]->GetRefActVelCurrent();
            Joints[i]->SetRefActForceCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentRefActForce);
            Shm_ref->ActForceReference[PODONum][mcId][mcCh] = Joints[i]->GetRefActForceCurrent();

        }
    }

private:
    int                     PODONum;
    pRBCORE_SHM_REFERENCE   Shm_ref;
    pRBCORE_SHM_SENSOR      Shm_sen;
    pRBCORE_SHM_COMMAND     Shm_com;
};

#endif // BASICJOINT_H
