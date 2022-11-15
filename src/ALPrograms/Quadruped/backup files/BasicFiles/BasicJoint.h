#ifndef BASICJOINT_H
#define BASICJOINT_H

#include <iostream>
//#include <math.h>
#include <cmath>

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
    ERR_WRONG_SELECTION,
    ERR_ALREADY_STOPPED
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
    double      RefQCurrentCurrent;
    double      RefPWMCurrent;
    double      RefDACCurrent;
//    double      RefSineCurrent;

public:
    JointClass(){
        RefAngleCurrent = 0.f; MoveFlag = false;
        RefQCurrentCurrent = RefPWMCurrent = RefDACCurrent = 0.f; //RefSineCurrent= 0.f;
        MoveFlag1 = MoveFlag2 = MoveFlag3 = MoveFlag4 = MoveFlag5 = false;
    }
    JointClass(const int jnum, const int id, const int ch){
        JNum = jnum; MCId = id; MCCh = ch; RefAngleCurrent = 0.f; MoveFlag = false;
        RefQCurrentCurrent = RefPWMCurrent = RefDACCurrent = 0.f; //RefSineCurrent = 0.f;
        MoveFlag1 = MoveFlag2 = MoveFlag3 = MoveFlag4 = MoveFlag5 = false;
    }

    void    SetRefAngleCurrent(const double ref)	{RefAngleCurrent = ref;}
    double  GetRefAngleCurrent()					{return RefAngleCurrent;}
    void    SetMoveFlag(unsigned char flag)         {MoveFlag = flag;}

    void    SetRefQCurrentCurrent(const double ref)	{RefQCurrentCurrent = ref;}
    double  GetRefQCurrentCurrent()					{return RefQCurrentCurrent;}
    void    SetRefPWMCurrent(const double ref)      {RefPWMCurrent = ref;}
    double  GetRefPWMCurrent()                      {return RefPWMCurrent;}
    void    SetRefDACCurrent(const double ref)      {RefDACCurrent = ref;}
    double  GetRefDACCurrent()                      {return RefDACCurrent;}
//    void    SetRefSineCurrent(const double ref)     {RefSineCurrent = ref;}
//    double  GetRefSineCurrent()                     {return RefSineCurrent;}
    void    SetMoveFlag1(unsigned char flag)         {MoveFlag1 = flag;}
    void    SetMoveFlag2(unsigned char flag)         {MoveFlag2 = flag;}
    void    SetMoveFlag3(unsigned char flag)         {MoveFlag3 = flag;}
    void    SetMoveFlag4(unsigned char flag)         {MoveFlag4 = flag;}
    void    SetMoveFlag5(unsigned char flag)         {MoveFlag5 = flag;}

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
    char    AskMoveFlag(){
        if(MoveFlag==true){
            return STILL_MOVING;
        }else{
            return MOVE_DONE;
        }
    }

    char    SetMovePosJoint(const double _angle, const double _msPeriod, const double _msSmooth, const unsigned int _mode){
        if(_msSmooth <= 0){
            FILE_LOG(logWARNING) << "Smooth time must be greater than zero(SetMovePosJoint)[JNum: " << JNum << "]";
            return ERR_GOAL_TIME;
        }
        if(_msPeriod < 2*_msSmooth){
            FILE_LOG(logWARNING) << "Period time must be greater than double of Smooth time(SetMovePosJoint)[JNum: " << JNum << "]";
            return ERR_GOAL_TIME;
        }

        MoveFlag1 = false;
        switch(_mode)
        {
        case MOVE_RELATIVE:	// relative mode
            RefAngleToGo = RefAngleCurrent + _angle;
            break;
        case MOVE_ABSOLUTE:	// absolute mode
            RefAngleToGo = _angle;
            break;
        default:
            FILE_LOG(logWARNING) << "Wrong reference mode(SetMovePosJoint)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefAngleInitial = RefAngleCurrent;
        RefAngleDelta = RefAngleToGo - RefAngleCurrent;
        RefVelPeak = RefAngleDelta/(_msPeriod-_msSmooth);
        RefAngleInitial2 = RefAngleInitial + 0.5f*RefVelPeak*_msSmooth;
        RefAngleInitial3 = RefAngleInitial2 + RefVelPeak*(_msPeriod - 2.0f*_msSmooth);
        CurrentTimeCount = 0;

        GoalTimeCount = (unsigned long)(_msPeriod/RT_TIMER_PERIOD_MS);
        AccTimeCount = (unsigned long)(_msSmooth/RT_TIMER_PERIOD_MS);
        DecTimeCount = (unsigned long)((_msPeriod-_msSmooth)/RT_TIMER_PERIOD_MS);
        MoveFlag1 = true;
        return ERR_OK;
    }
    char    SetStopPosJoint(){
        if(MoveFlag1==true){
            if(CurrentTimeCount<DecTimeCount){
                RefAngleToGo = RefAngleCurrent + (RefAngleToGo-RefAngleInitial3);
                RefAngleInitial3 = RefAngleCurrent;
                CurrentTimeCount = DecTimeCount;
            }
            return ERR_OK;
        }
        return ERR_ALREADY_STOPPED;
    }
    char    MovePosJoint(){
        if(MoveFlag1 == true){
            CurrentTimeCount++;
            if(GoalTimeCount <= CurrentTimeCount){
                GoalTimeCount = CurrentTimeCount = 0;
                RefAngleCurrent = RefAngleToGo;
                MoveFlag1 = false;
                return MOVE_DONE;
            }else{
                if(CurrentTimeCount<AccTimeCount){
                    RefAngleCurrent = RefAngleInitial+0.5f*RefVelPeak*((double)CurrentTimeCount*RT_TIMER_PERIOD_MS)
                            -0.5f*RefVelPeak*((double)AccTimeCount*RT_TIMER_PERIOD_MS)/RBCORE_PI*sin(RBCORE_PI/(double)AccTimeCount*(double)CurrentTimeCount);
                }else if(CurrentTimeCount>=AccTimeCount && CurrentTimeCount<DecTimeCount){
                    RefAngleCurrent = RefAngleInitial2 + RefVelPeak*((double)CurrentTimeCount-(double)AccTimeCount)*RT_TIMER_PERIOD_MS;
                }else{
                    RefAngleCurrent = RefAngleInitial3 + 0.5f*RefVelPeak*((double)CurrentTimeCount-(double)DecTimeCount)*RT_TIMER_PERIOD_MS
                            +0.5f*RefVelPeak*((double)AccTimeCount*RT_TIMER_PERIOD_MS)/RBCORE_PI*sin(RBCORE_PI/(double)AccTimeCount*((double)CurrentTimeCount-(double)DecTimeCount));
                }
            }
        }
        return STILL_MOVING;
    }
    char    AskMoveFlag1(){
        if(MoveFlag1==true){
            return STILL_MOVING;
        }else{
            return MOVE_DONE;
        }
    }

    char    SetMoveQCurrent(const double _torque, const double _msTime, const unsigned int _mode){
        if(_msTime <= 0){
            FILE_LOG(logWARNING) << "Goal time must be greater than zero(SetMoveQCurrent)[JNum: " << JNum << "]"; return ERR_GOAL_TIME;
        }

        MoveFlag2 = false;
        switch(_mode)
        {
        case MOVE_RELATIVE:	// relative mode
            RefQCurrentToGo = RefQCurrentCurrent + _torque;
            break;
        case MOVE_ABSOLUTE:	// absolute mode
            RefQCurrentToGo = _torque;
            break;
        default:
            FILE_LOG(logWARNING) << "Wrong reference mode(SetMoveQCurrent)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefQCurrentInitial = RefQCurrentCurrent;
        RefQCurrentDelta = RefQCurrentToGo - RefQCurrentCurrent;
        CurrentTimeCount2 = 0;

        GoalTimeCount2 = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
        MoveFlag2 = true;
        return ERR_OK;
    }
    char    MoveQCurrent(){
        if(MoveFlag2 == true){
            CurrentTimeCount2++;
            if(GoalTimeCount2 <= CurrentTimeCount2){
                GoalTimeCount2 = CurrentTimeCount2 = 0;
                RefQCurrentCurrent = RefQCurrentToGo;
                MoveFlag2 = false;
                return MOVE_DONE;
            }else{
                RefQCurrentCurrent = RefQCurrentInitial+RefQCurrentDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount2*(double)CurrentTimeCount2));
            }
        }
        return STILL_MOVING;
    }
    char    AskMoveFlag2(){
        if(MoveFlag2==true){
            return STILL_MOVING;
        }else{
            return MOVE_DONE;
        }
    }

    char    SetMovePWM(const double _pwm, const double _msTime, const unsigned int _mode){
        if(_msTime <= 0){
            FILE_LOG(logWARNING) << "Goal time must be greater than zero(SetMovePWM)[JNum: " << JNum << "]"; return ERR_GOAL_TIME;
        }

        MoveFlag3 = false;
        switch(_mode)
        {
        case MOVE_RELATIVE:	// relative mode
            RefPWMToGo = RefPWMCurrent + _pwm;
            break;
        case MOVE_ABSOLUTE:	// absolute mode
            RefPWMToGo = _pwm;
            break;
        default:
            FILE_LOG(logWARNING) << "Wrong reference mode(SetMovePWM)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefPWMInitial = RefPWMCurrent;
        RefPWMDelta = RefPWMToGo - RefPWMCurrent;
        CurrentTimeCount3 = 0;

        GoalTimeCount3 = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
        MoveFlag3 = true;
        return ERR_OK;
    }
    char    MovePWM(){
        if(MoveFlag3 == true){
            CurrentTimeCount3++;
            if(GoalTimeCount3 <= CurrentTimeCount3){
                GoalTimeCount3 = CurrentTimeCount3 = 0;
                RefPWMCurrent = RefPWMToGo;
                MoveFlag3 = false;
                return MOVE_DONE;
            }else{
                RefPWMCurrent = RefPWMInitial+RefPWMDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount3*(double)CurrentTimeCount3));
            }
        }
        return STILL_MOVING;
    }
    char    AskMoveFlag3(){
        if(MoveFlag3==true){
            return STILL_MOVING;
        }else{
            return MOVE_DONE;
        }
    }

    char    SetMoveDAC(const double _dac, const double _msTime, const unsigned int _mode){
        if(_msTime <= 0){
            FILE_LOG(logWARNING) << "Goal time must be greater than zero(SetMoveDAC)[JNum: " << JNum << "]"; return ERR_GOAL_TIME;
        }

        MoveFlag4 = false;
        switch(_mode)
        {
        case MOVE_RELATIVE:	// relative mode
            RefDACToGo = RefDACCurrent + _dac;
            break;
        case MOVE_ABSOLUTE:	// absolute mode
            RefDACToGo = _dac;
            break;
        default:
            FILE_LOG(logWARNING) << "Wrong reference mode(SetMoveDAC)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefDACInitial = RefDACCurrent;
        RefDACDelta = RefDACToGo - RefDACCurrent;
        CurrentTimeCount4 = 0;

        GoalTimeCount4 = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
        MoveFlag4 = true;
        return ERR_OK;
    }
    char    MoveDAC(){
        if(MoveFlag4 == true){
            CurrentTimeCount4++;
            if(GoalTimeCount4 <= CurrentTimeCount4){
                GoalTimeCount4 = CurrentTimeCount4 = 0;
                RefDACCurrent = RefDACToGo;
                MoveFlag4 = false;
                return MOVE_DONE;
            }else{
                RefDACCurrent = RefDACInitial+RefDACDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount4*(double)CurrentTimeCount4));
            }
        }
        return STILL_MOVING;
    }
    char    AskMoveFlag4(){
        if(MoveFlag4==true){
            return STILL_MOVING;
        }else{
            return MOVE_DONE;
        }
    }

    char    SetMoveSine(const double _sine, const double _msTime, const unsigned int _mode){
        if(_msTime <= 0){
            FILE_LOG(logWARNING) << "Goal time must be greater than zero(SetMoveSine)[JNum: " << JNum << "]"; return ERR_GOAL_TIME;
        }

        MoveFlag5 = false;
        switch(_mode)
        {
        case MOVE_RELATIVE:	// relative mode
            RefQCurrentToGo = RefQCurrentCurrent + _sine;
            break;
        case MOVE_ABSOLUTE:	// absolute mode
            RefQCurrentToGo = _sine;
            break;
        default:
            FILE_LOG(logWARNING) << "Wrong reference mode(SetMoveSine)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefQCurrentInitial = RefQCurrentCurrent;
        RefQCurrentDelta = RefQCurrentToGo - RefQCurrentCurrent;
        CurrentTimeCount5 = 0;

        GoalTimeCount5 = (unsigned long)(_msTime/RT_TIMER_PERIOD_MS);
        MoveFlag5 = true;
        return ERR_OK;
    }
    char    MoveSine(){
        if(MoveFlag5 == true){
            CurrentTimeCount5++;
            if(GoalTimeCount5 <= CurrentTimeCount5){
                GoalTimeCount5 = CurrentTimeCount5 = 0;
                RefQCurrentCurrent = RefQCurrentInitial;
                MoveFlag5 = false;
                return MOVE_DONE;
            }else{
//                RefQCurrentCurrent = RefQCurrentInitial+RefQCurrentDelta*0.5f*(1.0f-cos(RBCORE_PI/(double)GoalTimeCount5*(double)CurrentTimeCount5));
                RefQCurrentCurrent = RefQCurrentInitial + RefQCurrentDelta*sin(2*RBCORE_PI/(double)GoalTimeCount5*(double)CurrentTimeCount5);
            }
        }
        return STILL_MOVING;
    }
    char    AskMoveFlag5(){
        if(MoveFlag5==true){
            return STILL_MOVING;
        }else{
            return MOVE_DONE;
        }
    }
private:
    double			RefAngleDelta;
    double			RefAngleToGo;
    double			RefAngleInitial;
    unsigned long	GoalTimeCount;
    unsigned long	CurrentTimeCount;
    unsigned char	MoveFlag; //for velocity

    double          RefVelPeak;
    double          RefAngleInitial2;
    double          RefAngleInitial3;
    unsigned long   AccTimeCount;
    unsigned long   DecTimeCount;
    unsigned char	MoveFlag1; //for Position

    double			RefQCurrentDelta;
    double			RefQCurrentToGo;
    double			RefQCurrentInitial;
    unsigned long	GoalTimeCount2;
    unsigned long	CurrentTimeCount2;
    unsigned char   MoveFlag2; //for QCurrent

    double			RefPWMDelta;
    double			RefPWMToGo;
    double			RefPWMInitial;
    unsigned long	GoalTimeCount3;
    unsigned long	CurrentTimeCount3;
    unsigned char   MoveFlag3; //for PWM

    double			RefDACDelta;
    double			RefDACToGo;
    double			RefDACInitial;
    unsigned long	GoalTimeCount4;
    unsigned long	CurrentTimeCount4;
    unsigned char   MoveFlag4; //for DAC

//    double			RefSineDelta;
//    double			RefSineToGo;
//    double			RefSineInitial;
    unsigned long	GoalTimeCount5;
    unsigned long	CurrentTimeCount5;
    unsigned char   MoveFlag5; //for Sine Reference
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

    double  GetJointRefAngle(const int n)						{return Joints[n]->GetRefAngleCurrent();}
    void    SetJointRefAngle(const int n, const double _ref)	{Joints[n]->SetRefAngleCurrent(_ref);}

    double  GetJointRefQCurrent(const int n)					{return Joints[n]->GetRefQCurrentCurrent();}
    void    SetJointRefQCurrent(const int n, const double _ref)	{Joints[n]->SetRefQCurrentCurrent(_ref);}
    double  GetJointRefPWM(const int n)                         {return Joints[n]->GetRefPWMCurrent();}
    void    SetJointRefPWM(const int n, const double _ref)      {Joints[n]->SetRefPWMCurrent(_ref);}
    double  GetJointRefDAC(const int n)                         {return Joints[n]->GetRefDACCurrent();}
    void    SetJointRefDAC(const int n, const double _ref)      {Joints[n]->SetRefDACCurrent(_ref);}
//    double  GetJointRefSine(const int n)						{return Joints[n]->GetRefSineCurrent();}
//    void    SetJointRefSine(const int n, const double _ref)     {Joints[n]->SetRefSineCurrent(_ref);}

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

    char	SetMovePosJoint(const int _jnum, const double _angle, const double _msPeriod, const double _msSmooth, const unsigned int _mode){
        return Joints[_jnum]->SetMovePosJoint(_angle, _msPeriod, _msSmooth, _mode);
    }
    char	SetStopPosJoint(const int _jnum){
        return Joints[_jnum]->SetStopPosJoint();
    }
    char	MovePosJoint(const int _jnum){
        return Joints[_jnum]->MovePosJoint();
    }
    void    MoveAllPosJoint(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            MovePosJoint(i);
        }
    }

    char	SetMoveQCurrent(const int _jnum, const double _torque, const double _msTime, const unsigned int _mode){
        return Joints[_jnum]->SetMoveQCurrent(_torque, _msTime, _mode);
    }
    char	MoveQCurrent(const int _jnum){
        return Joints[_jnum]->MoveQCurrent();
    }
    void    MoveAllQCurrent(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            MoveQCurrent(i);
        }
    }
    char	SetMovePWM(const int _jnum, const double _pwm, const double _msTime, const unsigned int _mode){
        return Joints[_jnum]->SetMovePWM(_pwm, _msTime, _mode);
    }
    char	MovePWM(const int _jnum){
        return Joints[_jnum]->MovePWM();
    }
    void    MoveAllPWM(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            MovePWM(i);
        }
    }
    char	SetMoveDAC(const int _jnum, const double _dac, const double _msTime, const unsigned int _mode){
        return Joints[_jnum]->SetMoveDAC(_dac, _msTime, _mode);
    }
    char	MoveDAC(const int _jnum){
        return Joints[_jnum]->MoveDAC();
    }
    void    MoveAllDAC(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            MoveDAC(i);
        }
    }
    char	SetMoveSine(const int _jnum, const double _sine, const double _msTime, const unsigned int _mode){
        return Joints[_jnum]->SetMoveSine(_sine, _msTime, _mode);
    }
    char	MoveSine(const int _jnum){
        return Joints[_jnum]->MoveSine();
    }
    void    MoveAllSine(){
        for(int i=0; i<NO_OF_JOINTS; i++){
            MoveSine(i);
        }
    }

    void    JointUpdate(){
        int mcId, mcCh;
        for(int i=0; i<NO_OF_JOINTS; i++){
            mcId = Joints[i]->MCId;
            mcCh = Joints[i]->MCCh;
            Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
            Shm_ref->COCOAQCurrent_REF[PODONum][mcId] = Joints[i]->GetRefQCurrentCurrent();
            Shm_ref->JointFFpwm[PODONum][mcId][mcCh] = Joints[i]->GetRefPWMCurrent();
            Shm_ref->COCOACurrent_REF[PODONum][mcId] = Joints[i]->GetRefDACCurrent();
            if(Shm_com->MotionOwner[mcId][mcCh] == PODONum)
                Shm_com->ACK_SIGNAL[mcId][mcCh] = true;
        }
        Shm_com->SYNC_SIGNAL[PODONum] = false;
    }
    void    RefreshToCurrentReference(){
        int mcId, mcCh;
        for(int i=0; i<NO_OF_JOINTS; i++){
            Joints[i]->SetMoveFlag(false);
            Joints[i]->SetMoveFlag1(false);
            Joints[i]->SetMoveFlag2(false);
            Joints[i]->SetMoveFlag3(false);
            Joints[i]->SetMoveFlag4(false);
            Joints[i]->SetMoveFlag5(false);
            mcId = Joints[i]->MCId;
            mcCh = Joints[i]->MCCh;
            Joints[i]->SetRefAngleCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentReference);
            Joints[i]->SetRefQCurrentCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentQCurrentReference);
            Joints[i]->SetRefPWMCurrent(Shm_sen->ENCODER[mcId][mcCh].PWMffout);
            Joints[i]->SetRefDACCurrent(Shm_sen->ENCODER[mcId][mcCh].CurrentCurrentReference);

            Shm_ref->JointReference[PODONum][mcId][mcCh] = Joints[i]->GetRefAngleCurrent();
            Shm_ref->COCOAQCurrent_REF[PODONum][mcId] = Joints[i]->GetRefQCurrentCurrent();
//            Shm_ref->JointFFpwm[PODONum][mcId][mcCh] = 0;
            Shm_ref->JointFFpwm[PODONum][mcId][mcCh] = Joints[i]->GetRefPWMCurrent();
            Shm_ref->COCOACurrent_REF[PODONum][mcId] = Joints[i]->GetRefDACCurrent();
        }
    }

private:
    int                     PODONum;
    pRBCORE_SHM_REFERENCE   Shm_ref;
    pRBCORE_SHM_SENSOR      Shm_sen;
    pRBCORE_SHM_COMMAND     Shm_com;
};

#endif // BASICJOINT_H
