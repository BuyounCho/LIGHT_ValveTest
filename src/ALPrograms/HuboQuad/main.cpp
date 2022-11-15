

#include "BasicFiles/BasicSetting.h"
#include "Oinverse.h"
#include "OW_RBDL.h"
#include "ow_quad.h"
//#include "joint_inverse.h"
#include "joint_inverse_SW.h"
#include "ManualCAN.h"


// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *jCon;

enum HUBOQUAD_ALCOMMAND
{
    HUBOQUAD_AL_NO_ACT = 100,
    HUBOQUAD_AL_READYPOS,
    HUBOQUAD_AL_WALK_START,
    HUBOQUAD_AL_WALK_STOP,
    HUBOQUAD_AL_LOCK,
    HUBOQUAD_AL_NOCON,
    HUBOQUAD_AL_PDTEST,
};
int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;

OW_Quad OQ;
Oinverse Oi;
int cntforcountsleep = 0;
void countsleep(int microsec)
{
    cntforcountsleep = 0;
    while(microsec/RT_TIMER_PERIOD_MS/1000.0>cntforcountsleep)
    {
        usleep(1*1000);//1ms sleep
    }
}
ONELEGFKIK Oii;
bool PDtest = false;
int PDtestBNO = 0;
double PDtestKP = 0;
double PDtestKD = 0;
double PDtestref = 0;
void MoveQJ(QuadJoints _QJ,double ms)
{
    jCon->SetMoveJoint(RHR,Oii.q2m_rhr(_QJ.RHR)*R2Df,ms,MOVE_ABSOLUTE);
    jCon->SetMoveJoint(RHP,Oii.q2m_hp(_QJ.RHP)*R2Df,ms,MOVE_ABSOLUTE);
    jCon->SetMoveJoint(RKN,Oii.q2m_kn(_QJ.RKN)*R2Df,ms,MOVE_ABSOLUTE);

    jCon->SetMoveJoint(LHR,Oii.q2m_lhr(_QJ.LHR)*R2Df,ms,MOVE_ABSOLUTE);
    jCon->SetMoveJoint(LHP,Oii.q2m_hp(_QJ.LHP)*R2Df,ms,MOVE_ABSOLUTE);
    jCon->SetMoveJoint(LKN,Oii.q2m_kn(_QJ.LKN)*R2Df,ms,MOVE_ABSOLUTE);

//    jCon->SetMoveJoint(RSR,Oii.q2m_rhr(_QJ.RSR)*R2Df,ms,MOVE_ABSOLUTE);
//    jCon->SetMoveJoint(RSP,Oii.q2m_hp(_QJ.RSP)*R2Df,ms,MOVE_ABSOLUTE);
//    jCon->SetMoveJoint(REB,Oii.q2m_kn(_QJ.REB)*R2Df,ms,MOVE_ABSOLUTE);

//    jCon->SetMoveJoint(LSR,Oii.q2m_lhr(_QJ.LSR)*R2Df,ms,MOVE_ABSOLUTE);
//    jCon->SetMoveJoint(LSP,Oii.q2m_hp(_QJ.LSP)*R2Df,ms,MOVE_ABSOLUTE);
//    jCon->SetMoveJoint(LEB,Oii.q2m_kn(_QJ.LEB)*R2Df,ms,MOVE_ABSOLUTE);
}
void SetRef(QuadJoints _QJ)
{
    jCon->SetJointRefAngle(RHR,Oii.q2m_rhr(_QJ.RHR)*R2Df);
    jCon->SetJointRefAngle(RHP,Oii.q2m_hp(_QJ.RHP)*R2Df);
    jCon->SetJointRefAngle(RKN,Oii.q2m_kn(_QJ.RKN)*R2Df);

    jCon->SetJointRefAngle(LHR,Oii.q2m_lhr(_QJ.LHR)*R2Df);
    jCon->SetJointRefAngle(LHP,Oii.q2m_hp(_QJ.LHP)*R2Df);
    jCon->SetJointRefAngle(LKN,Oii.q2m_kn(_QJ.LKN)*R2Df);

//    jCon->SetJointRefAngle(RSR,Oii.q2m_rhr(_QJ.RSR)*R2Df);
//    jCon->SetJointRefAngle(RSP,Oii.q2m_hp(_QJ.RSP)*R2Df);
//    jCon->SetJointRefAngle(REB,Oii.q2m_kn(_QJ.REB)*R2Df);

//    jCon->SetJointRefAngle(LSR,Oii.q2m_lhr(_QJ.LSR)*R2Df);
//    jCon->SetJointRefAngle(LSP,Oii.q2m_hp(_QJ.LSP)*R2Df);
//    jCon->SetJointRefAngle(LEB,Oii.q2m_kn(_QJ.LEB)*R2Df);
}
QuadJoints GetJointPos()
{

    QuadJoints _QJ;

    _QJ.RHR = Oii.m2q_rhr(jCon->GetJointRefAngle(RHR)*D2Rf);
    _QJ.RHP = Oii.m2q_hp(jCon->GetJointRefAngle(RHP)*D2Rf);
    _QJ.RKN = Oii.m2q_kn(jCon->GetJointRefAngle(RKN)*D2Rf);

    _QJ.LHR = Oii.m2q_lhr(jCon->GetJointRefAngle(LHR)*D2Rf);
    _QJ.LHP = Oii.m2q_hp(jCon->GetJointRefAngle(LHP)*D2Rf);
    _QJ.LKN = Oii.m2q_kn(jCon->GetJointRefAngle(LKN)*D2Rf);

//    _QJ.RSR = Oii.m2q_rhr(jCon->GetJointRefAngle(RSR)*D2Rf);
//    _QJ.RSP = Oii.m2q_hp(jCon->GetJointRefAngle(RSP)*D2Rf);
//    _QJ.REB = Oii.m2q_kn(jCon->GetJointRefAngle(REB)*D2Rf);

//    _QJ.LSR = Oii.m2q_lhr(jCon->GetJointRefAngle(LSR)*D2Rf);
//    _QJ.LSP = Oii.m2q_hp(jCon->GetJointRefAngle(LSP)*D2Rf);
//    _QJ.LEB = Oii.m2q_kn(jCon->GetJointRefAngle(LEB)*D2Rf);
    return _QJ;
}
double GetJointCur(int Jnum)
{
    return sharedSEN->ENCODER[Jnum][0].CurrentCurrent;
}
QuadJoints GetJointCurrents()
{
    QuadJoints _QJ;

    _QJ.RHR = GetJointCur(RHR);
    _QJ.RHP = GetJointCur(RHP);
    _QJ.RKN = GetJointCur(RKN);

    _QJ.LHR = GetJointCur(LHR);
    _QJ.LHP = GetJointCur(LHP);
    _QJ.LKN = GetJointCur(LKN);

//    _QJ.RSR = GetJointCur(RSR);
//    _QJ.RSP = GetJointCur(RSP);
//    _QJ.REB = GetJointCur(REB);

//    _QJ.LSR = GetJointCur(LSR);
//    _QJ.LSP = GetJointCur(LSP);
//    _QJ.LEB = GetJointCur(LEB);
    return _QJ;
}
double GetJointPWM(int Jnum)
{
    return sharedSEN->ENCODER[Jnum][0].PWMin;
}
QuadJoints GetJointPWMs()
{
    QuadJoints _QJ;

    _QJ.RHR = GetJointPWM(RHR);
    _QJ.RHP = GetJointPWM(RHP);
    _QJ.RKN = GetJointPWM(RKN);

    _QJ.LHR = GetJointPWM(LHR);
    _QJ.LHP = GetJointPWM(LHP);
    _QJ.LKN = GetJointPWM(LKN);

//    _QJ.RSR = GetJointPWM(RSR);
//    _QJ.RSP = GetJointPWM(RSP);
//    _QJ.REB = GetJointPWM(REB);

//    _QJ.LSR = GetJointPWM(LSR);
//    _QJ.LSP = GetJointPWM(LSP);
//    _QJ.LEB = GetJointPWM(LEB);
    return _QJ;
}
double GetJointAngleEnc(int Jnum)//return in radian
{
    return sharedSEN->ENCODER[Jnum][0].CurrentPosition*D2Rf;
}
double GetJointVelEnc(int Jnum)//return in radian
{
    return sharedSEN->ENCODER[Jnum][0].CurrentVelocity*D2Rf;
}
QuadJoints GetJointPosEnc()
{

    QuadJoints _QJ;

    _QJ.RHR = Oii.m2q_rhr(GetJointAngleEnc(RHR));
    _QJ.RHP = Oii.m2q_hp(GetJointAngleEnc(RHP));
    _QJ.RKN = Oii.m2q_kn(GetJointAngleEnc(RKN));

    _QJ.LHR = Oii.m2q_lhr(GetJointAngleEnc(LHR));
    _QJ.LHP = Oii.m2q_hp(GetJointAngleEnc(LHP));
    _QJ.LKN = Oii.m2q_kn(GetJointAngleEnc(LKN));

//    _QJ.RSR = Oii.m2q_rhr(GetJointAngleEnc(RSR));
//    _QJ.RSP = Oii.m2q_hp(GetJointAngleEnc(RSP));
//    _QJ.REB = Oii.m2q_kn(GetJointAngleEnc(REB));

//    _QJ.LSR = Oii.m2q_lhr(GetJointAngleEnc(LSR));
//    _QJ.LSP = Oii.m2q_hp(GetJointAngleEnc(LSP));
//    _QJ.LEB = Oii.m2q_kn(GetJointAngleEnc(LEB));
    return _QJ;

}
QuadJointVels GetJointVel()
{

    QuadJointVels _dQJ;
    double encV, encP;

    encV = GetJointVelEnc(RHR);
    encP = GetJointAngleEnc(RHR);
    _dQJ.RHR = Oii.J_rhr(encP)*encV;
    encV = GetJointVelEnc(RHP);
    encP = GetJointAngleEnc(RHP);
    _dQJ.RHP = Oii.J_hp(encP)*encV;
    encV = GetJointVelEnc(RKN);
    encP = GetJointAngleEnc(RKN);
    _dQJ.RKN = Oii.J_kn(encP)*encV;

    encV = GetJointVelEnc(LHR);
    encP = GetJointAngleEnc(LHR);
    _dQJ.LHR = Oii.J_lhr(encP)*encV;
    encV = GetJointVelEnc(LHP);
    encP = GetJointAngleEnc(LHP);
    _dQJ.LHP = Oii.J_hp(encP)*encV;
    encV = GetJointVelEnc(LKN);
    encP = GetJointAngleEnc(LKN);
    _dQJ.LKN = Oii.J_kn(encP)*encV;

//    encV = GetJointVelEnc(RSR);
//    encP = GetJointAngleEnc(RSR);
//    _dQJ.RSR = Oii.J_rhr(encP)*encV;
//    encV = GetJointVelEnc(RSP);
//    encP = GetJointAngleEnc(RSP);
//    _dQJ.RSP = Oii.J_hp(encP)*encV;
//    encV = GetJointVelEnc(REB);
//    encP = GetJointAngleEnc(REB);
//    _dQJ.REB = Oii.J_kn(encP)*encV;

//    encV = GetJointVelEnc(LSR);
//    encP = GetJointAngleEnc(LSR);
//    _dQJ.LSR = Oii.J_lhr(encP)*encV;
//    encV = GetJointVelEnc(LSP);
//    encP = GetJointAngleEnc(LSP);
//    _dQJ.LSP = Oii.J_hp(encP)*encV;
//    encV = GetJointVelEnc(LEB);
//    encP = GetJointAngleEnc(LEB);
//    _dQJ.LEB = Oii.J_kn(encP)*encV;

    return _dQJ;

}

QuadPos QP;
QuadJoints QJ,QJ_RT;

double FT_GAIN[4] = {1.0/1000.0*700.0*2.5, 1.0,1.0/500.0*700.0, -1.0/530.0*700.0*10.0};
HPRL IMUof[3];
FILE* ffpPD = NULL;
int PDcnt = 0;
int SAVEcnt = 0;
const int SAVEmax = 20000;
double SAVE[10][SAVEmax];
filt C_notch[12],C_notch2[12];
int main(int argc, char *argv[])
{
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "HuboQuad");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }
    for(int i=0;i<3;i++)
    {
        //IMUof[i].init(0.9,2000*D2Rf,0.002);//working....
        IMUof[i].init(0.8,3000*D2Rf,0.002);//working....
        //englesburg icra 2018
    }
    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

     for(int i=0;i<12;i++)
     {
         //notch filter here? 46.8Hz
         if(i%3==0)//roll
         {
               C_notch[i].set_notch(46.8,0.6);
         }
         else
         {
            C_notch[i].set_notch(46.8,0.3);
         }
         C_notch2[i].set_notch(63,1.0);
     }
     OQ.test_MPC();

    //////////////////////////////////////
    while(__IS_WORKING){
        usleep(100*1000);
        if(OQ.saveflag)
        {
            OQ.do_save_all();
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            QJ = GetJointPos();
            QJ.pPel = vec3();
            QJ.qPel = quat();
            usleep(50*1000);
            WalkParams WP = OQ.GetWP();//hmm.... why needed?
            OQ.init_Quad(QJ,WP);

            OQ.isCOMadjusting = false;
            OQ.isCOMadjusting_finish = true;
            OQ.isWalking = true;
        }
        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
        case HUBOQUAD_AL_PDTEST:
        {
            FILE_LOG(logSUCCESS) << "Command REBPDtest received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]<0)
            {
                FILE_LOG(logSUCCESS) << "PDtestBNO strange";

                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]>11)
            {
                FILE_LOG(logSUCCESS) << "PDtestBNO strange";

                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]==1)
            {
                PDtestBNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];

                std::string str("PDTEST");
                std:;time_t rawtime;
                std::tm* timeinfo;
                char buffer[80];
                std::time(&rawtime);
                timeinfo = std::localtime(&rawtime);
                std::strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
                std::string timestr(buffer);
                str = str+timestr;
                str = str + ".txt";
                ffpPD= fopen(str.c_str(),"w");
                PDcnt = 0;


                FILE_LOG(logSUCCESS) << "test start " <<PDtestBNO;

                sharedREF->COCOACurrent_REF[PODO_NO][PDtestBNO] = 0;
                sharedREF->change_to_Ccon[PODO_NO][PDtestBNO] = true;
                PDtestref = sharedSEN->ENCODER[PDtestBNO][0].CurrentPosition*D2Rf;
                PDtestKP = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                PDtestKD = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

                PDtest = true;

            }
            else
            {
                PDtestBNO = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1];
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = PDtestBNO;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1] = 0;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//CONTROL OFF
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_POSITION;
                FILE_LOG(logSUCCESS) << "test end";
                PDtest = false;
                usleep(10*1000);
                for(int i=0;i<PDcnt;i++)
                {
                    for(int j=0;j<5;j++)
                    {
                        fprintf(ffpPD,"%f\t",SAVE[j][i]);
                    }
                    fprintf(ffpPD,"\n");
                    if(i%1000==0)
                    {
                        printf("saving... %d / %d\n",i,PDcnt);
                    }
                }
                fclose(ffpPD);
                 printf("save done\n");
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case HUBOQUAD_AL_NOCON:
        {

            FILE_LOG(logSUCCESS) << "Command NOCON received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            FILE_LOG(logSUCCESS) << "0: CONTROL OFF";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//CONTROL OFF
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_POSITION;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);
            if(OQ.isStandingControl)
            {
                OQ.save_all(2);
                OQ.isStandingControl = false;
            }
            if(OQ.isWalking)
            {
                OQ.save_all(2);
                OQ.isWalking = false;
            }
            FILE_LOG(logSUCCESS) << "1: UPDATE REF";//4~5 to be after position con on and findhome?
            for(int i=0;i<NO_OF_JOINTS;i++)
            {
                double enc = sharedSEN->ENCODER[i][0].CurrentPosition;
                jCon->SetJointRefAngle(i,enc);
            }
            FILE_LOG(logSUCCESS) << "CREF ZERO";
            for(int i=0;i<NO_OF_JOINTS;i++)
            {
                sharedREF->COCOACurrent_REF[PODO_NO][i] = 0;
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case HUBOQUAD_AL_LOCK:
        {

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

             FILE_LOG(logSUCCESS) << "Command LOCK_IN_PLACE received..";

             FILE_LOG(logSUCCESS) << "0: REF DISBLE";
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = false;//on
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
             while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
             {
                 usleep(50*1000);
             }
             usleep(50*1000);

             FILE_LOG(logSUCCESS) << "0: CONTROL OFF";
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//CONTROL OFF
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_POSITION;
             while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
             {
                 usleep(50*1000);
             }
             usleep(50*1000);

             FILE_LOG(logSUCCESS) << "1: ENC ENABLE";
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 1;//ON
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_ENCODER_ONOFF;
             while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
             {
                 usleep(50*1000);
             }
             FILE_LOG(logSUCCESS) << "2: FET OFF";
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 0;//FET OFF
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
             while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
             {
                 usleep(50*1000);
             }

             FILE_LOG(logSUCCESS) << "3: FET ON";
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//FET ON
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FET_ONOFF;
             while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
             {
                 usleep(50*1000);
             }
             usleep(50*1000);


             FILE_LOG(logSUCCESS) << "4: UPDATE REF";//4~5 to be after position con on and findhome?
             for(int i=0;i<NO_OF_JOINTS;i++)
             {
                double enc = sharedSEN->ENCODER[i][0].CurrentPosition;
                 jCon->SetJointRefAngle(i,enc);
             }

             FILE_LOG(logSUCCESS) << "5: REF ENABLE";
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = true;//on
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_MOTION_REF_ONOFF;
             while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
             {
                 usleep(50*1000);
             }
             usleep(50*1000);

             FILE_LOG(logSUCCESS) << "6: POSITION CONTROL ON";
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//CONTROL ON
             sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_POSITION;
             while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
             {
                 usleep(50*1000);
             }
             usleep(50*1000);


            usleep(2000*1000);
            FILE_LOG(logSUCCESS) << "HUBOQUAD_AL_LOCK_DONE";


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }       
        case HUBOQUAD_AL_WALK_START:
        {
            FILE_LOG(logSUCCESS) << "HUBOQUAD_AL_WALKSTART";
            if(OQ.isfalldown)
            {
                cout<< "FALLDOWN_NEED_RESET"<<endl;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }
            //NO_ROBOT_TEST
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1]==1)
            {
                OQ.NO_ROBOT_TEST = true;
            }
            else
            {
                OQ.NO_ROBOT_TEST = false;
            }
            //ADJUST_SLOPE
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[2]==1)
            {
                OQ.DO_ADJUST_SLOPE = true;
            }
            else
            {
                OQ.DO_ADJUST_SLOPE = false;
            }
            //FOOTZ_DIRECTION
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[3]==1)
            {
                OQ.DO_FOOTZ_DIRECTION = true;
            }
            else
            {
                OQ.DO_FOOTZ_DIRECTION = false;
            }

            if(OQ.NO_ROBOT_TEST)
            {
               cout<< "NO_ROBOT_TEST"<<endl;
            }
            else if(sharedSEN->IMU[0].NO_RESPONSE_CNT>200)
            {
                cout<< "SENSOR NOT RESPONDING!!"<<endl;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }
            WalkParams WP;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            WP.step_L = vec3(sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0],
                            sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1],
                            sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]);
            WP.step_Rot =sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3]*D2Rf;
//            if(isnan(WP.step_Rot))
            if(0)
            {
                cout<<"steprotnan"<<endl;
                WP.step_Rot = 0;
            }
            WP.step_T = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];



            WP.FB_L = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
            WP.LR_L = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
            WP.delZ = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
            WP.Gait = sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0];
            WP.overlap = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
            WP.dsp_ratio = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[9];
             WP.landing_depth = 0.0;
            if(fabs(WP.step_L.norm())>0.35)
            {
                cout<< "STEP TOO BIG"<<endl;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }
            if(WP.step_T<0.2)
            {
                cout<< "STEP TOO FAST"<<endl;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
                break;
            }
            //here some tunings
            if(WP.Gait==Trot)
            {
                WP.dsp_ratio = 0;
                WP.landing_depth = -0.005;
                //WP.landing_depth = 0;
            }

            if(WP.Gait==Wave||WP.Gait==Wave2)
            {
//                WP.dsp_ratio = 0.3;
                WP.landing_depth = -0.01*0.1;
            }


            if(OQ.isWalking==false)
            {

                jCon->RefreshToCurrentReference();
                jCon->SetAllMotionOwner();                
                QJ = GetJointPos();
                QJ.pPel = vec3();
                QJ.qPel = quat();
                FILE_LOG(logSUCCESS) << "ERR CLEAR";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_COCOA_ERROR_CLEAR_ALL;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
                //IMUZERO HERE
                FILE_LOG(logSUCCESS) << "IMU ZERO";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = 3;
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_SENSOR_IMU_NULL;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);


                FILE_LOG(logSUCCESS) << "START CONTROL";
                OQ.init_Quad(QJ,WP);
                OQ.start_Walking();
            }
            else
            {
                if(OQ.stopcnt<10)
                {
                    OQ.stopcnt = 10;
                }
                OQ.changeWalk(WP);
            }

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case HUBOQUAD_AL_WALK_STOP:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[1])
            {
                OQ.NOSAVE = true;
            }
            else
            {
                OQ.NOSAVE = false;
            }
            OQ.stop_Walking();            
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }        
        case HUBOQUAD_AL_READYPOS:
        {
            OQ.isWalking = false;
            OQ.isStandingControl = false;
            OQ.isfalldown = false;
            FILE_LOG(logSUCCESS) << "HUBOQUAD_AL_READYPOS";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            QJ.pPel = vec3();
            QJ.qPel = quat();
            QJ.RHR = 0*D2Rf;
            QJ.RHP = 45*D2Rf;
            QJ.RKN = -90*D2Rf;

            QJ.LHR = 0*D2Rf;
            QJ.LHP = 45*D2Rf;
            QJ.LKN = -90*D2Rf;

            QJ.RSR = 0*D2Rf;
            QJ.RSP = 45*D2Rf;
            QJ.REB = -90*D2Rf;

            QJ.LSR = 0*D2Rf;
            QJ.LSP = 45*D2Rf;
            QJ.LEB = -90*D2Rf;

            QP = Oi.FK(QJ);
            printf("pRF %f %f %f\n",QP.pRF.x,QP.pRF.y,QP.pRF.z);
            printf("pLF %f %f %f\n",QP.pLF.x,QP.pLF.y,QP.pLF.z);
            printf("pRH %f %f %f\n",QP.pRH.x,QP.pRH.y,QP.pRH.z);
            printf("pLH %f %f %f\n",QP.pLH.x,QP.pLH.y,QP.pLH.z);
            printf("pCOM %f %f %f\n",QP.pCOM.x,QP.pCOM.y,QP.pCOM.z);
            QP.pCOM.x = (QP.pRF.x+QP.pLF.x+QP.pRH.x+QP.pLH.x)/4.0;
            QP.pCOM.y = (QP.pRF.y+QP.pLF.y+QP.pRH.y+QP.pLH.y)/4.0;
            double comh = min(0.5,0.1+sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]);
            QP.pCOM.z = QP.pRF.z+comh;
            QJ = Oi.IK_COM(QP);
            printf("QJRF %f %f %f \n",QJ.RHR,QJ.RHP,QJ.RKN);
            printf("QJLF %f %f %f \n",QJ.LHR,QJ.LHP,QJ.LKN);
            printf("QJRH %f %f %f \n",QJ.RSR,QJ.RSP,QJ.REB);
            printf("QJLH %f %f %f \n",QJ.LSR,QJ.LSP,QJ.LEB);
            //MoveQJ(QJ,2000);
            double ms = 2000;
            QuadJoints _QJ = QJ;
            QuadJoints _QJnow = GetJointPos();
            _QJnow.qPel = quat();
            _QJnow.pPel = vec3();
            QuadPos _QPnow = Oi.FK(_QJnow);
            bool checkpitch = (_QPnow.pRF.x<0)&&(_QPnow.pLF.x<0);//hmm....
            if(checkpitch)
            {
                jCon->SetMoveJoint(RHR,Oii.q2m_rhr(_QJ.RHR)*R2Df,ms,MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RHP,Oii.q2m_hp(_QJ.RHP)*R2Df,ms,MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LHR,Oii.q2m_lhr(_QJ.LHR)*R2Df,ms,MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LHP,Oii.q2m_hp(_QJ.LHP)*R2Df,ms,MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(RSR,Oii.q2m_rhr(_QJ.RSR)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RSP,Oii.q2m_hp(_QJ.RSP)*R2Df,ms,MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LSR,Oii.q2m_lhr(_QJ.LSR)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LSP,Oii.q2m_hp(_QJ.LSP)*R2Df,ms,MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(RKN,Oii.q2m_kn(_QJ.RKN)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LKN,Oii.q2m_kn(_QJ.LKN)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LEB,Oii.q2m_kn(_QJ.LEB)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(REB,Oii.q2m_kn(_QJ.REB)*R2Df,ms,MOVE_ABSOLUTE);
            }
            else
            {
                jCon->SetMoveJoint(RHR,Oii.q2m_rhr(_QJ.RHR)*R2Df,ms,MOVE_ABSOLUTE);
                jCon->SetMoveJoint(RHP,Oii.q2m_hp(_QJ.RHP)*R2Df,ms,MOVE_ABSOLUTE);

                jCon->SetMoveJoint(LHR,Oii.q2m_lhr(_QJ.LHR)*R2Df,ms,MOVE_ABSOLUTE);
                jCon->SetMoveJoint(LHP,Oii.q2m_hp(_QJ.LHP)*R2Df,ms,MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(RSR,Oii.q2m_rhr(_QJ.RSR)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(RSP,Oii.q2m_hp(_QJ.RSP)*R2Df,ms,MOVE_ABSOLUTE);

//                jCon->SetMoveJoint(LSR,Oii.q2m_lhr(_QJ.LSR)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LSP,Oii.q2m_hp(_QJ.LSP)*R2Df,ms,MOVE_ABSOLUTE);

//                countsleep(2100*1000);

//                jCon->SetMoveJoint(RKN,Oii.q2m_kn(_QJ.RKN)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LKN,Oii.q2m_kn(_QJ.LKN)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(LEB,Oii.q2m_kn(_QJ.LEB)*R2Df,ms,MOVE_ABSOLUTE);
//                jCon->SetMoveJoint(REB,Oii.q2m_kn(_QJ.REB)*R2Df,ms,MOVE_ABSOLUTE);
            }
            QP = Oi.FK(QJ);
            printf("pRF %f %f %f\n",QP.pRF.x,QP.pRF.y,QP.pRF.z);
            printf("pLF %f %f %f\n",QP.pLF.x,QP.pLF.y,QP.pLF.z);
            printf("pRH %f %f %f\n",QP.pRH.x,QP.pRH.y,QP.pRH.z);
            printf("pLH %f %f %f\n",QP.pLH.x,QP.pLH.y,QP.pLH.z);
            printf("pCOM %f %f %f\n",QP.pCOM.x,QP.pCOM.y,QP.pCOM.z);
            countsleep(2100*1000);
            printf("ready done\n");

        }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}



//==============================//
// Task Thread
//==============================//
WalkSensors WS;
double tsoff;
RTIME looptime;
RTIME looptimeQP;
int QPcnt;
int ff250 = 0;
void RBQPTaskThread(void *)
{
    while(__IS_WORKING)
    {
        RTIME temptime = rt_timer_read();
        double calcms = (temptime-looptimeQP)*0.001*0.001;
        if(calcms>6)//nominal 4ms
        {printf("looptimeQP %f\n",calcms);}
        looptimeQP = temptime;

        //do_QP_solve
        //from WS
        if(OQ.isWalking)
        {
           OQ.calc_ff_tau_ddq_QP(); //do_QP_solve
        }

        rt_task_suspend(&rtQPTaskCon);
        QPcnt = 0;
    }
}
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        RTIME temptime = rt_timer_read();
        double calcms = (temptime-looptime)*0.001*0.001;
        WS.looptime = calcms;
        if(calcms>3.0)
        {printf("looptime %f\n",calcms);}
        looptime = temptime;
        WS.IMUangle = D2Rf*vec3(sharedSEN->IMU[0].Roll,sharedSEN->IMU[0].Pitch,sharedSEN->IMU[0].Yaw);
        WS.IMUquat = quat(sharedSEN->IMU[0].Q[0]
                ,sharedSEN->IMU[0].Q[1]
                ,sharedSEN->IMU[0].Q[2]
                ,sharedSEN->IMU[0].Q[3]);


        //filter here!!!!


        WS.IMUomega_raw = vec3(sharedSEN->IMU[0].RollVel
                ,sharedSEN->IMU[0].PitchVel
                ,sharedSEN->IMU[0].YawVel)*D2Rf;

        WS.IMUomega = vec3(IMUof[0].do_filt(WS.IMUomega_raw.x)
                ,IMUof[1].do_filt(WS.IMUomega_raw.y)
                ,IMUof[2].do_filt(WS.IMUomega_raw.z));

      //  WS.IMUomega = WS.IMUomega_raw;

        WS.IMUacc = vec3(sharedSEN->IMU[0].AccX
                ,sharedSEN->IMU[0].AccY
                ,sharedSEN->IMU[0].AccZ);


        WS.RF_Fz = 3000-sharedSEN->CONTACT_SENSOR_POS[2];
        WS.LF_Fz = 3400-sharedSEN->CONTACT_SENSOR_POS[3];
        WS.RH_Fz = 2600-sharedSEN->CONTACT_SENSOR_POS[0];
        WS.LH_Fz = 3200-sharedSEN->CONTACT_SENSOR_POS[1];
//        WS.LH_Fz = sharedSEN->FT[0].Mx*FT_GAIN[2];
//        WS.RH_Fz = sharedSEN->FT[0].My*FT_GAIN[3];
        if(OQ.NO_ROBOT_TEST)
        {
            WS.IMUquat = OQ.QP.qPel;
            WS.IMUangle.x = 0;
            WS.IMUangle.y = 0;
            WS.IMUangle.z = OQ.decomp_A[0];
            WS.IMUomega = OQ.QP.dqPel;
            WS.IMUomega_raw = OQ.QP.dqPel;
            WS.IMUacc = vec3(0,0,0);
        }

        WS.JointPos = GetJointPos();
        WS.JointPosEnc = GetJointPosEnc();
        WS.JointPos.qPel = WS.IMUquat;
        WS.JointPosEnc.qPel = WS.IMUquat;
        WS.JointVel = GetJointVel();
        WS.JointVel.dqPel = WS.IMUomega;
        WS.JointCurrent = GetJointCurrents();
        WS.JointPWM = GetJointPWMs();
        for(int i=0;i<12;i++)
        {
            WS.BIGERR[i] = sharedSEN->ENCODER[i][0].Cocoa_Data.BIGERROR_ONOFF;
            WS.ENCERR[i] = sharedSEN->ENCODER[i][0].Cocoa_Data.ENCERROR_ONOFF;
            WS.CANERR[i] = sharedSEN->ENCODER[i][0].Cocoa_Data.CANERROR_ONOFF;
            WS.NO_RESPONSE_CNT_COCOA[i] = sharedSEN->ENCODER[i][0].NO_RESPONSE_CNT;

        }
        WS.NO_RESPONSE_CNT = sharedSEN->IMU[0].NO_RESPONSE_CNT;
        if(OQ.NO_ROBOT_TEST)
        {
            WS.NO_RESPONSE_CNT  = 0;//NO_ROBOT_TEST
            WS.RF_Fz = 0;//never contact
            WS.LF_Fz = 0;
            WS.RH_Fz = 0;
            WS.LH_Fz = 0;
        }
        QPcnt++;
        WS.QPcnt = QPcnt;

//       if(ff250%100==0)
//       {
//        cout<<"ROLLANGLETEST "<<WS.JointPosEnc.RHR*R2Df<<endl;
//       }
        if(0)
        {
        cout<<"IMUquat1 "<<WS.IMUquat[0]<<" "<<WS.IMUquat[1]<<" "<<WS.IMUquat[2]<<" "<<WS.IMUquat[3]<<endl;

        mat3 IMUrotx = mat3(vec3(-1,0,0),WS.IMUangle.x);
        mat3 IMUroty = mat3(vec3(0,-1,0),WS.IMUangle.y);
        mat3 IMUrotz = mat3(vec3(0,0,-1),WS.IMUangle.z);
        mat3 IMUrot = IMUrotz*IMUroty*IMUrotx;//z-y-x
        quat IMUquat = quat(IMUrot);
        cout<<"IMUquat2 "<<IMUquat[0]<<" "<<IMUquat[1]<<" "<<IMUquat[2]<<" "<<IMUquat[3]<<endl;

        cout<<"IMUangle in degree"<<WS.IMUangle.x*D2Rf<<", "
           <<WS.IMUangle.y*D2Rf<<", "<<WS.IMUangle.z*D2Rf<<endl;
        }
        userData->M2G.RFFT = WS.RF_Fz;
        userData->M2G.LFFT = WS.LF_Fz;
        userData->M2G.RHFT = WS.RH_Fz;
        userData->M2G.LHFT = WS.LH_Fz;
        userData->M2G.Angle[0] = WS.IMUangle[0];
        userData->M2G.Angle[1] = WS.IMUangle[1];
        userData->M2G.Angle[2] = WS.IMUangle[2];
        userData->M2G.Q[0] = sharedSEN->IMU[0].Q[0];
        userData->M2G.Q[1] = sharedSEN->IMU[0].Q[1];
        userData->M2G.Q[2] = sharedSEN->IMU[0].Q[2];
        userData->M2G.Q[3] = sharedSEN->IMU[0].Q[3];
        userData->M2G.Omega[0] = WS.IMUomega[0];
        userData->M2G.Omega[1] = WS.IMUomega[1];
        userData->M2G.Omega[2] = WS.IMUomega[2];

        if(OQ.isWalking)
        {
            if(OQ.isCOMadjusting)
            {
                QJ_RT = OQ.COMadjust_onestep(WS);
            }
            else if(OQ.isCOMadjusting_finish)
            {
                QJ_RT = OQ.COMadjust_onestep(WS);
            }
            else if(OQ.GetWP().Gait==Trot)
            {
                QJ_RT = OQ.Trot_onestep(WS);
            }
            else if(OQ.GetWP().Gait==Standing)
            {
                QJ_RT = OQ.Standing_onestep(WS);
            }
            else if(OQ.GetWP().Gait==Wave)//not working yet
            {
                QJ_RT = OQ.Wave_onestep(WS);
            }
            else if(OQ.GetWP().Gait==Wave2)
            {
                QJ_RT = OQ.Wave2_onestep(WS);
            }
            else if(OQ.GetWP().Gait==Pronk)
            {
                QJ_RT = OQ.Pronk_onestep(WS);
            }
            else if(OQ.GetWP().Gait==Flytrot)
            {
                QJ_RT = OQ.Flytrot_onestep(WS);
            }
            else if(OQ.GetWP().Gait==Demo)
            {
                QJ_RT = OQ.Demo_onestep(WS);
            }
            userData->M2G.qPel[0] = OQ.QP.qPel[0];
            userData->M2G.qPel[1] = OQ.QP.qPel[1];
            userData->M2G.qPel[2] = OQ.QP.qPel[2];
            userData->M2G.qPel[3] = OQ.QP.qPel[3];


            double scale =1.0;//GGGGGG


                sharedREF->COCOACurrent_REF[PODO_NO][RHR] = (OQ.QJ_CRef.RHR+OQ.QJ_CFF.RHR)*scale;
                sharedREF->COCOACurrent_REF[PODO_NO][LHR] = (OQ.QJ_CRef.LHR+OQ.QJ_CFF.LHR)*scale;
//                sharedREF->COCOACurrent_REF[PODO_NO][RSR] = (OQ.QJ_CRef.RSR+OQ.QJ_CFF.RSR)*scale;
//                sharedREF->COCOACurrent_REF[PODO_NO][LSR] = (OQ.QJ_CRef.LSR+OQ.QJ_CFF.LSR)*scale;

                sharedREF->COCOACurrent_REF[PODO_NO][RHP] = (OQ.QJ_CRef.RHP+OQ.QJ_CFF.RHP)*scale;
                sharedREF->COCOACurrent_REF[PODO_NO][LHP] = (OQ.QJ_CRef.LHP+OQ.QJ_CFF.LHP)*scale;
//                sharedREF->COCOACurrent_REF[PODO_NO][RSP] = (OQ.QJ_CRef.RSP+OQ.QJ_CFF.RSP)*scale;
//                sharedREF->COCOACurrent_REF[PODO_NO][LSP] = (OQ.QJ_CRef.LSP+OQ.QJ_CFF.LSP)*scale;

                sharedREF->COCOACurrent_REF[PODO_NO][RKN] = (OQ.QJ_CRef.RKN+OQ.QJ_CFF.RKN)*scale;
                sharedREF->COCOACurrent_REF[PODO_NO][LKN] = (OQ.QJ_CRef.LKN+OQ.QJ_CFF.LKN)*scale;
//                sharedREF->COCOACurrent_REF[PODO_NO][REB] = (OQ.QJ_CRef.REB+OQ.QJ_CFF.REB)*scale;
//                sharedREF->COCOACurrent_REF[PODO_NO][LEB] = (OQ.QJ_CRef.LEB+OQ.QJ_CFF.LEB)*scale;



            for(int i=0;i<4;i++)
            {
//                if(OQ.changetoPcon[i])//full torque control
//                {
//                    sharedREF->change_to_Pcon[PODO_NO][RHR+i*3] = true;
//                    sharedREF->change_to_Pcon[PODO_NO][RKN+i*3] = true;
//                    sharedREF->change_to_Pcon[PODO_NO][RHP+i*3] = true;
//                    OQ.changetoPcon[i] = false;

//                }
//                if(OQ.changetoCcon[i])
//                {
//                    sharedREF->change_to_Ccon[PODO_NO][RHR+i*3] = true;
//                    sharedREF->change_to_Ccon[PODO_NO][RKN+i*3] = true;
//                    sharedREF->change_to_Ccon[PODO_NO][RHP+i*3] = true;
//                    OQ.changetoCcon[i] = false;
//                }
                if(OQ.changetoPcon[i])//hybrid control
                {
                    sharedREF->change_to_Pcon[PODO_NO][RKN+i*3] = true;
                    sharedREF->change_to_Pcon[PODO_NO][RHP+i*3] = true;
                    OQ.changetoPcon[i] = false;

                }
                if(OQ.changetoCcon[i])
                {
                    sharedREF->change_to_Ccon[PODO_NO][RKN+i*3] = true;
                    sharedREF->change_to_Ccon[PODO_NO][RHP+i*3] = true;
                    OQ.changetoCcon[i] = false;
                }

            }
            SetRef(QJ_RT);
        }
        else if(PDtest)
        {
            double Anglenow = GetJointAngleEnc(PDtestBNO);
            double Velnow = GetJointVelEnc(PDtestBNO);

            double ccc = PDtestKP*(PDtestref-Anglenow) +PDtestKD*(-Velnow);
            double maxA = 3;
            if(ccc >  maxA){ccc =  maxA;}
            if(ccc < -maxA){ccc = -maxA;}
            ccc = 2;
            if(cntforcountsleep%1000==0)
            {                
                std::cout<<PDtestBNO<<" ccc "<<ccc <<" ref "<<PDtestref*R2Df<<" now "<<Anglenow*R2Df<<std::endl;
                std::cout << "pgain "<<PDtestKP<<" dgain "<<PDtestKD<<std::endl;

                for(int i=0;i<12;i++)
                {
                    std::cout<<GetJointAngleEnc(i)*R2Df<<" ";
                    if(i!=PDtestBNO)
                    {
                        sharedREF->COCOACurrent_REF[PODO_NO][i] = 0;
                    }
                }
                std::cout<<std::endl;
            }
            sharedREF->COCOACurrent_REF[PODO_NO][PDtestBNO] = ccc;


            SAVE[0][PDcnt] = Anglenow;
            SAVE[1][PDcnt] = Velnow;
            SAVE[2][PDcnt] = ccc;
            SAVE[3][PDcnt] = GetJointCur(PDtestBNO);
            PDcnt++;
            if(PDcnt>SAVEmax){PDcnt = SAVEmax;}
        }
        else
        {
            sharedREF->COCOACurrent_REF[PODO_NO][RHP] = 0;
            sharedREF->COCOACurrent_REF[PODO_NO][RKN] = 0;
            sharedREF->COCOACurrent_REF[PODO_NO][LHP] = 0;
            sharedREF->COCOACurrent_REF[PODO_NO][LKN] = 0;
           // sharedREF->COCOACurrent_REF[PODO_NO][RSP] = 0;
            //sharedREF->COCOACurrent_REF[PODO_NO][REB] = 0;
            //sharedREF->COCOACurrent_REF[PODO_NO][LSP] = 0;
            //sharedREF->COCOACurrent_REF[PODO_NO][LEB] = 0;
        }

        jCon->MoveAllJoint();        
        cntforcountsleep++;
        sharedREF->NO_RESPONSE_CNT[PODO_NO] = 0;
        rt_task_suspend(&rtTaskCon);
    }
}
//==============================//



//==============================//
// Flag Thread
//==============================//

void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 10*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        //if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);

                ff250++;
                if(ff250%2==0)
                {
                    rt_task_resume(&rtQPTaskCon);
                }
            }
        //}

    }
}
//==============================//

VectorNd calc_5th(double t_0, double t_e,vec3 h_0,vec3 h_e)
{
    vec3 tempV = h_e;
    tempV[0] = 0;
    if((t_e-t_0<0.05)&&tempV.norm()<1e-8){t_e = t_0+0.05;}
    MatrixNd AAA(6,6);
    AAA(0,0) = pow(t_0,5);
    AAA(0,1) = pow(t_0,4);
    AAA(0,2) = pow(t_0,3);
    AAA(0,3) = pow(t_0,2);
    AAA(0,4) = pow(t_0,1);
    AAA(0,5) = 1;

    AAA(1,0) = 5*pow(t_0,4);
    AAA(1,1) = 4*pow(t_0,3);
    AAA(1,2) = 3*pow(t_0,2);
    AAA(1,3) = 2*pow(t_0,1);
    AAA(1,4) = 1;
    AAA(1,5) = 0;

    AAA(2,0) = 20*pow(t_0,3);
    AAA(2,1) = 12*pow(t_0,2);
    AAA(2,2) = 6*pow(t_0,1);
    AAA(2,3) = 2*1;
    AAA(2,4) = 0;
    AAA(2,5) = 0;

    AAA(3,0) = pow(t_e,5);
    AAA(3,1) = pow(t_e,4);
    AAA(3,2) = pow(t_e,3);
    AAA(3,3) = pow(t_e,2);
    AAA(3,4) = pow(t_e,1);
    AAA(3,5) = 1;

    AAA(4,0) = 5*pow(t_e,4);
    AAA(4,1) = 4*pow(t_e,3);
    AAA(4,2) = 3*pow(t_e,2);
    AAA(4,3) = 2*pow(t_e,1);
    AAA(4,4) = 1;
    AAA(4,5) = 0;

    AAA(5,0) = 20*pow(t_e,3);
    AAA(5,1) = 12*pow(t_e,2);
    AAA(5,2) = 6*pow(t_e,1);
    AAA(5,3) = 2*1;
    AAA(5,4) = 0;
    AAA(5,5) = 0;

    VectorNd BBB(6);
    BBB[0] = h_0[0];//position
    BBB[1] = h_0[1];//velocity
    BBB[2] = h_0[2];//acc
    BBB[3] = h_e[0];//position
    BBB[4] = h_e[1];//velocity
    BBB[5] = h_e[2];//acc

    return (AAA.inverse())*BBB;
}
