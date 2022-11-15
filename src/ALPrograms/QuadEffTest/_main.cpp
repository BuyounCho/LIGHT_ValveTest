#include "BasicFiles/BasicSetting.h"

#include "QUAD_BasicFunction.h"
#include "QUAD_jointsetmodel.h"

#define PODO_AL_NAME       "QuadEffTest"

// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *joint;
JointControlClass       *jCon;

// Program variable
int isTerminated;
int     __IS_WORKING = false;
int     __IS_GAZEBO = false;

int     PODO_NO = -1;

// QUAD Model
QUADJointSet           QUAD;

bool            OPERATION_ONOFF = false;
int             OPERATION_PARA_INT[10] = {0,};
double          OPERATION_PARA_DOUBLE[10] = {0.0,};

bool             save_Flag = false;
unsigned int     save_Index = 0;
float            save_Buf[SAVEKIND][SAVENUM] = {0,};

int main(int argc, char *argv[])
{
    // =================== Variables initialize ===================


    // =================== Program initiated ===================
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, PODO_AL_NAME);

    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    jCon->SetMotionOwner(0);

    jCon->RefreshToCurrentReference();
    jCon->SetAllMotionOwner();

    // Reference Enable
    for(int i=0;i<MAX_MC;i++) {
        sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_PosOrFor;
    }
    FILE_LOG(logSUCCESS) << "Setting Complete!!";

    // =========================================================

    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){

        case QUAD_TASK_SAVE_DATA:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]) {
                save_Flag = true;
                FILE_LOG(logWARNING) << "SAVING AL DATA START!!";
            } else {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == NULL) {
                    save_File();
                } else {
                    save_File(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                }
                save_Flag = false;
                FILE_LOG(logERROR) << "SAVING AL DATA END!!";
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUAD_TASK_NO_ACT;
            break;
        }

        case QUAD_TASK_ARMMOTION:
        {
            int ONOFF = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            int TESTTYPE = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            if(TESTTYPE == 0) { // Joint Move
                double _T = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // Moving Time
                double _Q = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]*D2R; // Desired Angle
                double _M = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]; // Moving Time
                double _L = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3]; // Moving Time
                QUAD.SetParameter_ArmTask(0,_T,_Q,_M,_L);
                QUAD.SetArmOpeartion(QUAD.ARM_OPERATION_MoveJoint);
            } else if (TESTTYPE == 1) {  // Sine Reference
                double _N = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2]; // Repeated Number
                double _T = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // Moving Time
                double _Q = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]*D2R; // Desired Angle
                double _M = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]; // Moving Time
                double _L = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3]; // Moving Time
                QUAD.SetParameter_ArmTask(_N,_T,_Q,_M,_L);
                QUAD.SetArmOpeartion(QUAD.ARM_OPERATION_SineWave);
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUAD_TASK_NO_ACT;
            break;
        }

        case QUAD_TASK_HOMEPOSE:
        {
            VectorNd _Q = VectorNd::Zero(QUAD.n_dof);
            _Q << 0.0, 0.0;
            QUAD.SetParameter_JointSpaceMove(3.0,_Q);
            QUAD.SetOpeartion(QUAD.OPERATION_JointSpaceMove);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUAD_TASK_NO_ACT;
            break;
        }

        case QUAD_TASK_TESTJOINTS:
        {
            int ONOFF = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            int TESTTYPE = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
            if(TESTTYPE == 0) { // Joint Move
                double _I = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2]; // Target Joint
                double _T = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // Moving Time
                double _Q = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]*D2R; // Desired Angle
                QUAD.SetParameter_TestJoints(_I,0,_T,_Q);
                QUAD.SetOpeartion(QUAD.OPERATION_TestJoints_MoveJoint);
            } else if (TESTTYPE == 1) {  // Sine Reference
                double _I = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2]; // Target Joint
                double _N = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[3]; // Repeated Number
                double _T = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // Moving Time
                double _Q = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]*D2R; // Desired Angle
                QUAD.SetParameter_TestJoints(_I,_N,_T,_Q);
                QUAD.SetOpeartion(QUAD.OPERATION_TestJoints_SineWave);
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUAD_TASK_NO_ACT;
            break;
        }

        case QUAD_TASK_SQUAT:
        {
            int ONOFF = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            if(ONOFF) {                
                double _N = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
                double _T = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double _z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                Vector3d _X = Vector3d(0.0,0.0,_z);
                QUAD.SetParameter_SquatMotion(_T,_X,_N);
                QUAD.SetOpeartion(QUAD.OPERATION_Squat);
            } else {
                if(QUAD.CheckOpeartion() == QUAD.OPERATION_Squat) {
                    QUAD.TerminateOpeartion();
                } else {
                    FILE_LOG(logERROR) << "Squat Motion is not being excuted...";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUAD_TASK_NO_ACT;
            break;
        }

        case QUAD_TASK_SWING:
        {
            int ONOFF = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            if(ONOFF) {
                double _N = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];
                double _T = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double _X = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                double _Z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                QUAD.SetParameter_SwingMotion(_T,_X,_Z,_N);
                QUAD.SetOpeartion(QUAD.OPERATION_Swing);
            } else {
                if(QUAD.CheckOpeartion() == QUAD.OPERATION_Swing) {
                    QUAD.TerminateOpeartion();
                } else {
                    FILE_LOG(logERROR) << "Swing Motion is not being excuted...";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUAD_TASK_NO_ACT;
            break;
        }

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
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        system_clock::time_point StartTime = system_clock::now();

        // ============================================================

        QUAD.UpdateSensorData();
        QUAD.UpdateStates();

        system_clock::time_point EndTime1 = system_clock::now();

        // ============================================================

        // Arm Task
        bool _FINISH_ARM = false;
        switch(QUAD.CheckArmOpeartion())
        {
        case QUAD.ARM_OPERATION_MoveJoint:
        {
            _FINISH_ARM = QUAD.ArmTask_MoveJoint();
            break;
        }
        case QUAD.ARM_OPERATION_SineWave:
        {
            _FINISH_ARM = QUAD.ArmTask_SineWave();
            break;
        }
        default:
            break;
        }
        if (_FINISH_ARM == true){
            QUAD.ResetArmOpeartion();
        }

        // Quad's Leg Task
        bool _FINISH = false;
        switch(QUAD.CheckOpeartion())
        {
        case QUAD.OPERATION_JointSpaceMove:
        {
            _FINISH = QUAD.JointSpaceMove();
            break;
        }
        case QUAD.OPERATION_TestJoints_MoveJoint:
        {
            _FINISH = QUAD.TestJoints_MoveJoint();
            break;
        }
        case QUAD.OPERATION_TestJoints_SineWave:
        {
            _FINISH = QUAD.TestJoints_SineWave();
            break;
        }
        case QUAD.OPERATION_Squat:
        {
            _FINISH = QUAD.SquatMotion();
            break;
        }
        case QUAD.OPERATION_Swing:
        {
            _FINISH = QUAD.SwingMotion();
            break;
        }
        default:
            break;
        }

        if (_FINISH == true){
            QUAD.ResetOpeartion();
            QUAD.Flag_InvKin = false;
            QUAD.Flag_InvDyn = false;

            FILE_LOG(logERROR) << " Operation is finished!! ";
        }

        system_clock::time_point EndTime2 = system_clock::now();

        // ============================================================

        QUAD.InverseKinematics();
        QUAD.InverseKinematics_PumpReference();
        QUAD.InverseDynamics();

        system_clock::time_point EndTime3 = system_clock::now();

        // ============================================================

        QUAD.UpdateReferenceStates();
        QUAD.SetReference_Joint();
        QUAD.SetReference_Actuator();

        system_clock::time_point EndTime4 = system_clock::now();

        // ============================================================
        // Data Display

        microseconds t_StateEstim = duration_cast<std::chrono::microseconds>(EndTime1 - StartTime);
        microseconds t_MotionGene = duration_cast<std::chrono::microseconds>(EndTime2 - EndTime1);
        microseconds t_InvKinemat = duration_cast<std::chrono::microseconds>(EndTime3 - EndTime2);
        microseconds t_RefeUpdate = duration_cast<std::chrono::microseconds>(EndTime4 - EndTime3);

        static int CntDisplay = 0;
        if(CntDisplay%100==0) {
//            FILE_LOG(logINFO) << "Qdes_arm : " <<  QUAD.Qdes_arm.row(0).transpose()*R2D;

//            FILE_LOG(logINFO) << "Xdes : " <<  QUAD.Xdes_Foot.col(0).transpose();
//            FILE_LOG(logINFO) << "dXdes_Foot : " <<  QUAD.dXdes_Foot.transpose();
//            FILE_LOG(logINFO) << "ddXdes_Foot : " <<  QUAD.ddXdes_Foot.transpose();

//            FILE_LOG(logINFO) << "Xnow : " <<  QUAD.Xnow_Foot.transpose();
//            FILE_LOG(logINFO) << "Xref : " <<  QUAD.Xref_Foot.transpose();

//            FILE_LOG(logINFO) << "Qnow : " <<  QUAD.Qnow.transpose()*R2D;
            //            FILE_LOG(logINFO) << "Qref : " <<  QUAD.Qref.transpose()*R2D;
//            FILE_LOG(logINFO) << "dQref : " <<  QUAD.dQref.transpose()*R2D;
//            FILE_LOG(logINFO) << "Xref_Foot : " <<  QUAD.Xref_Foot.transpose();
//            FILE_LOG(logINFO) << "dXref_Foot : " <<  QUAD.dXref_Foot.transpose();

//            FILE_LOG(logINFO) << "Pl_arm : ";
//            for(int i = 0; i < sharedREF->N_window-1; i++) {
//                FILE_LOG(logINFO) << sharedREF->LoadPressureReference_Future[i][RHY];
//            }

            //            FILE_LOG(logDEBUG3) << "StateEstim Time : "<< t_StateEstim.count() <<" usec";
            //            FILE_LOG(logDEBUG3) << "MotionGene Time : "<< t_MotionGene.count() <<" usec";
            //            FILE_LOG(logDEBUG3) << "InvKinemat Time : "<< t_InvKinemat.count() <<" usec";
            //            FILE_LOG(logDEBUG3) << "RefeUpdate Time : "<< t_RefeUpdate.count() <<" usec";
            //            FILE_LOG(logDEBUG3) << "====================================";

            CntDisplay = 0;
        } CntDisplay++;

        // ============================================================
        // Data Saving
        if(save_Flag == true){
            save_PutData(save_Index);
            save_Index++;
            if(save_Index == (SAVENUM-1)) {
                save_Index=0;
            }
        }

        rt_task_suspend(&rtTaskCon);
    }
}
//==============================//


//==============================//
// Flag Thread
//==============================//
int CntFlagThread = 0;
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 40*1000);

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) { // Daemon : 500Hz, SeungHoon AL : 250Hz
            jCon->JointUpdate();
            if(CntFlagThread%2 == 1) {
                rt_task_resume(&rtTaskCon);
                CntFlagThread = 1;
            } else {
//                rt_task_resume(&rtTaskCon);
            }
            CntFlagThread++;
        }

//        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
//            jCon->JointUpdate();
//            rt_task_resume(&rtTaskCon);
//        }
    }

}

//==============================//
// Saving function
//==============================//
void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = SYS_DT;

    // RIGHT HIP YAW (Board 0)
    save_Buf[1][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentRefAngle;
    save_Buf[2][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentAngle;
    save_Buf[3][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentAngVel;
    save_Buf[4][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentRefTorque;
    save_Buf[5][cur_Index] = sharedSEN->ENCODER[RHR][0].CurrentTorque;

    // RIGHT HIP ROLL (Board 1)
    save_Buf[6][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefAngle;
    save_Buf[7][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentAngle;
    save_Buf[8][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentAngVel;
    save_Buf[9][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentRefTorque;
    save_Buf[10][cur_Index] = sharedSEN->ENCODER[RHP][0].CurrentTorque;

    // PUMP
    save_Buf[11][cur_Index] = sharedREF->PumpPressureReference[0];
    save_Buf[12][cur_Index] = sharedSEN->PUMP[0].CurrentPressure;
    save_Buf[13][cur_Index] = sharedSEN->PUMP[0].CurrentRefVelocity;
    save_Buf[14][cur_Index] = sharedSEN->PUMP[0].CurrentVelocity;

    save_Buf[15][cur_Index] = sharedSEN->PUMP[0].CurrentHydraPower;
    save_Buf[16][cur_Index] = sharedSEN->PUMP[0].CurrentMechaPower;

    save_Buf[21][cur_Index] = QUAD.Xdes_Foot.col(0)(0);
    save_Buf[22][cur_Index] = QUAD.Xdes_Foot.col(0)(1);
    save_Buf[23][cur_Index] = QUAD.Xdes_Foot.col(0)(2);
    save_Buf[24][cur_Index] = QUAD.Xnow_Foot(0);
    save_Buf[25][cur_Index] = QUAD.Xnow_Foot(1);
    save_Buf[26][cur_Index] = QUAD.Xnow_Foot(2);
}

