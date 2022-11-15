

#include "BasicFiles/BasicSetting.h"
#include "myvariables.h"
#include "ManualCAN.h"
#include <unistd.h>
#include "main.h"
#include "BasicMatrix.h"
//#include "cmatrix"

#include "js_rbdl.h"

#include "QuadProg++.hh"

//typedef techsoft::matrix<double> Matrix;
//typedef std::valarray<double> Vector;
//#include "math.h"

using namespace std;
//using namespace Eigen; // Seungwoo commented

// Basic --------
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
RBCORE_SHM_SENSOR       sharedSEN2;
pUSER_SHM               userData;

JointControlClass       *jCon;


enum EXTERNAL_TORQUE_ESTIMATION
{
    EXT_NO_ACT = 300,
    EXT_START,
    EXT_STOP,
    EXT_START2,
    EXT_STOP2,
    EXT_TEST,
    HAP_3CH_START,
    HAP_3CH_STOP,
    HAP_3CH_PARA_UPDATE
};

enum JTS_COMMAND
{
    JTS_NO_ACT = 400,
    JTS_SETMOVEJOINT,
    JTS_DATASAVE,
    JTS_OFFSETNULLING
};

enum BITELE_COMMAND
{
    BITELE_NO_ACT = 500,
    BITELE_EXT_TORQUE_ESTIMATION_START,
    BITELE_EXT_TORQUE_ESTIMATION_STOP,
    BITELE_SAVE_ONLY_START,
    BITELE_SAVE_ONLY_STOP,
    BITELE_POSITION_POSITION,
    BITELE_POSITION_POSITION_STOP,
    BITELE_SINE_TEST,
    BITELE_PARA_UPDATE,
    BITELE_POS_LOCK
};

enum HUMANOID_COMMAND
{
    HUMANOID_NO_ACT = 600,
    HUMANOID_0_TO_90,
    HUMANOID_90_TO_0,
    CT_RD_MOT
};

enum CYCLOID_COMMAND
{
    CYCLOID_NO_ACT = 50,
    CYCLOID_AL_LOCK,
    CYCLOID_SINE_POS_TEST,
    CYCLOID_VAR_SINE_POS_TEST,
    CYCLOID_SINE_STOP,
    CYCLOID_EVAL_MODE_POSITION,
    CYCLOID_EVAL_MODE_TORQUE,
    CYCLOID_EVAL_MODE_DYNAMO,
    CYCLOID_EVAL_MODE_PARAMETER,
    CYCLOID_EVAL_TORQUE_NULLING
};

bool test_stop = false;
float temp_pos = 0.0;

unsigned int rt_count2 = 0;
double Master_CurRef =0.0;
double _sum_slave = 0.;
double _sum_master = 0.;
double _sum = 0.;
double master_current_err = 0;
double master_current_err_sum = 0;
double master_current_Pgain = 0;
double master_current_Igain = 0;

double xs_hat = 0;
double err_torque = 0;
double Q_gain = 0;
double R_gain = 1;
double Ke_hat = 0;
double xe_hat = 0;
int flag_xe_update = 0;
double Eobs = 0;
double xmk = 0;
double xmk1 = 0;
double xsk = 0;
double xsk1 = 0;
double fmk = 0;
double fsk = 0;
double slave_position_reference = 0;
double master_force_referecne = 0;
double filtered_slave_ref = 0;
double beta_gain = 0;
double xf = 0;
double comp_pos_f = 0;
double comp_vel_f = 0;
double comp_acc_f = 0;

Vector<double> outX;
Matrix<double> G;
Vector<double> g0;
Matrix<double> CE;
Vector<double> ce0;
Matrix<double> CI;
Vector<double> ci0;
//CE^T x + ce0 = 0
//CI^T x + ci0 >= 0



struct SystemInfo MasterInfo;
struct SystemInfo SlaveInfo;
struct SystemInfo Cycloid;

int     __IS_WORKING = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;

//Vector calc_5th(double t_0, double t_e,vec3 h_0,vec3 h_e);
enum TUTORIAL_COMMAND
{
    TUTORIAL_NO_ACT = 100,
    TUTORIAL_FINGER_CONTROL
};

void FingerControl(char right_left, char finger, char current);

double UPDOWN_upL;
double UPDOWN_stepT;
double UPDOWN_upRatio;


//MatrixXd test_function(MatrixXd m1)
//{
//    MatrixXd mT(4,4);

//    mT = m1;

//    return mT;
//}

//MatrixXd calc_Jacob(MatrixXd q)
//{
//    MatrixXd J;



//    return J;
//}

JS_DYNAMICS JD;

int main(int argc, char *argv[])
{
    //rbdl test
    //    cout <<" init matrix and vector " << endl;
    //    MatrixNd tempH(JD.model->dof_count,JD.model->dof_count);
    //    VectorNd tempQ(JD.model->dof_count);

    //    cout << "when q1 is 0 " << endl;
    //    tempQ(0) = 0.0;

    //    CompositeRigidBodyAlgorithm(*(JD.model),tempQ,tempH,1);

    //    VectorNd Tau = VectorNd::Zero (JD.model->dof_count);
    //    VectorNd tempQdot = VectorNd::Zero (JD.model->dof_count);
    //    NonlinearEffects(*(JD.model),tempQ,tempQdot,Tau);

    //    cout << "Mass Matrix : " << tempH << endl;
    //    cout << "Nonliear Effect : " << Tau << endl;

    //    cout << "when q1 is 90 " << endl;
    //    tempQ(0) = PI/2.0;

    //    CompositeRigidBodyAlgorithm(*(JD.model),tempQ,tempH,1);

    //    Tau = VectorNd::Zero (JD.model->dof_count);
    //    tempQdot = VectorNd::Zero (JD.model->dof_count);
    //    NonlinearEffects(*(JD.model),tempQ,tempQdot,Tau);

    //    cout << "Mass Matrix : " << tempH << endl;
    //    cout << "Nonliear Effect : " << Tau << endl;


    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "Cycloid");


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

    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
        case CT_RD_MOT:
        {
            FILE_LOG(logSUCCESS) << "Command CT_RD_MOT received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            CT_AMP = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            CT_START_POS = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            CT_STAND_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            CT_SWING_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];
            CT_IMPACT_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4];
            CT_NUM_REPETITION = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[5];
            CT_ALPHA_SMOOTH = 0.0;

            cout << "amp : " << CT_AMP << " START POS : " << CT_START_POS << "TIME : " << CT_STAND_TIME << ", " << CT_SWING_TIME << ", " << CT_IMPACT_TIME << " # : " << CT_NUM_REPETITION << endl;

            rt_count = 0;
            saveIndex = 0;

            OPERATION_MODE = OPERATION_CT_RD_MOT;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HUMANOID_NO_ACT;
            break;


        }
        case HUMANOID_0_TO_90:
        {
            FILE_LOG(logSUCCESS) << "Command HUMANOID_0_TO_90 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();


            rt_count = 0;
            saveIndex = 0;

            OPERATION_MODE = OPERATION_HUMANOID_0_TO_90;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HUMANOID_NO_ACT;
            break;
        }
        case HUMANOID_90_TO_0:
        {
            FILE_LOG(logSUCCESS) << "Command HUMANOID_90_TO_0 received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            rt_count = 0;

            OPERATION_MODE = OPERATION_HUMANOID_90_TO_0;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = HUMANOID_NO_ACT;
            break;
        }


        case BITELE_POS_LOCK:
        {

            FILE_LOG(logSUCCESS) << "Command BITELE_EXT_TORQUE_ESTIMATION received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            SlaveInfo.est_torque = 0;
            MasterInfo.est_torque = 0;

            SlaveInfo.N_gear = 50;
            MasterInfo.N_gear = 4.6635;

            SlaveInfo.TorqueConst = 27.6e-3; // Nm/A
            MasterInfo.TorqueConst = 0.244;  // Nm/A

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]

            OPERATION_MODE = OPERATION_BITELE_POSITION_LOCK;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;
        }
        case BITELE_PARA_UPDATE:
        {
            FILE_LOG(logSUCCESS) << "Command HAP_3CH_PARA_UPDATE received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            BITELE_Pgain_Master = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            BITELE_Dgain_Master = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;

        }
        case BITELE_SINE_TEST:
        {
            FILE_LOG(logSUCCESS) << "Command BITELE_EXT_TORQUE_ESTIMATION received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            BITELE_SIN_AMP = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0]; // unit, [deg]
            BITELE_SIN_FREQ = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1]; // unit, [hz]

            rt_count = 0;

            SlaveInfo.est_torque = 0;
            MasterInfo.est_torque = 0;

            SlaveInfo.N_gear = 50;
            MasterInfo.N_gear = 4.6635;

            SlaveInfo.TorqueConst = 27.6e-3; // Nm/A
            MasterInfo.TorqueConst = 0.244;  // Nm/A

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]

            OPERATION_MODE = OPERATION_BITELE_SINE_TEST;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;
        }
        case BITELE_POSITION_POSITION_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command BITELE_EXT_TORQUE_ESTIMATION_STOP received..";


            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            //            sharedREF->COCOACurrent_REF[PODO_NO][0] = 0.0;

            OPERATION_MODE = OPERATION_NONE;

            rt_count = 0;
            rt_count2 = 0;
            SaveFile();

            //Master --> RHR (0), Slave --> RHP (1)
            //            jCon->SetMoveJoint(RHR, 0.0, 3000.0, MOVE_ABSOLUTE);
            //            jCon->SetMoveJoint(RHP, 0.0, 3000.0, MOVE_ABSOLUTE);
            sharedREF->COCOACurrent_REF[PODO_NO][0] = 0.0;
            sharedREF->COCOACurrent_REF[PODO_NO][1] = 0.0;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;
        }
        case BITELE_POSITION_POSITION:
        {
            FILE_LOG(logSUCCESS) << "Command BITELE_EXT_TORQUE_ESTIMATION received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            BITELE_SIN_AMP = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0]; // unit, [deg]
            BITELE_SIN_FREQ = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1]; // unit, [hz]

            saveIndex = 0;
            rt_count = 0;
            _sum_master = 0.0;
            _sum_slave = 0.0;
            _sum = 0.0;
            master_current_err = 0;
            master_current_err_sum = 0;

            Eobs = 0;
            Ke_hat = 0;
            xmk = 0;
            xmk1 = 0;
            xsk = 0;
            xsk1 = 0;
            fmk = 0;
            fsk = 0;


            master_current_Pgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            master_current_Igain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];

            SlaveInfo.est_torque = 0;
            MasterInfo.est_torque = 0;

            SlaveInfo.N_gear = 50;
            MasterInfo.N_gear = 4.6635;

            SlaveInfo.TorqueConst = 27.6e-3; // Nm/A
            MasterInfo.TorqueConst = 0.244;  // Nm/A

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]

            comp_pos_f = SlaveInfo.pos;
            comp_vel_f = 0;
            comp_acc_f = 0;

            filtered_slave_ref = SlaveInfo.pos;
            beta_gain = 0;
            xf = 0;
            OPERATION_MODE = OPERATION_BITELE_POSITION_POSITION;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;


        }
        case BITELE_SAVE_ONLY_START:
        {
            FILE_LOG(logSUCCESS) << "Command BITELE_SAVE_ONLY_START received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            saveIndex = 0;
            rt_count = 0;

            OPERATION_MODE = OPERATION_BITELE_SAVE_ONLY;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;

        }
        case BITELE_SAVE_ONLY_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command BITELE_SAVE_ONLY_STOP received..";


            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            OPERATION_MODE = OPERATION_NONE;

            rt_count = 0;
            SaveFile();

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;
        }
        case BITELE_EXT_TORQUE_ESTIMATION_START:
        {
            FILE_LOG(logSUCCESS) << "Command BITELE_EXT_TORQUE_ESTIMATION received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            BITELE_SIN_AMP = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0]; // unit, [deg]
            BITELE_SIN_FREQ = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1]; // unit, [hz]

            saveIndex = 0;
            rt_count = 0;
            _sum_master = 0.0;
            _sum_slave = 0.0;
            _sum = 0.0;
            master_current_err = 0;
            master_current_err_sum = 0;

            Eobs = 0;
            Ke_hat = 0;
            xmk = 0;
            xmk1 = 0;
            xsk = 0;
            xsk1 = 0;
            fmk = 0;
            fsk = 0;

            master_current_Pgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            master_current_Igain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];

            SlaveInfo.est_torque = 0;
            MasterInfo.est_torque = 0;

            SlaveInfo.N_gear = 50;
            MasterInfo.N_gear = 4.6635;

            SlaveInfo.TorqueConst = 27.6e-3; // Nm/A
            MasterInfo.TorqueConst = 0.244;  // Nm/A

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]

            filtered_slave_ref = SlaveInfo.pos;
            beta_gain = 0;
            xf = 0;
            OPERATION_MODE = OPERATION_BITELE_EXT_TORQUE_ESTIMATION;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;
        }
        case BITELE_EXT_TORQUE_ESTIMATION_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command BITELE_EXT_TORQUE_ESTIMATION_STOP received..";


            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            //            sharedREF->COCOACurrent_REF[PODO_NO][0] = 0.0;

            OPERATION_MODE = OPERATION_NONE;

            rt_count = 0;
            rt_count2 = 0;
            SaveFile();


            //            jCon->SetMoveJoint(RHP, 0.0, 3000.0, MOVE_ABSOLUTE);
            sharedREF->COCOACurrent_REF[PODO_NO][0] = 0.0;
            sharedREF->COCOACurrent_REF[PODO_NO][1] = 0.0;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = BITELE_NO_ACT;
            break;

        }


        case JTS_OFFSETNULLING:
        {
            FILE_LOG(logSUCCESS) << "Command JTS_OFFSETNULLING received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            saveIndex = 0;
            rt_count = 0;

            JTS_SENSOR_1_OFFSET = 0.0;
            JTS_SENSOR_2_OFFSET = 0.0;

            OPERATION_MODE = OPERATION_JTS_OFFSETNULLING;


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = JTS_NO_ACT;
            break;

        }
        case JTS_DATASAVE:
        {

            FILE_LOG(logSUCCESS) << "Command JTS_DATASAVE received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            OPERATION_MODE = OPERATION_NONE;

            rt_count = 0;
            SaveFile();

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = JTS_NO_ACT;
            break;

        }
        case JTS_SETMOVEJOINT:
        {
            FILE_LOG(logSUCCESS) << "Command JTS_SETMOVEJOINT received..";
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            JTS_DESIRED_ANGLE = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0]; // unit, [deg]
            JTS_DESIRED_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1]; // unit, [s]

            saveIndex = 0;
            rt_count = 0;

            OPERATION_MODE = OPERATION_JTS_SETMOVEJOINT;

            //            jCon->SetMoveJoint(RHR, JTS_DESIRED_ANGLE, JTS_DESIRED_TIME*1000, MOVE_ABSOLUTE);

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = JTS_NO_ACT;
            break;
        }
        case EXT_START:
        {
            FILE_LOG(logSUCCESS) << "Command EXT_START received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            JS_CURRENT_REF_AMP = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            JS_CURRENT_REF_FREQ = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];

            rt_count = 0;
            saveIndex = 0;
            Master_Pos = sharedSEN->ENCODER[1][0].CurrentPosition; //deg (reference)
            lastVel = 0.0;

            OPERATION_MODE = OPERATION_EXT_SIN;

            //            jCon->SetMoveJoint(LHP, 90.0f, 2000.0, MOVE_RELATIVE);

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;

        }
        case EXT_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command EXT_STOP received..";


            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedREF->COCOACurrent_REF[PODO_NO][0] = 0.0;

            OPERATION_MODE = OPERATION_NONE;

            rt_count = 0;
            rt_count2 = 0;
            SaveFile();


            //            jCon->SetMoveJoint(LHP, 0.0, 3000.0, MOVE_ABSOLUTE);


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;

        }
        case EXT_TEST:
        {
            FILE_LOG(logSUCCESS) << "Command EXT_TEST received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            JS_CURRENT_REF_AMP = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            JS_CURRENT_REF_FREQ = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];

            rt_count = 0;
            saveIndex = 0;
            rt_count2 = 0;
            lastVel = 0.0;
            OPERATION_MODE = OPERATION_EXT_TEST;

            //            jCon->SetMoveJoint(LHP, 90.0f, 2000.0, MOVE_RELATIVE);

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case EXT_START2:
        {
            FILE_LOG(logSUCCESS) << "Command EXT_START2 received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            rt_count = 0;
            saveIndex = 0;
            OPERATION_MODE = OPERATION_EXT_SIN2;

            //            jCon->SetMoveJoint(LHP, 90.0f, 2000.0, MOVE_RELATIVE);

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;

        }
        case HAP_3CH_START:
        {
            FILE_LOG(logSUCCESS) << "Command HAP_3CH_START received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            HAP_3ch_Pgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            HAP_3ch_Dgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];


            temp_gain_master = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            temp_gain_slave = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];

            //            pos_force_ratio = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];

            cout << "P gain : " << HAP_3ch_Pgain << "D gain : " << HAP_3ch_Dgain << endl;
            cout << "temp gain master: " << temp_gain_master << " temp gain slave : " << temp_gain_slave << endl;
            rt_count = 0;
            saveIndex = 0;

            passivity_obs = 0.0;
            Master_CurRef = 0.0;
            _sum = 0.0;

            OPERATION_MODE = OPERATION_HAP_3CH;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;

        }
        case HAP_3CH_PARA_UPDATE:
        {
            FILE_LOG(logSUCCESS) << "Command HAP_3CH_PARA_UPDATE received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            HAP_3ch_Pgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            HAP_3ch_Dgain = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            pos_force_ratio = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;

        }
        case HAP_3CH_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command HAP_3CH_STOP received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            sharedREF->COCOACurrent_REF[PODO_NO][1] = 0.0;

            OPERATION_MODE = OPERATION_NONE;

            rt_count = 0;
            rt_count2 = 0;
            SaveFile();

            jCon->SetMoveJoint(LHP, 0.0, 3000.0, MOVE_ABSOLUTE);


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;

        }


        case QUBO_GOTO_POS:
        {
            FILE_LOG(logSUCCESS) << "Command QUBO_GOTO_POS received..";

            float hp_ref = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float kn_ref = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float time = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];

            if(hp_ref < LIM_HP_L)
                hp_ref = LIM_HP_L;
            if(hp_ref > LIM_HP_U)
                hp_ref = LIM_HP_U;
            if(kn_ref < LIM_KN_L)
                kn_ref = LIM_KN_L;
            if(kn_ref > LIM_KN_U)
                kn_ref = LIM_KN_U;

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            jCon->SetMoveJoint(LHP, hp_ref, time, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, kn_ref, time, MOVE_ABSOLUTE);
            //            jCon->

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_SAVE:
        {
            SaveFile();
            printf("File is saved...!!!\n");
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_MOVE_TRAPEZOIDAL:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command QUBO_MOVE_TRAPEZOIDAL received..";
            
            float x_ref = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float y_ref = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float time = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            float vmax = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];
            
            POS_X = x_ref;
            POS_Y = y_ref;
            TIME = time;
            VMAX = vmax;
            
            LegJoint Lj;
            Lj = InverseKinematics(POS_X,POS_Y);
            

            rt_count = 0;
            OPERATION_MODE = OPERATION_TRAPEZOIDAL;
            
            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(LHP, Lj.S1*100, TIME, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, -Lj.S2*100, TIME, MOVE_ABSOLUTE);
            
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_MOVE_JUMP:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command QUBO_MOVE_JUMP received..";

            float decel_1 = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float decel_2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float take_off_pos = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            float start_pos = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];
            float take_off_vel = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4];
            float landing_vel = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[5];
            float period = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[6];
            float stop_time = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[7];
            float cycle_num = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[8];
            int jump_mode = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];


            LegJoint Lj;


            DECEL_1 = decel_1;
            DECEL_2 = decel_2;
            TAKE_OFF_POS = take_off_pos;
            START_POS = start_pos;
            TAKE_OFF_VEL = take_off_vel;
            LANDING_VEL = landing_vel;
            PERIOD = period;
            STOP_TIME = stop_time;
            CYCLE_NUM = cycle_num;
            JUMP_MODE = jump_mode;
            ITERATION = 0;

            Lj = InverseKinematics(0,-START_POS);

            rt_count = 0;

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            jCon->SetMoveJoint(LHP, Lj.S1*100, 1000, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, -Lj.S2*100, 1000, MOVE_ABSOLUTE);

            OPERATION_MODE = OPERATION_JUMP;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_MOVE_SPRINGMASSMODEL:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command QUBO_MOVE_SPRINGMASSMODEL received..";

            float spring_constant = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float initial_position = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float initial_velocity = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            float delay_time = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];
            float aerial_seg_num = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4];
            float max_iteration = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[5];

            SPRING_CONSTANT = spring_constant;
            INITIAL_POS = initial_position;
            INITIAL_VEL = initial_velocity;
            DELAY_TIME = delay_time;
            AERIAL_SEG_NUM = aerial_seg_num;
            MAX_ITERATION = max_iteration;
            ITERATION = 0;

            SpringMassModel(SPRING_CONSTANT, INITIAL_POS, INITIAL_VEL, DELAY_TIME, AERIAL_SEG_NUM, 0);
            LegJoint Lj;
            //Lj = InverseKinematics(0,-FLOOR_POS);
            Lj = InverseKinematics(0,-INITIAL_POS);

            rt_count = 0;

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            jCon->SetMoveJoint(LHP, Lj.S1*100, 1000, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, -Lj.S2*100, 1000, MOVE_ABSOLUTE);

            OPERATION_MODE = OPERATION_SPRINGMASSMODEL;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_MOVE_SINE:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command QUBO_MOVE_SINE received..";

            float sin_initial_position = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float sin_magnitude = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float sin_frequency = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            float sin_cycle_num = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];

            SINE_INIT_POS = sin_initial_position;
            SINE_MAG = sin_magnitude;
            SINE_FREQ = 2*PI*sin_frequency;
            SINE_TOTAL_COUNT = (1/sin_frequency)*200*sin_cycle_num;

            LegJoint Lj;
            Lj = InverseKinematics(0,-SINE_INIT_POS);

            rt_count = 0;

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            jCon->SetMoveJoint(LHP, Lj.S1*100, 1000, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, -Lj.S2*100, 1000, MOVE_ABSOLUTE);

            //usleep(1100000);


            OPERATION_MODE = OPERATION_SINE;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_GAIN_OVERRIDE_FRICTION_COMPENSATION:
        {
            FILE_LOG(logSUCCESS) << "Command QUBO_GAIN_OVERRIDE_FRICCOM received..";
            double g_o = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            //override
            //MCJointGainOverride(JOINT_INFO[LKN].canch,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,100);
            MCJointGainOverride(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,100);
            MCBoardSetSwitchingMode(0, JOINT_INFO[LKN].bno, 0x01);//non_complementary
            int a_c = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            short v_s = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            int v_d = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
            //MCsetFrictionParameter(JOINT_INFO[LKN].canch,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,v_s,a_c,v_d);
            MCsetFrictionParameter(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,v_s,a_c,v_d);
            //enable
            //MCenableFrictionCompensation(JOINT_INFO[LKN].canch,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,1);
            MCenableFrictionCompensation(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,1);
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_GAIN_OVERRIDE_FRICTION_COMPENSATION_OFF:
        {
            FILE_LOG(logSUCCESS) << "Command QUBO_GAIN_OVERRIDE_FRICCOM_OFF received..";

            //disable
            //MCenableFrictionCompensation(JOINT_INFO[LKN].canch,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,0);
            MCenableFrictionCompensation(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,0);
            double g_o = 0;
            MCBoardSetSwitchingMode(0, JOINT_INFO[LKN].bno, 0x00);//complementary
            //MCJointGainOverride(JOINT_INFO[LKN].canch,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,1000);
            MCJointGainOverride(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,1000);

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_UPDOWN_TEST_START:
        {
            FILE_LOG(logSUCCESS) << "Command QUBO_UPDOWN_TEST_START received..";
            double x_init = 0;
            double y_init = -300;//mm
            TIME = 3000;

            LegJoint Lj;
            Lj = InverseKinematics(x_init,y_init);


            rt_count = 0;
            OPERATION_MODE = OPERATION_TRAPEZOIDAL;

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(LHP, Lj.S1*100, TIME, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, -Lj.S2*100, TIME, MOVE_ABSOLUTE);

            while(OPERATION_MODE ==OPERATION_TRAPEZOIDAL)
            {
                usleep(100*1000);
            }

            rt_count = 0;
            saveIndex = 0;
            UPDOWN_upL = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
            UPDOWN_stepT = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
            UPDOWN_upRatio = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
            OPERATION_MODE = OPERATION_UPDOWN;

            FILE_LOG(logSUCCESS) << "Command QUBO_UPDOWN_TEST_START done....";

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case QUBO_UPDOWN_TEST_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command QUBO_UPDOWN_TEST_STOP received..";
            OPERATION_MODE = OPERATION_NONE;
            saveFlag = 0;
            SaveFile();
            printf("finished 2...!!!\n");

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }


        case CYCLOID_AL_LOCK:
        {

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]==0){
                FILE_LOG(logSUCCESS) << "Command LOCK_IN_PLACE received..";

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

                FILE_LOG(logSUCCESS) << "4: UPDATE REF";
                for(int i=0;i<NO_OF_JOINTS;i++)
                {
                    double enc = sharedSEN->ENCODER[MC_ID_CH_Pairs[i].id][MC_ID_CH_Pairs[i].ch].CurrentPosition;
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

                FILE_LOG(logSUCCESS) << "6: FOC-NULLING ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//CONTROL ON
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_FOC_NULLING;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(2*1000000);
                }
                usleep(50*1000);
            }

            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]==1){
                FILE_LOG(logSUCCESS) << "7: FOC-POSITION CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//CONTROL ON
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_POSITION;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
            }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]==2){
                FILE_LOG(logSUCCESS) << "7: FOC-CURRENT CONTROL ON";
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//CONTROL ON
                sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_INIT_CONTROL_ONOFF_FOC_CURRENT;
                while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
                {
                    usleep(50*1000);
                }
                usleep(50*1000);
            }

            FILE_LOG(logSUCCESS) << "8: TORQUE SENSOR NULLING ON";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0] = -1;//all
//            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2] = 1;//CONTROL ON
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON_COCOA_TORQUE_NULLING;
            while(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND!=NO_ACT)
            {
                usleep(50*1000);
            }
            usleep(50*1000);

            //ROLL_FINDHOME SHOULD BE ADDED
            usleep(2000*1000);
            FILE_LOG(logSUCCESS) << "CYCLOID_AL_LOCK_DONE";


            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = NO_ACT;
            break;
        }
        case CYCLOID_SINE_POS_TEST:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command CYCLOID_SINE_TEST received..";

            float sin_initial_position = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float sin_magnitude = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float sin_frequency = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            unsigned int sin_cycle_num = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];

            SINE_INIT_POS = sin_initial_position;
            SINE_MAG = sin_magnitude;
            SINE_FREQ = 2*PI*sin_frequency;
//            SINE_FREQ = PI*sin_frequency; // For (1-cos) curve
            SINE_TOTAL_COUNT = (1/sin_frequency)*500*sin_cycle_num; // T = cycle_num*(x*2ms)

            cout << "SINE_INIT_POS : " << SINE_INIT_POS << " SINE_MAG :" << SINE_MAG << " SINE_FREQ :" << SINE_FREQ << " SINE_TOTAL_COUNT :" << SINE_TOTAL_COUNT << endl;

            jCon->RefreshToCurrentReference();                       // Seungwoo commented
            jCon->SetAllMotionOwner();                               // Seungwoo commented
            jCon->SetMoveJoint(RHR, SINE_INIT_POS, 500, MOVE_ABSOLUTE);

            usleep(0.6*1000000);

            rt_count = 0;

            OPERATION_MODE = OPERATION_CYCLOID_SINE_POS_TEST;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case CYCLOID_VAR_SINE_POS_TEST:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command CYCLOID_VAR_SINE_POS_TEST received..";

            float sin_initial_position = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float sin_magnitude = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float sin_initial_frequency = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            float sin_final_frequency = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];
            float sin_time_duration = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4];

            VAR_SINE_INIT_POS = sin_initial_position;
            VAR_SINE_MAG = sin_magnitude;
            VAR_SINE_INIT_FREQ = 2*PI*sin_initial_frequency;
            VAR_SINE_FINAL_FREQ = 2*PI*sin_final_frequency;
            VAR_SINE_TIME = sin_time_duration;
            VAR_SINE_TOTAL_COUNT = sin_time_duration*500;

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();
            jCon->SetMoveJoint(RHR, VAR_SINE_INIT_POS, 500, MOVE_ABSOLUTE);

            usleep(0.6*1000000);

            rt_count = 0;

            OPERATION_MODE = OPERATION_CYCLOID_VAR_SINE_POS_TEST;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case CYCLOID_SINE_STOP:
        {
            FILE_LOG(logSUCCESS) << "Command CYCLOID_SINE_STOP received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            sharedREF->COCOACurrent_REF[PODO_NO][RHR] = 0.0;
            sharedREF->COCOAQCurrent_REF[PODO_NO][RHR] = 0.0;
            sharedREF->COCOADCurrent_REF[PODO_NO][RHR] = 0.0;

            OPERATION_MODE = OPERATION_NONE;

//            test_stop = true;

            rt_count = 0;

            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_NO_ACT;
            SaveFile();

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case CYCLOID_EVAL_MODE_POSITION:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command CYCLOID_EVAL_MODE_POSITION received..";

            float pos_delta = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            float pos_time = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];
            float pos_cur = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;

            //// For (1-cos) Curve
            SINE_INIT_POS = pos_cur;
            SINE_MAG = pos_delta;
            SINE_FREQ = PI/pos_time;
//            SINE_FREQ = 1.0/pos_time;
            SINE_TOTAL_COUNT = 1.0/SINE_FREQ*500; // T = (x*2ms)

            cout << "SINE_INIT_POS : " << SINE_INIT_POS << " SINE_MAG :" << SINE_MAG << " SINE_FREQ :" << SINE_FREQ << " SINE_TOTAL_COUNT :" << SINE_TOTAL_COUNT << endl;

            jCon->RefreshToCurrentReference();                       // Seungwoo commented
            jCon->SetAllMotionOwner();                               // Seungwoo commented
            jCon->SetMoveJoint(RHR, SINE_INIT_POS, 500, MOVE_ABSOLUTE);

            usleep(0.6*1000000);

            rt_count = 0;

            OPERATION_MODE = OPERATION_CYCLOID_EVAL_POSITION;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case CYCLOID_EVAL_MODE_TORQUE:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command CYCLOID_EVAL_MODE_TORQUE received..";

            EVAL_MODE = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
            TORQUE_CONSTANT = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0];
            BACK_EMF = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1];

            if(EVAL_MODE==1){
                REF_CURRENT_Q = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
                REF_CURRENT_D = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3];
            }else if(EVAL_MODE==2){
                REF_TORQUE = sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2];
            }else{
                FILE_LOG(logERROR) << "Undefined Evaluation Mode";
            }

            jCon->RefreshToCurrentReference();                       // Seungwoo commented
            jCon->SetAllMotionOwner();                               // Seungwoo commented

            rt_count = 0;

            OPERATION_MODE = OPERATION_CYCLOID_EVAL_TORQUE;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case CYCLOID_EVAL_MODE_DYNAMO:
        {
            saveFlag = 1;
            FILE_LOG(logSUCCESS) << "Command CYCLOID_EVAL_MODE_DYNAMO received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            OPERATION_MODE = OPERATION_CYCLOID_EVAL_DYNAMO;

            rt_count = 0;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
            break;
        }
        case CYCLOID_EVAL_TORQUE_NULLING:
        {
            FILE_LOG(logSUCCESS) << "Command CYCLOID_EVAL_TORQUE_NULLING received..";

            jCon->RefreshToCurrentReference();
            jCon->SetAllMotionOwner();

            OPERATION_MODE = OPERATION_CYCLOID_EVAL_TORQUE_NULLING;

            rt_count = 0;

            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = QUBO_NO_ACT;
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
bool Foot_Landed;
double y_landed, y_landed2;
double landTime;
double landUp =0;//10mm
bool GOflag;
double KNEE_old;
double KneeV;
double y_upspeed = 0;
double ycon = 0;
double dF = 0;
double dF_f = 0;
double alpha = 0.95;
double old_F = 0;

double F_ref = 90;
double T = 0.3*5.5*100;
double kp = 7.0/1000.0*16;
double kd = kp*0.8;

MatrixNd _H = MatrixNd::Zero(JD.model->dof_count,JD.model->dof_count);
VectorNd _Q = VectorNd::Zero(JD.model->dof_count);
VectorNd _QDot = VectorNd::Zero(JD.model->dof_count);
VectorNd _QDotzero = VectorNd::Zero(JD.model->dof_count);
VectorNd _QDDot = VectorNd::Zero(JD.model->dof_count);
VectorNd Cqdot = VectorNd::Zero (JD.model->dof_count);
VectorNd g = VectorNd::Zero (JD.model->dof_count);
VectorNd N = VectorNd::Zero (JD.model->dof_count);
VectorNd p = VectorNd::Zero (JD.model->dof_count);
VectorNd est_torque_master = VectorNd::Zero (JD.model->dof_count);
VectorNd est_torque_slave = VectorNd::Zero (JD.model->dof_count);
VectorNd est_torque = VectorNd::Zero (JD.model->dof_count);


double max_sensor1 = 0;
double min_sensor1 =9999;

double max_sensor2 = 0;
double min_sensor2 =9999;

void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        //        cout << "TORQUE SENSOR1 : " << sharedSEN->ENCODER[1][0].Cocoa_Data.TORQUE_SENSOR1 << " Slave pos :" << sharedSEN->ENCODER[1][0].CurrentPosition/50.0 << endl;

        //        cout << "TORQUE SENSOR : " << BITELE_JTS << endl;
        //        BITELE_JTS = ((double)sharedSEN->ENCODER[1][0].Cocoa_Data.TORQUE_SENSOR1 - JTS_SENSOR_1_OFFSET)*0.0738;

        switch (OPERATION_MODE) {

        case OPERATION_CT_RD_MOT:
        {
            double temp_double = 0;
            double step_final = CT_NUM_REPETITION * (CT_STAND_TIME + CT_SWING_TIME + CT_IMPACT_TIME)*500.0;

            double step_stand_time = CT_STAND_TIME*500.0;
            double step_swing_time = CT_SWING_TIME*500.0;
            double step_impact_time = CT_IMPACT_TIME*500.0;

            temp_double = CT_STAND_TIME*500.0/3.0;
            if(rt_count < temp_double)
            {
                    CT_ALPHA_SMOOTH += 1/temp_double;
                    if(CT_ALPHA_SMOOTH >= 1)
                        CT_ALPHA_SMOOTH = 1;
            }

            else if(rt_count > step_final - temp_double)
            {
                CT_ALPHA_SMOOTH -= 1/temp_double;\

                if(CT_ALPHA_SMOOTH <= 0)
                    CT_ALPHA_SMOOTH = 0;
            }

            else
                CT_ALPHA_SMOOTH = 1;

            cout << "rt_time : " << rt_count << "final time :" << step_final/500 << "alpha : " << CT_ALPHA_SMOOTH << endl;


            int step_cycle = rt_count % (int)((CT_STAND_TIME + CT_SWING_TIME + CT_IMPACT_TIME)*500.0);


            double ref_pos = 0;

            if(step_cycle < step_stand_time)
            {
                ref_pos = CT_ALPHA_SMOOTH * (CT_START_POS);
            }
            else if(step_cycle < step_stand_time + step_swing_time)
            {
                ref_pos = CT_ALPHA_SMOOTH * (CT_START_POS - (CT_AMP+CT_START_POS)*sin((step_cycle-step_stand_time)/step_swing_time*PI));
            }
            else
            {
                ref_pos = CT_ALPHA_SMOOTH * (CT_START_POS + CT_AMP*5.0*sin((step_cycle-step_stand_time-step_swing_time)/step_impact_time*PI));
            }

            jCon->SetMoveJoint(RHR, ref_pos, 2.0, MOVE_ABSOLUTE);   //Master --> RHR (0)

            DataBuf[0][saveIndex]= rt_count/500.0; //time [s]
            DataBuf[1][saveIndex]= ref_pos; //reference [deg]
            DataBuf[2][saveIndex]= sharedSEN->ENCODER[0][0].CurrentPosition; //enc [deg]
            DataBuf[3][saveIndex]= sharedSEN->ENCODER[0][0].CurrentCurrent; //current [A]
            DataBuf[4][saveIndex]= CT_ALPHA_SMOOTH;

            ////  end of motion command, save index ++

            if(saveIndex > 150000-2)
                saveIndex = 150000-2;
            else
                saveIndex++;


            if(rt_count >= step_final-1)
                OPERATION_MODE = OPERATION_NONE;

            break;

        }
        case OPERATION_HUMANOID_0_TO_90:
        {

            double temp_t = rt_count/500.0;
            double tempRef;

            tempRef = 60.4184*temp_t*temp_t*temp_t -29.2380*temp_t*temp_t*temp_t*temp_t + 3.7701*temp_t*temp_t*temp_t*temp_t*temp_t;
            tempRef *= -160.0;

            jCon->SetMoveJoint(RHR, tempRef, 2.0, MOVE_ABSOLUTE);   //Master --> RHR (0)


            DataBuf[0][saveIndex]= rt_count/500.0; //time [s]
            DataBuf[1][saveIndex]= tempRef; //reference [deg]
            DataBuf[2][saveIndex]= sharedSEN->ENCODER[0][0].CurrentPosition; //input encoder [deg]
            DataBuf[3][saveIndex]= sharedSEN->ENCODER[1][0].CurrentPosition; //output encoder [deg]
            DataBuf[4][saveIndex]= sharedSEN->ENCODER[0][0].CurrentCurrent; //input encoder [deg]
            DataBuf[5][saveIndex]= sharedSEN->ENCODER[1][0].CurrentCurrent; //output encoder [deg]

            ////  end of motion command, save index ++
            saveIndex++;

            if(temp_t >= 3.0)
                OPERATION_MODE = OPERATION_NONE;

            break;
        }
        case OPERATION_HUMANOID_90_TO_0:
        {

            double temp_t = rt_count/500.0;
            double tempRef;

            temp_t = 3.0 - temp_t;

            tempRef = 60.4184*temp_t*temp_t*temp_t -29.2380*temp_t*temp_t*temp_t*temp_t + 3.7701*temp_t*temp_t*temp_t*temp_t*temp_t;
            tempRef *= -160.0;

            jCon->SetMoveJoint(RHR, tempRef, 2.0, MOVE_ABSOLUTE);   //Master --> RHR (0)

            DataBuf[0][saveIndex]= rt_count/500.0; //time [s]
            DataBuf[1][saveIndex]= tempRef; //reference [deg]
            DataBuf[2][saveIndex]= sharedSEN->ENCODER[0][0].CurrentPosition; //input encoder [deg]
            DataBuf[3][saveIndex]= sharedSEN->ENCODER[1][0].CurrentPosition; //output encoder [deg]
            DataBuf[4][saveIndex]= sharedSEN->ENCODER[0][0].CurrentCurrent; //input encoder [deg]
            DataBuf[5][saveIndex]= sharedSEN->ENCODER[1][0].CurrentCurrent; //output encoder [deg]



            ////  end of motion command, save index ++
            saveIndex++;

            if(temp_t <= 0.0)
                OPERATION_MODE = OPERATION_NONE;


            break;
        }

        case OPERATION_BITELE_SINE_TEST:
        {
            double tempRef = 0.0;
            double alpha = 0;
            double tempPWM = 0.0;

            //// for estimating the friction params

            if(rt_count < 250)
                alpha = rt_count/250.0;
            else if(rt_count < 500*10-250)
                alpha = 1;
            else if(rt_count < 500*10)
                alpha = (500*10-rt_count)/250.0;
            else
            {
                sharedREF->COCOACurrent_REF[PODO_NO][1] = 0.0;

                OPERATION_MODE = OPERATION_NONE;

                cout << "Finish!" << endl;
                break;
            }

            //            for position
            tempRef = alpha*BITELE_SIN_AMP*sin(2.0*PI*(float)rt_count*BITELE_SIN_FREQ/500.0) * PI/180.0; //pos [rad] output

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[1][0].PWMin;

            DataBuf[0][saveIndex]= tempRef; //position reference [rad], output
            DataBuf[1][saveIndex]= SlaveInfo.cur; //cur [A]
            DataBuf[2][saveIndex]= SlaveInfo.vel; //vel [rad/s]
            DataBuf[3][saveIndex]= SlaveInfo.pos; //pos [rad]
            DataBuf[4][saveIndex]= tempPWM; //PWM [unit]

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[0][0].PWMin;

            DataBuf[0+6][saveIndex]= tempRef; //position reference [rad], output
            DataBuf[1+6][saveIndex]= MasterInfo.cur; //cur [A]
            DataBuf[2+6][saveIndex]= MasterInfo.vel; //vel [rad/s]
            DataBuf[3+6][saveIndex]= MasterInfo.pos; //pos [rad]
            DataBuf[4+6][saveIndex]= tempPWM; //PWM [unit]

            //////////////////////


            tempRef = 200*(tempRef - SlaveInfo.pos) + 1.0*(0  - SlaveInfo.vel);

            if(tempRef > 10)
                tempRef = 10;
            else if(tempRef < -10)
                tempRef = -10;

            sharedREF->COCOACurrent_REF[PODO_NO][1] = tempRef;
            DataBuf[11][saveIndex]= tempRef; //Motor input [A]

            ////  end of motion command, save index ++
            saveIndex++;

            break;
        }
        case OPERATION_BITELE_POSITION_POSITION:
        {
            double tempPWM;
            double tempRef;

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[1][0].PWMin;

            //            DataBuf[0][saveIndex]= tempRef*PI/180.0/SlaveInfo.N_gear; //position reference [rad], output
            DataBuf[0][saveIndex]= tempRef; //position reference [rad], output
            DataBuf[1][saveIndex]= SlaveInfo.cur; //cur [A]
            DataBuf[2][saveIndex]= SlaveInfo.vel; //vel [rad/s]
            DataBuf[3][saveIndex]= SlaveInfo.pos; //pos [rad]
            DataBuf[4][saveIndex]= tempPWM; //PWM [unit]

            //            -0.2938   -1.0916   -1.2401    0.6636

            //            JD.calc_est_torque();

            double II;
            II = ((33.3e-3*1e-4 + 0.054e-4 + 608.97e-3*1e-6)*SlaveInfo.N_gear*SlaveInfo.N_gear + 440321e-3*1e-6);
            _Q(0) = SlaveInfo.pos;
            _QDot(0) = SlaveInfo.vel;

            CompositeRigidBodyAlgorithm(*(JD.model),_Q,_H,1);

            _H(0,0) += II*0.6636;   //calibrated value, 20170731

            NonlinearEffects(*(JD.model),_Q,_QDot,N);
            NonlinearEffects(*(JD.model),_Q,_QDotzero,g);

            Cqdot = N-g;

            p = _H*_QDot;

            double K = 2*PI*10;

            double fric = 0;

            fric = -0.2938*SlaveInfo.vel + tanh(SlaveInfo.vel/-1.0916)*(-1.2401);

            _sum_slave = _sum_slave*0.975 + (SlaveInfo.cur*SlaveInfo.N_gear*SlaveInfo.TorqueConst - fric + Cqdot(0) - g(0) - est_torque_slave(0))*0.002;

            est_torque_slave(0) = K*(_sum_slave - p(0));

            SlaveInfo.est_torque = SlaveInfo.est_torque*0.7 - est_torque_slave(0)*0.3;

            DataBuf[5][saveIndex]= SlaveInfo.est_torque; //external torque, output, [Nm]


            /////////////////////////////////   Master side, human torque estimation
            //            -0.3291    0.2669    1.2099    0.6594

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[0][0].PWMin;

            DataBuf[6][saveIndex]= tempRef*PI/180.0/MasterInfo.N_gear; //position reference [rad], output
            DataBuf[7][saveIndex]= MasterInfo.cur; //cur [A]
            DataBuf[8][saveIndex]= MasterInfo.vel; //vel [rad/s]
            DataBuf[9][saveIndex]= MasterInfo.pos; //pos [rad]
            DataBuf[10][saveIndex]= tempPWM; //PWM [unit]
            \
            ///////////////////////////////////////// compensated positino calc

            //            RBDL_DLLAPI void ForwardDynamics 	( 	Model &  	model,
            //                    const Math::VectorNd &  	Q,
            //                    const Math::VectorNd &  	QDot,
            //                    const Math::VectorNd &  	Tau,
            //                    Math::VectorNd &  	QDDot,
            //                    std::vector< Math::SpatialVector > *  	f_ext = NULL
            //                )

            //            _Q(0) = SlaveInfo.pos;
            //            _QDot(0) = SlaveInfo.vel;

            //            ForwardDynamics(*(JD.model),_Q,_QDot,);

            double torque_diff = 0;
            //            torque_diff = -(MasterInfo.cur*MasterInfo.N_gear*MasterInfo.TorqueConst - SlaveInfo.est_torque);
            torque_diff = -(MasterInfo.cur*MasterInfo.N_gear*MasterInfo.TorqueConst + SlaveInfo.cur*SlaveInfo.N_gear*SlaveInfo.TorqueConst);

            double dt = 1.0/500.0;

            if(fabs(torque_diff) < 1.0)
                torque_diff = 0;

            comp_acc_f = comp_acc_f*0.8 + (torque_diff*80)*0.2;
            comp_vel_f = comp_acc_f*dt + comp_vel_f*0.8;
            comp_pos_f = comp_vel_f*dt + comp_pos_f*1.0;


            DataBuf[11][saveIndex]= 0.01*torque_diff; //PWM [unit]
            DataBuf[12][saveIndex]= comp_pos_f; //PWM [unit]


            ///////////////////////////////////////// position-position teleoperation


            MasterInfo.est_torque = -MasterInfo.cur*MasterInfo.TorqueConst*MasterInfo.N_gear * 0.1 + MasterInfo.est_torque * 0.9;//master

            //master

            //            tempRef = 100*(SlaveInfo.pos - MasterInfo.pos)/2.0 + 1*(SlaveInfo.vel - MasterInfo.vel)/2.0 - 0.0*MasterInfo.est_torque/MasterInfo.N_gear/MasterInfo.TorqueConst;
            //            tempRef = 0;

            if(fabs(BITELE_JTS) < 1.0)
            {
                //no collision
                tempRef = 0;

            }
            else
            {
                //yes collision
                tempRef = 100*(SlaveInfo.pos - MasterInfo.pos)/2.0 + 1*(0 - MasterInfo.vel)/2.0;
            }


            if(tempRef > 10)
                tempRef = 10;
            else if(tempRef < -10)
                tempRef = -10;

            sharedREF->COCOACurrent_REF[PODO_NO][0] = tempRef;

            DataBuf[13][saveIndex]= BITELE_JTS; //Slave Torque [Nm]
            DataBuf[14][saveIndex]= MasterInfo.est_torque; //Master Torque [Nm]

            //slave
            //            tempRef = 100*(MasterInfo.pos - SlaveInfo.pos)/2.0 + 1*(MasterInfo.vel - SlaveInfo.vel)/2.0;
            //            + 1.0*MasterInfo.est_torque/SlaveInfo.N_gear/SlaveInfo.TorqueConst;

            if(fabs(BITELE_JTS) < 1.0)
            {
                //no collision
                tempRef = 100*(MasterInfo.pos - SlaveInfo.pos)/2.0 + 1*(0 - SlaveInfo.vel)/2.0;
            }
            else
            {
                //yes collision
                tempRef = MasterInfo.est_torque/SlaveInfo.N_gear/SlaveInfo.TorqueConst;
            }


            if(tempRef > 10)
                tempRef = 10;
            else if(tempRef < -10)
                tempRef = -10;

            sharedREF->COCOACurrent_REF[PODO_NO][1] = tempRef;


            ////            tempRef = (SlaveInfo.pos + comp_pos_f)* MasterInfo.N_gear * 180.0/PI ; //Master Position Reference, Input, [deg]
            //            tempRef = (MasterInfo.pos+MasterInfo.vel*0.01)* MasterInfo.N_gear * 180.0/PI ; //Master Position Reference, Input, [deg]
            //            jCon->SetMoveJoint(RHR, tempRef, 2.0, MOVE_ABSOLUTE);   //Master --> RHR (0)

            //            tempRef = (MasterInfo.pos) * SlaveInfo.N_gear * 180.0/PI ; //Slave Position Reference, Input, [deg]
            //            //            tempRef = (MasterInfo.pos) * SlaveInfo.N_gear * 180.0/PI ; //Slave Position Reference, Input, [deg]
            //            jCon->SetMoveJoint(RHP, tempRef, 2.0, MOVE_ABSOLUTE);   //Slave --> RHP (1)

            saveIndex++;

            break;
        }
        case OPERATION_BITELE_SAVE_ONLY:
        {
            DataBuf[0][saveIndex]= sharedSEN->ENCODER[0][0].CurrentCurrenta;
            DataBuf[1][saveIndex]= sharedSEN->ENCODER[0][0].CurrentCurrentb;
            DataBuf[2][saveIndex]= sharedSEN->ENCODER[0][0].CurrentCurrentc;
            DataBuf[3][saveIndex]= sharedSEN->ENCODER[0][0].CurrentPosition;
            DataBuf[4][saveIndex]= sharedSEN->ENCODER[0][0].CurrentVelocity;

            saveIndex++;

            break;
        }
        case OPERATION_BITELE_POSITION_LOCK:
        {


            double tempRef = 0.0;
            double tempPWM = 0.0;

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[1][0].PWMin;

            DataBuf[0][saveIndex]= tempRef; //position reference [rad], output
            DataBuf[1][saveIndex]= SlaveInfo.cur; //cur [A]
            DataBuf[2][saveIndex]= SlaveInfo.vel; //vel [rad/s]
            DataBuf[3][saveIndex]= SlaveInfo.pos; //pos [rad]
            DataBuf[4][saveIndex]= tempPWM; //PWM [unit]

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[0][0].PWMin;

            DataBuf[0+6][saveIndex]= tempRef*PI/180.0/MasterInfo.N_gear; //position reference [rad], output
            DataBuf[1+6][saveIndex]= MasterInfo.cur; //cur [A]
            DataBuf[2+6][saveIndex]= MasterInfo.vel; //vel [rad/s]
            DataBuf[3+6][saveIndex]= MasterInfo.pos; //pos [rad]
            DataBuf[4+6][saveIndex]= tempPWM; //PWM [unit]

            //////////////////////


            tempRef = BITELE_Pgain_Master*(0 - SlaveInfo.pos) + BITELE_Dgain_Master*(0 - SlaveInfo.vel);

            if(tempRef > 10)
                tempRef = 10;
            else if(tempRef < -10)
                tempRef = -10;

            sharedREF->COCOACurrent_REF[PODO_NO][1] = tempRef;
            DataBuf[11][saveIndex]= tempRef; //Motor input [A]

            ////  end of motion command, save index ++
            saveIndex++;

            break;



        }
        case OPERATION_BITELE_EXT_TORQUE_ESTIMATION:
        {

            double tempRef = 0.0;
            double alpha = 0;



            //// for estimating the friction params

            //            if(rt_count < 250)
            //                alpha = rt_count/250.0;
            //            else if(rt_count < 500*10-250)
            //                alpha = 1;
            //            else if(rt_count < 500*10)
            //                alpha = (500*10-rt_count)/250.0;


            //for position
            //            tempRef = alpha*BITELE_SIN_AMP*sin(2*PI*(float)rt_count*BITELE_SIN_FREQ/500.0)*N_gear; //pos [deg]
            //            jCon->SetMoveJoint(RHP, tempRef, 5.0, MOVE_ABSOLUTE);


            //for current
            //            if(rt_count % 500 > 250)
            //                tempRef = BITELE_SIN_AMP;
            //            else
            //                tempRef = -BITELE_SIN_AMP;

            //            sharedREF->COCOACurrent_REF[PODO_NO][1] = tempRef;

            //////////////////////////////// Slave side, external torque estimation

            double tempPWM;

            SlaveInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;   //input, current, [A]
            SlaveInfo.pos = sharedSEN->ENCODER[1][0].CurrentPosition*PI/180.0/SlaveInfo.N_gear; //output, [rad]
            SlaveInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity*PI/180.0/SlaveInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[1][0].PWMin;

            //            DataBuf[0][saveIndex]= tempRef*PI/180.0/SlaveInfo.N_gear; //position reference [rad], output
            DataBuf[0][saveIndex]= tempRef; //position reference [rad], output
            DataBuf[1][saveIndex]= SlaveInfo.cur; //cur [A]
            DataBuf[2][saveIndex]= SlaveInfo.vel; //vel [rad/s]
            DataBuf[3][saveIndex]= SlaveInfo.pos; //pos [rad]
            DataBuf[4][saveIndex]= tempPWM; //PWM [unit]

            //            -0.2938   -1.0916   -1.2401    0.6636

            //            JD.calc_est_torque();

            double II;
            II = ((33.3e-3*1e-4 + 0.054e-4 + 608.97e-3*1e-6)*SlaveInfo.N_gear*SlaveInfo.N_gear + 440321e-3*1e-6);
            _Q(0) = SlaveInfo.pos;
            _QDot(0) = SlaveInfo.vel;

            CompositeRigidBodyAlgorithm(*(JD.model),_Q,_H,1);

            _H(0,0) += II*0.6636;   //calibrated value, 20170731

            NonlinearEffects(*(JD.model),_Q,_QDot,N);
            NonlinearEffects(*(JD.model),_Q,_QDotzero,g);

            Cqdot = N-g;

            p = _H*_QDot;

            double K = 2*PI*10;

            double fric = 0;

            fric = -0.2938*SlaveInfo.vel + tanh(SlaveInfo.vel/-1.0916)*(-1.2401);

            _sum_slave = _sum_slave*0.975 + (SlaveInfo.cur*SlaveInfo.N_gear*SlaveInfo.TorqueConst - fric + Cqdot(0) - g(0) - est_torque_slave(0))*0.002;

            est_torque_slave(0) = K*(_sum_slave - p(0));

            SlaveInfo.est_torque = SlaveInfo.est_torque*0.7 - est_torque_slave(0)*0.3;

            DataBuf[5][saveIndex]= SlaveInfo.est_torque; //external torque, output, [Nm]


            /////////////////////////////////   Master side, human torque estimation
            //            -0.3291    0.2669    1.2099    0.6594

            MasterInfo.cur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, current, [A]
            MasterInfo.pos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/MasterInfo.N_gear; //output, [rad]
            MasterInfo.vel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/MasterInfo.N_gear; //output, [rad/s]
            tempPWM = sharedSEN->ENCODER[0][0].PWMin;

            DataBuf[0+6][saveIndex]= tempRef*PI/180.0/MasterInfo.N_gear; //position reference [rad], output
            DataBuf[1+6][saveIndex]= MasterInfo.cur; //cur [A]
            DataBuf[2+6][saveIndex]= MasterInfo.vel; //vel [rad/s]
            DataBuf[3+6][saveIndex]= MasterInfo.pos; //pos [rad]
            DataBuf[4+6][saveIndex]= tempPWM; //PWM [unit]



            if(fabs(SlaveInfo.est_torque) > 1.0)
            {
                if(flag_xe_update == 0)
                    flag_xe_update = 1;
            }
            else
            {
                xe_hat = SlaveInfo.pos;
                flag_xe_update = 0;
            }

            double beta_e = 0.75;

            if(flag_xe_update == 1)
            {
                xs_hat =  xe_hat - SlaveInfo.pos;

                if(xs_hat*SlaveInfo.est_torque >= 0)
                {
                    err_torque = SlaveInfo.est_torque - Ke_hat * xs_hat;
                    Q_gain = R_gain * xs_hat / (beta_e + xs_hat*xs_hat * R_gain);
                    R_gain = 1.0/beta_e*(1-Q_gain*xs_hat)*R_gain;
                    Ke_hat = Ke_hat + Q_gain*err_torque;

                    if(Ke_hat < 0)
                        Ke_hat = 0;
                }
            }

            double temp_Ke_hat = 0;

            if(flag_xe_update == 1)
            {
                if(Ke_hat < 750)
                    temp_Ke_hat = Ke_hat;
                else
                    temp_Ke_hat = 750;
            }
            else
            {
                temp_Ke_hat = 0;
            }




            DataBuf[11][saveIndex]= temp_Ke_hat; // Nm/rad



            //            if(fabs(SlaveInfo.est_torque) > 1.0)
            //            {
            //                if(flag_xe_update == 0)
            //                {
            //                    xe_hat = SlaveInfo.pos;
            //                    flag_xe_update = 1;
            //                }
            //            }
            //            else
            //            {
            //                flag_xe_update = 0;
            //            }

            //            double beta_e = 0.75;

            //            xs_hat =  xe_hat - SlaveInfo.pos;

            //            if(xs_hat*SlaveInfo.est_torque >= 0)
            //            {
            //                err_torque = SlaveInfo.est_torque - Ke_hat * xs_hat;
            //                Q_gain = R_gain * xs_hat / (beta_e + xs_hat*xs_hat * R_gain);
            //                R_gain = 1.0/beta_e*(1-Q_gain*xs_hat)*R_gain;
            //                Ke_hat = Ke_hat + Q_gain*err_torque;

            //                if(Ke_hat < 0)
            //                    Ke_hat = 0;
            //            }
            //            else
            //            {
            //                Ke_hat = 0.0;
            //            }

            //            DataBuf[11][saveIndex]= Ke_hat; // Nm/rad
            //            double temp_Ke_hat = 0;

            //            if(Ke_hat < 500)
            //                temp_Ke_hat = Ke_hat;
            //            else
            //                temp_Ke_hat = 500;

            fmk = MasterInfo.cur*MasterInfo.N_gear*MasterInfo.TorqueConst;
            fsk = SlaveInfo.est_torque;
            xmk = MasterInfo.pos;
            xsk = SlaveInfo.pos;

            double xsk_hat = 0;

            double tempA = 0;

            if(fabs(fsk) > 1)
            {
                tempA = -Eobs - fmk*(xmk - xmk1) + fsk*xsk1;
                if( fsk > 0)
                {
                    tempA /= fsk;
                    if(xmk > tempA)
                        xsk_hat = xmk;
                    else
                        xsk_hat = tempA;

                }
                else if( fsk < 0)
                {

                    tempA /= fsk;
                    if(xmk < tempA)
                        xsk_hat = xmk;
                    else
                        xsk_hat = tempA;

                }
            }
            else
                xsk_hat = xmk;


            xmk1 = xmk;
            xsk1 = xsk;

            DataBuf[14][saveIndex]= xsk_hat; // energy flow
            DataBuf[15][saveIndex]= xs_hat;
            DataBuf[16][saveIndex]= Ke_hat * xs_hat;
            DataBuf[17][saveIndex]= xe_hat;

            ////QP optimization?
            //            G.resize(1,1);
            //            g0.resize(1);
            //            CE.resize(1,0);
            //            ce0.resize(0);
            //            CI.resize(1,1);
            //            ci0.resize(1);
            //            outX.resize(1);

            ////            CE = 0;
            ////            ce0 = 0;

            //            CI[0][0] = fsk;
            //            ci0[0] = Eobs + fmk*(xmk-xmk1) - fsk*xsk1;

            //            G[0][0] = 1;
            //            g0[0] = -MasterInfo.pos;


            //            solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
            //            DataBuf[14][saveIndex]= outX[0]; // energy flow

            if(MasterInfo.cur*MasterInfo.N_gear*MasterInfo.TorqueConst * MasterInfo.vel < 0)
            {
                Eobs += MasterInfo.cur*MasterInfo.N_gear*MasterInfo.TorqueConst * MasterInfo.vel*0.002;
                //                Eobs += (SlaveInfo.est_torque * SlaveInfo.vel + MasterInfo.cur*MasterInfo.N_gear*MasterInfo.TorqueConst * MasterInfo.vel)*0.002;
            }

            Eobs *= 0.9;

            DataBuf[13][saveIndex]= Eobs; // energy flow



            //            tempRef = xsk * SlaveInfo.N_gear*180.0/PI; //slave input, [deg]
            //            tempRef = (MasterInfo.pos + 0.01*(xsk_hat - MasterInfo.pos))* SlaveInfo.N_gear*180.0/PI; //slave input, [deg]
            //            tempRef = MasterInfo.pos * SlaveInfo.N_gear*180.0/PI; //slave input, [deg]

            //            tempRef = (xe_hat - SlaveInfo.pos + MasterInfo.pos) * SlaveInfo.N_gear*180.0/PI;
            //            tempRef = (MasterInfo.pos) * SlaveInfo.N_gear*180.0/PI;

            //            filtered_slave_ref = 0.9*filtered_slave_ref + 0.1*(xe_hat - SlaveInfo.pos + MasterInfo.pos);
            //            tempRef = filtered_slave_ref * SlaveInfo.N_gear*180.0/PI;


            double beta_gain_target = 0;

            if(fabs(SlaveInfo.est_torque)> 1 && fabs(SlaveInfo.est_torque) > fabs(temp_Ke_hat*(SlaveInfo.pos - MasterInfo.pos)))
            {
                beta_gain_target = fabs( (fabs(SlaveInfo.est_torque) - fabs(temp_Ke_hat*(SlaveInfo.pos - MasterInfo.pos))) / fabs(SlaveInfo.est_torque) );

                if(beta_gain_target > 1)
                    beta_gain_target = 1;
                else if(beta_gain_target < 0)
                    beta_gain_target = 0;

            }
            else
            {
                beta_gain_target = 0;
            }


            if(fabs(SlaveInfo.est_torque) > 1)
            {
                if(fabs(SlaveInfo.est_torque) > fabs(temp_Ke_hat*(SlaveInfo.pos - MasterInfo.pos)))
                {
                    beta_gain += 0.005;
                    if(beta_gain > 1)
                        beta_gain = 1;
                }
                else if(fabs(SlaveInfo.est_torque) < fabs(temp_Ke_hat*(SlaveInfo.pos - MasterInfo.pos)))
                {
                    beta_gain -= 0.005;
                    if(beta_gain <0)
                        beta_gain = 0;
                }
            }
            else
            {
                beta_gain -= 0.01;
                if(beta_gain <0)
                    beta_gain = 0;
            }

            //            DataBuf[18][saveIndex]= beta_gain;

            //// impedance controller for slave

            double del_vel_im = MasterInfo.vel - SlaveInfo.vel;
            double del_pos_im = MasterInfo.pos - SlaveInfo.pos;

            double Bd = 0;
            double Kd = 0;

            double fd = - (Bd*del_vel_im + Kd*del_pos_im);

            if(fd - SlaveInfo.est_torque > 0.5)
            {
                xf -= 0.0025;

            }
            else if(fd - SlaveInfo.est_torque < -0.5)
            {
                xf += 0.0025;
            }

            xf = 0;
            //            beta_gain = 0;
            tempRef = (xf + MasterInfo.pos * (1-beta_gain) + SlaveInfo.pos*beta_gain) * SlaveInfo.N_gear*180.0/PI;
            jCon->SetMoveJoint(RHP, tempRef, 5.0, MOVE_ABSOLUTE);   //Master --> RHR (0), Slave --> RHP (1)
            DataBuf[18][saveIndex]= xf;
            DataBuf[19][saveIndex]= xf+MasterInfo.pos;


            ////controller for slave position
            //            filtered_slave_ref = 0.0*filtered_slave_ref + 1*((1-beta_gain)*MasterInfo.pos + beta_gain*SlaveInfo.pos);
            //            tempRef = (filtered_slave_ref) * SlaveInfo.N_gear*180.0/PI;
            //            jCon->SetMoveJoint(RHP, tempRef, 5.0, MOVE_ABSOLUTE);   //Master --> RHR (0), Slave --> RHP (1)

            //            tempRef = (MasterInfo.pos) * SlaveInfo.N_gear*180.0/PI;

            //            jCon->SetMoveJoint(RHP, tempRef, 5.0, MOVE_ABSOLUTE);   //Master --> RHR (0), Slave --> RHP (1)

            ////controller for master current

            //            tempRef = (temp_Ke_hat*(SlaveInfo.pos - MasterInfo.pos) + (1 - temp_Ke_hat/500.0)*SlaveInfo.est_torque) / MasterInfo.N_gear / MasterInfo.TorqueConst;
            //            if(xs_hat*SlaveInfo.est_torque >= 0)
            //                tempRef = temp_Ke_hat*xs_hat;
            //            else
            //                tempRef = 0;

            double torque_damp = 0;

            if(Eobs < 0)
                torque_damp = -1.0 * fabs(Eobs)* MasterInfo.vel;

            //            if((SlaveInfo.pos - MasterInfo.pos)*SlaveInfo.est_torque > 0)
            //                tempRef = (SlaveInfo.est_torque + torque_damp)/MasterInfo.N_gear/MasterInfo.TorqueConst ;
            //            else
            //                tempRef = (temp_Ke_hat*(SlaveInfo.pos - MasterInfo.pos) + SlaveInfo.est_torque + torque_damp)/MasterInfo.N_gear/MasterInfo.TorqueConst ;

            //            tempRef = temp_Ke_hat*(xe_hat - MasterInfo.pos)/MasterInfo.N_gear/MasterInfo.TorqueConst;
            tempRef = (temp_Ke_hat*(SlaveInfo.pos - MasterInfo.pos) + torque_damp)/MasterInfo.N_gear/MasterInfo.TorqueConst ;

            //output torque saturation;
            if(tempRef > 15)                tempRef = 15;
            else if(tempRef <-15)           tempRef = -15;

            sharedREF->COCOACurrent_REF[PODO_NO][0] = -tempRef;
            DataBuf[12][saveIndex]= tempRef*MasterInfo.N_gear*MasterInfo.TorqueConst; //Matser Force Output [Nm];

            ////  end of motion command, save index ++
            saveIndex++;

            //            if(rt_count > 500*10)
            //            {
            //                OPERATION_MODE = OPERATION_NONE;
            //                cout << "external torque estimation test, finish!!" << endl;
            //            }



            break;
        }
        case OPERATION_JTS_OFFSETNULLING:
        {

            JTS_SENSOR_1 = (double)sharedSEN->ENCODER[1][0].Cocoa_Data.TORQUE_SENSOR1;
            JTS_SENSOR_2 = (double)sharedSEN->ENCODER[1][0].Cocoa_Data.TORQUE_SENSOR2;

            JTS_SENSOR_1_OFFSET +=  JTS_SENSOR_1/200.0;
            JTS_SENSOR_2_OFFSET +=  JTS_SENSOR_2/200.0;

            if(rt_count >= 199)
            {

                OPERATION_MODE = OPERATION_NONE;

                cout << "NULLING DONE !!" << ", OFFSET1 : "<< JTS_SENSOR_1_OFFSET << ", OFFSET2 : " << JTS_SENSOR_2_OFFSET << endl;
            }

            break;

        }
        case OPERATION_JTS_SETMOVEJOINT:
        {

            JTS_SENSOR_1 = (double)sharedSEN->ENCODER[0][0].Cocoa_Data.TORQUE_SENSOR1 - JTS_SENSOR_1_OFFSET;
            JTS_SENSOR_2 = (double)sharedSEN->ENCODER[0][0].Cocoa_Data.TORQUE_SENSOR2 - JTS_SENSOR_2_OFFSET;


            cout << "JTS_SENSOR_11 : " << sharedSEN->ENCODER[0][0].Cocoa_Data.TORQUE_SENSOR1 << ", JTS_SENSOR_22 : " << sharedSEN->ENCODER[0][0].Cocoa_Data.TORQUE_SENSOR2 << endl;
            cout << "JTS_SENSOR_1 : " << JTS_SENSOR_1 << ", JTS_SENSOR_2 : " << JTS_SENSOR_2 << endl;

            if(rt_count == 0)
                jCon->SetMoveJoint(RHR, JTS_DESIRED_ANGLE, JTS_DESIRED_TIME*1000.0, MOVE_ABSOLUTE);
            //                jCon->SetMoveJoint(RHR, JTS_DESIRED_ANGLE*160.0/10240.0*4000.0, JTS_DESIRED_TIME*1000.0, MOVE_ABSOLUTE);

            double tempPos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/160.0; //output, rad
            double tempVel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/160.0; //output, rad/s

            DataBuf[0][saveIndex]= JTS_SENSOR_1;
            DataBuf[1][saveIndex]= JTS_SENSOR_2;
            DataBuf[2][saveIndex]= tempPos;
            DataBuf[3][saveIndex]= tempVel;
            DataBuf[4][saveIndex]= sharedSEN->ENCODER[0][0].CurrentCurrent;
            DataBuf[5][saveIndex]= sharedSEN->ENCODER[0][0].PWMin;
            DataBuf[6][saveIndex]= sharedSEN->ENCODER[0][0].CurrentReference*PI/180.0/160.0;


            saveIndex++;

            if(rt_count > JTS_DESIRED_TIME*500 - 1)
            {
                OPERATION_MODE = OPERATION_NONE;
                cout << "Finish !!" << endl;
            }

            break;
        }
        case OPERATION_EXT_TEST:
        {


            if(rt_count2 > 500*4)
                rt_count2 = 500*4+1;
            else
                rt_count2++;

            double tempRef = 0.0;
            JS_CURRENT_REF_FREQ = 2.0; // period 1- >1Hz, 2->2Hz
            JS_CURRENT_REF_AMP = 2.0;

            //            tempRef = JS_CURRENT_REF_AMP*sin(2*PI*(float)rt_count2/JS_CURRENT_REF_FREQ/500.0);
            tempRef = JS_CURRENT_REF_AMP;
            //            jCon->SetMoveJoint(RHR, tempRef*N_gear, 5.0, MOVE_ABSOLUTE);
            sharedREF->COCOACurrent_REF[PODO_NO][0] = tempRef;

            double tempCur, tempPos, tempVel,tempPWM;

            tempCur = sharedSEN->ENCODER[0][0].CurrentCurrent;   //input, torque
            tempPos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/N_gear; //output, rad
            tempVel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/N_gear; //output, rad/s
            tempPWM = sharedSEN->ENCODER[0][0].PWMin;
            //            tempAcc = (tempVel-lastVel)*200; //output, rad/s

            //            cout << "Ref : " << tempRef << endl;

            DataBuf[0][saveIndex]= tempRef; //current reference
            DataBuf[1][saveIndex]= tempCur;
            DataBuf[2][saveIndex]= tempVel;
            DataBuf[3][saveIndex]= tempPos;
            DataBuf[4][saveIndex]= tempPWM;

            //            DataBuf[4][saveIndex]= est_torque2(0);
            //            DataBuf[5][saveIndex]= est_torque2(0) + fric;
            //            DataBuf[6][saveIndex]= tempAcc;
            //            DataBuf[7][saveIndex]= N(0);

            saveIndex++;

            //            double InputTorque, tempPos, tempVel,tempAcc;

            //            InputTorque = sharedSEN->ENCODER[0][0].CurrentCurrent*N_gear*27.6e-3;   //input, torque
            //            tempPos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/N_gear; //output, rad
            //            tempVel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/N_gear; //output, rad/s
            //            tempAcc = (tempVel-lastVel)*200; //output, rad/s


            //            double II;
            //            II = ((33.3e-3*1e-4 + 0.054e-4 + 608.97e-3*1e-6)*N_gear*N_gear + 440321e-3*1e-6);   //output, intertia

            //            _Q(0) = tempPos;
            //            _QDot(0) = tempVel;
            //            _QDDot(0) = tempAcc;

            //            CompositeRigidBodyAlgorithm(*(JD.model),_Q,_H,1);

            //            _H(0,0) += II;

            //            NonlinearEffects(*(JD.model),_Q,_QDot,N);
            //            NonlinearEffects(*(JD.model),_Q,_QDotzero,g);

            //            Cqdot = N-g;

            //            p = _H*_QDot;

            //            // Ext torque estimaiton 1
            //            double K = 100.0;

            //            double fric;
            //            double fric_sat = 1.00;
            //            double fric_mu = 0.4;

            //            if(fabs(fric_mu * tempVel)>fric_sat)
            //                fric = fric_sat*sign(tempVel);
            //            else
            //                fric = fric_mu * tempVel;

            //            _sum = _sum + (InputTorque + Cqdot(0) - g(0) - est_torque(0) - fric)*0.005;

            //            est_torque(0) = K*(_sum - p(0));

            //            // Ext torque estimaiton 2
            //            est_torque2 =  _H*_QDDot + N ;


            //            lastVel = tempVel;
            break;
        }
        case OPERATION_EXT_SIN:
        {
            //            cout << "CURRENT INPUT IS 1A" << endl;


            // slave position control /////////////////////////////////////////////////////////

            MasterInfo.cur = sharedSEN->ENCODER[1][0].CurrentCurrent;
            MasterInfo.pos = Master_Pos * 0.9 + 0.1 * sharedSEN->ENCODER[1][0].CurrentPosition; //deg (reference)
            MasterInfo.vel = sharedSEN->ENCODER[1][0].CurrentVelocity; //deg/s

            Master_Cur = sharedSEN->ENCODER[1][0].CurrentCurrent;
            Master_Pos = Master_Pos * 0.9 + 0.1 * sharedSEN->ENCODER[1][0].CurrentPosition; //deg (reference)
            Master_Vel = sharedSEN->ENCODER[1][0].CurrentVelocity; //deg/s

            jCon->SetMoveJoint(LHP, Master_Pos, 0.005, MOVE_ABSOLUTE);

            // master current control /////////////////////////////////////////////////////////

            //            double tempRef;

            //            if(JS_CURRENT_REF_FREQ != 0)    tempRef = JS_CURRENT_REF_AMP*sin(2*PI*(float)rt_count/JS_CURRENT_REF_FREQ/200.0);
            //            else                            tempRef = JS_CURRENT_REF_AMP;

            //            sharedREF->COCOACurrent_REF[PODO_NO][0] = tempRef;

            double tempCur, tempPos, tempVel;

            tempCur = sharedSEN->ENCODER[0][0].CurrentCurrent;
            tempPos = sharedSEN->ENCODER[0][0].CurrentPosition*PI/180.0/N_gear; //rad
            tempVel = sharedSEN->ENCODER[0][0].CurrentVelocity*PI/180.0/N_gear; //rad/s

            double II;
            II = ((33.3e-3*1e-4 + 0.054e-4 + 608.97e-3*1e-6)*N_gear*N_gear + 440321e-3*1e-6);

            _Q(0) = tempPos;
            _QDot(0) = tempVel;
            CompositeRigidBodyAlgorithm(*(JD.model),_Q,_H,1);
            _H(0,0) += II;
            NonlinearEffects(*(JD.model),_Q,_QDot,N);
            NonlinearEffects(*(JD.model),_Q,_QDotzero,g);
            Cqdot = N-g;
            p = _H*_QDot;

            double K = 100.0;

            double fric;
            double fric_sat = 1.00;
            double fric_mu = 0.4;

            if(fabs(fric_mu * tempVel)>fric_sat)
                fric = fric_sat*sign(tempVel);
            else
                fric = fric_mu * tempVel;

            _sum = _sum + (tempCur*N_gear*27.6e-3 + Cqdot(0) - g(0) - est_torque(0) - fric)*0.005;

            est_torque(0) = K*(_sum - p(0));


            double alpha = 0.8;
            Master_CurRef = Master_CurRef*alpha + est_torque(0)/27.6e-3/100*(1.0-alpha);

            double temp_gain = 1.0;

            if(Master_CurRef >= 3.0*temp_gain) Master_CurRef = 3.0*temp_gain;
            else if(Master_CurRef <= -3.0*temp_gain) Master_CurRef = -3.0*temp_gain;

            sharedREF->COCOACurrent_REF[PODO_NO][1] = -Master_CurRef;

            cout << "Master Torque (Slave Estimated Torque) : " << Master_CurRef << "vel : " << tempVel << endl;

            // data display % Save //////////////////////////////////////////////////////////////////

            //            cout << est_torque << endl;


            //printf("SINE_TOTAL_COUNT = %f\n",SINE_TOTAL_COUNT);
            DataBuf[0][saveIndex]= tempCur*N_gear*27.6e-3;
            DataBuf[1][saveIndex]= est_torque(0);
            DataBuf[2][saveIndex]= tempVel;

            saveIndex++;

            //    tempQ(0) = PI/2.0;

            //    CompositeRigidBodyAlgorithm(*(JD.model),tempQ,tempH,1);

            //    Tau = VectorNd::Zero (JD.model->dof_count);
            //    tempQdot = VectorNd::Zero (JD.model->dof_count);
            //    NonlinearEffects(*(JD.model),tempQ,tempQdot,Tau);

            //    cout << "Mass Matrix : " << tempH << endl;
            //    cout << "Nonliear Effect : " << Tau << endl;

            break;
        }
        case OPERATION_EXT_SIN2:
        {

            // slave position control /////////////////////////////////////////////////////////

            Master_Cur = sharedSEN->ENCODER[1][0].CurrentCurrent;
            Master_Pos = sharedSEN->ENCODER[1][0].CurrentPosition; //deg (reference)
            Master_Vel = sharedSEN->ENCODER[1][0].CurrentVelocity; //deg/s

            jCon->SetMoveJoint(LHP, Master_Pos, 0.005, MOVE_ABSOLUTE);


            // master position control /////////////////////////////////////////////////////////

            Slave_Cur = sharedSEN->ENCODER[0][0].CurrentCurrent;
            Slave_Pos = sharedSEN->ENCODER[0][0].CurrentPosition; //deg (reference)
            Slave_Vel = sharedSEN->ENCODER[0][0].CurrentVelocity; //deg/s

            jCon->SetMoveJoint(LKN, Slave_Pos, 0.005, MOVE_ABSOLUTE);

            // data display % Save //////////////////////////////////////////////////////////////////

            //printf("SINE_TOTAL_COUNT = %f\n",SINE_TOTAL_COUNT);
            //            DataBuf[0][saveIndex]= tempCur*N_gear*27.6e-3;
            //            DataBuf[1][saveIndex]= est_torque(0);
            //            DataBuf[2][saveIndex]= tempVel;


            //            saveIndex++;


            break;
        }
        case OPERATION_HAP_3CH:
        {

            // slave position control /////////////////////////////////////////////////////////

            Master_Cur = sharedSEN->ENCODER[1][0].CurrentCurrent;
            Master_Pos = sharedSEN->ENCODER[1][0].CurrentPosition; //deg (reference)
            Master_Vel = sharedSEN->ENCODER[1][0].CurrentVelocity; //deg/s


            // master (force+position) control /////////////////////////////////////////////////////////

            Slave_Cur = sharedSEN->ENCODER[0][0].CurrentCurrent;
            Slave_Pos = sharedSEN->ENCODER[0][0].CurrentPosition; //deg (reference)
            Slave_Vel = sharedSEN->ENCODER[0][0].CurrentVelocity; //deg/s

            double II;
            II = ((33.3e-3*1e-4 + 0.054e-4 + 608.97e-3*1e-6)*N_gear*N_gear + 440321e-3*1e-6);
            _Q(0) = Slave_Pos/N_gear*PI/180;
            _QDot(0) = Slave_Vel/N_gear*PI/180;

            CompositeRigidBodyAlgorithm(*(JD.model),_Q,_H,1);

            _H(0,0) += II;

            NonlinearEffects(*(JD.model),_Q,_QDot,N);
            NonlinearEffects(*(JD.model),_Q,_QDotzero,g);

            Cqdot = N-g;

            p = _H*_QDot;

            double K = 100.0;

            double fric;
            double fric_sat = 1.00;
            double fric_mu = 0.4;

            if(fabs(fric_mu * Slave_Vel)>fric_sat)
                fric = fric_sat*sign(Slave_Vel);
            else
                fric = fric_mu * Slave_Vel;

            _sum = _sum + (Slave_Cur*N_gear*27.6e-3 + Cqdot(0) - g(0) - est_torque(0) - fric)*0.005;

            est_torque(0) = K*(_sum - p(0));

            // Current Reference LPF
            double alpha = 0.7;
            Master_CurRef = Master_CurRef*alpha - est_torque(0)/27.6e-3/50*(1.0-alpha);

            // Position + Force Control Fusion
            //            Master_CurRef = pos_force_ratio*(HAP_3ch_Pgain*(Slave_Pos-Master_Pos)/N_gear + HAP_3ch_Dgain * (Slave_Vel-Master_Vel)/N_gear) +  (1-pos_force_ratio) * Master_CurRef;

            if(Slave_Vel*est_torque(0) >= 0)
            {
                passivity_obs = 0.999 * passivity_obs + Slave_Vel*est_torque(0)*0.005/50.0;
                passivity_obs2 = 0.999 * passivity_obs2 + Slave_Vel*est_torque(0)*0.005/50.0;

            }
            else
            {
                passivity_obs = 0.8 * passivity_obs + Slave_Vel*est_torque(0)*0.005/50.0;
                passivity_obs2 = 0.5 * passivity_obs2 + Slave_Vel*est_torque(0)*0.005/50.0;

            }

            //            double temp_gain = 0.;

            //            if(passivity_obs > 1.0)
            //            {
            //                if(passivity_obs > 5.0)
            //                {
            //                    HAP_3ch_Pgain = 0.5;
            //                    HAP_3ch_Dgain = 0.025;
            //                    temp_gain = 0.2;
            //                }

            //                else
            //                {
            //                    HAP_3ch_Pgain = 0.5*(passivity_obs-1.0)/4.0;
            //                    HAP_3ch_Dgain = 0.025*(passivity_obs-1.0)/4.0;
            //                    temp_gain = 0.2*(passivity_obs-1.0)/4.0;
            //                }

            //            }
            //            else
            //            {
            //                HAP_3ch_Pgain = 0.0;
            //                HAP_3ch_Dgain = 0.0;
            //                temp_gain = 0.0;
            //            }

            //            if(passivity_obs2 > 1.0)
            //            {
            //                if(passivity_obs2 > 5.0)
            //                {
            //                    temp_gain = 0.2;
            //                }

            //                else
            //                {
            //                    temp_gain = 0.2*(passivity_obs2-1.0)/4.0;
            //                }

            //            }
            //            else
            //            {
            //                temp_gain = 0.0;
            //            }


            //            HAP_3ch_Pgain = 0.10;
            //            HAP_3ch_Dgain = 0.00;

            //            temp_gain =0.;

            //            Master_CurRef = pos_force_ratio*(HAP_3ch_Pgain*(Master_Pos-Slave_Pos)/N_gear + HAP_3ch_Dgain * (Master_Vel-Slave_Vel)/N_gear) +  (1-pos_force_ratio) * Master_CurRef;

            //            Master_CurRef = (HAP_3ch_Pgain*(Master_Pos-Slave_Pos)/N_gear + HAP_3ch_Dgain * (Master_Vel-Slave_Vel)/N_gear) + Master_CurRef;
            //            Master_CurRef = HAP_3ch_Pgain*(Slave_Pos-Master_Pos)/N_gear + HAP_3ch_Dgain *(Slave_Vel - Master_Vel)/N_gear + Master_CurRef*0.5;
            //            Master_CurRef = HAP_3ch_Pgain*(Slave_Pos-Master_Pos)/N_gear + HAP_3ch_Dgain *(Slave_Vel - Master_Vel)/N_gear + Master_CurRef*0.5;

            //            Master_CurRef = temp_gain/(1.0 + temp_gain)*(HAP_3ch_Pgain*(Slave_Pos-Master_Pos)/N_gear + HAP_3ch_Dgain *(Slave_Vel - Master_Vel)/N_gear) + 1.0/(1.0+temp_gain)*Master_CurRef;
            //            Master_CurRef = (HAP_3ch_Pgain*(Slave_Pos-Master_Pos)/N_gear + HAP_3ch_Dgain *( - Master_Vel)/N_gear)+Master_CurRef*0.5;

            Master_CurRef =temp_gain_master*(HAP_3ch_Pgain*(Slave_Pos-Master_Pos)/N_gear + HAP_3ch_Dgain *(Slave_Vel - Master_Vel)/N_gear) + (1.0 - temp_gain_master)*Master_CurRef;



            // Current Reference Saturation
            double sat_cur = 5.0;
            if(Master_CurRef >= sat_cur) Master_CurRef = sat_cur;
            else if(Master_CurRef <= -1.0*sat_cur) Master_CurRef = -1.0*sat_cur;

            sharedREF->COCOACurrent_REF[PODO_NO][1] = Master_CurRef;

            double Master_Pos_temp;

            Master_Pos_temp = (1.0 - temp_gain_slave)*Master_Pos + temp_gain_slave*Slave_Pos;

            jCon->SetMoveJoint(LHP, Master_Pos_temp, 0.005, MOVE_ABSOLUTE);




            //            passivity_obs2 = 1.0 * passivity_obs2 + Slave_Vel*est_torque(0)*0.005/50.0;
            passivity_obs3 = 1.0 * passivity_obs3 + Master_Cur* Master_Vel - Slave_Cur*Slave_Vel;
            passivity_obs4 = 0.95 * passivity_obs4 + Master_Cur* Master_Vel - Slave_Cur*Slave_Vel;

            if (fabs(Slave_Pos - Master_Pos)<1e-5 || fabs(est_torque(0))<0.5) est_stiff = 0.0;
            else est_stiff = est_torque(0)/(Slave_Pos - Master_Pos);


            // data display % Save //////////////////////////////////////////////////////////////////

            //printf("SINE_TOTAL_COUNT = %f\n",SINE_TOTAL_COUNT);
            DataBuf[0][saveIndex]= Master_Cur*N_gear*27.6e-3;
            DataBuf[1][saveIndex]= Slave_Cur*N_gear*27.6e-3;
            DataBuf[2][saveIndex]= Master_Pos/N_gear;
            DataBuf[3][saveIndex]= Slave_Pos/N_gear;
            DataBuf[4][saveIndex]= passivity_obs;
            DataBuf[5][saveIndex]= passivity_obs2;
            DataBuf[6][saveIndex]= est_stiff;
            DataBuf[7][saveIndex]= est_torque(0);
            DataBuf[8][saveIndex]= HAP_3ch_Dgain;

            DataBuf[9][saveIndex]= passivity_obs;
            DataBuf[10][saveIndex]= passivity_obs2;
            DataBuf[11][saveIndex]= passivity_obs3;
            DataBuf[12][saveIndex]= passivity_obs4;

            saveIndex++;

            break;
        }

        case OPERATION_CYCLOID_SINE_POS_TEST:
        {
            float cur_t =((float)rt_count)/500;
            float ref_pos = SINE_INIT_POS + SINE_MAG*(cos(SINE_FREQ*cur_t)-1);
            float ref_vel = -SINE_MAG*SINE_FREQ*sin(SINE_FREQ*cur_t);

//            float torque_voltage = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque;
//            float output_torque = -(torque_voltage+TORQUE_OFFSET-1.5)/0.85*100.0;

            //// (1-cos) curve
//            float ref_pos = SINE_INIT_POS + (SINE_MAG-SINE_INIT_POS)*0.5f*(1-cos(SINE_FREQ*cur_t));
//            float ref_vel = 0.5f*(SINE_MAG-SINE_INIT_POS)*SINE_FREQ*sin(SINE_FREQ*cur_t);
            ////
//            cout << "cur_t :" << cur_t << "ref_pos : " << ref_pos << endl;
            jCon->SetMoveJoint(RHR, ref_pos, 2, MOVE_ABSOLUTE);

//            cout << "input_current = " << sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent << " output_torque = " << torque << endl;

            DataBuf[0][saveIndex]= cur_t;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
            DataBuf[3][saveIndex]= ref_vel;
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;
            DataBuf[6][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;
            DataBuf[7][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1; // input torque
            DataBuf[8][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2; // output torque

//            COMMAND_STRUCT cmd;
//            cmd.USER_PARA_INT[0] = rt_count;
//            cmd.USER_PARA_FLOAT[0] = ref_pos;
//            cmd.USER_PARA_FLOAT[1] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
//            cmd.USER_PARA_FLOAT[2] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;
//            cmd.USER_PARA_FLOAT[3] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;
//            sharedCMD->COMMAND[MAX_AL-1] = cmd;

            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_SINE_POS_TEST;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = rt_count;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] = SINE_TOTAL_COUNT;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2] = ref_vel;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[5] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;

            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[6] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;
//            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[7] = output_torque;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[7] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1; // input torque
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[8] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2; // output torque

            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(rt_count >= SINE_TOTAL_COUNT)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_NO_ACT;

                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_CYCLOID_VAR_SINE_POS_TEST:
        {
            float cur_t =((float)rt_count)/500;
            float freq = VAR_SINE_INIT_FREQ + ((VAR_SINE_FINAL_FREQ-VAR_SINE_INIT_FREQ)/VAR_SINE_TIME)*cur_t;
//            float ref_pos = VAR_SINE_INIT_POS + VAR_SINE_MAG*sin(freq*cur_t);
            float ref_pos = VAR_SINE_INIT_POS + VAR_SINE_MAG*(cos(freq*cur_t)-1);
            float ref_vel = -VAR_SINE_MAG*freq*sin(freq*cur_t);
            jCon->SetMoveJoint(RHR, ref_pos, 2, MOVE_ABSOLUTE);

            DataBuf[0][saveIndex]= cur_t;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
            DataBuf[3][saveIndex]= ref_vel;
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;
            DataBuf[6][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;

            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_VAR_SINE_POS_TEST;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = rt_count;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] = VAR_SINE_TOTAL_COUNT;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2] = ref_vel;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[5] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;

            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(rt_count >= VAR_SINE_TOTAL_COUNT)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_NO_ACT;

                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_CYCLOID_EVAL_POSITION:
        {
            float cur_t =((float)rt_count)/500;
            float ref_pos = SINE_MAG*0.5f*(1-cos(SINE_FREQ*cur_t));
            float ref_vel = 0.5f*(SINE_MAG-SINE_INIT_POS)*SINE_FREQ*sin(SINE_FREQ*cur_t);
//            cout << "cur_t :" << cur_t << "ref_pos : " << ref_pos << endl;
            jCon->SetMoveJoint(RHR, ref_pos, 2, MOVE_RELATIVE);

//            temp_pos = ref_pos;

//            float ref_vel = 0;
//            jCon->SetMoveJoint(RHR, SINE_MAG, 1.0/SINE_FREQ, MOVE_RELATIVE);


            DataBuf[0][saveIndex]= cur_t;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
            DataBuf[3][saveIndex]= ref_vel;
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;
            DataBuf[6][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;

            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_EVAL_MODE_POSITION;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = rt_count;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[2] = SINE_TOTAL_COUNT;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2] = ref_vel;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[5] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;

            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(rt_count >= SINE_TOTAL_COUNT)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_NO_ACT;

                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_CYCLOID_EVAL_TORQUE:
        {
            float cur_t =((float)rt_count)/500;
            float ref_input_torque;
//            float torque_voltage = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque;
//            float output_torque = -(torque_voltage+TORQUE_OFFSET-1.5)/0.85*100.0;

            if(EVAL_MODE == 1){
                ref_input_torque = TORQUE_CONSTANT*REF_CURRENT_Q;
            }else if(EVAL_MODE == 2){
                ref_input_torque = REF_TORQUE;
                REF_CURRENT_Q = REF_TORQUE/TORQUE_CONSTANT;
                REF_CURRENT_D = 0.0;
            }

            sharedREF->COCOAQCurrent_REF[PODO_NO][RHR] = REF_CURRENT_Q;
            sharedREF->COCOADCurrent_REF[PODO_NO][RHR] = REF_CURRENT_D;

            DataBuf[0][saveIndex]= cur_t;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference; // input position ref
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition;  // input position
            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity;  // input velocity
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition;  // output position
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity;  // output velocity

            DataBuf[6][saveIndex]= REF_CURRENT_Q;                                                     // input current ref
            DataBuf[7][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;   // input current
            DataBuf[8][saveIndex]= ref_input_torque;                                              // input estimated torque ref
            DataBuf[9][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent * TORQUE_CONSTANT; // input estimated torque
//            DataBuf[10][saveIndex]= output_torque;
            DataBuf[10][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;   // input torque
            DataBuf[11][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;   // output torque


            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_EVAL_MODE_TORQUE;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = rt_count;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0] = REF_CURRENT_Q;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1] = sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentCurrent;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[2] = ref_input_torque;
//            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3] = output_torque;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[3] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1; // input torque
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[4] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2; // output torque

            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(test_stop == true)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_NO_ACT;

                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_CYCLOID_EVAL_DYNAMO:
        {
            float cur_t =((float)rt_count)/500;
//            float torque_voltage = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque;
//            float torque = -(torque_voltage+TORQUE_OFFSET-1.5)/0.85*100.0;

            DataBuf[0][saveIndex]= cur_t;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1; // cycloid input torque
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2; // cycloid output torque

//            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentReference; // cycloid input position reference
            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentPosition; // cycloid input angle
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHR)][MC_GetCH(RHR)].CurrentVelocity; // cycloid input velocity

            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentPosition; // cycloid output angle
            DataBuf[6][saveIndex]= sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentVelocity; // cycloid output velocity

            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_EVAL_MODE_DYNAMO;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1] = rt_count;
//            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0] = torque;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[0] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque1;
            sharedCMD->COMMAND[PODO_NO].USER_PARA_FLOAT[1] = sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque2;

            saveIndex++;

            if(saveIndex==150000)
                saveIndex=0;

            if(test_stop == true)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_NO_ACT;

                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_CYCLOID_EVAL_TORQUE_NULLING:
        {
//            unsigned int time = 4;          // [sec]
//            unsigned int total_count = time*500;

//            TORQUE_SUM = TORQUE_SUM + sharedSEN2.ENCODER[MC_GetID(RHP)][MC_GetCH(RHP)].CurrentTorque;

//            if(rt_count == total_count)
//            {
//                float torque_mean = TORQUE_SUM/((float)total_count);
//                if(torque_mean >= 1.5){
//                    TORQUE_OFFSET = torque_mean - 1.5;
//                }else if(torque_mean < 1.5){
//                    TORQUE_OFFSET = 1.5 - torque_mean;
//                }
//                printf("Torque Nulling finished ...!!!\n");
//                TORQUE_SUM = 0.;
//                rt_count = 0;
//                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = CYCLOID_NO_ACT;

//                OPERATION_MODE = OPERATION_NONE;
//            }
            break;
        }

        case OPERATION_NONE:
            break;

        case OPERATION_TRAPEZOIDAL:
        {
            /*
            float cur_t = ((float)rt_count)/200;



            float output_1 = SINE_MAG*sin(SINE_FREQ*cur_t);
            float output_2 = -SINE_MAG*sin(SINE_FREQ*cur_t);



            DEBUG = output_1;

            jCon->SetMoveJoint(LHP, output_1, 5, MOVE_ABSOLUTE);
            jCon->SetMoveJoint(LKN, output_2, 5, MOVE_ABSOLUTE);


            if(rt_count == SINE_TOTAL_COUNT){

                saveFlag = 0;
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                OPERATION_MODE = OPERATION_NONE;
            }
            break;
            */

            DataBuf[0][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentPosition;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentReference;
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentVelocity;
            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference;
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentVelocity;


            DataBuf[6][saveIndex]= sharedSEN2.FT[2].Mx;
            DataBuf[7][saveIndex]= sharedSEN2.FT[2].My;
            DataBuf[8][saveIndex]= sharedSEN2.FT[2].Fz;
            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(rt_count >= TIME/5+50){

                saveFlag = 0;
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_JUMP:
        {
            float cur_t = ((float)rt_count)/200;

            FootPos FP;
            LegJoint Lj;

            if (cur_t>=1.2)
            {
                if (JUMP_MODE == 0)
                {
                    FP = SawProfile(DECEL_1,TAKE_OFF_POS,START_POS,TAKE_OFF_VEL,cur_t-1.2);
                    Lj = InverseKinematics(-FP.X, -FP.Y);
                }
                else if (JUMP_MODE == 1)
                {
                    FP = TrapezoidalProfile(DECEL_1,DECEL_2,TAKE_OFF_POS,START_POS,TAKE_OFF_VEL,LANDING_VEL,PERIOD,STOP_TIME,cur_t-1.2);
                    Lj = InverseKinematics(-FP.X, -FP.Y);

                    //DataBuf[0][saveIndex]= FP.X;
                    //DataBuf[1][saveIndex]= FP.Y;
                }

                // DataBuf[0][saveIndex]= Lj.S1*100;
                // DataBuf[1][saveIndex]= -Lj.S2*100;
                jCon->SetJointRefAngle(LHP, Lj.S1*100);
                jCon->SetJointRefAngle(LKN, -Lj.S2*100);
            }


            DataBuf[0][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentPosition;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentReference;
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentVelocity;
            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference;
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentVelocity;


            DataBuf[6][saveIndex]= sharedSEN2.FT[2].Mx;
            DataBuf[7][saveIndex]= sharedSEN2.FT[2].My;
            DataBuf[8][saveIndex]= sharedSEN2.FT[2].Fz;
            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(ITERATION >= CYCLE_NUM)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                ITERATION= 0;
                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_SPRINGMASSMODEL:
        {
            float cur_t = ((float)rt_count)/200;

            FootPos FP;
            LegJoint Lj;

            if (cur_t>=1.2)
            {
                FP = SpringMassModel(SPRING_CONSTANT, INITIAL_POS, INITIAL_VEL, DELAY_TIME, AERIAL_SEG_NUM, cur_t-1.2);
                Lj = InverseKinematics(-FP.X, -FP.Y);

                //DataBuf[0][saveIndex]= FP.X;
                //DataBuf[1][saveIndex]= FP.Y;

                // DataBuf[0][saveIndex]= Lj.S1*100;
                // DataBuf[1][saveIndex]= -Lj.S2*100;
                jCon->SetJointRefAngle(LHP, Lj.S1*100);
                jCon->SetJointRefAngle(LKN, -Lj.S2*100);
            }
            DataBuf[0][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentPosition;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentReference;
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentVelocity;
            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference;
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentVelocity;


            DataBuf[6][saveIndex]= sharedSEN2.FT[2].Mx;
            DataBuf[7][saveIndex]= sharedSEN2.FT[2].My;
            DataBuf[8][saveIndex]= sharedSEN2.FT[2].Fz;


            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(ITERATION >= MAX_ITERATION)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                ITERATION= 0;
                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_SINE:
        {
            float cur_t =((float)rt_count)/500;
            float output_2 = 0;
            LegJoint Lj;
            if (cur_t>=1.2)
            {

                Lj = InverseKinematics(0,-SINE_INIT_POS);

                //float output_1 = Lj.S1*100 + SINE_MAG*sin(SINE_FREQ*cur_t);
                float output_2 = -(Lj.S2*100 + SINE_MAG*sin(SINE_FREQ*(cur_t-1.2)));
                //jCon->SetMoveJoint(LHP, output_1, 5, MOVE_RELATIVE);
                jCon->SetMoveJoint(LKN, output_2, 5, MOVE_ABSOLUTE);
            }

            DataBuf[0][saveIndex]= output_2;

            //printf("SINE_TOTAL_COUNT = %f\n",SINE_TOTAL_COUNT);
            DataBuf[0][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentPosition;
            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentReference;
            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentVelocity;
            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference;
            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentVelocity;


            DataBuf[6][saveIndex]= sharedSEN2.FT[2].Mx;
            DataBuf[7][saveIndex]= sharedSEN2.FT[2].My;
            DataBuf[8][saveIndex]= sharedSEN2.FT[2].Fz;

            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;

            if(rt_count-240 >= SINE_TOTAL_COUNT)
            {
                saveFlag = 0;
                SaveFile();
                printf("I'm finished ...!!!\n");
                rt_count = 0;
                OPERATION_MODE = OPERATION_NONE;
            }
            break;
        }
        case OPERATION_UPDOWN:
        {
//            float cur_t =((float)rt_count)/200;
//            LegJoint Lj;
//            double y_ud;

//            int PWMff = 5.5*KneeV;
//            double F = sharedSEN2.FT[2].Fz;
//            dF = (F-old_F)*200.0;
//            old_F = F;
//            dF_f = dF_f*alpha+dF*(1-alpha);

//            if (cur_t>=1.2)
//            {
//                double y_init = -300;
//                double tnow = fmod(cur_t-1.2,UPDOWN_stepT*2);
//                Vector FPs;
//                double t1 = tnow;
//                double t2 = t1*tnow;
//                double t3 = t2*tnow;
//                double t4 = t3*tnow;
//                double t5 = t4*tnow;
//                if(tnow<UPDOWN_stepT*UPDOWN_upRatio)
//                {

//                    //                    FPs = calc_5th(0,UPDOWN_stepT*UPDOWN_upRatio,vec3(0,0,0),vec3(UPDOWN_upL*1000,0,0));
//                    Foot_Landed = false;
//                    GOflag = true;
//                    sharedREF->JointFFpwm[PODO_NO][MC_GetID(LKN)][MC_GetCH(LKN)] =0;
//                    ycon = 0.9*ycon;
//                }
//                else if(tnow<UPDOWN_stepT&&Foot_Landed==false)
//                {
//                    //gain override here
//                    if(GOflag==true&&tnow>UPDOWN_stepT*(UPDOWN_upRatio+(1-UPDOWN_upRatio)*0.5))
//                    {
//                        double g_o = 900;
//                        double v_s = 400;
//                        double a_c = 80;
//                        double v_d = 0;

//                        MCJointGainOverride(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,0);
//                        MCBoardSetSwitchingMode(0, JOINT_INFO[LKN].bno, 0x01);//non_complementary
//                        //                        MCsetFrictionParameter(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,v_s,a_c,v_d);
//                        //                        MCenableFrictionCompensation(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,1);

//                        GOflag = false;
//                    }
//                    sharedREF->JointFFpwm[PODO_NO][MC_GetID(LKN)][MC_GetCH(LKN)] =PWMff;
//                    //  printf("KneeV %f PWMff %d ttt %d\n",KneeV, PWMff,sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].PWMffout);


//                    //                    FPs = calc_5th(UPDOWN_stepT*UPDOWN_upRatio,UPDOWN_stepT,vec3(UPDOWN_upL*1000,0,0),vec3(0,0,0));
//                    if(Foot_Landed==false&&sharedSEN2.FT[2].Fz>60)//90 when nominal
//                    {
//                        Foot_Landed=true;
//                        if(GOflag==true)
//                        {
//                            double g_o = 1000;//no position control
//                            double v_s = 400;
//                            double a_c = 70;
//                            double v_d = 0;

//                            MCJointGainOverride(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,0);
//                            MCBoardSetSwitchingMode(0, JOINT_INFO[LKN].bno, 0x01);//non_complementary
//                            GOflag = false;
//                        }
//                        printf("foot_landed\n");
//                        landTime = tnow;
//                    }
//                    y_landed =
//                            + FPs[0]*t5
//                            + FPs[1]*t4
//                            + FPs[2]*t3
//                            + FPs[3]*t2
//                            + FPs[4]*t1
//                            + FPs[5];
//                    y_landed2 = y_landed;
//                }
//                else if(tnow<UPDOWN_stepT&&Foot_Landed==true)
//                {
//                    //FPs = calc_5th(landTime,UPDOWN_stepT,vec3(y_landed,200,0),vec3(y_landed+landUp,0,0));
//                    //                    FPs = calc_5th(landTime,UPDOWN_stepT,vec3(y_landed,y_upspeed,0),vec3(y_landed+landUp,0,0));
//                    //just stop
//                    y_landed2 = y_landed+landUp;

//                    double con = sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference
//                            - sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
//                    double tt_ff = 0.0;
//                    if(F<F_ref)
//                    {tt_ff =90.0;}
//                    double FF = tt_ff+kp*(F_ref-F) -kd*(dF_f) - 1/T*con;

//                    sharedREF->JointFFpwm[PODO_NO][MC_GetID(LKN)][MC_GetCH(LKN)] = -FF;//PWMff;
//                }
//                else
//                {
//                    if(tnow<UPDOWN_stepT*1.3)//control more
//                    {
//                        Foot_Landed=true;
//                        if(GOflag==true)
//                        {
//                            double g_o = 1000;//no position control
//                            double v_s = 400;
//                            double a_c = 70;
//                            double v_d = 0;

//                            MCJointGainOverride(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,0);
//                            MCBoardSetSwitchingMode(0, JOINT_INFO[LKN].bno, 0x01);//non_complementary
//                            GOflag = false;
//                        }
//                        //printf("foot_landed\n");

//                        //                         FPs = calc_5th(UPDOWN_stepT*1,UPDOWN_stepT*1.3,vec3(y_landed2,0,0),vec3(y_landed2,0,0));
//                        double con = sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference
//                                - sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
//                        double tt_ff = 0.0;
//                        if(F<F_ref)
//                        {tt_ff =90.0;}
//                        double FF = tt_ff+kp*(F_ref-F) -kd*(dF_f) - 1/T*con;
//                        sharedREF->JointFFpwm[PODO_NO][MC_GetID(LKN)][MC_GetCH(LKN)] = -FF;//PWMff;
//                    }
//                    else
//                    {
//                        if(GOflag==false)
//                        {
//                            MCenableFrictionCompensation(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,0);
//                            double g_o = 0;
//                            MCBoardSetSwitchingMode(0, JOINT_INFO[LKN].bno, 0x00);//complementary
//                            MCJointGainOverride(0,JOINT_INFO[LKN].bno,JOINT_INFO[LKN].mch,g_o,300);
//                            GOflag = true;
//                        }
//                        sharedREF->JointFFpwm[PODO_NO][MC_GetID(LKN)][MC_GetCH(LKN)] =0;
//                        //gain override off here
//                        //                        FPs = calc_5th(UPDOWN_stepT*1.3,UPDOWN_stepT*2,vec3(y_landed2,0,0),vec3(0,0,0));
//                        ycon = ycon*0.9;
//                        sharedREF->JointFFpwm[PODO_NO][MC_GetID(LKN)][MC_GetCH(LKN)] =0;
//                    }
//                    //return
//                }
//                y_ud = y_init
//                        + FPs[0]*t5
//                        + FPs[1]*t4
//                        + FPs[2]*t3
//                        + FPs[3]*t2
//                        + FPs[4]*t1
//                        + FPs[5]+ycon;


//                Lj = InverseKinematics(0,y_ud);
//                KneeV = ((-Lj.S2)-KNEE_old)/(RT_TIMER_PERIOD_MS/1000.0);

//                KNEE_old = -Lj.S2;

//                jCon->SetJointRefAngle(LHP,Lj.S1*100);
//                jCon->SetJointRefAngle(LKN,-Lj.S2*100);
//            }



//            DataBuf[0][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentPosition;
//            DataBuf[1][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentReference;
//            DataBuf[2][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].CurrentVelocity;
//            DataBuf[3][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentPosition;
//            DataBuf[4][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentReference;
//            DataBuf[5][saveIndex]= sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].CurrentVelocity;


//            DataBuf[6][saveIndex]= sharedSEN2.FT[2].Mx;
//            DataBuf[7][saveIndex]= sharedSEN2.FT[2].My;
//            DataBuf[8][saveIndex]= sharedSEN2.FT[2].Fz;

//            DataBuf[9][saveIndex]= 0;
//            DataBuf[10][saveIndex]= y_ud;

//            DataBuf[11][saveIndex] = sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].PWMin;
//            DataBuf[12][saveIndex] = sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].PWMin;
//            DataBuf[13][saveIndex] = sharedSEN2.ENCODER[MC_GetID(LHP)][MC_GetCH(LHP)].PWMffout;
//            DataBuf[14][saveIndex] = sharedSEN2.ENCODER[MC_GetID(LKN)][MC_GetCH(LKN)].PWMffout;
//            DataBuf[15][saveIndex] = ycon;
//            saveIndex++;

//            if(saveIndex==100000)
//                saveIndex=0;

//            if(rt_count>=200*(1.2+UPDOWN_stepT*2*5))
//            {
//                saveFlag = 0;
//                SaveFile();
//                printf("finished 2...!!!\n");
//                rt_count = 0;
//                OPERATION_MODE = OPERATION_NONE;
//            }

            break;
        }

        default:
            break;
        }

        rt_count++;

        /*
        if(saveFlag == 1){
            DataBuf[0][saveIndex]=DEBUG;
            DataBuf[1][saveIndex]=3263;
            DataBuf[2][saveIndex]=1234;

            saveIndex++;

            if(saveIndex==100000)
                saveIndex=0;
        }
        */

        jCon->MoveAllJoint();
        rt_task_suspend(&rtTaskCon);
    }
}

//==============================//
// Flag Thread
//==============================//
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 10*1000);        // 300 usec

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

        if(HasAnyOwnership()){
            if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
                sharedSEN2 = *sharedSEN;
                jCon->JointUpdate();
                rt_task_resume(&rtTaskCon);
            }
        }
    }
}


//==============================//

void SaveFile(void)
{
    FILE* fp;
    unsigned int i, j;
    fp = fopen("Seungwoo_DATA.txt", "w");

    for(i=0 ; i<saveIndex ; i++)
    {
        for(j=0 ; j<num_data_type ; j++)
            fprintf(fp, "%f\t", DataBuf[j][i]);
        fprintf(fp, "\n");
    }
    fclose(fp);

    saveIndex=0;
    saveFlag=0;
}

//==============================//
