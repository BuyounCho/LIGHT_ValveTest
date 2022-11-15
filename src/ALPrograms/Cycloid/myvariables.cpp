#include "myvariables.h"
/*----------------------------------------------------------------------
 * Constant varialbes
 ---------------------------------------------------------------------*/
double      LIM_HP_U = 5300;
double      LIM_HP_L = 0;
double      LIM_KN_U = 0;
double      LIM_KN_L = -5400;
double      PI = 3.141592;
double      D2R = PI/180;
double      R2D = 180/PI;

float       DEBUG = 0;
/*----------------------------------------------------------------------
 * Trapezoidal parameters
 ---------------------------------------------------------------------*/
float       POS_X = 0;
float       POS_Y = 0;
float       TIME = 0;
float       VMAX = 0;
/*----------------------------------------------------------------------
 * Spring Mass Model parameters
 ---------------------------------------------------------------------*/
float       SPRING_CONSTANT = 0;
float       INITIAL_POS = 0;
float       INITIAL_VEL = 0;
float       DELAY_TIME = 0;
float       AERIAL_SEG_NUM = 0;
float       MAX_ITERATION = 0;
// float       FLOOR_POS = 0;
/*----------------------------------------------------------------------
 * SawProfile/TrapezoidalProfile parameters
 ---------------------------------------------------------------------*/
float       DECEL_1 = 0;
float       DECEL_2 = 0;
float       TAKE_OFF_POS = 0;
float       START_POS = 0;
float       TAKE_OFF_VEL = 0;
float       LANDING_VEL = 0;
float       PERIOD = 0;
float       STOP_TIME = 0;
float       CYCLE_NUM = 0;
int         JUMP_MODE = 0;
float       ITERATION = 0;
/*----------------------------------------------------------------------
 * Sine Profile parameters
 ---------------------------------------------------------------------*/
float           SINE_INIT_POS = 0;
float           SINE_MAG = 0;
float           SINE_FREQ = 0;
unsigned int    SINE_TOTAL_COUNT = 0;
/*----------------------------------------------------------------------
 * Saving
 ---------------------------------------------------------------------*/
unsigned int saveFlag=0;
unsigned int saveIndex=0;
int num_data_type = 100;
int num_data_idx = 150000;
float DataBuf[100][150000];
void SaveFile(void);
 int OPERATION_MODE= 0;
 unsigned int rt_count = 0;


 /*----------------------------------------------------------------------
  * Jaesung
  ---------------------------------------------------------------------*/
 float JS_CURRENT_REF_AMP=0;
 float JS_CURRENT_REF_FREQ=0;

 double BITELE_SIN_AMP = 0.0;
 double BITELE_SIN_FREQ = 0.0;

 double Master_Cur = 0.0;
 double Master_Pos = 0.0;
 double Master_Vel = 0.0;

 double Slave_Cur = 0.0;
 double Slave_Pos = 0.0;
 double Slave_Vel = 0.0;

 double N_gear = 50.0;

 double lastVel = 0.0;
 double HAP_3ch_Pgain = 0.0;
 double HAP_3ch_Dgain = 0.0;

 double temp_gain_master = 0.0;
 double temp_gain_slave = 0.0;

 double BITELE_Pgain_Master = 0.0;
 double BITELE_Dgain_Master = 0.0;

 double passivity_obs = 0.0;
 double passivity_obs2 = 0.0;
 double passivity_obs3 = 0.0;
 double passivity_obs4 = 0.0;



 double est_stiff = 0.0;

 double pos_force_ratio = 0.0;

 double BITELE_JTS = 0.0;

 /*----------------------------------------------------------------------
  * JTS
  ---------------------------------------------------------------------*/
double JTS_DESIRED_TIME = 0.0;
double JTS_DESIRED_ANGLE = 0.0;

double JTS_SENSOR_1 = 0.0;
double JTS_SENSOR_2 = 0.0;
double JTS_SENSOR_1_OFFSET = 0.0;
double JTS_SENSOR_2_OFFSET = 0.0;

/*----------------------------------------------------------------------
 * CT (COCOA TEST)
 ---------------------------------------------------------------------*/


double CT_AMP = 0.0;
double CT_START_POS = 0.0;
double CT_STAND_TIME = 0.0;
double CT_SWING_TIME = 0.0;
double CT_IMPACT_TIME = 0.0;
double CT_NUM_REPETITION = 0.0;
double CT_ALPHA_SMOOTH = 0.0;


/*----------------------------------------------------------------------
 * SEUNGWOO (CYCLOID TEST)
 ---------------------------------------------------------------------*/
float VAR_SINE_INIT_POS = 0.0;
float VAR_SINE_MAG = 0.0;
float VAR_SINE_INIT_FREQ = 0.0;
float VAR_SINE_FINAL_FREQ = 0.0;
float VAR_SINE_TIME = 0.0;
unsigned int VAR_SINE_TOTAL_COUNT = 0;

int   EVAL_MODE = 0;
float TORQUE_CONSTANT = 0.0;
float BACK_EMF = 0.0;
float REF_CURRENT_Q = 0.0;
float REF_CURRENT_D = 0.0;
float REF_TORQUE = 0.0;
float TORQUE_SUM = 0.0;
float TORQUE_OFFSET = 0.0;
