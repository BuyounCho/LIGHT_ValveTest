#ifndef MYVARIABLES
#define MYVARIABLES


/*----------------------------------------------------------------------
 * Constant varialbes
 ---------------------------------------------------------------------*/
extern double      LIM_HP_U;
extern double      LIM_HP_L;
extern double      LIM_KN_U;
extern double      LIM_KN_L;
extern double      PI;
extern double      D2R;
extern double      R2D;

extern float       DEBUG;
/*----------------------------------------------------------------------
 * Trapezoidal parameters
 ---------------------------------------------------------------------*/
extern float       POS_X;
extern float       POS_Y;
extern float       TIME;
extern float       VMAX;
/*----------------------------------------------------------------------
 * Spring Mass Model parameters
 ---------------------------------------------------------------------*/
extern float       SPRING_CONSTANT;
extern float       INITIAL_POS;
extern float       INITIAL_VEL;
extern float       DELAY_TIME;
extern float       AERIAL_SEG_NUM;
extern float       MAX_ITERATION;
// extern float       FLOOR_POS = 0;
/*----------------------------------------------------------------------
 * SawProfile/TrapezoidalProfile parameters
 ---------------------------------------------------------------------*/
extern float       DECEL_1;
extern float       DECEL_2;
extern float       TAKE_OFF_POS;
extern float       START_POS;
extern float       TAKE_OFF_VEL;
extern float       LANDING_VEL;
extern float       PERIOD;
extern float       STOP_TIME;
extern float       CYCLE_NUM;
extern int         JUMP_MODE;
extern float       ITERATION;
/*----------------------------------------------------------------------
 * Sine Profile parameters
 ---------------------------------------------------------------------*/
extern float        SINE_INIT_POS;
extern float        SINE_MAG;
extern float        SINE_FREQ;
extern unsigned int SINE_TOTAL_COUNT;
/*----------------------------------------------------------------------
 * Saving
 ---------------------------------------------------------------------*/
extern unsigned int saveFlag;
extern unsigned int saveIndex;
extern int num_data_type;
extern int num_data_idx;
extern float DataBuf[100][150000];

void SaveFile(void);

extern float JS_CURRENT_REF_AMP;
extern float JS_CURRENT_REF_FREQ;

extern double BITELE_SIN_AMP;
extern double BITELE_SIN_FREQ;

extern double Master_Cur;
extern double Master_Pos;
extern double Master_Vel;

extern double Slave_Cur;
extern double Slave_Pos;
extern double Slave_Vel;

extern double N_gear;

extern double lastVel;

extern double HAP_3ch_Pgain;
extern double HAP_3ch_Dgain;

extern double BITELE_Pgain_Master;
extern double BITELE_Dgain_Master;

extern double temp_gain_master;
extern double temp_gain_slave;


extern double passivity_obs;
extern double passivity_obs2;
extern double passivity_obs3;
extern double passivity_obs4;

extern double est_stiff;

extern double pos_force_ratio;

extern double JTS_DESIRED_TIME;
extern double JTS_DESIRED_ANGLE;

extern double JTS_SENSOR_1;
extern double JTS_SENSOR_2;
extern double JTS_SENSOR_1_OFFSET;
extern double JTS_SENSOR_2_OFFSET;

extern double BITELE_JTS;


extern unsigned int rt_count;


extern double CT_AMP;
extern double CT_START_POS;
extern double CT_STAND_TIME;
extern double CT_SWING_TIME;
extern double CT_IMPACT_TIME;
extern double CT_NUM_REPETITION;
extern double CT_ALPHA_SMOOTH;


extern float        VAR_SINE_INIT_POS;
extern float        VAR_SINE_MAG;
extern float        VAR_SINE_INIT_FREQ;
extern float        VAR_SINE_FINAL_FREQ;
extern float        VAR_SINE_TIME;
extern unsigned int VAR_SINE_TOTAL_COUNT;

extern int          EVAL_MODE;
extern float        TORQUE_CONSTANT;
extern float        BACK_EMF;
extern float        REF_CURRENT_Q;
extern float        REF_CURRENT_D;
extern float        REF_TORQUE;

extern float        TORQUE_SUM; // 4 seconds
extern float        TORQUE_OFFSET;



/*----------------------------------------------------------------------
 * Command
 ---------------------------------------------------------------------*/
enum QUBO_COMMAND
{
    QUBO_NO_ACT = 100,
    QUBO_GOTO_POS,
    QUBO_SAVE,
    QUBO_MOVE_TRAPEZOIDAL,
    QUBO_MOVE_JUMP,
    QUBO_MOVE_SPRINGMASSMODEL,
    QUBO_MOVE_SINE,
    QUBO_GAIN_OVERRIDE_FRICTION_COMPENSATION,
    QUBO_GAIN_OVERRIDE_FRICTION_COMPENSATION_OFF,
    QUBO_UPDOWN_TEST_START,
    QUBO_UPDOWN_TEST_STOP,
};
/*----------------------------------------------------------------------
 * Real Time Thread Command
 ---------------------------------------------------------------------*/
extern int OPERATION_MODE;
enum RT_COMMAND
{
    OPERATION_NONE = 0,
    OPERATION_TRAPEZOIDAL,
    OPERATION_JUMP,
    OPERATION_SPRINGMASSMODEL,
    OPERATION_SINE,
    OPERATION_UPDOWN,

    OPERATION_EXT_SIN = 100,
    OPERATION_EXT_SIN2,
    OPERATION_EXT_TEST,

    OPERATION_HAP_3CH = 200,

    OPERATION_JTS_SETMOVEJOINT = 300,
    OPERATION_JTS_OFFSETNULLING,

    OPERATION_BITELE_EXT_TORQUE_ESTIMATION = 400,
    OPERATION_BITELE_SAVE_ONLY,
    OPERATION_BITELE_POSITION_POSITION,
    OPERATION_BITELE_POSITION_LOCK,
    OPERATION_BITELE_SINE_TEST,

    OPERATION_HUMANOID_0_TO_90 = 500,
    OPERATION_HUMANOID_90_TO_0,
    OPERATION_CT_RD_MOT,

    OPERATION_CYCLOID_SINE_POS_TEST = 600,
    OPERATION_CYCLOID_SINE_CUR_TEST,
    OPERATION_CYCLOID_VAR_SINE_POS_TEST,
    OPERATION_CYCLOID_SINE_STOP,
    OPERATION_CYCLOID_EVAL_POSITION,
    OPERATION_CYCLOID_EVAL_TORQUE,
    OPERATION_CYCLOID_EVAL_DYNAMO,
    OPERATION_CYCLOID_EVAL_TORQUE_NULLING

};


#endif // MYVARIABLES

