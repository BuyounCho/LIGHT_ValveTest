#ifndef LIGHT3DWALKING_COMMANDS
#define LIGHT3DWALKING_COMMANDS

// Command Set =========================================
enum _LIGHT_PARAMETER_TYPES
{
    NO_PARAMTER = 0,

    CHECK_PARAMETERS,

    SUPPORTCONTROL_DSP,
    SUPPORTCONTROL_RSSP,
    SUPPORTCONTROL_LSSP,
    SUPPORTCONTROL_FLOAT,
    JOINGIMPEDANCE_RF,
    JOINGIMPEDANCE_LF,
    CoMLEADCOMPENSATE,
    ANKLETORQUECOMPENSATE

};

// Operation Mode =========================================
enum _OPERATION_MODE_TYPE
{
    _OPERATION_NO = 200,

    // Joint space control
    _OPERATION_GOTO_HOMEPOSE,

    // Task space control - Base is floating
    _OPERATION_WORKSPACE_MOVING_TEST,
    _OPERATION_AIRWALKING,

    // Task space control - Contact to ground
    _OPERATION_CoM_MOVING_RDSP,
    _OPERATION_CoM_MOVING_LDSP,
    _OPERATION_CoM_MOVING_RSSP,
    _OPERATION_CoM_MOVING_LSSP,
    _OPERATION_SUPPORT_TRANSITION,
    _OPERATION_SYSID_COM,

    _OPERATION_SQUAT_DSP,
    _OPERATION_SQUAT_SSP,

    _OPERATION_COM_MOVING_RDSP_SMOOTH,
    _OPERATION_COM_MOVING_LDSP_SMOOTH,

    _OPERATION_RFSWINGUP_DYNAMIC,
    _OPERATION_RFSWINGDOWN_DYNAMIC,
    _OPERATION_LFSWINGUP_DYNAMIC,
    _OPERATION_LFSWINGDOWN_DYNAMIC,

    _OPERATION_WALK,
    _OPERATION_WALK_withJOYSTICK,

    _OPERATION_JUMPTEST,

    _OPERATION_LIGHT_FULL_SCINARIO

};

#else
#endif // LIGHT3DWALKING_COMMANDS

