#ifndef MANUALCAN_H
#define MANUALCAN_H
#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/JointInformation.h"

#include "rbdl/rbdl.h"
//#include "LIGHT_var_and_func.h"
#include "LIGHT_info.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// Manual CAN for GainOverride
#define SW_MODE_COMPLEMENTARY           0x00
#define SW_MODE_NON_COMPLEMENTARY       0x01

using namespace std;

const struct {
    int canch;          // CAN Channel
    int bno;            // Board Number
    int mch;            // Motor Channel
} JOINT_INFO[NO_OF_JOINTS] = {
    {0,0,0}, {0,1,0}, {1,2,0},      // RHR, RHY, RHP,
    {0,3,0}, {0,4,0}, {0,5,0},      // RKN, RAP, RAR,
    {0,6,0}, {0,7,0}, {0,8,0},      // LHR, LHY, LHP,
    {0,9,0}, {0,10,0},{0,11,0},     // LKN, LAP, LAR,
    {0,12,0}
};


extern int	PushCANMessage(MANUAL_CAN MCData);

extern int MCSendFindhome(unsigned int _canch, unsigned int _bno,  unsigned int _VCh);
extern int MCJointENCZero(unsigned int _canch, unsigned int _bno,  unsigned int _VCh);
extern int MCSendPWMZero(unsigned int _canch, unsigned int _bno,  unsigned int _VCh);
extern int MCLoadCellsensorNull(unsigned int _canch, unsigned int _bno);

extern int MCJointSetControlMode(unsigned int _canch, unsigned int _bno, unsigned int _VCh, int _mode);
extern int MCSendPositionRef(unsigned int _canch, unsigned int _bno, short _pREF1, short _pREF2);
extern int MCSendVelocityRef(unsigned int _canch, unsigned int _bno, short _vREF1, short _vREF2);
extern int MCSendForceRef(unsigned int _canch, unsigned int _bno, short _fREF1, short _fREF2);
extern int MCSendSupplyPressure(unsigned int _canch, unsigned int _bno, short pres_s);
extern int MCSendOpenloopPWM(unsigned int _canch, unsigned int _bno, short PWM1, short PWM2);

extern int MC_IMU_RESET(void);
extern int MC_IMU_NULLING(void);
extern int MC_IMU_ENABLE(void);


#endif // MANUALCAN_H
