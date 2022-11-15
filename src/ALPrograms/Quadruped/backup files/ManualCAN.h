#ifndef MANUALCAN_H
#define MANUALCAN_H
#include "../../../share/Headers/RBSharedMemory.h"
#include "../../../share/Headers/JointInformation.h"

// Manual CAN for GainOverride
#define SW_MODE_COMPLEMENTARY           0x00
#define SW_MODE_NON_COMPLEMENTARY       0x01

const struct {
    int canch;          // CAN Channel
    int bno;            // Board Number
    int mch;            // Motor Channel
} JOINT_INFO[NO_OF_JOINTS] = {
    {1,8,1}, {1,9,1}     // LHP, LKN,
};


int	PushCANMessage(MANUAL_CAN MCData);
int MCJointGainOverride(unsigned int _canch, unsigned int _bno, int _mch, int logscale, short _msec);
int MCBoardSetSwitchingMode(unsigned int _canch, unsigned int _bno, int _mode);
int MCenableFrictionCompensation(unsigned int _canch, unsigned int _bno, unsigned int _mch, unsigned int _enable);
int MCsetFrictionParameter(unsigned int _canch, unsigned int _bno,
                                    unsigned int _mch, short _vel_saturation, int _amp_compen, int _vel_dead);
int MCJointEnableFeedbackControl(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _enable);
int MCJointSetControlMode(unsigned int _canch, unsigned int _bno, unsigned int _mch, int _mode);
int MCJointPWMCommand2chHR(unsigned int _canch, unsigned int _bno, int mode1, short duty1, int mode2, short duty2);
int MCWristFTsensorNull(unsigned int _canch, unsigned int _bno);
int MCJointRequestEncoderPosition(unsigned int _canch, unsigned _bno, int _mode);

#endif // MANUALCAN_H
