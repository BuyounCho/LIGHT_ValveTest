// This JointInformation.h is for DRC-HUBO+

#ifndef JOINT_INFORMATION_H
#define JOINT_INFORMATION_H

#include <QVector>
#include <QString>

enum JointSequentialNumber
{
    RHR = 0, RHY, RHP,
    RKN, RAP, RAR,
    LHR, LHY, LHP,
    LKN, LAP, LAR,
    WST, NO_OF_JOINTS
};
const int PEL = NO_OF_JOINTS;

const QString JointNameList[NO_OF_JOINTS] = {
    "RHR", "RHY", "RHP",
    "RKN", "RAP", "RAR",
    "LHR", "LHY", "LHP",
    "LKN", "LAP", "LAR",
    "WST"
};

const struct {
    int id;
    int ch;
} MC_ID_CH_Pairs[NO_OF_JOINTS] = {
    {0,0}, {1,0}, {2,0},
    {3,0}, {4,0}, {5,0},
    {6,0}, {7,0}, {8,0},
    {9,0}, {10,0}, {11,0},
    {12,0}
};

inline int MC_GetID(int jnum){
    return MC_ID_CH_Pairs[jnum].id;
}
inline int MC_GetCH(int jnum){
    return MC_ID_CH_Pairs[jnum].ch;
}

#endif // JOINT_INFORMATION_H
