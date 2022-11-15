#ifndef RBCAN_H
#define RBCAN_H

#include "RBLog.h"
#include "RBDataType.h"
#include "RBThread.h"

//#include <libpcan.h>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>


//#include <alchemy/task.h>


#define 	RBCAN_MAX_CAN_CHANNEL   10 //Seungwoo: 4 -> 6
#define 	RBCAN_MAX_MB			150
#define     MAX_SEARCH_CHANNEL      10

typedef enum{
    RBCAN_NODATA = 0,
    RBCAN_NEWDATA,
    RBCAN_OVERWRITE,
    RBCAN_SENTDATA
} RBCAN_STATUS;

typedef struct _RBCAN_MB_
{
    uint    id;         // Identifier
    uchar   data[8];    // Data
    uchar   dlc;        // Data Length Code
    uchar   status;     // MB status
    uchar   channel;    // CAN channel (0, 1, 2, ...)
    void printMB()
    {
        std::cout<<"id "<<id<<" Data "<<
                   (uint)data[0]<<" "<<(uint)data[1]<<" "<<
                   (uint)data[2]<<" "<<(uint)data[3]<<" "<<
                   (uint)data[4]<<" "<<(uint)data[5]<<" "<<
                   (uint)data[6]<<" "<<(uint)data[7]<<
                   " dlc "<<(uint)dlc<<std::endl;
    }
} RBCAN_MB, *pRBCAN_MB;



class RBCAN
{
public:
    RBCAN(int _ChNum = 7);
    ~RBCAN();

    void    Finish();
    void    RBResetCAN();

    // Loading mail box configuration
    int                 RBCAN_WriteData(RBCAN_MB _mb);
    int                 RBCAN_WriteDataDirectly(RBCAN_MB _mb);
    int                 RBCAN_ReadData(pRBCAN_MB _mb);
    int                 RBCAN_AddMailBox(unsigned int _id);
    int                 RBCAN_WriteEnable(int _suspend);
    int                 RBCAN_SeeData(pRBCAN_MB _mb);

    bool                IsWorking();
    void				*canHandler[RBCAN_MAX_CAN_CHANNEL];

private:
    // member functions
    int                 RBCAN_StartThread(void);
    static void         *RBCAN_ReadThread(void *_arg);
    static void         *RBCAN_WriteThread(void *_arg);
    int					RBCAN_GetMailBoxIndex(unsigned int _id);
    // member variables
    int					isWorking;
//    RT_TASK             canReadThreadHandler;
    ulong				canWriteThreadHandler;
    RBCAN_MB		canReadMB[RBCAN_MAX_MB];
    RBCAN_MB		canWriteMB[RBCAN_MAX_MB];

    int					chNum;
    int					canMBCounter;
    int					canHeadIndex;
    int					canTailIndex;
    int					canSendSuspend;

    int                 isSuspend;
};



#endif // RBCAN_H
