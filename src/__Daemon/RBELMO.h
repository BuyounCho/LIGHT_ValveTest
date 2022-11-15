#ifndef RBELMO_H
#define RBELMO_H

#include <fcntl.h>
#include <termios.h>
//#include <math.h>
#include <cmath>
#include <pthread.h>


#include <unistd.h>

#include "RBLog.h"
//#include <alchemy/task.h>

#define RS232_RECEIVE_DATA_BUFFER_SIZE	2000

enum RS_232_STATE
{
    RS_232_CLOSED,
    RS_232_OPENED,
    RS_232_ERROR
};

class RBELMO
{
public:
    RBELMO();
    ~RBELMO();

    // RS-232 ----
    int     RS232STATE;
    uint    StoredDataIndex;
    uint    ReadDataIndex;
    char    NewDataAvailable;
    char    ReceivedData[RS232_RECEIVE_DATA_BUFFER_SIZE];
    char    ReceivedByte;
    char    WantedByte;
    int     RS232DeviceHandler;


    // functions for serial communication
    int     RBOpenPort(int baud, int USBnum);
    int     RBClosePort(void);
    int     RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode);
    int     RBWritePort(const char *_uart_frame, uchar _bytes);
    int     RBGetReceivedDataByte(char *_data);

    int     RBgetSerial();
    int     RBEnable();
    int     RBDisable();
    int     RBSetCurrent(double current);
    bool    isEnabled(){return Enabled;}
    double MaxC;

private:
    int         isWorking;
    bool        Enabled;
    ulong     ReceiveThreadHandler;
    static void RBFOG_ReadThread(void *_arg);


    int     clearBuf(void);
};

#endif // RBELMO
