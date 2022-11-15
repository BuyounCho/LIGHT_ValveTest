#include "RBELMO.h"


using namespace std;

RBELMO::RBELMO()
{
    RS232DeviceHandler = 0;
    isWorking = false;
    Enabled = false;
    MaxC = 20.0;//later 20.0
}

RBELMO::~RBELMO(){
    isWorking = false;
    usleep(500*1000);
}


int RBELMO::RBDisable()
{
    char toSend[] = "MO=0;";//MOTOR DISABLE
    RBWritePort(toSend,5);
    Enabled = false;
    return 0;
}
int RBELMO::RBEnable()
{
    char toSend[] = "MO=1;";//MOTOR ENABLE
    RBWritePort(toSend,5);
    Enabled = true;
    return 0;
}
int RBELMO::RBSetCurrent(double current)
{

    char toSend[20];
    if(std::isnan(current)) current = 0.0;
    if(current>MaxC) current = MaxC;
    if(current<-MaxC) current = -MaxC;
    sprintf(toSend,"TC=%.2f;",current);
    RBWritePort(toSend,strlen(toSend));
    return 0;
}
int RBELMO::RBgetSerial()
{
    char toSend[] = "EO=0;";//disable echo
    RBWritePort(toSend,5);
    char toSend2[] = "UM=1;";//current control
    RBWritePort(toSend2,5);
    usleep(500*1000);
    while(RBGetReceivedDataByte(toSend)==1){ usleep(10*1000);}
    char toSend3[] = "SN[4];";//ask SERIAL
    RBWritePort(toSend3,6);
    usleep(500*1000);
    while(RBGetReceivedDataByte(toSend)==1){ usleep(10*1000);}
    char toSend4[] = "SN[4];";//ask SERIAL
    RBWritePort(toSend4,6);
    usleep(500*1000);
    char toRecv[11] = {0,};
    for(int i=0;i<8;i++)
    {
       RBGetReceivedDataByte(&(toRecv[i]));
    }
    printf(toRecv);
    printf("\n");



    return atoi(toRecv);
}

int RBELMO::RBOpenPort(int baud, int USBnum){
    int port_fd, ret;//, ret1, ret2;
    struct termios settings;

    char port_name[50];
    int i = USBnum;
    //for(i=0; i<5; i++){
        sprintf(port_name, "/dev/ttyUSB%d",i);
        port_fd = open(port_name, O_RDWR|O_NONBLOCK|O_NOCTTY);
        if(port_fd == -1){
            RS232STATE = RS_232_ERROR;
            FILE_LOG(logWARNING) << "RBFOG: RS232 OPEN FAILED.";
            return RS_232_ERROR;
        }else{
            fcntl(port_fd, F_SETFL, 0);
            RS232DeviceHandler = port_fd;

            // get current port setting
            tcgetattr(RS232DeviceHandler, &settings);

            //settng the baud rate add error checking
            //ret1 = cfsetispeed(&settings, baud);
            //ret2 = cfsetospeed(&settings, baud);
            //if(ret1==-1 && ret2==-1) return -2;

            //set local mode & enable receiver
            settings.c_cflag        = baud | CS8 | CLOCAL | CREAD;
            settings.c_iflag        = IGNPAR;
            settings.c_oflag        = 0;
            settings.c_lflag        = 0;
            settings.c_cc[VMIN]     = 0;
            settings.c_cc[VTIME]    = 0;

            ret = tcsetattr(RS232DeviceHandler, TCSANOW, &settings);
            if(ret == -1){
                RS232STATE = RS_232_ERROR;
                return RS_232_ERROR;
            }

            char TASKNAME[] = "RBELMO_READ_TASK0";
            TASKNAME[16] = '0' + i;

            if(rt_task_create(&ReceiveThreadHandler, TASKNAME, 0, 94, 0) == 0){
                cpu_set_t aCPU;
                CPU_ZERO(&aCPU);
                CPU_SET(1, &aCPU);
                if(rt_task_set_affinity(&ReceiveThreadHandler, &aCPU) != 0){
                    FILE_LOG(logWARNING) << "RBFOG: Read thread set affinity CPU failed..";
                }
                if(rt_task_start(&ReceiveThreadHandler, &RBFOG_ReadThread, this) == 0){

                }else{
                    FILE_LOG(logERROR) << "RBFOG: Read thread Creation Error";
                    RS232STATE = RS_232_ERROR;
                    return RS_232_ERROR;
                }
            }else{
                FILE_LOG(logERROR) << "RBFOG: Read thread Creation Error";
                RS232STATE = RS_232_ERROR;
                return RS_232_ERROR;
            }
            //break;
        }
    //}


    FILE_LOG(logSUCCESS) << "ELMO Port Open Success [ttyUSB" << i << "]";
    RS232STATE = RS_232_OPENED;
    return RS_232_OPENED;
}

int RBELMO::RBClosePort(void){
    isWorking = false;
    usleep(100*1000);

    rt_task_delete(&ReceiveThreadHandler);
    return true;
}
int RBELMO::RBReadPort(char *_uart_frame, uchar _bytes, uchar _mode){
    uchar receivedByte = 0;
    uchar index = 0;
    uint loopTime = 0;
    char receivedData[20];
    uchar i;

    if(_mode == 0x00){
        usleep(10000);
        return read(RS232DeviceHandler, _uart_frame, _bytes);
    }else{
        while(receivedByte < _bytes){
            usleep(10000);

            index = read(RS232DeviceHandler, &receivedData[index], 20);
            if(index > 0){
                for(i=0 ; i<index ; i++) _uart_frame[receivedByte+i] = receivedData[i];
                receivedByte += index;
            }

            if(loopTime > 50) return -1;
            else loopTime++;
        }
        return receivedByte;
    }
}

int RBELMO::RBWritePort(const char *_uart_frame, uchar _bytes){

    return write(RS232DeviceHandler, _uart_frame, _bytes);
}

int RBELMO::clearBuf(void){
    char temp[20];

    while(RBReadPort(temp, 20, 0x00) > 0);
    StoredDataIndex = ReadDataIndex = 0;

    return 1;
}

int RBELMO::RBGetReceivedDataByte(char *_data){
    if(StoredDataIndex%RS232_RECEIVE_DATA_BUFFER_SIZE != ReadDataIndex){
        *_data = ReceivedData[ReadDataIndex];
        ReadDataIndex++;

        ReadDataIndex %= RS232_RECEIVE_DATA_BUFFER_SIZE;
        return 1;
    }
    else return -1;
}


void RBELMO::RBFOG_ReadThread(void *_arg)
{
    RBELMO *fog = (RBELMO *)_arg;
    fog->isWorking = true;

    uchar index = 0;
    char tempData;

    char state=0;
    char rcv;


    rt_task_set_periodic(NULL, TM_NOW, 2*1000000);

    while(fog->isWorking)
    {
        rt_task_wait_period(NULL);

        while(true){
            index = read(fog->RS232DeviceHandler, &tempData, 1);

            if(index == 1){//read_and_store
                fog->StoredDataIndex %= RS232_RECEIVE_DATA_BUFFER_SIZE;
                fog->ReceivedData[fog->StoredDataIndex] = tempData;
                fog->StoredDataIndex += index;
            }else{
                break;
            }
            if(fog->StoredDataIndex>100)
            {
                fog->RBGetReceivedDataByte(&tempData);
            }

//            while(fog->RBGetReceivedDataByte(&rcv) == 1){

//            }
        }
    }
}

