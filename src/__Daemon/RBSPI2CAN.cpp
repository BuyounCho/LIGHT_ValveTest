#include "RBSPI2CAN.h"

#include "HydraulicActuatorController.h"
#include "HydraulicActuatorDataConverting.h"
#include "HydraulicPumpController.h"
#include "RBIMUSensor.h"
#include "RBFTSensor.h"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <vector>

extern pRBCORE_SHM_REFERENCE        sharedREF;
extern pRBCORE_SHM_SENSOR           sharedSEN;

extern ValveController              _DEV_VC[MAX_VC];
extern PumpController               _DEV_PC[MAX_PC];
extern RBIMUSensor                  _DEV_IMU[MAX_IMU];
extern RBFTSensor                   _DEV_FT[MAX_FT];

#define SPI_SPEED 4000000
unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned char spi_speed = SPI_SPEED;
unsigned char lsb = 0x01;


void ReadCAN_VC_General(unsigned char *data, int i);
void ReadCAN_VC_ActuatorData(unsigned char *data, int i);
void ReadCAN_VC_ValvePosnPWM(unsigned char *data, int i);
void ReadCAN_VC_Pressure(unsigned char *data, int i);
void ReadCAN_VC_OtherInfo(unsigned char *data, int i);
void ReadCAN_VC_Alart(unsigned char *data, int i);

void ReadCAN_PC_General(unsigned char *data, int i);
void ReadCAN_PC_PumpingData(unsigned char *data, int i);

void ReadCAN_IMU_General(unsigned char *data, int i);
void ReadCAN_IMU_Quaternion(unsigned char *data, int i);
void ReadCAN_IMU_LocalX(unsigned char *data, int i);
void ReadCAN_IMU_LocalY(unsigned char *data, int i);
void ReadCAN_IMU_LocalZ(unsigned char *data, int i);

void ReadCAN_FT_General(unsigned char *data, int i);
void ReadCAN_FT_MxMyFz(unsigned char *data, int i);
void ReadCAN_FT_FxFyMz(unsigned char *data, int i);

using namespace std;
rb_spi2can::rb_spi2can()
{
#ifdef EXTERNAL
    spi_1_fd = -1;
    spi_2_fd = -1;

    if(init_spi() != 0){
        cout << "init spi failed" << endl;
        Initialized = false;
    } else {
        //        cout << "init spi succeed" << endl;
        Initialized = true;
        thread_id = generate_rt_thread_hard(thread_handler, spi2can_thread, "spi2can", 3, 96, this);
    }
#else
    Initialized = true;
#endif

}

int rb_spi2can::init_spi(){
    int rv = 0;
    spi_1_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 0.0");
    spi_2_fd = open("/dev/spidev0.1", O_RDWR);
    if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 0.1");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
    rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");

    rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");

    rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
    if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (2)");
    return rv;
}

void *rb_spi2can::spi2can_thread(void *arg){
    const long PERIOD_US = SYS_DT_CAN_US;
    struct timespec TIME_NEXT;
    struct timespec TIME_NOW;
    unsigned long dead_miss_cnt = 0;

    unsigned int tt = 0;
    rb_spi2can *spi = (rb_spi2can*)arg;

    struct spi_ioc_transfer spi_tr1;
    struct spi_ioc_transfer spi_tr2;
    memset(&spi_tr1, 0, sizeof(struct spi_ioc_transfer));
    memset(&spi_tr2, 0, sizeof(struct spi_ioc_transfer));

    unsigned char tx_1[MAX_BYTE_PER_MSG] = {0,};
    unsigned char tx_2[MAX_BYTE_PER_MSG] = {0,};
    unsigned char rx_1[MAX_BYTE_PER_MSG] = {0,};
    unsigned char rx_2[MAX_BYTE_PER_MSG] = {0,};

    spi_tr1.tx_buf = (unsigned long)tx_1;
    spi_tr1.rx_buf = (unsigned long)rx_1;
    spi_tr1.len = MAX_BYTE_PER_MSG;
    spi_tr1.speed_hz = SPI_SPEED;
    spi_tr1.delay_usecs = 0;
    spi_tr1.bits_per_word = spi_bits_per_word;
    spi_tr1.cs_change = 1;

    spi_tr2.tx_buf = (unsigned long)tx_2;
    spi_tr2.rx_buf = (unsigned long)rx_2;
    spi_tr2.len = MAX_BYTE_PER_MSG;
    spi_tr2.speed_hz = SPI_SPEED;
    spi_tr2.delay_usecs = 0;
    spi_tr2.bits_per_word = spi_bits_per_word;
    spi_tr2.cs_change = 1;

    QByteArray recv_buf1;
    QByteArray recv_buf2;
    int count[12] = {0,};

    int cnt_test = 0;

    usleep(500*1000);

    //---------------------------------------------------------------------------------------------
    clock_gettime(CLOCK_REALTIME, &TIME_NEXT);
    while(true){

        //        static int CAN_ThreadCycle = 0;
        //        static timespec TIME_TIC;
        //        static timespec TIME_TOC;
        //        static int CNT_TICTOC = 0;
        //        CNT_TICTOC++;
        //        if(CNT_TICTOC%2==0) {
        //            clock_gettime(CLOCK_REALTIME, &TIME_TIC);
        //            CAN_ThreadCycle = timediff_us(&TIME_TOC, &TIME_TIC);
        //            cout << "Cycle Time (CAN Thread) : " << (double)CAN_ThreadCycle*0.001 << " ms " << endl;
        //            CNT_TICTOC = 0;
        //        } else {
        //            clock_gettime(CLOCK_REALTIME, &TIME_TOC);
        //            CAN_ThreadCycle = timediff_us(&TIME_TIC, &TIME_TOC);
        //            cout << "Cycle Time (CAN Thread) : " << (double)CAN_ThreadCycle*0.001 << " ms " << endl;
        //        }

        //////////////////////////////////////////////////////////////////////////////
        /// Transmitting Data
        //////////////////////////////////////////////////////////////////////////////

        // Reference Transmit Data ----------------------------------------------

        // Valve Controller
        static int idx1 = 0;
        static int idx2 = 0;
        for(int i=0; i<MAX_VC; i++){
            if(rb_can::set_reference_msg_VC(i)) {
                pthread_mutex_lock(&rb_can::mutex_reference_VC[i]);
                int _CH = _DEV_VC[i].CAN_CHANNEL;
                if(_CH==0) // SPI 0
                {
                    rb_can::reference_msg_VC[i].header = 0x89;
                    memcpy(&(tx_1[i*12]), &(rb_can::reference_msg_VC[i]), 12);
                }
                if(_CH==1) // SPI 0
                {
                    rb_can::reference_msg_VC[i].header = 0x77;
                    memcpy(&(tx_1[i*12]), &(rb_can::reference_msg_VC[i]), 12);
                }
                if(_CH==2)
                {
                    rb_can::reference_msg_VC[i].header = 0x89;
                    memcpy(&(tx_2[i*12]), &(rb_can::reference_msg_VC[i]), 12);
                }
                if(_CH==3)
                {
                    rb_can::reference_msg_VC[i].header = 0x77;
                    memcpy(&(tx_2[i*12]), &(rb_can::reference_msg_VC[i]), 12);
                }
                pthread_mutex_unlock(&rb_can::mutex_reference_VC[i]);
            }
        }

        // Pump Controller
        for(int i=0; i<MAX_PC; i++){
            if(rb_can::set_reference_msg_PC(i)) {
                pthread_mutex_lock(&rb_can::mutex_reference_PC[i]);
                int _CH = _DEV_PC[i].CAN_CHANNEL;
                if(_CH==0) // SPI 0
                {
                    rb_can::reference_msg_PC[i].header = 0x89;
                    memcpy(&(tx_1[MAX_BYTE_PER_MSG-12*(i+1)]), &(rb_can::reference_msg_PC[i]), 12);
                }
                if(_CH==1) // SPI 0
                {
                    rb_can::reference_msg_PC[i].header = 0x77;
                    memcpy(&(tx_1[MAX_BYTE_PER_MSG-12*(i+1)]), &(rb_can::reference_msg_PC[i]), 12);
                }
                if(_CH==2)
                {
                    rb_can::reference_msg_PC[i].header = 0x89;
                    memcpy(&(tx_2[MAX_BYTE_PER_MSG-12*(i+1)]), &(rb_can::reference_msg_PC[i]), 12);
                }
                if(_CH==3)
                {
                    rb_can::reference_msg_PC[i].header = 0x77;
                    memcpy(&(tx_2[MAX_BYTE_PER_MSG-12*(i+1)]), &(rb_can::reference_msg_PC[i]), 12);
                }
                pthread_mutex_unlock(&rb_can::mutex_reference_PC[i]);
            }
        }

        // General Command Transmit Data (SPI 0) : Valve & Pump ----------------------------------------
        if(rb_can::general_send_msgs1.size() > 0){
            ST_CAN temp_can = rb_can::general_send_msgs1[0];
            rb_can::general_send_msgs1.pop_front();
            memcpy(&(tx_1[12*MAX_VC]), &temp_can, 12);
        }else{
            memset(&(tx_1[12*MAX_VC]), 0, 12);
        }

        // General Command Transmit Data (SPI 1) : Valve & Pump ----------------------------------------
        if(rb_can::general_send_msgs2.size() > 0){
            ST_CAN temp_can = rb_can::general_send_msgs2[0];
            rb_can::general_send_msgs2.pop_front();
            memcpy(&(tx_2[12*MAX_VC]), &temp_can, 12);
        }else{
            memset(&(tx_2[12*MAX_VC]), 0, 12);
        }

        //////////////////////////////////////////////////////////////////////////////
        /// Receiving Data
        //////////////////////////////////////////////////////////////////////////////

        int rv1 = ioctl(spi->spi_1_fd, SPI_IOC_MESSAGE(1), &spi_tr1);
        int rv2 = ioctl(spi->spi_2_fd, SPI_IOC_MESSAGE(1), &spi_tr2);

        recv_buf1.append((const char*)rx_1, MAX_BYTE_PER_MSG);
        recv_buf2.append((const char*)rx_2, MAX_BYTE_PER_MSG);

        while(recv_buf1.size() >= 12){
            if(uchar(recv_buf1[0]) == 0x89) { // Channel 0,1

                int dlc = recv_buf1[1];
                uchar id1 = recv_buf1[2];
                uchar id2 = recv_buf1[3];
                ushort id = (ushort)id1 | ((ushort)id2<<8);
                unsigned char recv_data1[8];
                for(int i=0; i<8; i++){
                    recv_data1[i] = recv_buf1[4+i];
                }
                recv_buf1.remove(0, 12);

                // Classify CAN-RX Data with ID
                // 1100 ~ 1112 : [ValveController] General Information
                // 1200 ~ 1212 : [ValveController] Pos Vel Tor
                // 1300 ~ 1312 : [ValveController] Valve Opening and PWM
                // 1400 ~ 1412 : [ValveController] Pressure Data
                // 1500 ~ 1512 : [ValveController] Other Information
                // 1600 ~ 1612 : [ValveController] Alart Message
                // 1199        : [PumpController] General Info.
                // 1299        : [PumpController] Velocity
                // 1499        : [PumpController] Pressure
                // 82 ~ 86     : [IMU]

                // ============== Valve Controller ID ===========================

                // General Data (Requested Data)
                if(id>=_DEV_VC[0].ID_RCV_GENERAL && id<=_DEV_VC[MAX_VC-1].ID_RCV_GENERAL) {
                    int BNO = id - _DEV_VC[0].ID_RCV_GENERAL;
                    ReadCAN_VC_General(recv_data1, BNO);
                }
                // Position, Velocity, Force Data
                if(id>=_DEV_VC[0].ID_RCV_POSVEL && id<=_DEV_VC[MAX_VC-1].ID_RCV_POSVEL) {
                    int BNO = id - _DEV_VC[0].ID_RCV_POSVEL;
                    ReadCAN_VC_ActuatorData(recv_data1, BNO);
                }
                // Valve Position & PWM Data
                if(id>=_DEV_VC[0].ID_RCV_VALVEPOSnPWM && id<=_DEV_VC[MAX_VC-1].ID_RCV_VALVEPOSnPWM) {
                    int BNO = id - _DEV_VC[0].ID_RCV_VALVEPOSnPWM;
                    ReadCAN_VC_ValvePosnPWM(recv_data1, BNO);
                }
                // Pressure Data
                if(id>=_DEV_VC[0].ID_RCV_PRESSURE && id<=_DEV_VC[MAX_VC-1].ID_RCV_PRESSURE) {
                    int BNO = id - _DEV_VC[0].ID_RCV_PRESSURE;
                    ReadCAN_VC_Pressure(recv_data1, BNO);
                }
                // Other Information (for debugging)
                if(id>=_DEV_VC[0].ID_RCV_OTHERINFO && id<=_DEV_VC[MAX_VC-1].ID_RCV_OTHERINFO) {
                    int BNO = id - _DEV_VC[0].ID_RCV_OTHERINFO;
                    ReadCAN_VC_OtherInfo(recv_data1, BNO);
                }
                // Alart Message
                if(id>=_DEV_VC[0].ID_RCV_ALART && id<=_DEV_VC[MAX_VC-1].ID_RCV_ALART) {
                    int BNO = id - _DEV_VC[0].ID_RCV_ALART;
                    ReadCAN_VC_Alart(recv_data1, BNO);
                }

                // ============== Pump Controller ID ============================
                if(id==_DEV_PC[0].ID_RCV_GENERAL) { ReadCAN_PC_General(recv_data1, 0); }
                if(id==_DEV_PC[0].ID_RCV_VELOCITY) { ReadCAN_PC_PumpingData(recv_data1, 0); }

                // ============== IMU ID ============================
                if(id==_DEV_IMU[0].ID_RCV_GENERAL) { ReadCAN_IMU_General(recv_data1, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_QUAT) { ReadCAN_IMU_Quaternion(recv_data1, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_LOCAL_X) { ReadCAN_IMU_LocalX(recv_data1, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_LOCAL_Y) { ReadCAN_IMU_LocalY(recv_data1, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_LOCAL_Z) { ReadCAN_IMU_LocalZ(recv_data1, 0); }

                // ============== FTSensor ID ============================
                for(int i=0;i<MAX_FT;i++) {
                    if(id==_DEV_FT[i].ID_RCV_GENERAL) { ReadCAN_FT_General(recv_data1, i);}
                    if(id==_DEV_FT[i].ID_RCV_MXMYFZ) { ReadCAN_FT_MxMyFz(recv_data1, i); }
                    if(id==_DEV_FT[i].ID_RCV_FXFYMZ) { ReadCAN_FT_FxFyMz(recv_data1, i); }
                }

            }else{
                recv_buf1.remove(0, 1);
            }
        }

        while(recv_buf2.size() >= 12){
            if(uchar(recv_buf2[0]) == 0x89){ // Channel 2,3

                int dlc = recv_buf2[1];
                uchar id1 = recv_buf2[2];
                uchar id2 = recv_buf2[3];
                ushort id = (ushort)id1 | ((ushort)id2<<8);
                unsigned char recv_data2[8];
                for(int i=0; i<8; i++){
                    recv_data2[i] = recv_buf2[4+i];
                }
                recv_buf2.remove(0, 12);


                // General Data (Requested Data)
                if(id>=_DEV_VC[0].ID_RCV_GENERAL && id<=_DEV_VC[MAX_VC-1].ID_RCV_GENERAL) {
                    int BNO = id - _DEV_VC[0].ID_RCV_GENERAL;
                    ReadCAN_VC_General(recv_data2, BNO);
                }
                // Position, Velocity, Force Data
                if(id>=_DEV_VC[0].ID_RCV_POSVEL && id<=_DEV_VC[MAX_VC-1].ID_RCV_POSVEL) {
                    int BNO = id - _DEV_VC[0].ID_RCV_POSVEL;
                    ReadCAN_VC_ActuatorData(recv_data2, BNO);
                }
                // Valve Position & PWM Data
                if(id>=_DEV_VC[0].ID_RCV_VALVEPOSnPWM && id<=_DEV_VC[MAX_VC-1].ID_RCV_VALVEPOSnPWM) {
                    int BNO = id - _DEV_VC[0].ID_RCV_VALVEPOSnPWM;
                    ReadCAN_VC_ValvePosnPWM(recv_data2, BNO);
                }
                // Pressure Data
                if(id>=_DEV_VC[0].ID_RCV_PRESSURE && id<=_DEV_VC[MAX_VC-1].ID_RCV_PRESSURE) {
                    int BNO = id - _DEV_VC[0].ID_RCV_PRESSURE;
                    ReadCAN_VC_Pressure(recv_data2, BNO);
                }
                // Other Information (for debugging)
                if(id>=_DEV_VC[0].ID_RCV_OTHERINFO && id<=_DEV_VC[MAX_VC-1].ID_RCV_OTHERINFO) {
                    int BNO = id - _DEV_VC[0].ID_RCV_OTHERINFO;
                    ReadCAN_VC_OtherInfo(recv_data2, BNO);
                }
                // Alart Message
                if(id>=_DEV_VC[0].ID_RCV_ALART && id<=_DEV_VC[MAX_VC-1].ID_RCV_ALART) {
                    int BNO = id - _DEV_VC[0].ID_RCV_ALART;
                    ReadCAN_VC_Alart(recv_data2, BNO);
                }

                // ============== Pump Controller ID ============================
                if(id==_DEV_PC[0].ID_RCV_GENERAL) { ReadCAN_PC_General(recv_data2, 0); }
                if(id==_DEV_PC[0].ID_RCV_VELOCITY) { ReadCAN_PC_PumpingData(recv_data2, 0); }

                // ====================== IMU ID ============================
                if(id==_DEV_IMU[0].ID_RCV_GENERAL) { ReadCAN_IMU_General(recv_data2, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_QUAT) { ReadCAN_IMU_Quaternion(recv_data2, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_LOCAL_X) { ReadCAN_IMU_LocalX(recv_data2, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_LOCAL_Y) { ReadCAN_IMU_LocalY(recv_data2, 0); }
                if(id==_DEV_IMU[0].ID_RCV_DATA_LOCAL_Z) { ReadCAN_IMU_LocalZ(recv_data2, 0); }

                // ============== FTSensor ID ============================
                for(int i=0;i<MAX_FT;i++) {
                    if(id==_DEV_FT[i].ID_RCV_GENERAL) { ReadCAN_FT_General(recv_data2, i); }
                    if(id==_DEV_FT[i].ID_RCV_MXMYFZ) { ReadCAN_FT_MxMyFz(recv_data2, i); }
                    if(id==_DEV_FT[i].ID_RCV_FXFYMZ) { ReadCAN_FT_FxFyMz(recv_data2, i); }
                }

            }else{
                recv_buf2.remove(0, 1);
            }
        }

        clock_gettime(CLOCK_REALTIME, &TIME_NOW);
        timespec_add_us(&TIME_NEXT, PERIOD_US);
        //        if(timespec_cmp(&TIME_NOW, &TIME_NEXT) > 0){
        //            cout << "RT Deadline Miss, SPI " << ++dead_miss_cnt << endl;
        //        }
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &TIME_NEXT, NULL);
    }
    //---------------------------------------------------------------------------------------------
}

/////////////////////////////////////////////////////////////////
///  RX_CAN Data Handling (Valve Controller)
/////////////////////////////////////////////////////////////////

void ReadCAN_VC_General(unsigned char *data, int i)
{
    switch(data[0]) {
    case ValveController_GeneralMSG_CANCHECK:
        _DEV_VC[i].ConnectionStatus = true;
        break;
    case ValveController_GeneralMSG_ASK_BOARDNUMBER:
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board ID : " << (int)(data[1]) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        _DEV_VC[i].ConnectionStatus = true;
        break;
    case ValveController_GeneralMSG_ASK_BOARDOPERATIONMODE:
        _DEV_VC[i].Joints[0].HCB_Info.OPERATION_MODE = data[1];
        _DEV_VC[i].Joints[0].HCB_Info.SENSING_MODE = data[2];
        _DEV_VC[i].Joints[0].HCB_Info.CURRENT_CONTROL_MODE = data[3];
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board Oper. Mode : " << (int)(_DEV_VC[i].Joints[0].HCB_Info.OPERATION_MODE) << std::endl;
        //        std::cout << "Board Sensing Mode : " << (int)(_DEV_VC[i].Joints[0].HCB_Info.SENSING_MODE) << std::endl;
        //        std::cout << "Board Current Ctrl. Mode : " << (int)(_DEV_VC[i].Joints[0].HCB_Info.CURRENT_CONTROL_MODE) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_CANFREQ:
        _DEV_VC[i].Joints[0].HCB_Info.CAN_FREQ = (int16_t)((data[1])|(data[2]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board CAN Frequency : " << (int)(_DEV_VC[i].Joints[0].HCB_Info.CAN_FREQ) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_CTRLMODE:
        _DEV_VC[i].Joints[0].HCB_Info.CONTROL_MODE = (int)(data[1]);
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board Control Mode : " << (int)(_DEV_VC[i].Joints[0].HCB_Info.CONTROL_MODE) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_JOINTENCDIR:
        _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_DIRECTION = (int16_t)((data[1])|(data[2]<<8)); // 1 : Positive / -1 : Negative
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board Joint Enc Dir. : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINTENC_DIRECTION) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_VALVEINPUTDIR:
        _DEV_VC[i].Joints[0].HCB_Info.VALVEINPUT_DIRECTION = (int16_t)((data[1])|(data[2]<<8)); // 1 : Positive / 0 : Negative
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board Valve Input Dir. : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVEINPUT_DIRECTION) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_VALVEENCDIR:
        _DEV_VC[i].Joints[0].HCB_Info.VALVEENC_DIRECTION = (int16_t)((data[1])|(data[2]<<8)); // 1 : Positive / 0 : Negative
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board Valve Enc Dir. : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVEENC_DIRECTION) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_BOARDINPUTVOLTAGE:
    {
        int temp_V = (int)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.BOARD_IN_VOLTAGE = ((double)temp_V)/10.0;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board Supply Voltage : " << (_DEV_VC[i].Joints[0].HCB_Info.BOARD_IN_VOLTAGE) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_VALVEOPERVOLTAGE:
    {
        int temp_V = (int)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.BOARD_OPER_VOLTAGE = ((double)temp_V)/10.0;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Board Valve Operation Voltage : " << (_DEV_VC[i].Joints[0].HCB_Info.BOARD_OPER_VOLTAGE) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_VARIABLESUPPLY_ONOFF:
        _DEV_VC[i].Joints[0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF = (int)(data[1]);
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Variable Supply Pressure Mode : " << (_DEV_VC[i].Joints[0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_PIDGAIN:
        if (data[1]==0) {     // data[1] = 0 : Valve Position Gain
            int16_t temp_P = (int16_t)((data[2])|(data[3]<<8));
            int16_t temp_I = (int16_t)((data[4])|(data[5]<<8));
            int16_t temp_D = (int16_t)((data[6])|(data[7]<<8));
            _DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KP = (double)temp_P;
            _DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KI = (double)temp_I;
            _DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KD = (double)temp_D;
            //            std::cout << "=================================== " << std::endl;
            //            std::cout << "Valve Pos. P Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KP) << std::endl;
            //            std::cout << "Valve Pos. I Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KI) << std::endl;
            //            std::cout << "Valve Pos. D Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVE_P_KD) << std::endl;
            //            std::cout << "=================================== " << std::endl;
        } else if (data[1]==1) {     // data[1] = 1 : Joint Position Gain
            int16_t temp_P = (int16_t)((data[2])|(data[3]<<8));
            int16_t temp_I = (int16_t)((data[4])|(data[5]<<8));
            int16_t temp_D = (int16_t)((data[6])|(data[7]<<8));
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KP = (double)temp_P;
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KI = (double)temp_I;
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KD = (double)temp_D;
            //            std::cout << "=================================== " << std::endl;
            //            std::cout << "Joint Pos. P Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KP) << std::endl;
            //            std::cout << "Joint Pos. I Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KI) << std::endl;
            //            std::cout << "Joint Pos. D Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_P_KD) << std::endl;
            //            std::cout << "=================================== " << std::endl;
        } else if (data[1]==2) {     // data[1] = 2 : Joint Force/torque Gain
            int16_t temp_P = (int16_t)((data[2])|(data[3]<<8));
            int16_t temp_I = (int16_t)((data[4])|(data[5]<<8));
            int16_t temp_D = (int16_t)((data[6])|(data[7]<<8));
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KP = (double)temp_P;
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KI = (double)temp_I;
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KD = (double)temp_D;
            //            std::cout << "=================================== " << std::endl;
            //            std::cout << "Joint Force P Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KP) << std::endl;
            //            std::cout << "Joint Force I Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KI) << std::endl;
            //            std::cout << "Joint Force D Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_F_KD) << std::endl;
            //            std::cout << "=================================== " << std::endl;
        } else if (data[1]==3) {     // data[1] = 3 : Joint Spring&Damper
            int16_t temp_SPRING = (int16_t)((data[2])|(data[3]<<8));
            int16_t temp_DAMPER = (int16_t)((data[4])|(data[5]<<8));
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_SPRING = ((double)temp_SPRING)/10.0;
            _DEV_VC[i].Joints[0].HCB_Info.JOINT_DAMPER = ((double)temp_DAMPER)/100.0;
            std::cout << "=================================== " << std::endl;
            std::cout << "Joint Spring Constant : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_SPRING) << std::endl;
            std::cout << "Joint Damper Constant : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINT_DAMPER) << std::endl;
            std::cout << "=================================== " << std::endl;
        }
        break;
    case ValveController_GeneralMSG_ASK_VALVEDZ:
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_CENTER_POS = (int16_t)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_DZ_PLUS    = (int16_t)((data[3])|(data[4]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_DZ_MINUS   = (int16_t)((data[5])|(data[6]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Valve Center Pos. : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVE_CENTER_POS) << std::endl;
        //        std::cout << "Valve Positive DeadZone Pos : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVE_DZ_PLUS) << std::endl;
        //        std::cout << "Valve Negative DeadZone Pos : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVE_DZ_MINUS) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_VELCOMP:
        _DEV_VC[i].Joints[0].HCB_Info.VEL_COMPENSATION_K = (int)((data[1])|(data[2]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Velocity Compensation Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.VEL_COMPENSATION_K) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_COMPLIANCE:
        _DEV_VC[i].Joints[0].HCB_Info.ACTUATOR_COMPLIANCE_K = (int)((data[1])|(data[2]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Actuator Compliance Gain : " << (_DEV_VC[i].Joints[0].HCB_Info.ACTUATOR_COMPLIANCE_K) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_FEEDFORWARD:
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_FEEDFORWARD = (int)((data[1])|(data[2]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Valve Feedforward : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVE_FEEDFORWARD) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_BULKMODULUS:
        _DEV_VC[i].Joints[0].HCB_Info.BULK_MODULUS = (int)((data[1])|(data[2]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Bulk Modulus : " << (_DEV_VC[i].Joints[0].HCB_Info.BULK_MODULUS) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_CHAMBERVOL:
        _DEV_VC[i].Joints[0].HCB_Info.VOL_A = (int)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.VOL_B = (int)((data[3])|(data[4]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Chamber A Volume : " << (_DEV_VC[i].Joints[0].HCB_Info.VOL_A) << std::endl;
        //        std::cout << "Chamber B Volume : " << (_DEV_VC[i].Joints[0].HCB_Info.VOL_B) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_PISTONAREA:
        _DEV_VC[i].Joints[0].HCB_Info.PIS_AREA_A = (int)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.PIS_AREA_B = (int)((data[3])|(data[4]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Piston A Area : " << (_DEV_VC[i].Joints[0].HCB_Info.PIS_AREA_A) << std::endl;
        //        std::cout << "Piston B Area : " << (_DEV_VC[i].Joints[0].HCB_Info.PIS_AREA_B) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_SUPnRETPRES:
        _DEV_VC[i].Joints[0].HCB_Info.SUP_PRES = (int)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.RET_PRES = (int)((data[3])|(data[4]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Supply Pressure : " << (_DEV_VC[i].Joints[0].HCB_Info.SUP_PRES) << std::endl;
        //        std::cout << "Return Pressure : " << (_DEV_VC[i].Joints[0].HCB_Info.RET_PRES) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_JOINTENCLIMIT:
        _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_LIMIT_MINUS = (int16_t)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_LIMIT_PLUS = (int16_t)((data[3])|(data[4]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Joint Enc. Negative Limit : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINTENC_LIMIT_MINUS) << std::endl;
        //        std::cout << "Joint Enc. Positive Limit : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINTENC_LIMIT_PLUS) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_PISTONSTROKE:
        _DEV_VC[i].Joints[0].HCB_Info.PIS_STROKE = (int)((data[1])|(data[2]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Piston Stroke : " << (_DEV_VC[i].Joints[0].HCB_Info.PIS_STROKE) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_VALVEPOSLIMIT:
        _DEV_VC[i].Joints[0].HCB_Info.VALVEPOS_LIMIT_MINUS = (int16_t)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.VALVEPOS_LIMIT_PLUS = (int16_t)((data[3])|(data[4]<<8));
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Valve Pos. Negative Limit : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVEPOS_LIMIT_MINUS) << std::endl;
        //        std::cout << "Valve Pos. Positive Limit : " << (_DEV_VC[i].Joints[0].HCB_Info.VALVEPOS_LIMIT_PLUS) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    case ValveController_GeneralMSG_ASK_ENCPPP:
    {
        int16_t temp = (int)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.JOINTENC_PPP = (double)temp;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Joint Encoder Pulse per Position : " << (_DEV_VC[i].Joints[0].HCB_Info.JOINTENC_PPP) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_SENPPF:
    {
        int temp = (int)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.FORCESEN_PPF = (double)temp*0.001;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Sensor Pulse per Force/Torque : " << (_DEV_VC[i].Joints[0].HCB_Info.FORCESEN_PPF) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_SENPPP:
    {
        int temp_A = (int)((data[1])|(data[2]<<8));
        int temp_B = (int)((data[3])|(data[4]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.PRESSEN_PPP_A = (double)temp_A*0.01;
        _DEV_VC[i].Joints[0].HCB_Info.PRESSEN_PPP_B = (double)temp_B*0.01;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Sensor Pulse per Pressure A port : " << (_DEV_VC[i].Joints[0].HCB_Info.PRESSEN_PPP_A) << std::endl;
        //        std::cout << "Sensor Pulse per Pressure B port : " << (_DEV_VC[i].Joints[0].HCB_Info.PRESSEN_PPP_B) << std::endl;
        //        std::cout << "=================================== " << std::endl;
    }
    case ValveController_GeneralMSG_ASK_CONSTFRICTION:
    {
        int temp_Fric = (int16_t)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.CONST_FRIC = ((double)temp_Fric)/10.0;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Constant Friction : " << (_DEV_VC[i].Joints[0].HCB_Info.CONST_FRIC) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_VALVEGAINPLUS:
    {
        double Valve_Gain_1 = (double)(data[1]) * 0.02;
        double Valve_Gain_2 = (double)(data[2]) * 0.02;
        double Valve_Gain_3 = (double)(data[3]) * 0.02;
        double Valve_Gain_4 = (double)(data[4]) * 0.02;
        double Valve_Gain_5 = (double)(data[5]) * 0.02;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[0] = Valve_Gain_1;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[1] = Valve_Gain_2;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[2] = Valve_Gain_3;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[3] = Valve_Gain_4;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_PLUS[4] = Valve_Gain_5;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Plus valve gain :" <<"   1V: "<< Valve_Gain_1 << "  2V: "<< Valve_Gain_2 << " 3V: "<< Valve_Gain_3 << "    4V: "<< Valve_Gain_4 << "   5V: "<< Valve_Gain_5 << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_VALVEGAINMINUS:
    {
        double Valve_Gain_1 = (double)(data[1]) * 0.02;
        double Valve_Gain_2 = (double)(data[2]) * 0.02;
        double Valve_Gain_3 = (double)(data[3]) * 0.02;
        double Valve_Gain_4 = (double)(data[4]) * 0.02;
        double Valve_Gain_5 = (double)(data[5]) * 0.02;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[0] = Valve_Gain_1;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[1] = Valve_Gain_2;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[2] = Valve_Gain_3;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[3] = Valve_Gain_4;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_GAIN_MINUS[4] = Valve_Gain_5;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "Plus valve gain :" <<"   1V: "<< Valve_Gain_1 << "  2V: "<< Valve_Gain_2 << "   3V: "<< Valve_Gain_3 << "   4V: "<< Valve_Gain_4 << "   5V: "<< Valve_Gain_5 << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_HOMEPOSOFFSET:
    {
        int16_t temp_HomeposOffset = (int16_t)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.HOMEPOS_OFFSET = temp_HomeposOffset;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "HomePos Offset : " << (_DEV_VC[i].Joints[0].HCB_Info.HOMEPOS_OFFSET) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_HOMEPOSOPENING:
    {
        int16_t temp_HomeposValveOpening = (int16_t)((data[1])|(data[2]<<8));
        _DEV_VC[i].Joints[0].HCB_Info.HOMEPOS_VALVE_OPENING = temp_HomeposValveOpening;
        //        std::cout << "=================================== " << std::endl;
        //        std::cout << "HomePos Valve Opening : " << (_DEV_VC[i].Joints[0].HCB_Info.HOMEPOS_VALVE_OPENING) << std::endl;
        //        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_VOLTAGE2VALVEPOS_RESULT:
    {
        int16_t temp_PWM_Valve_ID = (int16_t)((data[1])|(data[2]<<8));
        int16_t temp_Valve_Pos_Valve_ID = (int16_t)((data[3])|(data[4]<<8));
        int16_t temp_idx_ID = (int16_t)data[5];

        _DEV_VC[i].Joints[0].HCB_Info.VALVE_PWM_VALVE_ID[temp_idx_ID] = temp_PWM_Valve_ID;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_POS_VALVE_ID[temp_idx_ID] = temp_Valve_Pos_Valve_ID;
        std::cout << "=================================== " << std::endl;
        std::cout << "PWM_VALVE_ID (index = "<< temp_idx_ID << ") : " << temp_PWM_Valve_ID << std::endl;
        std::cout << "Valve_Pos_Valve_ID : " << temp_Valve_Pos_Valve_ID << std::endl;
        std::cout << "=================================== " << std::endl;
        break;
    }
    case ValveController_GeneralMSG_ASK_VALVEPOS2FLOWRATE_RESULT:
    {
        int16_t temp_ValvePos_ID = (int16_t)((data[1])|(data[2]<<8));
        int16_t temp_Flowrate_Valve_ID = (int16_t)((data[3])|(data[4]<<8));
        int16_t temp_idx_ID = (int16_t)data[5];

        _DEV_VC[i].Joints[0].HCB_Info.VALVE_POSITION_FLOWRATE_ID[temp_idx_ID] = (double)temp_ValvePos_ID/100.0;
        _DEV_VC[i].Joints[0].HCB_Info.VALVE_FLOWRATE_FLOWRATE_ID[temp_idx_ID] = (double)temp_Flowrate_Valve_ID/1000.0;
        std::cout << "=================================== " << std::endl;
        std::cout << "ValvePos (index = "<< temp_idx_ID << ") : " << _DEV_VC[i].Joints[0].HCB_Info.VALVE_POSITION_FLOWRATE_ID[temp_idx_ID] << std::endl;
        std::cout << "Flowrate : " << _DEV_VC[i].Joints[0].HCB_Info.VALVE_FLOWRATE_FLOWRATE_ID[temp_idx_ID] << std::endl;
        std::cout << "=================================== " << std::endl;
        break;
    }
    default:
        break;
    }
}

void ReadCAN_VC_ActuatorData(unsigned char *data, int i)
{
    // data : CAN Data
    // i : Board Number

    int16_t temp_pos, temp_vel, temp_force_pulse;
    temp_pos = (int16_t)((data[0])|(data[1]<<8)); // Unit : deg or mm
    temp_vel = (int16_t)((data[2])|(data[3]<<8)); // Unit : deg/s or mm/s
    temp_force_pulse = (int16_t)((data[4])|(data[5]<<8)); // Unit : sensor pulse (0~4095)

    // @ joint : maximum 100deg(or mm) > multiply 200
    //           maximum 1000deg/s(or mm/s) multiply 20
    // @ force sensor : maximum 4096 > multiply 10
    _DEV_VC[i].Joints[0].HCB_Data.CurrentPosition = (double)temp_pos/200.0;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentVelocity = (double)temp_vel/20.0;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentForce = (double)temp_force_pulse/10.0/_DEV_VC[i].PULSE_PER_FORCETORQUE;
}

void ReadCAN_VC_ValvePosnPWM(unsigned char *data, int i)
{
    // data : CAN Data
    // i : Board Number
    int16_t temp_INFO1 = (int16_t)((data[0])|(data[1]<<8));
    int16_t temp_INFO2 = (int16_t)((data[2])|(data[3]<<8));
    int16_t temp_INFO3 = (int16_t)((data[4])|(data[5]<<8));

    // @ Valve Position : 0~4000 (DDV)  /  -10000 ~ 10000 (TSV)
    _DEV_VC[i].Joints[0].HCB_Data.CurrentValvePos    = (float)temp_INFO1;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentValvePosRef = (float)temp_INFO2;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentPWM         = (float)temp_INFO3;
}

void ReadCAN_VC_Pressure(unsigned char *data, int i)
{
    // data : CAN Data
    // i : Board Number
    int16_t temp_INFO1 = (int16_t)((data[0])|(data[1]<<8));
    int16_t temp_INFO2 = (int16_t)((data[2])|(data[3]<<8));

    // @ Pressure : 0 ~ 210  > multiply 100
    _DEV_VC[i].Joints[0].HCB_Data.CurrentPressureA = ((double)temp_INFO1)/100.0;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentPressureB = ((double)temp_INFO2)/100.0;
}

void ReadCAN_VC_OtherInfo(unsigned char *data, int i)
{
    // data : CAN Data
    // i : Board Number
    int16_t temp_INFO1 = (int16_t)((data[0])|(data[1]<<8));
    int16_t temp_INFO2 = (int16_t)((data[2])|(data[3]<<8));
    int16_t temp_INFO3 = (int16_t)((data[4])|(data[5]<<8));
    int16_t temp_INFO4 = (int16_t)((data[6])|(data[7]<<8));

    _DEV_VC[i].Joints[0].HCB_Data.CurrentTempData1 = temp_INFO1;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentTempData2 = temp_INFO2;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentTempData3 = temp_INFO3;
    _DEV_VC[i].Joints[0].HCB_Data.CurrentTempData4 = temp_INFO4;
}

void ReadCAN_VC_Alart(unsigned char *data, int i)
{
    // data : CAN Data
    // i : Board Number


}

/////////////////////////////////////////////////////////////////
///  RX_CAN Data Handling (Pump Controller)
/////////////////////////////////////////////////////////////////

void ReadCAN_PC_General(unsigned char *data, int i)
{
    switch(data[0]) {
    case PumpController_GeneralMSG_CANCHECK:
        _DEV_PC[i].ConnectionStatus = true;
        break;
    case PumpController_GeneralMSG_ASK_SPEEDREF:
        std::cout << "=================================== " << std::endl;
        std::cout << "Pump Speed Reference : " << (int)(data[1]) << std::endl;
        std::cout << "=================================== " << std::endl;
        break;
    case PumpController_GeneralMSG_ASK_CTRL_ONOFF:
        std::cout << "=================================== " << std::endl;
        if((int)data[1]) std::cout << "Control Mode : On" << std::endl;
        else  std::cout << "Control Mode : Off" << std::endl;
        std::cout << "=================================== " << std::endl;
        break;
    case PumpController_GeneralMSG_ASK_DATAREQUESTFLAG:
        std::cout << "=================================== " << std::endl;
        if((int)data[1]) std::cout << "Pressure Data Request : OFF" << std::endl;
        else std::cout << "Pressure Data Request : ON" << std::endl;
        std::cout << "=================================== " << std::endl;
        break;
    default:
        break;
    }
}

void ReadCAN_PC_PumpingData(unsigned char *data, int i)
{
    uint16_t temp_velocity_rpm = (uint16_t)((data[0])|(data[1]<<8));
    int16_t temp_pressure     = (int16_t)((data[2])|(data[3]<<8)); // Raw value (20210405)
    uint16_t temp_temperature  = (uint16_t)((data[4])|(data[5]<<8));

    _DEV_PC[i].CurrentVelocity     = (double)temp_velocity_rpm/10.0;
    _DEV_PC[i].CurrentTemperature  = (double)temp_temperature/10.0;

    double in = (double)temp_pressure;
    //        _DEV_PC[i].CurrentPressure = 0.07*in + 17.0; // Seunghoon's pump board (LIGHT2)
    //    _DEV_PC[i].CurrentPressure = 0.07336*in - 5.0; // Seunghoon's 2nd ver. pump board (LIGHT2)
    //    _DEV_PC[i].CurrentPressure = 0.060889*in + 3.35; // Seunghoon's 2nd ver. pump board (LIGHT2)
    _DEV_PC[i].CurrentPressure = 0.0534*in + 8.0; // Seunghoon's 2nd ver. pump board (LIGHT2)
}

/////////////////////////////////////////////////////////////////
///  RX_CAN Data Handling (IMU)
/////////////////////////////////////////////////////////////////

void ReadCAN_IMU_General(unsigned char *data, int i)
{
    // Just Check Connection Status
    _DEV_IMU[0].ConnectionStatus = true;
}

void ReadCAN_IMU_Quaternion(unsigned char *data, int i)
{
    // data : CAN Data
    int16_t temp_INFO1 = (int16_t)((data[0])|(data[1]<<8));
    int16_t temp_INFO2 = (int16_t)((data[2])|(data[3]<<8));
    int16_t temp_INFO3 = (int16_t)((data[4])|(data[5]<<8));
    int16_t temp_INFO4 = (int16_t)((data[6])|(data[7]<<8));
}

void ReadCAN_IMU_LocalX(unsigned char *data, int i)
{
    // data : CAN Data
    memcpy(&_DEV_IMU[0].wx_local,data,4);   // data[0]~data[3]
    memcpy(&_DEV_IMU[0].ax_local,data+4,4); // data[4]~data[7]
}

void ReadCAN_IMU_LocalY(unsigned char *data, int i)
{
    // data : CAN Data
    memcpy(&_DEV_IMU[0].wy_local,data,4);   // data[0]~data[3]
    memcpy(&_DEV_IMU[0].ay_local,data+4,4); // data[4]~data[7]
}

void ReadCAN_IMU_LocalZ(unsigned char *data, int i)
{
    // data : CAN Data
    memcpy(&_DEV_IMU[0].wz_local,data,4);   // data[0]~data[3]
    memcpy(&_DEV_IMU[0].az_local,data+4,4); // data[4]~data[7]
}

/////////////////////////////////////////////////////////////////
///  RX_CAN Data Handling (FT Sensor)
/////////////////////////////////////////////////////////////////

void ReadCAN_FT_General(unsigned char *data, int i)
{
    // Just Check Connection Status
    _DEV_FT[i].ConnectionStatus = true;
}

void ReadCAN_FT_MxMyFz(unsigned char *data, int i)
{
    _DEV_FT[i].MX = (double)((short)((data[2]<<8)|data[1]))/100.0f;
    _DEV_FT[i].MY = (double)((short)((data[4]<<8)|data[3]))/100.0f;
    _DEV_FT[i].FZ = (double)((short)((data[6]<<8)|data[5]))/10.0f;

    double alpha = 2.0*PI*_DEV_FT[i].CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001;
    _DEV_FT[i].MX_FILTERED = (1.0-alpha)*_DEV_FT[i].MX_FILTERED + alpha*_DEV_FT[i].MX;
    _DEV_FT[i].MY_FILTERED = (1.0-alpha)*_DEV_FT[i].MY_FILTERED + alpha*_DEV_FT[i].MY;
    _DEV_FT[i].FZ_FILTERED = (1.0-alpha)*_DEV_FT[i].FZ_FILTERED + alpha*_DEV_FT[i].FZ;
}

void ReadCAN_FT_FxFyMz(unsigned char *data, int i)
{
    _DEV_FT[i].FX = (double)((short)((data[2]<<8)|data[1]))/10.0f;
    _DEV_FT[i].FY = (double)((short)((data[4]<<8)|data[3]))/10.0f;
    _DEV_FT[i].MZ = (double)((short)((data[6]<<8)|data[5]))/100.0f;

    double alpha = 2.0*PI*_DEV_FT[i].CutOffFeq*(double)RT_TIMER_PERIOD_MS*0.001;
    _DEV_FT[i].FX_FILTERED = (1.0-alpha)*_DEV_FT[i].FX_FILTERED + alpha*_DEV_FT[i].FX;
    _DEV_FT[i].FY_FILTERED = (1.0-alpha)*_DEV_FT[i].FY_FILTERED + alpha*_DEV_FT[i].FY;
    _DEV_FT[i].MZ_FILTERED = (1.0-alpha)*_DEV_FT[i].MZ_FILTERED + alpha*_DEV_FT[i].MZ;
}
