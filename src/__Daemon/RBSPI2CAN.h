#ifndef RB_SPI2CAN_H
#define RB_SPI2CAN_H

#include "RBThread.h"
#include "RBCAN_new.h"
#include "../../share/Headers/JointInformation.h"

#include <linux/spi/spidev.h>

//#define MAX_CAN_PER_MSG     4
#define MAX_BYTE_PER_MSG    200


//typedef union{
//    ST_CAN  can_msg[MAX_CAN_PER_MSG];
//    unsigned char data[MAX_BYTE_PER_MSG];
//}U_MSG;


class rb_spi2can
{
    rb_spi2can();

    bool Initialized;

    int spi_1_fd;
    int spi_2_fd;

    int init_spi();

public:
    static rb_spi2can * getInstance(){
        static rb_spi2can obj;
        return &obj;
    }
    bool IsInitialized() {return Initialized;}

private:
    int         thread_id;
    pthread_t   thread_handler;
    static void *spi2can_thread(void *arg);
};

#endif // RB_SPI2CAN_H
