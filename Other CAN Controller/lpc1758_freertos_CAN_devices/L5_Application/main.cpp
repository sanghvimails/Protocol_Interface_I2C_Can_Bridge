
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "LPC17xx.h"
#include "stdio.h"
#include "stdint.h"
#include "can.h"
#include "portable.h"
#include "io.hpp"

#define baudrate 100
#define rx_queue 10
#define tx_queue 10

//changing the controller
#define controller1 0       //change to 1 to enable light sensor controller
#define controller2 1      //change to 1 to enable temperature sensor controller

#if controller1
#define MY_ID 7             //Light controller ID
//Dummy message IDs
#define MSG_ID0  0x0
#define MSG_ID1  0x1
#define MSG_ID2  0x2
#define MSG_ID3  0x3
#define MSG_ID4  0x4
#define MSG_ID5  0x5
#define MSG_ID6  0x6
#define MSG_ID7  0x7
#endif

#if controller2
#define MY_ID 6             //Temperature controller ID
//Dummy message ID's
#define MSG_ID0  0x1E
#define MSG_ID1  0x1C
#define MSG_ID2  0x1A
#define MSG_ID3  0x18
#define MSG_ID4  0x16
#define MSG_ID5  0x14
#define MSG_ID6  0x12
#define MSG_ID7  0xE
#endif

void CAN_read_task (void)       //CAN read task
{
    while(1)
    {
        can_msg_t rxmsg;
        rxmsg.data.qword = 0;               //reset the data field
        if(CAN_rx(can1, &rxmsg, 10))        //read from the CAN bus
        {
            printf("Id  : %x\n", rxmsg.msg_id); //Print the recived CAN ID

            if((rxmsg.msg_id>>8) == MY_ID)      //If the destination is this controller
            {
                if(rxmsg.data.bytes[0] > 99)
                    LD.setNumber((char)99);     //if received data is >99, print 99 on the display
                else
                    LD.setNumber((char)rxmsg.data.bytes[0]);    //Print the received dat on the 7segment displaty
                //print the received 8 byte data
//                printf("Data: %x", rxmsg.data.bytes[0]);
//                printf(" %x", rxmsg.data.bytes[1]);
//                printf(" %x", rxmsg.data.bytes[2]);
//                printf(" %x", rxmsg.data.bytes[3]);
//                printf(" %x", rxmsg.data.bytes[4]);
//                printf(" %x", rxmsg.data.bytes[5]);
//                printf(" %x", rxmsg.data.bytes[6]);
//                printf(" %x\n", rxmsg.data.bytes[7]);
            }
        }
    }

}
//Task to send CAN messages
//This task sends continuous CAN messages over the CAN bus. The 8 byte data is dummy bytes
class can2Task : public scheduler_task
{
   public:
       can2Task(uint8_t priority) :
           scheduler_task("can", 512*4, priority)
       {
           CAN_init(can1, baudrate, rx_queue, tx_queue, NULL, NULL);    //Initialize the CAN bus
           CAN_bypass_filter_accept_all_msgs();                         //Disable hardware acceptance filter
           CAN_reset_bus(can1);
       }
       bool run(void *p)
       {
           can_msg_t txmsg, rxmsg;
           uint8_t src, dst, msg_id;
           txmsg.frame_fields.is_29bit = 0;     //CAN IDs are 11 bit
           txmsg.frame_fields.data_len = 8;     //CAN data length is 8 bytes
#if controller1
               txmsg.data.qword = (int) LS.getRawValue();       //read the Light sensor reading
               printf("light sensor reading %x\n", (int) LS.getRawValue());
#endif
#if controller2
               txmsg.data.qword = (int) TS.getFarenheit();      //read the Temperature sensor reading
               printf("temp sensor reading %x\n", (int) TS.getFarenheit());
#endif
               src = MY_ID;         //Fill the source field
               dst = 1;             //Destination is the other controller (connected to the protocol interface via I2C)
               msg_id = MSG_ID0;
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);   //Make the 11 bit CAN ID
           if(CAN_tx(can1, &txmsg, portMAX_DELAY))              //send the data over CAN bus
               {
//                   puts("sent 1");
               }
           //Code for sendign dummy data
               src = MY_ID;
               dst = 1;
               msg_id = MSG_ID1;
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);
               txmsg.data.qword = 0x01;
               vTaskDelay(600);
               CAN_tx(can1, &txmsg, portMAX_DELAY);
               msg_id = MSG_ID2;
               vTaskDelay(600);
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);
               txmsg.data.qword = 0x02;
               vTaskDelay(600);
               CAN_tx(can1, &txmsg, portMAX_DELAY);
               msg_id = MSG_ID3;
               vTaskDelay(600);
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);
               txmsg.data.qword = 0x03;
               vTaskDelay(600);
               CAN_tx(can1, &txmsg, portMAX_DELAY);
               msg_id = MSG_ID4;
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);
               txmsg.data.qword = 0x04;
               vTaskDelay(600);
               CAN_tx(can1, &txmsg, portMAX_DELAY);
               msg_id = MSG_ID5;
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);
               txmsg.data.qword = 0x05;
               vTaskDelay(600);
               CAN_tx(can1, &txmsg, portMAX_DELAY);
               msg_id = MSG_ID6;
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);
               txmsg.data.qword = 0x06;
               vTaskDelay(600);
               CAN_tx(can1, &txmsg, portMAX_DELAY);
               msg_id = MSG_ID7;
               txmsg.msg_id = ((dst<<8) | (src<<5) | msg_id);
               txmsg.data.qword = 0x07;
               vTaskDelay(600);
               CAN_tx(can1, &txmsg, portMAX_DELAY);

           return true;
       }
};

int main(void)
{
    TaskHandle_t test1 = NULL;
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));
    scheduler_add_task(new can2Task(PRIORITY_HIGH));
    xTaskCreate((void(*)(void *))CAN_read_task, "i2c_receive_task", 1024, NULL, PRIORITY_HIGH, &test1);
    scheduler_start(); ///< This shouldn't return
    return -1;
}
