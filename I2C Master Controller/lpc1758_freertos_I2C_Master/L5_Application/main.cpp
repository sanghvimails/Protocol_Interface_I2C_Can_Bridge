
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "LPC17xx.h"
#include "stdint.h"
#include "i2c2.hpp"
#include "io.hpp"
#include "r_I2C_CAN.hpp"
#include "ultrasonic.hpp"
#include "string.h"
#include "ff.h"
#include <storage.hpp>


#define DATA_LEN 8
//For I2C initialization
#define I2C_SPEED 100
#define SLAVE_ADD 0x50
#define SLAVE_ADD2 0x60

//Mailbox length
#define MAILBOX_LEN1 9
#define MAILBOX_LEN2 5

//Controller IDs
#define CAN_DST1 6
#define CAN_DST2 7

//Messages for sending CAN message
#define MSG_ID1 0x1F
#define MSG_ID2 0x1

#if 1        //Dummy CAN ID's defined for testing
//For controller A
#define A_CANF0 0x1E0
#define A_CANF1 0x1E1
#define A_CANF2 0x1E2
#define A_CANF3 0x1E3
#define A_CANF4 0x1E4
#define A_CANF5 0x1E5
#define A_CANF6 0x1E6
#define A_CANF7 0x1E7

//For controller B
#define B_CANF0 0x1D0
#define B_CANF1 0x1DE
#define B_CANF2 0x1DC
#define B_CANF3 0x1DA
#define B_CANF4 0x1D8
#define B_CANF5 0x1D6
#define B_CANF6 0x1D4
#define B_CANF7 0x1CE

#endif

#define MAILBOX_STATUS 0x01

uint16_t BASE_POINTER = 0;
uint8_t MY_ID = 1;
bool toggle = true;
SemaphoreHandle_t event_signal;
BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

extern int temp1, temp2;

void data_avail_isr()
{
//    puts("Inside ISR... resuming task");
    xSemaphoreGiveFromISR(event_signal, &pxHigherPriorityTaskWoken);    //GiveSemaphore when interrupt received
}

void read_from_slave()
{
    uint8_t rx_buff1[MAILBOX_LEN1*10];
    uint8_t rx_buff2[MAILBOX_LEN2*10];
    while(1)
    {
            if(xSemaphoreTake(event_signal, portMAX_DELAY)) //Take semaphore (Mailbox is full if this returns)
            {
                int i = 0;
                uint16_t m_status = get_status_reg(MAILBOX_STATUS); //Read the status resistor
                if(0x01 & (m_status>>7))                            //If mailbox # 7 Full
                {
                    puts("Mailbox 7 full");
                    get_CAN_frame(7, rx_buff2, MAILBOX_LEN2);       //Read the mailbox from the I2C slave
                    i = 0;
                    //Print the received data
                    while(i < (10*MAILBOX_LEN2))
                    {
                        if((i)%10 == 0)
                        {
                            printf("CANID: %X%X DATA: ", rx_buff2[i++], rx_buff2[i]);
//                            continue;
                        }
                        else
                            printf("%x  ", rx_buff2[i]);
                        if((i+1)%10 == 0)
                        {
                            printf("\n");
                        }
                        i++;
                    }
                }
                if(0x01 & (m_status>>1))
                {
                    puts("Mailbox 1 full");
                    get_CAN_frame(1, rx_buff1, MAILBOX_LEN1);       //If mailbox # 1 Full, read the mailbox from the I2C slave
                    i = 0;
                    //print the reveived data
                    while(i < (10*MAILBOX_LEN1))
                    {
                        if((i)%10 == 0)
                        {
                            printf("CANID: %X%X DATA: ", rx_buff1[i++], rx_buff1[i]);
//                            continue;
                        }
                        else
                            printf("%x  ", rx_buff1[i]);
                        if((i+1)%10 == 0)
                        {
                            printf("\n");
                        }
                        i++;
                    }
                }
                puts("\n\n");
            }
    }
}

class send_mailbox_config : public scheduler_task
{
    public:
        send_mailbox_config(uint8_t priority) :
            scheduler_task("send_CAN_data", 2048, priority)
        {
            /* Nothing to init */
        }

        bool run(void *p)
        {
            static uint8_t mailbox_no = 0;
            static uint8_t mailbox_len = 1;
            static uint16_t id = 0x111;
            uint16_t mask_bits;

            if(SW.getSwitch(1))         //If switch is pressed, send configuration for Mailbox # 1
            {
                puts("Mailbox # 1 configuration message sent");
                mailbox_no = 1;
                mailbox_len = MAILBOX_LEN1;
                id = A_CANF4;
                mask_bits = 0x03;
                config_mailbox(mailbox_no, mailbox_len, id, mask_bits);     //send the configuration specifying
    }                                                                       //the CAN ID, Mailbox number and size (Mask bits if needed)
            if(SW.getSwitch(2))
            {
                puts("Mailbox # 7 configuration message sent");
                mailbox_no = 7;
                mailbox_len = MAILBOX_LEN2;
                id = B_CANF2;
                mask_bits = 0x0E;
                config_mailbox(mailbox_no, mailbox_len, id, mask_bits);     //send the configuration specifying
        }                                                                   //the CAN ID, Mailbox number and size (Mask bits if needed)
            vTaskDelay(500);
            return true;
        }
};

class send_CAN_data : public scheduler_task
{
    public:
        send_CAN_data(uint8_t priority) :
            scheduler_task("send_CAN_data", 2048, priority)
        {
            /* Nothing to init */
        }

        bool run(void *p)
        {
            uint8_t buff[DATA_LEN];     //data field
//            puts("****sent CAN frame");
            memset(buff,0,DATA_LEN);
            buff[0] = temp1;                                //Store sensor data in CM
            send_CAN_frame(CAN_DST1, MSG_ID1, buff);        //send the CAN frame for controller A
            vTaskDelay(500);
//            puts("****sent CAN frame");
            buff[0] = temp2;                                //Store sensor data in inches
            send_CAN_frame(CAN_DST2, MSG_ID2, buff);        //send the CAN frame for controller B
            vTaskDelay(500);
            return true;
        }
};

void init_time_sd()
{
    rtc_t time;
    time.month = 04; time.day = 26; time.year = 2015;
    time.hour = 04, time.min = 39; time.sec = 00;
    time.dow = 0;
    rtc_settime(&time);
}

int main(void)
{
        I2C_init(I2C_SPEED);                 //init i2c master for 100k
        init_time_sd();
        vSemaphoreCreateBinary( event_signal ); // Create the semaphore
        xSemaphoreTake(event_signal, 0);        // Take semaphore after creating it.
        scheduler_add_task(new terminalTask(PRIORITY_HIGH));
        TaskHandle_t test1 = NULL;
        xTaskCreate((void(*)(void *))read_from_slave, "i2c_receive_task", 1024, NULL, PRIORITY_HIGH, &test1);
        eint3_enable_port2(0,eint_rising_edge,data_avail_isr);   //register for rising edge interrupt
        scheduler_add_task(new send_CAN_data(PRIORITY_MEDIUM));
        scheduler_add_task(new send_mailbox_config(PRIORITY_MEDIUM));
        scheduler_add_task(new sendTrigTask()); //Task for getting the sensor data
        scheduler_start(); ///< This shouldn't return
        return -1;
}

