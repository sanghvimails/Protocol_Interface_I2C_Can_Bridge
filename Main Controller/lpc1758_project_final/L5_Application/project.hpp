/*
 * project.hpp
 *
 *  Created on: Apr 5, 2015
 *      Author: Mitesh_Sanghvi
 */

#ifndef L5_APPLICATION_PROJECT_HPP_
#define L5_APPLICATION_PROJECT_HPP_

#include "can.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "io.hpp"
#include "utilities.h"
#include "printf_lib.h"
#include "i2c_base.hpp"
#include "i2c2.hpp"
#include "scheduler_task.hpp"

#define slaveAddress 0x50
#define BAUDRATE 100
#define RX_QUEUE 10
#define TX_QUEUE 10
#define DEFAULT_MAILBOX_SIZE 15

typedef struct mailbox_t
{
    uint8_t mask    : 1;
    uint8_t size    : 4;
    uint16_t id     : 11;
    uint16_t mask_id: 11;
    uint8_t num     : 4;
    uint8_t data[10*16];
} mailbox_t;

const uint32_t ld3 = (1 << 20);
extern uint8_t i2c_mem[30+10];
extern int interrupted_mailbox;
extern bool status;
extern uint32_t status_register;
extern mailbox_t mailbox[16];
extern SemaphoreHandle_t i2c_receive;

class i2cTask : public scheduler_task
{
    public:
        i2cTask(uint8_t priority) :
            scheduler_task("I2C", 512*2, priority)
        {
            I2C2 &i2c = I2C2::getInstance();            // Initializing I2C as a Slave Device
            i2c.initSlave(slaveAddress, &i2c_mem[0]);
            LPC_GPIO1->FIODIR |= ld3;
            for(int i = 0; i < 16; i++)                 // Loop for configuring the default mailbox size to max value i.e. 15
            {
                mailbox[i].size = DEFAULT_MAILBOX_SIZE;
                mailbox[i].mask = false;
            }
            LD.clear();                                 // Clearing the LED display which is used to display the number of frames in a mailbox
        }
        bool run(void *p)
        {
            return true;
        }
};

class can_rxTask : public scheduler_task
{
    public:
        can_rxTask(uint8_t priority) :
            scheduler_task("can_rx", 512*2, priority)
        {
            CAN_init(can1, BAUDRATE, RX_QUEUE, TX_QUEUE, NULL, NULL);       // Initializing CAN bus
            CAN_bypass_filter_accept_all_msgs();
            CAN_reset_bus(can1);
            status_register = 0;                                            // Reset Status Register
        }

        bool run(void *p)
        {
            int i = 0, j = 0, k = 0;
            can_msg_t rxmsg;                                                // Creating object for sending message over CAN
            rxmsg.data.qword = 0;                                           // Clearing the data bytes of CAN object
            if(CAN_rx(can1, &rxmsg, portMAX_DELAY))                         // Waiting forever to receive a message over CAN bus
            {
                for(i = 0; i < 16; i++)                                     // Loop for checking received CAN_id == configured CAN_id
                {
                    uint16_t temp_mask_id = mailbox[i].id;
                    for(j = 0; j < 11; j++)                                 // Loop for checking received CAN_id with all combinations of configured CAN_id considering Mask Bits
                    {
                        if((1 << j) & (mailbox[i].mask_id))
                        {
                            if((1<<j) & rxmsg.msg_id)
                                temp_mask_id |= (1 << j);
                            else
                                  temp_mask_id &= ~(1 << j);
                        }
                    }
                    if(((rxmsg.msg_id == temp_mask_id)) && (mailbox[i].num < mailbox[i].size))
                    {
                        printf("CAN Received from %x; \n", (unsigned int)rxmsg.msg_id);
                        for(j = mailbox[i].num*10, k = 0; k < 8; j++, k++)  // Storing data into respective mailbox
                            mailbox[i].data[j+2] = rxmsg.data.bytes[k];
                        mailbox[i].data[j-8] = rxmsg.msg_id >> 8;           // Storing received CAN msg_id to mailbox
                        mailbox[i].data[j-7] = rxmsg.msg_id;
                        mailbox[i].num++;
                        if(mailbox[i].num >= mailbox[i].size)
                        {
                            if(!(status_register & (1 << i)))   // Check if number of frame in a mailbox >= configured size
                            {
                                LE.toggle(1);                   // Giving a trigger signal to I2C Master
                                LPC_GPIO1->FIOSET = ld3;
                                delay_us(50);
                                LPC_GPIO1->FIOCLR = ld3;
                                status_register |= (1 << i);    // Setting the respective bit in Status Register
                            }
                        }
                        char disp = (char)mailbox[1].num*10 + (char)mailbox[7].num;
                        LD.setNumber(disp);
                        break;
                    }
                }
            }
            return true;
        }
};

class can_txTask : public scheduler_task
{
    public:
        can_txTask(uint8_t priority) :
            scheduler_task("can_tx", 512*2, priority)
        {
            CAN_init(can1, BAUDRATE, RX_QUEUE, TX_QUEUE, NULL, NULL);
            CAN_bypass_filter_accept_all_msgs();
            CAN_reset_bus(can1);
        }

        bool run(void *p)
        {
            can_msg_t txmsg;
            txmsg.frame_fields.is_29bit = 0;
            if(xSemaphoreTake(i2c_receive, portMAX_DELAY))      // Wait forever for semaphore that is given by i2ctask when data is received by I2C Master to be sent on CAN bus
            {
                txmsg.frame_fields.data_len = i2c_mem[0] >> 3;
                for(int i = 0; i < txmsg.frame_fields.data_len; i++)
                    txmsg.data.bytes[i] = i2c_mem[i+2];         // Fill the data to be sent on CAN
                txmsg.msg_id = ((i2c_mem[0] << 8) | i2c_mem[1]);
                if(CAN_tx(can1, &txmsg, portMAX_DELAY))         // Wait forever till the message is sent over CAN bus
                    printf("CAN Transmit Done\n");
            }
            return true;
        }
};


#endif /* L5_APPLICATION_PROJECT_HPP_ */
