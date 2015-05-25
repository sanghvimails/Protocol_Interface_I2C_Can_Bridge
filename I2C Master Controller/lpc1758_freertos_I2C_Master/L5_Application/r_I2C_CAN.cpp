/*
 * r_I2C_CAN.cpp
 *
 *  Created on: Apr 13, 2015
 *      Author: Rutwik
 */
#include "r_I2C_CAN.hpp"

#define DATA_LEN 8
#define RW_ADDRESS 0x50         //I2C device address
#define WRITE_REG_ADD 0xFF      //Write command/address
#define STATUS_REG_ADD 0xFE     //Read status command

#define MAILBOX_STATUS 0x01

//My ID updated in the main.cpp by the programmer
extern uint8_t MY_ID;

I2C2& i2c = I2C2::getInstance(); // Get I2C driver instance

void I2C_init(unsigned int freq)
{
    i2c.init(freq);
}

void config_mailbox(uint8_t mailbox_no, uint8_t mailbox_size, uint16_t id, uint16_t mask_bits)
{
    uint8_t frame[4];
    memset(frame, 0, 4);
    frame[0] = 0;
    frame[0] |= mailbox_size<<3;        //Mailbox size at [6:3]
    frame[0] |= id>>8;                  //11 bit CAN ID MSB [2:0]
    frame[1] = id;                      //11 bit CAN ID Other bits [7:0]
    if(mask_bits == 0)                  //If mask is zero, send the frame directly
    {
        frame[0] &= ~(1<<7);            //Reset the Mast bit
        send_frame(RW_ADDRESS, mailbox_no, frame, 2);       //send frame (internally calls write_registers()
    }
    else
    {
        frame[0] |= (1<<7);             //set the mask bit at MSB
        frame[2] |= mask_bits >> 8;     //mask id MSBs [2:0]
        frame[3] = mask_bits;           //mask id other bits [7:0]
        send_frame(RW_ADDRESS, mailbox_no, frame, 4);       //send frame (internally calls write_registers()
    }

}

void send_CAN_frame(uint8_t dst, uint8_t msg_id, uint8_t *buff)
{
    uint8_t full_frame[10];
    unsigned int i = 0;
    struct can_id msg;
    union can_id_packed frame;
    int len = 8;
    msg.dlc = len;      //data length (not used) but included in the frame. I2C slave will neglect these bits
    msg.dst = dst;      //set DST field
    msg.id = msg_id;    //set MSG_ID field
    msg.src = MY_ID;    //Soure ID
    frame.full_id_struct = msg;
    full_frame[0] = frame.full_id >> 8; //Make the frame for sending to the I2C slave
    full_frame[1] = frame.full_id;
    while(i < len)
    {
        full_frame[i+2] = buff[i];
        i++;
    }
    send_frame(RW_ADDRESS, WRITE_REG_ADD, full_frame, len + 2); //send the frame
}

uint16_t get_status_reg(uint8_t state)
{
    uint8_t status_reg[2];      //To store both bytes in a single frame
    i2c.readRegisters(RW_ADDRESS, STATUS_REG_ADD, status_reg, 2);   //Read the status register
    printf("status Register is %x %x\n", status_reg[1], status_reg[0]); //Print the status register
    if(state == MAILBOX_STATUS)
    {
        return ((status_reg[1] << 8) | status_reg[0]);  //Return a 16 bit status register
    }

}

void get_CAN_frame(uint8_t mailbox_number, uint8_t *buff, int len)
{
    len = 10 * len; //10 bytes per frame
    uint8_t rx_buff[16*10]; //Length of mailbox is 16
    int i = 0;
    i2c.readRegisters(RW_ADDRESS, mailbox_number, rx_buff, len);    //Read the mailbox
    while(i < len)  //Update the buffer passed by calling function
    {
        buff[i] = rx_buff[i];
        i++;
    }
}

int send_frame(uint8_t slaveaddr, uint8_t register_address, uint8_t *frame, uint8_t len)
{
    i2c.writeRegisters(slaveaddr, register_address, frame, len);
    return 1;
}



