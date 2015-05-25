/*
 * r_I2C_CAN.hpp
 *
 *  Created on: Apr 13, 2015
 *      Author: Rutwik
 */
#include "stdint.h"
#ifndef L5_APPLICATION_R_I2C_CAN_HPP_
#define L5_APPLICATION_R_I2C_CAN_HPP_

#include "i2c2.hpp"
#include "stdint.h"
#include <utilities.h>
#include "stdio.h"
#include <can.h>
#include <string.h>

struct can_id       //Structure for 11 bit CAN ID including DLC (which is not used but included for future use)
{
     uint8_t id  : 5;
     uint8_t src : 3;
     uint8_t dst : 3;
     uint8_t dlc : 4;
};

union can_id_packed     //Packed CAN frame by using the above function (including DLC)
{
        struct can_id full_id_struct;
        uint16_t full_id : 15;
};

void send_CAN_frame(uint8_t, uint8_t, uint8_t *);
void config_mailbox(uint8_t, uint8_t, uint16_t, uint16_t);
void get_CAN_frame(uint8_t, uint8_t *, int);
int send_frame(uint8_t, uint8_t, uint8_t *, uint8_t);
uint16_t get_status_reg(uint8_t);
void I2C_init(unsigned int);

#endif /* L5_APPLICATION_R_I2C_CAN_HPP_ */
