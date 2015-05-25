/*
 * project.cpp
 *
 *  Created on: Apr 5, 2015
 *      Author: Mitesh_Sanghvi
 */

#include "i2c_base.hpp"
#include "i2c2.hpp"
#include "project.hpp"

uint8_t i2c_mem[40] = { 0 };
uint8_t can_mem[10*32*32] = { 0x1, 0x23, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf };
mailbox_t mailbox[16];
uint32_t status_register = 0;
SemaphoreHandle_t i2c_receive = NULL;
SemaphoreHandle_t allow_trigger = NULL;

