/*
 * ping_sensor.hpp
 *
 *  Created on: Oct 25, 2014
 *      Author: Manuj
 */

#ifndef ULTRASONIC_HPP_
#define ULTRASONIC_HPP_


#include <stdio.h>
#include "utilities.h"
#include "uart0_min.h"
#include "eint.h"
#include "scheduler_task.hpp"
#include "shared_handles.h"
#include "uart3.hpp"
#include "FreeRTOS.h"
#include "semphr.h"

const uint32_t echo_pin_left = (1<<1);    //P2.1

extern uint8_t ultraDist[5];    // [0]:left, [1]:center, [2]:right

void ext_callback_left();

void trigger_left();
void echo_left();

void initEchoInt_left(eint_intr_t eintType);

void init_io_left();

class sendTrigTask : public scheduler_task
{
    public :
        sendTrigTask();
        bool init(void);
        bool run(void *p);
};

class RxLeft : public scheduler_task
{
    public :
        RxLeft();
        bool run(void *p);
};

class RxRight : public scheduler_task
{
    public :
        RxRight();
        bool run(void *p);
};

class readAdcData : public scheduler_task
{
    public :
        readAdcData();
        bool init(void);
        bool run(void *p);
};

class otherSensorsTask : public scheduler_task
{
    public :
        otherSensorsTask();
        bool init(void);
        bool run(void *p);
};


#endif /* PING_SENSOR_HPP_ */
