/*  Ping))) 3 pin sensor

 *  by Rutwik (Preet's suggestion)       Date: 11/5/2014
 *
 *
 */

#include <stdio.h>
#include "utilities.h"
#include "uart0_min.h"
#include "eint.h"
#include "ultrasonic.hpp"
#include "tasks.hpp"
#include "io.hpp"
#include "adc0.h"
#include "semphr.h"
#include "queue.h"
#include "uart0.hpp"
#include "math.h"

bool edge_center = 1, edge_right = 0, edge_left = 0;      //1: flag for pulse status

int reading =0,distance,light_percent;
float voltage,bat_volt,light_reading,bat_reading;

int systime_center = 0, systime_right = 0, systime_left = 0;    //variable for storing system_get_uptime
int x = 0, y = 0, z = 0;
uint8_t ultraDist[5];
QueueHandle_t queueL = 0;
QueueHandle_t queueR = 0;
QueueHandle_t queueC = 0;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

int temp1, temp2;
//*********************** Function to filter the glitches

int calc_mean_val_left(int num)
{
#ifdef MEAN_QUEUE_LEN
#error "MEAN_QUEUE_LEN already defined!"
#else
#define MEAN_QUEUE_LEN 10
#endif

   static int queue[MEAN_QUEUE_LEN];
   static int sorted_queue[MEAN_QUEUE_LEN];
   static int head = 0;
   static int tail = (MEAN_QUEUE_LEN - 1);

   int i;
   int temp;

   int removed_data, removed_flag;
   int insert_data, insert_flag;

   removed_data = queue[head];
   head = ((head + 1) % MEAN_QUEUE_LEN);
   tail = ((tail + 1) % MEAN_QUEUE_LEN);
   queue[tail] = num;

   removed_flag = 0;
   insert_data = num;
   insert_flag = 0;
   for(i = 0; i < (MEAN_QUEUE_LEN - 1); i++) {
      if(removed_data == sorted_queue[i] || removed_flag) {
         sorted_queue[i] = sorted_queue[i + 1];
         removed_flag = 1;
      }

      if(insert_data < sorted_queue[i] || insert_flag) {
         temp = insert_data;
         insert_data = sorted_queue[i];
         sorted_queue[i] = temp;
         insert_flag = 1;
      }
   }
   sorted_queue[i] = insert_data;

   return sorted_queue[MEAN_QUEUE_LEN / 2];

#undef MEAN_QUEUE_LEN
}

//**********************Trigger and Echo Functions (including queue receive)
void trigger_left (void)
{
    LPC_GPIO2->FIODIR |= echo_pin_left;                //configure as output for next trigger pulse
    LPC_GPIO2->FIOCLR = echo_pin_left;
    LPC_GPIO2->FIOSET = echo_pin_left;
    delay_us(5);  // ideally only 2-5 us pulse required
    LPC_GPIO2->FIOCLR = echo_pin_left;
    systime_left = (int)sys_get_uptime_us();
    edge_left = 1;
    LPC_GPIO2->FIODIR &= ~echo_pin_left;     //configure as input
}

int wait_for_Lsensor_data()
{
    int value;
    LPC_GPIO2->FIODIR &= ~echo_pin_left;     //configure as input
    if (!xQueueReceive(queueL, &value, 30)) {
        value = 0;
    }

    edge_left = 1;
    return value;
}

//******************** Init functions for sensors and interrupt (callback functions)
void init_io_left()
{
    LPC_PINCON->PINSEL4 &= ~(3<<2);    //P2.1 as GPIO (for Echo pin)
    LPC_GPIO2->FIODIR |= echo_pin_left;      //configure as output
}

void initEchoInt_left(eint_intr_t eintType)
{
    eint3_enable_port2(1,eintType,ext_callback_left);  //P2.1 - Echo pin
}
void ext_callback_left()
{
    x = (int)sys_get_uptime_us() - systime_left; //get the duration of pulse
    xQueueSendFromISR(queueL, (void*)&x, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

sendTrigTask::sendTrigTask() : scheduler_task("trig", 4 * 512, PRIORITY_CRITICAL)
{

}

//********************** Trigger Task init function
bool sendTrigTask::init(void)
{
    setRunDuration(50);
    init_io_left();
    initEchoInt_left(eint_falling_edge);      //Look rising edge of echo pulse
    queueL = xQueueCreate(1, sizeof(int));
    LPC_PINCON->PINSEL3 |= (3 << 28); // ADC-4 is on P1.30, select this as ADC0.4

    return true;
}

bool sendTrigTask::run(void *p)
{
    int sensorLData = 0;
    //printf("\nTriggering left\n");
    trigger_left();
    sensorLData = calc_mean_val_left(wait_for_Lsensor_data());
    temp1 = ((sensorLData-750)*13*30/(22500*2.54));           //in inch
    temp2 = ((sensorLData-750)*13*30/22500);                  //in cm
    return true;

}

