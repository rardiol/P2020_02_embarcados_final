#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h" // CMSIS-RTOS

// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h"
#include "driverleds.h" // device drivers

#define LIFTS 3
#define POSITION_UPDATE_TIME 1
#define FLOORS 16
#define HEIGHT_PER_FLOOR 5000
#define MAX_HEIGHT (HEIGHT_PER_FLOOR*(FLOORS-1))
#define FLOOR_TOLERANCE 150
#define MESSAGE_LENGTH 20
#define MAX_MESSAGES 10
#define INITIALIZED_FLAG 0x1
#define CHECK_MESSAGE_FLAG 0x1
#define LIFT_WAIT_TIME 5000
//#define MESSAGE_MINIMUM_DELAY 0
//#define MESSAGE_REPEAT_DELAY (MESSAGE_MINIMUM_DELAY*LIFTS*10000)
#define END_OF_LINE '\r'
//#define RECEIVE_DELAY 10

#define CHECK(x) do { \
osStatus_t retval = (x); \
  if (retval != osOK) { \
    fprintf(stderr, "Runtime error: %s returned %d at %s:%d", #x, retval, __FILE__, __LINE__); \
      return /* or throw or whatever */; \
  } \
} while (0)

osMessageQueueId_t transmit_queue, destination_assignment_queue, position_update_queue;
osThreadId_t transmit_thread, destination_assignment_thread, position_update_thread;

osMessageQueueId_t receive_queue;
osThreadId_t receive_thread;

const osThreadAttr_t thread_reception_uart_attr = {
  //  .priority = osPriorityHigh
  .name = "reception"
};

const osThreadAttr_t thread_transmission_uart_attr = {
  //  .priority = osPriorityHigh
  .name = "transmission_uart"
};

const osThreadAttr_t thread_destination_assignment_attr = {
  .name = "destination_assignment"
};

const osThreadAttr_t thread_position_update_attr = {
  .name = "position_update"
};


const osThreadAttr_t thread_lift_attr[LIFTS] = {
  { .name = "Lift 0" },
  { .name = "Lift 1" },
  { .name = "Lift 2" }
};

enum lift_state {
  OPENING = 0,
  OPEN,
  WAITING,
  CLOSING,
  MOVING
};

enum lift_dir_state {
  UP = 1,
  STOPDIR = 0,
  DOWN = -1
};

struct lift {
  osMessageQueueId_t  inmsg;
  enum lift_dir_state direction;
  enum lift_state state;
  uint16_t floor;
  int32_t position;
  osThreadId_t thread_id;
  char code;
  uint16_t targets; // bit array
};

struct lift lifts[LIFTS];

__NO_RETURN void position_update(void *arg){
  
  char msg[MESSAGE_LENGTH];
  msg[1] = 'x';
  msg[2] = END_OF_LINE;
  msg[3] = '\0';
  int8_t curLift = 0;
  
  while(1) {
    curLift = (curLift + 1) % LIFTS;        
    if((lifts[curLift].state != MOVING) && (lifts[curLift].state != OPENING)){
      osDelay(POSITION_UPDATE_TIME);
      continue;
    }
    switch(curLift) {
    case 0:
      msg[0] = 'e';
      break;
    case 1:
      msg[0] = 'c';
      break;
    case 2:
      msg[0] = 'd';
      break;  
    }
    osMessageQueuePut(transmit_queue, msg, 0, osWaitForever);
    osMessageQueueGet(position_update_queue, &lifts[curLift].position, NULL, osWaitForever);
    osDelay(POSITION_UPDATE_TIME);
  }
}

__NO_RETURN void transmit(void *arg){
  
  char msg[MESSAGE_LENGTH];
  
  
  osThreadFlagsWait(INITIALIZED_FLAG, osFlagsWaitAll, osWaitForever);  
  while(1) {
    osMessageQueueGet(transmit_queue, msg, NULL, osWaitForever);
    /*for(int ii = 0; ii<MESSAGE_LENGTH;ii++){
    if(msg[ii] == '\0'){
    break;
  } else {
    UARTCharPutNonBlocking(UART0_BASE, msg[ii]);
  }
  }*/
    UARTwrite(msg, strlen(msg));
    UARTFlushTx(false);
    //    osDelay(1);
  }
}

__NO_RETURN void receive(void *arg){
  
  char ret[MESSAGE_LENGTH];
  
  while(UARTPeek('\r') == -1){
    osThreadFlagsWait(CHECK_MESSAGE_FLAG, osFlagsWaitAll, osWaitForever);      
  }
  
  // initialized message
  UARTgets(ret, MESSAGE_LENGTH);
  osThreadFlagsSet(transmit_thread, INITIALIZED_FLAG);
  
  while(1) {
    while(UARTPeek('\r') == -1){
      osThreadFlagsWait(CHECK_MESSAGE_FLAG, osFlagsWaitAll, osWaitForever);      
    }
    
    UARTgets(ret, MESSAGE_LENGTH);
    
    // Position
    char* strtolchar;
    int32_t position = strtol(ret, &strtolchar, 10);
    if(ret != strtolchar) {
      osMessageQueuePut(position_update_queue, &position, 0, osWaitForever);
      continue;
    }
    
    // External button press
    if(ret[1] == 'E'){
      osMessageQueuePut(destination_assignment_queue, ret, 0, osWaitForever);      
      continue;
    }
    
    // Other messages
    int32_t lift = -1;
    switch(ret[0]) {
    case 'e':
      lift = 0;
      break;
    case 'c':
      lift = 1;
      break;
    case 'd':
      lift = 2;
      break;
    }
    if(lift != -1) {
      osMessageQueuePut(lifts[lift].inmsg, ret, 0, osWaitForever);
      continue;
    }
  }
}

int8_t choose_lift(enum lift_dir_state targetDirection, int targetFloor) {
  int best_lift = 0;
  int best_distance = 10000;
  
  int straight_distance_to_target = lifts[targetFloor].floor - targetFloor;
  enum lift_dir_state direction_to_target;
  if(straight_distance_to_target > 0) {
    direction_to_target = UP;
  } else if (straight_distance_to_target < 0) {
    direction_to_target = DOWN;
  } else {
    direction_to_target = STOPDIR;
  }
  int distance_target_to_border;
  if(targetDirection == DOWN) {
    distance_target_to_border = 2*(FLOORS - 1 - targetFloor);
  } else {
    distance_target_to_border = 2*targetFloor;
  }  
  
  for(int ii = 0; ii<LIFTS; ii++){
    int distance = 0;
    
    // Stopped on the same floor
    if((lifts[ii].direction == STOPDIR) && (straight_distance_to_target == 0)) {
      distance = 0;
    }
    // stopped, direction to go to target is same as target_direction or
    else if(((lifts[ii].direction == STOPDIR) && (direction_to_target == targetDirection)) ||
            // in the way, same direction, . basic distance
            ((direction_to_target == targetDirection) && (lifts[ii].direction == targetDirection))
              ) {
                distance = abs(straight_distance_to_target);
              }
    // stopped, direction to go to target is not same as target_direction or
    else if(((lifts[ii].direction == STOPDIR) && (direction_to_target != targetDirection)) ||
            // in the way, wrong direction . basic_distnace+2*distance_target_to_border
            
            ((direction_to_target == targetDirection) && (lifts[ii].direction == targetDirection))
              ) {
                distance = abs(straight_distance_to_target) + distance_target_to_border;
              }
    
    // behind, wrong direction 2*distance_target_to_border-basic_distance
    else if(direction_to_target != lifts[ii].direction) {
      distance = 2*distance_target_to_border-straight_distance_to_target;
    } 
    // behind, same direction . 2*fullcycle-basic_distance
    else 
    {
      distance = 2*FLOORS-straight_distance_to_target;
    }
    
    if(distance < best_distance) {
      best_distance = distance;
      best_lift = ii;
    }
  }
  return best_lift;
}

__NO_RETURN void destination_assignment(void *arg){
  char msg[MESSAGE_LENGTH];
  while(1) {
    osMessageQueueGet(destination_assignment_queue, msg, NULL, osWaitForever);
    enum lift_dir_state direction = (msg[4] == 's') ? UP : DOWN;
    char* _i;
    osMessageQueuePut(lifts[choose_lift(direction, strtol(&msg[2], &_i, 10))].inmsg, msg, 0, osWaitForever);
  }
}

osStatus_t send_close(char outmsg[]){
  outmsg[1] = 'f';
  outmsg[2] = END_OF_LINE;
  outmsg[3] = '\0';
  return osMessageQueuePut(transmit_queue, outmsg, 0, osWaitForever);
}

osStatus_t send_open(char outmsg[]){
  outmsg[1] = 'a';
  outmsg[2] = END_OF_LINE;
  outmsg[3] = '\0';
  return osMessageQueuePut(transmit_queue, outmsg, 0, osWaitForever);
}

osStatus_t send_stop(char outmsg[]){
  outmsg[1] = 'p';
  outmsg[2] = END_OF_LINE;
  outmsg[3] = '\0';
  return osMessageQueuePut(transmit_queue, outmsg, 1, osWaitForever);
}

void remove_current_floor_targets(uint16_t floor, uint16_t* targets, char* outmsg) {
  uint16_t temp_targets = *targets & ~(1 << floor);
  if(*targets != temp_targets){
    *targets = temp_targets;
    outmsg[1] = 'D';
    outmsg[2] = floor + 'a';
    outmsg[3] = END_OF_LINE;
    outmsg[4] = '\0';
    osMessageQueuePut(transmit_queue, outmsg, 0, osWaitForever);
  }
}

osStatus_t send_on(char outmsg[], char target){
  outmsg[1] = 'L';
  outmsg[2] = target;
  outmsg[3] = END_OF_LINE;
  outmsg[4] = '\0';
  return osMessageQueuePut(transmit_queue, outmsg, 0, osWaitForever);
}

int check_floor(int32_t position, uint16_t* current_floor) {
  int32_t distance_to_nearest_floor = position % HEIGHT_PER_FLOOR;
  if(distance_to_nearest_floor < FLOOR_TOLERANCE){
    *current_floor = position / HEIGHT_PER_FLOOR;
    return 1;
  } else if(distance_to_nearest_floor > (HEIGHT_PER_FLOOR - FLOOR_TOLERANCE)){
    *current_floor = position / HEIGHT_PER_FLOOR + 1;
    return 1;
  }
  return 0;
}

void start_moving(struct lift* arg, char* outmsg){
  arg->state = MOVING;
  
  int above_floor;
  int below_floor;
  if(check_floor(arg->position, &arg->floor)){
    above_floor = arg->floor+1;
    below_floor = arg->floor-1;
  } else {
    above_floor = (arg->position / HEIGHT_PER_FLOOR) + 1;
    below_floor = (arg->position / HEIGHT_PER_FLOOR);
  }
  // get distances
  int32_t distance_up = INT32_MAX;
  for(int floor = above_floor; floor < FLOORS; floor++){
    if(((1 << floor) & arg->targets)) {
      distance_up = floor - arg->floor;
      break;
    }
  }
  int distance_down = INT32_MAX;
  for(int floor = below_floor; floor >= 0; floor--){
    if(((1 << floor) & arg->targets)) {
      distance_down = arg->floor - floor;
      break;
    }
  }
  
  // Update direction
  switch(arg->direction){
  case UP:
    if(distance_up < INT32_MAX){
      // arg->direction = UP;
    } else {
      arg->direction = DOWN;
    }
    break;
  case DOWN:
    if(distance_down < INT32_MAX){
      // arg->direction = DOWN;
    } else {
      arg->direction = UP;
    }
    break;
  case STOPDIR:
    if(distance_up < distance_down) {
      arg->direction = UP;
    } else {
      arg->direction = DOWN;
    }
    break;
  }
  
  // send up down message
  switch(arg->direction){
  case UP:
    outmsg[1] = 's';
    break;
  case DOWN:
    outmsg[1] = 'd';
    break;
  case STOPDIR:
    // error()
    break;
  }
  
  outmsg[2] = END_OF_LINE;
  outmsg[3] = '\0';
  osMessageQueuePut(transmit_queue, outmsg, 0, osWaitForever);          
  
  
}

__NO_RETURN void lift(void *arg2){
  struct lift* arg = (struct lift*) arg2;
  char msg[MESSAGE_LENGTH];
  char outmsg[MESSAGE_LENGTH];
  uint32_t delay = osWaitForever;
  
  outmsg[0] = arg->code;
  outmsg[1] = 'r';
  outmsg[2] = END_OF_LINE;
  outmsg[3] = '\0';
  osMessageQueuePut(transmit_queue, outmsg, 0, osWaitForever);
  
  arg->state = OPENING;
  
  // OPEN variables
  uint32_t start = 0;
  uint32_t now;
  
  while(1) {
    
    switch(arg->state) {
    case OPENING:
      if(!check_floor(arg->position, &arg->floor)) {
        
        start_moving(arg, outmsg);
        continue;
      }
      
      delay = 5;
      /*
      // Keep sending close till the simulator gets it
      delay = MESSAGE_REPEAT_DELAY;
      send_open(outmsg);
      */
      break;
    case OPEN:
      now = osKernelGetTickCount();
      if(start == 0) {
        start = osKernelGetTickCount();
      }
      if (now > start + LIFT_WAIT_TIME) {
        arg->state = WAITING;
        start = 0;
        continue;
      }
      delay = start + LIFT_WAIT_TIME - now;
      remove_current_floor_targets(arg->floor, &arg->targets, outmsg);
      
      break;
    case WAITING:
      delay = osWaitForever;
      remove_current_floor_targets(arg->floor, &arg->targets, outmsg);
      if(arg->targets){
        send_close(outmsg);
        arg->state = CLOSING;
        continue;
      } else {
        arg->direction = STOPDIR;
      }
      break;
    case CLOSING:
      delay = osWaitForever;
      
      /*
      // Keep sending close till the simulator gets it
      delay = MESSAGE_REPEAT_DELAY;
      send_close(outmsg);
      */
      break;
    case MOVING:
      delay = POSITION_UPDATE_TIME*LIFTS; // time for position_update to get to this lift
      
      // update floor
      int valid_floor = check_floor(arg->position, &arg->floor);
      
      // check if it should stop
      if(valid_floor && ((1 << arg->floor) & arg->targets)) {
        send_stop(outmsg);
        send_open(outmsg);
        arg->state = OPENING;
        continue;
      }
      break;
    }
    
    // handle messages
    {
      osStatus_t err = osMessageQueueGet(arg->inmsg, msg, NULL, delay);
      if(err != osOK) {
        continue;
      }
      // Floor update
      char* strtolchar;
      int new_floor = strtol(msg, &strtolchar, 10);
      if(msg != strtolchar) {
        arg->floor = new_floor;
        if((1 << arg->floor) & arg->targets) {
          send_stop(outmsg);
          send_open(outmsg);
          arg->state = OPENING;
          continue;
        }
      }
      // external button press
      else if(msg[1] == 'E') {
        char* _i;
        int16_t targetFloor = strtol(&msg[2], &_i, 10);
        arg->targets |= (1 << targetFloor);
        send_on(outmsg, targetFloor + 'a');
      }
      // internal button press
      else if(msg[1] == 'I') {
        int16_t targetFloor = msg[2] - 'a';
        arg->targets |= 1 << targetFloor;
        send_on(outmsg, msg[2]);
        osMessageQueuePut(transmit_queue, outmsg, 0, osWaitForever);
      }
      // doors open 
      else if(msg[1] == 'A') {
        if(arg->state == OPENING){
          arg->state = OPEN;
        } else {
          //error()
        }
      } 
      // doors closed
      else if(msg[1] == 'F') {
        if(arg->state == CLOSING){
          
          start_moving(arg, outmsg);
        } else {
          //error()
        }
      }    
    }
  }
}

//----------
// UART definitions
extern void UARTStdioIntHandler(void);

void UARTInit(void){
  // Enable UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
  
  UARTEchoSet(false);
  
  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 115200, SystemCoreClock);
  
  // Enable the GPIO Peripheral used by the UART.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
  
  // Configure GPIO Pins for UART mode.
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
} // UARTInit

void UART0_Handler(void){
  LEDOn(LED3);
  UARTStdioIntHandler();
  osThreadFlagsSet(receive_thread, CHECK_MESSAGE_FLAG);
  LEDOff(LED3);
} // UART0_Handler
//----------

void HardFault_Handler(void){
  LEDOn(LED2);
}
// osRtxIdleThread
__NO_RETURN void osRtxIdleThread(void *argument){
  (void)argument;
  
  while(1){
    //UARTprintf("Idle thread\n");
    asm("wfi");
  } // while
} // osRtxIdleThread

void main(void){
  LEDInit(LED4 | LED3 | LED2 | LED1);
  LEDOff(LED4 | LED3 | LED2 | LED1);
  LEDOn(LED1);
  UARTInit();
  
  if(osKernelGetState() == osKernelInactive)
    osKernelInitialize();
  
  transmit_queue = osMessageQueueNew(MAX_MESSAGES, MESSAGE_LENGTH, NULL);
  transmit_thread = osThreadNew(transmit, NULL, &thread_transmission_uart_attr);
  receive_queue = osMessageQueueNew(MAX_MESSAGES, MESSAGE_LENGTH, NULL);
  receive_thread = osThreadNew(receive, NULL, &thread_reception_uart_attr);
  
  destination_assignment_queue = osMessageQueueNew(MAX_MESSAGES, MESSAGE_LENGTH, NULL); 
  destination_assignment_thread = osThreadNew(destination_assignment, NULL, &thread_destination_assignment_attr);
  position_update_queue = osMessageQueueNew(1, sizeof(int32_t), NULL);
  position_update_thread = osThreadNew(position_update, NULL, &thread_position_update_attr);
  
  lifts[0].code = 'e';
  lifts[1].code = 'c';
  lifts[2].code = 'd';
  for(int ii = 0; ii < LIFTS; ii++){
    lifts[ii].inmsg = osMessageQueueNew(MAX_MESSAGES, MESSAGE_LENGTH, NULL);
    lifts[ii].thread_id = osThreadNew(lift, &lifts[ii], &thread_lift_attr[ii]);
  }
  
  if(osKernelGetState() == osKernelReady) {
    osStatus_t stat = osKernelStart();
  }
  
  while(1);
} // main
