#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"

#define BUFFER_SIZE 8
#define SEL_FLAG 0x1
#define PWM_FLAG 0x2

osThreadId_t controller_id;
osMessageQueueId_t msg_queue_id[4];

enum message {
  SELECTED,
  UNSELECTED,
  PWM
};

void buttons(){
  static uint32_t tick0 = 0;
  static uint32_t tick1 = 0;
  uint8_t input = GPIOIntStatus(GPIO_PORTJ_BASE, 1);

  if (input & GPIO_PIN_0) {   
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);
    if (tick0 < osKernelGetTickCount()) {
      tick0 = osKernelGetTickCount() + 200;
      osThreadFlagsSet(controller_id, SEL_FLAG);
    }
  }
  
  if (input & GPIO_PIN_1) {   
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_1);
    if (tick1 < osKernelGetTickCount()) {
      tick1 = osKernelGetTickCount() + 200;
      osThreadFlagsSet(controller_id, PWM_FLAG);
    }
  }
}

void controller(void* arg) {
  uint8_t selected_led = 0x0;
  enum message msg;
  
  while(1) {
    osThreadFlagsWait(SEL_FLAG | PWM_FLAG, osFlagsWaitAny | osFlagsNoClear, osWaitForever);
    uint8_t flags = osThreadFlagsGet();
    
    if(flags & SEL_FLAG) {
      msg = UNSELECTED;
      osMessageQueuePut(msg_queue_id[selected_led], &msg, 0, osWaitForever);
      
      selected_led = (selected_led + 1) % 4;
      
      msg = SELECTED;
      osMessageQueuePut(msg_queue_id[selected_led], &msg, 0, osWaitForever);
      
      osThreadFlagsClear(SEL_FLAG);
    }
    
    if(flags & PWM_FLAG) {
      msg = PWM;
      osMessageQueuePut(msg_queue_id[selected_led], &msg, 0, osWaitForever);
      
      osThreadFlagsClear(PWM_FLAG);
    }
  }
}

struct led_thread_data_t {
  osMutexId_t mutex;
  uint8_t led;
  osMessageQueueId_t queue;
};

void LEDOnMutex(uint8_t led, osMutexId_t mutex){
  osMutexAcquire(mutex, osWaitForever);
  LEDOn(led);
  osMutexRelease(mutex);
}

void LEDOffMutex(uint8_t led, osMutexId_t mutex){
  osMutexAcquire(mutex, osWaitForever);
  LEDOff(led);
  osMutexRelease(mutex);
}

void led(void *arg2){
  led_thread_data_t* arg = (led_thread_data_t*) arg2;
  uint8_t pwm = 10;
  uint8_t selected = 0;
  enum message msg;
  
  while(1) {
    if (selected) {
      for(int ii=0; ii< 50; ii++) {
        LEDOnMutex(arg->led, arg->mutex);
        osDelay(pwm);
        LEDOffMutex(arg->led, arg->mutex);
        osDelay(10- pwm);
      }
      LEDOffMutex(arg->led, arg->mutex);
      osDelay(500);
    } else {
      LEDOnMutex(arg->led, arg->mutex);
      osDelay(pwm);
      LEDOffMutex(arg->led, arg->mutex);
      osDelay(10- pwm);
    }
    
    osStatus_t ret = osMessageQueueGet(arg->queue, &msg, NULL, 0);
    if (ret == osOK) {
      if (msg == PWM) {
        pwm = (pwm + 1) % 11;
      } else if (msg == SELECTED) {
        selected = 1;
      } else if (msg == UNSELECTED) {
        selected = 0;
      }
    }
  }
} 

void main(void){
  SystemInit();

  osKernelInitialize();

  osMutexId_t mutex_GPION_id = osMutexNew(NULL);
  osMutexId_t mutex_GPIOF_id = osMutexNew(NULL);
  
  static led_thread_data_t led_thread_data[4]  = {
    {.mutex = mutex_GPION_id, .led = LED1},
    {.mutex = mutex_GPION_id, .led = LED2},
    {.mutex = mutex_GPIOF_id, .led = LED3},
    {.mutex = mutex_GPIOF_id, .led = LED4},
  };
  
  controller_id = osThreadNew(controller, NULL, NULL);
  
  for(int i = 0; i<=3; i++ ){
    msg_queue_id[i] = osMessageQueueNew(1, sizeof(enum message), NULL);
    led_thread_data[i].queue = msg_queue_id[i];
    osThreadNew(led, &led_thread_data[i], NULL);
  }
  
  enum message msg = SELECTED;
  osMessageQueuePut(msg_queue_id[0], &msg, 0, 0);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); // Habilita GPIO J (push-button SW1 = PJ0, push-button SW2 = PJ1)
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Aguarda final da habilitação
  
  GPIOIntDisable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1 ); // push-buttons SW1 e SW2 como entrada
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  
  GPIOIntRegister(GPIO_PORTJ_BASE, buttons);
  GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_FALLING_EDGE);
  GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  LEDInit(LED4 | LED3 | LED2 | LED1);

  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
} // main
