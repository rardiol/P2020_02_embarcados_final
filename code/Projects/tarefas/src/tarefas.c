#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS

/*
#define OS_TICK_FREQ                1000
#define OS_THREAD_NUM               1 // O que nao faz sentido
#define OS_THREAD_DEF_STACK_NUM     1 // Tambem nao

3
Pisca mais devagar

4
A cada 100 ticks de 1ms=100ms
thread1 teoricamente tem um periodo mais longo devido ao uso do osDelay, mas na pratica não é observavel.
Observado ~30 ativações em ~9.1s.

Pensamentos:
Com o thread2, se o tempo de processamento exceder 100 ticks, a thread nunca vai dormir
Com a thread1, se as threads começarem a ficarem muito tempo esperando para executar porque outra thread esta executando, o periodo de ativação vãi ser afetado. 

*/

struct thread_arg {
  uint32_t  ticks;
  uint8_t led;
};

osThreadId_t thread1_id, thread2_id, thread3_id, thread4_id;

/*
void thread1(void *arg){
  uint8_t state = 0;
  
  while(1){
    state ^= LED1;
    LEDWrite(LED1, state);
    osDelay(100);
  } // while
} // thread1
*/

void thread2(void *argin){
  struct thread_arg* arg = (struct thread_arg*) argin;
  uint8_t state = 0;
  uint32_t tick;
  
  while(1){
    tick = osKernelGetTickCount();
    
    state ^= arg->led;
    LEDWrite(arg->led, state);
    
    osDelayUntil(tick + arg->ticks);
  } // while
} // thread2

void main(void){
  LEDInit(LED2 | LED1);
  
  struct thread_arg args[4] = {
    {.ticks = 200, .led = LED1}, 
    {.ticks = 300, .led = LED2},
    {.ticks = 500, .led = LED3},
    {.ticks = 700, .led = LED4}
  };

  osKernelInitialize();

  thread1_id = osThreadNew(thread2, &args[0], NULL);
  thread2_id = osThreadNew(thread2, &args[1], NULL);
  thread3_id = osThreadNew(thread2, &args[2], NULL);
  thread4_id = osThreadNew(thread2, &args[3], NULL);

  if(osKernelGetState() == osKernelReady)
    osKernelStart();

  while(1);
} // main
