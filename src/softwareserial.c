/*
* This file is part of the hoverboardfirmware project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
* MIT
*/
#include "stm32f1xx_hal.h"
#include "config.h"
#include <memory.h>

#ifdef SOFTWARE_SERIAL
#include "softwareserial.h"

#define SOFTWARE_SERIAL_RX_TIMER_FREQ (SOFTWARE_SERIAL_BAUD*4)
#define SOFTWARE_SERIAL_TX_TIMER_FREQ (SOFTWARE_SERIAL_BAUD)

// assume 10 bit
#define SOFTWARE_SERIAL_MS_PER_BYTE (1000.0/((float)SOFTWARE_SERIAL_BAUD/10.0))


TIM_HandleTypeDef softwareserialtimer;
TIM_HandleTypeDef softwareserialtimerTX;

volatile unsigned int timerval = 0;
volatile unsigned int ssbits = 0;
int softserialbits = 0;
int softserialchars = 0;
int softserialtime = 0;
unsigned char softseriallastchar = 0xAA;


#define DOTX

#define SOFTWARE_SERIAL_BUFFER_SIZE 256
typedef struct tag_SOFTWARE_SERIAL_BUFFER {
    unsigned char buff[SOFTWARE_SERIAL_BUFFER_SIZE];
    int head; 
    int tail; 
    int bit;

    unsigned long lasttime;
    char lastvalue;
    
    // count of buffer overflows
    unsigned int overflow;

} SOFTWARE_SERIAL_BUFFER;

volatile SOFTWARE_SERIAL_BUFFER softwareserialRXbuffer;
volatile SOFTWARE_SERIAL_BUFFER softwareserialTXbuffer;

void SoftwareSerialReadTimer(void){
  unsigned int time = softwareserialtimer.Instance->CNT;
  timerval = time;
}

void SoftwareSerialInit(void){
  memset((void *)&softwareserialRXbuffer, 0, sizeof(softwareserialRXbuffer));
  memset((void *)&softwareserialTXbuffer, 0, sizeof(softwareserialTXbuffer));

  softwareserialRXbuffer.bit = -1; // awaiting start bit
  softwareserialTXbuffer.bit = -1; // awaiting start bit
  
  // setup our GPIO pin for rising and falling interrupts
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin = SOFTWARE_SERIAL_RX_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING_FALLING;
  HAL_GPIO_Init(SOFTWARE_SERIAL_RX_PORT, &GPIO_InitStruct);

  //SystemCoreClock
  // setup TIM2:
  __HAL_RCC_TIM2_CLK_ENABLE();
  softwareserialtimer.Instance = TIM2;
  softwareserialtimer.Init.Prescaler         = 64000000 / 2 / SOFTWARE_SERIAL_RX_TIMER_FREQ;
  softwareserialtimer.Init.CounterMode       = TIM_COUNTERMODE_UP;
  softwareserialtimer.Init.Period            = 0xFFFF;
  softwareserialtimer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  softwareserialtimer.Init.RepetitionCounter = 0;
  //softwareserialtimer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&softwareserialtimer);
  HAL_TIM_Base_Start(&softwareserialtimer);

#ifdef DOTX

  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = SOFTWARE_SERIAL_TX_PIN;
  HAL_GPIO_Init(SOFTWARE_SERIAL_TX_PORT, &GPIO_InitStruct);

  // setup TIM5:
  __HAL_RCC_TIM5_CLK_ENABLE();
  softwareserialtimerTX.Instance = TIM5;
  softwareserialtimerTX.Init.Prescaler         = 64000000 / 2 / SOFTWARE_SERIAL_TX_TIMER_FREQ;
  softwareserialtimerTX.Init.CounterMode       = TIM_COUNTERMODE_UP;
  softwareserialtimerTX.Init.Period            = 1;
  softwareserialtimerTX.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  softwareserialtimerTX.Init.RepetitionCounter = 0;
  //softwareserialtimerTX.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&softwareserialtimerTX);
  HAL_TIM_Base_Start(&softwareserialtimerTX);

  __HAL_TIM_ENABLE_IT(&softwareserialtimerTX, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);

#endif


  // and enable the interrupts.
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}


//////////////////////////////////////////////////////////
// get a single character if available.
// return -1 if none.
short softwareserial_getrx(){
    short t = -1;
    if (softwareserialRXbuffer.head != softwareserialRXbuffer.tail){
        t = softwareserialRXbuffer.buff[softwareserialRXbuffer.tail];
        softwareserialRXbuffer.tail = ((softwareserialRXbuffer.tail + 1 ) % SOFTWARE_SERIAL_BUFFER_SIZE);
    }
    return t;
}

//////////////////////////////////////////////////////////
// put a single character if room in output buffer
void softwareserial_puttx(unsigned char value){
    int count = softwareserialTXbuffer.head - softwareserialTXbuffer.tail;
    if (count < 0) 
        count += SOFTWARE_SERIAL_BUFFER_SIZE;

    if (count >= SOFTWARE_SERIAL_BUFFER_SIZE-2){
        softwareserialTXbuffer.overflow++;
        return;
    }
    softwareserialTXbuffer.buff[softwareserialTXbuffer.head] = value;
    softwareserialTXbuffer.head = ((softwareserialTXbuffer.head + 1 ) % SOFTWARE_SERIAL_BUFFER_SIZE);
}

//////////////////////////////////////////////////////////
// copy a buffer of data to the output buffer
// return length, or 0 if it won't fit
int softwareserial_Send(unsigned char *data, int len){
    int count = softwareserialTXbuffer.head - softwareserialTXbuffer.tail;
    if (count < 0) 
        count += SOFTWARE_SERIAL_BUFFER_SIZE;

    if (count >= SOFTWARE_SERIAL_BUFFER_SIZE-2){
        softwareserialTXbuffer.overflow++;
        return 0;
    }

    for (int i = 0; i < len; i++){
        softwareserial_puttx( data[i] );
    }
    return len;
}

//////////////////////////////////////////////////////////
// copy a buffer of data to the output buffer in chunks
// waiting if necessary for the data to leave so there is room
// use sparingly as this will pause main loop!!!
// used in 'protocol' for long strings, like '?' response
int softwareserial_Send_Wait(unsigned char *data, int len){
    int orglen = len;
    while (len){
        int count = softwareserialTXbuffer.head - softwareserialTXbuffer.tail;
        if (count < 0) 
            count += SOFTWARE_SERIAL_BUFFER_SIZE;

        int avail = (SOFTWARE_SERIAL_BUFFER_SIZE-3) - count;

        int sendlen = len;
        if (sendlen > avail){
            sendlen = avail;
        }
        softwareserial_Send(data, sendlen);
        len -= sendlen;
        data += sendlen;

        if (len > 0){
            // wait for an appropriate period
            float delay_ms = (len > sendlen)? sendlen:len;
            delay_ms *= SOFTWARE_SERIAL_MS_PER_BYTE;
            HAL_Delay((int)delay_ms + 2); //delay in ms, plus 2
        }
    }

    return orglen;
}



// interrupt on rising or falling edge of serial....
void softwareserialRXInterrupt(void){
    softserialbits++;

    char value = (SOFTWARE_SERIAL_RX_PORT->IDR & SOFTWARE_SERIAL_RX_PIN)?1:0;
    unsigned int time = softwareserialtimer.Instance->CNT;
    softserialtime = time;

    timerval = time;

    // time is in terms of baud *8
    int tdiff = time - softwareserialRXbuffer.lasttime;
    if (tdiff < 0) tdiff += 0x10000;

    int bits = (tdiff+2)/8;

    if (bits > 10) {
        bits = 10;
        softwareserialRXbuffer.bit = -2;
    }

    if ((softwareserialRXbuffer.bit == -2) && (softwareserialRXbuffer.lastvalue == 0)) {
        softwareserialRXbuffer.bit = -1;
        softwareserialRXbuffer.buff[softwareserialRXbuffer.head] = 0;
    }

    if (softwareserialRXbuffer.bit > -2){
        for (int i = 0; i < bits; i++){
            if (softwareserialRXbuffer.bit >= 0){
                softwareserialRXbuffer.buff[softwareserialRXbuffer.head] >>= 1;
                softwareserialRXbuffer.buff[softwareserialRXbuffer.head] |= (softwareserialRXbuffer.lastvalue?0x80:0);
            }
            softwareserialRXbuffer.bit++;
            if (softwareserialRXbuffer.bit >= 8){
                int count = softwareserialRXbuffer.head - softwareserialRXbuffer.tail;
                if (count < 0) count += SOFTWARE_SERIAL_BUFFER_SIZE;
                if (count >=  SOFTWARE_SERIAL_BUFFER_SIZE-2){
                    softwareserialRXbuffer.overflow++;
                    break;
                } else {
                    softserialchars++;
                    softseriallastchar = softwareserialRXbuffer.buff[softwareserialRXbuffer.head];
                    softwareserialRXbuffer.head = (softwareserialRXbuffer.head + 1) % SOFTWARE_SERIAL_BUFFER_SIZE;
                    softwareserialRXbuffer.bit = -2;
                    break;
                }
            }
        }
    }

    softwareserialRXbuffer.lastvalue = value;
    softwareserialRXbuffer.lasttime = time;

}


#ifdef DOTX
// transmit interrupt TIM5
void SoftwareSerial_ISR_Callback(void){
    short t = -1;

    // debug
   	sensor_set_flash(0, 3);

    if (softwareserialTXbuffer.head != softwareserialTXbuffer.tail){
        t = softwareserialTXbuffer.buff[softwareserialTXbuffer.tail];
        switch (softwareserialTXbuffer.bit){
            case -1: // send start bit
                HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_RESET);
                softwareserialTXbuffer.bit++;
                break;
            case 8: // send stop bit and next
                HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_SET);
                softwareserialTXbuffer.tail = ((softwareserialTXbuffer.tail + 1 ) % SOFTWARE_SERIAL_BUFFER_SIZE);
                softwareserialTXbuffer.bit = -1;
                ssbits++;
                break;
            default:
                if (t & (0x1 << softwareserialTXbuffer.bit)){
                    HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_SET);
                } else {
                    HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_RESET);
                }
                softwareserialTXbuffer.bit++;
                break;
        }
    } else {
        // send high
        HAL_GPIO_WritePin(SOFTWARE_SERIAL_TX_PORT, SOFTWARE_SERIAL_TX_PIN, GPIO_PIN_SET);
    }

}
#endif


#endif