/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ledseq.c - LED sequence handler
 */

#include <stdbool.h>

#include "ledseq.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"
#include "static_mem.h"

#include "led.h"
#include "param.h"
#include "log.h"
#include "debug.h"

#ifdef CALIBRATED_LED_MORSE
#define DOT 100
#define DASH (3 * DOT)
#define GAP DOT
#define LETTER_GAP (3 * DOT)
#define WORD_GAP (7 * DOT)
#endif // #ifdef CALIBRATED_LED_MORSE

/* Led sequence priority */
static ledseq_t const *sequences[] = {
        seq_testPassed,
        seq_lowbat,
        seq_charged,
        seq_charging,
        seq_chargingMax,
        seq_bootloader,
        seq_armed,
        seq_calibrated,
        seq_alive,
        seq_linkup,
};

/* Led sequences */
const ledseq_t seq_lowbat[] = {
        {true, LEDSEQ_WAITMS(1000)},
        {0,    LEDSEQ_LOOP},
};

const ledseq_t seq_armed[] = {
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(250)},
        {0,     LEDSEQ_LOOP},
};

const ledseq_t seq_calibrated[] = {
#ifndef CALIBRATED_LED_MORSE
        {true, LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(450)},
        {0, LEDSEQ_LOOP},
#else
        { true, LEDSEQ_WAITMS(DASH)},
        {false, LEDSEQ_WAITMS(GAP)},
        { true, LEDSEQ_WAITMS(DOT)},
        {false, LEDSEQ_WAITMS(GAP)},
        { true, LEDSEQ_WAITMS(DASH)},
        {false, LEDSEQ_WAITMS(GAP)},
        { true, LEDSEQ_WAITMS(DOT)},
        {false, LEDSEQ_WAITMS(LETTER_GAP)},
        { true, LEDSEQ_WAITMS(DOT)},
        {false, LEDSEQ_WAITMS(GAP)},
        { true, LEDSEQ_WAITMS(DOT)},
        {false, LEDSEQ_WAITMS(GAP)},
        { true, LEDSEQ_WAITMS(DASH)},
        {false, LEDSEQ_WAITMS(GAP)},
        { true, LEDSEQ_WAITMS(DOT)},
        {false, LEDSEQ_WAITMS(WORD_GAP)},
        {    0, LEDSEQ_LOOP},
#endif // ifndef CALIBRATED_LED_MORSE
};

const ledseq_t seq_alive[] = {
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(1950)},
        {0,     LEDSEQ_LOOP},
};


//TODO: Change, right now is called so fast it looks like seq_lowbat
const ledseq_t seq_altHold[] = {
        {true,  LEDSEQ_WAITMS(1)},
        {false, LEDSEQ_WAITMS(50)},
        {0,     LEDSEQ_STOP},
};

const ledseq_t seq_linkup[] = {
        {true,  LEDSEQ_WAITMS(1)},
        {false, LEDSEQ_WAITMS(0)},
        {0,     LEDSEQ_STOP},
};


const ledseq_t seq_charged[] = {
        {true, LEDSEQ_WAITMS(1000)},
        {0,    LEDSEQ_LOOP},
};

ledseq_t seq_charging[] = {
        {true,  LEDSEQ_WAITMS(200)},
        {false, LEDSEQ_WAITMS(800)},
        {0,     LEDSEQ_LOOP},
};

ledseq_t seq_chargingMax[] = {
        {true,  LEDSEQ_WAITMS(100)},
        {false, LEDSEQ_WAITMS(400)},
        {0,     LEDSEQ_LOOP},
};

const ledseq_t seq_bootloader[] = {
        {true,  LEDSEQ_WAITMS(500)},
        {false, LEDSEQ_WAITMS(500)},
        {0,     LEDSEQ_LOOP},
};

const ledseq_t seq_testPassed[] = {
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(50)},
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(50)},
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(50)},
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(50)},
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(50)},
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(50)},
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_STOP},
};

/* Led sequence handling machine implementation */
#define SEQ_NUM (sizeof(sequences)/sizeof(sequences[0]))

static void runLedseq(xTimerHandle xTimer);
//static int getPrio(const ledseq_t *seq);
//static void updateActive(led_t led);

//State of every sequence for every led: LEDSEQ_STOP if stopped or the current
//step
NO_DMA_CCM_SAFE_ZERO_INIT static int state[LED_NUM][SEQ_NUM];
//Active sequence for each led
NO_DMA_CCM_SAFE_ZERO_INIT static int activeSeq[LED_NUM];

NO_DMA_CCM_SAFE_ZERO_INIT static xTimerHandle
timer[LED_NUM];
NO_DMA_CCM_SAFE_ZERO_INIT static StaticTimer_t
timerBuffer[LED_NUM];

static xSemaphoreHandle ledseqSem;

static bool isInit = false;
static bool ledseqEnabled = false;


//static bool right_red_value = 1, left_red_value = 1, left_green_value = 1,
//        right_green_value = 1, left_blue_value = 1;//, right_blue_value = 1;

static float led_blink_frequency = 34;
static uint8_t samples_per_signal = 12;
static float old_sps = 12, old_lbf = 34;
static uint8_t current_led_value = 0;

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define SEQUENCE_LEN  ( 1 + 8 + 1 + 16)

static uint8_t loopSequence[SEQUENCE_LEN] = {
        0,
        0, 1, 1, 1, 0, 0, 0, 0,
        1,
        1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1
};
static uint8_t payload_len = 2;
static uint8_t payload = 1;
static uint8_t led_counter = 0;
static uint8_t debug = 0;

void setLeds(uint8_t val) {
    ledSet(LED_RED_R, val);
    ledSet(LED_RED_L, val);
    ledSet(LED_GREEN_L, val);
    ledSet(LED_GREEN_R, val);
    ledSet(LED_BLUE_L, val);
}

void ledseqInit() {
    int i, j;

    if (isInit)
        return;

    ledInit();

    //Initialise the sequences state
    for (i = 0; i < LED_NUM; i++) {
        activeSeq[i] = LEDSEQ_STOP;
        for (j = 0; j < SEQ_NUM; j++)
            state[i][j] = LEDSEQ_STOP;
    }
    old_sps = samples_per_signal;
    old_lbf = led_blink_frequency;
    timer[0] = xTimerCreateStatic("ledseqTimer", M2T(1000.0f * samples_per_signal / led_blink_frequency),
                                  pdTRUE, 0, runLedseq,
                                  &timerBuffer[0]);
    xTimerStart(timer[0], M2T(500));

    //Init the soft timers that runs the led sequences for each leds
//  for(i=0; i<LED_NUM; i++) {
//    timer[i] = xTimerCreateStatic("ledseqTimer", M2T(1000), pdFALSE, (void*)i, runLedseq, &timerBuffer[i]);
//  }

    vSemaphoreCreateBinary(ledseqSem);

    isInit = true;
}

static void runLedseq(xTimerHandle xTimer) {

    payload_len = MIN(MAX(payload_len,1), 8);
    if ((led_counter == 1 + payload_len) && (payload_len < 8))
        led_counter = 1 + 8;

    current_led_value = loopSequence[led_counter];


    if ((led_counter >= 1) && (led_counter < 1 + payload_len)) {
        current_led_value = (payload >> (led_counter - 1)) & 1;
        DEBUG_PRINT("Bit %d: %d\n", led_counter - 1, current_led_value);
//        DEBUG_PRINT("%d %d %d - %d %d %d %d \n",
////                    (uint8_t) payload & (1 << (led_counter - 2)),
//                    (payload >> (led_counter - 2)) & 1,
////                    0 != (payload & (1 << (led_counter - 2))),
//////                    (uint8_t) payload & (1 << (PAYLOAD_LEN - led_counter + 2)),
////                    (payload >> (PAYLOAD_LEN - led_counter + 2)) & 1,
//////                payload & (1 << (PAYLOAD_LEN - led_counter + 2)),
////        0 != (payload & (1 << (PAYLOAD_LEN - led_counter + 2))));

    }


    if (debug == 1) {
        current_led_value = led_counter % 2 == 0;
    }

    setLeds(current_led_value);
    DEBUG_PRINT("Setting LEDs to %d\n", current_led_value);

    led_counter = (led_counter + 1) % SEQUENCE_LEN;

    if ((old_sps != samples_per_signal) || (old_lbf != led_blink_frequency)) {
        DEBUG_PRINT("Changing period to %d\n", M2T(1000.0f * samples_per_signal / led_blink_frequency));
        xTimerChangePeriod(xTimer, M2T(1000.0f * samples_per_signal / led_blink_frequency), M2T(1000));

        old_sps = samples_per_signal;
        old_lbf = led_blink_frequency;
    }
}

bool ledseqTest(void) {
    bool status;

    status = isInit & ledTest();
#ifdef TURN_OFF_LEDS
    ledseqEnable(false);
    ledSet(LED_BLUE_L, 0);
#else
    ledseqEnable(true);
#endif

    return status;
}

void ledseqEnable(bool enable) {
    ledseqEnabled = enable;
}

void ledseqRun(led_t led, const ledseq_t *sequence) {
//  int prio = getPrio(sequence);
//
//  if(prio<0) return;
//
//  xSemaphoreTake(ledseqSem, portMAX_DELAY);
//  state[led][prio] = 0;  //Reset the seq. to its first step
//  updateActive(led);
//  xSemaphoreGive(ledseqSem);
//
//  //Run the first step if the new seq is the active sequence
//  if(activeSeq[led] == prio)
//    runLedseq(timer[led]);
}

void ledseqSetTimes(ledseq_t *sequence, int32_t onTime, int32_t offTime) {
    sequence[0].action = onTime;
    sequence[1].action = offTime;
}

void ledseqStop(led_t led, const ledseq_t *sequence) {
//  int prio = getPrio(sequence);
//
//  if(prio<0) return;
//
//  xSemaphoreTake(ledseqSem, portMAX_DELAY);
//  state[led][prio] = LEDSEQ_STOP;  //Stop the seq.
//  updateActive(led);
//  xSemaphoreGive(ledseqSem);
//
//  //Run the next active sequence (if any...)
//  runLedseq(timer[led]);
}

//Utility functions
//static int getPrio(const ledseq_t *seq)
//{
////  int prio;
////
////  //Find the priority of the sequence
////  for(prio=0; prio<SEQ_NUM; prio++)
////    if(sequences[prio]==seq) return prio;
////
////  return -1; //Invalid sequence
//    return 0;
//}

//static void updateActive(led_t led)
//{
//  int prio;
//
//  activeSeq[led]=LEDSEQ_STOP;
//  ledSet(led, false);
//
//  for(prio=0;prio<SEQ_NUM;prio++)
//  {
//    if (state[led][prio] != LEDSEQ_STOP)
//    {
//      activeSeq[led]=prio;
//      break;
//    }
//  }
//}

//PARAM_GROUP_START(led2)
//PARAM_ADD(PARAM_UINT8, RR, &right_red_value)
//PARAM_ADD(PARAM_UINT8, LR, &left_red_value)
//PARAM_ADD(PARAM_UINT8, RG, &right_green_value)
//PARAM_ADD(PARAM_UINT8, LG, &left_green_value)
//PARAM_ADD(PARAM_UINT8, LB, &left_blue_value)
////PARAM_ADD(PARAM_UINT8, RB, &right_blue_value)
//PARAM_GROUP_STOP(led2)

PARAM_GROUP_START(led3)
PARAM_ADD(PARAM_UINT8, Debug,
&debug)
PARAM_ADD(PARAM_FLOAT, BlinkFrequency,
&led_blink_frequency)
PARAM_ADD(PARAM_UINT8, SamplesPerSignal,
&samples_per_signal)
PARAM_ADD(PARAM_UINT8, payload,
&payload)
PARAM_ADD(PARAM_UINT8, payload_len,
&payload_len)
PARAM_GROUP_STOP(led3)

LOG_GROUP_START(led4)
LOG_ADD(LOG_UINT8, state,
&current_led_value)
LOG_GROUP_STOP(led4)