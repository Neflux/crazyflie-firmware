/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2020 Bitcraze AB
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

#define DEBUG_MODE

#ifdef DEBUG_MODE
#include "debug.h"
#endif

#ifdef CALIBRATED_LED_MORSE
  #define DOT 100
  #define DASH (3 * DOT)
  #define GAP DOT
  #define LETTER_GAP (3 * DOT)
  #define WORD_GAP (7 * DOT)
#endif // #ifdef CALIBRATED_LED_MORSE

#define LEDSEQ_CHARGE_CYCLE_TIME_500MA  1000
#define LEDSEQ_CHARGE_CYCLE_TIME_MAX    500

/* Led sequences */
ledseqStep_t seq_lowbat_def[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_lowbat = {
  .sequence = seq_lowbat_def,
  .led = LOWBAT_LED,
};

#define NO_CONTEXT 0
ledseqContext_t* sequences = NO_CONTEXT;

ledseqStep_t seq_calibrated_def[] = {
#ifndef CALIBRATED_LED_MORSE
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(450)},
  {    0, LEDSEQ_LOOP},
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

ledseqContext_t seq_calibrated = {
  .sequence = seq_calibrated_def,
  .led = SYS_LED,
};

ledseqStep_t seq_alive_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(1950)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_alive = {
  .sequence = seq_alive_def,
  .led = SYS_LED,
};

ledseqStep_t seq_linkup_def[] = {
  { true, LEDSEQ_WAITMS(1)},
  {false, LEDSEQ_WAITMS(0)},
  {    0, LEDSEQ_STOP},
};

ledseqContext_t seq_linkUp = {
  .sequence = seq_linkup_def,
  .led = LINK_LED,
};

ledseqContext_t seq_linkDown = {
  .sequence = seq_linkup_def,
  .led = LINK_DOWN_LED,
};

ledseqStep_t seq_charged_def[] = {
  { true, LEDSEQ_WAITMS(1000)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_charged = {
  .sequence = seq_charged_def,
  .led = CHG_LED,
};

ledseqStep_t seq_charging_def[] = {
  { true, LEDSEQ_WAITMS(200)},
  {false, LEDSEQ_WAITMS(800)},
  {    0, LEDSEQ_LOOP},
};

ledseqContext_t seq_charging = {
  .sequence = seq_charging_def,
  .led = CHG_LED,
};

ledseqStep_t seq_testPassed_def[] = {
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  { true, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_WAITMS(50)},
  {false, LEDSEQ_STOP},
};

ledseqContext_t seq_testPassed = {
  .sequence = seq_testPassed_def,
  .led = LINK_LED,
};

ledseqContext_t seq_testFailed = {
  .sequence = seq_testPassed_def,
  .led = SYS_LED,
};

struct ledseqCmd_s {
  enum {run, stop} command;
  ledseqContext_t *sequence;
};

/* Led sequence handling machine implementation */
static void runLedseq(xTimerHandle xTimer);
//static void updateActive(led_t led);

NO_DMA_CCM_SAFE_ZERO_INIT static ledseqContext_t* activeSeq[LED_NUM];

NO_DMA_CCM_SAFE_ZERO_INIT static xTimerHandle timer[LED_NUM];
NO_DMA_CCM_SAFE_ZERO_INIT static StaticTimer_t timerBuffer[LED_NUM];

static xSemaphoreHandle ledseqMutex;
static xQueueHandle ledseqCmdQueue;

static bool isInit = false;
static bool ledseqEnabled = false;

static void lesdeqCmdTask(void* param);

enum emission_state {
    EMISSION_IDLE = 0,          // idle: LEDs are always ON
    EMISSION_ACTIVE = 1,        // active: the message (packet) is sent repeatedly
    EMISSION_DEBUG = 2          // debug: the LEDs keep alternating between 0 (off) and 1 (on)
};
static uint8_t emission_mode = EMISSION_IDLE;   // default state: LEDs always on (idle)

// the stabilized frequency that the *DECODER* can achieve
static float led_blink_frequency = 34.74f, old_lbf;
// how many samples the *DECODER* is set to do per single signal/bit
static uint8_t samples_per_signal = 8, old_sps;

enum packet_state {
    PACKET_IDLE = 1,            // sending idle bits (LEDs on)
    PACKET_START_BIT = 2,       // sending the start bit (LEDs off)
    PACKET_PAYLOAD = 4,         // sending the payload, bit by bit
    PACKET_STOP_BITS = 8,       // sending the stop bits (pairs of '10's)
};
// example sequence:
// 11111111-mode set to 1-01110101010101110-mode set to 2-1010101010101010101010101010...
//  ^ idle bits, idle mode           ^ idle bit between one packet and another
//                        ^ start  ^^ stop bits (one pair only)
//                         ^^^^^^^^ payload of number 11101010 --> 234 TODO: big or little endian?
//                                    ^^^^^ start of repeated packet  ^^^^ debug mode, keeps alternating
//

// check the declaration for extra info
static uint8_t current_packet_state, current_led_value, payload_len, payload, active_payload, payload_progress_counter,
idle_bits, stop_bits;
static int8_t idle_left, stop_bits_left;

void setLeds(uint8_t val) {
    ledSet(LED_RED_R, val);
    ledSet(LED_RED_L, val);
    ledSet(LED_GREEN_L, val);
    ledSet(LED_GREEN_R, val);
    ledSet(LED_BLUE_L, val);
//    ledSet(LED_BLUE_R, val); // TODO: unlock also this LED with newer firmware
}

void ledseqInit() {
  if(isInit) {
    return;
  }

  ledInit();

  /* Led sequence priority */
  ledseqRegisterSequence(&seq_testPassed);
  ledseqRegisterSequence(&seq_testFailed);
  ledseqRegisterSequence(&seq_lowbat);
  ledseqRegisterSequence(&seq_charged);
  ledseqRegisterSequence(&seq_charging);
  ledseqRegisterSequence(&seq_calibrated);
  ledseqRegisterSequence(&seq_alive);
//  ledseqRegisterSequence(&seq_linkUp);
  ledseqRegisterSequence(&seq_linkDown);

  //Initialise the sequences state
  for(int i=0; i<LED_NUM; i++) {
    activeSeq[i] = 0;
  }

    /// ---- Minimal edit: instead of allocating LED_NUM timers, lets just use one for our purpose: regular blinking
//  //Init the soft timers that runs the led sequences for each leds
//  for(int i=0; i<LED_NUM; i++) {
//    timer[i] = xTimerCreateStatic("ledseqTimer", M2T(1000), pdFALSE, (void*)i, runLedseq, &timerBuffer[i]);
//  }

    // main settings, global variables
    old_sps = samples_per_signal;               // how many samples the *DECODER* is set to do per single signal/bit
    old_lbf = led_blink_frequency;              // the stabilized frequency that the *DECODER* can achieve

    current_led_value = 1;                      // default LED state: on
    payload_len = 8;                            // default payload length: 8
    payload_progress_counter = 0;               // setup for first packet, keeps track of payload bit progress
    current_packet_state = PACKET_IDLE;         // default state when packet blinking is enabled: start with idle bits
    idle_bits = 1;                              // default idle bits for each packet
    idle_left = idle_bits;                      // setup for first packet
    stop_bits = 1;                              // default stop bits inside packet
    stop_bits_left = stop_bits * 2;             // setup for first packet

    payload = active_payload = 85;              // default value to encode (last packet, current packet) [0:255]

    // setting just the first timer, for our purpose
    timer[0] = xTimerCreateStatic("ledseqTimer",
                                  M2T(1000.0f * samples_per_signal / led_blink_frequency), // our target frequency
                                  pdTRUE, 0, runLedseq,
                                  &timerBuffer[0]);
    xTimerStart(timer[0], M2T(500)); // let's wait a bit before starting the first loop
    DEBUG_PRINT("Emission loop started\n");
    /// ---- end of edit

  ledseqMutex = xSemaphoreCreateMutex();

  ledseqCmdQueue = xQueueCreate(10, sizeof(struct ledseqCmd_s));
  xTaskCreate(lesdeqCmdTask, LEDSEQCMD_TASK_NAME, LEDSEQCMD_TASK_STACKSIZE, NULL, LEDSEQCMD_TASK_PRI, NULL);

  isInit = true;
}

#define MIN(a, b) (((a)<(b))?(a):(b))
#define MAX(a, b) (((a)>(b))?(a):(b))
static void runLedseq(xTimerHandle xTimer) {

    if (!ledseqEnabled) return; // keeping it for external override if possible; not sure if this will ever fire

    if (emission_mode > EMISSION_DEBUG) emission_mode = 0; // sanitizing weird inputs, defaulting to state 0 (idling, always on)

    if (emission_mode != EMISSION_ACTIVE){ // either idle or debug mode
        DEBUG_PRINT((emission_mode==EMISSION_IDLE)?"Idle (always on)":"Debug (simple blink)");

        // in case of Debug mode, keep inverting the LEDs state
        current_led_value = (emission_mode==EMISSION_IDLE)?(1):(1 - current_led_value);

        // at any moment, the user could switch back to packet sending; refreshing/resetting counters does not hurt
        current_packet_state = PACKET_IDLE;
        idle_left = (int8_t) idle_bits;
        payload_progress_counter = 0;
    }
    else switch(current_packet_state){ // when emission mode is 1 (active packet encoding/sending)
        case PACKET_IDLE:
            DEBUG_PRINT("Idle");
            if (--idle_left <= 0){
                // time to start with the actual message
                current_packet_state = PACKET_START_BIT;
            }
            current_led_value = 1;
            break;
        case PACKET_START_BIT:
            DEBUG_PRINT("Start");
            current_led_value = 0;                          // start bit by definition is 0
            current_packet_state = PACKET_PAYLOAD;      // let's go to the payload part
            if (payload != active_payload)                  // if the user requested a different payload with cfclient
                active_payload = payload;                   // store it (only now, cannot happen mid-packet)
            break;
        case PACKET_PAYLOAD:
            DEBUG_PRINT("Payload (%d)", payload_progress_counter);

            // extract the respective bit of this time-step from the binary representation of 'active_payload'
            current_led_value = (active_payload >> (payload_len - payload_progress_counter - 1)) & 1;
            payload_progress_counter++; // update the progress

            // check whether the entire payload has been sent (sanitize the payload length if the user changed it)
            if (payload_progress_counter >= MIN(MAX(payload_len, 1), 8)) {
                payload_progress_counter = 0;               // reset the payload counter for the next message
                current_packet_state = PACKET_STOP_BITS;      // will start sending the STOP bits at the next iteration
                stop_bits_left = (int8_t) (stop_bits * 2);             // set up the remaining default bits (pairs of '10')
            }
            break;
        case PACKET_STOP_BITS:
            DEBUG_PRINT("Stop bit");

            // calculate the current value for the current stop bits (a sequence of (10) like 101010...)
            current_led_value = (stop_bits_left-- % 2) == 0;

            if (stop_bits_left == 0) {
                current_packet_state = PACKET_IDLE;
                idle_left = (int8_t) idle_bits;
            }
            break;
        default: {
            DEBUG_PRINT("Error: unexpected emission state (%d)", current_packet_state);

        }
    }

    // actually set the LEDs with the target bit
    DEBUG_PRINT(": %d\n", current_led_value);
    setLeds(current_led_value);

    // if either of the two parameters are changed via cfclient
    // reset the current packet sending and update the timer with a new blinking period
    if ((old_sps != samples_per_signal) || (old_lbf != led_blink_frequency)) {
        DEBUG_PRINT("Changing period to %d\n", M2T(1000.0f * samples_per_signal / led_blink_frequency));

        xTimerChangePeriod(xTimer, M2T(1000.0f * samples_per_signal / led_blink_frequency), M2T(1000));

        old_sps = samples_per_signal;
        old_lbf = led_blink_frequency;

        current_packet_state = PACKET_IDLE;
        idle_left = (int8_t) idle_bits;
        payload_progress_counter = 0;
    }
}


static void lesdeqCmdTask(void* param) {
  struct ledseqCmd_s command;
  while(1) {
    xQueueReceive(ledseqCmdQueue, &command, portMAX_DELAY);

    switch(command.command) {
      case run:
        ledseqRunBlocking(command.sequence);
        break;
      case stop:
        ledseqStopBlocking(command.sequence);
        break;
    }
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

bool ledseqRun(ledseqContext_t *context) {
  struct ledseqCmd_s command;
  command.command = run;
  command.sequence = context;
  if (xQueueSend(ledseqCmdQueue, &command, 0) == pdPASS) {
    return true;
  }
  return false;
}

void ledseqRunBlocking(ledseqContext_t *context) {
    /// easiest way to silence all the other components

//    const led_t led = context->led;
//
//    xSemaphoreTake(ledseqMutex, portMAX_DELAY);
//    context->state = 0;  //Reset the seq. to its first step
//    updateActive(led);
//    xSemaphoreGive(ledseqMutex);
//
//    // Run the first step if the new seq is the active sequence
//    if (activeSeq[led] == context) {
//        runLedseq(timer[led]);
//    }
}

void ledseqSetChargeLevel(const float chargeLevel) {
  int onTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA * chargeLevel;
  int offTime = LEDSEQ_CHARGE_CYCLE_TIME_500MA - onTime;

  seq_charging.sequence[0].action = onTime;
  seq_charging.sequence[1].action = offTime;
}

bool ledseqStop(ledseqContext_t *context) {
  struct ledseqCmd_s command;
  command.command = stop;
  command.sequence = context;
  if (xQueueSend(ledseqCmdQueue, &command, 0) == pdPASS) {
    return true;
  }
  return false;
}

void ledseqStopBlocking(ledseqContext_t *context) {
    /// easiest way to silence all the other components

//  const led_t led = context->led;
//
//  xSemaphoreTake(ledseqMutex, portMAX_DELAY);
//  context->state = LEDSEQ_STOP;  //Stop the seq.
//  updateActive(led);
//  xSemaphoreGive(ledseqMutex);
//
//  //Run the next active sequence (if any...)
//  runLedseq(timer[led]);
}

void ledseqRegisterSequence(ledseqContext_t* context) {
  context->state = LEDSEQ_STOP;
  context->nextContext = NO_CONTEXT;

  if (sequences == NO_CONTEXT) {
    sequences = context;
  } else {
    ledseqContext_t* last = sequences;
    if (last == context) {
      // Skip if already registered
      return;
    }

    while (last->nextContext != NO_CONTEXT) {
      last = last->nextContext;
      if (last == context) {
        // Skip if already registered
        return;
      }
    }

    last->nextContext = context;
  }
}

// Utility functions

/// easiest way to silence all the other components
//static void updateActive(led_t led) {
//  activeSeq[led] = NO_CONTEXT;
//  ledSet(led, false);
//
//  for (ledseqContext_t* sequence = sequences; sequence != 0; sequence = sequence->nextContext) {
//    if (sequence->led == led && sequence->state != LEDSEQ_STOP) {
//      activeSeq[led] = sequence;
//      break;
//    }
//  }
//}

/// for extra info, look at the variables definition and declaration
PARAM_GROUP_START(led_encode)
PARAM_ADD(PARAM_UINT8, emissionMode,        &emission_mode)                 // [0,1,2] (idle, active, debug)
PARAM_ADD(PARAM_FLOAT, blinkFrequency,      &led_blink_frequency)           // decoder frequency
PARAM_ADD(PARAM_UINT8, samplesPerSignal,    &samples_per_signal)            // decoder prefixed samples per signal
PARAM_ADD(PARAM_UINT8, payloadInteger,      &payload)                       // actual value to be encoded and 'blinked'
PARAM_ADD(PARAM_UINT8, payloadLength,       &payload_len)                   // payload length
PARAM_ADD(PARAM_UINT8, idleBits,            &idle_bits)                     // idle bits between one message and another
PARAM_ADD(PARAM_UINT8, stopBitsPairs,       &stop_bits)                     // stop bit pairs (10) at end of messages
PARAM_GROUP_STOP(led_encode)