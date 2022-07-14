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

#ifdef DEBUG_MODE // for some reason, this makes the PARAMs un-editable from cfclient
#include "debug.h"
#else
static uint8_t compiler = 0;
#define DEBUG_PRINT(fmt, ...) (compiler = 1-compiler)
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
        {true, LEDSEQ_WAITMS(1000)},
        {0,    LEDSEQ_LOOP},
};

ledseqContext_t seq_lowbat = {
        .sequence = seq_lowbat_def,
        .led = LOWBAT_LED,
};

#define NO_CONTEXT 0
ledseqContext_t *sequences = NO_CONTEXT;

ledseqStep_t seq_calibrated_def[] = {
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

ledseqContext_t seq_calibrated = {
        .sequence = seq_calibrated_def,
        .led = SYS_LED,
};

ledseqStep_t seq_alive_def[] = {
        {true,  LEDSEQ_WAITMS(50)},
        {false, LEDSEQ_WAITMS(1950)},
        {0,     LEDSEQ_LOOP},
};

ledseqContext_t seq_alive = {
        .sequence = seq_alive_def,
        .led = SYS_LED,
};

ledseqStep_t seq_linkup_def[] = {
        {true,  LEDSEQ_WAITMS(1)},
        {false, LEDSEQ_WAITMS(0)},
        {0,     LEDSEQ_STOP},
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
        {true, LEDSEQ_WAITMS(1000)},
        {0,    LEDSEQ_LOOP},
};

ledseqContext_t seq_charged = {
        .sequence = seq_charged_def,
        .led = CHG_LED,
};

ledseqStep_t seq_charging_def[] = {
        {true,  LEDSEQ_WAITMS(200)},
        {false, LEDSEQ_WAITMS(800)},
        {0,     LEDSEQ_LOOP},
};

ledseqContext_t seq_charging = {
        .sequence = seq_charging_def,
        .led = CHG_LED,
};

ledseqStep_t seq_testPassed_def[] = {
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

ledseqContext_t seq_testPassed = {
        .sequence = seq_testPassed_def,
        .led = LINK_LED,
};

ledseqContext_t seq_testFailed = {
        .sequence = seq_testPassed_def,
        .led = SYS_LED,
};

struct ledseqCmd_s {
    enum {
        run, stop
    } command;
    ledseqContext_t *sequence;
};

/* Led sequence handling machine implementation */
static void runLedseq(xTimerHandle xTimer);

//static void updateActive(led_t led);

NO_DMA_CCM_SAFE_ZERO_INIT static ledseqContext_t
* activeSeq[LED_NUM];

NO_DMA_CCM_SAFE_ZERO_INIT static xTimerHandle
timer[LED_NUM];
NO_DMA_CCM_SAFE_ZERO_INIT static StaticTimer_t
timerBuffer[LED_NUM];

static xSemaphoreHandle ledseqMutex;
static xQueueHandle ledseqCmdQueue;

static bool isInit = false;
static bool ledseqEnabled = false;

static void lesdeqCmdTask(void *param);


static uint8_t debug = 0;
static float led_blink_frequency = 34;
static uint8_t samples_per_signal = 12;
static float old_sps, old_lbf;


enum emission_state {
    EMISSION_IDLE = 1,
    EMISSION_START = 2,
    EMISSION_PAYLOAD = 4,
    EMISSION_END = 8,
};
static uint8_t current_emission_state, current_led_value, payload_len, payload, payload_progress_counter, idle_left;

#define IDLE_BITS   2

void setLeds(uint8_t val) {
    ledSet(LED_RED_R, val);
    ledSet(LED_RED_L, val);
    ledSet(LED_GREEN_L, val);
    ledSet(LED_GREEN_R, val);
    ledSet(LED_BLUE_L, val);
//    ledSet(LED_BLUE_R, val);
}

void ledseqInit() {
    if (isInit) {
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
    ledseqRegisterSequence(&seq_linkDown);

    //Initialise the sequences state
    for (int i = 0; i < LED_NUM; i++) {
        activeSeq[i] = 0;
    }

    //Init the soft timers that runs the led sequences for each leds
//    for (int i = 0; i < LED_NUM; i++) {
//        timer[i] = xTimerCreateStatic("ledseqTimer", M2T(1000), pdFALSE, (void *) i, runLedseq, &timerBuffer[i]);
//    }
    old_sps = samples_per_signal;
    old_lbf = led_blink_frequency;
    current_emission_state = EMISSION_IDLE;
    current_led_value = 1;
    payload_len = 4;
    idle_left = IDLE_BITS;
    payload = 10;
    payload_progress_counter = 0;

    DEBUG_PRINT("Emission loop started\n");
    timer[0] = xTimerCreateStatic("ledseqTimer", M2T(1000.0f * samples_per_signal / led_blink_frequency),
                                  pdTRUE, 0, runLedseq,
                                  &timerBuffer[0]);
    xTimerStart(timer[0], M2T(500));


    ledseqMutex = xSemaphoreCreateMutex();

    ledseqCmdQueue = xQueueCreate(10, sizeof(struct ledseqCmd_s));
    xTaskCreate(lesdeqCmdTask, LEDSEQCMD_TASK_NAME, LEDSEQCMD_TASK_STACKSIZE, NULL, LEDSEQCMD_TASK_PRI, NULL);

    isInit = true;


}

#define MIN(a, b) (((a)<(b))?(a):(b))
#define MAX(a, b) (((a)>(b))?(a):(b))
static void runLedseq(xTimerHandle xTimer) {

    if (!ledseqEnabled) {
        return;
    }

    if (debug){
        DEBUG_PRINT("Debug");
        current_led_value = 1 - current_led_value;
    }
    else switch(current_emission_state){
        case EMISSION_IDLE:
            DEBUG_PRINT("Idle");
            if (--idle_left <= 0){
                idle_left = IDLE_BITS;
                current_emission_state = EMISSION_START;
            }
            current_led_value = 1;
            break;
        case EMISSION_START:
            DEBUG_PRINT("Start");
            current_led_value = 0;
            current_emission_state = EMISSION_PAYLOAD;
            break;
        case EMISSION_PAYLOAD:
            DEBUG_PRINT("Payload (%d)", payload_progress_counter);
            current_led_value = (payload >> (payload_len - payload_progress_counter - 1)) & 1;
            payload_progress_counter++;
            if (payload_progress_counter >= MIN(MAX(payload_len, 1), 8)) {
                payload_progress_counter = 0;
                current_emission_state = EMISSION_END;
            }
            break;
        case EMISSION_END:
            DEBUG_PRINT("End");
            current_led_value = 1;
            current_emission_state = EMISSION_IDLE;
            break;
    }
    DEBUG_PRINT(": %d\n", current_led_value);
    setLeds(current_led_value);

    if ((old_sps != samples_per_signal) || (old_lbf != led_blink_frequency)) {

        DEBUG_PRINT("Changing period to %d\n", M2T(1000.0f * samples_per_signal / led_blink_frequency));

        xTimerChangePeriod(xTimer, M2T(1000.0f * samples_per_signal / led_blink_frequency), M2T(1000));

        old_sps = samples_per_signal;
        old_lbf = led_blink_frequency;
    }
}


static void lesdeqCmdTask(void *param) {
    struct ledseqCmd_s command;
    while (1) {
        xQueueReceive(ledseqCmdQueue, &command, portMAX_DELAY);

        switch (command.command) {
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
//    const led_t led = context->led;
//
//    xSemaphoreTake(ledseqMutex, portMAX_DELAY);
//    context->state = LEDSEQ_STOP;  //Stop the seq.
//    updateActive(led);
//    xSemaphoreGive(ledseqMutex);
//
//    //Run the next active sequence (if any...)
//    runLedseq(timer[led]);
}

void ledseqRegisterSequence(ledseqContext_t *context) {
    context->state = LEDSEQ_STOP;
    context->nextContext = NO_CONTEXT;

    if (sequences == NO_CONTEXT) {
        sequences = context;
    } else {
        ledseqContext_t * last = sequences;
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

//static void updateActive(led_t led) {
//    activeSeq[led] = NO_CONTEXT;
//    ledSet(led, false);
//
//    for (ledseqContext_t *sequence = sequences; sequence != 0; sequence = sequence->nextContext) {
//        if (sequence->led == led && sequence->state != LEDSEQ_STOP) {
//            activeSeq[led] = sequence;
//            break;
//        }
//    }
//}
//#ifndef DEBUG_MODE
PARAM_GROUP_START(led3)
PARAM_ADD(PARAM_UINT8, Debug,&debug)
PARAM_ADD(PARAM_FLOAT, BlinkFrequency, &led_blink_frequency)
PARAM_ADD(PARAM_UINT8, SamplesPerSignal, &samples_per_signal)
PARAM_ADD(PARAM_UINT8, payload, &payload)
PARAM_ADD(PARAM_UINT8, payload_len, &payload_len)
PARAM_GROUP_STOP(led3)
//#endif