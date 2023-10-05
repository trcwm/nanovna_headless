/*
 * Copyright (c) 2016-2017, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stddef.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "chprintf.h"
#include "strconvert.h"
#include "ch.h"

#include "hal.h"
#include "nanovna.h"
#include "si5351.h"
#include "usbcfg.h"
#include "cmdhandler.h"

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler(void);

void hard_fault_handler_c(uint32_t *sp) __attribute__((naked));

void HardFault_Handler(void)
{
    uint32_t *sp;
    //__asm volatile ("mrs %0, msp \n\t": "=r" (sp) );
    __asm volatile("mrs %0, psp \n\t" : "=r"(sp));
    hard_fault_handler_c(sp);
}

void hard_fault_handler_c(uint32_t *sp)
{
    (void)sp;
    while (true)
    {
    }
}

void enter_dfu(void)
{
    // touch_stop_watchdog();
    // int x = 5, y = 20;
    // lcd_set_colors(LCD_FG_COLOR, LCD_BG_COLOR);
    //  leave a last message
    // lcd_clear_screen();
    // lcd_drawstring(x, y, "DFU: Device Firmware Update Mode\n"
    //                      "To exit DFU mode, please reset device yourself.");
    boardDFUEnter();
}

#if 0
float measured[2][POINTS_COUNT][2];

static inline void dsp_start(int count)
{
    wait_count = count;
    accumerate_count = bandwidth_accumerate_count[bandwidth];
    reset_dsp_accumerator();
}

static inline void dsp_wait(void)
{
    while (accumerate_count > 0) __WFI();
}

#ifdef ENABLED_DUMP
static void duplicate_buffer_to_dump(int16_t *p)
{
    if (dump_selection == 1)
        p = samp_buf;
    else if (dump_selection == 2)
        p = ref_buf;
    memcpy(dump_buffer, p, sizeof dump_buffer);
}
#endif

#endif

int32_t g_callbackCount = 0;
int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];

void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
    // int16_t *p = &rx_buffer[offset];
    (void)i2sp;
    (void)n;

#if PORT_SUPPORTS_RT
    cnt_e = port_rt_get_counter_value();
    stat.interval_cycles = cnt_s - stat.last_counter_value;
    stat.busy_cycles = cnt_e - cnt_s;
    stat.last_counter_value = cnt_s;
#endif
    // stat.callback_count++;
    g_callbackCount++;
}

static const I2SConfig i2sconfig =
    {
        NULL,      // TX Buffer
        rx_buffer, // RX Buffer
        AUDIO_BUFFER_LEN * 2,
        NULL,             // tx callback
        i2s_end_callback, // rx callback
        0,                // i2scfgr
        2                 // i2spr
};

// I2C clock bus setting: depend from STM32_I2C1SW in mcuconf.h
static const I2CConfig i2ccfg =
    {
        .timingr = // TIMINGR register initialization. (use I2C timing configuration tool for
// STM32F3xx and STM32F0xx microcontrollers (AN4235))
#if STM32_I2C1SW == STM32_I2C1SW_HSI
        // STM32_I2C1SW == STM32_I2C1SW_HSI     (HSI=8MHz)
        // 400kHz @ HSI 8MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from
        // STM32 RM0091 Reference manual)
        STM32_TIMINGR_PRESC(0U) | STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(1U) |
        STM32_TIMINGR_SCLH(3U) | STM32_TIMINGR_SCLL(9U),
// Old values voodoo magic 400kHz @ HSI 8MHz
// 0x00300506,
#elif STM32_I2C1SW == STM32_I2C1SW_SYSCLK
        // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 48MHz)
        // 400kHz @ SYSCLK 48MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from
        // STM32 RM0091 Reference manual)
        STM32_TIMINGR_PRESC(5U) | STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
        STM32_TIMINGR_SCLH(3U) | STM32_TIMINGR_SCLL(9U),
// 600kHz @ SYSCLK 48MHz, manually get values, x1.5 I2C speed, but need calc timings
//  STM32_TIMINGR_PRESC(3U)  |
//  STM32_TIMINGR_SCLDEL(2U) | STM32_TIMINGR_SDADEL(2U) |
//  STM32_TIMINGR_SCLH(4U)   | STM32_TIMINGR_SCLL(4U),
#else
#error "Need Define STM32_I2C1SW and set correct TIMINGR settings"
#endif
        .cr1 = 0, // CR1 register initialization.
        .cr2 = 0  // CR2 register initialization.
};

static DACConfig dac1cfg1 =
    {
        // init:         2047U,
        init : 1922U,
        datamode :
            DAC_DHRM_12BIT_RIGHT
    };

static THD_WORKING_AREA(waThread1, 640);
static THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    chRegSetThreadName("cmdBackgroundThread");
    cmdBackgroundThread();
}

// Main thread stack size defined in makefile USE_PROCESS_STACKSIZE = 0x200
// Profile stack usage (enable threads command by def ENABLE_THREADS_COMMAND) show:
// Stack maximum usage = 472 bytes (need test more and run all commands), free stack = 40
// bytes
//
int main(void)
{
    halInit();
    chSysInit();

    // palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
    // palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(1) | PAL_STM32_OTYPE_OPENDRAIN);
    i2cStart(&I2CD1, &i2ccfg);
    si5351_init();

    // MCO on PA8
    // palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(0));
    /*
     * Initializes a serial-over-USB CDC driver.
     */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
     * Activates the USB driver and then the USB bus pull-up on D+.
     * Note, a delay is inserted in order to not have to disconnect the cable
     * after a reset.
     */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(200);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(200);

    /*
     * I2S Initialize
     */
    tlv320aic3204_init();
    i2sInit();
    i2sObjectInit(&I2SD2);
    i2sStart(&I2SD2, &i2sconfig);
    i2sStartExchange(&I2SD2);

    // create sweep thread
    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO - 1, Thread1, NULL);

    while (true)
    {
        if (SDU1.config->usbp->state == USB_ACTIVE)
        {
            do
            {
                uint8_t c;
                if (streamRead(gs_usb_stream, &c, 1) != 0)
                {
                    submitCmdByte(c);
                }
                else
                {
                    chThdSleepMilliseconds(200);
                }
            } while (SDU1.config->usbp->state == USB_ACTIVE);
        }
        chThdSleepMilliseconds(1000);
    }
}
