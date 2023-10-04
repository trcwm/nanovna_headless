/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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
#include "ch.h"

// Need enable HAL_USE_SPI in halconf.h
#define __USE_DISPLAY_DMA__

/*
 * main.c
 */

// Minimum frequency set
#define START_MIN                50000
// Maximum frequency set
#define STOP_MAX                 2700000000U
// Frequency offset (sin_cos table in dsp.c generated for this offset, if change need create new table)
#define FREQUENCY_OFFSET         5000
// Speed of light const
#define SPEED_OF_LIGHT           299792458
// pi const
#define VNA_PI                   3.14159265358979323846

#define POINTS_COUNT 101
extern float measured[2][POINTS_COUNT][2];

#define CAL_LOAD 0
#define CAL_OPEN 1
#define CAL_SHORT 2
#define CAL_THRU 3
#define CAL_ISOLN 4

#define CALSTAT_LOAD (1<<0)
#define CALSTAT_OPEN (1<<1)
#define CALSTAT_SHORT (1<<2)
#define CALSTAT_THRU (1<<3)
#define CALSTAT_ISOLN (1<<4)
#define CALSTAT_ES (1<<5)
#define CALSTAT_ER (1<<6)
#define CALSTAT_ET (1<<7)
#define CALSTAT_ED CALSTAT_LOAD
#define CALSTAT_EX CALSTAT_ISOLN
#define CALSTAT_APPLY (1<<8)
#define CALSTAT_INTERPOLATED (1<<9)

#define MAX_FREQ_TYPE 5
enum stimulus_type
{
    ST_START=0, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
};

void toggle_sweep(void);

#define SWEEP_ENABLE  0x01
#define SWEEP_ONCE    0x02
extern int8_t sweep_mode;
extern const char *info_about[];

/*
 * dsp.c
 */
// 5ms @ 48kHz
#define AUDIO_BUFFER_LEN 96
extern int16_t rx_buffer[];

#define STATE_LEN 32
#define SAMPLE_LEN 48

/*
* tlv320aic3204.c
*/

extern void tlv320aic3204_init(void);
extern void tlv320aic3204_set_gain(int lgain, int rgain);
extern void tlv320aic3204_select(int channel);

#define CONFIG_MAGIC 0x434f4e45 /* 'CONF' */

void enter_dfu(void);
