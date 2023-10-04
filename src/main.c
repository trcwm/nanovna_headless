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

#include "chprintf.h"

#include "strconvert.h"
#include "ch.h"

#include "hal.h"
#include "nanovna.h"
#include "si5351.h"
#include "usbcfg.h"

/*
 *  Shell settings
 */
// If need run shell as thread (use more amount of memory fore stack), after
// enable this need reduce spi_buffer size, by default shell run in main thread
// #define VNA_SHELL_THREAD

static BaseSequentialStream *shell_stream = (BaseSequentialStream *)&SDU1;

// Shell new line
#define VNA_SHELL_NEWLINE_STR "\r\n"
// Shell command promt
#define VNA_SHELL_PROMPT_STR "ch> "
// Shell max arguments
#define VNA_SHELL_MAX_ARGUMENTS 4
// Shell max command line size
#define VNA_SHELL_MAX_LENGTH 48

// Shell command functions prototypes
typedef void (*vna_shellcmd_t)(int argc, char *argv[]);
#define VNA_SHELL_FUNCTION(command_name) static void command_name(int argc, char *argv[])

// Shell command line buffer, args, nargs, and function ptr
static char shell_line[VNA_SHELL_MAX_LENGTH];
static char *shell_args[VNA_SHELL_MAX_ARGUMENTS + 1];
static uint16_t shell_nargs;
static volatile vna_shellcmd_t shell_function = 0;

static void update_frequencies(void);
static void set_frequencies(uint32_t start, uint32_t stop, uint16_t points);
static bool sweep(bool break_on_operation);

#define DRIVE_STRENGTH_AUTO (-1)
#define IS_HARMONIC_MODE(f) ((f) > FREQ_HARMONICS)

static int8_t drive_strength = DRIVE_STRENGTH_AUTO;
int8_t sweep_mode = SWEEP_ENABLE;
volatile uint8_t redraw_request = 0;  // contains REDRAW_XXX flags

// Version text, displayed in Config->Version menu, also send by info command
const char *info_about[] =
{
    BOARD_NAME,
    "2016-2020 Copyright @edy555",
    "Licensed under GPL. See: https://github.com/ttrftech/NanoVNA",
    "Version: " VERSION,
    "Build Time: " __DATE__ " - " __TIME__,
    "Kernel: " CH_KERNEL_VERSION,
    "Compiler: " PORT_COMPILER_NAME,
    "Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME,
    "Port Info: " PORT_INFO,
    "Platform: " PLATFORM_NAME,
    0  // sentinel
};

static THD_WORKING_AREA(waThread1, 640);
static THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    chRegSetThreadName("sweep");

    while (1)
    {
        bool completed = false;
        if (sweep_mode & (SWEEP_ENABLE | SWEEP_ONCE))
        {
            completed = sweep(true);
            sweep_mode &= ~SWEEP_ONCE;
        }
        else
        {
            __WFI();
        }
        // Run Shell command in sweep thread
        if (shell_function)
        {
            shell_function(shell_nargs - 1, &shell_args[1]);
            shell_function = 0;
            osalThreadSleepMilliseconds(10);
            continue;
        }

        if (sweep_mode & SWEEP_ENABLE && completed)
        {
            #if 0
            if ((domain_mode & DOMAIN_MODE) == DOMAIN_TIME) transform_domain();
            // Prepare draw graphics, cache all lines, mark screen cells for redraw
            plot_into_index(measured);
            redraw_request |= REDRAW_CELLS | REDRAW_BATTERY;

            if (uistat.marker_tracking)
            {
                int i = marker_search();
                if (i != -1 && active_marker != -1)
                {
                    markers[active_marker].index = i;
                    redraw_request |= REDRAW_MARKER;
                }
            }
            #endif
        }

        // plot trace and other indications as raster
        // draw_all(completed);  // flush markmap only if scan completed to prevent
        // remaining traces
    }
}

static inline void pause_sweep(void)
{
    sweep_mode &= ~SWEEP_ENABLE;
}

static inline void resume_sweep(void)
{
    sweep_mode |= SWEEP_ENABLE;
}

void toggle_sweep(void)
{
    sweep_mode ^= SWEEP_ENABLE;
}

// Shell commands output
static int shell_printf(const char *fmt, ...)
{
    va_list ap;
    int formatted_bytes;
    va_start(ap, fmt);
    formatted_bytes = chvprintf(shell_stream, fmt, ap);
    va_end(ap);
    return formatted_bytes;
}

VNA_SHELL_FUNCTION(cmd_pause)
{
    (void)argc;
    (void)argv;
    pause_sweep();
}

VNA_SHELL_FUNCTION(cmd_resume)
{
    (void)argc;
    (void)argv;

    // restore frequencies array and cal
    update_frequencies();
    //if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY)) cal_interpolate(lastsaveid);

    resume_sweep();
}

VNA_SHELL_FUNCTION(cmd_reset)
{
    (void)argc;
    (void)argv;

    if (argc == 1)
    {
        if (strcmp(argv[0], "dfu") == 0)
        {
            shell_printf("Performing reset to DFU mode\r\n");
            enter_dfu();
            return;
        }
    }
    shell_printf("Performing reset\r\n");

    rccEnableWWDG(FALSE);
    WWDG->CFR = 0x60;
    WWDG->CR = 0xff;

    /* wait forever */
    while (1)
        ;
}

const int8_t gain_table[] =
{
    0,   // 0 ~ 300MHz
    40,  // 300 ~ 600MHz
    50,  // 600 ~ 900MHz
    75,  // 900 ~ 1200MHz
    85,  // 1200 ~ 1500MHz
    95,  // 1500MHz ~
    95,  // 1800MHz ~
    95,  // 2100MHz ~
    95   // 2400MHz ~
};

#define DELAY_GAIN_CHANGE 2

static int adjust_gain(uint32_t newfreq)
{

#if 0    
    int new_order = newfreq / FREQ_HARMONICS;
    int old_order = si5351_get_frequency() / FREQ_HARMONICS;
    if (new_order != old_order)
    {
        tlv320aic3204_set_gain(gain_table[new_order], gain_table[new_order]);
        return DELAY_GAIN_CHANGE;
    }
#endif    
    return 0;
}

int set_frequency(uint32_t freq)
{
    int delay = adjust_gain(freq);
    int8_t ds = drive_strength;
    if (ds == DRIVE_STRENGTH_AUTO)
    {
        //FIXME:
        //ds = freq > FREQ_HARMONICS ? SI5351_CLK_DRIVE_STRENGTH_8MA
        //     : SI5351_CLK_DRIVE_STRENGTH_2MA;

        ds = SI5351_CLK_DRIVE_STRENGTH_2MA;
    }
    delay += si5351_set_frequency(freq, ds);
    return delay;
}



//
// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex
// return 2 If not found return -1 Used for easy parse command arguments
static int get_str_index(char *v, const char *list)
{
    int i = 0;
    while (1)
    {
        char *p = v;
        while (1)
        {
            char c = *list;
            if (c == '|') c = 0;
            if (c == *p++)
            {
                // Found, return index
                if (c == 0) return i;
                list++;  // Compare next symbol
                continue;
            }
            break;  // Not equal, break
        }
        // Set new substring ptr
        while (1)
        {
            // End of string, not found
            if (*list == 0) return -1;
            if (*list++ == '|') break;
        }
        i++;
    }
    return -1;
}

VNA_SHELL_FUNCTION(cmd_offset)
{
    if (argc != 1)
    {
        shell_printf("usage: offset {frequency offset(Hz)}\r\n");
        return;
    }
    si5351_set_frequency_offset(my_atoi(argv[0]));
}

VNA_SHELL_FUNCTION(cmd_freq)
{
    if (argc != 1)
    {
        goto usage;
    }
    uint32_t freq = my_atoui(argv[0]);

    pause_sweep();
    set_frequency(freq);
    return;
usage:
    shell_printf("usage: freq {frequency(Hz)}\r\n");
}

VNA_SHELL_FUNCTION(cmd_power)
{
    if (argc != 1)
    {
        shell_printf("usage: power {0-3|-1}\r\n");
        return;
    }
    drive_strength = my_atoi(argv[0]);
    //  set_frequency(frequency);
}


VNA_SHELL_FUNCTION(cmd_dac)
{
    // FIXME:
#if 0    
    int value;
    if (argc != 1)
    {
        shell_printf(
            "usage: dac {value(0-4095)}\r\n"
            "current value: %d\r\n",
            config.dac_value);
        return;
    }
    value = my_atoui(argv[0]);
    config.dac_value = value;
    dacPutChannelX(&DACD2, 0, value);
#endif
}

VNA_SHELL_FUNCTION(cmd_threshold)
{
#if 0    
    uint32_t value;
    if (argc != 1)
    {
        shell_printf(
            "usage: threshold {frequency in harmonic mode}\r\n"
            "current: %d\r\n",
            config.harmonic_freq_threshold);
        return;
    }
    value = my_atoui(argv[0]);
    config.harmonic_freq_threshold = value;
#endif
}

int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];

volatile uint8_t wait_count = 0;
volatile uint8_t accumerate_count = 0;

const int8_t bandwidth_accumerate_count[] =
{
    1,   // 1kHz
    3,   // 300Hz
    10,  // 100Hz
    33,  // 30Hz
    100  // 10Hz
};

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

void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
#if PORT_SUPPORTS_RT
    int32_t cnt_s = port_rt_get_counter_value();
    int32_t cnt_e;
#endif
    //int16_t *p = &rx_buffer[offset];
    (void)i2sp;
    (void)n;

    if (wait_count > 1)
    {
        --wait_count;
    }
    else if (wait_count > 0)
    {
        if (accumerate_count > 0)
        {
            //dsp_process(p, n);
            accumerate_count--;
        }
#ifdef ENABLED_DUMP
        duplicate_buffer_to_dump(p);
#endif
    }

#if PORT_SUPPORTS_RT
    cnt_e = port_rt_get_counter_value();
    stat.interval_cycles = cnt_s - stat.last_counter_value;
    stat.busy_cycles = cnt_e - cnt_s;
    stat.last_counter_value = cnt_s;
#endif
    //stat.callback_count++;
}

static const I2SConfig i2sconfig =
{
    NULL,       // TX Buffer
    rx_buffer,  // RX Buffer
    AUDIO_BUFFER_LEN * 2,
    NULL,              // tx callback
    i2s_end_callback,  // rx callback
    0,                 // i2scfgr
    2                  // i2spr
};

VNA_SHELL_FUNCTION(cmd_data)
{
#if 0    
    int i;
    int sel = 0;
    float(*array)[2];
    if (argc == 1) sel = my_atoi(argv[0]);

    if (sel == 0 || sel == 1)
        array = measured[sel];
    else if (sel >= 2 && sel < 7)
        array = cal_data[sel - 2];
    else
        goto usage;
    for (i = 0; i < sweep_points; i++) shell_printf("%f %f\r\n", array[i][0], array[i][1]);
    return;
usage:
#endif
    shell_printf("usage: data [array]\r\n");
}


#define DELAY_CHANNEL_CHANGE 2

// main loop for measurement
bool sweep(bool break_on_operation)
{
    shell_printf("AYE\r\n");
    return;    
#if 0    
    int i, delay;
    // blink LED while scanning
    palClearPad(GPIOC, GPIOC_LED);
    // Power stabilization after LED off, also align timings on i == 0
    for (i = 0; i < sweep_points; i++)    // 5300
    {
        if (frequencies[i] == 0) break;
        delay = set_frequency(frequencies[i]);  // 700
        tlv320aic3204_select(0);                // 60 CH0:REFLECT, reset and begin measure
        //dsp_start(delay + ((i == 0) ? 1 : 0));  // 1900
        //================================================
        // Place some code thats need execute while delay
        //================================================
        //dsp_wait();
        // calculate reflection coefficient
        //(*sample_func)(measured[0][i]);  // 60

        tlv320aic3204_select(1);          // 60 CH1:TRANSMISSION, reset and begin measure
        //dsp_start(DELAY_CHANNEL_CHANGE);  // 1700
        //================================================
        // Place some code thats need execute while delay
        //================================================
        //dsp_wait();
        // calculate transmission coefficient
        //(*sample_func)(measured[1][i]);  // 60
        // ======== 170 ===========
        //if (cal_status & CALSTAT_APPLY) apply_error_term_at(i);

        //if (electrical_delay != 0) apply_edelay_at(i);

        // back to toplevel to handle ui operation
        if (operation_requested && break_on_operation) return false;
    }
    // blink LED while scanning
    palSetPad(GPIOC, GPIOC_LED);
#endif    
    return true;
}

VNA_SHELL_FUNCTION(cmd_scan)
{
    shell_printf("AYE\r\n");
    return;

#if 0    
    uint32_t start, stop;
    int16_t points = sweep_points;
    int i;
    if (argc < 2 || argc > 4)
    {
        shell_printf("usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]\r\n");
        return;
    }

    start = my_atoui(argv[0]);
    stop = my_atoui(argv[1]);
    if (start == 0 || stop == 0 || start > stop)
    {
        shell_printf("frequency range is invalid\r\n");
        return;
    }
    if (argc >= 3)
    {
        points = my_atoi(argv[2]);
        if (points <= 0 || points > POINTS_COUNT)
        {
            shell_printf("sweep points exceeds range " define_to_STR(POINTS_COUNT) "\r\n");
            return;
        }
    }

    set_frequencies(start, stop, points);
    if (cal_auto_interpolate && (cal_status & CALSTAT_APPLY)) cal_interpolate(lastsaveid);
    pause_sweep();
    sweep(false);
    // Output data after if set (faster data recive)
    if (argc == 4)
    {
        uint16_t mask = my_atoui(argv[3]);
        if (mask)
        {
            for (i = 0; i < points; i++)
            {
                if (mask & 1) shell_printf("%u ", frequencies[i]);
                if (mask & 2) shell_printf("%f %f ", measured[0][i][0], measured[0][i][1]);
                if (mask & 4) shell_printf("%f %f ", measured[1][i][0], measured[1][i][1]);
                shell_printf("\r\n");
            }
        }
    }
#endif    
}

static void set_frequencies(uint32_t start, uint32_t stop, uint16_t points)
{
#if 0    
    uint32_t i;
    uint32_t step = (points - 1);
    uint32_t span = stop - start;
    uint32_t delta = span / step;
    uint32_t error = span % step;
    uint32_t f = start, df = step >> 1;
    for (i = 0; i <= step; i++, f += delta)
    {
        frequencies[i] = f;
        df += error;
        if (df >= step)
        {
            f++;
            df -= step;
        }
    }
    // disable at out of sweep range
    for (; i < POINTS_COUNT; i++) frequencies[i] = 0;
#endif
}

static void update_frequencies(void)
{
#if 0    
    uint32_t start, stop;
    start = get_sweep_frequency(ST_START);
    stop = get_sweep_frequency(ST_STOP);

    set_frequencies(start, stop, sweep_points);
    // operation_requested|= OP_FREQCHANGE;

    //update_marker_index();

    // set grid layout
    //update_grid();
#endif    
}

VNA_SHELL_FUNCTION(cmd_gain)
{
    int rvalue;
    int lvalue = 0;
    if (argc != 1 && argc != 2)
    {
        shell_printf("usage: gain {lgain(0-95)} [rgain(0-95)]\r\n");
        return;
    }
    rvalue = my_atoi(argv[0]);
    if (argc == 2) lvalue = my_atoi(argv[1]);
    tlv320aic3204_set_gain(lvalue, rvalue);
}

VNA_SHELL_FUNCTION(cmd_port)
{
    int port;
    if (argc != 1)
    {
        shell_printf("usage: port {0:TX 1:RX}\r\n");
        return;
    }
    port = my_atoi(argv[0]);
    tlv320aic3204_select(port);
}

VNA_SHELL_FUNCTION(cmd_bandwidth)
{
#if 0    
    if (argc != 1) goto usage;

    static const char bw_choice[] = "1000|300|100|30|10";
    int i = get_str_index(argv[0], bw_choice);
    if (i < 0) goto usage;

    bandwidth = i;
    return;
usage:
    shell_printf("usage: bandwidth {%s}\r\n", bw_choice);    
#endif 
    shell_printf("usage: bandwidth\r\n");
}


#ifndef VERSION
#define VERSION "unknown"
#endif

const char NANOVNA_VERSION[] = VERSION;

VNA_SHELL_FUNCTION(cmd_version)
{
    (void)argc;
    (void)argv;
    shell_printf("%s\r\n", NANOVNA_VERSION);
}

#ifdef ENABLE_INFO_COMMAND
VNA_SHELL_FUNCTION(cmd_info)
{
    (void)argc;
    (void)argv;
    int i = 0;
    while (info_about[i]) shell_printf("%s\r\n", info_about[i++]);
}
#endif

//=============================================================================
VNA_SHELL_FUNCTION(cmd_help);

#pragma pack(push, 2)
typedef struct
{
    const char *sc_name;
    vna_shellcmd_t sc_function;
    uint16_t flags;
} VNAShellCommand;
#pragma pack(pop)

// Some commands can executed only in sweep thread, not in main cycle
#define CMD_WAIT_MUTEX 1
static const VNAShellCommand commands[] = {{"version", cmd_version, 0},
    {"reset", cmd_reset, 0},
    {"freq", cmd_freq, CMD_WAIT_MUTEX},
    {"offset", cmd_offset, 0},
    {"dac", cmd_dac, 0},
    {"data", cmd_data, CMD_WAIT_MUTEX},
    {"bandwidth", cmd_bandwidth, 0},
    {"port", cmd_port, 0},
    {"gain", cmd_gain, 0},
    {"power", cmd_power, 0},
    {"scan", cmd_scan, CMD_WAIT_MUTEX},
    {"pause", cmd_pause, 0},
    {"resume", cmd_resume, 0},
    {"threshold", cmd_threshold, 0},
    {"help", cmd_help, 0},
#ifdef ENABLE_INFO_COMMAND
    {"info", cmd_info, 0},
#endif
    {NULL, NULL, 0}
};

VNA_SHELL_FUNCTION(cmd_help)
{
    (void)argc;
    (void)argv;
    const VNAShellCommand *scp = commands;
    shell_printf("Commands:");
    while (scp->sc_name != NULL)
    {
        shell_printf(" %s", scp->sc_name);
        scp++;
    }
    shell_printf(VNA_SHELL_NEWLINE_STR);
    return;
}

/*
 * VNA shell functions
 */

//
// Read command line from shell_stream
//
static int VNAShell_readLine(char *line, int max_size)
{
    // Read line from input stream
    uint8_t c;
    char *ptr = line;
    while (1)
    {
        // Return 0 only if stream not active
        if (streamRead(shell_stream, &c, 1) == 0) return 0;
        // Backspace or Delete
        if (c == 8 || c == 0x7f)
        {
            if (ptr != line)
            {
                static const char backspace[] = {0x08, 0x20, 0x08, 0x00};
                shell_printf(backspace);
                ptr--;
            }
            continue;
        }
        // New line (Enter)
        if (c == '\r')
        {
            shell_printf(VNA_SHELL_NEWLINE_STR);
            *ptr = 0;
            return 1;
        }
        // Others (skip)
        if (c < 0x20) continue;
        // Store
        if (ptr < line + max_size - 1)
        {
            streamPut(shell_stream, c);  // Echo
            *ptr++ = (char)c;
        }
    }
    return 0;
}

//
// Parse and run command line
//
static void VNAShell_executeLine(char *line)
{
    // Parse and execute line
    char *lp = line, *ep;
    shell_nargs = 0;
    while (*lp != 0)
    {
        // Skipping white space and tabs at string begin.
        while (*lp == ' ' || *lp == '\t') lp++;
        // If an argument starts with a double quote then its delimiter is another quote, else
        // delimiter is white space.
        ep = (*lp == '"') ? strpbrk(++lp, "\"") : strpbrk(lp, " \t");
        // Store in args string
        shell_args[shell_nargs++] = lp;
        // Stop, end of input string
        if ((lp = ep) == NULL) break;
        // Argument limits check
        if (shell_nargs > VNA_SHELL_MAX_ARGUMENTS)
        {
            shell_printf("UGH!\n\r");
            //shell_printf("too many arguments, max " define_to_STR(
            //                 VNA_SHELL_MAX_ARGUMENTS) "" VNA_SHELL_NEWLINE_STR);
            return;
        }
        // Set zero at the end of string and continue check
        *lp++ = 0;
    }
    if (shell_nargs == 0) return;
    // Execute line
    const VNAShellCommand *scp;
    for (scp = commands; scp->sc_name != NULL; scp++)
    {
        if (strcmp(scp->sc_name, shell_args[0]) == 0)
        {
            if (scp->flags & CMD_WAIT_MUTEX)
            {
                shell_function = scp->sc_function;
                // Wait execute command in sweep thread
                do
                {
                    osalThreadSleepMilliseconds(100);
                }
                while (shell_function);
            }
            else
            {
                scp->sc_function(shell_nargs - 1, &shell_args[1]);
            }
            return;
        }
    }
    shell_printf("%s?" VNA_SHELL_NEWLINE_STR, shell_args[0]);
}

#ifdef VNA_SHELL_THREAD
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */ 442);
THD_FUNCTION(myshellThread, p)
{
    (void)p;
    chRegSetThreadName("shell");
    shell_printf(VNA_SHELL_NEWLINE_STR "NanoVNA Shell" VNA_SHELL_NEWLINE_STR);
    while (true)
    {
        shell_printf(VNA_SHELL_PROMPT_STR);
        if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
            VNAShell_executeLine(shell_line);
        else  // Putting a delay in order to avoid an endless loop trying to read an
            // unavailable stream.
            osalThreadSleepMilliseconds(100);
    }
}
#endif

// I2C clock bus setting: depend from STM32_I2C1SW in mcuconf.h
static const I2CConfig i2ccfg =
{
    .timingr =  // TIMINGR register initialization. (use I2C timing configuration tool for
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
    .cr1 = 0,  // CR1 register initialization.
    .cr2 = 0   // CR2 register initialization.
};

static DACConfig dac1cfg1 =
{
    // init:         2047U,
    init : 1922U,
datamode :
    DAC_DHRM_12BIT_RIGHT
};

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
    chThdSleepMilliseconds(100);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

#if 0
    /*
     * SPI LCD Initialize
     */
    ili9341_init();

    /* restore config */
    config_recall();
    /* restore frequencies and calibration 0 slot properties from flash memory */
    caldata_recall(0);

    dac1cfg1.init = config.dac_value;
    /*
     * Starting DAC1 driver, setting up the output pin as analog as suggested
     * by the Reference Manual.
     */
    dacStart(&DACD2, &dac1cfg1);

    /* initial frequencies */
    update_frequencies();
#endif

    /*
     * I2S Initialize
     */
    tlv320aic3204_init();
    i2sInit();
    i2sObjectInit(&I2SD2);
    i2sStart(&I2SD2, &i2sconfig);
    i2sStartExchange(&I2SD2);

#if 0
    ui_init();
    // Initialize graph plotting
    plot_init();
    redraw_frame();
#endif

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO - 1, Thread1, NULL);

    while (1)
    {
        if (SDU1.config->usbp->state == USB_ACTIVE)
        {
#ifdef VNA_SHELL_THREAD
#if CH_CFG_USE_WAITEXIT == FALSE
#error "VNA_SHELL_THREAD use chThdWait, need enable CH_CFG_USE_WAITEXIT in chconf.h"
#endif
            thread_t *shelltp = chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO + 1,
                                                  myshellThread, NULL);
            chThdWait(shelltp);
#else
            shell_printf(VNA_SHELL_NEWLINE_STR "NanoVNA Shell" VNA_SHELL_NEWLINE_STR);
            do
            {
                shell_printf(VNA_SHELL_PROMPT_STR);
                if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
                    VNAShell_executeLine(shell_line);
                else
                    chThdSleepMilliseconds(200);
            }
            while (SDU1.config->usbp->state == USB_ACTIVE);
#endif
        }
        chThdSleepMilliseconds(1000);
    }
}

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

