#include "ch.h"
#include "hal.h"
#include "cobs.h"
#include "si5351.h"
#include "tlv320aic3204.h"
#include "usbcfg.h"
#include "cmdhandler.h"
#include "dsp.h"

void* memcpy(void *dst, const void *src, size_t bytes)
{
    const uint8_t *s = src;
    uint8_t *d = dst;
    for(size_t i=0; i<bytes; i++)
    {
        *d++ = *s++;
    }
    return dst;
}

extern int32_t g_callbackCount;

// If need run shell as thread (use more amount of memory fore stack), after
// enable this need reduce spi_buffer size, by default shell run in main thread
// #define VNA_SHELL_THREAD

BaseSequentialStream *gs_usb_stream = (BaseSequentialStream *)&SDU1;

// ******************************************************************************************
//   Command buffer, data from PC
// ******************************************************************************************

// Command buffer max command size
#define CMD_MAX_LENGTH 32

typedef struct CmdBuffer
{
    uint8_t m_buffer[CMD_MAX_LENGTH];
    uint8_t m_len;
} CmdBuffer_t;

CmdBuffer_t gs_cmdbuffer;

// ******************************************************************************************
//   Background thread for measurements
// ******************************************************************************************

void cmdBackgroundThread()
{
    while(true)
    {
        __WFI();
        //palClearPad(GPIOC, GPIOC_LED);

        // send data to PC
        //const size_t dataLen = sizeof(int32_t)*4;
        //uint8_t buffer[dataLen + 2];
        //uint32_t len = cobsEncode(gs_measurements, dataLen, buffer);
        //resultPacket[len++] = 0;
        //streamWrite(gs_usb_stream, resultPacket, len);
        //palSetPad(GPIOC, GPIOC_LED);
    }
}

void sendErrorPacket(const uint8_t commandCode)
{
    const uint8_t errorPacket[2] = {0xFF, commandCode};
    uint8_t resultPacket[4];
    uint8_t len = cobsEncode(errorPacket, 2, resultPacket);
    resultPacket[len++] = 0; // add packet terminator
    streamWrite(gs_usb_stream, resultPacket, len);
}

void submitCmdByte(uint8_t c)
{
    if (gs_cmdbuffer.m_len < CMD_MAX_LENGTH)
    {
        gs_cmdbuffer.m_buffer[gs_cmdbuffer.m_len] = c;
        
        if (c == 0) /* end of COBS buffer */
        {
            uint8_t cmddata[CMD_MAX_LENGTH+1];
            uint8_t datasize = cobsDecode(gs_cmdbuffer.m_buffer, gs_cmdbuffer.m_len, cmddata);
            gs_cmdbuffer.m_len = 0;
            bool result = executeCmd(cmddata, datasize);
            
            if (!result)
            {
                sendErrorPacket(cmddata[0]);
            }
        }
        else
        {
            gs_cmdbuffer.m_len++;
        }
    }
    else
    {
        // buffer overrun!
        gs_cmdbuffer.m_len = 0;
    }
}

bool executeCmd(const uint8_t *data, uint8_t datasize)
{
    if (datasize == 0) return false;

    uint8_t len = 0;
    uint8_t dataBuffer[48*2+2];
    uint8_t resultBuffer[sizeof(dataBuffer) + 2];

    switch(data[0])
    {
    case 0x00:  /* get number of callbacks */
        dataBuffer[len++] = 0x00; // the command
        memcpy(dataBuffer+1, &g_callbackCount, sizeof(g_callbackCount));
        len += sizeof(g_callbackCount);
        len = cobsEncode(dataBuffer, len, resultBuffer);
        resultBuffer[len++] = 0;
        streamWrite(gs_usb_stream, resultBuffer, len);
        return true;
    case 0x01:  /* get RAW buffer*/
        if (!dspIsDone() && !dspIsIdle())
        {
            return false;
        }
        dspRaw();
        dataBuffer[0] = 0x01;
        dspWaitDone();  // FIXME: this should have a time-out..
        memcpy(dataBuffer+1, (const uint8_t*)dspGetRawBufferPtr(), 48*2);
        len = cobsEncode(dataBuffer, 48*2+1, resultBuffer);
        resultBuffer[len++] = 0;
        streamWrite(gs_usb_stream, resultBuffer, len);
        return true;
    case 0x02:  /* get accumulated data*/
        if (!dspIsDone() && !dspIsIdle())
        {
            return false;
        }
        dspStart(1);
        dataBuffer[0] = 0x02;
        dspWaitDone();  // FIXME: this should have a time-out..
        memcpy(dataBuffer+1, (const uint8_t*)dspGetAccumulatorPtr(), sizeof(DSPAccumulators_t));
        len = cobsEncode(dataBuffer, sizeof(DSPAccumulators_t)+1, resultBuffer);
        resultBuffer[len++] = 0;
        streamWrite(gs_usb_stream, resultBuffer, len);
        return true;  
    case 0x03:  /* set frequency */
        if (datasize < 5) return false;
        {
            dataBuffer[0] = 0x03;
            // unaligned problems?!
            uint32_t freq;
            uint8_t* freqPtr = (uint8_t*)&freq;
            freqPtr[0] = data[1];
            freqPtr[1] = data[2];
            freqPtr[2] = data[3];
            freqPtr[3] = data[4];
            si5351_set_frequency(freq, SI5351_CLK_DRIVE_STRENGTH_2MA);
            len = cobsEncode(dataBuffer, 1, resultBuffer);
            resultBuffer[len++] = 0;
            streamWrite(gs_usb_stream, resultBuffer, len);
        }
        return true;
    case 0x04:  /* get frequency */        
        {
            dataBuffer[0] = 0x04;
            uint32_t freq = si5351_get_frequency();
            memcpy(dataBuffer+1, &freq, sizeof(freq));
            len = cobsEncode(dataBuffer, 1 + sizeof(freq), resultBuffer);
            resultBuffer[len++] = 0;
            streamWrite(gs_usb_stream, resultBuffer, len);
        }
        return true;
    case 0x05:  /* set measurement channel */
        {
            // FIXME: we need to wait a certain amount after 
            // setting the channel to avoid glitches
            // -> 2 I2S blocks
            switch(data[1])
            {
            case 0:
                tlv320aic3204_select(0); // select REFLECT channel
                break;
            case 1:
                tlv320aic3204_select(1); // select TRANSMISSION channel
                break;
            default:
                return false;
            }

            dataBuffer[0] = 0x05;
            len = cobsEncode(dataBuffer, 1, resultBuffer);
            resultBuffer[len++] = 0;
            streamWrite(gs_usb_stream, resultBuffer, len);
        }
        return true;
    case 0x06: /* measure list of frequencies, max 6 */
        {
            dataBuffer[0] = 0x06;
            uint32_t freq;
            uint8_t numFrequencies = data[1];

            if ((numFrequencies == 0) || (numFrequencies > 6))
            {
                return false;
            }

            const uint8_t *freqDataPtr = data + 2;
            uint8_t *outDataPtr = dataBuffer+1;
            uint8_t outDataSize = 1;
            for(int i=0; i<numFrequencies; i++)
            {
                // take care of unaligned data                
                memcpy(&freq, freqDataPtr+sizeof(freq)*i, sizeof(freq));
                
                // delay is approximately in ms.
                int delay = si5351_set_frequency(freq, SI5351_CLK_DRIVE_STRENGTH_2MA) + 1;
                chThdSleepMilliseconds(delay);
                
                dspStart(1);
                dspWaitDone();  // FIXME: this should have a time-out..

                memcpy(outDataPtr, dspGetAccumulatorPtr(), sizeof(DSPAccumulators_t));
                outDataPtr  += sizeof(DSPAccumulators_t);
                outDataSize += sizeof(DSPAccumulators_t);
            }

            len = cobsEncode(dataBuffer, outDataSize, resultBuffer);
            resultBuffer[len++] = 0;
            streamWrite(gs_usb_stream, resultBuffer, len);
        }
        return true;
    }
    
    return false;
}

#if 0

//
// Parse and run command line
//
static void VNAShell_executeCommand(char *line)
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
    {

    }
}

int set_frequency(uint32_t freq)
{
    int8_t ds = drive_strength;
    if (ds == DRIVE_STRENGTH_AUTO)
    {
        //FIXME:
        //ds = freq > FREQ_HARMONICS ? SI5351_CLK_DRIVE_STRENGTH_8MA
        //     : SI5351_CLK_DRIVE_STRENGTH_2MA;

        ds = SI5351_CLK_DRIVE_STRENGTH_2MA;
    }

    int delay = si5351_set_frequency(freq, ds);
    return delay;
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

#define DELAY_CHANNEL_CHANGE 2

// main loop for measurement
bool sweep(bool break_on_operation)
{
    //shell_printf("AYE\r\n");
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
#endif 

