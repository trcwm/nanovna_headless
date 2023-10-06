#pragma once
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DSP_IDLE,   // DSP does nothing.
    DSP_ACCUMULATE, // DSP accumulates ref and ch.
    DSP_DONE,   // DSP is done.
    DSP_RAW     // DSP stores ch channel in m_rawBuffer
} DSPState_t;

typedef struct DSPAccumulators_t
{
    int32_t    m_refI;
    int32_t    m_refQ;
    int32_t    m_chI;
    int32_t    m_chQ;
} DSPAccumulators_t;

typedef struct DSPContext_t
{
    DSPState_t m_dspState;
    DSPAccumulators_t m_accus;
    int32_t    m_blockCounter;
    int16_t    m_rawBuffer[48];
} DSPContext_t;

void dspReset(void);
bool dspIsDone(void);
bool dspIsIdle(void);
void dspStart(uint32_t blockCount);
void dspRaw(void);
void dspWaitDone(void);
void dspCallback(const uint32_t *ptr, size_t n);
const DSPAccumulators_t* dspGetAccumulatorPtr(void);
const int16_t* dspGetRawBufferPtr(void);
