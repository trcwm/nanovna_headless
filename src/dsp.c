#include "dsp.h"
#include "hal.h"

static const int16_t sincos_tbl[48][2] = {
  { 10533,  31029 }, { 27246,  18205 }, { 32698,  -2143 }, { 24636, -21605 },
  {  6393, -32138 }, {-14493, -29389 }, {-29389, -14493 }, {-32138,   6393 },
  {-21605,  24636 }, { -2143,  32698 }, { 18205,  27246 }, { 31029,  10533 },
  { 31029, -10533 }, { 18205, -27246 }, { -2143, -32698 }, {-21605, -24636 },
  {-32138,  -6393 }, {-29389,  14493 }, {-14493,  29389 }, {  6393,  32138 },
  { 24636,  21605 }, { 32698,   2143 }, { 27246, -18205 }, { 10533, -31029 },
  {-10533, -31029 }, {-27246, -18205 }, {-32698,   2143 }, {-24636,  21605 },
  { -6393,  32138 }, { 14493,  29389 }, { 29389,  14493 }, { 32138,  -6393 },
  { 21605, -24636 }, { 2143,  -32698 }, {-18205, -27246 }, {-31029, -10533 },
  {-31029,  10533 }, {-18205,  27246 }, {  2143,  32698 }, { 21605,  24636 },
  { 32138,   6393 }, { 29389, -14493 }, { 14493, -29389 }, { -6393, -32138 },
  {-24636, -21605 }, {-32698,  -2143 }, {-27246,  18205 }, {-10533,  31029 }
};

static DSPContext_t gs_dspContext;

void dspReset()
{
    gs_dspContext.m_blockCounter = 0;
    gs_dspContext.m_dspState = DSP_IDLE;
}

bool dspIsDone()
{
    return gs_dspContext.m_dspState == DSP_DONE;
}

bool dspIsIdle()
{
    return gs_dspContext.m_dspState == DSP_IDLE;
}

void dspStart(uint32_t blockCount)
{
    gs_dspContext.m_blockCounter = blockCount;
    gs_dspContext.m_accus.m_chI = 0;
    gs_dspContext.m_accus.m_chQ = 0;
    gs_dspContext.m_accus.m_refI = 0;
    gs_dspContext.m_accus.m_refQ = 0;
    gs_dspContext.m_dspState = DSP_ACCUMULATE;
}

void dspRaw()
{
    gs_dspContext.m_dspState = DSP_RAW;
}

void dspCallback(const uint32_t *ptr, size_t n)
{
    const size_t frames = n / 2;
    
    switch(gs_dspContext.m_dspState)
    {
    case DSP_IDLE:
        return;
    case DSP_ACCUMULATE:
        {
            for(size_t i=0; i<frames; i++)
            {
                const uint32_t src = *ptr++;
                const int16_t  ref = src & 0xFFFF;
                const int16_t  ch  = (src>>16) & 0xFFFF;
                
                const int32_t s = sincos_tbl[i][0];
                const int32_t c = sincos_tbl[i][1];
                gs_dspContext.m_accus.m_chI += ch * s / 16;
                gs_dspContext.m_accus.m_chQ += ch * c / 16;
                gs_dspContext.m_accus.m_refI += ref * s / 16;
                gs_dspContext.m_accus.m_refQ += ref * c / 16;
            }

            gs_dspContext.m_blockCounter--;
            if (gs_dspContext.m_blockCounter <= 0)
            {
                gs_dspContext.m_dspState = DSP_DONE;
            }
        }
        break;
    case DSP_DONE:
        break;
    case DSP_RAW:
        {            
            for(size_t i=0; i<frames; i++)
            {                
                const uint32_t src = *ptr++;
                const int16_t  ch  = (src>>16) & 0xFFFF;
                gs_dspContext.m_rawBuffer[i] = ch;
            }
            gs_dspContext.m_dspState = DSP_DONE;
        }
        break;
    default:
        gs_dspContext.m_dspState = DSP_IDLE;
        break;
    }
}

void dspWaitDone()
{
    while(gs_dspContext.m_dspState != DSP_DONE)
    {
        __WFI();
    }
}

const int16_t* dspGetRawBufferPtr()
{
    return gs_dspContext.m_rawBuffer;
}

const DSPAccumulators_t* dspGetAccumulatorPtr()
{
    return &gs_dspContext.m_accus;
}
