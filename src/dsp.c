#include "dsp.h"
#include "hal.h"

static const int16_t gs_nco_tbl[257][2] = {
    { 30000, 0 }, 
    { 29990, -736 }, 
    { 29963, -1472 }, 
    { 29918, -2206 }, 
    { 29855, -2940 }, 
    { 29774, -3672 }, 
    { 29675, -4401 }, 
    { 29558, -5128 }, 
    { 29423, -5852 }, 
    { 29271, -6573 }, 
    { 29100, -7289 }, 
    { 28913, -8001 }, 
    { 28708, -8708 }, 
    { 28485, -9410 }, 
    { 28246, -10106 }, 
    { 27989, -10796 }, 
    { 27716, -11480 }, 
    { 27426, -12157 }, 
    { 27119, -12826 }, 
    { 26796, -13488 }, 
    { 26457, -14141 }, 
    { 26102, -14786 }, 
    { 25731, -15423 }, 
    { 25345, -16049 }, 
    { 24944, -16667 }, 
    { 24527, -17274 }, 
    { 24096, -17870 }, 
    { 23650, -18456 }, 
    { 23190, -19031 }, 
    { 22716, -19595 }, 
    { 22228, -20146 }, 
    { 21727, -20686 }, 
    { 21213, -21213 }, 
    { 20686, -21727 }, 
    { 20146, -22228 }, 
    { 19595, -22716 }, 
    { 19031, -23190 }, 
    { 18456, -23650 }, 
    { 17870, -24096 }, 
    { 17274, -24527 }, 
    { 16667, -24944 }, 
    { 16049, -25345 }, 
    { 15423, -25731 }, 
    { 14786, -26102 }, 
    { 14141, -26457 }, 
    { 13488, -26796 }, 
    { 12826, -27119 }, 
    { 12157, -27426 }, 
    { 11480, -27716 }, 
    { 10796, -27989 }, 
    { 10106, -28246 }, 
    { 9410, -28485 }, 
    { 8708, -28708 }, 
    { 8001, -28913 }, 
    { 7289, -29100 }, 
    { 6573, -29271 }, 
    { 5852, -29423 }, 
    { 5128, -29558 }, 
    { 4401, -29675 }, 
    { 3672, -29774 }, 
    { 2940, -29855 }, 
    { 2206, -29918 }, 
    { 1472, -29963 }, 
    { 736, -29990 }, 
    { 0, -29999 }, 
    { -736, -29990 }, 
    { -1472, -29963 }, 
    { -2206, -29918 }, 
    { -2940, -29855 }, 
    { -3672, -29774 }, 
    { -4401, -29675 }, 
    { -5128, -29558 }, 
    { -5852, -29423 }, 
    { -6573, -29271 }, 
    { -7289, -29100 }, 
    { -8001, -28913 }, 
    { -8708, -28708 }, 
    { -9410, -28485 }, 
    { -10106, -28246 }, 
    { -10796, -27989 }, 
    { -11480, -27716 }, 
    { -12157, -27426 }, 
    { -12826, -27119 }, 
    { -13488, -26796 }, 
    { -14141, -26457 }, 
    { -14786, -26102 }, 
    { -15423, -25731 }, 
    { -16049, -25345 }, 
    { -16667, -24944 }, 
    { -17274, -24527 }, 
    { -17870, -24096 }, 
    { -18456, -23650 }, 
    { -19031, -23190 }, 
    { -19595, -22716 }, 
    { -20146, -22228 }, 
    { -20686, -21727 }, 
    { -21213, -21213 }, 
    { -21727, -20686 }, 
    { -22228, -20146 }, 
    { -22716, -19595 }, 
    { -23190, -19031 }, 
    { -23650, -18456 }, 
    { -24096, -17870 }, 
    { -24527, -17274 }, 
    { -24944, -16667 }, 
    { -25345, -16049 }, 
    { -25731, -15423 }, 
    { -26102, -14786 }, 
    { -26457, -14141 }, 
    { -26796, -13488 }, 
    { -27119, -12826 }, 
    { -27426, -12157 }, 
    { -27716, -11480 }, 
    { -27989, -10796 }, 
    { -28246, -10106 }, 
    { -28485, -9410 }, 
    { -28708, -8708 }, 
    { -28913, -8001 }, 
    { -29100, -7289 }, 
    { -29271, -6573 }, 
    { -29423, -5852 }, 
    { -29558, -5128 }, 
    { -29675, -4401 }, 
    { -29774, -3672 }, 
    { -29855, -2940 }, 
    { -29918, -2206 }, 
    { -29963, -1472 }, 
    { -29990, -736 }, 
    { -29999, 0 }, 
    { -29990, 736 }, 
    { -29963, 1472 }, 
    { -29918, 2206 }, 
    { -29855, 2940 }, 
    { -29774, 3672 }, 
    { -29675, 4401 }, 
    { -29558, 5128 }, 
    { -29423, 5852 }, 
    { -29271, 6573 }, 
    { -29100, 7289 }, 
    { -28913, 8001 }, 
    { -28708, 8708 }, 
    { -28485, 9410 }, 
    { -28246, 10106 }, 
    { -27989, 10796 }, 
    { -27716, 11480 }, 
    { -27426, 12157 }, 
    { -27119, 12826 }, 
    { -26796, 13488 }, 
    { -26457, 14141 }, 
    { -26102, 14786 }, 
    { -25731, 15423 }, 
    { -25345, 16049 }, 
    { -24944, 16667 }, 
    { -24527, 17274 }, 
    { -24096, 17870 }, 
    { -23650, 18456 }, 
    { -23190, 19031 }, 
    { -22716, 19595 }, 
    { -22228, 20146 }, 
    { -21727, 20686 }, 
    { -21213, 21213 }, 
    { -20686, 21727 }, 
    { -20146, 22228 }, 
    { -19595, 22716 }, 
    { -19031, 23190 }, 
    { -18456, 23650 }, 
    { -17870, 24096 }, 
    { -17274, 24527 }, 
    { -16667, 24944 }, 
    { -16049, 25345 }, 
    { -15423, 25731 }, 
    { -14786, 26102 }, 
    { -14141, 26457 }, 
    { -13488, 26796 }, 
    { -12826, 27119 }, 
    { -12157, 27426 }, 
    { -11480, 27716 }, 
    { -10796, 27989 }, 
    { -10106, 28246 }, 
    { -9410, 28485 }, 
    { -8708, 28708 }, 
    { -8001, 28913 }, 
    { -7289, 29100 }, 
    { -6573, 29271 }, 
    { -5852, 29423 }, 
    { -5128, 29558 }, 
    { -4401, 29675 }, 
    { -3672, 29774 }, 
    { -2940, 29855 }, 
    { -2206, 29918 }, 
    { -1472, 29963 }, 
    { -736, 29990 }, 
    { 0, 29999 }, 
    { 736, 29990 }, 
    { 1472, 29963 }, 
    { 2206, 29918 }, 
    { 2940, 29855 }, 
    { 3672, 29774 }, 
    { 4401, 29675 }, 
    { 5128, 29558 }, 
    { 5852, 29423 }, 
    { 6573, 29271 }, 
    { 7289, 29100 }, 
    { 8001, 28913 }, 
    { 8708, 28708 }, 
    { 9410, 28485 }, 
    { 10106, 28246 }, 
    { 10796, 27989 }, 
    { 11480, 27716 }, 
    { 12157, 27426 }, 
    { 12826, 27119 }, 
    { 13488, 26796 }, 
    { 14141, 26457 }, 
    { 14786, 26102 }, 
    { 15423, 25731 }, 
    { 16049, 25345 }, 
    { 16667, 24944 }, 
    { 17274, 24527 }, 
    { 17870, 24096 }, 
    { 18456, 23650 }, 
    { 19031, 23190 }, 
    { 19595, 22716 }, 
    { 20146, 22228 }, 
    { 20686, 21727 }, 
    { 21213, 21213 }, 
    { 21727, 20686 }, 
    { 22228, 20146 }, 
    { 22716, 19595 }, 
    { 23190, 19031 }, 
    { 23650, 18456 }, 
    { 24096, 17870 }, 
    { 24527, 17274 }, 
    { 24944, 16667 }, 
    { 25345, 16049 }, 
    { 25731, 15423 }, 
    { 26102, 14786 }, 
    { 26457, 14141 }, 
    { 26796, 13488 }, 
    { 27119, 12826 }, 
    { 27426, 12157 }, 
    { 27716, 11480 }, 
    { 27989, 10796 }, 
    { 28246, 10106 }, 
    { 28485, 9410 }, 
    { 28708, 8708 }, 
    { 28913, 8001 }, 
    { 29100, 7289 }, 
    { 29271, 6573 }, 
    { 29423, 5852 }, 
    { 29558, 5128 }, 
    { 29675, 4401 }, 
    { 29774, 3672 }, 
    { 29855, 2940 }, 
    { 29918, 2206 }, 
    { 29963, 1472 }, 
    { 29990, 736 },
    { 30000, 0 }
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
    gs_dspContext.m_ncoAccu = 0;
    gs_dspContext.m_ncoIncr = (uint32_t)((5000.0f / 48000.0f) * 65536.0f * 256.0f); // 5 kHz increment

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

void calcNCO(int16_t *sinval, int16_t *cosval)
{
    gs_dspContext.m_ncoAccu = (gs_dspContext.m_ncoAccu + gs_dspContext.m_ncoIncr) & 0x00FFFFFF;
    uint32_t offset = gs_dspContext.m_ncoAccu >> 16;
    uint32_t frac   = (gs_dspContext.m_ncoAccu >> 8) & 0xFF;    // 8 bit fractional offset

    const int16_t sv1 = gs_nco_tbl[offset][1];
    const int16_t sv2 = gs_nco_tbl[offset+1][1];
    const int16_t cv1 = gs_nco_tbl[offset][0];
    const int16_t cv2 = gs_nco_tbl[offset+1][0];

    *sinval = sv1 + (int16_t)((frac*(sv2 - sv1)) >> 8);
    *cosval = cv1 + (int16_t)((frac*(cv2 - cv1)) >> 8);
}

void dspCallback(const uint32_t *ptr, size_t n)
{
    static_assert(sizeof(gs_dspContext.m_accus) == 16);

    const size_t frames = n / 2;
    
    switch(gs_dspContext.m_dspState)
    {
    case DSP_IDLE:
        return;
    case DSP_ACCUMULATE:
        {
            int16_t s;
            int16_t c;
            for(size_t i=0; i<frames; i++)
            {
                const uint32_t src = *ptr++;
                const int16_t  ref = src & 0xFFFF;
                const int16_t  ch  = (src>>16) & 0xFFFF;

                calcNCO(&s, &c);

                gs_dspContext.m_accus.m_chI  += (ch * s) / 1024;
                gs_dspContext.m_accus.m_chQ  += (ch * c) / 1024;
                gs_dspContext.m_accus.m_refI += (ref * s) / 1024;
                gs_dspContext.m_accus.m_refQ += (ref * c) / 1024;
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
