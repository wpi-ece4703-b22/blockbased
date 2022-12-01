#include "xlaudio.h"
#include "xlaudio_armdsp.h"

#define BLOCKSIZE 8
#define NUMTAPS 58

q15_t taps[NUMTAPS + BLOCKSIZE - 1];
arm_fir_instance_q15 F;

typedef q15_t int16_T;

// from matlab
const int BL = 58;
const int16_T B[58] = {
     -232,    -47,    109,    319,    462,    430,    205,   -104,   -314,
     -278,     -1,    343,    504,    323,   -133,   -580,   -681,   -285,
      426,    990,    938,    132,  -1056,  -1847,  -1447,    478,   3523,
     6642,   8610,   8610,   6642,   3523,    478,  -1447,  -1847,  -1056,
      132,    938,    990,    426,   -285,   -681,   -580,   -133,    323,
      504,    343,     -1,   -278,   -314,   -104,    205,    430,    462,
      319,    109,    -47,   -232
};

void processBuffer(uint16_t x[BLOCKSIZE], uint16_t y[BLOCKSIZE]) {
    q15_t xq[BLOCKSIZE], yq[BLOCKSIZE];
    xlaudio_adc14_to_q15_vec(x, xq, BLOCKSIZE);
    arm_fir_q15(&F, xq, yq, BLOCKSIZE);
    xlaudio_q15_to_dac14_vec(yq, y, BLOCKSIZE);
}

uint16_t processSample(uint16_t x) {
      q15_t v = xlaudio_adc14_to_q15(x);
      taps[0] = v;
      int i;
      q15_t q = 0;
      for (i=0; i<NUMTAPS; i++)
          q = q + (taps[i] * B[i] >> 15);
      for (i=NUMTAPS-1; i>0; i--)
          taps[i] = taps[i-1];
      return xlaudio_q15_to_dac14(q);
}


#include <stdio.h>

int main(void) {
    WDT_A_hold(WDT_A_BASE);

    arm_fir_init_q15(&F, NUMTAPS, B, taps, BLOCKSIZE);

//    xlaudio_init_dma(FS_8000_HZ, XLAUDIO_J1_2_IN, BUFLEN_8, processBuffer);
    xlaudio_init_intr(FS_8000_HZ, XLAUDIO_J1_2_IN, processSample);

    uint32_t c = xlaudio_measurePerfBuffer(processBuffer);
    printf("DMA Cycles: %d\n", c);
     c = xlaudio_measurePerfSample(processSample);
    printf("INT Cycles: %d\n", c);

    xlaudio_run();

    return 1;
}

