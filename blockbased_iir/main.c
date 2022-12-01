#include "xlaudio.h"
#include "xlaudio_armdsp.h"

// coefficients produced using Matlab filterDesigner

typedef float32_t real32_T;

#define MWSPT_NSEC 9
const int NL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1 };
const real32_T NUM[MWSPT_NSEC][3] = {
  {
      0.583255887,              0,              0
  },
  {
                1,   -1.192962766,              1
  },
  {
      0.583255887,              0,              0
  },
  {
                1,   -1.584364414,              1
  },
  {
     0.1701243967,              0,              0
  },
  {
                1,   -1.320302367,              1
  },
  {
     0.1701243967,              0,              0
  },
  {
                1,   -1.499031901,              1
  },
  {
                1,              0,              0
  }
};
const int DL[MWSPT_NSEC] = { 1,3,1,3,1,3,1,3,1 };
const real32_T DEN[MWSPT_NSEC][3] = {
  {
                1,              0,              0
  },
  {
                1,   -1.368279934,   0.9712641239
  },
  {
                1,              0,              0
  },
  {
                1,   -1.421427727,   0.9723116755
  },
  {
                1,              0,              0
  },
  {
                1,   -1.352174163,   0.9914537072
  },
  {
                1,              0,              0
  },
  {
                1,   -1.462519288,   0.9920934439
  },
  {
                1,              0,              0
  }
};

// number of cascade sections
#define NUMSECTIONS 4
float32_t taps[4 * NUMSECTIONS];
float32_t coefficients[5 * NUMSECTIONS];

// shown as a function to illustrate how matlab format
// maps to ARM CMSIS format. For the most compact
// implementation, you would precompute this mapping
// and initialize coefficients as a constant array

void mat2armdsp() {
    coefficients[0]  =  NUM[0][0] * NUM[1][0];
    coefficients[1]  =  NUM[0][0] * NUM[1][1];
    coefficients[2]  =  NUM[0][0] * NUM[1][2];
    coefficients[3]  = -DEN[0][0] * DEN[1][1];
    coefficients[4]  = -DEN[0][0] * DEN[1][2];
    coefficients[5]  =  NUM[2][0] * NUM[3][0];
    coefficients[6]  =  NUM[2][0] * NUM[3][1];
    coefficients[7]  =  NUM[2][0] * NUM[3][2];
    coefficients[8]  = -DEN[2][0] * DEN[3][1];
    coefficients[9]  = -DEN[2][0] * DEN[3][2];
    coefficients[10] =  NUM[4][0] * NUM[5][0];
    coefficients[11] =  NUM[4][0] * NUM[5][1];
    coefficients[12] =  NUM[4][0] * NUM[5][2];
    coefficients[13] = -DEN[4][0] * DEN[5][1];
    coefficients[14] = -DEN[4][0] * DEN[5][2];
    coefficients[15] =  NUM[6][0] * NUM[7][0];
    coefficients[16] =  NUM[6][0] * NUM[7][1];
    coefficients[17] =  NUM[6][0] * NUM[7][2];
    coefficients[18] = -DEN[6][0] * DEN[7][1];
    coefficients[19] = -DEN[6][0] * DEN[7][2];
}

arm_biquad_casd_df1_inst_f32 F;

#define BLOCKSIZE 8

void processBuffer(uint16_t x[BLOCKSIZE], uint16_t y[BLOCKSIZE]) {
    float32_t xf[BLOCKSIZE], yf[BLOCKSIZE];
    xlaudio_adc14_to_f32_vec(x, xf, BLOCKSIZE);
    arm_biquad_cascade_df1_f32(&F, xf, yf, BLOCKSIZE);
    xlaudio_f32_to_dac14_vec(yf, y, BLOCKSIZE);
}

#include <stdio.h>

int main(void) {
    WDT_A_hold(WDT_A_BASE);

    mat2armdsp();
    arm_biquad_cascade_df1_init_f32(&F, NUMSECTIONS, coefficients, taps);

    xlaudio_init_dma(FS_8000_HZ, XLAUDIO_J1_2_IN, BUFLEN_8, processBuffer);

    uint32_t c = xlaudio_measurePerfBuffer(processBuffer);
    printf("DMA Cycles: %d\n", c);

    xlaudio_run();

    return 1;
}
