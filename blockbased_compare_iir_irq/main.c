#include "xlaudio.h"
#include "xlaudio_armdsp.h"

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

typedef struct cascadestate {
    float32_t s[2];  // state
    float32_t b[3];  // nominator coeff  b0 b1 b2
    float32_t a[2];  // denominator coeff   a1 a2
} cascadestate_t;

float32_t cascadeiir_transpose(float32_t x, cascadestate_t *p) {
    float32_t y = (x * p->b[0]) + p->s[0];
    p->s[0]     = (x * p->b[1]) - (y * p->a[0]) + p->s[1];
    p->s[1]     = (x * p->b[2]) - (y * p->a[1]);
    return y;
}

void createcascade(float32_t b0,
                   float32_t b1,
                   float32_t b2,
                   float32_t a1,
                   float32_t a2,
                   cascadestate_t *p) {
    p->b[0] = b0;
    p->b[1] = b1;
    p->b[2] = b2;
    p->a[0] = a1;
    p->a[1] = a2;
    p->s[0] = p->s[1] = 0.0f;
}

cascadestate_t stage1;
cascadestate_t stage2;
cascadestate_t stage3;
cascadestate_t stage4;

void initcascade() {
    createcascade( NUM[0][0] * NUM[1][0],
                   NUM[0][0] * NUM[1][1],
                   NUM[0][0] * NUM[1][2],
                   DEN[0][0] * DEN[1][1],
                   DEN[0][0] * DEN[1][2],
                   &stage1);

    createcascade( NUM[2][0] * NUM[3][0],
                   NUM[2][0] * NUM[3][1],
                   NUM[2][0] * NUM[3][2],
                   DEN[2][0] * DEN[3][1],
                   DEN[2][0] * DEN[3][2],
                   &stage2);

    createcascade( NUM[4][0] * NUM[5][0],
                   NUM[4][0] * NUM[5][1],
                   NUM[4][0] * NUM[5][2],
                   DEN[4][0] * DEN[5][1],
                   DEN[4][0] * DEN[5][2],
                   &stage3);

    createcascade( NUM[6][0] * NUM[7][0],
                   NUM[6][0] * NUM[7][1],
                   NUM[6][0] * NUM[7][2],
                   DEN[6][0] * DEN[7][1],
                   DEN[6][0] * DEN[7][2],
                   &stage4);

}

float32_t transposefilter(float32_t in) {
    float32_t v;
    v = cascadeiir_transpose(in, &stage1);
    v = cascadeiir_transpose(v, &stage2);
    v = cascadeiir_transpose(v, &stage3);
    v = cascadeiir_transpose(v, &stage4);
    return v;
}

uint16_t normaloperation(uint16_t x) {
    float32_t input = xlaudio_adc14_to_f32(x);

    return xlaudio_f32_to_dac14(transposefilter(input));
}

#include <stdio.h>

int main(void) {
    WDT_A_hold(WDT_A_BASE);

    initcascade();

    xlaudio_init_intr(FS_8000_HZ, XLAUDIO_J1_2_IN, normaloperation);

    int c = xlaudio_measurePerfSample(normaloperation);
    printf("Cycle Count %d\n", c);

    xlaudio_run();

    return 1;
}
