
/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S3>/Delay' */
  real_T Delay1_DSTATE;                /* '<S3>/Delay1' */
  real_T Delay_DSTATE_o;               /* '<S4>/Delay' */
  real_T Delay1_DSTATE_d;              /* '<S4>/Delay1' */
  real_T Delay_DSTATE_d;               /* '<S1>/Delay' */
  real_T Delay1_DSTATE_p;              /* '<S1>/Delay1' */
  real_T Delay_DSTATE_m;               /* '<S2>/Delay' */
  real_T Delay1_DSTATE_h;              /* '<S2>/Delay1' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Mic2_1;                       /* '<Root>/Mic2_1' */
  real_T Mic2_2;                       /* '<Root>/Mic2_2' */
  real_T Gain_Velocity2;               /* '<Root>/Gain_Velocity2' */
  real_T Gain_Pressure2;               /* '<Root>/Gain_Pressure2' */
  real_T Gain_BPF1;                    /* '<Root>/Gain_BPF1' */
  real_T Gain_BPF2;                    /* '<Root>/Gain_BPF2' */
  real_T a0_Velocity2;                 /* '<Root>/a0_Velocity2' */
  real_T ma1_Velocity2;                /* '<Root>/ma1_Velocity2' */
  real_T ma2_Velocity2;                /* '<Root>/ma2_Velocity2' */
  real_T b0_Velocity2;                 /* '<Root>/b0_Velocity2' */
  real_T b1_Velocity2;                 /* '<Root>/b1_Velocity2' */
  real_T b2_Velocity2;                 /* '<Root>/b2_Velocity2' */
  real_T a0_Voltage2;                  /* '<Root>/a0_Voltage2' */
  real_T ma1_Voltage2;                 /* '<Root>/ma1_Voltage2' */
  real_T ma2_Voltage2;                 /* '<Root>/ma2_Voltage2' */
  real_T b0_Voltage2;                  /* '<Root>/b0_Voltage2' */
  real_T b1_Voltage2;                  /* '<Root>/b1_Voltage2' */
  real_T b2_Voltage2;                  /* '<Root>/b2_Voltage2' */
  real_T a0_MHHC1;                     /* '<Root>/a0_MHHC1' */
  real_T ma1_MHHC1;                    /* '<Root>/ma1_MHHC1' */
  real_T ma2_MHHC1;                    /* '<Root>/ma2_MHHC1' */
  real_T b0_MHHC1;                     /* '<Root>/b0_MHHC1' */
  real_T b1_MHHC1;                     /* '<Root>/b1_MHHC1' */
  real_T b2_MHHC1;                     /* '<Root>/b2_MHHC1' */
  real_T a0_MHHC2;                     /* '<Root>/a0_MHHC2' */
  real_T ma1_MHHC2;                    /* '<Root>/ma1_MHHC2' */
  real_T ma2_MHHC2;                    /* '<Root>/ma2_MHHC2' */
  real_T b0_MHHC2;                     /* '<Root>/b0_MHHC2' */
  real_T b1_MHHC2;                     /* '<Root>/b1_MHHC2' */
  real_T b2_MHHC2;                     /* '<Root>/b2_MHHC2' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T AAC2;                         /* '<Root>/AAC2' */
  real_T ANC;                          /* '<Root>/ANC' */
} ExtY;