
/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  long Delay_DSTATE;                 /* '<S1>/Delay' */
  long Delay1_DSTATE;                /* '<S1>/Delay1' */
  long Delay_DSTATE_f;               /* '<S4>/Delay' */
  long Delay1_DSTATE_b;              /* '<S4>/Delay1' */
  long Delay_DSTATE_a;               /* '<S2>/Delay' */
  long Delay1_DSTATE_l;              /* '<S2>/Delay1' */
  long Delay_DSTATE_l;               /* '<S3>/Delay' */
  long Delay1_DSTATE_f;              /* '<S3>/Delay1' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  long Mic;                          /* '<Root>/Mic' */
  int Gain_AAC;                     /* '<Root>/Gain_AAC' */
  int Gain_BPF1;                    /* '<Root>/Gain_BPF1' */
  int Gain_BPF2;                    /* '<Root>/Gain_BPF2' */
  int a0_H1;                        /* '<Root>/a0_H1' */
  int ma1_H1;                       /* '<Root>/ma1_H1' */
  int ma2_H1;                       /* '<Root>/ma2_H1' */
  int b0_H1;                        /* '<Root>/b0_H1' */
  int b1_H1;                        /* '<Root>/b1_H1' */
  int b2_H1;                        /* '<Root>/b2_H1' */
  int a0_Optimization;              /* '<Root>/a0_Optimization' */
  int ma1_Optimization;             /* '<Root>/ma1_Optimization' */
  int ma2_Optimization;             /* '<Root>/ma2_Optimization' */
  int b0_Optimization;              /* '<Root>/b0_Optimization' */
  int b1_Optimization;              /* '<Root>/b1_Optimization' */
  int b2_Optimization;              /* '<Root>/b2_Optimization' */
  int a0_MHHC1;                     /* '<Root>/a0_MHHC1' */
  int ma1_MHHC1;                    /* '<Root>/ma1_MHHC1' */
  int ma2_MHHC1;                    /* '<Root>/ma2_MHHC1' */
  int b0_MHHC1;                     /* '<Root>/b0_MHHC1' */
  int b1_MHHC1;                     /* '<Root>/b1_MHHC1' */
  int b2_MHHC1;                     /* '<Root>/b2_MHHC1' */
  int a0_MHHC2;                     /* '<Root>/a0_MHHC2' */
  int ma1_MHHC2;                    /* '<Root>/ma1_MHHC2' */
  int ma2_MHHC2;                    /* '<Root>/ma2_MHHC2' */
  int b0_MHHC2;                     /* '<Root>/b0_MHHC2' */
  int b1_MHHC2;                     /* '<Root>/b1_MHHC2' */
  int b2_MHHC2;                     /* '<Root>/b2_MHHC2' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  long AAC;                          /* '<Root>/AAC' */
  long ANC;                          /* '<Root>/ANC' */
} ExtY;
