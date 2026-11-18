#ifndef SMC_SLIDINGMODEC_H
#define SMC_SLIDINGMODEC_H

typedef struct SMC {
    /* Configuration and public state. */
    float C;
    float K;
    float target;
    float error_eps;
    float q;
    float p;
    float u_max;
    float J;
    float angle;
    float ang_vel;
    float epsilon;
    float u;

    /* Internal state (history) used by the controller. */
    float error;
    float error_last;
    float target_dot;
    float target_ddot;
    float target_last;
    float s;
    float ds;
} SMC;

void SMC_Init(SMC *self,
              float C,
              float K,
              float ref,
              float error_eps,
              float q,
              float p,
              float u_max,
              float J,
              float epsilon);

void SMC_Reset(SMC *self);

void SMC_Tick(SMC *self, float angle_now, float angle_vel);

#endif 
