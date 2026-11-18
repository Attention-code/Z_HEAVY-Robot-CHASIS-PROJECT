#include "slidingmodec.h"

#include <math.h>
#include <stddef.h>

static int smc_signal(float value)
{
    if (value > 0.0f) {
        return 1;
    }
    if (value < 0.0f) {
        return -1;
    }
    return 0;
}

static float smc_sat(float value)
{
    if (fabsf(value) <= 1.0f) {
        return value;
    }
    return (float)smc_signal(value);
}

void SMC_Init(SMC *self,
              float C,
              float K,
              float ref,
              float error_eps,
              float q,
              float p,
              float u_max,
              float J,
              float epsilon)
{
    if (self == NULL) {
        return;
    }

    self->C = C;
    self->K = K;
    self->target = ref;
    self->error_eps = error_eps;
    self->q = q;
    self->p = p;
    self->u_max = u_max;
    self->J = J;
    self->epsilon = epsilon;

    SMC_Reset(self);
    self->target_last = ref;
}

void SMC_Reset(SMC *self)
{
    if (self == NULL) {
        return;
    }

    self->angle = 0.0f;
    self->ang_vel = 0.0f;
    self->u = 0.0f;
    self->error = 0.0f;
    self->error_last = 0.0f;
    self->target_dot = 0.0f;
    self->target_ddot = 0.0f;
    self->target_last = self->target;
    self->s = 0.0f;
    self->ds = 0.0f;
}

void SMC_Tick(SMC *self, float angle_now, float angle_vel)
{
    float e_qp;
    float error_dot;
    float qp;

    if (self == NULL) {
        return;
    }

    self->angle = angle_now;
    self->ang_vel = angle_vel;

    if (self->p == 0.0f) {
        self->u = 0.0f;
        self->target_last = self->target;
        return;
    }

    qp = self->q / self->p;
    self->error_last = self->error;
    self->error = self->angle - self->target;
    self->target_ddot = (self->target - self->target_last) - self->target_dot;
    self->target_dot = self->target - self->target_last;
    error_dot = self->ang_vel - self->target_dot;

    if (fabsf(self->error) < self->error_eps) {
        self->u = 0.0f;
        return;
    }

    if (fabsf(self->error) < 1.0f || self->error == 0.0f) {
        self->s = self->C * self->error + error_dot;
        self->ds = -self->epsilon * smc_sat(self->s) - self->K * self->s;
        self->u = self->J *
                  (self->target_ddot - self->C * error_dot -
                   self->epsilon * smc_sat(self->s) - self->K * self->s);
    } else {
        e_qp = (self->error < 0.0f)
                   ? -powf(fabsf(self->error), qp)
                   : powf(fabsf(self->error), qp);

        self->s = error_dot + self->C * e_qp;
        self->ds = -self->epsilon * smc_sat(self->s) - self->K * self->s;
        self->u = self->J *
                  (self->target_ddot + self->ds -
                   self->C * qp * error_dot * e_qp / self->error);
    }

    if (self->u > self->u_max) {
        self->u = self->u_max;
    } else if (self->u < -self->u_max) {
        self->u = -self->u_max;
    }

    self->target_last = self->target;
}
