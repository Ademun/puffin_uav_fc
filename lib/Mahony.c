#include "Mahony.h"

#define SAMPLE_FREQ 100.0f
#define twoKpDef 3.0f
#define twoKiDef 0.6f

static mahony_filter_t filter = {
    .twoKp = twoKpDef,
    .twoKi = twoKiDef,
    .q = {.w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f},
    .integralFBx = 0.0f,
    .integralFBy = 0.0f,
    .integralFBz = 0.0f,
    .invFreq = 1.0f / SAMPLE_FREQ,
};

void init_mahony_filter(float freq) { filter.invFreq = 1.0f / freq; }

void update_mahony_imu(float ax, float ay, float az, float gx, float gy,
                       float gz) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  gx *= G_DEG_TO_RAD;
  gy *= G_DEG_TO_RAD;
  gz *= G_DEG_TO_RAD;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = filter.q.x * filter.q.z - filter.q.w * filter.q.y;
    halfvy = filter.q.w * filter.q.x + filter.q.y * filter.q.z;
    halfvz = filter.q.w * filter.q.w - 0.5f + filter.q.z * filter.q.z;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (filter.twoKi > 0.0f) {
      filter.integralFBx += filter.twoKi * halfex * filter.invFreq;
      filter.integralFBy += filter.twoKi * halfey * filter.invFreq;
      filter.integralFBz += filter.twoKi * halfez * filter.invFreq;
      gx += filter.integralFBx;
      gy += filter.integralFBy;
      gz += filter.integralFBz;
    } else {
      filter.integralFBx = 0.0f;
      filter.integralFBy = 0.0f;
      filter.integralFBz = 0.0f;
    }

    gx += filter.twoKp * halfex;
    gy += filter.twoKp * halfey;
    gz += filter.twoKp * halfez;
  }

  gx *= (0.5f * filter.invFreq);
  gy *= (0.5f * filter.invFreq);
  gz *= (0.5f * filter.invFreq);

  qa = filter.q.w;
  qb = filter.q.x;
  qc = filter.q.y;
  filter.q.w += (-qb * gx - qc * gy - filter.q.z * gz);
  filter.q.x += (qa * gx + qc * gz - filter.q.z * gy);
  filter.q.y += (qa * gy - qb * gz + filter.q.z * gx);
  filter.q.z += (qa * gz + qb * gy - qc * gx);

  q_normm(&filter.q);
}

void get_orientation_quat(quat_t* data) {
  data->w = filter.q.w;
  data->x = filter.q.x;
  data->y = filter.q.y;
  data->z = filter.q.z;
}
