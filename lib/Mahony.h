#ifndef _LIB_MAHONY_
#define _LIB_MAHONY_

#include "geom.h"

typedef struct {
  float twoKp, twoKi;
  quat_t q;
  float integralFBx, integralFBy, integralFBz;
  float invFreq;
} mahony_filter_t;

void init_mahony_filter(float freq);

void update_mahony_imu(float ax, float ay, float az, float gx, float gy,
                       float gz);

void get_orientation_quat(quat_t* data);

#endif