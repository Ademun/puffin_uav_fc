#include "geom.h"

#include <math.h>

void q_normm(quat_t* q) {
  float n = invSqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  q->w *= n;
  q->x *= n;
  q->y *= n;
  q->z *= n;
}

void q_conj(quat_t* q, quat_t* b) {
  b->w = q->w;
  b->x = -q->x;
  b->y = -q->y;
  b->z = -q->z;
}

void q_mult(quat_t* a, quat_t* b, quat_t* c) {
  float w1 = a->w, x1 = a->x, y1 = a->y, z1 = a->z;
  float w2 = b->w, x2 = b->x, y2 = b->y, z2 = b->z;

  c->w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  c->x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  c->y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  c->z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

void q_multn(quat_t* q, float n) {
  q->w *= n;
  q->x *= n;
  q->y *= n;
  q->z *= n;
}

void q_vec(quat_t* q, vec3_t* v) {
  v->x = q->x;
  v->y = q->y;
  v->z = q->z;
};

void q_euler(quat_t* q, vec3_t* angles) {
  float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
  float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
  angles->y = atan2f(sinr_cosp, cosr_cosp);

  float sinp = 2.0f * (q->w * q->y - q->z * q->x);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  angles->x = asinf(sinp);

  float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
  float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
  angles->z = atan2f(siny_cosp, cosy_cosp);
}

float v_norm(vec3_t* v) {
  return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
};

void v_normm(vec3_t* v) {
  float n = invSqrt(v->x * v->x + v->y * v->y + v->z * v->z);
  v->x *= n;
  v->y *= n;
  v->z *= n;
}

void v_div(vec3_t* v, float n) {
  v->x /= n;
  v->y /= n;
  v->z /= n;
}

void v_mult(vec3_t* v, float n) {
  v->x *= n;
  v->y *= n;
  v->z *= n;
}

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}