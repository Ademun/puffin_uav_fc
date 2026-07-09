#ifndef _LIB_GEOM_
#define _LIB_GEOM_

#define G_DEG_TO_RAD 0.01745329251f
#define G_RAD_TO_DEG 57.29577951308f

typedef struct {
  float w, x, y, z;
} quat_t;

typedef struct {
  float x, y, z;
} vec3_t;

void q_normm(quat_t* q);

void q_conj(quat_t* q, quat_t* b);

void q_mult(quat_t* a, quat_t* b, quat_t* c);

void q_multn(quat_t* q, float n);

void q_vec(quat_t* q, vec3_t* v);

void q_euler(quat_t*q, vec3_t* angles); //roll, pitch, yaw

float v_norm(vec3_t* v);

void v_normm(vec3_t* v);

void v_div(vec3_t* v, float n);

void v_mult(vec3_t* v, float n);

float invSqrt(float x);

#endif