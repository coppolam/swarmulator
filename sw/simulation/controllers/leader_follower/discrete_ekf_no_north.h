/*
 * Copyright (C) Mario Coppola
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/relative_localization_filter/discrete_ekf_no_north.h"
 * @author Steven van der Helm, Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization
 * This implementation is was used in
 * van der Helm et al. "On-board Range-based Relative Localization for Micro Aerial Vehicles in indoor Leader-Follower Flight." (2018).
 * Available at https://arxiv.org/pdf/1805.07171.pdf
 */

#ifndef DISCRETE_EKF_NO_NORTH_H
#define DISCRETE_EKF_NO_NORTH_H

#include "stdlib.h"
#include "string.h"
#include "math.h"

#define EKF_N 9
#define EKF_M 7
#define EKF_L 6

#ifndef NOMINMAX

#ifndef Max
#define Max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef Min
#define Min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#endif  /* NOMINMAX */


/** Make a pointer to a matrix of _rows lines */
#define MAKE_MATRIX_PTR(_ptr, _mat, _rows) \
  float * _ptr[_rows]; \
  { \
    int __i; \
    for (__i = 0; __i < _rows; __i++) { _ptr[__i] = &_mat[__i][0]; } \
  }
/** o = a * b
 *
 * a: [m x n]
 * b: [n x 1]
 * o: [m x 1]
 */
static inline void float_mat_vect_mul(float *o, float **a, float *b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    o[i] = 0;
    for (j = 0; j < n; j++) {
      o[i] += a[i][j] * b[j];
    }
  }
}
/** a = 0 */
static inline void float_vect_zero(float *a, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] = 0.; }
}

/** o = a + b */
static inline void float_mat_sum(float **o, float **a, float **b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { o[i][j] = a[i][j] + b[i][j]; }
  }
}

/** a *= s */
static inline void float_vect_scale(float *a, const float s, const int n)
{
  int i;
  for (i = 0; i < n; i++) { a[i] *= s; }
}

/** o = a + b */
static inline void float_vect_sum(float *o, const float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] + b[i]; }
}

/** o = a - b */
static inline void float_vect_diff(float *o, const float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] - b[i]; }
}

/** o = a * b (element wise) */
static inline void float_vect_mul(float *o, const float *a, const float *b, const int n)
{
  int i;
  for (i = 0; i < n; i++) { o[i] = a[i] * b[i]; }
}


/** o = a * b
 *
 * a: [m x n]
 * b: [n x l]
 * o: [m x l]
 */
static inline void float_mat_mul(float **o, float **a, float **b, int m, int n, int l)
{
  int i, j, k;
  for (i = 0; i < m; i++) {
    for (j = 0; j < l; j++) {
      o[i][j] = 0.;
      for (k = 0; k < n; k++) {
        o[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

/** transpose non-square matrix */
static inline void float_mat_transpose(float **o, float **a, int n, int m)
{
  int i, j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < m; j++) {
      o[j][i] = a[i][j];
    }
  }
}

/** transpose square matrix */
static inline void float_mat_transpose_square(float **a, int n)
{
  int i, j;
  for (i = 0; i < n; i++) {
    for (j = 0; j < i; j++) {
      float t = a[i][j];
      a[i][j] = a[j][i];
      a[j][i] = t;
    }
  }
}

/** Make an n x n identity matrix (for matrix passed as array) */
static inline void float_mat_diagonal_scal(float **o, float v, int n)
{
  int i, j;
  for (i = 0 ; i < n; i++) {
    for (j = 0 ; j < n; j++) {
      if (i == j) {
        o[i][j] = v;
      } else {
        o[i][j] = 0.0;
      }
    }
  }
}

/** a = 0 */
static inline void float_mat_zero(float **a, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { a[i][j] = 0.; }
  }
}


/** a *= k, where k is a scalar value
 *
 * a: [m x n]
 * k: [1 x 1]
 */
static inline void float_mat_scale(float **a, float k, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      a[i][j] *= k;
    }
  }
}

typedef struct discrete_ekf_no_north {
  float X[EKF_N];  // state X
  float Xp[EKF_N]; // state prediction
  float Zp[EKF_M]; // measurement prediction
  float P[EKF_N][EKF_N]; // state covariance matrix
  float Q[EKF_N][EKF_N]; // proces covariance noise
  float R[EKF_M][EKF_M]; // measurement covariance noise
  float H[EKF_M][EKF_N]; // jacobian of the measure wrt X
  float G[EKF_N][EKF_L]; // Noise input
  float Fx[EKF_N][EKF_N]; // Jacobian of state
  float Ht[EKF_N][EKF_M]; // transpose of H
  float Phi[EKF_N][EKF_N]; // Jacobian
  float Gamma[EKF_N][EKF_L]; // Noise input

  float tmp1[EKF_N][EKF_N];
  float tmp2[EKF_N][EKF_N];
  float tmp3[EKF_N][EKF_N];
  float tmp4[EKF_N][EKF_N];

  float dt;
} discrete_ekf_no_north;


void extractPhiGamma(float **inmat, float **phi, float **gamma, int m, int n_a, int n_b);
void float_mat_combine(float **a, float **b, float **o, int m, int n_a, int n_b);
void c2d(int m, int nA, int nB, float **Fx, float **G, float dt, float **phi, float **gamma);
void discrete_ekf_no_north_fsym(float *statein, float *input, float *output);
void discrete_ekf_no_north_hsym(float *statein, float *output);
void discrete_ekf_no_north_Fx(float *statein, float *input, float **output);
void discrete_ekf_no_north_G(float *statein, float **output);
void discrete_ekf_no_north_Hx(float *statein, float **output);

extern void discrete_ekf_no_north_new(struct discrete_ekf_no_north *filter);
extern void discrete_ekf_no_north_predict(struct discrete_ekf_no_north *filter, float *U);
extern void discrete_ekf_no_north_update(struct discrete_ekf_no_north *filter, float *y);

#endif /* DISCRETE_EKF_NO_NORTH_H */
