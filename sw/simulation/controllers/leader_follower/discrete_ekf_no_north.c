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
 * @file "modules/relative_localization_filter/discrete_ekf_no_north.c"
 * @author Steven van der Helm, Mario Coppola
 * Discrete Extended Kalman Filter for Relative Localization without requiring knowledge of North
 * This implementation is from
 * van der Helm et al. "On-board Range-based Relative Localization for Micro Aerial Vehicles in indoor Leader-Follower Flight." (2018).
 * Available at https://arxiv.org/pdf/1805.07171.pdf
 */

#include "discrete_ekf_no_north.h"
#include <math.h>
#include <stdio.h> // needed for the printf statements

enum ekf_statein {x12, y12, z1, z2, u1, v1, u2, v2, gam};
enum ekf_input {u1dm, v1dm, u2dm, v2dm, r1m, r2m};

void extractPhiGamma(float **inmat, float **phi, float **gamma, int m, int n_a, int n_b)
{
  int totalsize = m + n_b;
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < totalsize; j++) {
      if (j < n_a) {
        phi[i][j] = inmat[i][j];
      } else {
        gamma[i][j - n_a] = inmat[i][j];
      }
    }
  }
}

/*
 * Creates a combined matrix o = [ a[m][n_a] b[m][n_b] ;
 *                                 0[m][n_a] 0[m][n_b] ]
 */
void float_mat_combine(float **a, float **b, float **o, int m, int n_a, int n_b)
{
  int totalsize = m + n_b;
  for (int i = 0; i < totalsize; i++) {
    for (int j = 0; j < totalsize; j++) {
      if (i < m && j < n_a) {
        o[i][j] = a[i][j];
      } else if (i < m) {
        o[i][j] = b[i][j - n_a];
      } else {
        o[i][j] = 0.0;
      }
    }
  }
}
/** a = b */
static inline void float_mat_copy(float **a, float **b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { a[i][j] = b[i][j]; }
  }
}
/*
 * Returns the real part of the log of v in base of n
 */
static inline float float_log_n(float v, float n)
{
  if (fabsf(v) < 1e-4) { // avoid inf
    return - 1.0E+30;
  }
  if (fabsf(n) < 1e-4) { // avoid nan
    return 0;
  }
  return logf(fabsf(v)) / logf(n);
}

/** a += k*b, where k is a scalar value
 *
 * a: [m x n]
 * b: [m x n]
 * k: [1 x 1]
 */
static inline void float_mat_sum_scaled(float **a, float **b, float k, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      a[i][j] += k * b[i][j];
    }
  }
}

/** o = a - b */
static inline void float_mat_diff(float **o, float **a, float **b, int m, int n)
{
  int i, j;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) { o[i][j] = a[i][j] - b[i][j]; }
  }
}
/** o = a * b
 *
 * a: [m x n]
 * b: [n x l]
 * o: [m x l]
 *
 * Multiply two matrices with eachother.
 * By using a temporary array to store result. The resulting matrix can be stored in one
 * of the input matrices when this function is used, which is useful for consecutive multiplications
 * (e.g. when doing matrix exponentiation), at the cost of some copy overhead.
 */
static inline void float_mat_mul_copy(float **o, float **a, float **b, int m, int n, int l)
{
  float temp[m][l];
  int i, j, k;
  for (i = 0; i < m; i++) {
    for (j = 0; j < l; j++) {
      temp[i][j] = 0.;
      for (k = 0; k < n; k++) {
        temp[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  MAKE_MATRIX_PTR(_o,  o,  m);
  MAKE_MATRIX_PTR(_temp,  temp,  m);
  float_mat_copy(_o, _temp, m, l);
}

/** Calculate inverse of any n x n matrix (passed as C array) o = mat^-1
Algorithm verified with Matlab.
Thanks to: https://www.quora.com/How-do-I-make-a-C++-program-to-get-the-inverse-of-a-matrix-100-X-100
*/
void float_mat_invert(float **o, float **mat, int n)
{
  int i, j, k;
  float t;
  float a[n][2 * n];

  // Append an identity matrix on the right of the original matrix
  for (i = 0; i < n; i++) {
    for (j = 0; j < 2 * n; j++) {
      if (j < n) {
        a[i][j] = mat[i][j];
      } else if ((j >= n) && (j == i + n)) {
        a[i][j] = 1.0;
      } else {
        a[i][j] = 0.0;
      }
    }
  }

  // Do the inversion
  for (i = 0; i < n; i++) {
    t = a[i][i]; // Store diagonal variable (temp)

    for (j = i; j < 2 * n; j++) {
      a[i][j] = a[i][j] / t; // Divide by the diagonal value
    }

    for (j = 0; j < n; j++) {
      if (i != j) {
        t = a[j][i];
        for (k = 0; k < 2 * n; k++) {
          a[j][k] = a[j][k] - t * a[i][k];
        }
      }
    }
  }

  // Cut out the identity, which has now moved to the left side
  for (i = 0 ; i < n ; i++) {
    for (j = n; j < 2 * n; j++) {
      o[i][j - n] = a[i][j];
    }
  }
}

/* Returns L-oo of matrix a */
float float_mat_norm_li(float **a, int m, int n)
{
  float row_sum;
  float value;

  value = 0.0;
  for (int i = 0; i < m; i++) {
    row_sum = 0.0;
    for (int j = 0; j < n; j++) {
      row_sum = row_sum + fabsf(a[i][j]);
    }
    value = Max(value, row_sum);
  }
  return value;
}

void float_mat_exp(float **a, float **o, int n)
{
  float a_norm, c, t;
  const int q = 6;
  float d[n][n];
  float x[n][n];
  float a_copy[n][n];
  int ee, k, s;
  int p;

  MAKE_MATRIX_PTR(_a,  a,  n);
  MAKE_MATRIX_PTR(_o,  o,  n);
  MAKE_MATRIX_PTR(_d,  d,  n);
  MAKE_MATRIX_PTR(_x,  x,  n);
  MAKE_MATRIX_PTR(_a_copy, a_copy, n);

  float_mat_copy(_a_copy, _a, n, n); // Make a copy of a to compute on
  a_norm = float_mat_norm_li(_a_copy, n, n);  // Compute the infinity norm of the matrix
  ee = (int)(float_log_n(a_norm, 2)) + 1;
  s = Max(0, ee + 1);
  t = 1.0 / powf(2.0, s);
  float_mat_scale(_a_copy, t, n, n);
  float_mat_copy(_x, _a_copy, n, n);  // x = a_copy
  c = 0.5;

  float_mat_diagonal_scal(_o, 1.0, n);  // make identiy
  float_mat_sum_scaled(_o, _a_copy, c, n, n);

  float_mat_diagonal_scal(_d, 1.0, n);
  float_mat_sum_scaled(_d, _a_copy, -c, n, n);

  p = 1;
  for (k = 2; k <= q; k++) {
    c = c * (float)(q - k + 1) / (float)(k * (2 * q - k + 1));
    float_mat_mul_copy(_x, _x, _a_copy, n, n, n);

    float_mat_sum_scaled(_o, _x, c, n, n);

    if (p) {
      float_mat_sum_scaled(_d, _x, c, n, n);
    } else {
      float_mat_sum_scaled(_d, _x, -c, n, n);
    }
    p = !p;
  }

  // E -> inverse(D) * E
  float temp[n][n];
  MAKE_MATRIX_PTR(_temp, temp, n);
  float_mat_invert(_temp, _d, n);
  float_mat_mul_copy(_o, _temp, _o, n, n, n);

  // E -> E^(2*S)
  for (k = 1; k <= s; k++) {
    float_mat_mul_copy(_o, _o, _o, n, n, n);
  }

}



/*
 * Continuous to discrete transition matrix
 */
void c2d(int m, int nA, int nB, float **Fx, float **G, float dt, float **phi, float **gamma)
{

  int totalsize = m + nB;
  float combmat[totalsize][totalsize];
  float expm[totalsize][totalsize];

  MAKE_MATRIX_PTR(_Fx, Fx, EKF_N);
  MAKE_MATRIX_PTR(_G, G, EKF_L);
  MAKE_MATRIX_PTR(_phi, phi, m);
  MAKE_MATRIX_PTR(_gamma, gamma, m);
  MAKE_MATRIX_PTR(_combmat, combmat, totalsize);
  MAKE_MATRIX_PTR(_expm, expm, totalsize);

  float_mat_scale(_Fx, dt, m, nA);
  float_mat_scale(_G, dt, m, nB);

  float_mat_combine(_Fx, _G, _combmat, m, nA, nB);
  float_mat_exp(_combmat, _expm, totalsize);
  extractPhiGamma(_expm, _phi, _gamma, m, nA, nB);
}

/*
 * Continuous time state transition equation
 * state is: {x_rel,y_rel,h1,h2,u1,v1,u2,v2,gamma}
 */

void discrete_ekf_no_north_fsym(float *statein, float *input, float *output)
{
  output[0] =  input[r1m] * statein[y12] - statein[u1] + statein[u2] * cosf(statein[gam]) - statein[v2] * sinf(
                 statein[gam]);
  output[1] = -input[r1m] * statein[x12] - statein[v1] + statein[u2] * sinf(statein[gam]) + statein[v2] * cosf(
                statein[gam]);
  output[2] = 0;
  output[3] = 0;
  output[4] = input[u1dm] + input[r1m] * statein[v1];
  output[5] = input[v1dm] - input[r1m] * statein[u1];
  output[6] = input[u2dm] + input[r2m] * statein[v2];
  output[7] = input[v2dm] - input[r2m] * statein[u2];
  output[8] = input[r2m] - input[r1m];
}

/*
 * Measurement equation, measures range, height1, height2, u1, v1, u2, v2
 */
void discrete_ekf_no_north_hsym(float *statein, float *output)
{
  output[0] = powf(powf((statein[z1] - statein[z2]), 2.0) + powf(statein[x12], 2.0) + powf(statein[y12], 2.0), 0.5);
  output[1] = statein[z1];
  output[2] = statein[z2];
  output[3] = statein[u1];
  output[4] = statein[v1];
  output[5] = statein[u2];
  output[6] = statein[v2];
}

void discrete_ekf_no_north_Fx(float *statein, float *input, float **output)
{
  MAKE_MATRIX_PTR(_output, output, EKF_N);
  float_mat_zero(_output, EKF_N, EKF_N);
  output[0][1] = input[r1m];
  output[0][4] = -1;
  output[0][6] = cosf(statein[gam]);
  output[0][7] = -sinf(statein[gam]);
  output[0][8] = -statein[v2] * cosf(statein[gam]) - statein[u2] * sinf(statein[gam]);
  output[1][0] = -input[r1m];
  output[1][5] = -1;
  output[1][6] = sinf(statein[gam]);
  output[1][7] = cosf(statein[gam]);
  output[1][8] = statein[u2] * cosf(statein[gam]) - statein[v2] * sinf(statein[gam]);
  output[4][5] = input[r1m];
  output[5][4] = -input[r1m];
  output[6][7] = input[r2m];
  output[7][6] = -input[r2m];
}

void discrete_ekf_no_north_G(float *statein, float **output)
{
  MAKE_MATRIX_PTR(_output, output, EKF_N);
  float_mat_zero(_output, EKF_N, EKF_L);
  output[0][4] = statein[y12];
  output[1][4] = -statein[x12];
  output[4][0] = 1;
  output[4][4] = statein[v1];
  output[5][1] = 1;
  output[5][4] = -statein[u1];
  output[6][2] = 1;
  output[6][4] = statein[v2];
  output[7][3] = 1;
  output[7][4] = -statein[u2];
  output[8][4] = -1;
  output[8][5] = 1;
}

void discrete_ekf_no_north_Hx(float *statein, float **output)
{
  MAKE_MATRIX_PTR(_output, output, EKF_N);
  float_mat_zero(_output, EKF_M, EKF_N);
  output[0][0] = statein[x12] / (powf(powf(statein[z1] - statein[z2], 2.0) + powf(statein[x12], 2.0) + powf(statein[y12],
                                      2.0), 0.5));
  output[0][1] = statein[y12] / (powf(powf(statein[z1] - statein[z2], 2.0) + powf(statein[x12], 2.0) + powf(statein[y12],
                                      2.0), 0.5));
  output[0][2] = (2 * statein[z1] - 2 * statein[z2]) / (2 * powf(powf(statein[z1] - statein[z2], 2.0) + powf(statein[x12],
                 2.0) + powf(statein[y12], 2.0), 0.5));
  output[0][3] = -(2 * statein[z1] - 2 * statein[z2]) / (2 * powf(powf(statein[z1] - statein[z2],
                 2.0) + powf(statein[x12],
                             2.0) + powf(statein[y12], 2.0), 0.5));
  output[1][2] = 1;
  output[2][3] = 1;
  output[3][4] = 1;
  output[4][5] = 1;
  output[5][6] = 1;
  output[6][7] = 1;
}

/*
 * Initialize the filter
 */
void discrete_ekf_no_north_new(struct discrete_ekf_no_north *filter)
{
  MAKE_MATRIX_PTR(_P, filter->P, EKF_N);
  MAKE_MATRIX_PTR(_Q, filter->Q, EKF_L);
  MAKE_MATRIX_PTR(_R, filter->R, EKF_M);

  float_mat_diagonal_scal(_P, 16, EKF_N); // P Matrix
  float_mat_diagonal_scal(_Q, powf(2, 2), EKF_L); // Q Matrix [inputs: a1x, a1y, a2x, a2y, r1, r2]
  filter->Q[4][4] = powf(0.2, 2);
  filter->Q[5][5] = powf(0.2, 2);

  float_mat_diagonal_scal(_R, 0.7, EKF_M); // R Matrix [range, h1, h2, u1, v1, u2, v2]
  filter->R[0][0] = powf(0.5, 2);
  filter->R[1][1] = powf(0.2, 2);
  filter->R[2][2] = powf(0.2, 2);

  float_vect_zero(filter->X, EKF_N); // Initial state
  filter->X[0] = 1.0; // Initial X estimate
  filter->X[1] = 1.0; // Initial Y estimate
  filter->dt = 0.1;  // Initial Est. time difference
}

/*
 * Perform the prediction step
 */
void discrete_ekf_no_north_predict(struct discrete_ekf_no_north *filter, float *U)
{
  float dX[EKF_N];

  MAKE_MATRIX_PTR(_tmp1, filter->tmp1, EKF_N);
  MAKE_MATRIX_PTR(_tmp2, filter->tmp2, EKF_N);
  MAKE_MATRIX_PTR(_tmp3, filter->tmp3, EKF_N);
  MAKE_MATRIX_PTR(_tmp4, filter->tmp4, EKF_N);
  MAKE_MATRIX_PTR(_P,    filter->P,    EKF_N);
  MAKE_MATRIX_PTR(_Q,    filter->Q,    EKF_N);
  MAKE_MATRIX_PTR(_H,    filter->H,    EKF_N);
  MAKE_MATRIX_PTR(_G,    filter->G,    EKF_N);
  MAKE_MATRIX_PTR(_Gamma, filter->Gamma, EKF_N);
  MAKE_MATRIX_PTR(_Phi,  filter->Phi,  EKF_N);
  MAKE_MATRIX_PTR(_Fx,   filter->Fx,   EKF_N);

  discrete_ekf_no_north_fsym(filter->X, U, dX);
  discrete_ekf_no_north_Fx(filter->X, U, _Fx);   // State transition matrix
  discrete_ekf_no_north_G(filter->X, _G);        // Input transition matrix

  float_vect_scale(dX, filter->dt, EKF_N);
  float_vect_sum(filter->Xp, filter->X, dX, EKF_N);

  c2d(EKF_N, EKF_N, EKF_L, _Fx, _G, filter->dt, _Phi, _Gamma);

  float_mat_mul(_tmp1, _Phi, _P, EKF_N, EKF_N, EKF_N);      // tmp1 <- Phi*P
  float_mat_transpose_square(_Phi, EKF_N);                  // tmp2 <- Phi'
  float_mat_mul(_tmp3, _tmp1, _Phi, EKF_N, EKF_N, EKF_N);   // tmp3 <- Phi*P*Phi'

  float_mat_mul(_tmp1, _Gamma, _Q, EKF_N, EKF_L, EKF_L);    // tmp1 <- Gamma*Q
  float_mat_transpose(_tmp2, _Gamma, EKF_N, EKF_L);         // tmp2 <- Gamma'
  float_mat_mul(_tmp4, _tmp1, _tmp2, EKF_N, EKF_L, EKF_N);  // tmp4 <- Gamma*Q*Gamma

  float_mat_sum(_P, _tmp3, _tmp4, EKF_N, EKF_N);            // P <- Phi*P*Phi' + Gamma*Q*Gamma'

  discrete_ekf_no_north_hsym(filter->Xp, filter->Zp);
  discrete_ekf_no_north_Hx(filter->Xp, _H);
}

/* Perform the update step

    Get Kalman Gain
      P12 = P * H';
      K = P12/(H * P12 + R);

    Update x
      x = x_p + K * (z - z_p);

    Update P
      P = (eye(numel(x)) - K * H) * P;
*/
void discrete_ekf_no_north_update(struct discrete_ekf_no_north *filter, float *Z)
{
  MAKE_MATRIX_PTR(_tmp1, filter->tmp1, EKF_N);
  MAKE_MATRIX_PTR(_tmp2, filter->tmp2, EKF_N);
  MAKE_MATRIX_PTR(_tmp3, filter->tmp3, EKF_N);
  MAKE_MATRIX_PTR(_P,    filter->P,    EKF_N);
  MAKE_MATRIX_PTR(_H,    filter->H,    EKF_M);
  MAKE_MATRIX_PTR(_Ht,   filter->Ht,   EKF_N);
  MAKE_MATRIX_PTR(_R,    filter->R,    EKF_M);

  //  E = H * P * H' + R
  float_mat_transpose(_Ht, _H, EKF_M, EKF_N); // Ht = H'
  float_mat_mul(_tmp2, _P, _Ht, EKF_N, EKF_N, EKF_M); // tmp2 = P*Ht = P*H'
  float_mat_mul(_tmp1, _H, _tmp2, EKF_M, EKF_N, EKF_M); // tmp1 = H*P*H'
  float_mat_sum(_tmp3, _tmp1, _R, EKF_M, EKF_M); // E = tmp1(=H*P*H') + R

  // K = P * H' * inv(E)
  float_mat_invert(_tmp1, _tmp3, EKF_M); // tmp1 = inv(E)
  float_mat_mul(_tmp3, _tmp2, _tmp1, EKF_N, EKF_M, EKF_M); // K(tmp3) = tmp2*tmp1

  // P = P - K * H * P
  float_mat_mul(_tmp1, _tmp3, _H, EKF_N, EKF_M, EKF_N); // tmp1 = K*H
  float_mat_mul(_tmp2, _tmp1, _P, EKF_N, EKF_N, EKF_N); // tmp3 = K*H*P
  float_mat_diff(_P, _P, _tmp2, EKF_N, EKF_N); // P - K*H*P

  //  X = X + K * err
  float err[EKF_M];
  float dx_err[EKF_N];

  float_vect_diff(err, Z, filter->Zp, EKF_M); // err = Z - Zp
  float_mat_vect_mul(dx_err, _tmp3, err, EKF_N, EKF_M); // dx_err = K*err
  float_vect_sum(filter->X, filter->Xp, dx_err, EKF_N); // X = Xp + dx_err
}
