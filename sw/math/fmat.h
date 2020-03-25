#ifndef FMAT_H
#define FMAT_H

#include <vector>
#include "trigonometry.h"
#include "auxiliary.h"

template<typename T>
class fmat
{
public:
  static void normalize(vector<T, std::allocator<T>> &matrix, uint n_rows, uint n_cols);
  static void normalize_larger_than_1(vector<T, std::allocator<T>> &matrix, uint n_rows, uint n_cols);
  static void make_identity(vector<T, std::allocator<T>> &matrix, uint n);
  static void add(uint n_row, uint n_col, vector<T, std::allocator<T>> &r, vector<T, std::allocator<T>> &a,
                  vector<T, std::allocator<T>> &b);
  static void print(uint n_row, uint n_col, vector<T, std::allocator<T>> &a);
  static void mult(uint n_rowa, uint n_cola, uint n_colb, vector<T, std::allocator<T>> &r,
                   vector<T, std::allocator<T>> &a, vector<T, std::allocator<T>> &b);
  static void scal_mult(uint n_row, uint n_col, vector<T, std::allocator<T>> &r, float k,
                        vector<T, std::allocator<T>> &a);
};

// #include "fmat.h"

template<class T>
void fmat<T>::normalize(vector<T, std::allocator<T>> &matrix, uint n_rows, uint n_cols)
{
  for (uint i = 0; i < n_rows; i++) {
    float sum_of_elems = 0;
    sum_of_elems = accumulate(matrix.begin() + i * n_rows, matrix.begin() + i * n_rows + (n_cols), sum_of_elems);
    if (sum_of_elems != 1.0) {
      for (uint j = 0; j < n_cols; j++) {
        matrix[i * n_rows + j] = matrix[i * n_rows + j] / sum_of_elems;
      }
    }
  }
}

template<class T>
void fmat<T>::normalize_larger_than_1(vector<T, std::allocator<T>> &matrix, uint n_rows, uint n_cols)
{
  for (uint i = 0; i < n_rows; i++) {
    float sum_of_elems = 0;
    sum_of_elems = accumulate(matrix.begin() + i * n_rows, matrix.begin() + i * n_rows + (n_cols), sum_of_elems);
    if (sum_of_elems != 1.0 && sum_of_elems > 0) {
      for (uint j = 0; j < n_cols; j++) {
        matrix[i * n_rows + j] = matrix[i * n_rows + j] / sum_of_elems;
      }
    }
  }
}

/* Make an identity matrix */
template<class T>
void fmat<T>::make_identity(vector<T, std::allocator<T>> &matrix, uint n)
{
  for (uint i = 0; i < n; i++) {
    for (uint j = 0; j < n; j++) {
      if (i == j) {
        matrix[i * n + j] = 1.0;
      } else {
        matrix[i * n + j] = 0.0;
      }
    }
  }
};

/* Function to add two matrices to eachother */
template<class T>
void fmat<T>::add(uint n_row, uint n_col, vector<T, std::allocator<T>> &r, vector<T, std::allocator<T>> &a,
                  vector<T, std::allocator<T>> &b)
{
  uint row, col, ridx;
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      r[ridx] = a[ridx] + b[ridx];
    }
  }
}

template<class T>
void fmat<T>::print(uint n_row, uint n_col, vector<T, std::allocator<T>> &a)
{
  uint row, col, ridx;
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      printf("%2.2f \t", (float)a[ridx]);
    }
    printf(";\n");
  }
}

/* Multiply two matrices with eachother */
template<class T>
void fmat<T>::mult(uint n_rowa, uint n_cola, uint n_colb, vector<T, std::allocator<T>> &r,
                   vector<T, std::allocator<T>> &a, vector<T, std::allocator<T>> &b)
{
  uint row, col, k, ridx, aidx, bidx;
  for (row = 0; row < n_rowa; row++) {
    for (col = 0; col < n_colb; col++) {
      ridx = col + row * n_colb;
      for (k = 0; k < n_cola; k++) {
        aidx = k + row * n_cola;
        bidx = col + k * n_colb;
        r[ridx] += a[aidx] * b[bidx];
      }
    }
  }
}

/* Function to multiply a matrix by a scalar value */
template<class T>
void fmat<T>::scal_mult(uint n_row, uint n_col, vector<T, std::allocator<T>> &r, float k,
                        vector<T, std::allocator<T>> &a)
{
  uint row, col, ridx;
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      r[ridx] = k * a[ridx];
    }
  }
}


#endif /*FMAT_H*/