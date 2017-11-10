#ifndef MATRIX_MANIPULATOR_H
#define MATRIX_MANIPULATOR_H

#include <stdio.h> // needed for the printf statements
#include <vector>

using namespace std;

class matrixmanipulator
{
  matrixmanipulator(){};
  ~matrixmanipulator(){};

public:
  vector<float> fmat_add(int n_row, int n_col, vector<float> &a, vector<float> &b);
  vector<float> fmat_sub(int n_row, int n_col, vector<float> &a, vector<float> &b);
  vector<float> fmat_transpose(int n_row, int n_col, vector<float> &a);
  vector<float> fmat_scal_mult(int n_row, int n_col, float k, vector<float> &a);
  vector<float> fmat_add_scal_mult(int n_row, int n_col, vector<float> &a, float k, vector<float> &b);
  vector<float> fmat_mult(int n_rowa, int n_cola, int n_colb, vector<float> &a, vector<float> &b);

  void fmat_print(int n_row, int n_col, vector<float> &a);
  void fmat_inverse(int n, vector<float> &inv, vector<float> &a);
  void fmat_make_zeroes(float *matrix, int row, int col);
  void fmat_make_identity(float *matrix, int n);
};

#endif /* MATRIX_MANIPULATOR_H */