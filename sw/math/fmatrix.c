#include "fmatrix.h"

/* Function to add two matrices to eachother */
vector<float> matrixmanipulator::fmat_add(int n_row, int n_col, vector<float> &a, vector<float> &b)
{
  int row, col, ridx;
  vector<float> r; //result
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      r[ridx] = a[ridx] + b[ridx];
    }
  }

  return r;
}

/* Function to subtract two matrices from eachother */
vector<float> matrixmanipulator::fmat_sub(int n_row, int n_col, vector<float> &a, vector<float> &b)
{
  int row, col, ridx;
  vector<float> r; //result
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      r[ridx] = a[ridx] - b[ridx];
    }
  }
  return r;
}

/* Function to obtain the transpose of a matrix */
vector<float> matrixmanipulator::fmat_transpose(int n_row, int n_col, vector<float> &a)
{
  int row, col, ridx, aidx;
  vector<float> r; //result
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      aidx = row * n_col + col;
      ridx = col * n_row + row;
      r[ridx] = a[aidx];
    }
  }
  return r;
}

/* Function to multiply a matrix by a scalar value */
vector<float> matrixmanipulator::fmat_scal_mult(int n_row, int n_col, float k, vector<float> &a)
{
  int row, col, ridx;
  vector<float> r; //result
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      if (a[ridx] != 0.0) {
        r[ridx] = k * a[ridx];
      } else {
        r[ridx] = 0.0;
      }
    }
  }
  return r;
}

/* Add a scalar value to a matrix */
vector<float> matrixmanipulator::fmat_add_scal_mult(int n_row, int n_col, vector<float> &a, float k, vector<float> &b)
{
  int row, col, ridx;
  vector<float> r; //result
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      r[ridx] = a[ridx] + k * b[ridx];
    }
  }
  return r;
}

/* Multiply two matrices with eachother */
vector<float> matrixmanipulator::fmat_mult(int n_rowa, int n_cola, int n_colb, vector<float> &a, vector<float> &b)
{
  int row, col, k, ridx, aidx, bidx;
  vector<float> r; //result
  for (row = 0; row < n_rowa; row++) {
    for (col = 0; col < n_colb; col++) {
      ridx = col + row * n_colb;
      r[ridx] = 0.;
      for (k = 0; k < n_cola; k++) {
        aidx = k + row * n_cola;
        bidx = col + k * n_colb;
        r[ridx] += a[aidx] * b[bidx];
      }
    }
  }
  return r;
}

/* Print the matrix */
void matrixmanipulator::fmat_print(int n_row, int n_col, vector<float> &a)
{
  int row, col, ridx;
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      printf("%2.2f\t", a[ridx]);
    }
    printf("\n");
  }
}


/* Calculate inverse of matrix

Compliments of:
https://www.quora.com/How-do-I-make-a-C++-program-to-get-the-inverse-of-a-matrix-100-X-100

Algorithm verified with Matlab
*/
void matrixmanipulator::fmat_inverse(int n, vector<float> &matinv, float *mat)
{
  int i, j, k, row, col;
  float t;

  float a[2 * n * n];

  /* Append an ide1ntity matrix on the right of the original matrix */
  for (row = 0; row < n; row++) {
    for (col = 0; col < 2 * n; col++) {
      if (col < n) {
        a[ row * 2 * n + col ] = mat[row * n + col];
      } else if ((col >= n) && (col == row + n)) {
        a[ row * 2 * n + col ] = 1.0;
      } else {
        a[ row * 2 * n + col ] = 0.0;
      }
    }
  }

  /* Do the inversion */
  for (i = 0; i < n; i++) {
    // Store diagonal variable (temp)
    t = a[ i * 2 * n + i ];

    for (j = i; j < 2 * n; j++) {
      // Divide by the diagonal value
      a[ i * 2 * n + j ] = a[ i * 2 * n + j ] / t;
    }

    for (j = 0; j < n; j++)

    {
      if (i != j) {
        t = a[ j * 2 * n + i ];

        for (k = 0; k < 2 * n; k++) {
          a[j * 2 * n + k] = a[j * 2 * n + k] - t * a[i * 2 * n + k];
        }

      }

    }

  }

  /* Cut out the identity, which has now moved to the left side */
  for (row = 0 ; row < n ; row++) {
    for (col = n; col < 2 * n; col++) {
      matinv[row * n + col - n] = a[row * 2 * n + col];
    }

  }

};

/* Make a matrix of zeros */
vector<float> matrixmanipulator::fmat_make_zeroes(int row, int col)
{
  int i, j;
  for (i = 0 ; i < row; i++) {
    for (j = 0 ; j < col; j++) {
      matrix[i * col + j] = 0.0;
    }
  }
};

/* Make an identity matrix */
vector<float> matrixmanipulator::fmat_make_identity(int n)
{
  int i, j;
  for (i = 0 ; i < n; i++) {
    for (j = 0 ; j < n; j++) {
      if (i == j) {
        matrix[i * n + j] = 1.0;
      } else {
        matrix[i * n + j] = 0.0;
      }
    }
  }
};