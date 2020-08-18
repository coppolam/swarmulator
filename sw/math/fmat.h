#ifndef FMAT_H
#define FMAT_H

#include "trigonometry.h"
#include "auxiliary.h"

/**
 * @brief Template class for matrix operations.
 *
 * Header class for matrix operations. Can be invoked anywhere by using fmat<type>::function(inputs).
 * The class uses a template type so the matrix can be any std::vector<type>.
 * @tparam T
 */
template<typename T>
class fmat
{
public:
  /**
   * Normalize a matrix with n rows and m columns
   *
   * @param matrix
   * @param n_rows number of rows
   * @param n_cols number of columns
   */
  static void normalize(std::vector<T, std::allocator<T>> &matrix, const uint &n_rows, const uint &n_cols);

  /**
   * @brief Normalize all rows that are larger than 1
   *
   * @param matrix
   * @param n_rows number of rows
   * @param n_cols number of columns
   */
  static void normalize_larger_than_1(std::vector<T, std::allocator<T>> &matrix, const uint &n_rows, const uint &n_cols);

  /**
   * @brief Make an identity matrix
   *
   * @param matrix Matrix to make into an identity matrix
   * @param n Size of matrix
   */
  static void make_identity(std::vector<T, std::allocator<T>> &matrix, const uint &n);

  /**
   * @brief Add two matrix
   *
   * @param n_row Number of rows
   * @param n_col Number of columns
   * @param r Output matrix = A + B
   * @param a matrix A
   * @param b matrix B
   */
  static void add(const uint &n_row, const uint &n_col, std::vector<T, std::allocator<T>> &r,
                  std::vector<T, std::allocator<T>> &a,
                  std::vector<T, std::allocator<T>> &b);

  /**
   * Print a matrix to the terminal
   *
   * @param n_row Number of rows
   * @param n_col Number of columns
   * @param a Matrix
   * @param name Name of matrix (as std::string)
   */
  static void print(const uint &n_row, const uint &n_col, std::vector<T, std::allocator<T>> &a, std::string name);

  /**
   * @brief Multiply two matrixces
   *
   * @param n_rowa Number of rows matrix A
   * @param n_cola Number of columns matrix A
   * @param n_colb Number of columns matrix B
   * @param r Output matrix = A * B
   * @param a Matrix A
   * @param b Matrix B
   */
  static void mult(const uint &n_rowa, const uint &n_cola, const uint &n_colb, std::vector<T, std::allocator<T>> &r,
                   std::vector<T, std::allocator<T>> &a, std::vector<T, std::allocator<T>> &b);

  /**
   * Multiply a matrix by a scalar k
   *
   * @param n_row Number of rows
   * @param n_col Number of columns
   * @param r Output matrix = k * A
   * @param k Scalar
   * @param a Matrix A
   */
  static void scal_mult(const uint &n_row, const uint &n_col, std::vector<T, std::allocator<T>> &r, float k,
                        std::vector<T, std::allocator<T>> &a);

  /**
   * Write the matrix to a CSV file.
   *
   * @param filename Name of file
   * @param mat Matrix
   * @param rows Number of rows
   * @param cols Number of columns
   */
  static void write_to_csv(const std::string &filename, const std::vector<T, std::allocator<T>> &mat, const uint &rows,
                           const uint &cols);
};

template<class T>
void fmat<T>::normalize(std::vector<T, std::allocator<T>> &matrix, const uint &n_rows, const uint &n_cols)
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
void fmat<T>::normalize_larger_than_1(std::vector<T, std::allocator<T>> &matrix, const uint &n_rows, const uint &n_cols)
{
  for (uint i = 0; i < n_rows; i++) {
    float sum_of_elems = 0;
    sum_of_elems = accumulate(matrix.begin() + i * n_rows, matrix.begin() + i * n_rows + (n_cols), sum_of_elems);
    if (sum_of_elems != 1.0 && sum_of_elems > 1) {
      for (uint j = 0; j < n_cols; j++) {
        matrix[i * n_rows + j] = matrix[i * n_rows + j] / sum_of_elems;
      }
    }
  }
}

template<class T>
void fmat<T>::make_identity(std::vector<T, std::allocator<T>> &matrix, const uint &n)
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

template<class T>
void fmat<T>::add(const uint &n_row, const uint &n_col, std::vector<T, std::allocator<T>> &r,
                  std::vector<T, std::allocator<T>> &a,
                  std::vector<T, std::allocator<T>> &b)
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
void fmat<T>::print(const uint &n_row, const uint &n_col, std::vector<T, std::allocator<T>> &a, std::string name)
{
  std::cout << name << " = [..." << std::endl;
  uint row, col, ridx;
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      std::cout << a[ridx];
      printf("\t");
    }
    printf(";\n");
  }
  std::cout << "];" << std::endl;
}

template<class T>
void fmat<T>::mult(const uint &n_rowa, const uint &n_cola, const uint &n_colb, std::vector<T, std::allocator<T>> &r,
                   std::vector<T, std::allocator<T>> &a, std::vector<T, std::allocator<T>> &b)
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

template<class T>
void fmat<T>::scal_mult(const uint &n_row, const uint &n_col, std::vector<T, std::allocator<T>> &r, float k,
                        std::vector<T, std::allocator<T>> &a)
{
  uint row, col, ridx;
  for (row = 0; row < n_row; row++) {
    for (col = 0; col < n_col; col++) {
      ridx = row * n_col + col;
      r[ridx] = k * a[ridx];
    }
  }
}

template<class T>
void fmat<T>::write_to_csv(const std::string &filename, const std::vector<T> &mat, const uint &rows, const uint &cols)
{
  std::ofstream file;
  file.open(filename.c_str());
  file << "#" << filename << "\n";
  for (uint i = 0; i < rows; i++) {
    for (uint j = 0; j < cols; j++) {
      float t = mat[j + i * rows];
      if (j < cols - 1) {
        file << t << ", \t";
      } else {
        file << t << "\n";
      }
    }
  }
  file.close();
  terminalinfo::info_msg("Wrote matrix " + filename);
}

#endif /*FMAT_H*/
