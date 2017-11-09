#ifndef ARRAYFUNCTIONS_H
#define ARRAYFUNCTIONS_H

#include "string.h" //memcpy
#include "stdlib.h"
#include "stdbool.h"

#ifndef ARM_COMPILER
#include "stdio.h"
#endif /*ARM_COMPILER*/

/* Structure holding values and indexes of an array, used for sorting while keeping original index*/
typedef struct indexed_array {
  float values;
  int index;
} indexed_array;

extern void  array_shiftleft(float *array, int size, int shift);
extern void  array_shiftright(float *array, int size, int shift);
extern void  array_shiftleft_bool(bool *array, int size, int shift);
extern void  array_shiftright_bool(bool *array, int size, int shift);
extern int   array_getminidx(int length, float *x);
extern int   array_getmaxidx(int length, float *x);
extern void  array_arraymin(int length, float *x1, float *x2);
extern void  array_arraymax(int length, float *x1, float *x2);
extern bool  array_find_int(int length, int *x, int value, int *location);
extern void  array_sortmintomax(int length, float *x);
extern void  array_sortmintomax_index(int length, struct indexed_array *x);

#ifndef ARM_COMPILER
extern void  array_print(int length, float *x);
extern void  array_print_int(int length, int *x);
extern void  array_print_bool(int length, bool *x);
#endif /*ARM_COMPILER*/

extern void  array_make_zeros(int length, float *x);
extern void  array_make_zeros_int(int length, int *x);
extern void  array_make_zeros_bool(int length, bool *x);
extern void  array_make_ones(int length, float *x);
extern void  array_mult_scal(int length, float *y, float k, float *x);
extern float array_sum(int length, float *x);
extern float array_sum_weighted(int length, float *x);
extern void  array_copy_bool(bool *array_out, bool *array_in, int size);

#endif  /*ARRAYFUNCTIONS_H*/