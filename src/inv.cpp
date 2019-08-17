//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//

// Include Files
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "inv.h"
#include "slamCore_emxutil.h"
#include "xtrsm.h"
#include "xgetrf.h"

// Function Declarations
static void eml_ipiv2perm(const int ipiv_data[], const int ipiv_size[2], int m,
  int perm_data[], int perm_size[2]);
static void invNxN(const emxArray_real_T *x, emxArray_real_T *y);

// Function Definitions

//
// Arguments    : const int ipiv_data[]
//                const int ipiv_size[2]
//                int m
//                int perm_data[]
//                int perm_size[2]
// Return Type  : void
//
static void eml_ipiv2perm(const int ipiv_data[], const int ipiv_size[2], int m,
  int perm_data[], int perm_size[2])
{
  int n;
  int yk;
  int k;
  if (m < 1) {
    n = 0;
  } else {
    n = m;
  }

  perm_size[0] = 1;
  perm_size[1] = n;
  if (n > 0) {
    perm_data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      perm_data[k - 1] = yk;
    }
  }

  n = ipiv_size[1];
  for (k = 0; k < n; k++) {
    if (ipiv_data[k] > 1 + k) {
      yk = perm_data[ipiv_data[k] - 1];
      perm_data[ipiv_data[k] - 1] = perm_data[k];
      perm_data[k] = yk;
    }
  }
}

//
// Arguments    : const emxArray_real_T *x
//                emxArray_real_T *y
// Return Type  : void
//
static void invNxN(const emxArray_real_T *x, emxArray_real_T *y)
{
  int n;
  int i2;
  int loop_ub;
  emxArray_real_T *b_x;
  int ipiv_data[100];
  int ipiv_size[2];
  int p_data[100];
  int p_size[2];
  int j;
  int i3;
  int i;
  n = x->size[0];
  i2 = y->size[0] * y->size[1];
  y->size[0] = x->size[0];
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, i2);
  loop_ub = x->size[0] * x->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    y->data[i2] = 0.0;
  }

  emxInit_real_T(&b_x, 2);
  i2 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[0];
  b_x->size[1] = x->size[1];
  emxEnsureCapacity_real_T(b_x, i2);
  loop_ub = x->size[0] * x->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    b_x->data[i2] = x->data[i2];
  }

  xgetrf(x->size[0], x->size[0], b_x, x->size[0], ipiv_data, ipiv_size);
  eml_ipiv2perm(ipiv_data, ipiv_size, x->size[0], p_data, p_size);
  for (loop_ub = 0; loop_ub < n; loop_ub++) {
    i2 = p_data[loop_ub];
    y->data[loop_ub + y->size[0] * (p_data[loop_ub] - 1)] = 1.0;
    for (j = loop_ub + 1; j <= n; j++) {
      if (y->data[(j + y->size[0] * (i2 - 1)) - 1] != 0.0) {
        i3 = j + 1;
        for (i = i3; i <= n; i++) {
          y->data[(i + y->size[0] * (i2 - 1)) - 1] -= y->data[(j + y->size[0] *
            (i2 - 1)) - 1] * b_x->data[(i + b_x->size[0] * (j - 1)) - 1];
        }
      }
    }
  }

  xtrsm(x->size[0], x->size[0], b_x, x->size[0], y, x->size[0]);
  emxFree_real_T(&b_x);
}

//
// Arguments    : const emxArray_real_T *x
//                emxArray_real_T *y
// Return Type  : void
//
void inv(const emxArray_real_T *x, emxArray_real_T *y)
{
  int i1;
  int loop_ub;
  if ((x->size[0] == 0) || (x->size[1] == 0)) {
    i1 = y->size[0] * y->size[1];
    y->size[0] = x->size[0];
    y->size[1] = x->size[1];
    emxEnsureCapacity_real_T(y, i1);
    loop_ub = x->size[0] * x->size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      y->data[i1] = x->data[i1];
    }
  } else {
    invNxN(x, y);
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
