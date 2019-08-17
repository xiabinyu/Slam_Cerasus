//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xswap.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//

// Include Files
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "xswap.h"

// Function Definitions

//
// Arguments    : int n
//                emxArray_real_T *x
//                int ix0
//                int incx
//                int iy0
//                int incy
// Return Type  : void
//
void xswap(int n, emxArray_real_T *x, int ix0, int incx, int iy0, int incy)
{
  int ix;
  int iy;
  int k;
  double temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < n; k++) {
    temp = x->data[ix];
    x->data[ix] = x->data[iy];
    x->data[iy] = temp;
    ix += incx;
    iy += incy;
  }
}

//
// File trailer for xswap.cpp
//
// [EOF]
//
