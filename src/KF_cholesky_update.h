//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: KF_cholesky_update.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 23-Sep-2019 13:55:32
//
#ifndef KF_CHOLESKY_UPDATE_H
#define KF_CHOLESKY_UPDATE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "slamCore_types.h"

// Function Declarations
extern void KF_cholesky_update(double x[500], double P[2500], const double
  v_data[], const int v_size[1], const emxArray_real_T *R, const emxArray_real_T
  *H, int length_x);

#endif

//
// File trailer for KF_cholesky_update.h
//
// [EOF]
//
