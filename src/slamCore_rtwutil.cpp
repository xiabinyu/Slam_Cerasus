//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: slamCore_rtwutil.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 23-Sep-2019 13:55:32
//

// Include Files
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "slamCore_rtwutil.h"

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// File trailer for slamCore_rtwutil.cpp
//
// [EOF]
//
