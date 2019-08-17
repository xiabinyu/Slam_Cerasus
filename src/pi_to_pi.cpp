//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pi_to_pi.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "pi_to_pi.h"

// Function Definitions

//
// function angle = pi_to_pi(angle)
//  Input: array of angles.
//  Tim Bailey 2000
// Arguments    : double *angle
// Return Type  : void
//
void pi_to_pi(double *angle)
{
  double x;
  boolean_T rEQ0;
  int ii_size_idx_0;
  int ii_size_idx_1;
  double q;
  double dv0[1];
  double dv1[1];
  x = *angle;
  if ((!rtIsInf(*angle)) && (!rtIsNaN(*angle))) {
    if (*angle == 0.0) {
      *angle = 0.0;
    } else {
      *angle = std::fmod(*angle, 6.2831853071795862);
      rEQ0 = (*angle == 0.0);
      if (!rEQ0) {
        q = std::abs(x / 6.2831853071795862);
        rEQ0 = (std::abs(q - std::floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        *angle = 0.0;
      } else {
        if (x < 0.0) {
          *angle += 6.2831853071795862;
        }
      }
    }
  } else {
    *angle = rtNaN;
  }

  if (*angle > 3.1415926535897931) {
    ii_size_idx_0 = 1;
    ii_size_idx_1 = 1;
  } else {
    ii_size_idx_0 = 0;
    ii_size_idx_1 = 0;
  }

  dv0[0] = *angle;
  ii_size_idx_0 = (signed char)ii_size_idx_0 * (signed char)ii_size_idx_1;
  for (ii_size_idx_1 = 0; ii_size_idx_1 < ii_size_idx_0; ii_size_idx_1++) {
    dv0[0] = *angle - 6.2831853071795862;
  }

  if (dv0[0] < -3.1415926535897931) {
    ii_size_idx_0 = 1;
    ii_size_idx_1 = 1;
  } else {
    ii_size_idx_0 = 0;
    ii_size_idx_1 = 0;
  }

  dv1[0] = dv0[0];
  ii_size_idx_0 = (signed char)ii_size_idx_0 * (signed char)ii_size_idx_1;
  for (ii_size_idx_1 = 0; ii_size_idx_1 < ii_size_idx_0; ii_size_idx_1++) {
    dv1[0] = dv0[0] + 6.2831853071795862;
  }

  *angle = dv1[0];
}

//
// File trailer for pi_to_pi.cpp
//
// [EOF]
//
