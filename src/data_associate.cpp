//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: data_associate.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "data_associate.h"
#include "slamCore_emxutil.h"
#include "pi_to_pi.h"
#include "slamCore_rtwutil.h"

// Function Definitions

//
// return normalised innovation squared (ie, Mahalanobis distance) and normalised distance
// Arguments    : const double x[500]
//                const double P[2500]
//                const double z[2]
//                const double R[4]
//                int idf
//                int length_x
//                double *nis
//                double *nd
// Return Type  : void
//
void compute_association(const double x[500], const double P[2500], const double
  z[2], const double R[4], int idf, int length_x, double *nis, double *nd)
{
  emxArray_real_T *H;
  int fpos;
  int i0;
  int loop_ub;
  double dx;
  double dy;
  double d2;
  double d;
  double xd;
  double yd;
  double xd2;
  double yd2;
  double v[2];
  int b_loop_ub;
  int j;
  int coffset;
  emxArray_real_T *b;
  int boffset;
  double P_data[2500];
  double y_data[100];
  int k;
  int aoffset;
  double S[4];
  double y[4];
  signed char ipiv_idx_0;
  boolean_T isodd;
  emxInit_real_T(&H, 2);

  // function [z,H]= observe_model(x, idf)
  //
  //  INPUTS:
  //    x - state vector
  //    idf - index of feature order in state
  //
  //  OUTPUTS:
  //    z - predicted observation
  //    H - observation Jacobian
  //
  //  Given a feature index (ie, the order of the feature in the state vector),
  //  predict the expected range-bearing observation of this feature and its Jacobian. 
  //
  //  Tim Bailey 2004.
  //  number of vehicle pose states
  if (idf > 1073741823) {
    fpos = MAX_int32_T;
  } else if (idf <= -1073741824) {
    fpos = MIN_int32_T;
  } else {
    fpos = idf << 1;
  }

  if (fpos > 2147483644) {
    fpos = MAX_int32_T;
  } else {
    fpos += 3;
  }

  fpos -= 2;

  //  position of xf in state idf对应路标的坐标信息处于矩阵x的第几个位置
  i0 = H->size[0] * H->size[1];
  H->size[0] = 2;
  H->size[1] = length_x;
  emxEnsureCapacity_real_T(H, i0);
  loop_ub = length_x << 1;
  for (i0 = 0; i0 < loop_ub; i0++) {
    H->data[i0] = 0.0;
  }

  //  auxiliary values
  dx = x[fpos] - x[0];
  dy = x[fpos + 1] - x[1];
  d2 = dx * dx + dy * dy;
  d = std::sqrt(d2);
  xd = dx / d;
  yd = dy / d;
  xd2 = dx / d2;
  yd2 = dy / d2;

  //  predict z
  //  calculate H
  H->data[0] = -xd;
  H->data[2] = -yd;
  H->data[4] = 0.0;
  H->data[1] = yd2;
  H->data[3] = -xd2;
  H->data[5] = -1.0;
  i0 = fpos << 1;
  H->data[i0] = xd;
  fpos = (1 + fpos) << 1;
  H->data[fpos] = yd;
  H->data[1 + i0] = -yd2;
  H->data[1 + fpos] = xd2;
  v[0] = z[0] - d;
  v[1] = z[1] - (rt_atan2d_snf(dy, dx) - x[2]);
  pi_to_pi(&v[1]);
  if (1 > length_x) {
    loop_ub = 0;
    b_loop_ub = 0;
  } else {
    loop_ub = length_x;
    b_loop_ub = length_x;
  }

  if ((H->size[1] == 1) || (loop_ub == 1)) {
    for (i0 = 0; i0 < b_loop_ub; i0++) {
      for (fpos = 0; fpos < loop_ub; fpos++) {
        P_data[fpos + loop_ub * i0] = P[fpos + 50 * i0];
      }
    }

    for (i0 = 0; i0 < b_loop_ub; i0++) {
      j = i0 << 1;
      y_data[j] = 0.0;
      coffset = H->size[1];
      for (fpos = 0; fpos < coffset; fpos++) {
        y_data[j] += H->data[fpos << 1] * P_data[fpos + loop_ub * i0];
      }
    }

    for (i0 = 0; i0 < b_loop_ub; i0++) {
      j = 1 + (i0 << 1);
      y_data[j] = 0.0;
      coffset = H->size[1];
      for (fpos = 0; fpos < coffset; fpos++) {
        y_data[j] += H->data[1 + (fpos << 1)] * P_data[fpos + loop_ub * i0];
      }
    }
  } else {
    fpos = H->size[1];
    for (j = 0; j < b_loop_ub; j++) {
      coffset = j << 1;
      boffset = j * fpos;
      y_data[coffset] = 0.0;
      y_data[coffset + 1] = 0.0;
      for (k = 0; k < fpos; k++) {
        aoffset = k << 1;
        i0 = boffset + k;
        d2 = P[i0 % loop_ub + 50 * (i0 / loop_ub)];
        y_data[coffset] += d2 * H->data[aoffset];
        y_data[coffset + 1] += d2 * H->data[aoffset + 1];
      }
    }
  }

  emxInit_real_T(&b, 2);
  i0 = b->size[0] * b->size[1];
  b->size[0] = H->size[1];
  b->size[1] = 2;
  emxEnsureCapacity_real_T(b, i0);
  loop_ub = H->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b->data[i0] = H->data[i0 << 1];
  }

  loop_ub = H->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    b->data[i0 + b->size[0]] = H->data[1 + (i0 << 1)];
  }

  emxFree_real_T(&H);
  if ((b_loop_ub == 1) || (b->size[0] == 1)) {
    S[0] = 0.0;
    for (i0 = 0; i0 < b_loop_ub; i0++) {
      S[0] += y_data[i0 << 1] * b->data[i0];
    }

    S[2] = 0.0;
    for (i0 = 0; i0 < b_loop_ub; i0++) {
      S[2] += y_data[i0 << 1] * b->data[i0 + b->size[0]];
    }

    S[1] = 0.0;
    for (i0 = 0; i0 < b_loop_ub; i0++) {
      S[1] += y_data[1 + (i0 << 1)] * b->data[i0];
    }

    S[3] = 0.0;
    for (i0 = 0; i0 < b_loop_ub; i0++) {
      S[3] += y_data[1 + (i0 << 1)] * b->data[i0 + b->size[0]];
    }
  } else {
    S[0] = 0.0;
    S[1] = 0.0;
    for (k = 0; k < b_loop_ub; k++) {
      fpos = k << 1;
      d2 = b->data[k];
      S[0] += d2 * y_data[fpos];
      S[1] += d2 * y_data[fpos + 1];
    }

    S[2] = 0.0;
    S[3] = 0.0;
    for (k = 0; k < b_loop_ub; k++) {
      i0 = k << 1;
      d2 = b->data[b_loop_ub + k];
      S[2] += d2 * y_data[i0];
      S[3] += d2 * y_data[i0 + 1];
    }
  }

  emxFree_real_T(&b);
  S[0] += R[0];
  S[1] += R[1];
  S[2] += R[2];
  S[3] += R[3];
  dx = std::abs(S[1]);
  dy = std::abs(S[0]);
  if (dx > dy) {
    d2 = S[0] / S[1];
    yd2 = 1.0 / (d2 * S[3] - S[2]);
    y[0] = S[3] / S[1] * yd2;
    y[1] = -yd2;
    y[2] = -S[2] / S[1] * yd2;
    y[3] = d2 * yd2;
  } else {
    d2 = S[1] / S[0];
    yd2 = 1.0 / (S[3] - d2 * S[2]);
    y[0] = S[3] / S[0] * yd2;
    y[1] = -d2 * yd2;
    y[2] = -S[2] / S[0] * yd2;
    y[3] = yd2;
  }

  ipiv_idx_0 = 1;
  *nis = (v[0] * y[0] + v[1] * y[1]) * v[0] + (v[0] * y[2] + v[1] * y[3]) * v[1];
  fpos = 0;
  if (dx > dy) {
    fpos = 1;
  }

  if (S[fpos] != 0.0) {
    if (fpos != 0) {
      ipiv_idx_0 = 2;
      d2 = S[0];
      S[0] = S[1];
      S[1] = d2;
      d2 = S[2];
      S[2] = S[3];
      S[3] = d2;
    }

    S[1] /= S[0];
  }

  if (S[2] != 0.0) {
    S[3] += S[1] * -S[2];
  }

  d2 = S[0] * S[3];
  isodd = false;
  if (ipiv_idx_0 > 1) {
    isodd = true;
  }

  if (isodd) {
    d2 = -d2;
  }

  *nd = *nis + std::log(d2);
}

//
// File trailer for data_associate.cpp
//
// [EOF]
//
