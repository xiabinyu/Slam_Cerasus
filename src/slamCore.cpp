//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: slamCore.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 23-Sep-2019 13:55:32
//

// Include Files
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "slamCore_emxutil.h"
#include "KF_cholesky_update.h"
#include "pi_to_pi.h"
#include "data_associate.h"
#include "slamCore_rtwutil.h"

// Function Declarations
static double rt_roundd_snf(double u);

// Function Definitions

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// Arguments    : double x[500]
//                double P[2500]
//                const double z_data[]
//                const int z_size[2]
//                int *length_x
// Return Type  : void
//
void slamCore(double x[500], double P[2500], const double z_data[], const int
              z_size[2], int *length_x)
{
  double zf[100];
  double zn[100];
  int idf[50];
  int n;
  int l_zn;
  int fpos;
  int qY;
  int Nf;
  int i4;
  int i;
  emxArray_real_T *H;
  int jbest;
  double nbest;
  double outer;
  int j;
  static const double R[4] = { 1.0E-6, 0.0, 0.0, 7.6154354946677152E-7 };

  double dy;
  double xd;
  int v_size[1];
  double v_data[100];
  emxArray_real_T *RR;
  emxArray_real_T *r0;
  signed char i5;
  signed char ii_idx_1;
  emxArray_int32_T *rng;
  emxArray_int32_T *rnm;
  emxArray_int32_T *r1;
  emxArray_int8_T *r2;
  emxArray_int32_T *r3;
  emxArray_real_T *b_P;
  emxArray_real_T *c_P;
  double Gv[6];
  double d;
  double yd;
  double xd2;
  double Gz[4];
  unsigned int nm1;
  int k;
  double b_Gv[4];
  double b_Gz[4];
  double c_Gv[6];
  double c_Gz[4];

  // 4.0
  // 25.0
  //  observation noises
  //  metres0.07
  //  radians
  //
  //  Simple gated nearest-neighbour data-association. No clever feature
  //  caching tricks to speed up association, so computation is O(N), where
  //  N is the number of features in the state.
  memset(&zf[0], 0, 100U * sizeof(double));
  memset(&zn[0], 0, 100U * sizeof(double));
  memset(&idf[0], 0, 50U * sizeof(int));

  //  zf= []; zn= [];
  //  idf=[];
  n = 0;
  l_zn = -1;

  //  number of vehicle pose states
  fpos = *length_x;
  if (fpos < -2147483645) {
    qY = MIN_int32_T;
  } else {
    qY = fpos - 3;
  }

  Nf = (int)rt_roundd_snf((double)qY / 2.0);

  //  number of features already in map
  //  linear search for nearest-neighbour, no clever tricks (like a quick
  //  bounding-box threshold to remove distant features; or, better yet,
  //  a balanced k-d tree lookup). TODO: implement clever tricks.
  i4 = z_size[1];
  for (i = 0; i < i4; i++) {
    jbest = 0;
    nbest = rtInf;

    // 初始定位正无穷
    outer = rtInf;

    //  search for neighbours
    for (j = 0; j < Nf; j++) {
      compute_association(x, P, *(double (*)[2])&z_data[i << 1], R, 1 + j,
                          *length_x, &dy, &xd);
      if ((dy < 50.0) && (xd < nbest)) {
        //  if within gate, store nearest-neighbour
        nbest = xd;
        jbest = 1 + j;
      } else {
        if (dy < outer) {
          //  else store best nis value
          outer = dy;
        }
      }

      //          if jbest ~= 0 %若Nf=0即此时x中只有小车的位置信息，则估计的值zn为测量值z 
      //              zf=  [zf  z(:,i)];
      //              idf= [idf jbest];
      //          end
    }

    //      if jbest == 0 && outer > gate2 % z too far to associate, but far enough to be a new feature 
    //          zn= [zn z(:,i)];
    //      end
    //  add nearest-neighbour to association list
    if (jbest != 0) {
      // 若Nf=0即此时x中只有小车的位置信息，则估计的值zn为测量值z
      n++;

      // disp(count)
      // zf=[zf z(:,i)]
      fpos = (n - 1) << 1;
      zf[fpos] = z_data[i << 1];
      zf[1 + fpos] = z_data[1 + (i << 1)];
      idf[n - 1] = jbest;
    } else {
      if (outer > 150.0) {
        //  z too far to associate, but far enough to be a new feature
        l_zn++;
        fpos = i << 1;
        jbest = l_zn << 1;
        zn[jbest] = z_data[fpos];
        zn[1 + jbest] = z_data[1 + fpos];
      }
    }
  }

  emxInit_real_T(&H, 2);

  //  function [x,P]= update(x,P,z,R,idf, batch)
  //
  //  Inputs:
  //    x, P - SLAM state and covariance
  //    z, R - range-bearing measurements and covariances
  //    idf - feature index for each z
  //    batch - switch to specify whether to process measurements together or sequentially 
  //
  //  Outputs:
  //    x, P - updated state and covariance
  //
  //
  i4 = H->size[0] * H->size[1];
  jbest = n << 1;
  H->size[0] = jbest;
  H->size[1] = *length_x;
  emxEnsureCapacity_real_T(H, i4);
  fpos = jbest * *length_x;
  for (i4 = 0; i4 < fpos; i4++) {
    H->data[i4] = 0.0;
  }

  v_size[0] = jbest;
  if (0 <= jbest - 1) {
    memset(&v_data[0], 0, (unsigned int)(jbest * (int)sizeof(double)));
  }

  emxInit_real_T(&RR, 2);
  i4 = RR->size[0] * RR->size[1];
  RR->size[0] = jbest;
  RR->size[1] = jbest;
  emxEnsureCapacity_real_T(RR, i4);
  jbest *= jbest;
  for (i4 = 0; i4 < jbest; i4++) {
    RR->data[i4] = 0.0;
  }

  emxInit_real_T(&r0, 2);
  for (i = 0; i < n; i++) {
    i5 = (signed char)((signed char)((signed char)(1 + i) << 1) - 1);
    ii_idx_1 = (signed char)(1 + i5);

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
    fpos = (idf[i] << 1) + 1;

    //  position of xf in state idf对应路标的坐标信息处于矩阵x的第几个位置
    i4 = r0->size[0] * r0->size[1];
    r0->size[0] = 2;
    r0->size[1] = *length_x;
    emxEnsureCapacity_real_T(r0, i4);
    jbest = *length_x << 1;
    for (i4 = 0; i4 < jbest; i4++) {
      r0->data[i4] = 0.0;
    }

    //  auxiliary values
    outer = x[fpos] - x[0];
    dy = x[fpos + 1] - x[1];
    nbest = outer * outer + dy * dy;
    d = std::sqrt(nbest);
    xd = outer / d;
    yd = dy / d;
    xd2 = outer / nbest;
    nbest = dy / nbest;

    //  predict z
    //  calculate H
    r0->data[0] = -xd;
    r0->data[2] = -yd;
    r0->data[4] = 0.0;
    r0->data[1] = nbest;
    r0->data[3] = -xd2;
    r0->data[5] = -1.0;
    i4 = fpos << 1;
    r0->data[i4] = xd;
    Nf = (1 + fpos) << 1;
    r0->data[Nf] = yd;
    r0->data[1 + i4] = -nbest;
    r0->data[1 + Nf] = xd2;
    jbest = r0->size[1];
    for (i4 = 0; i4 < jbest; i4++) {
      Nf = i4 << 1;
      H->data[(i5 + H->size[0] * i4) - 1] = r0->data[Nf];
      H->data[(ii_idx_1 + H->size[0] * i4) - 1] = r0->data[1 + Nf];
    }

    i4 = i << 1;
    xd = zf[1 + i4] - (rt_atan2d_snf(dy, outer) - x[2]);
    pi_to_pi(&xd);
    jbest = i5 - 1;
    v_data[jbest] = zf[i4] - d;
    fpos = ii_idx_1 - 1;
    v_data[fpos] = xd;
    RR->data[(i5 + RR->size[0] * jbest) - 1] = 1.0E-6;
    RR->data[(ii_idx_1 + RR->size[0] * (i5 - 1)) - 1] = 0.0;
    RR->data[(i5 + RR->size[0] * fpos) - 1] = 0.0;
    RR->data[(ii_idx_1 + RR->size[0] * (ii_idx_1 - 1)) - 1] =
      7.6154354946677152E-7;
  }

  KF_cholesky_update(x, P, v_data, v_size, RR, H, *length_x);

  // function [x,P]= augment(x,P,z,R)
  //
  //  Inputs:
  //    x, P - SLAM state and covariance
  //    z, R - range-bearing measurements and covariances, each of a new feature 
  //
  //  Outputs:
  //    x, P - augmented state and covariance
  //
  //  Notes:
  //    - We assume the number of vehicle pose states is three.
  //    - Only one value for R is used, as all measurements are assumed to
  //    have same noise properties.
  //
  //  Tim Bailey 2004.
  //  add new features to state
  emxFree_real_T(&RR);
  emxFree_real_T(&H);
  emxInit_int32_T(&rng, 2);
  emxInit_int32_T(&rnm, 2);
  emxInit_int32_T(&r1, 1);
  emxInit_int8_T(&r2, 1);
  emxInit_int32_T(&r3, 1);
  emxInit_real_T(&b_P, 2);
  emxInit_real_T(&c_P, 2);
  if (0 <= l_zn) {
    Gv[0] = 1.0;
    Gv[2] = 0.0;
    Gv[1] = 0.0;
    Gv[3] = 1.0;
  }

  for (i = 0; i <= l_zn; i++) {
    //
    //
    i4 = i << 1;
    nbest = x[2] + zn[1 + i4];
    yd = std::sin(nbest);
    nbest = std::cos(nbest);

    //  augment x
    if (*length_x > 2147483646) {
      qY = MAX_int32_T;
    } else {
      qY = *length_x + 1;
    }

    xd = zn[i4] * nbest;
    x[qY - 1] = x[0] + xd;
    if (*length_x > 2147483645) {
      qY = MAX_int32_T;
    } else {
      qY = *length_x + 2;
    }

    x[qY - 1] = x[1] + zn[i4] * yd;
    if (*length_x > 2147483645) {
      qY = MAX_int32_T;
    } else {
      qY = *length_x + 2;
    }

    //  jacobians
    outer = -zn[i4] * yd;
    Gv[4] = outer;
    Gv[5] = xd;
    Gz[0] = nbest;
    Gz[2] = outer;
    Gz[1] = yd;
    Gz[3] = xd;

    //  augment P
    if (*length_x > 2147483646) {
      jbest = MAX_int32_T;
    } else {
      jbest = *length_x + 1;
    }

    if (*length_x > 2147483645) {
      fpos = MAX_int32_T;
    } else {
      fpos = *length_x + 2;
    }

    if (fpos < jbest) {
      n = 0;
    } else {
      if ((jbest < 0) && (fpos >= 0)) {
        nm1 = (unsigned int)fpos - jbest;
      } else {
        nm1 = (unsigned int)(fpos - jbest);
      }

      n = (int)nm1 + 1;
    }

    i4 = rng->size[0] * rng->size[1];
    rng->size[0] = 1;
    rng->size[1] = n;
    emxEnsureCapacity_int32_T(rng, i4);
    if (n > 0) {
      rng->data[0] = jbest;
      for (k = 2; k <= n; k++) {
        jbest++;
        rng->data[k - 1] = jbest;
      }
    }

    i4 = r3->size[0];
    r3->size[0] = rng->size[1];
    emxEnsureCapacity_int32_T(r3, i4);
    jbest = rng->size[1];
    for (i4 = 0; i4 < jbest; i4++) {
      r3->data[i4] = rng->data[i4];
    }

    i4 = r1->size[0];
    r1->size[0] = r3->size[0];
    emxEnsureCapacity_int32_T(r1, i4);
    jbest = r3->size[0];
    for (i4 = 0; i4 < jbest; i4++) {
      r1->data[i4] = r3->data[i4] - 1;
    }

    i4 = r3->size[0];
    emxEnsureCapacity_int32_T(r3, i4);
    jbest = r3->size[0];
    for (i4 = 0; i4 < jbest; i4++) {
      r3->data[i4]--;
    }

    for (i4 = 0; i4 < 2; i4++) {
      for (Nf = 0; Nf < 3; Nf++) {
        fpos = i4 + (Nf << 1);
        c_Gv[fpos] = 0.0;
        c_Gv[fpos] = (Gv[i4] * P[50 * Nf] + Gv[i4 + 2] * P[1 + 50 * Nf]) + Gv[i4
          + 4] * P[2 + 50 * Nf];
      }

      for (Nf = 0; Nf < 2; Nf++) {
        fpos = Nf << 1;
        jbest = i4 + fpos;
        c_Gz[jbest] = 0.0;
        c_Gz[jbest] = Gz[i4] * R[fpos] + Gz[i4 + 2] * R[1 + fpos];
        b_Gv[jbest] = 0.0;
        b_Gv[jbest] = (c_Gv[i4] * Gv[Nf] + c_Gv[i4 + 2] * Gv[Nf + 2]) + c_Gv[i4
          + 4] * Gv[Nf + 4];
      }

      b_Gz[i4] = 0.0;
      dy = c_Gz[i4 + 2];
      b_Gz[i4] = c_Gz[i4] * nbest + dy * outer;
      b_Gz[i4 + 2] = 0.0;
      b_Gz[i4 + 2] = c_Gz[i4] * yd + dy * xd;
    }

    b_Gv[0] += b_Gz[0];
    b_Gv[1] += b_Gz[1];
    b_Gv[2] += b_Gz[2];
    b_Gv[3] += b_Gz[3];
    fpos = r1->size[0];
    jbest = r3->size[0];
    for (i4 = 0; i4 < jbest; i4++) {
      for (Nf = 0; Nf < fpos; Nf++) {
        P[r1->data[Nf] + 50 * r3->data[i4]] = b_Gv[Nf + fpos * i4];
      }
    }

    //  feature cov
    for (i4 = 0; i4 < 2; i4++) {
      for (Nf = 0; Nf < 3; Nf++) {
        fpos = i4 + (Nf << 1);
        c_Gv[fpos] = 0.0;
        c_Gv[fpos] = (Gv[i4] * P[50 * Nf] + Gv[i4 + 2] * P[1 + 50 * Nf]) + Gv[i4
          + 4] * P[2 + 50 * Nf];
      }
    }

    fpos = r3->size[0];
    for (i4 = 0; i4 < 3; i4++) {
      for (Nf = 0; Nf < fpos; Nf++) {
        P[r3->data[Nf] + 50 * i4] = c_Gv[Nf + fpos * i4];
      }
    }

    //  vehicle to feature xcorr
    i4 = b_P->size[0] * b_P->size[1];
    b_P->size[0] = 3;
    b_P->size[1] = rng->size[1];
    emxEnsureCapacity_real_T(b_P, i4);
    jbest = rng->size[1];
    for (i4 = 0; i4 < jbest; i4++) {
      b_P->data[3 * i4] = P[rng->data[i4] - 1];
      b_P->data[1 + 3 * i4] = P[rng->data[i4] + 49];
      b_P->data[2 + 3 * i4] = P[rng->data[i4] + 99];
    }

    jbest = b_P->size[1];
    for (i4 = 0; i4 < jbest; i4++) {
      P[50 * r3->data[i4]] = b_P->data[3 * i4];
      P[1 + 50 * r3->data[i4]] = b_P->data[1 + 3 * i4];
      P[2 + 50 * r3->data[i4]] = b_P->data[2 + 3 * i4];
    }

    if (qY > 3) {
      if (*length_x < 4) {
        n = 0;
      } else {
        n = *length_x - 3;
      }

      i4 = rnm->size[0] * rnm->size[1];
      rnm->size[0] = 1;
      rnm->size[1] = n;
      emxEnsureCapacity_int32_T(rnm, i4);
      if (n > 0) {
        rnm->data[0] = 4;
        jbest = 4;
        for (k = 2; k <= n; k++) {
          jbest++;
          rnm->data[k - 1] = jbest;
        }
      }

      i4 = r1->size[0];
      r1->size[0] = rnm->size[1];
      emxEnsureCapacity_int32_T(r1, i4);
      jbest = rnm->size[1];
      for (i4 = 0; i4 < jbest; i4++) {
        r1->data[i4] = rnm->data[i4];
      }

      i4 = r2->size[0];
      r2->size[0] = r1->size[0];
      emxEnsureCapacity_int8_T(r2, i4);
      jbest = r1->size[0];
      for (i4 = 0; i4 < jbest; i4++) {
        r2->data[i4] = (signed char)((signed char)r1->data[i4] - 1);
      }

      n = r1->size[0];
      i4 = r0->size[0] * r0->size[1];
      r0->size[0] = 2;
      r0->size[1] = r1->size[0];
      emxEnsureCapacity_real_T(r0, i4);
      for (j = 0; j < n; j++) {
        fpos = j << 1;
        jbest = j * 3;
        r0->data[fpos] = 0.0;
        r0->data[fpos + 1] = 0.0;
        for (k = 0; k < 3; k++) {
          Nf = k << 1;
          i4 = jbest + k;
          nbest = P[i4 % 3 + 50 * (r1->data[i4 / 3] - 1)];
          r0->data[fpos] += nbest * Gv[Nf];
          r0->data[fpos + 1] += nbest * Gv[Nf + 1];
        }
      }

      fpos = r3->size[0];
      jbest = r2->size[0];
      for (i4 = 0; i4 < jbest; i4++) {
        for (Nf = 0; Nf < fpos; Nf++) {
          P[r3->data[Nf] + 50 * r2->data[i4]] = r0->data[Nf + fpos * i4];
        }
      }

      //  map to feature xcorr
      i4 = c_P->size[0] * c_P->size[1];
      c_P->size[0] = rnm->size[1];
      c_P->size[1] = rng->size[1];
      emxEnsureCapacity_real_T(c_P, i4);
      jbest = rng->size[1];
      for (i4 = 0; i4 < jbest; i4++) {
        fpos = rnm->size[1];
        for (Nf = 0; Nf < fpos; Nf++) {
          c_P->data[Nf + c_P->size[0] * i4] = P[(rng->data[i4] + 50 * (rnm->
            data[Nf] - 1)) - 1];
        }
      }

      jbest = c_P->size[1];
      for (i4 = 0; i4 < jbest; i4++) {
        fpos = c_P->size[0];
        for (Nf = 0; Nf < fpos; Nf++) {
          P[(rnm->data[Nf] + 50 * r3->data[i4]) - 1] = c_P->data[Nf + c_P->size
            [0] * i4];
        }
      }
    }

    *length_x = qY;
  }

  emxFree_real_T(&c_P);
  emxFree_real_T(&b_P);
  emxFree_int32_T(&r3);
  emxFree_int8_T(&r2);
  emxFree_int32_T(&r1);
  emxFree_int32_T(&rnm);
  emxFree_int32_T(&rng);
  emxFree_real_T(&r0);

  // whos z
}

//
// File trailer for slamCore.cpp
//
// [EOF]
//
