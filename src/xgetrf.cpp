//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgetrf.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "xgetrf.h"
#include "xswap.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                emxArray_real_T *A
//                int lda
//                int ipiv_data[]
//                int ipiv_size[2]
// Return Type  : void
//
void xgetrf(int m, int n, emxArray_real_T *A, int lda, int ipiv_data[], int
            ipiv_size[2])
{
  int yk;
  int b_n;
  int u0;
  int jA;
  int j;
  int mmj;
  int b_tmp;
  int jp1j;
  int ix;
  double smax;
  int i8;
  double s;
  int i9;
  int ijA;
  if (m < n) {
    yk = m;
  } else {
    yk = n;
  }

  if (yk < 1) {
    b_n = 0;
  } else {
    b_n = yk;
  }

  ipiv_size[0] = 1;
  ipiv_size[1] = b_n;
  if (b_n > 0) {
    ipiv_data[0] = 1;
    yk = 1;
    for (jA = 2; jA <= b_n; jA++) {
      yk++;
      ipiv_data[jA - 1] = yk;
    }
  }

  if ((m < 1) || (n < 1)) {
  } else {
    u0 = m - 1;
    if (u0 >= n) {
      u0 = n;
    }

    for (j = 0; j < u0; j++) {
      mmj = m - j;
      b_tmp = j * (lda + 1);
      jp1j = b_tmp + 2;
      if (mmj < 1) {
        yk = 0;
      } else {
        yk = 1;
        if (mmj > 1) {
          ix = b_tmp;
          smax = std::abs(A->data[b_tmp]);
          for (jA = 2; jA <= mmj; jA++) {
            ix++;
            s = std::abs(A->data[ix]);
            if (s > smax) {
              yk = jA;
              smax = s;
            }
          }
        }
      }

      if (A->data[(b_tmp + yk) - 1] != 0.0) {
        if (yk - 1 != 0) {
          ipiv_data[j] = j + yk;
          xswap(n, A, j + 1, lda, j + yk, lda);
        }

        i8 = (b_tmp + mmj) - 1;
        for (yk = jp1j; yk <= i8 + 1; yk++) {
          A->data[yk - 1] /= A->data[b_tmp];
        }
      }

      b_n = n - j;
      yk = b_tmp + lda;
      jA = (b_tmp + lda) + 1;
      for (jp1j = 0; jp1j <= b_n - 2; jp1j++) {
        smax = A->data[yk];
        if (A->data[yk] != 0.0) {
          ix = b_tmp;
          i8 = jA + 1;
          i9 = (mmj + jA) - 1;
          for (ijA = i8; ijA <= i9; ijA++) {
            A->data[ijA - 1] += A->data[ix + 1] * -smax;
            ix++;
          }
        }

        yk += lda;
        jA += lda;
      }
    }
  }
}

//
// File trailer for xgetrf.cpp
//
// [EOF]
//
