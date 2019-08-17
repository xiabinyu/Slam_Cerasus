//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xpotrf.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "xpotrf.h"

// Function Definitions

//
// Arguments    : emxArray_real_T *A
//                int lda
// Return Type  : int
//
int xpotrf(emxArray_real_T *A, int lda)
{
  int info;
  int n;
  int colj;
  int j;
  boolean_T exitg1;
  int jj;
  double ajj;
  int ix;
  int iy;
  int k;
  int nmj;
  int jjp1;
  int coljp1;
  int i7;
  double c;
  int ia;
  n = A->size[0];
  info = 0;
  if ((A->size[0] != 0) && (A->size[1] != 0)) {
    colj = 0;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j <= n - 1)) {
      jj = colj + j;
      ajj = 0.0;
      if (j >= 1) {
        ix = colj;
        iy = colj;
        for (k = 0; k < j; k++) {
          ajj += A->data[ix] * A->data[iy];
          ix++;
          iy++;
        }
      }

      ajj = A->data[jj] - ajj;
      if (ajj > 0.0) {
        ajj = std::sqrt(ajj);
        A->data[jj] = ajj;
        if (j + 1 < n) {
          nmj = (n - j) - 2;
          jjp1 = jj + n;
          coljp1 = (colj + n) + 1;
          if ((j == 0) || (nmj + 1 == 0)) {
          } else {
            iy = jjp1;
            i7 = coljp1 + lda * nmj;
            for (jj = coljp1; lda < 0 ? jj >= i7 : jj <= i7; jj += lda) {
              ix = colj;
              c = 0.0;
              k = (jj + j) - 1;
              for (ia = jj; ia <= k; ia++) {
                c += A->data[ia - 1] * A->data[ix];
                ix++;
              }

              A->data[iy] += -c;
              iy += lda;
            }
          }

          ajj = 1.0 / ajj;
          i7 = (jjp1 + n * nmj) + 1;
          for (k = jjp1 + 1; n < 0 ? k >= i7 : k <= i7; k += n) {
            A->data[k - 1] *= ajj;
          }

          colj = coljp1 - 1;
        }

        j++;
      } else {
        A->data[jj] = ajj;
        info = j + 1;
        exitg1 = true;
      }
    }
  }

  return info;
}

//
// File trailer for xpotrf.cpp
//
// [EOF]
//
