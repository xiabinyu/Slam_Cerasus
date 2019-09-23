//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xtrsm.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 23-Sep-2019 13:55:32
//

// Include Files
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "xtrsm.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                const emxArray_real_T *A
//                int lda
//                emxArray_real_T *B
//                int ldb
// Return Type  : void
//
void xtrsm(int m, int n, const emxArray_real_T *A, int lda, emxArray_real_T *B,
           int ldb)
{
  int j;
  int jBcol;
  int k;
  int kAcol;
  int i10;
  int i;
  int i11;
  if ((n == 0) || ((B->size[0] == 0) || (B->size[1] == 0))) {
  } else {
    for (j = 0; j < n; j++) {
      jBcol = ldb * j - 1;
      for (k = m; k >= 1; k--) {
        kAcol = lda * (k - 1) - 1;
        i10 = k + jBcol;
        if (B->data[i10] != 0.0) {
          B->data[i10] /= A->data[k + kAcol];
          for (i = 0; i <= k - 2; i++) {
            i11 = (i + jBcol) + 1;
            B->data[i11] -= B->data[i10] * A->data[(i + kAcol) + 1];
          }
        }
      }
    }
  }
}

//
// File trailer for xtrsm.cpp
//
// [EOF]
//
