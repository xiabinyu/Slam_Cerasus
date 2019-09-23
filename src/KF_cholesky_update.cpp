//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: KF_cholesky_update.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 23-Sep-2019 13:55:32
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "slamCore.h"
#include "KF_cholesky_update.h"
#include "slamCore_emxutil.h"
#include "inv.h"
#include "xpotrf.h"

// Function Definitions

//
// Arguments    : double x[500]
//                double P[2500]
//                const double v_data[]
//                const int v_size[1]
//                const emxArray_real_T *R
//                const emxArray_real_T *H
//                int length_x
// Return Type  : void
//
void KF_cholesky_update(double x[500], double P[2500], const double v_data[],
  const int v_size[1], const emxArray_real_T *R, const emxArray_real_T *H, int
  length_x)
{
  int loop_ub;
  int b_loop_ub;
  emxArray_real_T *y;
  emxArray_real_T *a;
  int i6;
  emxArray_real_T *b;
  int coffset;
  int boffset;
  int c_loop_ub;
  int n;
  int j;
  emxArray_real_T *S;
  int i;
  int m;
  int inner;
  int aoffset;
  double temp;
  emxArray_real_T *SCholInv;
  double tmp_data[50];
  signed char b_tmp_data[500];
  double x_data[50];
  double P_data[2500];
  if (1 > length_x) {
    loop_ub = 0;
    b_loop_ub = 0;
  } else {
    loop_ub = length_x;
    b_loop_ub = length_x;
  }

  emxInit_real_T(&y, 2);
  emxInit_real_T(&a, 2);
  i6 = a->size[0] * a->size[1];
  a->size[0] = loop_ub;
  a->size[1] = b_loop_ub;
  emxEnsureCapacity_real_T(a, i6);
  for (i6 = 0; i6 < b_loop_ub; i6++) {
    for (coffset = 0; coffset < loop_ub; coffset++) {
      a->data[coffset + a->size[0] * i6] = P[coffset + 50 * i6];
    }
  }

  emxInit_real_T(&b, 2);
  i6 = b->size[0] * b->size[1];
  b->size[0] = H->size[1];
  b->size[1] = H->size[0];
  emxEnsureCapacity_real_T(b, i6);
  boffset = H->size[0];
  for (i6 = 0; i6 < boffset; i6++) {
    c_loop_ub = H->size[1];
    for (coffset = 0; coffset < c_loop_ub; coffset++) {
      b->data[coffset + b->size[0] * i6] = H->data[i6 + H->size[0] * coffset];
    }
  }

  if ((b_loop_ub == 1) || (b->size[0] == 1)) {
    i6 = y->size[0] * y->size[1];
    y->size[0] = a->size[0];
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(y, i6);
    loop_ub = a->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_loop_ub = b->size[1];
      for (coffset = 0; coffset < b_loop_ub; coffset++) {
        y->data[i6 + y->size[0] * coffset] = 0.0;
        boffset = a->size[1];
        for (aoffset = 0; aoffset < boffset; aoffset++) {
          y->data[i6 + y->size[0] * coffset] += a->data[i6 + a->size[0] *
            aoffset] * b->data[aoffset + b->size[0] * coffset];
        }
      }
    }
  } else {
    n = b->size[1];
    i6 = y->size[0] * y->size[1];
    y->size[0] = loop_ub;
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(y, i6);
    for (j = 0; j < n; j++) {
      coffset = j * loop_ub;
      boffset = j * b_loop_ub;
      for (i = 0; i < loop_ub; i++) {
        y->data[coffset + i] = 0.0;
      }

      for (c_loop_ub = 0; c_loop_ub < b_loop_ub; c_loop_ub++) {
        aoffset = c_loop_ub * loop_ub;
        temp = b->data[boffset + c_loop_ub];
        for (i = 0; i < loop_ub; i++) {
          y->data[coffset + i] += temp * a->data[aoffset + i];
        }
      }
    }
  }

  emxFree_real_T(&a);
  emxInit_real_T(&S, 2);
  if ((H->size[1] == 1) || (y->size[0] == 1)) {
    i6 = S->size[0] * S->size[1];
    S->size[0] = H->size[0];
    S->size[1] = y->size[1];
    emxEnsureCapacity_real_T(S, i6);
    loop_ub = H->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_loop_ub = y->size[1];
      for (coffset = 0; coffset < b_loop_ub; coffset++) {
        S->data[i6 + S->size[0] * coffset] = 0.0;
        boffset = H->size[1];
        for (aoffset = 0; aoffset < boffset; aoffset++) {
          S->data[i6 + S->size[0] * coffset] += H->data[i6 + H->size[0] *
            aoffset] * y->data[aoffset + y->size[0] * coffset];
        }
      }
    }
  } else {
    m = H->size[0];
    inner = H->size[1];
    n = y->size[1];
    i6 = S->size[0] * S->size[1];
    S->size[0] = H->size[0];
    S->size[1] = y->size[1];
    emxEnsureCapacity_real_T(S, i6);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        S->data[coffset + i] = 0.0;
      }

      for (c_loop_ub = 0; c_loop_ub < inner; c_loop_ub++) {
        aoffset = c_loop_ub * m;
        temp = y->data[boffset + c_loop_ub];
        for (i = 0; i < m; i++) {
          i6 = coffset + i;
          S->data[i6] += temp * H->data[aoffset + i];
        }
      }
    }
  }

  i6 = S->size[0] * S->size[1];
  coffset = S->size[0] * S->size[1];
  emxEnsureCapacity_real_T(S, coffset);
  loop_ub = i6 - 1;
  for (i6 = 0; i6 <= loop_ub; i6++) {
    S->data[i6] += R->data[i6];
  }

  emxInit_real_T(&SCholInv, 2);
  i6 = SCholInv->size[0] * SCholInv->size[1];
  SCholInv->size[0] = S->size[0];
  SCholInv->size[1] = S->size[1];
  emxEnsureCapacity_real_T(SCholInv, i6);
  loop_ub = S->size[1];
  for (i6 = 0; i6 < loop_ub; i6++) {
    b_loop_ub = S->size[0];
    for (coffset = 0; coffset < b_loop_ub; coffset++) {
      SCholInv->data[coffset + SCholInv->size[0] * i6] = (S->data[coffset +
        S->size[0] * i6] + S->data[i6 + S->size[0] * coffset]) * 0.5;
    }
  }

  i6 = S->size[0] * S->size[1];
  S->size[0] = SCholInv->size[0];
  S->size[1] = SCholInv->size[1];
  emxEnsureCapacity_real_T(S, i6);
  loop_ub = SCholInv->size[0] * SCholInv->size[1];
  for (i6 = 0; i6 < loop_ub; i6++) {
    S->data[i6] = SCholInv->data[i6];
  }

  //  make symmetric使得S矩阵保持对称
  n = S->size[1];
  if (S->size[1] != 0) {
    i6 = S->size[1];
    coffset = xpotrf(S, i6);
    if (coffset == 0) {
      coffset = n;
    } else {
      coffset--;
    }

    for (j = 0; j < coffset; j++) {
      i6 = j + 2;
      for (i = i6; i <= coffset; i++) {
        S->data[(i + S->size[0] * j) - 1] = 0.0;
      }
    }
  }

  // SChol为上三角矩阵
  inv(S, SCholInv);

  //  triangular matrix得到的SCholInv为SChol的逆矩阵
  if ((y->size[1] == 1) || (SCholInv->size[0] == 1)) {
    i6 = S->size[0] * S->size[1];
    S->size[0] = y->size[0];
    S->size[1] = SCholInv->size[1];
    emxEnsureCapacity_real_T(S, i6);
    loop_ub = y->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_loop_ub = SCholInv->size[1];
      for (coffset = 0; coffset < b_loop_ub; coffset++) {
        S->data[i6 + S->size[0] * coffset] = 0.0;
        boffset = y->size[1];
        for (aoffset = 0; aoffset < boffset; aoffset++) {
          S->data[i6 + S->size[0] * coffset] += y->data[i6 + y->size[0] *
            aoffset] * SCholInv->data[aoffset + SCholInv->size[0] * coffset];
        }
      }
    }
  } else {
    m = y->size[0];
    inner = y->size[1];
    n = SCholInv->size[1];
    i6 = S->size[0] * S->size[1];
    S->size[0] = y->size[0];
    S->size[1] = SCholInv->size[1];
    emxEnsureCapacity_real_T(S, i6);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        S->data[coffset + i] = 0.0;
      }

      for (c_loop_ub = 0; c_loop_ub < inner; c_loop_ub++) {
        aoffset = c_loop_ub * m;
        temp = SCholInv->data[boffset + c_loop_ub];
        for (i = 0; i < m; i++) {
          i6 = coffset + i;
          S->data[i6] += temp * y->data[aoffset + i];
        }
      }
    }
  }

  i6 = b->size[0] * b->size[1];
  b->size[0] = SCholInv->size[1];
  b->size[1] = SCholInv->size[0];
  emxEnsureCapacity_real_T(b, i6);
  loop_ub = SCholInv->size[0];
  for (i6 = 0; i6 < loop_ub; i6++) {
    b_loop_ub = SCholInv->size[1];
    for (coffset = 0; coffset < b_loop_ub; coffset++) {
      b->data[coffset + b->size[0] * i6] = SCholInv->data[i6 + SCholInv->size[0]
        * coffset];
    }
  }

  emxFree_real_T(&SCholInv);
  if ((S->size[1] == 1) || (b->size[0] == 1)) {
    i6 = y->size[0] * y->size[1];
    y->size[0] = S->size[0];
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(y, i6);
    loop_ub = S->size[0];
    for (i6 = 0; i6 < loop_ub; i6++) {
      b_loop_ub = b->size[1];
      for (coffset = 0; coffset < b_loop_ub; coffset++) {
        y->data[i6 + y->size[0] * coffset] = 0.0;
        boffset = S->size[1];
        for (aoffset = 0; aoffset < boffset; aoffset++) {
          y->data[i6 + y->size[0] * coffset] += S->data[i6 + S->size[0] *
            aoffset] * b->data[aoffset + b->size[0] * coffset];
        }
      }
    }
  } else {
    m = S->size[0];
    inner = S->size[1];
    n = b->size[1];
    i6 = y->size[0] * y->size[1];
    y->size[0] = S->size[0];
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(y, i6);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        y->data[coffset + i] = 0.0;
      }

      for (c_loop_ub = 0; c_loop_ub < inner; c_loop_ub++) {
        aoffset = c_loop_ub * m;
        temp = b->data[boffset + c_loop_ub];
        for (i = 0; i < m; i++) {
          i6 = coffset + i;
          y->data[i6] += temp * S->data[aoffset + i];
        }
      }
    }
  }

  // 卡尔曼增益
  if (1 > length_x) {
    loop_ub = 0;
  } else {
    loop_ub = length_x;
  }

  if ((y->size[1] == 1) || (v_size[0] == 1)) {
    b_loop_ub = y->size[0];
    for (i6 = 0; i6 < b_loop_ub; i6++) {
      tmp_data[i6] = 0.0;
      boffset = y->size[1];
      for (coffset = 0; coffset < boffset; coffset++) {
        temp = tmp_data[i6] + y->data[i6 + y->size[0] * coffset] *
          v_data[coffset];
        tmp_data[i6] = temp;
      }
    }
  } else {
    m = y->size[0];
    inner = y->size[1];
    if (0 <= m - 1) {
      memset(&tmp_data[0], 0, (unsigned int)(m * (int)sizeof(double)));
    }

    for (c_loop_ub = 0; c_loop_ub < inner; c_loop_ub++) {
      aoffset = c_loop_ub * m;
      for (i = 0; i < m; i++) {
        tmp_data[i] += v_data[c_loop_ub] * y->data[aoffset + i];
      }
    }
  }

  if (1 > length_x) {
    i6 = 0;
  } else {
    i6 = length_x;
  }

  b_loop_ub = (signed char)i6 - 1;
  for (coffset = 0; coffset <= b_loop_ub; coffset++) {
    b_tmp_data[coffset] = (signed char)coffset;
  }

  for (coffset = 0; coffset < loop_ub; coffset++) {
    x_data[coffset] = x[coffset] + tmp_data[coffset];
  }

  loop_ub = (signed char)i6;
  for (i6 = 0; i6 < loop_ub; i6++) {
    x[b_tmp_data[i6]] = x_data[i6];
  }

  //  update
  if (1 > length_x) {
    loop_ub = 0;
    b_loop_ub = 0;
  } else {
    loop_ub = length_x;
    b_loop_ub = length_x;
  }

  i6 = b->size[0] * b->size[1];
  b->size[0] = S->size[1];
  b->size[1] = S->size[0];
  emxEnsureCapacity_real_T(b, i6);
  boffset = S->size[0];
  for (i6 = 0; i6 < boffset; i6++) {
    c_loop_ub = S->size[1];
    for (coffset = 0; coffset < c_loop_ub; coffset++) {
      b->data[coffset + b->size[0] * i6] = S->data[i6 + S->size[0] * coffset];
    }
  }

  if ((S->size[1] == 1) || (b->size[0] == 1)) {
    i6 = y->size[0] * y->size[1];
    y->size[0] = S->size[0];
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(y, i6);
    boffset = S->size[0];
    for (i6 = 0; i6 < boffset; i6++) {
      c_loop_ub = b->size[1];
      for (coffset = 0; coffset < c_loop_ub; coffset++) {
        y->data[i6 + y->size[0] * coffset] = 0.0;
        m = S->size[1];
        for (aoffset = 0; aoffset < m; aoffset++) {
          y->data[i6 + y->size[0] * coffset] += S->data[i6 + S->size[0] *
            aoffset] * b->data[aoffset + b->size[0] * coffset];
        }
      }
    }
  } else {
    m = S->size[0];
    inner = S->size[1];
    n = b->size[1];
    i6 = y->size[0] * y->size[1];
    y->size[0] = S->size[0];
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(y, i6);
    for (j = 0; j < n; j++) {
      coffset = j * m;
      boffset = j * inner;
      for (i = 0; i < m; i++) {
        y->data[coffset + i] = 0.0;
      }

      for (c_loop_ub = 0; c_loop_ub < inner; c_loop_ub++) {
        aoffset = c_loop_ub * m;
        temp = b->data[boffset + c_loop_ub];
        for (i = 0; i < m; i++) {
          i6 = coffset + i;
          y->data[i6] += temp * S->data[aoffset + i];
        }
      }
    }
  }

  emxFree_real_T(&b);
  emxFree_real_T(&S);
  for (i6 = 0; i6 < b_loop_ub; i6++) {
    for (coffset = 0; coffset < loop_ub; coffset++) {
      P_data[coffset + loop_ub * i6] = P[coffset + 50 * i6] - y->data[coffset +
        y->size[0] * i6];
    }
  }

  emxFree_real_T(&y);
  for (i6 = 0; i6 < b_loop_ub; i6++) {
    for (coffset = 0; coffset < loop_ub; coffset++) {
      P[coffset + 50 * i6] = P_data[coffset + loop_ub * i6];
    }
  }
}

//
// File trailer for KF_cholesky_update.cpp
//
// [EOF]
//
