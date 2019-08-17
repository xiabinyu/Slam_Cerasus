//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: SlamStart.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 13-Aug-2019 12:09:36
//

// Include Files
#include <string.h>
#include "SlamStart.h"

// Function Definitions

//
// Arguments    : double z_data[]
//                int z_size[2]
//                double x[500]
//                double P[2500]
//                int *length_x
// Return Type  : void
//
void SlamStart(double z_data[], int z_size[2], double x[500], double P[2500],
               int *length_x)
{
  int i0;
  int i1;
  memset(&x[0], 0, 500U * sizeof(double));
  *length_x = 3;
  memset(&P[0], 0, 2500U * sizeof(double));
  z_size[0] = 2;
  z_size[1] = 360;
  for (i0 = 0; i0 < 360; i0++) {
    i1 = i0 << 1;
    z_data[i1] = 0.0;
    z_data[1 + i1] = 0.0;
  }
}

//
// File trailer for SlamStart.cpp
//
// [EOF]
//
