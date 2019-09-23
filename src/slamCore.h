//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: slamCore.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 23-Sep-2019 13:55:32
//
#ifndef SLAMCORE_H
#define SLAMCORE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "slamCore_types.h"

// Function Declarations
extern void slamCore(double x[500], double P[2500], const double z_data[], const
                     int z_size[2], int *length_x);

#endif

//
// File trailer for slamCore.h
//
// [EOF]
//
