//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predict.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 23-Sep-2019 13:51:52
//
#ifndef PREDICT_H
#define PREDICT_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "predict_types.h"

// Function Declarations
extern void predict(double x[500], double P[2500], double v, double g, double WB,
                    double dt, int lx);

#endif

//
// File trailer for predict.h
//
// [EOF]
//
