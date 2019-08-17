//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: data_associate.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//
#ifndef DATA_ASSOCIATE_H
#define DATA_ASSOCIATE_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "slamCore_types.h"

// Function Declarations
extern void compute_association(const double x[500], const double P[2500], const
  double z[2], const double R[4], int idf, int length_x, double *nis, double *nd);

#endif

//
// File trailer for data_associate.h
//
// [EOF]
//
