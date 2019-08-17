//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: slamCore_types.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//
#ifndef SLAMCORE_TYPES_H
#define SLAMCORE_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct emxArray_int32_T
{
  int *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

struct emxArray_int8_T
{
  signed char *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif

//
// File trailer for slamCore_types.h
//
// [EOF]
//
