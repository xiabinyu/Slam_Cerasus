//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: slamCore_emxutil.h
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 12-Aug-2019 10:58:49
//
#ifndef SLAMCORE_EMXUTIL_H
#define SLAMCORE_EMXUTIL_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "slamCore_types.h"

// Function Declarations
extern void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_int8_T(emxArray_int8_T *emxArray, int oldNumel);
extern void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int oldNumel);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_int8_T(emxArray_int8_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_int8_T(emxArray_int8_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

//
// File trailer for slamCore_emxutil.h
//
// [EOF]
//
