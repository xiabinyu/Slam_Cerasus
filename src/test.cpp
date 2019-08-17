/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 4.1
 * C/C++ source code generated on  : 12-Aug-2019 10:48:11
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include Files */

#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "predict_types.h"
#include "rt_nonfinite.h"
#include "predict.h"
//#include "predict/main.h"
#include "predict_terminate.h"
#include "predict_initialize.h"

/* Function Declarations */
static void argInit_500x1_real_T(double result[500]);
static void argInit_50x50_real_T(double result[2500]);
static int argInit_int32_T(void);
static double argInit_real_T(void);
static void main_predict(void);

/* Function Definitions */

/*
 * Arguments    : double result[500]
 * Return Type  : void
 */
static void argInit_500x1_real_T(double result[500])
{
    int idx0;

    /* Loop over the array to initialize each element. */
    for (idx0 = 0; idx0 < 500; idx0++) {
        /* Set the value of the array element.
           Change this value to the value that the application requires. */
        result[idx0] = argInit_real_T();
    }
}

/*
 * Arguments    : double result[2500]
 * Return Type  : void
 */
static void argInit_50x50_real_T(double result[2500])
{
    int idx0;
    int idx1;

    /* Loop over the array to initialize each element. */
    for (idx0 = 0; idx0 < 50; idx0++) {
        for (idx1 = 0; idx1 < 50; idx1++) {
            /* Set the value of the array element.
               Change this value to the value that the application requires. */
            result[idx0 + 50 * idx1] = argInit_real_T();
        }
    }
}

/*
 * Arguments    : void
 * Return Type  : int
 */
static int argInit_int32_T(void)
{
    return 0;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
    return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_predict(void)
{
    double x[500];
    double P[2500];

    /* Initialize function 'predict' input arguments. */
    /* Initialize function input argument 'x'. */
    argInit_500x1_real_T(x);

    /* Initialize function input argument 'P'. */
    argInit_50x50_real_T(P);

    /* Call the entry-point 'predict'. */
    predict(x, P, argInit_real_T(), argInit_real_T(), argInit_real_T(),
            argInit_real_T(), argInit_int32_T());
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
    (void)argc;
    (void)argv;

    /* Initialize the application.
       You do not need to do this more than one time. */
    predict_initialize();

    /* Invoke the entry-point functions.
       You can call entry-point functions multiple times. */
    main_predict();

    /* Terminate the application.
       You do not need to do this more than one time. */
    predict_terminate();
    return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
