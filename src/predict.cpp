//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: predict.cpp
//
// MATLAB Coder version            : 4.1
// C/C++ source code generated on  : 13-Aug-2019 20:45:15
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "predict.h"

// Function Definitions

//
// function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)
//
//  Inputs:
//    x, P - SLAM state and covariance
//    v, g - control inputs: velocity and gamma (steer angle)
//    Q - covariance matrix for velocity and gamma
//    WB - vehicle wheelbase
//    dt - timestep
//
//  Outputs:
//    xn, Pn - predicted state and covariance
//
//  Tim Bailey 2004.
// Arguments    : double x[500]
//                double P[2500]
//                double v
//                double g
//                double WB
//                double dt
//                int lx
// Return Type  : void
//
void predict(double x[500], double P[2500], double v, double g, double WB,
             double dt, int lx)
{
  double c;
  double s;
  double vts_tmp;
  double vts;
  double vtc;
  double Gv[9];
  double Gu[6];
  double q;
  int i0;
  int boffset;
  double b_Gu[6];
  int Gv_tmp;
  double b_Gv[9];
  double c_Gv[9];
  int coffset;
  double tmp_data[141];
  int k;
  boolean_T rEQ0;
  int aoffset;
  double dv0[1];
  double dv1[1];

  //  m/s
  //  radians
  c = g + x[2];
  s = std::sin(c);
  c = std::cos(c);
  vts_tmp = v * dt;
  vts = vts_tmp * s;
  vtc = vts_tmp * c;

  //  jacobians
  Gv[0] = 1.0;
  Gv[3] = 0.0;
  Gv[6] = -vts;
  Gv[1] = 0.0;
  Gv[4] = 1.0;
  Gv[7] = vtc;
  Gv[2] = 0.0;
  Gv[5] = 0.0;
  Gv[8] = 1.0;
  Gu[0] = dt * c;
  Gu[3] = -vts;
  Gu[1] = dt * s;
  Gu[4] = vtc;
  q = std::sin(g);
  Gu[2] = dt * q / WB;
  Gu[5] = vts_tmp * std::cos(g) / WB;

  //  predict covariance
  for (i0 = 0; i0 < 3; i0++) {
    for (boffset = 0; boffset < 3; boffset++) {
      Gv_tmp = i0 + 3 * boffset;
      b_Gv[Gv_tmp] = 0.0;
      b_Gv[Gv_tmp] = (Gv[i0] * P[50 * boffset] + Gv[i0 + 3] * P[1 + 50 * boffset])
        + Gv[i0 + 6] * P[2 + 50 * boffset];
    }

    b_Gu[i0] = 0.0;
    c = Gu[i0 + 3];
    b_Gu[i0] = Gu[i0] * 0.09 + c * 0.0;
    b_Gu[i0 + 3] = 0.0;
    b_Gu[i0 + 3] = Gu[i0] * 0.0 + c * 0.0027415567780803771;
    for (boffset = 0; boffset < 3; boffset++) {
      Gv_tmp = i0 + 3 * boffset;
      c_Gv[Gv_tmp] = 0.0;
      c_Gv[Gv_tmp] = (b_Gv[i0] * Gv[boffset] + b_Gv[i0 + 3] * Gv[boffset + 3]) +
        b_Gv[i0 + 6] * Gv[boffset + 6];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (boffset = 0; boffset < 3; boffset++) {
      Gv_tmp = i0 + 3 * boffset;
      b_Gv[Gv_tmp] = 0.0;
      b_Gv[Gv_tmp] = b_Gu[i0] * Gu[boffset] + b_Gu[i0 + 3] * Gu[boffset + 3];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    P[50 * i0] = c_Gv[3 * i0] + b_Gv[3 * i0];
    boffset = 1 + 3 * i0;
    P[1 + 50 * i0] = c_Gv[boffset] + b_Gv[boffset];
    boffset = 2 + 3 * i0;
    P[2 + 50 * i0] = c_Gv[boffset] + b_Gv[boffset];
  }

  if (lx > 3) {
    for (Gv_tmp = 0; Gv_tmp <= lx - 4; Gv_tmp++) {
      coffset = Gv_tmp * 3;
      boffset = Gv_tmp * 3;
      tmp_data[coffset] = 0.0;
      tmp_data[coffset + 1] = 0.0;
      tmp_data[coffset + 2] = 0.0;
      for (k = 0; k < 3; k++) {
        aoffset = k * 3;
        i0 = boffset + k;
        c = P[i0 % 3 + 50 * (3 + i0 / 3)];
        tmp_data[coffset] += c * Gv[aoffset];
        tmp_data[coffset + 1] += c * Gv[aoffset + 1];
        tmp_data[coffset + 2] += c * Gv[aoffset + 2];
      }
    }

    coffset = lx - 3;
    for (i0 = 0; i0 < coffset; i0++) {
      boffset = 50 * (3 + i0);
      P[boffset] = tmp_data[3 * i0];
      P[1 + boffset] = tmp_data[1 + 3 * i0];
      P[2 + boffset] = tmp_data[2 + 3 * i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (boffset = 0; boffset <= lx - 4; boffset++) {
        tmp_data[boffset + (lx - 3) * i0] = P[i0 + 50 * (3 + boffset)];
      }
    }

    coffset = lx - 3;
    for (i0 = 0; i0 < 3; i0++) {
      for (boffset = 0; boffset < coffset; boffset++) {
        P[(boffset + 50 * i0) + 3] = tmp_data[boffset + (lx - 3) * i0];
      }
    }
  }

  //  predict state
  c = x[2] + vts_tmp * q / WB;

  // function angle = pi_to_pi(angle)
  //  Input: array of angles.
  //  Tim Bailey 2000
  if ((!rtIsInf(c)) && (!rtIsNaN(c))) {
    if (c == 0.0) {
      s = 0.0;
    } else {
      s = std::fmod(c, 6.2831853071795862);
      rEQ0 = (s == 0.0);
      if (!rEQ0) {
        q = std::abs(c / 6.2831853071795862);
        rEQ0 = (std::abs(q - std::floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        s = 0.0;
      } else {
        if (c < 0.0) {
          s += 6.2831853071795862;
        }
      }
    }
  } else {
    s = rtNaN;
  }

  if (s > 3.1415926535897931) {
    Gv_tmp = 1;
    coffset = 1;
  } else {
    Gv_tmp = 0;
    coffset = 0;
  }

  dv0[0] = s;
  coffset = (signed char)Gv_tmp * (signed char)coffset;
  for (i0 = 0; i0 < coffset; i0++) {
    dv0[0] = s - 6.2831853071795862;
  }

  if (dv0[0] < -3.1415926535897931) {
    Gv_tmp = 1;
    coffset = 1;
  } else {
    Gv_tmp = 0;
    coffset = 0;
  }

  dv1[0] = dv0[0];
  coffset = (signed char)Gv_tmp * (signed char)coffset;
  for (i0 = 0; i0 < coffset; i0++) {
    dv1[0] = dv0[0] + 6.2831853071795862;
  }

  c = x[1] + vts;
  x[0] += vtc;
  x[1] = c;
  x[2] = dv1[0];
}

//
// File trailer for predict.cpp
//
// [EOF]
//
