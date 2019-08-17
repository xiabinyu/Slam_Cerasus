//
// Created by xiabi on 2019/7/25.
//

#include "SlamStart.h"
//#include "main.h"
#include "SlamStart_terminate.h"
#include "SlamStart_initialize.h"
#include "rt_nonfinite.h"
#include "slamCore.h"
//#include "main.h"
#include "slamCore_terminate.h"
#include "slamCore_initialize.h"
//#include "predict/rt_nonfinite.h"
#include "predict.h"
//#include "main.h"
#include "predict_terminate.h"
#include "predict_initialize.h"

#include "cerasus_slam_class.h"
cerasus_slam_class::cerasus_slam_class(){

    SlamStart_initialize();
    predict_initialize();

    // Call the entry-point 'SlamStart'.
    SlamStart(z_data, z_size, x, P, &length_x);
}


cerasus_slam_class::~cerasus_slam_class(){

    SlamStart_terminate();
}
void cerasus_slam_class::Update_Slam_imu(double s_ang){
    this->s_ang=s_ang;
}

void cerasus_slam_class::Update_Slam(double *Lidar,int Lidar_Size){

    int zl_size[2]={2,Lidar_Size};
    slamCore(x, P, Lidar, zl_size, &length_x);
}

void cerasus_slam_class::Update_Slam_rpm(double rpm){
    /* Initialize the application.
   You do not need to do this more than one time. */

    /* Invoke the entry-point functions.
       You can call entry-point functions multiple times. */
    predict(x, P, rpm, s_ang,WHEELBASE,
            DT_CONTROLS,length_x);

    /* Terminate the application.
       You do not need to do this more than one time. */
    predict_terminate();
}

void cerasus_slam_class::Get_Slam(double* x,double* y,double* theta){
    *x=x[0];
    *y=x[1];
    *theta=x[2];
}