//
// Created by xiabi on 2019/7/25.
//

#ifndef SLAM_PRJ_FINAL_CERASUS_SLAM_CLASS_H
#define SLAM_PRJ_FINAL_CERASUS_SLAM_CLASS_H


class cerasus_slam_class {

    //const
    double V= 4; // m/s
    double MAXG= 30*3.1415/180; // radians, maximum steering angle (-MAXG < g < MAXG)
    double RATEG= 180*3.1415/180; // rad/s, maximum rate of change in steer angle
    double WHEELBASE= 4; // metres, vehicle wheel-base
    double DT_CONTROLS= 0.03557; // seconds, time interval between control signals�����ź�֮���ʱ����

    // observation parameters
    double MAX_RANGE= 30.0; // metres
    //double DT_OBSERVE= 8*DT_CONTROLS; // seconds, time interval between observations�۲��ź�֮��ļ��

    // observation noises
    double sigmaR= 0.07; // metres
    double sigmaB= (1.0*3.1415/180); // radians

    // data association innovation gates (Mahalanobis distances)
    double GATE_REJECT= 4.0; // maximum distance for association
    double GATE_AUGMENT= 25.0; // minimum distance for creation of new feature
    // For 2-D observation:
    //   - common gates are: 1-sigma (1.0), 2-sigma (4.0), 3-sigma (9.0), 4-sigma (16.0)
    //   - percent probability mass is: 1-sigma bounds 40//, 2-sigma 86//, 3-sigma 99//, 4-sigma 99.9//.

    // waypoint proximity
    double AT_WAYPOINT= 1.0; // metres, distance from current waypoint at which to switch to next waypoint
    double NUMBER_LOOPS= 2; // number of loops through the waypoint list

    double x[500];
    double P[2500];
    double z_data[720];
    int z_size[2];
    int length_x;
    double s_ang;
    //Function from MatLab to c:
    //M2C_buildall();
    //M2C_slamcore();
    //M2C_getall();
public:
    cerasus_slam_class();
    ~cerasus_slam_class();
    void Update_Slam(double *Lidar,int Lidar_Size);
    void Update_Slam_rpm(double rpm);
    void Update_Slam_imu(double s_ang);
    void Get_Slam(double* x,double* y,double* theta);

};


#endif //SLAM_PRJ_FINAL_CERASUS_SLAM_CLASS_H
