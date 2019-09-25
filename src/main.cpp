

#include <iostream>
#include "cerasus_slam_class.h"
#include "slam_cerasus_const.h"
using namespace std;
static cerasus_slam_class CSC;
int main(int argc, char** argv){
    double x,y,th;
    CSC.Get_Slam(&x,&y,&th);
    cout<<x<<" "<<y<<" "<<th<<endl;
    CSC.Update_Slam_rpm(10000);

    CSC.Get_Slam(&x,&y,&th);
    cout<<x<<" "<<y<<" "<<th<<endl;
    return 0;
}
