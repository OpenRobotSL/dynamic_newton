#include <iostream>
#include "RobotDynamics.h"
using namespace std;

int main(int argc, char *argv[])
{
    unsigned int JOINT_NUM_=2;//关节数
    std::vector<Eigen::Vector3d> mass_center_;//质心在每个连杆坐标系中位置

    double l1 =1;//质心
    double l2 =1;


    Eigen::Vector3d vecMass[2];
    vecMass[0]<<l1,0,0;
    vecMass[1]<<l2,0,0;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        mass_center_.push_back(vecMass[i]);
    }

    std::vector<double> mass_;//连杆质量

    double m1 =1;
    double m2 =1;
    mass_.push_back(m1);
    mass_.push_back(m2);

    std::vector<Eigen::Matrix3d> inertia_;//惯量矩阵

    Eigen::Matrix3d intertia_normal;
    intertia_normal<< 0, 0, 0,
            0, 0, 0,
            0, 0, 0;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        inertia_.push_back(intertia_normal);
    }

    std::vector<double> damping_;//每个连杆的阻尼，乘以速度就是阻尼力

    for(int i =0;i<JOINT_NUM_;i++)
    {
        damping_.push_back(0.0);
    }

    std::vector<double> friction_;
    for(int i =0;i<JOINT_NUM_;i++)//摩擦力
    {
        friction_.push_back(0.0);
    }

    RobotDynamics  dyn_obj(JOINT_NUM_,mass_center_,inertia_,mass_,damping_,friction_);


    vector<Eigen::Matrix3d> R;
    Eigen::Matrix3d R_mat[JOINT_NUM_];

    double theta_rad[JOINT_NUM_];//关节1，2角度
    theta_rad[0] =0;
    theta_rad[1] =0;
    R_mat[0]<<     cos(theta_rad[0]),     -sin(theta_rad[0]),     0,//R旋转矩阵
            sin(theta_rad[0]),     cos(theta_rad[0]),     0,
            0,     0,     1;
    R_mat[1]<<     cos(theta_rad[1]),     -sin(theta_rad[1]),     0,
            sin(theta_rad[1]),     cos(theta_rad[1]),     0,
            0,     0,     1;


  cout <<"R_mat"<<R_mat[0]<<endl;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        R.push_back(R_mat[i]);
    }
    vector<Eigen::Vector3d> P;//每个坐标系原点相对前一个坐标系原点位移
    Eigen::Vector3d P_vec[JOINT_NUM_];
    P_vec[0]<<0,0,0;
    P_vec[1]<<l1,0,0;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        P.push_back(P_vec[i]);
    }

    vector<double> theta_d;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        theta_d.push_back(0);
    }

    vector<double> theta_dd;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        theta_dd.push_back(0);
    }

    double G = -9.8;//重力加速度
    vector<double> tau_jnt = dyn_obj.getTau(G,R,P,theta_d,theta_dd);
    cout<<"tau_jnt:"<<endl;
    for(int i =0;i<JOINT_NUM_;i++)
    {
        cout<<tau_jnt[i]<<endl;
    }
    return 0;
}
