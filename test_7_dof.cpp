/*#include <iostream>
#include "RobotDynamics.h"
using namespace std;

int main(int argc, char *argv[])
{
    unsigned int JOINT_NUM_=7;
    std::vector<Eigen::Vector3d> mass_center_;


    Eigen::Vector3d vecMass(0,0,0);

    for(int i =0;i<JOINT_NUM_;i++)
    {
        mass_center_.push_back(vecMass);
    }
    std::vector<Eigen::Matrix3d> inertia_;

    Eigen::Matrix3d intertia_normal;
    intertia_normal<< 0, 0, 0,
            0, 0, 0,
            0, 0, 0;



    for(int i =0;i<JOINT_NUM_;i++)
    {
        inertia_.push_back(intertia_normal);
    }


    std::vector<double> mass_;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        mass_.push_back(1);
    }

    std::vector<double> damping_;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        damping_.push_back(0.0);
    }

    std::vector<double> friction_;
    for(int i =0;i<JOINT_NUM_;i++)
    {
        friction_.push_back(0.0);
    }

    //    RobotDynamics  dyn_obj(JOINT_NUM_);
    //    dyn_obj.set_PC(mass_center_);
    //    dyn_obj.set_I(inertia_);
    //    dyn_obj.set_mass();

    RobotDynamics  dyn_obj(JOINT_NUM_,mass_center_,inertia_,mass_,damping_,friction_);


    vector<Eigen::Matrix3d> R;
    Eigen::Matrix3d R_mat[7];

    R_mat[0]<<     1,     0,     0,
            0,     1,     0,
            0,     0,     1;
    R_mat[1]<< 1.0000 ,        0,         0,
            0,    0.0000,    1.0000,
            0,   -1.0000,    0.0000;


    R_mat[2]<<1.0000,         0,         0,
            0,    0.0000,   -1.0000,
            0,    1.0000,    0.0000;

    R_mat[3]<<1.0000,         0,         0,
            0,    0.0000,   -1.0000,
            0,    1.0000,    0.0000;

    R_mat[4]<< 1.0000,         0,         0,
            0,    0.0000,    1.0000,
            0,   -1.0000,    0.0000;

    R_mat[5]<<1.0000,         0,         0,
            0,    0.0000,   -1.0000,
            0,    1.0000,    0.0000;

    R_mat[6]<<    1.0,   0,         0,
            0,    0.0000,   -1.0000,
            0,    1.0000,    0.0000;

    for(int i =0;i<JOINT_NUM_;i++)
    {
        R.push_back(R_mat[i]);
    }
    vector<Eigen::Vector3d> P;
    Eigen::Vector3d P_vec[7];
    P_vec[0].Zero();
    P_vec[1].Zero();
    P_vec[2]<<0,-0.255,0;
    P_vec[3]<<0.0760,0,0;
    P_vec[4]<< -0.0760,0.3150,0;
    P_vec[5]<< 0,0,0;
    P_vec[6]<< 0.0760,-0.0840,0;
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
    double G = -9.8;
    vector<double> tau_jnt = dyn_obj.getTau(G,R,P,theta_d,theta_dd);
    cout<<"tau_jnt:"<<endl;
    for(int i =0;i<JOINT_NUM_;i++)
    {
        cout<<tau_jnt[i]<<endl;
    }
    return 0;
}
*/
