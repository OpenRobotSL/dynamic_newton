#include <vector>
//#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <assert.h>

using namespace std;


class RobotDynamics
{
public:
    RobotDynamics(unsigned int JOINT_NUM_);
    RobotDynamics(unsigned int JOINT_NUM_, vector<Eigen::Vector3d> mass_center_,
                  vector<Eigen::Matrix3d> inertia_, vector<double> mass_,
                  vector<double> damping_, vector<double> friction_);

    ~RobotDynamics();

    vector<double> getTau(double G, vector<Eigen::Matrix3d> R, vector<Eigen::Vector3d> P,
                          vector<double> theta_d, vector<double> theta_dd);

    vector<double> getTau(double G, vector<Eigen::Quaterniond> q, vector<Eigen::Vector3d> P,
                          vector<double> theta_d, vector<double> theta_dd);

    void set_PC(vector<Eigen::Vector3d> mass_center_);
    void set_I(vector<Eigen::Matrix3d> inertia_);
    void set_mass(vector<double> mass_);
private:
    unsigned int JOINT_NUM;
    vector<Eigen::Vector3d> PC;
    vector<Eigen::Matrix3d> I;
    vector<double> m;
    vector<double> damping;
    vector<double> friction;
};
