#include <RobotDynamics.h>

RobotDynamics::RobotDynamics(unsigned int JOINT_NUM_):
    JOINT_NUM(JOINT_NUM_)
{

}
RobotDynamics::RobotDynamics(unsigned int JOINT_NUM_, vector<Eigen::Vector3d> mass_center_,
                             vector<Eigen::Matrix3d> inertia_, vector<double> mass_,
                             vector<double> damping_, vector<double> friction_) :
    JOINT_NUM(JOINT_NUM_)
{
    PC.assign(mass_center_.begin(), mass_center_.end());
    I.assign(inertia_.begin(), inertia_.end());
    m.assign(mass_.begin(), mass_.end());
    damping.assign(damping_.begin(), damping_.end());
    friction.assign(friction_.begin(), friction_.end());
    assert(JOINT_NUM == PC.size());
    assert(JOINT_NUM == I.size());
    assert(JOINT_NUM == m.size());
    assert(JOINT_NUM == damping.size());
    assert(JOINT_NUM == friction.size());
}

RobotDynamics::~RobotDynamics() {}
void  RobotDynamics::set_PC(vector<Eigen::Vector3d> mass_center_)
{
    PC.assign(mass_center_.begin(), mass_center_.end());
}
void RobotDynamics::set_I(vector<Eigen::Matrix3d> inertia_)
{
    I.assign(inertia_.begin(), inertia_.end());
}

void RobotDynamics::set_mass(vector<double> mass_)
{
    m.assign(mass_.begin(), mass_.end());
}


vector<double> RobotDynamics::
getTau(double G, vector<Eigen::Matrix3d> R, vector<Eigen::Vector3d> P,
       vector<double> theta_d, vector<double> theta_dd)
{
    assert(JOINT_NUM == R.size());
    assert(JOINT_NUM == P.size());
    assert(JOINT_NUM == theta_d.size());
    assert(JOINT_NUM == theta_dd.size());

    vector<Eigen::Vector3d> w(JOINT_NUM + 1);
    vector<Eigen::Vector3d> w_d(JOINT_NUM + 1);
    vector<Eigen::Vector3d> v_d(JOINT_NUM + 1);
    vector<Eigen::Vector3d> vC_d(JOINT_NUM + 1);
    vector<Eigen::Vector3d> F(JOINT_NUM + 1);
    vector<Eigen::Vector3d> N(JOINT_NUM + 1);
    vector<Eigen::Vector3d> f(JOINT_NUM + 2);
    vector<Eigen::Vector3d> n(JOINT_NUM + 2);
    for (int i = 0; i < JOINT_NUM + 1; i++)
    {
        w[i].setZero();
        w_d[i].setZero();
        v_d[i].setZero();
        vC_d[i].setZero();
        F[i].setZero();
        N[i].setZero();
    }
    for (int i = 0; i < JOINT_NUM + 2; i++)
    {
        f[i].setZero();//各个关节外力设置为0
        n[i].setZero();//各个关节外力矩设置为0
    }
    vector<double> tau(JOINT_NUM);
    Eigen::Vector3d Z(0, 0, 1);
    //v_d[0] = Eigen::Vector3d(0, 0, G);//基于世界坐标系的向下的重力方向,一般朝向Z负方向
    v_d[0] = Eigen::Vector3d(0, G, 0);//基于世界坐标系的向下的重力方向
    // https://zhuanlan.zhihu.com/p/34717170,//此例程G方向沿基坐标系的Y负方向
   for (int i = 0; i < JOINT_NUM; i++)
    {
        w[i + 1] = R[i].inverse() * w[i] + theta_d[i] * Z;
        w_d[i + 1] = R[i].inverse() * w_d[i] + (R[i].inverse() * w[i]).cross(theta_d[i] * Z) + theta_dd[i] * Z;
        v_d[i + 1] = R[i].inverse() * (w_d[i].cross(P[i]) + w[i].cross(w[i].cross(P[i])) + v_d[i]);
        vC_d[i + 1] = w_d[i + 1].cross(PC[i]) + w[i + 1].cross(w[i + 1].cross(PC[i])) + v_d[i + 1];
        F[i + 1] = m[i] * vC_d[i + 1];
        N[i + 1] = I[i] * w_d[i + 1] + w[i + 1].cross(I[i] * w[i + 1]);
    }
    Eigen::Matrix3d identity_mat;
    identity_mat.setIdentity();
    R.push_back(identity_mat);//最后一个R:力矩相对于最后一个连杆的姿态矩阵
    //P.push_back(Eigen::Vector3d(1, 0, 0));//最后一个P真实外力作用点在末端连杆坐标系位置
    P.push_back(Eigen::Vector3d(0, 0, 0));//最后一个P:真实外力作用点在末端连杆坐标系位置
    //    Eigen::Vector3d extf(0,-1,0);//
    //    f[JOINT_NUM+1]= extf;//外力向量,沿着Y轴施加-1N
    //n[JOINT_NUM+1] =extTor;//外力矩向量

    for (int i = JOINT_NUM; i >= 1; i--)
    {
        f[i] = R[i] * f[i + 1] + F[i];
        n[i] = N[i] + R[i] * n[i + 1] + PC[i - 1].cross(F[i]) + P[i].cross(R[i] * f[i + 1]);
        tau[i - 1] = n[i].transpose() * Z + damping[i - 1] * theta_d[i - 1];
    }
    return tau;
}


vector<double> RobotDynamics::
getTau(double G, vector<Eigen::Quaterniond> q, vector<Eigen::Vector3d> P,
       vector<double> theta_d, vector<double> theta_dd)
{
    vector<Eigen::Matrix3d> R;
    unsigned int len = q.size();
    R.resize(len);
    for (int i = 0; i < len; i++)
    {
        R[i] = q[i];
    }
    return getTau(G, R, P, theta_d, theta_dd);
}
