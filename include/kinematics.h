#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_
#include "head.h"

using namespace Eigen;
const double eps = 1e-6;
// inline double robot_rad = M_PI / 180;
inline double deg2rad = M_PI / 180;
// inline double robot_deg = 180 / M_PI;
inline double rad2deg = 180 / M_PI;
inline double deducrate = 262144;
inline double rad=M_PI/180;
inline double deg=180/M_PI;
class Robot
{
private:
    int dof=7;
    // const double d1 = 190e-3;
    // const double d3 = 300e-3;
    // const double d5 = 273e-3;
        const double d1 = 192e-3;
    const double d3 = 270e-3;
    const double d5 = 252e-3;
    const double d7 = 0e-3;
    double res=65536;
    double ratio=4;

    Matrix<double, 1, 6> tool;   
    Matrix<double, 1, 7> w;
    Matrix<double, 2, 7> limit;
    Matrix<double, 1, 7> init_qref,q2motor_direct,q2motor_offset;

    int deviceInd,canInd; 
    double rad2cnt=(ratio*res)/(2*M_PI);
    double vel2hz =(ratio*100)/(2*M_PI);

    uint8_t MotorsIDlist[7];
    int armID;
    Matrix<double, 7, 4> MDH;
    int select=0;
    int hand_flag;

public:
    // 构造函数，用于初始化机器人参数
    Robot(int mod);
    int q2motorid;
    Matrix<double, 1, 6> getTool();
    MatrixXd q2MotorAngle(Matrix<double, 1, 7>  &q);
    MatrixXd MotorAngle2q(Matrix<double, 1, 7>  &MotorAngle);
    MatrixXd getJointPos();
    MatrixXd getTcpPos();
    MatrixXd J_tcp_crossproduct(const Matrix<double, 1, 7>& q);
    Matrix4d fk(const Matrix<double, 1, 7>& q) ;
    int ik(const Matrix<double, 1, 6>& pos, Matrix<double, 1, 7>& q,double q2) ;
    int servoJ( Matrix<double, 1, 7>& qd) ;
    int moveJToPos(Matrix<double, 1, 6>& posd,double q2=0*rad);
     int moveJtoJoint(Matrix<double, 1, 7>& q);
    int moveLToPos(Matrix<double, 1, 6>& posd,double vel=0.1);
    bool check_Joint_withlimit(Matrix<double, 1, 7>  qc);
    int check_workspace_access(Matrix<double, 1, 6>& pos);

    int moveJToJoint_Tf(Matrix<double, 1, 7> &q_end, double Tf);

    int moveLToPos_eyehand(Matrix<double, 1, 6>& posd,double vel);
int checkJointLimits(Matrix<double, 1, 7>& qc);
    // vector<vector<double>> Robot::th_MoveLToPose(double control_frequency, double Tf, Matrix<double, 1, 6> &posd);
int moveLToPos_Opt(Matrix<double, 1, 6> &posd, double vel);
Matrix<double, 1, 7> quadratic_cost_function(const Eigen::Matrix<double, 1, 7> &q,const Eigen::Matrix<double, 1, 6> &Vtcp);
int moveLToPosIK(Matrix<double, 1, 6> &posd, double vel, double q2d);

int getOPt_IK(Matrix<double, 1, 6> &posd, Matrix<double, 1, 7>& q_current);
Matrix<double, 1, 7> quadratic_cost_function_AwayLimit(const Eigen::Matrix<double, 1, 7> &q,const Eigen::Matrix<double, 1, 6> &Vtcp);
};

#endif