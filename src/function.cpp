#include <iostream>
#include <fstream>

#include "function.h"
// 将沿着xyz坐标系平移运动转化为4*4矩阵
// #define pi acos(-1)
// const double eps = 1e-6;
// double rad=pi/180,deg=180/pi;
//  double rad()
//  {
//      return pi/180;
//  }
//  double deg()
//  {
//      return 180/pi;
//  }
// 将沿着xyz坐标系平移运动转化为4*4矩阵
MatrixXd Transl_xyz(Matrix<double, 1, 3> trans)
{
    double x = trans(0), y = trans(1), z = trans(2);
    Matrix<double, 4, 4> T_transl;
    T_transl << 1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1;
    return T_transl;
}

// 将沿着xyz坐标系旋转运动转化为4*4矩阵
// 将沿着xyz坐标系旋转运动转化为4*4矩阵
MatrixXd Rot_zyx(Matrix<double, 1, 3> theta)
{
    Matrix<double, 4, 4> T_R;
    Matrix<double, 3, 3> Rotx, Roty, Rotz, RPY;
    double a = theta(0), b = theta(1), c = theta(2);
    Rotz(0, 0) = cos(c);
    Rotz(0, 1) = -sin(c);
    Rotz(0, 2) = 0;
    Rotz(1, 0) = sin(c);
    Rotz(1, 1) = cos(c);
    Rotz(1, 2) = 0;
    Rotz(2, 0) = 0;
    Rotz(2, 1) = 0;
    Rotz(2, 2) = 1;

    Roty(0, 0) = cos(b);
    Roty(0, 1) = 0;
    Roty(0, 2) = sin(b);
    Roty(1, 0) = 0;
    Roty(1, 1) = 1;
    Roty(1, 2) = 0;
    Roty(2, 0) = -sin(b);
    Roty(2, 1) = 0;
    Roty(2, 2) = cos(b);

    Rotx(0, 0) = 1;
    Rotx(0, 1) = 0;
    Rotx(0, 2) = 0;
    Rotx(1, 0) = 0;
    Rotx(1, 1) = cos(a);
    Rotx(1, 2) = -sin(a);
    Rotx(2, 0) = 0;
    Rotx(2, 1) = sin(a);
    Rotx(2, 2) = cos(a);

    RPY = Rotz * Roty * Rotx;
    T_R.row(0) << RPY.row(0), 0;
    T_R.row(1) << RPY.row(1), 0;
    T_R.row(2) << RPY.row(2), 0;
    T_R.row(3) << 0, 0, 0, 1;
    return T_R;
}

MatrixXd TR(Matrix<double, 1, 6> transpose)
{

    Matrix<double, 1, 3> transl, rot;
    Matrix<double, 4, 4> T;
    transl << transpose(0), transpose(1), transpose(2);
    rot << transpose(3), transpose(4), transpose(5);
    T = Transl_xyz(transl) * Rot_zyx(rot);
    return T;
}

// 修正DH坐标
MatrixXd MDHTrans(double alpha, double a, double d, double theta)
{
    MatrixXd T(4, 4);
    T(0, 0) = cos(theta);
    T(0, 1) = -sin(theta);
    T(0, 2) = 0;
    T(0, 3) = a;
    T(1, 0) = sin(theta) * cos(alpha);
    T(1, 1) = cos(theta) * cos(alpha);
    T(1, 2) = -sin(alpha);
    T(1, 3) = -sin(alpha) * d;
    T(2, 0) = sin(theta) * sin(alpha);
    T(2, 1) = cos(theta) * sin(alpha);
    T(2, 2) = cos(alpha);
    T(2, 3) = cos(alpha) * d;
    T(3, 0) = 0;
    T(3, 1) = 0;
    T(3, 2) = 0;
    T(3, 3) = 1;

    return T;
}
Matrix<double, 1, 3> rotationMatrixToEulerAngles(Matrix<double, 3, 3> R)
{
    //    assert(isRotationMatrix(R));
    double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2(R(2, 1), R(2, 2));
        y = atan2(-R(2, 0), sy);
        z = atan2(R(1, 0), R(0, 0));
    }
    else
    {
        x = atan2(-R(1, 2), R(1, 1));
        y = atan2(-R(2, 0), sy);
        z = 0;
    }
    return {x, y, z};
}

Matrix<double, 1, 6> T2PosEulerAngles(Matrix<double, 4, 4> T)
{
    Matrix<double, 1, 3> p, euler;
    Matrix<double, 1, 6> pos;
    euler = rotationMatrixToEulerAngles(T.block(0, 0, 3, 3));
    p << T(0, 3), T(1, 3), T(2, 3);
    pos << p, euler;
    return pos;
}

Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q)
{
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 deg()rees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
// 4*4位姿矩阵转化为1*3轴角
// MatrixXd T_to_AngleAxis(Matrix<double, 4, 4>  T)
//{
//     Matrix<double, 4, 4> Tcp;
//     Matrix<double, 1, 3> vec,AngleAxis;
//     double k,theta;
//     double r11,r12,r13,r21,r22,r23,r31,r32,r33;
//     r11 = T(0, 0); r12 = T(0, 1); r13 = T(0, 2);
//     r21 = T(1, 0); r22 = T(1, 1); r23 = T(1, 2);
//     r31 = T(2, 0); r32 = T(2, 1); r33 = T(2, 2);
//     theta=acos((r11+r22+r33-1)/2);
//     k=(1/(2*sin(theta)));
//     vec<<r32-r23,r13-r31,r21-r12;
//     AngleAxis=theta*k*vec;
//     return AngleAxis;
// }

MatrixXd T2Axispos(Matrix<double, 4, 4> T)
{

    Matrix<double, 3, 3> rotation_matrix;
    Matrix<double, 3, 1> axis;
    Matrix<double, 1, 6> pos;
    rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::AngleAxisd angel_axisd(rotation_matrix);
    axis << angel_axisd.axis() * angel_axisd.angle();
    //      cout<<"axis="<<angel_axisd.axis()*angel_axisd.angle()<<endl;
    if (axis(1) > M_PI)
    {
        axis(1) = axis(1) - 2 * M_PI;
    }
    else if (axis(1) < -M_PI)
    {
        axis(1) = axis(1) + 2 * M_PI;
    }
    pos << T(0, 3), T(1, 3), T(2, 3), axis.transpose();
    return pos;
}
// 4*4位姿矩阵转化为1*6 Pos即位置xyz加轴角
Eigen::VectorXd T2Axis(const Eigen::Matrix4d &T)
{
    // 提取旋转矩阵元素
    double r11 = T(0, 0);
    double r12 = T(0, 1);
    double r13 = T(0, 2);
    double r21 = T(1, 0);
    double r22 = T(1, 1);
    double r23 = T(1, 2);
    double r31 = T(2, 0);
    double r32 = T(2, 1);
    double r33 = T(2, 2);

    // 计算旋转角度 theta
    //        cout<<"r11 + r22 + r33 - 1"<<","<<(r11 + r22 + r33 - 1) / 2<<endl;
    double theta;
    //        cout<<"T"<<T<<""<<","<<abs((r11 + r22 + r33 - 1) / 2)<<endl;
    Eigen::Vector3d angle_axis;
    if (std::abs(abs((r11 + r22 + r33 - 1) / 2) - 1) < eps)
    {
        // 当 theta 接近 0 时，轴角为零向量
        angle_axis.setZero();
    }
    else
    {
        theta = std::acos((r11 + r22 + r33 - 1) / 2);
        // 计算旋转轴 v
        Eigen::Vector3d v;
        v(0) = (r32 - r23) / (2 * std::sin(theta));
        v(1) = (r13 - r31) / (2 * std::sin(theta));
        v(2) = (r21 - r12) / (2 * std::sin(theta));

        // 计算轴角表示
        angle_axis = theta * v;
    }

    // 提取位置信息
    double x = T(0, 3);
    double y = T(1, 3);
    double z = T(2, 3);

    // 组合位置和轴角信息到 Pos 向量
    Eigen::VectorXd Pos(6);
    Pos << x, y, z, angle_axis(0), angle_axis(1), angle_axis(2);

    return Pos;
}
MatrixXd Axispos2T(Matrix<double, 1, 6> pos)
{
    Matrix<double, 4, 4> T;
    Matrix<double, 3, 1> vec;
    vec << pos(3), pos(4), pos(5);

    //    Eigen::AngleAxisd angel_axisd(rotation_matrix);
    Eigen::AngleAxisd rot_vec(vec.norm(), vec.normalized());
    Eigen::Matrix3d M = rot_vec.toRotationMatrix();
    T.block(0, 3, 3, 1) << pos(0), pos(1), pos(2);
    T.block(0, 0, 3, 3) << M;
    T.block(3, 0, 1, 4) << 0, 0, 0, 1;
    if (pos(3) == 0 && pos(4) == 0 && pos(5) == 0)
    {
        T.block(0, 0, 3, 3) << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;
    }
    return T;
}

// MatrixXd ndi2T(Matrix<double, 1, 6>  pos)
// {
//     Matrix<double, 1, 6> transpose;
//     Matrix<double, 4, 4> T;
//     double x=pos(0)*pow(10,-3),y=pos(1)*pow(10,-3),z=pos(2)*pow(10,-3),
//         rx=pos(3)*rad(),ry=pos(4)*rad(),rz=pos(5)*rad();
//     transpose<<x,y,z,rx,ry,rz;
//     T=TR(transpose);
//     return T;
// }

// MatrixXd displaypos(Matrix<double, 1, 6>  pos)
// {
//     Matrix<double, 1, 6> displaypos;
//     displaypos<<pos(0)*pow(10,3),pos(1)*pow(10,3),pos(2)*pow(10,3),
//         pos(3)*deg(),pos(4)*deg(),pos(5)*deg();
//     return displaypos;
// }
MatrixXd skew(Matrix<double, 1, 3> s)

{
    double vx = s(0), vy = s(1), vz = s(2);
    Matrix<double, 3, 3> R;
    R << 0, -vz, vy,
        vz, 0, -vx,
        -vy, vx, 0;
    return R;
}
MatrixXd line_interp(Matrix<double, 1, 6> p1, Matrix<double, 1, 6> p2, double dx)

{

    Matrix<double, 1, 6> dp, d, a, alp, offset;
    int m, n, j;
    double len, k;

    dp = p1 - p2;
    len = sqrt(pow(dp(0), 2) + pow(dp(1), 2) + pow(dp(2), 2));
    n = len / dx;
    MatrixXd Pos(n + 2, 6);
    m = n + 1;
    j = 1;
    for (int i = 0; i <= m; i++)
    {
        k = i / double(m);
        Pos.block(i, 0, 1, 6) = p1 + k * (p2 - p1);
    }
    return Pos;
}

MatrixXd circular_interp(Matrix<double, 1, 6> p1, Matrix<double, 1, 6> p2, Matrix<double, 1, 6> p3, double dx)

{

    Matrix<double, 1, 6> o13;
    int m, n, j;
    double len, k, x1, y1, z1, x2, y2, z2, x3, y3, z3, A1, B1, C1, D1, A2, B2, C2, D2, A3, B3, C3, D3, x0, y0, z0, r,
        ax, ay, az, L, d13, do2, theta, q;
    Matrix<double, 3, 3> A;
    Matrix<double, 3, 1> b, C, u, v, p;

    x1 = p1(0);
    x2 = p2(0);
    x3 = p3(0);
    y1 = p1(1);
    y2 = p2(1);
    y3 = p3(1);
    z1 = p1(2);
    z2 = p2(2);
    z3 = p3(2);

    A1 = (y1 - y3) * (z2 - z3) - (y2 - y3) * (z1 - z3);
    B1 = (x2 - x3) * (z1 - z3) - (x1 - x3) * (z2 - z3);
    C1 = (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
    D1 = -(A1 * x3 + B1 * y3 + C1 * z3);

    A2 = x2 - x1;
    B2 = y2 - y1;
    C2 = z2 - z1;
    D2 = -((pow(x2, 2) - pow(x1, 2)) + (pow(y2, 2) - pow(y1, 2)) + (pow(z2, 2) - pow(z1, 2))) / 2;

    A3 = x3 - x2;
    B3 = y3 - y2;
    C3 = z3 - z2;
    D3 = -((pow(x3, 2) - pow(x2, 2)) + (pow(y3, 2) - pow(y2, 2)) + (pow(z3, 2) - pow(z2, 2))) / 2;

    A << A1, B1, C1,
        A2, B2, C2,
        A3, B3, C3;
    b << -D1, -D2, -D3;

    //% 圆心
    C = A.inverse() * b;
    x0 = C(0);
    y0 = C(1);
    z0 = C(2);

    //% 外接圆半径
    r = sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2) + pow(z1 - z0, 2));

    //% 新坐标系Z0的方向余弦
    L = sqrt(pow(A1, 2) + pow(B1, 2) + pow(C1, 2));
    ax = A1 / L;
    ay = B1 / L;
    az = C1 / L;
    //% 新坐标系X0的方向余弦

    //%step2:得到13两点间的间距d
    d13 = sqrt(pow(p1(0) - p3(0), 2) + pow(p1(1) - p3(1), 2) + pow(p1(2) - p3(2), 2));
    o13 = (p1 + p3) / 2;

    do2 = sqrt(pow(o13(0) - p2(0), 2) + pow(o13(1) - p2(1), 2) + pow(o13(2) - p2(2), 2));
    // d13=sqrt((p1(1)-p3(1))^2+(p1(2)-p3(2))^2+(p1(3)-p3(3))^2);
    // o13=(p1+p3)/2;%得到13两点的中点，当弧角180度时，圆心在该点，以该点为衡量，判定优劣弧
    //%step3:得到13中点与2点间距do2
    // do2=sqrt((o13(1)-p2(1))^2+(o13(2)-p2(2))^2+(o13(3)-p2(3))^2);
    //%ste4:判断优弧还是劣弧
    if (do2 < d13 / 2)
    {
        theta = 2 * asin(d13 / 2 / r);
    }
    else
    {
        theta = 2 * M_PI - 2 * asin(d13 / 2 / r);
    }

    len = theta * r;
    //% 插补点数N
    n = len / dx;
    MatrixXd Pos(n + 2, 6);
    m = n + 1;
    v = (p1.block(0, 0, 1, 3).transpose() - C);
    u << ax, ay, az;
    j = 1;

    for (int i = 0; i <= m; i++)
    {
        k = i / double(m);
        q = theta * k;
        p = v * cos(q) + u.dot(v) * u * (1 - cos(q)) + u.cross(v) * sin(q) + C;

        Pos.block(i, 0, 1, 3) = p.transpose();
        Pos.block(i, 3, 1, 3) = p1.block(0, 3, 1, 3);
    }
    return Pos;
}

Eigen::MatrixXd pinv(Eigen::MatrixXd A) // 计算矩阵伪逆
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double pinvtoler = 1.e-8; // tolerance
    int row = A.rows();
    int col = A.cols();
    int k = min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); // 奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i < k; ++i)
    {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i)
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());

    return X;
}

// 正运动学 根据输入关节角度求出末端法兰相对于基座4*4矩阵
MatrixXd fkine(Matrix<double, 1, 6> theta)
{

    Matrix<double, 1, 6> d, a, alp, offset;
    Matrix<double, 4, 4> T1, T2, T3, T4, T5, T6, T7, T;
    Matrix<double, 1, 6> p;

    //    alp << 0,90,180,90,-90,90;
    //    a << 0,0,0.480015,0,0,0;
    //    d <<0.26,0,0,0.520137,0,0.192;
    //    offset<<0,90,90,0,0,0;
    //    alp=alp*rad();
    //    offset=offset*rad();

    //    alp<<0,-1.57104,-0.000294356,1.57027,-1.57031,1.56901;
    //    a<<   0,0.000175529,0.419961,0.000144852,-1.9803e-05,-0.069077;
    //    d<< 0.2665,-0.000116339,0,0.380242,-0.000232702,0.0951;
    //    offset<< 0,-1.56438,1.57089,0.00632152,-0.00257079,0;
    alp << 0, -1.57109, -0.000382754, 1.5709, -1.57184, 1.57136;
    a << 0, -3.37469e-05, 0.469603, -2.73644e-05, -0.000133765, -0.0776986;
    d << 0.326, -7.60105e-05, 0, 0.429973, 9.031e-05, 0.0973;
    offset << 0, -1.57001, 1.56756, -0.00575697, -0.00305482, 0;
    theta = theta + offset;
    T1 = MDHTrans(alp(0), a(0), d(0), theta(0));
    T2 = MDHTrans(alp(1), a(1), d(1), theta(1));
    T3 = MDHTrans(alp(2), a(2), d(2), theta(2));
    T4 = MDHTrans(alp(3), a(3), d(3), theta(3));
    T5 = MDHTrans(alp(4), a(4), d(4), theta(4));
    T6 = MDHTrans(alp(5), a(5), d(5), theta(5));
    //    T7 = MDHTrans(alp(6), a(6), d(6), theta(6));
    T = T1 * T2 * T3 * T4 * T5 * T6;
    // p = T_to_RPY(T);

    //    cout<<T<<endl;
    return T;
}

Eigen::Matrix<double, 1, 6> quinticInterp_CartSpace(
    double t,
    double totalTime,
    const Eigen::Matrix<double, 1, 6> &start,
    const Eigen::Matrix<double, 1, 6> &target)
{
    // 处理总时间为0的特殊情况
    if (totalTime <= 1e-6)
        return target;

    // 时间边界处理（C++14兼容）
    double clampedTime = t;
    if (clampedTime < 0.0)
        clampedTime = 0.0;
    if (clampedTime > totalTime)
        clampedTime = totalTime;
    const double ratio = clampedTime / totalTime;

    // 五次多项式核心计算
    const double r2 = ratio * ratio;
    const double polyTerm = r2 * ratio * (10 - 15 * ratio + 6 * r2);

    // 计算当前位置
    return start + (target - start) * polyTerm;
}

Eigen::Matrix<double, 1, 7> quinticInterp_JointSpace(
    double t,
    double totalTime,
    const Eigen::Matrix<double, 1, 7> &start,
    const Eigen::Matrix<double, 1, 7> &target)
{
    // 处理总时间为0的特殊情况
    if (totalTime <= 1e-6)
        return target;

    // 时间边界处理（C++14兼容）
    double clampedTime = t;
    if (clampedTime < 0.0)
        clampedTime = 0.0;
    if (clampedTime > totalTime)
        clampedTime = totalTime;
    const double ratio = clampedTime / totalTime;

    // 五次多项式核心计算
    const double r2 = ratio * ratio;
    const double polyTerm = r2 * ratio * (10 - 15 * ratio + 6 * r2);

    // 计算当前位置
    return start + (target - start) * polyTerm;
}
// Matrix<double, 4, 4>  eyehand_falan_calib(Matrix<double, 8, 6> Joint, Matrix<double, 8, 6> NDIpose)
// {
//     int len=8,nn=len-1;
//     MatrixXd T_base_tool(4,4*len),T_cam_cal(4,4*len);
//     MatrixXd M(4*len,4),N(4*len,4),sum_T_tool_cal(4,4);
//     sum_T_tool_cal.setZero(4,4);
//     Matrix<double, 3, 3>  A1,B1;
//     MatrixXd A(4,4*nn),B(4,4*nn);
//     MatrixXd AA(4*nn,4);
//     Matrix<double, 1, 6> joint;
//     Matrix<double, 1, 6> ndi_pos;

//     for (int i = 0; i < len;i++)
//     {
//         joint<<Joint.block(i,0,1,6);
//         T_base_tool.block(0,4*i,4,4) =fkine(joint);
//         ndi_pos<<NDIpose.block(i,0,1,6);
//         T_cam_cal.block(0,4*i,4,4)=ndi2T(ndi_pos);
//     }

//     for  (int i=0;i<len-1;i++)
//     {
//         int j=i+1;
//         A.block(0,4*i,4,4) =T_base_tool.block(0,4*i,4,4).inverse()*T_base_tool.block(0,4*j,4,4);
//         B.block(0,4*i,4,4) =T_cam_cal.block(0,4*i,4,4).inverse()*T_cam_cal.block(0,4*j,4,4);

//     }

//     for  (int i=0;i<len-1;i++)
//     {
//         A1=A.block(0,4*i,3,3);
//         B1=B.block(0,4*i,3,3);
//         Eigen::Quaterniond a(A1);
//         Eigen::Quaterniond b(B1);
//         Matrix<double, 1, 3>  m1,m2;
//         Matrix<double, 1, 4>  aq;
//         Matrix<double, 3, 3>  C1;
//         Matrix<double, 4, 4> AA1,BB1,dd;
//         m1<<a.x(),a.y(),a.z();
//         m2<<b.x(),b.y(),b.z();
//         aq<<a.x(),a.y(),a.z(),a.w();
//         MatrixXd::Identity(3,3) ;
//         C1.setIdentity(3,3) ;
//         AA1.block(0,0,1,4)<<a.w(),-a.x(),-a.y(),-a.z();
//         AA1.block(1,0,3,1)<<a.x(),a.y(),a.z();
//         AA1.block(1,1,3,3)=a.w()*C1+skew(m1);

//         BB1.block(0,0,1,4)<<b.w(),-b.x(),-b.y(),-b.z();
//         BB1.block(1,0,3,1)<<b.x(),b.y(),b.z();
//         BB1.block(1,1,3,3)=b.w()*C1-skew(m2);

//         AA.block(4*i,0,4,4)=AA1-BB1;

//     }

//     JacobiSVD<Eigen::MatrixXd> svd(AA,  ComputeThinU | ComputeThinV);
//     Matrix<double, 4, 4>T_tcp_calib, V = svd.matrixV();

//     Matrix<double, 4, 1> V1;
//     V1.block(0,0,3,1)=V.block(1,3,3,1);
//     V1(3,0)=V(0,3);
//     Eigen::Quaterniond v1q(V1);
//     Matrix<double, 3, 3> R=v1q.toRotationMatrix();

//     MatrixXd C(3*nn,3);
//     MatrixXd d(3*nn,1);
//     Matrix<double, 3, 3>  I;
//     I.setIdentity(3,3) ;
//     for  (int i=0;i<len-1;i++)
//     {
//         C.block(3*i,0,3,3)=I-A.block(0,4*i,3,3);
//         d.block(3*i,0,3,1)=A.block(0,4*i+3,3,1) -R*B.block(0,4*i+3,3,1);

//     }
//     Matrix<double, 3, 1>  t;
//     Matrix<double, 4, 4>  X1,t_tool_cal;
//     t=(C.transpose()*C).inverse()*C.transpose()*d;

//     T_tcp_calib.block(0,0,3,3)<<R;
//     T_tcp_calib.block(0,3,3,1)<<t;
//     T_tcp_calib.block(3,0,1,4)<<0,0,0,1;
//     return T_tcp_calib;
// }

// void Delay_mSec(unsigned int msec)
// {
//     QTime dieTime = QTime::currentTime().addMSecs(msec);
//     while (QTime::currentTime() < dieTime)
//         QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
// }

// 六个自由度和六维度力对应相乘的函数
MatrixXd multiplyWithDOF(Matrix<double, 1, 6> dof, Matrix<double, 1, 6> forces)
{
    Matrix<double, 1, 6> result;
    for (int i = 0; i < 6; ++i)
    {
        result[i] = dof[i] * forces[i];
    }
    return result;
}

MatrixXd slerp_interp(double t, Matrix<double, 1, 3> euler_start, Matrix<double, 1, 3> euler_end)
{
    Matrix<double, 4, 4> T_start, T_end;
    Matrix<double, 3, 3> R_start, R_end;

    Matrix<double, 1, 3> RPY_interpolated, euler;
    Matrix<double, 1, 4> q1, q2, q_inter;
    double epsilon = 1e-6, omega;
    T_start = Rot_zyx(euler_start);
    T_end = Rot_zyx(euler_end);
    R_start = T_start.block(0, 0, 3, 3);
    R_end = T_end.block(0, 0, 3, 3);
    Eigen::Quaterniond q_start(R_start);
    Eigen::Quaterniond q_end(R_end);
    q1 << q_start.w(), q_start.x(), q_start.y(), q_start.z();
    q2 << q_end.w(), q_end.x(), q_end.y(), q_end.z();

    q_inter = q1;
    omega = acos(q1.dot(q2));
    if (abs(omega) < epsilon)
    {
        q_inter = (1 - t) * q1 + q2 * t;
    }
    else
    {
        q_inter = (sin((1 - t) * omega) / sin(omega)) * q1 + (sin(t * omega) / sin(omega)) * q2;
    }
    q_inter = q_inter / q_inter.norm();
    Eigen::Quaterniond interpolated_quaternions(q_inter(0), q_inter(1), q_inter(2), q_inter(3));
    euler = Quaterniond2EulerAngles(interpolated_quaternions);

    return euler;
}
//********************************

int getFileRows(const char *fileName)
{
    ifstream fileStream;
    string tmp;
    int count = 0;                      // 行数计数器
    fileStream.open(fileName, ios::in); // ios::in 表示以只读的方式读取文件
    if (fileStream.fail())              // 文件打开失败:返回0
    {
        return 0;
    }
    else // 文件存在
    {
        while (getline(fileStream, tmp, '\n')) // 读取一行
        {
            if (tmp.size() > 0)
                count++;
        }
        fileStream.close();
        return count;
    }
}

int getFileColumns(const char *fileName)
{
    ifstream fileStream;
    fileStream.open(fileName, ios::in);

    double tmp = 0;
    int count = 0; // 列数计数器
    char c;        // 当前位置的字符
    c = fileStream.peek();
    while (('\n' != c) && (!fileStream.eof())) // 指针指向的当前字符，仅观测，不移动指针位置
    {
        fileStream >> tmp;
        ++count;
        c = fileStream.peek();
    }

    fileStream.close();
    return count;
}

double **getMatrix(const char *path, const int n, const int m)
{
    fstream myfile;
    myfile.open(path);

    double **mat = new double *[n];
    for (int i = 0; i < n; i++)
    {
        double *tmp = new double[m];
        mat[i] = tmp;
    }

    string tmpStr;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            myfile >> tmpStr; //
            double dValue = atof(tmpStr.c_str());
            mat[i][j] = dValue;
        }
    }
    return mat;
}

std::array<double, 3> extractCoordinates(const std::string &input)
{
    size_t start = input.find('[');
    size_t end = input.find(']', start);

    if (start == std::string::npos || end == std::string::npos)
    {
        throw std::invalid_argument("无效输入：未找到 '[' 或 ']'");
    }

    std::string coords_str = input.substr(start + 1, end - start - 1);
    std::stringstream ss(coords_str);
    std::array<double, 3> coords;
    char comma;

    if (!(ss >> coords[0] >> comma >> coords[1] >> comma >> coords[2]))
    {
        throw std::invalid_argument("无效输入：坐标格式错误");
    }



    return coords;
}

Eigen::Matrix3d Yaw_Rotation(float yaw)
{
    Eigen::Matrix3d T;
    T << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;
    return T;
}

Eigen::Matrix3d Pitch_Rotation(float pitch)
{
    Eigen::Matrix3d T;
    T << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
    return T;
}

Eigen::Matrix3d Roll_Rotation(float roll)
{
    Eigen::Matrix3d T;
    T << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);
    return T;
}

Eigen::Matrix4d neck_to_eye(int flag)
{
    int id30_position, id31_position, id32_position;
    // getPosition(hang_waist_id, 30, id30_position);
    // getPosition(hang_waist_id, 31, id31_position);
    // getPosition(hang_waist_id, 32, id32_position);

    id30_position = 0;
    // id31_position = 0;

    if (flag == 0)
    {
        id31_position = -20058;
    }

    else if (flag == 3)
    {
        id31_position = 10000;
    }
    else if (flag == 4)
    {
        id31_position = -10000;
    }
    else
    {
        id31_position = -22058;
    }

    id32_position = -0;

    // cout << "id30_position: " << id30_position << " " << "id31_position: " << id31_position << " " << "id32_position: " << id32_position << endl;
    float r_30 = id30_position / ((4 * 65536) / (2 * M_PI));
    float r_31 = id31_position / ((4 * 65536) / (2 * M_PI));
    float r_32 = id32_position / ((4 * 65536) / (2 * M_PI));
    // cout << "r_30: " << r_30 << " " << "r_31: " << r_31 << " " << "r_32: " << r_32 << endl;

    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T3 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T4 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T5 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T6 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T7 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T8 = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T9 = Eigen::Matrix4d::Identity();
    // Eigen::Vector3d
    // T1.block<3, 1>(0, 3) = pos;
    T1.block<3, 3>(0, 0) = Yaw_Rotation(r_30);
    // Eigen::Vector3d diff1; diff1 << 0, 0, 0;
    Eigen::Vector3d diff1;
    diff1 << 0, 0, 145;
    T2.block<3, 1>(0, 3) = diff1;
    T3.block<3, 3>(0, 0) = Roll_Rotation(M_PI / 2);
    T4.block<3, 3>(0, 0) = Yaw_Rotation(r_31);
    T5.block<3, 3>(0, 0) = Pitch_Rotation(M_PI / 2);
    T6.block<3, 3>(0, 0) = Yaw_Rotation(r_32);
    Eigen::Vector3d diff2;
    diff2 << 30, 135, 32;
    // Eigen::Vector3d diff2; diff2 << 35, 12, 95;
    T7.block<3, 1>(0, 3) = diff2;
    T8.block<3, 3>(0, 0) = Roll_Rotation(15 * M_PI / 180);
    T9.block<3, 3>(0, 0) = Yaw_Rotation(M_PI);

    return T1 * T2 * T3 * T4 * T5 * T6 * T7 * T8 * T9;
}

double calculateTf(const Eigen::Matrix<double, 1, 6> &axis_p1,
                   const Eigen::Matrix<double, 1, 6> &axis_p2,
                   double V_DEFAULT)
{
    // // 1. 提取平移和旋转分量
    // Eigen::Vector3d trans_p1 = axis_p1.head(3);  // 起始平移 (x,y,z)
    // Eigen::Vector3d trans_p2 = axis_p2.head(3);  // 目标平移 (x,y,z)
    // Eigen::Vector3d rot_p1 = axis_p1.tail(3);    // 起始旋转 (rx,ry,rz)
    // Eigen::Vector3d rot_p2 = axis_p2.tail(3);    // 目标旋转 (rx,ry,rz)

    Matrix<double, 1, 6> axis_p;
    Matrix<double, 4, 4> T_p1_p2, T_p1, T_p2;
    T_p1 = Axispos2T(axis_p1);
    T_p2 = Axispos2T(axis_p2);
    axis_p = T2Axispos(T_p1.inverse() * T_p2);

    // 2. 计算平移距离和旋转等效距离
    double d_trans = (axis_p.head(3)).norm(); // 平移距离 (m)
    double d_rot = (axis_p.tail(3)).norm();   // 旋转角度差 (rad)
    double d_rot_eq = 0.2 * d_rot;            // 旋转等效距离 (特征长度0.5m)

    // 3. 处理微小距离（无需运动）
    const double EPS = 1e-6; // 微小距离阈值 (m)
    bool trans_need_move = (d_trans > EPS);
    bool rot_need_move = (d_rot_eq > EPS);

    if (!trans_need_move && !rot_need_move)
    {
        return 0.0; // 已到达目标，无需运动
    }

    // 4. 分别计算平移和旋转的理论时间
    double t_trans = trans_need_move ? (d_trans / V_DEFAULT) : 0.0;
    double t_rot = rot_need_move ? (d_rot_eq / V_DEFAULT) : 0.0;

    // 5. 总时间取最大值（确保同步完成）
    double Tf = std::max(t_trans, t_rot);

    // 6. 强制最小时间约束（避免超短时间导致冲击）
    const double MIN_TF = 0.1; // 最小运动时间 (s)，可根据机械臂特性调整
    if (Tf < MIN_TF)
    {
        Tf = MIN_TF;
    }

    // 调试信息（可选）
    // std::cout << "平移距离: " << d_trans*1000 << "mm, 旋转等效距离: " << d_rot_eq*1000 << "mm\n";
    // std::cout << "平移时间: " << t_trans << "s, 旋转时间: " << t_rot << "s, 总时间: " << Tf << "s\n";

    return Tf;
}

Eigen::Matrix<double, 1, 7> joint_range_normalize(
    const Eigen::Matrix<double, 1, 7> &q,
    const Eigen::Matrix<double, 1, 7> &q_min,
    const Eigen::Matrix<double, 1, 7> &q_max)
{
    // 计算中点和单侧范围（向量化操作）
    Matrix<double, 1, 7> q_mid = (q_min + q_max) / 2.0;
    Matrix<double, 1, 7> half_range = (q_max - q_min) / 2.0;

    // 避免除零（向量化处理）
    half_range = half_range.cwiseMax(1e-6);

    // 计算归一化距离并限制在[0,1]（单步完成核心逻辑）
    return (q - q_mid).cwiseAbs().cwiseQuotient(half_range).cwiseMax(0.0).cwiseMin(1.0);
}

Eigen::Matrix<double, 1, 7> calc_segment_weight(
    const Eigen::Matrix<double, 1, 7> &x,
    double w_max)
{
    // 参数校验：w_max必须为正标量
    if (w_max <= 0)
    {
        throw std::invalid_argument("w_max必须是正标量（如100）！");
    }

    // 核心参数（常量表达式，编译时确定）
    constexpr double safe_threshold = 0.7;          // 安全区与警戒区分界点
    constexpr double safe_w_base = 1e-3;            // 安全区基础权重
    constexpr double growth_factor = 5;             // 警戒区增长系数
    constexpr double interval = 1 - safe_threshold; // 警戒区区间长度（0.3）

    // 初始化输出矩阵（1×7固定尺寸，无需手动指定大小）
    Eigen::Matrix<double, 1, 7> w;

    // 循环处理每个维度（利用固定尺寸特性，循环次数编译时确定）
    for (int i = 0; i < 7; ++i)
    {
        const double xi = x(i); // 访问1×7矩阵的第i列元素（0~6）

        if (xi <= safe_threshold)
        {
            // 安全区：x ≤ 0.7 → 基础权重
            w(i) = safe_w_base;
        }
        else if (xi < 1.0)
        {
            // 警戒区：0.7 < x < 1 → 指数增长到w_max
            const double ratio = (xi - safe_threshold) / interval; // 0~1比例
            w(i) = safe_w_base + (w_max - safe_w_base) * (1 - std::exp(-growth_factor * ratio));
        }
        else
        {
            // 超限区：x ≥ 1 → 最大权重
            w(i) = w_max;
        }
    }

    return w;
}


std::tuple<MatrixXd, MatrixXd, MatrixXd>
quinticInterp(
    double t,
    double totalTime,
    const MatrixXd& start,  // 统一 Matrixd：1×N
    const MatrixXd& target  // 统一 Matrixd：1×N
) {
    // 1. 输入合法性校验（新增“必须是1行”校验，适配关节数据）
    // 总时间为0：返回终点+零速度/加速度
    if (totalTime <= 1e-6) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {target, zero_mat, zero_mat};
    }
    // 起点/终点必须是1行（避免多行矩阵输入）
    if (start.rows() != 1 || target.rows() != 1) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {start, zero_mat, zero_mat};
    }
    // 起点/终点列数（关节数）必须一致
    if (start.cols() != target.cols()) {
        MatrixXd zero_mat(1, start.cols());
        zero_mat.setZero();
        return {start, zero_mat, zero_mat};
    }

    // 2. 时间边界夹紧（避免超界）
    double clampedTime = std::max(0.0, std::min(t, totalTime));
    double ratio = clampedTime / totalTime;  // 时间占比（0~1）

    // 3. 五次多项式核心项预计算（无冲击插值，工业级平滑）
    const double r2 = ratio * ratio;
    const double r3 = r2 * ratio;
    const double r4 = r3 * ratio;
    const double r5 = r4 * ratio;

    // 位置/速度/加速度项（解析导数，保证连续）
    const double pos_term = 10 * r3 - 15 * r4 + 6 * r5;
    const double vel_term = (30 * r2 - 60 * r3 + 30 * r4) / totalTime;
    const double acc_term = (60 * ratio - 180 * r2 + 120 * r3) / (totalTime * totalTime);

    // 4. 计算结果（Matrixd 直接运算，维度自动匹配 1×N）
    MatrixXd delta = target - start;  // 位置差（1×N）
    MatrixXd pos = start + delta * pos_term;
    MatrixXd vel = delta * vel_term;
    MatrixXd acc = delta * acc_term;

    return {pos, vel, acc};
}