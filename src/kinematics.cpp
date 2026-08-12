#include "kinematics.h"

#include "Ti5_socketcan.h"

Robot::Robot(int mod)
{
    Matrix<double, 1, 7> q_limit_min, q_limit_max;
    Matrix<double, 7, 1> theta, d, a, alpha, offset;
    if (mod == -1)
    {
        alpha << M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2;
        a << 0, 0, 0, 0, 0, 0, 0;
        d << d1, 0, d3, 0, d5, 0, d7;
        offset << M_PI / 2, -M_PI / 2, M_PI / 2, 0, 0, -M_PI / 2, 0;
        MDH << alpha, a, d, offset;
        // tool << 0, 0, 0, 0, 0, 0;
        tool << 150e-3, 20e-3, 0, 0, 0, 0;
        q_limit_min << -90, -90, -90, 0,  -90, -60, -90;
        q_limit_max <<  90,   0,  90,  140, 90, 60, 90;
        q_limit_min *= deg2rad;
        q_limit_max *= deg2rad;
        limit.row(0) = q_limit_min;
        limit.row(1) = q_limit_max;
        select = 1;
        q2motor_direct << 1, 1, 1, 1, 1, 1, 1;
        // q2motor_offset << 0, M_PI / 2, -M_PI / 2, 0, M_PI / 2, 0, 0; // 电机零位向模型零位转动
        q2motor_offset << 0, M_PI / 2, 0, 0, 0, 0, 0; // 电机零位向模型零位转动

        MotorsIDlist[0] = 16;
        MotorsIDlist[1] = 17;
        MotorsIDlist[2] = 18;
        MotorsIDlist[3] = 19;
        MotorsIDlist[4] = 20;
        MotorsIDlist[5] = 21;
        MotorsIDlist[6] = 22;
        armID = 1;
        hand_flag = 1;
        q2motorid =-1;
    }

    if (mod == 1)
    {
        alpha << M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2;
        a << 0, 0, 0, 0, 0, 0, 0;
        d << -d1, 0, d3, 0, d5, 0, d7;
        offset << M_PI / 2, -M_PI / 2, M_PI / 2, 0, 0, -M_PI / 2, 0;
        MDH << alpha, a, d, offset;
        // tool << 0, 0, 0, 0, 0, 0;
        tool << 150e-3, 20e-3, 0, 0, 0, 0;
        q_limit_min << -90, 0, -90, 0, -90, -60, -90;
        q_limit_max << 90, 90, 90, 140, 90, 60, 90;
        q_limit_min *= deg2rad;
        q_limit_max *= deg2rad;
        limit.row(0) = q_limit_min;
        limit.row(1) = q_limit_max;
        select = 1;
        q2motor_direct << -1, 1, 1, -1, 1, -1, 1;
        q2motor_offset << 0, -M_PI / 2, 0, 0, 0, 0, 0;
        MotorsIDlist[0] = 23;
        MotorsIDlist[1] = 24;
        MotorsIDlist[2] = 25;
        MotorsIDlist[3] = 26;
        MotorsIDlist[4] = 27;
        MotorsIDlist[5] = 28;
        MotorsIDlist[6] = 29;
        armID = 1;
        hand_flag = 0;
        q2motorid =1;
    }
}; // 带默认参数的构造函数

bool Robot::check_Joint_withlimit(Matrix<double, 1, 7> qc)
{
    Matrix<double, 1, 7> q_limit_min, q_limit_max;
    q_limit_min = limit.row(0);
    q_limit_max = limit.row(1);
    for (int i = 0; i < 7; i++)
    {
        if (qc(i) < q_limit_min(i) || qc(i) > q_limit_max(i))
        {
            return false;
        }
    }
    return true;
}

Matrix<double, 1, 6> Robot::getTool()
{
    return TR(tool);
}

MatrixXd Robot::q2MotorAngle(Matrix<double, 1, 7> &q)
{
    Matrix<double, 1, 7> MotorAngle;
    MotorAngle = q.cwiseProduct(q2motor_direct) + q2motor_offset;
    return MotorAngle;
}

MatrixXd Robot::MotorAngle2q(Matrix<double, 1, 7> &MotorAngle)
{
    Matrix<double, 1, 7> q;
    q = (MotorAngle - q2motor_offset).cwiseQuotient(q2motor_direct);
    return q;
}

MatrixXd Robot::getJointPos()
{
    Matrix<double, 1, 7> motor_q, q;
    int MotorPosition[dof];
    Eigen::MatrixXd dataList;
    // uint8_t MotorsIDlist[7];
    // 赋值操作

    if (hand_flag == 0)
    {
        get_motor_position(MotorPosition, 0);
    }
    else
    {
        get_motor_position(MotorPosition, 1);
    }

    // sendSimpleCanCommand(armID, 0, dof, MotorsIDlist, GET_PERSIONS, MotorPosition);
    //  for (int i = 0; i < dof; i++)
    //  {
    //      cout << "MotorPosition[i]: " << MotorPosition[i] << " ";
    //  }
    //  cout << endl;
    //  cout<<"MotorsIDlist="<<MotorsIDlist[0]<<","<<MotorsIDlist[1]<<endl;
    //  getPosition(0,0,dof,MotorsIDlist,MotorPosition);

//    cout << "电机7ge角度：" << MotorPosition[0] << "  " << MotorPosition[1] << "  " << MotorPosition[2] << "  " << MotorPosition[3] << "  " << MotorPosition[4]
       //  << "  " << MotorPosition[5] << "  " << MotorPosition[6] << endl;

    for (int i = 0; i < dof; i++)
    {

        motor_q(i) = static_cast<double>(MotorPosition[i]) / rad2cnt;
    }
    q = MotorAngle2q(motor_q);
    // cout << "q: " << endl
    //      << q << endl;

    return q;
}

MatrixXd Robot::getTcpPos()
{
    Matrix<double, 1, 7> motor_q, q;
    Matrix<double, 4, 4> T;
    Matrix<double, 1, 6> pos;
    q = getJointPos();
    // cout << "当前关节度角：" << q * 180 / 3.1415926 << endl;
    T = fk(q);
    pos = T2PosEulerAngles(T);
    return pos;
}

// 雅可比矩阵计算函数-矢量积法
MatrixXd Robot::J_tcp_crossproduct(const Matrix<double, 1, 7> &q)
{
    VectorXd alp = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);
    VectorXd q_offset = q.transpose() + offset;

    Matrix4d A1 = MDHTrans(alp(0), a(0), d(0), q_offset(0));
    Matrix4d A2 = MDHTrans(alp(1), a(1), d(1), q_offset(1));
    Matrix4d A3 = MDHTrans(alp(2), a(2), d(2), q_offset(2));
    Matrix4d A4 = MDHTrans(alp(3), a(3), d(3), q_offset(3));
    Matrix4d A5 = MDHTrans(alp(4), a(4), d(4), q_offset(4));
    Matrix4d A6 = MDHTrans(alp(5), a(5), d(5), q_offset(5));
    Matrix4d A7 = MDHTrans(alp(6), a(6), d(6), q_offset(6));
    Matrix4d A8 = TR(tool);

    Matrix4d T_b_end = A1 * A2 * A3 * A4 * A5 * A6 * A7 * A8;
    Matrix4d T1 = fk(q);
    Vector3d p_ee = T1.block<3, 1>(0, 3);

    MatrixXd J(6, 7);
    J.setZero();
    Matrix4d T = Matrix4d::Identity();
    for (int i = 0; i < 7; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q_offset(i));
        T = T * T_i;
        Vector3d z_i = T.block<3, 1>(0, 2);
        Vector3d p_i = T.block<3, 1>(0, 3);
        J.block<3, 1>(0, i) = z_i.cross(p_ee - p_i);
        J.block<3, 1>(3, i) = z_i;
    }

    MatrixXd transform(6, 6);
    transform.setZero();
    transform.block<3, 3>(0, 0) = T_b_end.block<3, 3>(0, 0);
    transform.block<3, 3>(3, 3) = T_b_end.block<3, 3>(0, 0);

    return transform.transpose() * J;
}

// 正向运动学函数
Matrix4d Robot::fk(const Matrix<double, 1, 7> &q)
{
    int n = dof;
    Matrix4d T = Matrix4d::Identity();
    VectorXd alp = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);

    for (int i = 0; i < n; ++i)
    {
        Matrix4d T_i = MDHTrans(alp(i), a(i), d(i), q(i) + offset(i));
        T = T * T_i;
    }
    // cout<<"Tool="<<tool<<endl;
    Matrix4d T_tcp = T * TR(tool);
    return T_tcp;
}

// 逆运动学函数
int Robot::ik(const Matrix<double, 1, 6> &pos, Matrix<double, 1, 7> &q, double q2)
{
    int ret = -1;

  
    double t3Ratio;
    double of = M_PI / 2;
    Matrix4d T_falan_tcp = TR(tool);
    Matrix4d T_base_tcp = TR(pos);
    Matrix4d T_base_falan = T_base_tcp * T_falan_tcp.inverse();
    Matrix4d T7 = T_base_falan;

    VectorXd alpha = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);
    double d1 = d(0);

    Matrix<double, 4, 1> p6 = T7 * Matrix<double, 4, 1>(0, 0, 0, 1);

    int i = 0;
    Matrix<double, 8, 7> allSolutions; // 固定大小矩阵

    double d31 = sqrt(d(2) * d(2) + a(3) * a(3));
    double d51 = sqrt(d(4) * d(4) + a(4) * a(4));

    d(2) = d31;
    d(4) = d51;

    Matrix<double, 4, 1> p2(0, -d1, 0, 1);
    double d26 = sqrt(pow(p2(0) - p6(0), 2) + pow(p2(1) - p6(1), 2) + pow(p2(2) - p6(2), 2));
    double t4_thresold = 3 * rad;
    // 第一个 if 条件
    if (d26 > abs(d(2) - d(4)) && d26 < d(2) + d(4))
    {
        for (double t4 : {M_PI - acos((d(2) * d(2) + d(4) * d(4) - d26 * d26) / (2 * d(2) * d(4))),
                          -(M_PI - acos((d(2) * d(2) + d(4) * d(4) - d26 * d26) / (2 * d(2) * d(4))))})
        {

            if (abs(t4) < t4_thresold)
            {
                cout << "QiYi_Reach" << "\n";
                return -2;
            }

            double t2 = q2;
            double px = p6(0);
            double py = p6(1);
            double pz = p6(2);
            double t3N = py + d(0) - (d(2) + cos(t4) * d(4)) * sin(q2);
            double t3D = -d(4) * sin(t4) * cos(q2);
            if (abs(t3N) < eps && abs(t3D) < eps)
            {
                t3Ratio = 0;
            }
            else
            {
                t3Ratio = t3N / t3D;
            }

            // 第二个 if 条件
            if (abs(t3Ratio) < 1 - eps)
            {
                for (double t3 : {asin(t3Ratio),
                                  M_PI - asin(t3Ratio)})
                {
                    double m = d(4) * cos(t3) * sin(t4);
                    double n = (cos(t2) * cos(t4) + sin(t2) * sin(t3) * sin(t4)) * d(4) + d(2) * cos(t2);

                    double A1 = (m * pz + n * px) / (m * m + n * n);
                    double B1 = (m * px - n * pz) / (m * m + n * n);
                    double t1 = atan2(A1, B1);

                    Matrix4d A1_mat = MDHTrans(alpha(0), a(0), d1, t1 + offset(0));
                    Matrix4d A2 = MDHTrans(alpha(1), a(1), d(1), t2 + offset(1));
                    Matrix4d A3 = MDHTrans(alpha(2), a(2), d(2), t3 + offset(2));
                    Matrix4d A4 = MDHTrans(alpha(3), a(3), d(3), t4 + offset(3));

                    Matrix4d T4 = A1_mat * A2 * A3 * A4;
                    Matrix4d T74 = T4.inverse() * T7;

                    // 第三个 if 条件
                    if (abs(T74(1, 2)) < 1 - eps)
                    {
                        for (double t6 : {acos(T74(1, 2)) + of, -acos(T74(1, 2)) + of})
                        {
                            if (i < 8)
                            {
                                double A5 = -T74(2, 2) / sin(t6 - of);
                                double B5 = -T74(0, 2) / sin(t6 - of);
                                double t5 = atan2(A5, B5);
                                double A7 = -T74(1, 1) / sin(t6 - of);
                                double B7 = T74(1, 0) / sin(t6 - of);
                                double t7 = atan2(A7, B7);

                                Matrix<double, 1, 7> theta;
                                theta << t1, t2, t3, t4, t5, t6, t7;

                                allSolutions.row(i) = theta;
                                i++;
                                ret = 1;
                            }
                            else
                            {
                                // 解的数量超过了 allSolutions 能存储的最大数量
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    if (ret == 1)
    {
        // 对所有解进行归一化处理
        for (int k = 0; k < i; ++k)
        {
            for (int j = 0; j < 7; ++j)
            {
                if (allSolutions(k, j) < -M_PI)
                {
                    allSolutions(k, j) += 2 * M_PI;
                }
                else if (allSolutions(k, j) > M_PI)
                {
                    allSolutions(k, j) -= 2 * M_PI;
                }
                else if (abs(abs(allSolutions(k, j)) - M_PI) < eps)
                {
                    allSolutions(k, j) = 0;
                }
            }
        }
        // cout<<"select="<<select<<endl;
        // cout<<"limit="<<limit*deg<<endl;

        double sum2 = (allSolutions.row(1) - q).cwiseAbs().sum();
        double sum4 = (allSolutions.row(3) - q).cwiseAbs().sum();
        // cout<<"allSolutions="<<allSolutions*deg<<", sum2="<<sum2<<", sum4="<<sum4<<endl;
        if (i >= 2)
        {
            if (sum2 <= sum4)
            {
                select = 1;
            }
            else
            {
                select = 3;
            }

            Matrix<double, 1, 7> selectedQ = allSolutions.row(select);
            bool withinLimit = true;
            for (int j = 0; j < 7; ++j)
            {
                if (selectedQ(0, j) < limit(0, j) || selectedQ(0, j) > limit(1, j))
                {
                    withinLimit = false;
                    return -2;
                    break;
                }
            }
            if (withinLimit)
            {
                ret = 0;
                q = selectedQ;
            }
        }
    }

    return ret;
}

// 核心函数：输入关节弧度值，无阻塞批量发送到电机
int Robot::servoJ(Matrix<double, 1, 7> &qd)
{
    Matrix<double, 1, 7> motor_q;
    motor_q = q2MotorAngle(qd);
    // 1. 弧度值转换为电机脉冲值（MotorPosition）
    int motorPositions[dof]; // 存储转换后的脉冲值
    for (int i = 0; i < dof; ++i)
    {
        // 转换公式：脉冲值 = (弧度值 * 减速比 * 分辨率) / π
        motorPositions[i] = static_cast<int32_t>(motor_q(i) * rad2cnt);
    }

    if (hand_flag == 0)
    {
        set_motor_position(motorPositions, 0);
    }
    else
    {
        set_motor_position(motorPositions, 1);
    }
    // 2. 调用非阻塞批量发送函数（复用优化后的sendCanCommand）
    // setPosition(armID, dof, MotorsIDlist, motorPositions);
    // cin >> stop;
    // sendCanCommand(0, 0, dof, MotorsIDlist, SET_MOTOR_POSITION , motorPositions);
    return 0; // 返回0表示成功提交（非阻塞模式不保证硬件已发送）
}

int Robot::moveJToPos(Matrix<double, 1, 6> &posd, double q2)
{
    Matrix<double, 1, 7> qc, q, qd;
    double t, dt, Tf;

    qc = getJointPos();
    // q3=q3*rad;
    int ret = ik(posd, qc, q2);
    qd = qc;
  //  cout << "ret :" << ret << endl;
    //cout << "qd:" << qd * deg << endl;
    t = 0;
    Tf = 2;
    dt = 5e-3;

    // cout<<"qcc="<<qc*rad2deg<<endl;
    // cout<<"qdd="<<qd*rad2deg<<endl;
    while (t < Tf)
    {
        q = quinticInterp_JointSpace(t, Tf, qc, qd);
        //  set_motor_position(0, 0, dof,MotorsIDlist,MotorPosition);
        servoJ(q);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        t = t + dt;
        // cout<<"q="<<q*rad2deg<<endl;
    }
    return 0;
}

int Robot::moveJtoJoint(Matrix<double, 1, 7> &qd)
{
    Matrix<double, 1, 7> qc, q, q_current;
    double t, dt, Tf;

    qc = getJointPos();
    // q3=q3*rad;
    //cout << "qcstart" << qc * deg << endl;
    //cout << "qdend" << qd * deg << endl;
    t = 0;
    Tf = 1.5;
    dt = 2e-3;
    auto cycle_start = chrono::high_resolution_clock::now();
    // cout<<"qcc="<<qc*rad2deg<<endl;=
    // cout<<"qdd="<<qd*rad2deg<<endl;
    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        q = quinticInterp_JointSpace(t, Tf, qc, qd);
        // q_current = getJointPos();
        //  set_motor_position(0, 0, dof,MotorsIDlist,MotorPosition);
        servoJ(q);

        double elapsed_ms = chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();
        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = chrono::duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }
        // std::this_thread::sleep_for(std::chrono::microseconds(4));
        t = t + dt;

        // static int print_counter = 0;
        // if (++print_counter % 1 == 0)
        // {
        //     cout << "t=  " << t << ", dt" << dt << ", ms=" << elapsed_ms << ", q=" << q * deg << "\n";
        // }
    }

    //     cycle_start=chrono::high_resolution_clock::now();
    //     q=quinticInterp_JointSpace(t, Tf, q_start, q_end);

    //     qc=getJointPos();

    //     //  set_motor_position(0, 0, dof,MotorsIDlist,MotorPosition);
    //     servoJ(  q) ;
    //     // speedJ(dq);

    //     double elapsed_ms=duration< double,std::milli> (chrono::high_resolution_clock::now()-cycle_start).count();

    //     double sleep_ms=max(0.0,dt*1000-elapsed_ms);
    //     if(sleep_ms>1e-6)
    //     {
    //        auto sleep_duration= duration< double,std::milli> (sleep_ms);
    //         std::this_thread::sleep_for(sleep_duration);
    //     }
    // // std::this_thread::sleep_for(std::chrono::milliseconds(2));

    //     t=t+dt;
    //     // cout<<t<<","<<elapsed_ms<<","<<dq<<endl;

    //     static int print_counter=0;
    //     if(++print_counter %1 ==0)
    //     {
    //         cout<<"t=  "<<t<<", dt"<<dt<<", ms="<<elapsed_ms<<", q="<<q*deg<<", qc="<<qc*deg  <<"\n";

    //     }

    return 0;
}

int Robot::moveLToPos(Matrix<double, 1, 6> &posd, double vel)
{
    Matrix<double, 1, 6> axis_pos, posc, pos1, pos2, axis_p1, axis_p2, v_tcp;
    Matrix<double, 1, 7> qc, q, q1, q_current;
    Matrix<double, 7, 1> dq;
    int32_t MotorPosition[dof];
    Matrix<double, 4, 4> T1, T2, Tc, Td, Tv;
    MatrixXd J(6, 7);
    J.setZero();
    double t = 0, dt, lamda,Tf;

     int ret1=check_workspace_access(posd);
    //  if(ret1==-1)
    //  {
    //     cout<<"WorkSpace_Limit"<<"\n";
    //     return -1;
    //  }
      if(vel <0 || vel>0.7)
     {
        cout<<"Vel_Limit"<<"\n";
        return -1;
     }
    qc = getJointPos();  
    // cout << "q_start=" << qc * rad2deg << endl;

    T1 = fk(qc);
    // cout << "T1: " << endl
    //      << T1 << endl;
    pos2 = posd;
    T2 = TR(pos2);
    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);
    // qc=q1;
    lamda = 200;

    dt = 5e-3;
    Tf = calculateTf(axis_p1, axis_p2, vel);
    // cin >> stop;
    auto cycle_start = chrono::high_resolution_clock::now();
    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        axis_pos = quinticInterp_CartSpace(t, Tf, axis_p1, axis_p2);

        // qc=getJointPos();
        //   qc=q;
        Tc = fk(qc);
        Td = Axispos2T(axis_pos);
        Tv = Tc.inverse() * Td;
        v_tcp = T2Axispos(Tv)/dt;
        J = J_tcp_crossproduct(qc);
        dq = pinv(J) * v_tcp.transpose() ;
        qc = qc + dq.transpose() * dt;
        // posc = getTcpPos();
        if (check_Joint_withlimit(qc) == false)
        {
            cout<< "qc "<<qc * deg<<"\n";
            cout<<"Joint_Limit"<<"\n";
            break;
        }
        servoJ(qc);
        double elapsed_ms = chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = chrono::duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t = t + dt;
        static int print_counter = 0;
    //    if (++print_counter % 10 == 0)
    //    {
    //        q_current=getJointPos();
    //     cout << "t=  " << t << ", dt" << dt << ", ms=" << elapsed_ms << ", q=" << qc * deg << ", qc=" << q_current * deg << "\n";
    //    }
    }

    return 0;
}

int Robot::check_workspace_access(Matrix<double, 1, 6> &pos)
{
    int ret = -1;

    Matrix4d T_falan_tcp = TR(tool);
    Matrix4d T_base_tcp = TR(pos);
    Matrix4d T_base_falan = T_base_tcp * T_falan_tcp.inverse();
    Matrix4d T7 = T_base_falan;

    VectorXd alpha = MDH.col(0);
    VectorXd a = MDH.col(1);
    VectorXd d = MDH.col(2);
    VectorXd offset = MDH.col(3);
    double d1 = d(0);

    Matrix<double, 4, 1> p6 = T7 * Matrix<double, 4, 1>(0, 0, 0, 1);

    int i = 0;
    Matrix<double, 8, 7> allSolutions; // 固定大小矩阵

    double d31 = sqrt(d(2) * d(2) + a(3) * a(3));
    double d51 = sqrt(d(4) * d(4) + a(4) * a(4));

    d(2) = d31;
    d(4) = d51;

    Matrix<double, 4, 1> p2(0, -d1, 0, 1);
    double d26 = sqrt(pow(p2(0) - p6(0), 2) + pow(p2(1) - p6(1), 2) + pow(p2(2) - p6(2), 2));

    // 第一个 if 条件
    if ((d26 > abs(d(2) - d(4)) && d26 < d(2) + d(4)))
    {

        double t4 = M_PI - acos((d(2) * d(2) + d(4) * d(4) - d26 * d26) / (2 * d(2) * d(4)));
        if (abs(t4) > 5 * rad)
        {
            return 0;
        }
    }
    return -1;
    // for (double t4 : {pi - acos((d(2) * d(2) + d(4) * d(4) - d26 * d26) / (2 * d(2) * d(4)))
}

int Robot::moveLToPos_eyehand(Matrix<double, 1, 6> &posd, double vel)
{

    Matrix<double, 1, 6> axis_pos, posc, pos1, pos2, axis_p1, axis_p2, v_tcp, pos_err;
    Matrix<double, 1, 7> qc, q, q1;
    Matrix<double, 7, 1> dq;
    int32_t MotorPosition[dof];
    Matrix<double, 4, 4> T1, T2, Tc, Td, Tv;
    MatrixXd J(6, 7);
    J.setZero();

    double t = 0, dt, lamda, Tf;

    // q<<0,0,0,1,0,0,0;
    // cout<<"q"<<q<<endl;
    // pos1<<0.3,-0.3,-0.3,-90*rad,90*rad,0;
    qc = getJointPos();
    //cout << "q_start=" << qc * deg << endl;

    T1 = fk(qc);
    pos2 = posd;
    T2 = TR(pos2);
    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);

    int ret1 = check_workspace_access(posd);
    if (ret1 == -1)
    {
        cout << "too far" << endl;
        return -2;
    }

    pos_err = axis_p2 - axis_p1;
    if (pos_err.norm() < 10e-3)
    {
        cout << "too jing" << endl;
        return -1;
    }

    Tf = calculateTf(axis_p1, axis_p2, vel);
    // qc=q1;
    lamda = 200;
    // Tf=2;
    dt = 2e-3;
     auto cycle_start = chrono::high_resolution_clock::now();
    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        axis_pos = quinticInterp_CartSpace(t, Tf, axis_p1, axis_p2);

        //qc = getJointPos();
        //   qc=q;
        Tc = fk(qc);
        Td = Axispos2T(axis_pos);
        Tv = Tc.inverse() * Td;
        v_tcp = T2Axispos(Tv);
        J = J_tcp_crossproduct(qc);
        dq = pinv(J) * v_tcp.transpose() * lamda;
        qc = qc + dq.transpose() * dt;

        int ret = checkJointLimits(qc);
        if (ret == -1)
        {
            cout << "q_reach_Limit=" << qc * deg << endl;
            return -1;
        }
        servoJ(qc);
          double elapsed_ms = chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = chrono::duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t = t + dt;
        // static int print_counter = 0;
        // cout<<"t="<<t<<","<<qc*deg<<endl;
    }

    return 0;
}

int Robot::checkJointLimits(Matrix<double, 1, 7> &qc)
{
    for (int i = 0; i < 7; ++i)
    {
        if (qc(i) < limit(0, i) || qc(i) > limit(1, i))
        {
            return -1;
        }
    }
    return 0;
}

int Robot::moveLToPos_Opt(Matrix<double, 1, 6> &posd, double vel)
{
    Matrix<double, 1, 6> axis_pos, posc, pos1, pos2, axis_p1, axis_p2, v_tcp;
    Matrix<double, 1, 7> qc, dq, q, q1, q_current, dq_current;
    // Matrix<double, 7, 1> dq;
    int32_t MotorPosition[dof];
    Matrix<double, 4, 4> T1, T2, Tc, Td, Tv;
    MatrixXd J(6, 7);
    J.setZero();
    double t = 0, dt, lamda, Tf, k;
    k = 0.5;
    int ret1 = check_workspace_access(posd);
    if (ret1 == -1)
    {

        cout << "WorkSpace_Limit" << endl;
        return -1;
    }
    qc = getJointPos();
    // if (abs(qc(3)) < 1 * rad)
    // {
    //     cout << "Robot_QiYI" << endl;
    //     return -1;
    // }

    cout << "q_start=" << qc * deg << endl;

    T1 = fk(qc);
    pos2 = posd;
    T2 = TR(pos2);
    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);
    Tf = calculateTf(axis_p1, axis_p2, vel);
    dt = 5e-3;
    auto cycle_start = chrono::high_resolution_clock::now();
    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        axis_pos = quinticInterp_CartSpace(t, Tf, axis_p1, axis_p2);

        Tc = fk(qc);
        Td = Axispos2T(axis_pos);
        Tv = Tc.inverse() * Td;
        v_tcp = T2Axispos(Tv) / dt;
        // J = J_tcp_crossproduct(qc);

        dq = quadratic_cost_function(qc, v_tcp);
        qc = qc + dq * dt;
        if (check_Joint_withlimit(qc) == false)
        {
            cout<< "qc "<<qc * deg<<"\n";
            cout<<"Joint_Limit"<<"\n";
            break;
        }
        servoJ(qc);
        double elapsed_ms = chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = chrono::duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }

        t = t + dt;

        static int print_counter = 0;
        // if (++print_counter % 10 == 0)
        // {
            
        //     cout << "t=  " << t << ", dt" << dt << ", ms=" << elapsed_ms << ", qdesired=" << qc * deg << ", qcurrent=" << q_current * deg << "\n";
        // }
    }

    return 0;
}


Matrix<double, 1, 7> Robot::quadratic_cost_function(
    const Eigen::Matrix<double, 1, 7> &q,
    const Eigen::Matrix<double, 1, 6> &Vtcp

)
{
    // 1. 参数初始化
    Eigen::Matrix<double, 1, 7> w, q_min, q_max;
    w << 5, 10, 5, 0, 5, 10, 5; // 关节权重

    double k = 0.5;      // 梯度下降系数
    double lambda = 0.1; // 阻尼系数

    q_min = limit.row(0);
    q_max = limit.row(1);

    // 2. 计算雅可比矩阵
    Eigen::Matrix<double, 6, 7> J_tcp = J_tcp_crossproduct(q);

    // 3. 计算代价函数梯度（关节极限约束）
    Eigen::Matrix<double, 7, 1> grad;                          // 梯度向量(7×1)
    Eigen::Matrix<double, 1, 7> q_mid = (q_min + q_max) / 2.0; // 关节中间位置
    // q_mid << 0, 0, 0, 0, 0, 0, 0;
    // q_mid = q_mid * rad;
    // q_mid(2)=-90*rad;
    //  q_mid(4)=90*rad;

    Eigen::Matrix<double, 1, 7> delta_q = q_max - q_min; // 关节活动范围

    for (int i = 0; i < 7; ++i)
    {
        double half_range = delta_q(i) / 2.0;
        grad(i) = w(i) * (q(i) - q_mid(i)) / (half_range * half_range);
    }

    // 4. 计算零空间投影矩阵
    Eigen::Matrix<double, 7, 7> I7 = Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::Matrix<double, 7, 6> J_pinv = J_tcp.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Matrix<double, 7, 7> null_proj = I7 - J_pinv * J_tcp;
    // 5. 计算零空间调整速度
    Eigen::Matrix<double, 7, 1> q_dot_null = -k * grad;
    Eigen::Matrix<double, 7, 1> dq_null = null_proj * q_dot_null;

    // 6. 计算主任务关节速度（阻尼最小二乘法）
    Eigen::Matrix<double, 6, 1> Vtcp_col = Vtcp.transpose(); // 转换为列向量
    Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 7, 1> dq_main = J_tcp.transpose() * (J_tcp * J_tcp.transpose() + lambda * lambda * I6).ldlt().solve(Vtcp_col);

    // 7. 总关节速度（主任务+零空间）
    Eigen::Matrix<double, 1, 7> dq = (dq_main + dq_null).transpose();
    
    return dq;
}
int Robot::moveLToPosIK(Matrix<double, 1, 6> &posd, double vel, double q2d)
{

    
    Matrix<double, 1, 6> axis_pos, posc, pos1, pos2, axis_p1, axis_p2, v_tcp;
    Matrix<double, 1, 7> qc, q, q1, q_current, dq_current;
    Matrix<double, 7, 1> dq;
    int32_t MotorPosition[dof];
    Matrix<double, 4, 4> T1, T2, Tc, Td, Tv;
    MatrixXd J(6, 7);
    J.setZero();
    double t = 0, dt, lamda, Tf, k;
    k = 0.5;
    int ret1 = check_workspace_access(posd);
    if (ret1 == -1)
    {

        cout << "WorkSpace_Limit" << endl;
        return -1;
    }
    qc = getJointPos();
    // if (abs(qc(3)) < 5 * rad)
    // {
    //     cout << "Robot_QiYI" << endl;
    //     return -1;
    // }


    T1 = fk(qc);
    pos2 = posd;
    T2 = TR(pos2);
    axis_p1 = T2Axispos(T1);
    axis_p2 = T2Axispos(T2);
    Tf = calculateTf(axis_p1, axis_p2, vel);
    dt = 5e-3;
    auto cycle_start = chrono::high_resolution_clock::now();

    double q2, q20 = qc(1);
    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        axis_pos = quinticInterp_CartSpace(t, Tf, axis_p1, axis_p2);

        q2 = q20 * (1 - t / Tf) + q2d * t / Tf;
        Tc = fk(qc);
        Td = Axispos2T(axis_pos);
        posd = T2PosEulerAngles(Td);
        int ret = ik(posd, qc, q2);
        if (check_Joint_withlimit(qc) == false)
        {
            cout<< "qc "<<qc * deg<<"\n";
            cout<<"Joint_Limit"<<"\n";
            // return -1;
            // break;
        }
        servoJ(qc);
        double elapsed_ms = chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = chrono::duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }

        t = t + dt;

        // static int print_counter = 0;
        // if (++print_counter % 10 == 0)
        // {
            
        //     cout << "t=  " << t << ", dt" << dt << ", ms=" << elapsed_ms << ", qc=" << qc * deg << "\n";
        // }
    }

    return 0;
}


int Robot::getOPt_IK(Matrix<double, 1, 6> &posd, Matrix<double, 1, 7>& q_current)
{
    Matrix<double, 1, 6> axis_pos, posc, pos1, pos2, axis_p1, axis_p2, v_tcp,err;
    Matrix<double, 1, 7> qc,qd, dq, q, q1;
    Matrix<double, 4, 4> T1, T2, Tc, Td, Tv;
    MatrixXd J(6, 7);
    J.setZero();
    double t = 0, dt, lamda, Tf, k,err_norm;
    int ret1 = check_workspace_access(posd);
    if (ret1 == -1)
    {

        cout << "WorkSpace_Limit" << endl;
        return -1;
    }
   
    Td = TR(posd);
   
    dt = 5e-3;
    q<<0,0,0,M_PI/2,0,0,0;
    int iter_max=50,i=0;
    while (i<iter_max)
    {
      
        Tc = fk(q);
        Tv = Tc.inverse() * Td;
        v_tcp = T2Axispos(Tv) / dt;
        err=  v_tcp*dt;
         if (err.norm()<eps)
        {
                // cout<<"iter="<<i<<endl;
                q_current=q;
            // if( check_Joint_withlimit(q_current))
            // {
                 return 0;
            // }
            // else
            // {
            //     return -1;
            // }

                
        }
    

        dq=quadratic_cost_function_AwayLimit(q, v_tcp);
        q = q + dq * dt;
           
       
        i=i+1;

        
    }

    return 0;
}


Matrix<double, 1, 7> Robot::quadratic_cost_function_AwayLimit(
    const Eigen::Matrix<double, 1, 7> &q,
    const Eigen::Matrix<double, 1, 6> &Vtcp)
{
    // 1. 参数初始化
    Eigen::Matrix<double, 1, 7> w, q_min, q_max,q_normlized;
    double w_max=50;
    double k = 1;      // 梯度下降系数
    double lambda = 0.1; // 阻尼系数

    q_min = limit.row(0);
    q_max = limit.row(1);

    // 2. 计算雅可比矩阵
    Eigen::Matrix<double, 6, 7> J_tcp = J_tcp_crossproduct(q);

    // 3. 计算代价函数梯度（关节极限约束）
    Eigen::Matrix<double, 7, 1> grad;                          // 梯度向量(7×1)
    Eigen::Matrix<double, 1, 7> q_mid = (q_min + q_max) / 2.0; // 关节中间位置
    Eigen::Matrix<double, 1, 7> delta_q = q_max - q_min; // 关节活动范围


    q_normlized=joint_range_normalize(q,   q_min,   q_max) ;
    w=calc_segment_weight(  q_normlized,   w_max);
   
    for (int i = 0; i < 7; ++i)
    {
        double half_range = delta_q(i) / 2.0;
        grad(i) = w(i) * (q(i) - q_mid(i)) / (half_range * half_range);
    }

    // 4. 计算零空间投影矩阵
    Eigen::Matrix<double, 7, 7> I7 = Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::Matrix<double, 7, 6> J_pinv = J_tcp.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Matrix<double, 7, 7> null_proj = I7 - J_pinv * J_tcp;
    // 5. 计算零空间调整速度
    Eigen::Matrix<double, 7, 1> q_dot_null = -k * grad;
    Eigen::Matrix<double, 7, 1> dq_null = null_proj * q_dot_null;

    // 6. 计算主任务关节速度（阻尼最小二乘法）
    Eigen::Matrix<double, 6, 1> Vtcp_col = Vtcp.transpose(); // 转换为列向量
    Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 7, 1> dq_main = J_tcp.transpose() * (J_tcp * J_tcp.transpose() + lambda * lambda * I6).ldlt().solve(Vtcp_col);

    // 7. 总关节速度（主任务+零空间）
    Eigen::Matrix<double, 1, 7> dq = (dq_main + dq_null).transpose();
    
    return dq;
}



int Robot::moveJToJoint_Tf(Matrix<double, 1, 7> &q_end, double Tf)
{
    Matrix<double, 1, 7> q_start, qc, q, dq, dqc;
    double t, dt;
    q_start = getJointPos();
    t = 0;
    dt = 5e-3;
    auto cycle_start = chrono::high_resolution_clock::now();
    // cout << "qcc=" << q_start * deg << endl;

    // cout << "qdd=" << q_end * deg << endl;

    while (t < Tf)
    {
        cycle_start = chrono::high_resolution_clock::now();
        q = quinticInterp_JointSpace(t, Tf, q_start, q_end);
        servoJ(q);

        double elapsed_ms = std::chrono::duration<double, std::milli>(chrono::high_resolution_clock::now() - cycle_start).count();

        double sleep_ms = max(0.0, dt * 1000 - elapsed_ms);
        if (sleep_ms > 1e-6)
        {
            auto sleep_duration = std::chrono::duration<double, std::milli>(sleep_ms);
            std::this_thread::sleep_for(sleep_duration);
        }

        t = t + dt;

        // static int print_counter = 0;
        // if (++print_counter % 100 == 0)
        // {
        //     qc = getJointPos();
        //     cout << "t=  " << t << ", dt" << dt << ", ms=" << elapsed_ms << ", q=" << q * deg << ", qc=" << qc * deg << "\n";
        // }
    }
    return 0;
}
