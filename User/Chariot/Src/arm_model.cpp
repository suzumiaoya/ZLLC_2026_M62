/**
 * @file arm_model.cpp
 * @author hsl
 * @brief 机械臂机器人学模型，用于正逆运动学计算以及轨迹规划
 * @version 0.1
 * @date 2025-12-03
 *
 * @copyright ZLLC 2026
 *
 */
#include "robotics.h"
#include "matrix.h"
#include "utils.h"
#include "arm_model.h"
#include "crt_gimbal.h"
#ifndef PI
#define PI 3.1415926535f
#endif

using namespace robotics;
using namespace matrixf;

Class_Arm_Model Arm_Model;

/*----------------------variables-----------------------*/

Class_Arm_Model::Class_Arm_Model()
{
    float qmin_init[6] = {-2.883f, -0.926f, 0.15f, -3.028f, -1.336f, -PI};
    float qmax_init[6] = {2.883f, 0.926f, 2.133f, 3.028f, 1.336f, PI};

    for(int i=0; i<6; i++) {
        qmin[i] = qmin_init[i];
        qmax[i] = qmax_init[i];
    }

    robot = CreateMyRobot();
}


// =========================================================
// 1. 定义机械臂模型 (基于 DH 参数)
// =========================================================
// Link(theta, d, a, alpha, type, offset, qmin, qmax, ...)
// 构型: Yaw - Pitch - Pitch - Roll - Pitch - Roll
// 数据: d1=8, a2=35, a3=15.1, a4=12.7, a5=11.5, d6=5.0

Serial_Link<6> Class_Arm_Model::CreateMyRobot()
{
    // (theta, d, a, alpha, type, offset, qmin, qmax)
    // --- Link 1: Yaw (Base -> J2) ---
    // d=8.0 (Z轴高度), alpha=PI/2 (旋转Z轴对齐Y轴供J2使用)
    links[0] = Link(0, 8.0f, 0, PI / 2, R, 0, -2.883f, 2.883f);

    // --- Link 2: Pitch1 (J2 -> J3) ---
    // a=35.0 (大臂长度), alpha=0 (J2与J3平行)
    links[1] = Link(0, 0, 35.0f, 0, R, PI / 2, -0.926f + PI / 2, 0.926f + PI / 2);

    // --- Link 3: Pitch2 (J3 -> J4) ---
    // a=15.1 (小臂长度), alpha=PI/2 (Pitch转Roll)
    links[2] = Link(0, 0, 0, -PI / 2, R, -PI / 2, 0.15f - PI / 2, 2.133f - PI / 2);

    // --- Link 4: Roll1 (J4 -> J5) ---
    // a=12.7 (Offset 1), alpha=-PI/2 (Roll转Pitch)
    links[3] = Link(0, 27.8f, 0, PI / 2, R, 0, -3.028f, 3.028f);

    // --- Link 5: Pitch3 (J5 -> J6) ---
    // a=11.5 (Offset 2), alpha=PI/2 (Pitch转Roll)
    links[4] = Link(0, 0, 0, -PI / 2, R, 0, -1.336f, 1.336f);

    // --- Link 6: Roll2 (J6 -> EE) ---
    // d=5.0 (末端长度), alpha=0
    // 注意：如果末端仅仅是延伸，通常放在d参数里
    links[5] = Link(0, 24.5f, 0, 0, R, 0, 0, 0);

    // 创建串联机械臂对象
    // Serial_Link 模板参数 <6> 表示6自由度
    return Serial_Link<6>(links);
}

// 迭代法求逆运动学，纯fw，狗都不用
bool Class_Arm_Model::SolveRobotIK_Iterative(float target_pos[3], float target_rpy[3], float q_result[6], float now_angle[6])
{
    // 构建目标齐次变换矩阵 Td
    // 使用库函数 rpy2t 将欧拉角转为旋转矩阵
    Matrixf<3, 1> rpy;
    rpy[0][0] = target_rpy[0];
    rpy[1][0] = target_rpy[1];
    rpy[2][0] = target_rpy[2];

    Matrixf<4, 4> T_rot = rpy2t(rpy);

    // 设置位置 p2t
    Matrixf<3, 1> pos;
    pos[0][0] = target_pos[0];
    pos[1][0] = target_pos[1];
    pos[2][0] = target_pos[2];

    // 组合 Td = [R P; 0 1]
    // 库中 rp2t 可以直接从 R 和 p 构建 T
    Matrixf<4, 4> Td = rp2t(t2r(T_rot), pos);

    // 2. 设置初始猜测 (Initial Guess)
    // 使用上一帧的角度或当前位置作为初始猜测
    float data[6] = {0};
    for (int i = 0; i < 6; i++)
    {
        if (now_angle[i] != 0)
        {
            data[i] = now_angle[i]; // 使用当前角度作为初始猜测
        }
        else
        {
            data[i] = 0; // 如果没有当前角度，则使用零位
        }
    }
    // 静态变量初始化一次，初始化时使用当前角度作为初始猜测
    static Matrixf<6, 1> q_guess = Matrixf<6, 1>(data);

    // 数值逆运动学 ikine
    // 参数: 目标T, 初始q, 容差tol(mm/rad), 最大迭代次数
    // ikine 内部实现了奇异性处理
    Matrixf<6, 1> q_sol = robot.ikine(Td, q_guess, 1e-2f, 40);

    // 检查是否收敛：计算 FK 看误差
    Matrixf<4, 4> T_check = robot.fkine(q_sol);
    Matrixf<3, 1> p_err = t2p(Td) - t2p(T_check);

    if (p_err.norm() > 1.0f)
    { // 误差大于 1mm 认为失败
        return false;
    }

    // 5. 输出结果并更新猜测
    q_guess = q_sol; // 更新猜测为当前解，用于下一帧热启动
    for (int i = 0; i < 6; i++)
    {
        q_result[i] = q_sol[i][0];
    }

    return true;
}

void Class_Arm_Model::model_to_control(float model_angles[6], float control_angles[6]) // 将建模解算出的角度转成电机实际控制的角度，这里的control_angle是用来给云台类中各个关节的目标角度值赋值用的，测试角度映射时可以顺带调用
{
    // 由于电机零点设置与建模时不同，此函数用于将模型中逆运动学得到的关节角度转换为电机控制角度
    control_angles[0] = model_angles[0] * 2.0f;              // J0-Yaw 减速比为2，零点和运动方向与控制层匹配
    control_angles[1] = model_angles[1] + 0.9268f;           // J1-Pitch1 电机直连，
    control_angles[2] = model_angles[2] * -1.5f + 3.43f;              // J2-Pitch2 方向相反，减速比1.5，这里把2.1345f看作零点，求当前角度在此基础上的增量，再取相反数乘减速比
    control_angles[3] = (model_angles[3] - 3.028f) * -50.0f; // J3-Roll 减速比为50，零点偏移173.5°，这里算出的是相对Roll_Min_Radian的增量，调用Gimbal中的Set_Target_Roll_Radian时会自动加上Roll_Min_Radian
    control_angles[4] = model_angles[4] - 1.277f;                     // J4-Pitch3 直连
    // roll2的零点关系，暂时写为相等
    control_angles[5] = model_angles[5];
}

void Class_Arm_Model::motor_to_model(float motor_angles[6], float model_angles[6], float cali_offset) // 正运动学求解和轨迹规划时会用到，这里的motor_angles使用电机实际反馈的角度
{
    // cali_offset -> J3-Roll的校准偏移量，恒为负数
    float roll_offset = 1.514f + cali_offset; // Roll上电时的位置相对于垂直向下的角度偏移
    // 由于电机零点设置与建模时不同，此函数用于将电机反馈的实际关节角度转换为模型中的关节角度进行正运动学求解
    model_angles[0] = (motor_angles[0] - PI);                       // J0-Yaw
    model_angles[1] = (motor_angles[1] - PI) * 4.0f - 0.9268f;    // J1-Pitch1
    model_angles[2] = -(motor_angles[2] - PI) / 1.5f + 2.1345f;   // J2-Pitch2 方向相反，减速比1.5
    model_angles[3] = -2.0f * (motor_angles[3] - PI - roll_offset); // J3-Roll 减速比为50，零点偏移PI（这个50的减速比真的需要除吗，存疑）
    model_angles[4] = motor_angles[4] - 0.5f * PI - 0.225f;                // J4-Pitch3
    // roll2的零点关系，暂时写为相等
    model_angles[5] = motor_angles[5];
}

void Class_Arm_Model::motor_to_model(float motor_angles[6], float model_angles[6], Class_Gimbal* Gimbal)
//重载，使用云台类中各个关节的Target_Angle作为输入，不需要校准偏移量，未测试
{
    model_angles[0] = Gimbal->Get_Target_Yaw_Radian();           // J0-Yaw
    model_angles[1] = Gimbal->Get_Target_Pitch_Radian();           // J1-Pitch1
    model_angles[2] = -Gimbal->Get_Target_Pitch_2_Radian();         // J2-Pitch2
    model_angles[3] = -(Gimbal->Get_Target_Roll_Radian() - Gimbal->Get_Roll_Min_Radian()) / 100.0f; // J3-Roll
    model_angles[4] = Gimbal->Get_Target_Pitch_3_Radian();         // J4-Pitch3
    model_angles[5] = Gimbal->Get_Target_Roll_2_Radian_Single();
}

float* Class_Arm_Model::get_now_motor_angles(Class_Gimbal* Gimbal)
//返回当前的电机角度，调用Gimbal中Motor对象的Get函数
{
    now_motor_angles[0] = Gimbal->Motor_DM_J0_Yaw.Get_Now_Angle();
    now_motor_angles[1] = Gimbal->Motor_DM_J1_Pitch.Get_Now_Angle();
    now_motor_angles[2] = Gimbal->Motor_DM_J2_Pitch_2.Get_Now_Angle();
    now_motor_angles[3] = Gimbal->Motor_DM_J3_Roll.Get_Now_Angle();
    now_motor_angles[4] = Gimbal->Motor_DM_J4_Pitch_3.Get_Now_Angle();
    now_motor_angles[5] = multi_to_single(Gimbal->Motor_6020_J5_Roll_2.Get_Now_Radian());
    return now_motor_angles;
}

/*测试用函数*/
// 测试角度映射辅助函数，将roll_2的多圈转成单圈(0~2PI)
float Class_Arm_Model::multi_to_single(float radian)
{
    float single_radian = fmod(radian, 2.0f * PI);
    if (single_radian < 0)
    {
        single_radian += 2.0f * PI;
    }

    return single_radian;
}

void Class_Arm_Model::show_FK_result(float joint_angles[6], float xyz_rpy[6])
{
    Matrixf<6, 1> q;
    for (int i = 0; i < 6; i++)
    {
        q[i][0] = joint_angles[i];
    }
    Matrixf<4, 4> T;
    T = robot.fkine(q);
    xyz_rpy[0] = t2p(T)[0][0];   // X
    xyz_rpy[1] = t2p(T)[1][0];   // Y
    xyz_rpy[2] = t2p(T)[2][0];   // Z
    xyz_rpy[3] = t2rpy(T)[0][0]; // Yaw
    xyz_rpy[4] = t2rpy(T)[1][0]; // Pitch
    xyz_rpy[5] = t2rpy(T)[2][0]; // Roll
}

float Class_Arm_Model::normalize_angle(float angle)
{
    // 使用 user_lib.h 中的宏
    return rad_format(angle);
}

/* *解析法求解器，人人都爱用，实测算出一组解需耗时3ms，计算放前台循环，否则会影响电机通信
 * 一次100mm的轨迹规划大概要算100个点的逆解，总耗时约300ms，速度比较客观，依然放在前台跑
 * pos_target: 目标位置 [x, y, z] 单位 mm
 * rpy_target: 目标姿态 [yaw, pitch, roll] 单位 rad
 * solutions: 输出逆解结果，最多8组解，每组6个关节角
 * return: 实际求解出的解的个数
 */
uint8_t Class_Arm_Model::ikine_pieper_solutions(float pos_target[3], float rpy_target[3], Matrixf<6, 1> solutions[8])
{
    Matrixf<3, 1> rpy;
    rpy[0][0] = rpy_target[0];
    rpy[1][0] = rpy_target[1];
    rpy[2][0] = rpy_target[2];

    Matrixf<4, 4> T_rot = rpy2t(rpy);

    // 设置位置 p2t
    Matrixf<3, 1> pos;
    pos[0][0] = pos_target[0];
    pos[1][0] = pos_target[1];
    pos[2][0] = pos_target[2];

    // 组合 Td = [R P; 0 1]
    // 库中 rp2t 可以直接从 R 和 p 构建 T
    Matrixf<4, 4> T_target = rp2t(t2r(T_rot), pos);

    int sol_count = 0;

    // 0. 提取参数
    float d1 = 8.0f;
    float a2 = 35.0f;
    float d4 = 27.8f;
    float d6 = 24.5f;
    float offset2 = PI / 2.0f;

    // 1. 求解手腕中心位置 (Wrist Center) P_c
    // P_c = P - d6 * a (a是旋转矩阵第三列, 即 Z 轴方向)
    Matrixf<3, 1> P = t2p(T_target);              // 取位置 [px; py; pz]
    Matrixf<3, 1> A = T_target.block<3, 1>(0, 2); // 取姿态矩阵的第三列 (Approach Vector)

    // P_c = P - d6 * A
    Matrixf<3, 1> Pc = P - A * d6;
    float Pcx = Pc[0][0];
    float Pcy = Pc[1][0];
    float Pcz = Pc[2][0];

    // 第一阶段：位置求解 q1, q2, q3

    // q1 的两个候选解 (正向/背向)
    float q1_candidates[2];
    q1_candidates[0] = atan2f(Pcy, Pcx);
    q1_candidates[1] = atan2f(-Pcy, -Pcx);

    for (int i = 0; i < 2; i++)
    {
        float q1 = normalize_angle(q1_candidates[i]);

        // 将手腕中心投影到臂平面
        float r = sqrtf(Pcx * Pcx + Pcy * Pcy);
        if (i == 1)
            r = -r; // 背向解，r 取负

        float s = Pcz - d1;

        // --- 几何参数计算 ---
        float D2 = r * r + s * s;
        float D = sqrtf(D2);

        // 余弦定理求内角 phi (a2 与 d4 的夹角)
        float cos_phi = (a2 * a2 + d4 * d4 - D2) / (2.0f * a2 * d4);

        // 检查是否超出工作空间
        if (fabsf(cos_phi) > 1.0001f)
        {
            continue;
        }
        // 数值误差修正
        if (cos_phi > 1.0f)
            cos_phi = 1.0f;
        if (cos_phi < -1.0f)
            cos_phi = -1.0f;

        float phi = acosf(cos_phi);

        // 余弦定理求仰角偏差 beta (a2 与 D 的夹角)
        float cos_beta = (a2 * a2 + D2 - d4 * d4) / (2.0f * a2 * D);

        if (cos_beta > 1.0f)
            cos_beta = 1.0f;
        if (cos_beta < -1.0f)
            cos_beta = -1.0f;

        float beta = acosf(cos_beta);
        float psi = atan2f(s, r);

        // 构造两组 q2/q3 解 (Elbow Up / Elbow Down)
        float q3_opts[2];
        float q2_geom_opts[2];

        // 解A: 肘部向上
        q3_opts[0] = PI - phi;
        q2_geom_opts[0] = psi - beta;

        // 解B: 肘部向下
        q3_opts[1] = phi - PI;
        q2_geom_opts[1] = psi + beta;

        for (int k = 0; k < 2; k++)
        {
            // 计算实际 q2, q3
            float q3 = normalize_angle(q3_opts[k]);
            float q2 = normalize_angle(q2_geom_opts[k] - offset2);

            // 姿态求解 q4, q5, q6

            // 计算前三轴产生的旋转矩阵 R03
            // R03 = R01 * R12 * R23

            // 预计算三角函数
            float c1 = cosf(q1), s1 = sinf(q1);
            float th2 = q2 + offset2; // R12 用的 theta
            float c2 = cosf(th2), s2 = sinf(th2);
            float th3 = q3; // R23 用的 theta
            float offset3 = -PI / 2.0f;
            float th3_mat = q3 + offset3;
            float c3 = cosf(th3_mat), s3 = sinf(th3_mat);

            // 手动构建矩阵 (比通用矩阵乘法快)
            // T01 (alpha=90): [c1 0 s1; s1 0 -c1; 0 1 0]
            Matrixf<3, 3> R01;
            R01[0][0] = c1;
            R01[0][1] = 0;
            R01[0][2] = s1;
            R01[1][0] = s1;
            R01[1][1] = 0;
            R01[1][2] = -c1;
            R01[2][0] = 0;
            R01[2][1] = 1;
            R01[2][2] = 0;

            // T12 (alpha=0): [c2 -s2 0; s2 c2 0; 0 0 1]
            Matrixf<3, 3> R12;
            R12[0][0] = c2;
            R12[0][1] = -s2;
            R12[0][2] = 0;
            R12[1][0] = s2;
            R12[1][1] = c2;
            R12[1][2] = 0;
            R12[2][0] = 0;
            R12[2][1] = 0;
            R12[2][2] = 1;

            // T23 (alpha=-90): [c3 0 -s3; s3 0 c3; 0 -1 0]
            Matrixf<3, 3> R23;
            R23[0][0] = c3;
            R23[0][1] = 0;
            R23[0][2] = -s3;
            R23[1][0] = s3;
            R23[1][1] = 0;
            R23[1][2] = c3;
            R23[2][0] = 0;
            R23[2][1] = -1;
            R23[2][2] = 0;

            Matrixf<3, 3> R03 = R01 * R12 * R23;

            // 计算 R36 = R03' * R_target
            Matrixf<3, 3> R_target = t2r(T_target);
            Matrixf<3, 3> R36 = R03.trans() * R_target;

            // === 欧拉角提取 (针对 alpha4=90, alpha5=-90) ===
            float r13 = R36[0][2];
            float r23 = R36[1][2];
            float r31 = R36[2][0];
            float r32 = R36[2][1];
            float r33 = R36[2][2];

            // q5 的两个解
            float q5_candidates[2];
            // atan2(sqrt(x^2+y^2), z)
            q5_candidates[0] = atan2f(sqrtf(r13 * r13 + r23 * r23), r33);
            q5_candidates[1] = atan2f(-sqrtf(r13 * r13 + r23 * r23), r33);

            for (int m = 0; m < 2; m++)
            {
                float q5 = normalize_angle(q5_candidates[m]);
                float q4, q6;
                float s5 = sinf(q5);

                if (fabsf(s5) < 1e-4f)
                { // 奇异点
                    q4 = 0.0f;
                    // R11 = c4c6 - s4s6 = c(4+6) -> 这种情况下
                    // 使用 MATLAB 逻辑: q6 = atan2(-r12, r11)
                    q6 = atan2f(-R36[0][1], R36[0][0]);
                }
                else
                {
                    // q4 = atan2(-r23, -r13)
                    q4 = atan2f(-r23 / s5, -r13 / s5);
                    // q6 = atan2(-r32, r31)
                    q6 = atan2f(-r32 / s5, r31 / s5);
                }

                q4 = normalize_angle(q4);
                q6 = normalize_angle(q6);

                // 保存这一组解
                if (sol_count < 8)
                {
                    float sol_data[6] = {q1, q2, q3, q4, q5, q6};
                    solutions[sol_count] = Matrixf<6, 1>(sol_data);
                    sol_count++;
                }
            }
        }
    }
    return sol_count;
}

uint8_t Class_Arm_Model::solution_filter(Matrixf<6, 1> solutions[8], bool valid[8])
{
    int valid_count = 0;
    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;
        for (int j = 0; j < 6; j++)
        {
            float angle = solutions[i][j][0];
            // 检查每个关节角度是否在范围内
            if (angle < qmin[j] || angle > qmax[j])
            {
                valid[i] = false;
                break;
            }
        }
        if (valid[i])
        {
            valid_count++;
        }
    }
    return valid_count;
}

uint8_t Class_Arm_Model::get_best_solution_index(Matrixf<6, 1> solutions[8], bool valid[8], float current_angle[6])
{
    float d[8] = {0.0f};    //欧式距离，等于各关节角差值平方之和再求平方根(这里略去求根这一步)
    float err[6] = {0.0f};  //各个关节的角度差值
    uint8_t best_index = 0; //求最值经典起手式
    for(int i =0; i < 8; i++)
    {
        if(!valid[i])
        {
            d[i] = 114514.1919f;
        }

        for(int j =0; j < 6; j++)
        {
            err[j] = current_angle[j] - solutions[i][j][0];
            err[j] *= err[j];
            d[i] += err[j];
        }

        if(d[i] < d[best_index]) best_index = i;
    }

    return best_index;
}
