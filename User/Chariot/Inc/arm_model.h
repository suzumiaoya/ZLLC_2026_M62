#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include "robotics.h" 
#include "matrix.h"
#include "user_lib.h"

/* Exported variables --------------------------------------------------------*/
class Class_Gimbal;

class Class_Arm_Model
{
public:
    robotics::Link links[6];
    robotics::Serial_Link<6> robot;
    float qmin[6];
    float qmax[6];
    float now_motor_angles[6];

    Class_Arm_Model();

    bool SolveRobotIK_Iterative(float target_pos[3], float target_rpy[3], float q_result[6], float now_angle[6]);

    void model_to_control(float model_angles[6], float control_angles[6]);
    void motor_to_model(float motor_angles[6], float model_angles[6], float cali_offset);
    void motor_to_model(float motor_angles[6], float model_angles[6], Class_Gimbal* Gimbal);

    float multi_to_single(float radian);
    void show_FK_result(float joint_angles[6], float xyz_rpy[6]);

    uint8_t ikine_pieper_solutions(float pos_target[3], float rpy_target[3], Matrixf<6, 1> solutions[8]);
    uint8_t solution_filter(Matrixf<6, 1> solutions[8], bool valid[8]);
    uint8_t get_best_solution_index(Matrixf<6, 1> solutions[8], bool valid[8], float current_angle[6]);
    float* get_now_motor_angles(Class_Gimbal* Gimbal);

private:
    robotics::Serial_Link<6> CreateMyRobot();
    static float normalize_angle(float angle);
};

extern Class_Arm_Model Arm_Model;

#endif // ARM_MODEL_H
