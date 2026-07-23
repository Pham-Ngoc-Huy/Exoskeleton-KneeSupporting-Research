#include <iostream>
#include <cmath>

struct Vec3{
    double x,y,z;
};

struct Vec6{
    double data[6];
};

struct Matrix3x6 {
    double data[3][6];
};

class ImpedanceController {
private:
    double K;
    double D;
    Vec3 x_desired;
public:
    ImpedanceController(): K(300.0), D(2.0* std::sqrt(300.0)){
        x_desired = {0.0, 0.0, 0.0};
    }
    void setTargetPose(const Vec3& target){
        x_desired = target;
    }
    Vec6 update(const Vec3& x_actual, const Vec3& v_actual, const Matrix3x6& J, const Vec6& gravity){

        //position error
        Vec3 e_pos;
        e_pos.x = x_desired.x - x_actual.x;
        e_pos.y = x_desired.y - x_actual.y;
        e_pos.z = x_desired.z - x_actual.z;

        // velocity error
        Vec3 e_vel;
        e_vel.x = -v_actual.x;
        e_vel.y = -v_actual.y;
        e_vel.z = -v_actual.z;

        // Cartesian force
        Vec3 F;

        F.x = K * e_pos.x + D * e_vel.x;
        F.y = K * e_pos.y + D * e_vel.y;
        F.z = K * e_pos.z + D * e_vel.z;

        // Tau = J^T * F
        Vec6 tau;

        for(int j = 0; j < 6; j++)
        {
            tau.data[j] = 
                J.data[0][j] * F.x +
                J.data[1][j] * F.y +
                J.data[2][j] * F.z +
                gravity.data[j];
        }

        return tau;
    }

};
int main()
{
    ImpedanceController controller;

    Vec3 x_des = {1.0, 0.5, 0.2};
    Vec3 x_act = {0.95, 0.48, 0.21};
    Vec3 v_act = {0.1, -0.05, 0.0};

    controller.setTargetPose(x_des);

    Matrix3x6 J = {{
        {0.5, 0.2, 0.1, 0.0, 0.3, 0.1},
        {0.1, 0.4, 0.2, 0.5, 0.2, 0.1},
        {0.2, 0.1, 0.6, 0.2, 0.4, 0.3}
    }};

    Vec6 gravity;
    for(int i = 0; i < 6; i++)
        gravity.data[i] = 0.5;

    Vec6 tau =
        controller.update(
            x_act,
            v_act,
            J,
            gravity);

    std::cout << "Joint Torques:\n";

    for(int i = 0; i < 6; i++)
        std::cout << tau.data[i] << " ";

    std::cout << std::endl;

    return 0;
}