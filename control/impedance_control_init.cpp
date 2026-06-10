#include <iostream>
#include <Eigen/Dense>

using JointVector = Eigen::Matrix<double, 6, 1>;
using CartesianVector = Eigen::Matrix<double, 3, 1>;
using JacobianMatrix = Eigen::Matrix<double, 3, 6>;
using StiffnessMatrix = Eigen::Matrix<double, 3, 3>;

class ImpedanceController {
private:
    // Impedance parameters
    // 1. Virtual stiffness
    StiffnessMatrix K_x_;
    // 2. Virtual damping
    StiffnessMatrix D_x_;

    // Target state
    CartesianVector x_desired_;

public:
    ImpedanceController() {
        // init stiffness (diagonal matrix for decoupled axis)
        K_x_ = StiffnessMatrix::Identity() * 300.0; // 300 N/m
        
        // design critical damping: D = 2*sqrt(K) assuming unit mass
        D_x_ = StiffnessMatrix::Identity() * (2.0 * std::sqrt(300.0)); 

        x_desired_.setZero();

    }

    void setTargetPose(const CartesianVector& x_des){
        x_desired_ = x_des;
    }

    JointVector update(const CartesianVector& x_actual, const CartesianVector& v_actual, const JacobianMatrix& Jacobian, const JointVector& gravity_torques)
    {
        CartesianVector position_error = x_desired_ - x_actual;
        CartesianVector velocity_error = -v_actual;

        CartesianVector F_cartesian =  (K_x_ * position_error) + (D_x_ * velocity_error);

        JointVector tau_impedance = Jacobian.transpose() * F_cartesian;

        JointVector tau_command = tau_impedance + gravity_torques;

        return tau_command;
    }
};

int main(){
    ImpedanceController controller;

    CartesianVector x_desired(1.0, 0.5, 0.2);
    CartesianVector x_actual(0.95, 0.48, 0.21);
    CartesianVector v_actual(0.1, -0.05, 0.0);

    JacobianMatrix Jacobian = JacobianMatrix::Random();
    JointVector gravity = JointVector::Constant(0.5);

    controller.setTargetPose(x_desired);

    JointVector commanded_torques = controller.update(x_actual, v_actual, Jacobian, gravity);
    std::cout << "Commanded Joint Torques:\n" << commanded_torques.transpose() << std::endl;
    return 0;
}