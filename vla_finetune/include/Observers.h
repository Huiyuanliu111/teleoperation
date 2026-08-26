#ifndef OBSERVERS_H_   /* Include guard */
#define OBSERVERS_H_

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "Dynamics.h"
#include <franka/robot.h>
#include <franka/model.h>
#include <array>

using namespace Eigen;

class Observers {
  public:
    Observers(Dynamics& dyn);
    ~Observers();
    
    VectorXd get_tau_ext_hat_filtered(const franka::RobotState& state);
    VectorXd get_O_F_ext_hat_K(const franka::RobotState& state, const franka::Model& model);
    VectorXd get_K_F_ext_hat_K(const franka::RobotState& state, const franka::Model& model);
    VectorXd get_xx_F_ext_hat_K(const franka::RobotState& state, const franka::Model& model, franka::Frame frame);

  private:
    VectorXd _integral;
    VectorXd _r; //residual
    Dynamics _dyn;
};


#endif // OBSERVERS_H_