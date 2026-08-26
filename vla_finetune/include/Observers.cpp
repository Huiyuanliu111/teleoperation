/** ------------------------- Revision Code History -------------------
*** Programming Language: C++
*** Description: Panda Dynamics
*** Released Date: Apr. 2023
*** Mario Tröbinger
*** mario.troebinger@tum.de
----------------------------------------------------------------------- */
// #pragma once
#include "Observers.h"

Observers::Observers(Dynamics& dyn) : _dyn(dyn){
  _integral.setZero(7);
  _r.setZero(7);
};

Observers::~Observers(){
};

VectorXd Observers::get_tau_ext_hat_filtered(const franka::RobotState& state){
  VectorXd tau_J = Map<VectorXd>(const_cast<double*>(state.tau_J.data()), state.tau_J.size());  
  VectorXd qd = Map<VectorXd>(const_cast<double*>(state.dq.data()), state.dq.size()); // measured torque in Eigen representation
  VectorXd q = Map<VectorXd>(const_cast<double*>(state.q.data()), state.q.size()); // measured torque in Eigen representation

  MatrixXd M = _dyn.get_M(q);
  MatrixXd C = _dyn.get_C(q, qd);
  VectorXd tau_G = _dyn.get_tau_G(q); // gravity torque
  VectorXd tau_Frict = _dyn.get_tau_F(qd); // frictionr torque  
  
  VectorXd p = M*qd; //Momentum
  double dt = 0.001; // can be read from the stepsize later
  double KO = 10;

  // _integral = _integral + (tau_J-tau_G - C.transpose()*qd-tau_Frict+_r)*dt;
  _integral = _integral + (tau_J-tau_G - C.transpose()*qd+_r)*dt;
  _r = KO * (p - _integral);

  return -_r; // minus to have the same sign like franka
};

VectorXd Observers::get_O_F_ext_hat_K(const franka::RobotState& state, const franka::Model& model){
  std::array<double, 42> zeroJacobian_array = model.zeroJacobian(franka::Frame::kStiffness, state);
  // different to the franka O_F_ext_hat_K!! mario: wrench acting on the stiffness frame, expressed in the base frame! 
  // franka: wrench acting on the stiffness frame transfered to the base frame(forces times lever arm O_T_K), expressed in the base frame
  Map<const Matrix<double, 6, 7>> zeroJacobian(zeroJacobian_array.data());

  VectorXd tau_ext_hat_filtered = get_tau_ext_hat_filtered(state);

  MatrixXd zeroJacobianT = zeroJacobian.transpose();
  MatrixXd pinv_zeroJacobianT = (zeroJacobian*zeroJacobianT).inverse()*zeroJacobian;

  VectorXd O_F_ext_hat_K = pinv_zeroJacobianT * tau_ext_hat_filtered;
  return O_F_ext_hat_K;
};

VectorXd Observers::get_K_F_ext_hat_K(const franka::RobotState& state, const franka::Model& model){
  VectorXd K_F_ext_hat_K = get_xx_F_ext_hat_K(state, model, franka::Frame::kStiffness);
  return K_F_ext_hat_K;
};

VectorXd Observers::get_xx_F_ext_hat_K(const franka::RobotState& state, const franka::Model& model, franka::Frame frame){ 
  std::array<double, 42> xx_Jacobian_array = model.bodyJacobian(frame, state);
  Map<const Matrix<double, 6, 7>> xxJacobian(xx_Jacobian_array.data());

  VectorXd tau_ext_hat_filtered = get_tau_ext_hat_filtered(state);

  MatrixXd xxJacobianT = xxJacobian.transpose();
  MatrixXd pinv_xxJacobianT = (xxJacobian*xxJacobianT).inverse()*xxJacobian;

  VectorXd xx_F_ext_hat_K = pinv_xxJacobianT * tau_ext_hat_filtered;
  return xx_F_ext_hat_K;
};




