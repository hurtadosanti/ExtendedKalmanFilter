#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "eigen3/Eigen/Dense"
#include <iostream>

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateLidar(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateRadar(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
private:
    Eigen::VectorXd CalculatePolar(const Eigen::VectorXd& x_state) {
        // recover state parameters
        float px = x_state(0);
        float py = x_state(1);
        float vx = x_state(2);
        float vy = x_state(3);

        // check division by zero
        if(px==0 || py==0){
            std::cout<<"Division by zero"<<std::endl;
            return x_state;
        }
        float rho, phi, rho_dot;
        rho = sqrt(px*px + py*py);
        phi = atan2(py, px);
        rho_dot = (px * vx + py * vy) / rho;

        Eigen::VectorXd v_polar = Eigen::VectorXd(3);
        v_polar <<  rho,phi,rho_dot;
        return v_polar;
    }

};

#endif // KALMAN_FILTER_H_
