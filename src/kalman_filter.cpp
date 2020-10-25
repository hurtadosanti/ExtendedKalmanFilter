#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {

}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::UpdateLidar(const VectorXd &z) {
    auto y = z - H_ * x_;
    auto Ht = H_.transpose();
    auto S = H_ * P_ * Ht + R_;
    auto PHt = P_ * Ht;
    auto K = PHt * S.inverse();

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    auto I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateRadar(const VectorXd &z) {

    auto hx = CalculatePolar(x_);
    MatrixXd y = z - hx;
    while(y(1) > M_PI){
        y(1) -= 2 * M_PI;
    }

    while(y(1) < -M_PI){
        y(1) += 2 * M_PI;
    }

    auto Ht = H_.transpose();

    auto S = H_ * P_ * Ht + R_;

    auto PHt = P_ * Ht;

    auto K = PHt * S.inverse();

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    auto I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

