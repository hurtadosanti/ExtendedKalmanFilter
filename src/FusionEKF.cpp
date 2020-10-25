#include "FusionEKF.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    // measurement matrix
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    // set the acceleration noise components
    noise_ax = 9;
    noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {
        /**
         * You'll need to convert radar from polar to cartesian coordinates.
         */

        // first measurement
        cout << "EKF: " << endl;
        auto x = VectorXd(4);
        auto P = MatrixXd(4, 4);
        P << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
        auto F = MatrixXd(4, 4);
        F << 1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;
        auto Q = MatrixXd(4, 4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            auto rho = measurement_pack.raw_measurements_[0];
            auto phi = measurement_pack.raw_measurements_[1];
            x << rho * cos(phi),
                    rho * sin(phi),
                    0,
                    0;
            //VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in
            ekf_.Init(x,P,F,Hj_,R_radar_,Q);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            x<< measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1],
                    0,
                    0;
            //VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in
            ekf_.Init(x,P,F,H_laser_,R_laser_,Q);
        }


        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /**
     * Prediction
     */

    /**
     * Update the state transition matrix F according to the new elapsed time.
     * Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    // 1. Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    // 2. Set the process covariance matrix Q
    auto dt2 = dt * dt;
    auto dt3 = dt2 * dt;
    auto dt4 = dt3 * dt;
    ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
            0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
            dt3 / 2 * noise_ax, 0, dt2 * noise_ax, 0,
            0, dt3 / 2 * noise_ay, 0, dt2 * noise_ay;
    ekf_.Predict();

    /**
     * Update
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        ekf_.R_ = R_radar_;
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.UpdateRadar(measurement_pack.raw_measurements_);
    } else {

        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.UpdateLidar(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
