#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    if(estimations.size()==0){
        std::cout<<"the estimation vector size should not be zero"<<std::endl;
        return rmse;
    }
    if(estimations.size()!=ground_truth.size()){
        std::cout<<"the estimation vector size should equal ground truth vector size"<<std::endl;
        return rmse;
    }
    // Accumulate squared residuals
    for (size_t i=0; i < estimations.size(); ++i) {
        VectorXd res =  (estimations[i]-ground_truth[i]);
        res = res.array()*res.array();
        rmse += res;
    }

    // Calculate the mean
    rmse = rmse/estimations.size();

    // Calculate the squared root
    rmse = rmse.array().sqrt();
    // return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // check division by zero
    if(px==0 || py==0){
        std::cout<<"Division by zero"<<std::endl;
        return Hj;
    }
    // compute the Jacobian matrix
    float base = px*px+py*py;

    Hj(0,0) = px/std::sqrt(base);
    Hj(0,1) = py/std::sqrt(base);
    Hj(0,2) = 0;
    Hj(0,3) = 0;

    Hj(1,0) = -1*(py/base);
    Hj(1,1) = px/base;
    Hj(1,2) = 0;
    Hj(1,3) = 0;

    Hj(2,0) = py*(vx*py-vy*px)/(std::sqrt(base)*base);
    Hj(2,1) = px*(vy*px-vx*py)/(std::sqrt(base)*base);
    Hj(2,2) = px/std::sqrt(base);
    Hj(2,3) = py/std::sqrt(base);


    return Hj;
}

