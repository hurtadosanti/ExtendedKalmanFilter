# ExtendedKalmanFilter
Extended Kalman Filter Project for the Self-Driving Car Nanodegree Program  

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Project Introduction
Now that you have learned how the extended Kalman filter works, you are going to implement the extended Kalman filter in C++. We are providing simulated lidar and radar measurements detecting a bicycle that travels around your vehicle. You will use a Kalman filter, lidar measurements and radar measurements to track the bicycle's position and velocity.


The only files you need to modify are FusionEKF.cpp, kalman_filter.cpp, and tools.cpp.
# Code that was be modified 
## FusionEKF.cpp

- Initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
- Initialize the Kalman filter position vector with the first sensor measurements
- Modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
- Call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.

## KalmanFilter Class
You will need to add your code to kalman_filter.cpp to implement the prediction and update equations.

## Tools
You will implement functions to calculate root mean squared error and the Jacobian matrix:
# Contributing
[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html)
# References
https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

https://review.udacity.com/#!/rubrics/748/view



### License
MIT License Copyright (c) 2016-2018 Udacity, Inc.