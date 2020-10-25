# ExtendedKalmanFilter
Extended Kalman Filter Project for the Self-Driving Car Nanodegree Program  

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Project Introduction
Now that you have learned how the extended Kalman filter works, you are going to implement the extended Kalman filter in C++. We are providing simulated lidar and radar measurements detecting a bicycle that travels around your vehicle. You will use a Kalman filter, lidar measurements and radar measurements to track the bicycle's position and velocity.


The only files you need to modify are FusionEKF.cpp, kalman_filter.cpp, and tools.cpp.
## Code that was be modified 
### FusionEKF.cpp

- Initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
- Initialize the Kalman filter position vector with the first sensor measurements
- Modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
- Call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.

### KalmanFilter Class
You will need to add your code to kalman_filter.cpp to implement the prediction and update equations.

### Tools
You will implement functions to calculate root mean squared error and the Jacobian matrix:

## Contributing
[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html)
### Build
To build, compile and run the code we use a [docker image](Dockerfile) together with CLion.
- To build the image run

        docker build -t dev/env .
        
- To run the image

        docker run -p 127.0.0.1:2222:22 -p 127.0.0.1:4567:4567 --name kalmanfilter-env --rm dev/env 

The code can be copy using ssh, then use cmake to setup and make to build. Finally run the KalmanFilter executable.

For more details of the Clion integration go to the post 
[Using Docker with CLion](https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/)

## References
https://github.com/udacity/CarND-Extended-Kalman-Filter-Project

https://review.udacity.com/#!/rubrics/748/view



## License
MIT License Copyright (c) 2016-2018 Udacity, Inc.