# **Extended Kalman Filters**

### Radar and Lidar sensor fusion utilizing extended kalman filters

---

**Extended Kalman Filters Project**

The goals / steps of this project are the following:

* Complete starter code to succesfully fuse lidar and radar sensor data with an extended kalman filter


[//]: # (Image References)
[image1]: ./Dataset1.png "Results 1"
[image2]: ./Dataset2.png "Results 2"

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

My project includes the following files:
* [main.cpp](../src/main.cpp) main program that runs the IO server (given from starter code)
* [FusionEKF.cpp](../src/FusionEKF.cpp) handles the measurment data and calls the EKF implementation
* [FusionEKF.h](../src/FusionEKF.h) header file for [FusionEKF.cpp](../src/FusionEKF.cpp)
* [kalman_filter.cpp](../src/kalman_filter.cpp) contains the extended kalman filter implementation
* [kalman_filter.h](../src/kalman_filter.h) header file for [kalman_filter.cpp](../src/kalman_filter.cpp)
* [tools.cpp](../src/tools.cpp) contains Jacobian and RMSE calculations
* [Dataset1.png](./Dataset1.png) result image from dataset 1
* [Dataset2.png](./Dataset2.png) result image from dataset 2

### Discussion

#### 1. Implementation

Implementation of the Extended Kalman Filter utilized most code from the lesseons adapted for this implementation.  The Provided R matrix values and Q noise values were utilized so no specific tuning was required.  A key aspect in implemenation was the initilization of the state vector with the first measurement values.  The x and y positions were used as-long as the initial values were not near zero (to prevent division by zero and NaN occurences), and initial x and y velocity values were zero.  Additionally other checks were necessary to prevent division by zero, and a check that the angles from radar measurement were between -pi and pi were also required.


#### 2. Results

Upon inspection of the first data set, one can clearly see that the result position is much cleaner and smoother than the incoming sensors and the nosie is greatly reduced:

Dataset 1:

![Dataset 1][image1]

Data set 2 provided similar results, however, extra car was necessary upon intialization of the state vector due to initial zero values of x and y:

Dataset 2:

![Dataset 2][image2]
