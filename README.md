# Extended Kalman Filter Project 

The goals / steps of this project are the following:

* Implement an Extended Kalman Filter that reads lidar and radar measurement and performs sensor fusion for state estimates.

---

## Rubric points
#### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.


#### 1. Your code should compile. 
```
mkdir build; cd build; cmake ..; make
./ExtendedKF
```

#### 2. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] 
I ran the simulator and got
* RMSE = [0.098, 0.085, 0.409, 0.470] <=[.11, .11, 0.52, 0.52] on dataset 1.
* RSME = [0.073, 0.097, 0.449, 0.464] <=[.11, .11, 0.52, 0.52] on dataset 2.

#### 3. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
Implemented the Kalman Filter following the logic from the lessons, adding some additional safeguards (validation of inputs, normalization of atan).

#### 3. Your Kalman Filter algorithm handles the first measurements appropriately.
I initialize the first measurements in `if (!is_initialized_)` condition in `FusionEKF::ProcessMeasurement`, based on receiving radar or lidar measurements.

#### 4. Your Kalman Filter algorithm first predicts then updates.
See the logic in `FusionEKF::ProcessMeasurement`.

#### 5. Your Kalman Filter can handle radar and lidar measurements.
I use both measurements and choose EKF with linearized update or KF based on a measurement type.

#### 6. Your algorithm should avoid unnecessary calculations.
Where applicable I cache the results.