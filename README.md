# IMUOrientationEstimator
Estimate 3DoFs orientation using IMU measurement.

## Method drivation and video
A detailed derivation of the algorithm and a video are in the **doc** folder of this repository. 

**Doc**
[EKF-Based IMU Orientation Estimation](https://github.com/ydsf16/IMUOrientationEstimator/blob/master/doc/EKF_Based_IMU_Orientation_Estimation.pdf).

**Video**
<https://github.com/ydsf16/IMUOrientationEstimator/blob/master/doc/IMUOrientationEstimator.mp4>

### 修改记录

- 适用于不同的小觅相机的版本，S1030
- 增加无人机中常用的 Manony 互补滤波器，对比与ESKF的性能

