


#pragma once
#include <iostream>
#include <Eigen/Core>
// #include <Utils.h>
#include <Eigen/Geometry>
// #include <queue>
#include <memory>
#include "Initializer.h"
#include <deque>
#include "Update.h"
#include "Propagator.h"
#include "Utils.h"

namespace OriEst{

// enum class status {
//     kValid,
//     kInValid
// };

class Estimator
{
public:
struct Config{
    std::size_t buffer_size = 1;
};
void fuck(){
    std::cout<<"fuck"<<std::endl;
}
/*构造函数*/
Estimator(const double gyro_noise, const double gyro_bias_noise, const double acc_noise);
status Estimate(double timestamp, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc, Eigen::Matrix3d* G_R_I);

private:
Eigen::Matrix3d acc_noise_mat_;
status status_; /*给定状态决定是否要初始化*/
double last_timestamp_; //标记上一时刻的时间
Eigen::Vector3d bg_;
Eigen::Matrix<double , 6,6> cov_;

Eigen::Matrix3d G_R_I_;
/*哪里出了问题？*/
std::unique_ptr<OriEst::Initializer> initializer_;
std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> acc_buffer_;
std::unique_ptr<OriEst::Propagator> propagator_;
Config config_;
int count;
};

}