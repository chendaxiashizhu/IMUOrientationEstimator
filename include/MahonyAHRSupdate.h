

#pragma once
#include <Eigen/Core>
#include "Utils.h"
#include <Eigen/Geometry>
#include <iostream>
#include "Initializer.h"
#include <memory>

namespace OriEst
{

    class MahonyAHRSupdate{
    public:
        MahonyAHRSupdate(const double& gyro_noise);
        void fuck(){
            std::cout<<"fuck"<<std::endl;
        }
        status Mahonyfilter(const double &timestamp, Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::Matrix3d *G_R_I);
        void SetParameter(const double &ki, const double &kp);
    private:
        status status_;
        std::unique_ptr<Initializer> initializer_;
        double Ki;
        double Kp;
        Eigen::Matrix3d G_R_I_;
        double last_time_;
        Eigen::Vector3d eInt_n;
        int count;
    };
} // namespace OriEst