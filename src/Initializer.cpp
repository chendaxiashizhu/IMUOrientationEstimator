



#include "Initializer.h"
#include <iostream>

namespace OriEst{
bool Initializer::Initialize(Eigen::Vector3d acc, Eigen::Matrix3d *G_R_I){
    /*保存数据到缓存中去*/
    acc_buffer_.push_back(acc);
    /*这里的处理不清楚*/
    if(acc_buffer_.size() <= config_.acc_buffer_size){
        /*这一步是关键的*/
        return false;/*下面的就不执行了*/
    }
    acc_buffer_.pop_front();
    //计算均值
    Eigen::Vector3d acc_mean(0.0,0.0,0.0);
    for(const Eigen::Vector3d & one_acc:acc_buffer_){
        acc_mean = acc_mean + one_acc;
    }
    /*这里的处理也很复杂*/
    acc_mean = acc_mean/ static_cast<double>(acc_buffer_.size());

    Eigen::Vector3d acc_std(0.0,0.0,0.0);
    for(const Eigen::Vector3d & one_acc:acc_buffer_){
        acc_std += (one_acc - acc_mean).cwiseAbs2();
    }

    acc_std = (acc_std/static_cast<double>(acc_buffer_.size())).cwiseSqrt();
    if(acc_std.norm()>config_.max_acc_std){
        std::cout<<"Initialization error"<<std::endl;
        return false;
    }
    
    const Eigen::Vector3d z_axis = acc_mean.normalized();
    const Eigen::Vector3d x_axis = 
    (Eigen::Vector3d::UnitX() - z_axis*z_axis.transpose()*Eigen::Vector3d::UnitX()).normalized();
    const Eigen::Vector3d y_axis = z_axis.cross(x_axis);

    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    *G_R_I = I_R_G.transpose();
    return true;
}


}