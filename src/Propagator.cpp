

#include "Propagator.h"

namespace OriEst
{
    Propagator::Propagator(const double &gyro_noise, const double &gyro_bias_noise):
        gyro_noise_(gyro_noise),
        gyro_bias_noise_(gyro_bias_noise)
    {

    }
    void Propagator::PropagateMeanAndCov(const Eigen::Matrix3d &begin_G_R_I,
                                         const Eigen::Vector3d &begin_bg,
                                         const Eigen::Matrix<double, 6, 6> &begin_cov,
                                         const Eigen::Vector3d &gyro,
                                         const double delta_t,
                                         Eigen::Matrix3d *end_G_R_I,
                                         Eigen::Vector3d *end_bg,
                                         Eigen::Matrix<double, 6, 6> *end_cov)
    {
        /*准备工作*/
        const Eigen::Vector3d unbiased_gyro = gyro - begin_bg; /*现对陀螺仪的数据进行校正*/
        const Eigen::Vector3d angle_vec = unbiased_gyro*delta_t;
        Eigen::Matrix3d dot_R;
        if(angle_vec.norm()<1e-12){
            dot_R = Eigen::Matrix3d::Identity() + SkewMat(angle_vec);
        }
        else{
            const double angle = angle_vec.norm();/*旋转角度*/
            Eigen::Vector3d axis = angle_vec/angle; /*旋转轴*/
            dot_R = Eigen::AngleAxisd(angle,axis).toRotationMatrix();

        }
        *end_G_R_I = begin_G_R_I*dot_R;
        *end_bg = begin_bg;

        /*算雅可比矩阵和Q矩阵的值*/
        Eigen::Matrix<double,6,6> Fx;
        Fx.topLeftCorner<3,3>() = dot_R.transpose();
        Fx.topRightCorner<3,3>() = -Eigen::Matrix3d::Identity()*delta_t;
        Fx.bottomLeftCorner<3,3>() = Eigen::Matrix3d::Zero();
        Fx.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity();


        Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double ,6,6>::Zero();
        Q.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity()*gyro_noise_*delta_t*delta_t;
        Q.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity()*gyro_bias_noise_*delta_t;


        *end_cov = Fx*begin_cov*Fx.transpose() + Q;

    }

} // namespace OriEst
