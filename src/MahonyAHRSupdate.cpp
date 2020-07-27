

#include "MahonyAHRSupdate.h"
#include <iostream>

namespace OriEst
{

    MahonyAHRSupdate::MahonyAHRSupdate(const double &gyro_noise) : status_(status::kInValid),
                                                                   last_time_(-1),
                                                                   initializer_(std::make_unique<Initializer>())
    {
        eInt_n.setZero(); /*比例积分常数，互补滤波器用到的量*/
        count = 0;
    }

    status MahonyAHRSupdate::Mahonyfilter(const double &timestamp, Eigen::Vector3d gyro, Eigen::Vector3d acc, Eigen::Matrix3d *G_R_I)
    {
        if (status_ == status::kInValid)
        {
            /*这里循环10次，读取10个数据，求均值，这一点很重要*/
            if (!initializer_->Initialize(acc, &G_R_I_))
            {
                G_R_I->setIdentity();
                // std::cout<<"初始化错误！s\n"<<std::endl;
                return status::kInValid; /*所以决定了这里返回的依然是kInValid，important！！！！！*/
            }
            last_time_ = timestamp;
            status_ = status::kValid;
            *G_R_I = G_R_I_;
            return status::kValid;
        }
        if (last_time_ == -1)
        {
            std::cout << "时间处理的不对!\n"
                      << std::endl;
        }

        double delta_t = timestamp - last_time_;

        Eigen::Vector3d Gravity(0.0, 0.0, 1);
        Eigen::Vector3d v = -G_R_I_.transpose() * Gravity;
        Eigen::Vector3d e = acc.normalized().cross(v);
        eInt_n += e * delta_t;
        Eigen::Vector3d Gyroscope = gyro + Kp * e + Ki * eInt_n;
        Eigen::Vector3d Gyroscope_dt = Gyroscope * delta_t;

        // Update state.
        Eigen::Matrix3d delta_Rot;
        if (Gyroscope_dt.norm() < 1e-12)
        {
            delta_Rot = Eigen::Matrix3d::Identity() + SkewMat(Gyroscope_dt);
        }
        else
        {
            const double angle = Gyroscope_dt.norm();
            const Eigen::Vector3d axis = Gyroscope_dt / angle;
            delta_Rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }

        G_R_I_ = G_R_I_ * delta_Rot;
        *G_R_I = G_R_I_;

        /*输出*/
        count++;
        if (count % 100 == 0)
        {
            std::cout << "互补滤波器" << std::endl;
        }
        /*时间处理*/
        last_time_ = timestamp;
        return status::kValid;
    }

    void MahonyAHRSupdate::SetParameter(const double &ki, const double &kp)
    {
        Ki = ki;
        Kp = kp;
        std::cout << "fuck you" << std::endl;
    }

} // namespace OriEst