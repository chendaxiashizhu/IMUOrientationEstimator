

#include "Estimator.h"

namespace OriEst
{

    Estimator::Estimator(const double gyro_noise, const double gyro_bias_noise, const double acc_noise)
        : status_(status::kInValid), last_timestamp_(-1.),
        initializer_(std::make_unique<OriEst::Initializer>()),
        propagator_(std::make_unique<OriEst::Propagator>(gyro_noise , gyro_bias_noise))
    {
        /*要注意这里的gyro的noise是不断更新的*/
        acc_noise_mat_ = Eigen::Matrix3d::Identity() * acc_noise;/*注意这里的*/
        count = 0;
    }
    /*进行状态估计，这个函数是最重要的一个*/
    status Estimator::Estimate(double timestamp, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc, Eigen::Matrix3d *G_R_I)
    {
        /*初始化的做法*/
        if (status_ == status::kInValid)
        {
            /*initializer_是一个指针*/
            /*如果不存够10组数，那么就一直在这个循环中*/
            if (!initializer_->Initialize(acc, &G_R_I_))
            {
                G_R_I->setIdentity();
                std::cout<<"initializating..."<<std::endl;
                return status::kInValid;
            }
            
            /*上面的if不运行了，说明初始化结束了，可以运行下面了*/
            last_timestamp_ = timestamp;
            bg_.setZero();
            cov_.setZero();
            cov_.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.5 * 0.5 * kDeg2Rad * kDeg2Rad;
            cov_.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.1 * 0.1 * kDeg2Rad * kDeg2Rad;
            *G_R_I = G_R_I_;

            status_ = status::kValid;
            return status::kValid;
        }
        /*下面是status_为kValid的时候，可以执行卡尔曼滤波更新的操作*//        
        double  delta_t = timestamp - last_timestamp_;
        last_timestamp_ = timestamp;

        //get ready
        Eigen::Matrix3d prior_G_R_I;
        Eigen::Vector3d prior_bg;
        Eigen::Matrix<double,6,6> prior_cov;
        propagator_->PropagateMeanAndCov(G_R_I_,bg_,cov_,gyro,delta_t,&prior_G_R_I , &prior_bg, &prior_cov);

        /*这里到底有什么用呢？*/
        acc_buffer_.push_back(acc);
        if(acc_buffer_.size()> config_.buffer_size){
            acc_buffer_.pop_front();
        }
        
        /*用acc_mean取代acc，实际上是做了一个平滑滤波处理*/
        Eigen::Vector3d acc_mean;
        for(const Eigen::Vector3d & one_acc:acc_buffer_){
            acc_mean += one_acc;
        }

        // acc_mean = acc_mean/acc_buffer_.size();/*这样显然是不行的*/
        acc_mean = acc_mean/ static_cast<double>(acc_buffer_.size());


        /*update*/
        Update(prior_G_R_I, prior_bg, prior_cov, acc_mean, acc_noise_mat_, &G_R_I_, &bg_, &cov_);
        *G_R_I = G_R_I_;
        count++;
        if(count%100==0){
            std::cout<<"ESKF"<<std::endl;
        }
        return status::kValid;
    }
} // namespace OriEst
