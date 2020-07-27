

#pragma once
#include <queue>
#include <Eigen/Dense>


namespace OriEst
{

    class Initializer
    {

    public:

    struct Config
    {
        /* data */
        size_t acc_buffer_size = 10;
        double max_acc_std = 1;
    
    };
    
        Initializer() = default;
        bool Initialize(Eigen::Vector3d acc, Eigen::Matrix3d *G_R_I);

    private:
    const Config config_;
    std::deque<Eigen::Vector3d ,Eigen::aligned_allocator<Eigen::Vector3d>> acc_buffer_;
    };
} // namespace OriEst
