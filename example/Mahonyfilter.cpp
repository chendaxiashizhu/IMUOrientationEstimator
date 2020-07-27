// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*new add*/
#include <Estimator.h>/*这就是作者的头文件*/
#include <opencv2/viz.hpp>
#include <mynteye/logger.h>
#include <mynteye/device/device.h>
#include <mynteye/device/utils.h>
#include <mynteye/util/times.h>

/**/
#include <opencv2/highgui/highgui.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"

#include "MahonyAHRSupdate.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[])
{

  //Init out orientation estimator.
  const double gyro_noise = 1e-6;
  const double gyro_bias_noise = 1e-8;
  const double acc_noise = 1e-3;
  /*需要三个参量*/
//   OriEst::Estimator orientation_estimatro(gyro_noise, gyro_bias_noise, acc_noise);
    OriEst::MahonyAHRSupdate mahonyAHRS(gyro_noise);
    // OriEst::Estimator qq(gyro_noise, gyro_bias_noise, acc_noise);
    mahonyAHRS.SetParameter(1.0,2.0);/*ki,kp*/
    // mahonyAHRS.fuck();
    // mahonyAHRS->SetParameter(0.2,2.0);


  // Set viz.
  cv::viz::Viz3d viz_windows("IMU Orientation");
  viz_windows.showWidget("ENU Frame", cv::viz::WCoordinateSystem(0.3));
  viz_windows.showWidget("IMU Frame", cv::viz::WCoordinateSystem(0.8));

  /*相机的 API*/
  auto &&api = API::Create(argc, argv);
  if (!api)
    return 1;

  bool ok;
  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok)
    return 1;
  api->ConfigStreamRequest(request);

  // Enable this will cache the motion datas until you get them.
  api->EnableMotionDatas();
  // Enable imu timestamp correspondence int device;
  api->EnableImuTimestampCorrespondence(true);

  api->Start(Source::ALL);

  cv::namedWindow("frame");

  while (true)
  {
    api->WaitForStreams();
    
    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);
    cv::Mat img;
    cv::hconcat(left_data.frame, right_data.frame, img);
    auto &&motion_datas = api->GetMotionDatas();
    std::size_t imu_count = 0;
    for (auto &&data : motion_datas)
    {
      ++imu_count;
      /*LOG(INFO) << "Imu frame_id: " << data.imu->frame_id
                  << ", timestamp: " << data.imu->timestamp
                  << ", accel_x: " << data.imu->accel[0]
                  << ", accel_y: " << data.imu->accel[1]
                  << ", accel_z: " << data.imu->accel[2]
                  << ", gyro_x: " << data.imu->gyro[0]
                  << ", gyro_y: " << data.imu->gyro[1]
                  << ", gyro_z: " << data.imu->gyro[2]
                  << ", temperature: " << data.imu->temperature;
                  */
      cv::imshow("frame", img);

      Eigen::Vector3d acc(data.imu->accel[0] * OriEst::kGravity,
                          data.imu->accel[1] * OriEst::kGravity,
                          data.imu->accel[2] * OriEst::kGravity);
      Eigen::Vector3d gyro(data.imu->gyro[0] * OriEst::kDeg2Rad,
                           data.imu->gyro[1] * OriEst::kDeg2Rad,
                           data.imu->gyro[2] * OriEst::kDeg2Rad);

      double timestamp = data.imu->timestamp * 1e-6;/*转换成为秒*/
      Eigen::Matrix3d G_R_I;
      OriEst::status status = mahonyAHRS.Mahonyfilter(timestamp, gyro, acc, &G_R_I);
    //   qq.fuck();

      // Show result.
      /*为什么要做这样的转换呢？*/
      cv::Mat cv_R = (cv::Mat_<float>(3, 3) << G_R_I(0, 0), G_R_I(0, 1), G_R_I(0, 2),
                                              G_R_I(1, 0), G_R_I(1, 1), G_R_I(1, 2),
                                              G_R_I(2, 0), G_R_I(2, 1), G_R_I(2, 2));
      cv::Affine3d pose;
      pose.linear(cv_R);
      pose.translate(cv::Vec3d(0., 0., 0.));
      viz_windows.setWidgetPose("IMU Frame", pose);
      /*其实这里的位置也可以添加*/

      if (imu_count % 10 == 0)
      {
        viz_windows.spinOnce(1);
      }
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q')
    { // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);
  return 0;
}
