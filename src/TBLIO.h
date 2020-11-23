//
// Created by xcy on 2020/11/22.
//

#ifndef SRC_TBLIO_H
#define SRC_TBLIO_H

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TBLIO {
private:
    ros::NodeHandle nh_;
    ros::Subscriber imuSub;
    void imuCallback(const sensor_msgs::ImuConstPtr& imuMsg);

    PreintegrationType *imu_preintegrated_;
    int correction_count;
    NonlinearFactorGraph *graph;
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Diagonal::shared_ptr velocity_noise_model;
    noiseModel::Diagonal::shared_ptr bias_noise_model;

public:
    TBLIO();
};


#endif //SRC_TBLIO_H
