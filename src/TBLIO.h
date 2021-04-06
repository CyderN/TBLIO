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
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud2.h>


using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TBLIO {
private:
    ros::NodeHandle nh_;
    ros::Subscriber imuSub;
    ros::Subscriber poseSub;
    void imuCallback(const sensor_msgs::ImuConstPtr& poseMsg);
    //void poseCallback(const geometry_msgs::PoseStampedPtr & imuMsg);
    void poseCallback(const sensor_msgs::PointCloud2::ConstPtr & imuMsg);

    PreintegrationType *imu_preintegrated_;
    int correction_count;
    NonlinearFactorGraph *graph;
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p;
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Diagonal::shared_ptr velocity_noise_model;
    noiseModel::Diagonal::shared_ptr bias_noise_model;

    NavState prev_state;
    NavState prop_state;
    imuBias::ConstantBias prev_bias;

    Values initial_values;

    // Keep track of the total error over the entire run for a simple performance metric.
    double current_position_error;
    double current_orientation_error;
    double output_time;
    int dt;

public:
    TBLIO();
};


#endif //SRC_TBLIO_H
