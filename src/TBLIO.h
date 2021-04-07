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
#include <gtsam/nonlinear/ISAM2.h>
#include <nav_msgs/Path.h>


#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf_conversions/tf_eigen.h>


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
    ros::Publisher imuPosePublisher;
    void imuCallback(const sensor_msgs::ImuConstPtr& poseMsg);
    void poseCallback(const geometry_msgs::PoseStampedPtr & imuMsg);

    PreintegrationType *imu_preintegrated_;
    int correction_count;
    NonlinearFactorGraph *graph;
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Diagonal::shared_ptr velocity_noise_model;
    noiseModel::Diagonal::shared_ptr bias_noise_model;

    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p;

    NavState prev_state;
    NavState prop_state;
    imuBias::ConstantBias prev_bias;

    Values initial_values;

    // Keep track of the total error over the entire run for a simple performance metric.
    double current_position_error;
    double current_orientation_error;
    double output_time;
    double dt;
    bool imuEmpty;

    ISAM2 optimizer;
    void resetOptimization();
    nav_msgs::Path robotPath;
    ros::Publisher path_pub;

public:
    TBLIO();
};


#endif //SRC_TBLIO_H