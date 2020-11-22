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
    TBLIO(){
        correction_count = 1;

        // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
        Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();
        initial_state(6) = 1.0;//Initialize W of Quaternion to 1.
        Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3), initial_state(4), initial_state(5));
        Point3 prior_point(initial_state.head<3>());
        Pose3 prior_pose(prior_rotation, prior_point);
        Vector3 prior_velocity(initial_state.tail<3>());
        imuBias::ConstantBias prior_imu_bias; // assume zero initial bias
        Values initial_values;
        int correction_count = 0;
        initial_values.insert(X(correction_count), prior_pose);
        initial_values.insert(V(correction_count), prior_velocity);
        initial_values.insert(B(correction_count), prior_imu_bias);
        // Assemble prior noise model and add it the graph.
        pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
        velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
        bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
        // Add all prior factors (pose, velocity, bias) to the graph.
        graph = new NonlinearFactorGraph();
        graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
        graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
        graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));
        // We use the sensor specs to build the noise model for the IMU factor.
        double accel_noise_sigma = 0.0003924;
        double gyro_noise_sigma = 0.000205689024915;
        double accel_bias_rw_sigma = 0.004905;
        double gyro_bias_rw_sigma = 0.000001454441043;
        Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
        Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
        Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
        Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
        Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
        Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration
        boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
        // PreintegrationBase params:
        p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
        p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
        // should be using 2nd order integration
        // PreintegratedRotation params:
        p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
        // PreintegrationCombinedMeasurements params:
        p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
        p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
        p->biasAccOmegaInt = bias_acc_omega_int;
        imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);

        imuSub = nh_.subscribe("imu/data_raw", 1000, &TBLIO::imuCallback, this);
        ros::spin();
    }
};


#endif //SRC_TBLIO_H
