//
// Created by xcy on 2020/11/22.
//

#include "TBLIO.h"

void TBLIO::poseCallback(const geometry_msgs::PoseStampedPtr & poseMsg){
    if(imuEmpty == true)return;
    /*IF KEYFRAME*/
    correction_count++;
    PreintegratedImuMeasurements& preint_imu = dynamic_cast<PreintegratedImuMeasurements&>(*imu_preintegrated_);
    ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                         X(correction_count  ), V(correction_count  ),
                         B(correction_count-1),
                         preint_imu);
    graph->add(imu_factor);
    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1),
                                                    B(correction_count  ),
                                                    zero_bias, bias_noise_model));


    noiseModel::Diagonal::shared_ptr correction_noise =
            noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
    double x, y, z, qx, qy, qz, qw;

    x = poseMsg->pose.position.x;
    y = -poseMsg->pose.position.y;
    z = -poseMsg->pose.position.z;


    tf::Quaternion tempq2;
    tf::quaternionMsgToTF(poseMsg->pose.orientation, tempq2);
    double roll,pitch,yaw;
    tf::Matrix3x3(tempq2).getRPY(roll, pitch, yaw);
    tempq2 = tf::createQuaternionFromRPY(roll, -pitch, -yaw);
    qx = tempq2.getX();
    qy = tempq2.getY();
    qz = tempq2.getZ();
    qw = tempq2.getW();

    Eigen::Matrix<double,7,1> lidarPose = Eigen::Matrix<double,7,1>::Zero();
    lidarPose(0) = x;
    lidarPose(1) = y;
    lidarPose(2) = z;
    lidarPose(3) = qx;
    lidarPose(4) = qy;
    lidarPose(5) = qz;
    lidarPose(6) = qw;
    PriorFactor<Pose3> poseFactor(X(correction_count),
                                  gtsam::Pose3(Rot3(qw,qx,qy,qz), Point3(x,y,z)), correction_noise);
    graph->add(poseFactor);
    // Now optimize and compare results.
    prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
    initial_values.insert(X(correction_count), prop_state.pose());
    initial_values.insert(V(correction_count), prop_state.v());
    initial_values.insert(B(correction_count), prev_bias);
    //LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
    ROS_INFO("Start Optimization");
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    optimizer.update(*graph,initial_values);
    optimizer.update();
    gtsam::Values result = optimizer.calculateEstimate();
    ofstream os("Pose2SLAMExample.dot");
    graph->saveGraph(os, result);
    graph->resize(0);
    initial_values.clear();
    endTime = clock();//计时结束
    cout << "Optimization run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
    ROS_INFO("End Optimization");

    // Overwrite the beginning of the preintegration for the next step.
    prev_state = NavState(result.at<Pose3>(X(correction_count)),
                          result.at<Vector3>(V(correction_count)));
    prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

    // Reset the preintegration object.
    imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

    // Print out the position and orientation error for comparison.
    Vector3 gtsam_position = prev_state.pose().translation();
    Vector3 position_error = gtsam_position - lidarPose.head<3>();
    current_position_error = position_error.norm();

    Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
    Quaternion gps_quat(lidarPose(6), lidarPose(3), lidarPose(4), lidarPose(5));
    Quaternion quat_error = gtsam_quat * gps_quat.inverse();
    quat_error.normalize();
    Vector3 euler_angle_error(quat_error.x()*2,
                              quat_error.y()*2,
                              quat_error.z()*2);
    current_orientation_error = euler_angle_error.norm();

    // display statistics
    cout << "Position error:" << current_position_error << "\t " << "Angular error:" << current_orientation_error << "\n";

    ROS_WARN("%f,\n%f,%f,%f,\n%f,%f,%f,%f,\n%f,%f,%f,\n%f,%f,%f,%f\n",
             output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
             gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
             lidarPose(0), lidarPose(1), lidarPose(2),
             gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

    output_time += 1.0;
    imuEmpty = true;

    my_pose.header = poseMsg->header;
    my_pose.pose.position.x = gtsam_position(0);
    my_pose.pose.position.y = -gtsam_position(1);
    my_pose.pose.position.z = -gtsam_position(2);

    tf::Quaternion tempq(gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w());
    tf::Matrix3x3(tempq).getRPY(roll, pitch, yaw);
    tempq = tf::createQuaternionFromRPY(roll, -pitch, -yaw);

    tf::quaternionTFToMsg(tempq, my_pose.pose.orientation);
    imuPosePublisher.publish(my_pose);

    robotPath.header = poseMsg->header;
    robotPath.poses.push_back(my_pose);
    path_pub.publish(robotPath);
}

void TBLIO::imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){

    //linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
    Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
    imu(0) = imuMsg->linear_acceleration.x;
    imu(1) = -imuMsg->linear_acceleration.y;
    imu(2) = -imuMsg->linear_acceleration.z;//Is this all minus
    imu(3) = imuMsg->angular_velocity.x;
    imu(4) = -imuMsg->angular_velocity.y;
    imu(5) = -imuMsg->angular_velocity.z;

//    imu(0) = 0.01;
//    imu(1) = 0.0;
//    imu(2) = -9.805;
//    imu(3) = 0.0;
//    imu(4) = 0.0;
//    imu(5) = 0.0;

    //Adding the IMU preintegration.
    imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
    imuEmpty = false;

    //Old if is here
}


void TBLIO::resetOptimization()
{
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer = gtsam::ISAM2(optParameters);

    gtsam::NonlinearFactorGraph newGraphFactors;
    *graph = newGraphFactors;

    gtsam::Values NewGraphValues;
    initial_values = NewGraphValues;
}

TBLIO::TBLIO(ros::NodeHandle* nh){
    if(nh == nullptr) return;

    nh_= nh;
    imuEmpty = true;

    // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
    Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();

    //my initial pose;
    initial_state(0) = 0.0;
    initial_state(1) = 0.0;
    initial_state(2) = 0.0;

    initial_state(6) = 1.0;//Initialize W of Quaternion to 1.
    Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3), initial_state(4), initial_state(5));
    Point3 prior_point(initial_state.head<3>());
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(initial_state.tail<3>());
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

    correction_count = 0;
    initial_values.insert(X(correction_count), prior_pose);
    initial_values.insert(V(correction_count), prior_velocity);
    initial_values.insert(B(correction_count), prior_imu_bias);
    // Assemble prior noise model and add it the graph.
    pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished()); // rad,rad,rad,m, m, m
    velocity_noise_model = noiseModel::Isotropic::Sigma(3,1e4); // m/s
    bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);
    // Add all prior factors (pose, velocity, bias) to the graph.
    graph = new NonlinearFactorGraph();
    graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
    graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
    graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));
    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = 0.01;
    double gyro_noise_sigma = 0.001;
    double accel_bias_rw_sigma = 0.00006;
    double gyro_bias_rw_sigma = 0.00003;
    Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
    Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
    Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration
    p = PreintegratedCombinedMeasurements::Params::MakeSharedD(10.0);
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

    // Store previous state for the imu integration and the latest predicted outcome.
    prev_state = NavState (prior_pose, prior_velocity);
    prop_state = prev_state;
    prev_bias = prior_imu_bias;

    // Keep track of the total error over the entire run for a simple performance metric.
    current_position_error = 0.0;
    current_orientation_error = 0.0;

    output_time = 0.0;
    dt = 0.01; //IMU at 100Hz

    imuSub = nh_->subscribe("imu/data_raw", 1, &TBLIO::imuCallback, this);
    poseSub = nh_->subscribe("my_pose", 1, &TBLIO::poseCallback, this);
    imuPosePublisher = nh_->advertise<geometry_msgs::PoseStamped>("imu_pose", 1);
    path_pub = nh_->advertise<nav_msgs::Path>("IMU_trajectory",1, true);
    ros::spin();
}