//
// Created by xcy on 2020/11/22.
//



#include "TBLIO.h"
void TBLIO::imuCallback(const sensor_msgs::ImuConstPtr& imuMsg){
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    //linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
    Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
    imu(0) = imuMsg->linear_acceleration.x;
    imu(1) = imuMsg->linear_acceleration.y;
    imu(2) = imuMsg->linear_acceleration.z;//Is this all minus
    imu(3) = imuMsg->angular_velocity.x;
    imu(4) = imuMsg->angular_velocity.y;
    imu(5) = imuMsg->angular_velocity.z;

    //Adding the IMU preintegration.
    int dt = 0.01; //IMU at 100Hz
    imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

    PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
    ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                         X(correction_count  ), V(correction_count  ),
                         B(correction_count-1),
                         *preint_imu);
    graph->add(imu_factor);
    imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
    graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1),
                                                    B(correction_count  ),
                                                    zero_bias, bias_noise_model));
    endTime = clock();//计时结束
    cout << "Imu run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}
TBLIO::TBLIO(){
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