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