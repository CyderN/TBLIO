/*
 * DBSCAN For VLP-16.
 * All right reserved.
 */

#include <queue>
#include <ros/ros.h>

#include <iostream>
#include <ceres/ceres.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ctime>


#include "groundRemovalRANSAC.h"
#include "dbscan_correction.h"

using namespace std;

double scan_num360;
double scan_range;
int min_cluster;
double distance_max;
double distance_min;
double height_max;
double height_min;
int MinPts;
double EPS, tree_residual, tree_radius_max, tree_radius_min;

double groundZMax;
double groundZMin;
double inlierRatio;
int sampleNum;
double confidence;
double inlinerThreshold;
double ratioCondition;
double upperBorder;
string laser_name;


class point{
public:
    float x;
    float y;
    float z;
    float rou;
    float sita;
    int cluster=0;
    int pointType=1;//1 noise 2 border 3 core
    int pts=0;//points in MinPts
    vector<int> corepts;
    int visited = 0;
    point (){}
    point (float a,float b,float c ,int d){
        x = a;
        y = b;
        z = c;
        cluster = d;
        rou = sqrt(a*a + b*b);
        sita = atan2(b , a);
    }
};

float squareDistance(point a,point b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    //return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)/5.0);
}

struct circleResidual {
    circleResidual(double xi, double yi)
            : xi_(xi),yi_(yi) {}
    template <typename T> bool operator()(const T* const x,
                                          const T* const y,
                                          const T* const r,
                                          T* residual) const {
        residual[0] = r[0]*r[0] - (x[0]-xi_)*(x[0]-xi_) - (y[0]-yi_)*(y[0]-yi_);//TODO
        return true;
    }
private:
    const double xi_;
    const double yi_;
};
ros::Publisher cloud_pub;
ros::Publisher tree_cloud_pub;
ros::Publisher tree_visual_cloud_pub;

void DBSCAN(sensor_msgs::PointCloud& dataset,double eps,int minpts){//按照xy密度来进行聚类。
    /*Project 3D to 2D*/
    sensor_msgs::PointCloud tempz = dataset;
    sensor_msgs::PointCloud tempTrue = dataset;

    sensor_msgs::PointCloud treeCenters = dataset;
    treeCenters.points.clear();
    treeCenters.channels.clear();
    for(int i = 0; i < tempTrue.points.size(); i++){
        //自己写的八叉树难以处理某一维度全为0的情况
        tempTrue.points[i].z = 0.05*rand() / double(RAND_MAX);
    }

    /*Run DBscan*/
    clock_t startTime, endTime;
    startTime = clock();//计时开始
    NewDbscanDriver oldDriver;
    oldDriver.setEPSandMinPts(eps, minpts);
    oldDriver.dbscanClustering(tempTrue);
    dataset.channels = oldDriver.PCLforOutput.channels;
    endTime = clock();//计时结束
    cout << "The clustering run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    startTime = clock();//计时开始
    /*Remove little cluster and add Residual*/
    auto ClusterBegin = dataset.channels[NewDbscanDriver::cluster].values.begin();
    auto ClusterEnd = dataset.channels[NewDbscanDriver::cluster].values.end();
    int maxCluster = (int)*max_element(ClusterBegin, ClusterEnd);

    /*Get cluster index*/
    vector<vector<int>> currentClusterIdx;
    currentClusterIdx.resize(maxCluster+1);

    for(int j = 0; j < dataset.points.size(); j++){
        currentClusterIdx[(int)dataset.channels[NewDbscanDriver::cluster].values[j]].push_back(j);
    }
    /*Remove little cluster*/
    for(int j = 1; j < currentClusterIdx.size(); j++){
        double temp_dist = dataset.points[currentClusterIdx[j].front()].x*dataset.points[currentClusterIdx[j].front()].x
                           +dataset.points[currentClusterIdx[j].front()].y*dataset.points[currentClusterIdx[j].front()].y;
        if(currentClusterIdx[j].size() < (min_cluster / temp_dist) ){
            for(int i = 0; i < currentClusterIdx[j].size(); i++){
                dataset.channels[NewDbscanDriver::type].values[currentClusterIdx[j][i]] = NewDbscanDriver::little;
                dataset.channels[NewDbscanDriver::cluster].values[currentClusterIdx[j][i]] = 0.0;
            }
            currentClusterIdx.erase(currentClusterIdx.begin() + j);
            j--;
        }
    }
    for(int j = 1; j < currentClusterIdx.size(); j++) {
        //优化初始值把类中第一个点的位置复制给他
        double x = dataset.points[currentClusterIdx[j].front()].x;
        double y = dataset.points[currentClusterIdx[j].front()].y;
        double r = 0.15;

        ceres::Problem problem;
        for (int i = 0; i < currentClusterIdx[j].size() ; ++i) {
            problem.AddResidualBlock(
                    new ceres::AutoDiffCostFunction<circleResidual, 1, 1, 1, 1>(
                            new circleResidual(dataset.points[currentClusterIdx[j][i]].x, dataset.points[currentClusterIdx[j][i]].y)),
                    //inverse normalization and transfer from deg. to rad.
                    NULL,
                    &x, &y, &r);
        }
        //solve the problem
        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        Solve(options, &problem, &summary);
        //std::cout << summary.BriefReport() << "\n";
        r= abs(r);
        //std::cout << "Before   x: " << x << " y: " << y << " r: " << r <<" residual: "<<summary.final_cost<<"\n";
        //push the result to output vector

        if(r>=tree_radius_min && r<=tree_radius_max && summary.final_cost<=tree_residual){
            //Final good tree output
            //std::cout << "Confirmed   x: " << x << " y: " << y << " r: " << r <<"\n";
            geometry_msgs::Point32 treeCenterPt;
            treeCenterPt.x = x;
            treeCenterPt.y = y;
            treeCenterPt.z = r;
            treeCenters.points.push_back(treeCenterPt);
//            //Check If Pass Through or Not
//            passThroughTesting(dataset, currentClusterIdx[j]);
        }else{
            //UNVISUAL NOISE
            for(int k = 0; k < currentClusterIdx[j].size(); k++){
                dataset.channels[NewDbscanDriver::type].values[currentClusterIdx[j][k]] = NewDbscanDriver::strange;
                dataset.channels[NewDbscanDriver::cluster].values[currentClusterIdx[j][k]] = 0.0;
            }
        }
    }
    tree_visual_cloud_pub.publish(dataset);
    treeCenters.header = dataset.header;
    tree_cloud_pub.publish(treeCenters);
    endTime = clock();//计时结束
    cout << "The detection run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}



void point_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    clock_t startTime,endTime;
    startTime = clock();//计时开始
    //Convert sensor_msgs::PointCloud2 to sensor_msgs::PointCloud
    sensor_msgs::PointCloud output;
    output.header = input->header;
    sensor_msgs::convertPointCloud2ToPointCloud(*input, output);
    cloud_pub.publish(output);

//    //test PCA
//    PCA_Eigen(output);

//    //test VFH
//    Voxel_Filter_Hash(output);

//    //test KDTree
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    KD_TREE_NN(output);
//    endTime = clock();//计时结束
//    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test Octree
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    OctreeDriver oldDriver;
//    oldDriver.octreeNNdemo(output);
//    endTime = clock();//计时结束
//    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test DBscan
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    DbscanDriver oldDriver;
//    oldDriver.dbscanClustering(output);
//    tree_cloud_pub.publish(oldDriver.PCLforOutput);
//    endTime = clock();//计时结束
//    cout << "The run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test FPFH
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    FPFHDriver oldDriver;
//    oldDriver.FPFH(output);
//    endTime = clock();//计时结束
//    cout << "The FPFH run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test Kmeans
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    tree_cloud_pub.publish(Kmeans(output));
//    endTime = clock();//计时结束
//    cout << "The K-means run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test gmmEM
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    tree_cloud_pub.publish(gmmEM(output));
//    endTime = clock();//计时结束
//    cout << "The gmmEM run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

//    //test spectralClustering
//    clock_t startTime,endTime;
//    startTime = clock();//计时开始
//    tree_cloud_pub.publish(spectralClustering(output));
//    endTime = clock();//计时结束
//    cout << "The spectralClustering run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

    //RANSAC BSCAN
        //test Kmeans
    startTime = clock();//计时开始
    ransacDriver oldDriver;

    oldDriver.setGroundZMaxAndMin(groundZMax, groundZMin);
    oldDriver.setInlinerRatio(inlierRatio);
    oldDriver.setSampleNum(sampleNum);
    oldDriver.setConfidence(confidence);
    oldDriver.setInlinerThreshold(inlinerThreshold);
    oldDriver.setRatioCondition(ratioCondition);
    oldDriver.setUpperBorder(upperBorder);

    oldDriver.groundRemove(output);
    //tree_cloud_pub.publish(oldDriver.PCLforOutput);

    for(int i = 0; i < oldDriver.PCLforOutput.points.size(); i++){
        if(oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values[i] == 1){
            oldDriver.PCLforOutput.points.erase(oldDriver.PCLforOutput.points.begin() + i);
            oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.erase(oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.begin() + i);
            i--;
        }
    }


    for(int i = 0; i < oldDriver.PCLforOutput.points.size(); i++){
        if(oldDriver.PCLforOutput.points[i].z >= 2.3 + height_max){
            oldDriver.PCLforOutput.points.erase(oldDriver.PCLforOutput.points.begin() + i);
            oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.erase(oldDriver.PCLforOutput.channels[ransacDriver::INLINER].values.begin() + i);
            i--;
        }
    }

    endTime = clock();//计时结束
    cout << "The RANSAC run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;


    sensor_msgs::PointCloud dataset = oldDriver.PCLforOutput;
    dataset.header = input->header;
    dataset.header.frame_id = laser_name;
    DBSCAN(dataset,EPS,MinPts);

//    //ORIGINAL DBSCAN
//    int counter = 0;
//    sensor_msgs::PointCloud dataset;
//    dataset.header = output.header;
//    for(auto pt_iter : output.points){
//        if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) <= (distance_max * distance_max))
//            if((pt_iter.x * pt_iter.x + pt_iter.y * pt_iter.y) >= (distance_max * distance_min))
//                if(pt_iter.z >= height_min)
//                    if(pt_iter.z <= height_max){
//                        counter++;
//                        dataset.points.push_back(pt_iter);
//                    }
//    }
//    cout<<"dataset_size: "<<dataset.points.size() << endl;
//    cout<<"EPS:     "<<EPS<<"      MinPts: "<<MinPts<<endl;



    endTime = clock();//计时结束
    cout << "The DBSCAN run time is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}





int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "dbscaner");
    ros::NodeHandle nh_;
    string tree_pt,scan_name;

    ros::param::get("~scan_topic_name",scan_name);

    ros::param::get("~EPS",EPS);
    ros::param::get("~MinPts",MinPts);
    cout<<"EPS:     "<<EPS<<"      MinPts: "<<MinPts<<endl;
    ros::param::get("~tree_residual",tree_residual);
    ros::param::get("~tree_radius_max",tree_radius_max);
    ros::param::get("~tree_radius_min",tree_radius_min);

    ros::param::get("~min_cluster",min_cluster);
    ros::param::get("~distance_max",distance_max);
    ros::param::get("~distance_min",distance_min);
    ros::param::get("~height_max",height_max);
    ros::param::get("~height_min",height_min);

    ros::param::get("~groundZMax",groundZMax);
    ros::param::get("~groundZMin",groundZMin);
    ros::param::get("~inlierRatio",inlierRatio);
    ros::param::get("~confidence",confidence);
    ros::param::get("~inlinerThreshold",inlinerThreshold);
    ros::param::get("~ratioCondition",ratioCondition);
    ros::param::get("~upperBorder",upperBorder);
    ros::param::get("~sampleNum",sampleNum);
    ros::param::get("~laser_name",laser_name);




    ros::Subscriber scan_sub = nh_.subscribe(scan_name, 1, point_callback);
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("cloud1", 100);
    tree_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("tree_center", 100);
    tree_visual_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("tree_cloud_visual", 100);



    ros::spin();
    return 0;
}