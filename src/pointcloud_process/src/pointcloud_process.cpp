#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"

using namespace std;
struct PointPair{
    PointPair(double x1_, double y1_, double z1_, double x2_, double y2_, double z2_, double radius_, double lamb_min, double lamb_max)
    {
        x1 = x1_; y1 = y1_; z1 = z1_;
        x2 = x2_; y2 = y2_; z2 = z2_;
        radius = radius_;
        lambda_max = lamb_max;
        lambda_min = lamb_min;
    }
    ~PointPair(){}
    double x1, y1, z1;
    double x2, y2, z2;
    double radius;
    double lambda_min;
    double lambda_max;
};
class PointCloudProcess{
public:
    PointCloudProcess(const ros::NodeHandle& nh_, double x0_, double y0_, double ph0_){
        nh = nh_;
        x0 = x0_;
        y0 = y0_;
        alpha = ph0_;
        radius_camera = 0.17;
        height_camera = 1.1484;
        w2c_1 = cv::Mat(4, 4, CV_64FC1);
        w2c_1.at<double>(0, 0) = sin(alpha);
        w2c_1.at<double>(0, 1) = 0;
        w2c_1.at<double>(0, 2) = cos(alpha);
        w2c_1.at<double>(0, 3) = radius_camera*cos(alpha) + x0;
        w2c_1.at<double>(1, 0) = -cos(alpha);
        w2c_1.at<double>(1, 1) = 0;
        w2c_1.at<double>(1, 2) = sin(alpha);
        w2c_1.at<double>(1, 3) = radius_camera*sin(alpha) + y0;
        w2c_1.at<double>(2, 0) = 0;
        w2c_1.at<double>(2, 1) = -1;
        w2c_1.at<double>(2, 2) = 0;
        w2c_1.at<double>(2, 3) = height_camera;
        w2c_1.at<double>(3, 0) = 0;
        w2c_1.at<double>(3, 1) = 0;
        w2c_1.at<double>(3, 2) = 0;
        w2c_1.at<double>(3, 3) = 1;
        w2c_2 = cv::Mat(4, 4, CV_64FC1);
        w2c_2.at<double>(0, 0) = cos(alpha);
        w2c_2.at<double>(0, 1) = 0;
        w2c_2.at<double>(0, 2) = -sin(alpha);
        w2c_2.at<double>(0, 3) = -radius_camera*sin(alpha) + x0;
        w2c_2.at<double>(1, 0) = sin(alpha);
        w2c_2.at<double>(1, 1) = 0;
        w2c_2.at<double>(1, 2) = cos(alpha);
        w2c_2.at<double>(1, 3) = radius_camera*cos(alpha) + y0;
        w2c_2.at<double>(2, 0) = 0;
        w2c_2.at<double>(2, 1) = -1;
        w2c_2.at<double>(2, 2) = 0;
        w2c_2.at<double>(2, 3) = height_camera;
        w2c_2.at<double>(3, 0) = 0;
        w2c_2.at<double>(3, 1) = 0;
        w2c_2.at<double>(3, 2) = 0;
        w2c_2.at<double>(3, 3) = 1;
        w2c_3 = cv::Mat(4, 4, CV_64FC1);
        w2c_3.at<double>(0, 0) = -sin(alpha);
        w2c_3.at<double>(0, 1) = 0;
        w2c_3.at<double>(0, 2) = -cos(alpha);
        w2c_3.at<double>(0, 3) = -radius_camera*cos(alpha) + x0;
        w2c_3.at<double>(1, 0) = cos(alpha);
        w2c_3.at<double>(1, 1) = 0;
        w2c_3.at<double>(1, 2) = -sin(alpha);
        w2c_3.at<double>(1, 3) = -radius_camera*sin(alpha) + y0;
        w2c_3.at<double>(2, 0) = 0;
        w2c_3.at<double>(2, 1) = -1;
        w2c_3.at<double>(2, 2) = 0;
        w2c_3.at<double>(2, 3) = height_camera;
        w2c_3.at<double>(3, 0) = 0;
        w2c_3.at<double>(3, 1) = 0;
        w2c_3.at<double>(3, 2) = 0;
        w2c_3.at<double>(3, 3) = 1;
        w2c_4 = cv::Mat(4, 4, CV_64FC1);
        w2c_4.at<double>(0, 0) = -cos(alpha);
        w2c_4.at<double>(0, 1) = 0;
        w2c_4.at<double>(0, 2) = sin(alpha);
        w2c_4.at<double>(0, 3) = radius_camera*sin(alpha) + x0;
        w2c_4.at<double>(1, 0) = -sin(alpha);
        w2c_4.at<double>(1, 1) = 0;
        w2c_4.at<double>(1, 2) = -cos(alpha);
        w2c_4.at<double>(1, 3) = -radius_camera*cos(alpha) + y0;
        w2c_4.at<double>(2, 0) = 0;
        w2c_4.at<double>(2, 1) = -1;
        w2c_4.at<double>(2, 2) = 0;
        w2c_4.at<double>(2, 3) = height_camera;
        w2c_4.at<double>(3, 0) = 0;
        w2c_4.at<double>(3, 1) = 0;
        w2c_4.at<double>(3, 2) = 0;
        w2c_4.at<double>(3, 3) = 1;

        pcl_pub1 = nh.advertise<sensor_msgs::PointCloud2>("pcl_processed_1", 1);
        pcl_sub1 = nh.subscribe("/kinect_first/kinect2/hd/points", 10, &PointCloudProcess::cloudCB1, this);
        pcl_sub2 = nh.subscribe("/kinect_second/kinect2/hd/points", 10, &PointCloudProcess::cloudCB2, this);
        pcl_sub3 = nh.subscribe("/kinect_third/kinect2/hd/points", 10, &PointCloudProcess::cloudCB3, this);
        pcl_sub4 = nh.subscribe("/kinect_fouth/kinect2/hd/points", 10, &PointCloudProcess::cloudCB4, this);

    }
    ~PointCloudProcess(){}

    void cloudCB1(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_origin;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled;
        sensor_msgs::PointCloud2 output;
        pcl::fromROSMsg(input, cloud_origin);
        cout<<"the number of points of origin cloud is :"<<cloud_origin.points.size()<<endl;

        //===============================down sample the original point cloud==================================//

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter(true);
        statFilter.setInputCloud(cloud_origin.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered);
        cout<<"the number of points of filtered cloud is :"<<cloud_filtered.points.size()<<endl;

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud_filtered.makeShared());
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(cloud_downsampled);
        cout<<"the number of points of downsampled cloud is :"<<cloud_downsampled.points.size()<<endl;

        //=====================================================================================================//


        //===============================remove the planer from the point cloud==================================//

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02);
        seg.setInputCloud(cloud_downsampled.makeShared());
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0)
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset");
        }
        std::cout<<"Planar coefficients:"<<coefficients->values[0]<<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<" "<<coefficients->values[3]<<endl;
        for(size_t i = 0; i < inliers->indices.size(); i++)
        {
            cloud_downsampled.points[inliers->indices[i]].x = 0;
            cloud_downsampled.points[inliers->indices[i]].y = 0;
            cloud_downsampled.points[inliers->indices[i]].z = 0;
        }

        cloud = cloud_downsampled;
        //=======================================================================================================//


        //===============================remove the arm model from the point cloud========================================//

        PointPair pp1(0.812,0.048,0.506,1.384,0.048,0.506,0.05,0.0,1.0);
        double x, y, z;
        double x1, y1, z1;
        double x2, y2, z2;
        double x3, y3, z3;
        double radius;
        double lambda;
        double lambda_min = -0.15;
        double lambda_max = 1.15;
        double length_away;

        cv::Mat posi1(4, 1, CV_64FC1);
        cv::Mat posi2(4, 1, CV_64FC1);
        cv::Mat posi1_prime(4, 1, CV_64FC1);
        cv::Mat posi2_prime(4, 1, CV_64FC1);
        posi1.at<double>(0, 0) = pp1.x1; posi1.at<double>(1, 0) = pp1.y1; posi1.at<double>(2, 0) = pp1.z1; posi1.at<double>(3, 0) = 1;
        posi2.at<double>(0, 0) = pp1.x2; posi2.at<double>(1, 0) = pp1.y2; posi2.at<double>(2, 0) = pp1.z2; posi2.at<double>(3, 0) = 1;
        posi1_prime = w2c_1.inv()*posi1;
        posi2_prime = w2c_1.inv()*posi2;
        x1 = posi1_prime.at<double>(0, 0);
        y1 = posi1_prime.at<double>(1, 0);
        z1 = posi1_prime.at<double>(2, 0);
        x2 = posi2_prime.at<double>(0, 0);
        y2 = posi2_prime.at<double>(1, 0);
        z2 = posi2_prime.at<double>(2, 0);
        radius = 0.1;

        pcl::PointCloud<pcl::PointXYZ>::iterator index = cloud.begin();
        //for(size_t i = 0; i < cloud.points.size(); i ++)
        for(index = cloud.begin(); index != cloud.end(); )
        {

             x3 = (*index).x;
             y3 = (*index).y;
             z3 = (*index).z;
             lambda = ((x3-x1)*(x2-x1)+(y3-y1)*(y2-y1)+(z3-z1)*(z2-z1))/((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
             if((lambda>lambda_min)&&(lambda<lambda_max))
             {
                  x = x1 + lambda*(x2 - x1);
                  y = y1 + lambda*(y2 - y1);
                  z = z1 + lambda*(z2 - z1);
                  length_away = sqrt((x - x3)*(x - x3)+(y - y3)*(y - y3)+(z - z3)*(z - z3));
                  if((length_away <= radius)||((x3 == 0)&&(y3 == 0)&&(z3 == 0)))
                  {
                      index = cloud.erase(index);
                  }
                  else
                  {
                      index ++;
                  }
              }
             else
             {
                 index ++;
             }
        }
        pcl::toROSMsg(cloud, output);
        cout<<"the number of points of processed cloud is :"<<cloud.points.size()<<endl;
        output.header.frame_id = "base_link_0";
        pcl_pub1.publish(output);
        //===============================================================================================================//
    }
    void cloudCB2(const sensor_msgs::PointCloud2& input){}
    void cloudCB3(const sensor_msgs::PointCloud2& input){}
    void cloudCB4(const sensor_msgs::PointCloud2& input){}
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub1;
        ros::Subscriber pcl_sub2;
            ros::Subscriber pcl_sub3;
                ros::Subscriber pcl_sub4;
    ros::Publisher pcl_pub1;
    double x0, y0, alpha;
    double radius_camera;
    double height_camera;
    cv::Mat w2c_1;
    cv::Mat w2c_2;
    cv::Mat w2c_3;
    cv::Mat w2c_4;

};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_create");
    ros::NodeHandle nh;

    double x0 = 0;
    double y0 = 0;
    double ph = 0;
    cout<<"begin process:"<<endl;

    PointCloudProcess pcp(nh, x0, y0, ph);
    ros::spin();
    return 0;
}
