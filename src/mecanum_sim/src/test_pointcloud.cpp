#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <math.h>

using namespace std;

struct Point3d
{
    Point3d(double x_, double y_, double z_){
        x = x_;
        y = y_;
        z = z_;
    }
    ~Point3d(){}
    double x,y,z;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_create");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;


    double x, y, z;
    double x1, y1, z1;
    double x2, y2, z2;
    double x3, y3, z3;
    double radius;
    double lambda;
    double lambda_min = 0;
    double lambda_max = 1;
    double length_away;
    double resolution = 0.01;

    x1 = 0.5; x2 = 0.6;
    y1 = -0.4; y2 = 0;
    z1 = 0.6; z2 = 1;
    radius = 0.05;

    double x_min = min(x1, x2) - radius + lambda_min*abs(x2 - x1);
    double x_max = max(x1, x2) + radius + (lambda_max - 1)*abs(x2 - x1);
    double y_min = min(y1, y2) - radius + lambda_min*abs(y2 - y1);
    double y_max = max(y1, y2) + radius + (lambda_max - 1)*abs(y2 - y1);
    double z_min = min(z1, z2) - radius + lambda_min*abs(z2 - z1);
    double z_max = max(z1, z2) + radius + (lambda_max - 1)*abs(z2 - z1);

    vector<Point3d> pts;

    for(int i = int(x_min/resolution); i < int(x_max/resolution); i ++)
    {
        for(int j = int(y_min/resolution); j < int(y_max/resolution); j ++)
        {
            for(int k = int(z_min/resolution); k < int(z_max/resolution); k ++)
            {
                x3 = i*resolution;
                y3 = j*resolution;
                z3 = k*resolution;
                lambda = ((x3-x1)*(x2-x1)+(y3-y1)*(y2-y1)+(z3-z1)*(z2-z1))/((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
                if((lambda>lambda_min)&&(lambda<lambda_max))
                {
                    x = x1 + lambda*(x2 - x1);
                    y = y1 + lambda*(y2 - y1);
                    z = z1 + lambda*(z2 - z1);
                    length_away = sqrt((x - x3)*(x - x3)+(y - y3)*(y - y3)+(z - z3)*(z - z3));
                    if(length_away <= radius)
                    {
                        Point3d p(x3,y3,z3);
                        pts.push_back(p);
                    }
                }
            }
        }
    }

    cout<<"points number is :"<<pts.size()<<endl;
    cloud.width = pts.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width*cloud.height);

    for(int i = 0; i < pts.size(); i ++)
    {
        cloud.points[i].x = pts[i].x;
        cloud.points[i].y = pts[i].y;
        cloud.points[i].z = pts[i].z;
    }

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
