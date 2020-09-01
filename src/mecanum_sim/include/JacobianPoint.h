#ifndef _JACOBIANPOINT_H
#define _JACOBIANPOINT_H
#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/mat.hpp"

using namespace std;
using namespace cv;

class JacobianPoint
{
public:
    JacobianPoint(){}
    JacobianPoint(int num_, double x_, double y_, double z_, double x0_, double y0_, double alpha, double q1_, double q2_, double q3_, double q4_, double q5_, double q6_ ){
        num = num_;
	x = x_;
	y = y_;
	z = z_;
	x0 = x0_;
	y0 = y0_;
	ph = alpha;
	q1 = q1_;
	q2 = q2_;
	q3 = q3_;
	q4 = q4_;
	q5 = q5_;
	q6 = q6_;
    a = 0.20;//0.5050;
    b = 0.0;//-0.1966;
    c = 0.4027;//0.6640;
	
}
    ~JacobianPoint(){}
    void calcJacobianWhole();
    void calcJacobianNear();
    void calcJacobianStar();
    double calcDistanceToObstacle(double x_o, double y_o, double z_o);
    void calcAngularVelocities(double w1, double w2, double w3, double w4, double w5, double w6, double w7, double w8, double w9);


int num;
double x0, y0;
double x, y, z, ph;
double a, b, c;
double distance;
double x_world, y_world, z_world;
double x1, x2, x3, x4;
double q1, q2, q3, q4, q5, q6;
double dq1, dq2, dq3, dq4, dq5, dq6, dph, dx0, dy0;
cv::Mat JacobianWhole;
cv::Mat JacobianNear;
cv::Mat JacobianStar;
cv::Mat JacobianWhole_pi;
cv::Mat JacobianWhole_2;
};

#endif
