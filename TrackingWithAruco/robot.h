#pragma once

#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/tracking.hpp>

class Robot
{
public:
	cv::Scalar ColorDetect{ 255, 0, 0 };
	Robot(int id, int n, cv::Rect bbox, cv::Mat& frame);
    cv::Rect trackRobot(cv::Mat& frame);
	int getId();
	bool verify = false;
	cv::Rect getPosition();
	void initPosition(cv::Rect bbox, cv::Mat& fram);
private:
	void trackerInit(int n, cv::Rect bbox, cv::Mat& frame);
	cv::Rect last_position;
	int id;
	cv::Ptr<cv::Tracker> tracker;
};

