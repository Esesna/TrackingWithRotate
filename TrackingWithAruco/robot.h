#pragma once

#include <iostream>

#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
//#include <opencv2/>
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

	float outFi = 0;
private:
	void trackerInit(int n, cv::Rect bbox, cv::Mat& frame);
	cv::Rect last_position;
	int id;
	cv::Ptr<cv::Tracker> tracker;

	//Optical Flow
	std::vector<cv::Scalar> colors;
	cv::Mat old_frame;
	cv::Point2f pBase = cv::Point2f(150, 0);
	cv::Mat old_gray;
	std::vector<cv::Point2f> p0, p1;
	std::vector<float> dFi;
	
};

