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

#define ANGLE_SEARCH false;
//#include <opencv2/>
class Robot
{
public:
	cv::Scalar ColorDetect{ 255, 0, 0 };
	Robot(int id, int n, cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners);
	cv::Rect trackRobot(cv::Mat& frame, cv::Mat& outImage);
	int getId();
	bool verify = false;
	cv::Rect getPosition();
	void initPosition(cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners);


	float outFi = 0;

	std::vector<cv::Point2f> p0, p1;
	std::vector< cv::Vec3d > tvecs = { 0 };
	std::vector< cv::Vec3d > rvecs = { 0 };
private:
	bool pointInMU(std::vector<cv::Point2f> p, cv::Point2f pointS);
	void trackerInit(int n, cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners);
	
	void initClass(cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners);
	bool checkClass(cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners);
	int DRC(cv::Mat& inputImg);
	int parametrDrc;

	cv::Rect last_position;
	int id;
	cv::Ptr<cv::Tracker> tracker;

	//Optical Flow
	std::vector<cv::Scalar> colors;
	cv::Mat old_frame;
	cv::Point2f pBase = cv::Point2f(150, 0);
	cv::Mat old_gray;
	std::vector<float> dFi;

	bool PSR(int tresh, cv::Mat& frame, cv::Rect bbox);
	cv::Mat showGrahf(std::vector<float> inputVec, std::vector<cv::Scalar> color, cv::Mat inputMat, std::vector<int> fromTo);
	cv::Mat last_frame;
};

