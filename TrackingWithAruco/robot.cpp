#include "robot.h"

Robot::Robot(int id, int n, cv::Rect bbox, cv::Mat& frame):
    id(id), last_position(bbox)
{
    //std::cout << "new robot: " << id << std::endl;
    trackerInit(n, bbox, frame);
}

void Robot::trackerInit(int n, cv::Rect bbox, cv::Mat& frame)
{
    std::string trackerTypes[8] = { "MIL", "KCF", "GOTURN", "CSRT", "DaSiamRPN" };
    std::string trackerType = trackerTypes[n];
    if (trackerType == "MIL")
        tracker = cv::TrackerMIL::create();
    if (trackerType == "KCF")
        tracker = cv::TrackerKCF::create();
    if (trackerType == "GOTURN")
        tracker = cv::TrackerGOTURN::create();
    if (trackerType == "CSRT")
        tracker = cv::TrackerCSRT::create();
    if (trackerType == "DaSiamRPN")
        tracker = cv::TrackerDaSiamRPN::create();

    bbox.x         -= bbox.width * 0.1;
    bbox.y         -= bbox.height * 0.1;
    bbox.width     += bbox.height * 0.2;
    bbox.height    += bbox.width * 0.2;

    tracker->init(frame, bbox);
}

void Robot::initPosition(cv::Rect bbox, cv::Mat& frame)
{
    last_position = bbox;
    tracker = cv::TrackerKCF::create();
    tracker->init(frame, bbox);
    ColorDetect = cv::Scalar(0, 255, 0);
}

cv::Rect Robot::trackRobot(cv::Mat& frame)
{
    int dW = last_position.width * 0.1;
    int dH = last_position.height * 0.1;

    last_position.x         -= dW;
    last_position.y         -= dH;
    last_position.width     += dW * 2;
    last_position.height    += dH * 2;

    bool ok = tracker->update(frame, last_position);

    last_position.x         += dW;
    last_position.y         += dH;
    last_position.width     -= dW * 2;
    last_position.height    -= dH * 2;

    if (ok)
    {
        ColorDetect = cv::Scalar(255, 0, 0);
        
        //last_position.width  /= 2;
        //last_position.height /= 2;
        //last_position.x += last_position.width / 2  ;
        //last_position.y += last_position.height / 2 ;

        return last_position;
    }
    else
    {
        ColorDetect = cv::Scalar(0, 0, 255);
        return cv::Rect2d(0, 0, 0, 0);
    }
}

int Robot::getId()
{
    return id;
}

cv::Rect Robot::getPosition()
{
    return last_position;
}