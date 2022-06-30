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
    bool ok = tracker->update(frame, last_position);

    //last_position.width = last_position.height;
    //last_position.height = last_position.width;
    // 
    //last_position.x = last_position.x;
    //last_position.y = last_position.y;

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