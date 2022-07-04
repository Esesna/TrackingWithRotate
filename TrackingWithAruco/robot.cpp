#include "robot.h"
#define M_PI 3.14159265358979323846
Robot::Robot(int id, int n, cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners):
    id(id), last_position(bbox)
{
    //std::cout << "new robot: " << id << std::endl;
    trackerInit(n, bbox, frame, markerCorners);
    cv::RNG rng;
    for (int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(cv::Scalar(r, g, b));
    }
}

void Robot::trackerInit(int n, cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners)
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

    //int dW = bbox.width * 0.1;
    //int dH = bbox.height * 0.1;

    //bbox.x -= dW;
    //bbox.y -= dH;
    //bbox.width += dW * 2;
    //bbox.height += dH * 2;

    tracker->init(frame, bbox);
    initPosition(bbox, frame, markerCorners);
}

void Robot::initPosition(cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners)
{
    old_frame = frame;
    last_position = bbox;
    tracker = cv::TrackerKCF::create();
    tracker->init(frame, bbox);
    ColorDetect = cv::Scalar(0, 255, 0);

    std::vector<cv::Point2f> p_temp;
    cv::cvtColor(frame, old_gray, cv::COLOR_BGR2GRAY);
    //cv::goodFeaturesToTrack(old_gray, p_temp, 100, 0.3, 3, cv::Mat(), 7, true, 0.04);
    p0 = {};
    //for (int i = 0; i < p_temp.size(); i++)
    //{
    //    if (pointInMU(markerCorners, p_temp[i]))
    //    {
    //        p0.push_back(p_temp[i]);
    //    }
    //}
    p0 = markerCorners;
    std::cout << "p0.size() = " << p0.size() << std::endl;

    pBase.x = 150;
    pBase.y = 0;

    // Create a mask image for drawing purposes
    dFi = {};
    for (int i = 0; i < p0.size(); i++)
    {
        cv::Point2f pTemptCos = p0[i];
        pTemptCos.x -= 150;
        pTemptCos.y -= 150;
        float cosFi;
        cosFi = (pBase.x * pTemptCos.x + pBase.y * pTemptCos.y);
        cosFi /= sqrt(pBase.x * pBase.x + pBase.y * pBase.y);
        cosFi /= sqrt(pTemptCos.x * pTemptCos.x + pTemptCos.y * pTemptCos.y);
        float Fi = acos(cosFi) * 180 / M_PI;
        if (pTemptCos.y < 0)
        {
            Fi = -Fi;
        }
        dFi.push_back(Fi);
    }
    if (dFi.size() > 0)
    {
        std::vector<float> dFiTemp = dFi;
        sort(dFiTemp.begin(), dFiTemp.end());
        outFi = dFiTemp[(int)((dFiTemp.size() - 1) / 2)];
    }
}

cv::Rect Robot::trackRobot(cv::Mat& frame, cv::Mat &outImage)
{
    //int dW = last_position.width * 0.1;
    //int dH = last_position.height * 0.1;

    //last_position.x         -= dW;
    //last_position.y         -= dH;
    //last_position.width     += dW * 2;
    //last_position.height    += dH * 2;

    bool ok = tracker->update(frame, last_position);

    //last_position.x         += dW;
    //last_position.y         += dH;
    //last_position.width     -= dW * 2;
    //last_position.height    -= dH * 2;

    if (ok)
    {
        cv::Mat frame_gray;
        std::vector<uchar> status;
        std::vector<float> err;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT)+(cv::TermCriteria::EPS), 10, 0.03);
        if (p0.size()>0)
        {
            calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, cv::Size(15, 15), 2, criteria);
        }
        std::vector<cv::Point2f> pTako = p0;
        std::vector<float> arrFi;

        for (int i = 0; i < p0.size(); i++)
        {
            pTako[i].x -= 150;
            pTako[i].y -= 150;

            float cosFi;
            cosFi = (pBase.x * pTako[i].x + pBase.y * pTako[i].y);
            cosFi /= sqrt(pBase.x * pBase.x + pBase.y * pBase.y);
            cosFi /= sqrt(pTako[i].x * pTako[i].x + pTako[i].y * pTako[i].y);
            float Fi = acos(cosFi) * 180 / M_PI;
            if (pTako[i].y < 0)
            {
                Fi = -Fi;
            }
            Fi -= dFi[i];
            arrFi.push_back(Fi);
        }
        if (arrFi.size()>0)
        {

            sort(arrFi.begin(), arrFi.end());
            outFi = arrFi[(int)((arrFi.size() - 1) / 2)];
        }


        std::vector<cv::Point2f> good_new;
        cv::Mat mask = cv::Mat::zeros(old_frame.size(), old_frame.type());

        cv::Point2f centerAruco;
        for (uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if (status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                //line(mask, p1[i], p0[i], colors[i], 2);
                //circle(outImage, p1[i], 5, colors[i], -1);
            }
            centerAruco.x += p1[i].x;
            centerAruco.y += p1[i].y;
        }
        centerAruco.x /= p1.size();
        centerAruco.y /= p1.size();

        //std::cout << "Angle: ";
        for (uint i = 0; i < good_new.size(); i++)
        {

            cv::Point2f pTemptCos = p0[i];
            pTemptCos.x -= centerAruco.x;
            pTemptCos.y -= centerAruco.y;

            float cosFi;
            cosFi = (pBase.x * pTemptCos.x + pBase.y * pTemptCos.y);
            cosFi /= sqrt(pBase.x * pBase.x + pBase.y * pBase.y);
            cosFi /= sqrt(pTemptCos.x * pTemptCos.x + pTemptCos.y * pTemptCos.y);
            float Fi = acos(cosFi) * 180 / M_PI;
            if (pTemptCos.y < 0)
            {
                Fi = -Fi;
            }
            Fi += 180;
            //std::cout << std::to_string(Fi) << " ";
        }
        //std::cout << std::endl;

        //circle(outImage, centerAruco, 3, cv::Scalar(0, 0, 255), -1);
        add(outImage, mask, outImage);

        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;


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

bool Robot::pointInMU(std::vector<cv::Point2f> p, cv::Point2f pointS)
{
    bool result = false;
    int j = p.size() - 1;
    for (int i = 0; i < p.size(); i++) {
        if ((p[i].y < pointS.y && p[j].y >= pointS.y || p[j].y < pointS.y && p[i].y >= pointS.y) &&
            (p[i].x + (pointS.y - p[i].y) / (p[j].y - p[i].y) * (p[j].x - p[i].x) < pointS.x))
            result = !result;
        j = i;
    }
    return result;
}