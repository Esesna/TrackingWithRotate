#include "robot.h"
#define M_PI 3.14159265358979323846
using namespace std;
extern int countFrames;

cv::Rect2d convertCornersToRect1(std::vector<cv::Point2f> corners);

Robot::Robot(int id, int n, cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners):
    id(id), last_position(bbox)
{
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
    //std::string trackerTypes[8] = { "MIL", "KCF", "GOTURN", "CSRT", "DaSiamRPN" };
    //std::string trackerType = trackerTypes[n];
    //if (trackerType == "MIL")
    //    tracker = cv::TrackerMIL::create();
    //if (trackerType == "KCF")
    //    tracker = cv::TrackerKCF::create();
    //if (trackerType == "GOTURN")
    //    tracker = cv::TrackerGOTURN::create();
    //if (trackerType == "CSRT")
    //    tracker = cv::TrackerCSRT::create();
    //if (trackerType == "DaSiamRPN")
    //    tracker = cv::TrackerDaSiamRPN::create();

    // int dW = bbox.width * 0.1;
    // int dH = bbox.height * 0.1;

    // bbox.x -= dW;
    // bbox.y -= dH;
    // bbox.width += dW * 2;
    // bbox.height += dH * 2;

    initPosition(bbox, frame, markerCorners);
}

void Robot::initPosition(cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners)
{
    last_frame = frame;
    old_frame = frame;
    last_position = bbox;


    // KCF classification
    // tracker = cv::TrackerKCF::create();
    // tracker->init(frame, bbox);

    // DRC classification
    initClass(bbox, frame, markerCorners);


    ColorDetect = cv::Scalar(0, 255, 0);
    //std::cout << "My tut kakimto huem\n";
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
    //std::cout << "p0.size() = " << p0.size() << std::endl;

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
    //bool ok = tracker->update(frame, last_position);
    //ok = PSR(10,frame, convertCornersToRect1(p0));

    cv::Mat frame_gray;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);

    if (p0.size()>0)
    {
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, cv::Size(15, 15), 2, criteria);
    }
    else
    {
        ColorDetect = cv::Scalar(0, 0, 255);
        return cv::Rect2d(0, 0, 0, 0);
    }

#if ANGLE_SEARCH
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
#endif
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
        else
        {
            ColorDetect = cv::Scalar(0, 0, 255);
            return cv::Rect2d(0, 0, 0, 0);
        }
#if ANGLE_SEARCH
        centerAruco.x += p1[i].x;
        centerAruco.y += p1[i].y;
#endif
    }
#if ANGLE_SEARCH
    centerAruco.x /= p1.size();
    centerAruco.y /= p1.size();

    std::cout << "Angle: ";
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
#endif
    //circle(outImage, centerAruco, 3, cv::Scalar(0, 0, 255), -1);
    add(outImage, mask, outImage);

    // Now update the previous frame and previous points
    old_gray = frame_gray.clone();
    
    p0 = good_new;
    
    cv::Point2f pMax = p0[0];
    cv::Point2f pMin = p0[0];

    for (uint i = 1; i < p0.size(); i++)
    {
        if (p0[i].x > pMax.x) pMax.x = p0[i].x;
        if (p0[i].y > pMax.y) pMax.y = p0[i].y;
        if (p0[i].x < pMin.x) pMin.x = p0[i].x;
        if (p0[i].y < pMin.y) pMin.y = p0[i].y;
    }

    cv::Rect2f bbox{pMin, pMax};

    last_position = bbox;
    //std::cout << "MinMax: " << pMin << " - " << pMax << "\n";
    //std::cout << "Rect2f: " << bbox << "\n";

    bool ok = checkClass(bbox, frame, p0);

    //cv::Mat frameTmp;
    //frame.copyTo(frameTmp);
    //cv::rectangle(frameTmp, bbox, cv::Scalar(0, 255, 255));
    //cv::imshow("sukaBlya" + std::to_string(ok), frameTmp);
    //cv::waitKey(0);
    
    if (!ok)
    {
        ColorDetect = cv::Scalar(0, 0, 255);
        return cv::Rect2d(0, 0, 0, 0);
    }

    ColorDetect = cv::Scalar(255, 0, 0);
        
    //last_position.width  /= 2;
    //last_position.height /= 2;
    //last_position.x += last_position.width / 2  ;
    //last_position.y += last_position.height / 2 ;
    return last_position;
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

cv::Rect2d convertCornersToRect1(std::vector<cv::Point2f> corners)
{
    double min_x = corners[0].x;
    double min_y = corners[0].y;
    double max_x = corners[0].x;
    double max_y = corners[0].y;

    for (int j = 1; j < 4; j++)
    {

        if (min_x > corners[j].x) min_x = corners[j].x;
        if (max_x < corners[j].x) max_x = corners[j].x;

        if (min_y > corners[j].y) min_y = corners[j].y;
        if (max_y < corners[j].y) max_y = corners[j].y;

    }

    double widht = (max_x - min_x);
    double height = (max_y - min_y);

    cv::Rect2d bbox = cv::Rect2d{ min_x, min_y, widht, height };

    return bbox;
}
bool Robot::PSR(int tresh, cv::Mat& frame, cv::Rect bbox)
{
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Mat F0 = (cv::Mat(old_gray, bbox) / 128) - 1;
    cv::Mat F1 = (cv::Mat(frame_gray, bbox) / 128) - 1;
    cv::GaussianBlur(F0, F0, cv::Size(7, 7), 0);
    cv::GaussianBlur(F1, F1, cv::Size(7, 7), 0);

    cv::Mat difF = F1 - F0;
    cv::Mat G = difF.mul(difF);

    double minVal;
    double maxVal;//
    cv::Point minLoc;
    cv::Point maxLoc;//

    cv::minMaxLoc(G, &minVal, &maxVal, &minLoc, &maxLoc);

    double gMax = maxVal;
    cv::Point LocMaxG = maxLoc;
    LocMaxG.x -= 5;
    LocMaxG.y -= 5;
    G = cv::Mat(G, cv::Rect(LocMaxG.x, LocMaxG.y, 11, 11));

    cv::Mat temp;

    temp = G - maxVal;
    temp = temp.mul(temp);

    float Su = sqrt(cv::sum(temp)[0] / (temp.rows * temp.cols));

    temp = G - maxVal;

    float Ssig = cv::sum(temp)[0] / (temp.rows * temp.cols);

    float PSRval = (gMax - Su) / Ssig;

    if (PSRval > tresh) return true;
    return false;
}

void Robot::initClass(cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners)
{
    cv::Mat frameTmp;
    frameTmp = frame(bbox);
    parametrDrc = DRC(frameTmp);
}

bool Robot::checkClass(cv::Rect bbox, cv::Mat& frame, std::vector<cv::Point2f> markerCorners)
{
    cv::Mat frameTmp;
    frameTmp = frame(bbox);
    float parametrDrcTmp = DRC(frameTmp);
    //std::cout << "parametrDrc / parametrDrcTmp = " << parametrDrc / parametrDrcTmp << std::endl;
    //std::cout << "parametrDrcTmp / parametrDrc = " << parametrDrcTmp / parametrDrc << std::endl;
    //std::cout << std::endl;

    if (parametrDrc / parametrDrcTmp > 0.7 and parametrDrcTmp / parametrDrc > 0.7)
    {
        parametrDrc = parametrDrcTmp;
        return true;
    }
    return true;
}

int Robot::DRC(cv::Mat& inputImg)
{
    cv::Mat gray_planes;
    cv::cvtColor(inputImg, gray_planes, cv::COLOR_BGR2GRAY);

    float range[] = { 0, 256 }; //the upper boundary is exclusive
    const float* histRange[] = { range };
    bool uniform = true, accumulate = false;
    cv::Mat gray_hist;
    int histSize = 256;
    calcHist(&gray_planes, 1, 0, cv::Mat(), gray_hist, 1, &histSize, histRange, uniform, accumulate);
    int size_img = gray_planes.size().height * gray_planes.size().width;
    int threshold_contrast = (int)((float)size_img * 0.2 / 100.0);
    int size_hist = gray_hist.size().height * gray_hist.size().width;
    int low_lvl = 0;
    int hight_lvl = 255;
    for (int i = 0; i < size_hist; i++)
    {
        if (gray_hist.at<float>(i) > threshold_contrast)
        {
            low_lvl = i;
            break;
        }
    }
    for (int i = 0; i < size_hist; i++)
    {
        if (gray_hist.at<float>(size_hist - i - 1) > threshold_contrast)
        {
            hight_lvl = size_hist - i - 1;
            break;
        }
    }
    int treshhold = (hight_lvl + low_lvl) / 2;
    
    float blackCount = 0;
    float whiteCount = 0;

    for (int i = low_lvl; i < treshhold; i++)
    {
        blackCount += gray_hist.at<float>(i);
    }
    for (int i = hight_lvl; i < 255; i++)
    {
        whiteCount += gray_hist.at<float>(i);
    }
    std::vector<float> arrayHis;
    if (gray_hist.isContinuous()) {
        // array.assign((float*)mat.datastart, (float*)mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
        arrayHis.assign((float*)gray_hist.data,
            (float*)gray_hist.data
            + gray_hist.total()
            * gray_hist.channels()
        );
    }
    else {
        for (int i = 0; i < gray_hist.rows; ++i) {
            arrayHis.insert(arrayHis.end(),
                gray_hist.ptr<float>(i),
                gray_hist.ptr<float>(i)
                + gray_hist.cols
                * gray_hist.channels()
            );
        }
    }
    cv::Mat showGr = showGrahf(arrayHis, { {255, 0, 0}, {255, 255, 0}, {0, 255, 0}, {0, 255, 255} }, cv::Mat(), { low_lvl, treshhold, hight_lvl, 255 });

    cv::imshow("Grahf " + std::to_string(id), showGr);
    //cv::imshow("Code N "+ std::to_string(id), inputImg);
    static int countN = 0;
    //cv::imwrite("./codes/Code_" + std::to_string(id) + "_" + std::to_string(countFrames) + ".jpg", inputImg);
    //cv::imwrite("./codes/Grahf_" + std::to_string(id) + "_" + std::to_string(countFrames) + ".jpg", showGr);
    auto copyPoints = p0;
    if (countN == 2750 or countN == 0)
    {
        std::cout << std::to_string(countN) << ") " << blackCount / whiteCount << "\n";
    }
    countN++;
    //cv::waitKey(0);
    return blackCount / whiteCount;
}

cv::Mat Robot::showGrahf(std::vector<float> inputVec, std::vector<cv::Scalar> color, cv::Mat inputMat, std::vector<int> fromTo)
{
    cv::Size sizeWindow{550,300};

    bool newGrahf = false;

    cv::Size comBlockSize{ 40,40 };

    cv::Size maxSize = cv::Size();
    
    maxSize.width = (int)inputVec.size();
    maxSize.height = 0;

    for (int i = 0; i < inputVec.size(); i++)
    {
        if (maxSize.height < inputVec[i])
        {
            maxSize.height = inputVec[i];
        }
    }
    maxSize.height = (int)maxSize.height;

    cv::Size2f dSize = cv::Size2f();

    dSize.height = (float)sizeWindow.height / maxSize.height;
    dSize.width  = (float)sizeWindow.width  / maxSize.width;

    sizeWindow += comBlockSize;

    if (inputMat.cols == 0 and inputMat.rows == 0)
    {
        newGrahf = true;
    }
    if (newGrahf)
    {
        inputMat = cv::Mat(sizeWindow + comBlockSize, CV_32FC3, cv::Scalar(0, 0, 0));
    }

    // Main
    int lastPoint = 0;
    for (int colN = 0; colN < color.size(); colN++)
    {
        for (int i = lastPoint; i < fromTo[colN]; i++)
        {
            cv::line(
                inputMat,
                cv::Point2f(
                    i * dSize.width + comBlockSize.width,
                    (sizeWindow.height - dSize.height * inputVec[i]) - comBlockSize.height * 0),
                cv::Point2f(
                    (i + 1) * dSize.width + comBlockSize.width,
                    (sizeWindow.height - dSize.height * inputVec[(int)i + 1]) - comBlockSize.height * 0),
                color[colN],
                2
            );
        }
        lastPoint = fromTo[colN];
    }
    if (newGrahf)
    {
        // Bottom
        cv::line(
            inputMat,
            cv::Point(comBlockSize.width, sizeWindow.height - comBlockSize.height * 0),
            cv::Point(sizeWindow.width, sizeWindow.height - comBlockSize.height * 0),
            cv::Scalar(255, 255, 255),
            2
        );
        //Left
        cv::line(
            inputMat,
            cv::Point(comBlockSize.width, comBlockSize.height),
            cv::Point(comBlockSize.width, sizeWindow.height - comBlockSize.height * 0),
            cv::Scalar(255, 255, 255),
            2
        );
    }
    // Axis X
    cv::putText(
        inputMat, //target image
        "0", //text
        cv::Point(comBlockSize.width - 20, sizeWindow.height - comBlockSize.height * 0 + 20), //top-left position
        cv::FONT_HERSHEY_DUPLEX,
        0.5,
        cv::Scalar(255, 255, 255), //font color
        1);
    cv::putText(
        inputMat, //target image
        "255", //text
        cv::Point(sizeWindow.width - 40, sizeWindow.height - comBlockSize.height * 0 + 20), //top-left position
        cv::FONT_HERSHEY_DUPLEX,
        0.5,
        cv::Scalar(255, 255, 255), //font color
        1);

    // Axis Y
    cv::putText(
        inputMat, //target image
        std::to_string(maxSize.height), //text
        cv::Point(comBlockSize.width - 20, comBlockSize.height - 10), //top-left position
        cv::FONT_HERSHEY_DUPLEX,
        0.5,
        cv::Scalar(255, 255, 255), //font color
        1);

    return inputMat;
}