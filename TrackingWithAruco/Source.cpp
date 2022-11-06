// This code is written by Sunita Nayak at BigVision LLC. It is based on the OpenCV project. It is subject to the license terms in the LICENSE file found in this distribution and at http://opencv.org/license.html

// Usage example:   ./augmented_reality_with_aruco.out --image=test.jpg
//                  ./augmented_reality_with_aruco.out --video=test.mp4
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "robot.h"



const char* keys =
"{help h usage ? | | Usage examples: \n\t\t./augmented_reality_with_aruco.out --image=test.jpg \n\t\t./augmented_reality_with_aruco.out --video=test.mp4}"
"{image i        |<none>| input image   }"
"{video v       |<none>| input video   }"
;
using namespace cv;
using namespace aruco;
using namespace std;


int countFrames = 0;
Rect2d convertCornersToRect(vector<Point2f> corners);


int main(int argc, char** argv)
{
    CommandLineParser parser(argc, argv, keys);
    parser.about("Use this script to do Augmented Reality using Aruco markers in OpenCV.");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    // Open a video file or an image file or a camera stream.
    string str, outputFile;
    VideoCapture cap;
    VideoWriter video;

    VideoWriter videoid0;
    VideoWriter videoid1;
    VideoWriter videoid2;
    VideoWriter videoid3;
    VideoWriter videoid4;

    Mat frame, blob;
    
    //Mat im_src = imread("new_scenery.jpg");

    outputFile = "ar_out_cpp.avi";
    try {
        
        outputFile = "ar_out_cpp.avi";
        if (parser.has("image"))
        {
            // Open the image file
            str = parser.get<String>("image");
            ifstream ifile(str);
            if (!ifile) throw("error");
            cap.open(str);
            str.replace(str.end()-4, str.end(), "_ar_out_cpp.jpg");
            outputFile = str;
        }
        else if (parser.has("video"))
        {
            // Open the video file
            str = parser.get<String>("video");
            ifstream ifile(str);
            if (!ifile) throw("error");
            cap.open(str);
            str.replace(str.end()-4, str.end(), "_ar_out_cpp");
            outputFile = str;
        }
        // Open the webcaom
        else cap.open(parser.get<int>("device"));
        
    }
    catch(...) {
        cout << "Could not open the input image/video stream" << endl;
        return 0;
    }
    
    // Get the video writer initialized to save the output video
    if (!parser.has("image")) {
        video.open(outputFile + ".avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT)));
    }
    video.open(outputFile + ".avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT)));
    videoid0.open(outputFile + "_0.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(300, 300));
    videoid1.open(outputFile + "_1.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(300, 300));
    videoid2.open(outputFile + "_2.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(300, 300));
    videoid3.open(outputFile + "_3.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(300, 300));
    videoid4.open(outputFile + "_4.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 28, Size(300, 300));

    // Create a window
    static const string kWinName = "Augmented Reality using Aruco markers in OpenCV";
    static const string inWinName = "inWinName";
    namedWindow(kWinName, WINDOW_NORMAL);
    //namedWindow(inWinName, WINDOW_NORMAL);
    // Process frames.
    std::vector<Robot> robots;
    auto time_point_1 = std::chrono::high_resolution_clock::now();
    auto time_point_2 = std::chrono::high_resolution_clock::now();
    bool first = true;
    int countFrames = 0;
    while (waitKey(1) < 0)
    {
        // get frame from the video
        cap >> frame;
        cv::Mat outputImage = frame.clone();
        if (frame.empty()) {
            cout << "Done processing !!!" << endl;
            cout << "Output file is stored as " << outputFile << endl;
            waitKey(3000);
            break;
        }
        //imshow(inWinName, frame);

        vector<int> markerIds;
        // Load the dictionary that was used to generate the markers.
        Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_50);
        // Declare the vectors that would contain the detected marker corners and the rejected marker candidates
        vector<vector<Point2f>> markerCorners, rejectedCandidates;
        // Initialize the detector parameters using default values
        Ptr<DetectorParameters> parameters = DetectorParameters::create();
        // Detect the markers in the image

        if (countFrames == 0)
        {
            detectMarkers(frame, dictionary, markerCorners, markerIds);
        }
        countFrames++;
        for (int i_robot = 0; i_robot < robots.size(); i_robot++)
        {
            cv::Rect tresh = robots[i_robot].trackRobot(frame, outputImage);
        }

        for (int i_id = 0; i_id < markerIds.size(); i_id++)
        {
            bool newRobot = true;
            for (int i_robot = 0; i_robot < robots.size(); i_robot++)
            {
                if (robots[i_robot].getId() == markerIds[i_id])
                {
                    newRobot = false;
                    robots[i_robot].initPosition(convertCornersToRect(markerCorners[i_id]) , frame, markerCorners[i_id]);
                    
                }
            }
            if (newRobot)
            {
                robots.push_back(Robot(markerIds[i_id], 1, convertCornersToRect(markerCorners[i_id]), frame, markerCorners[i_id]));
            }
        }
        vector<vector<Point2f>> markerCornersRobots;
        for (int i_robot = 0; i_robot < robots.size(); i_robot++)
        {
            if (robots[i_robot].ColorDetect == cv::Scalar(0, 0, 255))
            {
                robots.erase(robots.begin() + i_robot);
            }
            else
            {

                vector<Point2f> coordAngle = robots[i_robot].p0;
                markerCornersRobots.push_back(coordAngle);

                cv::Rect last_position = robots[i_robot].getPosition();

                //rectangle(outputImage, last_position, robots[i_robot].ColorDetect, 2, 1);
                for (int p_angle = 0; p_angle < coordAngle.size(); p_angle++)
                {
                    line(outputImage, coordAngle[p_angle], coordAngle[(p_angle + 1) % coordAngle.size()], robots[i_robot].ColorDetect, 2);
                }

                cv::Mat outROI = cv::Mat(frame, robots[i_robot].getPosition());

                cv::resize(outROI, outROI, cv::Size(300,300));
                switch (robots[i_robot].getId())
                {
                case 0:
                    videoid0.write(outROI);
                    break;
                case 1:
                    videoid1.write(outROI);
                    break;
                case 2:
                    videoid2.write(outROI);
                    break;
                case 3:
                    videoid3.write(outROI);
                    break;
                case 4:
                    videoid4.write(outROI);
                    break;
                }
            }
        }

        std::vector< cv::Vec3d > tvecs = { 0 };
        std::vector< cv::Vec3d > rvecs = { 0 };
        float cm[9] = { 1.0100000e+03, 0.00000000e+00, 0.64000000e+03, 0.00000000e+00, 1.00000000e+03, 0.36000000e+03, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00 };
        float dc[5] = { 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00 };

        cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cm);
        //cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
        cv::Mat distCoeffs = cv::Mat(5, 1, CV_32FC1, dc);

        cv::aruco::estimatePoseSingleMarkers(markerCornersRobots, 0.01, cameraMatrix, distCoeffs, rvecs, tvecs);

        for (int i_robot = 0; i_robot < robots.size(); i_robot++)
        {
            cv::drawFrameAxes(outputImage, cameraMatrix, cv::Mat(), rvecs[i_robot], tvecs[i_robot], 0.01);
        }
        time_point_2 = std::chrono::high_resolution_clock::now();
        float time_for_detect = std::chrono::duration_cast<std::chrono::milliseconds>(time_point_2 - time_point_1).count();
        float FPS = 1000 / time_for_detect;
        cv::putText(outputImage, "FPS: " + std::to_string(FPS), cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
        time_point_1 = std::chrono::high_resolution_clock::now();

        //cv::imwrite("./codes/main/main_" + std::to_string(countFrames) + ".jpg", outputImage);
        if (parser.has("image")) imwrite(outputFile, outputImage);
        else video.write(outputImage);
        cv::imshow(kWinName, outputImage);
    }

    cap.release();
    if (!parser.has("image")) video.release();

    videoid0.release();
    videoid1.release();
    videoid2.release();
    videoid3.release();
    videoid4.release();

    return 0;
}

Rect2d convertCornersToRect(vector<Point2f> corners)
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

    Rect2d bbox = Rect2d{ min_x, min_y, widht, height };

    return bbox;
}
