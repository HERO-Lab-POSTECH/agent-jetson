#include <ros/package.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <termios.h>

#include "ros_opencv_ipcam_qr/hero_ipcam_qr_msg.h"

using namespace cv;
using namespace std;

// for keyboard input
static struct termios initial_settings, new_settings;

static int peek_character = -1;

void init_keyboard();
void close_keyboard();
int _kbhit();
int _getch();
int _putch(int c);
///////////////////////////////////

namespace
{
    const char *about =
        "Calibration using a ChArUco board\n"
        "  To capture a frame for calibration, press 'c',\n"
        "  If input comes from video, press any key for next frame\n"
        "  To finish capturing, press 'ESC' key and calibration starts.\n";
    const char *keys =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{cd       |       | Input file with custom dictionary }"
        "{@outfile |<none> | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       | false | Apply refind strategy }"
        "{zt       | false | Assume zero tangential distortion }"
        "{a        |       | Fix aspect ratio (fx/fy) to this value }"
        "{pc       | false | Fix the principal point at the center }"
        "{sc       | false | Show detected chessboard corners after calibration }";
}
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}



class ImageConverter
{
public:
    ros::NodeHandle nh;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    int squaresX = 5;
    int squaresY = 7;
    float squareLength = 40;
    float markerLength = 30;
    int dictionaryId = 10;
    string outputFile = "/home/nvidia/catkin_ws/output.txt";

    bool showChessboardCorners = false;

    int calibrationFlags = 0;
    float aspectRatio = 1;

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    bool refindStrategy = true;
    int camId = 0;

    int waitTime = 10;

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard =
        aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    // collect data from each frame
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< Mat > allImgs;
    Size imgSize;

    int do_calib = 0;

    char pc_input, qq;

    ImageConverter() : it_(nh)
    {
        // Subscrive to input video feed and publish output video feed
        // image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
        image_sub_ = it_.subscribe("/rtsp_camera_relay/image", 1, &ImageConverter::imageCb, this);

        image_pub_ = it_.advertise("/rtsp_camera_relay/qr_result_image", 1);

        // fs["camera_matrix"] >> cameraMatrix;
        // fs["distortion_coefficients"] >> distCoeffs;
    }

    ~ImageConverter() {}

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        waitTime = 10;

        Mat imageCopy;

        // cv_ptr->image
        vector<int> ids;
        vector<vector<Point2f> > corners, rejected;

        // detect markers
        aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if (refindStrategy)
            aruco::refineDetectedMarkers(cv_ptr->image, board, corners, ids, rejected);

        // interpolate charuco corners
        Mat currentCharucoCorners, currentCharucoIds;
        if (ids.size() > 0)
            aruco::interpolateCornersCharuco(corners, ids, cv_ptr->image, charucoboard, currentCharucoCorners,
                                             currentCharucoIds);

        // draw results
        if (ids.size() > 0)
            aruco::drawDetectedMarkers(cv_ptr->image, corners);

        if (currentCharucoCorners.total() > 0)
            aruco::drawDetectedCornersCharuco(cv_ptr->image, currentCharucoCorners, currentCharucoIds);

        putText(cv_ptr->image, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        image_pub_.publish(cv_ptr->toImageMsg());

        char key = (char)waitKey(waitTime);
        if (_kbhit())
        {

            int ch = _getch();

            
        if (ch == 'q')
        {
            do_calib = 1;
        }
        if (ch == 'c' && ids.size() > 0)
        {
            cout << "Frame captured" << endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            allImgs.push_back(cv_ptr->image);
            imgSize = cv_ptr->image.size();
        }
        }


        if (do_calib == 1)
        {
            do_calib = 0;

            if (allIds.size() < 1)
            {
                cerr << "Not enough captures for calibration" << endl;
               
            }

            Mat cameraMatrix, distCoeffs;
            vector<Mat> rvecs, tvecs;
            double repError;

            if (calibrationFlags & CALIB_FIX_ASPECT_RATIO)
            {
                cameraMatrix = Mat::eye(3, 3, CV_64F);
                cameraMatrix.at<double>(0, 0) = aspectRatio;
            }

            // prepare data for calibration
            vector<vector<Point2f> > allCornersConcatenated;
            vector<int> allIdsConcatenated;
            vector<int> markerCounterPerFrame;
            markerCounterPerFrame.reserve(allCorners.size());
            for (unsigned int i = 0; i < allCorners.size(); i++)
            {
                markerCounterPerFrame.push_back((int)allCorners[i].size());
                for (unsigned int j = 0; j < allCorners[i].size(); j++)
                {
                    allCornersConcatenated.push_back(allCorners[i][j]);
                    allIdsConcatenated.push_back(allIds[i][j]);
                }
            }

            // calibrate camera using aruco markers
            double arucoRepErr;
            arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                                      markerCounterPerFrame, board, imgSize, cameraMatrix,
                                                      distCoeffs, noArray(), noArray(), calibrationFlags);

            // prepare data for charuco calibration
            int nFrames = (int)allCorners.size();
            vector<Mat> allCharucoCorners;
            vector<Mat> allCharucoIds;
            vector<Mat> filteredImages;
            allCharucoCorners.reserve(nFrames);
            allCharucoIds.reserve(nFrames);

            for (int i = 0; i < nFrames; i++)
            {
                cout<<i<<endl;
                // interpolate using camera parameters
                Mat currentCharucoCorners, currentCharucoIds;
                aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                                 currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                                 distCoeffs);

                allCharucoCorners.push_back(currentCharucoCorners);
                allCharucoIds.push_back(currentCharucoIds);
                filteredImages.push_back(allImgs[i]);
            }

            if (allCharucoCorners.size() < 4)
            {
                cerr << "Not enough corners for calibration" << endl;
                
            }
            cout<<"calibrateCameraCharuco"<<endl;
            // calibrate camera using charuco
            repError =
                aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imgSize,
                                              cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

            cout<<"saveCameraParams"<<endl;
            bool saveOk = saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
                                           cameraMatrix, distCoeffs, repError);
            if (!saveOk)
            {
                cerr << "Cannot save output file" << endl;
               
            }

            cout << "Rep Error: " << repError << endl;
            cout << "Rep Error Aruco: " << arucoRepErr << endl;
            cout << "Calibration saved to " << outputFile << endl;

        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_opencv_qr_calibration");
    cout << "Start" << endl;
    // namedWindow("out", 1);
    //  ros::NodeHandle nh;

    ImageConverter ic;

    init_keyboard();

    ros::spin();
    close_keyboard();
    return 0;
}


void init_keyboard()
{
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}

void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}

int _kbhit()
{
    unsigned char ch;
    int nread;

    if (peek_character != -1)
        return 1;

    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);

    if (nread == 1)
    {
        peek_character = ch;
        return 1;
    }

    return 0;
}

int _getch()
{
    char ch;

    if (peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;

        return ch;
    }
    read(0, &ch, 1);

    return ch;
}

int _putch(int c)
{
    putchar(c);
    fflush(stdout);

    return c;
}