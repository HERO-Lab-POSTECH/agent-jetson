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
#include <chrono>

#include "ros_opencv_ipcam_qr/hero_ipcam_qr_msg.h"

using namespace cv;
using namespace std;

char buf1[100];

struct tm tstruct;
// Mat cameraMatrix, distCoeffs;

float qr_save[7] = {
    0,
};

bool mouse_is_pressing = false;
int start_x, start_y, end_x, end_y;
int step = 0;
Mat cv_ptr;
cv::Mat imageCopy;
Rect roi;

Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);
Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);

int squaresX = 5;        //인쇄한 보드의 가로방향 마커 갯수
int squaresY = 7;        //인쇄한 보드의 세로방향 마커 갯수
float squareLength = 38; //검은색 테두리 포함한 정사각형의 한변 길이, mm단위로 입력
float markerLength = 19; //인쇄물에서의 마커 한변의 길이, mm단위로 입력
int dictionaryId = 10;   // DICT_6X6_250=10
string outputFile = "output.txt";

int calibrationFlags = 0;
float aspectRatio = 1;

Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

bool refindStrategy = true;
int camId = 0;

float rvecs_yaw[10] = {
    0,
};
float average_yaw = 0;
float qr_x = 0;
float qr_y = 0;
float qr_z = 0;
int yaw_count = 0;
int qr_valid = 0;

Ptr<aruco::Dictionary> dictionary =
    aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

// create charuco board object
Ptr<aruco::CharucoBoard> charucoboard =
    aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

// collect data from each frame
vector<vector<vector<Point2f>>> allCorners;
vector<vector<int>> allIds;
vector<Mat> allImgs;
Size imgSize;

int frame_count = 0;

class ImageConverter
{
public:
    ros::NodeHandle nh;

    ros::Publisher hero_ipcam_qr_pub =
        nh.advertise<ros_opencv_ipcam_qr::hero_ipcam_qr_msg>("/hero_ipcam_qr_msg", 100);
    ros_opencv_ipcam_qr::hero_ipcam_qr_msg hero_msg;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    chrono::steady_clock::time_point total_end;
    chrono::steady_clock::time_point total_pre;

    ImageConverter() : it_(nh)
    {

        cameraMatrix = (Mat1d(3, 3) << 654.27246716526, 0, 276.4639048547914, 0, 1281.931718850296, 353.6612948734696, 0, 0, 1);
        distCoeffs = (Mat1d(1, 5) << -0.4569663771471645, 0.322393109110365, 0.01342316947152003, -0.05619752541101617, -0.1287580146172017);

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

        ///////////////////////////////////////////////////////////////////////////////
        // Draw an example circle on the video stream
        // cout<<"do Sum"<<endl;
        //카메라 컬리브레이션으로 얻은 카메라 정보를 파일에서 읽어옴
        Mat ccamMatrix, ddistCoeffs;
        FileStorage fs("/home/nvidia/catkin_ws/output_final.txt", FileStorage::READ);
        if (!fs.isOpened())
            return;
        fs["camera_matrix"] >> ccamMatrix;
        fs["distortion_coefficients"] >> ddistCoeffs;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        std::vector<int> idss;

        std::vector<std::vector<cv::Point2f>> ccorners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary, ccorners, idss);
        // if at least one marker detected
        qr_valid = 0;
        if (idss.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(cv_ptr->image, ccorners, idss);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(ccorners, 0.08, ccamMatrix, ddistCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for (int i = 0; i < idss.size(); i++)
            {

                if (yaw_count >= 10)
                {
                    yaw_count = 0;
                }
                if (rvecs[i][1] > 0)
                {
                    rvecs[i][2] = -rvecs[i][2];
                }

                if (rvecs[i][1] > 0 && rvecs[i][0] > 0)
                {
                    rvecs[i][2] = -rvecs[i][2];
                }
                if (rvecs[i][1] < 0 && rvecs[i][0] < 0)
                {
                    rvecs[i][2] = -rvecs[i][2];
                }
                rvecs_yaw[yaw_count] = rvecs[i][2];

                average_yaw = rvecs_yaw[0] + rvecs_yaw[1] + rvecs_yaw[2] + rvecs_yaw[3] + rvecs_yaw[4] +
                              rvecs_yaw[5] + rvecs_yaw[6] + rvecs_yaw[7] + rvecs_yaw[8] + rvecs_yaw[9];
                average_yaw = average_yaw / 10;

                qr_valid = 1;
                qr_x = tvecs[i][0];
                qr_y = tvecs[i][1];
                qr_z = tvecs[i][2];
                yaw_count++;
                sprintf(buf1, "T : {%f", tvecs[i][0]);
                putText(cv_ptr->image, buf1, (ccorners[i][0] + ccorners[i][1] + ccorners[i][2] + ccorners[i][3]) / 4 - Point_<float>(0, 60), 0, 0.8, Scalar(0, 0, 255),
                        2, 16);
                sprintf(buf1, "     %f", tvecs[i][1]);
                putText(cv_ptr->image, buf1, (ccorners[i][0] + ccorners[i][1] + ccorners[i][2] + ccorners[i][3]) / 4 - Point_<float>(0, 30), 0, 0.8, Scalar(0, 0, 255),
                        2, 16);
                sprintf(buf1, "     %f}", tvecs[i][2]);
                putText(cv_ptr->image, buf1, (ccorners[i][0] + ccorners[i][1] + ccorners[i][2] + ccorners[i][3]) / 4, 0, 0.8, Scalar(0, 0, 255),
                        2, 16);
                sprintf(buf1, "R : {%f", rvecs[i][0]);
                putText(cv_ptr->image, buf1, (ccorners[i][0] + ccorners[i][1] + ccorners[i][2] + ccorners[i][3]) / 4 + Point_<float>(0, 30), 0, 0.8, Scalar(0, 0, 255),
                        2, 16);
                sprintf(buf1, "     %f", rvecs[i][1]);
                putText(cv_ptr->image, buf1, (ccorners[i][0] + ccorners[i][1] + ccorners[i][2] + ccorners[i][3]) / 4 + Point_<float>(0, 60), 0, 0.8, Scalar(0, 0, 255),
                        2, 16);
                sprintf(buf1, "     %f}", rvecs[i][2]);
                putText(cv_ptr->image, buf1, (ccorners[i][0] + ccorners[i][1] + ccorners[i][2] + ccorners[i][3]) / 4 + Point_<float>(0, 90), 0, 0.8, Scalar(0, 0, 255),
                        2, 16);
                cv::aruco::drawAxis(cv_ptr->image, ccamMatrix, ddistCoeffs, rvecs[i], tvecs[i], 0.1);
            }
        }
        hero_msg.T_X = qr_x;
        hero_msg.T_Y = qr_y;
        hero_msg.T_Z = qr_z;
        hero_msg.T_YAW = average_yaw;
        hero_msg.T_valid = qr_valid;
        hero_ipcam_qr_pub.publish(hero_msg);

        total_end = chrono::steady_clock::now();
        float total_fps = chrono::duration_cast<chrono::milliseconds>(total_end - total_pre).count();
        frame_count++;

        if (total_fps > 1000)
        {
            total_fps = (float)frame_count / total_fps * 1000.0;
            cout << "fps " << total_fps << std::endl;
            total_pre = total_end;
            frame_count = 0;
        }

        image_pub_.publish(cv_ptr->toImageMsg());

        // cv::imshow("out", cv_ptr->image);
        // imshow("out", imageCopy);
        // int key = cv::waitKey(10);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_opencv_ipcam_qr");
    cout << "Start" << endl;
    // namedWindow("out", 1);
    //  ros::NodeHandle nh;

    ImageConverter ic;

    ros::spin();

    return 0;
}
