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

#include <cmath>

using namespace cv;
using namespace std;

char buf1[100];
int ccount = 0;

class ImageConverter
{
public:
    ros::NodeHandle nh;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ofstream fout;

    ImageConverter() : it_(nh)
    {

        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);
        // image_sub_ = it_.subscribe("/rtsp_camera_relay/image", 1, &ImageConverter::imageCb, this);
        // image_sub_ = it_.subscribe("/rtsp_camera_relay/qr_result_image", 1, &ImageConverter::imageCb, this);

        // fs["camera_matrix"] >> cameraMatrix;
        // fs["distortion_coefficients"] >> distCoeffs;
    }

    ~ImageConverter()
    {
    }
    int key_save = 0;
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
        cout << key_save << endl;
        ccount++;
        sprintf(buf1, "/home/nvidia/catkin_ws/20220812_object/%05d.jpg", ccount);
        // cv::imshow("out", cv_ptr->image);
        if (ccount % 30 == 1) 
        {

            imwrite(buf1, cv_ptr->image);
            cout << buf1 << endl;
        }
        // fout<<ccount<<' '<<ipcam_qr_x<<' '<<ipcam_qr_y<<' '<<ipcam_qr_z<<' '<<ipcam_qr_yaw<<' '<<ipcam_qr_valid<<' '<<ag_position.x<<' '<<ag_position.y<<' '<<ag_position.z<<' '<<cy_position.x<<' '<<cy_position.y<<' '<<cy_position.z<<' '<<usbl_x<<' '<<usbl_y<<' '<<usbl_z<<' '<<object_x<<' '<<object_y<<' '<<endl;
        // cout<<ccount<<' '<<ipcam_qr_x<<' '<<ipcam_qr_y<<' '<<ipcam_qr_z<<' '<<ipcam_qr_yaw<<' '<<ipcam_qr_valid<<' '<<ag_position.x<<' '<<ag_position.y<<' '<<ag_position.z<<' '<<cy_position.x<<' '<<cy_position.y<<' '<<cy_position.z<<' '<<usbl_x<<' '<<usbl_y<<' '<<usbl_z<<' '<<object_x<<' '<<object_y<<' '<<endl;

        // fout<<ccount<<' '<<object_x<<' '<<object_y<<endl;
        // cout<<ccount<<' '<<object_x<<' '<<object_y<<endl;
        int key = cv::waitKey(10);
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_opencv_save");
    // namedWindow("out", 1);
    //  ros::NodeHandle nh;
    ImageConverter ic;

    ros::spin();

    return 0;
}
