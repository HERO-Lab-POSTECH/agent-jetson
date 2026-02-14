#include <ros/package.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <time.h>

#include "hero_msgs/hero_agent_vision.h"
#include "hero_msgs/hero_command.h"
#include "hero_msgs/hero_xy_cont.h"

using namespace cv;
using namespace std;

char buff[100];

RNG rng(12345);

// for find hsv color
int threshold1 = 80;
int threshold2 = 70;
int threshold3 = 120;
Mat kernel = getStructuringElement(MORPH_RECT, Size(2 * 5 + 1, 2 * 5 + 1),
								   Point(10, 10));

// Vec3b lower_blue1, upper_blue1, lower_blue2, upper_blue2, lower_blue3,
// upper_blue3;

Mat img_color;

Vec3b lower_blue1 = Vec3b(25, threshold1, threshold1);
Vec3b upper_blue1 = Vec3b(25 + 10, 255, 255);
Vec3b lower_blue2 = Vec3b(25 - 10, threshold1, threshold1);
Vec3b upper_blue2 = Vec3b(25, 255, 255);
Vec3b lower_blue3 = Vec3b(25 - 10, threshold1, threshold1);
Vec3b upper_blue3 = Vec3b(25, 255, 255);

Vec3b lower_object = Vec3b(25 - 10, threshold1, threshold1);
Vec3b higher_object = Vec3b(25 + 10, 255, 255);


Vec3b lower_black = Vec3b(0, 0, 0);
Vec3b 		higher_black = Vec3b(255, 255, threshold2);
Vec3b 		lower_white = Vec3b(0, 0, 255 - threshold3);
Vec3b 		higher_white = Vec3b(255, 150, 255);

Vec3b lower_laser = Vec3b(0, 0, 255 - 30);
Vec3b higher_laser = Vec3b(255, 255, 255);

int target_command = 0;
int left_check = 0; //left_check:1   right_check:2

void msgCallback_command(const std_msgs::Int8::ConstPtr &msg)
{
	int Command = msg->data;

	if (Command == '6')
	{
		target_command = 1; // white target
	}
	else if (Command == '7')
	{
		target_command = 2; // black target
	}
	else if (Command == '8')
	{
		target_command = 3; // object target
	}
	else if (Command == '9')
	{
		target_command = 4; // black target left
	}
	else if (Command == '-')
	{
		target_command = 5; // check left
	}
	else if (Command == '=')
	{
		target_command = 6; // check right
	}
	else
	{
		target_command = 0; // none target
	}
}

class ImageConverter
{
public:
	ros::NodeHandle nh;

	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	cv::Mat img_copy, gray, thresh, thresh2;
	cv::Mat img_hsv;
	int size, size2;

	Moments con_m[100];
	int sum_con_m = 0;


	clock_t start_time;

	ros::Publisher pub_xy_agent =
		nh.advertise<hero_msgs::hero_xy_cont>("/hero_agent/xy", 100);

	ros::Publisher pub_vision_agent =
		nh.advertise<hero_msgs::hero_agent_vision>("/hero_agent/vision", 100);

	ros::Subscriber sub_command =
		nh.subscribe("/hero_agent/vision_command", 100, msgCallback_command);

	hero_msgs::hero_xy_cont hero_xy_msg;
	hero_msgs::hero_agent_vision hero_vision_msg;

	ImageConverter() : it_(nh)
	{
		// image_sub_ = it_.subscribe("/hero_agent/cameramain/camera_image2", 1,
		// &ImageConverter::imageCb, this);
		image_sub_ = it_.subscribe("/image_raw", 1, &ImageConverter::imageCb, this);

		image_pub_ = it_.advertise("/image_result", 1);
	}

	~ImageConverter() {}

	void imageCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv_ptr->image.copyTo(img_color);
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		hero_xy_msg.VALID = 0;

		hero_vision_msg.WHITE_VALID = 0;
		hero_vision_msg.BLACK_VALID = 0;
		hero_vision_msg.OBJECT_VALID = 0;
		hero_vision_msg.LASER_VALID = 0;

		///////////////////////////////////////////////////////////////////////////////
		// Draw an example circle on the video stream

		int width = img_color.cols;
		int height = img_color.rows;

		start_time = clock();

		cvtColor(img_color, img_hsv, COLOR_BGR2HSV);

		Mat img_object, img_black, img_white, img_laser;
		cout << " hsv :" << clock() - start_time;
		start_time = clock();

		inRange(img_hsv(Rect(width / 3, 0, width / 3, height)), lower_object,
				higher_object, img_object);
		erode(img_object, img_object,
			  getStructuringElement(MORPH_RECT, Size(10, 10)));
		dilate(img_object, img_object,
			   getStructuringElement(MORPH_RECT, Size(30, 30)));

		Moments m = moments(img_object, true);
		Point p2(width / 3 + m.m10 / m.m00, m.m01 / m.m00);
		// cout << "m.m00  " << m.m00 <<"  m.m10  " << m.m10 / m.m00 <<"  m.m01  "
		// << m.m01 / m.m00 << endl;
		if (m.m00 > 5000)
		{
			hero_vision_msg.OBJECT_VALID = 1;
			circle(cv_ptr->image, p2, 10, Scalar(255, 0, 0), -1);
			if (target_command == 3)
			{
				hero_xy_msg.TARGET_X = width / 3 + m.m10 / m.m00 - 15;
				hero_xy_msg.TARGET_Y = m.m01 / m.m00 - height / 4;

				hero_xy_msg.VALID = 1;
			}
		}
		cout << " object :" << clock() - start_time;
		start_time = clock();
		//////////////////////////////////////////////////////////////////////////////////////////////////////

		
		//////////////////////////////////////////////////////////////////////////////////////////////////////
		inRange(img_hsv(Rect(width / 6, 0, width * 4 / 6, height)), lower_black,
				higher_black, img_black);
		erode(img_black, img_black,
			  getStructuringElement(MORPH_RECT, Size(10, 10)));
		dilate(img_black, img_black,
			   getStructuringElement(MORPH_RECT, Size(30, 30)));

		vector<vector<Point> > contours2;
		vector<Vec4i> hierarchy2;
		findContours(img_black, contours2, hierarchy2, RETR_TREE,
					 CHAIN_APPROX_NONE);
		if (contours2.size() < 10)
		{
			sum_con_m = 0;
			for (size_t i = 0; i < contours2.size(); i++)
			{
				Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256),
									  rng.uniform(0, 256));
				con_m[i] = moments(contours2[i], true);
				if (con_m[i].m00 > 10000)
				{

					drawContours(cv_ptr->image(Rect(width / 6, 0, width * 4 / 6, height)),
								 contours2, (int)i, color, 2, LINE_8, hierarchy2, 0);
					int for_yaw[width * 4 / 6] = {0,};
					for (int kk = width / 6; kk < width * 5 / 6; kk++)
					{
						for (int j = 0; j < height; j++)
						{
							if (cv_ptr->image.at<Vec3b>(j, kk)[0] == (uchar)color[0] && cv_ptr->image.at<Vec3b>(j, kk)[1] == (uchar)color[1] && cv_ptr->image.at<Vec3b>(j, kk)[2] == (uchar)color[2])
							{
								for_yaw[kk-width / 6] = j;
								
								break;
							}
						}
					}
					int yaw_sum = 0;
					int plus_or_minus = 0;
					int end_point = 0;
					for(int kk=width * 1 / 6;kk<width * 3 / 6;kk++)
					{
						if(kk==width/6)
						{
							if(for_yaw[kk]<=for_yaw[kk+1])
						{
							plus_or_minus = 1;
						}
						else
						{
							plus_or_minus = 2;
						}
						}

						if(for_yaw[kk]<=for_yaw[kk+1]&&plus_or_minus==1)
						{
							yaw_sum += for_yaw[kk+1]-for_yaw[kk];
						}
						else if(for_yaw[kk]>for_yaw[kk+1]&&plus_or_minus==1)
						{
							end_point = kk;
							break;
						}
						else if(for_yaw[kk]>=for_yaw[kk+1]&&plus_or_minus==2)
						{
							yaw_sum += for_yaw[kk]-for_yaw[kk+1];
						}
						else if(for_yaw[kk]<for_yaw[kk+1]&&plus_or_minus==2)
						{
							end_point = kk;
							break;
						}
						end_point = kk;
					}
					

					line(cv_ptr->image, Point(width * 1 / 6+width * 1 / 6,for_yaw[width * 1 / 6]), Point(end_point+width * 1 / 6,for_yaw[width * 1 / 6]-yaw_sum), Scalar(0,0,255), 5);
					
				hero_vision_msg.FOR_YAW = yaw_sum;
				}
				// cout<<' ' << i<<' ' << con_m[i].m00 ;
				sum_con_m += con_m[i].m00;
			}

			// cout<<endl;
			int count_black = 0;
			

			Point p[10];
			int target_i[10];

			for (size_t i = 0; i < contours2.size(); i++)
			{//sum_con_m / contours2.size() - 5000 < con_m[i].m00 &&
				if (
					con_m[i].m00 > 20000)
				{
					target_i[count_black] = i;
					Point p(width / 6 + con_m[i].m10 / con_m[i].m00,
							con_m[i].m01 / con_m[i].m00);

					circle(cv_ptr->image, p, 10, Scalar(0, 255, 255), -1);
					count_black++;

					
				}
			}

			if(count_black==1)
			{
				if (target_command == 4)
					{
						hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[0]].m10 / con_m[target_i[0]].m00 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[0]].m01 / con_m[target_i[0]].m00;

						hero_xy_msg.VALID = 1;
					}
					else if (target_command == 2)
					{
						hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[0]].m10 / con_m[target_i[0]].m00 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[0]].m01 / con_m[target_i[0]].m00;

						hero_xy_msg.VALID = 1;
					}
					else if (target_command == 5)
					{
						hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[0]].m10 / con_m[target_i[0]].m00 - width * 7 / 24 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[0]].m01 / con_m[target_i[0]].m00;

						hero_xy_msg.VALID = 1;
					}
					else if (target_command == 6 )
					{
						hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[0]].m10 / con_m[target_i[0]].m00 + width * 7 / 24 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[0]].m01 / con_m[target_i[0]].m00;

						hero_xy_msg.VALID = 1;
					}
			}
			else if(count_black>=2)
			{
				if (target_command == 4)
					{
						if(con_m[target_i[0]].m10 / con_m[target_i[0]].m00>con_m[target_i[1]].m10 / con_m[target_i[1]].m00)
						{
						hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[1]].m10 / con_m[target_i[1]].m00 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[1]].m01 / con_m[target_i[1]].m00;
						}
						else
						{
							hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[0]].m10 / con_m[target_i[0]].m00 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[0]].m01 / con_m[target_i[0]].m00;
						}

						hero_xy_msg.VALID = 1;
					}
					else if (target_command == 2)
					{
						if(con_m[target_i[0]].m10 / con_m[target_i[0]].m00<con_m[target_i[1]].m10 / con_m[target_i[1]].m00)
						{
						hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[1]].m10 / con_m[target_i[1]].m00 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[1]].m01 / con_m[target_i[1]].m00;
						}
						else
						{
							hero_xy_msg.TARGET_X = width / 6 + con_m[target_i[0]].m10 / con_m[target_i[0]].m00 + width * 1 / 24;
						hero_xy_msg.TARGET_Y = con_m[target_i[0]].m01 / con_m[target_i[0]].m00;
						}

						hero_xy_msg.VALID = 1;
					}
			}
			else if(count_black==3)
			{

			}


			hero_vision_msg.BLACK_VALID = count_black;
		}
		cout << " black :" << clock() - start_time;
		start_time = clock();
		//////////////////////////////////////////////////////////////////////////////////////////////////////
		inRange(img_hsv, lower_white, higher_white, img_white);
		erode(img_white, img_white,
			  getStructuringElement(MORPH_RECT, Size(10, 10)));
		dilate(img_white, img_white,
			   getStructuringElement(MORPH_RECT, Size(30, 30)));
		m = moments(img_white, true);
		// cout<<m.m00<<endl;
		Point p(m.m10 / m.m00, m.m01 / m.m00);
		// cout << "m.m00  " << m.m00 <<"  m.m10  " << m.m10 / m.m00 <<"  m.m01  "
		// << m.m01 / m.m00 << endl;
		if (m.m00 > 50000)
		{
			hero_vision_msg.WHITE_VALID = 1;
			circle(cv_ptr->image, p, 10, Scalar(0, 0, 255), -1);
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////
		cout << " white :" << clock() - start_time;
		start_time = clock();

		inRange(img_hsv, lower_laser, higher_laser, img_laser);

		int sum[width] = {
			0,
		};
		int all_sum = 0;
		int all_count = 0;
		int count = 0;
		for (int i = width * 1 / 2; i < width * 2 / 3; i++)
		{
			count = 0;
			for (int j = 0; j < height; j++)
			{
				if (img_laser.at<uchar>(j, i) != 0)
				{
					sum[i] = sum[i] + j;
					count++;
				}
			}
			if (count != 0)
			{
				sum[i] = sum[i] / count;
				all_count++;
				all_sum += sum[i];
				// cout<<i<<' '<<sum[i]<<' '<<endl;
				circle(cv_ptr->image, Point(i, sum[i]), 1, Scalar(255, 0, 255), -1);
			}
		}
		int big_laser = 0;
		int small_laser = 0;
		int small_laser_count = 0;
		if (all_count != 0)
		{
			hero_vision_msg.LASER_VALID = 1;
			sort(sum + width *1 / 2, sum + width * 2 / 3);

			for (int i = width * 1 / 2; i < width * 2 / 3; i++)
			{
				if (sum[i] != 0)
				{
					small_laser_count++;
					small_laser += sum[i];

					if (small_laser_count == 10)
					{
						break;
					}
				}
			}
			if (small_laser_count != 0)
				small_laser = small_laser / small_laser_count;

			for (int i = width * 2 / 3 - 10; i < width * 2 / 3; i++)
			{
				big_laser += sum[i];
			}
			big_laser = big_laser / 10;

			for (int i = width * 1 / 2; i < width * 2 / 3; i++)
			{
				if (small_laser != 0)
					circle(cv_ptr->image, Point(i, small_laser), 1, Scalar(0, 255, 0),
						   -1);
				if (big_laser != 0)
					circle(cv_ptr->image, Point(i, big_laser), 1, Scalar(0, 255, 255),
						   -1);
			}
		}
		cout << " laser :" << clock() - start_time << endl;
		hero_vision_msg.HIGH_LASER = big_laser;
		hero_vision_msg.LOW_LASER = small_laser;
		cout << small_laser << ' ' << big_laser << endl;
		//////////////////////////////////////////////////////////////////////

		/*
    ////////////////////////////////////////////////
    imshow("Color", cv_ptr->image);
    imshow("img_object", img_object);

    imshow("img_black", img_black);
    imshow("img_white", img_white);

    imshow("img_laser", img_laser);
    // Update GUI Window
    ////////////////////////////////////////////////////////////////////////////
    int key = cv::waitKey(10);
    if (key == 27)
            return;

*/
		if (target_command != 0)
		{
			pub_xy_agent.publish(hero_xy_msg);
			cout << "pub" << endl;
		}
		image_pub_.publish(cv_ptr->toImageMsg());
		pub_vision_agent.publish(hero_vision_msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_opencv_manipulation");

	ImageConverter ic;

	ros::spin();

	return 0;
}
