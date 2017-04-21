#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <config_ros2cv/pointsConfig.h>

using namespace std;
using namespace cv;

int xx1,xx2,yy1,yy2;
cv::Mat outImg;

image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

cv::Mat points_run(cv::Mat srcImg, int xx1, int xx2, int yy1, int yy2){

    if (srcImg.rows > 60 && srcImg.cols > 60)
    {
      cv::rectangle(srcImg, cv::Point(xx1, yy1), cv::Point(xx2, yy2), CV_RGB(255,0,0));
    }
    return srcImg;
}

//callback function for dynamic reconfigure
void callback(config_ros2cv::pointsConfig &config, uint32_t level) {
ROS_INFO("Reconfigure Request: %d %d %d %d",
          config.x1, config.x2,
          config.y1, config.y2
        );
      xx1 = config.x1;
      xx2 = config.x2;
      yy1 = config.y1;
      yy2 = config.y2;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());  return;
	}
	waitKey(2);
	imshow("src-camera", cv_ptr->image);

	outImg = points_run(cv_ptr->image, xx1, xx2, yy1, yy2);
	imshow("points", outImg);

	sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", outImg).toImageMsg();
	image_pub_.publish(msg_out);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros2cv2");
	ros::NodeHandle nh_;
        image_transport::ImageTransport it_(nh_);

	image_sub_ = it_.subscribe("/image_raw", 1, &imageCallback);
	image_pub_ = it_.advertise("/ros2cv2/points", 1);

    dynamic_reconfigure::Server<config_ros2cv::pointsConfig> server;
    dynamic_reconfigure::Server<config_ros2cv::pointsConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
	return 0;
}
