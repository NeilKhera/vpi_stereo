#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

#include "Disparity.h"

using namespace message_filters;
using namespace sensor_msgs;
using namespace cv;
using namespace std;

image_transport::Publisher pub;

void callback(const ImageConstPtr& left, const ImageConstPtr& right) {
  cv_bridge::CvImagePtr cv_left;
  try {
    cv_left = cv_bridge::toCvCopy(left);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImagePtr cv_right;
  try {
    cv_right = cv_bridge::toCvCopy(right);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  cv::Mat disparity = Disparity::compute_disparity(cv_left->image, cv_right->image);
  //compute_stereo(cv_left->image,cv_right->image);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", disparity).toImageMsg();
  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // depth image publisher
  pub = it.advertise("/camera/depth/image", 1);

  //left and right rectified images subscriber
  message_filters::Subscriber<Image> left_sub(nh, "camera/left/image_rect", 1);
  message_filters::Subscriber<Image> right_sub(nh, "camera/right/image_rect", 1);

  //time syncronizer to publish 2 images in the same callback function
  TimeSynchronizer<Image, Image> sync(left_sub, right_sub, 1);

  //call calback each time a new message arrives
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}
