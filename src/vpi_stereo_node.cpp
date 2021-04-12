#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "Disparity.h"

using namespace message_filters;
using namespace sensor_msgs;
//using namespace cv;
//using namespace std;

image_transport::Publisher pub;

void callback(const ImageConstPtr& left, const ImageConstPtr& right) {
  cv_bridge::CvImagePtr cv_left;
  try {
    cv_left = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::MONO16);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImagePtr cv_right;
  try {
    cv_right = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::MONO16);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat disparity = Disparity::compute_disparity(cv_left->image, cv_right->image);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, disparity).toImageMsg();
  pub.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vpi_stereo_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  pub = it.advertise("/camera/depth/image", 1);

  message_filters::Subscriber<Image> left_sub(nh, "/stereo/left/image_raw", 1);
  message_filters::Subscriber<Image> right_sub(nh, "/stereo/right/image_raw", 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_sub, right_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}
