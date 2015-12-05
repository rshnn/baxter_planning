#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  std::cout << "imageCallback" << std::endl;

  cv_bridge::CvImagePtr camera_image;
  try
  {
    camera_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }


}


  
int main(int argc, char **argv)
{

  std::cout << "Image Listener initiated" << std::endl;

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/cameras/left_hand_camera/image", 1, imageCallback);
  ros::spin();
}