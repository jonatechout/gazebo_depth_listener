
#include <gazebo/msgs/images_stamped.pb.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>


void cb(ConstImageStampedPtr& msg) {
  int w = msg->image().width();
  int h = msg->image().height();

  float* data = (float*)(msg->image().data().c_str());
  cv::Mat resImage = cv::Mat::zeros(cv::Size(w, h), CV_32F);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      resImage.at<float>(y, x) = data[w * y + x];
    }
  }

  cv::imwrite("test.png", resImage);
  std::cout << "saved" << std::endl;
}

int main(int _argc, char** _argv) {
  gazebo::client::setup(_argc, _argv);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  gazebo::transport::SubscriberPtr sub =
      node->Subscribe("~/intel_realsense_r200/link/camera/image", cb);

  while (true) {
    gazebo::common::Time::MSleep(10);
  }

  gazebo::client::shutdown();
}
