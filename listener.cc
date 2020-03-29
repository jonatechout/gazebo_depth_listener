
#include <gazebo/msgs/images_stamped.pb.h>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include <iostream>

void cb(ConstImageStampedPtr& msg) {
  const double DEG2RAD = 3.1416 / 180.0;

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

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const float depth = data[w * y + x];
      if (0.4 < depth && depth < 5.0) {
        const double unitNorm = sqrt((x - w / 2) * (x - w / 2) +
                                     (y - h / 2) * (y - h / 2) + 628 * 628);

        pcl::PointXYZRGBA point;
        point.x = depth * (x - w / 2) / unitNorm;
        point.y = depth * (y - h / 2) / unitNorm;
        point.z = depth * 628 / unitNorm;
        point.r = 255;
        point.g = 0;
        point.b = 0;
        point.a = 0;
        cloud->push_back(point);
      }
    }
  }

  pcl::io::savePCDFileBinary("p_cloud_binary.pcd", *cloud);

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
