/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>

class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  // camera1
  aruco::MarkerDetector mDetector1_;
  std::vector<aruco::Marker> markers1_;
  aruco::CameraParameters camParam1_;
  
  //camera2
  aruco::MarkerDetector mDetector2_;
  std::vector<aruco::Marker> markers2_;
  aruco::CameraParameters camParam2_;

  // node params
  double marker_size1_;
  bool useCamInfo1_;
  double marker_size2_;
  bool useCamInfo2_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it1_;
  image_transport::Subscriber image_sub1_;
  image_transport::ImageTransport it2_;
  image_transport::Subscriber image_sub2_;

  image_transport::Publisher image_pub1_;
  image_transport::Publisher debug_pub1_;
  image_transport::Publisher image_pub2_;
  image_transport::Publisher debug_pub2_;
  ros::Publisher pubID;

  cv::Mat inImage1_;
  cv::Mat inImage2_;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it1_(nh_), useCamInfo1_(true), it2_(nh_), useCamInfo2_(true)
  {
    image_sub1_ = it1_.subscribe("/robot/camera1/image_raw", 1, &ArucoMarkerPublisher::image_callback1, this);
    image_sub2_ = it2_.subscribe("/robot/camera2/image_raw", 1, &ArucoMarkerPublisher::image_callback2, this);
    image_pub1_ = it1_.advertise("result1", 1);
    image_pub2_ = it2_.advertise("result2", 1);
    debug_pub1_ = it1_.advertise("debug1", 1);
    debug_pub2_ = it2_.advertise("debug2", 1);
    pubID = nh_.advertise<std_msgs::Int32>("/ID", 1000);
    
    nh_.param<bool>("use_camera_info1", useCamInfo1_, false);
    nh_.param<bool>("use_camera_info2", useCamInfo2_, false);
    camParam1_ = aruco::CameraParameters();
    camParam2_ = aruco::CameraParameters();
  }

  void image_callback1(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishImage1 = image_pub1_.getNumSubscribers() > 0;
    bool publishDebug1 = debug_pub1_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage1_ = cv_ptr->image;
   
      // clear out previous detection results
      markers1_.clear();

      // ok, let's detect
      mDetector1_.detect(inImage1_, markers1_, camParam1_, marker_size1_, false);

		  std::cout << "The id of the detected marker detected is: ";
        for (std::size_t i = 0; i < markers1_.size(); ++i)
        {
          std::cout << markers1_.at(i).id << " ";
          
          // publish the ID on rostopic /ID
          msg = markers1_.at(i).id;
          pubID.publish(msg);
        }
        std::cout << std::endl;

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers1_.size(); ++i)
      {
        markers1_[i].draw(inImage1_, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage1)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage1_;
        image_pub1_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug1)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector1_.getThresholdedImage();
        debug_pub1_.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
  
  
  void image_callback2(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishImage2 = image_pub2_.getNumSubscribers() > 0;
    bool publishDebug2 = debug_pub2_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage2_ = cv_ptr->image;
   
      // clear out previous detection results
      markers2_.clear();

      // ok, let's detect
      mDetector2_.detect(inImage2_, markers2_, camParam2_, marker_size2_, false);

		    std::cout << "The id of the detected marker detected is: ";
        for (std::size_t i = 0; i < markers2_.size(); ++i)
        {
          std::cout << markers2_.at(i).id << " ";
          
          // publish the ID on rostopic /ID
          msg = markers2_.at(i).id;
          pubID.publish(msg);
        }
        std::cout << std::endl;

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers2_.size(); ++i)
      {
        markers2_[i].draw(inImage2_, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage2)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage2_;
        image_pub2_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug2)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector2_.getThresholdedImage();
        debug_pub2_.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");

  ArucoMarkerPublisher node;

  ros::spin();
}
