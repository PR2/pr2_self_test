/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "ros/node.h"
#include "image_msgs/Image.h"
#include "opencv_latest/CvBridge.h"
#include <stdio.h>
#include <signal.h>
#include <robot_mechanism_controllers/SetWaveform.h>
#include <realtime_tools/realtime_tools.h>
#include <robot_mechanism_controllers/trigger_controller.h>
#include <algorithm>
#include <qualification/TestResult.h>

class LedFlashTest
{
private:
  image_msgs::Image img_msg_;
  image_msgs::CvBridge img_bridge_;
  std::string window_name_;
  ros::Node &node_;
  double rate_;
  std::string led_set_waveform_;
  controller::trigger_configuration led_config_;
  robot_mechanism_controllers::SetWaveform::Response dummy_resp_;
  std::vector<double> intensities_;
  std::vector<double> led_time_;
  std::vector<double> exp_time_;
  int frame_;
  int skip_frames_;
  int keep_frames_;

public:
  LedFlashTest(ros::Node &node) : node_(node)
  {
    // Open the output file.

    // Get names for waveform generator.
  
    sleep(5); // Otherwise the others aren't ready. Plus let some frames go by...

    node_.param("led_controller", led_set_waveform_, (std::string) "led_controller");    
    led_set_waveform_ += "/set_waveform";

    // Initialize waveform generators.
    
    node_.param("~rate", rate_, 0.);
    
    led_config_.running = 1;
    led_config_.rep_rate = rate_;
    led_config_.phase = 0;
    led_config_.active_low = 1;
    led_config_.pulsed = 1;
    led_config_.duty_cycle = .5;

    SetWaveform(led_set_waveform_, led_config_);

    // Subscribe to image stream.

    node_.subscribe("image", img_msg_, &LedFlashTest::image_cb, this, 1);
  
    // Other parameters.
    node_.param("~skip", skip_frames_, 10);
    node_.param("~frames", keep_frames_, 100);
    frame_ = 0;
  }

  ~LedFlashTest()
  {
  }

  void SetWaveform(std::string s, robot_mechanism_controllers::SetWaveform::Request req)
  {
    ROS_DEBUG("Calling \"%s\"", s.c_str());
    if (!ros::service::call(s, req, dummy_resp_))
    {
      ROS_FATAL("Error calling \"%s\"", s.c_str());
      node_.shutdown();
    }
  }

  void image_cb()
  {
    frame_++;
    if (frame_ <= skip_frames_)
      return;
    
    // Compute image intensity.

    if (img_msg_.encoding.find("bayer") != std::string::npos)
      img_msg_.encoding = "mono";
    
    long long sum = 0;
    
    if (img_bridge_.fromImage(img_msg_, "mono"))
    {
      std::vector<unsigned char> data = img_msg_.uint8_data.data;
      int pixels = img_msg_.uint8_data.layout.dim[0].size * img_msg_.uint8_data.layout.dim[1].size;

      for (int i = 0; i < pixels; i++)
      {
        sum += data[i];
      }
      
      //ROS_INFO("Sum: %f", sum / 7e6);
    }
    
    double intensity = sum / 7e6;

    // Control logic
    
    double exp_time = img_msg_.header.stamp.toSec();
    exp_time_.push_back(exp_time);
    led_time_.push_back(controller::TriggerController::getTickStartTimeSec(exp_time, led_config_));
    intensities_.push_back(intensity);
  
    // If we are done, check the results and quit.
    if (frame_ == skip_frames_ + keep_frames_)
    {
      double max_i = *std::max_element(intensities_.begin(), intensities_.end());
      double min_i = *std::min_element(intensities_.begin(), intensities_.end());
      // double thresh = (max_i + min_i) / 2;
      double thresh_high = (2 * max_i + min_i) / 3;
      double thresh_low = (max_i + 2 * min_i) / 3;
      double max_high, min_high, max_low, min_low;
      node_.param("~max_high", max_high, 0.);
      node_.param("~min_high", min_high, 0.);
      node_.param("~max_low", max_low, 0.);
      node_.param("~min_low", min_low, 0.);
      
      //ROS_INFO("high: %f low: %f frame: %i size: %i", thresh_high; thresh_low, );

      bool fail = false;
      int nomatch = 0;
      for (int i = 0; i < keep_frames_; i++)
      {
        double delta = led_time_[i] - exp_time_[i];
        bool matched = false;
        if (delta <= max_high && delta >= min_high)
        {
          matched = true;
          if (intensities_[i] <= thresh_high)
          {
            ROS_ERROR("Frame %i: Not high intensity at %f, between %f and %f.", i, delta, min_high, max_high);
            fail = true;
          }
        }
        if (delta <= max_low && delta >= min_low)
        {
          matched = true;
          if (intensities_[i] >= thresh_low)
          {
            ROS_ERROR("Frame %i: Not low intensity at %f between %f and %f.", i, delta, min_low, max_low);
            fail = true;
          }
        }

        if (!matched)
        {
          nomatch++;
          //ROS_INFO("Frame %i, state %i at %f, does not match a rule.", i, intensities_[i] > thresh, delta);
        }
      }
    
      if (nomatch > keep_frames_ / 3)
        ROS_ERROR("More than a third of the frames did not match a rule.");

      node_.shutdown();
    
      qualification::TestResult::Response result;
      if (fail)
      {
        ROS_ERROR("LED test failed.");
      }
      else
      {
        ROS_INFO("LED test passed.");
      }

      qualification::TestResult::Response dummy_response;
      if (!ros::service::call("test_result", result, dummy_response))
      {
        ROS_ERROR("Error sending test result message.");
      }
    }
  }                 
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("timestamp_test");
  LedFlashTest tt(n);
  n.spin();
  
  return 0;
}
