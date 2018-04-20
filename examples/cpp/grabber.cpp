/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2018, OMRON SENTECH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of OMRON SENTECH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>

#include <omronsentech_camera/GetSDKInfo.h>
#include <omronsentech_camera/GetLastError.h>
#include <omronsentech_camera/GetDeviceList.h>
#include <omronsentech_camera/WriteNodeFloat.h>
#include <omronsentech_camera/GetGenICamNodeInfo.h>
#include <omronsentech_camera/GetImageAcquisitionStatus.h>
#include <omronsentech_camera/EnableImageAcquisition.h>
#include <omronsentech_camera/EnableTrigger.h>
#include <omronsentech_camera/SendSoftTrigger.h>

#define SERVER_NODE "stcamera_node"
#define MAX_GRABBED 10
#define TRIGGER_SELECTOR "FrameStart"
#define TRIGGER_SOURCE "Software"
int grabbed_images = 0;

// Get SentechSDK and GenTL information
void displaySDKInfo(ros::NodeHandle &n)
{
	ros::ServiceClient client = n.serviceClient<omronsentech_camera::GetSDKInfo>(
      SERVER_NODE "/get_sdk_info");
	omronsentech_camera::GetSDKInfo srv;
	if (client.call(srv))
  {
    std::cout << "SentechSDK ver.: " << srv.response.sdk_version << std::endl;
    for (size_t i = 0; i < srv.response.gentl_info_list.size(); i++)
    {
      std::cout << "GenTL vendor: " << srv.response.gentl_info_list[i].vendor 
          << std::endl;
      std::cout << "\tversion: " << srv.response.gentl_info_list[i].version 
          << std::endl;
      std::cout << "\tproducer: " << srv.response.gentl_info_list[i]
          .producer_version << std::endl;
      std::cout << "\tfullpath: " << srv.response.gentl_info_list[i]
          .full_path << std::endl;
      std::cout << "\tTL type: " << srv.response.gentl_info_list[i]
          .tltype << std::endl;
    }
  }
}

// Get one connected device's namespace
std::string getConnectedDeviceNamespace(ros::NodeHandle &n)
{
  ros::ServiceClient client = n.serviceClient<
      omronsentech_camera::GetDeviceList>(SERVER_NODE "/get_device_list");
	omronsentech_camera::GetDeviceList srv;
	if (client.call(srv))
	{
    for (size_t i = 0; i < srv.response.device_list.size(); i++)
    {
			if (srv.response.device_list[i].connected) 
				return srv.response.device_list[i].device_namespace;
		}
	}
	return "";
}

// Get last error
int getLastError(ros::NodeHandle &n, std::string dev_ns)
{
  const char NS[] = SERVER_NODE "/";
  std::string ns = NS + dev_ns + "/get_last_error";
	ros::ServiceClient client = n.serviceClient<
			omronsentech_camera::GetLastError>(ns);
	omronsentech_camera::GetLastError srv;
  client.call(srv);
  std::cout << "Last error: " << srv.response.error_code << std::endl;
  std::cout << srv.response.description << std::endl;
  return srv.response.error_code;
}

// Set Gain
int setGain(ros::NodeHandle &n, std::string dev_ns, double value)
{
  const char NS[] = SERVER_NODE "/";
  std::string ns = NS + dev_ns + "/write_node_float";
  ros::ServiceClient client = n.serviceClient<
    omronsentech_camera::WriteNodeFloat>(ns);
	omronsentech_camera::WriteNodeFloat srv;
  srv.request.module_name = "RemoteDevice";
  srv.request.node_name = "Gain";
  srv.request.value = value;
	if (client.call(srv)) return 0;
  return getLastError(n, dev_ns);
}

// Get Gain
int getGain(ros::NodeHandle &n, std::string dev_ns, double &value, 
    double &value_min, double &value_max)
{
  const char NS[] = SERVER_NODE "/";
  std::string ns = NS + dev_ns + "/get_genicam_node_info";
  ros::ServiceClient client = n.serviceClient<
    omronsentech_camera::GetGenICamNodeInfo>(ns);
	omronsentech_camera::GetGenICamNodeInfo srv;
  srv.request.module_name = "RemoteDevice";
  srv.request.node_name = "Gain";
	if (client.call(srv)) 
  {
    value = std::stod(srv.response.current_value);
    value_min = std::stod(srv.response.min_value);
    value_max = std::stod(srv.response.max_value);
    return 0;
  }
  return getLastError(n, dev_ns);
}

// Image callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (++grabbed_images > MAX_GRABBED) return;
  std::cout << "Grabbed " << grabbed_images << " images. WxH: " 
      << msg->width << " x " << msg->height << " Encoding: "
      << msg->encoding << std::endl;
}

// Get the status of image acquisition
bool isImageAcquisitionEnabled(ros::NodeHandle &n, std::string dev_ns)
{
  const char NS[] = SERVER_NODE "/";
  std::string ns = NS + dev_ns + "/get_image_acquisition_status";
  ros::ServiceClient client = n.serviceClient<
    omronsentech_camera::GetImageAcquisitionStatus>(ns);
	omronsentech_camera::GetImageAcquisitionStatus srv;
	if (client.call(srv)) 
  {
    return srv.response.value;
  }
  getLastError(n, dev_ns);
  return false;
}

// Set the status of image acquisition
int setImageAcquisition(ros::NodeHandle &n, std::string dev_ns, bool enabled)
{
  const char NS[] = SERVER_NODE "/";
  std::string ns = NS + dev_ns + "/enable_image_acquisition";
  ros::ServiceClient client = n.serviceClient<
    omronsentech_camera::EnableImageAcquisition>(ns);
	omronsentech_camera::EnableImageAcquisition srv;
  srv.request.value = enabled;
	if (client.call(srv)) 
  {
    return 0;
  }
  return getLastError(n, dev_ns);
}

// Set trigger
int setTrigger(ros::NodeHandle &n, std::string dev_ns, bool enabled)
{
  const char NS[] = SERVER_NODE "/";
  std::string ns = NS + dev_ns + "/enable_trigger";
  ros::ServiceClient client = n.serviceClient<
    omronsentech_camera::EnableTrigger>(ns);
	omronsentech_camera::EnableTrigger srv;
  srv.request.trigger_selector = TRIGGER_SELECTOR;
  srv.request.trigger_source = TRIGGER_SOURCE;
  srv.request.trigger_delayus = 0;
  srv.request.value= enabled;
	if (client.call(srv)) 
  {
    return 0;
  }
  return getLastError(n, dev_ns);
}

// Send trigger
int sendTrigger(ros::NodeHandle &n, std::string dev_ns)
{
  const char NS[] = SERVER_NODE "/";
  std::string ns = NS + dev_ns + "/send_soft_trigger";
  ros::ServiceClient client = n.serviceClient<
    omronsentech_camera::SendSoftTrigger>(ns);
	omronsentech_camera::SendSoftTrigger srv;
  srv.request.trigger_selector = TRIGGER_SELECTOR;
	if (client.call(srv)) 
  {
    return 0;
  }
  return getLastError(n, dev_ns);
}


// Main 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "grabber");
  ros::NodeHandle n;
  boost::thread th(boost::bind(&ros::spin));

  // Display SDK info
  displaySDKInfo(n);

  // Check connected devices
  std::string dev_ns = "";
  ros::Rate r(1);
  while (1)
  {
    dev_ns = getConnectedDeviceNamespace(n);
    if (!dev_ns.empty() || !ros::ok())
    {
      break;
    }
    r.sleep();
  }

  if (!dev_ns.empty()) 
  {
    // Read gain value
    double gain_value = 0;
    double gain_min_value = 0;
    double gain_max_value = 0;
    int ret = getGain(n, dev_ns, gain_value, gain_min_value, gain_max_value);
    if (ret == 0)
    {
      std::cout << "Current Gain: " << gain_value << std::endl;
    }
    double new_gain_value = ((int)(gain_min_value + gain_max_value)/2);

    // Change gain value 
    ret = setGain(n, dev_ns, new_gain_value);
    if (ret == 0)
    {
      std::cout << "Update Gain to " << new_gain_value << " was successful"
          << std::endl;
    }

    // Subscribe to the connected device
    const char NS[] = SERVER_NODE "/";
    std::string ns = NS + dev_ns + "/image_raw";
    ros::Subscriber image = n.subscribe(ns, 1, imageCallback);

    if (!isImageAcquisitionEnabled(n, dev_ns))
    {
      setImageAcquisition(n, dev_ns, true);
    }
    while(grabbed_images < MAX_GRABBED)
    {
      r.sleep();
    }

    // Trigger mode
    std::cout << std::endl << "Enable trigger selector: " << TRIGGER_SELECTOR <<
      ", source: " << TRIGGER_SOURCE << std::endl;
    try
    {
      setImageAcquisition(n, dev_ns, false);
      setTrigger(n, dev_ns, true);
      grabbed_images = 0;
      setImageAcquisition(n, dev_ns, true);
      for (int i = 0; i < 10; i++)
      {
        sendTrigger(n, dev_ns);
        std::cout << "Sleep for 1s" << std::endl;
        r.sleep();
      }
      setImageAcquisition(n, dev_ns, false);
      setTrigger(n, dev_ns, false);
      setImageAcquisition(n, dev_ns, true);
    }
    catch(...)
    {
    }

    // Set the original gain value
    setGain(n, dev_ns, gain_value);
  }
  
  ros::shutdown();
  return 0;
}
