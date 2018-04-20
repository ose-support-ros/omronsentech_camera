/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2018, OMRON SENTECH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *   * Neither the names of OMRON SENTECH nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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

/** \file stcamera_node.h
 *  \brief Class implementation for stcamera_node
 *
 *  This class implements the stcamera_node.
 */

#ifndef STCAMERA_STCAMERA_NODE_H
#define STCAMERA_STCAMERA_NODE_H

#include <ros/ros.h>
#include <StApi_TL.h>

#include "omronsentech_camera/stparameter.h"
#include "omronsentech_camera/stcamera_interface.h"

#include <omronsentech_camera/GenTLInfo.h>
#include <omronsentech_camera/DeviceConnection.h>
#include <omronsentech_camera/GetDeviceList.h>
#include <omronsentech_camera/GetModuleList.h>
#include <omronsentech_camera/GetSDKInfo.h>
#include <omronsentech_camera/GetGigEIPi.h>
#include <omronsentech_camera/SetGigEIPi.h>

namespace stcamera
{
  /** Map key: camera namespace; value: message structure DeviceConnection */
  typedef std::map<std::string, omronsentech_camera::DeviceConnection> 
      MapDeviceConnection;

  /** \class StCameraNode
   *  \brief Class implementation for stcamera_node
   *
   *  This class implements the stcamera_node.
   */ 
  class StCameraNode
  {
    public:

      /** Default Constructor.
       */
      StCameraNode();

      /** Constructor with parent node. This constructor is used by Nodelet.
       *
       * \param[in] nh Parent node.
       */
      StCameraNode(ros::NodeHandle nh);

      /** Destructor.
       */
      virtual ~StCameraNode();

      /** Initialization. This function is called by constructor.
       * In the initialization, list of allowed camera to connect is retrieved
       * and the GenTL modules are initialized. 
       */
      virtual void init();

      /** Function called inside process loop.
       *  In this function, device availability is checked. Any device lost
       *  will be announced through device_connection topic. Any new found
       *  device will be processed through calling initializeCamera() .
       */
      virtual void spin();

    protected:

      /** Function to initialize a camera.
       * In this function, whether connection to the given camera is allowed
       * will be checked.
       * \param[in] p_iface Pointer to the IStInterface of the camera.
       * \param[in] p_devinfo Pointer to the IStDeviceInfo of the device.
       * \return True if the camera is successfully initialized. 
       *            False otherwise.
       */
      bool initializeCamera(StApi::IStInterface *p_iface,
                            const StApi::IStDeviceInfo *p_devinfo);


      /** ROS service callback for obtaining list of all detected devices
       * including the disallowed camera to connect.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return Always true.
       */
      bool getDeviceListCallback(
          omronsentech_camera::GetDeviceList::Request &req,
          omronsentech_camera::GetDeviceList::Response &res);

      /** ROS service callback for obtaining all module name: System, Interface,
       * LocalDevice, RemoteDevice, DataStream.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return Always true.
       */
      bool getModuleListCallback(
          omronsentech_camera::GetModuleList::Request &req,
          omronsentech_camera::GetModuleList::Response &res);

      /** ROS service callback for obtaining SentechSDK and GenTL information.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return Always true.
       */
      bool getSDKInfoCallback(omronsentech_camera::GetSDKInfo::Request &req,
          omronsentech_camera::GetSDKInfo::Response &res);

      /** ROS service callback for obtaining IP address information of the
       * given GigEVision camera which is not opened yet.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return True on success. False on failure. 
       */
      bool getGigEIPCallback(omronsentech_camera::GetGigEIPi::Request &req,
          omronsentech_camera::GetGigEIPi::Response &res);

      /** ROS service callback for assigning IP address to the given GigEVision 
       * camera which is not opened yet.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return True on success. False on failure. 
       */
      bool setGigEIPCallback(omronsentech_camera::SetGigEIPi::Request &req,
          omronsentech_camera::SetGigEIPi::Response &res);
      
      /** Main node handle for stcamera_node */
      ros::NodeHandle nh_;

      /** Store the list of allowed camera or connected camera and the
       * corresponding instances. */
      MapCameraInterface map_camera_;

      /** Store the list of all detected camera (including disallowed to
       * connect) and the camera information. */
      MapDeviceConnection map_connection_;

      /** StParameter instance */
      StParameter param_;

      /** Publisher for announcing any device lost or newly found device that
       * is allowed to connect. */
      ros::Publisher msg_device_connection_;
      
      /** Server for get_device_list */
      ros::ServiceServer      srv_get_device_list_;
      /** Server for get_module_list */
      ros::ServiceServer      srv_get_module_list_; 
      /** Server for get_sdk_info */
      ros::ServiceServer      srv_get_sdk_info_;
      /** Server for get_gige_ip */
      ros::ServiceServer      srv_get_gige_ip_i_;
      /** Server for set_gige_ip */
      ros::ServiceServer      srv_set_gige_ip_i_;
      
      /** Variable for auto initialization of SentechSDK */
      StApi::CStApiAutoInit stapi_autoinit_;

      /** List of GenTL instances used by SentechSDK */
      StApi::CIStSystemPtrArray stapi_systems_;

    private:

      /** Helper function to fill the structure of DeviceConnection.
       * \param[in] p_tlinfo: Pointer to the IStSystemInfo of the device.
       * \param[in] p_iface: Pointer to the IStInterface of the device.
       * \param[in] p_devinfo: Pointer to the IStDeviceInfo of the device.
       * \param[in] device_namespace: Namespace of the device. 
       * \param[in] connected: whether the device is connected or not.
       * \return  Filled DeviceConnection.
       */
      omronsentech_camera::DeviceConnection fillDeviceConnectionData(
          const StApi::IStSystemInfo *p_tlinfo,
          StApi::IStInterface  *p_iface,
          const StApi::IStDeviceInfo *p_devinfo,
          std::string device_namespace = "",
          bool connected = false);

      /** Guard to prevent race when looking for new devices. */
      std::mutex mtx_update_device_list_;

      /** Guard to prevent race when accessing map_camera_. */
      std::mutex mtx_map_camera_;
  };
}
#endif
