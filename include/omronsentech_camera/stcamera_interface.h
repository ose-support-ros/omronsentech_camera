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

/** \file stcamera_interface.h
 *  \brief Base class to control a connected camera
 *
 *  This is a base class to control a connected camera, including callback
 *  for services. 
 */

#ifndef STCAMERA_STCAMERA_INTERFACE_H
#define STCAMERA_STCAMERA_INTERFACE_H

#include <mutex>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <omronsentech_camera/Event.h>
#include <omronsentech_camera/Chunk.h>

#include <omronsentech_camera/ReadNode.h>
#include <omronsentech_camera/ReadNodeBool.h>
#include <omronsentech_camera/ReadNodeEnum.h>
#include <omronsentech_camera/ReadNodeInt.h>
#include <omronsentech_camera/ReadNodeFloat.h>
#include <omronsentech_camera/ReadNodeString.h>

#include <omronsentech_camera/WriteNode.h>
#include <omronsentech_camera/WriteNodeBool.h>
#include <omronsentech_camera/WriteNodeEnumInt.h>
#include <omronsentech_camera/WriteNodeEnumStr.h>
#include <omronsentech_camera/WriteNodeInt.h>
#include <omronsentech_camera/WriteNodeFloat.h>
#include <omronsentech_camera/WriteNodeString.h>
#include <omronsentech_camera/ExecuteNode.h>

#include <omronsentech_camera/EnableChunk.h>
#include <omronsentech_camera/EnableTrigger.h>
#include <omronsentech_camera/EnableEventNode.h>
#include <omronsentech_camera/EnableImageAcquisition.h>
#include <omronsentech_camera/EnableEventAcquisition.h>

#include <omronsentech_camera/GetImageAcquisitionStatus.h>
#include <omronsentech_camera/GetEventAcquisitionStatusList.h>
#include <omronsentech_camera/GetEventNodeStatusList.h>
#include <omronsentech_camera/GetChunkList.h>
#include <omronsentech_camera/GetTriggerList.h>
#include <omronsentech_camera/GetEnumList.h>
#include <omronsentech_camera/GetGenICamNodeInfo.h>
#include <omronsentech_camera/SendSoftTrigger.h>
#include <omronsentech_camera/GetLastError.h>

#include <StApi_TL.h>
#include "omronsentech_camera/stparameter.h"

#include "omronsentech_camera/stheader.h"

namespace stcamera
{

/** Macro for failure callback. */
#define RETURN_ERR(X, MSG) \
  ROS_ERROR("%s %s %d: \n\t%s error: %s", \
      __FILE__,__func__,__LINE__,camera_namespace_.c_str(),MSG); \
  last_error_ = X; return false;

/** Macro for checking null. */
#define CHECK_NULLPTR(P, X, MSG) if (nullptr == P) \
  { RETURN_ERR(X, MSG); } 

/** Macro for common exception. */
#define CATCH_COMMON_ERR() catch(const StApi::CStGenTLErrorException &x) \
  {\
    RETURN_ERR(x.GetError(), x.GetDescription());\
  }\
  catch(GenICam::GenericException &x)\
  {\
    RETURN_ERR(GENICAM_ERROR, x.GetDescription());\
  }\
  catch(...)\
  {\
  }\
  RETURN_ERR(UNKNOWN_ERROR, UNKNOWN_ERROR_STR);

  /** A structure containing a pair of topic name for publishing data from
   * a callback function and the pointer to the instance of the callback. */
  struct StCallback
  {
    /** Name of the topic. */
    std::string topic_name_;
    /** Pointer to the callback. */
    StApi::IStRegisteredCallbackReleasable *cb_;
  };

  class StCameraInterface;

  /** Map key: camera namespace; value: a pointer to StCameraInterface 
   * instance that control the camera. */
  typedef std::map<std::string, StCameraInterface*> MapCameraInterface; 

  /** Map key: callback node name; value: struct StCallback */
  typedef std::map<std::string, struct StCallback> MapCallback;

  /** Map key: chunk node name; value: pointer to the chunk node */
  typedef std::map<std::string, GenApi::INode*> MapChunk;

  /** Map key: topic name; value: the publisher correspond to the topic name */
  typedef std::map<std::string, ros::Publisher> MapPublisher;

  /** \class StCameraInterface
   *  \brief Base class to control a connected camera
   *
   *  This is a base class to control a connected camera, including callback
   *  for services. 
   */ 
  class StCameraInterface
  {
    public:

      /** Constructor.
       *
       * \param[in] dev Pointer to the IStDeviceReleasable of the device.
       * \param[in] nh_parent The main ROS node handle.
       * \param[in] camera_namespace The namespace for the device.
       * \param[in] param Pointer to the StParameter class instance.
       * \param[in] queue_size Used for initializing publisher (Maximum number 
       *            of outgoing messages to be queued for delivery to 
       *            subscribers). Default is set to #STCAMERA_QUEUE_SIZE 
       */
      StCameraInterface(StApi::IStDeviceReleasable *dev,
                        ros::NodeHandle nh_parent, 
                        const std::string &camera_namespace, 
                        StParameter *param,
                        uint32_t queue_size = STCAMERA_QUEUE_SIZE);

      /** Destructor.
       *
       * All event node callbacks are deregistered and the callbacks are
       * released here. 
       */
      virtual ~StCameraInterface();

      /** Check if the device is disconnected.
       * \return True if device is already disconnected. False otherwise.
       */
      bool deviceIsLost();

    protected:

      /** GenTL System event callback function.
       * 
       * This function is called automatically when any GenTL System event 
       * occured.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventSystemCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** GenTL Interface event callback function.
       * 
       * This function is called automatically when any GenTL Interface event 
       * occured. The event data is published through the default event topic.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventInterfaceCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** GenTL Device event callback function.
       * 
       * This function is called automatically when any GenTL Device event 
       * occured. The event data is published through the default event topic.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventDeviceCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** GenICam node event callback function.
       * 
       * This function is called automatically if a GenICam node event is
       * registered. The event data is published through either the custom
       * event topic provided when enabling the event or the default event
       * topic.
       * \param[in] p Pointer to the GenICam node callback.
       * \param[in] pvContext user's pointer (pointer to the MapCallback
       *            of which the event belongs to)
       */
      void eventGenApiNodeCB(GenApi::INode *p, void *pvContext);

      /** GenTL DataStream event callback function.
       * 
       * This function is called automatically when any GenTL DataStream event 
       * occured. Except for GenTL EVENT_NEW_BUFFER of which data is published 
       * throught image_transport mechanism or chunk topic, other event data is 
       * published through the default event topic.
       * \param[in] p Pointer to the callback handle.
       * \param[in] pvContext user's pointer (not used).
       */
      void eventDataStreamCB(StApi::IStCallbackParamBase *p, void *pvContext);

      /** ROS service callback for obtaining GenICam node value regardless
       * the node value's interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeCallback(omronsentech_camera::ReadNode::Request &req,
          omronsentech_camera::ReadNode::Response &res);

      /** ROS service callback for obtaining GenICam node value with boolean
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeBoolCallback(omronsentech_camera::ReadNodeBool::Request &req,
          omronsentech_camera::ReadNodeBool::Response &res);

      /** ROS service callback for obtaining GenICam node value with enumeration
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeEnumCallback(omronsentech_camera::ReadNodeEnum::Request &req,
          omronsentech_camera::ReadNodeEnum::Response &res);

      /** ROS service callback for obtaining GenICam node value with integer
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeIntCallback(omronsentech_camera::ReadNodeInt::Request &req,
          omronsentech_camera::ReadNodeInt::Response &res);

      /** ROS service callback for obtaining GenICam node value with float
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeFloatCallback(
          omronsentech_camera::ReadNodeFloat::Request &req,
          omronsentech_camera::ReadNodeFloat::Response &res);

      /** ROS service callback for obtaining GenICam node value with string
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool readNodeStringCallback(
          omronsentech_camera::ReadNodeString::Request &req,
          omronsentech_camera::ReadNodeString::Response &res);

      /** ROS service callback for writing GenICam node value regardless 
       * the node value's interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeCallback(
          omronsentech_camera::WriteNode::Request &req,
          omronsentech_camera::WriteNode::Response &res);

      /** ROS service callback for writing GenICam node value with boolean
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeBoolCallback(
          omronsentech_camera::WriteNodeBool::Request &req,
          omronsentech_camera::WriteNodeBool::Response &res);

      /** ROS service callback for writing GenICam node value with enumeration
       * interface type, where the input is the integer value of the enumeration
       * entry.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeEnumIntCallback(
          omronsentech_camera::WriteNodeEnumInt::Request &req,
          omronsentech_camera::WriteNodeEnumInt::Response &res);

      /** ROS service callback for writing GenICam node value with enumeration
       * interface type, where the input is the symbolic name of the enumeration
       * entry.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeEnumStrCallback(
          omronsentech_camera::WriteNodeEnumStr::Request &req,
          omronsentech_camera::WriteNodeEnumStr::Response &res);

      /** ROS service callback for writing GenICam node value with integer
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeIntCallback(
          omronsentech_camera::WriteNodeInt::Request &req,
          omronsentech_camera::WriteNodeInt::Response &res);

      /** ROS service callback for writing GenICam node value with float
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeFloatCallback(
          omronsentech_camera::WriteNodeFloat::Request &req,
          omronsentech_camera::WriteNodeFloat::Response &res);

      /** ROS service callback for writing GenICam node value with string
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool writeNodeStringCallback(
          omronsentech_camera::WriteNodeString::Request &req,
          omronsentech_camera::WriteNodeString::Response &res);

      /** ROS service callback for executing GenICam node that has command
       * interface type.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool executeNodeCallback(
          omronsentech_camera::ExecuteNode::Request &req,
          omronsentech_camera::ExecuteNode::Response &res);

       /** ROS service callback for enabling or disabling chunk.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableChunkCallback(
          omronsentech_camera::EnableChunk::Request &req,
          omronsentech_camera::EnableChunk::Response &res);

      /** ROS service callback for enabling or disabling trigger.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableTriggerCallback( 
          omronsentech_camera::EnableTrigger::Request &req,
          omronsentech_camera::EnableTrigger::Response &res);

      /** ROS service callback for enabling or disabling GenICam event.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableEventNodeCallback( 
          omronsentech_camera::EnableEventNode::Request &req,
          omronsentech_camera::EnableEventNode::Response &res);

      /** ROS service callback for enabling or disabling image acquisition.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableImageAcquisitionCallback( 
          omronsentech_camera::EnableImageAcquisition::Request &req,
          omronsentech_camera::EnableImageAcquisition::Response &res);

      /** ROS service callback for enabling or disabling event acquisition. 
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool enableEventAcquisitionCallback( 
          omronsentech_camera::EnableEventAcquisition::Request &req,
          omronsentech_camera::EnableEventAcquisition::Response &res);

      /** ROS service callback for obtaining image acquisition status.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getImageAcquisitionStatusCallback( 
          omronsentech_camera::GetImageAcquisitionStatus::Request &req,
          omronsentech_camera::GetImageAcquisitionStatus::Response &res);

      /** ROS service callback for obtaining event acquisition status.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getEventAcquisitionStatusListCallback( 
          omronsentech_camera::GetEventAcquisitionStatusList::Request &req,
          omronsentech_camera::GetEventAcquisitionStatusList::Response &res);

      /** ROS service callback for obtaining GenICam node events status.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getEventNodeStatusListCallback( 
          omronsentech_camera::GetEventNodeStatusList::Request &req,
          omronsentech_camera::GetEventNodeStatusList::Response &res);

      /** ROS service callback for obtaining chunks provided by the camera.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getChunkListCallback(omronsentech_camera::GetChunkList::Request &req,
          omronsentech_camera::GetChunkList::Response &res);

      /** ROS service callback for obtaining triggers provided by the camera.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getTriggerListCallback(
          omronsentech_camera::GetTriggerList::Request &req,
          omronsentech_camera::GetTriggerList::Response &res);

      /** ROS service callback for obtaining enumeration entry of a given
       * GenICam node with interface type of enumeration.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getEnumListCallback(omronsentech_camera::GetEnumList::Request &req,
          omronsentech_camera::GetEnumList::Response &res);

      /** ROS service callback for obtaining information of a given GenICam 
       * node.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool getGenICamNodeInfoCallback(
          omronsentech_camera::GetGenICamNodeInfo::Request &req,
          omronsentech_camera::GetGenICamNodeInfo::Response &res);

      /** ROS service callback for sending software trigger execution command
       * to the camera.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return true on success. False otherwise. Last error code and 
       *         description can be retrieved through get_last_error service.
       */
      bool sendSoftTriggerCallback(
          omronsentech_camera::SendSoftTrigger::Request &req,
          omronsentech_camera::SendSoftTrigger::Response &res);

      /** ROS service callback for obtaining the last error.
       * \param[in] req ROS service request
       * \param[out] res ROS service response
       * \return always true.
       */
      bool getLastErrorCallback(
          omronsentech_camera::GetLastError::Request &req,
          omronsentech_camera::GetLastError::Response &res);

      /** Initialize information of the camera. This information is delivered 
       * to image subscribers upon publishing acquired images. 
       */  
      void initializeCameraInfo();

      /** A helper function to obtain the GenICam node for a given module name.
       * \param[in] module_name Name of the module: System, Interface,
       *             LocalDevice, RemoteDevice, DataStream.
       * \return nullptr if module name is invalid. Otherwise GenICam node of
       *         the module.
       */
      GenApi::INodeMap *getNodeMap(std::string &module_name);

      /** A helper function to obtain one of event callback mapping for a given
       * module name.
       * \param[in] module_name Name of the module: System, Interface,
       *             LocalDevice, RemoteDevice, DataStream.
       * \return nullptr if module name is invalid. Otherwise the callback map
       *         that corresponds to the module.
       */
      MapCallback *getCallbackMap(std::string &module_name);

      /** A helper function to publish an event through the default event
       * publisher.
       * \param[in] msg The message to deliver.
       */
      void publishEventDefault(omronsentech_camera::Event &msg);

      /** Namespace of the camera. The namespace is generated by using 
       * StParameter::getNamespace() function.
       */
      const std::string camera_namespace_;

      /** Smart pointer to the CIStDevice that holds the device instance. */
      StApi::CIStDevicePtr tl_dev_;

      /** Smart pointer to the CIStDevice that holds the device's datastream. */
      StApi::CIStDataStreamPtr tl_ds_;

      /** Child node handle for the camera. */
      ros::NodeHandle nh_; 

      /** Pointer to the instance of StParameter. */
      StParameter *param_;

      /** Used for delivering image to subscriber. */
      image_transport::ImageTransport it_; 

      /** Used for managing the camera info. */
      camera_info_manager::CameraInfoManager cinfo_;

      /** Publisher for image data. */
      image_transport::CameraPublisher it_campub_;

      /** Publisher for chunk data. */
      ros::Publisher msg_chunk_;

      /** Map of event publisher in pair of (topic name, publisher). */
      MapPublisher  map_msg_event_;

      /** Queue size for publisher (used when dynamically advertise custom
       * topic name).
       */
      uint32_t queue_size_;
  
      /** Server for read_node */
      ros::ServiceServer  srv_read_node_;
      /** Server for read_node_bool */
      ros::ServiceServer  srv_read_node_bool_;
      /** Server for read_node_enum */
      ros::ServiceServer  srv_read_node_enum_;
      /** Server for read_node_int */
      ros::ServiceServer  srv_read_node_int_;
      /** Server for read_node_float */
      ros::ServiceServer  srv_read_node_float_;
      /** Server for read_node_string */
      ros::ServiceServer  srv_read_node_string_;

      /** Server for write_node */
      ros::ServiceServer  srv_write_node_;
      /** Server for write_node_bool */
      ros::ServiceServer  srv_write_node_bool_;
      /** Server for write_node_enum_int */
      ros::ServiceServer  srv_write_node_enum_int_;
      /** Server for write_node_enum_str */
      ros::ServiceServer  srv_write_node_enum_str_;
      /** Server for write_node_int */
      ros::ServiceServer  srv_write_node_int_;
      /** Server for write_node_float */
      ros::ServiceServer  srv_write_node_float_;
      /** Server for write_node_string */
      ros::ServiceServer  srv_write_node_string_;
      /** Server for execute_node */
      ros::ServiceServer  srv_execute_node_;
      
      /** Server for enable_chunk */
      ros::ServiceServer  srv_enable_chunk_;    
      /** Server for enable_trigger */
      ros::ServiceServer  srv_enable_trigger_;  
      /** Server for enable_event_node */
      ros::ServiceServer  srv_enable_event_node_;
      /** Server for enable_image_acquisition */
      ros::ServiceServer  srv_enable_image_acquisition_;
      /** Server for enable_event_acquisition */
      ros::ServiceServer  srv_enable_event_acquisition_;
      
      /** Server for get_image_acquisition_status */
      ros::ServiceServer  srv_get_image_acquisition_status_;
      /** Server for get_event_acquisition_status_list */
      ros::ServiceServer  srv_get_event_acquisition_status_list_;
      /** Server for get_event_node_status_list */
      ros::ServiceServer  srv_get_event_node_status_list_;
      /** Server for get_chunk_list */
      ros::ServiceServer  srv_get_chunk_list_;
      /** Server for get_trigger_list */
      ros::ServiceServer  srv_get_trigger_list_;
      /** Server for get_enum_list */
      ros::ServiceServer  srv_get_enum_list_;
      /** Server for get_genicam_node_info */
      ros::ServiceServer  srv_get_genicam_node_info_;

      /** Server for send_soft_trigger */
      ros::ServiceServer  srv_send_soft_trigger_;  
      /** Server for get_last_error */
      ros::ServiceServer  srv_get_last_error_;  

      /** Guard to prevent race when enabling or disabling image acquisition. */
      std::mutex mtx_acquisition_;
      /** Flag to indicate if image acquisition is being enabled or not. */
      bool bool_acquisition_is_started_;

      /** Guard to prevent race when enabling or disabling event acquisition. */
      std::mutex mtx_event_;

      /** Flag to indicate if GenTL System event is being enabled or not. */
      bool bool_event_system_;
      /** Flag to indicate if GenTL Interface event is being enabled or not. */
      bool bool_event_interface_;
      /** Flag to indicate if GenTL Device event is being enabled or not. */
      bool bool_event_device_;
      /** Flag to indicate if GenTL DataStream event is being enabled or not. */
      bool bool_event_datastream_;

      /** Map the GenICam event node callback name (System module) with the 
       *  structure StCallback (pair of topic name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_system_;
      /** Map the GenICam event node callback name (Interface module) with the 
       *  structure StCallback (pair of topic name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_interface_;
      /** Map the GenICam event node callback name (LocalDevice module) with 
       *  the structure StCallback (pair of topic name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_localdevice_;
      /** Map the GenICam event node callback name (RemoteDevice module) with 
       *  the structure StCallback (pair of topic name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_remotedevice_;
      /** Map the GenICam event node callback name (DataStream module) with the 
       *  structure StCallback (pair of topic name and the pointer to the 
       *  callback instance. 
       */
      MapCallback map_event_datastream_;

			/** Guard to prevent race when enabling or disabling chunk. */
      std::mutex mtx_chunk_;
			/** Map the chunk name with the chunk's GenICam node */
      MapChunk  map_chunk_;

      /** Store the last error code */
      int32_t last_error_;

      /** destroyed */
      bool destroyed_;
  };
}
#endif
