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

#include "omronsentech_camera/stcamera_interface.h"
#include <omronsentech_camera/ElementFloat64.h>
#include <omronsentech_camera/ElementInt64.h>
#include <omronsentech_camera/ElementBool.h>
#include <omronsentech_camera/ElementString.h>
#include <omronsentech_camera/GenICamEvent.h>
#include <sensor_msgs/CameraInfo.h>

namespace stcamera
{

  StCameraInterface::StCameraInterface(
      StApi::IStDeviceReleasable *dev,
      ros::NodeHandle nh_parent, 
      const std::string &camera_namespace, 
      StParameter *param,
      uint32_t queue_size):
    camera_namespace_(camera_namespace),
    tl_dev_(dev),
    nh_(nh_parent, camera_namespace_),
    param_(param),
    it_(nh_),
    cinfo_(nh_),
    msg_chunk_(nh_.advertise<omronsentech_camera::Chunk>(
        std::string(STMSG_chunk), queue_size)),
    queue_size_(queue_size),
    it_campub_(it_.advertiseCamera(std::string(STMSG_image), queue_size)),
    srv_read_node_(nh_.advertiseService(std::string(STSRV_R_node), 
        &StCameraInterface::readNodeCallback, this)),
    srv_read_node_bool_(nh_.advertiseService(
        std::string(STSRV_R_node_bool), 
        &StCameraInterface::readNodeBoolCallback, this)),
    srv_read_node_enum_(nh_.advertiseService(
        std::string(STSRV_R_node_enum), 
        &StCameraInterface::readNodeEnumCallback, this)),
    srv_read_node_int_(nh_.advertiseService(
        std::string(STSRV_R_node_int), 
        &StCameraInterface::readNodeIntCallback, this)),
    srv_read_node_float_(nh_.advertiseService(
        std::string(STSRV_R_node_float), 
        &StCameraInterface::readNodeFloatCallback, this)),
    srv_read_node_string_(nh_.advertiseService(
        std::string(STSRV_R_node_string), 
        &StCameraInterface::readNodeStringCallback, this)),
    srv_write_node_(nh_.advertiseService(
        std::string(STSRV_W_node), 
        &StCameraInterface::writeNodeCallback, this)),
    srv_write_node_bool_(nh_.advertiseService(
        std::string(STSRV_W_node_bool), 
        &StCameraInterface::writeNodeBoolCallback, this)),
    srv_write_node_enum_int_(nh_.advertiseService(
        std::string(STSRV_W_node_enum_int), 
        &StCameraInterface::writeNodeEnumIntCallback, this)),
    srv_write_node_enum_str_(nh_.advertiseService(
        std::string(STSRV_W_node_enum_str), 
        &StCameraInterface::writeNodeEnumStrCallback, this)),
    srv_write_node_int_(nh_.advertiseService(
        std::string(STSRV_W_node_int), 
        &StCameraInterface::writeNodeIntCallback, this)),
    srv_write_node_float_(nh_.advertiseService(
        std::string(STSRV_W_node_float),
        &StCameraInterface::writeNodeFloatCallback, this)),
    srv_write_node_string_(nh_.advertiseService(
        std::string(STSRV_W_node_string), 
        &StCameraInterface::writeNodeStringCallback, this)),
    srv_execute_node_(nh_.advertiseService(
        std::string(STSRV_W_node_command), 
        &StCameraInterface::executeNodeCallback, this)),
    srv_enable_chunk_(nh_.advertiseService(
        std::string(STSRV_E_chunk), 
        &StCameraInterface::enableChunkCallback, this)),
    srv_enable_trigger_(nh_.advertiseService(
        std::string(STSRV_E_trigger), 
        &StCameraInterface::enableTriggerCallback, this)),
    srv_enable_event_node_(nh_.advertiseService(
        std::string(STSRV_E_event_node), 
        &StCameraInterface::enableEventNodeCallback, this)),
    srv_enable_image_acquisition_(nh_.advertiseService(
        std::string(STSRV_E_image_acquisition), 
        &StCameraInterface::enableImageAcquisitionCallback, this)),
    srv_enable_event_acquisition_(nh_.advertiseService(
        std::string(STSRV_E_event_acquisition), 
        &StCameraInterface::enableEventAcquisitionCallback, this)),
    srv_get_image_acquisition_status_(nh_.advertiseService(
        std::string(STSRV_G_image_acquisition_status), 
        &StCameraInterface::getImageAcquisitionStatusCallback, this)),
    srv_get_event_acquisition_status_list_(nh_.advertiseService(
        std::string(STSRV_G_event_acquisition_status_list), 
        &StCameraInterface::getEventAcquisitionStatusListCallback, this)),
    srv_get_event_node_status_list_(nh_.advertiseService(
        std::string(STSRV_G_event_node_status_list), 
        &StCameraInterface::getEventNodeStatusListCallback, this)),
    srv_get_chunk_list_(nh_.advertiseService(
        std::string(STSRV_G_chunk_list), 
        &StCameraInterface::getChunkListCallback, this)),
    srv_get_trigger_list_(nh_.advertiseService(
        std::string(STSRV_G_trigger_list), 
        &StCameraInterface::getTriggerListCallback, this)),
    srv_get_enum_list_(nh_.advertiseService(
        std::string(STSRV_G_enum_list), 
        &StCameraInterface::getEnumListCallback, this)),
    srv_get_genicam_node_info_(nh_.advertiseService(
        std::string(STSRV_G_genicam_node_info), 
        &StCameraInterface::getGenICamNodeInfoCallback, this)),
    srv_send_soft_trigger_(nh_.advertiseService(
        std::string(STSRV_send_soft_trigger), 
        &StCameraInterface::sendSoftTriggerCallback, this)),
    srv_get_last_error_(nh_.advertiseService(
        std::string(STSRV_G_last_error), 
        &StCameraInterface::getLastErrorCallback, this)),
    bool_event_system_(false),
    bool_event_interface_(false),
    bool_event_device_(false),
    bool_event_datastream_(false),
    last_error_(0),
    destroyed_(false)
  {
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);
    std::lock_guard<std::mutex> lock2(mtx_event_);
    std::lock_guard<std::mutex> lock3(mtx_chunk_);

    // initialize default_event
    std::string default_event = STMSG_event;
    map_msg_event_.insert(std::pair<std::string, ros::Publisher>(
        default_event, 
        nh_.advertise<omronsentech_camera::Event>(default_event, queue_size)));
    
    // initialize Data stream and register acquisition callback
    tl_ds_.Reset(tl_dev_->CreateIStDataStream(0));
    StApi::RegisterCallback(tl_ds_, *this, 
        &stcamera::StCameraInterface::eventDataStreamCB, (void *)NULL);

    // register event callback for GenTL modules Device, Interface, and System
    StApi::RegisterCallback(tl_dev_, *this,
        &stcamera::StCameraInterface::eventDeviceCB,
        (void *)NULL);
    StApi::RegisterCallback(tl_dev_->GetIStInterface(), *this,
        &stcamera::StCameraInterface::eventInterfaceCB,
        (void *)NULL);
    StApi::RegisterCallback(tl_dev_->GetIStInterface()->GetIStSystem(), *this,
        &stcamera::StCameraInterface::eventSystemCB,
        (void *)NULL);

    // register device lost
    if (tl_dev_->GetIStInterface()->GetIStSystem()->GetStSystemVendor() == 
        StApi::StSystemVendor_Sentech)
    {
      struct StCallback stc;
      stc.topic_name_ = default_event;

      std::string callback_node = "EventDeviceLost";
      GenApi::CNodeMapPtr p(tl_dev_->GetLocalIStPort()->GetINodeMap());
      GenApi::CNodePtr node_callback(p->GetNode(
            GenICam::gcstring(callback_node.c_str())));
      stc.cb_ = StApi::RegisterCallback(node_callback, *this,
          &stcamera::StCameraInterface::eventGenApiNodeCB,
          (void*)(&map_event_localdevice_), GenApi::cbPostInsideLock);
      GenApi::CEnumerationPtr p_event_selector(p->GetNode(
            GenICam::gcstring("EventSelector")));
      GenApi::CEnumEntryPtr p_event_selector_entry(
          p_event_selector->GetEntryByName(GenICam::gcstring("DeviceLost")));
      p_event_selector->SetIntValue(p_event_selector_entry->GetValue());
      GenApi::CEnumerationPtr p_event_notif(p->GetNode(
            GenICam::gcstring("EventNotification")));
      GenApi::CEnumEntryPtr p_event_notif_entry(
          p_event_notif->GetEntryByName(GenICam::gcstring("On")));
      p_event_notif->SetIntValue(p_event_notif_entry->GetValue());
      tl_dev_->StartEventAcquisitionThread();
      map_event_localdevice_[callback_node] = stc;
      bool_event_device_ = true;
    }

    // check if chunk is enabled. 
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();
      GenApi::INode *node_sel = mp->GetNode( GenICam::gcstring("ChunkSelector"));
      GenApi::INode *node_enable = mp->GetNode(GenICam::gcstring("ChunkEnable"));
      GenApi::INode *node_chunk_active = mp->GetNode(
          GenICam::gcstring("ChunkModeActive"));
      if (node_sel && node_enable && node_chunk_active)
      {
        GenApi::CBooleanPtr chunk_active(node_chunk_active);
        GenApi::CEnumerationPtr chunk_selector(node_sel);
        GenApi::NodeList_t nodelist;
        bool revert_chunk_mode = (false == chunk_active->GetValue());
        if (revert_chunk_mode) 
        {
          chunk_active->SetValue(true);
        }
        chunk_selector->GetEntries(nodelist);
        for (GenApi::NodeList_t::iterator it = nodelist.begin();
            it != nodelist.end(); it++)
        {
          if (GenApi::IsAvailable(*it))
          {
            GenApi::CEnumEntryPtr enum_entry(*it);
            std::string chunk_name = enum_entry->GetSymbolic().c_str();
            chunk_selector->SetIntValue(enum_entry->GetValue());
            GenApi::CBooleanPtr chunk_enable(node_enable);
            if (GenApi::IsReadable(chunk_enable) && 
                chunk_enable->GetValue() == true)
            {
              GenICam::gcstring chunk_value_name("Chunk");
              chunk_value_name.append(chunk_name.c_str());
              GenApi::CNodePtr p_chunk_value(mp->GetNode(
                    GenICam::gcstring(chunk_value_name)));
              if (!p_chunk_value) continue;
              MapChunk::iterator itm = map_chunk_.find(p_chunk_value
                  ->GetName().c_str());
              if (itm == map_chunk_.end() || itm->second == nullptr)
              {
                map_chunk_[p_chunk_value->GetName().c_str()] = p_chunk_value;
              }
            }
          }
        }
        if (revert_chunk_mode) 
        {
          chunk_active->SetValue(false);
        }
      }
    }
    catch(GenICam::GenericException &x)
    {
      ROS_ERROR("%s %s %d: \n\t%s GenICam error when checking chunk: %s",
          __FILE__,__func__,__LINE__, 
          camera_namespace_.c_str(), x.GetDescription());
    }
    
    initializeCameraInfo();
    
    // start acquisition
    bool_acquisition_is_started_ = true;
    tl_ds_->StartAcquisition();
    tl_dev_->AcquisitionStart(); 
  }

  StCameraInterface::~StCameraInterface()
  {
    ROS_INFO("Closing device %s", camera_namespace_.c_str());
    destroyed_ = true;
    std::lock_guard<std::mutex> lock(mtx_event_);
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);

    // Deregister all node callback
    MapCallback *cblist[5] = 
    {
      &map_event_system_, 
      &map_event_interface_, 
      &map_event_localdevice_,
      &map_event_remotedevice_, 
      &map_event_datastream_
    }; 
    for (int i = 0; i < 5; i++)
    {
      MapCallback *cb = cblist[i];
      for (MapCallback::iterator it = cb->begin(); it != cb->end(); it++)
      {
        StApi::IStRegisteredCallbackReleasable *cbf = it->second.cb_;
        cbf->Release();
      }
      cb->clear();
    }

    map_msg_event_.clear();

    if (!tl_dev_->IsDeviceLost())
    {
      if (bool_acquisition_is_started_)
      {
        tl_dev_->AcquisitionStop(); 
        tl_ds_->StopAcquisition();
        bool_acquisition_is_started_ = false;
      }
    }
    tl_ds_.Reset(nullptr);
    tl_dev_.Reset(nullptr);
    ROS_INFO("Closing device %s done.", camera_namespace_.c_str());
  }


  bool StCameraInterface::deviceIsLost()
  {
    return (tl_dev_->IsDeviceLost());
  }


  void StCameraInterface::eventSystemCB(StApi::IStCallbackParamBase *p, 
                                        void *pvContext)
  {
    if (destroyed_) return;
    omronsentech_camera::Event msg;
    msg.timestamp = ros::Time::now();
    msg.module_name = "System";
    if (p->GetCallbackType() == StApi::StCallbackType_GenTLEvent_SystemError)
    {
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
    }
    publishEventDefault(msg); 
  }

  void StCameraInterface::eventInterfaceCB(StApi::IStCallbackParamBase *p, 
                                           void *pvContext)
  {
    if (destroyed_) return;
    omronsentech_camera::Event msg;
    msg.timestamp = ros::Time::now();
    msg.module_name = "Interface";
    if (p->GetCallbackType() == StApi::StCallbackType_GenTLEvent_InterfaceError)
    {
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
    }
    publishEventDefault(msg); 
  }

  void StCameraInterface::eventDeviceCB(StApi::IStCallbackParamBase *p, 
                                        void *pvContext)
  {
    if (destroyed_) return;
    if (tl_dev_ && tl_dev_->IsDeviceLost()) return;
    omronsentech_camera::Event msg;
    msg.timestamp = ros::Time::now();
    msg.module_name = "LocalDevice";
    if (p->GetCallbackType() == StApi::StCallbackType_GenTLEvent_DeviceError)
    {
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
    }
    publishEventDefault(msg); 
  }

  void StCameraInterface::eventGenApiNodeCB(GenApi::INode *p, void *pvContext)
  {
    if (destroyed_) return;
    MapCallback *cblist[5] = 
    {
      &map_event_system_, 
      &map_event_interface_, 
      &map_event_localdevice_,
      &map_event_remotedevice_, 
      &map_event_datastream_
    }; 
    std::string cbModule[5] = 
    {
      "System", "Interface", "LocalDevice", "RemoteDevice", "DataStream"
    };
    MapCallback *cb = nullptr;
    int index = -1;
    std::string callback_node = p->GetName().c_str();

    // find the correct module
    for (int i = 0; i < 5; i ++)
    {
      if (pvContext == (void*)(&(*cblist[i])))
      {
        cb = cblist[i];
        index = i;
        break;
      }
    }
    if (-1 == index) return; // not found (shall never happen)

    omronsentech_camera::Event msg;
    msg.timestamp = ros::Time::now();
    msg.module_name = cbModule[index];
    msg.callback_node = callback_node;
    if (GenApi::IsReadable(p))
    {
      GenApi::CValuePtr pValue(p);
      msg.event_data = pValue->ToString();
    }
    MapPublisher::iterator it = map_msg_event_.find(
        ((*cb)[callback_node]).topic_name_);
    if (it != map_msg_event_.end()) // publish to custom topic
    {
      it->second.publish(msg);
      return;
    }
    publishEventDefault(msg);  // publish to default topic
  }

  void StCameraInterface::eventDataStreamCB(StApi::IStCallbackParamBase *p, 
                                            void *pvContext)
  {
    if (destroyed_) return;
    if (p->GetCallbackType() == 
        StApi::StCallbackType_GenTLEvent_DataStreamError)
    {
      omronsentech_camera::Event msg;
      msg.timestamp = ros::Time::now();
      msg.module_name = "DataStream";
      StApi::IStCallbackParamGenTLEventError *e = dynamic_cast<
          StApi::IStCallbackParamGenTLEventError*>(p);
      msg.event_name = "Error";
      msg.event_data = std::to_string(e->GetGCError()) + " " +
          e->GetDescription().c_str();
      publishEventDefault(msg);
      return;
    }
  
    if(p->GetCallbackType() == 
        StApi::StCallbackType_GenTLEvent_DataStreamNewBuffer)
    {
      try
      {
        StApi::CIStStreamBufferPtr p_streambuffer(tl_ds_->RetrieveBuffer(0));

        if (it_campub_.getNumSubscribers() == 0 && 
            msg_chunk_.getNumSubscribers() == 0) 
        {
          return;
        }
   
        omronsentech_camera::Chunk chunkdata;
        bool has_chunk = false;
        if (map_chunk_.size() > 0)
        {
          has_chunk = true;
          GenApi::CNodeMapPtr cmp(p_streambuffer->GetChunkINodeMap());
          for (MapChunk::iterator it = map_chunk_.begin(); 
              it != map_chunk_.end(); it++)
          {
            GenApi::INode *node = it->second;
            if (node == nullptr) continue;
            switch(node->GetPrincipalInterfaceType())
            {
              case GenApi::intfIInteger:
              {
                GenApi::CIntegerPtr ptr(node);
                omronsentech_camera::ElementInt64 data;
                data.data = ptr->GetValue();
                data.name = it->first;
                chunkdata.int64_list.push_back(data);
                break;
              }
              case GenApi::intfIFloat:
              {
                GenApi::CFloatPtr ptr(node);
                omronsentech_camera::ElementFloat64 data;
                data.data = ptr->GetValue();
                data.name = it->first;
                chunkdata.float64_list.push_back(data);
                break; 
              }
              case GenApi::intfIBoolean:
              {
                GenApi::CBooleanPtr ptr(node);
                omronsentech_camera::ElementBool data;
                data.data = ptr->GetValue();
                data.name = it->first;
                chunkdata.bool_list.push_back(data);
                break; 
              }
              case GenApi::intfIString:
              {
                GenApi::CStringPtr ptr(node);
                omronsentech_camera::ElementString data;
                data.data = ptr->GetValue();
                data.name = it->first;
                chunkdata.string_list.push_back(data);
                break; 
              }
              case GenApi::intfIValue:
              {
                GenApi::CValuePtr ptr(node);
                omronsentech_camera::ElementString data;
                data.data = ptr->ToString();
                data.name = it->first;
                chunkdata.string_list.push_back(data);
                break;
              }
            }
          }
        }

        if (p_streambuffer->GetIStStreamBufferInfo()->IsImagePresent())
        {
          StApi::IStImage *p_stimage = p_streambuffer->GetIStImage();
          const StApi::EStPixelFormatNamingConvention_t ePFNC = 
              p_stimage->GetImagePixelFormat();

          std::string encoding = "";
          switch(ePFNC)
          {
            case StApi::StPFNC_BayerBG8: 
              encoding = sensor_msgs::image_encodings::BAYER_BGGR8; 
              break;
            case StApi::StPFNC_BayerBG16: 
              encoding = sensor_msgs::image_encodings::BAYER_BGGR16; 
              break;
            case StApi::StPFNC_BayerGB8: 
              encoding = sensor_msgs::image_encodings::BAYER_GBRG8; 
              break;
            case StApi::StPFNC_BayerGB16: 
              encoding = sensor_msgs::image_encodings::BAYER_GBRG16; 
              break;
            case StApi::StPFNC_BayerRG8: 
              encoding = sensor_msgs::image_encodings::BAYER_RGGB8; 
              break;
            case StApi::StPFNC_BayerRG16: 
              encoding = sensor_msgs::image_encodings::BAYER_RGGB16; 
              break;
            case StApi::StPFNC_BayerGR8: 
              encoding = sensor_msgs::image_encodings::BAYER_GRBG8; 
              break;
            case StApi::StPFNC_BayerGR16: 
              encoding = sensor_msgs::image_encodings::BAYER_GRBG16; 
              break;
            case StApi::StPFNC_Mono8: 
              encoding = sensor_msgs::image_encodings::MONO8; 
              break;
            case StApi::StPFNC_Mono16: 
              encoding = sensor_msgs::image_encodings::MONO16; 
              break;
            case StApi::StPFNC_BGR8:
              encoding = sensor_msgs::image_encodings::BGR8;
              break;
          }
          if (encoding.empty())
          {
            ROS_WARN("%s %s %d: %s: %ld: %ld x %ld: unknown encoding %d",
                __FILE__,__func__,__LINE__,
                camera_namespace_.c_str(),
                p_streambuffer->GetIStStreamBufferInfo()->GetFrameID(),
                p_stimage->GetImageWidth(), 
                p_stimage->GetImageHeight(), 
                (int)ePFNC);
            return;
          }

          StApi::IStPixelFormatInfo *const p_pixelformat_info = 
              StApi::GetIStPixelFormatInfo(ePFNC);
          if (p_pixelformat_info->IsMono() || p_pixelformat_info->IsBayer() || p_pixelformat_info->IsColor())
          {
            //image data
            sensor_msgs::ImagePtr image(new sensor_msgs::Image);
            image->header.stamp = ros::Time((double)p_streambuffer
                ->GetIStStreamBufferInfo()->GetTimestamp()/1000000.0);
            image->width = p_stimage->GetImageWidth();
            image->height = p_stimage->GetImageHeight();
            image->step = p_stimage->GetImageLinePitch();
            image->encoding = encoding;
            image->is_bigendian = (p_streambuffer->GetIStStreamBufferInfo()
                ->GetPixelEndianness() == GenTL::PIXELENDIANNESS_BIG);
            const size_t buffer_size = image->height * image->step;
            image->data.resize(buffer_size);
            memcpy(&image->data[0], p_stimage->GetImageBuffer(), buffer_size);

            //camera info data
            sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(
                cinfo_.getCameraInfo()));
            ci->header.stamp = image->header.stamp;

            if (it_campub_.getNumSubscribers() > 0)
            {
              it_campub_.publish(image, ci);
            }

            if (has_chunk && msg_chunk_.getNumSubscribers() > 0)
            {
              chunkdata.timestamp = image->header.stamp;
              msg_chunk_.publish(chunkdata);
            }
          }
        }
        else
        {
          if (has_chunk && msg_chunk_.getNumSubscribers() > 0)
          {
            chunkdata.timestamp = ros::Time((double)p_streambuffer
                ->GetIStStreamBufferInfo()->GetTimestamp()/1000000.0);
            msg_chunk_.publish(chunkdata);
          }
        }
      }
      catch(const StApi::CStGenTLErrorException &x)
      {
        ROS_ERROR("%s %s %d: \n\t%s GenTL error when retrieving payload: %d %s", 
            __FILE__,__func__,__LINE__, 
            camera_namespace_.c_str(), x.GetError(), x.GetDescription());
      }
      catch(GenICam::GenericException &x)
      {
        ROS_ERROR("%s %s %d: \n\t%s GenICam error when retrieving payload: %s",
            __FILE__,__func__,__LINE__, 
            camera_namespace_.c_str(), x.GetDescription());
      }
    }
  }

  // read node
  bool StCameraInterface::readNodeCallback(
      omronsentech_camera::ReadNode::Request &req,
      omronsentech_camera::ReadNode::Response &res)
  {
    try
    {
      GenApi::INodeMap* mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      res.interface_type = GenApi::GetInterfaceName(node).c_str();
      switch(node->GetPrincipalInterfaceType())
      {
        case GenApi::intfIInteger:
        {
          GenApi::CIntegerPtr ptr(node);
          res.value = ptr->ToString();
          return true;
        }
        case GenApi::intfIBoolean:
        {
          GenApi::CBooleanPtr ptr(node);
          res.value = ptr->ToString();
          return true;
        }
        case GenApi::intfIFloat:
        {
          GenApi::CFloatPtr ptr(node);
          res.value = ptr->ToString();
          return true;
        }
        case GenApi::intfIString:
        {
          GenApi::CStringPtr ptr(node);
          res.value = ptr->GetValue().c_str();
          return true;
        }
        case GenApi::intfIRegister:
        {
          GenApi::CRegisterPtr ptr(node);
          res.value = ptr->ToString();
          return true;
        }
        case GenApi::intfICategory:
        {
          GenApi::CCategoryPtr ptr(node);
          res.value = ptr->ToString();
          return true;
        }
        case GenApi::intfIEnumeration:
        {
          GenApi::CEnumerationPtr ptr(node);
          res.value = ptr->ToString();
          return true;
        }
        default:
        {
          RETURN_ERR(NODE_ERROR, NODE_ERROR_STR);
        }
      }
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::readNodeBoolCallback(
      omronsentech_camera::ReadNodeBool::Request &req,
      omronsentech_camera::ReadNodeBool::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CBooleanPtr ptr(node);
      res.value = ptr->GetValue();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::readNodeEnumCallback(
      omronsentech_camera::ReadNodeEnum::Request &req,
      omronsentech_camera::ReadNodeEnum::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CEnumerationPtr ptr(node);
      res.value_int = ptr->GetIntValue();
      res.value_str = ptr->GetEntry(res.value_int)->GetSymbolic();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::readNodeIntCallback(
      omronsentech_camera::ReadNodeInt::Request &req,
      omronsentech_camera::ReadNodeInt::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CIntegerPtr ptr(node);
      res.value = ptr->GetValue();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::readNodeFloatCallback(
      omronsentech_camera::ReadNodeFloat::Request &req,
      omronsentech_camera::ReadNodeFloat::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CFloatPtr ptr(node);
      res.value = ptr->GetValue();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::readNodeStringCallback(
      omronsentech_camera::ReadNodeString::Request &req,
      omronsentech_camera::ReadNodeString::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CStringPtr ptr(node);
      res.value = ptr->GetValue();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  // write node
  bool StCameraInterface::writeNodeCallback(
      omronsentech_camera::WriteNode::Request &req,
      omronsentech_camera::WriteNode::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      switch(node->GetPrincipalInterfaceType())
      {
        case GenApi::intfIInteger:
        {
          GenApi::CIntegerPtr ptr(node);
          ptr->SetValue(std::stoi(req.value));
          return true;
        }
        case GenApi::intfIBoolean:
        {
          GenApi::CBooleanPtr ptr(node);
          std::string true_str = "true";
          std::string false_str = "false";
          bool true_value = false;
          bool false_value = false;
          if (req.value.size() == true_str.size())
          {
            true_value = true;
            for (std::string::const_iterator c1 = true_str.begin(), 
                c2 = req.value.begin(); c1 != true_str.end(); ++c1, ++c2) 
            {
              if (tolower(*c1) != tolower(*c2)) 
              {
                true_value = false;
                break;
              }
            }
            if (true_value)
            {
              ptr->SetValue(true);
              return true;
            }
          }
          if (req.value.size() == false_str.size())
          {
            false_value = true;
            for (std::string::const_iterator c1 = false_str.begin(), 
                c2 = req.value.begin(); c1 != false_str.end(); ++c1, ++c2) 
            {
              if (tolower(*c1) != tolower(*c2)) 
              {
                false_value = false;
                break;
              }
            }
            if (false_value)
            {
              ptr->SetValue(false);
              return true;
            }
          }
          RETURN_ERR(NODE_ERROR, NODE_ERROR_STR);
        }
        case GenApi::intfIFloat:
        {
          GenApi::CFloatPtr ptr(node);
          ptr->SetValue(std::stod(req.value));
          return true;
        }
        case GenApi::intfIString:
        {
          GenApi::CStringPtr ptr(node);
          ptr->SetValue(GenICam::gcstring(req.value.c_str()));
          return true;
        }
        case GenApi::intfIEnumeration:
        {
          GenApi::CEnumerationPtr ptr(node);
          GenApi::CEnumEntryPtr enum_entry(
              ptr->GetEntryByName(GenICam::gcstring(req.value.c_str())));
          ptr->SetIntValue(enum_entry->GetValue());
          return true;
        }
        default:
        {
          RETURN_ERR(NODE_ERROR, NODE_ERROR_STR);
        }
      }
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::writeNodeBoolCallback(
      omronsentech_camera::WriteNodeBool::Request &req,
      omronsentech_camera::WriteNodeBool::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CBooleanPtr ptr(node);
      ptr->SetValue(req.value);
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::writeNodeEnumIntCallback(
      omronsentech_camera::WriteNodeEnumInt::Request &req,
      omronsentech_camera::WriteNodeEnumInt::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CEnumerationPtr ptr(node);
      ptr->SetIntValue(req.value);
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::writeNodeEnumStrCallback(
      omronsentech_camera::WriteNodeEnumStr::Request &req,
      omronsentech_camera::WriteNodeEnumStr::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CEnumerationPtr ptr(node);
      GenApi::CEnumEntryPtr enum_entry(
          ptr->GetEntryByName(GenICam::gcstring(req.value.c_str())));
      ptr->SetIntValue(enum_entry->GetValue());
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::writeNodeIntCallback(
      omronsentech_camera::WriteNodeInt::Request &req,
      omronsentech_camera::WriteNodeInt::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CIntegerPtr ptr(node);
      ptr->SetValue(req.value);
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::writeNodeFloatCallback(
      omronsentech_camera::WriteNodeFloat::Request &req,
      omronsentech_camera::WriteNodeFloat::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CFloatPtr ptr(node);
      ptr->SetValue(req.value);

      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::writeNodeStringCallback(
      omronsentech_camera::WriteNodeString::Request &req,
      omronsentech_camera::WriteNodeString::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CStringPtr ptr(node);

      const GenICam::gcstring str = req.value.c_str();
      ptr->SetValue(str);

      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::executeNodeCallback(
      omronsentech_camera::ExecuteNode::Request &req,
      omronsentech_camera::ExecuteNode::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CCommandPtr ptr(node);
      ptr->Execute();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  // enable
  bool StCameraInterface::enableChunkCallback(
      omronsentech_camera::EnableChunk::Request &req,
      omronsentech_camera::EnableChunk::Response &res)
  {
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);
    if (bool_acquisition_is_started_)
    {
      RETURN_ERR(ACQ_ALREADY_ON_ERROR, ACQ_ALREADY_ON_ERROR_STR);
    }

    std::lock_guard<std::mutex> lock2(mtx_chunk_);

    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      GenApi::CEnumerationPtr chunk_selector(mp->GetNode(
            GenICam::gcstring("ChunkSelector")));
      GenApi::CBooleanPtr chunk_enable(mp->GetNode(
            GenICam::gcstring("ChunkEnable")));
      GenApi::CBooleanPtr chunk_active(mp->GetNode(
            GenICam::gcstring("ChunkModeActive"))); 
      if (!chunk_selector.IsValid() || !chunk_enable.IsValid() ||
          !chunk_active.IsValid())
      {
        RETURN_ERR(CHUNK_NOT_SUPPORTED_ERROR, CHUNK_NOT_SUPPORTED_ERROR_STR);
      }

      bool revert_chunk_mode = (false == chunk_active->GetValue());
      if (revert_chunk_mode) 
      {
        chunk_active->SetValue(true);
      }

      if (req.chunk_name.empty()) // All chunks
      {
        GenApi::NodeList_t chunk_list;
        chunk_selector->GetEntries(chunk_list);
        for (GenApi::NodeList_t::iterator itr = chunk_list.begin(); 
            itr != chunk_list.end(); ++itr)
        {
          GenApi::CEnumEntryPtr chunk_list_entry(*itr);
          if (GenApi::IsAvailable(chunk_list_entry))
          {
            chunk_selector->SetIntValue(chunk_list_entry->GetValue());
            if (GenApi::IsWritable(chunk_enable))
            {
              chunk_enable->SetValue(req.value);
              GenApi::CNodePtr chunk_value_node(mp->GetNode(
                  "Chunk" + chunk_list_entry->GetSymbolic()));
              if (chunk_value_node && req.value)
              {
                MapChunk::iterator itm = map_chunk_.find(chunk_value_node
                    ->GetName().c_str());
                if (itm == map_chunk_.end() || itm->second == nullptr)
                {
                  map_chunk_[chunk_value_node->GetName().c_str()] = 
                      chunk_value_node;
                }
              }
            }
          }
        }
        if (!req.value) 
        {
          map_chunk_.clear();
          chunk_active->SetValue(false);
        }
      }
      else
      {
        bool is_writable = true;
        bool all_chunk_disabled;
        GenApi::NodeList_t nodelist;
        GenApi::CEnumEntryPtr chunk_list_entry(chunk_selector
            ->GetEntryByName(GenICam::gcstring(req.chunk_name.c_str())));
        if (chunk_list_entry.IsValid() &&
            GenApi::IsReadable(chunk_list_entry))
        {
          chunk_selector->SetIntValue(chunk_list_entry->GetValue());
          if (!GenApi::IsWritable(chunk_enable))
          {
            is_writable = false;
          }
        }

        if (!is_writable)
        {
          if (revert_chunk_mode) 
          {
            chunk_active->SetValue(false);
          }
          RETURN_ERR(CHUNK_NAME_ERROR, CHUNK_NAME_ERROR_STR);
        }

        chunk_enable->SetValue(req.value);

        GenICam::gcstring chunk_value_name("Chunk");
        chunk_value_name.append(req.chunk_name.c_str());
        GenApi::CNodePtr chunk_value_node(mp->GetNode(chunk_value_name));
        if (chunk_value_node)
        {
          MapChunk::iterator itm = map_chunk_.find(chunk_value_node
              ->GetName().c_str());
          if (req.value)
          {
            if (itm == map_chunk_.end() || itm->second == nullptr)
            {
              map_chunk_[chunk_value_node->GetName().c_str()] = 
                  chunk_value_node;
            }
          }
        }
        
        // Disable chunk active if only Image is enabled.
        all_chunk_disabled = true;
        chunk_selector->GetEntries(nodelist);
        for (GenApi::NodeList_t::iterator it = nodelist.begin();
            it != nodelist.end(); it++)
        {
          if (GenApi::IsAvailable(*it))
          {
            GenApi::CEnumEntryPtr enum_entry(*it);
            if (enum_entry->GetValue() == 0) continue; // skip image.
            chunk_selector->SetIntValue(enum_entry->GetValue());
            if (chunk_enable->GetValue() == true)     
            {
              all_chunk_disabled = false;
              break;
            }
          }
        }
        if (all_chunk_disabled)
        {
          chunk_active->SetValue(false);
        }
      }

      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::enableTriggerCallback(
      omronsentech_camera::EnableTrigger::Request &req,
      omronsentech_camera::EnableTrigger::Response &res)
  {
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      // Node for selector and source
      GenApi::CEnumerationPtr trigger_selector(mp->GetNode(
          GenICam::gcstring("TriggerSelector")));
      GenApi::CEnumerationPtr trigger_source(mp->GetNode(
          GenICam::gcstring("TriggerSource")));
      if (!trigger_selector.IsValid() || !trigger_source.IsValid())
      {
        RETURN_ERR(TRIGGER_NOT_SUPPORTED_ERROR,TRIGGER_NOT_SUPPORTED_ERROR_STR); 
      }
      GenApi::CEnumEntryPtr trigger_selector_entry(
          trigger_selector->GetEntryByName(
            GenICam::gcstring(req.trigger_selector.c_str())));
      GenApi::CEnumEntryPtr trigger_source_entry(
          trigger_source->GetEntryByName(
            GenICam::gcstring(req.trigger_source.c_str())));
      if (!trigger_selector_entry.IsValid() || !trigger_source_entry.IsValid())
      {
        RETURN_ERR(TRIGGER_NAME_ERROR,TRIGGER_NAME_ERROR_STR);
      }

      // set selector
      if (trigger_selector->GetIntValue() != trigger_selector_entry->GetValue())
      {
        trigger_selector->SetIntValue(trigger_selector_entry->GetValue());
      }

      // set source
      if (trigger_source->GetIntValue() != trigger_source_entry->GetValue())
      {
        trigger_source->SetIntValue(trigger_source_entry->GetValue());
      }
      
      // set delay only if supported
      GenApi::CFloatPtr trigger_delay = mp->GetNode(
          GenICam::gcstring("TriggerDelay"));
      if (trigger_delay.IsValid())
      {
        if (trigger_delay->GetValue() != req.trigger_delayus)
        {
          trigger_delay->SetValue(req.trigger_delayus);
        }
      }

      // set mode
      GenApi::CEnumerationPtr trigger_mode = mp->GetNode(
          GenICam::gcstring("TriggerMode"));
      if (req.value == true)
      {
        GenApi::CEnumEntryPtr trigger_mode_entry(
            trigger_mode->GetEntryByName(GenICam::gcstring("On")));
        if (trigger_mode->GetIntValue() != trigger_mode_entry->GetValue())
        {
          trigger_mode->SetIntValue(trigger_mode_entry->GetValue());
        }
      }
      else
      {
        GenApi::CEnumEntryPtr trigger_mode_entry(
            trigger_mode->GetEntryByName(GenICam::gcstring("Off")));
        if (trigger_mode->GetIntValue() != trigger_mode_entry->GetValue())
        {
          trigger_mode->SetIntValue(trigger_mode_entry->GetValue());
        }
      }
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::enableEventNodeCallback(
      omronsentech_camera::EnableEventNode::Request &req,
      omronsentech_camera::EnableEventNode::Response &res)
  {
    std::lock_guard<std::mutex> lock(mtx_event_);
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *nodeCallback = mp->GetNode(
          GenICam::gcstring(req.callback_node.c_str()));
      CHECK_NULLPTR(nodeCallback, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CEnumerationPtr event_selector(mp->GetNode(
          GenICam::gcstring("EventSelector")));
      GenApi::CEnumerationPtr event_notif(mp->GetNode(
          GenICam::gcstring("EventNotification"))); 
      if (!event_selector.IsValid() || !event_notif.IsValid() ||
          !GenApi::IsWritable(event_selector))
      {
        RETURN_ERR(EVENT_NOT_SUPPORTED_ERROR,EVENT_NOT_SUPPORTED_ERROR_STR); 
      }

      GenApi::CEnumEntryPtr enum_entry(
          event_selector->GetEntryByName(
            GenICam::gcstring(req.node_name.c_str())));
      if (!enum_entry.IsValid())
      {
        RETURN_ERR(EVENT_NAME_ERROR, EVENT_NAME_ERROR_STR); 
      }
      event_selector->SetIntValue(enum_entry->GetValue());
      GenApi::CNodePtr node_callback(nodeCallback);
      GenICam::gcstring value = req.value ? "On" : "Off";
      GenApi::CEnumEntryPtr event_notif_entry( 
          event_notif->GetEntryByName(value));
      if (!GenApi::IsWritable(event_notif))
      {
        RETURN_ERR(EVENT_NOT_SUPPORTED_ERROR,EVENT_NOT_SUPPORTED_ERROR_STR);
      }
      event_notif->SetIntValue(event_notif_entry->GetValue());

      MapCallback *cb = getCallbackMap(req.module_name);
      MapCallback::iterator it = cb->find(req.callback_node);
      std::string topic_name = STMSG_event;
      if (!req.event_topic_name.empty())
      {
        topic_name += "/" + req.event_topic_name;
      }

      if (req.value) // set enabled = true
      {
        if (it == cb->end()) // not found in registered list
        {
          MapPublisher::iterator itpub = map_msg_event_.find(topic_name);
          if (itpub == map_msg_event_.end())
          {
            map_msg_event_.insert(std::pair<std::string, ros::Publisher>(
                topic_name, 
                nh_.advertise<omronsentech_camera::Event>(topic_name, 
                queue_size_)));
          }
          struct StCallback stc;
          stc.topic_name_ = topic_name;
          stc.cb_ = StApi::RegisterCallback(node_callback, *this,
              &stcamera::StCameraInterface::eventGenApiNodeCB,
              (void*)(&(*cb)), GenApi::cbPostInsideLock);
          (*cb)[req.callback_node] = stc;
          return true;
        }
        // already registered
        RETURN_ERR(EVENT_ALREADY_ON_ERROR, EVENT_ALREADY_ON_ERROR_STR);
      }

      // set enabled = false
      if (it != cb->end()) // found in registered list
      {
        StApi::IStRegisteredCallbackReleasable *cbf = it->second.cb_;
        cbf->Release();
        cb->erase(it);
        if (topic_name.compare(std::string(camera_namespace_ + 
            "/" + STMSG_event)) != 0) // not using default event topic
        {
          // remove publisher if no other event callback is using the topic
          MapCallback *cblist[5] = 
          {
            &map_event_system_, 
            &map_event_interface_, 
            &map_event_localdevice_,
            &map_event_remotedevice_, 
            &map_event_datastream_
          }; 
          for (int i = 0; i < 5; i++)
          {
            for (MapCallback::iterator itcb = (*cblist[i]).begin();
                itcb != (*cblist[i]).end(); itcb++)
            { // if other event is using the topic, do nothing and return.
              if (itcb->second.topic_name_.compare(topic_name) == 0)
              {
                return true; 
              }
            } 
          } 
          MapPublisher::iterator itpub = map_msg_event_.find(topic_name);
          if (itpub != map_msg_event_.end())
          {
            map_msg_event_.erase(itpub);
          } 
        }
        return true;
      }
      // not found in registered list
      RETURN_ERR(EVENT_ALREADY_OFF_ERROR, EVENT_ALREADY_OFF_ERROR_STR);
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::enableImageAcquisitionCallback(
      omronsentech_camera::EnableImageAcquisition::Request &req,
      omronsentech_camera::EnableImageAcquisition::Response &res)
  {
    std::lock_guard<std::mutex> lock(mtx_acquisition_);
    try
    {
      if (req.value == true)
      {
        //already started, return directly
        if (bool_acquisition_is_started_) 
        {
          RETURN_ERR(ACQ_ALREADY_ON_ERROR, ACQ_ALREADY_ON_ERROR_STR);
        }

        initializeCameraInfo();
        tl_ds_->StartAcquisition();
        tl_dev_->AcquisitionStart();
        bool_acquisition_is_started_ = true;
        return true;
      }

      if (bool_acquisition_is_started_)
      {
        tl_dev_->AcquisitionStop();
        tl_ds_->StopAcquisition();
        bool_acquisition_is_started_ = false;
        return true;
      }
      RETURN_ERR(ACQ_ALREADY_OFF_ERROR, ACQ_ALREADY_OFF_ERROR_STR);
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::enableEventAcquisitionCallback(
      omronsentech_camera::EnableEventAcquisition::Request &req,
      omronsentech_camera::EnableEventAcquisition::Response &res)
  {
    std::lock_guard<std::mutex> lock(mtx_event_);
    try
    {
        StApi::IStEventCtrl *event_control = nullptr;
        bool *event_value = nullptr;
        if (req.module_name.compare("System") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(
              tl_dev_->GetIStInterface()->GetIStSystem());
          event_value = &bool_event_system_;
        }
        else if (req.module_name.compare("Interface") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(
              tl_dev_->GetIStInterface());
          event_value = &bool_event_interface_;
        }
        else if (req.module_name.compare("LocalDevice") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(&(*tl_dev_));
              //(StApi::IStDevice*)tl_dev_);
          event_value = &bool_event_device_;
        }
        else if (req.module_name.compare("DataStream") == 0)
        {
          event_control = dynamic_cast<StApi::IStEventCtrl *>(&(*tl_ds_));
          event_value = &bool_event_datastream_;
        }
        else 
        {
          RETURN_ERR(MODULE_ERROR, MODULE_ERROR_STR);
        }
 
        if (event_control != nullptr)
        {
          if (req.value == true) // set enable
          {
            if (req.value == *event_value)
            {
              RETURN_ERR(EVENT_ALREADY_ON_ERROR, EVENT_ALREADY_ON_ERROR_STR);
            }
            event_control->StartEventAcquisitionThread();
            *event_value = true;
            return true;
          }

          // set disable
          if (req.value == *event_value) 
          {
            RETURN_ERR(EVENT_ALREADY_OFF_ERROR, EVENT_ALREADY_OFF_ERROR_STR);
          }
          event_control->StopEventAcquisitionThread();
          *event_value = false;
          return true;
        }
    }
    CATCH_COMMON_ERR();
  }

  // get
  bool StCameraInterface::getImageAcquisitionStatusCallback(
      omronsentech_camera::GetImageAcquisitionStatus::Request &req,
      omronsentech_camera::GetImageAcquisitionStatus::Response &res)
  {
    std::lock_guard<std::mutex> lock(mtx_acquisition_);
    res.value = bool_acquisition_is_started_;
    return true;
  }

  bool StCameraInterface::getEventAcquisitionStatusListCallback(
      omronsentech_camera::GetEventAcquisitionStatusList::Request &req,
      omronsentech_camera::GetEventAcquisitionStatusList::Response &res)
  {
    std::lock_guard<std::mutex> lock(mtx_event_);
    res.module_name_list.push_back("System");
    res.module_name_list.push_back("Interface");
    res.module_name_list.push_back("LocalDevice");
    res.module_name_list.push_back("RemoteDevice");
    res.module_name_list.push_back("DataStream");
    res.enabled_list.push_back(bool_event_system_); 
    res.enabled_list.push_back(bool_event_interface_); 
    res.enabled_list.push_back(bool_event_device_); //Local == Remote
    res.enabled_list.push_back(bool_event_device_); //Remote == Local
    res.enabled_list.push_back(bool_event_datastream_); 
    return true;
  }

  bool StCameraInterface::getEventNodeStatusListCallback(
      omronsentech_camera::GetEventNodeStatusList::Request &req,
      omronsentech_camera::GetEventNodeStatusList::Response &res)
  {
    GenApi::INodeMap *mp = getNodeMap(req.module_name);
    CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

    try
    {
      GenApi::CEnumerationPtr event_selector(mp->GetNode(
          GenICam::gcstring("EventSelector"))); 
      GenApi::CEnumerationPtr event_notif(mp->GetNode(
          GenICam::gcstring("EventNotification")));
      if (!event_selector.IsValid() || !event_notif.IsValid())
      {
        RETURN_ERR(EVENT_NOT_SUPPORTED_ERROR, EVENT_NOT_SUPPORTED_ERROR_STR);
      }
      if (GenApi::IsReadable(event_selector))
      {
        GenApi::NodeList_t nodelist;
        event_selector->GetEntries(nodelist);
        for (GenApi::NodeList_t::iterator it = nodelist.begin();
            it != nodelist.end(); it++)
        {
          if (!GenApi::IsAvailable(*it)) continue;

          GenApi::CEnumEntryPtr enum_entry(*it);
          event_selector->SetIntValue(enum_entry->GetValue());
          omronsentech_camera::GenICamEvent genicam_event;
          genicam_event.name = std::string(enum_entry->GetSymbolic().c_str());
          genicam_event.enabled = (event_notif->GetCurrentEntry()
              ->GetSymbolic().compare("On") == 0 ? true : false);

          //list the callback node:
          std::string nodeData = "Event" + genicam_event.name + "Data";
          GenApi::CNodePtr event_category_node(mp->GetNode(
                GenICam::gcstring(nodeData.c_str())));
          if (event_category_node.IsValid())
          {
            MapCallback *cb = getCallbackMap(req.module_name);
            GenApi::FeatureList_t features;
            GenApi::CCategoryPtr event_category(event_category_node);
            event_category->GetFeatures(features);
            for (GenApi::FeatureList_t::iterator it = features.begin();
                it != features.end(); it++)
            {
              GenApi::INode *node = (*it)->GetNode();
              if (!GenApi::IsImplemented(node)) continue;
              std::string cb_name = node->GetName().c_str();
              genicam_event.callback_node_list.push_back(cb_name.c_str());
              MapCallback::iterator itmap = cb->find(cb_name);
              if (itmap == cb->end() || itmap->second.cb_ == nullptr) 
              {
                genicam_event.callback_enabled_list.push_back(false);
              }
              else
              {
                genicam_event.callback_enabled_list.push_back(true);
              }
            }
          }
          res.event_node_list.push_back(genicam_event);
        }
      }
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::getChunkListCallback(
      omronsentech_camera::GetChunkList::Request &req,
      omronsentech_camera::GetChunkList::Response &res)
  {
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);
    if (bool_acquisition_is_started_)
    {
      RETURN_ERR(ACQ_ALREADY_ON_ERROR,ACQ_ALREADY_ON_ERROR_STR);
    }

    std::lock_guard<std::mutex> lock2(mtx_chunk_);
    GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();
    try
    {
      GenApi::CEnumerationPtr chunk_selector = mp->GetNode(
          GenICam::gcstring("ChunkSelector"));
      GenApi::NodeList_t nodelist;
      if (!chunk_selector.IsValid())
      {
        RETURN_ERR(CHUNK_NOT_SUPPORTED_ERROR,CHUNK_NOT_SUPPORTED_ERROR_STR);
      }

      GenApi::CBooleanPtr chunk_active = mp->GetNode(
          GenICam::gcstring("ChunkModeActive"));
      bool revert_chunk_mode = (false == chunk_active->GetValue());
      if (revert_chunk_mode) 
      {
        chunk_active->SetValue(true);
      }
      chunk_selector->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
          it != nodelist.end(); it++)
      {
        if (GenApi::IsAvailable(*it))
        {
          GenApi::CEnumEntryPtr enum_entry(*it);
          res.chunk_name_list.push_back(
              std::string(enum_entry->GetSymbolic().c_str()));
          chunk_selector->SetIntValue(enum_entry->GetValue());
          GenApi::CBooleanPtr enablePtr = mp->GetNode(
              GenICam::gcstring("ChunkEnable"));
          if (enablePtr.IsValid())
          {
            res.chunk_enabled_list.push_back(
                enablePtr->GetValue() ? true : false);
          }
        }
      }
      if (revert_chunk_mode) 
      {
        chunk_active->SetValue(false);
      }

      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::getTriggerListCallback(
      omronsentech_camera::GetTriggerList::Request &req,
      omronsentech_camera::GetTriggerList::Response &res)
  {
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      // Node for selector and source
      GenApi::CEnumerationPtr enum_sel = mp->GetNode(
          GenICam::gcstring("TriggerSelector"));
      GenApi::CEnumerationPtr enum_src = mp->GetNode(
          GenICam::gcstring("TriggerSource"));
      if (!enum_sel.IsValid() || !enum_src.IsValid())
      {
        RETURN_ERR(EVENT_NOT_SUPPORTED_ERROR, EVENT_NOT_SUPPORTED_ERROR_STR);
      }
      
      // Iterate selector
      GenApi::NodeList_t nodelist;
      enum_sel->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
          it != nodelist.end(); it++)
      {
        if (!GenApi::IsAvailable(*it)) continue;
        // Get selector
        GenApi::CEnumEntryPtr enum_entry(*it);
        res.trigger_selector_list.push_back(
            std::string(enum_entry->GetSymbolic().c_str()));
        enum_sel->SetIntValue(enum_entry->GetValue());

        // Get mode
        GenApi::CEnumerationPtr trigger_mode = mp->GetNode(
            GenICam::gcstring("TriggerMode"));
        if (trigger_mode.IsValid())
        {
          res.trigger_mode_list.push_back(trigger_mode->GetCurrentEntry()
              ->GetSymbolic().compare("On") == 0 ? true : false);
        }
        else
        {
          res.trigger_mode_list.push_back(false);
        }

        // Get delayus
        GenApi::CFloatPtr trigger_delay = mp->GetNode(
            GenICam::gcstring("TriggerDelay"));
        if (trigger_mode.IsValid() && GenApi::IsReadable(trigger_delay))
        {
          res.trigger_delayus_list.push_back(trigger_delay->GetValue());
        }
        else
        {
          res.trigger_delayus_list.push_back(0);
        }
      }

      // Iterate source
      nodelist.clear();
      enum_src->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
        it != nodelist.end(); it++)
      {
        if (!GenApi::IsAvailable(*it)) continue;
        // Get source
        GenApi::CEnumEntryPtr enum_entry(*it);
        res.trigger_source_list.push_back(
            std::string(enum_entry->GetSymbolic().c_str()));
      }
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::getEnumListCallback(
      omronsentech_camera::GetEnumList::Request &req,
      omronsentech_camera::GetEnumList::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str())); 
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);

      GenApi::CEnumerationPtr ptr(node);
      GenApi::NodeList_t nodelist;
      ptr->GetEntries(nodelist);
      for (GenApi::NodeList_t::iterator it = nodelist.begin();
          it != nodelist.end(); it++)
      {
        if (GenApi::IsAvailable(*it))
        {
          GenApi::CEnumEntryPtr enum_entry(*it);
          res.enum_value_int_list.push_back(enum_entry->GetValue());
          res.enum_value_str_list.push_back(
              std::string(enum_entry->GetSymbolic().c_str()));
        }
      }
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::getGenICamNodeInfoCallback(
      omronsentech_camera::GetGenICamNodeInfo::Request &req,
      omronsentech_camera::GetGenICamNodeInfo::Response &res)
  {
    try
    {
      GenApi::INodeMap *mp = getNodeMap(req.module_name);
      CHECK_NULLPTR(mp, MODULE_ERROR, MODULE_ERROR_STR);

      GenApi::INode *node = mp->GetNode(
          GenICam::gcstring(req.node_name.c_str()));
      CHECK_NULLPTR(node, NODE_ERROR, NODE_ERROR_STR);
      
      //string name
      res.name = node->GetName().c_str();

      //string description
      res.description = node->GetDescription().c_str();

      //string name_space
      GenApi::ENameSpace genicam_namespace = node->GetNameSpace();
      switch(genicam_namespace)
      {
        case(GenApi::Custom): 
          res.name_space = "Custom";
          break;
        case(GenApi::Standard):  
          res.name_space= "Standard"; 
          break;
        case(GenApi::_UndefinedNameSpace): 
        default:
          res.name_space = "";
          break;
      }

      //string interface_type
      res.interface_type = GenApi::GetInterfaceName(node).c_str();

      //string access_mode
      GenApi::EAccessMode genicam_accessmode = node->GetAccessMode();
      switch(genicam_accessmode)
      {
        case(GenApi::NI):  
          res.access_mode = "Not implemented";
          break;
        case(GenApi::NA):  
          res.access_mode = "Not available";
          break;
        case(GenApi::WO):  
          res.access_mode = "Write Only";
          break;
        case(GenApi::RO):  
          res.access_mode = "Read Only";
          break;
        case(GenApi::RW):  
          res.access_mode = "Read and Write";
          break;
        case(GenApi::_CycleDetectAccesMode):
          res.access_mode = "used internally for AccessMode cycle detection"; 
          break;
        case(GenApi::_UndefinedAccesMode):
        default:
          res.access_mode = "";
          break;
      }

      //string is_cachable
      GenApi::EYesNo genicam_accessmode_cacheable = 
          node->IsAccessModeCacheable();
      switch(genicam_accessmode_cacheable)
      {
        case(GenApi::Yes):  
          res.is_cachable = "Yes";
          break;
        case(GenApi::No):  
          res.is_cachable = "No";
          break;
        case(GenApi::_UndefinedYesNo):
        default:
          res.is_cachable = "";
          break;
      }
  
      //string visibility
      GenApi::EVisibility genicam_visibility = node->GetVisibility();
      switch(genicam_visibility)
      {
        case(GenApi::Beginner):  
          res.visibility = "Beginner";
          break;
        case(GenApi::Expert):  
          res.visibility = "Expert";
          break;
        case(GenApi::Guru):  
          res.visibility = "Guru";
          break;
        case(GenApi::Invisible):
          res.visibility = "Invisible";
          break;
        case(GenApi::_UndefinedVisibility):
        default:
          res.visibility = "";
      }

      //string caching_mode
      GenApi::ECachingMode genicam_caching_mode = node->GetCachingMode();
      switch(genicam_caching_mode)
      {
        case(GenApi::NoCache):
          res.caching_mode = "Does not use cache";
          break;
        case(GenApi::WriteThrough):  
          res.caching_mode = "Write to cache and register"; 
          break;
        case(GenApi::WriteAround):
          res.caching_mode = "Write to register, write to cache on read"; 
          break;
        case(GenApi::_UndefinedCachingMode):
        default:
          res.caching_mode = "";
      }

      //bool is_streamable
      res.is_streamable = node->IsStreamable() ? true : false;
    
      if (genicam_accessmode == GenApi::NA)
      {
        return true;
      }

      if (genicam_accessmode == GenApi::RW || genicam_accessmode == GenApi::RO)
      {
        switch(node->GetPrincipalInterfaceType())
        {
          case GenApi::intfIValue:
          {
            GenApi::CValuePtr ptr(node);
            res.current_value = ptr->ToString();
            break;
          }
          case GenApi::intfIInteger:
          {
            GenApi::CIntegerPtr ptr(node);
            res.current_value = ptr->ToString();
            res.min_value = std::to_string(ptr->GetMin());
            res.max_value = std::to_string(ptr->GetMax());
            res.unit = ptr->GetUnit().c_str();
            try
            {
              res.increment = std::to_string(ptr->GetInc());
            }
            catch(...)
            {
              res.increment = "";
            }
            break;
          }
          case GenApi::intfIBoolean:
          {
            GenApi::CBooleanPtr ptr(node);
            res.current_value = ptr->ToString();
            break;
          }
          case GenApi::intfICommand:
          {
            break;
          }
          case GenApi::intfIFloat:
          {
            GenApi::CFloatPtr ptr(node);
            res.current_value = ptr->ToString();
            res.min_value = std::to_string(ptr->GetMin());
            res.max_value = std::to_string(ptr->GetMax());
            res.unit = ptr->GetUnit().c_str();
            break;
          }
          case GenApi::intfIString:
          {
            GenApi::CStringPtr ptr(node);
            res.current_value = ptr->GetValue().c_str();
            break;
          }
          case GenApi::intfIRegister:
          {
            GenApi::CRegisterPtr ptr(node);
            res.current_value = ptr->ToString();
            break;
          }
          case GenApi::intfICategory:
          {
            GenApi::CCategoryPtr ptr(node);
            res.current_value = ptr->ToString();
            GenApi::FeatureList_t features;
            ptr->GetFeatures(features);
            for (GenApi::FeatureList_t::iterator it = features.begin();
                it != features.end(); it++)
            {
              GenApi::INode *node = (*it)->GetNode();
              if (!GenApi::IsImplemented(node)) continue;
              res.child_node_list.push_back(node->GetName().c_str());
            }
            break;
          }
          case GenApi::intfIEnumeration:
          {
            GenApi::CEnumerationPtr ptr(node);
            res.current_value = ptr->ToString();
            GenApi::NodeList_t nodelist;
            ptr->GetEntries(nodelist);
            for (GenApi::NodeList_t::iterator it = nodelist.begin();
                it != nodelist.end(); it++)
            {
              if (GenApi::IsAvailable(*it))
              {
                GenApi::CEnumEntryPtr enum_entry(*it);
                res.enum_value_int_list.push_back(enum_entry->GetValue());
                res.enum_value_str_list.push_back(
                    std::string(enum_entry->GetSymbolic().c_str()));
              }
            }
            break;
          }
          case GenApi::intfIEnumEntry:
          {
            GenApi::CEnumEntryPtr ptr(node);
            res.current_value = ptr->ToString();
            break;
          }
          case GenApi::intfIPort:
          {
            break;
          }
          default:
          {
            RETURN_ERR(NODE_ERROR, NODE_ERROR_STR);
          }
        }
      }
      return true;
    }
    CATCH_COMMON_ERR();
  }
 
  bool StCameraInterface::sendSoftTriggerCallback(
      omronsentech_camera::SendSoftTrigger::Request &req,
      omronsentech_camera::SendSoftTrigger::Response &res)
  {
    try
    {
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      //set selector
      GenApi::CEnumerationPtr enum_node = mp->GetNode(
          GenICam::gcstring("TriggerSelector"));
      GenApi::CEnumEntryPtr enum_entry(enum_node->GetEntryByName(
            GenICam::gcstring(req.trigger_selector.c_str())));
      if (enum_node->GetIntValue() != enum_entry->GetValue())
      {
        enum_node->SetIntValue(enum_entry->GetValue());
      }
      
      //send  
      GenApi::CCommandPtr cmd = mp->GetNode(
          GenICam::gcstring("TriggerSoftware"));
      cmd->Execute();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterface::getLastErrorCallback(
      omronsentech_camera::GetLastError::Request &req,
      omronsentech_camera::GetLastError::Response &res)
  {
    res.error_code = last_error_;
    switch(last_error_)
    {
      case 0:
        res.description = "No error.";
        return true;
      case UNKNOWN_ERROR:
        res.description = "Unknown exception occured.";
        return true;
      case GENICAM_ERROR:
        res.description = "Error occured when accessing GenICam code. If image acquisition is enabled, try to disable image acquisition before accessing the node.";
        return true;
      case MODULE_ERROR:
        res.description = "Module name is invalid. Check the list of "
            "module name by calling service 'get_module_list'";
        return true;
      case NODE_ERROR:
        res.description = "Either GenICam node name is invalid or "
            "the node is inaccessible.";
        return true;
      case EVENT_ALREADY_ON_ERROR:
        res.description = "Event already started.";
        return true;
      case EVENT_ALREADY_OFF_ERROR:
        res.description = "Event already terminated.";
        return true;
      case ACQ_ALREADY_ON_ERROR: 
        res.description = "Acquisition already started so that the requested "
            "operation cannot be processed.";
        return true;
      case ACQ_ALREADY_OFF_ERROR: 
        res.description = "Acquisition already disabled so that the requested "
            "operation cannot be processed.";
        return true;
      case CHUNK_NOT_SUPPORTED_ERROR:
        res.description = "This device does not support chunk.";
        return true;
      case CHUNK_NAME_ERROR:
        res.description = "The given chunk name does not exist or not "
            "available.";
        return true;
      case TRIGGER_NOT_SUPPORTED_ERROR:
        res.description = "Either trigger is not supported OR the GenICam node is non-standard. Please directly access the GenICam node.";
        return true;
      case TRIGGER_NAME_ERROR:
        res.description = "The given trigger selector and/or the source " 
            "does not exist or not available.";
        return true;
      case EVENT_NOT_SUPPORTED_ERROR:
        res.description = "Either Event callback is not supported or "
            "not available.";
        return true;
      case EVENT_NAME_ERROR:
        res.description = "The given event name and/or callback node "
            "does not exist or not available.";
        return true;
    }

    if ((last_error_ < -1000 && last_error_ > -1023) ||
        last_error_ <= -10000)
    {
      res.description = "GenTL error code. "
                        "Please refer to GenTL documentation.";
      return true;
    }
    res.description = "Error code is unknown.";
    return true;
  }


  void StCameraInterface::initializeCameraInfo()
  {
    // camera name
    if (!cinfo_.setCameraName(camera_namespace_))
    {
      ROS_WARN("Unable to set camera name %s", camera_namespace_.c_str());
    }

    // calibration file
    std::string calibration_file = param_->getCalibrationFile(nh_, 
        camera_namespace_);
    if (!calibration_file.empty())
    {
      ROS_INFO("Calib file %s: %s",camera_namespace_.c_str(),
          calibration_file.c_str());
      if (cinfo_.validateURL(calibration_file))
      {
        cinfo_.loadCameraInfo(calibration_file);
      }
      else
      {
        ROS_WARN("%s Invalid calibration file %s", 
            camera_namespace_.c_str(), calibration_file.c_str());
      }
    }
    else
    {
      ROS_WARN("%s No calibration file found", camera_namespace_.c_str());
    }
        
    try
    {
      sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(
            cinfo_.getCameraInfo()));
      GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();

      // frame id
      cam_info->header.frame_id = camera_namespace_;
      
      // binning x
      GenApi::CIntegerPtr bin_h_val(mp->GetNode(
            GenICam::gcstring("DecimationHorizontal")));
      cam_info->binning_x = bin_h_val->GetValue();

      // binning y
      GenApi::CIntegerPtr bin_v_val(mp->GetNode(
            GenICam::gcstring("DecimationVertical")));
      cam_info->binning_y = bin_v_val->GetValue();

      // width
      GenApi::CIntegerPtr width_max_val(mp->GetNode(
            GenICam::gcstring("WidthMax")));
      cam_info->width = width_max_val->GetValue();

      // height
      GenApi::CIntegerPtr height_max_val(mp->GetNode(
            GenICam::gcstring("HeightMax")));
      cam_info->height = height_max_val->GetValue();

      // ROI
      GenApi::CNodePtr roi_node(mp->GetNode(
            GenICam::gcstring("RegionSelector")));
      if (GenApi::IsWritable(roi_node))
      {
        GenApi::CEnumerationPtr roi_val(roi_node);
        GenApi::CEnumEntryPtr roi_entry(roi_val->GetEntryByName(
              GenICam::gcstring("Region0")));
        roi_val->SetIntValue(roi_entry->GetValue());
        GenApi::CIntegerPtr width(mp->GetNode(GenICam::gcstring("Width")));
        GenApi::CIntegerPtr height(mp->GetNode(GenICam::gcstring("Height")));
        GenApi::CIntegerPtr offsetx(mp->GetNode(GenICam::gcstring("OffsetX")));
        GenApi::CIntegerPtr offsety(mp->GetNode(GenICam::gcstring("OffsetY")));
        if (GenApi::IsReadable(width))
        {
          cam_info->roi.x_offset = offsetx->GetValue();
          cam_info->roi.y_offset = offsety->GetValue();
          if (width->GetValue() != cam_info->width ||
              height->GetValue() != cam_info->height)
          {
            cam_info->roi.width = width->GetValue();
            cam_info->roi.height = height->GetValue();
          }
        }
        cam_info->roi.do_rectify = 
          (cam_info->roi.width > 0 && cam_info->roi.width < cam_info->width) ||
          (cam_info->roi.height > 0 && cam_info->roi.height < cam_info->height);
      }
      cinfo_.setCameraInfo(*cam_info);
    }
    catch(...)
    {
    }
  }

  GenApi::INodeMap *StCameraInterface::getNodeMap(std::string &module_name)
  {
    if (module_name.compare("System") == 0)
    {
      return tl_dev_->GetIStInterface()->GetIStSystem()->GetIStPort()
        ->GetINodeMap();
    }
    if (module_name.compare("Interface") == 0)
    {
      return tl_dev_->GetIStInterface()->GetIStPort()->GetINodeMap();
    }
    if (module_name.compare("LocalDevice") == 0)
    {
      return tl_dev_->GetLocalIStPort()->GetINodeMap();
    }
    if (module_name.compare("RemoteDevice") == 0)
    {
      return tl_dev_->GetRemoteIStPort()->GetINodeMap();
    }
    if (module_name.compare("DataStream") == 0)
    {
      return tl_ds_->GetIStPort()->GetINodeMap();
    }
    return nullptr;
  }

  MapCallback *StCameraInterface::getCallbackMap(std::string &module_name)
  {
    if (module_name.compare("System") == 0)
    {
      return &map_event_system_;
    }
    if (module_name.compare("Interface") == 0)
    {
      return &map_event_interface_;
    }
    if (module_name.compare("LocalDevice") == 0)
    {
      return &map_event_localdevice_;
    }
    if (module_name.compare("RemoteDevice") == 0)
    {
      return &map_event_remotedevice_;
    }
    if (module_name.compare("DataStream") == 0)
    {
      return &map_event_datastream_;
    }
    return nullptr;
  }

  void StCameraInterface::publishEventDefault(omronsentech_camera::Event &msg)
  {
    std::string default_event = STMSG_event;
    MapPublisher::iterator it = map_msg_event_.find(default_event);
    if (it == map_msg_event_.end())
    {
      map_msg_event_.insert(std::pair<std::string, ros::Publisher>(
        default_event, 
        nh_.advertise<omronsentech_camera::Event>(default_event, 
            queue_size_)));
      map_msg_event_[default_event].publish(msg);
    }
    else
    {
      it->second.publish(msg);
    }
  }

} // end of namespace stcamera

