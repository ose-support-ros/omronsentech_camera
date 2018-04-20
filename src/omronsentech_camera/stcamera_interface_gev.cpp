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

#include "omronsentech_camera/stcamera_interface_gev.h"

namespace stcamera
{
  StCameraInterfaceGEV::StCameraInterfaceGEV(
      StApi::IStDeviceReleasable *dev,
      ros::NodeHandle nh_parent, 
      const std::string &camera_namespace, 
      StParameter *param,
      uint32_t queue_size):
    StCameraInterface(dev, nh_parent, camera_namespace, param,
        queue_size),
    srv_get_gige_ip_(nh_.advertiseService(std::string(STSRV_G_gige_ip), 
        &StCameraInterfaceGEV::getIPCallback, this)),
    srv_set_gige_ip_(nh_.advertiseService(std::string(STSRV_S_gige_ip), 
        &StCameraInterfaceGEV::setIPCallback, this))
  {
  }

  StCameraInterfaceGEV::~StCameraInterfaceGEV()
  {
  }

  bool StCameraInterfaceGEV::getIPCallback(
      omronsentech_camera::GetGigEIP::Request &req,
      omronsentech_camera::GetGigEIP::Response &res)
  {
    try
    {
      GenApi::CNodeMapPtr p_nodemap_iface(
          tl_dev_->GetIStInterface()->GetIStPort()->GetINodeMap());
      GenApi::CIntegerPtr p_integer_ip_iface(
        p_nodemap_iface->GetNode("GevInterfaceSubnetIPAddress"));
      GenApi::CIntegerPtr p_integer_subnet_iface(
          p_nodemap_iface->GetNode("GevInterfaceSubnetMask"));
      GenApi::CIntegerPtr p_integer_gw_iface(
          p_nodemap_iface->GetNode("GevInterfaceGateway"));
      res.if_ip_address = p_integer_ip_iface->ToString();
      res.if_ip_mask = p_integer_subnet_iface->ToString();
      res.if_ip_gateway = p_integer_gw_iface->ToString();


      GenApi::CNodeMapPtr p_nodemap_dev(
          tl_dev_->GetLocalIStPort()->GetINodeMap());
      GenApi::CIntegerPtr p_integer_ip_dev(
          p_nodemap_dev->GetNode("GevDeviceIPAddress"));
      GenApi::CIntegerPtr p_integer_subnet_dev(
          p_nodemap_dev->GetNode("GevDeviceSubnetMask"));
      GenApi::CIntegerPtr p_integer_gw_dev(
          p_nodemap_dev->GetNode("GevDeviceGateway"));
      res.dev_ip_address = p_integer_ip_dev->ToString();
      res.dev_ip_mask = p_integer_subnet_dev->ToString();
      res.dev_ip_gateway = p_integer_gw_dev->ToString();
      return true;
    }
    CATCH_COMMON_ERR();
  }

  bool StCameraInterfaceGEV::setIPCallback(
      omronsentech_camera::SetGigEIP::Request &req,
      omronsentech_camera::SetGigEIP::Response &res)
  {
    std::lock_guard<std::mutex> lock1(mtx_acquisition_);
    if (bool_acquisition_is_started_)
    {
      CHECK_NULLPTR(nullptr, ACQ_ALREADY_ON_ERROR, ACQ_ALREADY_ON_ERROR_STR);
    }
    try
    {
      StApi::IStInterface *p_iface = tl_dev_->GetIStInterface();
      if (p_iface->GetIStInterfaceInfo()->GetTLType().compare(TLTypeGEVName) != 
          0) 
      {
        last_error_ = GenTL::GC_ERR_INVALID_ID;
        return false;
      }

      GenApi::CNodeMapPtr p_nodemap(p_iface->GetIStPort()->GetINodeMap());
      GenApi::CIntegerPtr p_dev_selector(p_nodemap->GetNode("DeviceSelector"));
      GenICam::gcstring strMyID = tl_dev_->GetIStDeviceInfo()->GetID();
      const int64_t max_index = p_dev_selector->GetMax();
      for (int64_t k = 0; k <= max_index; k++)
      {
        p_dev_selector->SetValue(k);
        GenApi::CStringPtr p_device_id(p_nodemap->GetNode("DeviceID"));
        GenICam::gcstring strDeviceID = p_device_id->GetValue();
        
        if (strDeviceID.compare(strMyID.c_str()) != 0) continue;

        // deregister current callback callback
        MapCallback *cblist[3] = 
        {
          &map_event_localdevice_,
          &map_event_remotedevice_, 
          &map_event_datastream_
        }; 
        for (int i = 0; i < 3; i++)
        {
          MapCallback *cb = cblist[i];
          for (MapCallback::iterator it = cb->begin(); it != cb->end(); it++)
          {
            StApi::IStRegisteredCallbackReleasable *cbf = it->second.cb_;
            cbf->Release();
            MapPublisher::iterator itpub = 
                map_msg_event_.find(it->second.topic_name_);
            if (itpub != map_msg_event_.end())
            {
              map_msg_event_.erase(itpub);
            }
          }
          cb->clear();
        }

        // delete datastream and device 
        tl_ds_.Reset(NULL);
        tl_dev_.Reset(NULL);
        
        // update IP
        GenApi::CIntegerPtr p_integer_ip(
            p_nodemap->GetNode("GevDeviceForceIPAddress"));
        GenApi::CIntegerPtr p_integer_subnet(
            p_nodemap->GetNode("GevDeviceForceSubnetMask"));
        GenApi::CIntegerPtr p_integer_gw(
            p_nodemap->GetNode("GevDeviceForceGateway"));
        GenApi::CCommandPtr p_command(
            p_nodemap->GetNode("GevDeviceForceIP"));

        p_integer_ip->SetValue(ntohl(inet_addr(req.ip_address.c_str())));
        p_integer_subnet->SetValue(ntohl(inet_addr(req.ip_mask.c_str())));
        p_integer_gw->SetValue(ntohl(inet_addr(req.ip_gateway.c_str())));
        p_command->Execute();

        p_iface->UpdateDeviceList();

        // re-open device 
        tl_dev_.Reset(p_iface->CreateIStDevice(strMyID,
            GenTL::DEVICE_ACCESS_CONTROL));

        // re-open datastream module
        bool_event_datastream_ = false;
        tl_ds_.Reset(tl_dev_->CreateIStDataStream(0));
        StApi::RegisterCallback(tl_ds_, *this, 
            &stcamera::StCameraInterfaceGEV::eventDataStreamCB, (void *)NULL);

        // re-register only device lost
        if (tl_dev_->GetIStInterface()->GetIStSystem()->GetStSystemVendor() == 
            StApi::StSystemVendor_Sentech)
        {
          struct StCallback stc;
          stc.topic_name_ = STMSG_event;

          std::string callback_node = "EventDeviceLost";
          GenApi::CNodeMapPtr p(tl_dev_->GetLocalIStPort()->GetINodeMap());
          GenApi::CNodePtr node_callback(p->GetNode(callback_node.c_str()));
          stc.cb_ = StApi::RegisterCallback(node_callback, *this,
              &stcamera::StCameraInterfaceGEV::eventGenApiNodeCB,
              (void*)(&map_event_localdevice_), GenApi::cbPostInsideLock);
          GenApi::CEnumerationPtr p_event_selector(p->GetNode("EventSelector"));
          GenApi::CEnumEntryPtr p_event_selector_entry(
              p_event_selector->GetEntryByName("DeviceLost"));
          p_event_selector->SetIntValue(p_event_selector_entry->GetValue());
          GenApi::CEnumerationPtr p_event_notif(
              p->GetNode("EventNotification"));
          GenApi::CEnumEntryPtr p_event_notif_entry(
              p_event_notif->GetEntryByName("On"));
          p_event_notif->SetIntValue(p_event_notif_entry->GetValue());
          tl_dev_->StartEventAcquisitionThread();
          map_event_localdevice_[callback_node] = stc;
          bool_event_device_ = true;
        }

        // check if chunk is enabled. 
        GenApi::CNodeMapPtr mp = tl_dev_->GetRemoteIStPort()->GetINodeMap();
        GenApi::INode *node_sel = mp->GetNode("ChunkSelector");
        GenApi::INode *node_enable = mp->GetNode("ChunkEnable");
        if (node_sel && node_enable)
        {
          GenApi::CEnumerationPtr chunk_selector(node_sel);
          GenApi::NodeList_t nodelist;
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
                GenApi::CNodePtr p_chunk_value(mp->GetNode(chunk_value_name));
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
        }
        ROS_INFO("IP Address of the GigE Camera has been updated. " 
            "You have to re-register any custom callback event.");

        return true;
      }
    }
    CATCH_COMMON_ERR();
  }

} // end of namespace stcamera
