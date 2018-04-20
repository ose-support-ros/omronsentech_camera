/******************************************************************************
 *
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

#include "omronsentech_camera/stcamera_node.h"
#include "omronsentech_camera/stcamera_interface_u3v.h"
#include "omronsentech_camera/stcamera_interface_gev.h"
#include <omronsentech_camera/DeviceConnection.h>

#include "omronsentech_camera/stheader.h"

namespace stcamera
{
/** Macro for common exception. */
#define CATCH_GIGECALL_ERR() catch(const StApi::CStGenTLErrorException &x) \
  {\
    ROS_ERROR("%s %s %d: \n\tGenTL error: %d %s", \
        __FILE__,__func__,__LINE__, \
        x.GetError(), x.GetDescription());\
    return false; \
  }\
  catch(GenICam::GenericException &x)\
  {\
    ROS_ERROR("%s %s %d: \n\tGenICam error: %s", \
        __FILE__,__func__,__LINE__, \
        x.GetDescription());\
    return false; \
  }


  StCameraNode::StCameraNode():
    nh_("~"),
    msg_device_connection_(nh_.advertise<omronsentech_camera::DeviceConnection>(
        std::string(STMSG_device_connection), STCAMERA_QUEUE_SIZE)),
    srv_get_device_list_(nh_.advertiseService(
        std::string(STSRV_G_device_list),
        &StCameraNode::getDeviceListCallback, this)),
    srv_get_module_list_(nh_.advertiseService(
        std::string(STSRV_G_module_list),
        &StCameraNode::getModuleListCallback, this)),
    srv_get_sdk_info_(nh_.advertiseService(
        std::string(STSRV_G_sdk_info),
        &StCameraNode::getSDKInfoCallback, this)),
    srv_get_gige_ip_i_(nh_.advertiseService(
        std::string(STSRV_G_gige_ip),
        &StCameraNode::getGigEIPCallback, this)),
    srv_set_gige_ip_i_(nh_.advertiseService(
        std::string(STSRV_S_gige_ip),
        &StCameraNode::setGigEIPCallback, this))
  {
    init();
  }

  StCameraNode::StCameraNode(ros::NodeHandle nh):
    nh_(nh),
    msg_device_connection_(nh_.advertise<omronsentech_camera::DeviceConnection>(
        std::string(STMSG_device_connection), STCAMERA_QUEUE_SIZE)),
    srv_get_device_list_(nh_.advertiseService(
        std::string(STSRV_G_device_list),
        &StCameraNode::getDeviceListCallback, this)),
    srv_get_module_list_(nh_.advertiseService(
        std::string(STSRV_G_module_list),
        &StCameraNode::getModuleListCallback, this)),
    srv_get_sdk_info_(nh_.advertiseService(
        std::string(STSRV_G_sdk_info),
        &StCameraNode::getSDKInfoCallback, this)),
    srv_get_gige_ip_i_(nh_.advertiseService(
        std::string(STSRV_G_gige_ip),
        &StCameraNode::getGigEIPCallback, this)),
    srv_set_gige_ip_i_(nh_.advertiseService(
        std::string(STSRV_S_gige_ip),
        &StCameraNode::setGigEIPCallback, this))
  {
    init();
  }

  StCameraNode::~StCameraNode()
  {
    std::lock_guard<std::mutex> lock(mtx_map_camera_);

		for (MapCameraInterface::iterator it = map_camera_.begin();
				it != map_camera_.end();)
		{
			if (it->second != nullptr) 
      {
        delete it->second;
      }
      it = map_camera_.erase(it);
		}
  }

  void StCameraNode::init()
  {
    std::vector<std::string> camera_to_connect;
    param_.loadCameraList(nh_, camera_to_connect);

    std::string allowed="Allowed camera: ";
    if (param_.connectAllCamera()) 
    {
      allowed += "all";
    }
    else if (param_.connectFirstCameraOnly()) 
    {
      allowed += "First found camera";
    }
    else 
    {
      std::lock_guard<std::mutex> lock(mtx_map_camera_);
      
      // connect to certain camera only
      for (size_t i = 0; i < camera_to_connect.size(); i++)
      {
        // set interface to null for the allowed camera:
        map_camera_[param_.getNamespace(camera_to_connect[i])] = nullptr;
        allowed += camera_to_connect[i] + " ";
      }
    }
    ROS_DEBUG("Allowed to connect: %s", allowed.c_str());

    for (uint32_t i = StApi::StSystemVendor_Sentech; 
        i < StApi::StSystemVendor_Count; i++)
    {
      StApi::EStSystemVendor_t vendor = (StApi::EStSystemVendor_t)i;
      try
      {
        stapi_systems_.Register(
            StApi::CreateIStSystem(vendor, StApi::StInterfaceType_All));
      }
      catch(const GenICam::GenericException &x)
      {
        if (vendor == StApi::StSystemVendor_Sentech)
        {
          ROS_ERROR(
              "%s %s %d: Unable to initialize OMRON SENTECH GenTL Producer: %s",
              __FILE__,__func__,__LINE__, x.GetDescription());
        }
      }
    }

    for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
    {
      StApi::IStSystem *p_tl = stapi_systems_[i];
      p_tl->UpdateInterfaceList();
    }

  }

  void StCameraNode::spin()
  {
    size_t count = 0;
    std::lock_guard<std::mutex> lock(mtx_map_camera_);
    for (MapCameraInterface::iterator it = map_camera_.begin(); 
        it != map_camera_.end(); it++)
    {
      StCameraInterface *c = it->second;
      if (c != nullptr)
      {
        // check device availability
        if (c->deviceIsLost())
        {
          MapDeviceConnection::iterator it2 = map_connection_.find(it->first);
          if (it2 != map_connection_.end())
          {
            it2->second.connected = false;
            it2->second.timestamp = ros::Time::now();
            msg_device_connection_.publish(it2->second);
          }
          map_camera_[it->first] = nullptr;
          ROS_INFO("Device %s is disconnected.", it2->first.c_str());
          delete c;
        }
        else
        {
          count++;
        }
      }
    }

    bool connect_first_camera_only = param_.connectFirstCameraOnly();
    if (connect_first_camera_only)
    {
      if (count > 0)
      {
        return;
      }
    }
    else if (!param_.connectAllCamera())
    {
      bool all_connected = true;
      for (MapCameraInterface::iterator it = map_camera_.begin(); 
          it != map_camera_.end(); it++)
      {
        if (it->second == nullptr)
        {
          all_connected = false;
          break;
        }
      }
      if (all_connected) 
      {
        return;
      }
    }
    //else ROS_INFO("Connect to any devices");
    
    // search for camera
    try
    {
      std::lock_guard<std::mutex> lock(mtx_update_device_list_);
      for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
      {
        StApi::IStSystem *p_tl = stapi_systems_[i];
        size_t ifcount = p_tl->GetInterfaceCount();  
        for (size_t j = 0; j < ifcount; j++)
        {
          StApi::IStInterface *p_iface = p_tl->GetIStInterface(j);
          p_iface->UpdateDeviceList();  
  
          size_t devcount = p_iface->GetDeviceCount();
          for (size_t k = 0; k < devcount; k++)
          {
            if (p_iface->IsDeviceAvailable(k, GenTL::DEVICE_ACCESS_CONTROL))
            {
              const StApi::IStDeviceInfo *p_devinfo = 
                  p_iface->GetIStDeviceInfo(k);
              bool init_ok = initializeCamera(p_iface, p_devinfo);
              if (init_ok && connect_first_camera_only) 
              {
                return;
              }
              //ROS_INFO("connected %d", init_ok);
            }
          } // end loop device
        } // end loop interface
      } // end loop system
    }
    catch(...)
    {
    }
  }

  bool StCameraNode::initializeCamera(StApi::IStInterface *p_iface,
      const StApi::IStDeviceInfo *p_devinfo)
  {
    if (p_iface == nullptr || p_devinfo == nullptr)
    {
      return false;
    }

    std::string device_id = std::string(p_devinfo->GetID());
    std::string device_displayname = std::string(p_devinfo->GetDisplayName());
    std::string key = "";
    if (param_.connectAllCamera() || param_.connectFirstCameraOnly())
    {
      key = param_.getNamespace(device_id);
    }
    else
    {
      bool found = false;
      for (MapCameraInterface::iterator it = map_camera_.begin(); 
          it != map_camera_.end(); it++)
      {
        key = param_.getNamespace(device_id);
        if (it->first.compare(key) == 0) 
        {
          found = true;
          break;
        }
        key = param_.getNamespace(device_displayname);
        if (it->first.compare(key) == 0) 
        {
          found = true;
          break;
        }
      }
      if (!found) key = "";
    }

    if (key.empty()) 
    {
      return false;
    }

    try
    {
      StApi::IStDeviceReleasable *dev = p_iface->CreateIStDevice(
          GenICam::gcstring(device_id.c_str()),
          GenTL::DEVICE_ACCESS_CONTROL);

      if (p_devinfo->GetTLType().compare(TLTypeU3VName) == 0)
      {
        StCameraInterfaceU3V *pu3v = new StCameraInterfaceU3V(
            dev, nh_, key, &param_);
        map_camera_[key] = pu3v;
      }
      else if (p_devinfo->GetTLType().compare(TLTypeGEVName) == 0)
      {
        StCameraInterfaceGEV *pgev = new StCameraInterfaceGEV(
            dev, nh_, key, &param_);
        map_camera_[key] = pgev;
      }
      else
      {
        StCameraInterface *pcif = new StCameraInterface(
            dev, nh_, key, &param_);
        map_camera_[key] = pcif;
      }

      // store and publish connection msg
      omronsentech_camera::DeviceConnection msg = 
          fillDeviceConnectionData(dev->GetIStInterface()->GetIStSystem()
              ->GetIStSystemInfo(), dev->GetIStInterface(),
              dev->GetIStDeviceInfo(), key, true);

      map_connection_[key] = msg;

      msg_device_connection_.publish(msg);
   
      ROS_INFO("%s %s %d: %s (%s) is successfully initialized as %s",
          __FILE__,__func__,__LINE__,
          device_id.c_str(), device_displayname.c_str(), key.c_str());
      return true;
    }
    catch(GenICam::GenericException &x)
    {
      ROS_ERROR("%s %s %d: Unable to open device %s (%s): %s",
          __FILE__,__func__,__LINE__, device_id.c_str(), 
          device_displayname.c_str(), x.GetDescription());
    }
    return false;
  }

  bool StCameraNode::getDeviceListCallback(
      omronsentech_camera::GetDeviceList::Request &req,
      omronsentech_camera::GetDeviceList::Response &res)
  {
    std::lock_guard<std::mutex> lock(mtx_update_device_list_);
    try
    {
      for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
      {
        StApi::IStSystem *p_tl = stapi_systems_[i];
        size_t ifcount = p_tl->GetInterfaceCount(); 
        for (size_t j = 0; j < ifcount; j++)
        {
          StApi::IStInterface *p_iface = p_tl->GetIStInterface(j);
          p_iface->UpdateDeviceList();  
          size_t devcount = p_iface->GetDeviceCount(); 
          for (size_t k = 0; k < devcount; k++)
          {
            const StApi::IStDeviceInfo *p_devinfo = 
                p_iface->GetIStDeviceInfo(k);
            std::string device_id = std::string(p_devinfo->GetID());
            std::string device_sn = std::string(p_devinfo->GetDisplayName());
            std::string key_id = param_.getNamespace(device_id);
            std::string key_sn = param_.getNamespace(device_sn);
            MapDeviceConnection::iterator it = map_connection_.find(key_id);
            if (it == map_connection_.end())
            {
              it = map_connection_.find(key_sn);
            }
            if (it != map_connection_.end())
            {
              res.device_list.push_back(it->second);
              continue;
            }
            omronsentech_camera::DeviceConnection msg = 
                fillDeviceConnectionData(p_tl->GetIStSystemInfo(), p_iface,
                p_devinfo);
            res.device_list.push_back(msg);
          } // end loop device
        } // end loop interface
      } // end loop system
    }
    catch(...)
    {
    }
    return true;
  }


  bool StCameraNode::getModuleListCallback(
      omronsentech_camera::GetModuleList::Request &req,
      omronsentech_camera::GetModuleList::Response &res)
  {
    res.module_name_list.push_back("System");
    res.module_name_list.push_back("Interface");
    res.module_name_list.push_back("LocalDevice");
    res.module_name_list.push_back("RemoteDevice");
    res.module_name_list.push_back("DataStream");
    //"StreamBuffer" is not included.
    return true;
  }


  bool StCameraNode::getSDKInfoCallback(
      omronsentech_camera::GetSDKInfo::Request &req,
      omronsentech_camera::GetSDKInfo::Response &res)
  {
    res.sdk_version = StApi::GetStApiVersionText();

    for (size_t i = 0; i < stapi_systems_.GetSize(); i++)
    {
      omronsentech_camera::GenTLInfo data;
      const StApi::IStSystemInfo *info = 
          stapi_systems_[i]->GetIStSystemInfo();
      char gentl_ver[8];
      snprintf(gentl_ver,8,"%d.%d", info->GetGenTLVersionMajor(),
          info->GetGenTLVersionMinor());
      data.vendor = info->GetVendor();
      data.version = gentl_ver;
      data.producer_version = info->GetVersion();
      data.tltype = info->GetTLType();
      data.full_path = info->GetPathName();
      res.gentl_info_list.push_back(data);
    }
    return true;
  }

  bool StCameraNode::getGigEIPCallback(
      omronsentech_camera::GetGigEIPi::Request &req,
      omronsentech_camera::GetGigEIPi::Response &res)
  {
    try
    {
      for (size_t i = 0; i < stapi_systems_.GetSize();i++)
      {
        StApi::IStSystem *p_tl = stapi_systems_[i];
        for (size_t j = 0; j < p_tl->GetInterfaceCount(); j++)
        {
          StApi::IStInterface *p_iface = p_tl->GetIStInterface(j);
          if (p_iface->GetIStInterfaceInfo()->GetTLType()
              .compare(TLTypeGEVName) != 0)
          {
            continue;
          }

          p_iface->UpdateDeviceList();  
          GenApi::CNodeMapPtr p_nodemap(p_iface->GetIStPort()->GetINodeMap());
          GenApi::CIntegerPtr p_integer_ip(
            p_nodemap->GetNode("GevInterfaceSubnetIPAddress"));
          GenApi::CIntegerPtr p_integer_subnet(
              p_nodemap->GetNode("GevInterfaceSubnetMask"));
          GenApi::CIntegerPtr p_integer_gw(
              p_nodemap->GetNode("GevInterfaceGateway"));
          res.if_ip_address = p_integer_ip->ToString();
          res.if_ip_mask = p_integer_subnet->ToString();
          res.if_ip_gateway = p_integer_gw->ToString();

          GenApi::CIntegerPtr pDeviceSelector(
              p_nodemap->GetNode("DeviceSelector"));
          const int64_t nMaxIndex = pDeviceSelector->GetMax();
          for (int64_t k = 0; k <= nMaxIndex; k++)
          {
            pDeviceSelector->SetValue(k);
            GenApi::CStringPtr pIStringDeviceID(p_nodemap->GetNode("DeviceID"));
            GenICam::gcstring strDeviceID = pIStringDeviceID->GetValue();
            
            if (strDeviceID.compare(req.camera_id.c_str()) != 0)
            {
              continue;
            }
            GenApi::CIntegerPtr p_integer_ip(
                p_nodemap->GetNode("GevDeviceIPAddress"));
            GenApi::CIntegerPtr p_integer_subnet(
                p_nodemap->GetNode("GevDeviceSubnetMask"));
            GenApi::CIntegerPtr p_integer_gw(
                p_nodemap->GetNode("GevDeviceGateway"));
            res.dev_ip_address = p_integer_ip->ToString();
            res.dev_ip_mask = p_integer_subnet->ToString();
            res.dev_ip_gateway = p_integer_gw->ToString();
            return true;
          } // end loop device
        } // end loop interface
      } // end loop system
      ROS_ERROR("The Camera with the given ID %s could not be found.", 
        req.camera_id.c_str());
    }
    CATCH_GIGECALL_ERR();
    return false;
  }

  bool StCameraNode::setGigEIPCallback(
      omronsentech_camera::SetGigEIPi::Request &req,
      omronsentech_camera::SetGigEIPi::Response &res)
  {
    try
    {
      for (size_t i = 0; i < stapi_systems_.GetSize();i++)
      {
        StApi::IStSystem *p_tl = stapi_systems_[i];
        for (size_t j = 0; j < p_tl->GetInterfaceCount(); j++)
        {
          StApi::IStInterface *p_iface = p_tl->GetIStInterface(j);
          if (p_iface->GetIStInterfaceInfo()->GetTLType()
              .compare(TLTypeGEVName) != 0)
          {
            continue;
          }

          p_iface->UpdateDeviceList();  
          GenApi::CNodeMapPtr p_nodemap(p_iface->GetIStPort()->GetINodeMap());
          GenApi::CIntegerPtr pDeviceSelector(
              p_nodemap->GetNode("DeviceSelector"));
          const int64_t nMaxIndex = pDeviceSelector->GetMax();
          for (int64_t k = 0; k <= nMaxIndex; k++)
          {
            pDeviceSelector->SetValue(k);
            GenApi::CStringPtr pIStringDeviceID(p_nodemap->GetNode("DeviceID"));
            GenICam::gcstring strDeviceID = pIStringDeviceID->GetValue();
            
            if (strDeviceID.compare(req.camera_id.c_str()) != 0)
            {
              continue;
            }

            GenApi::CIntegerPtr p_integer_ip(
                p_nodemap->GetNode("GevDeviceForceIPAddress"));
            GenApi::CIntegerPtr p_integer_subnet(
                p_nodemap->GetNode("GevDeviceForceSubnetMask"));
            GenApi::CIntegerPtr p_integer_gw(
                p_nodemap->GetNode("GevDeviceForceGateway"));
            GenApi::CCommandPtr pCommand(
                p_nodemap->GetNode("GevDeviceForceIP"));

            p_integer_ip->SetValue(ntohl(inet_addr(req.ip_address.c_str())));
            p_integer_subnet->SetValue(ntohl(inet_addr(req.ip_mask.c_str())));
            p_integer_gw->SetValue(ntohl(inet_addr(req.ip_gateway.c_str())));
            pCommand->Execute();
            return true;
          } // end loop device
        } // end loop interface
      } // end loop system
      ROS_ERROR("The Camera with the given ID %s could not be found.", 
        req.camera_id.c_str());
    }
    CATCH_GIGECALL_ERR();
    return false;
  }
 
  omronsentech_camera::DeviceConnection StCameraNode::fillDeviceConnectionData(
          const StApi::IStSystemInfo *p_tlinfo, StApi::IStInterface* p_iface,
          const StApi::IStDeviceInfo *p_devinfo, std::string device_namespace,
          bool connected)
  {
    omronsentech_camera::DeviceConnection msg;

    msg.timestamp = ros::Time::now();
    msg.device_id = p_devinfo->GetID().c_str();
    msg.device_model = p_devinfo->GetModel().c_str();
    msg.device_serial = p_devinfo->GetSerialNumber().c_str();
    msg.device_namespace = device_namespace;
    msg.device_tltype = p_devinfo->GetTLType().c_str();

    if (msg.device_tltype.compare(TLTypeGEVName) == 0)
    {
      try
      {
        GenApi::CNodeMapPtr p_nodemap(p_iface->GetIStPort()->GetINodeMap());
        GenApi::CIntegerPtr p_integer_ip(
            p_nodemap->GetNode("GevInterfaceSubnetIPAddress"));
        GenApi::CIntegerPtr p_integer_subnet(
            p_nodemap->GetNode("GevInterfaceSubnetMask"));
        GenApi::CIntegerPtr p_integer_gw(
            p_nodemap->GetNode("GevInterfaceGateway"));
        msg.device_tl_specific_field.push_back("if_ip_address");
        msg.device_tl_specific_value.push_back(
            p_integer_ip->ToString().c_str());
        msg.device_tl_specific_field.push_back("if_ip_mask");
        msg.device_tl_specific_value.
            push_back(p_integer_subnet->ToString().c_str());
        msg.device_tl_specific_field.push_back("if_ip_gateway");
        msg.device_tl_specific_value.push_back(
            p_integer_gw->ToString().c_str());

        GenApi::CIntegerPtr pDeviceSelector(
            p_nodemap->GetNode("DeviceSelector"));
        const int64_t nMaxIndex = pDeviceSelector->GetMax();
        for (int64_t k = 0; k <= nMaxIndex; k++)
        {
          pDeviceSelector->SetValue(k);
          GenApi::CStringPtr pIStringDeviceID(p_nodemap->GetNode("DeviceID"));
          GenICam::gcstring strDeviceID = pIStringDeviceID->GetValue();
          
          if (strDeviceID.compare(msg.device_id.c_str()) != 0)
          {
            continue;
          }
          GenApi::CIntegerPtr p_integer_ip(
              p_nodemap->GetNode("GevDeviceIPAddress"));
          GenApi::CIntegerPtr p_integer_subnet(
              p_nodemap->GetNode("GevDeviceSubnetMask"));
          GenApi::CIntegerPtr p_integer_gw(
              p_nodemap->GetNode("GevDeviceGateway"));
          msg.device_tl_specific_field.push_back("dev_ip_address");
          msg.device_tl_specific_value.
              push_back(p_integer_ip->ToString().c_str());
          msg.device_tl_specific_field.push_back("dev_ip_mask");
          msg.device_tl_specific_value.
              push_back(p_integer_subnet->ToString().c_str());
          msg.device_tl_specific_field.push_back("dev_ip_gateway");
          msg.device_tl_specific_value.
              push_back(p_integer_gw->ToString().c_str());
          break; 
        }
      }
      catch(...)
      {
      }
    }
    msg.connected = connected;

    // gentl info
    char gentl_ver[8];
    snprintf(gentl_ver,8,"%d.%d", p_tlinfo->GetGenTLVersionMajor(),
        p_tlinfo->GetGenTLVersionMinor());
    msg.device_gentl_info.vendor = p_tlinfo->GetVendor();
    msg.device_gentl_info.version = gentl_ver;
    msg.device_gentl_info.producer_version = p_tlinfo->GetVersion();
    msg.device_gentl_info.tltype = p_tlinfo->GetTLType();
    msg.device_gentl_info.full_path = p_tlinfo->GetPathName();

    return msg;
  }

} // end of namespace stcamera
