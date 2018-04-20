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

#include "omronsentech_camera/stparameter.h"
#include <algorithm>

namespace stcamera
{
  StParameter::StParameter() 
  {
  }

  StParameter::~StParameter()
  {
  }

  void StParameter::loadCameraList(const ros::NodeHandle &nh, 
      std::vector<std::string> &camera_to_connect)
  {
    vec_camera_to_connect_.clear();
    camera_to_connect.clear();

    if (nh.hasParam("camera_to_connect"))
    {
      nh.getParam("camera_to_connect", vec_camera_to_connect_);
    }

    for (size_t i = 0; i < vec_camera_to_connect_.size(); i++)
    {
      std::string camera_id = vec_camera_to_connect_[i];
      if (camera_id.compare("all") == 0) 
      {
        continue;
      }

      // fill in the return arguments
      camera_to_connect.push_back(camera_id);
    }
  }

  std::string StParameter::getCalibrationFile(const ros::NodeHandle &nh,
                                              std::string camera_namespace)
  {
    std::string param_key = "";
    if (!camera_namespace.empty())
    {
      if (nh.searchParam("calibration_file", param_key))
      {
        std::string value = "";
        nh.getParam(param_key, value);
        return value;
      }
    }

    if (nh.searchParam("default_calibration_file", param_key))
    {
      std::string value = "";
      nh.getParam(param_key, value);
      return value;
    }
    return "";
  }

  bool StParameter::connectFirstCameraOnly()
  {
    return (vec_camera_to_connect_.size() == 0);
  }

  bool StParameter::connectAllCamera()
  {
    for (size_t i = 0; i < vec_camera_to_connect_.size(); i++)
    {
      std::string value = vec_camera_to_connect_[i];
      if (value.compare("all") == 0) return true;
    }
    return false;
  }

  std::string StParameter::getNamespace(std::string camera_id)
  {
    std::replace_if(camera_id.begin(), camera_id.end(), ::ispunct, '_');
    return std::string("dev_" + camera_id);
  }

} // end of namespace stcamera
