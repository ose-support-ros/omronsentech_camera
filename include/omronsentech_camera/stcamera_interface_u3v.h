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

/** \file stcamera_interface_u3v.h
 *  \brief Class to control a connected USB3Vision camera
 *
 *  This class is specific for USB3Vision camera.
 *  If there is a more specific USB3Vision camera to control, derive this class.
 */

#ifndef STCAMERA_STCAMERA_INTERFACE_U3V_H
#define STCAMERA_STCAMERA_INTERFACE_U3V_H

#include "omronsentech_camera/stcamera_interface.h"

namespace stcamera
{
  /** \class StCameraInterfaceU3V
   *  \brief Class to control a connected GigEVision camera
   *
   *  This class is specific for USB3Vision camera.
   *  If there is a more specific USB3Vision camera to control, derive this 
   *  class.
   */
  class StCameraInterfaceU3V: public StCameraInterface
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
      StCameraInterfaceU3V(StApi::IStDeviceReleasable *dev,
                           ros::NodeHandle nh_parent, 
                           const std::string &camera_namespace, 
                           StParameter *param,
                           uint32_t queue_size = STCAMERA_QUEUE_SIZE);

      /** Destructor.
       */
      virtual ~StCameraInterfaceU3V();
  };
}
#endif
