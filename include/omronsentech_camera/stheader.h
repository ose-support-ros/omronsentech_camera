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

/** \file stheader.h
 *  \brief Contains definitions and macros
 *
 * Contains definitions for error code, name of topics and services, and macro
 * to deal with common exception.
 */

#ifndef STCAMERA_STHEADER_H
#define STCAMERA_STHEADER_H

/**  Max number of topic's item to be queued for delivery to subscribers. */
#define STCAMERA_QUEUE_SIZE         5

/** 
 * \defgroup ERROR_CODE_GROUP List of returned error code other than from GenTL
 * \{
 */
/** Error code returned when unknown exception happened. */
#define UNKNOWN_ERROR               29999
#define UNKNOWN_ERROR_STR "Unknown exception error"

/** Error code returned when any GenICam/GenApi library throws exception. */
#define GENICAM_ERROR               30000
#define GENICAM_ERROR_STR "GenICam error"

/** Error code returned when module name is invalid. Possible module name are
 * as follows: System, Interface, LocalDevice, RemoteDevice, DataStream. */
#define MODULE_ERROR				        30001
#define MODULE_ERROR_STR "Invalid module name"

/** Error code returned when either node name is invalid or inaccessible. */
#define NODE_ERROR					        30002
#define NODE_ERROR_STR "Invalid GenICam node"

/** Error code returned when enabling an-already-enabled event. */
#define EVENT_ALREADY_ON_ERROR		  30003
#define EVENT_ALREADY_ON_ERROR_STR "Event already ON"

/** Error code returned when disabling an-already-disabled event. */
#define EVENT_ALREADY_OFF_ERROR     30004
#define EVENT_ALREADY_OFF_ERROR_STR "Event already OFF"

/** Error code returned when enabling an-already-enabled image acquisition. */
#define ACQ_ALREADY_ON_ERROR		    30005
#define ACQ_ALREADY_ON_ERROR_STR "Image acquisition already ON"

/** Error code returned when disabling an-already-disabled image acquisition. */
#define ACQ_ALREADY_OFF_ERROR       30006
#define ACQ_ALREADY_OFF_ERROR_STR "Image acquisition already OFF"

/** Error code returned when accessing chunk while chunk is not supported. */
#define CHUNK_NOT_SUPPORTED_ERROR   30007
#define CHUNK_NOT_SUPPORTED_ERROR_STR "Chunk not supported"

/** Error code returned when enabling non-existance/inaccessible chunk. */
#define CHUNK_NAME_ERROR            30008
#define CHUNK_NAME_ERROR_STR "Chunk inaccessible"

/** Error code returned when trigger is not fully supported. */
#define TRIGGER_NOT_SUPPORTED_ERROR   30009
#define TRIGGER_NOT_SUPPORTED_ERROR_STR "Trigger not supported"

/** Error code returned when enabling non-existance/inaccessible trigger 
 * selector or source. */
#define TRIGGER_NAME_ERROR            30010
#define TRIGGER_NAME_ERROR_STR "Trigger inaccessible" 

/** Error code returned when event is not fully supported. */
#define EVENT_NOT_SUPPORTED_ERROR     30011
#define EVENT_NOT_SUPPORTED_ERROR_STR "Event not supported"

/** Error code returned when enabling non-existance/inaccessible event or 
 * event node callback. */
#define EVENT_NAME_ERROR              30012
#define EVENT_NAME_ERROR_STR "Event inaccessible"

/** \} */


/**
 * \defgroup TOPIC_NAME_GROUP List of topic name
 * \{
 */

/** Topic name for device_connection. */
#define STMSG_device_connection     "device_connection"

/** Topic name for image. */
#define STMSG_image                 "image_raw"

/** Topic name for chunk. */
#define STMSG_chunk                 "chunk"

/** Topic name for default event. */
#define STMSG_event                 "event"

/** \} */


/**
 * \defgroup SERVICE_NAME_GROUP List of service name
 * \{
 */

/** Service name for obtaining the list of detected devices. */
#define STSRV_G_device_list         "get_device_list"

/** Service name for obtaining the description of a given error code. */
#define STSRV_G_error_code_info     "get_error_code_info"

/** Service name for obtaining the list of module name. */
#define STSRV_G_module_list         "get_module_list"

/** Service name for obtaining the SentechSDK version and GenTL information. */
#define STSRV_G_sdk_info            "get_sdk_info"

/** Service name for obtaining the IP configuration of a GigEVision camera. */
#define STSRV_G_gige_ip             "get_gige_ip"

/** Service name for assigning IP address to a GigEVision camera. */
#define STSRV_S_gige_ip             "set_gige_ip"

/** Service name for obtaining GenICam node value regardless the interface type. 
 */
#define STSRV_R_node                "read_node"

/** Service name for obtaining GenICam node value with boolean interface type.
 */
#define STSRV_R_node_bool           "read_node_bool"

/** Service name for obtaining GenICam node value with enumeration interface 
 * type. */
#define STSRV_R_node_enum           "read_node_enum"

/** Service name for obtaining GenICam node value with integer interface type. 
 */
#define STSRV_R_node_int            "read_node_int"

/** Service name for obtaining GenICam node value with float interface type. */
#define STSRV_R_node_float          "read_node_float"

/** Service name for obtaining GenICam node value with string interface type. */
#define STSRV_R_node_string         "read_node_string"

/** Service name for writing GenICam node value regardless the interface type. 
 */
#define STSRV_W_node                "write_node"

/** Service name for writing GenICam node value with boolean interface type. */
#define STSRV_W_node_bool           "write_node_bool"

/** Service name for writing GenICam node value with Enumeration interface type
 * using the integer value of the enumeration entry. */
#define STSRV_W_node_enum_int       "write_node_enum_int"

/** Service name for writing GenICam node value with Enumeration interface type
 * using the symbolic name of the enumeration entry. */
#define STSRV_W_node_enum_str       "write_node_enum_str"

/** Service name for writing GenICam node value with integer interface type. */
#define STSRV_W_node_int            "write_node_int"

/** Service name for writing GenICam node value with float interface type. */
#define STSRV_W_node_float          "write_node_float"

/** Service name for writing GenICam node value with string interface type. */
#define STSRV_W_node_string         "write_node_string"

/** Service name for executing GenICam node command. */
#define STSRV_W_node_command        "execute_node"

/** Service name for enabling or disabling a given chunk. */
/** Service name for enabling or disabling a given chunk. */
#define STSRV_E_chunk               "enable_chunk"

/** Service name for enabling or disabling a given trigger. */
#define STSRV_E_trigger             "enable_trigger"

/** Service name for enabling or disabling a given GenICam node event. */
#define STSRV_E_event_node          "enable_event_node"

/** Service name for enabling or disabling image acquisition. */
#define STSRV_E_image_acquisition   "enable_image_acquisition"

/** Service name for enabling or disabling event acquisition. */
#define STSRV_E_event_acquisition   "enable_event_acquisition"

/** Service name for obtaining image acquisition status. */ 
#define STSRV_G_image_acquisition_status  "get_image_acquisition_status"

/** Service name for obtaining all event acquisition status. */ 
#define STSRV_G_event_acquisition_status_list "get_event_acquisition_status_list"

/** Service name for obtaining GenICam node event status. */ 
#define STSRV_G_event_node_status_list    "get_event_node_status_list"

/** Service name for obtaining the list of chunks provided by the device. */
#define STSRV_G_chunk_list                "get_chunk_list"

/** Service name for obtaining the list of triggers provided by the device. */
#define STSRV_G_trigger_list              "get_trigger_list"

/** Service name for obtaining the list of enumeration entries for a 
 * given GenICam Enumeration node. */
#define STSRV_G_enum_list                 "get_enum_list"

/** Service name for obtaining the information of a given GenICam node. */
#define STSRV_G_genicam_node_info         "get_genicam_node_info"

/** Service name for sending software trigger execution command. */
#define STSRV_send_soft_trigger           "send_soft_trigger"

/** Service name for obtaining the last error information. */
#define STSRV_G_last_error                "get_last_error"

/** \} */

#endif
