# ROS-based OMRON SENTECH Camera SDK

## 1. Overview
This omronsentech_camera package is provided by OMRON SENTECH, CO.,LTD.
Any cameras supported by Linux-based SentechSDK are also supported by this package.
This package provides generic access to the camera's *GenICam GenApi nodes* and functions through SentechSDK's transport layer module. Note that *GenICam GenApi node* is a different thing from the term *node* in ROS. Each GenICam GenApi node corresponds to the camera device's feature or register that can be accessed through a generic interface. The detail of the GenICam GenApi can be found in the [GenICam GenApi documentation].

## 2. Installation
This package has been tested on ROS-Kinetic running under Ubuntu 16.04 64bit.
Since this package basically wraps SentechSDK's transport layer module, SentechSDK must be installed prior to the installation of this package. The SentechSDK (USB3Vision and GigEVision) can be downloaded from the following URL:
https://sentech.co.jp/products/USB/software.html.

Below are the list of ROS packages needed to build the package:

1. camera_info_manager
2. image_transport
3. message_generation
4. sensor_msgs
5. std_msgs
6. pluginlib
7. nodelet
8. roscpp
9. roslaunch
10. roslint
11. rospy

For installation, either clone the omronsentech_camera.git or copy the directory omronsentech_camera to your catkin workspace. Before building the package as a common ROS package using catkin_make, please ensure that the SentechSDK environment variable has been exported:

``$ echo $STAPI_ROOT_PATH``

In case that the environment variables has not been exported, if the SentechSDK installation folder is in /opt/sentech, you can export the environment variables used by SentechSDK by executing:

``$ source /opt/sentech/.stprofile``

The structure of the directory inside the omronsentech_camera is as follows:
<pre>
/
|-./README.md
|-./README-ja.md
|-./CMakeLists.txt
|-./nodelet_plugins.xml
|-./package.xml
|-./rosdoc.yaml
|-./doc/
|      |-./html/
|      |-./manifest.yaml
|
|-./examples/
|      |-./cpp/grabber.cpp
|      |-./python/grabber.py
|
|-./include/
|      |-./omronsentech_camera/*.h
|                             
|-./launch/
|      |-./*.launch
|      |-./*.yaml
|
|-./msg/
|      |-./*.msg
|
|-./src/
|      |-./omronsentech_camera/*.cpp
|                             
|-./srv/
       |-./*.srv
</pre>

To generate the documentation in the doc folder, execute the following command in the folder omronsentech_camera:

``$ rosdoc_lite -o doc ``

Please note that doxygen and rosdoc_lite package are required to build the documentation.

## 3. Node stcamera_node
For easy usage, the stcamera_node provided in the package will automatically connect to either the first found camera, predefined camera, or any camera that it can find (depend on the **camera_to_connect** parameter) and continuously acquire images from the camera.

To run the stcamera_node, execute one of the following command:

* ``$ rosrun omronsentech_camera stcamera_node``

or

* ``$ roslaunch omronsentech_camera stcamera_node.launch``

If the node is run through stcamera_node.launch script, you can easily configure the parameters in stcamera_node.yaml to control which camera to connect. See the next section for the detail of parameter configuration.

### 3.1. Node Parameters
Parameters value are case sensitive. There are only three parameters to configure, as follows:

  * **camera_to_connect** : used to assign which camera to connect. This parameter must be set before the stcamera_node is running. Changing this parameter when stcamera_node is running will not take any effect. Set empty to connect to the first found camera. Set to ["all"] to connect to all found camera. Set to the CAMERA_ID or CAMERA_MODEL(SERIAL) to only connect to the specified camera(s). Examples:
    * ``camera_to_connect:[]``: Connect to first found camera.
    * ``camera_to_connect:["all"]``: Connect to all camera. If the keyword 'all' presents, the nodes will attempt to connect to any found camera.
    * ``camera_to_connect:["00:11:1c:f6:yy:xx","STC-MCS510U3V(00XXYY0)"]``: Connect only to GigEVision camera with MAC address 00:11:1c:f6:yy:xx and USB3Vision camera with model STC-MCS510U3V and serial number 00XXYY0.
    * ``camera_to_connect:["14210003XXYY"]``: Connect only to USB3Vision camera with ID 14210003XXYY.
  * **default_calibration_file** : full path to calibration file used by any connected camera if there is no calibration file configured for the camera. If this parameter is modified when the stcamera_node is running and the camera is already connected, please disable and re-enable image acquisition of the camera to take effect the changes. To generate the calibration file and rectify the image, please refer to the ROS tutorial as follows:
      * http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration
      * http://wiki.ros.org/camera_calibration_parsers
      * http://wiki.ros.org/image_proc
  * *{dev\_CAMERA-NS}*/**calibration\_file** : Full path to calibration file for camera with namespace *{dev\_CAMERA-NS}*. If this parameter is modified when the stcamera_node is running and the camera is already connected, please disable and re-enable image acquisition of the camera to take effect the changes. See next section for the detail of the camera's namespace.


### 3.2. Camera Namespace
Once stcamera_node is connected to a camera, it will advertise topics and services correspond to the camera. To differentiate between the connected camera, each camera has its own namespace *{dev\_CAMERA-NS}*. The namespace *{dev\_CAMERA-NS}* is automatically generated by stcamera_node with the following rules:

* If the parameter **camera_to_connect** is empty or set to ["all"], the format of the namespace becomes *dev\_{CAMERA\_ID}*, where *{CAMERA\_ID}* is the ID of the camera. For example, if the connected camera has ID 14210003xxYY, the namespace becomes *dev_14210003xxYY*. Any non-alphanumeric characters in the camera ID, such as in GigEVision's MAC address, will be replaced with underscore.
* If the parameter **camera_to_connect** is set to either CAMERA\_MODEL(SERIAL) or CAMERA\_ID, the format of the namespace either becomes *dev\_{CAMERA\_MODEL\_SERIAL\_}* or *dev\_{CAMERA\_ID}*, respectively, depending on the parameter values. *{CAMERA\_MODEL\_SERIAL\_}* is the converted value from CAMERA_MODEL(SERIAL). *{CAMERA\_ID}* is the ID of the camera. Any non-alphanumeric characters in either of them will be replaced with underscore.

The namespace *{dev\_CAMERA-NS}* can also be retrieved from the value of *device_namespace* field returned by service call **get_device_list** or message from  **device_connection** topic.

### 3.3. Published Topics
The topics published by the stcamera_node are as follows:

No. | Topics | Description
:---: | ----------------------- | ----------------------------------------------
1. | **device_connection** | Published when connection or disconnection happened.
2. | *{dev\_CAMERA-NS}*/**camera_info** | Based on the ROS image_transport.
3. | *{dev\_CAMERA-NS}*/**chunk** | Published when chunk data is acquired.
4. | *{dev\_CAMERA-NS}*/**event** | Published when camera event is registered and event data corresponds to the registered event is acquired.
5. | *{dev\_CAMERA-NS}*/**image_raw\*** | Published when image data is acquired, based on the ROS image_transport.


### 3.4. Services
The services advertised by the stcamera_node are as follows:

*(Please be aware that the GenICam node mentioned below is a completely different thing from the ROS node)*:

* **get_device_list** : Get the list of detected devices information.
* **get_gige_ip** : Get the IP address of a given GigEVision camera's MAC.
* **get_module_list** : Get the list of the corresponding GenICam GenTL module name as follows: *System*, *Interface*, *LocalDevice*, *RemoteDevice*, and *DataStream*. The module name must be passed on as an argument when accessing a specific GenICam node. For example, to access *Gain* of the camera device using read or write_node service, you have to specify that the *Gain* resides in RemoteDevice module.
* **get_sdk_info** : Get the information of SentechSDK and GenTL producer version.
* **set_gige_ip** : Set the IP address of a given GigEVision camera's MAC.
* *{dev\_CAMERA-NS}*/**enable_chunk** : Enable or disable chunk.
* *{dev\_CAMERA-NS}*/**enable_event_acquisition** : Enable or disable event acquisition for a given module.
* *{dev\_CAMERA-NS}*/**enable_event_node** : Enable or disable event callback for a given GenICam node.
* *{dev\_CAMERA-NS}*/**enable_image_acquisition** : Enable or disable image acquisition. Image acquisition is automatically enabled when the camera is connected to the stcamera_node.
* *{dev\_CAMERA-NS}*/**enable_trigger** : Enable or disable trigger for a given trigger selector and source.
* *{dev\_CAMERA-NS}*/**execute_node** : Execute a given GenICam node that has interface type of Command.
* *{dev\_CAMERA-NS}*/**get_chunk_list** : Get the list of chunks supported by the camera.
* *{dev\_CAMERA-NS}*/**get_enum_list** : Get the list of enumeration of a given GenICam enumeration node.
* *{dev\_CAMERA-NS}*/**get_event_acquisition_status_list** : Get the event acquisition status for all modules.
* *{dev\_CAMERA-NS}*/**get_event_node_status_list** : Get the GenICam nodes event acquisition status for a given module.
* *{dev\_CAMERA-NS}*/**get_genicam_node_info** : Get information for a given GenICam node.
* *{dev\_CAMERA-NS}*/**get_image_acquisition_status** : Get the image acquisition status.
* *{dev\_CAMERA-NS}*/**get_last_error** : Get the last execution status information. See the next section for the list of available error codes.
* *{dev\_CAMERA-NS}*/**get_trigger_list** : Get the list of trigger supported by the camera.
* *{dev\_CAMERA-NS}*/**image_raw*** : Based on the ROS image_transport.
* *{dev\_CAMERA-NS}*/**read_node** : Read a given GenICam node regardless the node interface type.
* *{dev\_CAMERA-NS}*/**read_node_bool** : Read a given GenICam node that has interface type of boolean.
* *{dev\_CAMERA-NS}*/**read_node_enum** : Read a given GenICam node that has interface type of enumeration.
* *{dev\_CAMERA-NS}*/**read_node_float** : Read a given GenICam node that has interface type of float.
* *{dev\_CAMERA-NS}*/**read_node_int** : Read a given GenICam node that has interface type of integer.
* *{dev\_CAMERA-NS}*/**read_node_string** : Read a given GenICam node that has interface type of string.
* *{dev\_CAMERA-NS}*/**send_soft_trigger** : Send software trigger.
* *{dev\_CAMERA-NS}*/**set_camera_info** : Based on the ROS camera_info_manager (not directly used by this node)
* *{dev\_CAMERA-NS}*/**write_node** : Write string value to a given GenICam node regardless the node interface type. The internal of stcamera_node will automatically convert the value to the GenICam node's interface type.
* *{dev\_CAMERA-NS}*/**write_node_bool** : Write boolean value to a given GenICam node that has interface type of boolean.
* *{dev\_CAMERA-NS}*/**write_node_enum_int** : Write integer value to a given GenICam node that has interface type of enumeration.
* *{dev\_CAMERA-NS}*/**write_node_enum_str** : Write string value (symbolic name of the enumeration entry) to a given GenICam node that has interface type of enumeration.
* *{dev\_CAMERA-NS}*/**write_node_float** : Write float value to a given GenICam node that has interface type of float.
* *{dev\_CAMERA-NS}*/**write_node_int** : Write integer value to a given GenICam node that has interface type of integer.
* *{dev\_CAMERA-NS}*/**write_node_string** : Write string value to a given GenICam node that has interface type of string.

Attention:

* Depending on the camera model, some GenICam nodes cannot be accessed during image acquisition. In that case, please disable image acquisition before accessing the node by using service call **enable_image_acquisition** with parameter *false*. For example:<br />
``$ rosservice call /stcamera_node/dev_CAMERA-NS/enable_image_acquisition false``

* When writing to GenICam node with Integer interface type, the value must be a multiple of the increment value of the node. Use **get_genicam_node_info** to get the increment value of the node. The example below shows that the Width must be a multiple of 16:<br />
``$ rosservice call /stcamera_node/dev_CAMERA-NS/get_genicam_node_info RemoteDevice Width``
<pre>
name: "Width"
description: "Width of the image provided by the device (in pixels)."
name_space: "Standard"
interface_type: "IInteger"
access_mode: "Read Only"
is_cachable: "Yes"
visibility: "Beginner"
caching_mode: "Write to register, write to cache on read"
is_streamable: True
enum_value_str_list: []
enum_value_int_list: []
current_value: "2448"
min_value: "64"
max_value: "2448"
increment: "16"
unit: ''
child_node_list: []
</pre>


### 3.5. Error codes
 Codes | Description
 ---: | ---
 < 0 | GenTL error codes. Please refer to [GenICam GenTL documentation].
 0 | No error.
 30000 | GenICam GenericException occurred.
 30001 | Module name is invalid.
 30002 | (GenICam) Node is either invalid or inaccessible.
 30003 | Event already ON.
 30004 | Event already OFF.
 30005 | Image acquisition already ON.
 30006 | Image acquisition already OFF.
 30007 | Chunk is not supported.
 30008 | Chunk name is not valid.
 30009 | Trigger is not supported.
 30010 | Trigger name is not valid.
 30011 | Event is not supported.
 30012 | Event name is not valid.

## 4. Usage
### 4.1. Displaying acquired images with image_view and Changing Camera Gain:
For a simple viewing and operation, you can refer to the following steps:

* Launch the stcamera_node and allow to connect to all cameras:<br />
``$ roslaunch omronsentech_camera stcamera_node.launch``
* Get the camera namespace:<br />
``$ rosservice call /stcamera_node/get_device_list``<br />
	Output:
<pre>
device_list:
    -
    timestamp:
      secs: 1521005471
      nsecs: 777053680
    device_id: "142100030510"
    device_model: "STC-MCS510U3V"
    device_serial: "XXYYYZZ"
    device_tltype: "U3V"
    device_tl_specific_field: []
    device_tl_specific_value: []
    device_gentl_info:
      vendor: "OMRON_SENTECH"
      version: "1.5"
      producer_version: "1.5.X.X.X"
      full_path: "/opt/sentech/lib/libstgentl.cti"
      tltype: "Mixed"
    device_namespace: "dev_14210XXYYYZZ"
    connected: True
</pre>

* Run the image_view:<br />
``$ rosrun image_view image_view image:=/stcamera_node/dev_14210XXYYYZZ/image_raw`` where dev_14210XXYYYZZ is the namespace of the connected camera.
* To change the gain of the camera (IFloat type) to 100.0, for example, use the GenICam write node service as follows:<br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/write_node_float RemoteDevice Gain 100.0``<br /> or <br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/write_node RemoteDevice Gain "'100.0'"``

* To read the current gain value, you can use the GenICam read node service as follows:<br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/read_node_float RemoteDevice``<br /> or <br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/read_node RemoteDevice Gain``

* To check the Gain node information by using **get_genicam_node_info** :<br />
``$ rosservice call /stcamera_node/dev_14210XXYYYZZ/get_genicam_node_info RemoteDevice Gain``<br />
Output:
<pre>
name: "Gain"
description: "Controls the selected gain as an absolute physical value. This is an amplification\
	\ factor applied to the video signal."
name_space: "Standard"
interface_type: "IFloat"
access_mode: "Read and Write"
is_cachable: "Yes"
visibility: "Beginner"
caching_mode: "Write to register, write to cache on read"
is_streamable: True
enum_value_str_list: []
enum_value_int_list: []
current_value: "100"
min_value: "0.000000"
max_value: "192.000000"
increment: ''
unit: ''
child_node_list: []
</pre>

### 4.2. Launch Script Examples and Termination
Examples of launch script and their parameters are included in the package. Please refer to the following procedure to use them:

* **stcamera_node.launch**:
This launch script will run stcamera_node. The node will attempt to connect to any detected cameras. You can directly launch this script without any modification or configure the calibration file (optional) in stcamera_node.yaml.
* **stcamera_nodelet_manager.launch**:
This launch script will run the nodelet version of stcamera_node. The node will attempt to connect to any detected cameras. You can directly launch this script without any modification or configure the calibration file (optional) in stcamera_nodelet_manager.yaml.
* **stcamera_node_imageproc.launch**:
This launch script will run two nodes: stcamera_node and image_view that displays streamed images from the connected camera. To launch this script, you have to modify the launch script and configuration file by specifying the CAMERA_ID or CAMERA_MODEL(SERIAL), the namespace dev_CAMERA-NS, and the calibration file (optional).
  * Change the argument **camera_namespace** in the launch file:<br /> ``<arg name="camera_namespace" default="dev_CAMERA-NS" />``
  * Change the parameter of **camera_to_connect** in the yaml file:<br /> ``camera_to_connect: ["CAMERA_ID or CAMERA_MODEL(SERIAL)"]``
  * Change the calibration path file either **default_calibration_file** or *{dev_CAMERA-NS}*/**calibration_file** (optional):<br />``default_calibration_file: file:///home/mydata/calibration.xml``

To terminate the stcamera_node, use the following ROS command:<br />
``$ rosnode kill /stcamera_node``


### 4.3. Node Client Examples
In the examples folder, node "grabber" is provided as a straightforward node client of stcamera_node. The grabber (provided in C++ and Python) does the following:

1. Display the SentechSDK and GenTL information by calling **get_sdk_info**.
2. Search for a connected camera and get the camera namespace by calling **get_device_list**.
3. Read and write the camera's Gain by calling **get_genicam_node_info** and **write_node_float** respectively.
4. Subscribe to image_raw topic and grab up to 10 images. The image sizes and encoding information are printed on screen.
5. Set trigger mode to ON (trigger selector is set to FrameStart and the source is set to Software) by calling **enable_trigger**), then grab up to 10 images with interval 1 seconds in between by sending software trigger through **send_soft_trigger**.

If you are using GigEVision camera, please ensure that the camera has a valid IP address. The IP address can be checked by calling either **get_device_list** or **get_gige_ip**. A static IP address can be assigned by using **set_gige_ip** service call. For example: <br />
``$ rosservice call /stcamera_node/get_gige_ip "00:11:1c:xx:xx:xx"`` to display the current IP address of GigEVision camera with MAC 00:11:1c:xx:xx:xx, and
<br />
``$ rosservice call /stcamera_node/set_gige_ip "00:11:1c:xx:xx:xx" 192.168.y.z 255.255.255.0 0.0.0.0`` to assign a static IP address to GigEVision camera with MAC 00:11:1c:xx:xx:xx.

## 5. Customization
The stcamera_node provides generic functionality to control the OMRON SENTECH camera. If customization is needed to accommodate a specific camera, you can extend StCameraInterface class and either create a new ROS node or modify the existing stcamera_node (StCameraNode class) to support the specific camera. If you prefer the latter, you will need to modify the initialization process of the camera interface in StCameraNode class which can be found in function initializeCamera(). Please refer to the doxygen documentation for the structure of the StCameraInterface and the StCameraNode classes.

## 6. Questions/Feedbacks and Bug Reports
If you have any questions/feedbacks or found any bugs, please contact support-ros@sentech.co.jp.

[GenICam GenApi documentation]:http://www.emva.org/wp-content/uploads/GenICam_Standard_v2_0.pdf
[GenICam GenTL documentation]:http://www.emva.org/wp-content/uploads/GenICam_GenTL_1_5.pdf
