time   timestamp            # ROS time stamp to indicate when the camera is
                            # connected (if connected), or disconnected (if
                            # once connected then disconnected), or retrieved
                            # (if the camera is not connected)

string device_id            # device ID
string device_model         # device model
string device_serial        # device serial number
string device_tltype        # based on GenTL TL type

string[] device_tl_specific_field  # Specific info depending on the TL type
string[] device_tl_specific_value  # Corresponding value for the field

omronsentech_camera/GenTLInfo device_gentl_info # GenTL producer used 
string device_namespace     # ROS namespace for the device if connected
bool   connected            # true if connected. false if disconnected
