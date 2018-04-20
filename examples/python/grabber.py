import rospy
import roslib
import time
import os
from omronsentech_camera.srv import *
from omronsentech_camera.msg import *
from sensor_msgs.msg import Image

SERVER_NODE = "stcamera_node"
MAX_GRABBED = 10
TRIGGER_SELECTOR = "FrameStart"
TRIGGER_SOURCE = "Software"
grabbed_images = 0

# Get SentechSDK and GenTL information
def displaySDKInfo():
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/get_sdk_info"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, GetSDKInfo)
        resp = cmd()
        print resp
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)

# Get one connected device's namespace
def getConnectedDeviceNamespace():
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/get_device_list"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, GetDeviceList)
        resp = cmd()
        output = (x.device_namespace for x in resp.device_list if x.connected)
        for dev_ns in output:
            return dev_ns
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return ""

# Get last error
def getLastError(dev_ns):
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/" + dev_ns + "/get_last_error"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, GetLastError)
        resp = cmd()
        print "Last error: %d" %(resp.error_code)
        print resp.description
        return resp.error_code
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return 0

# Set Gain
def setGain(dev_ns, value):
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/" + dev_ns + "/write_node_float"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, WriteNodeFloat)
        cmd("RemoteDevice", "Gain", value)
        return 0
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return getLastError(dev_ns)

# Get Gain
def getGain(dev_ns):
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/" + dev_ns + "/get_genicam_node_info"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, GetGenICamNodeInfo)
        resp = cmd("RemoteDevice", "Gain")
        return {"value": float(resp.current_value), 
                "min_value":float(resp.min_value),
                "max_value":float(resp.max_value)}
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return getLastError(dev_ns)

# Image callback
def imageCallback(msg):
    global grabbed_images
    global MAX_GRABBED
    grabbed_images = grabbed_images + 1
    if (grabbed_images > MAX_GRABBED):
        return
    print "Grabbed %d images. WxH: %d x %d Encoding %s" %(
            grabbed_images, msg.width, msg.height, msg.encoding)

# Get the status of image acquisition
def isImageAcquisitionEnabled(dev_ns):
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/" + dev_ns + "/get_image_acquisition_status"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, GetImageAcquisitionStatus)
        resp = cmd()
        return resp.value
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return getLastError(dev_ns)

# Set the status of image acquisition
def setImageAcquisition(dev_ns, enabled):
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/" + dev_ns + "/enable_image_acquisition"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, EnableImageAcquisition)
        cmd(enabled)
        return 0
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return getLastError(dev_ns)

# Set trigger
def setTrigger(dev_ns, enabled):
    global SERVER_NODE
    global TRIGGER_SELECTOR
    global TRIGGER_SOURCE
    try:
        client = SERVER_NODE + "/" + dev_ns + "/enable_trigger"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, EnableTrigger)
        cmd(TRIGGER_SELECTOR, TRIGGER_SOURCE, 0, enabled)
        return 0
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return getLastError(dev_ns)

# Send trigger
def sendTrigger(dev_ns):
    global SERVER_NODE
    try:
        client = SERVER_NODE + "/" + dev_ns + "/send_soft_trigger"
        rospy.wait_for_service(client)
        cmd = rospy.ServiceProxy(client, SendSoftTrigger)
        cmd(TRIGGER_SELECTOR)
        return 0
    except rospy.ServiceException, e:
        rospy.logerr("Service call %s failed: %s", client, e)
    except:
        rospy.logerr("Service call %s got exception", client)
    return getLastError(dev_ns)


# Main 
if __name__ == '__main__':
    rospy.init_node("grabber", anonymous=True)
    
    # Display SDK info
    displaySDKInfo()

    # Check connected devices
    dev_ns = ""
    while(True):
        dev_ns = getConnectedDeviceNamespace()
        if (dev_ns != ""):
            break;
        time.sleep(1)

    if (dev_ns != ""):
        # Read gain value
        cur_gain = getGain(dev_ns)
        gain_value = cur_gain["value"]
        new_gain = round((cur_gain["min_value"] + cur_gain["max_value"])/2,0);
        print "Min gain: %.2f" %(cur_gain["min_value"])
        print "Max gain: %.2f" %(cur_gain["max_value"])
        print "Current gain: %.2f" %(gain_value)

        # Change gain value 
        if (setGain(dev_ns, new_gain) == 0):
            print "Update gain to %.2f was successful" %(new_gain)

        # Subscribe to the connected device
        x = rospy.Subscriber(SERVER_NODE + "/" + dev_ns + "/image_raw", Image,
                imageCallback)
        if (not isImageAcquisitionEnabled(dev_ns)):
            setImageAcquisition(dev_ns, True)
        while(grabbed_images < MAX_GRABBED):
            time.sleep(1)

        # Trigger mode
        print
        print "Enable trigger selector: %s, source: %s" %(TRIGGER_SELECTOR, 
                TRIGGER_SOURCE)
        try:
            setImageAcquisition(dev_ns, False)
            setTrigger(dev_ns, True)
            grabbed_images = 0
            setImageAcquisition(dev_ns, True)
            time.sleep(1)
            for i in range(0,10):
                sendTrigger(dev_ns)
                print "Sleep for 1s"
                time.sleep(1)
            setImageAcquisition(dev_ns, False)
            setTrigger(dev_ns, False)
            setImageAcquisition(dev_ns, True)
        except:
            pass

        # Set the original gain value
        setGain(dev_ns, gain_value)

