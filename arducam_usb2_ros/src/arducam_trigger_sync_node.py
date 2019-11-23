#!/usr/bin/env python
import os
import time
import cv2
import numpy as np
import json
from ImageConvert import *
import ArducamSDK

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import TimeReference

from arducam_usb2_ros.srv import WriteReg, WriteRegResponse
from arducam_usb2_ros.srv import ReadReg, ReadRegResponse


global cfg,handle,Width,Heigth,color_mode
next_trigger_counter = 1
cfg = {}
handle = {}


def configBoard(fileNodes):
    global handle
    for i in range(0,len(fileNodes)):
        fileNode = fileNodes[i]
        buffs = []
        command = fileNode[0]
        value = fileNode[1]
        index = fileNode[2]
        buffsize = fileNode[3]
        for j in range(0,len(fileNode[4])):
            buffs.append(int(fileNode[4][j],16))
        ArducamSDK.Py_ArduCam_setboardConfig(handle,int(command,16),int(value,16),int(index,16),int(buffsize,16),buffs)

pass

def writeSensorRegs(fileNodes):
    global handle
    for i in range(0,len(fileNodes)):
        fileNode = fileNodes[i]
        if fileNode[0] == "DELAY":
            time.sleep(float(fileNode[1])/1000)
            continue
        regAddr = int(fileNode[0],16)
        val = int(fileNode[1],16)
        ArducamSDK.Py_ArduCam_writeSensorReg(handle,regAddr,val)

pass

def camera_initFromFile(fialeName):
    global cfg,handle,Width,Height,color_mode
    #load config file
    config = json.load(open(fialeName,"r"))

    camera_parameter = config["camera_parameter"]
    Width = int(camera_parameter["SIZE"][0])
    Height = int(camera_parameter["SIZE"][1])

    BitWidth = camera_parameter["BIT_WIDTH"]
    ByteLength = 1
    if BitWidth > 8 and BitWidth <= 16:
        ByteLength = 2
    FmtMode = int(camera_parameter["FORMAT"][0])
    color_mode = (int)(camera_parameter["FORMAT"][1])
    rospy.loginfo("color mode %d",color_mode)

    I2CMode = camera_parameter["I2C_MODE"]
    I2cAddr = int(camera_parameter["I2C_ADDR"],16)
    TransLvl = int(camera_parameter["TRANS_LVL"])
    cfg = {"u32CameraType":0x4D091031,
            "u32Width":Width,"u32Height":Height,
            "usbType":0,
            "u8PixelBytes":ByteLength,
            "u16Vid":0,
            "u32Size":0,
            "u8PixelBits":BitWidth,
            "u32I2cAddr":I2cAddr,
            "emI2cMode":I2CMode,
            "emImageFmtMode":FmtMode,
            "u32TransLvl":TransLvl }


    ret,handle,rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)

    if ret == 0:
        usb_version = rtn_cfg['usbType']
        rospy.loginfo("USB VERSION:%d",usb_version)
        #config board param
        configBoard(config["board_parameter_dev2"])
        writeSensorRegs(config["register_parameter"])

        rtn_val,datas = ArducamSDK.Py_ArduCam_readUserData(handle,0x400-16, 16)
        rospy.loginfo("Serial: %c%c%c%c-%c%c%c%c-%c%c%c%c",datas[0],datas[1],datas[2],datas[3],
                                                    datas[4],datas[5],datas[6],datas[7],
                                                    datas[8],datas[9],datas[10],datas[11])

        return True
    else:
        rospy.logfatal("Device open fail (rtn_val = %d)",ret)
        return False

pass

def write_register(request):
    global handle
    register = request.register
    value = request.value
    rtn_val = ArducamSDK.Py_ArduCam_writeSensorReg(handle,register,value)
    if rtn_val == 0:
        output = 'Value %d written to register %d' % (value, register)
    else:
        output = 'Invalid register'
    return WriteRegResponse(output)

def read_register(request):
    global handle
    register = request.register
    rtn_val, output = ArducamSDK.Py_ArduCam_readSensorReg(handle,register)
    if rtn_val == 0:
        output = 'Register %d: %d' % (register, output)
    else:
        output = 'Invalid register'
    return ReadRegResponse(output)


def rosShutdown():
    global handle
    rtn_val = ArducamSDK.Py_ArduCam_close(handle)
    if rtn_val == 0:
        rospy.loginfo("device close success!")
    else:
        rospy.logerr("device close fail!")



def time_subscriber_callback(time_data):
    global next_trigger_counter

    if next_trigger_counter == time_data.header.seq:
        while ArducamSDK.Py_ArduCam_isFrameReady(handle) != 1:
            continue

        while True:
            rtn_val,data,rtn_cfg = ArducamSDK.Py_ArduCam_getSingleFrame(handle)
            if rtn_val == 0:
                break

        datasize = rtn_cfg['u32Size']
        if datasize > 0:
            image = convert_image(data,rtn_cfg,color_mode)
            if h_flip:
                image = cv2.flip(image, 1)
            if v_flip:
                image = cv2.flip(image, 0)
            try:
                img_msg = bridge.cv2_to_imgmsg(image, "bgr8")
                exposure_time = rospy.Time.from_sec(0.01)
                img_msg.header.stamp.secs = time_data.header.stamp.secs + exposure_time.to_sec()
                img_msg.header.stamp.nsecs = time_data.header.stamp.nsecs + exposure_time.to_nsec()
                img_msg.header.seq = time_data.header.seq
                img_msg.header.frame_id = id_frame
                pub_trigger.publish(img_msg)

            except CvBridgeError as e:
                rospy.logwarn("Frame not published")
    else:
        rospy.logwarn("trigger not in sync (seq expected %u, got %u)!", next_trigger_counter, time_data.header.seq)

    next_trigger_counter += 1


if __name__ == "__main__":

    rospy.init_node("arducam_trigger_ros_node")
    pub_trigger = rospy.Publisher("cam0/image_raw", Image, queue_size=1)
    time_subscriber = rospy.Subscriber("imu/trigger_time", TimeReference, time_subscriber_callback)
    bridge = CvBridge()

    try:
        config_file_name = rospy.get_param("~config_file")
    except:
        rospy.logfatal("Config file path incorrect.")
        exit()

    h_flip = rospy.get_param("~horizontal_flip", False)
    v_flip = rospy.get_param("~vertical_flip", False)
    id_frame = rospy.get_param("~frame_id", "cam")


    if camera_initFromFile(config_file_name):
        ret_val = ArducamSDK.Py_ArduCam_setMode(handle,ArducamSDK.EXTERNAL_TRIGGER_MODE)
        if(ret_val == ArducamSDK.USB_BOARD_FW_VERSION_NOT_SUPPORT_ERROR):
           print("USB_BOARD_FW_VERSION_NOT_SUPPORT_ERROR")
           exit()

        service_write = rospy.Service('arducam/write_reg', WriteReg, write_register)
        service_read = rospy.Service('arducam/read_reg', ReadReg, read_register)

        rospy.on_shutdown(rosShutdown)
        rospy.spin()

    else:
      exit()
