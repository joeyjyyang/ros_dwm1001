#!/usr/bin/env python

import rospy, time, serial, os
from ros_dwm1001.msg import Tag, Anchor

# initialize serial port connections
serialPortDWM1001 = serial.Serial(
    port       = "/dev/ttyACM0",
    baudrate   = 115200,
    #parity = serial.PARITY_ODD
    #stopbits = serial.STOPBITS_TWO
    #bytesize = serial.SEVENBITS
)

# Initiaize ROS
tag_msg = Tag()
anchor1_msg = Anchor()
anchor2_msg = Anchor()
anchor3_msg = Anchor()
anchor4_msg = Anchor()
anchor_msgs = [anchor1_msg, anchor2_msg, anchor3_msg, anchor4_msg]

tag_pub = rospy.Publisher("tag", Tag, queue_size=1)
anchor1_pub = rospy.Publisher("anchor1", Anchor, queue_size=1)
anchor2_pub = rospy.Publisher("anchor2", Anchor, queue_size=1)
anchor3_pub = rospy.Publisher("anchor3", Anchor, queue_size=1)
anchor4_pub = rospy.Publisher("anchor4", Anchor, queue_size=1)

# Array to hold Anchors
anchors = []

# Number of anchors in system.
num_anchors = 4

# Tag object.
tag = {
  "x" : 0,
  "y" : 0, 
  "z" : 0,
  "quality_factor" : 0
}
          
def init():
    global serialPortDWM1001

    # Create anchor objects.
    createAnchors()
    # close the serial port in case the previous run didn't closed it properly
    serialPortDWM1001.close()
    # sleep for one sec
    time.sleep(1)
    # open serial port
    serialPortDWM1001.open()

    # check if the serial port is opened
    if(serialPortDWM1001.isOpen()):
        rospy.loginfo("Port opened: "+ str(serialPortDWM1001.name) )
        # start sending commands to the board so we can initialize the board
        # reset incase previuos run didn't close properly
        serialPortDWM1001.write(b'reset')
        # send ENTER two times in order to access api
        serialPortDWM1001.write(b'\r')
        time.sleep(0.5)
        serialPortDWM1001.write(b'\r')
        time.sleep(0.5)
        # send a third one - just in case
        serialPortDWM1001.write(b'\r')
        # give some time to DWM1001 to wake up
        time.sleep(2)
        
    else:
        rospy.loginfo("Can't open port: "+ str(serialPortDWM1001.name))

def run():
    global serialPortDWM1001

    # send command lec, so we can get positions is CSV format
    serialPortDWM1001.write(b'lec')
    serialPortDWM1001.write(b'\r')
    rospy.loginfo("Reading DWM1001 coordinates")
    time.sleep(1)

def set_rate():
    global serialPortDWM1001
     
    serialPortDWM1001.write(b'aurs')
    serialPortDWM1001.write(b' ')
    serialPortDWM1001.write(b'1')
    time.sleep(0.5)
    serialPortDWM1001.write(b' ')
    serialPortDWM1001.write(b'1')
    serialPortDWM1001.write(b'\r')
    time.sleep(1)
    rospy.loginfo("Set UWB publish rate to 100 ms or 10 Hz")
  
def createAnchors():
  global anchors, num_anchors
   
  for index in range(num_anchors):
    anchor = {
      #"anchor_number" : anchor_data[0 + offset],
      "id" : 0,
      "x" : 0,
      "y" : 0,
      "z" : 0,
      "distance" : 0
    }
    anchors.append(anchor)
  rospy.loginfo("Created anchors.")
  
def parseSerial(raw_line):
  global anchors, tag, num_anchors
  
  line = raw_line.strip().split(",")
  
  # Check if serial data is valid based on first index.
  if line[0] == "DIST":
    #rospy.loginfo(line)

    # Expected (valid) data length.
    valid_length = 2 + num_anchors * 6 + 5
      
    # Check if serial data is valid based on total length and number of anchors.
    if len(line) == valid_length and int(line[1]) == num_anchors:
        anchor_data_index = 2
        anchor_data_length = num_anchors * 6
        anchor_data = line[anchor_data_index : anchor_data_index + anchor_data_length]
    
         # Update Anchor objects.
        for index in range(num_anchors):
          offset = index * 6
      
          anchors[index]["id"] = anchor_data[1 + offset]
          anchors[index]["x"] = anchor_data[2 + offset]
          anchors[index]["y"] = anchor_data[3 + offset]
          anchors[index]["z"] = anchor_data[4 + offset]
          anchors[index]["distance"] = anchor_data[5 + offset]

        tag_data_index = anchor_data_index + anchor_data_length
        tag_data = line[tag_data_index : ]

        # Update Tag object.
        tag["x"] = float(tag_data[1])
        tag["y"] = float(tag_data[2])
        tag["z"] = float(tag_data[3])
        tag["quality_factor"] = float(tag_data[4])

def publishData():
  global serialPortDWM1001, anchors, tag, tag_msg, anchor_msgs, tag_pub, anchor1_pub, anchor2_pub, anchor3_pub, anchor4_pub
  
  # just read everything from serial port
  serial_line = serialPortDWM1001.read_until()
   
  # Parse raw line.
  parseSerial(serial_line)

  # Populate ROS messages.
  time_stamp = rospy.get_rostime()
 
  tag_msg.header.stamp = time_stamp
  tag_msg.x = tag["x"]
  tag_msg.y = tag["y"]
  tag_msg.z = tag["z"]
  tag_msg.quality_factor = tag["quality_factor"]

  for index in range(len(anchors)):
    anchor_msgs[index].header.stamp = time_stamp
    anchor_msgs[index].id = anchors[index]["id"]
    anchor_msgs[index].x = anchors[index]["x"]
    anchor_msgs[index].y = anchors[index]["y"]
    anchor_msgs[index].z = anchors[index]["z"]
    anchor_msgs[index].distance = anchors[index]["distance"]

  # Publish UWB data to ROS.
  tag_pub.publish(tag_msg)
  anchor1_pub.publish(anchor1_msg) 
  anchor2_pub.publish(anchor2_msg)
  anchor3_pub.publish(anchor3_msg)
  anchor4_pub.publish(anchor4_msg)
  
'''
def end(rate):
    rospy.loginfo("Quitting, and sending reset command to dev board")
    serialPortDWM1001.write(b'reset')
    serialPortDWM1001.write(b'\r')
    rate.sleep()
    
    if "reset" in serialReadLine:
        rospy.loginfo("succesfully closed ")
        serialPortDWM1001.close()
'''
if __name__ == '__main__':
    
    rospy.init_node("dwm1001_node")
    rate = rospy.Rate(10)
    init()
    set_rate()
    run()
    
    try:
        while not rospy.is_shutdown():
            publishData()
            rate.sleep()
   
    except rospy.ROSInterruptException:
        pass
	#rospy.loginfo("end")
        #end(rate)
