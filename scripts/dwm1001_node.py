#!/usr/bin/env python

import rospy, time, serial, os
from ros_dwm1001.msg import Tag, Anchor

serialReadLine = ""

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

tag_pub = rospy.Publisher("tag", Tag, queue_size=1)
anchor1_pub = rospy.Publisher("anchor1", Anchor, queue_size=1)
anchor2_pub = rospy.Publisher("anchor2", Anchor, queue_size=1)
anchor3_pub = rospy.Publisher("anchor3", Anchor, queue_size=1)

def run():
    # allow serial port to be detected by user
    os.popen("sudo chmod 777 /dev/ttyACM0", "w")

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
        # sleep for half a second
        time.sleep(0.5)
        serialPortDWM1001.write(b'\r')
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        serialPortDWM1001.write(b'\r')

        # give some time to DWM1001 to wake up
        time.sleep(2)
        # send command lec, so we can get positions is CSV format
        serialPortDWM1001.write(b'lec')
        serialPortDWM1001.write(b'\r')
        rospy.loginfo("Reading DWM1001 coordinates")
        rospy.loginfo("test")
    
    else:
        rospy.loginfo("Can't open port: "+ str(serialPortDWM1001.name))
 
def parse(raw_line):
  # Remove 2 blank spaces at end of line first.
  # Split by comma after.
  line = raw_line.strip().split(",")

  num_anchors = int(line[1])

  anchor_data_index = 2
  anchor_data_length = num_anchors*6
  anchor_data = line[anchor_data_index : anchor_data_index + anchor_data_length]
  print(anchor_data)
  #print(len(anchor_data))

  # Create anchor objects.
  for num in range(num_anchors):
    offset = num*6
    anchor = {
    "anchor_number" : anchor_data[0 + offset],
      "anchor_id" : anchor_data[1 + offset],
      "x" : anchor_data[2 + offset],
      "y" : anchor_data[3 + offset],
      "z" : anchor_data[4 + offset],
      "distance" : anchor_data[5 + offset]
    }
    print(anchor)

  tag_data_index = anchor_data_index + anchor_data_length
  tag_data = line[tag_data_index : tag_data_index + 5]
  print(tag_data)
  #print(len(tag_data))

  # Create tag object.
  tag = {
    "x" : float(tag_data[1]),
    "y" : float(tag_data[2]),
    "z" : float(tag_data[3]),
    "quality_factor" : float(line[4])
  }
  print(tag)

def publish():
    # just read everything from serial port
    serialReadLine = serialPortDWM1001.read_until()

    # Parse raw line.
    line = parse(serialReadLine)
    rospy.loginfo(line)
    #self.pubblishCoordinatesIntoTopics(self.splitByComma(serialReadLine))
    
def end(rate):
    rospy.loginfo("Quitting, and sending reset command to dev board")
    serialPortDWM1001.write(b'reset')
    serialPortDWM1001.write(b'\r')
    rate.sleep()
    
    if "reset" in serialReadLine:
        rospy.loginfo("succesfully closed ")
        serialPortDWM1001.close()

if __name__ == '__main__':
    
    rospy.init_node("dwm1001_node")
    rate = rospy.Rate(15)
    run()
    
    try:
        while not rospy.is_shutdown():
            publish()
            rate.sleep()
   
    except rospy.ROSInterruptException:
        rospy.loginfo("end")
        end(rate)


