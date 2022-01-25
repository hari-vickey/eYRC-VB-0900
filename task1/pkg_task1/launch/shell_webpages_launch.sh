#!/bin/bash
echo ""
echo ""
echo "*** Task-1 ***"
echo ""
echo ""
echo ""
echo "To pass data to ROS IOT Bridge Node,  MQTT Client should Publish on this MQTT Topic:     eyrc/BiAdKaHa/iot_to_ros"
echo ""
echo "To get data from ROS IOT Bridge Node,  MQTT Client should Subscribe on this MQTT Topic:     eyrc/BiAdKaHa/ros_to_iot"
echo ""
echo ""
echo "Publish [start] on [eyrc/wt/mqtt/iot_to_ros] to start the turtle"
echo ""
echo ""
# Store URL in a variable
URL1="http://www.hivemq.com/demos/websocket-client/"
URL2="https://docs.google.com/spreadsheets/d/1jOrxGCkh6kU4UuMdLOQ37XkvztC-1chgZrSRxCtxBQ0/edit#gid=0"

# Print some message
echo "** Opening hivemq in Browser **"
echo "** Opening spreadsheet in Browser **"

# Use Browser to open the URL in a new window
x-www-browser -new-window $URL1 $URL2
 
