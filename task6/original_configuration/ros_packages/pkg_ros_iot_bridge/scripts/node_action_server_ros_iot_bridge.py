#!/usr/bin/env python
# ROS Node - Action Server - IoT ROS Bridge
'''
This python file runs a ROS-node of name  IotRosBridgeActionServer.
This node is the bridge between ROS and IOT.
This node acts as an Action Server
So it will process the goals Asynchronously
This node subscribes to the mqtt topic to receive incomingorders
This node also publishes the orders received to ros topic
All the data are pushed to the spreadsheets using pyiot module
'''
# Importing all Required Modules
import threading
from datetime import datetime, timedelta
import json
import rospy
import actionlib
from pkg_ros_iot_bridge.msg import msgRosIotAction, msgRosIotResult
from pkg_ros_iot_bridge.msg import msgMqttSub
from pyiot import iot

# Class IotRosBridegeActionServer
class IotRosBridgeActionServer(object):
    """
    Class IotRosBridgeActionServer is a action server
    to node_t5_1 and node_t5_2
    """
    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_ros_iot',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)
        '''
            * self.on_goal - It is the fuction pointer
            It points to a function which will be called
            when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer
            It points to a function which will be called
            when the Action Server receives a Cancel Request.
        '''
        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published
        # To ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe
        # To ROS Topic (/ros_iot_bridge/mqtt/sub)
        # to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_ros_topic,
                                               msgMqttSub,
                                               queue_size=10)
        # self.mqtt_sub_callback() function will be called
        # when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo('\033[94m' + "MQTT Subscribe Thread Started" + '\033[0m')
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")
        #Printing All parameters
        rospy.loginfo("parameters Loaded to ROS-IOT Server\n" + str(param_config_iot))
        #Starting the Action Server
        self._as.start()
        rospy.loginfo("Action server is up")

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        """
        This is a callback function for mqtt Subscription
        This is function is called whenever, there is a order
        from the user
        This function is also threaded to push incoming orders
        to spreadsheet
        """
        payload = str(message.payload.decode("utf-8"))
        print("[MQTT SUB CB] Userdata: ", userdata)
        print("[MQTT SUB CB] Client: ", client)
        print("[MQTT SUB CB] Message: ", payload)
        print("[MQTT SUB CB] Topic: ", message.topic)
        thread = threading.Thread(name="worker", target=self.func_pushdata_orders, args=(payload,))
        thread.start()
        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload
        self._handle_ros_pub.publish(msg_mqtt_sub)

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        """
        On receiving a goal from client this function will be executed.
        This function will accept the goals which are appropriate
        """
        goal = goal_handle.get_goal()
        rospy.loginfo("Received new goal from Client")
        sheet = goal.sheet_name
        # Validate incoming goal parameters
        if sheet == "Inventory" or sheet == "OrdersDispatched" or sheet == "OrdersShipped":
            goal_handle.set_accepted()
            self.process_goal(goal_handle)
        else:
            goal_handle.set_rejected()
            return

    # This function is a separate thread to process Goal.
    def process_goal(self, goal_handle):
        """
        This function is the process goal Asynchronously
        and calls the appropriate function
        to push data to spreadsheet
        """
        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))
        goal = goal_handle.get_goal()
        # Goal Processing
        if goal.sheet_name == "Inventory":
            self.func_pushdata_inventory(goal)
        elif goal.sheet_name == "OrdersDispatched":
            self.func_pushdata_dispatch(goal)
        elif goal.sheet_name == "OrdersShipped":
            self.func_pushdata_shipped(goal)
        rospy.loginfo("Send goal result to client")
        result = msgRosIotResult()
        result.flag_success = True
        rospy.loginfo("Succeeded")
        goal_handle.set_succeeded(result)
        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    # Function to push data to Inventory Sheet
    @classmethod
    def func_pushdata_inventory(cls, goal):
        """
        This function is to push package data in shelf to
        Inventory spreadsheet using pyiot module
        """
        color = goal.package_color
        name = goal.package_name
        if color == "red":
            split = name.split('n')
            skt = datetime.now().strftime('%m%y')
            sku = 'R' + split[1] + skt
            item = "Medicine"
            pri = "HP"
            stn = str(split[1])
            stno = 'R' + stn[0] + ' ' +  'C' + stn[1]
            cost = 450
        elif color == "yellow":
            split = name.split('n')
            skt = datetime.now().strftime('%m%y')
            sku = 'Y' + split[1] + skt
            item = "Food"
            pri = "MP"
            stn = str(split[1])
            stno = 'R' + stn[0] + ' ' +  'C' + stn[1]
            cost = 250
        else:
            split = name.split('n')
            skt = datetime.now().strftime('%m%y')
            sku = 'G' + split[1] + skt
            item = "Clothes"
            pri = "LP"
            stn = str(split[1])
            stno = 'R' + stn[0] + ' ' + 'C' + stn[1]
            cost = 150
        iot.push_to_inventorysheet(sku, item, pri, stno, cost, 1)
        result = msgRosIotResult()
        result.flag_success = True

    #Function to push data to incoming orders sheet
    @classmethod
    def func_pushdata_orders(cls, message):
        """
        This function is to push incoming orders to
        IncomingOrders spreadsheet using pyiot module
        """
        orders = json.loads(message)
        if orders['item'] == "Medicine":
            priority = "HP"
            ass_cost = 450
        elif orders['item'] == "Food":
            priority = "MP"
            ass_cost = 250
        else:
            priority = "LP"
            ass_cost = 150
        iot.push_to_incomingorderssheet(orders['order_id'],
                                        orders['order_time'],
                                        orders['item'],
                                        priority,
                                        1,
                                        orders['city'],
                                        orders['lon'],
                                        orders['lat'],
                                        ass_cost)

    # Function to push data to OrdersDispatched Sheet
    @classmethod
    def func_pushdata_dispatch(cls, goal):
        """
        This function is to push Dispatched orders to
        OrdersDispatched spreadsheet using pyiot module
        """
        color = goal.package_color
        status = goal.dispatch_status
        time = goal.dispatch_time
        ordid = goal.order_id
        city = goal.city
        if color == "red":
            item = "Medicine"
            pri = "HP"
            cost = 450
        elif color == "yellow":
            item = "Food"
            pri = "MP"
            cost = 250
        else:
            item = "Clothes"
            pri = "LP"
            cost = 150
        iot.push_to_ordersdispatchedsheet(ordid, city, item, pri, 1, cost, status, time)
        result = msgRosIotResult()
        result.flag_success = True

    # Function to push data to Ordersshipped Sheet
    @classmethod
    def func_pushdata_shipped(cls, goal):
        """
        This function is to push Shipped package orders to
        OrdersShipped spreadsheet using pyiot module
        """
        color = goal.package_color
        status = goal.shipped_status
        time = goal.shipped_time
        ordid = goal.order_id
        city = goal.city
        if color == "red":
            item = "Medicine"
            pri = "HP"
            cost = 450
            delivery = datetime.now() + timedelta(days=1)
        elif color == "yellow":
            item = "Food"
            pri = "MP"
            cost = 250
            delivery = datetime.now() + timedelta(days=3)
        else:
            item = "Clothes"
            pri = "LP"
            cost = 150
            delivery = datetime.now() + timedelta(days=5)
        est = delivery.strftime('%Y-%m-%d')
        iot.push_to_ordersshippedsheet(ordid, city, item, pri, 1, cost, status, time, str(est))
        result = msgRosIotResult()
        result.flag_success = True
    #Function to cancel a goal
    @classmethod
    def on_cancel(cls, goal):
        """
        This function is to cancel any incoming goal
        which is requested by the client
        """
        goal_id = goal.get_goal_id()
        rospy.loginfo("Received cancel request for the goal " + str(goal_id))

# Main Function
def main():
    """
    This is the main method of this code
    This method will Initialize ROS Node,
    Also, create a instance to the Class IotRosBridgeActionServer
    This main function also uses rospy spin to make the callback function
    executes infintely
    """
    # Initialize Node Iot ROS Bridge Action Server
    rospy.init_node('node_iot_ros_bridge_action_server')
    # Creating a Object of class IotRosBridgeActionServer
    IotRosBridgeActionServer()
    # Do not exit and Loop Forever
    rospy.spin()
if __name__ == '__main__':
    main()
