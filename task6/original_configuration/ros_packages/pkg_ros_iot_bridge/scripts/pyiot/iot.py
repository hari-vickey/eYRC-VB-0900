from multiprocessing.dummy import Pool
import time
import requests
import sys
import paho.mqtt.client as mqtt
URL1 = "https://script.google.com/macros/s/AKfycbxZjLKDKs_WZ6DHznHcRSnoy2_bh68jGCpYCY4dtaQXAVcoszN8VL4_/exec"
URL2 = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
i = 1
j = 1
k = 1
l = 1
class print_colour:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# -----------------  MQTT SUB -------------------
def iot_func_callback_sub(client, userdata, message):
    print("message received " ,str(message.payload.decode("utf-8")))
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

def mqtt_subscribe_thread_start(arg_callback_func, arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_qos):
    try:
        mqtt_client = mqtt.Client()
        mqtt_client.on_message = arg_callback_func
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.subscribe(arg_mqtt_topic, arg_mqtt_qos)
        time.sleep(1) # wait
        # mqtt_client.loop_forever() # starts a blocking infinite loop
        mqtt_client.loop_start()    # starts a new thread
        return 0
    except:
        return -1


# -----------------  MQTT PUB -------------------
def mqtt_publish(arg_broker_url, arg_broker_port, arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos):
    try:
        mqtt_client = mqtt.Client("mqtt_pub")
        mqtt_client.connect(arg_broker_url, arg_broker_port)
        mqtt_client.loop_start()

        print("Publishing message to topic", arg_mqtt_topic)
        mqtt_client.publish(arg_mqtt_topic, arg_mqtt_message, arg_mqtt_qos)
        time.sleep(0.1) # wait

        mqtt_client.loop_stop() #stop the loop
        return 0
    except:
        return -1


# ------- Push data into Inventory sheet of team spreadsheet -------
def push_to_inventorysheet(arg_sku, arg_item, arg_pri, arg_stno, arg_cost, arg_quan):
    global i
    parameters = {"id":"Inventory", "Team Id":"VB#0900", "Unique Id":"BiAdKaHa", "SKU":arg_sku, "Item":arg_item, "Priority":arg_pri, "Storage Number":arg_stno, "Cost":arg_cost, "Quantity": arg_quan}
    response = requests.get(URL1, params=parameters)
    print('\033[94m' + "pushed to Inventory sheet of Team spreadsheet " + str(i) + " " + str(response.content)  + '\033[0m')
    response = requests.get(URL2, params=parameters)
    print('\033[94m' + "pushed to Inventory sheet of E-yantra spreadsheet " + str(i) + " " + str(response.content) + '\033[0m')
    i += 1

# ------- Push data into Inventory sheet of team spreadsheet -------
def push_to_incomingorderssheet(arg_ordid, arg_time, arg_item, arg_pri, arg_quan, arg_city, arg_lon, arg_lat, arg_cost):
    global j
    parameters = {"id":"IncomingOrders", "Team Id":"VB#0900", "Unique Id":"BiAdKaHa", "Order ID":arg_ordid, "Order Date and Time":arg_time, "Item":arg_item, "Priority":arg_pri, "Order Quantity": arg_quan, "City":arg_city, "Longitude":arg_lon, "Latitude":arg_lat, "Cost":arg_cost}
    response = requests.get(URL1, params=parameters)
    print('\033[94m' + "pushed to IncomingOrders sheet of Team spreadsheet " + str(j) + " " + str(response.content) + '\033[0m')
    response = requests.get(URL2, params=parameters)
    print('\033[94m' + "pushed to IncomingOrders sheet of E-yantra spreadsheet " + str(j) + " " + str(response.content) + '\033[0m')
    j += 1

# ------- Push data into OrdersDispatched sheet of team spreadsheet -------
def push_to_ordersdispatchedsheet(arg_ordid, arg_city, arg_item, arg_pri, arg_dispquan, arg_cost, arg_dispstat, arg_disptime):
    global k
    parameters = {"id":"OrdersDispatched", "Team Id":"VB#0900", "Unique Id":"BiAdKaHa", "Order ID":arg_ordid, "City":arg_city, "Item":arg_item, "Priority":arg_pri, "Dispatch Quantity": arg_dispquan, "Cost":arg_cost, "Dispatch Status":arg_dispstat, "Dispatch Date and Time":arg_disptime}
    response = requests.get(URL1, params=parameters)
    print('\033[94m' + "pushed to OrdersDispatched sheet of Team spreadsheet " + str(k) + " "  + str(response.content) + '\033[0m')
    response = requests.get(URL2, params=parameters)
    print('\033[94m' + "pushed to OrdersDispatched sheet of E-yantra spreadsheet " + str(k) + " " + str(response.content) + '\033[0m')
    k += 1

# ------- Push data into OrdersShipped sheet of team spreadsheet -------
def push_to_ordersshippedsheet(arg_ordid, arg_city, arg_item, arg_pri, arg_shipquan, arg_cost, arg_shipstat, arg_shiptime, arg_est):
    global l
    parameters = {"id":"OrdersShipped", "Team Id":"VB#0900", "Unique Id":"BiAdKaHa", "Order ID":arg_ordid, "City":arg_city, "Item":arg_item, "Priority":arg_pri, "Shipped Quantity": arg_shipquan, "Cost":arg_cost, "Shipped Status":arg_shipstat, "Shipped Date and Time":arg_shiptime, "Estimated Time of Delivery":arg_est}
    response = requests.get(URL1, params=parameters)
    print('\033[94m' + "pushed to OrdersShipped sheet of Team spreadsheet " + str(l) + " " + str(response.content) + '\033[0m')
    response = requests.get(URL2, params=parameters)
    print('\033[94m' + "pushed to OrdersShipped sheet of E-yantra spreadsheet " + str(l) + " " + str(response.content) + '\033[0m')
    l += 1
