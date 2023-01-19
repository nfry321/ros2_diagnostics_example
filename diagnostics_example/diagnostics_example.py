#!/usr/bin/env python3 
import rclpy #required to use ros2 functionalities
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default as default
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import random
class DiagnosticsExample(Node):

    def __init__ (self): 
        super().__init__("diagnostics_example")
        self.get_logger().info("Initialising Example") #code to do stuff here e.g log

        self.diagnostic_pub = self.create_publisher(
			msg_type = DiagnosticArray,
			topic = "diagnostics",
			qos_profile = default,
		)
        # Create Key-Value pairs. These contain detailed info or data related to the status
        # We can add as many as we want in an array as part of a DiagnosticStatus message
        self.temp1 = KeyValue()
        self.temp1.value = "--"
        self.temp1.key = "Temp (deg)"
        self.temp2 = KeyValue()
        self.temp2.value = "--"
        self.temp2.key = "Temp (deg)"

        # Create and fill a status message, one per device/module that is returning a status
        self.status1 = DiagnosticStatus()
        # Levels can be OK/WARN/ERROR/STALE
        self.status1.level = DiagnosticStatus.OK
        self.status1.name = "Sensor 1"
        self.status1.message = "Initialised"
        self.status1.hardware_id = "mcp9600 0x66"
        #add the key value pairs here
        self.status1.values.append(self.temp1)

        self.status2 = DiagnosticStatus()
        self.status2.level = DiagnosticStatus.OK
        self.status2.name = "Temperature Sensor"
        self.status2.message = "Initialised"
        self.status2.hardware_id = "mcp9600 0x67"
        #add the key value pairs here
        self.status2.values.append(self.temp2)

        # the /diagnostics topic must be published as msg type diagnosticArray
        # this is an array of all of the DiagnosticStatus msgs in the node
        self.diaArray = DiagnosticArray()
        self.diaArray.header.stamp.sec = 999
        self.diaArray.status.append(self.status1)
        self.diaArray.status.append(self.status2)

        # Publish initial status
        self.diagnostic_pub.publish(self.diaArray)

        self.timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        # If you need a heartbeat or regualar debug info you could publish using a timer
        # or just publish where you would report a status change.
        temp = random.randrange(0, 100)
        self.checkTemp(temp,1)
        temp = random.randrange(0, 100)
        self.checkTemp(temp,2)
        self.diagnostic_pub.publish(self.diaArray)

    def checkTemp(self, temp, sensor):
        if sensor == 1:
            status = self.status1
            keyval = self.temp1
        elif sensor == 2:
            status = self.status2
            keyval = self.temp2

        keyval.value = str(temp)
        if temp <= 50:
            status.level = DiagnosticStatus.OK
            status.message = "All good in the hood"
        elif 50 < temp < 75:
            status.level = DiagnosticStatus.WARN
            status.message = "It's getting hot in here..."
        else: 
            status.level = DiagnosticStatus.ERROR
            status.message = "Damn that's hot"
        

def main(args=None):
    rclpy.init(args=args) #required in all ros2 nodes to init ros2 communication
    node = DiagnosticsExample() #create node
    rclpy.spin(node) #keeps looping here, so callbacks can be called
    rclpy.shutdown() #tidy up at end of node

if __name__ == "__main__":
    main()