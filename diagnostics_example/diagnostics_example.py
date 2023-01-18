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
        self.temp = KeyValue()
        self.temp.value = str(0)
        self.temp.key = "Motor Temp (deg)"

        # Create and fill a status message, one per device/module that is returning a status
        self.status = DiagnosticStatus()
        # Levels can be OK/WARN/ERROR/STALE
        self.status.level = DiagnosticStatus.OK
        self.status.name = "Motor Temperature"
        self.status.message = "Initialised"
        self.status.hardware_id = "mcp9600"
        #add the key value pairs here
        self.status.values.append(self.temp)

        # the /diagnostics topic must be published as msg type diagnosticArray
        # this is an array of all of the DiagnosticStatus msgs in the node
        self.diaArray = DiagnosticArray()
        self.diaArray.status.append(self.status)

        # Publish initial status
        self.diagnostic_pub.publish(self.diaArray)

        self.timer = self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        # If you need a heartbeat or regualar debug info you could publish using a timer
        # or just publish where you would report a status change.
        temp = random.randrange(0, 100)
        self.temp.value = str(temp)
        if temp < 70:
            self.status.level = DiagnosticStatus.OK
            self.status.message = "All good in the hood"
        elif 71 < temp < 81:
            self.status.level = DiagnosticStatus.WARN
            self.status.message = "It's getting hot in here..."
        else: 
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = "Damn that's hot"
        self.diagnostic_pub.publish(self.diaArray)

def main(args=None):
    rclpy.init(args=args) #required in all ros2 nodes to init ros2 communication
    node = DiagnosticsExample() #create node
    rclpy.spin(node) #keeps looping here, so callbacks can be called
    rclpy.shutdown() #tidy up at end of node

if __name__ == "__main__":
    main()