#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_vg_modbus_tcp.comModbusTcp import communication
from onrobot_vg_control.baseOnRobotVG import onrobotbaseVG
from onrobot_vg_control.msg import OnRobotVGInput
from onrobot_vg_control.msg import OnRobotVGOutput


class OnRobotVGTcp(Node):
    def __init__(self):
        super().__init__('onrobot_vg_tcp_node')

        # Retrieve parameters
        ip = self.declare_parameter('onrobot.ip', '192.168.1.1').value
        port = self.declare_parameter('onrobot.port', '502').value
        changer_addr = self.declare_parameter('onrobot.changer_addr', '65').value
        dummy = self.declare_parameter('onrobot.dummy', False).value

        # Gripper is a VG gripper with a Modbus/TCP connection
        self.gripper = onrobotbaseVG()
        self.gripper.client = communication(dummy)

        # Connects to the IP address received as an argument
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic named 'OnRobotVGInput'
        self.pub = self.create_publisher(OnRobotVGInput, 'OnRobotVGInput', 10)

        # The Gripper command is received from the topic named 'OnRobotVGOutput'
        self.create_subscription(OnRobotVGOutput, 'OnRobotVGOutput', self.gripper.refreshCommand, 10)

        self.prev_msg = []
        self.timer = self.create_timer(0.1, self.main_loop)
    def main_loop(self):
        # Get and publish the Gripper status
        status = self.gripper.getStatus()
        self.pub.publish(status)

            # Send the most recent command
        if not self.prev_msg == self.gripper.message:  # find new message
            self.get_logger().info("Sending message.")
            self.gripper.sendCommand()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OnRobotVGTcp()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()