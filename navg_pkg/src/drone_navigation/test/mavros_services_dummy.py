#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class MavrosServicesDummy(Node):
    def __init__(self):
        super().__init__('mavros_services_dummy')
        self.create_service(SetMode, '/mavros/set_mode', self.on_set_mode)
        self.create_service(CommandBool, '/mavros/cmd/arming', self.on_arm)
        self.create_service(CommandTOL, '/mavros/cmd/takeoff', self.on_takeoff)
        self.create_service(CommandTOL, '/mavros/cmd/land', self.on_land)
        self.get_logger().info('MAVROS services dummy up: set_mode, arming, takeoff, land')

    def on_set_mode(self, req, res):
        self.get_logger().info(f"set_mode -> {req.custom_mode!r}")
        res.mode_sent = True
        return res

    def on_arm(self, req, res):
        self.get_logger().info(f"arming -> {bool(req.value)}")
        res.success = True
        return res

    def on_takeoff(self, req, res):
        self.get_logger().info(f"takeoff -> altitude={req.altitude}")
        res.success = True
        return res

    def on_land(self, req, res):
        self.get_logger().info("land -> solicitado")
        res.success = True
        return res

def main():
    rclpy.init()
    node = MavrosServicesDummy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()