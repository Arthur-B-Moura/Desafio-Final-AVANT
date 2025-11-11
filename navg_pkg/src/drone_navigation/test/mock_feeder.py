#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32MultiArray
import math

class Feeder(Node):
    def __init__(self):
        super().__init__('mock_feeder')
        self.state_pub = self.create_publisher(State, '/mavros/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        self.vision_state_pub = self.create_publisher(String, '/vision/state', 10)
        self.line_pos_pub = self.create_publisher(Int32MultiArray, '/vision/line_position', 10)
        self.gancho_status_pub = self.create_publisher(String, '/gancho/status', 10)

        self.t = 0.0
        self.altitude = 0.0
        self.timer = self.create_timer(0.1, self.step)
        # boot -> line_follow -> centering -> return -> landing -> done
        self.phase = 'boot'
        self.armed = True

    def publish_state(self, mode='GUIDED'):
        s = State()
        s.mode = mode
        s.armed = self.armed
        s.connected = True
        self.state_pub.publish(s)

    def publish_pose(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        p = PoseStamped()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        self.pose_pub.publish(p)

    def publish_line(self, x, y):
        msg = Int32MultiArray()
        msg.data = [x, y]
        self.line_pos_pub.publish(msg)

    def step(self):
        # Estado padrão (mode/armed)
        self.publish_state()

        # 1) Subida até 3m, depois inicia visão
        if self.phase == 'boot':
            self.altitude = min(3.0, self.altitude + 0.5 * 0.1)  # ~0.5 m/s
            self.publish_pose(z=self.altitude)
            if self.altitude >= 3.0:
                self.phase = 'line_follow'

        # 2) Seguindo linha
        if self.phase == 'line_follow':
            cx = 320 + int(60 * math.sin(self.t))
            self.publish_line(cx, 240)
            st = String(); st.data = 'seguindo'
            self.vision_state_pub.publish(st)
            if self.t > 10.0:
                st.data = 'vermelho_detectado'
                self.vision_state_pub.publish(st)
                self.publish_line(340, 260)
                self.phase = 'centering'

        # 3) Centralizando até “gancho soltar”
        if self.phase == 'centering':
            self.publish_line(325, 245)
            if self.t > 13.0:
                g = String(); g.data = 'released'
                self.gancho_status_pub.publish(g)
                self.phase = 'return'

        # 4) Retorno “próximo” do home por alguns segundos
        if self.phase == 'return':
            self.publish_pose(x=0.05*math.cos(self.t), y=0.05*math.sin(self.t), z=self.altitude)
            if self.t > 16.0:
                self.phase = 'landing'

        # 5) Sequência de pouso e desarme
        if self.phase == 'landing':
            # Desce ~0.3 m/s
            self.altitude = max(0.0, self.altitude - 0.3 * 0.1)
            self.publish_pose(z=self.altitude)
            if self.altitude <= 0.05:
                # Simula desarmar
                self.armed = False
                self.publish_state()
                self.phase = 'done'

        # 6) Final
        if self.phase == 'done':
            # Mantém no chão, desarmado
            self.publish_pose(z=0.0)

        self.t += 0.1

def main():
    rclpy.init()
    node = Feeder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()