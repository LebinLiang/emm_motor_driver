import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json
import time

class SyncControlNode(Node):
    def __init__(self):
        super().__init__('sync_control_node')
        self.pub = self.create_publisher(String, 'motor_cmd', 10)
        self.sub = self.create_subscription(JointState, 'motor_states', self.state_callback, 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.step = 0
        self.get_logger().info('Sync Control Node Started')

    def state_callback(self, msg):
        """处理电机状态反馈"""
        info_str = "Motor Status: "
        for i, name in enumerate(msg.name):
            pos = msg.position[i]
            vel = msg.velocity[i]
            info_str += f"[{name}: Pos={pos:.2f}°, Vel={vel:.2f}RPM] "
        self.get_logger().info(info_str)

    def send_cmd(self, cmd_dict):
        msg = String()
        msg.data = json.dumps(cmd_dict)
        self.pub.publish(msg)
        self.get_logger().info(f'Sent: {msg.data}')

    def timer_callback(self):
        if self.step == 0:
            # 1. 使能所有电机
            self.send_cmd({"cmd": "enable"})
            self.step += 1
        elif self.step == 1:
            # 2. 位置清零
            self.send_cmd({"cmd": "reset"})
            self.step += 1
        elif self.step == 2:
            # 3. 同步运动: 电机1顺时针, 电机2逆时针
            cmd = {
                "cmd": "sync_pos",
                "motors": [
                    {"id": 1, "val": 3200, "speed": 600, "dir": 0, "rel": True},
                    {"id": 2, "val": 3200, "speed": 600, "dir": 1, "rel": True}
                ]
            }
            self.send_cmd(cmd)
            self.step += 1
        elif self.step == 3:
            # 等待运动完成 (此处仅做简单延时)
            self.step += 1
        elif self.step == 4:
            # 4. 同步反向运动
            cmd = {
                "cmd": "sync_pos",
                "motors": [
                    {"id": 1, "val": 3200, "speed": 600, "dir": 1, "rel": True},
                    {"id": 2, "val": 3200, "speed": 600, "dir": 0, "rel": True}
                ]
            }
            self.send_cmd(cmd)
            self.step += 1
        elif self.step == 5:
             # 5. 停止
            self.send_cmd({"cmd": "stop"})
            self.get_logger().info('Sequence complete')
            self.step += 1
            # self.destroy_node() # 可选: 序列完成后退出

def main(args=None):
    rclpy.init(args=args)
    node = SyncControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
