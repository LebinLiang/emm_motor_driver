import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import time
from .emm_driver import MotorController, Motor

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('motor_ids', [1])
        self.declare_parameter('motor_names', ['motor_1'])
        self.declare_parameter('loop_rate', 10.0)
        
        # 获取参数
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.motor_ids = self.get_parameter('motor_ids').value
        self.motor_names = self.get_parameter('motor_names').value
        loop_rate = self.get_parameter('loop_rate').value
        
        if len(self.motor_ids) != len(self.motor_names):
            self.get_logger().error('motor_ids and motor_names length mismatch!')
            return

        # 初始化控制器
        try:
            self.controller = MotorController(port, baud)
            self.get_logger().info(f'Connected to serial port {port} at {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            return

        # 添加电机
        self.id_map = {} # name -> id
        for i, mid in enumerate(self.motor_ids):
            name = self.motor_names[i]
            self.controller.add_motor(mid, name)
            self.id_map[name] = mid
            self.get_logger().info(f'Added motor {mid} as {name}')
            
        # 发布者
        self.joint_pub = self.create_publisher(JointState, 'motor_states', 10)
        
        # 订阅者
        self.cmd_sub = self.create_subscription(String, 'motor_cmd', self.cmd_callback, 10)
        
        # 定时器
        self.timer = self.create_timer(1.0/loop_rate, self.timer_callback)
        
        self.get_logger().info('Motor Driver Node Initialized')

    def cmd_callback(self, msg):
        """
        处理控制指令
        格式: JSON string
        {
            "id": 1,          // 可选，默认所有
            "cmd": "vel",     // enable, disable, reset, vel, pos, stop
            "val": 500,       // 速度(rpm) 或 位置(pulses)
            "acc": 50,        // 加速度
            "dir": 0,         // 0=CW, 1=CCW
            "rel": true       // 相对运动
        }
        """
        try:
            data = json.loads(msg.data)
            cmd_type = data.get('cmd')
            
            # 处理同步指令
            if cmd_type == 'sync_pos':
                motor_list = data.get('motors', [])
                ctrl_cmds = []
                for m_cmd in motor_list:
                    mid = m_cmd.get('id')
                    if mid is None: continue
                    
                    ctrl_cmds.append({
                        'addr': mid,
                        'direction': int(m_cmd.get('dir', 0)),
                        'velocity': int(m_cmd.get('speed', 500)),
                        'acceleration': int(m_cmd.get('acc', 50)),
                        'pulses': int(m_cmd.get('val', 0)),
                        'relative': bool(m_cmd.get('rel', True))
                    })
                self.controller.synchronous_position_control(ctrl_cmds)
                self.get_logger().info(f'Sent sync pos control to {len(ctrl_cmds)} motors')
                return
            elif cmd_type == 'sync_vel':
                motor_list = data.get('motors', [])
                ctrl_cmds = []
                for m_cmd in motor_list:
                    mid = m_cmd.get('id')
                    if mid is None: continue
                    
                    ctrl_cmds.append({
                        'addr': mid,
                        'direction': int(m_cmd.get('dir', 0)),
                        'velocity': int(m_cmd.get('val', 0)),
                        'acceleration': int(m_cmd.get('acc', 50))
                    })
                self.controller.synchronous_velocity_control(ctrl_cmds)
                self.get_logger().info(f'Sent sync vel control to {len(ctrl_cmds)} motors')
                return

            target_id = data.get('id')
            
            motors_to_ctrl = []
            if target_id is not None:
                m = self.controller.get_motor(target_id)
                if m: motors_to_ctrl.append(m)
            else:
                # Control all
                for mid in self.motor_ids:
                    motors_to_ctrl.append(self.controller.get_motor(mid))
            
            for motor in motors_to_ctrl:
                if cmd_type == 'enable':
                    motor.enable()
                    self.get_logger().info(f'Motor {motor.addr} Enabled')
                elif cmd_type == 'disable':
                    motor.disable()
                    self.get_logger().info(f'Motor {motor.addr} Disabled')
                elif cmd_type == 'reset':
                    motor.reset_position()
                    self.get_logger().info(f'Motor {motor.addr} Reset')
                elif cmd_type == 'stop':
                    motor.stop_now()
                    self.get_logger().info(f'Motor {motor.addr} Stopped')
                elif cmd_type == 'vel':
                    val = int(data.get('val', 0))
                    acc = int(data.get('acc', 50))
                    direction = int(data.get('dir', 0))
                    # 如果val为负，自动处理方向
                    if val < 0:
                        direction = 1 if direction == 0 else 0
                        val = abs(val)
                    motor.velocity_control(direction, val, acc)
                elif cmd_type == 'pos':
                    val = int(data.get('val', 0))
                    speed = int(data.get('speed', 500)) # pos control needs speed
                    acc = int(data.get('acc', 50))
                    direction = int(data.get('dir', 0))
                    rel = bool(data.get('rel', True))
                    motor.position_control(direction, speed, acc, val, rel)
                    
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON command')
        except Exception as e:
            self.get_logger().error(f'Command execution error: {e}')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for name in self.motor_names:
            mid = self.id_map[name]
            motor = self.controller.get_motor(mid)
            if motor:
                # 读取状态 (注意：串口读取是阻塞的，过多电机可能会导致循环变慢)
                # 建议优化：在单独线程读取，或者减少读取频率
                try:
                    pos = motor.get_current_position()
                    vel = motor.get_velocity()
                    
                    msg.name.append(name)
                    msg.position.append(float(pos)) # 角度
                    msg.velocity.append(float(vel)) # RPM
                except Exception as e:
                    self.get_logger().warn(f'Failed to read motor {mid}: {e}')
        
        self.joint_pub.publish(msg)

    def destroy_node(self):
        self.controller.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
