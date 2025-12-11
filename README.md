# EMM Motor Driver for ROS 2

这是一个用于 EMM V5 串行电机的 ROS 2 驱动包。它支持通过串口控制多个电机，包括位置控制、速度控制和多电机同步控制。

## 功能特性

- **多电机支持**：通过单个串口控制多个电机。
- **灵活配置**：通过 YAML 文件配置串口参数和电机 ID。
- **多种控制模式**：
  - 速度控制
  - 位置控制（相对/绝对）
  - 同步位置控制
  - 同步速度控制
- **状态反馈**：实时发布电机的位置和速度信息。

## 安装与构建

1.  将此包放入您的 ROS 2 工作空间的 `src` 目录中。
2.  在工作空间根目录下运行构建命令：

    ```bash
    colcon build --packages-select emm_motor_driver
    ```

3.  加载环境：

    ```bash
    source install/setup.bash
    ```

## 使用方法

### 1. 配置

编辑 `src/emm_motor_driver/config/params.yaml` 文件以匹配您的硬件设置：

```yaml
motor_driver_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"  # 串口设备路径
    baudrate: 115200             # 波特率
    loop_rate: 5.0               # 状态发布频率 (Hz)
    motor_ids: [1, 2]            # 电机 ID 列表
    motor_names: ["joint_1", "joint_2"] # 对应的 ROS 关节名称
```

### 2. 启动驱动节点

```bash
ros2 launch emm_motor_driver driver.launch.py
```

### 3. 控制接口

驱动节点订阅 `/motor_cmd` 话题，接收 `std_msgs/String` 类型的 JSON 格式指令。

#### 指令格式示例

**单电机控制：**

*   **使能**：`{"cmd": "enable", "id": 1}` (省略 id 则控制所有)
*   **失能**：`{"cmd": "disable", "id": 1}`
*   **停止**：`{"cmd": "stop", "id": 1}`
*   **位置清零**：`{"cmd": "reset", "id": 1}`
*   **速度控制**：
    ```json
    {
      "id": 1,
      "cmd": "vel",
      "val": 500,      // 速度 (RPM)
      "acc": 50,       // 加速度
      "dir": 0         // 方向 (0=CW, 1=CCW)
    }
    ```
*   **位置控制**：
    ```json
    {
      "id": 1,
      "cmd": "pos",
      "val": 3600,     // 脉冲数
      "speed": 600,    // 速度 (RPM)
      "acc": 50,       // 加速度
      "dir": 0,        // 方向
      "rel": true      // true=相对运动, false=绝对运动
    }
    ```

**多电机同步控制：**

*   **同步位置控制**：
    ```json
    {
      "cmd": "sync_pos",
      "motors": [
        {"id": 1, "val": 3200, "speed": 600, "dir": 0, "rel": true},
        {"id": 2, "val": 3200, "speed": 600, "dir": 1, "rel": true}
      ]
    }
    ```
*   **同步速度控制**：
    ```json
    {
      "cmd": "sync_vel",
      "motors": [
        {"id": 1, "val": 500, "dir": 0},
        {"id": 2, "val": 500, "dir": 1}
      ]
    }
    ```

#### 命令行调试

您可以使用 `ros2 topic pub` 直接发送指令进行测试：

**使能所有电机：**
```bash
ros2 topic pub /motor_cmd std_msgs/msg/String "data: '{\"cmd\": \"enable\"}'" --once
```

**速度控制 (电机1以500RPM转动)：**
```bash
ros2 topic pub /motor_cmd std_msgs/msg/String "data: '{\"id\": 1, \"cmd\": \"vel\", \"val\": 500, \"acc\": 50}'" --once
```

**位置控制 (电机2相对移动3600脉冲)：**
```bash
ros2 topic pub /motor_cmd std_msgs/msg/String "data: '{\"id\": 2, \"cmd\": \"pos\", \"val\": 3600, \"speed\": 600, \"rel\": true}'" --once
```

**停止电机1：**
```bash
ros2 topic pub /motor_cmd std_msgs/msg/String "data: '{\"id\": 1, \"cmd\": \"stop\"}'" --once
```

### 4. 运行示例

包中包含一个同步控制的示例节点：

```bash
ros2 run emm_motor_driver sync_example
```

该示例将演示：
1. 使能所有电机
2. 位置清零
3. 同步运动（电机1顺时针，电机2逆时针）
4. 等待
5. 同步反向运动
6. 停止

**注意**：该节点同时订阅 `/motor_states` 话题，会在终端实时打印电机的位置和速度反馈信息。

## 话题列表

-   **发布**：
    -   `/motor_states` (`sensor_msgs/JointState`): 电机的位置和速度反馈。

-   **订阅**：
    -   `/motor_cmd` (`std_msgs/String`): 控制指令输入。
