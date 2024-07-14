# ros2-line-follower

- [STM32でmicro-ROSを使う](https://qiita.com/hirekatsu0523/items/83e1503ac4c7fdecd217)

## 1. Development Environment
- Ubuntu 24.04
- ROS2 Rolling
- micro-ROS

## 2. Packages
|            Package          |            Description        |
|            :---:            |             :---:             |
|     `follower_description`  |      robot description       　|
|     `follower_follow`       |      for following line  　　|
|     `follower_msgs`         |      custom messages     　　  |

## 3. Messages
### 3.3 Custom Message
|          data          |              Type              |             Description           |
|          :---:         |             :---:              |               :---:               |
|           `u1`         |        std_msgs::Float64       |            motor torque           |
|           `u2`         |        std_msgs::Float64       |            motor torque           |
|           `u3`         |        std_msgs::Float64       |            motor torque　　　　　　 |
|           `u4`         |        std_msgs::Float64       |            motor torque　　　　　　 |

### 3.2 Subscriber

### 3.3 Publisher

## 4. Execution
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
or
```bash
ros2 launch micro_ros_agent micro_ros_agent_launch.py 
```

```bash
ros2 launch follower_follow follower_follow.launch
```