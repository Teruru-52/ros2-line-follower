# ros2-line-follower

## 1. Development Environment
- Ubuntu 22.04
- ROS2 Humble
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
