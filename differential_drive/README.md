# differential_drive

This package calculates the odometry of a differential drive robot and publishes an odometry message along the tf transform of the base link to the odom frame. It also takes twist commands and generates velocity commands for the right and left sides of the robot,.

## diff_drive topic
### Subscribed topics
* cmd_vel - (geometry_msgs/Twist): Twist commands
* wheel_enc - (differential_drive/Encoders): Encoder values accounting for wrap around of both sides of the robot

### Published topics
* wheel_vel_target - (differential_drive/VelocityTargets): Velocity targets for both sides of the robot
* odom - (nav_msgs/Odometry): Generated odometry message

### Parameters
* ~base_width (float, default:0.2): Width between the left wheel to the right wheel
* ~timeout_ticks (int, default:20): Number of loop iterations before a timeout occurs
* ~ticks_per_meter (int, default:50): Number of encoder ticks per meter
* ~encoder_min (int, default:-32768): Minimum encoder value
* ~encoder_max (int, default:32768): Maximum encoder value
* ~base_frame_id (string, default:"base_link"): Name of base frame
* ~odom_frame_id (string, default:"odom"): Name of odom frame
* ~publish_tf (bool, default:true): Broadcast tf transform for base_link to odom