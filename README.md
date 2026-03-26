# edag_dron
 
https://github.com/bitcraze/crazyflie-simulation/tree/pid_gazebo

https://repositorio.tec.mx/items/4aab863a-e919-4e35-9526-a2bd274342a0

ros2 topic pub /dron/motor_1/speed std_msgs/msg/Float64 "{data: 500.0}"

ros2 topic pub /dron/cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.2}}"

Micro-XRCE-DDS-Agent udp4 -p 8888

MicroXRCEAgent udp4 -p 8888    Este es el bueno

make px4_sitl ignition_gazebo_iris
make px4_sitl gz_iris
make list_config_targets | grep gazebo
make px4_sitl gz_x500      Este es el bueno

/fmu/in/trajectory_setpoint