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
/fmu/in/vehicle_command

s
commander arm -f
commander takeoff
commander land

el git clone debe de ser en la carpeta Documents, el launch lo ocupa

el parameters.bson esta en /PX4-Autopilot/build/px4-autopilot/rootfs (en teoria, no se si sea ese realmente)

param ser MPC_XY_VEL_MAX 2.0
param save

gz topic -l
gz topic -e -t /camera_main

ros2 run ros_gz_bridge parameter_bridge '/camera_main@sensor_msgs/msg/Image@gz.msgs.Image'

ros2 run rqt_image_view rqt_image_view


Para correr lo basico:
MicroXRCEAgent udp4 -p 8888
cd src
cd PX4-Autopilot
make px4_sitl gz_x500

cd ..
cd ..
python3 camera.py
ros2 run vision receptor_udp
ros2 run master mission_handler
ros2 run rqt_image_view rqt_image_view

o

ros2 launch master gazebo_launch.py


Para modificar los modelos
edag_dron/src/PX4-Autopilot/Tools/simulation/gz/
Los modelos van dentro de models
Luego hay que modificar el link del default.sdf en worlds