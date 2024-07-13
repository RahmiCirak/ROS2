
#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=gz_x500_depth ./build/px4_sitl_default/bin/px4 -i 1",
    
    "ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image /camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked /depth_camera@sensor_msgs/msg/Image@ignition.msgs.Image",

    "cd ~/VSco_WS/UAV_det && python3 uav_camera_det.py"

    # Run QGroundControl
    # "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
