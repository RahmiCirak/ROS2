# ROS2
My gradution project's steps
-----------------------------
current failitures :
-----------------------------
px4 terminalinden gelenler :
INFO  [commander] Ready for takeoff!
INFO  [uxrce_dds_client] synchronized with time offset 1716325905402526us
INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/battery_status data writer, topic id: 19
INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/estimator_status_flags data writer, topic id: INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/failsafe_flags data writer, topic id: 93
INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/manual_control_setpoint data writer, topic id:INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/position_setpoint_triplet data writer, topic iINFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/sensor_combined data writer, topic id: 203
INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/timesync_status data writer, topic id: 226
INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/vehicle_attitude data writer, topic id: 242
INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/vehicle_control_mode data writer, topic id: 24INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/vehicle_global_position data writer, topic id:INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/vehicle_gps_position data writer, topic id: 25INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/vehicle_local_position data writer, topic id: INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/vehicle_odometry data writer, topic id: 261
INFO  [uxrce_dds_client] successfully created rt/px4_2/fmu/out/vehicle_status data writer, topic id: 266
INFO  [uxrce_dds_client] time sync converged
WARN  [health_and_arming_checks] Preflight: GPS Horizontal Pos Drift too high
WARN  [health_and_arming_checks] Preflight: GPS Horizontal Pos Drift too high
WARN  [timesync] time jump detected. Resetting time synchroniser.
WARN  [uxrce_dds_client] time sync no longer converged
INFO  [uxrce_dds_client] time sync converged
ERROR [vehicle_angular_velocity] unable to find or subscribe to selected sensor (1310988)
WARN  [timesync] time jump detected. Resetting time synchroniser.
WARN  [uxrce_dds_client] time sync no longer converged
WARN  [health_and_arming_checks] Preflight Fail: No valid data from Accel 0
WARN  [health_and_arming_checks] Preflight Fail: No valid data from Gyro 0
ERROR [vehicle_angular_velocity] unable to find or subscribe to selected sensor (1310988)
ERROR [vehicle_angular_velocity] unable to find or subscribe to selected sensor (1310988)
ERROR [vehicle_angular_velocity] unable to find or subscribe to selected sensor (1310988)

---------------------------------------------

programı çalıştırdığım terminal:
[velocity_control-4] [INFO] [1716325933.614290136] [px4_offboard.velocity]: Arm command send
[velocity_control-4] [INFO] [1716325933.714092174] [px4_offboard.velocity]: Arm command send
[velocity_control-4] [INFO] [1716325933.770992945] [px4_offboard.velocity]: NAV_STATUS: 4
[velocity_control-4] [INFO] [1716325933.815365103] [px4_offboard.velocity]: Loiter, Offboard
[velocity_control-4] [INFO] [1716325933.816326520] [px4_offboard.velocity]: Arm command send
[velocity_control-4] [INFO] [1716325933.816889835] [px4_offboard.velocity]: OFFBOARD
[velocity_control-4] [INFO] [1716325933.939451582] [px4_offboard.velocity]: NAV_STATUS: 14
[velocity_control-4] [INFO] [1716325940.293489039] [px4_offboard.velocity]: FlightCheck: False
[velocity_control-4] [INFO] [1716325940.316408605] [px4_offboard.velocity]: Offboard, Flight Check Failed
[velocity_control-4] [INFO] [1716325940.318377994] [px4_offboard.velocity]: IDLE
[velocity_contro2-5] [INFO] [1716325940.361758135] [px4_offboard.velocity2]: FlightCheck: False
[velocity_control-4] [INFO] [1716325949.977982131] [px4_offboard.velocity]: FlightCheck: True

---------------------------------------------

Görsel bildirimlerim :
	- Rviz 2 tane açılıyor ve sapıtıyor.
	- Gazebo 2 tane açılıyor ama normal
	- 1. drone a sinyal gönderene kadar sıkıntı yok ve "space" basıp çalıştırdığında 
	pervaneler dönüyor ama çok kısa süresonra duruyor ve hatalar başlıyor.
	
---------------------------------------------

[INFO] [launch]: Default logging verbosity is set to INFO
[WARNING] [rviz2-8]: there are now at least 2 nodes with the name /rviz2 created within this launch context
---------
[velocity_control-4]     wait_set.wait(timeout_nsec)
[velocity_control-4] KeyboardInterrupt
[ERROR] [velocity_contro2-5]: process has died [pid 130658, exit code -2, cmd '/home/rahmi/ros2_px4_offboard_example_ws/install/px4_offboard/lib/px4_offboard/velocity_contro2 --ros-args -r __node:=velocity2 -r __ns:=/px4_offboard'].
[ERROR] [velocity_control-4]: process has died [pid 130656, exit code -2, cmd '/home/rahmi/ros2_px4_offboard_example_ws/install/px4_offboard/lib/px4_offboard/velocity_control --ros-args -r __node:=velocity -r __ns:=/px4_offboard'].
[INFO] [rviz2-7]: process has finished cleanly [pid 130662]
[INFO] [rviz2-8]: process has finished cleanly [pid 130664]
[ERROR] [visualizer2-6]: process has died [pid 130660, exit code -2, cmd '/home/rahmi/ros2_px4_offboard_example_ws/install/px4_offboard/lib/px4_offboard/visualizer2 --ros-args -r __node:=visualizer2 -r __ns:=/px4_offboard'].
[ERROR] [visualizer-1]: process has died [pid 130650, exit code -2, cmd '/home/rahmi/ros2_px4_offboard_example_ws/install/px4_offboard/lib/px4_offboard/visualizer --ros-args -r __node:=visualizer -r __ns:=/px4_offboard'].
