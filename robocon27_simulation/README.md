# robocon27_simulation

4-wheel mecanum robot (+ two "due" climbing units) for Gazebo (gz-sim) and RViz.

## Build
    cd ~/ros2_ws && colcon build --packages-select robocon27_simulation && source install/setup.bash

## Run
    ros2 launch robocon27_simulation simulation.launch.py            # Gazebo + bridge + TF
    ros2 launch robocon27_simulation simulation.launch.py rviz:=true # ... and RViz
    ros2 run    robocon27_simulation wheel_teleop                    # keyboard (see keys on start)
    ros2 launch robocon27_simulation display.launch.py               # RViz + TF tree only, no Gazebo

Any Twist source works on /cmd_vel (linear.x forward, linear.y left, angular.z CCW).
Due units: std_msgs/Float64 (rad/s) on /due_wheel/front_due_joint/cmd_vel and /due_wheel/back_due_joint/cmd_vel.

## Frames (REP-103, all axis-aligned with base_link)
base_link = geometric centre of the base mesh. Wheel / due frames sit on their own axle; joints spin about +Y,
so a positive velocity drives the robot forward (+X).
Links: front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel, front_due, back_due.

## Re-exporting from SolidWorks
Put the raw export in cad_export/solidworks_export.urdf, the STLs in meshes/, edit the MASS table at the top of
tools/generate_urdf.py (placeholders!), then:  python3 tools/generate_urdf.py   (writes urdf/due_wheel.urdf)
