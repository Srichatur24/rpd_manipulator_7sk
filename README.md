# RPD Manipulator 7SK

## Dependencies
- [servo control library](https://github.com/Srichatur24/st_sc_servo_control_lib)

## Build the packages
```bash
colcon build --packages-up-to rpd_manipulator_7sk
```

## Start the Manipulator
- Hardware
```bash
ros2 launch rpd_manipulator_7sk_bringup rpd_manipulator_7sk.launch.py
```
- Mock hardware
``` bash
ros2 launch rpd_manipulator_7sk_bringup rpd_manipulator_7sk.launch.py use_mock_hardware:=true
```
- Gazebo
``` bash
ros2 launch rpd_manipulator_7sk_bringup rpd_manipulator_7sk.launch.py use_sim:=true
```

## Launch Moveit
- Hardware / Mock hardware
```bash
ros2 launch rpd_manipulator_7sk_moveit_config rpd_manipulator_7sk_moveit.launch.py
```
- Gazebo
```bash
ros2 launch rpd_manipulator_7sk_moveit_config rpd_manipulator_7sk_moveit.launch.py use_sim:=true
```
