.PHONY: build

SHELL := /bin/bash

build:
	TERM=dumb colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

run:
	colcon build
	source install/setup.bash && ros2 run pioneer3dx_controller Pioneer3dx_Controller

debug: build
	source install/setup.bash && ros2 run --prefix 'gdb -ex start --args' pioneer3dx_controller Pioneer3dx_Controller

mode_config:
	ros2 service call /p3dx_command_change_mode pioneer3dx_controller/srv/ChangeMode "{mode: 1}"
mode_run:
	ros2 service call /p3dx_command_change_mode pioneer3dx_controller/srv/ChangeMode "{mode: 2}"
mode_invalid:
	ros2 service call /p3dx_command_change_mode pioneer3dx_controller/srv/ChangeMode "{mode: 3}"
	ros2 service call /p3dx_command_change_mode pioneer3dx_controller/srv/ChangeMode "{mode: 0}"
