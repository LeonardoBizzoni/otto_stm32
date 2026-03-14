.PHONY = build

SHELL := /bin/bash

run:
	colcon build
	source install/setup.bash && ros2 run pioneer3dx_controller Pioneer3dx_Controller

debug:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
	source install/setup.bash && ros2 run --prefix 'gdb -ex start --args' pioneer3dx_controller Pioneer3dx_Controller
