.PHONY = build

SHELL := /bin/bash

run:
	colcon build
	source install/setup.bash && ros2 run pioneer3DX_controller Pioneer3DX_Controller

debug:
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
	source install/setup.bash && ros2 run --prefix 'gdb -ex start --args' pioneer3DX_controller Pioneer3DX_Controller
