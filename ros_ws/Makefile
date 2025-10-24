
build:
	colcon build --symlink-install

src:
	bass source ./install/setup.bash

clean:
	rm -rf build install log

find:
	ros2 pkg executables | fzf


	


.PHONY: build source clean
