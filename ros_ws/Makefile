
build:
	colcon build --symlink-install

# Build with debug symbols for GDB debugging
build-debug:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build with debug symbols and address sanitizer
build-debug-asan:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=address -g"

# Build release with optimizations
build-release:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build with RelWithDebInfo (optimized but with debug symbols)
build-relwithdebinfo:
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

src:
	bass source ./install/setup.bash

clean:
	rm -rf build install log

find:
	ros2 pkg executables | fzf

# Clean CMake cache and rebuild with debug symbols
clean-debug:
	colcon build --cmake-clean-cache --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Debug a specific node with GDB (usage: make debug-node PKG=<package> NODE=<node>)
debug-node:
	@if [ -z "$(PKG)" ] || [ -z "$(NODE)" ]; then \
		echo "Usage: make debug-node PKG=<package_name> NODE=<node_name>"; \
		echo "Example: make debug-node PKG=sekirei_moveit_config NODE=moveit_bridge.py"; \
		exit 1; \
	fi
	ros2 run --prefix 'gdb -ex run --args' $(PKG) $(NODE)

# Debug a node with xterm GUI (usage: make debug-node-gui PKG=<package> NODE=<node>)
debug-node-gui:
	@if [ -z "$(PKG)" ] || [ -z "$(NODE)" ]; then \
		echo "Usage: make debug-node-gui PKG=<package_name> NODE=<node_name>"; \
		echo "Example: make debug-node-gui PKG=sekirei_moveit_config NODE=moveit_bridge.py"; \
		exit 1; \
	fi
	ros2 run --prefix 'xterm -e gdb -ex run --args' $(PKG) $(NODE)

# Debug a test executable (usage: make debug-test TEST=<path_to_test>)
debug-test:
	@if [ -z "$(TEST)" ]; then \
		echo "Usage: make debug-test TEST=<path_to_test_executable>"; \
		echo "Example: make debug-test TEST=./build/my_pkg/test/test_my_code"; \
		exit 1; \
	fi
	bash -c "source install/setup.bash && gdb -ex run $(TEST)"

unitless:
	ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 0}"

twist:
	ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 1}"


.PHONY: build build-debug build-debug-asan build-release build-relwithdebinfo src clean clean-debug find debug-node debug-node-gui debug-test
