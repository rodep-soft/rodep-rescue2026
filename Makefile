.PHONY: help setup setup-microros build build-ros2 build-flutter build-microros up down exec status logs clean prune restart rebuild rebuild-ros2 rebuild-flutter rebuild-microros shell-flutter shell-microros

# Default target - show help
help:
	@echo "Available targets:"
	@echo ""
	@echo "=== Initial Setup ==="
	@echo "  setup            - First-time setup (build all + extract micro-ROS firmware)"
	@echo "  setup-microros   - Extract micro-ROS firmware from container to host"
	@echo ""
	@echo "=== Docker Build ==="
	@echo "  build            - Build all Docker images"
	@echo "  build-ros2       - Build only ros2_container"
	@echo "  build-flutter    - Build only flutter"
	@echo "  build-microros   - Build only microros_agent"
	@echo ""
	@echo "=== Docker Run ==="
	@echo "  up               - Start containers in background"
	@echo "  down             - Stop and remove containers"
	@echo "  restart          - Restart containers"
	@echo "  rebuild          - Rebuild and restart all containers"
	@echo "  rebuild-ros2     - Rebuild and restart ros2_container"
	@echo "  rebuild-flutter  - Rebuild and restart flutter"
	@echo "  rebuild-microros - Rebuild and restart microros_agent"
	@echo ""
	@echo "=== Shell Access ==="
	@echo "  exec             - Execute bash in ros2_container"
	@echo "  shell-flutter    - Execute bash in flutter container"
	@echo "  shell-microros   - Execute bash in microros_agent container"
	@echo ""
	@echo "=== Logs & Status ==="
	@echo "  logs             - Show container logs (use 'make logs SERVICE=ros2_container' for specific)"
	@echo "  logs-f        - Follow container logs"
	@echo "  status        - Check Docker daemon status"
	@echo "  ps            - Show running containers"
	@echo "  clean         - Stop containers and remove volumes"
	@echo "  prune         - Remove all unused Docker resources"
	@echo "  git-status    - Show detailed git status"
	@echo "  git-graph     - Show git log graph"
	@echo "  format-cpp    - Format ROS2 C++ files with clang-format"
	@echo "  check-format  - Check C++ formatting without modifying files"
	@echo "  tidy          - Run clang-tidy static analysis"
	@echo "  tidy-fix      - Run clang-tidy with auto-fix"

# === Initial Setup ===
setup: check-submodules build setup-microros
	@echo "✅ Setup complete! You can now:"
	@echo "  - Edit micro-ROS setup: microros_ws/src/micro_ros_setup/"
	@echo "  - Edit FreeRTOS apps: microros_ws/src/micro_ros_setup/freertos_apps/apps/"
	@echo "  - Start containers: make up"
	@echo "  - Access micro-ROS container: make shell-microros"

check-submodules:
	@echo "Checking submodules..."
	@if [ ! -f microros_ws/src/micro_ros_setup/.git ] && [ ! -d microros_ws/src/micro_ros_setup/.git ]; then \
		echo "⚠️  Submodule not initialized. Running: git submodule update --init --recursive"; \
		git submodule update --init --recursive; \
	else \
		echo "✅ Submodules OK"; \
	fi

setup-microros:
	@echo "Extracting micro-ROS firmware from container to host..."
	@mkdir -p microros_ws/firmware
	@docker run --rm -v $(PWD)/microros_ws/firmware:/host_firmware rodep-rescue2026-microros_agent bash -c "cp -r /root/microros_ws/firmware/* /host_firmware/" 2>/dev/null || true
	@echo "✅ Firmware extracted to microros_ws/firmware/"

# === Docker Build & Run ===
build:
	docker compose build

# Build specific services
build-ros2:
	docker compose build ros2_container

build-flutter:
	docker compose build flutter

build-microros:
	docker compose build microros_agent

up:
	docker compose up -d

down:
	docker compose down

restart:
	docker compose restart

rebuild:
	docker compose down
	docker compose build --no-cache
	docker compose up -d

# Rebuild specific services
rebuild-ros2:
	docker compose stop ros2_container
	docker compose build --no-cache ros2_container
	docker compose up -d ros2_container

rebuild-flutter:
	docker compose stop flutter
	docker compose build --no-cache flutter
	docker compose up -d flutter

rebuild-microros:
	docker compose stop microros_agent
	docker compose build --no-cache microros_agent
	docker compose up -d microros_agent

# === Docker Shell Access ===
exec:
	docker compose exec ros2_container bash

shell-flutter:
	docker compose exec flutter bash

shell-microros:
	docker compose exec microros_agent bash

# === Docker Logs ===
logs:
	@if [ -z "$(SERVICE)" ]; then \
		docker compose logs --tail=100; \
	else \
		docker compose logs --tail=100 $(SERVICE); \
	fi

logs-f:
	@if [ -z "$(SERVICE)" ]; then \
		docker compose logs -f; \
	else \
		docker compose logs -f $(SERVICE); \
	fi

# === Docker Status ===
status:
	systemctl status docker

ps:
	docker compose ps

# === Docker Cleanup ===
clean:
	docker compose down -v
	@echo "Containers stopped and volumes removed"

prune:
	@echo "WARNING: This will remove all unused Docker resources!"
	@read -p "Continue? [y/N] " confirm && [ "$$confirm" = "y" ] || exit 1
	docker system prune -af --volumes
	@echo "Docker cleanup complete"

# === Git Shortcuts ===
git-status:
	@git status -sb
	@echo "\n--- Untracked files ---"
	@git ls-files --others --exclude-standard

git-graph:
	@git log --graph --oneline --decorate --all -20

# === Code Formatting ===
format-cpp:
	@echo "Formatting ROS2 C++ files..."
	@find ros_ws/src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
		! -path '*/eProsima/*' ! -path '*/ros2/*' ! -path '*/micro_ros_setup/*' \
		-exec clang-format -i {} +
	@echo "Done!"

check-format:
	@echo "Checking C++ formatting..."
	@find ros_ws/src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
		! -path '*/eProsima/*' ! -path '*/ros2/*' ! -path '*/micro_ros_setup/*' \
		-exec clang-format --dry-run --Werror {} +

# === Code Analysis ===
tidy:
	@echo "Running clang-tidy on ROS2 C++ files..."
	@if [ ! -f ros_ws/build/compile_commands.json ]; then \
		echo "Error: compile_commands.json not found!"; \
		echo "Please build your ROS2 workspace first:"; \
		echo "  cd ros_ws && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"; \
		exit 1; \
	fi
	@find ros_ws/src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
		! -path '*/eProsima/*' ! -path '*/ros2/*' ! -path '*/micro_ros_setup/*' \
		! -path '*/build/*' ! -path '*/install/*' ! -path '*/log/*' \
		-exec clang-tidy -p ros_ws/build --config-file=ros_ws/src/.clang-tidy {} + 2>&1 | grep -v "^[0-9]* warnings generated"

tidy-fix:
	@echo "Running clang-tidy with auto-fix on ROS2 C++ files..."
	@if [ ! -f ros_ws/build/compile_commands.json ]; then \
		echo "Error: compile_commands.json not found!"; \
		echo "Please build your ROS2 workspace first:"; \
		echo "  cd ros_ws && colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"; \
		exit 1; \
	fi
	@find ros_ws/src -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.h' \) \
		! -path '*/eProsima/*' ! -path '*/ros2/*' ! -path '*/micro_ros_setup/*' \
		! -path '*/build/*' ! -path '*/install/*' ! -path '*/log/*' \
		-exec clang-tidy -p ros_ws/build --config-file=ros_ws/src/.clang-tidy --fix {} + 2>&1 | grep -v "^[0-9]* warnings generated"
