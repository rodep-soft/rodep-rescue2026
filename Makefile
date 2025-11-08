.PHONY: help build up down exec status logs clean prune restart rebuild shell-flutter

# Default target - show help
help:
	@echo "Available targets:"
	@echo "  build         - Build Docker images"
	@echo "  up            - Start containers in background"
	@echo "  down          - Stop and remove containers"
	@echo "  restart       - Restart containers"
	@echo "  rebuild       - Rebuild and restart containers"
	@echo "  exec          - Execute bash in ros2_container"
	@echo "  shell-flutter - Execute bash in flutter container"
	@echo "  logs          - Show container logs (use 'make logs SERVICE=ros2_container' for specific)"
	@echo "  logs-f        - Follow container logs"
	@echo "  status        - Check Docker daemon status"
	@echo "  ps            - Show running containers"
	@echo "  clean         - Stop containers and remove volumes"
	@echo "  prune         - Remove all unused Docker resources"
	@echo "  git-status    - Show detailed git status"
	@echo "  git-graph     - Show git log graph"
	@echo "  format-cpp    - Format ROS2 C++ files with clang-format"
	@echo "  check-format  - Check C++ formatting without modifying files"

# === Docker Build & Run ===
build:
	docker compose build

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

# === Docker Shell Access ===
exec:
	docker compose exec ros2_container bash

shell-flutter:
	docker compose exec flutter bash

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
