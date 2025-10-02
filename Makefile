
build:
	docker compose build

up:
	docker compose up -d

down:
	docker compose down

exec:
	docker compose exec ros2_container bash

status:
	systemctl status docker
