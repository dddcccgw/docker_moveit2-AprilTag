.PHONY: help docker-build docker-up docker-down docker-logs docker-exec \
        build clean install-deps verify setup ros-run test clean-docker

# Colors for output
BLUE := \033[0;34m
GREEN := \033[0;32m
RED := \033[0;31m
NC := \033[0m # No Color

# Project variables
PROJECT_NAME := apriltag_ros2
DOCKER_IMAGE := apriltag_ros2:humble
DOCKER_CONTAINER := apriltag_humble
ROS_DISTRO := humble
PYTHON_VERSION := 3.10

help:
	@echo "$(BLUE)AprilTag ROS2 Intel D435 - Development Commands$(NC)"
	@echo ""
	@echo "$(GREEN)Docker Operations:$(NC)"
	@echo "  make docker-build      - Build Docker image"
	@echo "  make docker-up         - Start Docker container"
	@echo "  make docker-down       - Stop Docker container"
	@echo "  make docker-restart    - Restart Docker container"
	@echo "  make docker-logs       - View Docker container logs"
	@echo "  make docker-exec       - Execute bash in Docker container"
	@echo "  make docker-clean      - Remove Docker image and container"
	@echo ""
	@echo "$(GREEN)Build Operations:$(NC)"
	@echo "  make build             - Build ROS 2 packages (native)"
	@echo "  make rebuild           - Clean build ROS 2 packages"
	@echo "  make install-deps      - Install Python dependencies"
	@echo "  make clean             - Clean build artifacts"
	@echo ""
	@echo "$(GREEN)Verification & Testing:$(NC)"
	@echo "  make verify            - Verify ROS 2 setup"
	@echo "  make setup             - Full setup (install deps + build)"
	@echo "  make test              - Run tests"
	@echo ""
	@echo "$(GREEN)ROS 2 Operations:$(NC)"
	@echo "  make ros-list          - List all ROS 2 packages"
	@echo "  make ros-apriltag      - List AprilTag packages"
	@echo "  make ros-topics        - List ROS 2 topics"
	@echo "  make ros-nodes         - List ROS 2 nodes"
	@echo ""
	@echo "$(GREEN)Utilities:$(NC)"
	@echo "  make help              - Show this help message"
	@echo ""

# ============================================================================
# Docker Operations
# ============================================================================

docker-build:
	@echo "$(BLUE)Building Docker image: $(DOCKER_IMAGE)$(NC)"
	docker-compose build --progress=plain

docker-up:
	@echo "$(BLUE)Starting Docker container: $(DOCKER_CONTAINER)$(NC)"
	docker-compose up -d
	@sleep 2
	@echo "$(GREEN)Container started successfully!$(NC)"
	@echo "Enter with: docker exec -it $(DOCKER_CONTAINER) bash"

docker-up-attach:
	@echo "$(BLUE)Starting Docker container in attached mode$(NC)"
	docker-compose up

docker-down:
	@echo "$(BLUE)Stopping Docker container$(NC)"
	docker-compose down

docker-restart:
	@echo "$(BLUE)Restarting Docker container$(NC)"
	docker-compose restart

docker-logs:
	@echo "$(BLUE)Docker container logs:$(NC)"
	docker-compose logs -f

docker-exec:
	@echo "$(BLUE)Entering Docker container shell$(NC)"
	docker exec -it $(DOCKER_CONTAINER) bash

docker-clean:
	@echo "$(RED)Removing Docker image and container$(NC)"
	docker-compose down -v
	docker rmi $(DOCKER_IMAGE) || true

docker-ps:
	@echo "$(BLUE)Running Docker containers:$(NC)"
	docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

# ============================================================================
# Build Operations (Native)
# ============================================================================

install-deps:
	@echo "$(BLUE)Installing Python dependencies$(NC)"
	pip3 install --upgrade pip setuptools wheel
	pip3 install numpy opencv-python pyrealsense2 dt-apriltags scipy

source-ros:
	@echo "$(BLUE)Sourcing ROS 2 Humble$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash

build: source-ros
	@echo "$(BLUE)Building ROS 2 packages$(NC)"
	colcon build --symlink-install
	@echo "$(GREEN)Build complete!$(NC)"
	@echo "Source with: source install/setup.bash"

rebuild: clean build

build-single:
	@echo "$(BLUE)Building apriltag_detector package$(NC)"
	colcon build --packages-select apriltag_detector --symlink-install

clean:
	@echo "$(RED)Cleaning build artifacts$(NC)"
	rm -rf build install log .colcon
	@echo "$(GREEN)Clean complete!$(NC)"

# ============================================================================
# Verification & Testing
# ============================================================================

verify:
	@echo "$(BLUE)Verifying ROS 2 setup$(NC)"
	bash verify-ros2-setup.sh

setup: install-deps build verify
	@echo "$(GREEN)Setup complete!$(NC)"

test:
	@echo "$(BLUE)Running tests$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	colcon test --packages-select apriltag_detector

test-verbose:
	@echo "$(BLUE)Running tests with verbose output$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	colcon test --packages-select apriltag_detector --event-handlers console_direct+

# ============================================================================
# ROS 2 Operations
# ============================================================================

ros-source:
	@source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash

ros-list:
	@echo "$(BLUE)All ROS 2 packages:$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 pkg list | head -20

ros-apriltag:
	@echo "$(BLUE)AprilTag related packages:$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 pkg list | grep apriltag

ros-topics:
	@echo "$(BLUE)ROS 2 topics:$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 topic list

ros-nodes:
	@echo "$(BLUE)ROS 2 nodes:$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 node list

ros-run-map:
	@echo "$(BLUE)Running AprilTag Map Builder$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run apriltag_detector apriltag_map

ros-run-validator:
	@echo "$(BLUE)Running Camera Position Validator$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run apriltag_detector camera_validator

ros-run-calibration:
	@echo "$(BLUE)Running Calibration Data Recorder$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	source install/setup.bash && \
	ros2 run apriltag_detector record_calibration

# ============================================================================
# Docker-based Operations
# ============================================================================

docker-build-dev: docker-build docker-up
	@echo "$(GREEN)Development Docker environment ready!$(NC)"

docker-rebuild: docker-clean docker-build docker-up

# Run command in Docker
docker-cmd:
	@docker exec -it $(DOCKER_CONTAINER) bash -c "source /root/ws/install/setup.bash && $(CMD)"

# Build in Docker
docker-build-pkg:
	@echo "$(BLUE)Building in Docker$(NC)"
	docker exec -it $(DOCKER_CONTAINER) bash -c "\
		source /opt/ros/$(ROS_DISTRO)/setup.bash && \
		cd /root/ws && \
		colcon build --symlink-install"

# Run AprilTag in Docker
docker-run-map:
	@echo "$(BLUE)Running AprilTag Map Builder in Docker$(NC)"
	docker exec -it $(DOCKER_CONTAINER) bash -c "\
		source /root/ws/install/setup.bash && \
		ros2 run apriltag_detector apriltag_map"

docker-run-validator:
	@echo "$(BLUE)Running Camera Position Validator in Docker$(NC)"
	docker exec -it $(DOCKER_CONTAINER) bash -c "\
		source /root/ws/install/setup.bash && \
		ros2 run apriltag_detector camera_validator"

# ============================================================================
# Development Workflow
# ============================================================================

# Watch for changes and rebuild
watch:
	@echo "$(BLUE)Watching for changes... (Press Ctrl+C to stop)$(NC)"
	source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	while true; do \
		inotifywait -r -e modify apriltag_detector/ && \
		echo "Changes detected, rebuilding..." && \
		colcon build --packages-select apriltag_detector --symlink-install; \
	done

# Format code
format:
	@echo "$(BLUE)Formatting Python code$(NC)"
	black apriltag_detector/
	isort apriltag_detector/

# Lint code
lint:
	@echo "$(BLUE)Linting Python code$(NC)"
	flake8 apriltag_detector/
	pylint apriltag_detector/

# Development setup
dev-setup: install-deps
	@echo "$(BLUE)Setting up development environment$(NC)"
	pip3 install black isort flake8 pylint pytest pytest-cov
	@echo "$(GREEN)Development environment ready!$(NC)"

# ============================================================================
# Info & Status
# ============================================================================

info:
	@echo "$(BLUE)Project Information:$(NC)"
	@echo "  Project Name: $(PROJECT_NAME)"
	@echo "  Docker Image: $(DOCKER_IMAGE)"
	@echo "  ROS Distribution: $(ROS_DISTRO)"
	@echo "  Python Version: $(PYTHON_VERSION)"
	@echo ""
	@echo "$(BLUE)System Information:$(NC)"
	@echo "  Docker: $$(docker --version)"
	@echo "  Docker Compose: $$(docker-compose --version)"
	@echo "  Python: $$(python3 --version)"
	@echo ""

status:
	@echo "$(BLUE)Status:$(NC)"
	@docker ps --filter "name=$(DOCKER_CONTAINER)" --format "{{.Names}}: {{.Status}}" || echo "Container not running"
	@echo ""
	@echo "$(BLUE)Python Packages:$(NC)"
	@pip3 list | grep -E 'numpy|opencv|pyrealsense|apriltags|scipy'

# ============================================================================
# Cleanup
# ============================================================================

clean-all: clean docker-clean
	@echo "$(RED)Removed all build artifacts and Docker resources$(NC)"

clean-logs:
	@echo "$(RED)Cleaning logs$(NC)"
	rm -rf log/
	@echo "$(GREEN)Logs cleaned$(NC)"

clean-docker-images:
	@echo "$(RED)Removing all AprilTag Docker images$(NC)"
	docker images | grep apriltag | awk '{print $$3}' | xargs docker rmi -f || true

distclean: clean-all clean-logs
	@echo "$(RED)Distribution clean complete$(NC)"

# ============================================================================
# Default target
# ============================================================================

.DEFAULT_GOAL := help
