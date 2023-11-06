all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build     - build image"
	@echo "   4. make clean     - remove image"
	@echo ""

build:
	@docker build \
		--file Dockerfile \
		--tag f1nextgen/rpi-cross-compile \
		.
clean:
	@docker rmi f1nextgen/rpi-cross-compile