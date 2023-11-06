# F1-NextGen Mid Level Controller

Since the processing power of the Raspberry Pi Zero module is insufficient for a "quick" compilation, we made tools to
help you compile ROS2 on your PC for the Raspberry Pi Zero.

## Prerequisites

Please make sure everything is ready for you to proceed:

- Raspberry Pi OS ready on your Raspberry
  Pi ([tutorial](https://www.raspberrypi.org/documentation/installation/installing-images/)).
- SSH access to the Raspberry Pi (`ssh pi@[raspberry_pi_ip]`).
- Docker on your PC ([tutorial](https://docs.docker.com/get-docker/)).

## ROS2 Cross-Compilation on Your PC

This repository contains a Dockerfile with a cross-compiled ROS2 Foxy Workspace for the Raspberry Pi Zero.
To build the image, execute

```bash
make build
```

Now, you can use the script we provide to run the container:

```bash
./run.sh
```

It also mounts the `src` folder in the docker container.
The entrypoint for the container is set to the mounted workspace.

Now, to compile your code, you can use the custom command:

```bash
cross-colcon-build
```

This builds all packages in the mounted workspace with the right parameters and toolchain for the Raspberry Pi.

## Using the Cross-Compiled Code on your Raspberry Pi

To transfer the cross-compiled code to your Raspberry Pi, you first have to copy over the ROS2 workspace.
This only has to be done once, since it does not change afterwards.
Transfer the ROS2 workspace to your Pi with the custom command we provide:

```bash
scp-ros2_ws <USER> <IP>
```

After the workspace is transfered, you can do the same for your vehicle workspace. 
This has to be done every time you change something on the code:

```bash
scp-vehicle_ws <USER> <IP>
```