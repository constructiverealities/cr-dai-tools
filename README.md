[![Running pipeline demo](https://img.youtube.com/vi/DeX8ALlisKY/0.jpg)](https://www.youtube.com/watch?v=DeX8ALlisKY)

# Constructive Realities DepthAI toolkit

This library is a set of tools to help create DepthAI pipelines and run depthai devices in a standard way.

Currently it supports two main points of functionality:

- Introspective creation of a DepthAI pipeline: Enumerates imagers and calibration info on device to decide which nodes to add with defaults for camera properties
    - Metadata YML file to further specify resolution or output settings
- If ROS is available, a ROS node which takes a pipeline (built by the built in generator or not) and exposes all its outputs as ROS topics
    - Includes dynamic reconfigure support for stereo, mono and RGB cameras as well as for device wide settings. 

# Platform support

Currently, due to USB / docker constraints the releases only support linux. ROS2 and thus windows / macosx support is coming
soon.

# Quickstart

Check out this repo, and then run:

```bash
DOCKER_BUILDKIT=1 docker-compose up demo
```

If you don't have docker / docker-compsoe installed, please refer to https://docs.docker.com/get-docker/