# FlowCV RealSense Camera Plugin

Adds RealSense Camera source input to [FlowCV](https://github.com/FlowCV-org/FlowCV)

---

### Build Instructions

Prerequisites:

* Build [libRealSense](https://github.com/IntelRealSense/librealsense) from source or install the required system runtime and development packages.
* Clone [FlowCV](https://github.com/FlowCV-org/FlowCV) repo.
* See System specific build requirements for FlowCV [Building From Source](http://docs.flowcv.org/building_source/build_from_source.html)


Build Steps:
1. Clone this repo.
2. cd to the repo directory
3. Run the following commands:

If using libRealSense from source build:
```shell
mkdir Build
cd Build
cmake .. -DREALSENSE_DIR=/path/to/librealsense -DFlowCV_DIR=/path/to/FlowCV/FlowCV_SDK
make
```

If using libRealSense from install package you don't have to specify the REALSENSE_DIR on linux.

