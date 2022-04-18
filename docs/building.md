# Building the Azure Kinect ROS Driver

## Pre-requisites

Before trying to build the Azure Kinect ROS Driver, you will need to install two required dependencies:

- ROS Melodic (Ubuntu or Windows 10)
- Azure Kinect Sensor SDK

### ROS

Follow the [installation instructions](https://wiki.ros.org/Installation) for your operating system of choice: [Ubuntu](https://wiki.ros.org/Installation/Ubuntu), or [Windows](https://wiki.ros.org/Installation/Windows) to install ROS. Verify that ROS is working and that you can build sample projects in your catkin workspace before trying to build the Azure Kinect ROS Driver.

### Azure Kinect Sensor SDK

Follow the [installation instructions](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#Installation) in the Azure Kinect Sensor SDK repo to install the sensor SDK for your platform.

The Azure Kinect ROS Driver includes CMake files which will try to locate the Azure Kinect Sensor SDK. Installing the SDK in a non-default location will result in compile failures when CMake is unable to locate the SDK.

The Azure Kinect ROS Driver requires version of v1.3.0 of the Azure Kinect Sensor SDK to compile.

#### Alternate SDK Installation

Instead of installing the Azure Kinect Sensor SDK to the system path (using the `.msi` installer on Windows or the `.deb` installer on Ubuntu) you can extract the SDK the `.\ext\sdk` folder. You should have the following folder structure after extracting the SDK into `.\ext\sdk`:

```
.\ext\sdk\
          bin\
          include\
          lib\
```

Please note that the Azure Kinect Sensor SDK zip files do not contain the depth engine shared library. You will need to acquire a version of the depth engine shared library and place it in the .\ext\sdk\bin folder so that it can be consumed by the build system. Failing to download the depth engine when using an extracted SDK will result in the node crashing on launch.

For more information, please consult the [Azure Kinect Sensor SDK usage guide](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md).

## Compiling

Once the Azure Kinect Sensor SDK has been installed, the ROS node can be built using `catkin_make`. Please note that you may need to run `catkin_make --force-cmake` to update the SDK binaries which are copied into the ROS output folders.

