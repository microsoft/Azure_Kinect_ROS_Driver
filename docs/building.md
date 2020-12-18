# Building the Azure Kinect ROS Driver

## Pre-requisites

Before trying to build the Azure Kinect ROS Driver, you will need to install two required dependencies:

- ROS2 Foxy (Ubuntu 20.04 or Windows 10)
- Azure Kinect Sensor SDK

### ROS

Follow the [installation instructions](https://index.ros.org/doc/ros2/Installation/Foxy/) for your operating system of choice: [Ubuntu](https://index.ros.org/doc/ros2/Installation/Foxy/) (be sure to install [colcon](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#install-colcon)), or [Windows](https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html) to install ROS2. Verify that ROS2 is working and that you can build sample projects in your workspace before trying to build the Azure Kinect ROS Driver.

### Azure Kinect Sensor SDK

If you are using Windows 10, follow the [installation instructions](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#Installation) in the Azure Kinect Sensor SDK repo to install the sensor SDK for your platform. 
If you are using Ubuntu 20.04 the provided installation instructions will have to be modified as the binaries are not installable through the repository yet, [follow these steps](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1263#issuecomment-710698591) for a work around. Be sure to [setup udev rules](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#linux-device-setup) to use the Azure Kinect SDK without being 'root'. 

The Azure Kinect ROS Driver includes CMake files which will try to locate the Azure Kinect Sensor SDK. Installing the SDK in a non-default location will result in compile failures when CMake is unable to locate the SDK.

The Azure Kinect ROS Driver requires version of v1.1.0 of the Azure Kinect Sensor SDK to compile.

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

Once the Azure Kinect Sensor SDK has been installed, the ROS node can be built using `colcon build`. Please note that you may need to run `colcon build --force-cmake-configure` to update the SDK binaries which are copied into the ROS output folders.

#### Windows 10 platform:
Open a terminal and navigate to your workspace:
```
c:\opt\ros\foxy\x64\setup.bat
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b foxy-devel
pip3 install xacro
cd Azure_Kinect_ROS_Driver
colcon build 
install\setup.bat
```
#### Ubuntu 20.04 platform:
Open a terminal and navigate to your workspace:
```
source /opt/ros/foxy/setup.bash
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b foxy-devel
pip3 install xacro
sudo apt install ros-foxy-joint-state-publisher
cd Azure_Kinect_ROS_Driver
colcon build 
source install/setup.bash
```

## Troubleshooting 
- If you are having trouble verfying Azure Kinect image streams using the k4aviewer, [try updating the drivers for your graphics card.](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/918). 