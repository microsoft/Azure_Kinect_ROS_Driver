# Azure Kinect ROS Driver

This project is a ROS node which publishes sensor data from the Azure Kinect Developer Kit. This ROS node outputs a variety of sensor data, including:

- A point cloud, optionally colored using the color camera
- Raw color and depth images, including CameraInfo messages containing calibration information
- Rectified depth images in the color camera resolution, which can be used as input to a variety of SLAM algorithms
- The IMU sensor stream

The camera is fully configurable using a variety of options which can be specified in ROS launch files or on the command line.

# Pre-requisites

1. A working installation of ROS Melodic (Ubuntu or Windows 10)
2. A copy of the Azure Kinect Sensor SDK

# Building

Before you are able to build the Azure Kinect ROS Driver, you will need to setup the Azure Kinect Sensor SDK so that it can be found by catkin. The Azure Kinect ROS Driver includes a cmake file which will attempt to locate the sensor SDK in several locations:

- Windows 10: ```C:\Program Files\Azure Kinect SDK```. This is the default install location for the SDK.
- Ubuntu: Installed to the system path. This is the default install location for the SDK.
- All platforms: inside the ```.\ext\sdk``` folder. 

Once the Azure Kinect Sensor SDK has been installed, the ROS node can be built using ```catkin_make```. Please note that you may need to run ```catkin_make --force-cmake``` to update the SDK binaries which are copied into the ROS output folders.

# Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., label, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.
