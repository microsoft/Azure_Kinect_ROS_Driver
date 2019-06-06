# Azure Kinect ROS Driver

This project is a ROS node which publishes sensor data from the Azure Kinect Developer Kit. Developers working with ROS can use this node to connect an Azure Kinect Developer Kit to an existing ROS installation.

## Features

This ROS node outputs a variety of sensor data, including:

- A PointCloud2, optionally colored using the color camera
- Raw color and depth Images, including CameraInfo messages containing calibration information
- Rectified depth Images in the color camera resolution
- Rectified color Images in the depth camera resolution
- The IMU sensor stream

The camera is fully configurable using a variety of options which can be specified in ROS launch files or on the command line.

However, this node does ***not*** expose all the sensor data from the Azure Kinect. It does not provide access to:

- Skeleton tracking data
- Microphone array data

## Status

This code is provided as a starting point for using the Azure Kinect Developer Kit with ROS. It is being actively maintained, and new features are welcome.

For information on how to contribute, please see our [contributing guide](CONTRIBUTING.md).

## Building

The Azure Kinect ROS Driver uses catkin to build. For instructions on how to build the project please see
[building](docs/building.md).

## Join Our Developer Program

Complete your developer profile [here](https://aka.ms/iwantmr) to get connected with our Mixed Reality Developer Program. You will receive the latest on our developer tools, events, and early access offers.

## Code of Conduct

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.

## Reporting Security Issues
Security issues and bugs should be reported privately, via email, to the
Microsoft Security Response Center (MSRC) at <[secure@microsoft.com](mailto:secure@microsoft.com)>.
You should receive a response within 24 hours. If for some reason you do not, please follow up via
email to ensure we received your original message. Further information, including the
[MSRC PGP](https://technet.microsoft.com/en-us/security/dn606155) key, can be found in the
[Security TechCenter](https://technet.microsoft.com/en-us/security/default).

## License

[MIT License](LICENSE)