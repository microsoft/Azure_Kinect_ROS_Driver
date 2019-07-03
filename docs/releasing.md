# Releasing Azure Kinect ROS Driver

Releases of the Azure Kinect ROS Driver are a snapshot of a known-good version of the source code for a particular distribution of ROS. The current release process is:

1. Pick the [ROS Distribution](http://wiki.ros.org/Distributions) you wish to publish for (currently, we only publish for ROS Melodic Morenia, but a ROS2 port is planned).
1. Checkout and build the code for that ROS Distribution. Code for a particular distribution lives under a branch with the same name as the distribution (for example, the `melodic` branch for ROS Melodic Morenia).
1. Test the ROS node on a live ROS system, using Azure Kinect DK hardware.
   - You must test that you can compile on Linux and Windows.
   - You must test that you can compile by installing the Azure Kinect Sensor SDK to the system path. On Windows, this means installing the Azure Kinect Sensor SDK with the `.msi` installer. On Linux, this means installing via the package manager.
   - You must test that you can compile by extracting the Azure Kinect Sensor SDK and depth engine to the `./ext/sdk` folder on both Windows and Linux.
   - You must test that the node is producing all required topics, and that the data on all topics appears correct. For a complete list of topics, see the [usage guide](usage.md)
   - You must verify the node produces a colorized and non-colorized point cloud when using the `rgb_point_cloud` option
   - You must run the node and nodelet
   - You must verify that [image_proc](http://wiki.ros.org/image_proc) can process the image_raw topics from the depth and color cameras
   - You must verify that [depth_image_proc/point_cloud_xyz](http://wiki.ros.org/depth_image_proc?distro=melodic#depth_image_proc.2BAC8-point_cloud_xyz) can produce a point cloud from the rectified depth image
   - You must verify that [depth_image_proc/point_cloud_xyzrgb](http://wiki.ros.org/depth_image_proc?distro=melodic#depth_image_proc.2BAC8-point_cloud_xyzrgb) can produce a colorized point cloud from the rectified depth and color images

1. Once satisfied that the node is working correctly, create a new GitHub release for the commit you tested. 
   - The Release should be named `<ROS Distro Short Name> <Current Date>`. For example, the first version of the ROS Melodic Morenia release was "Melodic 6/21/2019"
   - The tag of the release should be `<ROS Distro Short Name>_<Current Date>`. For example, the tag of the first version of the ROS Melodic Morenia release was "melodic_6-21-2019"