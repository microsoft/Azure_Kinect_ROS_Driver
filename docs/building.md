# Building the Azure Kinect ROS Driver

## Pre-requisites

Before trying to build the Azure Kinect ROS Driver, you will need to install two required dependencies:

- ROS Melodic (Ubuntu or Windows 10)
- Azure Kinect Sensor SDK

## Dependency Set Up

1. Install ROS Melodic
    - Follow the [installation instructions](https://wiki.ros.org/Installation) for your operating system of choice: [Ubuntu](https://wiki.ros.org/Installation/Ubuntu), or [Windows](https://wiki.ros.org/Installation/Windows)
    
2. Install the Azure Kinect Sensor SDK
    - Ubuntu
        1.	Download the package and repository signing key

            ```
            curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
            ```

        1.	Install the key
        
            ```
            sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
            ```

        1.	Add the packages.microsoft.com ubuntu 18.04 repo to your packages list
        
            ```
            sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/    microsoft-ubuntu-bionic-prod/ bionic main" > /etc/apt/sources.list.d/   microsoft-ubuntu-bionic-prod.list'
            ```
        
        1.	Update your package list

            ```
            sudo apt-get update
            ```
        
        1.	Install the k4a development package

            ```
            sudo apt-get install libk4a1.1-dev k4a-tools
            ```

    - Windows
        1. Download and install the Azure Kinect Sensor SDK installer from the [Azure Kinect Sensor SDK project](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)

## Alternate SDK Installation

Instead of installing the Azure Kinect Sensor SDK to the system path (using the `.msi` installer on Windows or the `.deb` installer on Ubuntu) you can extract the SDK the `.\ext\sdk` folder. 

## Compiling

Once the Azure Kinect Sensor SDK has been installed, the ROS node can be built using `catkin_make`. Please note that you may need to run `catkin_make --force-cmake` to update the SDK binaries which are copied into the ROS output folders.

