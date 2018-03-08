# Setting up Phantom Omni (Instructions are compiled from various sources and own experiences)

source 1: http://fsuarez6.github.io/projects/geomagic-touch-in-ros/

source 2: https://github.com/MurpheyLab/trep_omni

The following are instructions on installing the Phantom Omni on Ubuntu. Tested on Ubuntu 14.04 with ROS-Indigo

***

## Phantom Omni Driver and OpenHaptics Toolkit Installation

These instructions were tested on Ubuntu 14.04 using a firewire card with the VT6315 controller.

### Required Packages:
* OpenHapticsAE\_Linux\_v3\_0
* Linux\_JUJU\_PPD

### Installation Steps:

Goto the OpenHaptics\_Linux\_v3\_0/PHANTOM Device Drivers/64-bit folder. Run:

    sudo dpkg -i phantomdevicedrivers_4.3-3_amd64.deb

Goto Linux\_JUJU\_PDD_64-bit 

Copy  libPHANToMIO.so.4.3 in to /usr/lib.
Create symbolic links: 

    sudo ln -s  /usr/lib/libPHANToMIO.so.4.3  /usr/lib/libPHANToMIO.so 
    sudo ln -s  /usr/lib/libPHANToMIO.so.4.3  /usr/lib/libPHANToMIO.so.4

Copy the PhantomTest program from the JUJU PPD:

    sudo cp PHANToMConfiguration /usr/sbin/

Install the following dependencies.

    sudo apt-get install freeglut3-dev libmotif4 libglw1-mesa x11proto-dri2-dev libdrm2 libncurses5-dev

Create a driver symlink. In the /usr/lib/x86_64-linux-gnu directory:

    sudo ln -s /usr/lib/x86_64-linux-gnu/libraw1394.so.11 /usr/lib/x86_64-linux-gnu/libraw1394.so.8

Create a udev rule for the Phantom Omni

    sudo touch /lib/udev/rules.d/50-phantom-firewire.rules

Add the following line to 50-phantom-firewire.rules

    SUBSYSTEM=="firewire", ATTR{vendor}=="0x000b99", MODE="0660", GROUP="plugdev"

If the Phantom Omni is currently plugged in, restart driver with

    sudo modprobe -r firewire_ohci
    sudo modprobe firewire_ohci

At this point, test the Phantom Omni by running the configuration app

    PHANToMConfiguration

The Phantom Model field must be set to Omni, then click OK.  Next, run the Phantom Test app

    PHANToMTest

Go through the calibration test and make sure that the values are changing.  If everything is working, continue to install the OpenHaptics Toolkit by going to the OpenHaptics\_Linux\_v3\_0/OpenHaptics-AE 3.0/64-bit folder and running

    sudo dpkg -i openhaptics-ae_3.0-2_amd64.deb

After installation, remove duplicate shared libraries from /usr/lib64

    sudo rm /usr/lib64/libPHANToMIO.so /usr/lib64/libPHANToMIO.so.4 /usr/lib64/libPHANToMIO.so.4.3

Create a configuration file in /etc/ld.so.conf.d

    sudo touch /etc/ld.so.conf.d/openhaptics.conf

Add the following line to openhaptics.conf

    /usr/lib64

Rerun ldconfig to find the new shared libraries

    sudo ldconfig

The OpenHaptics Toolkit and Phantom Omni driver should now be installed!  The ROS phantom\_omni and omni\_description packages can be used to interface ROS with the Omni hardware.

Examples are located in: OpenHapticsAE_Linux_v3_0/OpenHaptics-AE 3.0/64-bit/openhaptics-AE-3.0/usr/share/3DTouch/examples

#ROS - Package installation

Goto to your workspace folder and then inside src

    git clone https://github.com/rezeck/phantom_omni.git

Check for any missing dependencies using rosdep:

    source /opt/ros/ros-indigo/setup.bash

    rosdep update

    rosdep check --from-paths . --ignore-src --rosdistro ros-indigo

After installing the missing dependencies compile your ROS workspace. e.g.

    cd ~/catkin_ws && catkin_make

    source ~/catkin_ws/devel/setup.bash

Allow Access to the FireWire Interface

This package provides a dummy driver that allows you to access the FireWire interface. For new installations or every time you upgrade your linux kernel, please run the following command:

    rosrun omni_common initialize_device.sh -c

roslaunch omni_common omni.launch


#Possible errors

    [ERROR] [1501178583.149302484]: Failed to initialize haptic device

Step 1: in the omni.cpp define a macro name

    #define DEVICE_NAME "Default PHANToM"

Step 2: line 302 in omni.cpp change to

    hHD = hdInitDevice(DEVICE_NAME);

Step 3: run the roslaunch as root!
    
    i.e. sudo su

    you may have to source the bash file again

    source ~/catkin_ws/devel/setup.bash

    roslaunch omni_common omni.launch


My reasoning: HD_DEFAULT_DEVICE is a HDString and it is set as NULL by default, somehow this is causing the problem since the API is trying to access a NULL value. So we need to set the actual name of the device. This should be a bug in the API.



