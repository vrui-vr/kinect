# Getting started with the Kinect package

!!! info "Heads up!"
    VRUI must be installed before the Kinect package can be installed.

1. Install VRUI [(go to instructions $\rightarrow$)](https://vrui-vr.github.io/docs/vrui/installation/)
2. Install Kinect package!!!

## Prerequisites

This software requires the Vrui VR toolkit, version 13.0 build 001 or newer. It also requires `libusb` version 1.0, and that Vrui was configured with support for `libusb` prior to its installation. To properly work with multiple Kinect-for-Xbox version 1473 and/or Kinect-for-Windows devices, the `libusb` library needs to provide USB bus topology query functions such as those provided by the `libusbx` fork. This software can optionally create Kinect 3D video streaming plug-ins for Vrui's collaboration infrastructure (version 2.9 or newer). The presence of Vrui's collaboration infrastructure will be detected automatically during the build process.