# Kinect 

This application includes the software necessary to connect an unmodified, off-the-shelf, Kinect device to a regular computer, and use it as a 3D camera for a variety of 3D graphics and virtual reality applications. 

The software is composed of several classes wrapping aspects of the underlying libusb library into an exception-safe C++ framework, classes encapsulating control of the Kinect's tilt motor and color and depth cameras, and a class encapsulating the operations necessary to reproject a combined depth and color video stream into 3D space. It also contains several utility applications, including a simple calibration utility.


This software is based on the reverse engineering work of Hector Martin Cantero (marcan42 on twitter and YouTube). The Kinect package doesn't use any of his code, but the "magic incantations" that need to be sent to the Kinect to enable the cameras and start streaming. Without owning an XBox, those incantations were essential to learn its USB protocol. Thanks Hector!

The Kinect driver code and the 3D reconstruction code are entirely written from scratch in C++, using the [Vrui VR toolkit](https://github.com/vrui-vr) for 3D rendering management and interaction.

The Kinect is an accessory for Microsoft's Xbox game console, containing an array of microphones, an active-sensing depth camera using structured light, and a color camera. The Kinect is intended to be used as a controller-free game controller, tracking the body or bodies of one or more players in its field of view.

This software converts the Kinect into a 3D camera by combining the depth and color image streams received from the device, and projecting them back out into 3D space in such a way that real 3D objects inside the cameras' field of view are recreated virtually, at their proper sizes (see the below video).


<iframe width="560" height="315" src="https://www.youtube.com/embed/VJ9Ih-49O3o?si=rlILsY4rUtTTDAbG" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

**Documentation:** [vrui-vr.github.io/kinect](https://vrui-vr.github.io/kinect)

**Source code:** [github.com/vrui-vr/kinect](https://github.com/vrui-vr/kinect)