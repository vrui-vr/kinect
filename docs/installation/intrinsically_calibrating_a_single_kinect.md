# Intrinsically calibrating a single Kinect

Before a Kinect can be used as a 3D camera, its two cameras need to be calibrated. Calibration will calculate a 4x4 projection matrix to map the depth camera's image stream into 3D space as a 3D point cloud or triangulated surface, and a 4x4 projection matrix to map the color camera's image stream onto that point cloud or surface. After proper calibration, the virtual 3D objects captured by a Kinect will precisely match the original real objects in shape and size.

The quickest way to get intrinsic calibration data for a Kinect is to download the factory calibration data that is stored in each Kinect's firmware. The Kinect package contains a utility for this purpose. In a terminal, run:

```sh
sudo KinectUtil getCalib <Kinect index>
```

where `<Kinect index>` is the index of the Kinect whose data you want to download. If you have only one Kinect connected to your computer, its index is 0. KinectUtil will download the data and copy it into a file that will later be found automatically by all Kinect applications. The quality of factory calibration is generally not as good as what can be achieved by custom calibration, see below, especially for Xbox 360 Kinects.

Custom intrinsic calibration has two sub-procedures. The first, optional, one is to calculate per-pixel depth correction equations. Due to radial lens distortion in the IR pattern projector and the IR camera, a completely flat surface seen by the Kinect will not be reconstructed as a flat surface, but as a bowl shape. Depth correction will calculate a linear equation for each depth pixel that can rectify distance values. The second sub-procedure is to calculate 4x4 homogeneous depth unprojection and color projection matrices. The results of sub-procedure two depend on sub-procedure one, meaning that the second has to be performed every time after the first one is performed.