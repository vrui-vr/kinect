<!-- todo this is a duplicate of arsandbox/docs/installation/hardware/step1.md -->
Per-pixel Depth Correction
--------------------------

Calculating per-pixel depth correction equations only requires a flat
and ideally vertical surface large enough to fill the Kinect's
field-of-view at larger distances (up to about 2 meters). These are the
calibration procedure's steps:

1. Start RawKinectViewer for the Kinect to be calibrated.

2. Create a "Calibrate Depth Lens" tool by binding it to two (2) buttons
   (keyboard keys or mouse buttons). Here, we will use the keys "1" and
   "2". Pressing the "1" key will capture the current depth image as a
   calibration point, and pressing the "2" key will calculate per-pixel
   depth correction equations after several depth images have been
   captured.

3. Move the Kinect such that it faces the flat surface orthogonally and
   at close distance. The best approach is to position the Kinect to be
   roughly orthogonal to the surface, and push it so close that the
   entire depth image turns black (this happens at a distance of around
   50cm). Then slowly pull the Kinect back until about half the pixels
   have valid depth values. Then carefully rotate the Kinect until the
   valid pixels are distributed evenly across the depth image, and then
   pull back slightly further until most or all pixels have valid depth.

4. Press the "1" key to capture the current depth image. This will
   capture a number of frames, as indicated by the pop-up window. Do not
   move the Kinect while the pop-up window stays on the screen.

5. Move the Kinect further back, ideally until a sharp change in depth
   mapping color. Carefully rotate the Kinect such that pixels of
   different colors are evenly distributed, and adjust the distance
   until there are approximately the same number of pixels of either
   color.

6. Repeat from step 4, until the distance from the Kinect to the flat
   surface is larger than the largest distance you intend to capture
   during use, or repeat until the Kinect is approximately 2m away from
   the surface (the maximum practical capture distance).

7. Press the "2" key to calculate per-pixel depth correction equations.
   When the calculation is complete, the correction parameters will
   automatically be written to a DepthCorrection-<serial number>.dat
   file in the Kinect package's configuration directory. The correction
   calculation may fail if there are too many depth pixels that don't
   have valid depth data, i.e., show up in black, in too many depth
   images. If an error message appears after pressing "2", capture
   additional depth images within the desired depth range as in step 4,
   and press "2" again.

8. Exit from RawKinectViewer.
