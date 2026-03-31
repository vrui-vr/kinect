To merge the 3D reconstructions of multiple Kinect cameras, first
intrinsically calibrate each one using `RawKinectViewer`, as described
above and shown in the videos. Then extrinsically calibrate all Kinect
devices with respect to an arbitrarily chosen world coordinate system.

1. Place calibration target into field of view of all Kinect devices to
   be calibrated (use `RawKinectViewer` on each to check).

2. For each Kinect device:

   2a. Run `RawKinectViewer` and fit a grid to the calibration target's
       image in the depth stream (it is not necessary to fit to the
       color stream).

   2b. Save and unproject the grid to receive a list of 3D tie points on
       the console.

   2c. Copy the tie points into a file `KinectPoints-<serial number>.csv`.

3. Create a file TargetPoints.csv, containing the 3D interior corner
   positions of the calibration target, in some arbitrary right-handed
   world coordinate system using an arbitrary unit of measurement, in
   the same order as printed by `RawKinectViewer` (going from left to
   right, and then bottom to top). See example target file below.

4. For each Kinect device:

   4a. Run the Kinect device's tie points and the target points through
       AlignPoints:


       AlignPoints -OG KinectPoints-<serial number>.csv TargetPoints.csv


   4b. Observe the mismatches between purple camera points and green
       target points in AlignPoints' display. If the discrepancies are
       too large, or there is no fit at all, repeat step two for the
       Kinect device.

   4c. Paste the final best fitting transform displayed by AlignPoints
       (everything after the "Best transformation:" header) into a
       `ExtrinsicParameters-<serial number>.txt` file in the Kinect
       package's configuration directory (`Kinect-<version>` in the Vrui
       installation's `etc/` directory).

5. Run `KinectViewer` on all Kinect devices. This will show all individual
   3D video streams matching more or less seamlessly, depending on how
   much care was taken during intrinsic and extrinsic calibration. Under
   ideal circumstances, there should be no noticeable mismatches between
   individual streams.

Unlike intrinsic calibration, extrinsic calibration has to be repeated
any time any of the calibrated Kinect devices are moved, even by very
little. It is recommended to rigidly attach all Kinect devices to a
sturdy frame before attempting precise extrinsic calibration in a
production setting. If precision is paramount, it might even be
necessary to repeat calibration periodically if nothing changed, to
account for time-varying deformations inside the devices themselves.

An example TargetPoints.csv file, for a 5x4 calibration grid with a tile
size of 3" (a 5x4 grid has 4x3 interior corners):

```txt
0, 0, 0
3, 0, 0
6, 0, 0
9, 0, 0
0, 3, 0
3, 3, 0
6, 3, 0
9, 3, 0
0, 6, 0
3, 6, 0
6, 6, 0
9, 6, 0
```

The resulting world space will use whatever units were used to define
the target points; in the example above, world space will use inches.

<!-- There is a complementary write-up of the calibration procedure at http://doc-ok.org/?p=295 -->