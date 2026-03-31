<!-- todo this is a duplicate of arsandbox docs/installation/hardware/step2.md -->
# Projection matrix calculation

The new intrinsic calibration method uses a semi-transparent checkerboard calibration target. The idea is that the target looks like a checkerboard both to the depth and color cameras. The user presents the calibration target to the Kinect in a sequence of different poses, and manually matches a 2D distorted grid to the observed calibration target in the depth and color cameras' views using the `RawKinectViewer` utility. After a sufficient number of poses have been captured, `RawKinectViewer` calculates the two calibration matrices and stores them in a uniquely identified file for subsequent retrieval whenever a Kinect device is activated.

The easiest and most precise way to build a semi-transparent checkerboard is to print a grid pattern onto a large sheet of paper, glue the entire sheet of paper onto an IR-transparent glass plate, cut precisely along all grid lines using a sharp knife and metal ruler, and then peel off every other paper square and remove any glue residue from the now transparent tiles. The grid should have an odd number of tiles in both directions such that all four corner tiles can stay opaque. Mark the center of the lower-left corner tile with a small dot (big enough to be visible in the Kinect's color image stream). This mark will be used to check for grid alignment during calibration.

The number of tiles, and the size of each tile, can be configured in `RawKinectViewer`, but the default grid layout has 7 $\times$ 5 tiles of 3.5" $\times$ 3.5" each. This leads to an overall target size of 24.5" $\times$ 17.5", which is just the right size to cover the Kinect's full viewing range.

The following is the intrinsic calibration procedure for a Kinect. There are two tutorial videos on YouTube:

* [Video 1](http://www.youtube.com/watch?v=Qo05LVxdlfo) explains steps 0 to 9 in
 the following procedure.
* [Video 2](http://www.youtube.com/watch?v=VQh4joyZwx8) explains step 10 in the
 following procedure.

## Steps

1. Prepare calibration target

2. Start `RawKinectViewer` for the Kinect to be calibrated. If the calibration target does not have the default layout, specify the layout using the `-gridSize` and `-tileSize` command line options.

3. Create a "Draw Grids" tool by binding it to six (6) buttons (keyboard keys or mouse buttons). Here, we will use the keys ++1++, ++w++, ++2++, ++3++, ++4++, ++5++, in that order. This will create two green grids, one in the depth and one in the color image stream. The grid can be deformed by grabbing any grid intersection using the mouse and the ++1++ key, or translated by grabbing the dot in the center of the grid, or rotated around the center by grabbing the dot slightly outside the right edge of the grid, all using the ++1++ key.

4. Place the calibration target in front of the Kinect at the next position and orientation. The target should be placed in a range of distances, starting from the near depth cutoff to the end of the desired tracking range, and in a variety of orientations from straight-on to about 45 degree angles. Calibration requires at least 4 calibration poses, but ideally a larger number to improve calibration quality.

5. Collect an average depth frame by turning on the "Average Frames" button in the main menu. Wait until the depth image does not change anymore.

6. Align both the depth and color grids grids to the observed grids. The dot in the center of the lower-left corner tile must roughly coincide with the mark in the lower-left corner tile of the calibration target to ensure that the grid is not flipped or rotated. The orientation of the depth and color stream grids will be very similar.

7. Store the current calibration grids by pressing the ++2++ key.

8. Turn off the "Average Frames" button in the main menu so that the depth image stream updates in real time again.

9. Repeat from step 3 with the next calibration pose.

10. After all calibration poses have been captured, calculate the intrinsic calibration by pressing the ++4++ key. This will print some diagnostic information to the terminal, and create the calibration file specific to the Kinect device in the Kinect package's configuration directory. The files are named `IntrinsicParameters-<serial number>.dat` and contain the two calibration matrices in binary format. They cannot be hand-edited.

11. After calibration, use KinectViewer and a 3D measurement tool to ensure that the size and shape of the virtual calibration target as reconstructed by the Kinect matches the real calibration target. A good calibration will create a match to a few percent. Keep in mind that measurements inside KinectViewer will be reported in centimeters.

<!-- There is a complementary write-up of the calibration procedure at
http://doc-ok.org/?p=289 -->