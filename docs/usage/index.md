# Running the Kinect package

## Step 1: Find all connected Kinect devices

Run

```sh
./bin/KinectUtil list
```

to list all Kinect devices connected to the host computer.

## Step 2: Camera calibration

### Option 1: Factory calibration

Run

```sh
sudo ./bin/KinectUtil getCalib <camera index>
```

to download factory calibration data from the `<camera index>-`th Kinect device on the host's USB bus (with the first device having index 0) and create an intrinsic camera calibration file for that device in the Kinect package's configuration file directory.

### Option 2: Intrinsic calibration

Run

```sh
./bin/RawKinectViewer <camera index>
```

to intrinsically calibrate the `<camera index>-`th Kinect device on the host's USB bus (with the first device having index 0), using a semi-transparent checkerboard calibration target (see next section for details). This generates a binary `IntrinsicParameters-<serial number>.dat` file in <`Vrui install dir>/etc/Kinect-<version>`, where `<serial number>` is the unique factory-assigned serial number of the Kinect device as displayed in step 1.

!!! Note
    Either of the above options only have to be performed once for each Kinect device, unless the resulting calibration proves unsatisfactory. Kinect devices typically retain their intrinsic calibration over time.

<!-- ? what does this step do -->
## Step 3

Run

```sh
./bin/KinectViewer -c <camera index>
```

where `<camera index>` is the zero-based index of the Kinect device to use if there are multiple ones.