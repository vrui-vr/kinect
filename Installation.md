# Simple Kinect 3D Video Package Installation

Follow these instructions to install the Kinect 3D Video package with 
standard settings. These instructions assume that you have already 
installed the [Vrui VR Toolkit](https://github.com/vrui-vr/vrui), 
following the simple installation instructions therein.

In the following, when asked to enter something into a terminal, each line you 
are supposed to enter starts with a `$` denoting the terminal's command prompt. 
Do *not* enter that `$`, but enter everything that follows, and end each line by 
pressing the Enter key.

Angle brackets `<>` in commands below are placeholders, meaning that 
you have to replace everything between, and including, the angle 
brackets with some text that depends on your specific circumstances. For 
example, if your host has eight CPUs, instead of entering `-j<number of 
CPUs>` as part of some command, you would enter `-j8`.

## Step 0: Download The Kinect 3D Video Package Repository From GitHub

If you are reading this, you probably have already done this. :) If 
not, download the entire Kinect 3D Video package repository from 
github, either by cloning the repository using `git clone`, or by 
downloading and unpacking a zip file.

### Downloading And Unpacking A Zip File From GitHub

If you are unfamiliar with git and/or GitHub, you should probably go 
the zip file route. On [the Kinect 3D Video Package repository's main 
page](https://github.com/vrui-vr/kinect), click on the green "<> Code" 
button, and then click on "Download ZIP" in the menu that pops up in 
response. Depending on your browser settings, you may be asked where to 
store the file being downloaded, or it might be stored in a default 
location such as your `Downloads` directory. Either way, take note of 
what the zip file is called, and where it is stored.

Then, once the file is completely downloaded, enter the following line 
into a terminal window:
```
$ cd ~/src
```
assuming that you already created the `src` directory according to 
Vrui's installation instructions.

Then enter into the same terminal window:
```
$ unzip <path to downloaded zip file>
```
where you replace `<path to downloaded zip file>` with the full path to 
the zip file, for example `~/Downloads/kinect-main.zip`.

Finally, check for the name of your new Kinect directory by entering:
```
$ ls
``` 
which will list all files in the `src` directory, which should include 
a new directory called something like `kinect-main`. Take note of this 
name, and then enter into that directory by entering into the terminal 
window:
```
$ cd <Kinect directory>
```
where you replace `<Kinect directory>` with the name of the directory 
where you cloned/unpacked the Kinect package in the previous step, as 
printed by `ls`.

## Step 1: Build The Kinect 3D Video Package

To build the Kinect 3D Video package, enter into the same terminal window:
```
$ make VRUI_MAKEDIR=<Vrui build system location>
```
where you replace `<Vrui build system location>` with the location of 
Vrui's build system on your host, as described in Vrui's installation 
instructions. For example, your command might end up being:
```
$ make VRUI_MAKEDIR=/usr/local/share/Vrui-13.1/make
```

You can speed up the build process if your host has multiple CPUs or CPU cores. 
Instead of the above, enter into the same terminal:
```
$ make VRUI_MAKEDIR=<Vrui build system location> -j<number of cpus>
```
again replacing `<Vrui build system location>` with the location of 
Vrui's build system on your host, and replacing `<number of cpus>` with 
the number of CPUs or CPU cores on your host, say `-j8` if you have 
eight cores. Note that there is no space between the `-j` and the 
number of cores.

Once `make` is done, check that there were no error messages. The 
quickest way to check whether the Kinect 3D Video package built 
successfully is to enter the `make` command a second time, exactly as you 
entered it the first time. The easiest way to repeat a previous command 
is to press the "cursor up" key until the command appears on the 
command line, and then press the Enter key. If everything went well 
the first time, the second command will print:
```
make: Nothing to be done for 'all'.
```

## Step 2: Install The Kinect 3D Video Package

After building the Kinect 3D Video package successfully, you can 
install it with your existing Vrui installation by entering the 
following into the same terminal window:
```
$ sudo make VRUI_MAKEDIR=<Vrui build system location> install
```
which will ask you for your user account's password to install the 
Kinect 3D Video package in a system 
location, and then install it. This should be quick. After the command 
completes, check that there were no errors.

## Step 3: Install Device Access Rules

On Linux, additional devices such as Kinect 3D cameras are by default 
only accessible to administrator users. Since you will probably want to 
run applications that access your 3D camera from a regular user 
account, the Kinect 3D Video package contains a device access rule file 
that grants access to several common 3D camera types to regular users.

In order to install this rule file, enter the following into the same 
terminal window:
```
$ sudo make VRUI_MAKEDIR=<Vrui build system location> installudevrules
```

After running this command, unplug and re-plug your 3D camera(s) from 
your host to apply the new rules.

## Step 4: Check Your 3D Camera(s)

You can check whether the 3D camera(s) connected to your host are 
detected by the Kinect 3D Video package by entering into a terminal 
window:
```
$ KinectUtil list
```
which will list all detected and supported 3D cameras on your host. If 
there are no 3D cameras, there will be no output.

## Step 4: Download 3D Camera Calibration Data

If you have a first-generation Kinect camera, you should download the 
calibration data stored in its firmware. Enter into a terminal window:
```
$ sudo KinectUtil getCalib <camera index>
```
where you replace `<camera index>` with the index of the 
first-generation Kinect camera whose data you want to download in the 
list of cameras printed by `KinectUtil list`. The index is zero-based, 
meaning that if, for example, your first-generation Kinect camera is 
the first camera in the list, you would enter:
```
$ sudo KinectUtil getCalib 0
```

The above command requires `sudo`, and may ask for your user account's 
password, because the downloaded calibration data will be stored in the 
Kinect 3D Video package's configuration directory, which is in a system 
location.

If you have multiple connected first-generation Kinect cameras, you 
would run the same command for each of them, with their respective list 
indices.

