!!! tip
    It is recommended to download or move the source packages for Vrui and the Kinect 3D Video Capture Project into a `src` directory underneath the user's home directory. Otherwise, references to `~/src` in the following instructions need to be changed.

<!-- ## Step 1: Install Vrui's collaboration infrastructure (optional)

<!-- todo add collab infrastructure to docs and link here -->
More info about the collaboration infrastructure and the installation guide can be found at [github.com/vrui-vr/collaboration/blob/main/README](https://github.com/vrui-vr/collaboration/blob/main/README). -->

## Step 1: Unpack the Kinect 3D Video Capture

### Option 1: Downloading and unpacking a zip file from GitHub

On [the Kinect repository's main page](https://github.com/vrui-vr/kinect), click on the green "<> Code" button, and then click on "Download ZIP" in the menu that pops up in response.

![Downloading a ZIP from a GitHub repo](download_zip.png)

Depending on your browser settings, you may be asked where to store the file being downloaded, or it might be stored in a default location, such as your `Downloads` directory. Take note of what the zip file is called and where it is stored.

Assuming that you already created the `src` directory *according to Vrui's installation instructions*, enter the following line into a terminal window once the file is completely downloaded:

```sh
cd ~/src
```

Then enter into the same terminal window:

```sh
unzip <path to downloaded zip file>
```

Replace `<path to downloaded zip file>` with the full path to the zip file, for example `~/Downloads/kinect-main.zip`.

Finally, check for the name of your new Kinect directory by entering:

```sh
ls
```

which will list all files in the `src` directory, which should include a new directory called `kinect-main`. Take note of this name, and then enter into that directory by typing this command into the terminal window:

```sh
cd <Kinect directory>
```

where you replace `<Kinect directory>` with the name of the directory where you cloned/unpacked the Kinect in the previous step, as printed by `ls`.

### Option 2: Clone the repository from GitHub

Assuming that you already created the `src` directory *according to Vrui's installation instructions*, navigate to the `src` directory on your computer in the terminal window.

```sh
cd ~/src
```

Then, clone the repository from GitHub:

```sh
git clone https://github.com/vrui-vr/kinect.git
```

Finally, check for the name of your new Kinect directory by entering:

```sh
ls
```

which will list all files in the `src` directory, which should include a new directory called `kinect`. Take note of this name, and then enter into that directory by typing this command into the terminal window:

```sh
cd <Kinect directory>
```

where you replace `<Kinect directory>` with the name of the directory where you cloned/unpacked the Kinect in the previous step, as printed by `ls`.

!!! warning
    If your installed Vrui version is not 13.0, or Vrui's installation directory was changed from the default of `/usr/local`, adapt the makefile using a text editor.

    Change the value of `VRUI_MAKEDIR` close to the beginning of the file as follows: `VRUI_MAKEDIR := <Vrui install dir>/share/make`, where <Vrui install dir> is the installation directory chosen when you installed Vrui. Use `$(HOME)` to refer to the user's home directory instead of `~`.

## Step 2: Build the Kinect 3D Video Capture Project

Run the following from inside the `<Kinect directory>`:

```sh
make
```

Once `make` has finished running, check that there were no error messages. The quickest way to check whether the Kinect built successfully is to run the `make` command a second time, *exactly* as you entered it the first time.

If everything went well the first time, the second run will print:

```sh
make: Nothing to be done for 'all'.
```

## Step 3: Install the Kinect 3D Video Capture Project

If Vrui was installed in `/usr/local` or elsewhere outside the user's home directory:

```sh
sudo make install
```

This will ask for the user's password to install the Kinect package inside Vrui's installation directory.

If Vrui was installed inside the user's home directory:

```sh
make install
```

## Step 4: Install a udev rule file to give access to all users (optional, Linux-only)

By default, Kinect devices can only be accessed by the root user. This is inconvenient and a security risk, as all Kinect applications must be run as root.

The Kinect package contains a `udev` rule file to give full access to any Kinect device to the user currently logged in to the local console.

First, run:

```sh
sudo make installudevrules
```

This will ask for the user's password to copy the rule file,
`69-Kinect.rules`, into the `udev` rule directory `/etc/udev/rules.d`. *After* the rule file has been installed, it needs to be activated.

The recommended way is to restart the device manager:

```sh
sudo udevadm control --reload
sudo udevadm trigger --action=change
```

Alternatively, the rule can be activated by unplugging the Kinect from its USB port, waiting a few seconds, and then plugging it back in, or, if everything else fails, by restarting the computer. After initial installation, the rule file will be activated automatically when the computer boots or when a Kinect device is plugged into any USB port.
