# realsense-holder-calibration
A tool for calibrating the iCub neck to RealSense transformation matrix when using the RealSense holder

## Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [ViSP](https://visp.inria.fr/install/)

## How to build

```
git clone https://github.com/robotology-playground/realsense-holder-calibration
mkdir build
cd build
cmake ../
make install
```

Note: this repository use CMake `ICUBcontribHelpers` helpers and will automatically detect where to install the package. If the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) is used, the package will be installed in superbuild install path.

## How to configure

1. Copy the template `yarpmanager` application from `<package_install_dir>/share/ICUBcontrib/templates/realsense-holder-calibration.xml` to e.g. `.local/share/yarp/applications`
2. Import the context of this package using `yarp-config context --import realsense-holder-calibration`
3. Open the local configuration file `.local/share/yarp/context/realsense-holder-calibration/config.ini` and make sure that:
   - the camera intrinsics in `[CAMERA_INTRINSICS]` match those of the adopted RGB input
   - the `robot_name` is the correct one
   - the `eye_version` matches that of the used robot
   - suitable calibration poses are provided (the order is `torso_yaw`, `torso_pitch`, `torso_roll`, `neck_pitch`, `neck_roll`, `neck_yaw`)
   - the number of poses `number_of_poses` matches the actual number of poses

If a `RealSense` camera is used and it is accessed via the associated `yarpdev`, it is possible to obtain the intrinsic parameters of the RGB sensor using
```
yarp rpc /depthCamera/rpc:i`
> visr get intp
```

## How to prepare

Print the [chessboard](https://visp-doc.inria.fr/download/calib-grid/OpenCV_Chessboard.pdf) provided by [VISP](https://visp.inria.fr/install/). If possible print it on a `A3` paper.

## How to collect data

1. Make sure that iCub is up and running (the torso and the head are required)
2. Move in a folder where the data will be saved, e.g. `cd <data_folder>`
3. Open the `yarpmanager` from `<data_folder>`
4. Open the `Eye-hand_calibration` application
5. Run all the applications and connect the ports
> This will also run the `yarpdev` for the RealSense camera. If a different camera is used, please do not run the `yarpdev` and change the input port names in the `yarpmanager` window accordingly.
6. Open an RPC client via `yarp rpc /realsense-holder-calibration/rpc:i`
7. Type `start` to start the data acquisition
> **Warning**: the robot will execute the poses as provided in the configuration file and will wait for `wait_time` before moving to the next pose. Robot motion can be stopped at any time by typing `stop` in the RPC client. The parameter `wait_time` can be specified in the configuration file.

> **Note:** Make sure that the chessboard is completely visibile in all calibration poses, e.g. by checking on the `yarpview`. If not, move the chessboard or change the torso and head joints configuration in the local configuration file, close the module by typing `quit`, re-open it and start the procedure again.
8. The robot will go back in home position (torso and neck set to zero) after the data acquisition is complete.

## How to obtain the calbration matrix

Open a terminal and run `realsense-holder-calibration-process` having the following synopsis:
```
Synopsis: realsense-holder-calibration-process <path_to_visp_build> <path_to_images> <number_poses> <width> <height> <square_size (meters)>
```
where `<path_to_images>` is `<data_folder>` (as indicated in the section #how-to-collect-data) and `<square_size>` is the length of the side of the square in the chessboard (in meters).

If the provided [chessboard](https://visp-doc.inria.fr/download/calib-grid/OpenCV_Chessboard.pdf) is used, then `<width> = 9`, `<height> = 6` and `<square_size (meters)> = 0.036`. Please verify the length using a ruler given that options of the printer might alter it.

After running the script, the list of collected images will be shown. Click using the left button of the mouse to move to the next pose. It is important that the chessboard is detected in **all** the images.

![image](https://user-images.githubusercontent.com/6014499/125421147-00ba8399-006d-4d5e-ae55-ecb3894c6732.png)

After all images have been considered, the script will provide the calibration matrix (from the [iCubHeadCenter](https://robotology.github.io/robotology-documentation/doc/html/classiCub_1_1iKin_1_1iCubHeadCenter.html) frame to the RealSense RGB frame) as output, e.g. 

```
Transformation from iCubHeadCenter to RealSense RGB frame:

0.9986644239  0.03441962741  -0.03853125538  -0.05194061188
-0.03017476666  0.9939304797  0.1057907607  -0.1212326766
0.0419386677  -0.1044867974  0.9936416141  0.02958705726
0  0  0  1
```

The output can also be found in `txt` and `yaml` format in `<data_folder>/eMc.txt` and `<data_folder>/eMc.yaml`. The latter provides the transformation as a 6-dimensional vector in the form `(x, y, z, u_x, u_y, u_z)` where `(u_x, u_y, u_z)` is the product between the axis and the angle of the axis/angle parametrization of the rotation matrix.


