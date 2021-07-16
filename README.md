# realsense-holder-calibration

![image](https://user-images.githubusercontent.com/6014499/125424864-4f1a5d42-202f-4103-825a-93a4e78c3165.png)

A tool for estimating the [iCubHeadCenter](https://robotology.github.io/robotology-documentation/doc/html/classiCub_1_1iKin_1_1iCubHeadCenter.html) to RealSense transformation matrix when using the iCub RealSense holder.

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

Note: this repository use CMake `ICUBcontribHelpers` helpers and will automatically detect where to install the package. If the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) is used, the package will be installed in the superbuild install path.

## How to configure

### Module `realsense-holder-calibration`

1. Copy the template `yarpmanager` application from `<package_install_dir>/share/ICUBcontrib/templates/realsense-holder-calibration.xml` to e.g. `.local/share/yarp/applications`
2. Import the context of this package using `yarp-config context --import realsense-holder-calibration`
3. Open the local configuration file `.local/share/yarp/context/realsense-holder-calibration/config.ini` and make sure that:
   - the camera intrinsics in `[CAMERA_INTRINSICS]` match those of the adopted RGB input
   - the `robot_name` is the correct one
   - the `eye_version` matches that of the used robot
   - suitable calibration poses are provided (the order is `torso_yaw`, `torso_roll`, `torso_pitch`, `neck_pitch`, `neck_roll`, `neck_yaw`)
   - the number of poses `number_of_poses` matches the actual number of poses

#### How to obtain the intrinsic parameters of the RealSense
If a `RealSense` camera is used and it is accessed via the associated `yarpdev`, it is possible to obtain the intrinsic parameters of the RGB sensor using
```
yarp rpc /depthCamera/rpc:i
> visr get intp
```

#### iCubGenova01 configuration files
If the robot of interest is `iCubGenova01` the configuration files `config_iCubGenova01_holder_tilt.ini` and `config_iCubGenova01_holder_no_tilt.ini` are available in the context `realsense-holder-calibration` after a succesfull installation of the package. They can be loaded by running the application using the parameter 
```
--from config_iCubGenova01_holder[_no]_tilt.ini
```
| `no_tilt` variant  | `tilt` variant |
| ------------- | ------------- |
|<p align="center"> <img src=https://user-images.githubusercontent.com/9716288/99807903-6c4fc300-2b40-11eb-9856-4725f4e541b7.png width="150"> </p> | <p align="center">  <img src=https://user-images.githubusercontent.com/9716288/101136146-3107c680-360d-11eb-808d-3b109b9579d5.png width="150"> </p> |

Please check the intrinsic parameters of your camera, and eventually change them, before using these configuration files.

### Module `realsense-holder-publisher`

1. Copy the template `yarpmanager` application from `<package_install_dir>/share/ICUBcontrib/templates/realsense-holder-publisher.xml` to e.g. `.local/share/yarp/applications`
2. Import the context of this package using `yarp-config context --import realsense-holder-publisher`
3. The local configuration file `.local/share/yarp/context/realsense-holder-publisher/config.ini` will contain:
   - the robot name
   - the desired update period of the module
   - the eye version of the robot
   - (optional) the absolute path to the calibration matrix file (produced by the module `realsense-holder-calibration`)

If the path of the calibration matrix file is not provided, the module will search for it in `.local/share/yarp/context/realsense-holder-publisher/eMc.txt`.

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

## How to obtain the calibration matrix

Open a terminal and run `realsense-holder-calibration-process` having the following synopsis:
```
Synopsis: realsense-holder-calibration-process <path_to_visp_build> <path_to_images> <number_poses> <width> <height> <square_size (meters)>
```
where 
- `<path_to_visp_build>` is the [ViSP](https://visp.inria.fr/install/) build folder
- `<path_to_images>` is `<data_folder>` (as indicated in the section [how to collect data](https://github.com/robotology-playground/realsense-holder-calibration/blob/main/README.md#how-to-collect-data))
- `<number_poses` is the same as the parameter `number_of_poses` in the configuration file
- `<width>` is the width of chessboard
- `<height>` is the height of the chessboard
- `<square_size>` is the length of the side of the square in the chessboard (in meters)

If the provided [chessboard](https://visp-doc.inria.fr/download/calib-grid/OpenCV_Chessboard.pdf) is used, then `<width> = 9`, `<height> = 6` and `<square_size (meters)> = 0.036` (assuming that it has been printed on a A3 paper). Please verify the length using a ruler given that the options of your printer might alter it.

After running the script, the list of collected images will be shown. Click using the left button of the mouse to move to the next pose. It is important that the chessboard is detected in **all** the images.

<img src=https://user-images.githubusercontent.com/6014499/125697702-0cac5ca3-33db-4ffd-9aea-89e96903358a.png width="300">

After all images have been considered, the script will provide the calibration matrix (from the [iCubHeadCenter](https://robotology.github.io/robotology-documentation/doc/html/classiCub_1_1iKin_1_1iCubHeadCenter.html) frame to the RealSense RGB frame) as output, e.g. 

```
Transformation from iCubHeadCenter to RealSense RGB frame:

 0.9986644239   0.03441962741 -0.03853125538 -0.05194061188
-0.03017476666  0.9939304797   0.1057907607  -0.1212326766
 0.0419386677  -0.1044867974   0.9936416141   0.02958705726
 0              0              0              1
```

The output can also be found in `txt` and `yaml` format in `<data_folder>/eMc.txt` and `<data_folder>/eMc.yaml`. The latter provides the transformation as a 6-dimensional vector in the form `(x, y, z, u_x, u_y, u_z)` where `(u_x, u_y, u_z)` is the product between the axis and the angle of the axis/angle parametrization of the rotation matrix.

#### Calibration matrices of pre-existing holders

```
Variant no-tilt

 0.9987039757   0.02941588291 -0.04153401947 -0.05439578875
-0.02502241057  0.9944078874   0.102600353   -0.1173573506
 0.04431983652 -0.1014280992   0.9938551669   0.02902883402
 0              0              0              1
```

```
Variant tilt

 0.9986796783   0.03750826048 -0.0351002926 -0.05794521346
-0.001885871738 0.7095898996   0.7046123884 -0.05406200527
 0.0513355981  -0.7036158787   0.7087237484  0.06424080642
 0              0              0             1
```

## How to get the camera pose in the iCub root frame

1. Make sure that iCub is up and running (the torso and the head are required)
2. Open the `yarpmanager`
3. Open the `Eye-hand_calibration publisher` application
4. Specify the desired configuration file in the parameters using `--from <nome_of_config_file>` if any
5. Run `realsense-holder-publisher`

The pose of the camera will be available from `/realsense-holder-publisher/pose:o` as a list of `<x> <y> <z> <axis_x> <axis_y> <axis_z> <angle>` numbers where `<x>`, `<y>` and `<z>` are the 3D coordinates of the camera in the robot root frame, while the `<axis_?>` and `<angle>` are the axis/angle representation of the rotation matrix from the robot root frame to the camera reference frame.


### Maintainers

This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/xenvre.png" width="40">](https://github.com/xenvre) | [@xenvre](https://github.com/xenvre) |
| [<img src="https://github.com/gabrielecaddeo.png" width="40">](https://github.com/gabrielecaddeo) | [@gabrielecaddeo](https://github.com/gabrielecaddeo) |
