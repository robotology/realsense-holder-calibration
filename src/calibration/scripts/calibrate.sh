#===============================================================================
#
# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

if [ $# -ne 6 ]
then
    echo "Synopsis: "$(basename $0)" <path_to_visp_build> <path_to_images> <number_poses> <width> <height> <square_size (meters)>"
    exit 1
fi

VISP_BUILD_DIR=$1
IMAGES_PATH=$2
N_POSES=$3
W=$4
H=$5
SIZE=$6

CAMERA_POSE_ESTIMATOR_PATH=$VISP_BUILD_DIR/tutorial/calibration/tutorial-chessboard-pose
CALIBRATOR_PATH=$VISP_BUILD_DIR/tutorial/calibration/tutorial-hand-eye-calibration

(cd $IMAGES_PATH && $CAMERA_POSE_ESTIMATOR_PATH -w $W -h $H --square_size $SIZE --camera_name Camera --intrinsic ./camera.xml --input ./image-%d.png)
(cd $IMAGES_PATH && $CALIBRATOR_PATH --ndata $N_POSES)
clear
echo "Transformation from iCubHeadCenter to RealSense RGB frame:"
echo ""
cat $IMAGES_PATH/eMc.txt
echo ""
