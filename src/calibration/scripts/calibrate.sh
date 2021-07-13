VISP_BUILD_DIR=?
CAMERA_POSE_ESTIMATOR_PATH=$VISP_BUILD_DIR/tutorial-chessboard-pose
CALIBRATOR_PATH=$VISP_BUILD_DIR/tutorial-hand-eye-calibration

W=?
H=?
SIZE=?
N_POSES=?

$CAMERA_POSE_ESTIMATOR_PATH -w $W -h $H --square_size $SIZE --camera_name Camera --intrinsic ./camera.xml --input image-%d.png
$CALIBRATOR_PATH --ndata $N_POSES
