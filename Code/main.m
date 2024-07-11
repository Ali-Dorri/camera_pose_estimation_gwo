image_1 = imread("../Data/Image_01.png");
image_2 = imread("../Data/Image_02.png");

% Create intrinsic parameters matrix knwon from dataset
[height, width, dim] = size(image_1);
intrinsics = cameraIntrinsics([585 585], [320 240], [height width]);

actual_pose_1 = readmatrix("../Data/ImagePose_01.csv");
actual_pose_2 = readmatrix("../Data/ImagePose_02.csv");
actual_pose_1_to_2 = actual_pose_2 / actual_pose_1;

test_cam_pose_estimation(image_1, image_2, intrinsics, 12, 100, 5, 1, actual_pose_1_to_2);