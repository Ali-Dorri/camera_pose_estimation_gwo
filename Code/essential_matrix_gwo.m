image_1 = imread("../Data/Image_01.png");
image_2 = imread("../Data/Image_02.png");

% Create intrinsic parameters matrix knwon from dataset
[height, width, dime] = size(image_1);
intrinsics = cameraIntrinsics([585 585], [320 240], [height width]);

% Remove lens distortion
I1 = undistortImage(image_1, intrinsics);
I2 = undistortImage(image_2, intrinsics);
I1gray = im2gray(I1);
I2gray = im2gray(I2);

% extract feature points
imagePoints1 = detectSURFFeatures(I1gray);
imagePoints2 = detectSURFFeatures(I2gray);
features1 = extractFeatures(I1gray, imagePoints1, Upright=true);
features2 = extractFeatures(I2gray, imagePoints2, Upright=true);

% find match points
indexPairs = matchFeatures(features1,features2);
matchedPoints1 = imagePoints1(indexPairs(:,1));
matchedPoints2 = imagePoints2(indexPairs(:,2));

poses = initial_random_poses(matchedPoints1, matchedPoints2, intrinsics, 12);
fitness = CameraPoseFitness(matchedPoints1, matchedPoints2, intrinsics);
best_pose = gwo(poses, fitness, 400);
disp(best_pose);
disp(best_pose.Position);

function poses = initial_random_poses(matchedPoints1, matchedPoints2, intrinsics, min_poses)
    poses = zeros(4, 4, min_poses);
    pose_count = 0;
    while(pose_count < min_poses)
        new_poses = random_essential_poses(matchedPoints1, matchedPoints2, intrinsics);
        additional_count = size(new_poses, 3);
        new_count = pose_count + additional_count;
        
        % Increase poses size
        if new_count > min_poses
            additional_poses = new_count - min_poses;
            poses(:, :, end + 1 : end + additional_poses) = zeros(4, 4, additional_poses);
        end
    
        poses(:, :, pose_count + 1 : new_count) = new_poses;
        pose_count = new_count;
    end
end

function poses = random_essential_poses(matchedPoints1, matchedPoints2, intrinsics)
    all_count = size(matchedPoints1, 1);
    indices = floor(rand([1 5]) * (all_count - 1) + 1);
    points1 = matchedPoints1.Location(indices, :).';
    points2 = matchedPoints2.Location(indices, :).';
    [~, rotations, translations, ~] = five_point_algorithm(points1, points2, ...
        intrinsics.K, intrinsics.K);
    
    count = size(rotations, 1);
    poses = zeros(4, 4, count);
    for i = 1 : count
        cam_rotation = rotations{i}.';
        cam_translation = -translations{i};
        poses(:, :, i) = [cam_rotation cam_translation; 0 0 0 1];
    end
end

% figure
% showExtrinsics(cameraParams, 'CameraCentric');
% figure
% showExtrinsics(cameraParams, 'PatternCentric');

% function test_accuracy(best_pose)
%     % get actual poses from text files and calculate actual relative pose
%     % mse the best pose and the actual relative pose
% end

% For testing results, do this:
% Run the gwo with on different iteration counts (for each iteration, run it few times and get
% the average mse as the result)
% then plot the mse-iterations

% Also check other operations requested in the project