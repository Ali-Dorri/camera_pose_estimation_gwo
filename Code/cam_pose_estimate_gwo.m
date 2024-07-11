function estimated_pose = cam_pose_estimate_gwo(image_1, image_2, cam_intrinsics, ...
    population_size, iterations)
    % Remove lens distortion
    I1 = undistortImage(image_1, cam_intrinsics);
    I2 = undistortImage(image_2, cam_intrinsics);
    I1gray = im2gray(I1);
    I2gray = im2gray(I2);
    
    % Extract feature points
    imagePoints1 = detectSURFFeatures(I1gray);
    imagePoints2 = detectSURFFeatures(I2gray);
    features1 = extractFeatures(I1gray, imagePoints1, Upright=true);
    features2 = extractFeatures(I2gray, imagePoints2, Upright=true);
    
    % Find match points
    indexPairs = matchFeatures(features1,features2);
    matchedPoints1 = imagePoints1(indexPairs(:,1));
    matchedPoints2 = imagePoints2(indexPairs(:,2));
    
    % Estimate the camera pose
    poses = initial_random_poses(matchedPoints1, matchedPoints2, cam_intrinsics, population_size);
    fitness = CameraPoseFitness(matchedPoints1, matchedPoints2, cam_intrinsics);
    best_pose = gwo(poses, fitness, iterations);
    estimated_pose = best_pose.Position;
end

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