function test_cam_pose_estimation(image_1, image_2, intrinsics, ...
    population_size, max_iterations, iteration_count, iteration_repeat, actual_pose)
    errors_tran = zeros(iteration_count, 2);
    errors_rot = zeros(iteration_count, 2);
    step = (max_iterations - 1) / (iteration_count - 1);
    index = 1;
    for i = 1:step:max_iterations
        iterations = floor(i);

        [err_tran_avg, err_rot_avg] = deal(0);
        for j = 1:iteration_repeat
            pose = cam_pose_estimate_gwo(image_1, image_2, intrinsics, population_size, iterations);
            [err_tran, err_rot] = err_pose_estimate(pose, actual_pose);
            err_tran_avg = err_tran_avg + err_tran;
            err_rot_avg = err_rot_avg + err_rot;
        end
        err_tran_avg = err_tran_avg / iteration_repeat;
        err_rot_avg = err_rot_avg / iteration_repeat;

        errors_tran(index, :) = [iterations err_tran_avg];
        errors_rot(index, :) = [iterations err_rot_avg];
        index = index + 1;
    end

    plot_errors(errors_tran, errors_rot);
end

function [err_tran, err_rot] = err_pose_estimate(pose, actual_pose)
    err_tran = norm(pose(1:3, 4) - actual_pose(1:3, 4));
    delta_rot = pose(1:3, 1:3) / actual_pose(1:3, 1:3);
    angle_axis = rotm2axang(orth(delta_rot));
    err_rot = angle_axis(4);
end

function plot_errors(errors_tran, errors_rot)
    figure('Name', 'Camera Pose Errors', 'NumberTitle', 'off');
    
    subplot(2, 1, 1);
    plot(errors_tran(:, 1), errors_tran(:, 2));
    xlabel 'Iterations';
    ylabel 'Distance';

    subplot(2, 1, 2);
    plot(errors_rot(:, 1), errors_rot(:, 2));
    xlabel 'Iterations';
    ylabel 'Angle Offset';
end