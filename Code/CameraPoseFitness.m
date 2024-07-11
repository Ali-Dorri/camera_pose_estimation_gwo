classdef CameraPoseFitness
    %Provides a fitness function for the estimated camera rotation and
    %translation
    
    properties
        Points1
        Points2
        Intrinsics
    end
    
    methods
        function obj = CameraPoseFitness(points1, points2, intrinsics)
            obj.Points1 = points1;
            obj.Points2 = points2;
            obj.Intrinsics = intrinsics;
        end
        
        function fitness = get_fitness(obj, transformation)
            %Get fittness of the camera relative pose (rotation, translation)
            fitness = calculate_fitness(obj.Points1, obj.Points2, obj.Intrinsics, transformation);
        end
    end
end

function fitness = calculate_fitness(points1, points2, intrinsics, transformation)
    cam_pos_1 = [0; 0; 0; 1];
    cam_fwd_1 = [0; 1; 0; 1];
    cam_right_1 = [1; 0; 0; 1];
    cam_up_1 = [0; 0; 1; 1];
    translation = [transformation(1:3, 4); 0];
    cam_pos_2 = cam_pos_1 + translation;
    cam_fwd_2 = transformation * cam_fwd_1 - cam_pos_2;
    cam_right_2 = transformation * cam_right_1 - cam_pos_2;
    cam_up_2 = transformation * cam_up_1 - cam_pos_2;
    
    fitness = 0;
    for i = 1 : size(points1, 1)
        direction_1 = camera_pixel_to_direction(cam_fwd_1, cam_right_1, cam_up_1, ...
            intrinsics, points1.Location(i, :));
        direction_2 = camera_pixel_to_direction(cam_fwd_2, cam_right_2, cam_up_2, ...
            intrinsics, points2.Location(i, :));
        distance = line_distance(cam_pos_1, direction_1, cam_pos_2, direction_2);
        fitness = fitness + distance;
    end
end

function world_direction = camera_pixel_to_direction(cam_fwd, cam_right, cam_up, intrinsics, pixel)
    % Calculate world point on a plane with 1 unit farther from the
    % camera's position in the camera's forward direction
    % Becuase the plane distance is 1 unity, the local x and y positions
    % are the same as local tan_x and tan_y
    local_x = pixel(1) - intrinsics.PrincipalPoint(1) / intrinsics.FocalLength(1);
    local_y = pixel(2) - intrinsics.PrincipalPoint(2) / intrinsics.FocalLength(2);

    % Ignore the fourth element (1)
    fwd = cam_fwd(1:3, 1);
    right = cam_right(1:3, 1);
    up = cam_up(1:3, 1);
    
    world_direction = fwd + right * local_x + up * local_y;
end

function distance = line_distance(point_1, direction_1, point_2, direction_2)
    cross_vector = cross(direction_1, direction_2);
    delta_4 = point_1 - point_2;
    delta_3 = delta_4(1:3, 1);
    distance = abs(dot(cross_vector, delta_3)) / norm(cross_vector);
end