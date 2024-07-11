function position = gwo(positions, fitness_obj, iterations)
%Simple Implementation Of Grey Wolf Optimizer algorithm for camera pose
%estimation

    position_size_all = size(positions);
    position_size = [position_size_all(1) position_size_all(2)];
    a = 2;
    [A1, C1] = update_ac(a, position_size);
    [A2, C2] = update_ac(a, position_size);
    [A3, C3] = update_ac(a, position_size);
    
    count = size(positions, 3);
    fitness = zeros(count);
    alpha = Wolf(inf, []);
    beta = Wolf(inf, []);
    delta = Wolf(inf, []);
    [alpha, beta, delta] = calculate_fitness_wolfs(positions, fitness, fitness_obj, ...
        alpha, beta, delta);
    
    for iteration = 1 : iterations
        update_positions(positions, A1, C1, A2, C2, A3, C3, alpha.Position, beta.Position, ...
            delta.Position);
        a = lerp(2, 0, iteration / iterations);
        [A1, C1] = update_ac(a, position_size);
        [A2, C2] = update_ac(a, position_size);
        [A3, C3] = update_ac(a, position_size);
        [alpha, beta, delta] = calculate_fitness_wolfs(positions, fitness, fitness_obj, ...
        alpha, beta, delta);
    end

    position = alpha;
end

function [A, C] = update_ac(a, size)
    r1 = rand(size);
    r2 = rand(size);
    A = 2 .* a .* r1 - a;
    C = 2 .* r2;
end

function [alpha_new, beta_new, delta_new] = calculate_fitness_wolfs(positions, fitness, fitness_obj, ...
    alpha, beta, delta)
    for i = 1 : size(positions, 3)
        relative_pose = positions(:, :, i);
        fitness(i) = fitness_obj.get_fitness(relative_pose);
        if fitness(i) < alpha.Score
            alpha.Score = fitness(i);
            alpha.Position = relative_pose;
        elseif fitness(i) < beta.Score
            beta.Score = fitness(i);
            beta.Position = relative_pose;
        elseif fitness(i) < delta.Score
            delta.Score = fitness(i);
            delta.Position = relative_pose;
        end
    end

    alpha_new = alpha;
    beta_new = beta;
    delta_new = delta;
end

function value = lerp(start_, end_, t)
    value = t * (end_ - start_) + start_;
end

function update_positions(positions, A1, C1, A2, C2, A3, C3, alpha, beta, delta)
    for i = 1 : size(positions, 3)
        position = positions(:, :, i);
        D_alpha = abs(C1 .* alpha - position);
        D_beta = abs(C2 .* beta - position);
        D_delta = abs(C3 .* delta - position);
        X1 = alpha - A1 .* D_alpha;
        X2 = beta - A2 .* D_beta;
        X3 = delta - A3 .* D_delta;
        positions(:, :, i) = (X1 + X2 + X3) / 3;
    end
end