function ref = ReferenceWithPoints(points, T_s, max_velocity)
    % Function to expand a vector of points by adding interpolated points
    % based on the sampling time and maximum velocity
    %
    % Inputs:
    % - points: matrix of points (each column is a point [x; y; theta])
    % - T_s: sampling time
    % - max_velocity: maximum velocity (assumed constant for interpolation)
    %
    % Output:
    % - ref: matrix with interpolated points

    % Validate input
    if size(points, 1) ~= 3
        error('The input points must be a 3xN matrix.');
    end
    if size(points, 2) < 2
        error('The points matrix must contain at least two points.');
    end
    if T_s <= 0
        error('The sampling time T_s must be positive.');
    end
    if max_velocity <= 0
        error('The maximum velocity must be positive.');
    end

    % Initialize the expanded reference
    ref = [];

    % Loop through each consecutive pair of points
    for i = 1:(size(points, 2) - 1)
        % Start and end points
        p_start = points(:, i);
        p_end = points(:, i + 1);

        % Calculate distance between the start and end points
        distance = norm(p_end(1:2) - p_start(1:2));

        % Compute the time needed to traverse the distance
        time_to_traverse = distance / max_velocity;

        % Calculate the number of interpolation steps
        num_interpolation_points = ceil(time_to_traverse / T_s);

        % Interpolate each dimension
        interpolated_points = [];
        for dim = 1:size(points, 1)
            interpolated_points = [interpolated_points; ...
                                   linspace(p_start(dim), p_end(dim), num_interpolation_points + 2)];
        end

        % Exclude the last point to avoid duplication
        ref = [ref, interpolated_points(:, 1:end-1)];
    end

    % Add the final point
    ref = [ref, points(:, end)];
end
