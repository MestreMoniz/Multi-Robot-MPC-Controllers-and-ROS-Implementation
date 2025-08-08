function [ref_estimated, intersection_indices, has_intersection, times_target, times_follower, distances_follower, distances_target] = ...
    estimateIntersectionPoint(target_path, target_pos, target_vel, follower_pos, follower_vel, Ts, max_intersections)
% Estimate interception position of the follower robot with the target robot
% considering the target follows a predefined trajectory.
%
% Inputs:
%   - target_path: Matrix of trajectory positions [x; y; theta] (3 x N)
%   - target_pos: Current position [x; y; theta] of the target robot (3 x 1)
%   - target_vel: Current velocity [v; omega] of the target robot (2 x 1)
%   - follower_pos: Current position [x; y; theta] of the follower robot (3 x 1)
%   - follower_vel: Current velocity [v; omega] of the follower robot (2 x 1)
%   - Ts: Sampling time
%   - max_intersections: Maximum number of intersection points to find
%
% Outputs:
%   - ref_estimated: Intersection reference positions [x; y; theta] (3 x M) for M intersections
%   - intersection_indices: Indices of the trajectory points corresponding to intersections
%   - has_intersection: Boolean indicating if intersection exists (true/false)
%   - times_target: Times for the target to reach each point in the path
%   - times_follower: Times for the follower to reach each point in the path
%   - distances_follower: Euclidean distances between the follower and each point in the path
%   - distances_target: Cumulative distances traveled by the target at each point

% Unpack velocities
v_target = target_vel(1);
v_follower = follower_vel(1);

% Unpack follower position
x_follower = follower_pos(1);
y_follower = follower_pos(2);

% Initialize variables
num_points = size(target_path, 2);
ref_estimated = []; % Store all intersection points found
intersection_indices = []; % Store indices of intersections
has_intersection = false;
min_time_diff = Ts * v_target / 2; % Tolerance for intersection time

% Initialize cumulative distances and arrays to store times/distances
cumulative_distance_target = 0;
distances_target = zeros(1, num_points + 1);
times_target = zeros(1, num_points + 1);
times_follower = zeros(1, num_points + 1);
distances_follower = zeros(1, num_points + 1);

% Include the target's current position in the path
full_path = [target_pos, target_path];

% Loop through trajectory points starting at i = 2
for i = 2:num_points + 1
    % Calculate segment distance for the target's path
    dx = full_path(1, i) - full_path(1, i - 1);
    dy = full_path(2, i) - full_path(2, i - 1);
    segment_distance = sqrt(dx^2 + dy^2);
    cumulative_distance_target = cumulative_distance_target + segment_distance;

    % Store cumulative distance for the target
    distances_target(i) = cumulative_distance_target;

    % Time for the target to reach this point
    times_target(i) = cumulative_distance_target / v_target;

    % Euclidean distance for the follower to this point
    distances_follower(i) = sqrt((x_follower - full_path(1, i))^2 + (y_follower - full_path(2, i))^2);

    % Time for the follower to reach this point
    times_follower(i) = distances_follower(i) / v_follower;

    % Check for intersection
    if (abs(times_follower(i) - times_target(i)) < min_time_diff && length(intersection_indices) ==0 ) || ...
            (times_follower(i) < times_target(i) && ...
             norm(full_path(1:2, i) - ref_estimated(1:2, end)) >= 2)
        % Store intersection
        ref_estimated = [ref_estimated, full_path(:, i)]; 
        intersection_indices = [intersection_indices, i]; 
        has_intersection = true;

        % Stop if maximum intersections are found
        if length(intersection_indices) >= max_intersections
            break;
        end
    end
end

% Remove the initial values (zero at i = 1) for times and distances
times_target = times_target(2:i);
times_follower = times_follower(2:i);
distances_follower = distances_follower(2:i);
distances_target = distances_target(2:i);

end
