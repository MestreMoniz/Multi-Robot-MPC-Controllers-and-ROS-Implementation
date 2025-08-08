function [selected_ref_1, selected_ref_2] = selectBestReferences(...
    ref_estimated_1, has_intersection_1, intersection_indices_1, ...
    ref_estimated_2, has_intersection_2, intersection_indices_2)
% Selects the best references for two robots based on intersections.
%
% Inputs:
%   - ref_estimated_1: Estimated references for robot 1 (3 x M1)
%   - has_intersection_1: Boolean, true if robot 1 has intersections
%   - intersection_indices_1: Indices of intersections for robot 1
%   - ref_estimated_2: Estimated references for robot 2 (3 x M2)
%   - has_intersection_2: Boolean, true if robot 2 has intersections
%   - intersection_indices_2: Indices of intersections for robot 2
%
% Outputs:
%   - selected_ref_1: Selected reference for robot 1 [x; y; theta]
%   - selected_ref_2: Selected reference for robot 2 [x; y; theta]

% Default outputs
selected_ref_1 = [];
selected_ref_2 = [];

% Proceed only if both robots have intersections
if has_intersection_1 && has_intersection_2
    % Find the earliest intersection for each robot
    min_index_1 = intersection_indices_1(1); % First intersection for Robot 1
    min_index_2 = intersection_indices_2(1); % First intersection for Robot 2
    
    if min_index_1 < min_index_2
        % Robot 1 intersects first
        selected_ref_1 = ref_estimated_1(:, 1); % First intersection for Robot 1
        
        % Check for a valid second reference for Robot 2
        for i = 2:length(intersection_indices_2) % Start at the second element
            if intersection_indices_2(i) > min_index_1 && ...
               norm(ref_estimated_2(:, i) - selected_ref_1) > 2
                selected_ref_2 = ref_estimated_2(:, i); % Select farther intersection
                break; % Stop after finding a valid reference
            end
        end
        
        % Default to the first intersection if no valid farther reference is found
        if isempty(selected_ref_2)
            selected_ref_2 = ref_estimated_2(:, 1);
        end
    else
        % Robot 2 intersects first
        selected_ref_2 = ref_estimated_2(:, 1); % First intersection for Robot 2
        
        % Check for a valid second reference for Robot 1
        for i = 2:length(intersection_indices_1) % Start at the second element
            if intersection_indices_1(i) > min_index_2 && ...
               norm(ref_estimated_1(:, i) - selected_ref_2) > 2
                selected_ref_1 = ref_estimated_1(:, i); % Select farther intersection
                break; % Stop after finding a valid reference
            end
        end
        
        % Default to the first intersection if no valid farther reference is found
        if isempty(selected_ref_1)
            selected_ref_1 = ref_estimated_1(:, 1);
        end
    end
else
    % Handle cases where one or both robots have no intersections
    if has_intersection_1
        selected_ref_1 = ref_estimated_1(:, 1);
    end
    
    if has_intersection_2
        selected_ref_2 = ref_estimated_2(:, 1);
    end
end

end
