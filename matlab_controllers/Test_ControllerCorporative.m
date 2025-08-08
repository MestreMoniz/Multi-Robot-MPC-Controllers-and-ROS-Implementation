clear;
close all;
clear controllerCooperative
% Simulation Parameters
T_s = 0.1;             % Sampling time
T = 10;                % Total simulation time
N = round(T / T_s);    % Number of time steps
target_vel_max = 10;    % Maximum velocity of the target

% Initial positions
x_f1 = [80; 2; 0];   % Initial state of Follower 1 [x; y; theta]
x_f2 = [30; 1; 0];   % Initial state of Follower 2 [x; y; theta]

% Target positions and reference
target_initial_pos = [-5; 0; 0];          
target_final_pos = target_initial_pos + [target_vel_max * T; 0; 0]; 
points = [target_initial_pos, target_final_pos];
ref = ReferenceWithPoints(points, T_s, target_vel_max);

% Preallocate state histories
x_f1_history = zeros(3, N+1);
x_f2_history = zeros(3, N+1);
x_f1_history(:,1)= x_f1;
x_f2_history(:,1) = x_f2;

% Prepare figure
figure;
hold on;
grid on;
follower1_plot = plot(x_f1(1), x_f1(2), 'b-o', 'DisplayName', 'Follower 1');
follower2_plot = plot(x_f2(1), x_f2(2), 'r-o', 'DisplayName', 'Follower 2');
reference_plot = plot(ref(1, 1), ref(2, 1), 'k--', 'DisplayName', 'Reference');
xlabel('X Position');
ylabel('Y Position');
legend;
title('Real-time Trajectories of Followers and Reference');
axis equal;

% Simulation Loop
for i = 1:N
    
    % Stack follower positions for controller input
    x_stack = [x_f1_history(:, i);  x_f2_history(:, i)];
    
    % Get control inputs from the cooperative controller
    u_mpc = controllerCooperative(x_stack); % Returns [v1; omega1; v2; omega2]
    u_mpc_1 = u_mpc(1:2); % [v; omega] for Follower 1
    u_mpc_2 = u_mpc(3:4); % [v; omega] for Follower 2
    
    % Update states using discrete dynamics
    x_f1_history(:, i+1) = x_f1_history(:, i) + T_s * [cos(x_f1_history(3, i)), 0; sin(x_f1_history(3, i)), 0; 0, 1] * u_mpc_1;
     x_f2_history(:, i+1) =  x_f2_history(:, i) + T_s *[cos(x_f2_history(3, i)), 0; sin(x_f2_history(3, i)), 0; 0, 1] * u_mpc_2;
     
      % Update plots
    set(follower1_plot, 'XData', x_f1_history(1, 1:i+1), 'YData', x_f1_history(2, 1:i+1));
    set(follower2_plot, 'XData', x_f2_history(1, 1:i+1), 'YData', x_f2_history(2, 1:i+1));
    set(reference_plot, 'XData', ref(1, 1:i+1), 'YData', ref(2, 1:i+1));

    
    drawnow; % Refresh the figure
    

end

