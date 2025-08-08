clear controllerTracking
% Parameters
T_s = 0.1;     % Sampling time (0.005s for calculation based on formula)
t =0:T_s:20*pi; % Exercise 2)
T = length(t);
horizon = 10;

ref = [5 * sin(t / 5); 5 * cos(t / 10);zeros(1, length(t))];
% Extend the reference by repeating the last value for N steps
r_extended = [ref, ref(:,1+1:horizon+1)];

% Initial conditions
x0 = [0; 0; 0];  % Initial state [x, y, theta]
v_prev = 0;      % Initial previous linear velocity
omega_prev = 0;  % Initial previous angular velocity
u_mpc_last = [0; 0]; % Initial control input

% Initialize state and cost variables
x = x0;                  % State variable
u_mpc_trajectory = [];   % Store control actions
states_trajectory = x0;  % Store states
cost_total = 0;          % Total accumulated cost

% Simulation loop
for k = 1:T
    
    future_r = r_extended(:,k:k+horizon);   % Reference trajectory over horizon

    % Measure computation time for controllerSimple
    tic;
    u_mpc = controllerTracking(x);
    cpu_time = toc;

%     % Apply penalty if computation time is above 25ms
%     if cpu_time > 0.025
%         penalty = 10^(200 * (cpu_time - 0.025));
%     else
%         penalty = 0;
%     end
% 
%     % If computation time exceeds 30ms, discard the computation and use last control
%     if cpu_time > 0.03
%         u_mpc = u_mpc_last;
%         fprintf('Computation discarded at time %.4fs due to high CPU time (%.4fs).\n', k * T_s, cpu_time);
%     else
%         u_mpc_last = u_mpc; % Update last valid control if within time limit
%     end

    % Update state using control input
    v = u_mpc(1);
    omega = u_mpc(2);
    x = x + T_s * [cos(x(3)), 0; sin(x(3)), 0; 0, 1] * u_mpc;

    % Store trajectories
    u_mpc_trajectory = [u_mpc_trajectory, u_mpc];
    states_trajectory = [states_trajectory, x];

    % Update previous velocity variables
    v_prev = v;
    omega_prev = omega;

    % Calculate the cost for this step
    state_error = norm(x(1:2) - ref(1:2))^2;
    control_effort = norm(u_mpc)^2;
    cost_k = 10 * state_error + control_effort; %+ penalty;

    % Accumulate total cost
    cost_total = cost_total + cost_k;
    disp(k)
end

fprintf('Total cost accumulated over the simulation: %.2f\n', cost_total);

% Plot the results
figure;
subplot(2,1,1);
plot(states_trajectory(1, :), states_trajectory(2, :), 'b-o'); % Path of the robot
hold on;
plot(ref(1,:), ref(2,:), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % Reference point
xlabel('x (m)');
ylabel('y (m)');
title('Robot Trajectory with controllerSimple');
grid on;

subplot(2,1,2);
plot((1:length(u_mpc_trajectory)) * T_s, u_mpc_trajectory(1, :), '-g', 'LineWidth', 1.5); % Linear velocity
hold on;
plot((1:length(u_mpc_trajectory)) * T_s, u_mpc_trajectory(2, :), '-m', 'LineWidth', 1.5); % Angular velocity
xlabel('Time (s)');
ylabel('Control inputs');
legend('Linear velocity', 'Angular velocity');
title('Control Inputs Over Time');
grid on;
