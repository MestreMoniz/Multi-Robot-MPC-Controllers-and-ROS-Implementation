clear;
close all;
clear controllerCooperative
% Simulation Parameters
T_s = 0.1;             % Sampling time

% Initial positions
x_f1 = [5; 5; 0];   % Initial state of Follower 1 [x; y; theta]
x_f2 = [2; -2; 0];   % Initial state of Follower 2 [x; y; theta]

omega = pi / 20;  % Velocidade angular
        r = 2;            % Raio do círculo
        v_central = 2;    % Velocidade do veículo central

        % Tempo de simulação
        t_total = 80;     % Tempo total (segundos)
        t = 0:T_s:t_total; % Vetor de tempo
        T =length(t);
        
        horizon = 10; 



% Posição inicial do target
        p_central_ini = [-5; 0]; 
        
        % Movimento do veículo central (centro do círculo)
        p_central = [p_central_ini(1) + v_central * t; zeros(1, length(t))];

        % Movimento circular dos robôs
        theta = omega * t; % Ângulo ao longo do tempo
        p1 = [p_central(1, :) + r * cos(theta);
              p_central(2, :) + r * sin(theta);
              zeros(1, T)];
        p2 = [p_central(1, :) + r * cos(theta + pi);
              p_central(2, :) + r * sin(theta + pi);
              zeros(1, T)];

        % Extend the reference
        p1_extended = [p1, p1(:, end * ones(1, horizon))];
        p2_extended = [p2, p2(:, end * ones(1, horizon))];



% Preallocate state histories
x_f1_history = zeros(3, T+1);
x_f2_history = zeros(3, T+1);
x_f1_history(:,1)= x_f1;
x_f2_history(:,1) = x_f2;


% Prepare figure
figure;
hold on;
grid on;
follower1_plot = plot(x_f1(1), x_f1(2), 'b-o', 'DisplayName', 'Follower 1');
follower2_plot = plot(x_f2(1), x_f2(2), 'r-o', 'DisplayName', 'Follower 2');
reference_plot = plot(p_central(1, 1), p_central(2, 1), 'k--', 'DisplayName', 'Reference');

xlabel('X Position');
ylabel('Y Position');
legend;
title('Real-time Trajectories of Followers and Reference');
axis equal;

% Simulation Loop
for i = 1:T-1
    
    % Stack follower positions for controller input
    x_stack = [x_f1_history(:, i);  x_f2_history(:, i)];
    
    % Get control inputs from the cooperative controller
    u_mpc = controllerMovie(x_stack); % Returns [v1; omega1; v2; omega2]
    u_mpc_1 = u_mpc(1:2); % [v; omega] for Follower 1
    u_mpc_2 = u_mpc(3:4); % [v; omega] for Follower 2
    
    % Update states using discrete dynamics
    x_f1_history(:, i+1) = x_f1_history(:, i) + T_s * [cos(x_f1_history(3, i)), 0; sin(x_f1_history(3, i)), 0; 0, 1] * u_mpc_1;
     x_f2_history(:, i+1) =  x_f2_history(:, i) + T_s *[cos(x_f2_history(3, i)), 0; sin(x_f2_history(3, i)), 0; 0, 1] * u_mpc_2;
     
      % Update plots
    set(follower1_plot, 'XData', x_f1_history(1, 1:i+1), 'YData', x_f1_history(2, 1:i+1));
    set(follower2_plot, 'XData', x_f2_history(1, 1:i+1), 'YData', x_f2_history(2, 1:i+1));
    set(reference_plot, 'XData', p_central(1, 1:i+1), 'YData', p_central(2, 1:i+1));

    
    drawnow; % Refresh the figure
    

end

