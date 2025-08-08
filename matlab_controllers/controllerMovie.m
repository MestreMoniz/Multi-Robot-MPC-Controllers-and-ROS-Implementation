function [u_mpc] = controllerMovie(x0)

persistent controller p1_extended p2_extended i u_mpc_prev_1 u_mpc_prev_2 horizon
 
 % Initialize persistent variables on first call
    if isempty(controller)
         b = 1;           % Distance between wheels
        v_max = 2;      % Maximum speed (m/s)
        a_max = 1;     % Maximum acceleration (m/s^2)
        T_s = 0.1;       % Sampling time
        horizon = 10;    % Prediction horizon for MPC
        omega = pi / 20;  % Velocidade angular
        r = 2;            % Raio do círculo
        v_central = 2;    % Velocidade do veículo central

        % Tempo de simulação
        t_total = 80;     % Tempo total (segundos)
        t = 0:T_s:t_total; % Vetor de tempo
        T =length(t);
        
        i = 1;  % Starting index
        
        % Movimento do veículo central (centro do círculo)
        p_central = [v_central * t; zeros(1, length(t))];

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
        
        % Initialize previous control inputs
        u_mpc_prev_1 = [0; 0];
        u_mpc_prev_2 = [0; 0];


        % State and control variables
        nx = 3; % Number of states
        nu = 2; % Number of inputs

        x = sdpvar(repmat(nx, 1, horizon + 1), repmat(1, 1, horizon + 1));
        vr = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));
        vl = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));
        u = sdpvar(repmat(nu, 1, horizon), repmat(1, 1, horizon));
        a = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));
        
        % Use u_prev instead of v_prev and omega_prev
        u_prev = sdpvar(repmat(nu, 1, horizon + 1), repmat(1, 1, horizon + 1));
        
        r = sdpvar(repmat(nx, 1, horizon + 1), repmat(1, 1, horizon + 1));

        % Cost weights
        Q = diag([10, 10, 0]); % Tracking error
        R = diag([1, 1]);      % Control effort

        % Initialize objective and constraints
        objective = 0;
        constraints = [];

        for k = 1:horizon
            % Compute linear and angular velocities
            v = 0.5 * (vr{k} + vl{k});
            omega = (vr{k} - vl{k}) / b;
            u{k} = [v; omega];

            % Cost function
            objective = objective + (x{k+1} - r{k})' * Q * (x{k+1} - r{k}) + u{k}' * R * u{k};

            % Dynamics constraints
            x_next = x{k}(1) + T_s * v * cos(x{k}(3));
            y_next = x{k}(2) + T_s * v * sin(x{k}(3));
            theta_next = x{k}(3) + T_s * omega;

            constraints = [constraints, x{k+1}(1) == x_next];
            constraints = [constraints, x{k+1}(2) == y_next];
            constraints = [constraints, x{k+1}(3) == theta_next];

            % Control constraints
            constraints = [constraints, -v_max <= v <= v_max];

            % Acceleration constraints
            a{k} = (v - u_prev{k}(1)) / T_s;
            constraints = [constraints, -a_max <= a{k} <= a_max];



            % Store previous control inputs
            constraints = [constraints, u_prev{k+1} == u{k}];
        end

        % Define the optimizer
        parameters_in = {x{1}, [r{:}], u_prev{1}};
        solutions_out = {[u{:}]};
        options = sdpsettings('verbose', 0, 'debug', 0);
        controller = optimizer(constraints, objective, options, parameters_in, solutions_out);
    end
    
     follower_1_pos = x0(1:3);
     follower_2_pos = x0(4:6);
    
    
      % Get the reference trajectory for the current horizon
    future_r1 = p1_extended(:, i:i + horizon);
    % Get the reference trajectory for the current horizon
    future_r2 = p2_extended(:, i:i + horizon);

    % Solve the optimization problem
    inputs_1 = {follower_1_pos, future_r1, u_mpc_prev_1};
    inputs_2 = {follower_2_pos, future_r2, u_mpc_prev_2};

    
    % Resolver MPC para os dois robôs
    [solutions_1, diagnostics_1] = controller{inputs_1};
    [solutions_2, diagnostics_2] = controller{inputs_2};
    

    % Verificar se o problema é inviável
    if diagnostics_1 == 1 || diagnostics_2 == 1
        error('The problem is infeasible');
    end
    
    
    % Extrair entradas de controle e previsões de estados
    u_mpc_prev_1 = solutions_1(:, 1);  % Controle do robô 1
    u_mpc_prev_2 = solutions_2(:, 1);  % Controle do robô 2
    u_mpc = [u_mpc_prev_1;u_mpc_prev_2];
    
    i = i +1;
end

