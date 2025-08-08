function [u_mpc] = controllerTracking(x0)
    % Persistent variables to retain initialization across calls
    persistent i r_extended controller u_mpc_prev horizon

    

    % Initialize persistent variables on first call
    if isempty(controller)
        
         % System definitions
        b = 1;           % Distance between wheels
        v_max = 10;      % Maximum speed (m/s)
        a_max = 0.5;     % Maximum acceleration (m/s^2)
    %     alpha_max = 4;   % Maximum angular acceleration (rad/s^2)
        T_s = 0.1;       % Sampling time
        horizon = 10;    % Prediction horizon for MPC
        
        i = 1;  % Starting index
        % Simulation time
        t = 0:T_s:20 * pi; 

        % Reference definition
        ref = [5 * sin(t / 5); 5 * cos(t / 10); zeros(1, length(t))];

        % Extend the reference
        r_extended = [ref, ref(:, end * ones(1, horizon))];

        % Initialize previous control inputs
        u_mpc_prev = [0; 0];

        % State and control variables
        nx = 3; % Number of states
        nu = 2; % Number of inputs

        x = sdpvar(repmat(nx, 1, horizon + 1), repmat(1, 1, horizon + 1));
        vr = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));
        vl = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));
        u = sdpvar(repmat(nu, 1, horizon), repmat(1, 1, horizon));
        a = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));
%         alpha = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));
        
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

            % Angular acceleration constraints
%             alpha{k} = (omega - u_prev{k}(2)) / T_s;
%             constraints = [constraints, -alpha_max <= alpha{k} <= alpha_max];

            % Store previous control inputs
            constraints = [constraints, u_prev{k+1} == u{k}];
        end

        % Define the optimizer
        parameters_in = {x{1}, [r{:}], u_prev{1}};
        solutions_out = {[u{:}]};
        options = sdpsettings('verbose', 1, 'debug', 1);
        controller = optimizer(constraints, objective, options, parameters_in, solutions_out);
    end

    % Get the reference trajectory for the current horizon
    future_r = r_extended(:, i:i + horizon);

    % Solve the optimization problem
    inputs = {x0, future_r, u_mpc_prev};
    [solutions, diagnostics] = controller{inputs};

    % Check for infeasibility
    if diagnostics == 1
        error('The problem is infeasible');
    end

    % Extract control input
    u_mpc = solutions(:, 1);

    % Update previous control inputs
    u_mpc_prev = u_mpc;

    % Increment the reference index
    if i < size (r_extended-horizon,2)
    i = i + 1;
end
