function [u_mpc] = controllerSimple(x0)
    % Persistent variables to retain initialization across calls
    persistent controller horizon b v_max a_max alpha_max T_s ref Q R u_mpc_prev

    % Check if persistent variables are already initialized
    if isempty(controller)
        % Initialize persistent variables
        b = 1;                % Distance between wheels
        v_max = 10;           % Maximum speed (m/s)
        a_max = 1;            % Maximum acceleration (m/s^2)
        alpha_max = 4;      % Maximum angular acceleration (rad/s^2)
        T_s = 0.1;            % Sampling time
        horizon = 2;          % Prediction horizon for MPC
        ref = [10; 10; 0];    % Target reference position
        Q = diag([10, 10, 0]); % State weights
        R = diag([8, 1]);      % Control input weights

        % Initialize `u_mpc_prev` to zero
        u_mpc_prev = [0; 0];

        % Define YALMIP variables
        nx = 3; % Number of states
        nu = 2; % Number of inputs

        x  = sdpvar(repmat(nx, 1, horizon + 1), repmat(1, 1, horizon + 1)); % States as cell array
        vr = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));           % Right wheel velocity
        vl = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));           % Left wheel velocity
        u = sdpvar(repmat(nu, 1, horizon), repmat(1, 1, horizon));           % Control inputs
        a = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));            % Linear acceleration
%         alpha = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));        % Angular acceleration
        u_prev = sdpvar(repmat(nu, 1, horizon+1), repmat(1, 1, horizon+1));           % Control inputs previous 


        % Initialize objective and constraints
        objective = 0;
        constraints = [];

        % Define the MPC optimization problem
        for k = 1:horizon
            % Linear and angular speed inputs
            v = 0.5 * (vr{k} + vl{k});
            omega = (vr{k} - vl{k}) / b;
            u{k} = [v; omega];

            % Tracking error cost
            objective = objective + (x{k+1} - ref)' * Q * (x{k+1} - ref) + u{k}' * R * u{k};

            % Discretized dynamics
            x_next = x{k}(1) + T_s * v * cos(x{k}(3));
            y_next = x{k}(2) + T_s * v * sin(x{k}(3));
            theta_next = x{k}(3) + T_s * omega;

            % System dynamics constraints
            constraints = [constraints, x{k+1}(1) == x_next];
            constraints = [constraints, x{k+1}(2) == y_next];
            constraints = [constraints, x{k+1}(3) == theta_next];
%             constraints = [constraints, u_prev{k+1}(1) == v];
%             constraints = [constraints, u_prev{k+1}(2) == omega];
            % Store previous control inputs
            constraints = [constraints, u_prev{k+1} == u{k}];

            % Control input constraints
            constraints = [constraints, -v_max <= v <= v_max];

            % Linear acceleration constraint
            a{k} = (v - u_prev{k}(1)) / T_s;
            constraints = [constraints, -a_max <= a{k} <= a_max];

            % Angular acceleration constraint
%             alpha{k} = (omega - u_prev{k}(2)) / T_s;
%             constraints = [constraints, -alpha_max <= alpha{k} <= alpha_max];
        end

        % Define the optimizer
        parameters_in = {x{1}, u_prev{1}};
        solutions_out = {[u{:}]};
        options = sdpsettings('verbose', 0);
        controller = optimizer(constraints, objective, options, parameters_in, solutions_out);
    end

    % Solve the MPC problem
    inputs = {x0, u_mpc_prev};
    [solutions, diagnostics] = controller{inputs};

    % Check for infeasibility
    if diagnostics == 1
        error('The problem is infeasible');
    end

    % Extract control input for the current time step
    u_mpc = solutions(:, 1);

    % Update persistent variable with the current control input
    u_mpc_prev = u_mpc;
end
