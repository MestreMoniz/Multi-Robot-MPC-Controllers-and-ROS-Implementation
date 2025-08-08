function [u_mpc] = controllerCooperative(x0)
    % ===========================================
    % CONTROLLER COOPERATIVE FUNCTION
    % ===========================================
    % Inputs:
    %   x0 - Current states of the system [follower_1; follower_2]
    % Outputs:
    %   u_mpc - Control inputs for the followers
    %
    % Persistent variables:
    %   - controller: MPC optimizer instance
    %   - has_intersection_1/2: Intersection status for followers
    %   - selected_ref_1/2: Selected references for followers
    %   - u_mpc_prev_1/2: Previous control inputs
    %   - ref: Reference trajectory
    %   - i: Current time step index
    %   - target_vel_max, follower_v_max: Speed constraints
    %   - T_s: Sampling time
    %   - max_intersections: Maximum intersections to evaluate
    % ===========================================

    % Persistent variables
    persistent controller has_intersection_1 has_intersection_2 selected_ref_1 selected_ref_2 ref_estimated_1 ref_estimated_2 intersection_indices_1 intersection_indices_2
    persistent u_mpc_prev_1 u_mpc_prev_2 ref i target_vel_max follower_v_max T_s max_intersections min_dist_diff

    % Initialize persistent variables
    if isempty(controller)
        % System parameters
        b = 1;                  % Distance between wheels
        follower_v_max = 2;     % Max follower speed (m/s)
        target_vel_max = 10;    % Max target speed (m/s)
        max_intersections = 4;  % Max intersections to evaluate
        T_s = 0.1;              % Sampling time
        horizon = 4;            % MPC prediction horizon
        time = 10;              % Simulation time
        i = 1;                  % Time step index
        min_dist_diff = T_s *target_vel_max ;      % Minimum distance difference to update references
        v_max = 10;
        a_max = 1;
        
        % Initial positions
        target_initial_pos = [-5; 0; 0];          % Target initial position [x; y; theta]
        target_final_pos = target_initial_pos + [target_vel_max * time; 0; 0]; % Compute final position
        
        % Reference trajectory
        points = [target_initial_pos, target_final_pos];
        ref = ReferenceWithPoints(points, T_s, target_vel_max);
        
        % Initialize previous control inputs
        u_mpc_prev_1 = [0; 0];
        u_mpc_prev_2 = [0; 0];
        
        % MPC setup
        nx = 3; % Number of states
        nu = 2; % Number of inputs
        Q = diag([10, 10, 0.1]);   % State weighting matrix
        R = diag([0.1, 0.01]);    % Input weighting matrix
    
    
        % Define optimization variables
        x = sdpvar(repmat(nx, 1, horizon + 1), repmat(1, 1, horizon + 1)); % States
        vr = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));         % Right wheel velocity
        vl = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));         % Left wheel velocity
        u = sdpvar(repmat(nu, 1, horizon), repmat(1, 1, horizon));         % Control inputs
        a = sdpvar(repmat(1, 1, horizon), repmat(1, 1, horizon));          % Accelerations
        u_prev = sdpvar(repmat(nu, 1, horizon + 1), repmat(1, 1, horizon + 1)); % Previous control inputs
        r = sdpvar(repmat(nx,1,1), repmat(1,1,1)); % Reference position
        
        % Objective and constraints
        objective = 0;
        constraints = [];
        
 
    for k = 1:horizon
        
        % Linear and angular speed inputs
        v = 0.5 * (vr{k} + vl{k});
        omega = (vr{k} - vl{k}) / b;
        u{k} = [v; omega];
        
        % Tracking error cost (distance to reference position)
        objective = objective + (x{k+1} - r)' * Q * (x{k+1} - r) + u{k}' * R * u{k};
        
        % Discretized dynamics
        x_next = x{k}(1) + T_s * v * cos(x{k}(3));
        y_next = x{k}(2) + T_s * v * sin(x{k}(3));
        theta_next = x{k}(3) + T_s * omega;
        
        
        % System dynamics constraints
        constraints = [constraints, x{k+1}(1) == x_next];
        constraints = [constraints, x{k+1}(2) == y_next];
        constraints = [constraints, x{k+1}(3) == theta_next];
        % Store previous control inputs
        constraints = [constraints, u_prev{k+1} == u{k}];

        % Control input constraints
        constraints = [constraints, -v_max <= v <= v_max];

        % Linear acceleration constraint
        a{k} = (v - u_prev{k}(1)) / T_s;
        constraints = [constraints, -a_max <= a{k} <= a_max];
    end
    
    % ============================================================
    % DEFINE THE OPTIMIZER
    % ============================================================
    
    parameters_in = {x{1},r, u_prev{1}}; 
    solutions_out = {[u{:}]}; 
    options = sdpsettings('verbose', 0, 'debug', 0);
    controller = optimizer(constraints, objective, options, parameters_in, solutions_out);
    
    % ============================================================
    % UPDATE REFERENCES AND EVALUATE INTERSECTIONS
    % ============================================================
    
    [ref_estimated_1, intersection_indices_1, has_intersection_1, ~, ~, ~, ~] = ...
    estimateIntersectionPoint(ref, ref(:,1), target_vel_max, x0(1:3), follower_v_max, T_s, max_intersections);

    [ref_estimated_2, intersection_indices_2, has_intersection_2, ~, ~, ~, ~] = ...
    estimateIntersectionPoint(ref, ref(:,1), target_vel_max, x0(4:6), follower_v_max, T_s, max_intersections);

    [selected_ref_1, selected_ref_2] = selectBestReferences( ref_estimated_1, has_intersection_1, intersection_indices_1, ref_estimated_2, has_intersection_2, intersection_indices_2); 
end

    follower_1_pos = x0(1:3);
    follower_2_pos = x0(4:6);
    target_pos = ref(:,i);
     % Distâncias do target para os robôs
    dist_1_target = norm(follower_1_pos(1:2) - target_pos(1:2));
    dist_2_target = norm(follower_2_pos(1:2) - target_pos(1:2));
    
    % Caso 1: Apenas o Robo 1 tem uma interseção
    if has_intersection_1 && ~has_intersection_2
        % Verificar se o Robo 1 chegou à referência
        if norm(follower_1_pos(1:2) - selected_ref_1(1:2)) < min_dist_diff            
            % Verificar se o target está próximo (dentro do limite de 2 unidades)
            if dist_1_target >= 0 && dist_1_target <= 1
                % Caso ideal: Target está próximo, mudar foco para segui-lo
                selected_ref_1 = target_pos;
                has_intersection_1 = false; % Desconsiderar interseções
                
                % Recalcular interseções para o Robo 2
                [ref_estimated_2, intersection_indices_2, has_intersection_2, ~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end), target_pos, target_vel_max, follower_2_pos, follower_v_max, T_s, max_intersections);
            end
            % Verificar se o Robo 1 encontra o target no caminho
        elseif dist_1_target <= 1
            selected_ref_1 = target_pos; % Seguir o target diretamente
            has_intersection_1 = false; % Desconsiderar interseções
            
            % Recalcular interseções para o Robo 2
            [ref_estimated_2, intersection_indices_2, has_intersection_2, ~, ~, ~, ~] = ...
                estimateIntersectionPoint(ref(:, i:end),  target_pos, target_vel_max, follower_2_pos, follower_v_max, T_s, max_intersections);
        else
            % Caso 3: O target chegou primeiro à interseção
            if norm(target_pos(1:2) - selected_ref_1(1:2)) < min_dist_diff
                
                % Recalcular interseções para ambos os robôs
                [ref_estimated_1, intersection_indices_1, has_intersection_1, ~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end), target_pos, target_vel_max, follower_1_pos, follower_v_max, T_s, max_intersections);
                
                [ref_estimated_2, intersection_indices_2, has_intersection_2, ~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end),  target_pos, target_vel_max, follower_2_pos, follower_v_max, T_s, max_intersections);
                
                % Atualizar as referências se novas interseções forem encontradas
                if has_intersection_1
                    [selected_ref_1, ~] = ...
                        selectBestReferences(ref_estimated_1, has_intersection_1, intersection_indices_1, ...
                        ref_estimated_2, has_intersection_2, intersection_indices_2);
                else
                    selected_ref_1 = target_pos; % Seguir o target diretamente
                end
            end
        end
        
        % Verificar se há interseções para o Robo 2
        if has_intersection_2
            [selected_ref_1, selected_ref_2] = ...
                selectBestReferences(ref_estimated_1, has_intersection_1, intersection_indices_1, ...
                ref_estimated_2, has_intersection_2, intersection_indices_2);
        else
            % Caso contrário, o Robo 2 segue diretamente o target
            selected_ref_2 = target_pos;
        end
        
        % Caso 2: Apenas o Robo 2 tem uma interseção
    elseif ~has_intersection_1 && has_intersection_2
        % Verificar se o Robo 2 chegou à referência
        if norm(follower_2_pos(1:2) - selected_ref_2(1:2)) < min_dist_diff            
            % Verificar se o target está próximo (dentro do limite de 2 unidades)
            if dist_2_target >= 0 && dist_2_target <= 1
                % Caso ideal: Target está próximo, mudar foco para segui-lo
                selected_ref_2 = target_pos;
                has_intersection_2 = false; % Desconsiderar interseções
                
                % Recalcular interseções para o Robo 1
                [ref_estimated_1, intersection_indices_1, has_intersection_1, ~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end),target_pos, target_vel_max, follower_1_pos, follower_v_max, T_s, max_intersections);
            end
            % Verificar se o Robo 2 encontra o target no caminho
        elseif dist_2_target <= 1
            selected_ref_2 = target_pos; % Seguir o target diretamente
            has_intersection_2 = false; % Desconsiderar interseções
            
            % Recalcular interseções para o Robo 1
            [ref_estimated_1, intersection_indices_1, has_intersection_1, ~, ~, ~, ~] = ...
                estimateIntersectionPoint(ref(:, i:end), target_pos, target_vel_max, follower_1_pos, follower_v_max, T_s, max_intersections);
        else
            % Caso 3: O target chegou primeiro à interseção
            if norm(target_pos(1:2) - selected_ref_2(1:2)) < min_dist_diff
                
                % Recalcular interseções para ambos os robôs
                [ref_estimated_1, intersection_indices_1, has_intersection_1, ~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end),  target_pos, target_vel_max, follower_1_pos, follower_v_max, T_s, max_intersections);
                
                [ref_estimated_2, intersection_indices_2, has_intersection_2, ~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end),  target_pos, target_vel_max, follower_2_pos, follower_v_max, T_s, max_intersections);
                
                % Atualizar as referências se novas interseções forem encontradas
                if has_intersection_2
                    [~, selected_ref_2] = ...
                        selectBestReferences(ref_estimated_1, has_intersection_1, intersection_indices_1, ...
                        ref_estimated_2, has_intersection_2, intersection_indices_2);
                else
                    selected_ref_2 = target_pos; % Seguir o target diretamente
                end
            end
        end
        
        % Verificar se há interseções para o Robo 1
        if has_intersection_1
            [selected_ref_1, selected_ref_2] = ...
                selectBestReferences(ref_estimated_1, has_intersection_1, intersection_indices_1, ...
                ref_estimated_2, has_intersection_2, intersection_indices_2);
        else
            % Caso contrário, o Robo 1 segue diretamente o target
            selected_ref_1 = target_pos;
        end
        
        
        % Caso 3: Ambos os robôs têm pontos de interseção
    elseif has_intersection_1 && has_intersection_2
        % Robo 1 segue a referência escolhida
        if norm(follower_1_pos(1:2) - selected_ref_1(1:2)) < min_dist_diff            
            % Verificar se o target está próximo (dentro do limite de 2 unidades)
            if dist_1_target >= 0 && dist_1_target <= 1
                % Caso ideal: Target está próximo, mudar foco para segui-lo
                selected_ref_1 = target_pos;
                has_intersection_1 = false; % Desconsiderar interseções
              
            end
            % Verificar se o Robo 1 encontra o target no caminho
        elseif dist_1_target <= 1
            selected_ref_1 = target_pos; % Seguir o target diretamente
            has_intersection_1 = false; % Desconsiderar interseções
            
           
        else
            % Caso 3: O target chegou primeiro à interseção
            if norm(target_pos(1:2) - selected_ref_1(1:2)) < min_dist_diff
                
                % Recalcular interseções para o robo 1
                [ref_estimated_1, intersection_indices_1, has_intersection_1, ~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end),  target_pos, target_vel_max, follower_1_pos, follower_v_max, T_s, max_intersections);
                % Atualizar as referências se novas interseções forem encontradas
                if has_intersection_1
                    [selected_ref_1, ~] = ...
                        selectBestReferences(ref_estimated_1, has_intersection_1, intersection_indices_1, ...
                        ref_estimated_2, has_intersection_2, intersection_indices_2);
                else
                    selected_ref_1 = target_pos; % Seguir o target diretamente
                end
            end
        end
        
        % Robo 2 segue a referência escolhida
        if norm(follower_2_pos(1:2) - selected_ref_2(1:2)) < min_dist_diff            
            % Verificar se o target está próximo (dentro do limite de 2 unidades)
            if dist_2_target >= 0 && dist_2_target <= 1
                % Caso ideal: Target está próximo, mudar foco para segui-lo
                selected_ref_2 = target_pos;
                has_intersection_2 = false; % Desconsiderar interseções
                
            end
            % Verificar se o Robo 2 encontra o target no caminho
        elseif dist_2_target <= 1
            selected_ref_2 = target_pos; % Seguir o target diretamente
            has_intersection_2 = false; % Desconsiderar interseções
            
        else
            % Caso 3: O target chegou primeiro à interseção
            if norm(target_pos(1:2) - selected_ref_2(1:2)) < min_dist_diff
                
                % Recalcular interseções para o robo 2
                
                [ref_estimated_2, intersection_indices_2, has_intersection_2,~, ~, ~, ~] = ...
                    estimateIntersectionPoint(ref(:, i:end),  target_pos, target_vel_max, follower_2_pos, follower_v_max, T_s, max_intersections);
                
                % Atualizar as referências se novas interseções forem encontradas
                if has_intersection_2
                    [selected_ref_1, selected_ref_2] = ...
                        selectBestReferences(ref_estimated_1, has_intersection_1, intersection_indices_1, ...
                        ref_estimated_2, has_intersection_2, intersection_indices_2);

                else
                    selected_ref_2 = target_pos; % Seguir o target diretamente
                end
            end
        end
        % Caso 4: Nenhum robô tem interseção, ambos seguem o target
    elseif ~has_intersection_1 && ~has_intersection_2
        selected_ref_1 = target_pos;
        selected_ref_2 = target_pos;
        
    end
    
    
    
    
    % Entradas para o controlador dos dois robôs
    inputs_1 = {follower_1_pos, selected_ref_1,  u_mpc_prev_1 };
    inputs_2 = {follower_2_pos, selected_ref_2, u_mpc_prev_2};
    
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

