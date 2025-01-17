%% CPCS Project I
clearvars
close all
clc

% Clear Persistent Variables
clear controllerSimple
clear controllerTracking
clear controllerCooperative
clear controllerMovie

% Code Setup 
question = 1; % question = {1,2,3,4}
plot_tclock = 1;
dynamic_plot = 1;

monte_carlo = 10;
avg_time = zeros(monte_carlo, 1);

%% Dyamic Plot Colors
alphaValue = 0.5;

lineColors = [
    150 192 150; %200 255 200;
    200 128 255;
    150 192 255;
    150 192 150;
    255 192 150;
    255 150 150;
    ] / 255;

markerColors = [
    0.47 0.67 0.19;  % Green
    0.49 0.18 0.56;  % Purple
    0.00 0.45 0.70;  % Blue
    0.30 0.75 0.93;  % Light Blue
    0.93 0.69 0.13;  % Yellow
    0.85 0.33 0.10;  % Red
    ];

% Red
lineColorA = lineColors(6,:);
markerColorA = markerColors(6,:);

% Blue
lineColorB = lineColors(3,:);
markerColorB = markerColors(3,:);

% Purple
lineColorC = lineColors(2,:);
markerColorC = markerColors(2,:);

%% Question 1
if question == 1
    Ts = 0.005;
    T = 1500;
    ref = [10;10];
    
    % Array Size
    nu = 2;
    nx = 3;
    x0 = [0;0;0];
    
    for MC = 1:monte_carlo
        % Simulation Variables
        utraj = zeros(nu,T);
        xtraj = zeros(nx,T+1);
        xtraj(:,1) = x0;
        xtraj(:,2) = x0;
        
        t = zeros(T-1,1);
        
        for k = 2:T
            tic
            utraj(:,k) = controllerSimple(xtraj(:,k));
            t(k) = toc;
            if t(k)*1e3 > 30
               utraj(:,k) = utraj(:,k-1); 
            end
            [~, xOde] = ode45(@(t,y) ssCar(y, utraj(:,k)), [0 Ts], xtraj(:, k));
            xtraj(:,k+1) = xOde(end,:);
            % disp(['k = ', num2str(k)]);
        end
        avg_time(MC) = mean(t(2:end)) * 1e3;
    
        disp(['Iteration ', num2str(MC), ':']);
        disp(['Maximum value of t: ' num2str(max(t(2:end))*1e3) ' ms']);
        disp(['Minimum value of t: ' num2str(min(t(2:end))*1e3) ' ms']);
        disp(['Average value of t: ' num2str(avg_time(MC)) ' ms']);

        clear controllerSimple
    end

    overall_avg_time = mean(avg_time);
    disp('----------');
    disp(['Overall Average Time across ', num2str(monte_carlo), ' Iterations: ', num2str(overall_avg_time), ' ms']);

    if plot_tclock == 1
        figure('Position', [565, 250, 660, 520]);
        hold on
        plot(t(3:end));
        yline(0.03, '--', 'Critical');
        yline(0.025, '--', 'Penalty');
        ylabel("Clock~Time~(s)", 'FontSize', 11)
        xlabel("Sample", 'FontSize', 11)
        title("$\mathbf{Clock~Time~per~Iteration}$", 'FontSize', 12)
        axis padded
        grid on
        grid minor
        hold off
    end

    if dynamic_plot == 1
        fig = figure('Position', [565, 250, 660, 520]);
        title("$\mathbf{Robot~Car~Trajectory}$", 'FontSize', 12);
        xlabel("X~Axis~(m)", 'FontSize', 11)
        ylabel("Y~Axis~(m)", 'FontSize', 11)
        axis padded
        hold on
        grid on
        grid minor
        box on
        
        % Plot Reference
        scatter(ref(1,:), ref(2,:), 'r+', linewidth=1.4)
        
        % Continous Model
        PathA = animatedline('Color', [lineColorB, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agentA = scatter(xtraj(1, 1), xtraj(2, 1), 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorB/1.3, 'MarkerFaceColor', markerColorB, 'DisplayName', 'ASV');
        
        for index = 1:length(xtraj)-1
            addpoints(PathA, xtraj(1,index), xtraj(2,index));
            set(agentA, 'XData', xtraj(1,index), 'YData', xtraj(2,index));

            drawnow;
        end
        hold off
    end
    clear controllerSimple
end

%% Question 2
if question == 2
    Ts = 0.005;
    T = round((20*pi)/Ts);
    ref = [5*sin((1:T+-1)*Ts/5); 5*cos((1:T+-1)*Ts/10)]; % Reference
    
    % Array Size
    nu = 2;
    nx = 3;
    x0 = [0;0;0];
    
    for MC = 1:monte_carlo
        % Simulation Variables
        utraj = zeros(nu,T);
        xtraj = zeros(nx,T+1);
        xtraj(:,1) = x0;
        xtraj(:,2) = x0;
        
        t = zeros(T-1,1);
        
        for k = 2:T
            tic
            utraj(:,k) = controllerTracking(xtraj(:,k));
            t(k) = toc;
            if t(k)*1e3 > 30
               utraj(:,k) = utraj(:,k-1); 
            end
            [~, xOde] = ode45(@(t,y) ssCar(y, utraj(:,k)), [0 Ts], xtraj(:, k));
            xtraj(:,k+1) = xOde(end,:);
            % disp(['k = ', num2str(k)]);
        end
        avg_time(MC) = mean(t(2:end)) * 1e3;
    
        disp(['Iteration ', num2str(MC), ':']);
        disp(['Maximum value of t: ' num2str(max(t(2:end))*1e3) ' ms']);
        disp(['Minimum value of t: ' num2str(min(t(2:end))*1e3) ' ms']);
        disp(['Average value of t: ' num2str(avg_time(MC)) ' ms']);

        clear controllerTracking
    end

    overall_avg_time = mean(avg_time);
    disp('----------');
    disp(['Overall Average Time across ', num2str(monte_carlo), ' Iterations: ', num2str(overall_avg_time), ' ms']);

    if plot_tclock == 1
        figure('Position', [565, 250, 660, 520]);
        hold on
        plot(t(3:end));
        yline(0.03, '--', 'Critical');
        yline(0.025, '--', 'Penalty');
        ylabel("Clock~Time~(s)", 'FontSize', 11)
        xlabel("Sample", 'FontSize', 11)
        title("$\mathbf{Clock~Time~per~Iteration}$", 'FontSize', 12)
        axis padded
        grid on
        grid minor
        hold off
    end

    if dynamic_plot == 1
        fig = figure('Position', [565, 250, 660, 520]);
        title("$\mathbf{Robot~Car~Trajectory}$", 'FontSize', 12);
        xlabel("X~Axis~(m)", 'FontSize', 11)
        ylabel("Y~Axis~(m)", 'FontSize', 11)
        axis padded
        hold on
        grid on
        grid minor
        box on
        
        % Plot Reference
        plot(ref(1,:), ref(2,:), 'r--', linewidth=1.4)
        
        % Continous Model
        PathA = animatedline('Color', [lineColorB, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agentA = scatter(xtraj(1, 1), xtraj(2, 1), 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorB/1.3, 'MarkerFaceColor', markerColorB, 'DisplayName', 'ASV');
        
        for index = 1:length(xtraj)-1
            addpoints(PathA, xtraj(1,index), xtraj(2,index));
            set(agentA, 'XData', xtraj(1,index), 'YData', xtraj(2,index));

            drawnow;
        end
        hold off
    end
    clear controllerTracking
end

%% Question 3
if question == 3
    Ts = 0.005;
    time = Ts:Ts:10;
    T = length(time);
    
    % Array Size
    nu = 4;
    nx = 6;

    % Car Initial Positions
    extraCar = [-5;0;0];
    firstCar = [10;-1.5;0];
    secondCar = [90;-0.25;0];
    
    % Target Vehicle Reference
    extraCarRef = [-5 + 10 .* time; -1 * ones(size(time))];

    for MC = 1:monte_carlo
        % Simulation Variables
        utraj = 2*ones(nu,T);
        xtraj = zeros(nx,T+1);
        xtraj(1:3,1) = firstCar;
        xtraj(4:6,1) = secondCar;
        xtraj(1:3,2) = firstCar;
        xtraj(4:6,2) = secondCar;
        
        t = zeros(T-1,1);
        
        for k = 2:T
            tic
            utraj(:,k) = controllerCooperative(xtraj(:,k));
            t(k) = toc;
            if t(k)*1e3 > 30
               utraj(:,k) = utraj(:,k-1); 
            end
            [~, xOde] = ode45(@(t,y) ssCar(y, utraj(1:2,k)), [0 Ts], xtraj(1:3, k));
            xtraj(1:3,k+1) = xOde(end,:);
            [~, xOde] = ode45(@(t,y) ssCar(y, utraj(3:4,k)), [0 Ts], xtraj(4:6, k));
            xtraj(4:6,k+1) = xOde(end,:);
            % disp(['k = ', num2str(k)]);
        end
        avg_time(MC) = mean(t(2:end)) * 1e3;
    
        disp(['Iteration ', num2str(MC), ':']);
        disp(['Maximum value of t: ' num2str(max(t(2:end))*1e3) ' ms']);
        disp(['Minimum value of t: ' num2str(min(t(2:end))*1e3) ' ms']);
        disp(['Average value of t: ' num2str(avg_time(MC)) ' ms']);

        clear controllerCooperative
    end

    overall_avg_time = mean(avg_time);
    disp('----------');
    disp(['Overall Average Time across ', num2str(monte_carlo), ' Iterations: ', num2str(overall_avg_time), ' ms']);

    if plot_tclock == 1
        figure('Position', [565, 250, 660, 520]);
        hold on
        plot(t(3:end));
        yline(0.03, '--', 'Critical');
        yline(0.025, '--', 'Penalty');
        ylabel("Clock~Time~(s)", 'FontSize', 11)
        xlabel("Sample", 'FontSize', 11)
        title("$\mathbf{Clock~Time~per~Iteration}$", 'FontSize', 12)
        axis padded
        grid on
        grid minor
        hold off
    end
    
    if dynamic_plot == 1
        fig = figure('Position', [565, 250, 660, 520]);
        title("$\mathbf{Robot~Car~Trajectory}$", 'FontSize', 12);
        xlabel("X~Axis~(m)", 'FontSize', 11)
        ylabel("Y~Axis~(m)", 'FontSize', 11)
        axis padded
        hold on
        grid on
        grid minor
        box on
        
        plot(extraCarRef(1,:), extraCarRef(2,:), 'k--', linewidth=1.4)

        % Continous Model
        PathA = animatedline('Color', [lineColorA, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agentA = scatter(xtraj(1, 1), xtraj(2, 1), 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorA/1.3, 'MarkerFaceColor', markerColorA, 'DisplayName', 'ASV');
        
        PathB = animatedline('Color', [lineColorB, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agentB = scatter(xtraj(4, 1), xtraj(5, 1), 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorB/1.3, 'MarkerFaceColor', markerColorB, 'DisplayName', 'ASV');
        
        PathD = animatedline('Color', [lineColorC, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agentD = scatter(extraCarRef(1, 1), extraCarRef(2, 1)+1, 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorC/1.3, 'MarkerFaceColor', markerColorC, 'DisplayName', 'ASV');
        
        for index = 1:length(xtraj)-1
            addpoints(PathA, xtraj(1,index), xtraj(2,index));
            set(agentA, 'XData', xtraj(1,index), 'YData', xtraj(2,index));
        
            addpoints(PathB, xtraj(4,index), xtraj(5,index));
            set(agentB, 'XData', xtraj(4,index), 'YData', xtraj(5,index)); 
        
            addpoints(PathD, extraCarRef(1,index), extraCarRef(2,index)+1);
            set(agentD, 'XData', extraCarRef(1,index), 'YData', extraCarRef(2,index)+1);
        
            drawnow;
        end
        hold off
    end
    clear controllerCooperative
end

%% Question 4
if question == 4
    Ts = 0.005;
    time = Ts:Ts:80;
    T = length(time);
    
    % Array Size
    nu = 4;
    nx = 6;

    % Cars Initial Positions
    extraCar = [-5;0;0];
    firstCar = [-3;-2;pi/2];
    secondCar = [-7;2;-pi/2];
    
    % Target Vehicle Reference
    extraCarRef = [-5 + 0.1 .* time; zeros(size(time))];

    % Tracking Car References
    carRef = carReferences();
    ref1 = carRef(1:2, :);
    ref2 = carRef(3:4, :);

    for MC = 1:monte_carlo
        % Simulation Variables
        utraj = 2*ones(nu,T);
        xtraj = zeros(nx,T+1);
        xtraj(1:3,1) = firstCar;
        xtraj(4:6,1) = secondCar;
        xtraj(1:3,2) = firstCar;
        xtraj(4:6,2) = secondCar;
        
        t = zeros(T-1,1);
        
        for k = 2:T
            tic
            utraj(:,k) = controllerMovie(xtraj(:,k));
            t(k) = toc;
            if t(k)*1e3 > 30
               utraj(:,k) = utraj(:,k-1); 
            end
            [~, xOde] = ode45(@(t,y) ssCar(y, utraj(1:2,k)), [0 Ts], xtraj(1:3, k));
            xtraj(1:3,k+1) = xOde(end,:);
            [~, xOde] = ode45(@(t,y) ssCar(y, utraj(3:4,k)), [0 Ts], xtraj(4:6, k));
            xtraj(4:6,k+1) = xOde(end,:);
            % disp(['k = ', num2str(k)]);
        end
        avg_time(MC) = mean(t(2:end)) * 1e3;
    
        disp(['Iteration ', num2str(MC), ':']);
        disp(['Maximum value of t: ' num2str(max(t(2:end))*1e3) ' ms']);
        disp(['Minimum value of t: ' num2str(min(t(2:end))*1e3) ' ms']);
        disp(['Average value of t: ' num2str(avg_time(MC)) ' ms']);

        clear controllerMovie
    end

    overall_avg_time = mean(avg_time);
    disp('----------');
    disp(['Overall Average Time across ', num2str(monte_carlo), ' Iterations: ', num2str(overall_avg_time), ' ms']);

    if plot_tclock == 1
        figure('Position', [565, 250, 660, 520]);
        hold on
        plot(t(3:end));
        yline(0.03, '--', 'Critical');
        yline(0.025, '--', 'Penalty');
        ylabel("Clock~Time~(s)", 'FontSize', 11)
        xlabel("Sample", 'FontSize', 11)
        title("$\mathbf{Clock~Time~per~Iteration}$", 'FontSize', 12)
        axis padded
        grid on
        grid minor
        hold off
    end

    if dynamic_plot == 1
        fig = figure('Position', [565, 250, 660, 520]);
        title("$\mathbf{Robot~Car~Trajectory}$", 'FontSize', 12);
        xlabel("X~Axis~(m)", 'FontSize', 11)
        ylabel("Y~Axis~(m)", 'FontSize', 11)
        axis padded
        hold on
        grid on
        grid minor
        box on
    
        % Plot References
        plot(ref1(1,:), ref1(2,:), 'r--', linewidth=1.4)
        plot(ref2(1,:), ref2(2,:), 'b--', linewidth=1.4)
        
        % Continous Model
        Path1 = animatedline('Color', [lineColorA, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agent1 = scatter(xtraj(1, 1), xtraj(2, 1), 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorA/1.3, 'MarkerFaceColor', markerColorA, 'DisplayName', 'ASV');
        
        Path2 = animatedline('Color', [lineColorB, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agent2 = scatter(xtraj(4, 1), xtraj(5, 1), 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorB/1.3, 'MarkerFaceColor', markerColorB, 'DisplayName', 'ASV');
        
        Path3 = animatedline('Color', [lineColorC, alphaValue], 'HandleVisibility', 'off', 'LineWidth', 2);
        agent3 = scatter(extraCarRef(1, 1), extraCarRef(2, 1), 'LineWidth', 1, ...
            'MarkerEdgeColor', markerColorC/1.3, 'MarkerFaceColor', markerColorC, 'DisplayName', 'ASV');
        
        for index = 1:length(xtraj)-1
            addpoints(Path1, xtraj(1,index), xtraj(2,index));
            set(agent1, 'XData', xtraj(1,index), 'YData', xtraj(2,index));
        
            addpoints(Path2, xtraj(4,index), xtraj(5,index));
            set(agent2, 'XData', xtraj(4,index), 'YData', xtraj(5,index));
        
            addpoints(Path3, extraCarRef(1,index), extraCarRef(2,index));
            set(agent3, 'XData', extraCarRef(1,index), 'YData', extraCarRef(2,index));
    
            drawnow;
        end
        hold off
    end
    clear controllerMovie
end