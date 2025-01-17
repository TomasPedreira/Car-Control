%% CPCS Project I: Question 1
clearvars
close all
clc

% Code Setup 
monte_carlo = 10; % 1 Iteration Takes a LOT of time!
avg_time = zeros(monte_carlo, 1);

%% Question 1 with MPC
Ts = 0.005;

% Define Model Variables
N = 4; % Nmin = 3;

% Weights
Q = diag([50, 50, 0]);
R = diag([1, 1]);

x0 = [0;0;0]; % Initial State
ref = [10;10;0]; % Reference

% Array Size
nu = 2;
nx = 3;

for MC = 1:monte_carlo
    % MPC SDP Variables (from Yalmip)
    u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
    x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
    prevVel = sdpvar(1,1);
    
    % Simulation variables
    T = 3000;
    utraj = zeros(nu,T);
    xtraj = zeros(nx, T+1);
    xtraj(:,1) = x0;
    xtraj(:,2) = x0;
    
    t = zeros(T-1,1);
    
    % Initialize Cost Function + Constraint Array
    J = 0;
    F = [];
    
    % Horizon: N=1
    vel = 0.5*(u{1}(1)+u{1}(2));
    J = J + (x{1} - ref)'*Q*(x{1} - ref) + u{1}'*R*u{1};
    F = [F, [x{2}==x{1}+ssCar(x{1},u{1})*Ts, vel <= 5, vel-prevVel <= 1*Ts]];
    % Horizon: N>1
    for index = 2:N
        vel = 0.5*(u{index}(1)+u{index}(2)); 
        J = J + (x{index} - ref)'*Q*(x{index} - ref) + u{index}'*R*u{index};
        F = [F, [x{index+1}==x{index}+ssCar(x{index},u{index})*Ts, vel <= 5, vel-0.5*(u{index-1}(1)+u{index-1}(2)) <= 1*Ts]];
    end
    controller = optimizer(F, J, sdpsettings('solver','ipopt','verbose', 0), {x{1} prevVel}, {[u{:}]});

    for k = 2:T
        tic
        u = controller{{x{1} == xtraj(:,k), prevVel == 0.5*(utraj(1,k-1)+utraj(2,k-1))}};  
        t(k) = toc;
        utraj(:,k) = u(:,1);
        [~, xOde] = ode45(@(t,y) ssCar(y, utraj(:,k)), [0 Ts], xtraj(:, k));
        xtraj(:,k+1) = xOde(end,:);
    end
    avg_time(MC) = mean(t(2:end)) * 1e3;

    disp(['Iteration ', num2str(MC), ':']);
    disp(['Maximum value of t: ' num2str(max(t(2:end))*1e3) ' ms']);
    disp(['Minimum value of t: ' num2str(min(t(2:end))*1e3) ' ms']);
    disp(['Average value of t: ' num2str(avg_time(MC)) ' ms']);

    yalmip('clear')
end

overall_avg_time = mean(avg_time);
disp('----------');
disp(['Overall Average Time across ', num2str(monte_carlo), ' Iterations: ', num2str(overall_avg_time), ' ms']);

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

%% Dynamic Plot
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
lineColorB = lineColors(3,:);
markerColorB = markerColors(3,:);

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

% save('controllerSimpleMPC_data_PCroberto.mat', 'xtraj', 'ref', 'overall_avg_time', 'MC');