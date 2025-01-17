%% CPCS Project I - Ex4 Ref Testbench 
clearvars
close all
clc

carRef = carReferences();

% Extract Car References
ref1 = carRef(1:2, :);
ref2 = carRef(3:4, :);

% Plot
fig = figure('Position', [565, 250, 660, 520]);
title("$\mathbf{Robot~Car~References}$", 'FontSize', 12)
xlabel("X~Axis~(m)", 'FontSize', 11)
ylabel("Y~Axis~(m)", 'FontSize', 11)
axis padded
hold on
grid on
grid minor
box on
plot(ref1(1,:), ref1(2,:), 'r', 'LineWidth', 1.5)
plot(ref2(1,:), ref2(2,:), 'b', 'LineWidth', 1.5)
legend('Car 1', 'Car 2');