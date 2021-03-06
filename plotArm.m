function plotArm(arm_motions, p0, pf, move_str)
% plot continuous motion profiles for each method

fig = figure('Name', 'Arm');
methods = {'PTR', 'MTNB', 'MTN', 'MBP'};

for j=1:4
    q = arm_motions{j};
    % joint configurations
    q1 = q(1, :); q2 = q(2, :); q3 = q(3, :);

    % plot desired trajectory
    subplot(2, 2, j);
    plot([p0(1) pf(1)], [p0(2) pf(2)], 'Color', 'black', 'LineWidth', 4); hold on;

    % continual motion with evenly-spaced steps
    step = floor(size(q, 2) / 10);
    for i = 1:step:size(q, 2)
        axis square; grid on;
        axis([0.0 2.5 -1.5 1.0]); %t2

        % short form for trig functions
        q1i = q1(i); q2i = q2(i); q3i = q3(i);
        c1 = cos(q1i); c12 = cos(q1i + q2i); c123 = cos(q1i + q2i + q3i);
        s1 = sin(q1i); s12 = sin(q1i + q2i); s123 = sin(q1i + q2i + q3i);

        if i == 1
            options = struct('Color', 'blue', 'LineWidth', 4);
        elseif i + step > size(q, 2)
            options = struct('Color', 'green', 'LineWidth', 4);
        else
            options = struct('Color', 'red', 'LineWidth', 1);
        end

        % plot each link
        plot([0 c1], [0 s1], options); hold on;
        plot([c1 c1+c12], [s1 s1+s12], options); hold on;
        plot([c1+c12 c1+c12+c123], [s1+s12 s1+s12+s123], options); hold on;

        xlabel('x [m]'); ylabel('y [m]');
    end
    
    title(methods{j});
end

print(fig, sprintf('results/%s_arm_motions.png', move_str),'-dpng','-r300');
savefig(sprintf('results/%s_arm_motions.fig', move_str));
end