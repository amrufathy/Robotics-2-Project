function plotArm(q, p0, pf)
q1 = q(1, :);
q2 = q(2, :);
q3 = q(3, :);

figure('Name', 'Arm');
plot([p0(1) pf(1)], [p0(2) pf(2)], 'Color', 'black', 'LineWidth', 3); hold on;

for i = 1:150:size(q, 2)
    axis square; grid on;
%     axis([0.0 2.5 -1.5 1]); %t2
    
    % short form for trig functions
    q1i = q1(i); q2i = q2(i); q3i = q3(i);
    c1 = cos(q1i); c12 = cos(q1i + q2i); c123 = cos(q1i + q2i + q3i);
    s1 = sin(q1i); s12 = sin(q1i + q2i); s123 = sin(q1i + q2i + q3i);

    if i == 1
        options = struct('Color', 'blue', 'LineWidth', 2);
    elseif i + 150 >= size(q, 2)
        options = struct('Color', 'green', 'LineWidth', 2);
    else
        options = struct('Color', 'black', 'LineWidth', 0.5);
    end
    
    % plot each link
    plot([0 c1], [0 s1], options); hold on;
    plot([c1 c1+c12], [s1 s1+s12], options); hold on;
    plot([c1+c12 c1+c12+c123], [s1+s12 s1+s12+s123], options); hold on;
    
    xlabel('x [m]');
    ylabel('y [m]');
end

end