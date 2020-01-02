function videoSimulation(q_opt, move_str)

transformMatrix = @(theta) [cos(theta), -sin(theta), cos(theta);...
                            sin(theta),  cos(theta), sin(theta);...
                            0, 0, 1];

[ptr_q, mtnb_q, mtn_q, mbp_q] = q_opt{:};

N = size(ptr_q, 2);

%% build frames data
frame0_ptr_list = zeros(2, N);
frame0_mtnb_list = zeros(2, N);
frame0_mtn_list = zeros(2, N);
frame0_mbp_list = zeros(2, N);
for i = 1:N
    % mtnb frames
    transMatPTR = transformMatrix(ptr_q(1, i));
    frame1_ptr_list(:, i) = transMatPTR(1 : 2, 3);
    transMatPTR = transMatPTR * transformMatrix(ptr_q(2, i));
    frame2_ptr_list(:, i) = transMatPTR(1 : 2, 3);
    transMatPTR = transMatPTR * transformMatrix(ptr_q(3, i));
    frame3_ptr_list(:, i) = transMatPTR(1 : 2, 3);
    
    % MTNB frames
    transMatMTNB = transformMatrix(mtnb_q(1, i));
    frame1_mtnb_list(:, i) = transMatMTNB(1 : 2, 3);
    transMatMTNB = transMatMTNB * transformMatrix(mtnb_q(2, i));
    frame2_mtnb_list(:, i) = transMatMTNB(1 : 2, 3);
    transMatMTNB = transMatMTNB * transformMatrix(mtnb_q(3, i));
    frame3_mtnb_list(:, i) = transMatMTNB(1 : 2, 3);
    
    % MTN frames
    transMatMTN = transformMatrix(mtn_q(1, i));
    frame1_mtn_list(:, i) = transMatMTN(1 : 2, 3);
    transMatMTN = transMatMTN * transformMatrix(mtn_q(2, i));
    frame2_mtn_list(:, i) = transMatMTN(1 : 2, 3);
    transMatMTN = transMatMTN * transformMatrix(mtn_q(3, i));
    frame3_mtn_list(:, i) = transMatMTN(1 : 2, 3);
    
    % MBP frames
    transMatMBP = transformMatrix(mbp_q(1, i));
    frame1_mbp_list(:, i) = transMatMBP(1 : 2, 3);
    transMatMBP = transMatMBP * transformMatrix(mbp_q(2, i));
    frame2_mbp_list(:, i) = transMatMBP(1 : 2, 3);
    transMatMBP = transMatMBP * transformMatrix(mbp_q(3, i));
    frame3_mbp_list(:, i) = transMatMBP(1 : 2, 3);
end

%%
hfig = figure('Name', 'Video of Motion');
set(gcf,'Position',[0 0 800 800]);

%% set PTR data bindings
subplot(2, 2, 1);
ptr_link1_x = [frame0_ptr_list(1, 1), frame1_ptr_list(1, 1)];
ptr_link1_y = [frame0_ptr_list(2, 1), frame1_ptr_list(2, 1)];
ptrPlotLink1 = plot(ptr_link1_x, ptr_link1_y, 'o-k', 'LineWidth', 3);
set(ptrPlotLink1, 'XDataSource', 'ptr_link1_x')
set(ptrPlotLink1, 'YDataSource', 'ptr_link1_y')

hold on

ptr_link2_x = [frame1_ptr_list(1, 1), frame2_ptr_list(1, 1)];
ptr_link2_y = [frame1_ptr_list(2, 1), frame2_ptr_list(2, 1)];
ptrPlotLink2 = plot(ptr_link2_x, ptr_link2_y, 'o-b', 'LineWidth', 3);
set(ptrPlotLink2, 'XDataSource', 'ptr_link2_x')
set(ptrPlotLink2, 'YDataSource', 'ptr_link2_y')

ptr_link3_x = [frame2_ptr_list(1, 1), frame3_ptr_list(1, 1)];
ptr_link3_y = [frame2_ptr_list(2, 1), frame3_ptr_list(2, 1)];
ptrPlotLink3 = plot(ptr_link3_x, ptr_link3_y, 'o-r', 'LineWidth', 3);
set(ptrPlotLink3, 'XDataSource', 'ptr_link3_x')
set(ptrPlotLink3, 'YDataSource', 'ptr_link3_y')

ptr_end_effector_x = [frame3_ptr_list(1, 1)];
ptr_end_effector_y = [frame3_ptr_list(2, 1)];
ptrPlotEndEffector = plot(ptr_end_effector_x, ptr_end_effector_y, 'm', 'LineWidth', 2);
set(ptrPlotEndEffector, 'XDataSource', 'ptr_end_effector_x')
set(ptrPlotEndEffector, 'YDataSource', 'ptr_end_effector_y')

title('PTR');
axis([0.0 2.5 -1.5 1.0]);
xlabel('x [m]'); ylabel('y [m]');

%% set MTNB data bindings
subplot(2, 2, 2);
mtnb_link1_x = [frame0_mtnb_list(1, 1), frame1_mtnb_list(1, 1)];
mtnb_link1_y = [frame0_mtnb_list(2, 1), frame1_mtnb_list(2, 1)];
mtnbPlotLink1 = plot(mtnb_link1_x, mtnb_link1_y, 'o-k', 'LineWidth', 3);
set(mtnbPlotLink1, 'XDataSource', 'mtnb_link1_x')
set(mtnbPlotLink1, 'YDataSource', 'mtnb_link1_y')

hold on

mtnb_link2_x = [frame1_mtnb_list(1, 1), frame2_mtnb_list(1, 1)];
mtnb_link2_y = [frame1_mtnb_list(2, 1), frame2_mtnb_list(2, 1)];
mtnbPlotLink2 = plot(mtnb_link2_x, mtnb_link2_y, 'o-b', 'LineWidth', 3);
set(mtnbPlotLink2, 'XDataSource', 'mtnb_link2_x')
set(mtnbPlotLink2, 'YDataSource', 'mtnb_link2_y')

mtnb_link3_x = [frame2_mtnb_list(1, 1), frame3_mtnb_list(1, 1)];
mtnb_link3_y = [frame2_mtnb_list(2, 1), frame3_mtnb_list(2, 1)];
mtnbPlotLink3 = plot(mtnb_link3_x, mtnb_link3_y, 'o-r', 'LineWidth', 3);
set(mtnbPlotLink3, 'XDataSource', 'mtnb_link3_x')
set(mtnbPlotLink3, 'YDataSource', 'mtnb_link3_y')

mtnb_end_effector_x = [frame3_mtnb_list(1, 1)];
mtnb_end_effector_y = [frame3_mtnb_list(2, 1)];
mtnbPlotEndEffector = plot(mtnb_end_effector_x, mtnb_end_effector_y, 'm', 'LineWidth', 2);
set(mtnbPlotEndEffector, 'XDataSource', 'mtnb_end_effector_x')
set(mtnbPlotEndEffector, 'YDataSource', 'mtnb_end_effector_y')

title('MTNB');
axis([0.0 2.5 -1.5 1.0]);
xlabel('x [m]'); ylabel('y [m]');

%% set MTN data bindings
subplot(2, 2, 3);
mtn_link1_x = [frame0_mtn_list(1, 1), frame1_mtn_list(1, 1)];
mtn_link1_y = [frame0_mtn_list(2, 1), frame1_mtn_list(2, 1)];
mtnPlotLink1 = plot(mtn_link1_x, mtn_link1_y, 'o-k', 'LineWidth', 3);
set(mtnPlotLink1, 'XDataSource', 'mtn_link1_x')
set(mtnPlotLink1, 'YDataSource', 'mtn_link1_y')

hold on

mtn_link2_x = [frame1_mtn_list(1, 1), frame2_mtn_list(1, 1)];
mtn_link2_y = [frame1_mtn_list(2, 1), frame2_mtn_list(2, 1)];
mtnPlotLink2 = plot(mtn_link2_x, mtn_link2_y, 'o-b', 'LineWidth', 3);
set(mtnPlotLink2, 'XDataSource', 'mtn_link2_x')
set(mtnPlotLink2, 'YDataSource', 'mtn_link2_y')

mtn_link3_x = [frame2_mtn_list(1, 1), frame3_mtn_list(1, 1)];
mtn_link3_y = [frame2_mtn_list(2, 1), frame3_mtn_list(2, 1)];
mtnPlotLink3 = plot(mtn_link3_x, mtn_link3_y, 'o-r', 'LineWidth', 3);
set(mtnPlotLink3, 'XDataSource', 'mtn_link3_x')
set(mtnPlotLink3, 'YDataSource', 'mtn_link3_y')

mtn_end_effector_x = [frame3_mtn_list(1, 1)];
mtn_end_effector_y = [frame3_mtn_list(2, 1)];
mtnPlotEndEffector = plot(mtn_end_effector_x, mtn_end_effector_y, 'm', 'LineWidth', 2);
set(mtnPlotEndEffector, 'XDataSource', 'mtn_end_effector_x')
set(mtnPlotEndEffector, 'YDataSource', 'mtn_end_effector_y')

title('MTN');
axis([0.0 2.5 -1.5 1.0]);
xlabel('x [m]'); ylabel('y [m]');

%% set MBP data bindings
subplot(2, 2, 4);
mbp_link1_x = [frame0_mbp_list(1, 1), frame1_mbp_list(1, 1)];
mbp_link1_y = [frame0_mbp_list(2, 1), frame1_mbp_list(2, 1)];
mbpPlotLink1 = plot(mbp_link1_x, mbp_link1_y, 'o-k', 'LineWidth', 3);
set(mbpPlotLink1, 'XDataSource', 'mbp_link1_x')
set(mbpPlotLink1, 'YDataSource', 'mbp_link1_y')

hold on

mbp_link2_x = [frame1_mbp_list(1, 1), frame2_mbp_list(1, 1)];
mbp_link2_y = [frame1_mbp_list(2, 1), frame2_mbp_list(2, 1)];
mbpPlotLink2 = plot(mbp_link2_x, mbp_link2_y, 'o-b', 'LineWidth', 3);
set(mbpPlotLink2, 'XDataSource', 'mbp_link2_x')
set(mbpPlotLink2, 'YDataSource', 'mbp_link2_y')

mbp_link3_x = [frame2_mbp_list(1, 1), frame3_mbp_list(1, 1)];
mbp_link3_y = [frame2_mbp_list(2, 1), frame3_mbp_list(2, 1)];
mbpPlotLink3 = plot(mbp_link3_x, mbp_link3_y, 'o-r', 'LineWidth', 3);
set(mbpPlotLink3, 'XDataSource', 'mbp_link3_x')
set(mbpPlotLink3, 'YDataSource', 'mbp_link3_y')

mbp_end_effector_x = [frame3_mbp_list(1, 1)];
mbp_end_effector_y = [frame3_mbp_list(2, 1)];
mbpPlotEndEffector = plot(mbp_end_effector_x, mbp_end_effector_y, 'm', 'LineWidth', 2);
set(mbpPlotEndEffector, 'XDataSource', 'mbp_end_effector_x')
set(mbpPlotEndEffector, 'YDataSource', 'mbp_end_effector_y')

title('MBP');
axis([0.0 2.5 -1.5 1.0]);
xlabel('x [m]'); ylabel('y [m]');

%% refresh plot frames
f = 1;
for i = 1 : N
    % ptr subplot
    ptr_link1_x = [frame0_ptr_list(1, f), frame1_ptr_list(1, f)];
    ptr_link1_y = [frame0_ptr_list(2, f), frame1_ptr_list(2, f)];
    refreshdata(ptrPlotLink1, 'caller')
    
    ptr_link2_x = [frame1_ptr_list(1, f), frame2_ptr_list(1, f)];
    ptr_link2_y = [frame1_ptr_list(2, f), frame2_ptr_list(2, f)];
    refreshdata(ptrPlotLink2, 'caller')
    
    ptr_link3_x = [frame2_ptr_list(1, f), frame3_ptr_list(1, f)];
    ptr_link3_y = [frame2_ptr_list(2, f), frame3_ptr_list(2, f)];
    refreshdata(ptrPlotLink3, 'caller')
    
    ptr_end_effector_x = [ptr_end_effector_x, frame3_ptr_list(1, f)];
    ptr_end_effector_y = [ptr_end_effector_y, frame3_ptr_list(2, f)];
    refreshdata(ptrPlotEndEffector, 'caller')
    
    % mtnb subplot
    mtnb_link1_x = [frame0_mtnb_list(1, f), frame1_mtnb_list(1, f)];
    mtnb_link1_y = [frame0_mtnb_list(2, f), frame1_mtnb_list(2, f)];
    refreshdata(mtnbPlotLink1, 'caller')
    
    mtnb_link2_x = [frame1_mtnb_list(1, f), frame2_mtnb_list(1, f)];
    mtnb_link2_y = [frame1_mtnb_list(2, f), frame2_mtnb_list(2, f)];
    refreshdata(mtnbPlotLink2, 'caller')
    
    mtnb_link3_x = [frame2_mtnb_list(1, f), frame3_mtnb_list(1, f)];
    mtnb_link3_y = [frame2_mtnb_list(2, f), frame3_mtnb_list(2, f)];
    refreshdata(mtnbPlotLink3, 'caller')
    
    mtnb_end_effector_x = [mtnb_end_effector_x, frame3_mtnb_list(1, f)];
    mtnb_end_effector_y = [mtnb_end_effector_y, frame3_mtnb_list(2, f)];
    refreshdata(mtnbPlotEndEffector, 'caller')
    
    % mtn subplot
    mtn_link1_x = [frame0_mtn_list(1, f), frame1_mtn_list(1, f)];
    mtn_link1_y = [frame0_mtn_list(2, f), frame1_mtn_list(2, f)];
    refreshdata(mtnPlotLink1, 'caller')
    
    mtn_link2_x = [frame1_mtn_list(1, f), frame2_mtn_list(1, f)];
    mtn_link2_y = [frame1_mtn_list(2, f), frame2_mtn_list(2, f)];
    refreshdata(mtnPlotLink2, 'caller')
    
    mtn_link3_x = [frame2_mtn_list(1, f), frame3_mtn_list(1, f)];
    mtn_link3_y = [frame2_mtn_list(2, f), frame3_mtn_list(2, f)];
    refreshdata(mtnPlotLink3, 'caller')
    
    mtn_end_effector_x = [mtn_end_effector_x, frame3_mtn_list(1, f)];
    mtn_end_effector_y = [mtn_end_effector_y, frame3_mtn_list(2, f)];
    refreshdata(mtnPlotEndEffector, 'caller')
    
    % mbp subplot
    mbp_link1_x = [frame0_mbp_list(1, f), frame1_mbp_list(1, f)];
    mbp_link1_y = [frame0_mbp_list(2, f), frame1_mbp_list(2, f)];
    refreshdata(mbpPlotLink1, 'caller')
    
    mbp_link2_x = [frame1_mbp_list(1, f), frame2_mbp_list(1, f)];
    mbp_link2_y = [frame1_mbp_list(2, f), frame2_mbp_list(2, f)];
    refreshdata(mbpPlotLink2, 'caller')
    
    mbp_link3_x = [frame2_mbp_list(1, f), frame3_mbp_list(1, f)];
    mbp_link3_y = [frame2_mbp_list(2, f), frame3_mbp_list(2, f)];
    refreshdata(mbpPlotLink3, 'caller')
    
    mbp_end_effector_x = [mbp_end_effector_x, frame3_mbp_list(1, f)];
    mbp_end_effector_y = [mbp_end_effector_y, frame3_mbp_list(2, f)];
    refreshdata(mbpPlotEndEffector, 'caller')
    
    M(i) = getframe(hfig);
    f = f + 1;
end

%% save video
v = VideoWriter(sprintf('results/%s_vid_sim', move_str));
v.FrameRate = 90;
v.Quality = 100;
open(v);
writeVideo(v, M);
close(v);
end