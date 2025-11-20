%% Part II : Symbolic Math for Reciprocating motion with quick return mechanism

clear; clc; close all;

%% ====== ตั้งค่าการ export ======
doVideo = true;          % บันทึกเป็น .mp4 (video)
doGIF   = false;         % บันทึกเป็น .gif (animation)

videoFile = 'quick_return.mp4';
gifFile   = 'quick_return.gif';

%% 1) Geometry
r       = 0.03;      % [m] crank radius  (circle of red pin)
R       = 0.025;     % [m] scale for rack displacement
L_slot  = 0.14;      % [m] length of yellow lever above pivot

O = [0.00, 0.08];    % crank center
B = [0.00, 0.00];    % FIXED pivot of yellow lever (black pin, on ground)
rack_y = -0.02;      % rack level

N      = 300;                         % number of frames
th_vec = linspace(0, 2*pi, N);        % crank angle

%% 2) Reference angle (theta = 90 deg) for quick-return
theta_top = pi/2;
C_top     = O + r*[cos(theta_top), sin(theta_top)];
phi_top   = atan2(C_top(2)-B(2), C_top(1)-B(1));   % lever angle when crank at top

%% 3) Figure setup
figure('Name','Quick-Return Animation','Color','w');
axis equal; hold on; grid on;
xlabel('x [m]'); ylabel('y [m]');
xlim([-0.08 0.18]);
ylim([-0.06 0.14]);
title('Quick-Return Animation');

% graphic objects (no black dash baseline)
h_crank   = plot(NaN,NaN,'Color',[0 0.6 0],'LineWidth',3); % green crank
h_slot    = plot(NaN,NaN,'y-','LineWidth',4);              % yellow lever (slot+gear arm)
h_rack    = plot(NaN,NaN,'Color',[1 0.5 0],'LineWidth',6); % orange rack

h_pinC    = plot(NaN,NaN,'ro','MarkerFaceColor','r');      % red pin
h_pivotB  = plot(B(1),B(2),'ko','MarkerFaceColor','k');    % fixed black pivot

% pathways (history)
C_hist   = nan(N,2);   % red pin path
Tip_hist = nan(N,2);   % upper yellow tip path
L_hist   = nan(N,2);   % left end of rack path
R_hist   = nan(N,2);   % right end of rack path

h_pathC  = plot(NaN,NaN,'r--','LineWidth',1.5);                  % path of red pin
h_pathY  = plot(NaN,NaN,'y--','LineWidth',1.5);                  % path of yellow tip
h_pathL  = plot(NaN,NaN,'--','Color',[1 0.5 0],'LineWidth',1.2); % path left rack end
h_pathR  = plot(NaN,NaN,'--','Color',[1 0.5 0],'LineWidth',1.2); % path right rack end

rack_len = 0.06;   % half rack length

%% ====== เตรียมตัวแปรสำหรับ export ======
if doVideo
    vw = VideoWriter(videoFile,'MPEG-4');
    vw.FrameRate = 1/0.02;   % ค่าประมาณตาม pause ตอน forward
    open(vw);
end

% ถ้าใช้ GIF ให้ลบไฟล์เก่า (ถ้ามี)
if doGIF && exist(gifFile,'file')
    delete(gifFile);
end

%% 4) Animation loop
x_prev = NaN;

for k = 1:N
    theta = th_vec(k);

    %  red pin on green crank 
    C = O + r*[cos(theta), sin(theta)];     % position of red pin

    % lever angle: line from pivot B through C
    phi = atan2(C(2)-B(2), C(1)-B(1));

    %  rack displacement from lever angle (quick-return) 
    phi_rel = phi - phi_top;
    x_rack  = R*phi_rel;
    Q       = [x_rack, rack_y];             % rack centre

    % rack ends (for pathways)
    L = [Q(1) - rack_len, rack_y];          % left end
    R = [Q(1) + rack_len, rack_y];          % right end

    %  lower intersection of lever with rack line (gear contact) 
    if abs(sin(phi)) < 1e-6
        H = [B(1), rack_y];
    else
        s_rack = (rack_y - B(2))/sin(phi);  % parameter along lever from pivot
        H      = B + s_rack*[cos(phi), sin(phi)];
    end

    %  upper tip of yellow slot (purely visual) 
    top_slot = B + L_slot*[cos(phi), sin(phi)];

    % store histories
    C_hist(k,:)   = C;
    Tip_hist(k,:) = top_slot;
    L_hist(k,:)   = L;
    R_hist(k,:)   = R;

    %% update drawings
    % crank
    set(h_crank,'XData',[O(1) C(1)], 'YData',[O(2) C(2)]);

    % yellow lever from lower contact H up through pivot B to top_slot
    set(h_slot,'XData',[H(1) B(1) top_slot(1)], ...
               'YData',[H(2) B(2) top_slot(2)]);

    % rack (solid)
    set(h_rack,'XData',[Q(1)-rack_len, Q(1)+rack_len], ...
               'YData',[rack_y rack_y]);

    % markers
    set(h_pinC, 'XData',C(1),'YData',C(2));

    % pathways (dashed)
    set(h_pathC,'XData',C_hist(1:k,1),'YData',C_hist(1:k,2));       % red
    set(h_pathY,'XData',Tip_hist(1:k,1),'YData',Tip_hist(1:k,2));   % yellow tip
    set(h_pathL,'XData',L_hist(1:k,1),'YData',L_hist(1:k,2));       % orange left
    set(h_pathR,'XData',R_hist(1:k,1),'YData',R_hist(1:k,2));       % orange right

    title(sprintf('Quick-Return Animation (\\theta = %.2f rad)',theta));
    drawnow;

    %% ====== EXPORT FRAME ======
    frame = getframe(gcf);   % จับภาพกราฟทั้งหน้าต่าง

    if doVideo
        writeVideo(vw, frame);
    end

    if doGIF
        [im,map] = frame2im(frame);      % image + colormap
        [A,cm]   = rgb2ind(im,256);
        delay    = 0.02;                 % หน่วงเวลาต่อเฟรม (s)
        if k == 1
            imwrite(A,cm,gifFile,'gif','LoopCount',Inf,'DelayTime',delay);
        else
            imwrite(A,cm,gifFile,'gif','WriteMode','append','DelayTime',delay);
        end
    end

    %% quick-return timing (rack forward slow, return fast)
    if isnan(x_prev)
        pause(0.02);
    else
        if x_rack >= x_prev
            pause(0.020);      % forward stroke (slow)
        else
            pause(0.008);      % return stroke (fast)
        end
    end
    x_prev = x_rack;
end

%% ====== ปิดไฟล์วิดีโอ ======
if doVideo
    close(vw);
end
