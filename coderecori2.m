%% Part II : Symbolic Math for Reciprocating motion 
%            with quick return mechanism + Animation + Graphs

clear; clc; close all;

%% export setting
doVideo = true;         
doGIF   = false;        

videoFile = 'quick_return.avi';
gifFile   = 'quick_return.gif';

%% 1) Geometry
r       = 0.03;      % [m] crank radius  (circle of red pin)
R_scale = 0.025;     % [m/rad] scale for rack displacement (lever angle -> rack x)
L_slot  = 0.14;      % [m] length of yellow lever above pivot

O = [0.00, 0.08];    % crank center
B = [0.00, 0.00];    % FIXED pivot of yellow lever (black pin, on ground)
rack_y = -0.02;      % rack level

N      = 300;                         % number of frames
th_vec = linspace(0, 2*pi, N);        % crank angle

%% 2) Reference angle (theta = 90 deg) for quick-return mapping
theta_top = pi/2;
C_top     = O + r*[cos(theta_top), sin(theta_top)];
phi_top   = atan2(C_top(2)-B(2), C_top(1)-B(1));   % lever angle when crank at top

%% 3) Figure setup (Animation)
figure('Name','Quick-Return Animation','Color','w');
axis equal; hold on; grid on;
xlabel('x [m]'); ylabel('y [m]');
xlim([-0.08 0.18]);
ylim([-0.06 0.14]);
title('Quick-Return Animation');

% graphic objects
h_crank   = plot(NaN,NaN,'Color',[0 0.6 0],'LineWidth',3); % green crank
h_slot    = plot(NaN,NaN,'y-','LineWidth',4);              % yellow lever (slot+gear arm)
h_rack    = plot(NaN,NaN,'Color',[1 0.5 0],'LineWidth',6); % orange rack

h_pinC    = plot(NaN,NaN,'ro','MarkerFaceColor','r');      % red pin
h_pivotB  = plot(B(1),B(2),'ko','MarkerFaceColor','k');    % fixed black pivot

% pathways (history)
C_hist    = nan(N,2);   % red pin path
Tip_hist  = nan(N,2);   % upper yellow tip path
L_hist    = nan(N,2);   % left end of rack path
R_hist    = nan(N,2);   % right end of rack path
Q_hist    = nan(N,2);   % center of rack
phi_hist  = nan(N,1);   % lever angle history
x_rack_hist = nan(N,1); % rack displacement history

h_pathC  = plot(NaN,NaN,'r--','LineWidth',1.5);                  % path of red pin
h_pathY  = plot(NaN,NaN,'y--','LineWidth',1.5);                  % path of yellow tip
h_pathL  = plot(NaN,NaN,'--','Color',[1 0.5 0],'LineWidth',1.2); % path left rack end
h_pathR  = plot(NaN,NaN,'--','Color',[1 0.5 0],'LineWidth',1.2); % path right rack end

rack_len = 0.06;   % half rack length

%% Variable export setting
if doVideo
    vw = VideoWriter(videoFile,'Motion JPEG AVI');  
    vw.FrameRate = 30;
    open(vw);
end

if doGIF && exist(gifFile,'file')
    delete(gifFile);
end

%% 4) Animation loop
for k = 1:N
    theta = th_vec(k);

    %  red pin on green crank 
    C = O + r*[cos(theta), sin(theta)];     % position of red pin

    % lever angle: line from pivot B through C
    phi = atan2(C(2)-B(2), C(1)-B(1));

    %  rack displacement from lever angle (quick-return geometry)
    phi_rel = phi - phi_top;
    x_rack  = R_scale*phi_rel;
    Q       = [x_rack, rack_y];             % rack centre

    Lp = [Q(1) - rack_len, rack_y];         % left end
    Rp = [Q(1) + rack_len, rack_y];         % right end

    %  lower intersection of lever with rack line (gear contact approx.) 
    if abs(sin(phi)) < 1e-6
        H = [B(1), rack_y];
    else
        s_rack = (rack_y - B(2))/sin(phi);  % parameter along lever from pivot
        H      = B + s_rack*[cos(phi), sin(phi)];
    end

    %  upper tip of yellow slot (purely visual) 
    top_slot = B + L_slot*[cos(phi), sin(phi)];

    % store histories (for later graphs)
    C_hist(k,:)      = C;
    Tip_hist(k,:)    = top_slot;
    L_hist(k,:)      = Lp;
    R_hist(k,:)      = Rp;
    Q_hist(k,:)      = Q;
    phi_hist(k)      = phi;
    x_rack_hist(k)   = x_rack;

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

    %% QUICK-RETURN TIMING
    if sin(theta) > 0
        pause(0.06);    % Forward - slow
    else
        pause(0.01);    % Return - quick
    end

    %% EXPORT FRAME
    frame = getframe(gcf);  

    if doVideo
        writeVideo(vw, frame);   
    end

    if doGIF
        [im,map] = frame2im(frame);
        [A,cm]   = rgb2ind(im,256);
        if k == 1
            imwrite(A,cm,gifFile,'gif','LoopCount',Inf,'DelayTime',0.03);
        else
            imwrite(A,cm,gifFile,'gif','WriteMode','append','DelayTime',0.03);
        end
    end
end

%% Close video
if doVideo
    close(vw);
end

%% ============================================================
% PART II – GRAPH ANALYSIS (Position / Velocity / Acceleration)

% Assumption crank rotate with constant omega 
omega_an = 10;                       
theta_vec = th_vec;                    
dtheta    = theta_vec(2) - theta_vec(1);
t_vec     = theta_vec / omega_an; 

x_vec = x_rack_hist(:).';             
% ความเร็ว: v = dx/dt = (dx/dθ)*omega
dx_dtheta = gradient(x_vec, dtheta);
v_vec     = dx_dtheta * omega_an;
% ความเร่ง: a = dv/dt = (dv/dθ)*omega
dv_dtheta = gradient(v_vec, dtheta);
a_vec     = dv_dtheta * omega_an;

%% 1) Graph x(θ), v(θ), a(θ)
figure('Name','Rack vs Crank Angle','Color','w');

subplot(3,1,1);
plot(theta_vec, x_vec,'LineWidth',1.5);
xlabel('\theta [rad]');
ylabel('x_{rack}(\theta) [m]');
title('Rack Displacement x(\theta)');
grid on;

subplot(3,1,2);
plot(theta_vec, v_vec,'r','LineWidth',1.5);
xlabel('\theta [rad]');
ylabel('v_{rack}(\theta) [m/s]');
title('Rack Velocity v(\theta)');
grid on;

subplot(3,1,3);
plot(theta_vec, a_vec,'k','LineWidth',1.5);
xlabel('\theta [rad]');
ylabel('a_{rack}(\theta) [m/s^2]');
title('Rack Acceleration a(\theta)');
grid on;

%% 2) Graph x(t), v(t), a(t)
figure('Name','Rack vs Time','Color','w');

subplot(3,1,1);
plot(t_vec, x_vec,'LineWidth',1.5);
xlabel('t [s]');
ylabel('x_{rack}(t) [m]');
title('Rack Displacement x(t)');
grid on;

subplot(3,1,2);
plot(t_vec, v_vec,'r','LineWidth',1.5);
xlabel('t [s]');
ylabel('v_{rack}(t) [m/s]');
title('Rack Velocity v(t)');
grid on;

subplot(3,1,3);
plot(t_vec, a_vec,'k','LineWidth',1.5);
xlabel('t [s]');
ylabel('a_{rack}(t) [m/s^2]');
title('Rack Acceleration a(t)');
grid on;

%% 3) Graph φ(θ)
figure('Name','Lever Angle vs Crank Angle','Color','w');
plot(theta_vec, phi_hist,'g','LineWidth',1.5);
xlabel('\theta [rad]');
ylabel('\phi(\theta) [rad]');
title('Lever Angle \phi vs Crank Angle \theta');
grid on;
