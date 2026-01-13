%% vehicle_dlqr_trajectory_tracking_demo.m
% Discrete-time model + DLQR controller for vehicle trajectory tracking
% - Nonlinear plant: bicycle model (4-wheel proxy)
% - Control design: linearized error model (continuous) -> ZOH discretization -> dlqr
% - Control updated every Ts with zero-order hold
%
% Plant state: [X; Y; psi; vx; vy; r]
% Error state (control design): xe = [vy; r; ey; epsi; ev]
% Input: u = [steering_input, throttle]
%
% Requires: Control System Toolbox for dlqr()

clear; close all; clc;

%% ----------------------- Vehicle parameters ------------------------------
params.m  = 1500;      % mass [kg]
params.Iz = 2500;      % yaw inertia [kg m^2]
params.lf = 1.2;       % CG to front axle [m]
params.lr = 1.6;       % CG to rear axle [m]
params.Cf = 80000;     % front cornering stiffness [N/rad]
params.Cr = 80000;     % rear cornering stiffness [N/rad]

%% ----------------------- Sampling / simulation ---------------------------
Ts   = 0.02;        % controller sampling time [s] (50 Hz)
Tend = 25;          % [s]
t    = (0:Ts:Tend).';
N    = numel(t);

% Nominal linearization speed
Vx0 = 15;           % [m/s]

%% ----------------------- Reference trajectory ----------------------------
kappa_ref = 0.01*sin(0.35*t) + 0.005*sin(0.10*t);   % [1/m]
v_ref     = Vx0 + 1.0*sin(0.15*t);                  % [m/s]
a_ref     = [diff(v_ref)/Ts; 0];                    % approx dv/dt

[Xref, Yref, psiref] = integrate_reference(t, v_ref, kappa_ref);

%% ----------------------- Continuous-time linear error model --------------
m  = params.m; Iz = params.Iz; lf = params.lf; lr = params.lr; Cf = params.Cf; Cr = params.Cr;

A11 = -(Cf+Cr)/(m*Vx0);
A12 = -(Vx0 + (lf*Cf - lr*Cr)/(m*Vx0));
A21 = -(lf*Cf - lr*Cr)/(Iz*Vx0);
A22 = -(lf^2*Cf + lr^2*Cr)/(Iz*Vx0);

Bvydelta = Cf/m;
Brdelta  = lf*Cf/Iz;

% Continuous domain model of the controller:
% error_state_derivative = Ac*error_state + Bc*input
% Your controller is supposed to regulate the error states!

Ac = zeros(5,5);
Bc = zeros(5,2);

% [vy; r]
Ac(1,1) = A11; Ac(1,2) = A12;
Ac(2,1) = A21; Ac(2,2) = A22;
Bc(1,1) = Bvydelta;
Bc(2,1) = Brdelta;

% ey_dot = vy + Vx0*epsi
Ac(3,1) = 1;
Ac(3,4) = Vx0;

% epsi_dot = r   (ref curvature handled by feedforward)
Ac(4,2) = 1;

% ev_dot = ax    (ref accel handled by feedforward)
Bc(5,2) = 1;
%% ---------------------------------------------------------------
% DESIGN YOUR CONTROLLER HERE!
% get the error state dynamics (using the associated matrices)
% don't forget to discretize it using c2d_zoh_exact below!
% get the gain vector (compatible to the error state vector), etc.
%---------------------------------------------------------------

%% ----------------------- Nonlinear plant simulation -----------------------
% We'll simulate the nonlinear plant with a smaller integration step inside each Ts
dt_int = Ts/10;                   % integration step for RK4
n_int  = round(Ts/dt_int);

x = zeros(N,6);                   % [X Y psi vx vy r]

% Initial condition offset from reference
x(1,:) = [ Xref(1)-2.0,  Yref(1)+1.0,  psiref(1)+deg2rad(8),  Vx0-5,  0.0,  0.0 ];

u_log   = zeros(N,2);
ey_log  = zeros(N,1);
epsi_log= zeros(N,1);
ev_log  = zeros(N,1);

for k = 1:N-1
    % current plant state
    X   = x(k,1);  Y   = x(k,2);  psi = x(k,3);
    vx  = x(k,4);  vy  = x(k,5);  r   = x(k,6);

    % reference at this sample
    Xr   = Xref(k); Yr = Yref(k); psir = psiref(k);
    vr   = v_ref(k);
    ar   = a_ref(k);
    kap  = kappa_ref(k);

    % errors
    [ey, epsi] = lateral_heading_error(X,Y,psi, Xr,Yr,psir);
    ev = vx - vr;

    % simple feedforward (starter) inputs:
    steering_feed_forward = (lf+lr)*kap;
    throttle_feed_forward = ar;

%---------------------------------------------------------------
% CALCULATE THE INPUTS HERE! Don't forget to add the feed-forward inputs to
% your regulation inputs.
% steering_input = ...;
% throttle = ...;
%---------------------------------------------------------------

    % saturations (optional but realistic)
    steering_input = min(max(steering_input, deg2rad(-25)), deg2rad(25));
    throttle    = min(max(throttle, -6), 3);

    u_log(k,:)  = [steering_input, throttle];
    ey_log(k)   = ey;
    epsi_log(k) = epsi;
    ev_log(k)   = ev;

    % propagate nonlinear plant over [t_k, t_{k+1}] with ZOH input
    xk = x(k,:).';
    for j = 1:n_int
        xk = rk4_step(@(xx) bicycle_dynamics(xx, [steering_input; throttle], params), xk, dt_int);
        xk(4) = max(xk(4), 0.5);  % keep vx positive
    end
    x(k+1,:) = xk.';
end

% log last
u_log(end,:) = u_log(end-1,:);
ey_log(end)  = ey_log(end-1);
epsi_log(end)= epsi_log(end-1);
ev_log(end)  = ev_log(end-1);

%% ----------------------- Plots -------------------------------------------
figure('Name','Trajectory');
plot(Xref, Yref, 'LineWidth', 1.6); hold on;
plot(x(:,1), x(:,2), '--', 'LineWidth', 1.4);
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
legend('Reference','Vehicle (nonlinear sim)','Location','Best');
title('Trajectory tracking with DLQR (discrete controller, nonlinear plant)');

figure('Name','Tracking errors');
subplot(3,1,1);
plot(t, ey_log, 'LineWidth', 1.2); grid on;
ylabel('e_y [m]'); title('Cross-track error');

subplot(3,1,2);
plot(t, rad2deg(epsi_log), 'LineWidth', 1.2); grid on;
ylabel('e_\psi [deg]'); title('Heading error');

subplot(3,1,3);
plot(t, ev_log, 'LineWidth', 1.2); grid on;
xlabel('t [s]'); ylabel('e_v [m/s]'); title('Speed error');

figure('Name','Inputs');
subplot(2,1,1);
plot(t, rad2deg(u_log(:,1)), 'LineWidth', 1.2); grid on;
ylabel('\delta [deg]'); title('Steering input');

subplot(2,1,2);
plot(t, u_log(:,2), 'LineWidth', 1.2); grid on;
xlabel('t [s]'); ylabel('a_x [m/s^2]'); title('Longitudinal accel command');

%% ======================= Helper functions ================================
function xdot = bicycle_dynamics(x, u, params)
% Nonlinear bicycle model with linear tire forces (small slip angles)
% State: x = [X; Y; psi; vx; vy; r]
% Input: u = [delta; ax]
    X = x(1); Y = x(2); psi = x(3); %#ok<NASGU>
    vx = x(4); vy = x(5); r = x(6);

    delta = u(1);
    ax    = u(2);

    m  = params.m; Iz = params.Iz; lf = params.lf; lr = params.lr; Cf = params.Cf; Cr = params.Cr;

    vx_eff = max(vx, 0.5);

    alpha_f = delta - (vy + lf*r)/vx_eff;
    alpha_r = -(vy - lr*r)/vx_eff;

    Fyf = Cf * alpha_f;
    Fyr = Cr * alpha_r;

    Xdot   = vx*cos(x(3)) - vy*sin(x(3));
    Ydot   = vx*sin(x(3)) + vy*cos(x(3));
    psidot = r;

    vxdot = ax + r*vy;
    vydot = (Fyf + Fyr)/m - r*vx;
    rdot  = (lf*Fyf - lr*Fyr)/Iz;

    xdot = [Xdot; Ydot; psidot; vxdot; vydot; rdot];
end

function [Xref, Yref, psiref] = integrate_reference(t, v_ref, kappa_ref)
% Integrate reference pose from v and curvature:
% psidot = v*kappa, Xdot = v cos psi, Ydot = v sin psi
    Ts = t(2)-t(1);
    N = numel(t);
    Xref = zeros(N,1); Yref = zeros(N,1); psiref = zeros(N,1);
    for k = 1:N-1
        psidot = v_ref(k)*kappa_ref(k);
        psiref(k+1) = psiref(k) + Ts*psidot;
        Xref(k+1) = Xref(k) + Ts*(v_ref(k)*cos(psiref(k)));
        Yref(k+1) = Yref(k) + Ts*(v_ref(k)*sin(psiref(k)));
    end
end

function [ey, epsi] = lateral_heading_error(X,Y,psi, Xr,Yr,psir)
% Cross-track error (in ref normal direction) and heading error
    ep = [X - Xr; Y - Yr];
    n_hat = [-sin(psir); cos(psir)];
    ey = n_hat.' * ep;
    epsi = wrapToPi(psi - psir);
end

function xnext = rk4_step(f, x, h)
% One Runge-Kutta 4 step
    k1 = f(x);
    k2 = f(x + 0.5*h*k1);
    k3 = f(x + 0.5*h*k2);
    k4 = f(x + h*k3);
    xnext = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
end

function [Ad, Bd] = c2d_zoh_exact(A, B, Ts)
% Exact ZOH discretization using augmented matrix exponential:
% exp([A B; 0 0]*Ts) = [Ad Bd; 0 I]
    n = size(A,1);
    m = size(B,2);
    M = [A, B; zeros(m,n), zeros(m,m)];
    Md = expm(M*Ts);
    Ad = Md(1:n, 1:n);
    Bd = Md(1:n, n+1:n+m);
end
