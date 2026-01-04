% MATLAB Script for Defining a Discrete Linear System

% Continuous-time system matrices
A = [0  0  0  0  0  0  1  0  0  0  0  0;
     0  0  0  0  0  0  0  1  0  0  0  0;
     0  0  0  0  0  0  0  0  1  0  0  0;
     0  0  0  0  0  0  0  0  0  1  0  0;
     0  0  0  0  0  0  0  0  0  0  1  0;
     0  0  0  0  0  0  0  0  0  0  0  1;
    -2  1  0  0  0  0  0  0  0  0  0  0;
     1 -2  1  0  0  0  0  0  0  0  0  0;
     0  1 -2  1  0  0  0  0  0  0  0  0;
     0  0  1 -2  1  0  0  0  0  0  0  0;
     0  0  0  1 -2  1  0  0  0  0  0  0;
     0  0  0  0  1 -1  0  0  0  0  0  0];

B = [0  0  0;
     0  0  0;
     0  0  0;
     0  0  0;
     0  0  0;
     0  0  0;
     1  0  0;
    -1  0 -1;
     0  0  0;
     0  0  0;
     0  1  0;
     0 -1  0];

% Output matrix (example: observe all displacements)

C = [1 0 0 0 0 0 0 0 0 0 0 0];

% Sampling time
Ts = 0.01; % Choose an appropriate sampling time

% Discretize the system
sys_continuous = ss(A, B, C, zeros(size(C, 1), size(B, 2)));
sys_discrete = c2d(sys_continuous, Ts);

% Extract discrete-time matrices
Ad = sys_discrete.A;
Bd = sys_discrete.B;
Cd = sys_discrete.C;

% Display the discrete system matrices
disp('Discrete-time A matrix:');
disp(Ad);
disp('Discrete-time B matrix:');
disp(Bd);
disp('Discrete-time C matrix:');
disp(Cd);

% Simulate the system with an initial condition
x0 = [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]; % Initial state
N = 1000; % Number of simulation steps
u = zeros(size(B, 2), N); % Zero input for open-loop simulation

% Preallocate state and output arrays
x = zeros(size(Ad, 1), N);
y = zeros(size(Cd, 1), N);
x(:, 1) = x0;

% Simulate the system
for k = 1:N-1
    x(:, k+1) = Ad * x(:, k) + Bd * u(:, k);
    y(:, k) = Cd * x(:, k);
end
y(:, N) = Cd * x(:, N);

% Plot the displacements
time = (0:N-1) * Ts;
figure;
plot(time, y');
xlabel('Time (s)');
ylabel('Displacements');
legend('d1', 'd2', 'd3', 'd4', 'd5', 'd6');
title('Displacements of the Masses');
grid on;
