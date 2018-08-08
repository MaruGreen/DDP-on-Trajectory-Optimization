
function [newTraj, newInput, cost] = Update2 ( current_traj, current_input, Q, R, L, dt, ref_traj, ref_input, full )

% This function update the trajectory by the formulation from Wiki's
% current_traj (6 x N)
% current_input (2 x N-1)

if nargin < 9
    full = false;
end

%% Initialize some parameters
N = size(current_traj,2);
ref_input = [ref_input zeros(2,1)];
costX = 0;
costX1 = 0;
costX2 = 0;
costX3 = 0;
costX4 = 0;
costX5 = 0;
costX6 = 0;
costU = 0;

%% Backward pass
% initial the space to restore the result
K = zeros(2,6*N);
del_u = zeros(2,N);
% at first, initial the V_X and V_XX as zero
V_X = zeros(6,1);
V_XX = zeros(6,6);
for i = N : -1 : 1
    % just to calculate the A and B
    state = current_traj(:,i);
    if i == N
        input = [0;0];
    else
        input = current_input(:,i);
    end
    % get fx,fu,fxx, fuu and fxu
    [f_X,f_u,fxx,fxu,fuu] = find2diff(state,input,dt,L);
    fux =  permute(fxu,[1 3 2]);
    % obtain the differential of Q
    Q_X = f_X' * V_X + Q * (state - ref_traj(:,i));
    Q_u = f_u' * V_X + R * (input - ref_input(:,i));
    Q_XX = Q + f_X' * V_XX * f_X;
    Q_uu = R + f_u' * V_XX * f_u;
    Q_uX = f_u' * V_XX * f_X;
    %Q_Xu = f_X' * V_XX * f_u; % no need
    %size(fxx)
    %size(fux)
    %size(fuu) % how to multi, can get the right size?
    % want 2nd diff?
    if full
        Q_XX = Q_XX + vectens(V_X, permute(fxx,[2 3 1]));
        Q_uu = Q_uu + vectens(V_X, permute(fuu,[2 3 1]));
        Q_uX = Q_uX + vectens(V_X, permute(fux,[2 3 1]));   % fxu and Q_uX should be noticed
    end
    % obtain K and del_u and save
    %Ki = -inv(Q_uu) * Q_uX;  % 2x6
    %ki = -inv(Q_uu) * Q_u;   % 2x1
    Ki = -Q_uu \ Q_uX;  % 2x6
    ki = -Q_uu \ Q_u;   % 2x1
    K(:,6*i-5:6*i) = Ki;
    del_u(:,i) = ki;
    % obtain V_X and V_XX for i = i - 1
    V_X = Q_X + Ki'*Q_uu*ki + Ki'*Q_u + Q_uX'*ki;     % copy from iLQG, different from wiki
    V_XX = Q_XX + Ki'*Q_uu*Ki + Ki'*Q_uX + Q_uX'*Ki;  % copy from iLQG, different from wiki
    V_XX = 0.5 * (V_XX + V_XX');
end

%% Forward Pass
newTraj = current_traj;
newInput = current_input;
for i = 1 : 1 : N-1
    du = (0.9) * del_u(:,i) + K(:,6*i-5:6*i) * (newTraj(:,i) - current_traj(:,i));
    newInput(:,i) = current_input(:,i) + du;
    [new, ~, ~] = transKine(newTraj(:,i),newInput(:,i),dt,L);
    if i >= N
        continue;
    end
    newTraj(:,i+1) = new;
    costX1 = costX1 + 0.5 * (newTraj(1,i) - ref_traj(1,i))' * Q(1,1) * (newTraj(1,i) - ref_traj(1,i));
    costX2 = costX2 + 0.5 * (newTraj(2,i) - ref_traj(2,i))' * Q(2,2) * (newTraj(2,i) - ref_traj(2,i));
    costX3 = costX3 + 0.5 * (newTraj(3,i) - ref_traj(3,i))' * Q(3,3) * (newTraj(3,i) - ref_traj(3,i));
    costX4 = costX4 + 0.5 * (newTraj(4,i) - ref_traj(4,i))' * Q(4,4) * (newTraj(4,i) - ref_traj(4,i));
    costX5 = costX5 + 0.5 * (newTraj(5,i) - ref_traj(5,i))' * Q(5,5) * (newTraj(5,i) - ref_traj(5,i));
    costX6 = costX6 + 0.5 * (newTraj(6,i) - ref_traj(6,i))' * Q(6,6) * (newTraj(6,i) - ref_traj(6,i));
    costX = costX + 0.5 * (newTraj(:,i) - ref_traj(:,i))' * Q * (newTraj(:,i) - ref_traj(:,i));
    costU = costU + 0.5 * (newInput(:,i) - ref_input(:,i))' * R * (newInput(:,i) - ref_input(:,i));
end

i = N;
costX = costX + 0.5 * (newTraj(:,i) - ref_traj(:,i))' * Q * (newTraj(:,i) - ref_traj(:,i));
cost = [costX;costU;costX+costU;costX1;costX2;costX3;costX4;costX5;costX6];

end

%% Help function to find diff about dynamics f(x,u)
function [fx,fu,fxx,fxu,fuu] = find2diff(x,u,dt,L)

n = size(x,1); % n = 6
m = size(u,1); % m = 2
velo = x(3);
theta = x(4);
phi = x(5);

% first diff
fx = eye(n);
fx(1,3) = dt * cos(theta);
fx(2,3) = dt * sin(theta);
fx(4,3) = -dt * tan(phi) / L;
fx(1,4) = -velo * dt * sin(theta);
fx(2,4) = velo * dt * cos(theta);
fx(4,5) = -velo * dt / (L * (cos(phi))^2);
fx(5,6) = dt;

fu = zeros(n,m);
fu(3,1) = dt;
fu(6,2) = dt;

% second diff
JJ = zeros(n*(n+m),n+m);
% for v
JJ(13,4) = -dt * sin(theta);
JJ(14,4) = dt * cos(theta);
JJ(16,5) = -dt / (L * (cos(phi))^2);
% for theta
JJ(19,3) = -dt * sin(theta);
JJ(19,4) = -velo * dt * cos(theta);
JJ(20,3) = dt * cos(theta);
JJ(20,4) = -velo * dt * sin(theta);
% for phi
JJ(28,3) = -dt / (L * (cos(phi))^2);
JJ(28,5) = -2* velo * dt * tan(theta) / (L * (cos(phi))^2);
% reshape for exacting
JJ = reshape(JJ,[n n+m n+m]);
JJ = 0.5 * (JJ + permute(JJ,[1 3 2]));
% exacting
fxx = JJ(:,1:n,1:n);
fxu = JJ(:,1:n,n+1:n+m);
fuu = JJ(:,n+1:n+m,n+1:n+m);

end

%% Tensor multiplication for DDP terms
function output = vectens(Vx, tensor)
% Vx (6x1)
% tensor (6x0x0)
n = size(Vx,1);
output = zeros(size(tensor,1),size(tensor,2));
for i = 1 : n
    output = output + Vx(i,1) * tensor(:,:,i);
end

end

