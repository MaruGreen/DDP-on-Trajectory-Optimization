
initial;

%% Define the variables
N = size(traj_x,2);
Q = eye(6);
R = 1*eye(2);
Q(1,1) = 2.5;
Q(2,2) = 2.5;
Q(3,3) = 1;
Q(4,4) = 1;
Q(5,5) = 0.1;
Q(6,6) = 0;
L = 0.4;
iter = 400;
% change initial state
init_velo = 1.8;
init_orie = [0.6 0.3 0 -0.3];
init_phi = 0;
init_dphi = 0;
init_state = [init_velo;init_orie(1);init_phi;init_dphi];
% initial trajectory
current_traj = [traj_x; traj_y; zeros(4,N)];
current_input = zeros(2,N);
dt = time_step;

%% Complete the initial trajectory and input
% first run, find velocity and theta
for i = 1 : N-1
    v_x = (current_traj(1,i+1) - current_traj(1,i)) / dt;
    v_y = (current_traj(2,i+1) - current_traj(2,i)) / dt;
    % we assume the initial traj is going forward
    current_traj(3,i) = sqrt(v_x^2 + v_y^2);
    current_traj(4,i) = atan2(v_y,v_x);
end
current_traj(3:4,N) = current_traj(3:4,N-1);
% second run, find phi
for i = 1 : N-1
    del = current_traj(4,i) - current_traj(4,i+1);
    temp = del * L / (current_traj(3,i) * dt);
    current_traj(5,i) = atan(temp);
end
current_traj(5,N) = current_traj(5,N-1);
% third run, find phi_dot
for i = 1 : N-1
    current_traj(6,i) = (current_traj(5,i+1) - current_traj(5,i)) / dt;
end
current_traj(6,N) = current_traj(6,N-1);

% last run, find input
%for i = 1 : N-1
%    current_input(1,i) = (current_traj(3,i+1) - current_traj(3,i)) / dt;
%    current_input(2,i) = (current_traj(6,i+1) - current_traj(6,i)) / dt;
%end

%% Setting the initial state of the car
%current_traj(:,1)
current_traj(3:6,1) = init_state;

%% Update
Cost = zeros(9,iter);
% input is one fewer than trajectory
%current_traj = current_traj(:,1:end);
current_input = current_input(:,1:end-1);
%current_traj = Test_comm(current_traj(:,1),current_input,dt,L);
% restore initial trajectory as reference
ref_traj = current_traj;
ref_input = current_input;
for i = 1 : iter
    [newTraj, newInput, newCost] = Update2(current_traj,current_input,Q,R,L,dt,ref_traj,ref_input,false);
    Cost(:,i) = newCost;
    current_traj = newTraj;
    current_input = newInput;
    x = newTraj(1,:);
    y = newTraj(2,:);
    plot(x,y,'m')
    hold on
    if i == iter
    output = Test_comm( newTraj(:,1), newInput, dt, L );
    x = output(1,:);
    y = output(2,:);
    plot(x,y,'-*b')
    hold on
    end
end

%% 2
init_state = [init_velo;init_orie(2);init_phi;init_dphi];
% initial trajectory
current_traj = [traj_x; traj_y; zeros(4,N)];
current_input = zeros(2,N);
dt = time_step;

%% Complete the initial trajectory and input
% first run, find velocity and theta
for i = 1 : N-1
    v_x = (current_traj(1,i+1) - current_traj(1,i)) / dt;
    v_y = (current_traj(2,i+1) - current_traj(2,i)) / dt;
    % we assume the initial traj is going forward
    current_traj(3,i) = sqrt(v_x^2 + v_y^2);
    current_traj(4,i) = atan2(v_y,v_x);
end
current_traj(3:4,N) = current_traj(3:4,N-1);
% second run, find phi
for i = 1 : N-1
    del = current_traj(4,i) - current_traj(4,i+1);
    temp = del * L / (current_traj(3,i) * dt);
    current_traj(5,i) = atan(temp);
end
current_traj(5,N) = current_traj(5,N-1);
% third run, find phi_dot
for i = 1 : N-1
    current_traj(6,i) = (current_traj(5,i+1) - current_traj(5,i)) / dt;
end
current_traj(6,N) = current_traj(6,N-1);

% last run, find input
%for i = 1 : N-1
%    current_input(1,i) = (current_traj(3,i+1) - current_traj(3,i)) / dt;
%    current_input(2,i) = (current_traj(6,i+1) - current_traj(6,i)) / dt;
%end

%% Setting the initial state of the car
%current_traj(:,1)
current_traj(3:6,1) = init_state;

%% Update
Cost = zeros(9,iter);
% input is one fewer than trajectory
%current_traj = current_traj(:,1:end);
current_input = current_input(:,1:end-1);
%current_traj = Test_comm(current_traj(:,1),current_input,dt,L);
% restore initial trajectory as reference
ref_traj = current_traj;
ref_input = current_input;
for i = 1 : iter
    [newTraj, newInput, newCost] = Update2(current_traj,current_input,Q,R,L,dt,ref_traj,ref_input,false);
    Cost(:,i) = newCost;
    current_traj = newTraj;
    current_input = newInput;
    x = newTraj(1,:);
    y = newTraj(2,:);
    plot(x,y,'m')
    hold on
    if i == iter
    output = Test_comm( newTraj(:,1), newInput, dt, L );
    x = output(1,:);
    y = output(2,:);
    plot(x,y,'-*b')
    hold on
    end
end

%% 3
init_state = [init_velo;init_orie(3);init_phi;init_dphi];
% initial trajectory
current_traj = [traj_x; traj_y; zeros(4,N)];
current_input = zeros(2,N);
dt = time_step;

%% Complete the initial trajectory and input
% first run, find velocity and theta
for i = 1 : N-1
    v_x = (current_traj(1,i+1) - current_traj(1,i)) / dt;
    v_y = (current_traj(2,i+1) - current_traj(2,i)) / dt;
    % we assume the initial traj is going forward
    current_traj(3,i) = sqrt(v_x^2 + v_y^2);
    current_traj(4,i) = atan2(v_y,v_x);
end
current_traj(3:4,N) = current_traj(3:4,N-1);
% second run, find phi
for i = 1 : N-1
    del = current_traj(4,i) - current_traj(4,i+1);
    temp = del * L / (current_traj(3,i) * dt);
    current_traj(5,i) = atan(temp);
end
current_traj(5,N) = current_traj(5,N-1);
% third run, find phi_dot
for i = 1 : N-1
    current_traj(6,i) = (current_traj(5,i+1) - current_traj(5,i)) / dt;
end
current_traj(6,N) = current_traj(6,N-1);

% last run, find input
%for i = 1 : N-1
%    current_input(1,i) = (current_traj(3,i+1) - current_traj(3,i)) / dt;
%    current_input(2,i) = (current_traj(6,i+1) - current_traj(6,i)) / dt;
%end

%% Setting the initial state of the car
%current_traj(:,1)
current_traj(3:6,1) = init_state;

%% Update
Cost = zeros(9,iter);
% input is one fewer than trajectory
%current_traj = current_traj(:,1:end);
current_input = current_input(:,1:end-1);
%current_traj = Test_comm(current_traj(:,1),current_input,dt,L);
% restore initial trajectory as reference
ref_traj = current_traj;
ref_input = current_input;
for i = 1 : iter
    [newTraj, newInput, newCost] = Update2(current_traj,current_input,Q,R,L,dt,ref_traj,ref_input,false);
    Cost(:,i) = newCost;
    current_traj = newTraj;
    current_input = newInput;
    x = newTraj(1,:);
    y = newTraj(2,:);
    plot(x,y,'m')
    hold on
    if i == iter
    output = Test_comm( newTraj(:,1), newInput, dt, L );
    x = output(1,:);
    y = output(2,:);
    plot(x,y,'-*b')
    hold on
    end
end

%% 4
init_state = [init_velo;init_orie(4);init_phi;init_dphi];
% initial trajectory
current_traj = [traj_x; traj_y; zeros(4,N)];
current_input = zeros(2,N);
dt = time_step;

%% Complete the initial trajectory and input
% first run, find velocity and theta
for i = 1 : N-1
    v_x = (current_traj(1,i+1) - current_traj(1,i)) / dt;
    v_y = (current_traj(2,i+1) - current_traj(2,i)) / dt;
    % we assume the initial traj is going forward
    current_traj(3,i) = sqrt(v_x^2 + v_y^2);
    current_traj(4,i) = atan2(v_y,v_x);
end
current_traj(3:4,N) = current_traj(3:4,N-1);
% second run, find phi
for i = 1 : N-1
    del = current_traj(4,i) - current_traj(4,i+1);
    temp = del * L / (current_traj(3,i) * dt);
    current_traj(5,i) = atan(temp);
end
current_traj(5,N) = current_traj(5,N-1);
% third run, find phi_dot
for i = 1 : N-1
    current_traj(6,i) = (current_traj(5,i+1) - current_traj(5,i)) / dt;
end
current_traj(6,N) = current_traj(6,N-1);

% last run, find input
%for i = 1 : N-1
%    current_input(1,i) = (current_traj(3,i+1) - current_traj(3,i)) / dt;
%    current_input(2,i) = (current_traj(6,i+1) - current_traj(6,i)) / dt;
%end

%% Setting the initial state of the car
%current_traj(:,1)
current_traj(3:6,1) = init_state;

%% Update
Cost = zeros(9,iter);
% input is one fewer than trajectory
%current_traj = current_traj(:,1:end);
current_input = current_input(:,1:end-1);
%current_traj = Test_comm(current_traj(:,1),current_input,dt,L);
% restore initial trajectory as reference
ref_traj = current_traj;
ref_input = current_input;
for i = 1 : iter
    [newTraj, newInput, newCost] = Update2(current_traj,current_input,Q,R,L,dt,ref_traj,ref_input,false);
    Cost(:,i) = newCost;
    current_traj = newTraj;
    current_input = newInput;
    x = newTraj(1,:);
    y = newTraj(2,:);
    plot(x,y,'m')
    hold on
    if i == iter
    output = Test_comm( newTraj(:,1), newInput, dt, L );
    x = output(1,:);
    y = output(2,:);
    plot(x,y,'-*b')
    hold on
    end
end

axis([35 45 -8 2])
title('Optimized Trajectories with Different Initial Orientations')
hold off








