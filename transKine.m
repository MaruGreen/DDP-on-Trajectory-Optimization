
function [newState, A, B] = transKine ( state, input, dt, L )

% This function is the discrete time transfer system. As the system is
% nonlinear, we can not use MATLAB build-in function.
% state is 6x1 state vector  when t = n
% input is 2x1 control input when t = n
% dt is the value of time step
% newState is 6x1, the state when t = n+1

%% Eextrace state elements
theta = state(4);
phi = state(5);

%% Build matrix A and B
A = eye(6);
A(1,3) = dt * cos(theta);
A(2,3) = dt * sin(theta);
A(4,3) = -dt * tan(phi) / L;
A(5,6) = dt;

B = zeros(6,2);
B(3,1) = dt;
B(6,2) = dt;

%% Update
newState = A * state + B * input;

end



