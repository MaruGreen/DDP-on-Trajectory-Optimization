
function output = Test_comm ( initial, input, dt, L )

% This function is to test the command sequence
% initial is 6x1 initial state vector when t = 0
% input is 2x(N-1) control input 
% dt is the value of time step
% output is 6xN, a complete trajectory

N = size(input,2) + 1;
output = zeros(6,N);
output(:,1) = initial;

for i = 1 : N-1
    [temp,~,~] = transKine ( output(:,i), input(:,i), dt, L );
    output(:,i+1) = temp;
end

end



