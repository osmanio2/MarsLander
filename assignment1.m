% This script execute the Euler and Vertel algorithms for the mass on the
% spring and plot the trajectories of the dynamical variables as a
% function of time.
%--------------------------------------------------------------------------
% Initialising boundary/initial conditions

dt = input('What is time step delta t? ');
k = input('What value is the spring constant k? ');
m = input('What value is the mass m? ');

%  t= 0:dt:1000;
 t = linspace(0,99,(99/dt) + 2); 
% x = zeros(size(t));
% v = x;
% x(1) = xi;
% v(1) = vi;
%E = 0.5 * k * x(1)^2 + 0.5 * m * v(1)^2;
%--------------------------------------------------------------------------
% Implementing Euler method:

a = @(x)(-k/m) * x;

[ x , v ] = motionEquation(a, 1, 1, dt, 99);
% for i = 2:length(t)
%     x(i) = x(i-1) + dt * v(i-1);
%    %disp('Distance is');
%    % x(i)
%     v(i) = v(i-1) - dt * (k/m) * x(i-1);
%    % disp('Velocity is');
%    % v(i)
% end
% size(t)
% size(x)
% size(v)
figure;
plot(t,x,'r')
hold on;
plot(t,v,'g')
hold off;
%--------------------------------------------------------------------------
% Implementing Vertlet method:
% for i = 3:length(t)
%     x(i) = 2 * x(i-1) - x(i-2) - dt^2 * (k/m) * x(i-1);
%    %disp('Distance is');
%    % x(i)
%     v(i-1) = (1/(2*dt)) * ( x(i) - x(i-2));
%     v(i) = (1/(dt)) * ( x(i) - x(i-1));
%    % disp('Velocity is');
%    % v(i)
% end
[ x , v ] = motionEquation(a, 1, 2, dt, 99);
figure(2);
plot(t,x,'b')
hold on;
plot(t,v,'r')
%--------------------------------------------------------------------------
% % Finding the critical value of time step dt:
% 
% dt = 1.9:0.001:2;
% for j = 1: length(dt)
%     0:dt(j):1000;
%     for i = 3:length(t)
%     x(i) = 2 * x(i-1) - x(i-2) - dt(j)^2 * (k/m) * x(i-1);
%    %disp('Distance is');
%    % x(i)
%     v(i-1) = (1/(2*dt(j))) * ( x(i) - x(i-2));
%     v(i) = (1/(dt(j))) * ( x(i) - x(i-1));
%    % disp('Velocity is');
%    % v(i)
%     end
% fprintf('difference between initial and final velocity for dt = %0.5g', dt(j));
% 
% 1 - max(x)
% 
% 1 - max(v)
% 
% 
% end
