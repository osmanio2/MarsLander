function [ x , v ] = motionEquation(a, dimensions, algorthim, dt, end_t )
%MOTIONEQUATION takes the acceleration and outputs position and velocity.
%   This function takes the acceleration of the motion, as a function, time 
%   step 'dt' and final time and the algorithm to calculate the velocity 
%   and position of the body. 'algorithm' can be 1 for "Euler"  and 2 for 
%   "Vertel". This function works independently of the spatial dimensions.
%   The function also takes the number of dimensions of the motion.



L = (end_t/dt);
x = zeros(L+1,dimensions);
v = x;
v(1,:) = ones(1,dimensions);
i = 1;
if(algorthim == 1)
    for t = 0 : L
        x(i+1,:) = x(i,:) + v(i,:) .* dt;
        v(i+1,:) = v(i,:) + a(x(i,:)) .* dt;
        i = i + 1;
    end
end


if(algorthim == 2)
    x(2,:) = v(1,:) .* (dt);
    v(2,:) = (x(2,:) - x(1,:)) ./ dt;
    i = 2;
     for t = 1 : L
         x(i+1,:) = (2 * x(i,:)) - x(i-1,:) + a(x(i,:)) .* (dt^2);
         v(i+1,:) = (x(i+1,:) - x(i,:)) ./ dt;
         i = i + 1;
     end
end


end

