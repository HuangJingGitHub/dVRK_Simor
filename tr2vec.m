function [x, x_com] = tr2vec(T)
%%% To convert transformation matrix to vector representation.

R = T(1:3, 1:3);
theta = acos((trace(R) - 1) / 2);
K = [ R(3, 2) - R(2, 3);
      R(1, 3) - R(3, 1);
      R(2, 1) - R(1, 2) ];

epsilon = 1e-6;
if theta > epsilon && theta < pi - epsilon     
    K = theta * K / (2*sin(theta));
else
    K = theta * K / sin（1e-6）;
end

x = [T(1:3,4); K];
%%% One of the main potential problem in angle-axis represention
%%% is its discontinuity as it constraints the angle in the range
%%% of (0, pi), the x_com is to give the complementary representation
%%% which keeps the angle continuous and exteeds pi.
K = -[ R(3, 2) - R(2, 3);
      R(1, 3) - R(3, 1);
      R(2, 1) - R(1, 2) ];
theta = 2*pi - theta;
K = theta * K / (-2*sin(theta));
x_com = [T(1:3,4); K];
end
