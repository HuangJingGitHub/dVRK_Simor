function robot = spherical_robot(q)

robot.DH = [
    % type  |  alpha  |        a         |         d         |       theta
       1      pi/2            0             0               pi/2 + q(1);
       1      -pi/2           0             0               -pi/2 + q(2);
       1      -pi/2           0             0               -(pi/4 - q(3));
       1      0               1             0               0]; 
end