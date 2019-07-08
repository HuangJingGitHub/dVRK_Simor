classdef suj1 < handle
    properties (Access = protected)
        joint_num = 6;
        robot_DH = [
                % type  |  alpha  |    a    |     d     |     theta
                    2      0         0.0896       0         0;
                    1      0         0            0.4166    0;
                    1      0         0.4318       0.1429    0;  
                    1      0         0.4318       -0.1302   1.5708;
                    1      1.5708    0            0.4089    0;
                    1      -1.5708   0            -0.1029   -1.5708];
        suj2world = [-1    0     0    -0.1016;
                     0     -1    0    -0.1016;
                     0     0     1    0.4300;
                     0     0     0    1];
        tool_origin2suj_tip = [0    1    0    0.4864; 
                               -1   0    0    0;
                               0    0    1    0.1524;
                               0    0    0    1];
    end
    properties (Access = public)
        q_current;
        tip_pos;
        tip_pos_world;
        tool_origin_pos_world;
    end
    
    methods (Access = public)
        function self = suj1(q)
            self.q_current = q;
            self.tip_pos = self.forwardK();
            self.tip_pos_world = self.suj2world * self.tip_pos;
            self.tool_origin_pos_world = self.tip_pos_world * self.tool_origin2suj_tip;          
        end
        
        function tip_pos = forwardK(self)
            tip_pos = eye(4);

            for i = 1 : self.joint_num
                if self.robot_DH(i, 1) == 1
                    theta = self.robot_DH(i, 5) + self.q_current(i);
                    d     = self.robot_DH(i, 4);
                else
                    theta = self.robot_DH(1, 5);
                    d = self.robot_DH(i, 4) + self.q_current(i);
                end
                alpha = self.robot_DH(i, 2);
                a     = self.robot_DH(i, 3);
                
                Ti = [cos(theta)             -sin(theta)            0            a;
                      sin(theta)*cos(alpha)  cos(theta)*cos(alpha)  -sin(alpha)  -sin(alpha)*d;
                      sin(theta)*sin(alpha)  cos(theta)*sin(alpha)  cos(alpha)   cos(alpha)*d;
                      0                      0                      0            1];
                tip_pos = tip_pos * Ti;
            end  
        end
    end
end
    