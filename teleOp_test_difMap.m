classdef teleOp_test < handle
  properties(Access = public)
%     publisher;
%     jointStateMsg;
    tStart = tic;
    dt = 0.001;
    
    T_psm_mtm = [ -1        0         0;
                   0        -1        0;   
                   0         0        1];   
    psm_q;
    psm_x_cur;
    psm_x_dsr_pre;
    psm_x_dsr;
    psm_x_initial;
    mtm_x_initial;
    mtm_x;
    lambda = diag([500 500 500 200 200 200]);
    % 800 500 is also effective gain
    counter;
    origin_pos;
    fig;
    %define your variables

  end

    methods(Access = public)
         
        function obj = teleOp_test(mtm_q_initial,psm_q_initial)%,publisher,jointStateMsg)  % Constructor
            obj.psm_q = psm_q_initial;
            mtm = dVRK_MTMmodel(mtm_q_initial);
            psm = dVRK_PSMmodel(psm_q_initial);
            obj.mtm_x  = MTM_FK(mtm);
            obj.mtm_x_initial = obj.mtm_x;
            [obj.psm_x_cur, obj.origin_pos, ~] = PSM_FK(psm);
            obj.psm_x_initial = obj.psm_x_cur; 
            obj.psm_x_dsr = obj.psm_x_initial;  %obj.T_psm_mtm * obj.mtm_x;
            obj.psm_x_dsr_pre = obj.psm_x_initial;  %obj.T_psm_mtm * obj.mtm_x;            
            
            obj.counter = 1;
            obj.fig = init_fig(obj.psm_x_cur, obj.origin_pos);
            xdif_matrix(eye(4), eye(4))
            %initialize your variables

%             if (nargin > 3)
%                 obj.publisher = publisher;
%                 obj.jointStateMsg = jointStateMsg;
%             end    
        end

        function  [psm_q, tracking_err] = run(obj, mtm_q)
            mtm = dVRK_MTMmodel(mtm_q);
            psm = dVRK_PSMmodel(obj.psm_q);
            [obj.psm_x_cur, obj.origin_pos, J] = PSM_FK(psm);
            obj.mtm_x = MTM_FK(mtm);
            %%%obj.psm_x_dsr = obj.T_psm_mtm * obj.mtm_x;
            %%%obj.psm_x_dsr = obj.psm_x_initial + obj.T_psm_mtm *
            %%%(obj.mtm_x - obj.mtm_x_initial);   %%% Typical mistake:
            %%% direct difference of rotation matrix no meaning. 
            [mtm_R, mtm_x_dif] = xdif_matrix(obj.mtm_x, obj.mtm_x_initial);
            obj.psm_x_dsr(1:3, 1:3) = mtm_R * obj.psm_x_initial(1:3, 1:3);
            obj.psm_x_dsr(1:3, 4) = obj.T_psm_mtm * mtm_x_dif + obj.psm_x_initial(1:3, 4);
            psm_xdot_dsr = PSM_Vel_Cal(obj.psm_x_dsr, obj.psm_x_dsr_pre);
            
            tracking_err = xdif_lite(obj.psm_x_dsr, obj.psm_x_cur); 
            qdot_psm = pinv(J) * (psm_xdot_dsr + obj.lambda * tracking_err);
            obj.psm_q = obj.psm_q + qdot_psm * obj.dt;
            psm_q = obj.psm_q;
            obj.psm_x_dsr_pre = obj.psm_x_dsr;
            
            obj.counter = obj.counter + 1;
            if ~mod(obj.counter, 10)
                init_fig_run(obj.fig, obj.psm_x_cur, obj.origin_pos);
            end
                
            %transform mtm_q to psm desired Cartesian position and psm desired Cartesian velocity
            %and then call inverse kinematics function

        end    
        
%         function  callback_update_mtm_q(obj,q)
%             obj.jointStateMsg.Position = obj.run(q);
%             tElapsed = toc(obj.tStart);
%             if (tElapsed > 0.033)
%                 obj.tStart = tic;
%                 obj.publisher.send(obj.jointStateMsg);
%             end    
%         end

    end
end
