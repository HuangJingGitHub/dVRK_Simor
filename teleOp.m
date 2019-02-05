classdef teleOp < handle
  properties(Access = public)
    publisher;
    jointStateMsg;
    tStart = tic;
    dt = 0.001;
    
    T_psm_mtm = [ -1        0         0   -0.0001;
                   0        -1        0   -0.3639;
                   0         0        1   -0.0359;
                   0         0        0    1 ];
    psm_q;
    psm_x_cur;
    psm_x_dsr_pre;
    psm_x_dsr;
    mtm_x;
    lambda = diag([500 500 500 200 200 200]);
    
    counter;
    origin_pos;
    fig;
    %define your variables

  end

    methods(Access = public)
        
        function obj = teleOp(mtm_q_initial,psm_q_initial,publisher,jointStateMsg)  % Constructor
            obj.psm_q = psm_q_initial;
            mtm = dVRK_MTMmodel(mtm_q_initial);
            psm = dVRK_PSMmodel(psm_q_initial);
            obj.mtm_x = MTM_FK(mtm);
            [obj.psm_x_cur, obj.origin_pos, ~] = PSM_FK(psm);
            obj.psm_x_dsr = obj.T_psm_mtm * obj.mtm_x;
            obj.psm_x_dsr_pre = obj.psm_x_dsr;  %obj.T_psm_mtm * obj.mtm_x;
            
            obj.counter = 1;
            % obj.fig = init_fig(obj.psm_x_cur, obj.origin_pos);
            obj.fig = figure('Position', [50 50 500 800]) 
            % More compact, just need a figure handle, no need to initialize objects.
            %initialize your variables

            if (nargin > 3)
                obj.publisher = publisher;
                obj.jointStateMsg = jointStateMsg;
            end    
        end

        function  [psm_q,tracking_err] = run(obj, mtm_q)
            mtm = dVRK_MTMmodel(mtm_q);
            psm = dVRK_PSMmodel(obj.psm_q);
            [obj.psm_x_cur, obj.origin_pos, J] = PSM_FK(psm);
            obj.mtm_x = MTM_FK(mtm);
            obj.psm_x_dsr = obj.T_psm_mtm * obj.mtm_x;
            psm_xdot_dsr = PSM_Vel_Cal(obj.psm_x_dsr, obj.psm_x_dsr_pre);
            
            tracking_err = xdif_lite(obj.psm_x_dsr, obj.psm_x_cur); 
            qdot_psm = pinv(J) * (psm_xdot_dsr + obj.lambda * tracking_err);
            obj.psm_q = obj.psm_q + qdot_psm * obj.dt;
            psm_q = obj.psm_q;
            obj.psm_x_dsr_pre = obj.psm_x_dsr;
            
            obj.counter = obj.counter + 1;
            if ~mod(obj.counter, 20)
                init_fig_run(obj.fig, obj.psm_x_cur, obj.origin_pos);
                obj.counter = 0;
            end
                
            %transform mtm_q to psm desired Cartesian position and psm desired Cartesian velocity
            %and then call inverse kinematics function

        end    
        
        function  callback_update_mtm_q(obj,q)
            obj.jointStateMsg.Position = obj.run(q);
            tElapsed = toc(obj.tStart);
            if (tElapsed > 0.033)
                obj.tStart = tic;
                obj.publisher.send(obj.jointStateMsg);
            end    
        end

    end
end
