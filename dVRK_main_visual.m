%%% ENGG5402 Advanced Robotics Course Project: dVRK_Simulator
clear
close all
clc

load('dvrk_mtm_psm.mat');
fre = 1000;
t_sample = 1 / fre;

num_sample        = size(mtm_q, 2);
mtm_x_test        = zeros(4,4,num_sample);
psm_x_dsr_test    = zeros(4,4,num_sample);
psm_xdot_dsr_test = zeros(6, num_sample-1);
T_psm_mtm         =  psm_x_dsr(:,:,1) / mtm_x(:,:,1) ;  % Transformation matrix from mtm base frame to psm base frame.

%%% Finish the 1st step calculation, for in the loop structure dependency on
%%% precious result exists.
q_mtm = mtm_q(:, 1);
mtm   = dVRK_MTMmodel(q_mtm);
mtm_x_test(:,:,1)     = MTM_FK(mtm);
psm_x_dsr_test(:,:,1) = T_psm_mtm * mtm_x_test(:,:,1);

T1 = zeros(1,num_sample - 1);
T2 = zeros(1, num_sample -1);
for i = 2 : num_sample
    q_mtm = mtm_q(:, i);
    mtm   = dVRK_MTMmodel(q_mtm);
    mtm_x_test(:,:,i)        = MTM_FK(mtm);
    psm_x_dsr_test(:,:,i)    =  T_psm_mtm * mtm_x_test(:,:,i);
    psm_xdot_dsr_test(:, i-1) = PSM_Vel_Cal(psm_x_dsr_test(:,:,i), psm_x_dsr_test(:,:,i-1));
end  

psm_q_test       = zeros(6, num_sample);
psm_x_act_test   = zeros(4, 4, num_sample);
origin_pos = zeros(3, 7, num_sample);
psm_q_test(:, 1) = psm_q_initial;
lambda = diag([1000 1000 1000 800 800 800]);
% lambda = diag([500 500 500 200 200 200]);
ang_er = zeros(1, num_sample);

% [fig, arrow_x, arrow_y, arrow_z] = figure_init(1000*pos, psm_x_act_test(:,:,1))

for i = 1 : num_sample - 1
    q_psm = psm_q_test(:,i);
    psm   = dVRK_PSMmodel(q_psm);
    [psm_x_act_test(:,:,i), origin_pos(:,:,i), J] = PSM_FK(psm);
    % xer = tr2vec(psm_x_dsr(:,:,i)) tr2vec(psm_x_act_test(:,:,i))- tr2vec(psm_x_act_test(:,:,i));
    xer = xdif_lite(psm_x_dsr_test(:,:,i), psm_x_act_test(:,:,i)); 
    ang_er(1, i) = norm(xer(1:6));
    qdot_psm = pinv(J) * (psm_xdot_dsr_test(:,i) + lambda * xer);
    psm_q_test(:, i+1) = psm_q_test(:, i) + qdot_psm * t_sample;  
    
%    [arrow_x, arrow_y, arrow_z] = fig_update(arrow_x, arrow_y, arrow_z, 1000*pos, psm_x_act_test(:,:,i));
end


ratio = 0.1;
t1 = ratio * psm_x_act_test(:,:,1); %psm_x_dsr(:,:,1);
p1 = origin_pos(:,:,1);
pn_1 = p1(:,end) + t1(1:3,1);
pn_2 = p1(:,end) + t1(1:3,2);
pn_3 = p1(:,end) + t1(1:3,3);

fig = figure('Position', [100, 60, 700, 550]);

arrow_x = quiver3( p1(1,end),...
                   p1(2,end),...
                   p1(3,end),t1(1,1), t1(2,1), t1(3,1),'LineWidth',2, 'Color','k');
hold on               
arrow_y = quiver3( p1(1,end),...
                   p1(2,end),...
                   p1(3,end),t1(1,2), t1(2,2), t1(3,2),'LineWidth',2, 'Color','g');  
hold on               
arrow_z = quiver3( p1(1,end),...
                   p1(2,end),...
                   p1(3,end),t1(1,3), t1(2,3), t1(3,3),'LineWidth',2, 'Color','r');
hold on
l_tool = line([p1(1,3), p1(1,4)], [p1(2,3), p1(2,4)], [p1(3,3), p1(3,4)],...
               'Color', 'b', 'Marker', '.', 'MarkerSize', 10, 'LineWidth', 2);
l_P2Y = line([p1(1,4), p1(1,6)], [p1(2,4), p1(2,6)], [p1(3,4), p1(3,6)],...   % Frame origins of {4} and {5} are the same.
               'Color', 'b', 'Marker', '.', 'MarkerSize', 10, 'LineWidth', 2);
l_Y2C = line([p1(1,6), p1(1,7)], [p1(2,6), p1(2,7)], [p1(3,6), p1(3,7)],...
               'Color', 'b', 'Marker', '.', 'MarkerSize', 10, 'LineWidth', 2);

axis([-0.25, 0.25, -0.25,0.25, -0.4,0.4])
axis square
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
title_tip = sprintf('Tip Position: x:%5.4f y:%5.4f z:%5.4f', p1(1),p1(2),p1(3));
title({'PSM Trajectory', title_tip});
view(-150,30);
name_x = text(pn_1(1),pn_1(2),pn_1(3), 'x','fontsize', 10);
name_y = text(pn_2(1),pn_2(2),pn_2(3),  'y', 'color', 'g', 'fontsize', 10);
name_z = text(pn_3(1),pn_3(2),pn_3(3),  'z', 'color', 'r','fontsize', 10);

for i = 1 : 5: num_sample
    t1 = ratio * psm_x_act_test(:,:,i);
    p_tip = t1(1:3,4) / ratio;
    p1 = origin_pos(:,:,i);
    pn_1 = p1(:,end) + t1(1:3,1);
    pn_2 = p1(:,end) + t1(1:3,2);
    pn_3 = p1(:,end) + t1(1:3,3);
    
    set(l_tool, 'xdata', [p1(1,3), p1(1,4)],...
            'ydata', [p1(2,3), p1(2,4)],...
            'zdata', [p1(3,3), p1(3,4)]);
    set(l_P2Y, 'xdata', [p1(1,4), p1(1,6)],...
           'ydata', [p1(2,4), p1(2,6)],...
           'zdata', [p1(3,4), p1(3,6)]);  
    set(l_Y2C, 'xdata', [p1(1,6), p1(1,7)],...
           'ydata', [p1(2,6), p1(2,7)],...
           'zdata', [p1(3,6), p1(3,7)]);    
    set( arrow_x,'XData', p_tip(1),'YData', p_tip(2), 'Zdata', p_tip(3),...
         'UData',t1(1,1), 'VData', t1(2,1), 'WData', t1(3,1));
    set( arrow_y,'XData', p_tip(1),'YData', p_tip(2), 'Zdata', p_tip(3),...
         'UData',t1(1,2), 'VData', t1(2,2), 'WData', t1(3,2)); 
    set( arrow_z,'XData', p_tip(1),'YData', p_tip(2), 'Zdata', p_tip(3),...
         'UData',t1(1,3), 'VData', t1(2,3), 'WData', t1(3,3)); 
     
     set(name_x, 'Position',pn_1);
     set(name_y, 'Position',pn_2);     
     set(name_z, 'Position',pn_3);
     
     hold on
     plot3(p_tip(1), p_tip(2), p_tip(3),'Marker','.','MarkerSize',5, 'Color','c');
     title_tip = sprintf('Tip Position: x:%5.4f y:%5.4f z:%5.4f', p_tip(1),p_tip(2),p_tip(3));
     title({'PSM Trajectory', title_tip});
     
%     pause(0.001*5)
%     delete(arrow_x);
%     delete(arrow_y);
%     delete(arrow_z);
%     arrow_x = quiver3( p_tip(1,end),...
%                        p_tip(2,end),...
%                        p_tip(3,end),t1(1,1), t1(2,1), t1(3,1),'LineWidth',2, 'Color','b');
%     hold on               
%     arrow_y = quiver3( p_tip(1,end),...
%                        p_tip(2,end),...
%                        p_tip(3,end),t1(1,2), t1(2,2), t1(3,2),'LineWidth',2, 'Color','g');  
%     hold on               
%     arrow_z = quiver3( p_tip(1,end),...
%                        p_tip(2,end),...
%                        p_tip(3,end),t1(1,3), t1(2,3), t1(3,3),'LineWidth',2, 'Color','r');
    drawnow
end

% figure
% plot(ang_er);