%%% Test on SingSite forward kinematics.
clear
close all
clc

L = 0.0879;
W = 0.0462;
twist = pi/6;
quadratic_a = W / L^2;
caula = zeros(3, 100);
caula(3, :) = -linspace(0, L, 100);
caula(2, :) = -quadratic_a * caula(3, :).^2 * cos(twist);
caula(1, :) = quadratic_a * caula(3, :).^2 * sin(twist);

 
psm_q_test = zeros(6, 100);
psm_q_test(3, 1:50) = linspace(0, 0.03, 50);
psm_q_test(3, 50:end) = linspace(0.03, 0, 51);
psm_q_test(4, 1:50) = linspace(0, 2*pi, 50);
psm_q_test(4, 50:end) = linspace(2*pi, 0, 51);

disp_fig = figure('Position',[150, 100, 700, 500]);

for i = 1 : 100
    psm = dVRK_PSMmodel_SS(psm_q_test(:,i));
    [T, ~, ~, T4]= PSM_FK(psm);
    fig_update(disp_fig, T, T4, caula);
    pause(0.05);
end

% function fig = fig_init(T)
%     fig = figure('Position',[150, 100, 900, 500]);
% end

function fig_update(h, T, T4, cau_shape)
    ratio = 0.18;
    R = T(1:3, 1:3);
    X = T(1:3, 4);
    Ori = ratio * R;
    
    figure(h);
    cla
%     p = fill3([-0.1 0.1 0.1 -0.1],[-0.1 -0.1 0.1 0.1],[0 0 0 0],[0.7 0.7 0.7 0.7],'edgecolor','none')
%     alpha(p, 0.3)
    plot3(cau_shape(1,:), cau_shape(2,:), cau_shape(3,:), '.k', 'MarkerSize',10);
    hold on
    
    text(-0.01, 0, 0, 'RCM', 'Fontsize',10);
    text(ratio, 0, 0, 'x_0');
    text(0, ratio, 0, 'y_0');
    text(0, 0, ratio, 'z_0');
    base_x = quiver3(0, 0, 0, ratio, 0, 0, 'LineWidth', 1.5, 'Color', 'b');
    hold on
    base_y = quiver3(0, 0, 0, 0, ratio, 0, 'LineWidth', 1.5, 'Color', 'g');
    hold on
    base_z = quiver3(0, 0, 0, 0, 0, ratio, 'LineWidth', 1.5, 'Color', 'r');
    hold on
    text(X(1)+Ori(1,1), X(2)+Ori(2,1), X(3)+Ori(3,1), 'x_{tip}');
    text(X(1)+Ori(1,2), X(2)+Ori(2,2), X(3)+Ori(3,2), 'y_{tip}');
    text(X(1)+Ori(1,3), X(2)+Ori(2,3), X(3)+Ori(3,3), 'z_{tip}');
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,1), Ori(2,1), Ori(3,1), 'LineWidth', 1.5, 'Color', 'b');
    hold on
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,2), Ori(2,2), Ori(3,2), 'LineWidth', 1.5, 'Color', 'g');
    hold on
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,3), Ori(2,3), Ori(3,3), 'LineWidth', 1.5, 'Color', 'r');
    
    %%% Test on frame {4}
    hold on
    R = T4(1:3, 1:3);
    X = T4(1:3, 4);
    Ori = ratio * R;
    text(X(1)+Ori(1,1), X(2)+Ori(2,1), X(3)+Ori(3,1), 'x_4');
    text(X(1)+Ori(1,2), X(2)+Ori(2,2), X(3)+Ori(3,2), 'y_4');
    text(X(1)+Ori(1,3), X(2)+Ori(2,3), X(3)+Ori(3,3), 'z_4');
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,1), Ori(2,1), Ori(3,1), 'LineWidth', 1.5, 'Color', 'b');
    hold on
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,2), Ori(2,2), Ori(3,2), 'LineWidth', 1.5, 'Color', 'g');
    hold on
    tip_x = quiver3(X(1), X(2), X(3), Ori(1,3), Ori(2,3), Ori(3,3), 'LineWidth', 1.5, 'Color', 'r');
    

%     p = patch('Faces',[1,2,3,4],'Vertices',[-0.1 -0.1 0; 0.1 -0.1 0; 0.1 0.1 0;-0.1 0.1 0],'Facecolor','b')
%     alpha(p,0.3)
    axis equal
    axis([-0.4 0.35 -0.3 0.4 -0.3 0.2]);
    grid on
    view(160, 22);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('PSM Tip Trajectory with Cannula');
end