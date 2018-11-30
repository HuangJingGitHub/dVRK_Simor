function [fig, l_tool, l_P2Y, l_Y2C, arrow_x, arrow_y, arrow_z] = figure_init(frame_pos, T)
ratio = 100;
R = ratio * T(1:3, 1:3);
Ori_x = [frame_pos(1,end) + R(1,1), frame_pos(2,end) + R(2,1), frame_pos(3,end) + R(3,1)];
Ori_y = [frame_pos(1,end) + R(1,2), frame_pos(2,end) + R(2,2), frame_pos(3,end) + R(3,2)];
Ori_z = [frame_pos(1,end) + R(1,3), frame_pos(2,end) + R(2,3), frame_pos(3,end) + R(3,3)];

fig = figure('Position', [50, 50, 1000, 550]);
grid on
axis([-200,200, -200,200, -300,200])
xlabel('x', 'Fontname','Times','Fontsize', 12);
ylabel('y', 'Fontname','Times','Fontsize', 12);
zlabel('z', 'Fontname','Times','Fontsize', 12);
view(70,35);

l_tool = line([frame_pos(1,3), frame_pos(1,4)], [frame_pos(2,3), frame_pos(2,4)], [frame_pos(3,3), frame_pos(3,4)],...
               'Color', 'b', 'Marker', '.', 'MarkerSize', 20, 'LineWidth', 2);
l_P2Y = line([frame_pos(1,4), frame_pos(1,6)], [frame_pos(2,4), frame_pos(2,6)], [frame_pos(3,4), frame_pos(3,6)],...  % Frame origins of {4} and {5} are the same.
               'Color', 'b', 'Marker', '.', 'MarkerSize', 20, 'LineWidth', 2);
l_Y2C = line([frame_pos(1,6), frame_pos(1,7)], [frame_pos(2,6), frame_pos(2,7)], [frame_pos(3,6), frame_pos(3,7)],...
               'Color', 'b', 'Marker', '.', 'MarkerSize', 20, 'LineWidth', 2);
hold on
arrow_x = arrow3(frame_pos(:,end)', Ori_x, 'b');
arrow_y = arrow3(frame_pos(:,end)', Ori_y, 'g');
arrow_z = arrow3(frame_pos(:,end)', Ori_z, 'r');
           
end