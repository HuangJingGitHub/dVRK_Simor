clear
close all
clc

load('dvrk_mtm_psm.mat');

steps = size(mtm_q, 2);

robot = teleOp_test(mtm_q(:,1), psm_q_initial);

for i = 1 : steps
    robot.run(mtm_q(:,i));
end
