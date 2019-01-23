%%% Singularity Search %%%
clear; close all
addpath('../Simulation');
PSM_q1_lim = [-pi/4 pi/4];
PSM_q2_lim = [-pi/4 pi/4];
PSM_q3_lim = [0 0.02];
PSM_q = zeros(6,1);
J_det = zeros(1, 51*51);
counter = 1;

for psm_q1 = PSM_q1_lim(1): (PSM_q1_lim(2)-PSM_q1_lim(1))/50 : PSM_q1_lim(2)
    PSM_q(1) = psm_q1;
    for psm_q2 = PSM_q2_lim(1): (PSM_q2_lim(2)-PSM_q2_lim(1))/50 : PSM_q2_lim(2)
        PSM_q(2) = psm_q2;
        for psm_q3 = PSM_q3_lim(1): (PSM_q3_lim(2)-PSM_q3_lim(1))/50 : PSM_q3_lim(2) 
        PSM_q(3) = psm_q3;
        psm = dVRK_PSMmodel_SS(PSM_q);
        [~, ~, J] = PSM_FK(psm);
        J = J(1:3,:);
        J_det(1, counter) = det(J*J');
        %J_det(1, counter) = rank(J*J');
        counter = counter + 1;
        end
    end
end

figure('position',[150 150 600 400])
plot(J_det);