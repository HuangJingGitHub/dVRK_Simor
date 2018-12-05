function xerr = xdif(T_dsr, T_act)
%%% Give 2 homogenerous transformation matrices, return the difference 6x1
%%% vector between them. The first 3 elements are Cartesian coordinates
%%% differences. The last 3 elements are orientation differences expressed in
%%% angle-axis way.

    x_dsr = T_dsr(1:3, 4);
    x_act = T_act(1:3, 4);
    R_dsr = T_dsr(1:3, 1:3);
    R_act = T_act(1:3, 1:3);
    Re = R_dsr * R_act';          % It is actually fixed frame rotation from original orientation set 
    K =  [ Re(3,2) - Re(2,3);      % to final orientation set. Then use premultiplication.
           Re(1,3) - Re(3,1);
           Re(2,1) - Re(1,2) ];
  
    %%% *** To avoid singular scenarios is quite inportant for robustness.
    %%% In certain cases, theta is quite small and MATLAB will regard
    %%% it as ecactly zero (or so close to pi). This cause zero devision
    %%% and K will bacome NaN. That cause problems, thus thresold is set  
    %%% to avoid this potential problem. ***
    %%% *** Actually, if just for control porpose, theta can be ignored.
    %%% Returning K above as the orientation error is sufficient of
    %%% controller. And this may be inportant if high computation speed is 
    %%% needed, that's why there is a lite version of this function. ***
    
    theta = acos((trace(Re) - 1) / 2);          % Built-in acos() returns angle in [0, pi].
    epsilon = 1e-5;                             % epsilon cannot be set too small.
    if theta > epsilon && theta < pi - epsilon          
        K = theta * K / (2*sin(theta));                 
    else
        K = theta * K / sin(1e-5);
    end
    
    angdif = K; 
    xdif = x_dsr - x_act;
    xerr = [xdif; angdif];
end
